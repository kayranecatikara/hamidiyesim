"""
frpn_guidance.py — FRPN tabanlı GPS güdümü uçuş döngüsü (F4).

gps_guidance.py'nin YERİNE GEÇMEZ, YANINDA durur. Seçim:
    AVCI_GPS_GUDUM=istasyon   (varsayılan) → eski, uçuşta denenmiş yasa
    AVCI_GPS_GUDUM=frpn                    → bu modül

Arayüz sözleşmesi gps_guidance ile BİREBİR aynıdır (supervisor ve gcs_server
hiçbir değişiklik olmadan çalışsın diye):
    run_frpn_guidance(conn, get_plane, get_iris, stop_event, cfg=Cfg)
    status["durum"] ∈ {WARMUP, ARAMA, KILIT, DROPOUT, DURDU}
    status["d_h"], status["menzil"]

ESKİSİNDEN FARKLARI (hepsi F3 tezgâhında ölçüldü, tools/frpn_replay.py):
  1) KESTİRİCİ: EMA+sonlu fark yerine IMM (CV+CA).
     Ölçülen: hız hatası düz uçuşta 8.7×, dairede 2.4× küçük; şişme
     1.052× → 1.001×. Ayrıca telemetri güdümden yavaş olduğunda ara kareler
     tahmin() ile doldurulur — EMA bunu yapamıyor, donuk konum kovalıyordu.
  2) NİŞAN NOKTASI: istasyon artık 1000 m'den değil, menzile bağlı olarak
     devreye giriyor (frpn.sanal_hedef; ≥120 m saf kesme, ≤30 m tam istasyon).
     Eski davranış tüm yaklaşmayı kuyruk kovalamaya mahkûm ediyordu.
  3) YASA: frpn.komut — v_hedef + V_c·û + K_ZEM·ZEM/t_go.
     Eskisiyle AYNI üç terimli yapıda; fark kazançlarda (KD_H 0.20 → K_ZEM 0.60)
     ve kapanma teriminin tavanlı olmasında.
  4) DİKEY FREN FARKINDALIĞI: iniş hızı √(2·a_z·(irtifa−taban)) ile sınırlı.
     11:34 çakılmasının panzehiri; eskisinde yoktu.

Tezgâh sonucu (15 m altında toplam tutunma, 4 yörünge):
    eski yasa : 97.9 s (EMA/20Hz) · 112.4 s (IMM/20Hz) · 113.4 s (IMM/1Hz)
    bu modül  : 142.0 s           · 139.9 s            · 125.5 s
UÇUŞTA HENÜZ DOĞRULANMADI — ilk uçuş A/B olarak yapılmalı.
"""

import csv
import math
import os
import time

from control.guidance import frpn
from control.guidance.common import (
    clamp, limit_acceleration, normalize_angle, send_velocity,
)
from control.guidance.guidance_core import hedef_kadraj_hatasi
from control.guidance.hedef_kestirim import IMM


def _env_f(name, default):
    return float(os.environ.get(name, default))


class Cfg:
    LOOP_HZ = 20.0

    # ── Araç zarfı (2026-08-04 karakterizasyonunda ÖLÇÜLDÜ) ──
    # Bunlar aracın yapabildiğinden fazlasını istememek için var. Eski modülde
    # V_MAX=18 iken firmware tavanı 10 m/s idi ve güdüm bunu bilmiyordu; artık
    # parametreler düzeltildi (WPNAV_SPEED 25, WPNAV_ACCEL 8) ve gerçek ölçüm:
    #   düz uçuşta ~18 m/s, dairede ~14.4 m/s, yanal ivme %95 9.31 m/s²
    V_MAX = _env_f("AVCI_FRPN_VMAX", 18.0)
    VZ_MAX = 6.0
    MAX_ACCEL = 8.0           # m/s²; WPNAV_ACCEL ile hizalı
    LOOKUP_MIN_ALT = 8.0      # m; iniş tabanı
    AZ_FREN = 2.5             # m/s²; WPNAV_ACCEL_Z — fren hesabı bunu kullanır

    # ── Yaw ──
    YAW_DEADBAND = math.radians(3.0)
    YAW_RATE_MAX = math.radians(120.0)

    # ── Telemetri tazeliği ──
    HOLD_S = 3.0              # s; bu kadar donuk telemetri → DROPOUT
    HANDOFF_RANGE = 20.0      # m; d_h altında durum=KILIT (salt etiket)


status = {
    "durum": "WARMUP", "d_h": None, "menzil": None,
    "kadraj_yaw_deg": None, "kadraj_elev_deg": None, "none_count": 0,
}

_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

_CSV_ALANLAR = [
    "t", "dt", "durum", "d_h", "menzil",
    "tgt_x", "tgt_y", "tgt_z", "tgt_vx", "tgt_vy", "tgt_vz",
    "tgt_ax", "tgt_ay", "tgt_az", "w_cv", "w_ca",
    "iris_x", "iris_y", "iris_z", "iris_vx", "iris_vy", "iris_vz",
    "iris_roll_deg", "iris_pitch_deg", "iris_yaw_deg",
    "st_x", "st_y", "st_z", "gecis_w",
    "t_go", "zem_norm", "v_c", "zem_kirpildi",
    "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg", "vz_fren_sinir",
    "kadraj_yaw_deg", "kadraj_elev_deg", "kadraj_pitch_hata_deg", "u_px", "v_px",
]


def _sleep(t_start, period):
    elapsed = time.monotonic() - t_start
    if elapsed < period:
        time.sleep(period - elapsed)


def _inis_tavani(irtifa, cfg):
    """Fren-farkında iniş hızı sınırı: v ≤ √(2·a·(irtifa − taban)).

    11:34 çakılmasının doğrudan panzehiri. Hedefin dalışını kovalarken dikey
    hız tavana dayanıyordu ama frenlemek için gereken mesafe hesaba
    katılmıyordu; 6 m/s'den 1 m/s² ile durmak 18 m ister. Artık iniş hızı
    kalan irtifayla sınırlanıyor.
    """
    pay = max(0.0, irtifa - cfg.LOOKUP_MIN_ALT)
    return min(cfg.VZ_MAX, math.sqrt(2.0 * cfg.AZ_FREN * pay)) if pay > 0 else 0.0


def run_frpn_guidance(conn, get_plane, get_iris, stop_event, cfg=Cfg,
                      frpn_cfg=frpn.Cfg):
    loop_period = 1.0 / cfg.LOOP_HZ
    kf = IMM()
    son_ham = None
    t_son_taze = None
    none_count = 0

    vx_prev = vy_prev = vz_prev = 0.0
    cmd_yaw = None
    prev_time = None
    loop_count = 0

    os.makedirs(_LOG_DIR, exist_ok=True)
    csv_yol = os.path.join(_LOG_DIR, time.strftime("gps_guidance_%Y%m%d_%H%M%S.csv"))
    f = open(csv_yol, "w", newline="")
    w = csv.DictWriter(f, fieldnames=_CSV_ALANLAR, extrasaction="ignore")
    w.writeheader()

    print("=" * 60)
    print("[FRPN] FRPN tabanlı GPS güdümü — IMM kestirici + menzile bağlı istasyon")
    print(f"[FRPN] K_C={frpn_cfg.K_C} V_C_MAX={frpn_cfg.V_C_MAX} "
          f"K_ZEM={frpn_cfg.K_ZEM} | geçiş {frpn_cfg.GECIS_BASLA:.0f}→"
          f"{frpn_cfg.GECIS_TAM:.0f} m | V_MAX={cfg.V_MAX}")
    print(f"[FRPN] log: {csv_yol}")
    print("=" * 60)

    def _hover():
        send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)

    try:
        while not stop_event.is_set():
            now = time.monotonic()
            dt = (now - prev_time) if prev_time is not None else loop_period
            dt = clamp(dt, 0.001, 0.2)
            prev_time = now

            iris = get_iris()
            ix, iy, iz = iris["x"], iris["y"], iris["z"]
            ivx = iris.get("vx", 0.0)
            ivy = iris.get("vy", 0.0)
            ivz = iris.get("vz", 0.0)
            iroll = iris.get("roll", 0.0)
            ipitch = iris.get("pitch", 0.0)
            iyaw = iris.get("yaw", 0.0)
            plane = get_plane()

            # ── 1) KESTİRİM (IMM) ──
            # Her kare ilerlet; telemetri TAZE ise ölçüm uygula. Bu ayrım
            # zorunlu: tek çağrıda ikisini yapmak zamanı çift sayar (bkz.
            # hedef_kestirim modül notu).
            kf.tahmin(dt)
            ham = (plane["x"], plane["y"], plane["z"])
            donuk = bool(plane.get("frozen", False))
            if (not donuk) and ham != son_ham:
                kf.olcum(ham)
                son_ham = ham
                t_son_taze = now
                none_count = 0
            else:
                none_count += 1
            status["none_count"] = none_count

            kes = kf.durum()

            # ── 2) WARMUP / DROPOUT ──
            if not kes["hazir"]:
                _hover()
                status.update(durum="WARMUP", d_h=None, menzil=None)
                loop_count += 1
                _sleep(now, loop_period)
                continue
            if t_son_taze is not None and (now - t_son_taze) > cfg.HOLD_S:
                _hover()
                vx_prev = vy_prev = vz_prev = 0.0
                status.update(durum="DROPOUT")
                loop_count += 1
                _sleep(now, loop_period)
                continue

            p_h = kes["p"]
            v_h = kes["v"]

            ex, ey = p_h[0] - ix, p_h[1] - iy
            d_h = math.hypot(ex, ey)
            menzil = math.sqrt(ex * ex + ey * ey + (p_h[2] - iz) ** 2)

            # ── 3) SANAL HEDEF (menzile bağlı istasyon) ──
            p_s, tani = frpn.sanal_hedef(p_h, v_h, (ix, iy, iz), frpn_cfg)

            # ── 4) FRPN YASASI ──
            dp = (p_s[0] - ix, p_s[1] - iy, p_s[2] - iz)
            dv = (v_h[0] - ivx, v_h[1] - ivy, v_h[2] - ivz)
            res = frpn.komut(dp, dv, v_h, frpn_cfg)
            vx, vy, vz = res["v_cmd"]

            # ── 5) SINIRLAR ──
            vmag = math.hypot(vx, vy)
            if vmag > cfg.V_MAX and vmag > 1e-6:
                s = cfg.V_MAX / vmag
                vx *= s
                vy *= s
            # dikey: fren-farkında iniş tavanı (aşağı = +z)
            irtifa = -iz
            vz_asagi_max = _inis_tavani(irtifa, cfg)
            vz = clamp(vz, -cfg.VZ_MAX, vz_asagi_max)

            # ── 6) YAW: burun GERÇEK hedefe (sanal hedefe değil) ──
            bearing = math.atan2(ey, ex)
            if cmd_yaw is None:
                cmd_yaw = bearing
            yaw_err = normalize_angle(bearing - cmd_yaw)
            if abs(yaw_err) > cfg.YAW_DEADBAND:
                step = clamp(yaw_err, -cfg.YAW_RATE_MAX * dt, cfg.YAW_RATE_MAX * dt)
                cmd_yaw = normalize_angle(cmd_yaw + step)

            # ── 7) İVME SINIRI + GÖNDER ──
            vx, vy, vz = limit_acceleration(
                vx, vy, vz, vx_prev, vy_prev, vz_prev, cfg.MAX_ACCEL, dt)
            vx_prev, vy_prev, vz_prev = vx, vy, vz
            send_velocity(conn, vx, vy, vz, cmd_yaw)

            # ── 8) KADRAJ HATASI (ölçüm; güdüme girmez) ──
            kad = hedef_kadraj_hatasi(p_h, (ix, iy, iz), iroll, ipitch, iyaw)

            durum = "KILIT" if d_h < cfg.HANDOFF_RANGE else "ARAMA"
            status.update(durum=durum, d_h=round(d_h, 1), menzil=round(menzil, 1),
                          kadraj_yaw_deg=round(math.degrees(kad["yaw_hata"]), 1),
                          kadraj_elev_deg=round(math.degrees(kad["elev"]), 1))

            a_h = kes["a"]
            w.writerow({
                "t": round(now, 3), "dt": round(dt, 4), "durum": durum,
                "d_h": round(d_h, 2), "menzil": round(menzil, 2),
                "tgt_x": round(p_h[0], 2), "tgt_y": round(p_h[1], 2),
                "tgt_z": round(p_h[2], 2),
                "tgt_vx": round(v_h[0], 2), "tgt_vy": round(v_h[1], 2),
                "tgt_vz": round(v_h[2], 2),
                "tgt_ax": round(a_h[0], 2), "tgt_ay": round(a_h[1], 2),
                "tgt_az": round(a_h[2], 2),
                "w_cv": round(kes["w_cv"], 3), "w_ca": round(kes["w_ca"], 3),
                "iris_x": round(ix, 2), "iris_y": round(iy, 2), "iris_z": round(iz, 2),
                "iris_vx": round(ivx, 2), "iris_vy": round(ivy, 2),
                "iris_vz": round(ivz, 2),
                "iris_roll_deg": round(math.degrees(iroll), 1),
                "iris_pitch_deg": round(math.degrees(ipitch), 1),
                "iris_yaw_deg": round(math.degrees(iyaw), 1),
                "st_x": round(p_s[0], 2), "st_y": round(p_s[1], 2),
                "st_z": round(p_s[2], 2), "gecis_w": round(tani["gecis_w"], 3),
                "t_go": round(res["t_go"], 2),
                "zem_norm": round(res["zem_norm"], 2),
                "v_c": round(res["v_c"], 2),
                "zem_kirpildi": int(res["zem_kirpildi"]),
                "vx_cmd": round(vx, 2), "vy_cmd": round(vy, 2), "vz_cmd": round(vz, 2),
                "yaw_cmd_deg": round(math.degrees(cmd_yaw), 1),
                "vz_fren_sinir": round(vz_asagi_max, 2),
                "kadraj_yaw_deg": round(math.degrees(kad["yaw_hata"]), 2),
                "kadraj_elev_deg": round(math.degrees(kad["elev"]), 2),
                "kadraj_pitch_hata_deg": round(math.degrees(kad["pitch_hata"]), 2),
                "u_px": round(kad["u"], 1) if kad["u"] is not None else "",
                "v_px": round(kad["v"], 1) if kad["v"] is not None else "",
            })
            f.flush()

            loop_count += 1
            if loop_count % int(cfg.LOOP_HZ * 3) == 0:
                print(f"[FRPN] {durum} d_h={d_h:.1f}m menzil={menzil:.1f}m "
                      f"t_go={res['t_go']:.1f}s ZEM={res['zem_norm']:.1f}m "
                      f"geçiş_w={tani['gecis_w']:.2f} "
                      f"v=({vx:+.1f},{vy:+.1f},{vz:+.1f}) "
                      f"CA={kes['w_ca']:.2f}")

            _sleep(now, loop_period)

        send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)
        status.update(durum="DURDU")
        print("[FRPN] Stop sinyali — döngü sonlandı.")
    finally:
        f.close()
        print(f"[FRPN] log kapatıldı: {csv_yol}")
