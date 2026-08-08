"""
bbox_ibvs.py — SAF görüntü tabanlı görsel güdüm (IBVS), yalnız bbox.

YARIŞMA KURALI (üstün kısıt, bkz. UYGULANACAK.md D0): görsel temas varken
hedefin GPS'i güdümde KULLANILAMAZ — canlı GPS akışı yasak.

Bu modül iki girdiyle çalışır:
  1. Tespit kutusu (cx, cy, w, h, conf) — her kare, tek canlı kaynak.
  2. Drone'un KENDİ durumu (yaw, kendi hızı) — kendi sensörü, kural serbest.
  3. DONDURULMUŞ TAŞIYICI (ff_hiz): devir ANINDA, yani görsel temas kurulmadan
     ÖNCEKİ son GPS kestiriminden alınan hedef hız vektörü. Görsel faz boyunca
     BİR DAHA OKUNMAZ.
     ⚠ YAPISAL GARANTİ: bu bir SAYI ÜÇLÜSÜ olarak geçilir, callback değil —
     döngünün canlı GPS'e erişimi FİZİKSEL OLARAK YOKTUR. Kural ihlali
     "yapmamayı seçmek"le değil, yapamamakla güvence altında.

Kontrol yasası:
  YAW     : yatay piksel hatası (cx − CX) → burun hedefe döner.
  DİKEY   : ff_vz + dikey piksel hatası (cy − CY_NISAN) → tırman/alçal.
  YATAY   : v = ff_taşıyıcı + v_kapanma · û_LOS
            ff_taşıyıcı hedefin seyir hızını üstlenir (drone geride kalmaz);
            kutu boyutundan gelen v_kapanma yalnız ARADAKİ FARKI kapatır.

NEDEN TAŞIYICI ŞART (2026-08-08 uçuş dersi): saf kutu-boyutu modeli 12 m'de
yalnız 8 m/s üretiyordu; hedef 15 m/s gidiyor → drone geride kalıp kutuyu
kaybediyor, faz 3.5 s'de kopuyordu. Kutu boyutu MENZİL vekilidir, HIZ vekili
değil: küçük kutu "uzak" demek zorunda değil, hedef zaten küçük.

Arayüz (supervisor.run_hybrid ile uyumlu):
  run_bbox_ibvs(conn, get_iris, wait_pose, stop_event, cfg, kayip_kare_esik,
                ff_hiz=(vx,vy,vz))
    get_iris() -> {..., "yaw": rad, "vx","vy","vz": m/s}  (drone KENDİ durumu)
    wait_pose(son_seq, timeout) -> {"seq","pose",...}  (pose = bbox kaydı | None)
  Dönüş: 'durduruldu' (stop_event) | 'kayip' (kayip_kare_esik ardışık kutusuz).
"""

import csv
import math
import os
import time

from vision import geometry as geo
from control.guidance.common import (
    clamp, normalize_angle, send_velocity, limit_acceleration,
)


def _env_f(name, default):
    return float(os.environ.get(name, default))


class Cfg:
    LOOP_HZ = 20.0

    # ── KADRAJ NİŞAN NOKTASI ──
    # ⚠ GEOMETRİ (2026-08-08 uçuş dersi): kamera gövdeye 25° YUKARI tilt'li.
    # SEVİYE (co-altitude) bir hedef kadrajda merkezde DEĞİL, AŞAĞIDA görünür:
    #     cy_seviye = CY + FY·tan(25°) = 240 + 166.6·0.466 ≈ 318 px
    # İlk sürümde nişan 210 (üst) alınmıştı — bu "hedefin ~8 m ALTINA dal"
    # demekti: drone vz'yi tavana (+4) yapıştırıp sürekli alçaldı, hedef
    # kadrajın altından (cy→390→dışarı) kaçtı, faz 3.1 s'de koptu.
    # DÜZELTME: nişanı seviye-hedef konumunun hafif ÜSTÜNE al (drone hedefin
    # az altında kalsın — gökyüzü arka planı + terminal pop-up). tan(20°) ile
    # ~10° altı: cy ≈ 240 + FY·tan(20°) ≈ 300.
    _CY_SEVIYE = geo.CY + geo.FY * math.tan(math.radians(20.0))
    CY_NISAN = _env_f("AVCI_IBVS_CY", round(_CY_SEVIYE, 0))  # ≈300 px
    CX_NISAN = geo.CX                           # px; yatay merkez (320)

    # ── YAW ──
    # eps_yaw = atan((cx − CX)/FX); komut = iris_yaw + K_YAW·eps. K_YAW=1 tam
    # düzeltme (ArduPilot kendi yaw hızıyla slew eder). <1 yumuşatır.
    K_YAW = _env_f("AVCI_IBVS_KYAW", 1.0)
    YAW_ESIK = math.radians(1.0)   # bu açının altında yaw komutu güncellenmez

    # ── DİKEY ──
    # eps_elev = atan((cy − CY_NISAN)/FY); v_z = K_VZ · V_NOM · eps_elev.
    # cy büyük (hedef kadrajda AŞAĞIDA) → hedef boresight'ın altında → ALÇAL
    # (vz>0, NED down+). Nominal hızla ölçekli: hızlı giderken dik açı daha çok
    # dikey hız ister (irtifayı korumak için).
    # K_VZ 1.2 → 0.5, VZ_MAX 4 → 3 (2026-08-08): ilk sürüm dikey hızı çok
    # agresifti (10° hata → 2.5 m/s) ve tavana yapışıp salındı. Nişan doğru
    # yere gelince (≈300) hata küçük kalır; yumuşak kazanç yeter.
    K_VZ = _env_f("AVCI_IBVS_KVZ", 0.5)
    VZ_MAX = 3.0                    # m/s; dikey hız tavanı
    V_NOM = 12.0                   # m/s; dikey ölçekleme için nominal ileri hız

    # ── KAPANMA (kutu büyüklüğü = MENZİL vekili; taşıyıcının ÜSTÜNE eklenir) ──
    # boyut = sqrt(w·h). Büyük kutu = yakın. Kapanma hızı boyut REF'e
    # yaklaştıkça azalır, REF'i aşınca NEGATİF olur (geri çekil):
    #     v_kap = clamp(K_FWD·(REF − boyut), −V_KAPANMA_GERI, V_KAPANMA_MAX)
    # Bu hız LOS yönünde (burun yönü) uygulanır; taşıyıcı zaten hedefin seyrini
    # üstlendiği için kapanmanın işi YALNIZ aradaki farkı yemek.
    #
    # REF ölçümden (2026-08-08 uçuş logu): 12 m'de kutu ≈ 12 px → boyut ~1/menzil.
    # 6 m hedef tutuş için REF ≈ 25 px. Profil:
    #     30 m (5px) → +6.0 (tavan)   12 m (12px) → +4.6   8 m (18px) → +2.5
    #      6 m (25px) → 0 (denge)      4 m (37px) → −2.0 (tavan, geri)
    BOYUT_REF = _env_f("AVCI_IBVS_REF", 25.0)   # px; sqrt(w·h) denge boyutu
    K_FWD = _env_f("AVCI_IBVS_KFWD", 0.35)      # (m/s)/px
    V_KAPANMA_MAX = _env_f("AVCI_IBVS_VKAP", 6.0)   # m/s; taşıyıcı üstü kapanma
    V_KAPANMA_GERI = 2.0           # m/s; çok yaklaşınca geri çekilme tavanı
    V_TOPLAM_MAX = _env_f("AVCI_IBVS_VMAX", 18.0)   # m/s; yatay toplam tavan
    MAX_ACCEL = 12.0               # m/s²; komut hızı değişim sınırı

    # ── KUTU GEÇERLİLİĞİ ──
    CONF_MIN = _env_f("AVCI_IBVS_CONF", 0.35)   # bunun altı kutu = yok sayılır
    BOYUT_MIN = 6.0                # px; bundan küçük kutu güvenilmez (gürültü)


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

_CSV_ALANLAR = [
    "t", "dt", "durum", "cx", "cy", "w", "h", "boyut", "conf",
    "eps_yaw_deg", "eps_elev_deg", "iris_yaw_deg",
    "v_kapanma", "ff_vx", "ff_vy", "ff_vz",
    "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg", "kayip_sayac",
]


def komut(cx, cy, w, h, iris_yaw, ff_hiz=(0.0, 0.0, 0.0), cfg=Cfg):
    """IBVS kontrol yasası (MAVLink yok, CANLI GPS yok). Test edilebilir.

    Girdi:
      (cx,cy,w,h) : tespit kutusu — TEK canlı kaynak
      iris_yaw    : drone kendi yaw'ı (rad) — kendi sensörü
      ff_hiz      : DONDURULMUŞ taşıyıcı (vx,vy,vz) NED — devir anında bir kez
                    alınmış sabit üçlü; bu fonksiyon onu yalnız TOPLAR.
    Çıktı: (vx_ned, vy_ned, vz, yaw_cmd, tani)
    """
    boyut = math.sqrt(max(w, 0.0) * max(h, 0.0))

    # YAW: yatay açı hatası → burun hedefe
    eps_yaw = math.atan((cx - cfg.CX_NISAN) / geo.FX)
    yaw_cmd = normalize_angle(iris_yaw + cfg.K_YAW * eps_yaw)

    # KAPANMA: kutu boyutu menzil vekili → LOS yönünde ek hız
    v_kap = clamp(cfg.K_FWD * (cfg.BOYUT_REF - boyut),
                  -cfg.V_KAPANMA_GERI, cfg.V_KAPANMA_MAX)

    # TAŞIYICI + KAPANMA (kapanma burun/LOS yönünde)
    vx_ned = ff_hiz[0] + v_kap * math.cos(yaw_cmd)
    vy_ned = ff_hiz[1] + v_kap * math.sin(yaw_cmd)
    vmag = math.hypot(vx_ned, vy_ned)
    if vmag > cfg.V_TOPLAM_MAX and vmag > 1e-6:
        s = cfg.V_TOPLAM_MAX / vmag
        vx_ned *= s
        vy_ned *= s

    # DİKEY: taşıyıcı dikey + elev hatası → tırman/alçal
    eps_elev = math.atan((cy - cfg.CY_NISAN) / geo.FY)   # cy büyük → hedef altta
    vz = clamp(ff_hiz[2] + cfg.K_VZ * cfg.V_NOM * eps_elev,
               -cfg.VZ_MAX, cfg.VZ_MAX)

    tani = {"boyut": boyut, "eps_yaw": eps_yaw, "eps_elev": eps_elev,
            "v_kapanma": v_kap}
    return vx_ned, vy_ned, vz, yaw_cmd, tani


def _kutu_gecerli(pose, cfg):
    """pose kaydından geçerli kutu çıkar → (cx,cy,w,h,conf) veya None."""
    if pose is None:
        return None
    conf = pose.get("conf", 0.0)
    if conf < cfg.CONF_MIN:
        return None
    bbox = pose.get("bbox")
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        w, h = (x2 - x1), (y2 - y1)
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    elif pose.get("cx") is not None:
        cx, cy = pose["cx"], pose["cy"]
        w = pose.get("w", 0.0)
        h = pose.get("h", 0.0)
    else:
        return None
    if math.sqrt(max(w, 0.0) * max(h, 0.0)) < cfg.BOYUT_MIN:
        return None
    return cx, cy, w, h, conf


def run_bbox_ibvs(conn, get_iris, wait_pose, stop_event, cfg=Cfg,
                  kayip_kare_esik=20, ff_hiz=(0.0, 0.0, 0.0)):
    """bbox IBVS görsel güdüm döngüsü. Kutu akışına kilitli (wait_pose).

    ff_hiz: DONDURULMUŞ taşıyıcı (vx,vy,vz) NED — devir anında supervisor'ın
    okuduğu son GPS hız kestirimi. ⚠ SAYI ÜÇLÜSÜ, callback DEĞİL: döngünün
    görsel faz boyunca canlı GPS'e erişimi yoktur (D0 kuralı yapısal garanti).

    kayip_kare_esik ardışık geçersiz-kutu karesi → 'kayip' döner (görsel temas
    kesildi; supervisor GPS fazına döner). stop_event → 'durduruldu'.
    """
    loop_period = 1.0 / cfg.LOOP_HZ
    son_seq = 0
    kayip_sayac = 0
    ff = (float(ff_hiz[0]), float(ff_hiz[1]), float(ff_hiz[2]))
    # İvme sınırlayıcı drone'un GERÇEK hızından başlar (kendi sensörü).
    # Sıfırdan başlarsa devir anında 15 m/s'lik seyir "frenlenmiş" gibi
    # rampalanır (12 m/s² ile 1.25 s) — hedef o sırada kaçar.
    _i0 = get_iris()
    vx_p = float(_i0.get("vx", 0.0) or 0.0)
    vy_p = float(_i0.get("vy", 0.0) or 0.0)
    vz_p = float(_i0.get("vz", 0.0) or 0.0)
    prev_time = None
    cmd_yaw = None

    os.makedirs(_LOG_DIR, exist_ok=True)
    csv_yol = os.path.join(_LOG_DIR, time.strftime("bbox_ibvs_%Y%m%d_%H%M%S.csv"))
    f = open(csv_yol, "w", newline="")
    w_csv = csv.DictWriter(f, fieldnames=_CSV_ALANLAR, extrasaction="ignore")
    w_csv.writeheader()
    print(f"[IBVS] bbox görsel güdüm başladı (CANLI GPS YOK — yarışma kuralı) — "
          f"dondurulmuş taşıyıcı=({ff[0]:+.1f},{ff[1]:+.1f},{ff[2]:+.1f}) m/s "
          f"|{math.hypot(ff[0], ff[1]):.1f}|, REF={cfg.BOYUT_REF:.0f}px, "
          f"CY_nişan={cfg.CY_NISAN:.0f}, kayıp eşiği={kayip_kare_esik} kare "
          f"— log: {csv_yol}")

    try:
        while not stop_event.is_set():
            kayit = wait_pose(son_seq, timeout=0.5)
            if kayit is None:
                # kare akışı durdu — temas kesildi say (akış yoksa ilerleme yok)
                kayip_sayac += 1
                if kayip_sayac >= kayip_kare_esik:
                    print("[IBVS] kare akışı/temas kesildi → 'kayip'")
                    return "kayip"
                continue
            son_seq = kayit["seq"]

            now = time.monotonic()
            dt = (now - prev_time) if prev_time is not None else loop_period
            dt = clamp(dt, 0.001, 0.5)
            prev_time = now

            iris = get_iris()
            iyaw = iris.get("yaw", 0.0)

            kutu = _kutu_gecerli(kayit["pose"], cfg)
            if kutu is None:
                kayip_sayac += 1
                if kayip_sayac >= kayip_kare_esik:
                    print(f"[IBVS] {kayip_kare_esik} ardışık kutusuz kare → 'kayip'")
                    return "kayip"
                # Kutu yok: kapanma kesilir ama TAŞIYICI sürer — hedefin seyri
                # bir karede değişmez; sıfır komut vermek 15 m/s'lik farkı
                # açar ve kısa bir tespit boşluğunu kalıcı kayba çevirir.
                send_velocity(conn, ff[0], ff[1], ff[2], cmd_yaw or iyaw)
                w_csv.writerow({"t": round(now, 3), "dt": round(dt, 4),
                                "durum": "KUTU_YOK", "kayip_sayac": kayip_sayac,
                                "iris_yaw_deg": round(math.degrees(iyaw), 1)})
                f.flush()
                continue

            kayip_sayac = 0
            cx, cy, bw, bh, conf = kutu
            vx, vy, vz, yaw_cmd, tani = komut(cx, cy, bw, bh, iyaw, ff, cfg)
            cmd_yaw = yaw_cmd

            # ivme sınırı (komut hızı sıçramasın)
            vx, vy, vz = limit_acceleration(vx, vy, vz, vx_p, vy_p, vz_p,
                                            cfg.MAX_ACCEL, dt)
            vx_p, vy_p, vz_p = vx, vy, vz
            send_velocity(conn, vx, vy, vz, yaw_cmd)

            w_csv.writerow({
                "t": round(now, 3), "dt": round(dt, 4), "durum": "IBVS",
                "cx": round(cx, 1), "cy": round(cy, 1),
                "w": round(bw, 1), "h": round(bh, 1),
                "boyut": round(tani["boyut"], 1), "conf": round(conf, 3),
                "eps_yaw_deg": round(math.degrees(tani["eps_yaw"]), 1),
                "eps_elev_deg": round(math.degrees(tani["eps_elev"]), 1),
                "iris_yaw_deg": round(math.degrees(iyaw), 1),
                "v_kapanma": round(tani["v_kapanma"], 2),
                "ff_vx": round(ff[0], 2), "ff_vy": round(ff[1], 2),
                "ff_vz": round(ff[2], 2),
                "vx_cmd": round(vx, 2), "vy_cmd": round(vy, 2),
                "vz_cmd": round(vz, 2),
                "yaw_cmd_deg": round(math.degrees(yaw_cmd), 1),
                "kayip_sayac": 0,
            })
            f.flush()

            _elapsed = time.monotonic() - now
            if _elapsed < loop_period:
                time.sleep(loop_period - _elapsed)

        send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)
        return "durduruldu"
    finally:
        f.close()
        print(f"[IBVS] log kapatıldı: {csv_yol}")
