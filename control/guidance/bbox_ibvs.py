"""
bbox_ibvs.py — SAF görüntü tabanlı görsel güdüm (IBVS), yalnız bbox.

YARIŞMA KURALI (üstün kısıt, bkz. UYGULANACAK.md D0): görsel temas varken
hedefin GPS'i güdümde KULLANILAMAZ. Bu modül yalnız tespit kutusunu (cx, cy,
w, h, conf) ve drone'un KENDİ attitude'unu (yaw — gövde→NED dönüşümü için)
kullanır. Hedef GPS'i, hedef hızı, menzil kestirimi HİÇBİRİ girmez.

Kontrol yasası (üç kanal, hepsi piksel uzayından):
  YAW     : yatay piksel hatası (cx − CX) → burun hedefe döner.
  DİKEY   : dikey piksel hatası (cy − CY_NISAN) → tırman/alçal (vz, NED down+).
  İLERİ   : kutu büyüklüğü menzil vekili — büyük kutu = yakın = yavaşla.
            Kutu küçükse (uzak) tam yaklaşma hızı; büyüdükçe azalır, tabanlı.

Neden saf: "ekran ortasında → ileri ver" yanılsaması ancak hedef gerçekten
önümüzdeyken doğrudur; kuyruk-benzeri girişte (yarışma hattı, GPS fazı 66°
kuyruktan devrediyor) bu güvenli. Yandan girişte değil — o yüzden GPS fazı
devri yalnız kuyruğa yakınken açar (supervisor kapısı).

Arayüz (supervisor.run_hybrid ile uyumlu):
  run_bbox_ibvs(conn, get_iris, wait_pose, stop_event, cfg, kayip_kare_esik)
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

    # ── İLERİ (kutu büyüklüğü = menzil vekili) ──
    # boyut = sqrt(w·h). Büyük kutu = yakın. İleri hız boyut REF'e yaklaştıkça
    # azalır: v = clamp(K_FWD·(REF − boyut), V_GERI, V_ILERI). Kutu kaybolursa
    # (küçük/None) döngü kayıp sayar; burada değil.
    # REF: terminal-öncesi tutulmak istenen görünür boyut. ~8-10 m'de kutu
    # tarafına göre ~60-90 px; REF=70 kaba başlangıç, uçuşla kalibre edilecek.
    BOYUT_REF = _env_f("AVCI_IBVS_REF", 70.0)   # px; sqrt(w·h) hedef boyut
    K_FWD = _env_f("AVCI_IBVS_KFWD", 0.14)      # (m/s)/px
    V_ILERI_MAX = _env_f("AVCI_IBVS_VMAX", 16.0)  # m/s
    V_GERI_MAX = 3.0               # m/s; çok yaklaşınca hafif geri (tavan)
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
    "v_fwd", "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg", "kayip_sayac",
]


def komut(cx, cy, w, h, iris_yaw, cfg=Cfg):
    """SAF IBVS kontrol yasası (MAVLink yok, GPS yok). Test edilebilir.

    Girdi: kutu merkezi (cx,cy) px, kutu boyu (w,h) px, drone yaw (rad).
    Çıktı: (vx_ned, vy_ned, vz, yaw_cmd, tani)  — tani = ara değerler dict'i.
    """
    boyut = math.sqrt(max(w, 0.0) * max(h, 0.0))

    # YAW: yatay açı hatası → burun hedefe
    eps_yaw = math.atan((cx - cfg.CX_NISAN) / geo.FX)
    yaw_cmd = normalize_angle(iris_yaw + cfg.K_YAW * eps_yaw)

    # İLERİ: kutu boyutu menzil vekili
    v_fwd = clamp(cfg.K_FWD * (cfg.BOYUT_REF - boyut),
                  -cfg.V_GERI_MAX, cfg.V_ILERI_MAX)

    # Gövde ileri → NED (burun yönü = yaw_cmd)
    vx_ned = v_fwd * math.cos(yaw_cmd)
    vy_ned = v_fwd * math.sin(yaw_cmd)

    # DİKEY: elev hatası → tırman/alçal
    eps_elev = math.atan((cy - cfg.CY_NISAN) / geo.FY)   # cy büyük → hedef altta
    vz = clamp(cfg.K_VZ * cfg.V_NOM * eps_elev, -cfg.VZ_MAX, cfg.VZ_MAX)

    tani = {"boyut": boyut, "eps_yaw": eps_yaw, "eps_elev": eps_elev,
            "v_fwd": v_fwd}
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
                  kayip_kare_esik=20):
    """SAF bbox IBVS görsel güdüm döngüsü. Kutu akışına kilitli (wait_pose).

    kayip_kare_esik ardışık geçersiz-kutu karesi → 'kayip' döner (görsel temas
    kesildi; supervisor GPS fazına döner). stop_event → 'durduruldu'.
    """
    loop_period = 1.0 / cfg.LOOP_HZ
    son_seq = 0
    kayip_sayac = 0
    vx_p = vy_p = vz_p = 0.0
    prev_time = None
    cmd_yaw = None

    os.makedirs(_LOG_DIR, exist_ok=True)
    csv_yol = os.path.join(_LOG_DIR, time.strftime("bbox_ibvs_%Y%m%d_%H%M%S.csv"))
    f = open(csv_yol, "w", newline="")
    w_csv = csv.DictWriter(f, fieldnames=_CSV_ALANLAR, extrasaction="ignore")
    w_csv.writeheader()
    print(f"[IBVS] SAF bbox görsel güdüm başladı (GPS YOK — yarışma kuralı) — "
          f"REF={cfg.BOYUT_REF:.0f}px, CY_nişan={cfg.CY_NISAN:.0f}, "
          f"kayıp eşiği={kayip_kare_esik} kare — log: {csv_yol}")

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
                # kutu yok: son yaw'ı koru, ileri kes (kör ilerleme yok)
                send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or iyaw)
                w_csv.writerow({"t": round(now, 3), "dt": round(dt, 4),
                                "durum": "KUTU_YOK", "kayip_sayac": kayip_sayac,
                                "iris_yaw_deg": round(math.degrees(iyaw), 1)})
                f.flush()
                continue

            kayip_sayac = 0
            cx, cy, bw, bh, conf = kutu
            vx, vy, vz, yaw_cmd, tani = komut(cx, cy, bw, bh, iyaw, cfg)
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
                "v_fwd": round(tani["v_fwd"], 2),
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
