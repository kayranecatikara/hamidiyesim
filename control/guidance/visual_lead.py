"""
visual_lead.py — IBVS lead pursuit döngüsü (olay güdümlü, kameraya kilitli).

Sabit Hz'te DÖNMEZ: kamera 30 Hz, kare geldikçe işler (sabit döngü kare tekrarı
ve bayat veri üretir). Her yeni pose kaydında:
  bayat kare kapısı → GUIDED kontrolü → guidance_core.process → adaptör → CSV.

Saat tasarımı (sim/duvar saati birbirine karıştırılmaz):
  dt        = ardışık kare header.stamp farkı (sim saati) — filtre/rampa bunu kullanır
  gecikme_s = time.time() - wall_recv (karenin gcs'e geliş duvar anı; komut anında)
  gecikme > GECIKME_TAVAN_S → kare atlanır, komut GÖNDERİLMEZ, son komut korunur.

Kullanım (gcs_server): run_visual_lead(conn, wait_pose, get_plane_truth, stop_event)
  wait_pose        : vision.detection_state.wait_new_pose
  get_plane_truth  : hedefin GERÇEK NED pozu (çerçeve-ofset düzeltmeli) — SADECE
                     menzil_gercek/kapanma_hizi logu için, güdüme girmez.
"""

import csv
import math
import os
import time

from control import mav_common
from control.guidance.adapter_copter import CopterAdapter
from control.guidance.adapter_fixedwing import FixedWingAdapter
from control.guidance.guidance_core import Cfg, LeadPursuitCore, govde_to_dunya

_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

_CSV_ALANLAR = [
    "t_ros", "dt", "gecikme_s", "bbox", "a", "b", "olcek_ham", "eps_deg",
    "duzeltme", "olcek", "yandanlik_ham", "yandanlik_filtreli", "kalite",
    "lead_deg", "u_nisan_x", "u_nisan_y", "u_nisan_z",
    "u_govde_x", "u_govde_y", "u_govde_z", "yaw_hata_deg", "pitch_hata_deg",
    "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg", "v_doygun", "yaw_doygun",
    "durum", "flip_sayaci", "eksen_disi_deg", "govde_yukselti_deg",
    "menzil_kestirim_m", "menzil_gercek_m", "kapanma_hizi_ms", "mod",
    "pitch_body_deg", "kamera_dunya_pitch_deg",
]

# durum kodları (CSV): ok / cozumsuz / kanat_dusuk / kpt_dusuk / tespit_yok /
#                      bayat / mod_hata / attitude_yok


class _ArasState:
    """Kendi aracımızın MAVLink durumunu conn'dan non-blocking drenajla günceller
    (visual thread aktifken iris telem worker durur — port deseni gcs'te)."""
    def __init__(self):
        self.attitude = None      # (roll, pitch, yaw) rad
        self.mode = None          # HEARTBEAT custom_mode
        self.pos = None           # (x, y, z) NED, iris çerçevesi

    def drenaj(self, conn):
        while True:
            msg = conn.recv_match(
                type=["ATTITUDE", "HEARTBEAT", "LOCAL_POSITION_NED"],
                blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "ATTITUDE":
                self.attitude = (msg.roll, msg.pitch, msg.yaw)
            elif t == "HEARTBEAT" and msg.get_srcSystem() != 255:
                self.mode = msg.custom_mode
            elif t == "LOCAL_POSITION_NED":
                self.pos = (msg.x, msg.y, msg.z)


def run_visual_lead(conn, wait_pose, get_plane_truth, stop_event, cfg=Cfg,
                    kayip_kare_esik=None):
    """kayip_kare_esik verilirse (supervisor hibrit modu): bu kadar ARDIŞIK
    pose'suz kare → 'kayip' döner (görsel temas kesildi, GPS'e dönülecek).
    Dönüş: 'durduruldu' (stop_event) | 'kayip' (temas kaybı)."""
    core = LeadPursuitCore(cfg)
    if cfg.PLATFORM == "copter":
        adapter = CopterAdapter(cfg)
    else:
        adapter = FixedWingAdapter(cfg)   # stub: command() NotImplementedError

    aras = _ArasState()
    son_seq = 0            # _pose_seq 0'dan başlar; ilk GERÇEK kareyi bekle
    menzil_onceki = None
    t_menzil_onceki = None
    bayat_sayaci = 0
    kayip_sayaci = 0       # ardışık pose'suz kare (temas kaybı takibi)
    son_kayit_wall = time.time()

    os.makedirs(_LOG_DIR, exist_ok=True)
    csv_yol = os.path.join(_LOG_DIR,
                           time.strftime("visual_lead_%Y%m%d_%H%M%S.csv"))
    f = open(csv_yol, "w", newline="")
    w = csv.DictWriter(f, fieldnames=_CSV_ALANLAR, extrasaction="ignore")
    w.writeheader()
    print(f"[LEAD] IBVS lead pursuit başladı (platform={cfg.PLATFORM}, "
          f"K_LEAD={cfg.K_LEAD}, V_KAPANMA={cfg.V_KAPANMA}) — log: {csv_yol}")

    def _satir(row):
        w.writerow(row)
        f.flush()

    try:
        while not stop_event.is_set():
            kayit = wait_pose(son_seq, timeout=0.5)
            if kayit is None:                 # yeni kare yok (timeout)
                if (kayip_kare_esik is not None
                        and time.time() - son_kayit_wall > 1.0):
                    print("[LEAD WARN] kare akışı kesildi (>1 s) — temas kaybı")
                    return "kayip"
                continue
            son_seq = kayit["seq"]
            son_kayit_wall = time.time()
            pose, stamp = kayit["pose"], kayit["stamp"]
            wall_recv = kayit["wall_recv"]

            aras.drenaj(conn)
            satir = {"t_ros": stamp, "flip_sayaci": core.flip_sayaci,
                     "mod": aras.mode}

            # menzil_gercek + kapanma hızı — SADECE LOG (güdüme girmez)
            if aras.pos is not None and get_plane_truth is not None:
                p = get_plane_truth()
                if p is not None:
                    d = math.sqrt((p["x"] - aras.pos[0]) ** 2
                                  + (p["y"] - aras.pos[1]) ** 2
                                  + (p["z"] - aras.pos[2]) ** 2)
                    satir["menzil_gercek_m"] = round(d, 3)
                    if menzil_onceki is not None and stamp and t_menzil_onceki \
                            and stamp > t_menzil_onceki:
                        satir["kapanma_hizi_ms"] = round(
                            -(d - menzil_onceki) / (stamp - t_menzil_onceki), 2)
                    menzil_onceki, t_menzil_onceki = d, stamp

            if pose is None:                  # bu karede tespit yok → komut yok
                satir["durum"] = "tespit_yok"
                _satir(satir)
                kayip_sayaci += 1
                if kayip_kare_esik is not None and kayip_sayaci >= kayip_kare_esik:
                    print(f"[LEAD WARN] görsel temas kesildi "
                          f"({kayip_sayaci} ardışık kare tespit yok)")
                    return "kayip"
                continue
            kayip_sayaci = 0                  # pose var → temas sürüyor

            # bayat kare kapısı (duvar saati — aynı saat cinsinden ölçüm)
            gecikme = (time.time() - wall_recv) if wall_recv else 0.0
            satir["gecikme_s"] = round(gecikme, 4)
            satir["bbox"] = "|".join(str(v) for v in pose["bbox"])
            if gecikme > cfg.GECIKME_TAVAN_S:
                bayat_sayaci += 1
                satir["durum"] = "bayat"
                print(f"[LEAD WARN] bayat kare atlandı ({gecikme*1000:.0f} ms "
                      f"> {cfg.GECIKME_TAVAN_S*1000:.0f} ms, toplam {bayat_sayaci})")
                _satir(satir)
                continue                      # komut GÖNDERME, son komut korunur

            # GUIDED kontrolü
            if aras.mode != mav_common.COPTER_MODE_GUIDED:
                satir["durum"] = "mod_hata"
                print(f"[LEAD ERROR] mod GUIDED değil (custom_mode={aras.mode}) "
                      f"— komut gönderilmiyor")
                _satir(satir)
                continue

            res = core.process(pose, stamp, aras.attitude)
            for warntip in res["warn"]:
                print(f"[LEAD WARN] {warntip} (kare t={stamp:.3f})")

            satir.update({
                "dt": res["dt"], "a": round(res["a"], 2), "b": round(res["b"], 2),
                "olcek_ham": round(res["olcek_ham"], 2),
                "eps_deg": round(res["eps_deg"], 2),
                "duzeltme": round(res["duzeltme"], 4),
                "olcek": round(res["olcek"], 2),
                "yandanlik_ham": round(res["yandanlik_ham"], 4),
                "yandanlik_filtreli": round(res["yandanlik_f"], 4),
                "kalite": round(res["kalite"], 3),
                "lead_deg": round(res["lead_deg"], 2),
                "u_nisan_x": round(float(res["u_nisan"][0]), 5),
                "u_nisan_y": round(float(res["u_nisan"][1]), 5),
                "u_nisan_z": round(float(res["u_nisan"][2]), 5),
                "u_govde_x": round(float(res["u_govde"][0]), 5),
                "u_govde_y": round(float(res["u_govde"][1]), 5),
                "u_govde_z": round(float(res["u_govde"][2]), 5),
                "yaw_hata_deg": round(res["yaw_hata_deg"], 2),
                "pitch_hata_deg": round(res["pitch_hata_deg"], 2),
                "durum": res["durum"], "flip_sayaci": res["flip_sayaci"],
                "eksen_disi_deg": round(res["eksen_disi_deg"], 2),
                "govde_yukselti_deg": round(res["pitch_hata_deg"], 2),
                "menzil_kestirim_m": round(res["menzil_kestirim_m"], 2),
            })

            if aras.attitude is not None:
                mevcut_yaw = aras.attitude[2]
                cmd = adapter.command(conn, res["u_govde"], res["yaw_hata"],
                                      aras.attitude, res["dt"], mevcut_yaw)
                satir.update({
                    "vx_cmd": round(cmd["v_cmd"][0], 2),
                    "vy_cmd": round(cmd["v_cmd"][1], 2),
                    "vz_cmd": round(cmd["v_cmd"][2], 2),
                    "yaw_cmd_deg": round(math.degrees(cmd["yaw_cmd"]), 1),
                    "v_doygun": int(cmd["v_doygun"]),
                    "yaw_doygun": int(cmd["yaw_doygun"]),
                })
                # quad'a özgü izleme: burun eğimi + kameranın DÜNYAYA göre bakışı
                # (ivme tavanı aşılırsa kamera yere bakmaya başlar — Cfg.IVME_TAVAN)
                pitch_body = math.degrees(aras.attitude[1])
                satir["pitch_body_deg"] = round(pitch_body, 2)
                kam = govde_to_dunya(
                    [math.cos(math.radians(cfg.KAMERA_TILT_DEG)), 0.0,
                     -math.sin(math.radians(cfg.KAMERA_TILT_DEG))],
                    *aras.attitude)
                satir["kamera_dunya_pitch_deg"] = round(
                    math.degrees(math.asin(max(-1.0, min(1.0, -float(kam[2]))))), 2)
            else:
                satir["durum"] = satir["durum"] if res["warn"] else "attitude_yok"

            _satir(satir)
        return "durduruldu"
    finally:
        f.close()
        print(f"[LEAD] IBVS lead pursuit durdu — log kapatıldı: {csv_yol}")
