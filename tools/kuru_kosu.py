"""Adım 8 kuru-koşu: sim başlatmadan overlay+log+komut_kaydı zinciri.

repo kökünden: PYTHONPATH=. python tools/kuru_kosu.py

Sentetik tek kare ile: KilitTakip → kilit_overlay.ciz → kilit_log.kaydet,
komut_kaydı sarmalayıcısı gerçek send_velocity'yi kaydediyor mu doğrular.
"""
import os
import numpy as np

from control.kilitlenme import KilitTakip
from vision import kilit_overlay
from control import kilit_log
from control import komut_kaydi

# 1) Komut kaydı sarmalayıcısını kur (idempotens: iki kez çağır)
n1 = komut_kaydi.kur()
n2 = komut_kaydi.kur()          # ikinci çağrı çifte sarmamalı → 0
print(f"komut_kaydi.kur(): 1.={n1} modül, 2.={n2} (idempotent)")

# 2) Sarmalayıcı gerçekten kaydediyor mu? (sahte conn ile send_velocity çağır)
from control.guidance import gps_guidance as g
class _SahteConn:
    class mav:
        @staticmethod
        def set_position_target_local_ned_send(*a, **k):  # DoW köprüsü no-op
            pass
    target_system = target_component = 1
g.send_velocity(_SahteConn(), 1.0, 2.0, 3.0, 0.5)
print(f"son_komut yakalandı: {komut_kaydi.get_son_komut()}")

# 3) Sentetik kare + KilitTakip durumu (merkezde, kilit sağlayan bbox)
W, H = 640, 480
frame = np.full((H, W, 3), 40, np.uint8)
kt = KilitTakip(W, H)
cx, cy, bw, bh = W // 2, H // 2, int(0.30 * W), int(0.30 * H)
bbox = (cx - bw // 2, cy - bh // 2, cx + bw // 2, cy + bh // 2)
kd = None
for i in range(20):            # ~0.6 s yoğun besleme → doğrulama + kilit
    kd = kt.guncelle(bbox, now=1.0 + i * 0.033)
print(f"kdurum: faz={kd['faz']} kilit={kd['anlik_kilit']} marj={kd['marj']:.2f} "
      f"dogr={kd['tespit_dogrulandi']} kum={kd['kumulatif_s']:.2f}")

# 4) Overlay çiz (istisna atmamalı, boyut korunmalı)
before = frame.shape
img = kilit_overlay.ciz(frame, kd, t_sim=1.66)
assert img.shape == before, "overlay boyutu değiştirdi"
out_png = os.path.join(os.path.dirname(__file__), "kuru_kosu_overlay.png")
try:
    import cv2
    cv2.imwrite(out_png, img)
    print(f"overlay çizildi → {out_png}")
except Exception as e:
    print(f"overlay çizildi (png yazılamadı: {e})")

# 5) CSV log: tespitli + tespitsiz (bbox boş) satır yaz, kapat, oku
csv_yol = kilit_log.baslat()
def _row(kd, t, los=(0.1, 0.2, 0.97), menzil=12.3, ref=11.0, komut=(1.0, 2.0, 3.0, 0.5)):
    ah = kd.get("ah_kutu") if kd else None
    x1, y1, x2, y2 = ah if ah else ("", "", "", "")
    return {"t_sim": t, "faz": (kd or {}).get("faz"), "gorev_faz": "VISUAL",
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "marj": round((kd or {}).get("marj", 0.0), 3),
            "kumulatif_s": round((kd or {}).get("kumulatif_s", 0.0), 3),
            "kesintisiz_s": round((kd or {}).get("kesintisiz_s", 0.0), 3),
            "kilit": int(bool((kd or {}).get("anlik_kilit"))),
            "kilit_isteri_ok": int(bool((kd or {}).get("kilit_isteri_ok"))),
            "tespit_dogrulandi": int(bool((kd or {}).get("tespit_dogrulandi"))),
            "menzil": menzil, "menzil_ref": ref,
            "vx": komut[0], "vy": komut[1], "vz": komut[2], "yaw": komut[3],
            "los_x": los[0], "los_y": los[1], "los_z": los[2]}
kilit_log.kaydet(_row(kd, 1.66))
kilit_log.kaydet(_row(None, 1.70))     # tespitsiz kare (bbox boş) — ş3
kilit_log.kapat()
with open(csv_yol) as f:
    satirlar = f.read().strip().splitlines()
print(f"CSV {csv_yol}: {len(satirlar)} satır (1 başlık + 2 veri), düşen={kilit_log.dusen_sayisi()}")
print("  başlık:", satirlar[0][:70], "...")
print("  veri1 :", satirlar[1][:70], "...")
print("  veri2 :", satirlar[2][:70], "...  (bbox boş)")
print("KURU-KOŞU OK")
