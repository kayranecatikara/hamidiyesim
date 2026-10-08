#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
TARET DENEME — "taret gerçekten dönüyor mu, kamera onunla dönüyor mu?"
================================================================================
NİYE VAR (CLAUDE.md §5.1 MEKANİZMA KAPISI):
  Bir özelliğin eklendiğini SDF'te görmek kanıt değildir. Taret için sorulacak
  iki soru var ve ikisi de ÖLÇÜLMELİ:
    1. Eklem komut edilen açıya GİDİYOR mu? (eklem durumu okunur)
    2. KAMERA o açıyla birlikte dönüyor mu? (kare kaydedilir, GÖZLE bakılır)
  İkincisi olmadan birincisi yetmez: eklem dönüp kamera dönmüyorsa
  "namluyla aynı doğrultu" iddiası çürür.

NASIL ÇALIŞIR:
  - gz-transport'tan /iris_cam/image konusuna abone olur (30 Hz, R8G8B8)
  - her duruş için  /model/<araç>/joint/<eklem>/0/cmd_pos  konusuna açı yazar
  - oturmasını bekler, eklem durumunu okur, o anki kareyi JPG kaydeder
  - komut ile gerçekleşen açı arasındaki hatayı raporlar

KULLANIM:
    python3 tools/taret_dene.py --dizin logs/vakkas/taret
================================================================================
"""
import argparse
import math
import os
import re
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

ARAC = "iris_with_ardupilot"
DUNYA = "avci"
KAM_KONU = "/iris_cam/image"

# (pan°, tilt°, ad) — taretin tasarım zarfının köşeleri
DURUSLAR = [
    (0,    25,  "00_varsayilan_pan0_tilt25"),
    (0,     0,  "01_duz_ileri"),
    (0,    90,  "02_tam_yukari"),
    (0,   -30,  "03_alt_sinir"),
    (90,    0,  "04_tam_sol"),
    (-90,   0,  "05_tam_sag"),
    (180,   0,  "06_tam_geri"),
    (45,   45,  "07_capraz"),
    (0,    25,  "08_varsayilana_donus"),
]


def eklem_durumu():
    """Gazebo'dan taret eklemlerinin GERÇEK açılarını oku (derece)."""
    try:
        ham = subprocess.run(
            ["gz", "topic", "-e", "-t",
             "/world/%s/model/%s/joint_state" % (DUNYA, ARAC), "-n", "1"],
            capture_output=True, text=True, timeout=6).stdout
    except Exception:
        return {}
    cikti = {}
    for blok in ham.split("joint {"):
        ad = re.search(r'name:\s*"([^"]+)"', blok)
        if not ad or "turret" not in ad.group(1):
            continue
        poz = re.search(r"position:\s*([-\d.eE+]+)", blok)
        hiz = re.search(r"velocity:\s*([-\d.eE+]+)", blok)
        cikti[ad.group(1)] = (
            math.degrees(float(poz.group(1))) if poz else None,
            float(hiz.group(1)) if hiz else None)
    return cikti


def komut(eklem, aci_rad):
    subprocess.run(
        ["gz", "topic", "-t",
         "/model/%s/joint/%s/0/cmd_pos" % (ARAC, eklem),
         "-m", "gz.msgs.Double", "-p", "data: %.6f" % aci_rad],
        capture_output=True, timeout=6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dizin", default="logs/vakkas/taret")
    ap.add_argument("--bekle", type=float, default=6.0,
                    help="oturma için EN FAZLA beklenecek saniye")
    ap.add_argument("--aci-esik", type=float, default=0.5,
                    help="oturmuş sayılmak için açı hatası tavanı (derece)")
    ap.add_argument("--hiz-esik", type=float, default=0.05,
                    help="oturmuş sayılmak için açısal hız tavanı (rad/s)")
    a = ap.parse_args()
    os.makedirs(a.dizin, exist_ok=True)

    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.image_pb2 import Image as GzImage
    except Exception as e:
        print("⛔ gz-transport Python bağı yok: %s" % e)
        return 1

    kutu = {"kare": None, "n": 0}
    kilit = threading.Lock()

    def cb(msg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            with kilit:
                kutu["kare"] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                kutu["n"] += 1
        except Exception:
            pass

    node = GzNode()
    if not node.subscribe(GzImage, KAM_KONU, cb):
        print("⛔ %s konusuna abone olunamadı" % KAM_KONU)
        return 1
    print("kamera dinleniyor: %s" % KAM_KONU)

    t0 = time.time()
    while kutu["n"] == 0 and time.time() - t0 < 15:
        time.sleep(0.2)
    if kutu["n"] == 0:
        print("⛔ 15 s'de hiç kare gelmedi — Gazebo ayakta mı?")
        return 1
    with kilit:
        h, w = kutu["kare"].shape[:2]
    print("ilk kare geldi: %dx%d  (%d kare)" % (w, h, kutu["n"]))

    print("\n" + "=" * 78)
    print("  %-26s %14s %16s   %13s %s"
          % ("duruş", "komut (pan,tilt)", "gerçek (pan,tilt)", "hata", "oturma"))
    print("=" * 78)
    hatalar = []
    for pan, tilt, ad in DURUSLAR:
        # ⛔ SABİT SÜRE BEKLEMEK ÖLÇÜM HATASIDIR: ilk sürümde 2 s bekleniyordu
        #   ve 90°'lik komutta "5.97° hata" çıkmıştı. Gerçekte eklem daha
        #   oturmamıştı. Doğrusu OTURANA KADAR beklemek ve OTURMA SÜRESİNİ
        #   ölçmektir — bu zaten taretin asıl performans ölçütüdür.
        t_bas = time.time()
        komut("turret_yaw_joint", math.radians(pan))
        komut("turret_pitch_joint", math.radians(tilt))
        gp = gt = None
        oturma = float("nan")
        while time.time() - t_bas < a.bekle:
            d = eklem_durumu()
            gp = d.get("turret_yaw_joint", (None, None))[0]
            gt = d.get("turret_pitch_joint", (None, None))[0]
            vp = d.get("turret_yaw_joint", (None, None))[1]
            vt = d.get("turret_pitch_joint", (None, None))[1]
            if None in (gp, gt, vp, vt):
                continue
            # ±180° sınırına dayanan komutlar orada KİLİTLENİR; hatayı
            # eklem sınırına göre değerlendir.
            if (abs(gp - pan) <= a.aci_esik and abs(gt - tilt) <= a.aci_esik
                    and abs(vp) < a.hiz_esik and abs(vt) < a.hiz_esik):
                oturma = time.time() - t_bas
                break
        hp = abs(gp - pan) if gp is not None else float("nan")
        ht = abs(gt - tilt) if gt is not None else float("nan")
        hatalar.append((ad, hp, ht, oturma))
        with kilit:
            kare = None if kutu["kare"] is None else kutu["kare"].copy()
        if kare is not None:
            etiket = "pan %+.0f  tilt %+.0f" % (gp or 0, gt or 0)
            cv2.putText(kare, etiket, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 0), 3)
            cv2.putText(kare, etiket, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 1)
            # nişan haçı: namlu ekseni kadrajın tam ortasıdır
            cv2.drawMarker(kare, (w // 2, h // 2), (0, 0, 255),
                           cv2.MARKER_CROSS, 26, 1)
            cv2.imwrite(os.path.join(a.dizin, "%s.jpg" % ad), kare)
        print("  %-26s   %+5.0f , %+5.0f    %+7.2f , %+7.2f   %5.2f°, %5.2f°   %s"
              % (ad, pan, tilt, gp if gp is not None else float("nan"),
                 gt if gt is not None else float("nan"), hp, ht,
                 ("%.2f s" % oturma) if oturma == oturma else "OTURMADI"))

    print("-" * 78)
    enb = max(max(h1, h2) for _, h1, h2, _t in hatalar
              if not (math.isnan(h1) or math.isnan(h2)))
    sureler = [t for _, _, _, t in hatalar if t == t]
    print("  en büyük açı hatası : %.2f°" % enb)
    if sureler:
        print("  oturma süresi       : en kısa %.2f s · en uzun %.2f s"
              % (min(sureler), max(sureler)))
    if len(sureler) < len(hatalar):
        print("  ⛔ %d duruş %.0f s içinde OTURMADI"
              % (len(hatalar) - len(sureler), a.bekle))
    print("  kareler: %s/" % a.dizin)
    if enb < 1.0:
        print("  ✔ MEKANİZMA KAPISI GEÇTİ — taret komut edilen açılara gidiyor.")
    else:
        print("  ⛔ MEKANİZMA KAPISI DÜŞTÜ — eklem komuta oturmuyor (>1°).")
    print("  ⚠ Kameranın taretle döndüğü GÖRÜNTÜLERE BAKARAK doğrulanır;")
    print("    yalnız bu tablo yetmez (CLAUDE.md §2 adım 4).")
    return 0 if enb < 1.0 else 1


if __name__ == "__main__":
    sys.exit(main())
