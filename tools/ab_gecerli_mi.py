#!/usr/bin/env python3
"""ab_gecerli_mi.py — iki A/B kolunun KIYASLANABİLİR olup olmadığını denetler.

NEDEN VAR
---------
2026-08-08'de dört A/B ("PN_KAPI, PN_MAX, V_KAPANMA, IVME") yapıldı, hepsi
"fark yok" çıktı ve öyle raporlandı. Sonradan ölçüldü ki hedef uçak DAİRE
senaryosunda irtifa tutmuyor (+35…+92 m/dk, hiç oturmuyor); iki kol 134-175 m
farklı irtifada uçmuş. Yani ölçtüğümüz şey değişikliğin etkisi değildi.
Dördü de çöpe gitti.

Bu araç o kontrolü gözden alıp koda veriyor: kıyastan ÖNCE çalıştır, KIRMIZI
derse o A/B'yi rapor etme.

KULLANIM
--------
    PYTHONPATH=. python3 tools/ab_gecerli_mi.py          # son iki koşu
    PYTHONPATH=. python3 tools/ab_gecerli_mi.py A.csv B.csv

Çıkış kodu: kollar kıyaslanabilirse 0, değilse 1.
"""
from __future__ import annotations

import csv
import glob
import os
import sys

_KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOGLAR = os.path.join(_KOK, "logs")

# Eşikler — gerekçeleri:
#   irtifa 25 m : ölçülen tırmanma 35-92 m/dk; 25 m'lik fark ~20-40 saniyelik
#                 kayma demek, menzil/geometri sonucunu görünür şekilde kaydırır.
#   süre  %40   : bir kol yarı sürede biterse (ör. erken vuruş) örneklem sayısı
#                 kıyaslanamaz.
#   asgari 40 s : altında irtifa platosuna oturmamış olabilir.
#   tırmanma ±15 m/dk : düz uçuşta plato ölçüldü (65 m ve 134 m'de tam 0.0);
#                 dairede 35-92 m/dk. 15 ikisini ayırır. Bu eşik ASIL SEBEBİ
#                 yakalar: kollar aynı irtifada başlasa bile hedef tırmanmaya
#                 devam ediyorsa fark koşu boyunca büyür.
IRTIFA_ESIK_M = 25.0
SURE_ORAN_ESIK = 0.40
ASGARI_SURE_S = 40.0
TIRMANMA_ESIK_MDK = 15.0


def _son_ikisi():
    d = sorted(glob.glob(os.path.join(_LOGLAR, "gps_guidance_*.csv")),
               key=os.path.getmtime)
    # Aynı oturumda birden çok kısa dosya oluşabiliyor; anlamlı uzunlukta
    # olanları al (başlıktan ibaret dosyalar kıyasa girmemeli).
    anlamli = [y for y in d if os.path.getsize(y) > 20000]
    return anlamli[-2:]


def _oku(yol):
    ts, alt, menzil = [], [], []
    with open(yol, newline="", encoding="utf-8") as f:
        for s in csv.DictReader(f):
            try:
                ts.append(float(s["t"]))
                # tgt_z NED'dir (aşağı POZİTİF). İrtifa = -tgt_z; işareti
                # burada çeviriyoruz ki tabloda "43 m" yazsın, "-43" değil.
                alt.append(-float(s["tgt_z"]))
                menzil.append(float(s["menzil"]))
            except (KeyError, ValueError):
                continue
    return ts, alt, menzil


def _ozet(yol):
    ts, alt, menzil = _oku(yol)
    if len(ts) < 20:
        return None
    sure = ts[-1] - ts[0]
    # Hedef irtifasının ORTANCASI — tek tük telemetri sıçraması etkilemesin
    sirali = sorted(alt)
    ortanca = sirali[len(sirali) // 2]
    # Tırmanma hızı (m/dk): son %20 ile ilk %20 arasındaki fark
    n = len(alt)
    d = max(1, n // 5)
    bas = sum(alt[:d]) / d
    son = sum(alt[-d:]) / d
    tirmanma = (son - bas) / sure * 60.0 if sure > 0 else 0.0
    return {
        "ad": os.path.basename(yol), "sure": sure, "n": n,
        "irtifa": ortanca, "tirmanma": tirmanma,
        "menzil_min": min(menzil) if menzil else float("nan"),
    }


def main(argv):
    yollar = argv[1:] if len(argv) > 2 else _son_ikisi()
    if len(yollar) < 2:
        print("⚠ kıyaslanacak iki koşu bulunamadı (logs/gps_guidance_*.csv)")
        return 1

    ozetler = [_ozet(y) for y in yollar]
    if any(o is None for o in ozetler):
        print("⚠ koşulardan biri boş/kısa — kıyas yok")
        return 1
    a, b = ozetler[0], ozetler[1]

    print(f"{'kol':<34}{'süre s':>9}{'hedef irtifa m':>16}{'tırmanma m/dk':>15}"
          f"{'en yakın m':>12}")
    for etiket, o in (("A  " + a["ad"], a), ("B  " + b["ad"], b)):
        print(f"{etiket:<34}{o['sure']:>9.1f}{o['irtifa']:>16.1f}"
              f"{o['tirmanma']:>15.1f}{o['menzil_min']:>12.1f}")

    sorunlar = []
    d_irt = abs(a["irtifa"] - b["irtifa"])
    if d_irt > IRTIFA_ESIK_M:
        sorunlar.append(
            f"hedef irtifası {d_irt:.1f} m farklı (eşik {IRTIFA_ESIK_M:.0f} m) — "
            f"iki kol aynı geometride uçmamış")
    for o in (a, b):
        if o["sure"] < ASGARI_SURE_S:
            sorunlar.append(f"{o['ad']} yalnız {o['sure']:.0f} s sürmüş "
                            f"(asgari {ASGARI_SURE_S:.0f} s)")
    kisa, uzun = sorted((a["sure"], b["sure"]))
    if uzun > 0 and kisa / uzun < SURE_ORAN_ESIK:
        sorunlar.append(f"süreler çok farklı ({kisa:.0f} s ↔ {uzun:.0f} s)")
    # ASIL SEBEP DENETİMİ: kollar şu an eşit irtifada olsa bile, hedef
    # tırmanmaya devam ediyorsa iki koşu FARKLI anlarda farklı geometride
    # olur ve fark her saniye büyür. 08-08'de olan tam olarak buydu.
    for o in (a, b):
        if abs(o["tirmanma"]) > TIRMANMA_ESIK_MDK:
            sorunlar.append(
                f"{o['ad']}: hedef {o['tirmanma']:+.0f} m/dk ile irtifa "
                f"değiştiriyor (eşik ±{TIRMANMA_ESIK_MDK:.0f}) — geometri "
                f"koşu boyunca kayıyor, plato beklenmemiş")

    print()
    if sorunlar:
        print("KIRMIZI — bu A/B KIYASLANAMAZ:")
        for s in sorunlar:
            print(f"  ✗ {s}")
        print("\nNe yapmalı: her kol için simi baştan kur")
        print("  bash scripts/start_harmonic.sh yeniden")
        print("ve DÜZ senaryo kullan (dairede hedef sürekli tırmanıyor).")
        return 1
    print("YEŞİL — kollar kıyaslanabilir. (Bu yalnız GEOMETRİ denetimidir;")
    print("değişikliğin etkisi ayrıca ölçülmeli.)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
