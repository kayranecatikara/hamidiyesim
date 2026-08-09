#!/usr/bin/env python3
"""KUTU KAYMASI — çarpışma rotasında mıyız? (terminal hücumun asıl ölçütü)

Denizcilik kuralı: çarpışma rotasındaysan karşı gemi ufuktaki yerini
DEĞİŞTİRMEZ. Aynısı burada: drone gerçekten çarpacak bir rotadaysa hedef
kamera kadrajında SABİT bir noktada durur (büyür ama kaymaz).

Bu araç, terminal hücumun son N karesinde kutunun dikey piksel konumunun
(cy) ne kadar kaydığını ölçer:
     0'a yakın → çarpışma rotası ✓
     + (artı)  → kutu aşağı kayıyor = drone yükseliyor  → ÜSTÜNDEN geçiyoruz
     − (eksi)  → kutu yukarı kayıyor = drone alçalıyor  → ALTINDAN geçiyoruz

⚠ NEDEN VURUŞ SAYISINDAN İYİ BİR ÖLÇÜT: vuruş/ıska ikili ve gürültülü —
3-5 koşuda %60-70'lik bir oranı ayırt edemez (2026-08-09'da 5/5 çıkan
yapılandırma tekrarda 2/5 verdi). Kutu kayması ise SÜREKLİ bir büyüklük ve
doğrudan mekanizmayı ölçer; birkaç koşuda bile eğilim görünür.

Kullanım:
    python3 tools/kutu_kaymasi.py logs/bbox_ibvs_*.csv
    python3 tools/kutu_kaymasi.py --dizin logs        (son 10 log)
"""
import csv
import glob
import os
import statistics as st
import sys

SON_N = 8          # son kaç kare üzerinden kayma ölçülür (~0.4 s)


def kayma(yol, son_n=SON_N):
    """Bir güdüm logundan terminal kutu kaymasını döndürür (px) veya None."""
    try:
        rows = [r for r in csv.DictReader(open(yol))
                if r.get("durum") == "TERMINAL" and r.get("cy")]
    except OSError:
        return None
    if len(rows) < son_n:
        return None
    cy = [float(r["cy"]) for r in rows]
    return cy[-1] - cy[-son_n]


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--dizin":
        yollar = sorted(glob.glob(os.path.join(sys.argv[2], "bbox_ibvs_*.csv")),
                        key=os.path.getmtime)[-10:]
    else:
        yollar = sys.argv[1:]
    if not yollar:
        print(__doc__)
        raise SystemExit(1)

    degerler = []
    print(f"KUTU KAYMASI (son {SON_N} kare, px):")
    for y in yollar:
        k = kayma(y)
        if k is None:
            continue
        degerler.append(k)
        yorum = ("çarpışma rotası ✓" if abs(k) < 25 else
                 ("ÜSTTEN geçiyor ⚠" if k > 0 else "ALTTAN geçiyor ⚠"))
        print(f"  {os.path.basename(y):40} {k:+7.0f} px   {yorum}")
    if degerler:
        mutlak = [abs(d) for d in degerler]
        print(f"\n  n={len(degerler)}  |kayma| medyan {st.median(mutlak):.0f} px  "
              f"en kötü {max(mutlak):.0f} px  "
              f"|kayma|<25 olan: {sum(1 for m in mutlak if m < 25)}/{len(mutlak)}")


if __name__ == "__main__":
    main()
