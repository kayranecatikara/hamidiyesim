"""Gerçek hedef-göreli salınım geometrisinin regresyon testleri.

Kullanım:
    PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_salinim
"""

import math

from tools import salinim


def yakin(a, b, esik=1e-9):
    return abs(a - b) <= esik


def main():
    sonuclar = []

    def kontrol(ad, kosul, detay=""):
        sonuclar.append((ad, bool(kosul)))
        print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")

    print("Hedef-göreli salınım geometrisi")

    # Kuzeye giden hedefin sağı doğudur: +y.
    yanal, boyuna, hiz = salinim.hedef_cercevesi(
        vx=10.0, vy=0.0, dx=0.0, dy=5.0)
    kontrol("S1 kuzeye giden hedefin doğusu SAĞ (+)",
            yakin(yanal, 5.0) and yakin(boyuna, 0.0) and yakin(hiz, 10.0),
            f"yanal={yanal:+.1f}, boyuna={boyuna:+.1f}")

    # Doğuya giden hedefin sağı güneydir: -x.
    yanal, boyuna, _ = salinim.hedef_cercevesi(
        vx=0.0, vy=12.0, dx=-7.0, dy=0.0)
    kontrol("S2 doğuya giden hedefin güneyi SAĞ (+)",
            yakin(yanal, 7.0) and yakin(boyuna, 0.0),
            f"yanal={yanal:+.1f}, boyuna={boyuna:+.1f}")

    # Aynı hedefin kuzeyindeki drone soldadır; işaret mutlaka negatif.
    yanal, boyuna, _ = salinim.hedef_cercevesi(
        vx=0.0, vy=12.0, dx=7.0, dy=0.0)
    kontrol("S3 doğuya giden hedefin kuzeyi SOL (-)",
            yakin(yanal, -7.0) and yakin(boyuna, 0.0),
            f"yanal={yanal:+.1f}, boyuna={boyuna:+.1f}")

    # Dönüş ortonormal olmalı: uzunluk hedef ekseninde korunur.
    yanal, boyuna, _ = salinim.hedef_cercevesi(
        vx=3.0, vy=4.0, dx=-2.5, dy=6.0)
    kontrol("S4 eksen dönüşü mesafeyi korur",
            yakin(math.hypot(yanal, boyuna), math.hypot(-2.5, 6.0)),
            f"dönüş={math.hypot(yanal, boyuna):.6f}, "
            f"ham={math.hypot(-2.5, 6.0):.6f}")

    kontrol("S5 düşük hedef hızında yön reddedilir",
            salinim.hedef_cercevesi(0.1, 0.1, 1.0, 1.0) is None)

    kalan = [ad for ad, ok in sonuclar if not ok]
    print(f"SONUÇ: {len(sonuclar) - len(kalan)}/{len(sonuclar)} geçti"
          + (f" — KALAN: {kalan}" if kalan else " — HEPSİ GEÇTİ ✓"))
    return 1 if kalan else 0


if __name__ == "__main__":
    raise SystemExit(main())
