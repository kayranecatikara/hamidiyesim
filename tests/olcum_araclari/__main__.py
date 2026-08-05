"""Ölçüm aracı testlerinin tamamı:  python3 -m tests.olcum_araclari"""

import sys

from tests.olcum_araclari import (test_gecis_analiz, test_gudum_karne,
                                  test_parm_denetle)

MODULLER = [test_gecis_analiz, test_gudum_karne, test_parm_denetle]


def main():
    gecen = toplam = 0
    for m in MODULLER:
        g, t = m.main()
        gecen += g
        toplam += t
        print()
    print("=" * 60)
    durum = "HEPSİ GEÇTİ ✓" if gecen == toplam else "BAŞARISIZ ✗"
    print(f"ÖLÇÜM ARAÇLARI TOPLAM: {gecen}/{toplam} geçti — {durum}")
    return gecen == toplam


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
