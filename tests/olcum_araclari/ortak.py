"""Ortak test koşum yardımcıları (repo geneliyle aynı PASS/FAIL biçimi)."""

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def sifirla():
    _sonuclar.clear()


def ozet(baslik):
    """Dönüş: (gecen, toplam)."""
    gecen = sum(1 for _, ok, _ in _sonuclar if ok)
    toplam = len(_sonuclar)
    print("=" * 60)
    durum = "HEPSİ GEÇTİ ✓" if gecen == toplam else "BAŞARISIZ ✗"
    print(f"{baslik}: {gecen}/{toplam} geçti — {durum}")
    return gecen, toplam
