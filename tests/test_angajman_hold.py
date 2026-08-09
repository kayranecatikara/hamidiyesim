"""KilitTakip.guncelle(angajman=True) — taahhüt edilmiş STRIKE dalışında kesintisiz
kilit sayacı SIFIRLANMAZ (şartname 6.1.3: angajmanda merkez/%5 aranmaz, kriter
aktif takip). Normal fazda (angajman=False) hedef merkez AV'den çıkınca kilit
kopar ve kesintisiz sıfırlanır.
"""

from control.kilitlenme import KilitTakip

W, H = 640, 480
DT = 0.05


def _merkez_bbox(cx=320, cy=240, w=80, h=80):
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def _off_merkez_bbox():
    """Merkez AV DIŞINDA (cx=560 > 0.75W=480), hedef kareyi dolduran büyük kutu —
    yakın mesafe dalışının geometrisi."""
    return _merkez_bbox(cx=560, cy=240, w=300, h=300)


def _kilit_bina_et(kt, sure=3.5, t0=0.0):
    """Merkezî sürekli kilit besleyerek kesintisiz sayacı büyüt."""
    t = t0
    n = int(round(sure / DT))
    for i in range(1, n + 1):
        t = round(t0 + i * DT, 6)
        d = kt.guncelle(_merkez_bbox(), t)
    assert d["anlik_kilit"] is True
    assert d["kesintisiz_s"] >= sure - 0.2
    return t, d["kesintisiz_s"]


def test_angajmanda_kesintisiz_sifirlanmaz():
    kt = KilitTakip(W, H)
    t, kes0 = _kilit_bina_et(kt)
    # Dalış: hedef merkez AV'den çıkar (kare dolar). angajman=True → kilit korunur.
    for i in range(1, 11):                     # ~0.5 sn (toleransı fazlasıyla aşar)
        t = round(t + DT, 6)
        d = kt.guncelle(_off_merkez_bbox(), t, angajman=True)
        assert d["merkez_av_icinde"] is False  # merkez gerçekten AV dışında
        assert d["anlik_kilit"] is True        # ama angajmanda kilit KORUNUR
    assert d["kesintisiz_s"] >= kes0           # kesintisiz SIFIRLANMADI, arttı


def test_normal_fazda_merkez_disi_kilidi_kirar():
    kt = KilitTakip(W, H)
    t, kes0 = _kilit_bina_et(kt)
    # Aynı geometri ama angajman=False → merkez AV dışı kilidi kırar.
    for i in range(1, 11):                     # ~0.5 sn > köprü bütçesi → segment ölür
        t = round(t + DT, 6)
        d = kt.guncelle(_off_merkez_bbox(), t, angajman=False)
        assert d["anlik_kilit"] is False
    assert d["kesintisiz_s"] == 0.0            # kesintisiz sıfırlandı
