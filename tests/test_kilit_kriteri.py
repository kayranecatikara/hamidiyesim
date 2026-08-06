"""vision/kilit_kriteri testleri — çözünürlük-parametrik, sabit piksel YOK.

Tüm sınırlar config oranlarından (ESIK_BILDIRIM, av_sinirlari) türetilir.
"""

import pytest

from config.kilit_sabitler import ESIK_BILDIRIM, av_sinirlari
from vision.kilit_kriteri import KilitKriter, kriter_degerlendir


@pytest.fixture(params=[(640, 480), (1280, 720)], ids=["640x480", "1280x720"])
def cozunurluk(request):
    return request.param


def _merkez(W, H):
    """AV'nin tam ortası (her zaman AV içinde)."""
    x_min, x_max, y_min, y_max = av_sinirlari(W, H)
    return ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0)


def _thr(W, H):
    return (ESIK_BILDIRIM * W, ESIK_BILDIRIM * H)


def test_1_boyut_veya_kurali(cozunurluk):
    W, H = cozunurluk
    cx, cy = _merkez(W, H)
    thr_w, thr_h = _thr(W, H)

    # İki eksen de eşik altı -> kilit YOK
    r = kriter_degerlendir(cx, cy, 0.5 * thr_w, 0.5 * thr_h, W, H)
    assert r.boyut_ok is False
    assert r.kilit is False

    # Yalnız w >= eşik (h eşik altı) -> VEYA sağlanır, kilit VAR
    r = kriter_degerlendir(cx, cy, thr_w, 0.5 * thr_h, W, H)
    assert r.boyut_ok is True
    assert r.merkez_av_icinde is True
    assert r.kilit is True


def test_2_merkez_kurali(cozunurluk):
    W, H = cozunurluk
    cx, cy = _merkez(W, H)
    thr_w, thr_h = _thr(W, H)

    # Merkez AV içinde, bbox kenarları AV'yi taşıyor (çok büyük bbox) -> VAR.
    # KILIT_MODU="merkez": yalnız merkez konumu önemli, kenar taşması değil.
    r = kriter_degerlendir(cx, cy, 0.9 * W, 0.9 * H, W, H)
    x_min, x_max, y_min, y_max = av_sinirlari(W, H)
    assert cx - 0.45 * W < x_min and cx + 0.45 * W > x_max  # kenarlar gerçekten taşıyor
    assert r.merkez_av_icinde is True
    assert r.kilit is True

    # Merkez AV dışına çıkınca -> YOK (boyut yeterli olsa bile).
    disari_cx = 0.80 * W  # AV üst sınırı 0.75W'nin dışı
    r = kriter_degerlendir(disari_cx, cy, 2 * thr_w, 2 * thr_h, W, H)
    assert r.merkez_av_icinde is False
    assert r.kilit is False


def test_3_marj_baskin_eksen(cozunurluk):
    W, H = cozunurluk
    cx, cy = _merkez(W, H)
    thr_w, thr_h = _thr(W, H)

    # w baskın: w/thr_w = 2.0 > h/thr_h = 1.0 -> marj = 2.0
    r = kriter_degerlendir(cx, cy, 2.0 * thr_w, 1.0 * thr_h, W, H)
    assert r.marj == pytest.approx(2.0)

    # h baskın: h/thr_h = 3.0 > w/thr_w = 1.0 -> marj = 3.0
    r = kriter_degerlendir(cx, cy, 1.0 * thr_w, 3.0 * thr_h, W, H)
    assert r.marj == pytest.approx(3.0)
