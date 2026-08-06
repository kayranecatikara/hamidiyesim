"""control/menzil_tutucu — SAF kontrol yasası testleri (-p no:anyio).

Sabit değerler config'ten türetilir; başlangıç referansı bant ortasından seçilir.
"""

import pytest

from config.kilit_sabitler import (
    MARJ_REF,
    MARJ_UST,
    MENZIL_ADIM_M,
    MENZIL_REF_ALT,
    MENZIL_REF_UST,
)
from control.menzil_tutucu import MenzilTutucu

REF0 = (MENZIL_REF_ALT + MENZIL_REF_UST) / 2.0   # bant ortası (ör. 13.0)
KUCUK_MARJ = MARJ_REF - 0.2                       # bandın altı
BUYUK_MARJ = MARJ_UST + 0.2                       # bandın üstü
BANT_MARJ = (MARJ_REF + MARJ_UST) / 2.0           # bant içi


def test_a_gps_ve_angajmanda_pasif():
    t = MenzilTutucu(REF0)
    assert t.adim_uygula("GPS", KUCUK_MARJ) == (False, None)
    assert t.adim_uygula("ANGAJMAN", KUCUK_MARJ) == (False, None)
    assert t.adim_uygula("BEKLE", BUYUK_MARJ) == (False, None)
    assert t.ref == REF0                           # referans hiç değişmedi


def test_b_visualde_yon():
    # marj < REF -> küçülür (yaklaş)
    t = MenzilTutucu(REF0)
    aktif, yeni = t.adim_uygula("VISUAL", KUCUK_MARJ)
    assert aktif is True
    assert yeni == pytest.approx(REF0 - MENZIL_ADIM_M)

    # marj > UST -> büyür (uzaklaş)
    t2 = MenzilTutucu(REF0)
    aktif, yeni = t2.adim_uygula("VISUAL", BUYUK_MARJ)
    assert aktif is True
    assert yeni == pytest.approx(REF0 + MENZIL_ADIM_M)


def test_c_bant_ici_dokunmaz():
    t = MenzilTutucu(REF0)
    assert t.adim_uygula("VISUAL", BANT_MARJ) == (True, None)
    assert t.adim_uygula("VISUAL", MARJ_REF) == (True, None)   # alt sınır dahil
    assert t.adim_uygula("VISUAL", MARJ_UST) == (True, None)   # üst sınır dahil
    assert t.ref == REF0


def test_d_adim_siniri():
    # Tek adımda değişim en fazla MENZIL_ADIM_M kadar.
    t = MenzilTutucu(REF0)
    _, yeni = t.adim_uygula("VISUAL", KUCUK_MARJ)
    assert abs(yeni - REF0) == pytest.approx(MENZIL_ADIM_M)


def test_e_clamp_alt_ust():
    # Alt sınıra yakın başla, küçülmeye çalış -> ALT'ı aşmaz.
    t = MenzilTutucu(MENZIL_REF_ALT + MENZIL_ADIM_M / 2.0)
    _, yeni = t.adim_uygula("VISUAL", KUCUK_MARJ)
    assert yeni == pytest.approx(MENZIL_REF_ALT)
    assert t.ref >= MENZIL_REF_ALT

    # Üst sınıra yakın başla, büyümeye çalış -> UST'u aşmaz.
    t2 = MenzilTutucu(MENZIL_REF_UST - MENZIL_ADIM_M / 2.0)
    _, yeni = t2.adim_uygula("VISUAL", BUYUK_MARJ)
    assert yeni == pytest.approx(MENZIL_REF_UST)
    assert t2.ref <= MENZIL_REF_UST


def test_f_marj_none_dokunmaz():
    t = MenzilTutucu(REF0)
    assert t.adim_uygula("VISUAL", None) == (True, None)
    assert t.ref == REF0


def test_g_surekli_kucuk_marj_alta_yakinsar_ve_durur():
    """Yön + adım sınırı + clamp birlikte: marj hep REF altında -> ref ALT'a
    ardışık adımlarla yakınsar ve orada durur (daha fazla küçülmez)."""
    t = MenzilTutucu(REF0)
    # Yakınsama için yeterli tık: (REF0-ALT)/ADIM + emniyet
    n = int((REF0 - MENZIL_REF_ALT) / MENZIL_ADIM_M) + 5
    for _ in range(n):
        t.adim_uygula("VISUAL", KUCUK_MARJ)
    assert t.ref == pytest.approx(MENZIL_REF_ALT)     # ALT'a oturdu
    # Bir tık daha: artık değişmez (clamp'te sabit)
    aktif, yeni = t.adim_uygula("VISUAL", KUCUK_MARJ)
    assert aktif is True
    assert yeni is None                               # referans DEĞİŞMEDİ
    assert t.ref == pytest.approx(MENZIL_REF_ALT)
