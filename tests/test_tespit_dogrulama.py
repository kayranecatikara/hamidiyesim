"""vision/tespit_dogrulama testleri (şartname 6.1.1) — süre tabanlı, -p no:anyio.

Tespit örnekleri yoğun kare akışıyla (dt <= ORNEK_TAVAN_SN) beslenir.
"""

import pytest

from config.kilit_sabitler import (
    DOGRULAMA_SN,
    ORNEK_TAVAN_SN,
    TESPIT_TUTARLILIK_ORAN,
)
from vision.tespit_dogrulama import TespitDogrulama

DT = 0.05  # nominal kare adımı (< ORNEK_TAVAN_SN)


def bes(td, a, b, val, dt=DT):
    """(a, b] arası yoğun besleme; son kararı döndürür (td'de a örneği olmalı)."""
    n = max(1, int(round((b - a) / dt)))
    r = None
    for i in range(1, n + 1):
        r = td.guncelle(a + (b - a) * i / n, val)
    return r


def test_1_tek_kare_dogrulanmaz():
    td = TespitDogrulama()
    assert td.guncelle(0.0, True) is False           # tek örnek
    assert td.guncelle(0.03, True) is False           # pencere dolmadı


def test_2_pencere_boyu_tutarli_tespit_dogrulanir():
    td = TespitDogrulama()
    td.guncelle(0.0, True)
    r = bes(td, 0.0, DOGRULAMA_SN + 0.1, True)         # pencere boyu kesintisiz
    assert r is True


def test_3_kesik_tespit_kural_sinirinda():
    # Yeterli oran (kısa boşluk): pencere içinde tespitli oran > eşik -> doğrulanır.
    td = TespitDogrulama()
    td.guncelle(0.0, True)
    bes(td, 0.0, 0.65, True)
    bes(td, 0.65, 0.75, False)                         # ~0.10 boşluk
    r = bes(td, 0.75, 1.0, True)
    assert r is True                                   # oran ~0.75 > 0.6

    # Yetersiz oran (uzun boşluk): tespitli oran < eşik -> doğrulanmaz.
    td2 = TespitDogrulama()
    td2.guncelle(0.0, True)
    bes(td2, 0.0, 0.6, True)
    bes(td2, 0.6, 0.9, False)                          # ~0.30 boşluk
    r2 = bes(td2, 0.9, 1.0, True)
    assert r2 is False                                 # oran ~0.35 < 0.6


def test_4_tespit_kaybolunca_makul_surede_duser():
    td = TespitDogrulama()
    td.guncelle(0.0, True)
    r0 = bes(td, 0.0, 0.6, True)
    assert r0 is True                                  # doğrulandı

    # Kayıp: kapı ~ (1 - ORAN)*DOGRULAMA_SN = 0.2 sn içinde düşmeli.
    r1 = bes(td, 0.6, 0.7, False)                      # kayıptan 0.1 sn sonra
    assert r1 is True                                  # henüz düşmedi
    r2 = bes(td, 0.7, 0.9, False)                      # kayıptan 0.3 sn sonra
    assert r2 is False                                 # düştü


def test_5_donma_araligi_tespitli_sayilmaz_kapi_duser():
    """ş: dt > ORNEK_TAVAN_SN olan aralık tespitli sayılmaz -> kapı düşer."""
    td = TespitDogrulama()
    td.guncelle(0.0, True)
    r0 = bes(td, 0.0, 0.6, True)
    assert r0 is True                                  # yoğun akışla doğrulandı

    # Tespit hattı DONAR: tespit_var hâlâ True ama dt tavanı aşıyor.
    donma_dt = ORNEK_TAVAN_SN + 0.1                    # > tavan
    r1 = td.guncelle(0.6 + donma_dt, True)             # aralık tespitli SAYILMAZ
    assert r1 is False                                 # donma penceresi kapıyı açık tutamaz
