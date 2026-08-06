"""control/kilit_sure testleri — süre tabanlı, sabit ms YAZMA.

Boşluk/bütçe süreleri KARE_TOLERANS_ORAN ve o anki kilitli süreden türetilir.
Kilitli aralıklar YOĞUN kare akışıyla (dt <= ORNEK_TAVAN_SN) beslenir; seyrek
iki-örnek beslemesi 6.1.4 tavan kuralıyla artık boşluk sayılır.
"""

import pytest

from config.kilit_sabitler import (
    KARE_TOLERANS_ORAN,
    KESINTISIZ_SN,
    KUMULATIF_SN,
    ORNEK_TAVAN_SN,
    PENCERE_SN,
)
from control.kilit_sure import KilitSure

DT = 0.05  # nominal kare adımı (< ORNEK_TAVAN_SN)


def kilitli_bes(ks, t0, t1, dt=DT):
    """[t0, t1] arası KESİNTİSİZ kilit: yoğun kareler (adım <= dt <= tavan)."""
    araliksure = t1 - t0
    n = max(1, int(round(araliksure / dt))) if araliksure > 0 else 1
    d = ks.guncelle(t0, True)
    for i in range(1, n + 1):
        d = ks.guncelle(t0 + araliksure * i / n, True)
    return d


def test_4_kesik_kesik_kumulatif_pencere_ok():
    """[1,4] (3 sn) + [7,9] (2 sn) = 5 sn kümülatif -> pencere_ok."""
    ks = KilitSure()
    kilitli_bes(ks, 1.0, 4.0)   # seg1 = 3 sn
    ks.guncelle(5.0, False)     # boşluk -> segment biter
    d = kilitli_bes(ks, 7.0, 9.0)  # seg2 = 2 sn
    assert d.kumulatif_sn == pytest.approx(5.0)
    assert d.kumulatif_sn >= KUMULATIF_SN
    assert d.pencere_ok is True


def test_5_pencere_sonrasi_kesintisiz_ok():
    """pencere_ok + ardından 3.0 sn kesintisiz segment -> kesintisiz_ok."""
    ks = KilitSure()
    kilitli_bes(ks, 1.0, 4.0)   # 3 sn
    ks.guncelle(5.0, False)
    d = kilitli_bes(ks, 7.0, 10.0)  # kesintisiz 3 sn
    assert d.pencere_ok is True                      # kümülatif 3+3 = 6
    assert d.kesintisiz_sn == pytest.approx(KESINTISIZ_SN)
    assert d.kesintisiz_ok is True


def test_6_kayip_kesintisiz_sifir_kumulatif_korunur_sonra_ok():
    ks = KilitSure()
    d = kilitli_bes(ks, 0.0, 2.9)   # kesintisiz 2.9 sn (henüz ok değil)
    assert d.kesintisiz_sn == pytest.approx(2.9)
    assert d.kesintisiz_ok is False

    # Bütçeyi aşan kayıp: budget = KARE_TOLERANS_ORAN*2.9 ~ 0.145 sn; 0.5 > bu.
    d = ks.guncelle(3.4, False)
    assert d.kesintisiz_sn == pytest.approx(0.0)      # kesintisiz sıfırlandı
    assert d.kumulatif_sn == pytest.approx(2.9)       # kümülatif korunur

    # Tekrar deneme: 3.0 sn kesintisiz -> kesintisiz_ok
    d = kilitli_bes(ks, 3.9, 6.9)
    assert d.kesintisiz_sn == pytest.approx(3.0)
    assert d.kesintisiz_ok is True


def test_7_pencere_disina_cikinca_kumulatif_azalir():
    ks = KilitSure()
    d4 = kilitli_bes(ks, 0.0, 4.0)       # seg [0,4], kümülatif 4
    assert d4.kumulatif_sn == pytest.approx(4.0)

    # t=13'te pencere [3,13]; [0,4]'ün yalnız [3,4]=1 sn'si kalır.
    d13 = ks.guncelle(0.0 + PENCERE_SN + 3.0, False)  # t = 13
    assert d13.kumulatif_sn == pytest.approx(1.0)
    assert d13.kumulatif_sn < d4.kumulatif_sn


def test_8_butce_ici_koprulenir_asan_bolunur():
    kilitli = 2.0
    butce = KARE_TOLERANS_ORAN * kilitli
    ic_bosluk = 0.5 * butce
    asan_bosluk = 2.0 * butce

    # Bütçe içinde: köprülenir -> segment bölünmez, kesintisiz boşluk boyunca sürer.
    ks = KilitSure()
    kilitli_bes(ks, 0.0, kilitli)                      # [0,2], kilitli 2 sn
    ks.guncelle(kilitli + 0.4 * ic_bosluk, False)
    ks.guncelle(kilitli + ic_bosluk, True)             # boşluk köprülendi
    d = kilitli_bes(ks, kilitli + ic_bosluk, kilitli + ic_bosluk + 0.95)  # kilit sürüyor
    assert d.kesintisiz_sn == pytest.approx(kilitli + ic_bosluk + 0.95)  # ~3.0, bölünmedi

    # Bütçeyi aşan: bölünür -> kesintisiz yalnız bölünme sonrası süreyi sayar.
    ks2 = KilitSure()
    kilitli_bes(ks2, 0.0, kilitli)
    ks2.guncelle(kilitli + 0.4 * asan_bosluk, False)
    ks2.guncelle(kilitli + asan_bosluk, True)          # yeni segment başlar
    d2 = kilitli_bes(ks2, kilitli + asan_bosluk, kilitli + asan_bosluk + 1.0)
    assert d2.kesintisiz_sn == pytest.approx(1.0)      # yalnız bölünme sonrası


def test_9_bosluk_basta_veya_sonda_koprulenmez():
    # Segment BAŞINDA boşluk (kilitli süre ~0, bütçe 0) -> köprülenmez.
    ks = KilitSure()
    ks.guncelle(0.0, True)     # tek anlık kilit, kilitli süre 0
    ks.guncelle(0.02, False)   # baştaki boşluk
    d = kilitli_bes(ks, 0.05, 1.05)   # yeni segment (köprü yok)
    assert d.kesintisiz_sn == pytest.approx(1.0)  # 1.05 değil -> baş boşluk sayılmadı

    # Segment SONUNDA boşluk (kapatan kilit yok) -> köprülenmez.
    ks2 = KilitSure()
    kilitli_bes(ks2, 0.0, 2.0)              # kesintisiz 2.0
    d2 = ks2.guncelle(2.05, False)          # sondaki boşluk
    assert d2.kesintisiz_sn == pytest.approx(2.0)   # 2.05 değil
    assert d2.kumulatif_sn == pytest.approx(2.0)    # boşluk kümülatife girmedi


def test_10_koprulenen_bosluk_butceyi_sismez():
    """Pozitif geri besleme yok: köprülenen boşluk bütçe tabanına katılmaz.

    kilitli=2.0 -> bütçe=0.1. g1=0.08 köprülenir; g2=0.022 ile toplam köprü
    0.102 > 0.1 olur. Bütçe tabanı yalnız gerçek kilitli süre (2.0) ise segment
    BÖLÜNMELİDİR. Bütçe köprülenen boşlukla şişseydi (2.08 -> 0.104) g2 de
    köprülenir ve kesintisiz ~2.1 kalırdı -> yanlış.
    """
    kilitli = 2.0
    ks = KilitSure()
    kilitli_bes(ks, 0.0, kilitli)       # gerçek kilitli 2.0
    # g1 = 0.08 (bütçe 0.1 içinde) -> köprülenir
    ks.guncelle(kilitli + 0.04, False)
    ks.guncelle(kilitli + 0.08, True)
    # g2 = 0.022 -> köprülenen toplam 0.102 > 0.1 -> bölünmeli (gerçek tabanla)
    ks.guncelle(kilitli + 0.10, False)
    d = ks.guncelle(kilitli + 0.102, True)
    assert d.kesintisiz_sn < 1.0        # bölündü (şişme olsaydı ~2.1 olurdu)


def test_11_koprulenen_bosluk_kumulatif_ve_kesintisize_dahil():
    """6.1.4: köprülenen boşluk 'beyan edilen sürenin tamamı' olarak sayılır."""
    kilitli = 2.0
    bosluk = 0.5 * (KARE_TOLERANS_ORAN * kilitli)  # 0.05, bütçe içinde
    ks = KilitSure()
    kilitli_bes(ks, 0.0, kilitli)                  # [0,2]
    ks.guncelle(kilitli + 0.4 * bosluk, False)
    d = ks.guncelle(kilitli + bosluk, True)        # boşluk köprülendi
    assert d.kesintisiz_sn == pytest.approx(kilitli + bosluk)   # köprü dahil
    assert d.kumulatif_sn == pytest.approx(kilitli + bosluk)    # köprü dahil
    assert d.kesintisiz_sn > kilitli


def test_12_tavan_asan_dt_kilitli_sayilmaz_ve_bolunur():
    """Kilit→kilit arası dt > ORNEK_TAVAN_SN -> aralık kilitli sayılmaz.

    Bütçe yetmiyorsa segment bölünür; kümülatif o (donma/atlama) aralığını
    İÇERMEZ.
    """
    ks = KilitSure()
    ks.guncelle(0.0, True)
    ks.guncelle(0.1, True)             # dt=0.1 <= tavan -> kilitli, kilitli süre 0.1
    # Büyük sıçrama: dt = ORNEK_TAVAN_SN + 0.3 > tavan; bütçe=0.05*0.1=0.005 yetmez.
    buyuk_dt = ORNEK_TAVAN_SN + 0.3
    d = ks.guncelle(0.1 + buyuk_dt, True)
    assert d.kumulatif_sn == pytest.approx(0.1)   # yalnız gerçek kilitli 0.1
    assert d.kumulatif_sn < 0.1 + buyuk_dt        # sıçrama aralığı sayılmadı
    assert d.kesintisiz_sn == pytest.approx(0.0)  # segment bölündü (yeni segment)


def test_13_tavan_alti_dt_regresyon_degismedi():
    """dt tavan altında (normal kare akışı) -> mevcut davranış değişmedi."""
    ks = KilitSure()
    d = kilitli_bes(ks, 0.0, 3.0, dt=0.035)   # ~35 ms nominal kareler
    assert d.kumulatif_sn == pytest.approx(3.0)
    assert d.kesintisiz_sn == pytest.approx(3.0)
    assert 0.035 < ORNEK_TAVAN_SN             # tavan yanlış tetiklenmez
