"""Adım 8B bildirim iskeleti testleri (-p no:anyio).

Beyan aralığı: gerçek kilitli örnekle başlar/biter; beyanın TAMAMI şartı sağlar.
"""

import csv

import pytest

from config.kilit_sabitler import KUMULATIF_SN
from control.kilit_sure import KilitSure
from control.kilitlenme import KilitTakip
from comms.kilit_bildirim import KuruKosuBildirici

DT = 0.05


def kilitli_bes(ks, t0, t1, dt=DT):
    """[t0, t1] arası KESİNTİSİZ kilit — yoğun kareler."""
    araliksure = t1 - t0
    n = max(1, int(round(araliksure / dt))) if araliksure > 0 else 1
    d = ks.guncelle(t0, True)
    for i in range(1, n + 1):
        d = ks.guncelle(t0 + araliksure * i / n, True)
    return d


def _uc_iki_senaryo():
    """[1,4] (3 sn) + [7,9] (2 sn); pencere_ok t=9'da dolar."""
    ks = KilitSure()
    kilitli_bes(ks, 1.0, 4.0)
    ks.guncelle(5.0, False)     # boşluk -> segment biter
    d = kilitli_bes(ks, 7.0, 9.0)
    return ks, d


def test_beyan_araligi_dogru_uc_iki():
    ks, d = _uc_iki_senaryo()
    assert d.pencere_ok is True                     # tetik anı
    baslangic, bitis, kumulatif = ks.beyan_araligi(9.0)
    assert baslangic == pytest.approx(1.0)          # ilk segmentin gerçek başı
    assert bitis == pytest.approx(9.0)              # pencere_ok anındaki gerçek kilitli örnek
    assert kumulatif == pytest.approx(5.0)
    assert kumulatif >= KUMULATIF_SN                # beyanın tamamı şartı sağlar


def test_bitis_tespitsiz_kareyi_kapsamaz():
    """ş: pencere_ok dolmuşken 1 kare kayıp -> bitis son KİLİTLİ örnek (< t)."""
    ks, _ = _uc_iki_senaryo()
    ks.guncelle(9.05, False)                        # tespitsiz kare
    baslangic, bitis, kumulatif = ks.beyan_araligi(9.05)
    assert bitis == pytest.approx(9.0)              # 9.05 DEĞİL — kilitsiz an kapsanmaz
    assert bitis < 9.05
    assert kumulatif >= KUMULATIF_SN                # şart yine sağlanır


def test_kilittakip_passthrough_ayni_sonuc():
    kt = KilitTakip(640, 480)
    # merkezde, kilit sağlayan bbox; yoğun besle -> pencere_ok
    cx, cy, w, h = 320, 240, 200, 150
    bbox = (cx - w // 2, cy - h // 2, cx + w // 2, cy + h // 2)
    t = 0.0
    d = None
    while t <= 6.0:
        d = kt.guncelle(bbox, t)
        t += DT
    assert d["kilit_isteri_ok"] is True
    ar = kt.beyan_araligi(t - DT)
    assert ar is not None
    baslangic, bitis, kumulatif = ar
    assert baslangic == pytest.approx(0.0, abs=DT)  # ilk kilitli örnek ~0
    assert kumulatif >= KUMULATIF_SN


def test_kuru_kosu_bildirici_csv_yazar(tmp_path):
    yol = tmp_path / "bildirim_test.csv"
    b = KuruKosuBildirici(csv_yol=str(yol))
    b.bildir(1.0, 9.0, 5.0)
    b.kapat()
    with open(yol) as f:
        satirlar = list(csv.DictReader(f))
    assert len(satirlar) == 1
    r = satirlar[0]
    assert float(r["baslangic_t"]) == pytest.approx(1.0)
    assert float(r["bitis_t"]) == pytest.approx(9.0)
    assert float(r["kumulatif_sn"]) == pytest.approx(5.0)
    assert r["sart_saglandi"] == "1"                # 5.0 >= KUMULATIF_SN


def test_kuru_kosu_bildirici_sart_saglanmaz(tmp_path):
    yol = tmp_path / "bildirim_test2.csv"
    b = KuruKosuBildirici(csv_yol=str(yol))
    b.bildir(1.0, 3.0, KUMULATIF_SN - 1.0)          # eksik kümülatif
    b.kapat()
    with open(yol) as f:
        r = list(csv.DictReader(f))[0]
    assert r["sart_saglandi"] == "0"
