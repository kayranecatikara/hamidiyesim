"""Oran regülasyonlu APPROACH kontrolü testleri (-p no:anyio)."""

from config.kilit_sabitler import AYAR
from control.yaklasma_kontrol import (
    YaklasmaKontrol, SEBEP_BANT, SEBEP_YAKLAS, SEBEP_GERI, SEBEP_TAVAN,
    SEBEP_RMIN, SEBEP_PASIF)

DT = 0.1


def konver(yk, oran, menzil, rs=6.5, n=80, bbox=True, aktif=True):
    """n adım sabit oran besle (EMA otursun); son Karar'ı döndür. rs zincirlenir."""
    son = None
    for _ in range(n):
        son = yk.adim(oran, bbox, menzil, rs, DT, aktif=aktif)
        rs = son.range_set
    return son


def test_dusuk_oran_yaklasir():
    yk = YaklasmaKontrol()
    k = konver(yk, 0.03, menzil=12.0)
    assert k.komut == "yaklas" and k.sebep == SEBEP_YAKLAS
    assert k.oran_ema < AYAR.ORAN_BANT_ALT
    assert k.oran_hatasi > 0                          # setpoint - ema > 0


def test_yuksek_oran_geri_cekilir():
    yk = YaklasmaKontrol()
    k = konver(yk, 0.092, menzil=6.0)                 # bant üstü, tavan altı
    assert k.komut == "geri" and k.sebep == SEBEP_GERI
    assert k.geri_hiz == AYAR.V_MAX_RETREAT
    assert k.oran_hatasi < 0


def test_tavan_birincil_emniyet():
    yk = YaklasmaKontrol()
    k = konver(yk, 0.12, menzil=5.5)                  # tavanı aşar
    assert k.komut == "geri" and k.sebep == SEBEP_TAVAN
    assert k.geri_hiz == AYAR.V_MAX_RETREAT


def test_bant_ici_komut_yok():
    yk = YaklasmaKontrol()
    k = konver(yk, AYAR.ORAN_SETPOINT, menzil=6.0)    # tam setpoint
    assert k.komut == "yok" and k.sebep == SEBEP_BANT
    assert abs(k.oran_hatasi) < 0.005


def test_latch_sinira_degince_birakmaz_setpointe_kadar():
    yk = YaklasmaKontrol()
    # 1) düşük oran → düzeltme başlar (yaklas)
    k1 = konver(yk, 0.05, menzil=12.0)
    assert k1.komut == "yaklas"
    # 2) oran bant İÇİNE girdi (0.070) ama setpoint (0.075) değil → latch sürer
    k2 = konver(yk, 0.070, menzil=8.0)
    assert k2.komut == "yaklas"                       # sınırda BIRAKMADI
    assert yk._duzeltiyor is True
    # 3) setpoint'e vardı → latch bırakır, tut
    k3 = konver(yk, AYAR.ORAN_SETPOINT, menzil=6.0)
    assert k3.komut == "yok" and k3.sebep == SEBEP_BANT


def test_bbox_yoksa_ema_guncellenmez():
    yk = YaklasmaKontrol()
    konver(yk, 0.04, menzil=10.0, n=40)              # ema ~0.04
    ema0 = yk._ema
    for _ in range(30):                              # bbox YOK → ema sabit
        yk.adim(0.20, False, 10.0, 6.5, DT)
    assert yk._ema == ema0


def test_bbox_varken_rmin_durdurmaz():
    # bbox VAR + düşük oran + menzil R_MIN altında bile → yaklaşmaya devam;
    # RANGE_SET R_MIN'in altına (RANGE_SET_MIN'e doğru) inebilir. Emniyet = oran tavanı.
    yk = YaklasmaKontrol()
    k = konver(yk, 0.03, menzil=4.0, rs=5.0)     # menzil < R_MIN ama bbox var
    assert k.komut == "yaklas"
    assert k.range_set < AYAR.R_MIN_GUVENLI
    assert k.range_set >= AYAR.RANGE_SET_MIN


def test_bbox_kaybinda_rmin_yedegi():
    # bbox KAYBOLUNCA yaklaşma yok; RANGE_SET R_MIN altına inmez, sebep RMIN.
    yk = YaklasmaKontrol()
    konver(yk, 0.03, menzil=8.0, n=40)           # önce ema otursun (bbox'la)
    k = None
    for _ in range(10):
        k = yk.adim(0.0, False, 4.0, 4.0, DT)    # bbox YOK, menzil < R_MIN
    assert k.komut == "yok" and k.sebep == SEBEP_RMIN
    assert k.range_set >= AYAR.R_MIN_GUVENLI


def test_pasif_dokunmaz():
    yk = YaklasmaKontrol()
    k = yk.adim(0.2, True, 4.0, 6.5, DT, aktif=False)
    assert k.komut == "yok" and k.sebep == SEBEP_PASIF and k.range_set == 6.5
