"""D-V ve D-N — dikey kanal deneylerinin kapsam testleri.

Kullanım:
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_dikey_kanal.py

NE KANITLIYOR (§5.10 yapısal garanti):
  * DV1 — `KAPANMA_MIN` YALNIZ terminalde iş görür; tutuş (seyir) fazında
    komut BİT BİT aynı kalır.
  * DV2 — taban yalnız kapanma yavaşken bağlar; hızlı kesişimde çıktı aynı.
  * DN1 — `CY_NISAN` terminal kesişim yasasını DEĞİŞTİRMEZ (o mutlak
    yükseliş kullanır), yalnız tutuş fazını kaydırır.
  * DN2 — 318 px gerçekten "seviye" demek (kamera 25° tilt).
"""

import math

from control.guidance import bbox_ibvs as B
from vision import geometry as geo


def komutla(cfg, terminal, cy=280.0, kapanma=0.3):
    """Tek bir komut() çağrısı — diğer her şey sabit."""
    return B.komut(cx=320.0, cy=cy, w=30.0, h=20.0, iris_yaw=0.0,
                   hiz_I=15.0, dt=0.05, cfg=cfg, terminal=terminal,
                   los_hiz=(0.0, 0.0), iris_pitch=0.0, iris_vz=0.0,
                   kapanma=kapanma, iris_roll=0.0, yaw_hizi=0.0)


class Kol(B.Cfg):
    """Cfg'nin kopyası — sınıf niteliklerini bozmadan deney yapmak için."""
    pass


def main():
    sonuclar = []

    def kontrol(ad, kosul, detay=""):
        sonuclar.append((ad, bool(kosul)))
        print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")

    print("Dikey kanal — D-V / D-N kapsam testleri")

    # ── DV1: TUTUŞ FAZI ETKİLENMEZ ──────────────────────────────────────
    class A(Kol):
        KAPANMA_MIN = 1.5

    class Bk(Kol):
        KAPANMA_MIN = 6.0

    ayni = True
    kirli = []
    for cy in (200.0, 260.0, 301.0, 340.0, 400.0):
        for kap in (0.0, 0.3, 1.0, 3.0, 12.0):
            a = komutla(A, False, cy, kap)
            b = komutla(Bk, False, cy, kap)
            if a[:4] != b[:4]:
                ayni = False
                kirli.append((cy, kap, a[2], b[2]))
    kontrol("DV1 tutuş fazında KAPANMA_MIN'in ETKİSİ YOK (25 kombinasyon)",
            ayni, "" if ayni else f"bozan: {kirli[:2]}")

    # ── DV2: HIZLI KAPANMADA DA ETKİSİ YOK (taban bağlamaz) ─────────────
    ayni = True
    for cy in (240.0, 280.0, 330.0):
        for kap in (7.0, 12.0, 20.0):       # hepsi 6.0'ın ÜSTÜNDE
            a = komutla(A, True, cy, kap)
            b = komutla(Bk, True, cy, kap)
            if a[:4] != b[:4]:
                ayni = False
    kontrol("DV2 kapanma > 6 m/s iken terminalde de ETKİSİ YOK", ayni)

    # ── DV3: YAVAŞ KAPANMADA DİKEY KOMUT BÜYÜR (mekanizma) ──────────────
    a = komutla(A, True, 260.0, 0.2)        # hedef yukarıda, kapanma ~0
    b = komutla(Bk, True, 260.0, 0.2)
    kontrol("DV3 yavaş kapanmada dikey komut BÜYÜR (mekanizma çalışıyor)",
            abs(b[2]) > abs(a[2]) * 1.5,
            f"vz {a[2]:+.2f} → {b[2]:+.2f} m/s")

    # ── DN1: CY_NISAN TERMİNAL KESİŞİMİNİ DEĞİŞTİRMEZ ───────────────────
    class C(Kol):
        CY_NISAN = 301.0

    class D(Kol):
        CY_NISAN = 318.0

    ayni = True
    kirli = []
    for cy in (200.0, 260.0, 320.0, 400.0):
        for kap in (0.5, 3.0, 10.0):
            a = komutla(C, True, cy, kap)
            b = komutla(D, True, cy, kap)
            if a[:4] != b[:4]:
                ayni = False
                kirli.append((cy, kap, a[2], b[2]))
    kontrol("DN1 CY_NISAN terminal kesişimini DEĞİŞTİRMEZ (12 kombinasyon)",
            ayni, "" if ayni else f"bozan: {kirli[:2]}")

    # ── DN2: TUTUŞTA İSE DEĞİŞTİRİR ─────────────────────────────────────
    a = komutla(C, False, 280.0, 1.0)
    b = komutla(D, False, 280.0, 1.0)
    kontrol("DN2 tutuş fazında CY_NISAN dikey komutu DEĞİŞTİRİR",
            abs(b[2] - a[2]) > 0.05, f"vz {a[2]:+.2f} → {b[2]:+.2f} m/s")

    # yön doğru mu: nişan aşağı kayınca DAHA ÇOK tırmanmalı (vz NED negatif)
    kontrol("DN2b nişan 318'de daha ÇOK tırmanır (vz daha negatif)",
            b[2] < a[2], f"{a[2]:+.3f} → {b[2]:+.3f}")

    # ── DN3: 318 GERÇEKTEN SEVİYE Mİ ────────────────────────────────────
    seviye = geo.CY + geo.FY * math.tan(math.radians(25.0))
    kontrol("DN3 318 px = kamera 25° tilt'in seviye karşılığı",
            abs(seviye - 318.0) < 1.0, f"hesap {seviye:.1f} px")
    mevcut = geo.CY + geo.FY * math.tan(math.radians(20.0))
    kontrol("DN3b mevcut 301 px = ufkun 5° ÜSTÜ (biz altta kalıyoruz)",
            abs(mevcut - 301.0) < 1.0, f"hesap {mevcut:.1f} px")

    # ── DN4/DV4: PANEL DÜĞMELERİ DOĞRU ALANA BAĞLI (§5.1) ───────────────
    from control import gcs_server as gcs
    for ad, alan, env, dg in (
            ("dv_dikey_taban", "KAPANMA_MIN", "AVCI_IBVS_KAPANMA_MIN", (1.5, 6.0)),
            ("dn_nisan_seviye", "CY_NISAN", "AVCI_IBVS_CY", (301.0, 318.0))):
        satir = gcs._OZELLIKLER.get(ad)
        if not satir:
            kontrol(f"{ad} panelde var", False)
            continue
        _alan, tip, _e, _a, _env, _dg = satir
        sinif, _ad = gcs._hedef_cfg(_alan)
        kontrol(f"{ad} → bbox_ibvs.Cfg.{alan}, env {env}",
                sinif is B.Cfg and _ad == alan and _env == env
                and tip == "deger" and _dg == dg)


    # ── DS1: KAPALIYKEN TUTUŞ YASASI BİT BİT AYNI ───────────────────────
    class E(Kol):
        TUTUS_SONUM = 0.0

    class F(Kol):
        TUTUS_SONUM = 0.6

    ayni = True
    kirli = []
    for cy in (180.0, 240.0, 301.0, 360.0, 430.0):
        for vzi in (-3.0, -0.5, 0.0, 0.5, 3.0):
            a = B.komut(320.0, cy, 30.0, 20.0, 0.0, 15.0, 0.05, E, False,
                        (0.0, 0.0), 0.0, vzi, 1.0, 0.0, 0.0)
            b0 = B.komut(320.0, cy, 30.0, 20.0, 0.0, 15.0, 0.05, Kol, False,
                         (0.0, 0.0), 0.0, vzi, 1.0, 0.0, 0.0)
            if a[:4] != b0[:4]:
                ayni = False
                kirli.append((cy, vzi))
    kontrol("DS1 TUTUS_SONUM=0 iken tutuş yasası BİT BİT eski (25 kombinasyon)",
            ayni, "" if ayni else f"bozan: {kirli[:2]}")

    # ── DS2: TERMİNALE DOKUNMAZ ─────────────────────────────────────────
    ayni = True
    for cy in (240.0, 300.0, 380.0):
        for vzi in (-2.0, 0.0, 2.0):
            a = B.komut(320.0, cy, 30.0, 20.0, 0.0, 15.0, 0.05, E, True,
                        (0.0, 0.0), 0.0, vzi, 1.0, 0.0, 0.0)
            b0 = B.komut(320.0, cy, 30.0, 20.0, 0.0, 15.0, 0.05, F, True,
                         (0.0, 0.0), 0.0, vzi, 1.0, 0.0, 0.0)
            if a[:4] != b0[:4]:
                ayni = False
    kontrol("DS2 D-S terminal fazına DOKUNMAZ (9 kombinasyon)", ayni)

    # ── DS3: SÖNÜMLEME DOĞRU YÖNDE ──────────────────────────────────────
    # Araç istenenden HIZLI tırmanıyorsa (iris_vz çok negatif) komut GERİ
    # çekilmeli; yavaş kalıyorsa komut artmalı.
    cy = 260.0                       # hedef yukarıda → tırmanma istenir
    yavas = B.komut(320.0, cy, 30.0, 20.0, 0.0, 15.0, 0.05, F, False,
                    (0.0, 0.0), 0.0, 0.0, 1.0, 0.0, 0.0)[2]
    hizli = B.komut(320.0, cy, 30.0, 20.0, 0.0, 15.0, 0.05, F, False,
                    (0.0, 0.0), 0.0, -3.0, 1.0, 0.0, 0.0)[2]
    kontrol("DS3 araç zaten hızlı tırmanıyorsa komut GERİ çekilir",
            hizli > yavas, f"vz {yavas:+.2f} (durgun) → {hizli:+.2f} (tırmanırken)")

    from control import gcs_server as gcs3
    satir = gcs3._OZELLIKLER.get("ds_tutus_sonum")
    if satir:
        _alan, tip, _e, _a, _env, _dg = satir
        sinif, _ad = gcs3._hedef_cfg(_alan)
        kontrol("DS4 panel düğmesi Cfg.TUTUS_SONUM'a bağlı",
                sinif is B.Cfg and _ad == "TUTUS_SONUM"
                and _env == "AVCI_IBVS_TUTUS_SONUM" and _dg == (0.0, 0.6))
    else:
        kontrol("DS4 panel düğmesi var", False)

    kalan = [ad for ad, ok in sonuclar if not ok]
    print(f"SONUÇ: {len(sonuclar) - len(kalan)}/{len(sonuclar)} geçti"
          + (f" — KALAN: {kalan}" if kalan else " — HEPSİ GEÇTİ ✓"))
    return 1 if kalan else 0


if __name__ == "__main__":
    raise SystemExit(main())
