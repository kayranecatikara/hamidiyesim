"""Ö-T — terminal mandalını SÜREYLE bırakma testleri.

Kullanım:
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_ot_terminal_sure.py

NE KANITLIYOR:
  * T1 — YAPISAL GARANTİ (§5.10): TERM_BIRAK_S=0 iken mandal davranışı
    Ö-M'nin bit bit aynısı. Ö-T kapalıyken hiçbir şey değişmez.
  * T2-T5 — açıkken: sayaç birikiyor mu, kapanınca sıfırlanıyor mu,
    eşiği aşınca mandal bırakılıyor mu, mandal yokken sayaç sıfır mı.
  * T6 — MEKANİZMA KAPISI (§5.1): `term_kapanmasiz` CSV sütunu tanımlı ve
    panel düğmesi doğru alana bağlı.

Mandal döngüsü canlı uçuş içinde olduğu için burada AYNI mantık birebir
yeniden kurulup sınanıyor; kodla tutarlılığı T7 kaynak denetimiyle bağlanır.
"""

import os
import re

from control.guidance import bbox_ibvs as B


def mandal_adimi(cfg, terminal_mandal, sayac, en_iyi, menzil, dt):
    """bbox_ibvs döngüsündeki Ö-T bloğunun birebir aynısı."""
    if terminal_mandal and cfg.TERM_BIRAK_S > 0.0 and menzil is not None:
        if en_iyi is None or menzil < en_iyi - cfg.TERM_BIRAK_EPS:
            en_iyi = menzil
            sayac = 0.0
        else:
            sayac += dt
            if sayac >= cfg.TERM_BIRAK_S:
                terminal_mandal = False
                sayac = 0.0
                en_iyi = None
    elif not terminal_mandal:
        sayac = 0.0
        en_iyi = None
    return terminal_mandal, sayac, en_iyi


class SahteCfg:
    TERM_BIRAK_S = 0.0
    TERM_BIRAK_EPS = 0.5


def main():
    sonuclar = []

    def kontrol(ad, kosul, detay=""):
        sonuclar.append((ad, bool(kosul)))
        print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")

    print("Ö-T · terminal mandalını süreyle bırak")
    c = SahteCfg()

    # ── T1: KAPALIYKEN HİÇBİR ŞEY OLMAZ (yapısal garanti, §5.10) ────────
    c.TERM_BIRAK_S = 0.0
    mandal, sayac, en_iyi, bozan = True, 0.0, None, None
    for _ in range(2000):                    # 100 s, menzil hiç iyileşmiyor
        mandal, sayac, en_iyi = mandal_adimi(c, mandal, sayac, en_iyi, 8.0, 0.05)
        if mandal is not True or sayac != 0.0:
            bozan = (mandal, sayac)
            break
    kontrol("T1 kapalıyken mandal 100 s ilerlemesiz kalsa da DÜŞMEZ",
            mandal is True and sayac == 0.0, "" if not bozan else f"bozan: {bozan}")

    # ── T2: AÇIKKEN İLERLEME YOKSA EŞİKTE BIRAKIR ───────────────────────
    c.TERM_BIRAK_S = 4.0
    mandal, sayac, en_iyi, gecen = True, 0.0, None, 0.0
    mandal, sayac, en_iyi = mandal_adimi(c, mandal, sayac, en_iyi, 8.0, 0.05)
    while mandal and gecen < 20.0:
        mandal, sayac, en_iyi = mandal_adimi(c, mandal, sayac, en_iyi, 8.0, 0.05)
        gecen += 0.05
    kontrol("T2 açıkken 4 s ilerlemesiz sonrası mandal BIRAKILIR",
            mandal is False and abs(gecen - 4.0) < 0.11,
            f"bırakma süresi {gecen:.2f} s")

    # ── T3: YENİ EN İYİ SAYACI SIFIRLAR ─────────────────────────────────
    mandal, sayac, en_iyi = True, 0.0, None
    mandal, sayac, en_iyi = mandal_adimi(c, mandal, sayac, en_iyi, 8.0, 0.05)
    for _ in range(70):                                  # 3.5 s ilerleme yok
        mandal, sayac, en_iyi = mandal_adimi(c, mandal, sayac, en_iyi, 8.0, 0.05)
    yarim = sayac
    mandal, sayac, en_iyi = mandal_adimi(c, mandal, sayac, en_iyi, 7.0, 0.05)
    kontrol("T3 yeni en iyi menzil sayacı SIFIRLAR",
            mandal is True and sayac == 0.0 and abs(en_iyi - 7.0) < 1e-9,
            f"({yarim:.2f} s birikmişti, yeni en iyi {en_iyi:.1f} m)")

    # ── T4: SÜREKLİ YAKLAŞIRKEN ASLA BIRAKMAZ ───────────────────────────
    mandal, sayac, en_iyi, m = True, 0.0, None, 60.0
    for _ in range(1000):
        m -= 0.05                                        # 1 m/s ile kapanıyor
        mandal, sayac, en_iyi = mandal_adimi(c, mandal, sayac, en_iyi, m, 0.05)
    kontrol("T4 sürekli yaklaşırken 50 s boyunca mandal DÜŞMEZ",
            mandal is True, f"menzil {m:.1f} m")

    # ── T5: SALINIM SAYACI SIFIRLAMAZ — ASIL DÜZELTME ───────────────────
    # ⚠ İLK TASARIM BURADA ÇÖKTÜ: anlık kapanmaya bakıyordu ve menzil
    # 7↔12 m salınırken kapanma karelerin %44.9'unda eşiği aşıp sayacı
    # sıfırlıyordu (kullanıcı uçuşu 181339). En iyi menzil MONOTON olduğu
    # için salınımdan etkilenmez.
    mandal, sayac, en_iyi = True, 0.0, None
    mandal, sayac, en_iyi = mandal_adimi(c, mandal, sayac, en_iyi, 7.0, 0.05)
    i, dustu = 0, False
    while i < 400 and mandal:                            # 20 s salınım
        m = 7.0 if (i // 10) % 2 == 0 else 12.0          # 7 ↔ 12 m gidip gel
        mandal, sayac, en_iyi = mandal_adimi(c, mandal, sayac, en_iyi, m, 0.05)
        i += 1
    dustu = mandal is False
    kontrol("T5 7↔12 m SALINIMINDA mandal bırakılır (ilk tasarım bunu kaçırdı)",
            dustu, f"{i * 0.05:.1f} s içinde")

    # ── T6: MANDAL YOKKEN TEMİZ ─────────────────────────────────────────
    mandal, sayac, en_iyi = mandal_adimi(c, False, 3.0, 9.0, 8.0, 0.05)
    kontrol("T6 mandal kapalıyken sayaç ve en iyi sıfırlanır",
            sayac == 0.0 and en_iyi is None)

    # ── T7: MEKANİZMA KAPISI (§5.1) ─────────────────────────────────────
    kontrol("T7 `term_kapanmasiz` CSV sütunu tanımlı",
            "term_kapanmasiz" in B._CSV_ALANLAR)
    kontrol("T7b Cfg alanları var",
            hasattr(B.Cfg, "TERM_BIRAK_S") and hasattr(B.Cfg, "TERM_BIRAK_EPS"))
    kontrol("T7c varsayılan KAPALI (kill-switch)",
            B.Cfg.TERM_BIRAK_S == 0.0, f"TERM_BIRAK_S={B.Cfg.TERM_BIRAK_S}")

    from control import gcs_server as gcs
    satir = gcs._OZELLIKLER.get("ot_term_sure")
    kontrol("T7d panelde düğme var", satir is not None)
    if satir:
        alan, tip, _et, _ac, env, deger = satir
        sinif, ad = gcs._hedef_cfg(alan)
        kontrol("T7e düğme bbox_ibvs.Cfg.TERM_BIRAK_S'e yazıyor",
                sinif is B.Cfg and ad == "TERM_BIRAK_S", f"{sinif.__name__}.{ad}")
        kontrol("T7f env anahtarı ve açık/kapalı değerleri doğru",
                env == "AVCI_IBVS_TERM_BIRAK_S" and tip == "deger"
                and deger == (0.0, 4.0))

    # ── T8: TESTTEKİ MANTIK KODDAKİYLE AYNI MI ──────────────────────────
    kaynak = open(B.__file__, encoding="utf-8").read()
    kontrol("T8 kod en iyi menzili EPS ile kıyaslıyor",
            "term_en_iyi - cfg.TERM_BIRAK_EPS" in kaynak)
    kontrol("T8b kodda eşik aşılınca mandal bırakılıyor",
            "term_kapanmasiz >= cfg.TERM_BIRAK_S" in kaynak)
    kontrol("T8c kod ANLIK kapanmayı ARTIK kullanmıyor (çürütülen tasarım)",
            "cfg.TERM_BIRAK_KAPANMA" not in kaynak)
    kontrol("T8d Ö-M'nin menzil kapısı DURUYOR (Ö-T onu değiştirmedi)",
            "cfg.MENZIL_PX_M / boyut_simdi > cfg.TERM_BIRAK_M" in kaynak)

    kalan = [ad for ad, ok in sonuclar if not ok]
    print(f"SONUÇ: {len(sonuclar) - len(kalan)}/{len(sonuclar)} geçti"
          + (f" — KALAN: {kalan}" if kalan else " — HEPSİ GEÇTİ ✓"))
    return 1 if kalan else 0


if __name__ == "__main__":
    raise SystemExit(main())
