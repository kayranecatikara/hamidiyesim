"""Hedef irtifa tutucusunun regresyon testleri.

Kullanım:
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_irtifa_tutucu.py

NE KANITLIYOR:
  * İT1 — YAPISAL GARANTİ (CLAUDE.md §5.10): tutucu KAPALIYKEN pitch komutu,
    özellik eklenmeden önceki açık çevrim değerin BİT BİT aynısıdır. 32 girdi
    kombinasyonunda sınanır. Yani kill-switch gerçekten kapatıyor.
  * İT2-İT5 — açıkken işaret, sönüm, sınır ve tabanın korunması.
  * İT6 — MEKANİZMA KAPISI (§5.1): panel düğmesinin yazdığı alan ile
    senaryonun okuduğu alan AYNI. 2026-08-13'te K-V2 düğmesi yanlış sınıfa
    yazdığı için özellik hiç çalışmamıştı; bu test onun tekrarını engeller.
"""

import os

from control import run_plane_scenario as rps


def main():
    sonuclar = []

    def kontrol(ad, kosul, detay=""):
        sonuclar.append((ad, bool(kosul)))
        print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")

    print("Hedef irtifa tutucusu")

    _asil = rps.irtifa_tut_acik
    try:
        # ── İT1: KAPALIYKEN ÇIKTI DEĞİŞMEZ ──────────────────────────────
        # Açık çevrim davranış: düz fazda pitch=0, dairede pitch=taban.
        # Tutucu kapalıyken irtifa hatası ve tırmanma NE OLURSA OLSUN
        # komut tabana eşit kalmalı.
        rps.irtifa_tut_acik = lambda: False
        ayni = True
        kirli = []
        for taban in (0, 150, 172, 212):
            for hata_m in (-50.0, 0.0, 0.3, 97.0):
                for vz in (-3.0, 0.29):
                    rps._pos["z"] = -100.0
                    rps._pos["vz"] = vz
                    hedef = 100.0 + hata_m
                    cikti = rps._irtifa_pitch(hedef, taban)
                    if cikti != taban:
                        ayni = False
                        kirli.append((taban, hata_m, vz, cikti))
        kontrol("İT1 kapalıyken çıktı = taban (32 kombinasyon, bit bit)",
                ayni, "" if ayni else f"bozan girdiler: {kirli[:3]}")

        rps.irtifa_tut_acik = lambda: True

        # ── İT2: DENGEDE MÜDAHALE YOK ───────────────────────────────────
        rps._pos["z"] = -100.0
        rps._pos["vz"] = 0.0
        kontrol("İT2 hata=0 ve tırmanma=0 iken düzeltme yok",
                rps._irtifa_pitch(100.0, 150) == 150,
                f"çıktı={rps._irtifa_pitch(100.0, 150)}")

        # ── İT3: BUGÜN ÖLÇÜLEN DURUM GERİ ÇAĞRILIYOR MU ─────────────────
        # 172144 uçuşu: hedefte tgt_vz medyan −0.29 m/s (yükseliyor).
        # Hata sıfırken bile sönüm terimi burnu AŞAĞI almalı.
        rps._pos["z"] = -100.0
        rps._pos["vz"] = -0.29          # NED: negatif = yükseliyor
        cikti = rps._irtifa_pitch(100.0, 0)
        kontrol("İT3 tırmanırken burun AŞAĞI (sönüm terimi)",
                cikti < 0, f"çıktı={cikti} (beklenen ≈ -17)")

        # Alçalırken tersi olmalı.
        rps._pos["vz"] = 0.29
        kontrol("İT3b alçalırken burun YUKARI",
                rps._irtifa_pitch(100.0, 0) > 0,
                f"çıktı={rps._irtifa_pitch(100.0, 0)}")

        # ── İT4: SINIR — DÜZELTME KIRPILIR, TABAN KIRPILMAZ ─────────────
        # Dairenin yük faktörü payı (taban) kırpmaya DAHİL DEĞİL; yoksa
        # büyük irtifa hatasında daire trimi kaybolur ve uçak alçalır.
        rps._pos["z"] = -100.0
        rps._pos["vz"] = 0.0
        tavan = rps._irtifa_pitch(1000.0, 212)      # +900 m hata → doyum
        taban_ = rps._irtifa_pitch(-1000.0, 212)    # −1100 m hata → doyum
        kontrol("İT4 düzeltme ±IRTIFA_PITCH_MAX ile sınırlı, taban korunur",
                tavan == 212 + rps.IRTIFA_PITCH_MAX
                and taban_ == 212 - rps.IRTIFA_PITCH_MAX,
                f"tavan={tavan}, taban={taban_}")

        # Sınır, stall payı bırakacak kadar ılımlı mı (≈13.5°)?
        kontrol("İT4b pitch sınırı ≤ 300 birim (~13.5°)",
                rps.IRTIFA_PITCH_MAX <= 300,
                f"IRTIFA_PITCH_MAX={rps.IRTIFA_PITCH_MAX}")

        # ── İT5: GAZA DOKUNMAZ ──────────────────────────────────────────
        # Tutucu YALNIZ pitch üretir; throttle yolu bu satırlardan geçmez.
        kaynak = open(rps.__file__, encoding="utf-8").read()
        govde = kaynak[kaynak.index("def _irtifa_pitch"):]
        govde = govde[:govde.index("\ndef ", 1)]
        kontrol("İT5 _irtifa_pitch gaza dokunmaz (throttle geçmiyor)",
                "throttle" not in govde and "gcs_throttle" not in govde)
    finally:
        rps.irtifa_tut_acik = _asil

    # ── İT6: MEKANİZMA KAPISI — düğme doğru alana mı yazıyor ────────────
    from control import gcs_server as gcs
    from control.senaryo_cfg import SenaryoCfg
    satir = gcs._OZELLIKLER.get("hedef_irtifa_tut")
    kontrol("İT6a panelde düğme var", satir is not None)
    if satir:
        alan, tip, _etiket, _acik, env, acik_deger = satir
        sinif, ad = gcs._hedef_cfg(alan)
        kontrol("İT6b düğme SenaryoCfg.IRTIFA_TUT'a yazıyor",
                sinif is SenaryoCfg and ad == "IRTIFA_TUT",
                f"{sinif.__name__}.{ad}")
        kontrol("İT6c kill-switch env anahtarı bildirilmiş",
                env == "AVCI_SCN_IRTIFA_TUT" and tip == "bool"
                and acik_deger is True)
        # Düğmeyi kapat → endpoint kapalı demeli → senaryo kapalı okur.
        eski = SenaryoCfg.IRTIFA_TUT
        try:
            setattr(sinif, ad, False)
            kontrol("İT6d düğme kapanınca /api/senaryo_ayar kapalı döner",
                    gcs.get_senaryo_ayar()["irtifa_tut"] is False,
                    str(gcs.get_senaryo_ayar()))
            setattr(sinif, ad, True)
            kontrol("İT6e düğme açılınca açık döner",
                    gcs.get_senaryo_ayar()["irtifa_tut"] is True)
        finally:
            SenaryoCfg.IRTIFA_TUT = eski

    # ── İT7: ENV VARSAYILANI ────────────────────────────────────────────
    import importlib
    eski_env = os.environ.get("AVCI_SCN_IRTIFA_TUT")
    try:
        os.environ["AVCI_SCN_IRTIFA_TUT"] = "0"
        import control.senaryo_cfg as sc
        importlib.reload(sc)
        kontrol("İT7 AVCI_SCN_IRTIFA_TUT=0 özelliği kapatır",
                sc.SenaryoCfg.IRTIFA_TUT is False)
        os.environ["AVCI_SCN_IRTIFA_TUT"] = "1"
        importlib.reload(sc)
        kontrol("İT7b =1 açar", sc.SenaryoCfg.IRTIFA_TUT is True)
    finally:
        if eski_env is None:
            os.environ.pop("AVCI_SCN_IRTIFA_TUT", None)
        else:
            os.environ["AVCI_SCN_IRTIFA_TUT"] = eski_env
        importlib.reload(sc)

    kalan = [ad for ad, ok in sonuclar if not ok]
    print(f"SONUÇ: {len(sonuclar) - len(kalan)}/{len(sonuclar)} geçti"
          + (f" — KALAN: {kalan}" if kalan else " — HEPSİ GEÇTİ ✓"))
    return 1 if kalan else 0


if __name__ == "__main__":
    raise SystemExit(main())
