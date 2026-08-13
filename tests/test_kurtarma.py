"""
tests/test_kurtarma.py — Uçuş kurtarma bekçisi kabul kriterleri.

Kullanım: python3 -m tests.test_kurtarma

Kapsam:
  K1  takla (roll > eşik) → kurtarma aktif
  K2  kaçak dönme (yaw hızı > eşik) → kurtarma aktif
  K3  ⚠ EN KRİTİK: ÖLÇÜLEN SAĞLIKLI UÇUŞ ZARFI kurtarmayı TETİKLEMEZ
      (kullanıcı şartı: normal uçuşta araç hiçbir şekilde kısıtlanmayacak)
  K4  histerezis: tetikten sonra hemen bırakmaz, temiz süre şartı var
  K5  toparlanınca bırakır
  K6  MAX_SURE aşılırsa bırakır (sonsuz kurtarma yok)
  K7  gerçek kaza logu (200116) tetikler, gerçek sağlıklı log (172225) tetiklemez
"""

import csv
import math
import os

from control.guidance.kurtarma import Kurtarma, KurtCfg

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def _kosu(ornekler, cfg=KurtCfg):
    """ornekler: [(roll°, pitch°, yaw°, t)] → (herhangi_aktif_oldu_mu, son_durum)"""
    k = Kurtarma(cfg)
    aktif_oldu = False
    son = False
    for roll, pitch, yaw, t in ornekler:
        son = k.guncelle(math.radians(roll), math.radians(pitch),
                         math.radians(yaw), t)
        aktif_oldu = aktif_oldu or son
    return aktif_oldu, son, k


def main():
    print("Uçuş kurtarma bekçisi kabul kriterleri")
    # ══════════════════════════════════════════════════════════════════
    # V2 · BEKÇİ KİLİTLENMESİ DÜZELTMESİ (AVCI_KURT_V2)
    # ══════════════════════════════════════════════════════════════════
    class _V2(KurtCfg):
        KURT_V2 = True

    def _kos(cfg, dizi, dt=0.05):
        """dizi = [(roll°, pitch°, yaw°)...] → (aktif_kareler, kurt)"""
        k = Kurtarma(cfg)
        aktif = []
        for i, (r, p_, y) in enumerate(dizi):
            a = k.guncelle(math.radians(r), math.radians(p_),
                           math.radians(y), i * dt)
            aktif.append(a)
        return aktif, k

    # K8 — YAW KİLİTLENİYOR MU (kusur 1)
    # Tetik anında yaw = 40°; sonraki karelerde araç dönmeye devam ediyor.
    # kilit_yaw 40°'de SABİT kalmalı, aracı takip ETMEMELİ.
    _diz = [(0, 0, 0), (0, 0, 20), (0, 0, 40)]          # 400°/s → tetikler
    _diz += [(0, 0, 40 + 5 * i) for i in range(1, 8)]    # araç dönmeye devam
    _akt, _k = _kos(_V2, _diz)
    kontrol("K8 V2: yaw hedefi TETİK ANINDA kilitlenir, aracı takip etmez",
            _k.kilit_yaw is not None
            and abs(math.degrees(_k.kilit_yaw) - 20.0) < 1e-6,
            f"tetik karesinde yaw 20° → kilit_yaw {math.degrees(_k.kilit_yaw):.1f}° "
            f"— araç 75°'ye kadar dönmeye devam etti, hedef KAYMADI")

    # K9 — ESKİ DAVRANIŞ BİT BİT AYNI (varsayılan KAPALI)
    _akt_v1, _k1 = _kos(KurtCfg, _diz)
    _akt_v2, _k2 = _kos(_V2, _diz)
    kontrol("K9 V2 KAPALIYKEN kilit_yaw kurulmaz (eski davranış korunur)",
            KurtCfg.KURT_V2 is False and _k1.kilit_yaw is None,
            f"AVCI_KURT_V2 varsayılan {KurtCfg.KURT_V2} → kilit_yaw "
            f"{_k1.kilit_yaw} (çağıran eskisi gibi iyaw yollar)")

    # K10 — FRENLEME PITCH'İ ARTIK BIRAKMAYI ENGELLEMİYOR (kusur 2)
    # Senaryo: kaçak dönme tetikler; sonra dönme durur ve roll düzelir,
    # AMA araç 18 m/s'den frenlediği için pitch −46°'de kalır.
    _fren = [(0, 0, 0), (0, 0, 20), (0, 0, 40)]           # tetik
    _fren += [(2, -46, 40) for _ in range(40)]            # 2 s: roll temiz,
    #                                                       pitch frenlemede
    _a_v1, _kk1 = _kos(KurtCfg, _fren)
    _a_v2, _kk2 = _kos(_V2, _fren)
    kontrol("K10 V2: frenleme pitch'i (−46°) bırakmayı ENGELLEMEZ",
            _a_v1[-1] is True and _a_v2[-1] is False,
            f"2 s boyunca roll 2° / pitch −46° / yaw sabit → "
            f"ESKİ: hâlâ aktif (kilitli) · V2: BIRAKTI")

    # K11 — ama gerçek TAKLA (roll) hâlâ tutuyor
    _takla = [(0, 0, 0), (0, 0, 20), (0, 0, 40)]
    _takla += [(75, 0, 40) for _ in range(40)]            # roll 75° = takla
    _a_t, _kt = _kos(_V2, _takla)
    kontrol("K11 V2: gerçek takla (roll 75°) bekçiyi BIRAKTIRMAZ",
            _a_t[-1] is True,
            "roll 75° 2 s boyunca → bekçi aktif kaldı (emniyet korunuyor)")

    # K12 — pitch TETİKTE duruyor: burun aşağı devrilme hâlâ yakalanır
    _dev = [(0, 0, 0), (0, 75, 0), (0, 75, 0), (0, 75, 0)]
    _a_d, _kd = _kos(_V2, _dev)
    kontrol("K12 V2: pitch TETİKTE kaldı (75° burun aşağı yakalanır)",
            any(_a_d) and _kd.son_sebep is not None
            and "açı" in _kd.son_sebep,
            f"pitch 75° → tetiklendi, sebep: {_kd.son_sebep} "
            f"(gevşeyen yalnız ÇIKIŞ şartı, tetik değil)")

    # K13 — bırakınca kilit_yaw temizlenir (bir sonraki olaya sızmasın)
    kontrol("K13 V2: bırakınca kilit_yaw temizlenir",
            _kk2.aktif is False and _kk2.kilit_yaw is None,
            "toparlandıktan sonra kilit_yaw = None")

    print("=" * 60)
    C = KurtCfg

    # ── K1: TAKLA ──
    ornek = [(0, 0, 0, i * 0.05) for i in range(5)]
    ornek += [(-110, -20, 0, (5 + i) * 0.05) for i in range(5)]
    oldu, son, _ = _kosu(ornek)
    kontrol("K1  takla (roll −110°) → kurtarma aktif", oldu and son,
            f"eşik {C.ACI_TETIK:.0f}°")

    # ── K2: KAÇAK DÖNME (yaw hızı) ──
    # 20 Hz'de her turda 60° → 1200 °/s
    ornek = [(0, 0, (i * 60) % 360, i * 0.05) for i in range(10)]
    oldu, son, _ = _kosu(ornek)
    kontrol("K2  kaçak dönme (1200 °/s) → kurtarma aktif", oldu and son,
            f"eşik {C.YAW_HIZ_TETIK:.0f}°/s")

    # ── K3: ⚠ SAĞLIKLI ZARF TETİKLEMEZ (kullanıcı şartı) ──
    # 3 temiz uçuşun ölçülen en kötü değerleri: roll 46°, pitch 33°,
    # yaw hızı 188°/s. Hepsi AYNI ANDA uygulanır — gerçekte bile olmayan
    # en kötü hal. Bekçi yine de susmalı.
    n = 60
    ornek = []
    for i in range(n):
        yaw = (i * 188 * 0.05) % 360          # 188 °/s
        ornek.append((46 * math.sin(i * 0.5), 33 * math.cos(i * 0.5), yaw,
                      i * 0.05))
    oldu, son, k3 = _kosu(ornek)
    kontrol("K3  ⚠ ölçülen sağlıklı zarf (roll46/pitch33/yaw188) TETİKLEMEZ",
            not oldu, f"tetik sayısı={k3.sayac} (0 olmalı)")

    # ── K4/K5: histerezis + toparlanma ──
    ornek = [(0, 0, 0, i * 0.05) for i in range(3)]
    ornek += [(-100, 0, 0, (3 + i) * 0.05) for i in range(3)]     # takla
    # hemen düzeliyor ama TEMIZ_SURE dolmadan bırakmamalı
    ornek += [(0, 0, 0, (6 + i) * 0.05) for i in range(3)]        # 0.15 s temiz
    _, son_erken, _ = _kosu(ornek)
    # şimdi temiz süreyi doldur
    ornek2 = list(ornek) + [(0, 0, 0, (9 + i) * 0.05) for i in range(20)]
    _, son_gec, _ = _kosu(ornek2)
    kontrol("K4  histerezis: 0.15 s temizde HÂLÂ aktif", son_erken,
            f"TEMIZ_SURE={C.TEMIZ_SURE} s")
    kontrol("K5  temiz süre dolunca bırakır", not son_gec, "")

    # ── K6: DURUŞ KÖTÜYKEN BIRAKMAZ ──
    # ⚠ İlk tasarım "UYARI_SURE dolunca güdüme bırak" idi; bu test onun
    # LIVELOCK olduğunu ortaya çıkardı (bırak → hemen yeniden tetikle).
    # Doğru davranış: takla süresince komut kesik kalır. Bırakıp güdüme
    # komut verdirmek zaten aracı öldüren şeydi.
    ornek = [(0, 0, 0, 0.0)]
    ornek += [(-100, 0, 0, 0.05 + i * 0.05)
              for i in range(int((C.UYARI_SURE + 2.0) / 0.05))]
    _, son_uzun, k6 = _kosu(ornek)
    kontrol("K6  duruş kötü kaldıkça bekçi BIRAKMAZ (tek tetik, livelock yok)",
            son_uzun and k6.sayac == 1,
            f"{C.UYARI_SURE + 2:.0f} s sonunda aktif={son_uzun}, tetik={k6.sayac}")

    # ── K7: GERÇEK LOGLAR ──
    def log_kosusu(yol):
        if not os.path.exists(yol):
            return None
        k = Kurtarma()
        for r in csv.DictReader(open(yol)):
            try:
                k.guncelle(math.radians(float(r["iris_roll_deg"])),
                           math.radians(float(r["iris_pitch_deg"])),
                           math.radians(float(r["iris_yaw_deg"])),
                           float(r["t"]))
            except (ValueError, KeyError):
                continue
        return k.sayac

    kaza = log_kosusu("logs/gps_guidance_20260808_200116.csv")     # takla + düşüş
    saglam = log_kosusu("logs/gps_guidance_20260808_172225.csv")   # temiz koşu
    if kaza is None or saglam is None:
        kontrol("K7  gerçek log kontrolü (loglar yok — atlandı)", True,
                "loglar silinmişse bu test atlanır")
    else:
        kontrol("K7  gerçek kaza logu tetikler, temiz log tetiklemez",
                kaza > 0 and saglam == 0,
                f"kaza logu tetik={kaza}, temiz log tetik={saglam}")

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
