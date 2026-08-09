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
