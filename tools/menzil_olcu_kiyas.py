#!/usr/bin/env python3
"""KUTU→MENZİL ÖLÇÜSÜ KIYASI — hangi büyüklük menzili daha iyi veriyor?

⚠ ÇEVRİMDIŞI ANALİZ (§2): eski loglardan hipotez üretir, KABUL KARARI VERMEZ.

Soru: R = C/p bağıntısında `p` olarak ne alalım?
  (a) sqrt(w·h)      — geometrik ortalama (bugünkü)
  (b) sqrt(w²+h²)    — KÖŞEGEN (kullanıcının önerisi)
  (c) w              — yalnız genişlik (kanat açıklığı)
  (d) h              — yalnız yükseklik
  (e) max(w,h)

Yöntem: 20 Hz güdüm logundaki (w,h) ile 10 Hz telem.csv'deki GERÇEK menzil
duvar saatiyle eşlenir. Her ölçü için C = medyan(p·R_gercek) kalibre edilir,
sonra R_tahmin = C/p ile GERÇEK menzil kıyaslanır.

⚠ GERÇEK MENZİL YALNIZ ANALİZDE KULLANILIR — güdüm onu görmez (§10).

Kullanım: python3 tools/menzil_olcu_kiyas.py <kacamak_dizini> [...]
"""
import csv, glob, math, os, statistics as st, sys


def F(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


OLCULER = {
    "sqrt(w·h)  [bugünkü]": lambda w, h: math.sqrt(w * h),
    "sqrt(w²+h²) KÖŞEGEN": lambda w, h: math.hypot(w, h),
    "w (genişlik)": lambda w, h: w,
    "h (yükseklik)": lambda w, h: h,
    "max(w,h)": lambda w, h: max(w, h),
}


def ornekle(dizinler):
    """(gercek_menzil, w, h) uclulerini topla."""
    ornek = []
    for d in dizinler:
        tp = os.path.join(d, "telem.csv")
        if not os.path.exists(tp):
            continue
        tel = []
        for x in csv.DictReader(open(tp)):
            p = (F(x["plane_x"]), F(x["plane_y"]), F(x["plane_z"]))
            i = (F(x["iris_x"]), F(x["iris_y"]), F(x["iris_z"]))
            t = F(x["wall_t"])
            if None in p + i or t is None:
                continue
            tel.append((t, math.dist(p, i)))
        if not tel:
            continue
        t0, t1 = tel[0][0], tel[-1][0]
        for lp in glob.glob("logs/bbox_ibvs_*.csv"):
            m = os.path.getmtime(lp)
            if not (t0 - 70 <= m <= t1 + 70):
                continue
            rows = list(csv.DictReader(open(lp)))
            if not rows:
                continue
            son = F(rows[-1]["t"])
            if son is None:
                continue
            ofs = m - son          # monotonik saat → duvar saati
            for x in rows:
                tt, w, h = F(x["t"]), F(x["w"]), F(x["h"])
                if None in (tt, w, h) or w < 4 or h < 3:
                    continue
                tw = tt + ofs
                if not (t0 <= tw <= t1):
                    continue
                # en yakın telem örneği (10 Hz → en fazla 50 ms sapma)
                k = min(range(len(tel)), key=lambda n: abs(tel[n][0] - tw))
                if abs(tel[k][0] - tw) > 0.06:
                    continue
                ornek.append((tel[k][1], w, h))
    return ornek


def main():
    dizinler = sys.argv[1:]
    if not dizinler:
        print(__doc__)
        return 1
    ornek = ornekle(dizinler)
    print(f"{len(ornek)} eşleşmiş örnek (gerçek menzil ↔ kutu)\n")
    if len(ornek) < 50:
        print("⛔ örnek az, kıyas anlamsız")
        return 1

    # ── model tabanlı beklenti (mini_talon_vtail collision mesh) ──
    FX = 166.58
    KANAT, BOY, YUK = 1.280, 0.814, 0.286
    print("MODEL ÖLÇÜLERİ (collision mesh'ten):")
    print(f"  kanat açıklığı {KANAT:.3f} m · gövde boyu {BOY:.3f} m · "
          f"yükseklik {YUK:.3f} m")
    print(f"  arkadan bakışta beklenen: sqrt(w·h) → "
          f"{math.sqrt(KANAT*YUK):.3f} m,  köşegen → "
          f"{math.hypot(KANAT, YUK):.3f} m\n")

    print(f"{'ölçü':<22}{'kalibre C':>11}{'ima S':>8}"
          f"{'BAĞIL HATA p50':>16}{'p90':>8}{'saçılma':>10}")
    print("  " + "─" * 72)
    sonuc = {}
    for ad, fn in OLCULER.items():
        p = [fn(w, h) for _, w, h in ornek]
        R = [r for r, _, _ in ornek]
        # C = medyan(p·R) — her örnek kendi C'sini ima eder
        Cs = [pi * ri for pi, ri in zip(p, R)]
        C = st.median(Cs)
        # bağıl hata: |R_tahmin − R_gercek| / R_gercek
        hata = [abs(C / pi - ri) / ri for pi, ri in zip(p, R)]
        hata.sort()
        p50 = hata[len(hata) // 2]
        p90 = hata[int(0.9 * len(hata))]
        # saçılma: C tahminlerinin kendi içindeki değişkenliği (düşük = tutarlı)
        sac = (st.median([abs(c - C) for c in Cs]) / C)
        sonuc[ad] = (C, p50, p90, sac)
        print(f"  {ad:<20}{C:>11.1f}{C/FX:>8.3f}"
              f"{100*p50:>14.0f}%{100*p90:>7.0f}%{100*sac:>9.0f}%")

    print()
    en_iyi = min(sonuc, key=lambda k: sonuc[k][1])
    print(f"  ⭐ EN DÜŞÜK BAĞIL HATA: {en_iyi}  "
          f"(p50 %{100*sonuc[en_iyi][1]:.0f})")
    bug = sonuc["sqrt(w·h)  [bugünkü]"]
    kos = sonuc["sqrt(w²+h²) KÖŞEGEN"]
    print(f"  bugünkü p50 %{100*bug[1]:.0f} → köşegen p50 %{100*kos[1]:.0f}"
          f"   ({'İYİLEŞME' if kos[1] < bug[1] else 'KÖTÜLEŞME'} "
          f"%{100*abs(kos[1]-bug[1])/bug[1]:.0f})")
    print(f"\n  ⚠ Bugün kullanılan sabit MENZIL_PX_M = 160.0; "
          f"ölçülen kalibre {bug[0]:.1f} "
          f"(%{100*(160.0-bug[0])/bug[0]:+.0f} sapma)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


def kirilim(ornek, ad_fn, kirilimlar, baslik, etiket_fn):
    """Ölçüleri bir kırılıma göre ayrı ayrı değerlendir."""
    print(f"\n{baslik}")
    basliklar = [etiket_fn(k) for k in kirilimlar]
    print(f"  {'ölçü':<22}" + "".join(f"{b:>14}" for b in basliklar))
    print("  " + "─" * (22 + 14 * len(kirilimlar)))
    for ad, fn in OLCULER.items():
        satir = f"  {ad:<22}"
        for k in kirilimlar:
            alt = [o for o in ornek if k[0](o)]
            if len(alt) < 30:
                satir += f"{'—':>14}"
                continue
            p = [fn(w, h) for _, w, h in alt]
            R = [r for r, _, _ in alt]
            C = st.median([pi * ri for pi, ri in zip(p, R)])
            hata = sorted(abs(C / pi - ri) / ri for pi, ri in zip(p, R))
            satir += f"{100*hata[len(hata)//2]:>12.0f}% "
        print(satir)
