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
        # 2026-08-23: kacamak_testi.py "kacamak.csv" yazıyor; eski koşularda
        # dosya adı "telem.csv" idi. İkisi de kabul edilir.
        tp = next((os.path.join(d, f) for f in ("telem.csv", "kacamak.csv")
                   if os.path.exists(os.path.join(d, f))), None)
        if tp is None:
            print(f"  ⚠ {d}: telem.csv / kacamak.csv yok, atlanıyor")
            continue
        tel = []
        for x in csv.DictReader(open(tp)):
            if "plane_x" not in x or "wall_t" not in x:
                print(f"  ⚠ {tp}: wall_t/konum sütunları yok — bu koşu "
                      f"2026-08-23 öncesi, kalibrasyon için kullanılamaz")
                break
            p = (F(x["plane_x"]), F(x["plane_y"]), F(x["plane_z"]))
            i = (F(x["iris_x"]), F(x["iris_y"]), F(x["iris_z"]))
            t = F(x["wall_t"])
            if None in p + i or t is None:
                continue
            tel.append((t, math.dist(p, i), p, i))
        if not tel:
            continue
        # ── BAKIŞ AÇISI (aspect) — 2026-08-24 eklendi ──
        # Eski C sabitleri "0-15° görüş açısı bandında" ölçülmüştü (bkz.
        # DEVAM.md ve bbox_ibvs yorumu). Bu araç bandı SÜZMÜYORDU, yani
        # tüm açıları harmanlayıp FARKLI bir büyüklük veriyordu.
        # Açı, hedefin HIZ YÖNÜNDEN türetilir (telemetride heading yok):
        #     ileri = birim(konum[k+1] − konum[k−1])
        #     los   = birim(avci − hedef)
        #     aci   = açı(−ileri, los)      0° = TAM ARKADAN (kuyruk)
        aci = []
        for k in range(len(tel)):
            a, b = max(0, k-1), min(len(tel)-1, k+1)
            p0, p1 = tel[a][2], tel[b][2]
            v = [p1[j]-p0[j] for j in range(3)]
            nv = math.sqrt(sum(c*c for c in v))
            los = [tel[k][3][j]-tel[k][2][j] for j in range(3)]
            nl = math.sqrt(sum(c*c for c in los))
            if nv < 1e-6 or nl < 1e-6:
                aci.append(float("nan")); continue
            # kuyruk ekseni = −ileri
            cs = sum((-v[j]/nv)*(los[j]/nl) for j in range(3))
            aci.append(math.degrees(math.acos(max(-1.0, min(1.0, cs)))))
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
                ornek.append((tel[k][1], w, h, aci[k]))
    return ornek


def main():
    dizinler = sys.argv[1:]
    if not dizinler:
        print(__doc__)
        return 1
    # --tum-acilar verilirse band uygulanmaz (tarama/kıyas için)
    band = "--tum-acilar" not in dizinler
    dizinler = [d for d in dizinler if not d.startswith("--")]
    ornek = ornekle(dizinler)
    ham = len(ornek)
    if band:
        ornek = [o for o in ornek if o[3] == o[3] and o[3] <= 15.0]
        print(f"{ham} ham örnek → {len(ornek)} tanesi 0-15° BANDINDA "
              f"(%{100*len(ornek)/max(1,ham):.0f})")
        print("  ⚠ Eski C sabitleri bu bantta ölçülmüştü; kıyas ancak böyle "
              "anlamlı.\n  Tüm açılar için: --tum-acilar\n")
    else:
        print(f"{len(ornek)} eşleşmiş örnek — TÜM AÇILAR (band yok)\n")
    if len(ornek) < 50:
        print("⛔ örnek az, kıyas anlamsız")
        return 1

    # ── model tabanlı beklenti (mini_talon_vtail collision mesh) ──
    FX = 166.58
    KANAT, BOY, YUK = 1.718, 1.093, 0.383   # X-UAV Talon (2026-08-22 ölçekleme)
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
        p = [fn(w, h) for _, w, h, _a in ornek]
        R = [r for r, _, _, _a in ornek]
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
    # ⚠ 2026-08-24: burada "160.0" ELLE YAZILMIŞTI ve eskimişti (o değer
    # 2026-08-19'da 185.7 olmuştu, sonra Talon ölçeklemesiyle 249.2).
    # Artık kodun GERÇEK değerleri okunuyor — rapor kendiliğinden güncel kalır.
    try:
        import sys as _s
        _s.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from control.guidance.bbox_ibvs import Cfg as _C
        _kod_car, _kod_kos = _C.MENZIL_PX_M_CARPIM, _C.MENZIL_PX_M_KOSEGEN
        _kul = _C.BOYUT_OLCU
    except Exception:
        _kod_car = _kod_kos = None
        _kul = "?"
    print("\n  ── KODDAKİ DEĞERLE KIYAS ──")
    if _kod_car is None:
        print("     (bbox_ibvs okunamadı)")
    else:
        for _ad, _kod, _olc in (("CARPIM", _kod_car, bug[0]),
                                ("KOSEGEN", _kod_kos, kos[0])):
            _yildiz = " ⭐ KULLANILAN" if _kul.upper() in _ad else ""
            print(f"     MENZIL_PX_M_{_ad:7s} kod {_kod:7.1f} · ölçülen {_olc:7.1f}"
                  f" → %{100*(_kod-_olc)/_olc:+5.1f} sapma{_yildiz}")
        print("     (kod > ölçülen ⇒ güdüm hedefi olduğundan UZAK sanıyor)")
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
