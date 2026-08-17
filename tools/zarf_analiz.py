#!/usr/bin/env python3
"""FAZ C — komut zarfi seviyesi analizi (AYAR_FAZ_C.md'de ILAN EDILEN olcutler)."""
import csv, glob, json, math, os, statistics as st, itertools, sys
G = 9.81

def p90(v):
    v = sorted(v)
    return v[int(.9 * len(v))] if len(v) > 5 else float("nan")

def dz_ve_enyakin(ad):
    """<25 m'lik her yaklasmada EN YAKIN andaki dz ve mesafe."""
    p = f"logs/kacamak/{ad}/telem.csv"
    if not os.path.exists(p): return None
    R = []
    for x in csv.DictReader(open(p)):
        try:
            px, py, pz = float(x["plane_x"]), float(x["plane_y"]), float(x["plane_z"])
            ix, iy, iz = float(x["iris_x"]), float(x["iris_y"]), float(x["iris_z"])
        except (ValueError, KeyError): continue
        R.append((math.sqrt((px-ix)**2 + (py-iy)**2 + (pz-iz)**2), pz - iz))
    yak, akt = [], None
    for d, z in R:
        if d < 25:
            akt = akt or []; akt.append((d, z))
        elif akt:
            yak.append(akt); akt = None
    if akt: yak.append(akt)
    if not yak: return None
    return ([min(a, key=lambda q: q[0])[1] for a in yak],
            [min(a, key=lambda q: q[0])[0] for a in yak])

def yatis(ad):
    """20 Hz gudum logundan |yatis| p90 — mekanizma kapisi."""
    R = []
    for y in glob.glob(f"logs/kacamak/{ad}/bbox_ibvs_*.csv"):
        for x in csv.DictReader(open(y)):
            try:
                v = abs(float(x["iris_roll_deg"]))
                if v == v and v < 90: R.append(v)
            except (ValueError, KeyError, TypeError): pass
    return p90(R) if len(R) > 50 else float("nan")

def kutu_orani(ad):
    k = t = 0
    for y in glob.glob(f"logs/kacamak/{ad}/bbox_ibvs_*.csv"):
        for x in csv.DictReader(open(y)):
            t += 1
            if x.get("durum") not in ("KUTU_YOK", "TERM_KOR"): k += 1
    return 100.0 * k / t if t else float("nan")

def perm(a, b):
    hep = a + b; goz = abs(st.median(a) - st.median(b)); c = n = 0
    for idx in itertools.combinations(range(len(hep)), len(a)):
        x = [hep[i] for i in idx]; y = [hep[i] for i in range(len(hep)) if i not in idx]
        n += 1
        if abs(st.median(x) - st.median(y)) >= goz - 1e-12: c += 1
    return goz, c / n

ET = {"esk": "ESKI  45°/8 /12", "ara": "ARA   55°/15/20", "yen": "YENI  70°/26/40"}
G_ = {}
print("=" * 108)
print("FAZ C · KOMUT ZARFI SEVIYESI — square (surekli manevra), n=3/kol")
print("=" * 108)
print(f"{'kosu':<12}{'kol':>5}{'yak':>5}{'|dz| med':>10}{'en yakin':>10}"
      f"{'|yatis| p90':>13}{'kutu%':>8}   dz degerleri")
for ad in sorted(os.listdir("logs/kacamak")):
    if not ad.startswith("TD") or not ad[2:4].isdigit(): continue
    kol = ad.split("_")[1]
    r = dz_ve_enyakin(ad)
    if not r:
        print(f"{ad:<12}{kol:>5}   yaklasma yok"); continue
    d, e = r
    dzm = st.median([abs(z) for z in d]); enm = st.median(e)
    yp = yatis(ad); ko = kutu_orani(ad)
    G_.setdefault(kol, {"dz": [], "en": [], "yat": [], "kutu": []})
    G_[kol]["dz"].append(dzm); G_[kol]["en"].append(enm)
    G_[kol]["yat"].append(yp); G_[kol]["kutu"].append(ko)
    print(f"{ad:<12}{kol:>5}{len(d):>5}{dzm:>9.2f}m{enm:>9.2f}m"
          f"{yp:>12.1f}°{ko:>7.1f}%   {[round(z,2) for z in d]}")
print("-" * 108)
for kol in ("esk", "ara", "yen"):
    g = G_.get(kol)
    if not g: continue
    print(f"  {ET[kol]:<18} n={len(g['dz'])}"
          f" | BIRINCIL-1 |dz| {st.median(g['dz']):5.2f} m"
          f" | BIRINCIL-2 |yatis| p90 {st.median(g['yat']):5.1f}°"
          f" | ES: en yakin {st.median(g['en']):6.2f} m"
          f" | kutu {st.median(g['kutu']):4.1f}%")
if all(k in G_ and len(G_[k]["dz"]) >= 3 for k in ("esk", "ara", "yen")):
    print()
    for ad, key in (("BIRINCIL-1 |dz|", "dz"), ("BIRINCIL-2 |yatis| p90", "yat"),
                    ("ES en yakin menzil", "en")):
        for a, b in (("yen", "ara"), ("yen", "esk"), ("ara", "esk")):
            g, p = perm(G_[a][key], G_[b][key])
            print(f"  {ad:<24} {a} vs {b}: "
                  f"{[round(x,2) for x in G_[a][key]]} vs {[round(x,2) for x in G_[b][key]]}"
                  f"  fark {g:.2f}  p={p:.3f}")
        print()
    # ILAN EDILEN KAPI
    yen_en = st.median(G_["yen"]["en"])
    print("ILAN EDILEN GECERLILIK ESI: en yakin menzil, YENI kolun %125'ini")
    print(f"asan kol ELENIR.  YENI = {yen_en:.2f} m  ->  esik {1.25*yen_en:.2f} m")
    for kol in ("ara", "esk"):
        e = st.median(G_[kol]["en"])
        print(f"  {ET[kol]:<18} en yakin {e:6.2f} m  "
              f"{'✓ gecti' if e <= 1.25*yen_en else '⛔ ELENDI'}")
