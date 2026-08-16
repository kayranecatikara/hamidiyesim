#!/usr/bin/env python3
"""D kampanyasi — D_JERK_TARAMA.md'de ILAN EDILEN olcutler."""
import csv, glob, json, math, os, statistics as st, itertools, sys
G = 9.81
KOVA = (10, 20)

def f(x, k):
    try:
        v = float(x[k]); return v if v == v else None
    except (KeyError, TypeError, ValueError): return None

def hz(seri):
    seri = sorted(seri)
    if len(seri) < 20: return None
    n = sum(1 for a, b in zip(seri, seri[1:]) if a[1]*b[1] < 0)
    s = sum(min(b[0]-a[0], 1.0) for a, b in zip(seri, seri[1:]) if b[0] > a[0])
    return n/s if s > 1 else None

def p90(v):
    v = sorted(v); return v[int(.9*len(v))] if len(v) > 5 else None

def kosu(ad):
    kok = os.path.join("logs/kacamak", ad)
    o = json.load(open(os.path.join(kok, "olay.json")))
    ivme, cx_k, roll_k, roll_hep, boyutlar = [], [], [], [], []
    kutulu = toplam = kurtarma = 0
    kova_n = 0
    for y in sorted(glob.glob(os.path.join(kok, "bbox_ibvs_*.csv"))):
        r = list(csv.DictReader(open(y)))
        if len(r) < 5: continue
        toplam += len(r)
        for x in r:
            d = x.get("durum", "")
            if d == "KURTARMA": kurtarma += 1
            rl = f(x, "iris_roll_deg"); t = f(x, "t")
            if rl is not None and t is not None:
                ivme.append((t, G*math.tan(math.radians(rl))))
                roll_hep.append((t, rl))
            if d in ("KUTU_YOK", "TERM_KOR"): continue
            kutulu += 1
            b = f(x, "boyut"); cx = f(x, "cx")
            if b and b > 0: boyutlar.append(b)
            if None in (b, cx, t) or not b: continue
            R = 160.0/b
            if KOVA[0] <= R < KOVA[1]:
                kova_n += 1
                cx_k.append((t, cx-320.0))
                if rl is not None: roll_k.append((t, rl))
    ivme.sort()
    jerk = sorted(abs(a1-a0)/(t1-t0) for (t0,a0),(t1,a1) in zip(ivme, ivme[1:])
                  if 0.02 < t1-t0 < 0.3)
    m = list(csv.DictReader(open(os.path.join(kok, "meta.csv"))))
    d = [f(x, "mesafe") for x in m]; d = [x for x in d if x is not None]
    return dict(
        ad=ad, imha=bool(o["imha"]), en_yakin_20hz=(160.0/max(boyutlar) if boyutlar else None),
        en_yakin_1hz=o["en_yakin"],
        med_d=st.median(d) if d else None, s60=sum(1 for x in d if x < 60),
        jerk_p90=p90(jerk), kutu=100.0*kutulu/toplam if toplam else 0,
        kova_n=kova_n, kurtarma=kurtarma,
        cx_hz=hz(cx_k), cx_p90=p90([abs(v) for _, v in cx_k]),
        roll_hz=hz(roll_k), roll_p90=p90([abs(v) for _, v in roll_k]),
    )

def med(v):
    v = [x for x in v if x is not None]
    return st.median(v) if v else float('nan')

def perm(a, b):
    hep = a+b; goz = abs(st.median(a)-st.median(b)); c = t = 0
    for idx in itertools.combinations(range(len(hep)), len(a)):
        x = [hep[i] for i in idx]; y = [hep[i] for i in range(len(hep)) if i not in idx]
        t += 1
        if abs(st.median(x)-st.median(y)) >= goz-1e-12: c += 1
    return goz, c/t

SEV = {}
for ad in sorted(os.listdir("logs/kacamak")):
    if not ad.startswith("D") or "_kare" not in ad: continue
    if not os.path.exists(f"logs/kacamak/{ad}/olay.json"): continue
    j = int(ad.split("_")[1][1:])
    SEV.setdefault(j, []).append(kosu(ad))

print("="*118)
print("D KAMPANYASI · KARE — jerk taramasi")
print("="*118)
print(f"{'kosu':<16}{'jerk':>5}{'isabet':>8}{'medD':>8}{'60m s':>7}{'enyak20':>9}"
      f"{'jerkp90':>9}{'kutu%':>7}{'kova_n':>8}{'cx_dgs':>8}{'cx_p90':>8}{'rl_dgs':>8}{'rl_p90':>8}{'KURT':>6}")
for j in sorted(SEV):
    for s in SEV[j]:
        print(f"{s['ad']:<16}{j:>5}{'EVET' if s['imha'] else 'hayir':>8}"
              f"{s['med_d']:>8.1f}{s['s60']:>7}"
              f"{(s['en_yakin_20hz'] or float('nan')):>9.1f}{(s['jerk_p90'] or float('nan')):>9.2f}"
              f"{s['kutu']:>7.1f}{s['kova_n']:>8}"
              f"{(s['cx_hz'] or float('nan')):>8.3f}{(s['cx_p90'] or float('nan')):>8.0f}"
              f"{(s['roll_hz'] or float('nan')):>8.3f}{(s['roll_p90'] or float('nan')):>8.1f}{s['kurtarma']:>6}")
print("-"*118)
print(f"{'SEVIYE MEDYAN':<16}{'jerk':>5}{'isabet':>8}{'medD':>8}{'60m s':>7}{'enyak20':>9}"
      f"{'jerkp90':>9}{'kutu%':>7}{'kova_n':>8}{'cx_dgs':>8}{'cx_p90':>8}{'rl_dgs':>8}{'rl_p90':>8}{'KURT':>6}")
for j in sorted(SEV):
    g = SEV[j]
    print(f"{'  jerk '+str(j):<16}{j:>5}{str(sum(s['imha'] for s in g))+'/'+str(len(g)):>8}"
          f"{med([s['med_d'] for s in g]):>8.1f}{med([s['s60'] for s in g]):>7.0f}"
          f"{med([s['en_yakin_20hz'] for s in g]):>9.1f}{med([s['jerk_p90'] for s in g]):>9.2f}"
          f"{med([s['kutu'] for s in g]):>7.1f}{med([s['kova_n'] for s in g]):>8.0f}"
          f"{med([s['cx_hz'] for s in g]):>8.3f}{med([s['cx_p90'] for s in g]):>8.0f}"
          f"{med([s['roll_hz'] for s in g]):>8.3f}{med([s['roll_p90'] for s in g]):>8.1f}"
          f"{sum(s['kurtarma'] for s in g):>6}")
print()
if len(SEV) >= 2 and min(len(v) for v in SEV.values()) >= 3:
    for ad, key in (("BIRINCIL-A medyan mesafe", "med_d"),
                    ("BIRINCIL-B cx dgs/s (kova)", "cx_hz"),
                    ("IK-1 60 m icinde sure", "s60")):
        for a, b in itertools.combinations(sorted(SEV), 2):
            va = [s[key] for s in SEV[a] if s[key] is not None]
            vb = [s[key] for s in SEV[b] if s[key] is not None]
            if len(va) < 3 or len(vb) < 3: continue
            g, p = perm(va, vb)
            print(f"{ad:<28} j{a} vs j{b}: {[round(x,3) for x in va]} vs {[round(x,3) for x in vb]}  fark {g:.3f}  p={p:.3f}")
        print()
