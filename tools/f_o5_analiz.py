#!/usr/bin/env python3
"""F kampanyasi (O5 karede) — O5_KARE.md'de ILAN EDILEN olcutler."""
import csv, glob, json, math, os, statistics as st, itertools, sys

def f(x, k):
    try:
        v = float(x[k]); return v if v == v else None
    except (KeyError, TypeError, ValueError): return None

def hz(seri):
    seri = sorted(seri)
    if len(seri) < 20: return None
    n = sum(1 for a, b in zip(seri, seri[1:]) if a[1]*b[1] < 0 and 0 < b[0]-a[0] < 1.0)
    s = sum(b[0]-a[0] for a, b in zip(seri, seri[1:]) if 0 < b[0]-a[0] < 1.0)
    return n/s if s > 1 else None

def p90(v):
    v = sorted(v); return v[int(.9*len(v))] if len(v) > 5 else None

def rota(ad):
    """telem.csv 10 Hz, KOSULSUZ -> kutu yanliligindan bagimsiz salinim."""
    d = []
    for x in csv.DictReader(open(f"logs/kacamak/{ad}/telem.csv")):
        try: d.append((float(x["wall_t"]), float(x["iris_x"]), float(x["iris_y"]),
                       float(x["mesafe"])))
        except (ValueError, KeyError): pass
    psi = []
    for i in range(5, len(d)):
        t0, x0, y0, _ = d[i-5]; t1, x1, y1, m1 = d[i]
        if math.hypot(x1-x0, y1-y0) < 0.5: continue
        psi.append((t1, math.atan2(y1-y0, x1-x0), m1))
    pd = []
    for (t0, a0, _), (t1, a1, m1) in zip(psi, psi[1:]):
        dt = t1-t0
        if not (0.05 < dt < 0.5): continue
        da = (a1-a0+math.pi) % (2*math.pi) - math.pi
        pd.append((t1, math.degrees(da/dt), m1))
    s = [x for x in pd if x[2] < 60]
    if len(s) < 30: return None, None
    return hz([(t, v) for t, v, _ in s]), p90([abs(v) for _, v, _ in s])

def kosu(ad):
    kok = os.path.join("logs/kacamak", ad)
    o = json.load(open(os.path.join(kok, "olay.json")))
    kutulu = toplam = tavanli = taban = 0
    vv, boyutlar = [], []
    for y in sorted(glob.glob(os.path.join(kok, "bbox_ibvs_*.csv"))):
        r = list(csv.DictReader(open(y)))
        if len(r) < 5: continue
        toplam += len(r)
        for x in r:
            if x.get("durum") in ("KUTU_YOK", "TERM_KOR"): continue
            kutulu += 1
            t = x.get("donus_tavan", "")
            if t not in ("", None):
                tavanli += 1
                if float(t) <= 10.001: taban += 1
            v = f(x, "v_los");  b = f(x, "boyut")
            if v is not None: vv.append(v)
            if b and b > 0: boyutlar.append(b)
    m = list(csv.DictReader(open(os.path.join(kok, "meta.csv"))))
    d = [f(x, "mesafe") for x in m]; d = [x for x in d if x is not None]
    P = [(f(x, "wall_t"), f(x, "iris_x"), f(x, "iris_y"), f(x, "iris_spd")) for x in m]
    P = [p for p in P if None not in p]
    R = []
    for i in range(2, len(P)):
        (t0,x0,y0,_), (t1,x1,y1,v1), (t2,x2,y2,_) = P[i-2], P[i-1], P[i]
        h1 = math.atan2(y1-y0, x1-x0); h2 = math.atan2(y2-y1, x2-x1)
        dh = (h2-h1+math.pi) % (2*math.pi) - math.pi; dt = t2-t1
        if dt < 0.5 or abs(dh) < math.radians(1.5) or v1 < 5: continue
        R.append(v1/abs(dh/dt))
    ph, pp = rota(ad)
    return dict(ad=ad, imha=bool(o["imha"]),
                s60=sum(1 for x in d if x < 60), med_d=st.median(d) if d else None,
                en_yakin=(160.0/max(boyutlar) if boyutlar else None),
                kutu=100.0*kutulu/toplam if toplam else 0,
                tavan=100.0*tavanli/kutulu if kutulu else 0,
                taban=100.0*taban/kutulu if kutulu else 0,
                v_med=st.median(vv) if vv else None,
                R_med=st.median(R) if R else None,
                psi_hz=ph, psi_p90=pp)

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

def goster(baslik, K, O, birincil=True):
    print("="*122); print(baslik); print("="*122)
    print(f"{'kosu':<18}{'kol':>4}{'isabet':>8}{'60m s':>7}{'medD':>8}{'enyak':>8}"
          f"{'kutu%':>7}{'TAVAN%':>8}{'taban%':>8}{'v_los':>7}{'R med':>8}{'psi dgs':>9}{'psi p90':>9}")
    for g, et in ((K, "K"), (O, "Ö5")):
        for s in g:
            print(f"{s['ad']:<18}{et:>4}{'EVET' if s['imha'] else 'hayir':>8}"
                  f"{s['s60']:>7}{s['med_d']:>8.1f}{(s['en_yakin'] or float('nan')):>8.1f}"
                  f"{s['kutu']:>7.1f}{s['tavan']:>8.1f}{s['taban']:>8.1f}"
                  f"{(s['v_med'] or float('nan')):>7.2f}{(s['R_med'] or float('nan')):>8.1f}"
                  f"{(s['psi_hz'] or float('nan')):>9.3f}{(s['psi_p90'] or float('nan')):>9.1f}")
    print("-"*122)
    for et, g in (("KONTROL", K), ("Ö5     ", O)):
        print(f"  {et} n={len(g)} ISABET {sum(s['imha'] for s in g)}/{len(g)}"
              f" | 60m {med([s['s60'] for s in g]):5.1f}s"
              f" medD {med([s['med_d'] for s in g]):5.1f}m"
              f" enyak {med([s['en_yakin'] for s in g]):4.1f}m"
              f" kutu {med([s['kutu'] for s in g]):4.1f}%"
              f" | TAVAN {med([s['tavan'] for s in g]):4.1f}%"
              f" taban {med([s['taban'] for s in g]):4.1f}%"
              f" v {med([s['v_med'] for s in g]):5.2f}"
              f" R {med([s['R_med'] for s in g]):5.1f}m"
              f" | psi {med([s['psi_hz'] for s in g]):5.3f}"
              f" p90 {med([s['psi_p90'] for s in g]):4.1f}")
    if birincil and len(K) >= 3 and len(O) >= 3:
        print()
        for ad, key in (("BIRINCIL 60 m icinde sure", "s60"),
                        ("IK-1 medyan mesafe", "med_d"),
                        ("IK-2 psi dgs/s", "psi_hz")):
            a = [s[key] for s in K if s[key] is not None]
            b = [s[key] for s in O if s[key] is not None]
            if len(a) < 3 or len(b) < 3: continue
            g, p = perm(a, b)
            print(f"  {ad:<28} K={[round(x,3) for x in a]}  Ö5={[round(x,3) for x in b]}"
                  f"  fark {g:.3f}  p={p:.3f}")
    print()

G = {}
for ad in sorted(os.listdir("logs/kacamak")):
    if not ad.startswith("F") or not os.path.exists(f"logs/kacamak/{ad}/olay.json"): continue
    kol = ad.split("_")[1]
    tip = "kare" if "_kare" in ad else ("daire" if "daire" in ad else "duz")
    G.setdefault((tip, kol), []).append(kosu(ad))

if G.get(("kare","K")): goster("KARE — KAZANIM SENARYOSU", G[("kare","K")], G.get(("kare","O"),[]))
if G.get(("duz","K")):  goster("DUZ — REGRESYON",           G[("duz","K")],  G.get(("duz","O"),[]))
if G.get(("daire","K")):goster("DAIRE — MODEL CURUTME (KARAR VERMEZ)", G[("daire","K")], G.get(("daire","O"),[]), False)
