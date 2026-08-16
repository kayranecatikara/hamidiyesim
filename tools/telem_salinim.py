#!/usr/bin/env python3
"""SALINIM — KUTUDAN BAGIMSIZ olcum. Kaynak telem.csv (10 Hz, KOSULSUZ).

Kutu-tabanli cx olcutu §5.2 tuzagina acik: hedefi daha cok kaybeden kol
daha sakin gorunur. telem.csv her karede yaziliyor -> secilim yok.
Rota acisi ψ, aracin KENDI konum izinden turetilir; ψ̇ isaret degisimi =
'kontrolsuz savrulma'. 10 Hz, salinim periyodu 4-5 s -> ~45 ornek/periyot.
"""
import csv, math, os, statistics as st, sys, itertools

def yukle(ad):
    p = f"logs/kacamak/{ad}/telem.csv"
    r = list(csv.DictReader(open(p)))
    out = []
    for x in r:
        try:
            out.append((float(x["wall_t"]), float(x["iris_x"]), float(x["iris_y"]),
                        float(x["mesafe"]), x["faz"]))
        except (ValueError, KeyError):
            pass
    return out

def analiz(ad, kova=(0.0, 60.0)):
    d = yukle(ad)
    # 0.5 s'lik pencerede rota acisi (GPS gurultusunu sondur)
    N = 5
    psi = []
    for i in range(N, len(d)):
        t0, x0, y0, _, _ = d[i-N]; t1, x1, y1, m1, fz = d[i]
        if math.hypot(x1-x0, y1-y0) < 0.5:      # duruyorsa aci tanimsiz
            continue
        psi.append((t1, math.atan2(y1-y0, x1-x0), m1, fz))
    # ψ̇ (°/s)
    pd = []
    for (t0, a0, m0, f0), (t1, a1, m1, f1) in zip(psi, psi[1:]):
        dt = t1-t0
        if not (0.05 < dt < 0.5):
            continue
        da = (a1-a0+math.pi) % (2*math.pi) - math.pi
        pd.append((t1, math.degrees(da/dt), m1, f1))
    s = [x for x in pd if kova[0] <= x[2] < kova[1]]
    if len(s) < 30:
        return None
    n = sum(1 for a, b in zip(s, s[1:]) if a[1]*b[1] < 0 and 0 < b[0]-a[0] < 1.0)
    sure = sum(b[0]-a[0] for a, b in zip(s, s[1:]) if 0 < b[0]-a[0] < 1.0)
    ap = sorted(abs(x[1]) for x in s)
    return dict(n=len(s), sure=sure,
                psi_dgs=n/sure if sure > 1 else None,
                psi_med=st.median(ap), psi_p90=ap[int(.9*len(ap))])

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
    if not os.path.exists(f"logs/kacamak/{ad}/telem.csv"): continue
    SEV.setdefault(int(ad.split("_")[1][1:]), []).append(ad)

for lo, hi in ((0, 60), (0, 20), (20, 60)):
    print("="*96)
    print(f"ROTA SALINIMI (telem 10 Hz, KOSULSUZ) — GERCEK menzil {lo}-{hi} m")
    print("="*96)
    print(f"{'kosu':<16}{'jerk':>5}{'ornek':>8}{'sure s':>8}{'psi dgs/s':>11}"
          f"{'|psi| med':>11}{'|psi| p90':>11}")
    ozet = {}
    for j in sorted(SEV):
        vals = []
        for ad in SEV[j]:
            a = analiz(ad, (lo, hi))
            if a is None:
                print(f"{ad:<16}{j:>5}{'--- yetersiz ornek':>40}")
                continue
            vals.append(a)
            print(f"{ad:<16}{j:>5}{a['n']:>8}{a['sure']:>8.0f}"
                  f"{(a['psi_dgs'] or float('nan')):>11.3f}{a['psi_med']:>11.1f}{a['psi_p90']:>11.1f}")
        ozet[j] = vals
    print("-"*96)
    for j in sorted(ozet):
        v = ozet[j]
        if not v: continue
        print(f"{'  MEDYAN jerk '+str(j):<16}{j:>5}{med([a['n'] for a in v]):>8.0f}"
              f"{med([a['sure'] for a in v]):>8.0f}{med([a['psi_dgs'] for a in v]):>11.3f}"
              f"{med([a['psi_med'] for a in v]):>11.1f}{med([a['psi_p90'] for a in v]):>11.1f}")
    if lo == 0 and hi == 60:
        print()
        for a, b in itertools.combinations(sorted(ozet), 2):
            va = [x['psi_dgs'] for x in ozet[a]]; vb = [x['psi_dgs'] for x in ozet[b]]
            if len(va) < 3 or len(vb) < 3: continue
            g, p = perm(va, vb)
            print(f"  psi dgs/s  j{a} vs j{b}: {[round(x,3) for x in va]} vs "
                  f"{[round(x,3) for x in vb]}  fark {g:.3f}  p={p:.3f}")
    print()
