#!/usr/bin/env python3
"""S kampanyasi — S_SALINIM.md'de ILAN EDILEN olcutler."""
import csv, glob, json, math, os, statistics as st, itertools, sys
G = 9.81
def f(x,k):
    try:
        v=float(x[k]); return v if v==v else None
    except: return None
def p90(v):
    v=sorted(v); return v[int(.9*len(v))] if len(v)>5 else None
def hz(seri):
    seri=sorted(seri)
    if len(seri)<20: return None
    n=sum(1 for a,b in zip(seri,seri[1:]) if a[1]*b[1]<0 and 0<b[0]-a[0]<1.0)
    s=sum(b[0]-a[0] for a,b in zip(seri,seri[1:]) if 0<b[0]-a[0]<1.0)
    return n/s if s>1 else None

def rota(ad, menzil=60.0):
    d=[]
    for x in csv.DictReader(open(f"logs/kacamak/{ad}/telem.csv")):
        try: d.append((float(x["wall_t"]),float(x["iris_x"]),float(x["iris_y"]),float(x["mesafe"])))
        except (ValueError,KeyError): pass
    psi=[]
    for i in range(5,len(d)):
        t0,x0,y0,_=d[i-5]; t1,x1,y1,m1=d[i]
        if math.hypot(x1-x0,y1-y0)<0.5: continue
        psi.append((t1,math.atan2(y1-y0,x1-x0),m1))
    pd=[]
    for (t0,a0,_),(t1,a1,m1) in zip(psi,psi[1:]):
        dt=t1-t0
        if not (0.05<dt<0.5): continue
        pd.append((t1,math.degrees(((a1-a0+math.pi)%(2*math.pi)-math.pi)/dt),m1))
    s=[x for x in pd if x[2]<menzil]
    if len(s)<30: return None,None
    return hz([(t,v) for t,v,_ in s]), p90([abs(v) for _,v,_ in s])

def kosu(ad):
    kok=f"logs/kacamak/{ad}"
    o=json.load(open(f"{kok}/olay.json"))
    kutulu=toplam=kurt=0
    sonum_n=0
    ivme=[]; boyutlar=[]; cx_s=[]; slew=[]; komut_slew=[]
    for y in sorted(glob.glob(f"{kok}/bbox_ibvs_*.csv")):
        r=list(csv.DictReader(open(y)))
        if len(r)<5: continue
        toplam+=len(r)
        onceki=None
        for x in r:
            if x.get("durum")=="KURTARMA": kurt+=1
            rl=f(x,"iris_roll_deg"); t=f(x,"t")
            if rl is not None: ivme.append(abs(G*math.tan(math.radians(rl))))
            if x.get("durum") in ("KUTU_YOK","TERM_KOR"): onceki=None; continue
            kutulu+=1
            v=f(x,"sonum_deg")
            if v is not None and abs(v)>1e-9: sonum_n+=1
            b=f(x,"boyut"); cx=f(x,"cx")
            if b and b>0: boyutlar.append(b)
            if cx is not None and t is not None: cx_s.append((t,cx-320.0))
            iy=f(x,"iris_yaw_deg"); e=f(x,"eps_yaw_deg"); l=f(x,"lead_az_deg") or 0.0
            hy=None if None in (iy,e) else iy+e+l
            if onceki and hy is not None and t is not None:
                dt=t-onceki[0]
                if 0.02<dt<0.3:
                    komut_slew.append(abs(((hy-onceki[1]+180)%360-180)/dt))
            if hy is not None and t is not None: onceki=(t,hy)
    m=list(csv.DictReader(open(f"{kok}/meta.csv")))
    d=[f(x,"mesafe") for x in m]; d=[x for x in d if x is not None]
    ph,pp=rota(ad)
    return dict(ad=ad, imha=bool(o["imha"]),
        s60=sum(1 for x in d if x<60), med_d=st.median(d) if d else None,
        en_yakin=(160.0/max(boyutlar) if boyutlar else None),
        kutu=100.0*kutulu/toplam if toplam else 0, kurt=kurt,
        ivme_p90=p90(ivme), ivme_med=st.median(ivme) if ivme else None,
        psi_hz=ph, psi_p90=pp,
        cx_hz=hz(cx_s), cx_p90=p90([abs(v) for _,v in cx_s]),
        mek_sonum=100.0*sonum_n/kutulu if kutulu else 0,
        kslew_p90=p90(komut_slew))

def med(v):
    v=[x for x in v if x is not None]
    return st.median(v) if v else float('nan')
def perm(a,b):
    hep=a+b; goz=abs(st.median(a)-st.median(b)); c=t=0
    for idx in itertools.combinations(range(len(hep)),len(a)):
        x=[hep[i] for i in idx]; y=[hep[i] for i in range(len(hep)) if i not in idx]
        t+=1
        if abs(st.median(x)-st.median(y))>=goz-1e-12: c+=1
    return goz,c/t

tip = sys.argv[1] if len(sys.argv)>1 else "kare"
KOL={}
for ad in sorted(os.listdir("logs/kacamak")):
    if not (ad.startswith("SL") or ad.startswith("SW")) or not ad[2:4].isdigit(): continue
    if not os.path.exists(f"logs/kacamak/{ad}/olay.json"): continue
    if tip=="kare" and ("duz" in ad or "daire" in ad): continue
    if tip=="duz" and "duz" not in ad: continue
    if tip=="daire" and "daire" not in ad: continue
    parca=ad.split("_")
    KOL.setdefault(parca[1],[]).append(kosu(ad))

print("="*136)
print(f"S KAMPANYASI · {tip.upper()}   (jerk 10 HER KOLDA AYNI)")
print("="*136)
print(f"{'kosu':<12}{'kol':>5}{'isabet':>7}{'PSI dgs/s':>11}{'psi p90':>9}"
      f"{'IVME p90':>10}{'ivme med':>10}{'60m s':>7}{'medD':>7}{'kutu%':>7}{'enyak':>7}"
      f"{'kslew p90':>11}{'MEK%':>7}{'cx dgs':>8}{'KURT':>6}")
for kol in sorted(KOL):
    for s in KOL[kol]:
        mek=s['mek_sonum']
        print(f"{s['ad']:<12}{kol:>5}{'EVET' if s['imha'] else '-':>7}"
              f"{(s['psi_hz'] or float('nan')):>11.3f}{(s['psi_p90'] or float('nan')):>9.1f}"
              f"{(s['ivme_p90'] or float('nan')):>10.2f}{(s['ivme_med'] or float('nan')):>10.2f}"
              f"{s['s60']:>7}{s['med_d']:>7.1f}{s['kutu']:>7.1f}{(s['en_yakin'] or float('nan')):>7.1f}"
              f"{(s['kslew_p90'] or float('nan')):>11.1f}{mek:>7.1f}"
              f"{(s['cx_hz'] or float('nan')):>8.3f}{s['kurt']:>6}")
print("-"*136)
K=KOL.get("K",[])
for kol in sorted(KOL):
    g=KOL[kol]
    mek=med([s['mek_sonum'] for s in g])
    ip=med([s['ivme_p90'] for s in g])
    oran=(100.0*ip/med([s['ivme_p90'] for s in K])) if K else float('nan')
    print(f"  {kol:<4} n={len(g)}  PSI {med([s['psi_hz'] for s in g]):6.3f}"
          f"  p90 {med([s['psi_p90'] for s in g]):5.1f}"
          f" | IVME p90 {ip:5.2f} (kontrolun %{oran:.0f}'i)"
          f" med {med([s['ivme_med'] for s in g]):4.2f}"
          f" | 60m {med([s['s60'] for s in g]):5.0f}s"
          f" medD {med([s['med_d'] for s in g]):5.1f}m"
          f" kutu {med([s['kutu'] for s in g]):4.1f}%"
          f" enyak {med([s['en_yakin'] for s in g]):4.1f}m"
          f" | kslew p90 {med([s['kslew_p90'] for s in g]):5.1f}"
          f" MEK {mek:4.1f}%")
if len(K)>=3:
    print()
    for kol in sorted(KOL):
        if kol=="K": continue
        g=KOL[kol]
        if len(g)<3: continue
        for ad,key in (("BIRINCIL psi dgs/s","psi_hz"),("IK-1 60m sure","s60"),
                       ("KISIT ivme p90","ivme_p90")):
            a=[s[key] for s in K if s[key] is not None]
            b=[s[key] for s in g if s[key] is not None]
            gg,pp=perm(a,b)
            print(f"  {kol:<4} {ad:<22} K={[round(x,3) for x in a]} vs {[round(x,3) for x in b]}  fark {gg:.3f} p={pp:.3f}")
