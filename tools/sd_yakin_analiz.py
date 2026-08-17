#!/usr/bin/env python3
"""S kampanyasi DALGA 3 — YAKIN MENZIL (<30 m) salinim + doz-tepki.
BIRINCIL kosmadan once ilan edildi: psi_dot isaret degisimi/s, gercek
menzil < 30 m, telem.csv 10 Hz KOSULSUZ."""
import csv, glob, json, math, os, statistics as st, itertools, sys
G=9.81
def f(x,k):
    try:
        v=float(x[k]); return v if v==v else None
    except: return None
def p90(v):
    v=sorted(v); return v[int(.9*len(v))] if len(v)>5 else None
def hz(s):
    s=sorted(s)
    if len(s)<20: return None
    n=sum(1 for a,b in zip(s,s[1:]) if a[1]*b[1]<0 and 0<b[0]-a[0]<1.0)
    t=sum(b[0]-a[0] for a,b in zip(s,s[1:]) if 0<b[0]-a[0]<1.0)
    return (n/t if t>1 else None), t
def rota(ad, menzil):
    d=[]
    for x in csv.DictReader(open(f"logs/kacamak/{ad}/telem.csv")):
        try: d.append((float(x["wall_t"]),float(x["iris_x"]),float(x["iris_y"]),float(x["mesafe"])))
        except (ValueError,KeyError): pass
    psi=[]
    for i in range(5,len(d)):
        t0,x0,y0,_=d[i-5]; t1,x1,y1,m1=d[i]
        if math.hypot(x1-x0,y1-y0)<0.5: continue
        psi.append((t1,math.atan2(y1-y0,x1-x0),m1))
    out=[]
    for (t0,a0,_),(t1,a1,m1) in zip(psi,psi[1:]):
        dt=t1-t0
        if 0.05<dt<0.5 and m1<menzil:
            out.append((t1,math.degrees(((a1-a0+math.pi)%(2*math.pi)-math.pi)/dt)))
    if len(out)<20: return None,None,None
    h,sure=hz(out)
    return h, p90([abs(v) for _,v in out]), sure
def kosu(ad):
    kok=f"logs/kacamak/{ad}"
    o=json.load(open(f"{kok}/olay.json"))
    kutulu=toplam=mek=0; ivme=[]; boyutlar=[]
    for y in sorted(glob.glob(f"{kok}/bbox_ibvs_*.csv")):
        r=list(csv.DictReader(open(y)))
        if len(r)<5: continue
        toplam+=len(r)
        for x in r:
            rl=f(x,"iris_roll_deg")
            if rl is not None: ivme.append(abs(G*math.tan(math.radians(rl))))
            if x.get("durum") in ("KUTU_YOK","TERM_KOR"): continue
            kutulu+=1
            s=f(x,"sonum_deg")
            if s is not None and abs(s)>1e-9: mek+=1
            b=f(x,"boyut")
            if b and b>0: boyutlar.append(b)
    h30,p30,s30=rota(ad,30.0)
    h15,_,_=rota(ad,15.0)
    return dict(ad=ad, imha=bool(o["imha"]), en_yakin_olay=o["en_yakin"],
        psi30=h30, psi30_p90=p30, sure30=s30, psi15=h15,
        ivme_p90=p90(ivme), kutu=100.0*kutulu/toplam if toplam else 0,
        mek=100.0*mek/kutulu if kutulu else 0,
        en_yakin=(160.0/max(boyutlar) if boyutlar else None))
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
KOL={}
for ad in sorted(os.listdir("logs/kacamak")):
    if not ad.startswith("SD") or not ad[2:4].isdigit(): continue
    if not os.path.exists(f"logs/kacamak/{ad}/olay.json"): continue
    KOL.setdefault(ad.split("_")[1],[]).append(kosu(ad))
ETIKET={"K":"K  SONUM=0","A":"A  SONUM=0.30","B":"B  SONUM=0.60"}
print("="*118)
print("DALGA 3 · DUZ + KACAMAK — YAKIN MENZIL SALINIMI (birincil <30 m, ILAN EDILDI)")
print("="*118)
print(f"{'kosu':<18}{'kol':>4}{'isabet':>8}{'PSI<30m':>9}{'sure s':>8}{'psi p90':>9}"
      f"{'PSI<15m':>9}{'IVME p90':>10}{'kutu%':>7}{'enyak':>7}{'MEK%':>7}")
for kol in ("K","A","B"):
    for s in KOL.get(kol,[]):
        print(f"{s['ad']:<18}{kol:>4}{'EVET' if s['imha'] else '-':>8}"
              f"{(s['psi30'] or float('nan')):>9.3f}{(s['sure30'] or 0):>8.0f}"
              f"{(s['psi30_p90'] or float('nan')):>9.1f}{(s['psi15'] or float('nan')):>9.3f}"
              f"{(s['ivme_p90'] or float('nan')):>10.2f}{s['kutu']:>7.1f}"
              f"{(s['en_yakin'] or float('nan')):>7.1f}{s['mek']:>7.1f}")
print("-"*118)
K=KOL.get("K",[])
for kol in ("K","A","B"):
    g=KOL.get(kol,[])
    if not g: continue
    ip=med([s['ivme_p90'] for s in g])
    oran=100.0*ip/med([s['ivme_p90'] for s in K]) if K else float('nan')
    print(f"  {ETIKET[kol]:<14} n={len(g)}  ISABET {sum(s['imha'] for s in g)}/{len(g)}"
          f" | PSI<30m {med([s['psi30'] for s in g]):6.3f}"
          f"  <15m {med([s['psi15'] for s in g]):6.3f}"
          f"  p90 {med([s['psi30_p90'] for s in g]):5.1f}"
          f" | IVME p90 {ip:5.2f} (kontrolun %{oran:.0f}'i)"
          f" | kutu {med([s['kutu'] for s in g]):4.1f}%"
          f" enyak {med([s['en_yakin'] for s in g]):4.1f}m"
          f" MEK {med([s['mek'] for s in g]):4.1f}%"
          f" sure {med([s['sure30'] for s in g]):3.0f}s")
if len(K)>=3:
    print()
    for kol in ("A","B"):
        g=KOL.get(kol,[])
        if len(g)<3: continue
        for ad,key in (("BIRINCIL psi<30m","psi30"),("KISIT ivme p90","ivme_p90"),
                       ("IK kutu orani","kutu")):
            a=[s[key] for s in K if s[key] is not None]
            b=[s[key] for s in g if s[key] is not None]
            gg,pp=perm(a,b)
            print(f"  {kol} {ad:<20} K={[round(x,3) for x in a]} vs {[round(x,3) for x in b]}  fark {gg:.3f} p={pp:.3f}")
