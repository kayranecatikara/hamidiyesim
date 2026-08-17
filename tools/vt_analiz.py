#!/usr/bin/env python3
"""V_TERMINAL taramasi — V_TERMINAL.md'de ILAN EDILEN olcutler."""
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
    return n/t if t>1 else None
def rota(ad,menzil=30.0):
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
    return hz(out) if len(out)>20 else None
def kosu(ad):
    kok=f"logs/kacamak/{ad}"
    o=json.load(open(f"{kok}/olay.json"))
    vt=[];term_n=0;kutulu=toplam=kurt=0;ivme=[];kisik=0
    for y in sorted(glob.glob(f"{kok}/bbox_ibvs_*.csv")):
        r=list(csv.DictReader(open(y)))
        if len(r)<5: continue
        toplam+=len(r)
        for x in r:
            if x.get("durum")=="KURTARMA": kurt+=1
            rl=f(x,"iris_roll_deg")
            if rl is not None and abs(rl)<90: ivme.append(abs(G*math.tan(math.radians(rl))))
            if x.get("durum")=="TERMINAL":
                term_n+=1
                v=f(x,"v_los")
                if v is not None: vt.append(v)
            if x.get("durum") not in ("KUTU_YOK","TERM_KOR"): kutulu+=1
    # terminal son 2 s nisan sapmasi
    en=None
    for y in glob.glob(f"{kok}/bbox_ibvs_*.csv"):
        r=[x for x in csv.DictReader(open(y)) if x.get("boyut")]
        if len(r)<20: continue
        try: b=max(float(x["boyut"]) for x in r if x["boyut"])
        except ValueError: continue
        if en is None or b>en[0]: en=(b,r)
    nis=None
    if en:
        r=en[1]; ts=max(float(x["t"]) for x in r if x.get("t"))
        cx=[abs(float(x["cx"])-320) for x in r
            if x.get("t") and ts-float(x["t"])<2.0 and x.get("cx")
            and x["durum"] not in ("KUTU_YOK","TERM_KOR")]
        nis=st.median(cx) if cx else None
    hedef={"K":16.0,"A":20.0,"B":24.0}[ad.split("_")[1]]
    return dict(ad=ad, imha=bool(o["imha"]), en_yakin=o["en_yakin"],
        v_med=st.median(vt) if vt else None,
        kisik=100.0*sum(1 for v in vt if v<hedef-0.5)/len(vt) if vt else 0,
        term_s=term_n/20.0, kutu=100.0*kutulu/toplam if toplam else 0,
        ivme_p90=p90(ivme), psi=rota(ad), nisan=nis, kurt=kurt, hedef=hedef)
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
sen = sys.argv[1] if len(sys.argv)>1 else "duz"
KOL={}
for ad in sorted(os.listdir("logs/kacamak")):
    if not ad.startswith("VT") or not ad[2:4].isdigit(): continue
    if not os.path.exists(f"logs/kacamak/{ad}/olay.json"): continue
    p=ad.split("_")
    tip = "duz" if ("yatay" in ad or "capraz" in ad or "dikey" in ad) else "kare"
    if tip!=sen: continue
    KOL.setdefault(p[1],[]).append(kosu(ad))
ET={"K":"K  V=16","A":"A  V=20","B":"B  V=24"}
print("="*116)
print(f"V_TERMINAL TARAMASI · {sen.upper()}")
print("="*116)
print(f"{'kosu':<18}{'kol':>4}{'isabet':>8}{'ENYAKIN':>9}{'v_los med':>11}{'kisik%':>8}"
      f"{'term s':>8}{'nisan px':>10}{'kutu%':>7}{'ivme90':>8}{'psi':>7}{'KURT':>6}")
for kol in ("K","A","B"):
    for s in KOL.get(kol,[]):
        print(f"{s['ad']:<18}{kol:>4}{'EVET' if s['imha'] else '-':>8}{s['en_yakin']:>8.2f}m"
              f"{(s['v_med'] or float('nan')):>11.2f}{s['kisik']:>7.0f}%{s['term_s']:>8.1f}"
              f"{(s['nisan'] or float('nan')):>10.0f}{s['kutu']:>7.1f}"
              f"{(s['ivme_p90'] or float('nan')):>8.2f}{(s['psi'] or float('nan')):>7.3f}{s['kurt']:>6}")
print("-"*116)
for kol in ("K","A","B"):
    g=KOL.get(kol,[])
    if not g: continue
    print(f"  {ET[kol]:<10} n={len(g)}  ISABET {sum(s['imha'] for s in g)}/{len(g)}"
          f" | EN YAKIN {med([s['en_yakin'] for s in g]):5.2f} m"
          f" | v_los {med([s['v_med'] for s in g]):5.2f} (kisik %{med([s['kisik'] for s in g]):.0f})"
          f" term {med([s['term_s'] for s in g]):4.1f}s"
          f" | nisan {med([s['nisan'] for s in g]):4.0f}px"
          f" kutu {med([s['kutu'] for s in g]):4.1f}%"
          f" ivme90 {med([s['ivme_p90'] for s in g]):5.2f}"
          f" psi {med([s['psi'] for s in g]):5.3f}")
K=KOL.get("K",[])
if len(K)>=3:
    print()
    for kol in ("A","B"):
        g=KOL.get(kol,[])
        if len(g)<3: continue
        for ad,key in (("BIRINCIL en yakin","en_yakin"),("es: nisan sapmasi","nisan")):
            a=[s[key] for s in K if s[key] is not None]
            b=[s[key] for s in g if s[key] is not None]
            gg,pp=perm(a,b)
            print(f"  {kol} {ad:<20} K={[round(x,2) for x in a]} vs {[round(x,2) for x in b]}  fark {gg:.2f} p={pp:.3f}")
