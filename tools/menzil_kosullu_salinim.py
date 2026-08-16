#!/usr/bin/env python3
"""Menzil-KOSULLU salinim: ayni menzilde K5 ve C15 ayni mi salaniyor?"""
import csv, glob, math, os, statistics as st, collections

KOVA = [(0,10),(10,20),(20,40),(40,999)]

def f(x,k):
    try:
        v=float(x[k]); return v if v==v else None
    except (KeyError,TypeError,ValueError): return None

def kosu(ad):
    kok=os.path.join("logs/kacamak",ad)
    kova={k:{"roll":[], "cx":[], "lam":[], "n":0} for k in KOVA}
    for y in sorted(glob.glob(os.path.join(kok,"bbox_ibvs_*.csv"))):
        r=list(csv.DictReader(open(y)))
        if len(r)<5: continue
        for x in r:
            if x.get("durum") in ("KUTU_YOK","TERM_KOR"): continue
            b=f(x,"boyut"); t=f(x,"t"); rl=f(x,"iris_roll_deg"); cx=f(x,"cx")
            lam=f(x,"los_hiz_az")
            if None in (b,t,rl,cx) or b<=0: continue
            R=160.0/b
            for k in KOVA:
                if k[0]<=R<k[1]:
                    kova[k]["n"]+=1
                    kova[k]["roll"].append((t,rl))
                    kova[k]["cx"].append((t,cx-320.0))
                    if lam is not None: kova[k]["lam"].append(abs(lam))
                    break
    return kova

def hz(seri):
    seri=sorted(seri)
    if len(seri)<20: return None
    n=sum(1 for a,b in zip(seri,seri[1:]) if a[1]*b[1]<0)
    s=sum(min(b[0]-a[0],1.0) for a,b in zip(seri,seri[1:]) if b[0]>a[0])
    return n/s if s>1 else None

def p90(v):
    v=sorted(v); return v[int(.9*len(v))] if len(v)>5 else None

def birlestir(adlar):
    top={k:{"roll":[],"cx":[],"lam":[],"n":0} for k in KOVA}
    for a in adlar:
        kv=kosu(a)
        for k in KOVA:
            # her kosunun t'si kendi saatinde -> hz'yi kosu basina hesaplayip
            # ortalamak dogru; burada listeleri kosu bazli sakla
            top[k]["n"]+=kv[k]["n"]
            top[k]["roll"].append(kv[k]["roll"])
            top[k]["cx"].append(kv[k]["cx"])
            top[k]["lam"]+=kv[k]["lam"]
    return top

def med(v):
    v=[x for x in v if x is not None]
    return st.median(v) if v else float('nan')

def yaz(baslik, gruplar):
    print("="*104); print(baslik); print("="*104)
    print(f"{'menzil kovasi':>16}{'kol':>6}{'kare':>8}{'kare%':>7}"
          f"{'roll dgs/s':>12}{'|roll| p90':>12}{'cx dgs/s':>10}{'|cx| p90':>10}{'|lam| p90 °/s':>14}")
    for k in KOVA:
        for et,g in gruplar:
            tp=sum(g[q]["n"] for q in KOVA)
            n=g[k]["n"]
            rh=med([hz(s) for s in g[k]["roll"]])
            ch=med([hz(s) for s in g[k]["cx"]])
            rp=med([p90([abs(v) for _,v in s]) for s in g[k]["roll"] if len(s)>5])
            cp=med([p90([abs(v) for _,v in s]) for s in g[k]["cx"] if len(s)>5])
            lp=p90(g[k]["lam"])
            lp=math.degrees(lp) if lp else float('nan')
            ad=f"{k[0]}-{k[1] if k[1]<999 else '+'} m"
            print(f"{ad:>16}{et:>6}{n:>8}{100.0*n/tp if tp else 0:>6.1f}%"
                  f"{rh if rh else float('nan'):>12.3f}{rp if rp else float('nan'):>12.1f}"
                  f"{ch if ch else float('nan'):>10.3f}{cp if cp else float('nan'):>10.0f}{lp:>14.1f}")
        print("-"*104)

kare_K=birlestir(["C01_K_kare","C03_K_kare","C05_K_kare","C07_K_kare"])
kare_C=birlestir(["C02_C_kare","C04_C_kare","C06_C_kare","C08_C_kare"])
duz_K=birlestir(["C09_K_duz_yatay","C11_K_duz_capraz","C15_K_duz_yatay","C17_K_duz_capraz"])
duz_C=birlestir(["C10_C_duz_yatay","C12_C_duz_capraz","C16_C_duz_yatay","C18_C_duz_capraz"])

yaz("KARE — menzil kovasi icinde salinim (jerk 5 vs 15)", [("K5",kare_K),("C15",kare_C)])
yaz("DUZ — menzil kovasi icinde salinim (jerk 5 vs 15)", [("K5",duz_K),("C15",duz_C)])
