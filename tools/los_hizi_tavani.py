import csv,glob,math,os,statistics as st
G=9.81
def f(x,k):
    try:
        v=float(x[k]); return v if v==v else None
    except: return None
def topla(adlar):
    out=[]
    for ad in adlar:
        for y in sorted(glob.glob(f"logs/kacamak/{ad}/bbox_ibvs_*.csv")):
            r=list(csv.DictReader(open(y)))
            if len(r)<5: continue
            for x in r:
                if x.get("durum") in ("KUTU_YOK","TERM_KOR"): continue
                b=f(x,"boyut"); lam=f(x,"los_hiz_az"); v=f(x,"v_los")
                if None in (b,lam,v) or b<=0 or v<1: continue
                out.append((160.0/b, abs(lam), v))
    return out
def rapor(et,d):
    print(f"\n### {et}  (n={len(d)} kare)")
    print(f"{'menzil':>10}{'kare':>7}{'|lam| med':>11}{'|lam| p90':>11}"
          f"{'omega_max':>11}{'asan %':>9}{'gerekli V':>11}")
    for lo,hi in ((0,10),(10,20),(20,40)):
        s=[x for x in d if lo<=x[0]<hi]
        if len(s)<20: continue
        lam=[math.degrees(x[1]) for x in s]
        V=st.median([x[2] for x in s])
        om=math.degrees(G*math.tan(math.radians(45))/V)
        asan=100.0*sum(1 for x in s if math.degrees(x[1])>om)/len(s)
        lm=st.median(lam); lp=sorted(lam)[int(.9*len(lam))]
        # gerekli V: omega_max >= lam_p90  ->  V <= g*tan45/lam_p90
        Vg=G*math.tan(math.radians(45))/math.radians(lp)
        print(f"{f'{lo}-{hi} m':>10}{len(s):>7}{lm:>11.1f}{lp:>11.1f}"
              f"{om:>11.1f}{asan:>8.0f}%{Vg:>10.1f}m/s")
rapor("KARE · jerk 5",  topla(["C01_K_kare","C03_K_kare","C05_K_kare","C07_K_kare"]))
rapor("KARE · jerk 15", topla(["C02_C_kare","C04_C_kare","C06_C_kare","C08_C_kare"]))
rapor("DUZ  · jerk 5",  topla(["C09_K_duz_yatay","C11_K_duz_capraz","C15_K_duz_yatay","C17_K_duz_capraz"]))
rapor("DUZ  · jerk 15", topla(["C10_C_duz_yatay","C12_C_duz_capraz","C16_C_duz_yatay","C18_C_duz_capraz"]))
