#!/usr/bin/env python3
"""Kampanya H analizi — terminal hiz koruma (D2).

Her kosu icin: mekanizma kapisi + birincil olcut + gecerlilik esleri +
ikincil olcutler. Kullanim:  python3 tools/h_analiz.py <kosu_dizini> [...]
"""
import csv, glob, json, math, os, statistics as st, sys

def F(x):
    try: return float(x)
    except: return None
def med(v):
    v=[x for x in v if x is not None]
    return st.median(v) if v else None
def p90(v):
    v=sorted(x for x in v if x is not None)
    return v[int(.9*len(v))] if v else None

def guduem_loglari(t_bas, t_son):
    """Kosu penceresine denk gelen bbox_ibvs loglari."""
    out=[]
    for p in glob.glob('logs/bbox_ibvs_*.csv'):
        m=os.path.getmtime(p)
        if t_bas-70 <= m <= t_son+70: out.append(p)
    return sorted(out)

def coz(d):
    ad=os.path.basename(d.rstrip('/'))
    # --- olay.json: isabet / en yakin ---
    oj={}
    if os.path.exists(d+'/olay.json'):
        oj=json.load(open(d+'/olay.json'))
    # --- 10 Hz telem: gercek konum ---
    tp=d+'/telem.csv'
    tel=[x for x in csv.DictReader(open(tp))] if os.path.exists(tp) else []
    for x in tel:
        for k in ('wall_t','mesafe','plane_x','plane_y','plane_z','iris_x','iris_y','iris_z'):
            x[k]=F(x[k])
    tel=[x for x in tel if x['plane_x'] is not None and x['iris_x'] is not None]
    t0=tel[0]['wall_t'] if tel else 0; t1=tel[-1]['wall_t'] if tel else 0
    # gercek 3B menzil + hedefin ucus yonu
    for i,x in enumerate(tel):
        x['r']=math.dist((x['plane_x'],x['plane_y'],x['plane_z']),
                         (x['iris_x'],x['iris_y'],x['iris_z']))
        if i>=5:
            p=tel[i-5]; dx=x['plane_x']-p['plane_x']; dy=x['plane_y']-p['plane_y']
            x['hd']=math.atan2(dy,dx) if math.hypot(dx,dy)>2 else None
        else: x['hd']=None
    # --- yaklasmalar: yerel minimumlar (< 15 m) ---
    yak=[]
    i=1
    while i<len(tel)-1:
        if tel[i]['r']<15 and tel[i]['r']<=tel[i-1]['r'] and tel[i]['r']<tel[i+1]['r']:
            # bu dipten sonra 8 m'nin uzerine cikana kadar atla
            k=i
            j=i
            while j<len(tel) and tel[j]['r']<15:
                if tel[j]['r']<tel[k]['r']: k=j
                j+=1
            yak.append(k); i=j
        else: i+=1
    olc=[]
    for k in yak:
        x=tel[k]; hd=x['hd']
        if hd is None:
            for q in range(k,max(0,k-30),-1):
                if tel[q]['hd'] is not None: hd=tel[q]['hd']; break
        if hd is None: continue
        dx=x['iris_x']-x['plane_x']; dy=x['iris_y']-x['plane_y']
        olc.append(dict(
            r=x['r'],
            dikey=-(x['iris_z']-x['plane_z']),           # + = avci YUKARIDA
            yanal=-dx*math.sin(hd)+dy*math.cos(hd),
            boyuna=dx*math.cos(hd)+dy*math.sin(hd)))
    # --- 20 Hz gudum logu ---
    LG=guduem_loglari(t0,t1)
    ter=[]; tum=[]; girisler=[]
    for p in LG:
        rows=list(csv.DictReader(open(p)))
        tum+=rows
        ter+=[x for x in rows if x['durum']=='TERMINAL']
        for i in range(1,len(rows)):
            if rows[i]['durum']=='TERMINAL' and rows[i-1]['durum']!='TERMINAL':
                onc=rows[max(0,i-20):i]; son=rows[i:i+20]
                if len(onc)<10 or len(son)<10: continue
                v=[F(x['v_los']) for x in onc if F(x['v_los']) is not None]
                pit=[F(x['iris_pitch_deg']) for x in son if F(x['iris_pitch_deg']) is not None]
                cy=[F(x['cy']) for x in son if F(x['cy']) is not None]
                if v and pit and cy:
                    girisler.append((med(v), max(pit), max(cy)))
    v_ter=[F(x['v_los']) for x in ter if F(x['v_los']) is not None]
    vz=[F(x['vz_cmd']) for x in ter if F(x['vz_cmd']) is not None]
    rl=[F(x['iris_roll_deg']) for x in tum if F(x['iris_roll_deg']) is not None]
    isar=sum(1 for i in range(1,len(vz)) if vz[i]*vz[i-1]<0)
    # --- gorsel temas: kayit dizini (0.5 s) ---
    kt=d+'/meta.csv'
    tem={}
    if os.path.exists(kt):
        r=[x for x in csv.DictReader(open(kt))]
    # 0-5 m temas: 20 Hz logdan (kutu var = satir var) yerine telem+ter esle
    # basitce: terminal karelerinde conf olan oran
    cf=[F(x['conf']) for x in ter]
    return dict(ad=ad, imha=oj.get('imha'), en_yakin_olay=oj.get('en_yakin'),
        n_yak=len(olc), olc=olc, girisler=girisler,
        v_ter=med(v_ter), v_ter_n=len(v_ter),
        vz_isaret=isar/(len(vz)/20) if vz else None, vz_p90=p90([abs(v) for v in vz]),
        roll_p90=p90([abs(v) for v in rl]), ter_kare=len(ter), tum_kare=len(tum),
        conf_med=med(cf))

def yaz(R):
    print(f"\n{'='*72}\n{R['ad']}   imha={R['imha']}  olay_en_yakin={R['en_yakin_olay']}")
    kap = "✓ GECERLI" if (R['v_ter'] is not None and abs(R['v_ter']-16.0)>0.05) else "⚠ v_los=16 (kontrol kolu davranisi)"
    print(f"  MEKANIZMA KAPISI: terminal v_los medyan = {R['v_ter']}  ({R['ter_kare']} kare)  -> {kap}")
    print(f"  yaklasma sayisi: {R['n_yak']}")
    if R['olc']:
        print("      r      DIKEY   YANAL  BOYUNA")
        for o in R['olc']:
            print(f"    {o['r']:6.2f}  {o['dikey']:+7.2f} {o['yanal']:+7.2f} {o['boyuna']:+7.2f}")
        print(f"    -> |dikey| MEDYAN = {med([abs(o['dikey']) for o in R['olc']]):.2f} m"
              f"   en yakin MEDYAN = {med([o['r'] for o in R['olc']]):.2f} m")
    if R['girisler']:
        print(f"  terminal girisi (n={len(R['girisler'])}):"
              f" v_once med {med([g[0] for g in R['girisler']]):.2f}"
              f" | pitch tepe med {med([g[1] for g in R['girisler']]):.1f}°"
              f" | cy tepe med {med([g[2] for g in R['girisler']]):.0f}"
              f" | kadraj disi {sum(1 for g in R['girisler'] if g[2]>440)}/{len(R['girisler'])}")
    print(f"  salinim: vz isaret {R['vz_isaret']:.2f}/s  |vz| p90 {R['vz_p90']:.2f}"
          f"  |roll| p90 {R['roll_p90']:.1f}°   terminal conf med {R['conf_med']}")

if __name__ == '__main__':
    Rs=[coz(d) for d in sys.argv[1:]]
    for R in Rs: yaz(R)
    # kol ozeti
    K=[R for R in Rs if '_K_' in R['ad']]; D=[R for R in Rs if '_D_' in R['ad']]
    if K and D:
        print(f"\n{'='*72}\nKOL OZETI  (KONTROL n={len(K)}  DENEY n={len(D)})")
        def kol(g,ad):
            dv=[abs(o['dikey']) for R in g for o in R['olc']]
            rr=[o['r'] for R in g for o in R['olc']]
            gi=[x for R in g for x in R['girisler']]
            print(f"  {ad:8} |dikey| med {med(dv) if dv else '-':>6}  en yakin med {med(rr) if rr else '-':>6}"
                  f"  pitch tepe med {med([x[1] for x in gi]) if gi else '-':>6}"
                  f"  kadraj disi {sum(1 for x in gi if x[2]>440)}/{len(gi)}"
                  f"  vz isaret med {med([R['vz_isaret'] for R in g]):.2f}"
                  f"  isabet {sum(1 for R in g if R['imha'])}/{len(g)}")
        kol(K,'KONTROL'); kol(D,'DENEY')
