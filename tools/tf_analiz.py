#!/usr/bin/env python3
"""Kampanya TF analizi — tek faz (terminal fazi kaldirildi) A/B.

Kullanim: python3 tools/tf_analiz.py <kosu_dizini> [...]
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

def _loglar(t0, t1):
    return sorted(p for p in glob.glob('logs/bbox_ibvs_*.csv')
                  if t0-70 <= os.path.getmtime(p) <= t1+70)

def coz(d):
    ad=os.path.basename(d.rstrip('/'))
    oj={}
    if os.path.exists(d+'/olay.json'):
        oj=json.load(open(d+'/olay.json'))
    tel=[x for x in csv.DictReader(open(d+'/telem.csv'))]
    for x in tel:
        for k in ('wall_t','plane_x','plane_y','plane_z','iris_x','iris_y','iris_z'):
            x[k]=F(x[k])
    tel=[x for x in tel if x['plane_x'] is not None and x['iris_x'] is not None]
    t0, t1 = tel[0]['wall_t'], tel[-1]['wall_t']
    for i,x in enumerate(tel):
        x['r']=math.dist((x['plane_x'],x['plane_y'],x['plane_z']),
                         (x['iris_x'],x['iris_y'],x['iris_z']))
        x['hd']=None
        if i>=5:
            p=tel[i-5]; dx=x['plane_x']-p['plane_x']; dy=x['plane_y']-p['plane_y']
            if math.hypot(dx,dy)>2: x['hd']=math.atan2(dy,dx)

    # yaklasmalar: 15 m altindaki her bolumun DIBI
    olc=[]; i=1
    while i < len(tel)-1:
        if tel[i]['r'] < 15:
            j=i; k=i
            while j<len(tel) and tel[j]['r']<15:
                if tel[j]['r']<tel[k]['r']: k=j
                j+=1
            x=tel[k]; hd=x['hd']
            if hd is None:
                for q in range(k, max(0,k-40), -1):
                    if tel[q]['hd'] is not None: hd=tel[q]['hd']; break
            if hd is not None:
                dx=x['iris_x']-x['plane_x']; dy=x['iris_y']-x['plane_y']
                olc.append(dict(
                    r=x['r'], t=x['wall_t']-t0,
                    dikey=-(x['iris_z']-x['plane_z']),      # + = avci YUKARIDA
                    yanal=-dx*math.sin(hd)+dy*math.cos(hd),
                    boyuna=dx*math.cos(hd)+dy*math.sin(hd)))
            i=j
        else: i+=1

    # 20 Hz gudum logu
    LG=_loglar(t0,t1)
    tum=[]; durum={}
    for p in LG:
        for x in csv.DictReader(open(p)):
            tum.append(x); durum[x['durum']]=durum.get(x['durum'],0)+1
    # hucum kareleri = TEK_FAZ ya da TERMINAL
    huc=[x for x in tum if x['durum'] in ('TEK_FAZ','TERMINAL')]
    kutulu=[x for x in tum if x['durum'] in ('TEK_FAZ','TERMINAL','IBVS')]
    cy=[F(x['cy']) for x in kutulu if F(x['cy']) is not None]
    vz=[F(x['vz_cmd']) for x in kutulu if F(x['vz_cmd']) is not None]
    rl=[F(x['iris_roll_deg']) for x in tum if F(x['iris_roll_deg']) is not None]
    pit=[F(x['iris_pitch_deg']) for x in tum if F(x['iris_pitch_deg']) is not None]
    isar=sum(1 for i in range(1,len(vz)) if vz[i]*vz[i-1]<0)

    # SON 3 SANIYE (en yakin andan geriye) — adil salinim penceresi
    t_en = min(tel, key=lambda x: x['r'])['wall_t']
    s_vz=[]; s_rl=[]
    for p in LG:
        rows=list(csv.DictReader(open(p)))
        if not rows: continue
        son=F(rows[-1]['t'])
        if son is None: continue
        ofs=os.path.getmtime(p)-son
        for x in rows:
            t=F(x['t'])
            if t is None or not (t_en-3.0 <= t+ofs <= t_en): continue
            if F(x['vz_cmd']) is not None: s_vz.append(F(x['vz_cmd']))
            if F(x['iris_roll_deg']) is not None: s_rl.append(F(x['iris_roll_deg']))

    return dict(ad=ad, imha=oj.get('imha'), en_yakin=oj.get('en_yakin'),
        olc=olc, durum=durum, log_sayisi=len(LG),
        term_kare=durum.get('TERMINAL',0)+durum.get('TERM_KOR',0),
        kor_kare=durum.get('TERM_KOR',0),
        v_med=med([F(x['v_los']) for x in huc]),
        cy_p90=p90(cy), cy_disi=(sum(1 for c in cy if c>440)/len(cy) if cy else None),
        vz_isaret=(isar/(len(vz)/20) if vz else None),
        vz_p90=p90([abs(v) for v in vz]),
        roll_p90=p90([abs(v) for v in rl]), pitch_p90=p90([abs(v) for v in pit]),
        s_vz_p90=p90([abs(v) for v in s_vz]), s_roll_p90=p90([abs(v) for v in s_rl]),
        )

def yaz(R):
    print(f"\n{'='*74}\n{R['ad']}   imha={R['imha']}   en_yakin={R['en_yakin']} m")
    kapi = ("✓ TEK FAZ (hic TERMINAL yok)" if R['durum'].get('TEK_FAZ')
            and not R['term_kare'] else
            f"KONTROL (TERMINAL+KOR {R['term_kare']} kare)")
    print(f"  MEKANIZMA: {kapi}   |  gudum logu {R['log_sayisi']}  |  durum {R['durum']}")
    print(f"  yaklasma sayisi: {len(R['olc'])}")
    if R['olc']:
        print("      r      DIKEY   YANAL  BOYUNA")
        for o in R['olc']:
            print(f"    {o['r']:6.2f}  {o['dikey']:+7.2f} {o['yanal']:+7.2f} {o['boyuna']:+7.2f}")
        print(f"    -> |dikey| MED {med([abs(o['dikey']) for o in R['olc']]):.2f} m"
              f"   en yakin MED {med([o['r'] for o in R['olc']]):.2f} m")
    print(f"  hucum v_los med {R['v_med']}  |  cy p90 {R['cy_p90']}  kadraj disi %{100*(R['cy_disi'] or 0):.0f}")
    print(f"  salinim: vz isaret {R['vz_isaret']:.2f}/s  |vz| p90 {R['vz_p90']:.2f}"
          f"  |roll| p90 {R['roll_p90']:.1f}°  |pitch| p90 {R['pitch_p90']:.1f}°")
    print(f"  SON 3 s: |vz| p90 {R['s_vz_p90']}  |roll| p90 {R['s_roll_p90']}")

if __name__ == '__main__':
    Rs=[coz(d) for d in sys.argv[1:]]
    for R in Rs: yaz(R)
    K=[R for R in Rs if R['ad'].endswith('_K')]
    T=[R for R in Rs if R['ad'].endswith('_T')]
    if K and T:
        print(f"\n{'='*74}\nKOL OZETI  (KONTROL n={len(K)}  TEK FAZ n={len(T)})")
        print(f"{'olcut':<26}{'KONTROL':>10}{'TEK FAZ':>10}")
        def s(ad, fn, g=None):
            a=[fn(R) for R in K]; b=[fn(R) for R in T]
            a=[x for x in a if x is not None]; b=[x for x in b if x is not None]
            if not a or not b: return
            print(f"{ad:<26}{st.median(a):>10.2f}{st.median(b):>10.2f}")
        s('|dikey| iska (BIRINCIL)', lambda R: med([abs(o['dikey']) for o in R['olc']]))
        s('kosunun en yakini (m)',   lambda R: R['en_yakin'])
        s('kadraj disi orani',       lambda R: R['cy_disi'])
        s('cy p90',                  lambda R: R['cy_p90'])
        s('gudum logu (faz dususu)', lambda R: R['log_sayisi'])
        s('kor hucum karesi',        lambda R: R['kor_kare'])
        s('|pitch| p90',             lambda R: R['pitch_p90'])
        s('vz isaret /s',            lambda R: R['vz_isaret'])
        s('SON 3s |vz| p90',         lambda R: R['s_vz_p90'])
        s('SON 3s |roll| p90',       lambda R: R['s_roll_p90'])
        print(f"{'ISABET':<26}{sum(1 for R in K if R['imha']):>7}/{len(K)}"
              f"{sum(1 for R in T if R['imha']):>9}/{len(T)}")
