#!/usr/bin/env python3
"""takla_analiz.py — G kampanyası ölçütleri (G-D2 alçalma tavanı).

Kullanım: python3 tools/takla_analiz.py logs/kayit/<ad> [logs/kayit/<ad2> ...]

BİRİNCİL ÖLÇÜT: KURTARMA'nın uçuş süresindeki payı (%).
  Kullanıcının şikâyetini birebir ölçer: araç ne kadar süre havada asılı
  kalıyor (bekçi hızı sıfırlamış halde) ve hedef bu sırada kaçıyor.

⚠ GEÇERLİLİK EŞİ (§5.2): KURTARMA payı KÖTÜ bir sebeple de düşer — araç hiç
  kovalamayıp uslu uçarsa da takla atmaz. Bu yüzden yanında ZORUNLU olarak
  medyan d_h ve gerçekleşen yatay hız raporlanır. Kovalama ölmüşse
  "iyileşme" sayılmaz.

⚠ MEKANİZMA SÜTUNU (§5.1): vz_alc_kirp_m. Deney kolunda sıfırsa özellik
  çalışmamıştır → o koşu GEÇERSİZ.
"""
import csv, glob, math, os, sys, collections

def f(r, k, d=0.0):
    try: return float(r[k])
    except Exception: return d

def med(v):
    s = sorted(v)
    return s[len(s) // 2] if s else float('nan')

def kosu_oz(dizin):
    loglar = sorted(glob.glob(os.path.join(dizin, 'gps_guidance_*.csv')),
                    key=os.path.getmtime)
    kare = kurt = olay = 0
    mroll = 0.0
    kirp_kare = 0; kirp_max = 0.0
    dh = []; yatay = []; dz = []
    for p in loglar:
        try: rows = list(csv.DictReader(open(p)))
        except Exception: continue
        if len(rows) < 5: continue
        kare += len(rows)
        kurt += sum(1 for r in rows if r['durum'] == 'KURTARMA')
        olay += sum(1 for i in range(1, len(rows))
                    if rows[i]['durum'] == 'KURTARMA'
                    and rows[i-1]['durum'] != 'KURTARMA')
        mroll = max([mroll] + [abs(f(r, 'iris_roll_deg')) for r in rows])
        for i in range(1, len(rows)):
            r, pr = rows[i], rows[i-1]
            k = f(r, 'vz_alc_kirp_m')
            if k > 0: kirp_kare += 1; kirp_max = max(kirp_max, k)
            if r['durum'] == 'KURTARMA': continue
            dh.append(f(r, 'd_h'))
            dz.append(abs(f(r, 'dz_m')))
            dt = f(r, 't') - f(pr, 't')
            if 0.01 < dt < 0.2:
                vx = (f(r, 'iris_x') - f(pr, 'iris_x')) / dt
                vy = (f(r, 'iris_y') - f(pr, 'iris_y')) / dt
                h = math.hypot(vx, vy)
                if h < 40: yatay.append(h)
    kol = '?'
    kp = os.path.join(dizin, 'KOL.txt')
    if os.path.exists(kp): kol = open(kp).read().split()[0]
    return dict(ad=os.path.basename(dizin), kol=kol, segment=len(loglar),
                kare=kare, kurt_pay=100*kurt/kare if kare else float('nan'),
                olay=olay, mroll=mroll, kirp_kare=kirp_kare, kirp_max=kirp_max,
                dh=med(dh), yatay=med(yatay), dz=med(dz))

def main(argv):
    if not argv:
        print(__doc__); return 1
    ozetler = [kosu_oz(d) for d in argv]
    print(f"{'koşu':<10}{'kol':>4}{'kare':>7}{'KURT%':>7}{'takla':>6}"
          f"{'max|roll|':>10}{'kırp_kare':>10}{'d_h':>7}{'yatay':>7}{'|dz|':>6}")
    print("-" * 74)
    for o in ozetler:
        print(f"{o['ad']:<10}{o['kol']:>4}{o['kare']:>7}{o['kurt_pay']:>6.0f}%"
              f"{o['olay']:>6}{o['mroll']:>9.0f}°{o['kirp_kare']:>10}"
              f"{o['dh']:>7.0f}{o['yatay']:>7.1f}{o['dz']:>6.1f}")
    for kol, ad in (('K', 'KONTROL'), ('D', 'DENEY')):
        g = [o for o in ozetler if o['kol'] == kol]
        if not g: continue
        print(f"\n{ad} (n={len(g)}):  KURTARMA payı medyan "
              f"{med([o['kurt_pay'] for o in g]):.0f}%  ·  takla "
              f"{sum(o['olay'] for o in g)}  ·  d_h medyan "
              f"{med([o['dh'] for o in g]):.0f} m  ·  yatay "
              f"{med([o['yatay'] for o in g]):.1f} m/s  ·  |dz| "
              f"{med([o['dz'] for o in g]):.1f} m")
    return 0

if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
