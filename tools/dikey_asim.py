#!/usr/bin/env python3
"""DİKEY AŞIM ÖLÇÜMÜ — T1b (DIKEY_ROLL) kıyası için.

Kullanım:  python3 tools/dikey_asim.py logs/kayit/<dizin>

Ne ölçer: her yaklaşmada aracın hedefin NE KADAR ÜSTÜNE çıktığı.
  dz = plane_z − iris_z   (NED, z aşağı pozitif)
    dz < 0  → hedef bizden YUKARIDA  (normal seyir, ~−3 m)
    dz > 0  → BİZ hedefin ÜSTÜNDEYİZ (aşım — istenmeyen)
"""
import csv, math, sys, os, statistics as st

def yukle(d):
    p = os.path.join(d, "kayit.csv")
    if not os.path.exists(p):
        p = os.path.join(d, "meta.csv")
    r = []
    for x in csv.DictReader(open(p)):
        try:
            px, py, pz = (float(x["plane_x"]), float(x["plane_y"]), float(x["plane_z"]))
            ix, iy, iz = (float(x["iris_x"]), float(x["iris_y"]), float(x["iris_z"]))
        except (ValueError, KeyError, TypeError):
            continue
        d3 = math.sqrt((px-ix)**2 + (py-iy)**2 + (pz-iz)**2)
        t = x.get("gecen_s") or x.get("kare_yasi_s") or len(r)
        r.append((float(t), d3, pz - iz))
    return r

def main():
    if len(sys.argv) < 2:
        print(__doc__); raise SystemExit(1)
    r = yukle(sys.argv[1])
    if not r:
        print("⛔ veri okunamadı"); raise SystemExit(2)

    # YAKLAŞMA = mesafe 25 m altına indiği kesintisiz bölüm
    yak, akt = [], None
    for t, d, dz in r:
        if d < 25.0:
            if akt is None: akt = []
            akt.append((t, d, dz))
        elif akt is not None:
            yak.append(akt); akt = None
    if akt: yak.append(akt)

    print(f"kayıt: {sys.argv[1]}   {len(r)} örnek, {len(yak)} yaklaşma (<25 m)\n")
    if not yak:
        print("⚠ hiç yaklaşma yok (mesafe 25 m altına hiç inmedi)"); return

    asimlar, enyakinlar = [], []
    print(f"{'#':>3}{'en yakın':>10}{'o andaki dz':>13}{'SONRAKİ 5 s max dz':>21}{'sonuç':>10}")
    for i, a in enumerate(yak, 1):
        j = min(range(len(a)), key=lambda k: a[k][1])
        t0, dmin, dz0 = a[j]
        son = [x for x in r if t0 <= x[0] <= t0 + 5.0]
        asim = max((x[2] for x in son), default=dz0)
        asimlar.append(asim); enyakinlar.append(dmin)
        print(f"{i:>3}{dmin:>9.2f}m{dz0:>12.2f}m{asim:>19.2f}m"
              f"{('AŞIM' if asim > 1.0 else 'temiz'):>10}")

    n_asim = sum(1 for a in asimlar if a > 1.0)
    print("\n" + "="*62)
    print(f"  BİRİNCİL · dikey aşım medyanı        {st.median(asimlar):+6.2f} m")
    print(f"  aşım yaşanan yaklaşma                 {n_asim}/{len(yak)}")
    print(f"  en yakın menzil medyanı               {st.median(enyakinlar):6.2f} m")
    print("="*62)
    print("\n  KIYAS — DIKEY_ROLL KAPALI iken (kullanıcı uçuşu 20260817_123938):")
    print("    dikey aşım medyanı  +8.37 m   ·   aşım 4/4   ·   en yakın 10.14 m")
    print("\n  AÇIK hâlde beklenen: aşım medyanı 1 m'nin ALTINA inmeli,")
    print("  aşım yaşanan yaklaşma oranı düşmeli, en yakın menzil kötüleşmemeli.")

main()
