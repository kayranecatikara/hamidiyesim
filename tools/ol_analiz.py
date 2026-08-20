#!/usr/bin/env python3
"""Kampanya OL — kutu→menzil ölçüsü (çarpım ↔ köşegen) A/B analizi.

MEKANİZMA KAPISI (§5.1): deney kolunda `boyut` sütunu köşegen değeri
göstermeli (w,h'den yeniden hesaplanıp doğrulanır) ve bağıl menzil hatası
düşmeli. Gerçek menzil YALNIZ ANALİZDE kullanılır (§10).

Kullanım: python3 tools/ol_analiz.py <kacamak_dizini> [...]
"""
import csv, glob, json, math, os, statistics as st, sys


def F(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _loglar(t0, t1):
    return sorted(p for p in glob.glob("logs/bbox_ibvs_*.csv")
                  if t0 - 70 <= os.path.getmtime(p) <= t1 + 70)


def coz(d):
    ad = os.path.basename(d.rstrip("/"))
    oj = json.load(open(d + "/olay.json")) if os.path.exists(d + "/olay.json") else {}

    tel = []
    for x in csv.DictReader(open(d + "/telem.csv")):
        p = (F(x["plane_x"]), F(x["plane_y"]), F(x["plane_z"]))
        i = (F(x["iris_x"]), F(x["iris_y"]), F(x["iris_z"]))
        t = F(x["wall_t"])
        if None in p + i or t is None:
            continue
        tel.append(dict(t=t, p=p, i=i, R=math.dist(p, i),
                        dz=-(i[2] - p[2])))
    if not tel:
        return None
    t0, t1 = tel[0]["t"], tel[-1]["t"]

    # yaklaşmalar: 20 m altındaki her bölümün dibi
    olc, k = [], 1
    while k < len(tel) - 1:
        if tel[k]["R"] < 20:
            j = k; m = k
            while j < len(tel) and tel[j]["R"] < 20:
                if tel[j]["R"] < tel[m]["R"]:
                    m = j
                j += 1
            olc.append(dict(r=tel[m]["R"], dz=tel[m]["dz"]))
            k = j
        else:
            k += 1

    # 20 Hz güdüm logu + menzil hatası
    tum, hata, olcu_ok, olcu_kotu = [], [], 0, 0
    for lp in _loglar(t0, t1):
        rows = list(csv.DictReader(open(lp)))
        if not rows:
            continue
        son = F(rows[-1]["t"])
        if son is None:
            continue
        ofs = os.path.getmtime(lp) - son
        for x in rows:
            tum.append(x)
            tt, w, h, b = F(x["t"]), F(x["w"]), F(x["h"]), F(x["boyut"])
            if None in (tt, w, h, b) or w < 4 or h < 3:
                continue
            # ── MEKANİZMA KAPISI: `boyut` hangi formülle üretilmiş? ──
            car, kos = math.sqrt(w * h), math.hypot(w, h)
            if abs(b - kos) < 0.06:
                olcu_ok += 1          # köşegen
            elif abs(b - car) < 0.06:
                olcu_kotu += 1        # çarpım
            tw = tt + ofs
            if not (t0 <= tw <= t1):
                continue
            n = min(range(len(tel)), key=lambda q: abs(tel[q]["t"] - tw))
            if abs(tel[n]["t"] - tw) > 0.06:
                continue
            hata.append((b, tel[n]["R"]))

    def p90(v):
        v = sorted(v)
        return v[int(.9 * len(v))] if v else None

    bagil = None
    if len(hata) > 50:
        Cf = st.median([b * r for b, r in hata])
        e = sorted(abs(Cf / b - r) / r for b, r in hata)
        bagil = (e[len(e) // 2], e[int(.9 * len(e))], Cf, len(e))

    kutulu = [x for x in tum if x["durum"] == "GORSEL"]
    vz = [F(x["vz_cmd"]) for x in kutulu if F(x["vz_cmd"]) is not None]
    rl = [abs(F(x["iris_roll_deg"])) for x in tum
          if F(x["iris_roll_deg"]) is not None]
    return dict(ad=ad, imha=oj.get("imha"), en_yakin=oj.get("en_yakin"),
                olc=olc, log=len(_loglar(t0, t1)),
                olcu_kosegen=olcu_ok, olcu_carpim=olcu_kotu,
                bagil=bagil, vz_p90=p90([abs(v) for v in vz]),
                roll_p90=p90(rl), kare=len(kutulu))


def yaz(R):
    kos, car = R["olcu_kosegen"], R["olcu_carpim"]
    kol = ("KÖŞEGEN" if kos > car * 3 else
           "çarpım" if car > kos * 3 else f"⚠ KARIŞIK ({kos}/{car})")
    print(f"\n{'='*72}\n{R['ad']}   imha={R['imha']}  en_yakin={R['en_yakin']} m")
    print(f"  MEKANİZMA: log'daki `boyut` sütunu → {kol}"
          f"   (köşegen {kos} kare / çarpım {car} kare)")
    if R["bagil"]:
        p50, p9, Cf, n = R["bagil"]
        print(f"  BAĞIL MENZİL HATASI: p50 %{100*p50:.0f}  p90 %{100*p9:.0f}"
              f"   (kalibre C={Cf:.1f}, n={n})")
    print(f"  yaklaşma {len(R['olc'])} · görsel kare {R['kare']} · log {R['log']}")
    if R["olc"]:
        print("      " + "  ".join(f"{o['r']:.2f}" for o in R["olc"][:10]))
        print(f"    -> en yakın MED {st.median([o['r'] for o in R['olc']]):.2f} m"
              f"   |dikey| MED {st.median([abs(o['dz']) for o in R['olc']]):.2f} m")
    print(f"  |vz| p90 {R['vz_p90']}  |yatış| p90 {R['roll_p90']}")


if __name__ == "__main__":
    Rs = [coz(d) for d in sys.argv[1:]]
    Rs = [r for r in Rs if r]
    for R in Rs:
        yaz(R)
    C = [r for r in Rs if r["ad"].endswith("_C")]
    K = [r for r in Rs if r["ad"].endswith("_K")]
    if C and K:
        print(f"\n{'='*72}\nKOL ÖZETİ  (çarpım n={len(C)}  köşegen n={len(K)})")
        print(f"{'ölçüt':<28}{'ÇARPIM':>11}{'KÖŞEGEN':>11}")
        def s(ad, fn):
            a = [fn(r) for r in C if fn(r) is not None]
            b = [fn(r) for r in K if fn(r) is not None]
            if a and b:
                print(f"{ad:<28}{st.median(a):>11.2f}{st.median(b):>11.2f}")
        s("bağıl menzil hatası p50", lambda r: 100*r["bagil"][0] if r["bagil"] else None)
        s("bağıl menzil hatası p90", lambda r: 100*r["bagil"][1] if r["bagil"] else None)
        s("koşunun en yakını (m)", lambda r: r["en_yakin"])
        s("yaklaşma dibi MED (m)", lambda r: st.median([o["r"] for o in r["olc"]]) if r["olc"] else None)
        s("|dikey| MED (m)", lambda r: st.median([abs(o["dz"]) for o in r["olc"]]) if r["olc"] else None)
        s("|vz| p90", lambda r: r["vz_p90"])
        s("|yatış| p90 (°)", lambda r: r["roll_p90"])
        print(f"{'İSABET':<28}{sum(1 for r in C if r['imha']):>8}/{len(C)}"
              f"{sum(1 for r in K if r['imha']):>10}/{len(K)}")
