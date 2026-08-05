#!/usr/bin/env python3
"""
pose_vs_gt_viz.py — "Kötü pose neden GT'den iyi uçuyor?" sorusunun grafiği.

Paradoks (2026-08-05): güdümün algı girdisi Gazebo'nun GERÇEK pozuna
çevrildiğinde (AVCI_GT_ROT=on) isabet ARTMADI, hatta düştü. Oysa GT hatasız,
pose modeli hatalı. Bu araç iki modun loglarını yan yana koyup nedenini
gösterir.

Ölçülen dört şey:
  1. Pose'un nişan sapması        — pose gerçekten ne kadar hatalı?
  2. Görsel fazın çalıştığı menzil — hangi modda güdüm NEREDE devrede?
  3. Yandanlık → lead üretimi      — hata komuta ne kadar yansıyor?
  4. Sapmanın menzile bağlılığı    — pose nerede güvenilir, nerede değil?

Kaynak sütunlar (visual_lead CSV, bkz. _cevap_anahtari):
  pose_yaw_sapma_deg / pose_elev_sapma_deg  — pose kestirimi − gerçek
  gercek_yaw_deg / gercek_elev_deg          — telemetriden gerçek açılar
  yandanlik_ham, lead_deg, kalite, menzil_gercek_m

Mod ayrımı: yapılandırma damgası varsa GT=on/off; damgasız eski loglarda
`duzeltme` sütunu (GT modunda yükselti düzeltmesi uygulanmaz → daima 1.0).

Kullanım:
  python3 tools/pose_vs_gt_viz.py                  # tüm loglar
  python3 tools/pose_vs_gt_viz.py --sonra 20260805 # bu damgadan sonrakiler
  python3 tools/pose_vs_gt_viz.py -o /tmp/x.png
"""
import argparse
import csv
import glob
import os
import statistics as st

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_LOGS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")

# Grafik paleti — pose sıcak, GT soğuk; ikisi renk körlüğünde de ayrışır.
C_POSE, C_GT = "#E8833A", "#3A7CE8"


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def mod_tespit(rows):
    """Log hangi modda alınmış: 'pose' | 'gt' | None (karar verilemedi)."""
    if rows and rows[0].get("yapilandirma"):
        y = dict(kv.split("=", 1) for kv in rows[0]["yapilandirma"].split(","))
        return "gt" if y.get("GT") == "on" else "pose"
    # Damgasız eski log: GT modunda yükselti düzeltmesi UYGULANMAZ (daima 1.0).
    dz = [_f(r.get("duzeltme")) for r in rows]
    dz = [d for d in dz if d is not None]
    if len(dz) < 10:
        return None
    return "gt" if all(abs(d - 1.0) < 1e-9 for d in dz) else "pose"


def topla(sonra=None):
    """Log dosyalarını moda göre ayırıp ölçüm dizilerini toplar."""
    veri = {m: {"sapma_yaw": [], "sapma_elev": [], "menzil": [], "yandanlik": [],
                "lead": [], "kalite": [], "menzil_sapma": []} for m in ("pose", "gt")}
    sayac = {"pose": 0, "gt": 0, "atlanan": 0}
    for f in sorted(glob.glob(os.path.join(_LOGS, "visual_lead_*.csv"))):
        damga = os.path.basename(f)[12:-4]
        if sonra and damga < sonra:
            continue
        with open(f) as fh:
            rows = list(csv.DictReader(fh))
        m = mod_tespit(rows)
        if m is None:
            sayac["atlanan"] += 1
            continue
        sayac[m] += 1
        v = veri[m]
        for r in rows:
            if r.get("durum") != "ok":          # yalnız güdümün İŞLEDİĞİ kareler
                continue
            mz = _f(r.get("menzil_gercek_m"))
            sy, se = _f(r.get("pose_yaw_sapma_deg")), _f(r.get("pose_elev_sapma_deg"))
            if mz is not None:
                v["menzil"].append(mz)
                if sy is not None:
                    v["menzil_sapma"].append((mz, abs(sy)))
            if sy is not None:
                v["sapma_yaw"].append(sy)
            if se is not None:
                v["sapma_elev"].append(se)
            for ad, k in (("yandanlik", "yandanlik_ham"), ("lead", "lead_deg"),
                          ("kalite", "kalite")):
                x = _f(r.get(k))
                if x is not None:
                    v[ad].append(x)
    return veri, sayac


def _ozet(a):
    if not a:
        return "veri yok"
    a = sorted(a)
    return (f"n={len(a)}  medyan={st.median(a):.2f}  "
            f"p10={a[int(len(a)*.1)]:.2f}  p90={a[int(len(a)*.9)]:.2f}")


def ciz(veri, sayac, cikti):
    fig, ax = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle("Pose modeli vs Gazebo gerçek pozu — neden 'kötü' algı daha iyi uçuyor?",
                 fontsize=15, fontweight="bold")
    P, G = veri["pose"], veri["gt"]

    # ── 1) Pose'un nişan sapması ──
    a = ax[0][0]
    if P["sapma_yaw"]:
        a.hist([x for x in P["sapma_yaw"] if -60 < x < 60], bins=60,
               color=C_POSE, alpha=.85, label="yaw sapması")
        med = st.median(P["sapma_yaw"])
        a.axvline(0, color="#222", lw=1.5, ls="--", label="hatasız (GT)")
        a.axvline(med, color="#B03A00", lw=2, label=f"pose medyan {med:+.1f}°")
    a.set_title("1) Pose GERÇEKTEN hatalı\n(nişan sapması, pose modu uçuşları)",
                fontsize=11, fontweight="bold")
    a.set_xlabel("pose kestirimi − gerçek  (derece)")
    a.set_ylabel("kare sayısı")
    a.legend(fontsize=9)
    a.grid(alpha=.3)

    # ── 2) Görsel fazın çalıştığı menzil — PARADOKSUN CEVABI ──
    a = ax[0][1]
    kutu = [x for x in (P["menzil"], G["menzil"]) if x]
    etiket = [f"POSE\n({sayac['pose']} log)", f"GT\n({sayac['gt']} log)"]
    etiket = [e for e, x in zip(etiket, (P["menzil"], G["menzil"])) if x]
    if kutu:
        bp = a.boxplot(kutu, labels=etiket, showfliers=False, patch_artist=True,
                       medianprops=dict(color="#111", lw=2))
        for patch, c in zip(bp["boxes"], (C_POSE, C_GT)):
            patch.set_facecolor(c)
            patch.set_alpha(.75)
    a.axhspan(0, 5, color="#37E06B", alpha=.16)
    a.text(0.98, 4.6, "görsel fazın TASARLANDIĞI bant (son metreler)",
           ha="right", va="top", fontsize=8.5, color="#1a7a3a",
           transform=a.get_yaxis_transform())
    a.set_ylim(0, 40)
    a.set_title("2) ASIL FARK: güdüm NEREDE çalışıyor\n(yalnız durum=ok kareler)",
                fontsize=11, fontweight="bold")
    a.set_ylabel("hedefe gerçek menzil (m)")
    a.grid(alpha=.3, axis="y")

    # ── 3) Yandanlık → lead ──
    a = ax[1][0]
    for ad, v, c in (("POSE", P, C_POSE), ("GT", G, C_GT)):
        if v["lead"]:
            a.hist([x for x in v["lead"] if x <= 40], bins=45, alpha=.6,
                   color=c, label=f"{ad}  (medyan {st.median(v['lead']):.1f}°)")
    a.set_title("3) Üretilen lead (öne nişan) açısı", fontsize=11, fontweight="bold")
    a.set_xlabel("lead (derece)")
    a.set_ylabel("kare sayısı")
    a.legend(fontsize=9)
    a.grid(alpha=.3)

    # ── 4) Sapma menzile bağlı mı ──
    # Beklentinin AKSİNE: pose 3-25 m'de 1-2° ile neredeyse kusursuz; patlama
    # yalnız son 3 metrede. Orada hedef kadrajı taşırıyor ve nişan vektörü
    # dikeye yaklaşıp azimut TANIMSIZLAŞIYOR (guidance_core azimut_kalite ile
    # bunu zaten söndürüyor). Yani bu bir model hatası değil, geometri tekilliği.
    a = ax[1][1]
    if P["menzil_sapma"]:
        bantlar = [(0, 3), (3, 6), (6, 10), (10, 15), (15, 25), (25, 60)]
        xs, ys, ns, cs = [], [], [], []
        for lo, hi in bantlar:
            s = [d for m, d in P["menzil_sapma"] if lo <= m < hi]
            if len(s) >= 5:
                xs.append(f"{lo}-{hi}")
                ys.append(st.median(s))
                ns.append(len(s))
                cs.append("#C0392B" if hi <= 3 else C_POSE)
        if xs:
            b = a.bar(xs, ys, color=cs, alpha=.85)
            for rect, n in zip(b, ns):
                a.text(rect.get_x() + rect.get_width() / 2, rect.get_height(),
                       f"n={n}", ha="center", va="bottom", fontsize=8)
            a.text(0.5, 0.55, "← azimut TEKİLLİĞİ\n(hedef kadrajı taşırıyor,\n"
                              "model hatası değil)",
                   transform=a.transAxes, fontsize=9, color="#C0392B", va="top")
            a.text(0.5, 0.30, "3 m ötede pose sapması 1-2°\n→ pose ASLINDA İYİ",
                   transform=a.transAxes, fontsize=10, color="#1a7a3a",
                   fontweight="bold", va="top")
    a.set_title("4) Pose 3 m'nin ÖTESİNDE neredeyse kusursuz\n"
                "(patlama yalnız son metrelerde, geometri tekilliği)",
                fontsize=11, fontweight="bold")
    a.set_xlabel("hedefe menzil bandı (m)")
    a.set_ylabel("|yaw sapması| medyanı (derece)")
    a.grid(alpha=.3, axis="y")

    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    fig.savefig(cikti, dpi=110)
    return cikti


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sonra", help="bu damgadan sonraki loglar (ör. 20260805)")
    ap.add_argument("-o", "--cikti", default="/tmp/pose_vs_gt.png")
    args = ap.parse_args()

    veri, sayac = topla(args.sonra)
    print(f"log: pose={sayac['pose']}  gt={sayac['gt']}  atlanan={sayac['atlanan']}")
    for m in ("pose", "gt"):
        v = veri[m]
        if not v["menzil"]:
            continue
        print(f"\n── {m.upper()} modu ──")
        print(f"  güdümün çalıştığı menzil : {_ozet(v['menzil'])}")
        print(f"  yandanlık (ham)          : {_ozet(v['yandanlik'])}")
        print(f"  lead açısı               : {_ozet(v['lead'])}")
        print(f"  kalite                   : {_ozet(v['kalite'])}")
        if v["sapma_yaw"]:
            mutlak = [abs(x) for x in v["sapma_yaw"]]
            print(f"  |pose yaw sapması|       : {_ozet(mutlak)}")
    print("\n→", ciz(veri, sayac, args.cikti))


if __name__ == "__main__":
    raise SystemExit(main())
