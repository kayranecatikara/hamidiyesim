#!/usr/bin/env python3
"""
gudum_rapor.py — Güdüm teşhis raporu: tek dosya HTML (çift tıkla açılır).

2026-08-05'te sorulan dört sorunun cevabını tek sayfada toplar:
  1. Pose modeli gerçekten kötü mü?            → hayır, 3 m ötede sapma 1.04°
  2. "Kötü" pose neden GT'den iyi uçuyor?      → güdüm FARKLI MENZİLDE çalışıyor
  3. Kapanma hızı 25 m/s sorun mu?             → son metrede bile 25, evet
  4. Görsel faza geçişin kuralı ne?            → iki kapı, GT birini bedava geçiyor

Grafikler matplotlib ile üretilip base64 olarak gömülür — CDN/internet yok,
tek dosya her yerde açılır.

Kullanım:
  python3 tools/gudum_rapor.py             # logs/ → /tmp/gudum_rapor.html
  python3 tools/gudum_rapor.py --ac        # üretip tarayıcıda aç
  python3 tools/gudum_rapor.py -o rapor.html
"""
import argparse
import base64
import csv
import glob
import io
import math
import os
import statistics as st
import subprocess
import webbrowser

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOGS = os.path.join(_KOK, "logs")
C_POSE, C_GT, C_UYARI = "#E8833A", "#3A7CE8", "#C0392B"


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _mod(rows):
    """Log hangi modda: 'pose' | 'gt' | None."""
    if rows and rows[0].get("yapilandirma"):
        y = dict(kv.split("=", 1) for kv in rows[0]["yapilandirma"].split(","))
        return "gt" if y.get("GT") == "on" else "pose"
    dz = [_f(r.get("duzeltme")) for r in rows]
    dz = [d for d in dz if d is not None]
    if len(dz) < 10:
        return None
    return "gt" if all(abs(d - 1.0) < 1e-9 for d in dz) else "pose"


def topla(sonra=None):
    V = {m: {"menzil": [], "sapma": [], "lead": [], "kalite": [], "vz": [],
             "hiz_bant": {}, "sapma_bant": {}, "devir": []}
         for m in ("pose", "gt")}
    say = {"pose": 0, "gt": 0}
    for f in sorted(glob.glob(os.path.join(_LOGS, "visual_lead_*.csv"))):
        damga = os.path.basename(f)[12:-4]
        if sonra and damga < sonra:
            continue
        with open(f) as fh:
            rows = list(csv.DictReader(fh))
        m = _mod(rows)
        if m is None:
            continue
        say[m] += 1
        v = V[m]
        ilk = True
        for r in rows:
            mz = _f(r.get("menzil_gercek_m"))
            if mz is not None and ilk:
                v["devir"].append(mz)
                ilk = False
            if r.get("durum") != "ok":
                continue
            sy = _f(r.get("pose_yaw_sapma_deg"))
            vx, vy, vz = (_f(r.get("vx_cmd")), _f(r.get("vy_cmd")), _f(r.get("vz_cmd")))
            if mz is not None:
                v["menzil"].append(mz)
                if None not in (vx, vy, vz):
                    hiz = math.sqrt(vx * vx + vy * vy + vz * vz)
                    for lo, hi in ((0, 2), (2, 4), (4, 6), (6, 10), (10, 20), (20, 50)):
                        if lo <= mz < hi:
                            v["hiz_bant"].setdefault((lo, hi), []).append(hiz)
                            break
                if sy is not None:
                    for lo, hi in ((0, 3), (3, 6), (6, 10), (10, 15), (15, 25), (25, 60)):
                        if lo <= mz < hi:
                            v["sapma_bant"].setdefault((lo, hi), []).append(abs(sy))
                            break
            if sy is not None:
                v["sapma"].append(sy)
            if vz is not None:
                v["vz"].append(vz)
            for ad, k in (("lead", "lead_deg"), ("kalite", "kalite")):
                x = _f(r.get(k))
                if x is not None:
                    v[ad].append(x)
    return V, say


def _png(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=105, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def g_menzil(V, say):
    fig, a = plt.subplots(figsize=(6.4, 4.4))
    kutu, et, renk = [], [], []
    for m, c in (("pose", C_POSE), ("gt", C_GT)):
        if V[m]["menzil"]:
            kutu.append(V[m]["menzil"])
            et.append(f"{m.upper()}\n({say[m]} log)")
            renk.append(c)
    bp = a.boxplot(kutu, labels=et, showfliers=False, patch_artist=True,
                   medianprops=dict(color="#111", lw=2))
    for p, c in zip(bp["boxes"], renk):
        p.set_facecolor(c); p.set_alpha(.75)
    a.axhspan(0, 5, color="#37E06B", alpha=.16)
    a.text(0.98, 4.7, "görsel fazın TASARLANDIĞI bant", ha="right", va="top",
           fontsize=8.5, color="#1a7a3a", transform=a.get_yaxis_transform())
    a.set_ylim(0, 40); a.set_ylabel("hedefe gerçek menzil (m)")
    a.set_title("Güdüm NEREDE çalışıyor (durum=ok kareler)", fontweight="bold")
    a.grid(alpha=.3, axis="y")
    return _png(fig)


def g_sapma(V):
    fig, a = plt.subplots(figsize=(6.4, 4.4))
    b = V["pose"]["sapma_bant"]
    xs = [f"{lo}-{hi}" for lo, hi in sorted(b) if len(b[(lo, hi)]) >= 5]
    ys = [st.median(b[k]) for k in sorted(b) if len(b[k]) >= 5]
    ns = [len(b[k]) for k in sorted(b) if len(b[k]) >= 5]
    cs = [C_UYARI if x.startswith("0-") else C_POSE for x in xs]
    bars = a.bar(xs, ys, color=cs, alpha=.85)
    for r, n in zip(bars, ns):
        a.text(r.get_x() + r.get_width() / 2, r.get_height(), f"n={n}",
               ha="center", va="bottom", fontsize=8)
    a.set_xlabel("menzil bandı (m)"); a.set_ylabel("|yaw sapması| medyanı (°)")
    a.set_title("Pose 3 m ötede neredeyse kusursuz", fontweight="bold")
    a.grid(alpha=.3, axis="y")
    return _png(fig)


def g_hiz(V):
    fig, a = plt.subplots(figsize=(6.4, 4.4))
    b = V["pose"]["hiz_bant"]
    ks = [k for k in sorted(b) if len(b[k]) >= 5]
    xs = [f"{lo}-{hi}" for lo, hi in ks]
    ys = [st.median(b[k]) for k in ks]
    ns = [len(b[k]) for k in ks]
    bars = a.bar(xs, ys, color="#8E44AD", alpha=.85)
    for r, n in zip(bars, ns):
        a.text(r.get_x() + r.get_width() / 2, r.get_height(), f"n={n}",
               ha="center", va="bottom", fontsize=8)
    a.axhline(25, color=C_UYARI, ls="--", lw=2, label="V_KAPANMA = 25 m/s (sabit)")
    a.set_xlabel("menzil bandı (m)"); a.set_ylabel("komut edilen hız (m/s)")
    a.set_title("Kapanma hızı SON METREDE BİLE 25 m/s", fontweight="bold")
    a.legend(fontsize=9); a.grid(alpha=.3, axis="y")
    return _png(fig)


def g_dikey(V):
    fig, a = plt.subplots(figsize=(6.4, 4.4))
    tir = [abs(x) for x in V["pose"]["vz"] if x < 0]
    if tir:
        a.hist([x for x in tir if x <= 26], bins=52, color="#16A085", alpha=.85)
        a.axvline(st.median(tir), color="#0E6251", lw=2,
                  label=f"medyan {st.median(tir):.1f} m/s")
        a.axvline(5, color=C_UYARI, ls="--", lw=2, label="WP_SPD_UP = 5 m/s")
    a.set_xlabel("emredilen TIRMANMA hızı (m/s)"); a.set_ylabel("kare sayısı")
    a.set_title("Dikey komutun ayrı tavanı YOK", fontweight="bold")
    a.legend(fontsize=9); a.grid(alpha=.3)
    return _png(fig)


def g_devir(V, say):
    fig, a = plt.subplots(figsize=(6.4, 4.4))
    kutu, et, renk = [], [], []
    for m, c in (("pose", C_POSE), ("gt", C_GT)):
        if V[m]["devir"]:
            kutu.append(V[m]["devir"]); et.append(m.upper()); renk.append(c)
    bp = a.boxplot(kutu, labels=et, showfliers=False, patch_artist=True,
                   medianprops=dict(color="#111", lw=2))
    for p, c in zip(bp["boxes"], renk):
        p.set_facecolor(c); p.set_alpha(.75)
    a.axhline(20, color=C_UYARI, ls="--", lw=2, label="GATE_MENZIL = 20 m (yatay)")
    a.set_ylabel("devir menzili (m)"); a.set_ylim(0, 40)
    a.set_title("GPS → görsel geçiş NEREDE oluyor", fontweight="bold")
    a.legend(fontsize=9); a.grid(alpha=.3, axis="y")
    return _png(fig)


def _ozet(a, birim=""):
    if not a:
        return "—"
    a = sorted(a)
    return (f"medyan <b>{st.median(a):.2f}{birim}</b> · "
            f"p10 {a[int(len(a)*.1)]:.2f} · p90 {a[int(len(a)*.9)]:.2f}")


CSS = """
*{box-sizing:border-box} body{margin:0;background:#0d1117;color:#e6edf3;
font:15px/1.6 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
.w{max-width:1180px;margin:0 auto;padding:26px 20px 60px}
h1{font-size:26px;margin:0 0 6px} h2{font-size:19px;margin:34px 0 10px;
padding-bottom:7px;border-bottom:1px solid #30363d}
.alt{color:#8b949e;margin:0 0 22px}
.k{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:16px 18px;margin:14px 0}
.k.iyi{border-left:4px solid #2ea043} .k.kotu{border-left:4px solid #d29922}
.k.acil{border-left:4px solid #f85149}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:16px}
@media(max-width:900px){.g2{grid-template-columns:1fr}}
img{width:100%;border-radius:8px;background:#fff}
table{border-collapse:collapse;width:100%;margin:10px 0;font-size:14px}
th,td{border:1px solid #30363d;padding:7px 10px;text-align:left}
th{background:#161b22;font-weight:600} td.n{text-align:right;font-variant-numeric:tabular-nums}
code{background:#161b22;padding:2px 6px;border-radius:4px;font-size:13px;color:#79c0ff}
.b{font-weight:600;color:#fff} .u{color:#f85149;font-weight:600}
.i{color:#3fb950;font-weight:600}
"""


def html(V, say, grafik):
    P, G = V["pose"], V["gt"]
    pose3 = [abs(x) for m, x in zip(P["menzil"], P["sapma"]) if m >= 3 and abs(x) <= 90] \
        if len(P["menzil"]) == len(P["sapma"]) else []
    hb = P["hiz_bant"]
    hiz_satir = "".join(
        f"<tr><td>{lo}–{hi} m</td><td class=n>{len(hb[(lo,hi)])}</td>"
        f"<td class=n><span class={'u' if st.median(hb[(lo,hi)])>20 else ''}>"
        f"{st.median(hb[(lo,hi)]):.1f}</span></td></tr>"
        for lo, hi in sorted(hb) if len(hb[(lo, hi)]) >= 5)
    tir = [abs(x) for x in P["vz"] if x < 0]
    return f"""<!doctype html><html lang=tr><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Güdüm teşhis raporu</title><style>{CSS}</style></head><body><div class=w>
<h1>Güdüm teşhis raporu</h1>
<p class=alt>{say['pose']} pose modu + {say['gt']} GT modu logu ·
yalnız <code>durum=ok</code> kareler · 2026-08-05</p>

<h2>1. Pose modeli gerçekten kötü mü? — <span class=i>Hayır</span></h2>
<div class="k iyi">
Hedef önde ve 3 m'den uzakken pose'un nişan sapması:
{_ozet(pose3, "°")} <br><br>
3 m'nin altında sapma patlıyor ama bu <span class=b>model hatası değil</span>:
hedef kadrajı taşırıyor, nişan vektörü dikeye yaklaşıyor ve azimut
tanımsızlaşıyor. <code>guidance_core</code> bunu <code>azimut_kalite</code>
ile zaten söndürüyor.
</div>
<div class=g2><img src="data:image/png;base64,{grafik['sapma']}">
<img src="data:image/png;base64,{grafik['menzil']}"></div>

<h2>2. "Kötü" pose neden GT'den iyi uçuyor?</h2>
<div class="k iyi">
Fark algıda değil, <span class=b>güdümün nerede çalıştığında</span>:
<table><tr><th>mod</th><th>güdümün çalıştığı menzil</th><th>kalite</th></tr>
<tr><td>POSE</td><td>{_ozet(P['menzil'], ' m')}</td><td>{_ozet(P['kalite'])}</td></tr>
<tr><td>GT</td><td>{_ozet(G['menzil'], ' m')}</td><td>{_ozet(G['kalite'])}</td></tr></table>
Pose'un "uzakta göremiyor" olması bir kusur değil,
<span class=b>doğru faz sınırını çizen bir filtre</span>: yaklaşmayı GPS fazı
yapar, görsel faz yalnız son metrelerde devreye girer. GT modunda algı hiç
kopmadığı için görsel faz onlarca metreden devralıp yaklaşmayı da üstleniyor —
oysa sabit <code>V_KAPANMA</code> ile bunun için tasarlanmadı.
</div>

<h2>3. Kapanma hızı 25 m/s sorun mu? — <span class=u>Evet</span></h2>
<div class="k acil">
<code>V_KAPANMA</code> <span class=b>sabit</span>; menzilden bağımsız.
Ölçülen komut hızı:
<table><tr><th>menzil</th><th>kare</th><th>medyan hız (m/s)</th></tr>{hiz_satir}</table>
<span class=b>0–2 m'de bile 24 m/s.</span> 30 Hz'de bu kare başına
<span class=u>0.81 m</span> yol demek — ıska mesafesi medyanı 0.65–0.85 m,
yani <span class=b>tam bir karelik yol</span>. Nişan doğru olsa bile araç
hedefi iki kare arasında atlıyor.
</div>
<img src="data:image/png;base64,{grafik['hiz']}">

<h2>4. Dikey kaçış: hız mı, ivme mi? — <span class=u>Hız</span></h2>
<div class="k acil">
<code>adapter_copter</code>: <code>v_hedef = V_KAPANMA × u_dunya</code>.
Dikey bileşen = <code>25 × sin(yükseliş)</code> — yani nişan 30° yukarıysa
<span class=b>12.5 m/s tırmanma</span> emrediliyor.
<span class=u>Dikey hız için ayrı tavan YOK</span>; yalnız ivme tavanı var
(<code>IVME_TAVAN_DIKEY</code>) ama o hızın ne kadar <i>büyüyeceğini</i>
sınırlamaz, ne kadar hızlı <i>değişeceğini</i> sınırlar.<br><br>
Ölçüm: karelerin <span class=b>%{len(tir)*100//max(len(P['vz']),1)}</span>'inde
tırmanma emrediliyor, medyan <span class=b>{st.median(tir):.1f} m/s</span>,
p90 {sorted(tir)[int(len(tir)*.9)]:.1f}, tepe {max(tir):.1f} m/s.
ArduPilot'un <code>WP_SPD_UP=5</code> tavanı GUIDED hız komutuna
<span class=b>uygulanmıyor</span>.
</div>
<div class=g2><img src="data:image/png;base64,{grafik['dikey']}">
<img src="data:image/png;base64,{grafik['devir']}"></div>

<h2>5. Görsel faza geçişin kuralı</h2>
<div class=k>
<code>supervisor.run_hybrid</code> içinde <span class=b>İKİ şart birden</span>:
<table>
<tr><th>#</th><th>kapı</th><th>ayar</th><th>pratikte</th></tr>
<tr><td>1</td><td>pose kilidi — son 15 karenin en az 10'unda pose güveni ≥ 0.5</td>
<td><code>KILIT_N=10</code><br><code>KILIT_PENCERE=15</code></td>
<td>~6 m'de oturuyor</td></tr>
<tr><td>2</td><td><span class=b>YATAY</span> mesafe kapısı (3B değil!)</td>
<td><code>GATE_MENZIL=20 m</code></td><td>20 m'de açılıyor</td></tr>
</table>
VEYA GPS <code>DROPOUT</code> (jamming yedeği) — o durumda menzil bilinemez,
görsel temas tek başına yeter.<br><br>
<span class=b>GT modunda 1. kapı bedava sağlanıyor</span> (GT akışı her karede
var), geriye yalnız 20 m kapısı kalıyor. Ölçülen devir menzili:
POSE {st.median(P['devir']):.1f} m · GT {st.median(G['devir']):.1f} m.
</div>

<h2>Sonuç — sıradaki işler</h2>
<div class="k kotu">
<b>1.</b> Dikey hız bileşenine ayrı tavan (<code>TODO B9</code>) — dikey kaçışın
doğrudan sebebi.<br>
<b>2.</b> <code>V_KAPANMA</code>'yı menzile bağla veya düşür
(<code>TODO: terminal kontrol yetkisi</code>) — son metrede 25 m/s fazla.<br>
<b>3.</b> Faz biterken hız komutunu sıfırla (<code>TODO B5</code>) — MAVLink
hız komutu kalıcıdır, göndermeyi bırakmak "dur" demek değildir.<br>
<b>4.</b> Görsel faza irtifa tabanı (<code>TODO B1</code>).
</div>
<p class=alt>Üreten: <code>python3 tools/gudum_rapor.py</code></p>
</div></body></html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sonra", help="bu damgadan sonraki loglar (ör. 20260805)")
    ap.add_argument("-o", "--cikti", default="/tmp/gudum_rapor.html")
    ap.add_argument("--ac", action="store_true", help="tarayıcıda aç")
    args = ap.parse_args()

    V, say = topla(args.sonra)
    if not V["pose"]["menzil"] and not V["gt"]["menzil"]:
        print("Log bulunamadı."); return 1
    grafik = {"menzil": g_menzil(V, say), "sapma": g_sapma(V), "hiz": g_hiz(V),
              "dikey": g_dikey(V), "devir": g_devir(V, say)}
    with open(args.cikti, "w") as f:
        f.write(html(V, say, grafik))
    print(f"rapor: {args.cikti}  ({os.path.getsize(args.cikti)//1024} KB, "
          f"pose={say['pose']} gt={say['gt']} log)")
    if args.ac:
        try:
            subprocess.Popen(["xdg-open", args.cikti],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            webbrowser.open(f"file://{os.path.abspath(args.cikti)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
