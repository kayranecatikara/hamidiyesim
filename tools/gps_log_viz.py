#!/usr/bin/env python3
"""
gps_log_viz.py — GPS güdüm CSV loglarını tek-dosya interaktif HTML panele çevirir.

Panelde her uçuş için:
  - Özet kartları (en yakın menzil, KILIT %, hız farkı, komut doygunluğu, telemetri Hz)
  - Kuşbakışı yörünge (drone vs hedef) ve kamera nişangâhı (hedefin u,v izi)
  - HIZ grafiği: drone gerçek / hedef / komut — dönüş fazları gölgeli (ana teşhis)
  - Menzil d_h, kadraj açıları, araç eğimi (roll/pitch), irtifa profili
  - DÜZ vs DÖNÜŞ faz tablosu (manevrada ne kaybediliyor)
  - Tüm uçuşların karşılaştırma tablosu + veriden türetilen otomatik yorum

Çıktı tamamen kendine yeten tek HTML — internet/CDN gerektirmez.

Kullanım:
  python3 tools/gps_log_viz.py                     # en yeni 8 log
  python3 tools/gps_log_viz.py --last 20           # en yeni 20 log
  python3 tools/gps_log_viz.py logs/a.csv logs/b.csv
  python3 tools/gps_log_viz.py --last 4 -o rapor.html --open

Log formatı ve kolon anlamları: docs/GPS_LOGGING.md
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics as st
import sys
import webbrowser

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOG_DIR = os.path.join(_ROOT, "logs")
_VARSAYILAN_CIKTI = os.path.join(_LOG_DIR, "gps_log_panel.html")
_HEDEF_NOKTA = 700    # uçuş başına HTML'e gömülecek yaklaşık nokta (downsample)
_MIN_KARE = 20        # bundan kısa loglar (anlık başlat/durdur) panele alınmaz

# Faz eşikleri — hedefin dönüş hızı (°/s), yörünge fazını ayırmak için
_DONUS_ESIK = 15.0
_DUZ_ESIK = 8.0
# Drone hızında ışınlanma/EKF sıçramalarını ele (m/s) — gerçek araç bunu aşamaz
_HIZ_TAVAN = 60.0


def _fnum(satir, alan):
    try:
        return float(satir.get(alan, ""))
    except (TypeError, ValueError):
        return None


def _med(dizi):
    temiz = [v for v in dizi if v is not None]
    return st.median(temiz) if temiz else None


def _r(v, n=1):
    return None if v is None else round(v, n)


def _tazelik_hz(satirlar, alan, t):
    """Bir telemetri alanının GERÇEK tazeleme frekansı (değer kaç saniyede bir değişiyor)."""
    araliklar = []
    son_deger = None
    son_t = None
    for i, r in enumerate(satirlar):
        v = r.get(alan)
        if v != son_deger:
            if son_t is not None:
                araliklar.append(t[i] - son_t)
            son_t = t[i]
            son_deger = v
    m = _med(araliklar)
    return (1.0 / m) if m and m > 1e-6 else None


def _medyan_filtre(dizi, pencere=7):
    """Kayan medyan — örnekleme artefaktını (testere deseni) eler, kenarı bozmaz.

    Neden gerekli: telemetri ~25 Hz gelirken log 20 Hz yazıyor; bazı karelerde iki
    örnek, bazılarında sıfır düşüyor → konum türevi kare-kare zıplıyor. Medyan
    (ortalama değil) sıçramaları atarken gerçek hız seviyesini kaydırmaz.
    """
    n = len(dizi)
    y = pencere // 2
    out = []
    for i in range(n):
        pen = [v for v in dizi[max(0, i - y):min(n, i + y + 1)] if v is not None]
        out.append(st.median(pen) if pen else None)
    return out


def _drone_hizi(satirlar, t):
    """Drone'un GERÇEK yer hızı — iris konumunun türevi (yalnız taze örnekler arası).

    Komut hızıyla karıştırma: komut 20 m/s olsa da araç bunu uygulayamıyor olabilir;
    bu fonksiyon aracın fiilen ne yaptığını verir. EKF sıçramaları (_HIZ_TAVAN) elenir.
    Ara karelerde son geçerli değer taşınır (zero-order hold) — grafik sürekli olsun.
    """
    n = len(satirlar)
    hiz = [None] * n
    onceki_i = None
    onceki_xy = None
    for i, r in enumerate(satirlar):
        xy = (r.get("iris_x"), r.get("iris_y"))
        if xy != onceki_xy:
            if onceki_i is not None:
                dt = t[i] - t[onceki_i]
                x1, y1 = _fnum(r, "iris_x"), _fnum(r, "iris_y")
                x0 = _fnum(satirlar[onceki_i], "iris_x")
                y0 = _fnum(satirlar[onceki_i], "iris_y")
                if 1e-3 < dt < 1.0 and None not in (x0, y0, x1, y1):
                    v = math.hypot(x1 - x0, y1 - y0) / dt
                    if v < _HIZ_TAVAN:
                        hiz[i] = v
            onceki_i, onceki_xy = i, xy
    son = None
    for i in range(n):
        if hiz[i] is None:
            hiz[i] = son
        else:
            son = hiz[i]
    return _medyan_filtre(hiz)


def _donus_hizi(satirlar, t):
    """Hedefin dönüş hızı (°/s) — hız vektörü yönünün değişimi. Faz ayrımı için."""
    n = len(satirlar)
    oran = [None] * n
    onceki_h = None
    onceki_t = None
    yum = None
    for i, r in enumerate(satirlar):
        vx, vy = _fnum(r, "tgt_vx"), _fnum(r, "tgt_vy")
        if vx is None or vy is None or math.hypot(vx, vy) < 1.0:
            oran[i] = yum
            continue
        h = math.degrees(math.atan2(vy, vx))
        if onceki_h is not None and t[i] > onceki_t:
            d = (h - onceki_h + 180) % 360 - 180
            ham = abs(d) / (t[i] - onceki_t)
            yum = ham if yum is None else 0.15 * ham + 0.85 * yum   # EMA (gürültü süz)
        onceki_h, onceki_t = h, t[i]
        oran[i] = yum
    return oran


def _faz(oran):
    """0 = düz uçuş, 1 = dönüş (manevra), 2 = geçiş/bilinmiyor."""
    out = []
    for v in oran:
        if v is None:
            out.append(2)
        elif v >= _DONUS_ESIK:
            out.append(1)
        elif v < _DUZ_ESIK:
            out.append(0)
        else:
            out.append(2)
    return out


def _faz_ozet(idx, drone, hedef, komut, roll, dh):
    if len(idx) < 5:
        return None
    return {
        "n": len(idx),
        "drone": _r(_med([drone[i] for i in idx])),
        "hedef": _r(_med([hedef[i] for i in idx])),
        "komut": _r(_med([komut[i] for i in idx])),
        "roll": _r(_med([roll[i] for i in idx])),
        "dh": _r(_med([dh[i] for i in idx])),
    }


def _log_yukle(yol):
    """Tek CSV'yi oku → türetilmiş seriler + özet istatistik + downsample'lı noktalar."""
    with open(yol) as f:
        satirlar = list(csv.DictReader(f))
    if len(satirlar) < _MIN_KARE:
        return None

    n = len(satirlar)
    t = [(_fnum(r, "t") or 0.0) for r in satirlar]
    t0 = t[0]
    sure = t[-1] - t0

    drone_h = _drone_hizi(satirlar, t)
    hedef_h = [None if (_fnum(r, "tgt_vx") is None) else
               math.hypot(_fnum(r, "tgt_vx"), _fnum(r, "tgt_vy")) for r in satirlar]
    komut_h = [None if (_fnum(r, "vx_cmd") is None) else
               math.hypot(_fnum(r, "vx_cmd"), _fnum(r, "vy_cmd")) for r in satirlar]
    roll = [abs(_fnum(r, "iris_roll_deg") or 0.0) for r in satirlar]
    pitch = [abs(_fnum(r, "iris_pitch_deg") or 0.0) for r in satirlar]
    dh = [_fnum(r, "d_h") for r in satirlar]
    faz = _faz(_donus_hizi(satirlar, t))

    # --- özet istatistikler (TAM veriden; downsample'dan değil) ---
    dh_temiz = [v for v in dh if v is not None]
    kilit = sum(1 for r in satirlar if r.get("durum") == "KILIT")
    komut_temiz = [v for v in komut_h if v is not None]
    v_tavan = max(komut_temiz) if komut_temiz else 0.0
    doygun = (sum(1 for v in komut_temiz if v > v_tavan * 0.98) / len(komut_temiz) * 100
              if komut_temiz and v_tavan > 1 else 0.0)
    d_med, h_med = _med(drone_h), _med(hedef_h)

    ozet = {
        "sure": round(sure, 1),
        "kare": n,
        "telem_hz": _r(_tazelik_hz(satirlar, "tgt_x", t)),
        "iris_hz": _r(_tazelik_hz(satirlar, "iris_x", t)),
        "dh_min": _r(min(dh_temiz)) if dh_temiz else None,
        "dh_med": _r(_med(dh_temiz)),
        "kilit_pct": round(kilit / n * 100, 1),
        "kilit_kare": kilit,
        "drone_med": _r(d_med),
        "hedef_med": _r(h_med),
        "komut_med": _r(_med(komut_h)),
        "hiz_fark": _r((d_med - h_med) if (d_med is not None and h_med is not None) else None),
        "v_tavan": _r(v_tavan),
        "doygun": round(doygun, 0),
        "roll_med": _r(_med(roll)),
        "roll_max": _r(max(roll) if roll else None),
        "duz": _faz_ozet([i for i in range(n) if faz[i] == 0], drone_h, hedef_h, komut_h, roll, dh),
        "donus": _faz_ozet([i for i in range(n) if faz[i] == 1], drone_h, hedef_h, komut_h, roll, dh),
    }

    # --- downsample → panel noktaları ---
    adim = max(1, n // _HEDEF_NOKTA)
    pts = []
    for i in range(0, n, adim):
        r = satirlar[i]
        iz, tz = _fnum(r, "iris_z"), _fnum(r, "tgt_z")
        pts.append({
            "t": round(t[i] - t0, 2),
            "dh": _r(dh[i]),
            "durum": r.get("durum"),
            "yaw": _r(_fnum(r, "kadraj_yaw_deg")),
            "elev": _r(_fnum(r, "kadraj_elev_deg")),
            "u": _r(_fnum(r, "u_px"), 0), "v": _r(_fnum(r, "v_px"), 0),
            "ix": _r(_fnum(r, "iris_x")), "iy": _r(_fnum(r, "iris_y")),
            "tx": _r(_fnum(r, "tgt_x")), "ty": _r(_fnum(r, "tgt_y")),
            "ds": _r(drone_h[i]), "ts": _r(hedef_h[i]), "cs": _r(komut_h[i]),
            "rl": _r(roll[i]), "pt": _r(pitch[i]),
            "az": _r(-iz) if iz is not None else None,
            "tz": _r(-tz) if tz is not None else None,
            "ph": faz[i],
        })

    damga = os.path.basename(yol).replace("gps_guidance_", "").replace(".csv", "")
    guzel = damga
    if len(damga) == 15 and "_" in damga:          # YYYYMMDD_HHMMSS
        g, s = damga.split("_")
        guzel = f"{g[6:8]}.{g[4:6]} {s[0:2]}:{s[2:4]}:{s[4:6]}"
    return {"stamp": damga, "etiket": guzel, "dosya": os.path.basename(yol),
            "n_orig": n, "ozet": ozet, "pts": pts}


def _loglari_sec(logs, last):
    if logs:
        return logs
    hepsi = sorted(glob.glob(os.path.join(_LOG_DIR, "gps_guidance_*.csv")),
                   key=os.path.getmtime, reverse=True)
    return list(reversed(hepsi[:last]))            # eskiden yeniye


def panel_uret(logs=None, last=8, out=_VARSAYILAN_CIKTI, sessiz=False):
    """Logları HTML panele çevirir; yazılan dosya yolunu döndürür (yoksa None).

    Uçuş sonunda otomatik çağrılabilsin diye import edilebilir tutuldu
    (bkz. control/guidance/gps_guidance.py — döngü biterken panel tazelenir).
    """
    yollar = _loglari_sec(logs, last)
    ucuslar = []
    for y in yollar:
        try:
            u = _log_yukle(y)
        except Exception as e:                      # bozuk/yarım log paneli engellemesin
            if not sessiz:
                print(f"  atlandı: {os.path.basename(y)} ({e})")
            continue
        if u:
            ucuslar.append(u)
            if not sessiz:
                print(f"  yüklendi: {u['dosya']}  ({u['n_orig']} kare → {len(u['pts'])} nokta)")
    if not ucuslar:
        return None
    ucuslar.reverse()                               # en yeni uçuş ilk sırada/seçili
    html = _SABLON.replace("__DATA__", json.dumps(ucuslar, separators=(",", ":")))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as f:
        f.write(html)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="GPS güdüm loglarını HTML panele çevirir.")
    ap.add_argument("logs", nargs="*", help="CSV log dosyaları (boşsa en yeni --last kadarı)")
    ap.add_argument("--last", type=int, default=8, help="dosya verilmezse en yeni kaç log (varsayılan 8)")
    ap.add_argument("-o", "--out", default=_VARSAYILAN_CIKTI, help="çıktı HTML yolu")
    ap.add_argument("--open", action="store_true", help="oluşturunca tarayıcıda aç")
    args = ap.parse_args(argv)

    yol = panel_uret(args.logs, args.last, args.out)
    if not yol:
        print("Panele alınacak geçerli log bulunamadı.", file=sys.stderr)
        return 1
    kb = os.path.getsize(yol) // 1024
    print(f"\nPanel yazıldı: {yol}  ({kb} KB)")
    print(f"  tarayıcı: file://{os.path.abspath(yol)}")
    print(f"  GCS açıkken: http://localhost:8000/loglar/{os.path.basename(yol)}")
    if args.open:
        webbrowser.open("file://" + os.path.abspath(yol))
    return 0


_SABLON = r'''<!doctype html><html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GPS Güdüm — Uçuş Log Paneli</title>
<style>
:root{
  /* seri renkleri — kategorik, CVD doğrulanmış (dataviz validator: tüm kontroller geçti) */
  --s1:#1fa894;   /* drone  */
  --s2:#c67d28;   /* hedef  */
  --s3:#7a6ed6;   /* komut  */
  --bg:#0e1417;--panel:#151d21;--panel2:#1b262b;--line:#25343a;--ink:#dfe8e6;--ink-dim:#8fa39f;
  --accent:#35e0c9;--good:#4ade80;--warn:#fbbf24;--bad:#f87171;--grid:rgba(120,150,150,.13);
  --band:rgba(198,125,40,.13);
}
@media (prefers-color-scheme: light){:root{
  --s1:#0f9e8c;--s2:#c9741a;--s3:#6355d0;
  --bg:#eef2f1;--panel:#fff;--panel2:#f6f9f8;--line:#d3ddda;--ink:#17241f;--ink-dim:#5c6d68;
  --accent:#0b7d6e;--good:#15803d;--warn:#a16207;--bad:#b91c1c;--grid:rgba(20,60,55,.10);
  --band:rgba(201,116,26,.11);}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font-family:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;line-height:1.5;-webkit-font-smoothing:antialiased}
.mono{font-family:ui-monospace,"SF Mono",Menlo,Consolas,monospace;font-variant-numeric:tabular-nums}
.wrap{max-width:1240px;margin:0 auto;padding:26px 20px 64px}
h1{font-size:21px;margin:0 0 4px;text-wrap:balance}
.sub{color:var(--ink-dim);font-size:13px;max-width:78ch}
.eyebrow{font-size:11px;letter-spacing:.16em;text-transform:uppercase;color:var(--accent);font-weight:600;margin-bottom:9px}
.flights{display:flex;flex-wrap:wrap;gap:8px;margin:18px 0 20px}
.fbtn{background:var(--panel);border:1px solid var(--line);color:var(--ink-dim);padding:7px 12px;border-radius:8px;cursor:pointer;font-size:12.5px;display:flex;flex-direction:column;gap:1px;text-align:left}
.fbtn:hover{border-color:var(--accent);color:var(--ink)}
.fbtn.on{background:var(--panel2);border-color:var(--accent);color:var(--ink)}
.fbtn b{font-size:12px;font-weight:600}.fbtn span{font-size:10.5px}
.fbtn:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
.kpis{display:grid;grid-template-columns:repeat(5,1fr);gap:11px;margin-bottom:18px}
.kpi{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:13px 14px;position:relative;overflow:hidden}
.kpi .lab{font-size:10.5px;letter-spacing:.07em;text-transform:uppercase;color:var(--ink-dim)}
.kpi .val{font-size:23px;font-weight:600;margin-top:5px;font-variant-numeric:tabular-nums}
.kpi .note{font-size:11px;color:var(--ink-dim);margin-top:2px}
.kpi .stripe{position:absolute;left:0;top:0;bottom:0;width:3px;background:var(--ink-dim)}
.kpi.good .stripe{background:var(--good)}.kpi.warn .stripe{background:var(--warn)}.kpi.bad .stripe{background:var(--bad)}
.kpi .val.good{color:var(--good)}.kpi .val.warn{color:var(--warn)}.kpi .val.bad{color:var(--bad)}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:16px;align-items:start;margin-bottom:16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:15px 15px 11px;position:relative}
.card h2{font-size:13px;margin:0 0 3px}.card .cap{font-size:11.5px;color:var(--ink-dim);margin:0 0 11px}
canvas{width:100%;display:block}
.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:11.5px;color:var(--ink-dim);margin-top:9px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;vertical-align:-1px}
.legend i.dash{width:13px;height:0;border-top:2px dashed currentColor;border-radius:0}
.charts{display:flex;flex-direction:column;gap:14px}
.verdict{background:var(--panel2);border:1px solid var(--line);border-left:3px solid var(--accent);border-radius:8px;padding:13px 16px;font-size:13px;margin-top:14px}
.verdict b{color:var(--accent)}
table{width:100%;border-collapse:collapse;font-size:12px}
th,td{text-align:right;padding:6px 8px;border-bottom:1px solid var(--line);font-variant-numeric:tabular-nums;white-space:nowrap}
th{color:var(--ink-dim);font-weight:600;font-size:10.5px;letter-spacing:.05em;text-transform:uppercase}
td:first-child,th:first-child{text-align:left}
tbody tr{cursor:pointer}
tbody tr:hover{background:var(--panel2)}
tbody tr.on{background:var(--panel2);box-shadow:inset 3px 0 0 var(--accent)}
.tbl-wrap{overflow-x:auto}
.pos{color:var(--good)}.neg{color:var(--bad)}
.tip{position:absolute;pointer-events:none;background:var(--panel2);border:1px solid var(--line);
  border-radius:7px;padding:7px 9px;font-size:11.5px;opacity:0;transition:opacity .09s;z-index:9;
  box-shadow:0 4px 14px rgba(0,0,0,.28);white-space:nowrap}
.tip b{font-weight:600}
.tip .row{display:flex;gap:7px;align-items:center;justify-content:space-between}
.tip i{display:inline-block;width:8px;height:8px;border-radius:2px;margin-right:4px}
@media(max-width:980px){.kpis{grid-template-columns:repeat(3,1fr)}}
@media(max-width:860px){.grid2{grid-template-columns:1fr}.kpis{grid-template-columns:repeat(2,1fr)}}
</style></head><body>
<div class="wrap">
<div class="eyebrow">Avcı İHA · GPS Güdüm Telemetrisi</div>
<h1>Uçuş Log Paneli</h1>
<p class="sub">Her uçuşta drone'un hedefi nasıl kovaladığı. <b>Hız grafiği</b> ana teşhis aracıdır:
drone gerçek hızı hedefin altına düşüyorsa yetişemez. Dönüş (manevra) fazları grafiklerde
gölgeli banttır — kayıp genelde orada olur. Kadraj hedefi: yaw 0°, elev 25°, d_h ≈ 11 m.</p>
<div class="flights" id="flights"></div>
<div class="kpis" id="kpis"></div>

<div class="charts">
  <div class="card"><h2>Hız — drone gerçek vs hedef vs komut</h2>
    <p class="cap">Gölgeli bant = hedefin dönüş (manevra) fazı. Drone çizgisi hedefin <em>altındaysa</em>
    açı kapanmaz. Komut ile gerçek arasındaki fark = aracın uygulayamadığı istek (fizik limiti).</p>
    <canvas id="hiz"></canvas>
    <div class="legend"><span><i style="background:var(--s1)"></i>drone gerçek</span>
      <span><i style="background:var(--s2)"></i>hedef</span>
      <span><i style="background:var(--s3)"></i>komut</span>
      <span><i style="background:var(--band)"></i>dönüş fazı</span></div></div>
</div>

<div class="grid2" style="margin-top:16px">
  <div class="card"><h2>Kuşbakışı yörünge · N-E</h2>
    <p class="cap">● başlangıç, ▲ son. Eş ölçek. Drone hedeften <em>büyük</em> yay çiziyorsa dışarıda orbit atıyordur.</p>
    <canvas id="traj"></canvas>
    <div class="legend"><span><i style="background:var(--s2)"></i>hedef</span><span><i style="background:var(--s1)"></i>drone</span></div>
  </div>
  <div class="card"><h2>Kamera nişangâhı · 640×480</h2>
    <p class="cap">Hedefin (u,v) izi — soluktan parlağa = zaman. Artı = merkez (320,240).</p>
    <canvas id="reticle"></canvas>
    <div class="legend"><span><i style="background:var(--good)"></i>KILIT</span><span><i style="background:var(--warn)"></i>ARAMA</span><span><i style="background:var(--bad)"></i>kadraj dışı</span></div>
  </div>
</div>

<div class="charts">
  <div class="card"><h2>Menzil d_h — hedefe yatay mesafe</h2>
    <p class="cap">Yeşil kesikli = kadraj istasyonu 11 m. Yaklaşınca iner; sabit yüksekte asılı kalması "yetişemiyor" demektir.</p>
    <canvas id="range"></canvas>
    <div class="legend"><span><i style="background:var(--s1)"></i>d_h</span><span style="color:var(--good)"><i class="dash"></i>istasyon 11 m</span></div></div>
  <div class="card"><h2>Kadraj açıları — elev (dikey) &amp; yaw (yatay)</h2>
    <p class="cap">Hedef: elev 25° (kamera tilt'i), yaw 0°. İkisi de hedefine oturduğunda hedef kare merkezindedir.</p>
    <canvas id="angles"></canvas>
    <div class="legend"><span><i style="background:var(--s1)"></i>elev</span><span><i style="background:var(--s2)"></i>yaw</span></div></div>
  <div class="card"><h2>Araç eğimi — |roll| &amp; |pitch|</h2>
    <p class="cap">Eğim = yatay ivme (a ≈ g·tan θ). Dönüşte düşük kalıyorsa araç yeterince yaslanamıyordur (itki rezervi yok).</p>
    <canvas id="tilt"></canvas>
    <div class="legend"><span><i style="background:var(--s1)"></i>|roll|</span><span><i style="background:var(--s2)"></i>|pitch|</span></div></div>
  <div class="card"><h2>İrtifa profili</h2>
    <p class="cap">Yerden yükseklik. Drone istasyonu hedefin ~4.6 m altındadır (kamera 25° yukarı baksın diye).</p>
    <canvas id="alt"></canvas>
    <div class="legend"><span><i style="background:var(--s1)"></i>drone</span><span><i style="background:var(--s2)"></i>hedef</span></div></div>
</div>

<div class="card" style="margin-top:16px"><h2>Faz kırılımı — düz uçuş vs dönüş</h2>
  <p class="cap">Aynı uçuşun iki rejimi. Manevrada drone hızı hedefin altına düşüyorsa sorun dönüş kabiliyetindedir.</p>
  <div class="tbl-wrap"><table id="phase"></table></div></div>

<div class="card" style="margin-top:16px"><h2>Tüm uçuşlar</h2>
  <p class="cap">Satıra tıkla → o uçuşu yukarıda aç. Δhız = drone − hedef (negatif ise yetişemez).</p>
  <div class="tbl-wrap"><table id="all"></table></div></div>

<div class="verdict" id="verdict"></div>
</div>
<script>
const DATA=__DATA__;
const css=k=>getComputedStyle(document.documentElement).getPropertyValue(k).trim();
const f1=v=>(v==null||isNaN(v))?'—':(+v).toFixed(1);
const f0=v=>(v==null||isNaN(v))?'—':Math.round(v);
let cur=0;

/* ---------- kadraj istatistiği (yakın rejim) ---------- */
function stats(f){
  const near=f.pts.filter(p=>p.dh!=null&&p.dh<20&&p.durum==='KILIT'&&p.yaw!=null);
  const base=near.length?near:f.pts.filter(p=>p.yaw!=null);
  const avg=(a,g)=>{const v=a.map(g).filter(x=>x!=null&&!isNaN(x));return v.length?v.reduce((x,y)=>x+y,0)/v.length:NaN;};
  return{yaw:avg(base,p=>Math.abs(p.yaw)),elev:avg(base,p=>p.elev),
         u:avg(base,p=>p.u),v:avg(base,p=>p.v),n:base.length};
}
const cls=(v,g,w)=>isNaN(v)||v==null?'':(v<=g?'good':v<=w?'warn':'bad');

/* ---------- uçuş seçici ---------- */
function renderFlights(){const el=document.getElementById('flights');el.innerHTML='';
  DATA.forEach((f,i)=>{const b=document.createElement('button');b.className='fbtn'+(i===cur?' on':'');
    b.setAttribute('aria-pressed',i===cur);
    b.innerHTML='<b>'+f.etiket+'</b><span class="mono">'+f.ozet.sure+'s · '+f.n_orig+' kare</span>';
    b.onclick=()=>{cur=i;draw();};el.appendChild(b);});}

/* ---------- özet kartları ---------- */
function renderKpis(){const o=DATA[cur].ozet,s=stats(DATA[cur]);
  const fark=o.hiz_fark;
  const cards=[
    {lab:'En yakın menzil',val:f1(o.dh_min)+' m',note:'istasyon 11 m',c:cls(o.dh_min,14,30)},
    {lab:'Hız farkı (drone−hedef)',val:(fark>0?'+':'')+f1(fark)+' m/s',
     note:fark<0?'drone daha YAVAŞ → yetişemez':'drone daha hızlı',c:fark==null?'':(fark<0?'bad':fark<2?'warn':'good')},
    {lab:'Komut doygunluğu',val:f0(o.doygun)+'%',note:'tavan '+f1(o.v_tavan)+' m/s',
     c:o.doygun>=90?'bad':o.doygun>=60?'warn':'good'},
    {lab:'KILIT oranı',val:f1(o.kilit_pct)+'%',note:o.kilit_kare+' kare · devir bandı',
     c:o.kilit_pct>=20?'good':o.kilit_pct>0?'warn':'bad'},
    {lab:'Telemetri',val:f1(o.telem_hz)+' Hz',note:'hedef konumu · log 20 Hz tavan',
     c:o.telem_hz>=15?'good':o.telem_hz>=8?'warn':'bad'},
  ];
  document.getElementById('kpis').innerHTML=cards.map(c=>
    '<div class="kpi '+c.c+'"><div class="stripe"></div><div class="lab">'+c.lab+
    '</div><div class="val '+c.c+'">'+c.val+'</div><div class="note">'+c.note+'</div></div>').join('');}

/* ---------- canvas yardımcıları ---------- */
function fit(id,ratio){const c=document.getElementById(id),r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;
  const h=ratio?Math.round(r.width*ratio):160;c.width=r.width*d;c.height=h*d;
  const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);return{x,w:r.width,h,c};}

/* ---------- jenerik zaman serisi (hover'lı) ---------- */
const TS={};
function drawTS(id,opt){
  const{x,w,h,c}=fit(id,opt.ratio||0.20);x.clearRect(0,0,w,h);
  const P=DATA[cur].pts.filter(p=>p.t!=null);if(!P.length)return;
  const pad={l:44,r:12,t:12,b:20};
  const T=P.map(p=>p.t),tmin=Math.min(...T),tmax=Math.max(...T);
  let vals=[];opt.series.forEach(s=>P.forEach(p=>{const v=p[s.k];if(v!=null&&!isNaN(v))vals.push(v);}));
  (opt.refs||[]).forEach(r=>vals.push(r.v));
  if(!vals.length)return;
  let ymin=opt.ymin!=null?opt.ymin:Math.min(...vals),ymax=opt.ymax!=null?opt.ymax:Math.max(...vals);
  if(ymax-ymin<1e-6)ymax=ymin+1;
  const m=(ymax-ymin)*0.08;ymin-=m;ymax+=m;
  const X=t=>pad.l+(t-tmin)/(tmax-tmin||1)*(w-pad.l-pad.r),Y=v=>pad.t+(ymax-v)/(ymax-ymin)*(h-pad.t-pad.b);
  /* dönüş fazı bantları — arka planda (kayıp burada olur) */
  if(opt.bands){x.fillStyle=css('--band');let i=0;
    while(i<P.length){if(P[i].ph===1){let j=i;while(j<P.length&&P[j].ph===1)j++;
      x.fillRect(X(P[i].t),pad.t,Math.max(1.5,X(P[Math.min(j,P.length-1)].t)-X(P[i].t)),h-pad.t-pad.b);i=j;}else i++;}}
  /* ızgara + eksen etiketleri */
  x.font='10px ui-monospace,monospace';x.textBaseline='middle';
  const tick=[ymin+m,(ymin+ymax)/2,ymax-m];
  tick.forEach(g=>{x.strokeStyle=css('--grid');x.beginPath();x.moveTo(pad.l,Y(g));x.lineTo(w-pad.r,Y(g));x.stroke();
    x.fillStyle=css('--ink-dim');x.textAlign='right';x.fillText(g.toFixed(Math.abs(g)<10?1:0),pad.l-5,Y(g));});
  /* referans çizgileri */
  (opt.refs||[]).forEach(r=>{x.setLineDash([4,4]);x.strokeStyle=css(r.c);x.globalAlpha=.75;x.lineWidth=1.4;
    x.beginPath();x.moveTo(pad.l,Y(r.v));x.lineTo(w-pad.r,Y(r.v));x.stroke();x.setLineDash([]);x.globalAlpha=1;});
  /* seriler */
  opt.series.forEach(s=>{x.strokeStyle=css(s.c);x.lineWidth=2;x.lineJoin='round';x.beginPath();let st=false;
    P.forEach(p=>{const v=p[s.k];if(v==null||isNaN(v)){st=false;return;}
      const px=X(p.t),py=Y(Math.max(ymin,Math.min(ymax,v)));st?x.lineTo(px,py):x.moveTo(px,py);st=true;});x.stroke();});
  /* zaman ekseni */
  x.fillStyle=css('--ink-dim');x.textAlign='left';x.fillText('0s',pad.l,h-9);
  x.textAlign='right';x.fillText(Math.round(tmax)+'s',w-pad.r,h-9);
  x.strokeStyle=css('--line');x.lineWidth=1;x.strokeRect(.5,.5,w-1,h-1);
  TS[id]={P,X,Y,pad,w,h,tmin,tmax,opt};
  hover(id);
}
/* crosshair + tooltip — zaman serisi grafiklerinin tümünde */
function hover(id){
  const g=TS[id];if(!g)return;const c=document.getElementById(id);
  let tip=c.parentNode.querySelector('.tip.'+id);
  if(!tip){tip=document.createElement('div');tip.className='tip '+id;c.parentNode.appendChild(tip);}
  if(c._hooked)return;c._hooked=true;
  c.addEventListener('mousemove',e=>{
    const G=TS[id];if(!G)return;const r=c.getBoundingClientRect(),mx=e.clientX-r.left;
    let best=null,bd=1e9;
    G.P.forEach(p=>{const d=Math.abs(G.X(p.t)-mx);if(d<bd){bd=d;best=p;}});
    if(!best||bd>40){tip.style.opacity=0;draw1(id);return;}
    draw1(id);
    const{x}=fit0(id);x.save();x.strokeStyle=css('--ink-dim');x.globalAlpha=.55;x.setLineDash([3,3]);x.lineWidth=1;
    x.beginPath();x.moveTo(G.X(best.t),G.pad.t);x.lineTo(G.X(best.t),G.h-G.pad.b);x.stroke();x.restore();
    G.opt.series.forEach(s=>{const v=best[s.k];if(v==null||isNaN(v))return;
      x.fillStyle=css(s.c);x.strokeStyle=css('--panel');x.lineWidth=2;
      x.beginPath();x.arc(G.X(best.t),G.Y(Math.max(0,v)),4,0,7);x.fill();x.stroke();});
    let html='<b class="mono">'+best.t.toFixed(1)+' s</b>'+(best.ph===1?' · <span style="color:var(--s2)">dönüş</span>':'');
    G.opt.series.forEach(s=>{const v=best[s.k];
      html+='<div class="row"><span><i style="background:'+css(s.c)+'"></i>'+s.lab+'</span><b class="mono">'+
        (v==null||isNaN(v)?'—':(+v).toFixed(1)+(G.opt.unit||''))+'</b></div>';});
    tip.innerHTML=html;tip.style.opacity=1;
    const tw=tip.offsetWidth,left=Math.min(Math.max(4,G.X(best.t)-tw/2),G.w-tw-4);
    tip.style.left=left+'px';tip.style.top=(c.offsetTop+8)+'px';
  });
  c.addEventListener('mouseleave',()=>{tip.style.opacity=0;draw1(id);});
}
function fit0(id){const c=document.getElementById(id),d=window.devicePixelRatio||1;
  const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);return{x};}
const TSOPT={
  hiz:{ratio:0.20,unit:' m/s',bands:true,ymin:0,
       series:[{k:'ds',c:'--s1',lab:'drone'},{k:'ts',c:'--s2',lab:'hedef'},{k:'cs',c:'--s3',lab:'komut'}]},
  range:{ratio:0.17,unit:' m',bands:true,ymin:0,refs:[{v:11,c:'--good'}],
       series:[{k:'dh',c:'--s1',lab:'d_h'}]},
  angles:{ratio:0.19,unit:'°',bands:true,ymin:-40,ymax:60,refs:[{v:25,c:'--s1'},{v:0,c:'--s2'}],
       series:[{k:'elev',c:'--s1',lab:'elev'},{k:'yaw',c:'--s2',lab:'yaw'}]},
  tilt:{ratio:0.17,unit:'°',bands:true,ymin:0,
       series:[{k:'rl',c:'--s1',lab:'|roll|'},{k:'pt',c:'--s2',lab:'|pitch|'}]},
  alt:{ratio:0.17,unit:' m',bands:true,
       series:[{k:'az',c:'--s1',lab:'drone'},{k:'tz',c:'--s2',lab:'hedef'}]},
};
function draw1(id){drawTS(id,TSOPT[id]);}

/* ---------- yörünge ---------- */
function drawTraj(){const{x,w,h}=fit('traj',0.85);x.clearRect(0,0,w,h);x.fillStyle=css('--panel2');x.fillRect(0,0,w,h);
  const P=DATA[cur].pts.filter(p=>p.ix!=null&&p.tx!=null);if(!P.length)return;
  let xs=[],ys=[];P.forEach(p=>{xs.push(p.ix,p.tx);ys.push(p.iy,p.ty);});
  const emin=Math.min(...ys),emax=Math.max(...ys),nmin=Math.min(...xs),nmax=Math.max(...xs);
  const pad=26,span=Math.max(emax-emin,nmax-nmin)*1.08||1,ecx=(emin+emax)/2,ncx=(nmin+nmax)/2;
  const sc=(Math.min(w,h)-2*pad)/span,PX=e=>w/2+(e-ecx)*sc,PY=n=>h/2-(n-ncx)*sc;
  x.strokeStyle=css('--grid');x.lineWidth=1;
  for(let g=-500;g<=500;g+=50){x.beginPath();x.moveTo(PX(ecx+g),0);x.lineTo(PX(ecx+g),h);x.stroke();
    x.beginPath();x.moveTo(0,PY(ncx+g));x.lineTo(w,PY(ncx+g));x.stroke();}
  const path=(g,col)=>{x.strokeStyle=col;x.lineWidth=2;x.lineJoin='round';x.beginPath();
    P.forEach((p,i)=>{const a=g(p);i?x.lineTo(PX(a[0]),PY(a[1])):x.moveTo(PX(a[0]),PY(a[1]));});x.stroke();};
  path(p=>[p.ty,p.tx],css('--s2'));path(p=>[p.iy,p.ix],css('--s1'));
  const mk=(e,n,col,tri)=>{x.fillStyle=col;x.strokeStyle=css('--panel2');x.lineWidth=2;x.beginPath();
    if(tri){x.moveTo(PX(e),PY(n)-5.5);x.lineTo(PX(e)-4.8,PY(n)+4.2);x.lineTo(PX(e)+4.8,PY(n)+4.2);}else x.arc(PX(e),PY(n),4.2,0,7);
    x.closePath();x.fill();x.stroke();};
  mk(P[0].ty,P[0].tx,css('--s2'),0);mk(P.at(-1).ty,P.at(-1).tx,css('--s2'),1);
  mk(P[0].iy,P[0].ix,css('--s1'),0);mk(P.at(-1).iy,P.at(-1).ix,css('--s1'),1);
  x.strokeStyle=css('--line');x.lineWidth=1;x.strokeRect(.5,.5,w-1,h-1);
  x.fillStyle=css('--ink-dim');x.font='10px ui-monospace,monospace';x.textAlign='left';x.textBaseline='alphabetic';
  x.fillText('E →',w-30,h-8);x.fillText('N ↑',8,16);
  x.strokeStyle=css('--ink-dim');x.beginPath();x.moveTo(14,h-16);x.lineTo(14+50*sc,h-16);x.stroke();x.fillText('50 m',16,h-20);}

/* ---------- nişangâh ---------- */
function drawReticle(){const{x,w,h}=fit('reticle',0.75);const sx=w/640,sy=h/480;
  x.clearRect(0,0,w,h);x.fillStyle=css('--panel2');x.fillRect(0,0,w,h);x.strokeStyle=css('--grid');x.lineWidth=1;
  for(let g=0;g<=640;g+=80){x.beginPath();x.moveTo(g*sx,0);x.lineTo(g*sx,h);x.stroke();}
  for(let g=0;g<=480;g+=80){x.beginPath();x.moveTo(0,g*sy);x.lineTo(w,g*sy);x.stroke();}
  const CX=320*sx,CY=240*sy;x.strokeStyle=css('--accent');x.lineWidth=1.4;
  x.beginPath();x.moveTo(CX-16,CY);x.lineTo(CX+16,CY);x.moveTo(CX,CY-16);x.lineTo(CX,CY+16);x.stroke();
  x.globalAlpha=.5;x.beginPath();x.arc(CX,CY,30,0,7);x.stroke();x.globalAlpha=1;
  const pts=DATA[cur].pts.filter(p=>p.u!=null&&p.v!=null);
  pts.forEach((p,i)=>{let u=p.u,v=p.v,off=false;
    if(u<0||u>640||v<0||v>480){off=true;u=Math.max(0,Math.min(640,u));v=Math.max(0,Math.min(480,v));}
    x.globalAlpha=.14+.86*(i/Math.max(1,pts.length));
    x.fillStyle=off?css('--bad'):(p.durum==='KILIT'?css('--good'):css('--warn'));
    x.beginPath();x.arc(u*sx,v*sy,off?2.4:3.1,0,7);x.fill();});
  x.globalAlpha=1;x.strokeStyle=css('--line');x.strokeRect(.5,.5,w-1,h-1);}

/* ---------- faz tablosu ---------- */
function renderPhase(){const o=DATA[cur].ozet,el=document.getElementById('phase');
  const row=(ad,d)=>{if(!d)return '<tr><td>'+ad+'</td><td colspan="6" style="color:var(--ink-dim)">yeterli örnek yok</td></tr>';
    const fark=(d.drone!=null&&d.hedef!=null)?d.drone-d.hedef:null;
    return '<tr><td>'+ad+'</td><td class="mono">'+d.n+'</td><td class="mono">'+f1(d.drone)+
      '</td><td class="mono">'+f1(d.hedef)+'</td><td class="mono '+(fark<0?'neg':'pos')+'">'+
      (fark>0?'+':'')+f1(fark)+'</td><td class="mono">'+f1(d.komut)+'</td><td class="mono">'+
      f1(d.roll)+'°</td><td class="mono">'+f1(d.dh)+'</td></tr>';};
  el.innerHTML='<thead><tr><th>Faz</th><th>kare</th><th>drone m/s</th><th>hedef m/s</th><th>Δhız</th>'+
    '<th>komut m/s</th><th>|roll|</th><th>d_h m</th></tr></thead><tbody>'+
    row('Düz uçuş',o.duz)+row('Dönüş (manevra)',o.donus)+'</tbody>';}

/* ---------- tüm uçuşlar ---------- */
function renderAll(){const el=document.getElementById('all');
  el.innerHTML='<thead><tr><th>Uçuş</th><th>süre</th><th>Hz</th><th>d_h min</th><th>d_h ort</th>'+
    '<th>KILIT</th><th>drone</th><th>hedef</th><th>Δhız</th><th>doygun</th></tr></thead><tbody>'+
    DATA.map((f,i)=>{const o=f.ozet,fk=o.hiz_fark;
      return '<tr class="'+(i===cur?'on':'')+'" data-i="'+i+'"><td>'+f.etiket+'</td><td class="mono">'+o.sure+
        's</td><td class="mono">'+f1(o.telem_hz)+'</td><td class="mono">'+f1(o.dh_min)+'</td><td class="mono">'+
        f0(o.dh_med)+'</td><td class="mono">'+f1(o.kilit_pct)+'%</td><td class="mono">'+f1(o.drone_med)+
        '</td><td class="mono">'+f1(o.hedef_med)+'</td><td class="mono '+(fk<0?'neg':'pos')+'">'+
        (fk>0?'+':'')+f1(fk)+'</td><td class="mono">'+f0(o.doygun)+'%</td></tr>';}).join('')+'</tbody>';
  el.querySelectorAll('tbody tr').forEach(tr=>tr.onclick=()=>{cur=+tr.dataset.i;draw();
    window.scrollTo({top:0,behavior:'smooth'});});}

/* ---------- otomatik yorum ---------- */
function verdict(){const f=DATA[cur],o=f.ozet,s=stats(f),el=document.getElementById('verdict');
  let m=[];
  if(o.hiz_fark!=null&&o.hiz_fark<0){
    m.push('<b>Hız yetersiz:</b> drone medyan <b class="mono">'+f1(o.drone_med)+' m/s</b>, hedef <b class="mono">'+
      f1(o.hedef_med)+' m/s</b> → açı kapanmıyor. Komut doygunluğu <b class="mono">'+f0(o.doygun)+
      '%</b> (tavan '+f1(o.v_tavan)+' m/s): güdüm daha fazlasını istiyor ama '+
      (o.doygun>=90?'<b>yazılım hız tavanına dayanmış</b> — V_MAX yükseltilmeli':'araç uygulayamıyor')+'.');
  }
  if(o.donus&&o.duz&&o.donus.drone!=null&&o.duz.drone!=null&&o.donus.drone<o.duz.drone-2){
    m.push('<b>Manevra kaybı:</b> düz uçuşta <b class="mono">'+f1(o.duz.drone)+'</b> m/s yapan drone dönüşte <b class="mono">'+
      f1(o.donus.drone)+'</b> m/s\'ye düşüyor (hedef dönüşte '+f1(o.donus.hedef)+'). Dönüşte |roll| '+
      f1(o.donus.roll)+'° — araç yeterince yaslanamıyorsa itki rezervi yok demektir.');
  }
  if(o.kilit_kare===0&&o.dh_med>40){
    m.push('<b>Hiç yaklaşamadı:</b> ort d_h <b class="mono">'+f0(o.dh_med)+' m</b> (en yakın '+f1(o.dh_min)+
      ' m), 0 KILIT → görsel devir yok. Yörüngede drone hedeften <b>büyük yay</b> çiziyorsa kuyruk-takibi dönen hedefte içeri kesemiyordur.');
  }else if(o.kilit_kare>0){
    m.push('<b>Kadraja girdi:</b> en yakın <b class="mono">'+f1(o.dh_min)+' m</b>, |yaw| ≈ <b class="mono">'+
      f1(s.yaw)+'°</b> (u_px '+f0(s.u)+'/320), elev ≈ <b class="mono">'+f1(s.elev)+'°</b> (hedef 25° → hedef '+
      (s.v>240?'merkezin altında':'merkeze yakın')+'). <b class="mono">'+o.kilit_kare+'</b> KILIT karesi ('+f1(o.kilit_pct)+'%).');
  }
  if(o.telem_hz!=null&&o.telem_hz<8){
    m.push('<b>Telemetri yavaş:</b> hedef konumu yalnız <b class="mono">'+f1(o.telem_hz)+
      ' Hz</b> tazeleniyor → güdüm bayat konuma komut veriyor (faz gecikmesi). MAVProxy <code>--streamrate</code> kontrol edilmeli.');
  }
  el.innerHTML=m.length?m.join('<br><br>'):'Belirgin bir sorun imzası bulunamadı.';}

function draw(){renderFlights();renderKpis();
  Object.keys(TSOPT).forEach(id=>drawTS(id,TSOPT[id]));
  drawTraj();drawReticle();renderPhase();renderAll();verdict();}
let rt;window.addEventListener('resize',()=>{clearTimeout(rt);rt=setTimeout(draw,140);});
matchMedia('(prefers-color-scheme: dark)').addEventListener('change',draw);
draw();
</script></body></html>'''


if __name__ == "__main__":
    sys.exit(main())
