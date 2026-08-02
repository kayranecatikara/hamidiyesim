#!/usr/bin/env python3
"""gecis_analiz.py — bir uçuşun terminal geçişlerini ölçer.

    python3 tools/gecis_analiz.py              # en son uçuş
    python3 tools/gecis_analiz.py 126 127      # belirli BIN'ler (sıra önemsiz)
    python3 tools/gecis_analiz.py --liste      # son 10 uçuşu listele

NEDEN VAR: "drone nereden ıskaladı" sorusunun dürüst cevabı CSV'de YOK.
`visual_lead` CSV'sindeki `menzil_gercek_m` MAVLink telemetrisinden geliyor;
EKF çerçeve ofsetinden etkileniyor ve en yakın anı geriden gösteriyor
(ölçüldü: CSV 3.20 m derken kara kutu 0.21 m diyordu). Tek dürüst kaynak
İKİ ARACIN kara kutusu.

YÖNTEM: her iki `.BIN`'den `POS` (Lat/Lng/Alt) alınır, `GPS.GWk`+`GPS.GMS`
ile ortak GPS saatine hizalanır (iki SITL'in boot saati farklı), sonra
aradaki yatay/dikey mesafe hesaplanır. `visual_lead` CSV'leri uçuş
penceresine göre eşleştirilip durum dağılımı eklenir.

OKUMA: `DIKEY` = uçak_irtifa − kopter_irtifa.
  pozitif → uçak yukarıda, drone ALTTAN geçti
  negatif → drone üstten geçti
  |dikey| < 0.4 → aynı seviye (temas ihtimali)
"""

import bisect
import csv
import glob
import math
import os
import statistics as st
import sys

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink yok:  pip install pymavlink")

LOG_DIR = os.path.expanduser("~/ardupilot/logs")
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSV_DIR = os.path.join(PROJ, "logs")


def _oku(yol):
    """POS + PSCD + ATT oku, zaman damgalarını GPS haftası saatine taşı."""
    m = mavutil.mavlink_connection(yol)
    pos, gps, att = [], [], []
    pscd = 0
    while True:
        msg = m.recv_match(type=["POS", "GPS", "PSCD", "ATT"], blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == "POS":
            pos.append((msg.TimeUS / 1e6, msg.Lat, msg.Lng, msg.Alt))
        elif t == "ATT":
            att.append((msg.TimeUS / 1e6, msg.Yaw, msg.DesYaw))
        elif t == "PSCD":
            pscd += 1
        elif getattr(msg, "Status", 0) >= 3:
            gps.append((msg.TimeUS / 1e6, msg.GWk, msg.GMS))
    if not pos or not gps:
        return None
    # iki SITL'in boot saati farklı; ortak eksen GPS haftası saniyesi
    off = st.median([(g[1] * 604800 + g[2] / 1000.0) - g[0] for g in gps])
    return {
        "pos": [(t + off, la, ln, al) for t, la, ln, al in pos],
        "att": [(t + off, y, d) for t, y, d in att],
        "kopter": pscd > 0,          # PSCD yalnız kopterde var
    }


def _cift_bul(argv):
    """BIN çiftini seç: argümandan ya da en yeni ikili."""
    if len(argv) >= 2:
        nums = [int(a) for a in argv[:2]]
        yollar = [os.path.join(LOG_DIR, "%08d.BIN" % n) for n in nums]
    else:
        hepsi = sorted(glob.glob(os.path.join(LOG_DIR, "*.BIN")),
                       key=os.path.getmtime)
        if len(hepsi) < 2:
            sys.exit(f"{LOG_DIR} içinde en az 2 .BIN yok")
        yollar = hepsi[-2:]
    veri = [(_oku(y), y) for y in yollar]
    if any(v is None for v, _ in veri):
        sys.exit("BIN okunamadı (POS/GPS yok — uçuş çok kısa olabilir)")
    kopter = [(v, y) for v, y in veri if v["kopter"]]
    ucak = [(v, y) for v, y in veri if not v["kopter"]]
    if not kopter or not ucak:
        sys.exit("Bir kopter + bir uçak kaydı gerekli (PSCD ile ayırt ediliyor). "
                 "Elle ver: python3 tools/gecis_analiz.py <kopter> <ucak>")
    return kopter[0], ucak[0]


def _geometri(K, U):
    """Kopter zaman ekseninde (yatay, dikey, menzil) dizisi."""
    lat0 = K["pos"][0][1]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    tu = [p[0] for p in U["pos"]]
    t0 = K["pos"][0][0]
    G = []
    for t, la, ln, al in K["pos"]:
        i = bisect.bisect_left(tu, t)
        if i == 0 or i >= len(tu):
            continue
        t1, la1, ln1, al1 = U["pos"][i - 1]
        t2, la2, ln2, al2 = U["pos"][i]
        if not (0 < t2 - t1 < 0.5):          # telemetri boşluğu — atla
            continue
        w = (t - t1) / (t2 - t1)
        dn = (la1 + (la2 - la1) * w - la) * mlat
        de = (ln1 + (ln2 - ln1) * w - ln) * mlon
        dd = (al1 + (al2 - al1) * w) - al    # + = uçak yukarıda
        yat = math.hypot(dn, de)
        G.append((t - t0, math.hypot(yat, dd), yat, dd))
    return G


def _yaklasmalar(G, esik=12.0):
    """Yerel menzil minimumları (aynı geçişin tekrarını eler)."""
    out = []
    for i in range(2, len(G) - 2):
        if G[i][1] < esik and G[i][1] <= min(G[i - 2][1], G[i - 1][1],
                                             G[i + 1][1], G[i + 2][1]):
            if not out or G[i][0] - out[-1][0] > 2.0:
                out.append(G[i])
    return out


def _wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def _titreme(att, t0, a=20.0, b=45.0):
    """Seyir penceresinde yaw takip hatası — heading titremesi ölçüsü."""
    W = [(t - t0, y, d) for t, y, d in att if a <= t - t0 <= b]
    if len(W) < 50:
        return None
    e = [_wrap(y - d) for _, y, d in W]
    em = st.mean(e)
    gecis = sum(1 for i in range(1, len(e)) if (e[i] - em) * (e[i - 1] - em) < 0)
    sure = W[-1][0] - W[0][0]
    dd = st.mean(abs(_wrap(W[i][2] - W[i - 1][2])) for i in range(1, len(W)))
    dy = st.mean(abs(_wrap(W[i][1] - W[i - 1][1])) for i in range(1, len(W)))
    return dict(std=st.pstdev(e), tepe=max(e) - min(e),
                hz=gecis / 2 / sure if sure else 0, komut=dd, gercek=dy)


def _csv_ozet(bin_yol, sure):
    """Uçuş penceresine düşen visual_lead CSV'lerini özetle."""
    from collections import Counter
    son = os.path.getmtime(bin_yol)
    bas = son - sure - 120
    out = []
    for f in sorted(glob.glob(os.path.join(CSV_DIR, "visual_lead_*.csv")),
                    key=os.path.getmtime):
        mt = os.path.getmtime(f)
        if not (bas <= mt <= son + 120):
            continue
        try:
            rows = list(csv.DictReader(open(f)))
        except OSError:
            continue
        if len(rows) < 5:
            continue

        def fl(r, k):
            try:
                v = float(r[k])
                return v if v == v else None
            except (ValueError, KeyError, TypeError):
                return None

        c = Counter(r["durum"] for r in rows)
        n = len(rows)
        men = [fl(r, "menzil_gercek_m") for r in rows
               if fl(r, "menzil_gercek_m") is not None]
        ts = [fl(r, "t_ros") for r in rows if fl(r, "t_ros")]
        if not men:
            continue
        out.append(dict(ad=os.path.basename(f)[21:27], n=n,
                        sure=ts[-1] - ts[0] if len(ts) > 1 else 0,
                        giris=men[0], enyakin=min(men),
                        ok=100 * c["ok"] / n,
                        kpt=100 * c.get("kpt_dusuk", 0) / n,
                        kor=100 * c.get("kor_dalis", 0) / n,
                        vur=bool(c.get("vuruldu"))))
    return out


def main():
    argv = [a for a in sys.argv[1:] if not a.startswith("-")]
    if "--liste" in sys.argv:
        hepsi = sorted(glob.glob(os.path.join(LOG_DIR, "*.BIN")),
                       key=os.path.getmtime)[-10:]
        import time
        for y in hepsi:
            print(f"  {os.path.basename(y):14s} "
                  f"{time.strftime('%H:%M', time.localtime(os.path.getmtime(y)))} "
                  f"{os.path.getsize(y)/1e6:6.1f} MB")
        return 0

    (K, kyol), (U, uyol) = _cift_bul(argv)
    G = _geometri(K, U)
    if not G:
        sys.exit("İki kayıt zaman olarak örtüşmüyor — aynı uçuşun çifti mi?")

    print(f"kopter {os.path.basename(kyol)}   uçak {os.path.basename(uyol)}   "
          f"{len(G)} ortak örnek, {G[-1][0]:.0f} s")

    print("\n── GEÇİŞLER (kara kutu, iki aracın POS'u) ──")
    print(f"{'t':>7s} {'menzil':>7s} {'yatay':>6s} {'DİKEY':>7s}  yorum")
    yak = _yaklasmalar(G)
    if not yak:
        print("  (12 m altına inen yaklaşma yok)")
    for t, r, y, d in yak:
        yorum = ("ALTTAN geçti" if d > 0.4 else
                 "üstten geçti" if d < -0.4 else "AYNI SEVİYE ← temas ihtimali")
        print(f"{t:7.1f} {r:6.2f}m {y:5.2f}m {d:+6.2f}m  {yorum}")
    if yak:
        dik = [d for _, _, _, d in yak]
        print(f"  dikey artık: medyan {st.median(dik):+.2f} m  "
              f"(pozitif = sistematik alttan geçiyor)")

    t = _titreme(K["att"], K["pos"][0][0])
    if t:
        print("\n── SEYİRDE HEADING TİTREMESİ (t=20-45 s) ──")
        print(f"  yaw takip hatası std {t['std']:.2f}°  tepe-tepe {t['tepe']:.2f}°  "
              f"{t['hz']:.2f} Hz")
        print(f"  komut (DesYaw) {t['komut']:.3f}°/kare  vs  gerçek heading "
              f"{t['gercek']:.3f}°/kare"
              + ("   → güdüm değil, araç attitude döngüsü"
                 if t["gercek"] > 5 * max(t["komut"], 1e-6) else ""))

    ozet = _csv_ozet(kyol, G[-1][0])
    if ozet:
        print("\n── GÖRSEL FAZLAR (visual_lead CSV) ──")
        print(f"{'saat':8s} {'kare':>5s} {'süre':>6s} {'giriş':>7s} {'ok%':>4s} "
              f"{'kpt%':>5s} {'kör%':>5s} {'enyakın':>8s} sonuç")
        for o in ozet:
            print(f"{o['ad'][:2]}:{o['ad'][2:4]}:{o['ad'][4:]} {o['n']:5d} "
                  f"{o['sure']:5.1f}s {o['giris']:6.2f}m {o['ok']:3.0f}% "
                  f"{o['kpt']:4.0f}% {o['kor']:4.0f}% {o['enyakin']:7.2f}m  "
                  f"{'✓ VURULDU' if o['vur'] else 'ıska'}")
        print(f"  faz sayısı {len(ozet)}   giriş medyan "
              f"{st.median([o['giris'] for o in ozet]):.2f} m   "
              f"kör_dalış medyan {st.median([o['kor'] for o in ozet]):.0f}%   "
              f"vuruş {sum(1 for o in ozet if o['vur'])}/{len(ozet)}")
        print("  NOT: vuran geçişlerde kör_dalış ≤ %3 olageldi (4/4). "
              "Bu oran yükseldiyse terminal algı kopmuş demektir.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
