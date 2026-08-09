#!/usr/bin/env python3
"""Güdüm CSV'sini rejimlere ayırıp özetler: DÜZ vs DÖNÜŞ.

Otonom uçuş testinin analiz bacağı (bkz. docs/OTONOM_UCUS_TESTI.md).
Hedefin açısal hızını tgt_vx/vy başlık türevinden kestirir (EMA'lı) ve
her satırı DÜZ (|ω|<0.05), DÖNÜŞ (|ω|>0.12) ya da geçiş sayar. Rejim
başına medyan/persantil tablosu basar.

⚠ İki uçuşu kıyaslarken İKİSİNE DE AYNI segmentasyonu uygula — panelden
elle okunan "en iyi an" değerleriyle medyanlar karşılaştırılamaz (2026-08-08
dersi: "kare kenarı 14 m" okuması, medyanı hep ~22 m olan dağılımın en iyi
anıymış).

Kullanım:
    python3 tools/ucus_analiz.py logs/gps_guidance_YYYYMMDD_HHMMSS.csv [ikinci.csv ...]

Birden çok CSV verilirse her biri aynı yöntemle basılır (kıyas için).
İlk ~30 s (yaklaşma) atılır.
"""
import csv
import math
import statistics as st
import sys

ATLA_SATIR = 600          # 20 Hz'de ilk 30 s (yaklaşma) analiz dışı
OM_DUZ = 0.05             # rad/s; altı DÜZ
OM_DONUS = 0.12           # rad/s; üstü DÖNÜŞ
OM_EMA = 0.1              # açısal hız kestirimi yumuşatması


def _f(r, k):
    v = r.get(k, "")
    try:
        return float(v) if v not in ("", "None") else None
    except ValueError:
        return None


def analiz(yol):
    rows = list(csv.DictReader(open(yol)))
    if len(rows) < ATLA_SATIR + 200:
        print(f"{yol}: yetersiz veri ({len(rows)} satır)")
        return

    onceki_hdg = None
    onceki_t = None
    om_ema = 0.0
    om = [0.0] * len(rows)
    for i, r in enumerate(rows):
        t = _f(r, "t")
        vx = _f(r, "tgt_vx")
        vy = _f(r, "tgt_vy")
        if vx is not None and t is not None and math.hypot(vx, vy) > 3:
            hdg = math.atan2(vy, vx)
            if onceki_hdg is not None and 0 < t - onceki_t < 0.5:
                dh = (hdg - onceki_hdg + math.pi) % (2 * math.pi) - math.pi
                om_ema = OM_EMA * (dh / (t - onceki_t)) + (1 - OM_EMA) * om_ema
            onceki_hdg = hdg
            onceki_t = t
        om[i] = abs(om_ema)

    idx = list(range(ATLA_SATIR, len(rows)))
    gruplar = {
        "DÜZ  ": [i for i in idx if om[i] < OM_DUZ],
        "DÖNÜŞ": [i for i in idx if om[i] > OM_DONUS],
    }
    mh = "menzil_ham" if _f(rows[-1], "menzil_ham") is not None else "menzil"

    print(f"\n══ {yol}  ({len(rows)} satır ≈ {len(rows)/20/60:.1f} dk)")
    for ad, ix in gruplar.items():
        if len(ix) < 100:
            print(f"  {ad}: yetersiz örnek ({len(ix)})")
            continue

        def med(k):
            v = [x for x in (_f(rows[i], k) for i in ix) if x is not None]
            return st.median(v) if v else None

        def pct(k, q):
            v = sorted(x for x in (_f(rows[i], k) for i in ix) if x is not None)
            return v[q * len(v) // 100] if v else None

        satir = (f"  {ad} (n={len(ix)}, ~{len(ix)/20:.0f} s)  "
                 f"menzil med {med(mh):.1f} [p10 {pct(mh,10):.1f} p90 {pct(mh,90):.1f}]  "
                 f"pitch {med('iris_pitch_deg'):+.1f}°  "
                 f"kadraj {med('kadraj_pitch_hata_deg'):+.1f}°  "
                 f"v_px {med('v_px'):.0f}")
        ev = med("ist_elev_deg")
        if ev is not None:
            satir += f"  ist_elev {ev:.1f}°"
        print(satir)

    ev = [x for x in (_f(r, "ist_elev_deg") for r in rows[ATLA_SATIR:]) if x is not None]
    if len(ev) > 100:
        dif = [abs(b - a) for a, b in zip(ev, ev[1:])]
        print(f"  ist_elev tick adımı: med {st.median(dif):.3f}°, max {max(dif):.2f}° "
              f"(sıçrama = salınım şüphesi)")

    # ── GEÇERLİLİK: koşu sağlıklı koşullarda mıydı? ──
    # 2026-08-08 dersi: düz uçuş testinde hedef 12 m'ye alçalmış, güdüm 8 m
    # irtifa tabanına dayanmış — sayılar "iyi" görünürken koşul temsilî
    # değildi. Menzil özetine bakmadan önce bu satır kontrol edilir.
    alt = [-x for x in (_f(rows[i], "tgt_z") for i in idx) if x is not None]
    spd = [math.hypot(a, b) for a, b in
           ((_f(rows[i], "tgt_vx"), _f(rows[i], "tgt_vy")) for i in idx)
           if a is not None and b is not None]
    if alt and spd:
        u1 = "⚠ " if (min(alt) < 20 or max(alt) > 250) else ""
        u2 = "⚠ " if not (8 <= st.median(spd) <= 22) else ""
        print(f"  GEÇERLİLİK: {u1}hedef irtifa med {st.median(alt):.0f} "
              f"[min {min(alt):.0f}, max {max(alt):.0f}] m (bant 20-250) | "
              f"{u2}hedef hız med {st.median(spd):.1f} m/s")
        if u1 or u2:
            print("  ⚠ Koşul bandın dışında — bu koşunun menzil sayılarına"
                  " tek başına güvenme, koşuyu yinele.")


def gorsel_geometri(meta_yol):
    """meta.csv'den ASPECT AÇISI + MESAFE EĞİLİMİ — görsel faz sağlık ölçütü.

    ⚠ NEDEN VAR (2026-08-08, pahalı ders): bbox-IBVS'in ilk sürümü "160 s
    kesintisiz faz, mesafe medyanı 7.2 m" diye RAPOR EDİLDİ ve iyi sanıldı.
    Kullanıcı videodan yakaladı: drone hedefi YANDAN görüyordu ve mesafe
    sürekli AÇILIYORDU. Medyan bunu gizler; aspect ve eğilim gizlemez.

    aspect = hedefin KUYRUK yönü ile hedef→drone vektörü arasındaki açı:
        0°  = tam kuyrukta (IBVS için ideal)
        90° = tam yanda (bbox-IBVS burada hedefi kaçırır)
    """
    rows = [r for r in csv.DictReader(open(meta_yol))
            if r.get("chase_aktif") == "True"
            and r.get("plane_x") not in ("", None)]
    if len(rows) < 20:
        print(f"{meta_yol}: yetersiz kayıt")
        return

    def g(r, k):
        return float(r[k])

    ornek = []
    for a, b in zip(rows, rows[3:]):          # ~3 s'lik taban çizgisi
        dt = g(b, "wall_t") - g(a, "wall_t")
        if dt <= 0:
            continue
        hdg = math.atan2(g(b, "plane_y") - g(a, "plane_y"),
                         g(b, "plane_x") - g(a, "plane_x"))
        rx = g(b, "iris_x") - g(b, "plane_x")
        ry = g(b, "iris_y") - g(b, "plane_y")
        kuy = math.atan2(-math.sin(hdg), -math.cos(hdg))     # kuyruk yönü
        asp = abs((math.atan2(ry, rx) - kuy + math.pi) % (2 * math.pi) - math.pi)
        ornek.append((g(b, "wall_t"), math.degrees(asp),
                      float(b["mesafe"]) if b["mesafe"] not in ("", "None") else None))

    if not ornek:
        return
    t0 = ornek[0][0]
    asps = [a for _, a, _ in ornek]
    ms = [m for _, _, m in ornek if m is not None]
    ilk = [m for t, _, m in ornek if m is not None and t - t0 < (ornek[-1][0] - t0) / 3]
    son = [m for t, _, m in ornek if m is not None and t - t0 > 2 * (ornek[-1][0] - t0) / 3]
    print(f"\n══ GÖRSEL GEOMETRİ — {meta_yol}")
    print(f"  aspect: med {st.median(asps):.0f}°  ilk %10 {sorted(asps)[len(asps)//10]:.0f}°  "
          f"son çeyrek med {st.median(asps[3*len(asps)//4:]):.0f}°   "
          f"(0=kuyruk, 90=yan)")
    if ilk and son:
        yon = "AÇILIYOR ⚠" if st.median(son) > st.median(ilk) + 1.0 else "kapanıyor/sabit"
        print(f"  mesafe: ilk üçte bir med {st.median(ilk):.1f} m → "
              f"son üçte bir med {st.median(son):.1f} m  → {yon}")
    if st.median(asps[3*len(asps)//4:]) > 45:
        print("  ⚠ SON ÇEYREKTE YAN GEOMETRİ — bbox-IBVS bu pozdan hedefi kaçırır.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(1)
    for yol in sys.argv[1:]:
        if yol.endswith("meta.csv"):
            gorsel_geometri(yol)
        else:
            analiz(yol)
