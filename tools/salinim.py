#!/usr/bin/env python3
"""SALINIM — aracın hedefe göre GERÇEK yanal hareketinden ölçülür.

NEDEN VAR (kullanıcı 2026-08-11, öncekini çürüterek):
"Salınımı bbox'ın oynamasından analiz etme. Bizim dronun hedef manevra
yaptıktan sonraki hareketlerine bak: hedefin solundayken sağına geçiyor mu?
Hedef sağa manevra yaptı, biz de sağa döndük — sonra hedefin DAHA DA sağına
geçiyor muyuz? Çok sağına geçiyorsak salınım vardır."

ESKİ ÖLÇÜT NEDEN YANLIŞTI: salınım `cx` (kutunun kadrajdaki yeri) işaret
değişiminden sayılıyordu. Kutu YOKSA sayılamıyordu — yani hedefi kaybeden
koşu "sakin" görünüyordu. Ölçüt, hedefi kaybetmeyi ödüllendiriyordu.
Bu araç kutuya HİÇ bakmaz; iki aracın GPS'inden geometriyi kurar, o yüzden
görsel temas kopsa bile ölçebilir.

GEOMETRİ — drone'un konumu HEDEFİN çerçevesinde:
    ĥ = hedefin gidiş yönü (hızından, yumuşatılmış)
    n̂ = ĥ'nin 90° sağı
    yanal (cross) = (drone − hedef) · n̂   → + ise drone hedefin SAĞINDA
    boyuna (along) = (drone − hedef) · ĥ   → − ise drone hedefin ARKASINDA

İDEAL: yanal ≈ 0 (hedefin tam arkasında), boyuna < 0 (kuyrukta).
SALINIM: yanal'ın işaret değiştirmesi = drone bir yandan öbür yana geçiyor.
AŞIM: hedef sağa kırdıktan sonra yanal'ın hedefin sağına doğru fırlaması.

Kullanım:
    python3 tools/salinim.py <koşu_dizini> [<koşu_dizini> ...]
"""
import csv
import json
import math
import os
import statistics as st
import sys

OLU_BANT_M = 2.0     # m; yanal işaret değişimi ölü bandı (gürültü sayılmasın)
HIZ_MIN = 3.0        # m/s; hedef bu hızın altındaysa yön güvenilmez


def fl(x, k):
    try:
        v = float(x[k])
        return v if v == v else None
    except (TypeError, ValueError, KeyError):
        return None


def coz(dizin):
    """telem.csv (10 Hz) → hedef çerçevesinde (t, yanal, boyuna, mesafe)."""
    yol = os.path.join(dizin, "telem.csv")
    if not os.path.exists(yol):
        return None
    r = list(csv.DictReader(open(yol)))
    if len(r) < 50:
        return None
    t0 = fl(r[0], "wall_t") or 0.0
    ham = []
    for x in r:
        px, py = fl(x, "plane_x"), fl(x, "plane_y")
        ix, iy = fl(x, "iris_x"), fl(x, "iris_y")
        t = fl(x, "wall_t")
        if None in (px, py, ix, iy, t):
            continue
        ham.append((t - t0, px, py, ix, iy, x.get("faz", "")))
    if len(ham) < 50:
        return None

    # hedefin gidiş yönü: konum türevi, ~0.5 s penceresiyle yumuşatılmış
    N = 5
    seri = []
    for i in range(len(ham)):
        a = ham[max(0, i - N)]
        b = ham[min(len(ham) - 1, i + N)]
        dt = b[0] - a[0]
        if dt < 1e-3:
            continue
        vx, vy = (b[1] - a[1]) / dt, (b[2] - a[2]) / dt
        hiz = math.hypot(vx, vy)
        if hiz < HIZ_MIN:
            continue
        hx, hy = vx / hiz, vy / hiz          # ĥ birim yön
        nx, ny = hy, -hx                     # n̂ = ĥ'nin 90° sağı (NED: x kuzey, y doğu)
        dx, dy = ham[i][3] - ham[i][1], ham[i][4] - ham[i][2]
        seri.append({
            "t": ham[i][0],
            "yanal": dx * nx + dy * ny,
            "boyuna": dx * hx + dy * hy,
            "mesafe": math.hypot(dx, dy),
            "faz": ham[i][5],
        })
    return seri or None


def olc(dizin):
    ad = os.path.basename(dizin.rstrip("/"))
    seri = coz(dizin)
    if not seri:
        return {"ad": ad, "hata": "telem.csv yok/yetersiz"}
    olay = {}
    oy = os.path.join(dizin, "olay.json")
    if os.path.exists(oy):
        olay = json.load(open(oy))
    tt = olay.get("tetik_t")

    # ── PENCERE: tetikten kaydın SONUNA kadar ──
    # ⚠ Önce tetik+20 s alınmıştı; P09'da asıl geçiş tetikten 34 s SONRA
    # olduğu için pencere olayı ıskalıyordu. Kaçamak sonrası toparlanma ve
    # yeniden yaklaşma bu ölçütün asıl konusu — sonuna kadar bakılır.
    pen = [s for s in seri if tt is None or s["t"] >= tt]
    if len(pen) < 30:
        pen = seri

    yanal = [s["yanal"] for s in pen]
    # işaret değişimi = drone hedefin bir yanından öbür yanına geçti
    gecis, onceki = 0, 0
    for v in yanal:
        t = 1 if v > OLU_BANT_M else (-1 if v < -OLU_BANT_M else 0)
        if t and onceki and t != onceki:
            gecis += 1
        if t:
            onceki = t
    sure = max(pen[-1]["t"] - pen[0]["t"], 1e-6)

    ay = [abs(v) for v in yanal]
    arka = [s for s in pen if s["boyuna"] < 0]        # hedefin arkasında mı
    # KAÇAMAK YÖNÜNDE AŞIM (kullanıcının tarifi): tüm kaçamaklar SAĞA kırıyor
    # (yatay aileron 2000, capraz 1950) → drone hedefin sağına ne kadar taştı?
    asim_sag = max(yanal)
    onde = [s for s in pen if s["boyuna"] > 0]        # hedefin ÖNÜNE geçti mi
    return {
        "ad": ad,
        "kacamak": olay.get("kacamak"),
        "imha": bool(olay.get("imha")),
        "en_yakin": olay.get("en_yakin"),
        "gecis": gecis,                                # kaç kez yandan yana geçti
        "gecis_hz": round(gecis / sure, 3),
        "yanal_med": round(st.median(ay), 1),          # tipik yanal kaçıklık
        "yanal_p90": round(sorted(ay)[int(0.9 * (len(ay) - 1))], 1),
        "yanal_max": round(max(ay), 1),                # en büyük AŞIM
        "arkada_pay": round(100.0 * len(arka) / len(pen)),   # kuyrukta kalma %
        "asim_sag": round(asim_sag, 1),      # kaçamak yönünde tepe aşım (m)
        "onde_pay": round(100.0 * len(onde) / len(pen)),     # önüne geçme %
        "sure": round(sure, 1),
        "n": len(pen),
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(1)
    sonuc = [olc(d) for d in sys.argv[1:] if os.path.isdir(d)]
    iyi = [o for o in sonuc if not o.get("hata")]
    for o in sonuc:
        if o.get("hata"):
            print(f"  ⚠ {o['ad']}: {o['hata']}")
    if not iyi:
        return
    print(f"\n{'koşu':<22}{'kaçamak':<8}{'isabet':>7}{'en yakın':>10}"
          f"{'yandan yana geçiş':>19}{'yanal med':>11}{'SAĞA AŞIM':>11}"
          f"{'önde %':>8}")
    print("─" * 98)
    for o in iyi:
        print(f"{o['ad']:<22}{str(o['kacamak']):<8}{'✓' if o['imha'] else '✗':>7}"
              f"{str(o['en_yakin']):>9} m{o['gecis']:>13} ({o['gecis_hz']:.2f}/s)"
              f"{o['yanal_med']:>10} m{o['asim_sag']:>10} m{o['onde_pay']:>7}%")
    print("\n  yandan yana geçiş = drone hedefin bir yanından öbür yanına geçti "
          f"(±{OLU_BANT_M:.0f} m ölü bant) — SALINIMIN doğrudan ölçüsü")
    print("  SAĞA AŞIM = kaçamak yönünde (sağa) hedefin ne kadar yanına taştı "
          "— kullanıcının tarif ettiği AŞIM")
    print("  önde % = hedefin ÖNÜNE geçilen kare oranı (kuyruk geometrisi bozuldu mu)")


if __name__ == "__main__":
    main()
