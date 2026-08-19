#!/usr/bin/env python3
"""Vuruş KALİTESİ — kontrollü mü, şans eseri mi? + salınım ölçümü.

NEDEN VAR (kullanıcı kuralı 2026-08-10): "salınım olması büyük bir sorun;
araç dengesizce öyle göte bata vurursa hedef aracı bunu iyi diye sayma."
Yalnız isabet+menzile bakan bir ölçüt, savrulup şans eseri çarpan aracı
ödüllendirir. Bu araç ikisini AYIRIR.

VURUŞ SINIFI — temas öncesi son PENCERE saniyesindeki karelerden:
  KONTROLLÜ  : hedef kesintisiz kadrajda, cx merkeze yakın ve sakin,
               kutu boyutu düzgün büyüyor, yatış sakin
  ŞANS       : temas kopuk, hedef kenarda, boyut sıçramalı ya da araç salınıyor
Her ölçüt tek tek raporlanır — "neden şans sayıldı" görünür olsun.

SALINIM (tüm kaçamak sonrası pencere için):
  cx işaret değişimi/s, yatış işaret değişimi/s, |yatış| p90,
  yaw komutu değişim hızı p90, görsel temas kesintisi sayısı

⚠ SALINIM ÖLÇÜTÜ TEK BAŞINA OKUNMAZ (2026-08-11, kullanıcı yakaladı):
cx işaret değişimi YALNIZ kutu olan karelerde hesaplanabiliyor. Hedef
kadrajdan çıkarsa o kareler hiç sayılmaz — yani HEDEFİ DAHA ÇOK KAYBEDEN
koşu DAHA AZ SALINIMLI görünür. Ölçüt, hedefi kaybetmeyi ödüllendirir.
Bu yüzden `temas_oran` HER ZAMAN yanında raporlanır ve %60'ın altındaysa
salınım değeri GÜVENİLMEZ damgası yer. Bir özelliği "salınımı azalttı"
diye kabul etmeden önce temas oranının DÜŞMEDİĞİ doğrulanmalıdır.

Kullanım:
    python3 tools/vurus_kalitesi.py <koşu_dizini> [<koşu_dizini> ...]
Koşu dizini `kacamak_testi.py` çıktısıdır (olay.json + kacamak.csv +
arşivlenmiş bbox_ibvs_*.csv + frames/).
"""
import csv
import glob
import json
import math
import os
import statistics as st
import sys

CX = 320.0
PENCERE = 2.0          # s; temas öncesi incelenen süre
OLU_BANT_PX = 25.0     # px; cx işaret değişimi ölü bandı
OLU_BANT_ROLL = 10.0   # °


def f(x, k):
    try:
        v = float(x[k])
        return v if v == v else None
    except (TypeError, ValueError, KeyError):
        return None


def isaret_degisim(vals, olu):
    s, onceki = 0, 0
    for v in vals:
        t = 1 if v > olu else (-1 if v < -olu else 0)
        if t and onceki and t != onceki:
            s += 1
        if t:
            onceki = t
    return s


def bbox_oku(d):
    r = []
    for y in sorted(glob.glob(os.path.join(d, "bbox_ibvs_*.csv"))):
        r += list(csv.DictReader(open(y)))
    return r


def olc(d):
    ad = os.path.basename(d.rstrip("/"))
    olay = json.load(open(os.path.join(d, "olay.json")))
    r = bbox_oku(d)
    if len(r) < 20:
        return {"ad": ad, "hata": "bbox arşivi yok/yetersiz"}

    kutulu = [x for x in r if (f(x, "boyut") or 0) >= 6.0]
    if len(kutulu) < 20:
        return {"ad": ad, "hata": "kutulu kare yetersiz"}

    # ── SALINIM (tüm görsel devir boyunca) ──
    sure = 0.0
    for a, b in zip(kutulu, kutulu[1:]):
        dt = (f(b, "t") or 0) - (f(a, "t") or 0)
        if 1e-3 < dt < 0.5:
            sure += dt
    sure = max(sure, 1e-6)
    cx_sap = [f(x, "cx") - CX for x in kutulu if f(x, "cx") is not None]
    roll = [f(x, "iris_roll_deg") or 0.0 for x in kutulu]
    yaw = [f(x, "yaw_cmd_deg") for x in kutulu]
    yaw_hiz = []
    for (a, ya), (b, yb) in zip(zip(kutulu, yaw), zip(kutulu[1:], yaw[1:])):
        dt = (f(b, "t") or 0) - (f(a, "t") or 0)
        if ya is None or yb is None or not (1e-3 < dt < 0.5):
            continue
        dyaw = (yb - ya + 180) % 360 - 180
        yaw_hiz.append(abs(dyaw) / dt)
    kesinti = sum(1 for x in r if x.get("durum") == "KUTU_YOK")

    # ⚠ TEMAS ORANI — salınım ölçütünün geçerlilik şartı (yukarıdaki nota bak)
    temas_oran = 100.0 * len(kutulu) / max(len(r), 1)

    o = {
        "ad": ad,
        "temas_oran": round(temas_oran, 0),
        "salinim_guvenilir": temas_oran >= 60.0,
        "imha": bool(olay.get("imha")),
        "en_yakin": olay.get("en_yakin"),
        "kacamak": olay.get("kacamak"),
        "cx_degisim_hz": round(isaret_degisim(cx_sap, OLU_BANT_PX) / sure, 3),
        "roll_degisim_hz": round(isaret_degisim(roll, OLU_BANT_ROLL) / sure, 3),
        "roll_p90": round(sorted(abs(v) for v in roll)[int(0.9 * (len(roll) - 1))], 0),
        "yaw_hiz_p90": (round(sorted(yaw_hiz)[int(0.9 * (len(yaw_hiz) - 1))], 0)
                        if yaw_hiz else None),
        "kesinti_kare": kesinti,
        "kutulu_kare": len(kutulu),
    }

    # ── VURUŞ SINIFI: temas öncesi son PENCERE saniyesi ──
    if not o["imha"]:
        o["sinif"] = "—"
        o["gerekce"] = "vuruş yok"
        return o
    son_t = f(kutulu[-1], "t")
    pen = [x for x in kutulu if son_t - (f(x, "t") or 0) <= PENCERE]
    ham_pen = [x for x in r if son_t - (f(x, "t") or 0) <= PENCERE]
    if len(pen) < 10:
        o["sinif"] = "ŞANS"
        o["gerekce"] = f"temas öncesi {PENCERE:.0f} s'de yalnız {len(pen)} kutulu kare"
        return o

    kopuk = sum(1 for x in ham_pen if x.get("durum") == "KUTU_YOK")
    pcx = [abs(f(x, "cx") - CX) for x in pen if f(x, "cx") is not None]
    pboyut = [f(x, "boyut") for x in pen if f(x, "boyut")]
    proll = [f(x, "iris_roll_deg") or 0.0 for x in pen]
    # boyut düzgün büyüyor mu: son çeyrek / ilk çeyrek ve geri-gidiş oranı
    n4 = max(1, len(pboyut) // 4)
    buyume = st.median(pboyut[-n4:]) / max(st.median(pboyut[:n4]), 1e-6)
    geri = sum(1 for a, b in zip(pboyut, pboyut[1:]) if b < a * 0.85)

    olcut = {
        "temas kesintisiz": (kopuk == 0, f"{kopuk} kopuk kare"),
        "hedef merkezde (medyan<90px)": (st.median(pcx) < 90.0,
                                         f"medyan {st.median(pcx):.0f} px"),
        "hedef kenarda değil (p90<180px)":
            (sorted(pcx)[int(0.9 * (len(pcx) - 1))] < 180.0,
             f"p90 {sorted(pcx)[int(0.9*(len(pcx)-1))]:.0f} px"),
        "kutu düzgün büyüyor (>1.5×)": (buyume > 1.5, f"{buyume:.2f}×"),
        "boyut sıçraması yok (<%15 kare)":
            (geri <= 0.15 * len(pboyut), f"{geri}/{len(pboyut)} geri gidiş"),
        "yatış sakin (işaret değişimi≤1)":
            (isaret_degisim(proll, OLU_BANT_ROLL) <= 1,
             f"{isaret_degisim(proll, OLU_BANT_ROLL)} değişim"),
    }
    kalan = [k for k, (ok, _) in olcut.items() if not ok]
    o["sinif"] = "KONTROLLÜ" if not kalan else "ŞANS"
    o["olcut"] = {k: (ok, det) for k, (ok, det) in olcut.items()}
    o["gerekce"] = ("tüm ölçütler geçti" if not kalan
                    else "kalan: " + ", ".join(kalan))
    return o


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
          f"{'SINIF':>11}{'temas %':>9}{'cx dğş/s':>10}{'roll p90':>10}"
          f"{'yaw °/s p90':>12}")
    print("─" * 104)
    for o in iyi:
        uyari = "" if o["salinim_guvenilir"] else " ⚠"
        print(f"{o['ad']:<22}{o['kacamak']:<8}{'✓' if o['imha'] else '✗':>7}"
              f"{o['en_yakin']:>9} m{o['sinif']:>11}{o['temas_oran']:>8.0f}%"
              f"{str(o['cx_degisim_hz']) + uyari:>10}{o['roll_p90']:>9.0f}°"
              f"{o['yaw_hiz_p90'] if o['yaw_hiz_p90'] is not None else '—':>12}")
    if any(not o["salinim_guvenilir"] for o in iyi):
        print("\n  ⚠ = temas oranı %60'ın altında; salınım değeri GÜVENİLMEZ "
              "(hedef kadrajda yokken salınım ölçülemez, sıfır görünür).")
    print()
    for o in iyi:
        if o.get("olcut"):
            print(f"  {o['ad']} → {o['sinif']}  ({o['gerekce']})")
            for k, (ok, det) in o["olcut"].items():
                print(f"      {'✓' if ok else '✗'} {k:<34} {det}")
    kon = [o for o in iyi if o["sinif"] == "KONTROLLÜ"]
    sans = [o for o in iyi if o["sinif"] == "ŞANS"]
    print(f"\n  KONTROLLÜ vuruş: {len(kon)}   ŞANS vuruşu: {len(sans)}   "
          f"vuruş yok: {sum(1 for o in iyi if not o['imha'])}")


if __name__ == "__main__":
    main()
