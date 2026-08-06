#!/usr/bin/env python3
"""
tools/zarf_olc.py — ARAÇ ZARFI KARAKTERİZASYONU (F0).

    python3 tools/zarf_olc.py            # en son gps güdüm logu
    python3 tools/zarf_olc.py 145230     # damgası 145230 olan log

NEDEN VAR: 2026-08-04'te ArduCopter parametrelerinin iki kez yanlış adla
yazıldığı ve aracın firmware varsayılanlarıyla uçtuğu ortaya çıktı.
Parametreler düzeltildi; düzeltmenin uçuşta karşılığı ÖLÇÜLMEDEN bilinemez.

⚠ ASIL DARBOĞAZ HIZ DEĞİL, DÖNÜŞ (2026-08-04 ölçümü):
İlk teşhis "araç 10 m/s tavanda kalıyor" idi ve YANLIŞTI — loglar aracın
17-18 m/s yaptığını gösterdi, yani WPNAV_SPEED komutu kırpmıyor. Bağlayan
kısıt WPNAV_ACCEL (yanal ivme) çıktı:
    13:28 dönen hedef : yanal ivme medyan 1.68, %99 2.74 m/s² → 2.5 tavanına
                        dayalı, örneklerin %28'i tavanda; hız medyan 12.3 m/s
    14:52 düz kovalama: yanal ivme medyan 0.26 → tavan hiç zorlanmadı;
                        hız medyan 17.3 m/s (komut 18)
Aritmetik: 15 m/s'de 9.8°/s dönmek 2.57 m/s² ister; 2.5 m/s² tavanı 9.5°/s'de
kesiyor. Hedef 7.1°/s medyan, %90'da 34.6°/s dönüyordu. Yani araç hedefin
DÖNÜŞÜNÜ takip edemiyordu — düzlükte kazandığını her virajda geri veriyordu
(63 dakikada menzil 42-147 m arasında salındı, hiç kapanmadı).

Bu yüzden bu aracın BAŞLIK METRİĞİ yanal ivmedir, tepe hız değil.

Kaynak tutarlı: mode_guided.cpp:295 — set_max_speed_accel_NE_cm(speed_xy_cms,
wp_nav->get_wp_acceleration_cmss()); ivme WPNAV_ACCEL'den gelir, hız ayrı
bir değişkenden.

Kendi hızımız CSV'de yok (yalnız komut var), bu yüzden konumdan sayısal türevle
çıkarılır. Türev gürültülüdür — bu yüzden medyan ve yüzdelikler kullanılır,
tepe değerlere GÜVENİLMEZ (GPS sıçraması 40+ m/s gösterebilir).
"""

import csv
import glob
import math
import os
import statistics as st
import sys

_KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOG = os.path.join(_KOK, "logs")

# Beklenen tavanlar (sim/ardupilot_params/avci_copter.parm — düzeltme sonrası)
BEKLENEN = {
    "yatay_hiz": 25.0,       # WPNAV_SPEED 2500 cm/s
    "yatay_ivme": 8.0,       # WPNAV_ACCEL 800 cm/s²
    "tirmanma": 6.0,         # WPNAV_SPEED_UP 600
    "inis": 4.0,             # WPNAV_SPEED_DN 400
}
# Düzeltme ÖNCESİ fiili tavanlar (firmware varsayılanı) — kıyas için
ONCESI = {"yatay_hiz": 10.0, "yatay_ivme": 2.5, "tirmanma": 2.5, "inis": 1.5}


def _f(satir, alan):
    try:
        return float(satir[alan])
    except (KeyError, TypeError, ValueError):
        return None


def _yuzde(dizi, p):
    if not dizi:
        return float("nan")
    s = sorted(dizi)
    return s[min(len(s) - 1, int(p * len(s)))]


def logu_bul(damga=None):
    adaylar = sorted(glob.glob(os.path.join(_LOG, "gps_guidance_*.csv")))
    if not adaylar:
        sys.exit("logs/ içinde gps_guidance_*.csv yok — önce bir uçuş yapın.")
    if damga:
        eslesen = [y for y in adaylar if damga in os.path.basename(y)]
        if not eslesen:
            sys.exit(f"'{damga}' damgalı log bulunamadı.")
        return eslesen[-1]
    return adaylar[-1]


def olc(yol):
    with open(yol) as f:
        g = list(csv.DictReader(f))
    if len(g) < 20:
        sys.exit(f"{os.path.basename(yol)}: çok kısa ({len(g)} satır) — "
                 "araç zarfını ölçmeye yetmez.")

    # ── Hız: UZUN TABANLI türev ──
    # Ardışık kareden türev almak (~50 ms) GPS sıçramalarını 40+ m/s'lik sahte
    # tepelere çevirir; %95'lik dilim bile kirlenir ve araç açılmamışken
    # "tavan açıldı" denir. ~0.5 s'lik tabanda gürültü kök-N ile bastırılır,
    # gerçek ivmeler (8 m/s² → 0.5 s'de 4 m/s) hâlâ görünür.
    TABAN_S = 0.5

    ornekler = []            # (t, x, y, z)
    komut_h = []
    for r in g:
        t = _f(r, "t")
        ix, iy, iz = _f(r, "iris_x"), _f(r, "iris_y"), _f(r, "iris_z")
        if None not in (t, ix, iy, iz):
            ornekler.append((t, ix, iy, iz))
        vx, vy = _f(r, "vx_cmd"), _f(r, "vy_cmd")
        if None not in (vx, vy):
            komut_h.append(math.hypot(vx, vy))

    hizlar = []              # (t, yatay_hiz, dikey_hiz, vx, vy)
    j = 0
    for i in range(len(ornekler)):
        while j < len(ornekler) and ornekler[j][0] - ornekler[i][0] < TABAN_S:
            j += 1
        if j >= len(ornekler):
            break
        dt = ornekler[j][0] - ornekler[i][0]
        if not (TABAN_S * 0.8 < dt < TABAN_S * 2.0):
            continue
        vxi = (ornekler[j][1] - ornekler[i][1]) / dt
        vyi = (ornekler[j][2] - ornekler[i][2]) / dt
        vh = math.hypot(vxi, vyi)
        vz = (ornekler[j][3] - ornekler[i][3]) / dt      # NED: +z aşağı
        if vh < 45 and abs(vz) < 25:
            hizlar.append((ornekler[i][0], vh, vz, vxi, vyi))

    gercek_h = [h[1] for h in hizlar]
    tirmanma = [-h[2] for h in hizlar if h[2] < -0.3]
    inis = [h[2] for h in hizlar if h[2] > 0.3]

    # ── YANAL İVME — bu aracın başlık metriği ──
    # Hız vektörünün dönme hızı × hız. Bağlayan kısıt buydu (bkz. modül başlığı).
    yanal, donus = [], []
    for i in range(1, len(hizlar)):
        dt = hizlar[i][0] - hizlar[i - 1][0]
        if not (0.02 < dt < 1.0):
            continue
        onc, sim = hizlar[i - 1], hizlar[i]
        if onc[1] < 3.0 or sim[1] < 3.0:    # dururken yön anlamsız
            continue
        a1 = math.atan2(onc[4], onc[3])
        a2 = math.atan2(sim[4], sim[3])
        d = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
        w = abs(d) / dt                 # rad/s
        if w * sim[1] < 30:
            donus.append(math.degrees(w))
            yanal.append(w * sim[1])    # m/s²

    return {
        "yanal": yanal,
        "donus": donus,
        "dosya": os.path.basename(yol),
        "satir": len(g),
        "komut_h": komut_h,
        "gercek_h": gercek_h,
        "tirmanma": tirmanma,
        "inis": inis,
    }


def _rapor_satiri(ad, olculen, beklenen, oncesi, birim="m/s", asgari_ornek=15):
    if len(olculen) < asgari_ornek:
        print(f"  {ad:<22} YETERSİZ VERİ ({len(olculen)} örnek) — bu eksende "
              f"araç zorlanmamış, ölçüm sonuçsuz")
        return None
    p95 = _yuzde(olculen, 0.95)
    med = st.median(olculen)
    # Karar p95'e göre: medyan "ne kadar hızlı uçtuk"u değil "genelde ne
    # istendi"yi yansıtır; tavanın açılıp açılmadığını üst yüzdelik gösterir.
    if p95 >= beklenen * 0.8:
        durum = "✓ TAVAN AÇILDI"
    elif p95 >= oncesi * 1.3:
        durum = "~ KISMEN"
    else:
        durum = "✗ HÂLÂ ESKİ TAVANDA"
    print(f"  {ad:<22} medyan {med:6.2f}  %95 {p95:6.2f} {birim}   "
          f"(önce {oncesi:.1f} → hedef {beklenen:.1f})   {durum}")
    return p95


def main():
    damga = sys.argv[1] if len(sys.argv) > 1 else None
    yol = logu_bul(damga)
    d = olc(yol)

    print("=" * 74)
    print(f"ARAÇ ZARFI KARAKTERİZASYONU — {d['dosya']}  ({d['satir']} satır)")
    print("=" * 74)
    print("\nKendi hızımız konumdan türetildi (CSV'de yok). Tepe değerlere")
    print("güvenmeyin — GPS sıçraması şişirir; karar %95'lik dilime göredir.\n")

    if d["komut_h"]:
        print(f"  {'komut edilen yatay':<22} medyan {st.median(d['komut_h']):6.2f}  "
              f"%95 {_yuzde(d['komut_h'], 0.95):6.2f} m/s   (güdümün istediği)")
    print()

    # BAŞLIK METRİĞİ önce: bağlayan kısıt buydu.
    p95_a = _rapor_satiri("YANAL İVME  ★", d["yanal"],
                          BEKLENEN["yatay_ivme"], ONCESI["yatay_ivme"], "m/s²")
    if d["donus"]:
        print(f"  {'dönüş hızı':<22} medyan {st.median(d['donus']):6.2f}  "
              f"%95 {_yuzde(d['donus'], 0.95):6.2f} °/s   "
              f"(hedef 7-35 °/s dönüyor)")
    p95_h = _rapor_satiri("GERÇEKLEŞEN yatay hız", d["gercek_h"],
                          BEKLENEN["yatay_hiz"], ONCESI["yatay_hiz"])
    _rapor_satiri("GERÇEKLEŞEN tırmanma", d["tirmanma"],
                  BEKLENEN["tirmanma"], ONCESI["tirmanma"])
    _rapor_satiri("GERÇEKLEŞEN iniş", d["inis"],
                  BEKLENEN["inis"], ONCESI["inis"])

    print("\n" + "-" * 74)
    # ⚠ KARAR MANTIĞI: "tavana ulaşılmadı" ile "tavan yok" AYRI şeylerdir.
    # Bir uçuş dönüş tavanını hiç zorlamadıysa ölçüm SONUÇSUZDUR — "açılmadı"
    # demek yanlış olur. Ayrım noktası eski tavan (2.5 m/s²): onu AŞMIŞSAK yeni
    # tavan kesin aktiftir, aşmamışsak uçuş yeterince agresif değildi.
    if p95_a is None:
        print("KARAR: ⊘ SONUÇSUZ — yanal ivme ölçülemedi, uçuşta dönüş yok.")
        print("       Hedefin DESEN uçtuğu bir koşu gerekir.")
    elif ONCESI["yatay_ivme"] * 1.1 < p95_a < 5.0:
        print(f"KARAR: ⊘ SONUÇSUZ ama OLUMLU İŞARET. Yanal ivme {p95_a:.2f} m/s²,")
        print(f"       ESKİ tavanı ({ONCESI['yatay_ivme']:.1f}) aşmış → yeni tavan AKTİF.")
        print("       Ama 8 m/s²'ye kadar zorlanmamış: uçuş yeterince agresif değildi")
        print("       ya da hedef keskin manevra yapmadı. Zarfın üst ucu ölçülmedi.")
        if p95_h:
            print(f"       frpn.Cfg.V_C_MAX önerisi: {min(22.0, p95_h * 0.95):.0f} m/s")
    elif p95_a <= ONCESI["yatay_ivme"] * 1.1:
        print(f"KARAR: ⊘ SONUÇSUZ — yanal ivme {p95_a:.2f} m/s², eski tavanın "
              f"({ONCESI['yatay_ivme']:.1f}) altında.")
        print("       Uçuş dönüş tavanını HİÇ zorlamamış; bu bir başarısızlık")
        print("       kanıtı DEĞİL, ölçüm yapılamadı demek. Hedef desen uçmalı.")
    elif p95_a >= 5.0:
        print("KARAR: ✓ DÖNÜŞ ZARFI AÇILDI. Asıl darboğaz olan yanal ivme artık")
        print(f"       {p95_a:.1f} m/s²'ye çıkabiliyor (eskiden 2.5'te kesiliyordu).")
        print(f"       15 m/s'de mümkün dönüş: {math.degrees(p95_a / 15.0):.0f} °/s")
        print("       F3 katsayı ayarı bu zarfa göre yapılabilir.")
        if p95_h:
            print(f"       frpn.Cfg.V_C_MAX önerisi: {min(22.0, p95_h * 0.95):.0f} m/s")
    elif p95_a >= 3.2:
        print("KARAR: ~ KISMEN açıldı. Dönüş yetkisi arttı ama hedefin keskin")
        print("       manevralarına (%90'da 34.6 °/s) hâlâ yetişmeyebilir.")
        print("       WPNAV_ACCEL'i 800 → 1000 denemeyi düşünün (TEK değişken).")
    else:
        print("KARAR: ✗ DÖNÜŞ ZARFI AÇILMADI — parametre uygulanmamış olabilir.")
        print("       KONTROL: grep -iE '^WPNAV_ACCEL' ~/ardupilot/mav_5_1.parm")
        print("       Çıktı 250 ise param dosyası SITL'e hiç geçmemiş demektir.")
    print("-" * 74)
    print("\nYALPALAMA kontrolü ayrı: Gazebo'da araca bakın ve MP'de tutum")
    print("grafiğine bakın. Salınım varsa geri çekme sırası:")
    print("  1) WPNAV_JERK 4 → 2      2) ANGLE_MAX 4500 → 3500")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
