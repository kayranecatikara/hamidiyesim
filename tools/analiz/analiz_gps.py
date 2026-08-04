#!/usr/bin/env python3
"""
tools/analiz/analiz_gps.py — GPS fazının (gps_guidance.py) performans analizi.

ANA SORU: GPS fazı hedefi kovalayıp görsel faza devredebiliyor mu?

Devir için menzilin `supervisor.SupCfg.GATE_MENZIL` (20 m) altına inmesi VE
orada KALMASI gerekir. Bu script menzilin neden kapanmadığını ayrıştırır:

  1. Yasa yanlış yeri mi gösteriyor?      → komut yönü ↔ istasyon noktası açısı
  2. Komut uygulanıyor mu?                → GERÇEKLEŞEN / KOMUT hız oranı
  3. Avcı hedeften hızlı mı?              → iki aracın gerçek yer hızı
  4. Dikey kanal sağlam mı?               → istasyon irtifa hatası
  5. Doygunluk var mı?                    → komutun V_MAX tavanında geçen oranı

2026-08-01'de bu ayrıştırma `V_MAX = 28`'in doygunluk patolojisini ortaya
çıkardı: yasa doğruydu (komut yönü 8.3° hatayla istasyonu gösteriyordu) ama
komutun yalnız %24'ü gerçekleşiyordu ve avcı UZAKTAYKEN YAKINDAKİNDEN yavaştı.
Bkz. docs/GUDUM_ARIZA_ZINCIRI_20260801.md.

Hız ölçümü CSV'deki `iris_x/iris_y` ve `tgt_x/tgt_y` konumlarının türevidir
(pencere tabanlı, gürültüye karşı). `tgt_vx/tgt_vy` kolonu gps_guidance'ın
EMA-filtreli KESTİRİMİdir — karşılaştırma için ayrıca yazdırılır.

Kullanım:
    python3 -m tools.analiz.analiz_gps                 # en yeni gps_guidance CSV
    python3 -m tools.analiz.analiz_gps logs/gps_*.csv  # belirli dosya(lar)
Salt okuma — simülasyona bağlanmaz.
"""

import glob
import math
import os
import sys

from tools.analiz.ortak import (
    LOG_DIZIN, MIN_SATIR, altbaslik, baslik, ist, ist_yaz, kolon, yukle,
)

# Görsel devir kapısı (supervisor.SupCfg.GATE_MENZIL ile aynı olmalı)
GATE_MENZIL = 20.0
# Hız türevi penceresi (s). Telemetri gürültüsünü söndürecek kadar uzun,
# manevrayı kaçırmayacak kadar kısa.
HIZ_PENCERE_S = 1.0


def _sayilar(satirlar, ad):
    """kolon() None'lu liste döndürür (eski loglarda yeni kolonlar boştur).
    Üzerinde karşılaştırma yapacaksak None'ları ELEMEK zorundayız — yoksa
    TypeError analizi ortasından keser."""
    return [s[ad] for s in satirlar
            if isinstance(s.get(ad), (int, float))]


def _dosyalar(argv):
    yollar = [a for a in argv if a.endswith(".csv")]
    if yollar:
        return yollar
    hepsi = sorted(glob.glob(os.path.join(LOG_DIZIN, "gps_guidance_*.csv")))
    if not hepsi:
        print(f"HATA: {LOG_DIZIN} altında gps_guidance_*.csv yok.\n"
              "Önce bir chase uçuşu yapılmalı (bkz. CLAUDE.md §3).")
        sys.exit(1)
    return [hepsi[-1]]


def _pencere_hizi(satirlar, kx, ky, pencere=HIZ_PENCERE_S):
    """Konum türevinden yer hızı listesi (m/s). Pencere tabanlı: ardışık kare
    farkı 20 Hz'te birkaç santimetre olur ve telemetri yuvarlaması baskın gelir."""
    out = []
    i = 0
    n = len(satirlar)
    while i < n - 1:
        t0 = satirlar[i].get("t")
        x0, y0 = satirlar[i].get(kx), satirlar[i].get(ky)
        j = i
        while j < n - 1 and (satirlar[j].get("t") or 0) - (t0 or 0) < pencere:
            j += 1
        t1 = satirlar[j].get("t")
        x1, y1 = satirlar[j].get(kx), satirlar[j].get(ky)
        i = j
        if None in (t0, t1, x0, y0, x1, y1) or (t1 - t0) < pencere * 0.5:
            continue
        out.append(math.hypot(x1 - x0, y1 - y0) / (t1 - t0))
    return out


def _komut_yon_hatasi(satirlar):
    """Komut hız vektörü ile (istasyon − avcı) vektörü arasındaki açı (derece).
    0'a yakınsa yasa doğru yeri gösteriyor demektir."""
    out = []
    for s in satirlar:
        sx, sy = s.get("st_x"), s.get("st_y")
        ix, iy = s.get("iris_x"), s.get("iris_y")
        vx, vy = s.get("vx_cmd"), s.get("vy_cmd")
        if None in (sx, sy, ix, iy, vx, vy):
            continue
        ex, ey = sx - ix, sy - iy
        ne, nv = math.hypot(ex, ey), math.hypot(vx, vy)
        if ne < 1e-6 or nv < 1e-6:
            continue
        kos = max(-1.0, min(1.0, (ex * vx + ey * vy) / (ne * nv)))
        out.append(math.degrees(math.acos(kos)))
    return out


def _gerceklesen_komut_orani(satirlar, menzil_alt=None):
    """GERÇEKLEŞEN hız / KOMUT hız. 1.0 = komut tam uygulanıyor.
    menzil_alt verilirse yalnız o menzilin ÜSTÜNDEKİ kareler (tam gaz beklenen
    bölge) kullanılır."""
    out = []
    i = 0
    n = len(satirlar)
    while i < n - 1:
        s = satirlar[i]
        t0, x0, y0 = s.get("t"), s.get("iris_x"), s.get("iris_y")
        vx, vy, mz = s.get("vx_cmd"), s.get("vy_cmd"), s.get("menzil")
        j = i
        while j < n - 1 and (satirlar[j].get("t") or 0) - (t0 or 0) < HIZ_PENCERE_S:
            j += 1
        t1, x1, y1 = satirlar[j].get("t"), satirlar[j].get("iris_x"), satirlar[j].get("iris_y")
        i = j
        if None in (t0, t1, x0, y0, x1, y1, vx, vy):
            continue
        if menzil_alt is not None and (mz is None or mz <= menzil_alt):
            continue
        komut = math.hypot(vx, vy)
        if komut < 1.0 or (t1 - t0) < HIZ_PENCERE_S * 0.5:
            continue
        out.append(math.hypot(x1 - x0, y1 - y0) / (t1 - t0) / komut)
    return out


def _dosya_ozeti(yol, satirlar):
    ad = os.path.basename(yol)
    ts = [s.get("t") for s in satirlar if s.get("t") is not None]
    sure = (max(ts) - min(ts)) if len(ts) >= 2 else 0.0
    durumlar = {}
    for s in satirlar:
        d = s.get("durum") or "(boş)"
        durumlar[d] = durumlar.get(d, 0) + 1
    print(f"\n▸ {ad}  —  {len(satirlar)} kare, {sure:.0f} s")
    sirali = sorted(durumlar.items(), key=lambda kv: -kv[1])
    print("   durum: " + "  ".join(f"{k}={v}" for k, v in sirali))
    kilit = durumlar.get("KILIT", 0)
    if kilit == 0:
        print("   ⚠ Hiç KILIT karesi yok — menzil devir bandına (%.0f m) HİÇ inmedi."
              % GATE_MENZIL)
    return sure


def analiz(yol, satirlar):
    sure = _dosya_ozeti(yol, satirlar)

    # ── 1) Menzil: kapandı mı ──
    altbaslik("MENZİL — devir kapısı açıldı mı")
    mz = [s.get("menzil") for s in satirlar]
    ist_yaz("menzil", ist(mz), " m")
    ist_yaz("d_h (yatay)", ist(kolon(satirlar, "d_h")), " m")
    gecerli = [m for m in mz if m is not None]
    if gecerli:
        alti = sum(1 for m in gecerli if m < GATE_MENZIL)
        print(f"  menzil < {GATE_MENZIL:.0f} m olan kare: {alti}/{len(gecerli)} "
              f"(%{100.0 * alti / len(gecerli):.1f})")

    # ── 2) Hız dengesi: avcı hedeften hızlı mı ──
    altbaslik("HIZ DENGESİ — avcı hedeften hızlı mı (kapanmanın ön koşulu)")
    ih = _pencere_hizi(satirlar, "iris_x", "iris_y")
    th = _pencere_hizi(satirlar, "tgt_x", "tgt_y")
    ist_yaz("avcı gerçek yer hızı", ist(ih), " m/s")
    ist_yaz("hedef gerçek yer hızı", ist(th), " m/s")
    ist_yaz("hedef hızı (CSV kestirimi)", ist(
        [math.hypot(s.get("tgt_vx") or 0.0, s.get("tgt_vy") or 0.0) for s in satirlar]),
        " m/s")
    if ih and th:
        fark = (sum(ih) / len(ih)) - (sum(th) / len(th))
        durum = "avcı HIZLI ✓" if fark > 0.5 else (
            "marj YOK ✗ — menzil kapanamaz" if fark < 0.5 else "sınırda")
        print(f"  → ortalama hız farkı: {fark:+.1f} m/s   {durum}")

    # ── 3) Komut uygulanıyor mu (doygunluk patolojisi testi) ──
    altbaslik("KOMUT ↔ GERÇEKLEŞEN — doygunluk patolojisi var mı")
    kv = [math.hypot(s.get("vx_cmd") or 0.0, s.get("vy_cmd") or 0.0) for s in satirlar]
    ist_yaz("komut |v_yatay|", ist(kv), " m/s")
    if kv:
        tavan = max(kv)
        doygun = sum(1 for v in kv if v > tavan * 0.98)
        print(f"  komut tavanda (>{tavan * 0.98:.1f}): {doygun}/{len(kv)} "
              f"(%{100.0 * doygun / len(kv):.0f})")
    tum = _gerceklesen_komut_orani(satirlar)
    uzak = _gerceklesen_komut_orani(satirlar, menzil_alt=50.0)
    if tum:
        print(f"  GERÇEKLEŞEN/KOMUT (tümü)        med={sorted(tum)[len(tum)//2]:.2f}"
              "   (1.0 = komut tam uygulanıyor)")
    if uzak:
        print(f"  GERÇEKLEŞEN/KOMUT (menzil>50 m) med={sorted(uzak)[len(uzak)//2]:.2f}"
              "   ← tam gaz gitmesi gereken bölge")
    if tum and uzak:
        m_t, m_u = sorted(tum)[len(tum) // 2], sorted(uzak)[len(uzak) // 2]
        if m_u < m_t - 0.05:
            print("  ⚠ UZAKTAYKEN YAKINDAKİNDEN KÖTÜ — doygunluk patolojisi imzası.")
            print("    Hız hatası × PSC_VELXY_P, ulaşılabilir ivmeyi aşıyor;")
            print("    attitude tavana yapışıp yatış limit çevrimine giriyor.")
            print("    Çare: V_MAX'ı airframe'in sürdürebildiği banda çek.")

    # ── 4) Yasa doğru yeri mi gösteriyor ──
    altbaslik("YASA — komut yönü istasyon noktasını gösteriyor mu")
    ay = _komut_yon_hatasi(satirlar)
    ist_yaz("komut yönü hatası", ist(ay), "°")
    if ay:
        iyi = sum(1 for a in ay if a < 30.0)
        print(f"  30° içinde: %{100.0 * iyi / len(ay):.0f}   "
              "(yüksekse yasa/çerçeve sorunu, düşükse yasa SAĞLAM)")

    # ── 5) Dikey kanal ──
    altbaslik("DİKEY KANAL — istasyon irtifası tutuluyor mu")
    dz = [abs(s["iris_z"] - s["st_z"]) for s in satirlar
          if s.get("iris_z") is not None and s.get("st_z") is not None]
    ist_yaz("|iris_z − st_z|", ist(dz), " m")
    irt = [-s["iris_z"] for s in satirlar if s.get("iris_z") is not None]
    if irt:
        print(f"  avcı irtifası: min={min(irt):.1f} m  max={max(irt):.1f} m")
        if min(irt) < 3.0:
            print("  ⚠ Avcı yere değmiş — çakılma ya da hedefi yere kadar takip.")

    # ── 6) Kadraj (merkezleme başarısı) ──
    altbaslik("KADRAJ — hedef kameranın ortasında mı (GPS fazının asıl görevi)")
    ist_yaz("kadraj yaw hatası", ist(kolon(satirlar, "kadraj_yaw_deg")), "°")
    ist_yaz("kadraj yükseliş", ist(kolon(satirlar, "kadraj_elev_deg")), "°")
    print("  (yükseliş hedefi = kamera tilt'i 25°; yaw hedefi = 0°)")
    ky = _sayilar(satirlar, "kuyruk_aci_deg")
    if ky:
        ist_yaz("kuyruk açısı", ist(ky), "°")
        arkada = sum(1 for a in ky if a <= 60.0)
        print(f"  kuyrukta (≤60°): %{100.0 * arkada / len(ky):.0f}   "
              "(0° = tam arkasında; devir kapısı bunu ister)")

    # ── 7) DÖNÜŞ TUZAĞI — saf takip mi kesme mi ──
    # 2026-08-04'te bulundu. Menzil kapanmamasının sebebi "avcı yavaş" DEĞİL:
    # avcı düz hatta 19 m/s yapabiliyor (gps_guidance_20260801_17{0837,3359}),
    # ama daire uçan hedefi saf takip ederken 8.3 m/s'ye düşüyor.
    # Sebep geometrik: istasyon noktası hedefle birlikte YÖRÜNGEDE dönüyor,
    # komut vektörü sürekli döndüğü için avcı düz gidemiyor. Dönen bir hız
    # vektörünü sürdürmenin bedeli yanal ivmedir:  v = a_yanal / omega_komut.
    # Ölçüm (173612): eğim medyanı 14.6° → a_yanal = g·tan(14.6°) = 2.55 m/s²,
    # omega_komut medyanı 15 °/s = 0.26 rad/s → v = 9.8 m/s. Gerçekleşen 9.4.
    # Birebir tutuyor: avcı hız değil DÖNÜŞ sınırlı.
    altbaslik("DÖNÜŞ TUZAĞI — komut yönü ne kadar hızlı dönüyor")
    don = []
    for a, b in zip(satirlar, satirlar[1:]):
        dt = (b.get("t") or 0) - (a.get("t") or 0)
        if not (0.02 < dt < 0.5):
            continue
        for s in (a, b):
            if s.get("vx_cmd") is None or s.get("vy_cmd") is None:
                break
        else:
            ca = math.degrees(math.atan2(a["vy_cmd"], a["vx_cmd"]))
            cb = math.degrees(math.atan2(b["vy_cmd"], b["vx_cmd"]))
            don.append(abs((cb - ca + 180.0) % 360.0 - 180.0) / dt)
    ist_yaz("komut yönü dönme hızı", ist(don), "°/s")
    egim = _sayilar(satirlar, "iris_egim_deg")
    if not egim:
        # Eski loglarda iris_egim_deg yok ama roll/pitch var — türetilebilir.
        # cos(eğim) = cos(roll)·cos(pitch) (itki ekseninin düşeyle açısı).
        egim = [math.degrees(math.acos(max(-1.0, min(1.0,
                math.cos(math.radians(s["iris_roll_deg"]))
                * math.cos(math.radians(s["iris_pitch_deg"]))))))
                for s in satirlar
                if isinstance(s.get("iris_roll_deg"), (int, float))
                and isinstance(s.get("iris_pitch_deg"), (int, float))]
    if egim:
        ist_yaz("avcı toplam eğim", ist(egim), "°")
    ist_yaz("t_go (kesme öngörüsü)", ist(kolon(satirlar, "t_go_s")), " s")
    ist_yaz("hedef dönüş hızı (omega)", ist(kolon(satirlar, "tgt_omega_deg")), "°/s")
    if don and egim:
        w_med = sorted(don)[len(don) // 2]
        e_med = sorted(egim)[len(egim) // 2]
        a_yanal = 9.81 * math.tan(math.radians(e_med))
        if w_med > 1.0:
            v_tavan = a_yanal / math.radians(w_med)
            print(f"  → dönüş sınırlı hız tavanı = a_yanal/omega = "
                  f"{a_yanal:.2f}/{math.radians(w_med):.3f} = {v_tavan:.1f} m/s")
            gv = _pencere_hizi(satirlar, "iris_x", "iris_y")
            if gv:
                ger = sorted(gv)[len(gv) // 2]
                if abs(ger - v_tavan) < 0.35 * max(ger, 1.0):
                    print(f"    GERÇEKLEŞEN medyan {ger:.1f} m/s bu tavanla "
                          "uyuşuyor → DÖNÜŞ TUZAĞI ✗")
                    print("    Çare hız değil GEOMETRİ: istasyonu hedefin ANLIK "
                          "konumuna değil")
                    print("    ÖNGÖRÜLEN konumuna kur (Cfg.KESME) — komut yönü "
                          "yavaş döner, avcı düz gider.")
                else:
                    print(f"    GERÇEKLEŞEN medyan {ger:.1f} m/s tavandan uzak "
                          "→ dönüş tuzağı BASKIN DEĞİL")

    # ── 8) EKF ↔ konum türevi (hız ölçümü kimin doğru) ──
    ekf = _sayilar(satirlar, "iris_hiz")
    if ekf:
        altbaslik("HIZ ÖLÇÜMÜ — ArduPilot ne diyor, konum türevi ne diyor")
        ist_yaz("avcı hızı (ArduPilot/EKF)", ist(ekf), " m/s")
        gv = _pencere_hizi(satirlar, "iris_x", "iris_y")
        if gv:
            e_med, g_med = sorted(ekf)[len(ekf) // 2], sorted(gv)[len(gv) // 2]
            fark = abs(e_med - g_med)
            print(f"  EKF medyanı {e_med:.2f} m/s ↔ konum türevi {g_med:.2f} m/s "
                  f"(fark {fark:.2f})")
            if fark > 2.0:
                print("  ⚠ İKİSİ UYUŞMUYOR — ArduPilot kendi hızını yanlış biliyor.")
                print("    Hız kontrolcüsü hatayı GÖREMEZ; eğim küçük kalır, "
                      "komut hiç uygulanmaz.")
                print("    Bu bir GÜDÜM ayarı sorunu DEĞİL, EKF/telemetri sorunudur.")
            else:
                print("  ✓ Uyuşuyorlar — hız ölçümü güvenilir, sorun başka yerde.")

    return sure


def main(argv):
    baslik("GPS FAZI — menzil neden kapanmıyor / kapandı mı?")
    bulundu = False
    for yol in _dosyalar(argv):
        satirlar = yukle(yol)
        if len(satirlar) < MIN_SATIR:
            print(f"  ATLANDI  {os.path.basename(yol)} — {len(satirlar)} satır")
            continue
        analiz(yol, satirlar)
        bulundu = True
    if not bulundu:
        print("HATA: analiz edilebilir GPS logu yok.")
        return 1
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
