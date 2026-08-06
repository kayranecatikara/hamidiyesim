"""
tests/test_frpn.py — FRPN hız formu yasasının kabul kriterleri (F1).

Gazebo'suz, MAVLink'siz, saf matematik. Kullanım: python3 -m tests.test_frpn

Kapsam:
  P1-P4   çarpışma üçgeni: manevrasız hedefte ZEM ÖZDEŞ SIFIR (türetmenin kanıtı)
  P5-P8   geometri sağduyusu: kuyruk / kafa kafaya / dik kesme / çapraz
  P9-P12  sayısal koruma: ‖Δv‖→0, ‖Δp‖→0, t_go kırpma, ZEM tavanı
  P13-P16 sanal hedef: sabit açı, kuyruk yönü, yavaş hedef, yer koruması
  P17-P19 dejenerasyon: istasyonda söner, kapanma tavanı, yumuşak duruş
  P20     makale katsayılarının bizde neden kullanılmadığının regresyon kaydı
"""

import math

from control.guidance import frpn

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def _n(v):
    return math.sqrt(sum(x * x for x in v))


def main():
    print("FRPN hız formu — kabul kriterleri")
    print("=" * 60)
    C = frpn.Cfg

    # ══════════════════════════════════════════════════════════════════
    # P1-P4 · ÇARPIŞMA ÜÇGENİ
    # Türetmenin özü: v_cmd = v_hedef + V_c·û uygulanınca ZEM özdeş 0 olmalı.
    # Testi "yasayı uygula, sonucu geri besle, ZEM'e bak" diye kuruyoruz.
    # ══════════════════════════════════════════════════════════════════
    print("\n── P1-P4: çarpışma üçgeni (manevrasız hedefte ZEM ≡ 0) ──")

    senaryolar = [
        ("P1  kuyruktan (hedef uzaklaşıyor)", (100.0, 0.0, 0.0), (16.0, 0.0, 0.0)),
        ("P2  dik kesme (hedef yandan geçiyor)", (100.0, 0.0, 0.0), (0.0, 16.0, 0.0)),
        ("P3  çapraz + dikey bileşen", (80.0, 60.0, -20.0), (10.0, -12.0, 2.0)),
        ("P4  kafa kafaya (hedef üstümüze geliyor)", (150.0, 0.0, 0.0), (-18.0, 0.0, 0.0)),
    ]
    for ad, dp, v_hedef in senaryolar:
        # Çarpışma üçgeninin kendisi: v_drone = v_hedef + V_c·û
        u, ndp = frpn._birim(dp)
        v_c = min(max(C.K_C * ndp, C.V_C_MIN), C.V_C_MAX)
        v_drone = tuple(v_hedef[i] + v_c * u[i] for i in range(3))
        dv = tuple(v_hedef[i] - v_drone[i] for i in range(3))
        r = frpn.komut(dp, dv, v_hedef, C)
        kontrol(ad, r["zem_norm"] < 1e-6,
                f"‖ZEM‖={r['zem_norm']:.2e} m (t_go={r['t_go']:.2f}s)")

    # Aynı durumda 3. terim de sıfır olmalı → komut saf üçgen çözümü
    dp = (100.0, 0.0, 0.0); v_hedef = (16.0, 0.0, 0.0)
    u, ndp = frpn._birim(dp)
    v_c = min(max(C.K_C * ndp, C.V_C_MIN), C.V_C_MAX)
    v_drone = tuple(v_hedef[i] + v_c * u[i] for i in range(3))
    dv = tuple(v_hedef[i] - v_drone[i] for i in range(3))
    r = frpn.komut(dp, dv, v_hedef, C)
    kontrol("P4b düzeltme terimi çarpışma rotasında susuyor",
            _n(r["terim_zem"]) < 1e-6, f"‖terim_zem‖={_n(r['terim_zem']):.2e}")
    kontrol("P4c komut = hedef hızı + kapanma",
            abs(r["v_cmd"][0] - (16.0 + v_c)) < 1e-9 and abs(r["v_cmd"][1]) < 1e-9,
            f"v_cmd={tuple(round(x,3) for x in r['v_cmd'])}")

    # ══════════════════════════════════════════════════════════════════
    # P5-P8 · GEOMETRİ SAĞDUYUSU
    # ══════════════════════════════════════════════════════════════════
    print("\n── P5-P8: geometri sağduyusu ──")

    # Duran hedef, 50 m kuzeyde, drone duruyor → komut düpedüz kuzeye
    r = frpn.komut((50.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), C)
    kontrol("P5  duran hedef → komut hedefe doğru",
            r["v_cmd"][0] > 0 and abs(r["v_cmd"][1]) < 1e-9 and abs(r["v_cmd"][2]) < 1e-9,
            f"v_cmd={tuple(round(x,2) for x in r['v_cmd'])}")

    # Hedef yandan geçiyor, drone duruyor → komutun YANAL bileşeni olmalı
    # (saf takip burada doğrudan hedefe bakardı; kesme öne nişan alır)
    dp = (100.0, 0.0, 0.0); v_hedef = (0.0, 20.0, 0.0); v_drone = (0.0, 0.0, 0.0)
    dv = tuple(v_hedef[i] - v_drone[i] for i in range(3))
    r = frpn.komut(dp, dv, v_hedef, C)
    kontrol("P6  yandan geçen hedefte komut ÖNE nişan alıyor",
            r["v_cmd"][1] > 5.0,
            f"doğu bileşeni={r['v_cmd'][1]:.2f} m/s (saf takipte 0 olurdu)")

    # Aynı geometride ZEM sıfır DEĞİL (çünkü drone duruyor, üçgende değil)
    kontrol("P7  üçgende olmayan geometride ZEM > 0",
            r["zem_norm"] > 1.0, f"‖ZEM‖={r['zem_norm']:.1f} m")

    # Hedef bize doğru geliyorsa t_go kısalır
    r_yak = frpn.komut((100.0, 0.0, 0.0), (-20.0, 0.0, 0.0), (-20.0, 0.0, 0.0), C)
    r_uzak = frpn.komut((100.0, 0.0, 0.0), (5.0, 0.0, 0.0), (5.0, 0.0, 0.0), C)
    kontrol("P8  yaklaşan hedefte t_go, uzaklaşandan kısa",
            r_yak["t_go"] < r_uzak["t_go"],
            f"yaklaşan={r_yak['t_go']:.1f}s < uzaklaşan={r_uzak['t_go']:.1f}s")

    # ══════════════════════════════════════════════════════════════════
    # P9-P12 · SAYISAL KORUMA
    # ══════════════════════════════════════════════════════════════════
    print("\n── P9-P12: sayısal koruma (NaN/patlama yok) ──")

    r = frpn.komut((30.0, 0.0, 0.0), (0.0, 0.0, 0.0), (15.0, 0.0, 0.0), C)
    sonlu = all(math.isfinite(x) for x in r["v_cmd"])
    kontrol("P9  ‖Δv‖=0 (hız eşleşmiş) → t_go tavanda, komut sonlu",
            sonlu and r["t_go"] == C.T_GO_MAX,
            f"t_go={r['t_go']:.1f}s v_cmd={tuple(round(x,2) for x in r['v_cmd'])}")

    r = frpn.komut((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (15.0, 0.0, 0.0), C)
    kontrol("P10 ‖Δp‖=0 (istasyona vardık) → sıfır bölme yok",
            all(math.isfinite(x) for x in r["v_cmd"]),
            f"v_cmd={tuple(round(x,2) for x in r['v_cmd'])}")

    # Çok yakın + çok hızlı: t_go alt sınıra dayanmalı
    r = frpn.komut((0.5, 0.0, 0.0), (-30.0, 0.0, 0.0), (0.0, 0.0, 0.0), C)
    kontrol("P11 çok yakın+hızlı → t_go alt sınırda kırpıldı",
            r["t_go"] == C.T_GO_MIN, f"t_go={r['t_go']:.2f}s")

    # Büyük ZEM: düzeltme kendi tavanını aşmamalı
    r = frpn.komut((200.0, 0.0, 0.0), (0.0, 60.0, 0.0), (0.0, 60.0, 0.0), C)
    kontrol("P12 büyük ZEM → düzeltme kendi tavanında kalıyor",
            _n(r["terim_zem"]) <= C.V_ZEM_MAX + 1e-9 and r["zem_kirpildi"],
            f"‖terim_zem‖={_n(r['terim_zem']):.2f} ≤ {C.V_ZEM_MAX}")

    # ══════════════════════════════════════════════════════════════════
    # P13-P16 · SANAL HEDEF (kadraj istasyonu)
    # ══════════════════════════════════════════════════════════════════
    print("\n── P13-P16: sanal hedef (kadraj istasyonu) ──")

    p_h = (0.0, 0.0, -50.0)
    v_h = (16.0, 0.0, 0.0)                       # doğuya değil kuzeye uçuyor

    # UZAKTA ofset OLMAMALI — saf kesme (mimari revizyonun özü)
    p_d = (-500.0, 0.0, -50.0)
    p_s, tani = frpn.sanal_hedef(p_h, v_h, p_d, C)
    kontrol("P13 uzakta ofset YOK → nişan doğrudan hedefte (saf kesme)",
            tani["gecis_w"] == 0.0
            and abs(p_s[0] - p_h[0]) < 1e-9 and abs(p_s[2] - p_h[2]) < 1e-9,
            f"menzil={tani['menzil_gercek']:.0f}m w={tani['gecis_w']:.2f} "
            f"(GECIS_BASLA={C.GECIS_BASLA:.0f}m)")

    # YAKINDA tam istasyon — devir geometrisi
    p_d = (-20.0, 0.0, -50.0)
    p_s, tani = frpn.sanal_hedef(p_h, v_h, p_d, C)
    kontrol("P13b yakında tam istasyon: kuyrukta + altta",
            tani["gecis_w"] == 1.0 and p_s[0] < p_h[0] and p_s[2] > p_h[2],
            f"menzil={tani['menzil_gercek']:.0f}m arka={tani['d_arka']:.2f}m "
            f"alt={tani['d_alt']:.2f}m")

    # Geçiş MONOTON ve süreksizliksiz olmalı (komutta sıçrama yaratmasın)
    ws = []
    for menzil in (200.0, 120.0, 100.0, 75.0, 50.0, 30.0, 15.0):
        p_d = (-menzil, 0.0, -50.0)
        _, t = frpn.sanal_hedef(p_h, v_h, p_d, C)
        ws.append((menzil, t["gecis_w"]))
    monoton = all(ws[i][1] <= ws[i + 1][1] + 1e-12 for i in range(len(ws) - 1))
    kontrol("P13c geçiş ağırlığı monoton ve süreksizliksiz",
            monoton and ws[0][1] == 0.0 and ws[-1][1] == 1.0,
            f"{[(int(m), round(w,2)) for m, w in ws]}")

    # SABİT AÇI: geçiş tamamlandıktan sonra yaklaşırken yükseliş DEĞİŞMEMELİ.
    # (Kamera 25° aşağı bakıyor; bu açı büyürse hedef kadrajın tepesinden taşar.)
    aci_ornekleri = []
    for menzil in (11.0, 8.0, 6.0, 4.0, 2.0):
        p_d = (-menzil, 0.0, -50.0)              # hedefin menzil kadar gerisinde
        p_s, tani = frpn.sanal_hedef(p_h, v_h, p_d, C)
        if tani["d_arka"] > 1e-6:
            aci = math.degrees(math.atan2(tani["d_alt"], tani["d_arka"]))
            aci_ornekleri.append((menzil, aci))
    sapma = max(abs(a - C.ISTASYON_ELEV_DEG) for _, a in aci_ornekleri)
    kontrol("P14 SABİT AÇI: yaklaşırken yükseliş açısı korunuyor",
            sapma < 1e-6,
            f"menzil 11→2 m: açı {aci_ornekleri[0][1]:.2f}°→{aci_ornekleri[-1][1]:.2f}° "
            f"(hedef {C.ISTASYON_ELEV_DEG}°)")

    # P14b KAMERA KISITI: istasyondaki yükseliş kameranın gördüğü bandın içinde mi?
    # Kamera YUKARI bakıyor (+25°) — kanıt aşağıda ayrıca test ediliyor (P14c).
    # Dikey yarı-açı 55.25° (640x480, FOV 110.5° — vision/geometry.py).
    # Görülebilir bant: ufkun 30.25° ALTI ... 80.25° ÜSTÜ.
    KAMERA_TILT = 25.0
    DIKEY_YARI = 55.25
    alt_kenar = KAMERA_TILT - DIKEY_YARI          # -30.25°
    ust_kenar = KAMERA_TILT + DIKEY_YARI          # +80.25°
    pay = min(C.ISTASYON_ELEV_DEG - alt_kenar, ust_kenar - C.ISTASYON_ELEV_DEG)
    kontrol("P14b istasyon yükselişi kadraj bandının içinde",
            alt_kenar + 5.0 < C.ISTASYON_ELEV_DEG < ust_kenar - 5.0,
            f"istasyon {C.ISTASYON_ELEV_DEG:.1f}° ∈ ({alt_kenar:.1f}°, {ust_kenar:.1f}°), "
            f"en yakın kenara pay {pay:.1f}°")

    # P14c KAMERA YÖNÜ — bu bir regresyon bekçisi. Analiz bir kez ters işaretle
    # yapıldı ("kamera aşağı bakıyor" sanıldı) ve yanlış tasarım kısıtı üretti.
    # Boresight'ı koddan hesaplayıp yönü kalıcı olarak sabitliyoruz.
    from control.guidance.guidance_core import kamera_to_govde
    import numpy as _np
    bore = kamera_to_govde(_np.array([0.0, 0.0, 1.0]), math.radians(KAMERA_TILT))
    bore_elev = math.degrees(math.atan2(-bore[2], math.hypot(bore[0], bore[1])))
    kontrol("P14c kamera YUKARI bakıyor (+25°), aşağı değil",
            bore_elev > 0 and abs(bore_elev - KAMERA_TILT) < 1e-6,
            f"boresight gövde FRD={tuple(round(float(x),3) for x in bore)} "
            f"→ yükseliş {bore_elev:+.1f}° (FRD'de z<0 = yukarı)")

    # Yavaş hedef: hız yönü güvenilmez → LOS gerisi kullanılmalı
    p_d = (-100.0, 0.0, -50.0)
    p_s, tani = frpn.sanal_hedef(p_h, (0.5, 0.0, 0.0), p_d, C)
    kontrol("P15 yavaş hedefte yön kaynağı LOS'a düşüyor",
            tani["yon_kaynak"] == "los" and p_s[0] < p_h[0],
            f"yon_kaynak={tani['yon_kaynak']}")

    # Yer koruması: alçak hedefte istasyon tabanın altına inmemeli
    p_s, tani = frpn.sanal_hedef((0.0, 0.0, -6.0), v_h, (-100.0, 0.0, -6.0), C)
    kontrol("P16 alçak hedefte istasyon yer tabanına kırpıldı",
            tani["yer_kirpma"] and abs(-p_s[2] - C.LOOKUP_MIN_ALT) < 1e-9,
            f"istasyon irtifası={-p_s[2]:.1f}m (taban {C.LOOKUP_MIN_ALT}m)")

    # ══════════════════════════════════════════════════════════════════
    # P17-P19 · DEJENERASYON DAVRANIŞI
    # ══════════════════════════════════════════════════════════════════
    print("\n── P17-P19: dejenerasyon (istasyonda ne oluyor) ──")

    # İstasyondayız, hızımız hedefle eşleşmiş → komut ≈ hedef hızı (asılı kal)
    r = frpn.komut((0.05, 0.0, 0.0), (0.0, 0.0, 0.0), (16.0, 0.0, 0.0), C)
    fark = _n(tuple(r["v_cmd"][i] - (16.0, 0.0, 0.0)[i] for i in range(3)))
    kontrol("P17 istasyonda komut ≈ hedef hızı (kararlı hold)",
            fark <= C.V_C_MIN + 1e-6,
            f"|v_cmd − v_hedef| = {fark:.3f} m/s ≤ V_C_MIN={C.V_C_MIN}")

    # Kapanma hızı tavanı: 1000 m'de bile V_C_MAX'ı aşmamalı
    r = frpn.komut((1000.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), C)
    kontrol("P18 uzak menzilde kapanma tavanda kalıyor",
            abs(r["v_c"] - C.V_C_MAX) < 1e-9,
            f"v_c={r['v_c']:.1f} = V_C_MAX={C.V_C_MAX}")

    # Yumuşak duruş: kapanma hızı menzille birlikte monoton azalmalı
    v_ler = [frpn.komut((d, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), C)["v_c"]
             for d in (100.0, 50.0, 20.0, 10.0, 4.0, 1.0)]
    monoton = all(v_ler[i] >= v_ler[i + 1] for i in range(len(v_ler) - 1))
    kontrol("P19 yaklaşırken kapanma hızı monoton azalıyor (yumuşak duruş)",
            monoton, f"100→1 m: {[round(v,1) for v in v_ler]}")

    # ══════════════════════════════════════════════════════════════════
    # P20 · REGRESYON KAYDI — makale katsayıları neden kullanılmadı
    # ══════════════════════════════════════════════════════════════════
    print("\n── P20: makale katsayılarının ölçek kaydı ──")
    # arXiv 2405.13542 denklem (19), Tablo II: G=19.7, W=5.1e-2.
    # Bizim tipik menzilimizde takip terimi G·W·‖Δp‖ ivme bütçesini kaç kat aşıyor?
    G_MAKALE, W_MAKALE = 19.7, 5.1e-2
    IVME_BUTCE = 5.0                              # m/s² (WP_ACC, ölçüldü)
    for menzil, etiket in ((100.0, "tipik devir öncesi"), (11.0, "istasyon")):
        a_takip = G_MAKALE * W_MAKALE * menzil
        kat = a_takip / IVME_BUTCE
        if menzil == 100.0:
            kontrol("P20 makale katsayısı bizim ölçekte doygunluğa mahkûm",
                    kat > 10.0,
                    f"{etiket} ({menzil:.0f} m): takip terimi {a_takip:.0f} m/s² "
                    f"= bütçenin {kat:.0f}× katı")
        else:
            print(f"        (bilgi) {etiket} ({menzil:.0f} m): "
                  f"{a_takip:.1f} m/s² = bütçenin {kat:.1f}× katı")

    # ══════════════════════════════════════════════════════════════════
    print("\n" + "=" * 60)
    gecen = sum(1 for _, ok, _ in _sonuclar if ok)
    toplam = len(_sonuclar)
    print(f"SONUÇ: {gecen}/{toplam} geçti — "
          f"{'HEPSİ GEÇTİ ✓' if gecen == toplam else 'BAŞARISIZ ✗'}")
    if gecen != toplam:
        for ad, ok, detay in _sonuclar:
            if not ok:
                print(f"  FAIL: {ad}  {detay}")
    return 0 if gecen == toplam else 1


if __name__ == "__main__":
    raise SystemExit(main())
