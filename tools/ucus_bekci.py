#!/usr/bin/env python3
"""Uçuş bekçisi: otonom test sırasında "sapıtma"yı CANLI yakalar.

NEDEN VAR (2026-08-08): otonom düz uçuş testinde hedef uçak yavaşça 12 m
irtifaya alçaldı; test sonrası da kontrolsüz bırakılınca yere çakılıp SITL
fiziğiyle yerin 1738 m altına savruldu. İkisi de ancak SONRADAN fark edildi.
Bu bekçi test boyunca çalışır; sağlık bandı dışına SÜREKLİ çıkan her durumda
tek satır İHLAL basar ve 1 koduyla çıkar → testi koşan taraf (Claude/insan)
uçuşu durdurup simi baştan kurar. O koşunun verisi GEÇERSİZ sayılır.

Kullanım:
    python3 tools/ucus_bekci.py <sure_s> [gecikme_s]
        sure_s     : bekçinin toplam çalışma süresi (uçuş süresi + pay)
        gecikme_s  : kalkış payı — bu süre boyunca yalnız "aşırı" ihlaller
                     bakılır (varsayılan 30)

Sağlık bandı (3 ardışık örnekte = ~6 s süreklilik aranır, tekil sıçrama
alarm ürettirmez):
    hedef irtifa   > 20 m        hedef hız   6..25 m/s
                   (üst sınır 2026-08-23'te KALDIRILDI — Gazebo hava
                    yoğunluğu irtifayla değişmiyor, üst sınırın fiziksel
                    gerekçesi yoktu)
    drone irtifa   > 4 m (takip aktifken)
    mesafe         < 150 m (takip aktifken)
    API            10 s'den uzun cevapsız kalmamalı
"""
import json
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8000"
ARALIK = 2.0          # s; örnekleme
ESIK = 3              # ardışık ihlal örneği → alarm


def getir(yol):
    with urllib.request.urlopen(BASE + yol, timeout=3) as r:
        return json.loads(r.read())


def main():
    sure = float(sys.argv[1]) if len(sys.argv) > 1 else 420.0
    gecikme = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    t0 = time.time()
    sayac = {}            # kural adı → ardışık ihlal sayısı
    api_son_ok = time.time()
    mesafe_onceki = None  # kapanma tespiti için (uzun yaklaşma ≠ ıraksama)

    def ihlal(ad, detay):
        sayac[ad] = sayac.get(ad, 0) + 1
        if sayac[ad] >= ESIK:
            print(f"İHLAL: {ad} — {detay}", flush=True)
            raise SystemExit(1)

    def temiz(ad):
        sayac[ad] = 0

    while time.time() - t0 < sure:
        time.sleep(ARALIK)
        gecti = time.time() - t0
        try:
            tel = getir("/api/debug/telem")["telemetry_state"]
            chase = getir("/api/chase_status")
            api_son_ok = time.time()
        except Exception:
            if time.time() - api_son_ok > 10.0:
                print("İHLAL: API cevapsız — gcs_server ölmüş olabilir", flush=True)
                raise SystemExit(1)
            continue

        p, i = tel["plane"], tel["iris"]
        p_alt, i_alt = -p["z"], -i["z"]
        p_spd = p.get("speed") or 0.0
        aktif = bool(chase.get("active"))
        mesafe = chase.get("distance") or 0.0

        # her koşulda geçerli "aşırı" sınırlar (kalkış payında da)
        # ⚠ 400 m ÜST SINIRI KALDIRILDI (2026-08-23) — bkz. aşağıdaki not.
        # Yalnız "yerin altına savrulma" (SITL fiziği patladı) kaldı.
        if p_alt < -5:
            ihlal("hedef-irtifa-asiri", f"{p_alt:.0f} m (YER ALTI — SITL fiziği bozuk)")
        else:
            temiz("hedef-irtifa-asiri")

        if gecti < gecikme:
            continue

        # ═══ ÜST İRTİFA SINIRI KALDIRILDI — 2026-08-23, kullanıcı kararı ═══
        # ESKİ KURAL: 20 ≤ irtifa ≤ 250 m. Üst sınırın gerekçesi
        # "koşular karşılaştırılabilir olsun" diye savunulmuştu. ÖLÇÜLDÜ VE
        # ÇÜRÜTÜLDÜ: model.sdf'teki 7 LiftDrag eklentisinin hepsinde
        # <air_density>1.2041</air_density> SABİT — Gazebo yoğunluğu irtifayla
        # değiştirmiyor. Yani simde 45 m'deki uçuş ile 900 m'deki uçuş
        # aerodinamik olarak BİREBİR AYNI. Karşılaştırılabilirlik savı geçersiz.
        #
        # KULLANICI GEREKÇESİ: "yarışmada hedefin nasıl uçacağını bilmiyoruz,
        # avcı bütün ihtimallere dayanıklı olmalı." Yüksekte uçan bir hedef
        # GEÇERLİ bir senaryodur; yapay bir tavanla onu test dışı bırakmak
        # avcıyı gerçek göreve hazırlıksız bırakır.
        #
        # ALT SINIR 20 m DURUYOR — gerçek tehlike: hedef alçalırsa avcı drone
        # peşinden yere çarpar (2026-08-08'de yaşandı, bu bekçinin varlık
        # sebebi de o).
        if p_alt < 20.0:
            ihlal("hedef-irtifa", f"{p_alt:.0f} m (alt sınır 20 m)")
        else:
            temiz("hedef-irtifa")
        if not (6.0 <= p_spd <= 25.0):
            ihlal("hedef-hiz", f"{p_spd:.1f} m/s (bant 6-25)")
        else:
            temiz("hedef-hiz")
        if aktif and i_alt < 4.0:
            ihlal("drone-irtifa", f"{i_alt:.1f} m (takip aktifken <4)")
        else:
            temiz("drone-irtifa")
        # Iraksama: uzak OLMAK değil, uzak olup KAPANMAMAK ihlaldir.
        # (İlk canlı koşuda ders: 351 m geriden başlayan meşru yaklaşma
        # 2.9 m/s'le kapanırken alarm çaldı. Kapanıyorsa sorun yok.)
        kapaniyor = (mesafe_onceki is not None
                     and mesafe < mesafe_onceki - 0.3 * ARALIK)
        if aktif and mesafe > 150.0 and not kapaniyor:
            ihlal("mesafe-iraksama", f"{mesafe:.0f} m (takip aktifken >150 ve kapanmıyor)")
        else:
            temiz("mesafe-iraksama")
        mesafe_onceki = mesafe if aktif else None

    print("BEKCI: süre doldu, ihlal yok", flush=True)


if __name__ == "__main__":
    main()
