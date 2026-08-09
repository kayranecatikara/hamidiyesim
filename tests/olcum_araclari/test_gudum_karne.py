"""tools/gudum_karne.py — CSV karnesinin metrik toplamı.

    python3 -m tests.olcum_araclari.test_gudum_karne

Karne, iki koşuyu kıyaslamanın (`--kiyasla`) tek yolu; bir metrik sessizce
yanlış toplanırsa "yeni ayar daha iyi" hükmü de yanlış olur. Buradaki testler
sentetik CSV satırlarıyla saf toplama mantığını denetler.

K1-K3  _vis_metrik    faz sonucu sınıflaması, en yakın menzil, vuruş bayrağı
K4     _vis_metrik    pose oranı — hangi satır "pose var" sayılıyor
K5-K6  _gps_metrik    istasyonda oturma yüzdesi, kadraj hatası yalnız KİLİT'te
K7-K8  _sentetik      eski test artefaktı süzgeci (gerçek uçuşu ELEMEMELİ)
K9-K11 _gecis_metrik  faz geçişi sağlığı (2026-08-09: "gps kopup duruyo")
"""

import math

from tests.olcum_araclari.ortak import kontrol, ozet, sifirla
from tools import gudum_karne as gk

VIS = "logs/visual_lead_20260804_164352.csv"
GPS = "logs/gps_guidance_20260804_164352.csv"


def _vis_satir(t, menzil, durum, kalite="0.8", yaw="1.0", lead="5.0"):
    return {"t_ros": str(t), "menzil_gercek_m": str(menzil), "durum": durum,
            "kalite": kalite, "yaw_hata_deg": yaw, "lead_deg": lead}


def main():
    sifirla()
    print("Ölçüm aracı: gudum_karne (CSV metrikleri)")
    print("=" * 60)

    # ── K1: temas olan faz VURULDU sayılmalı, en yakın menzil doğru ──
    rows = [_vis_satir(0.0, 10.0, "ok"), _vis_satir(0.1, 4.0, "ok"),
            _vis_satir(0.2, 0.7, "ok"), _vis_satir(0.3, 0.9, "vuruldu")]
    v = gk._vis_metrik([(VIS, rows)])
    kontrol("K1  temas olan faz VURULDU, en yakın menzil minimumdan",
            v["fazlar"][0]["sonuc"] == "VURULDU" and v["vurus"] is True
            and abs(v["en_yakin"] - 0.7) < 1e-9
            and abs(v["fazlar"][0]["devir_menzil"] - 10.0) < 1e-9,
            f"sonuç={v['fazlar'][0]['sonuc']} devir={v['fazlar'][0]['devir_menzil']:.1f} m "
            f"en yakın={v['en_yakin']:.2f} m")

    # ── K2: 15+ tespit_yok karesi = 'kayıp'; altındaki = 'ıska/koptu' ──
    kayip = [_vis_satir(i * 0.03, 8.0, "tespit_yok") for i in range(15)]
    iska = [_vis_satir(i * 0.03, 8.0, "tespit_yok") for i in range(14)]
    v_kayip = gk._vis_metrik([(VIS, kayip)])["fazlar"][0]["sonuc"]
    v_iska = gk._vis_metrik([(VIS, iska)])["fazlar"][0]["sonuc"]
    kontrol("K2  faz sonucu sınıflaması (kayıp eşiği 15 kare)",
            v_kayip == "kayıp" and v_iska == "ıska/koptu",
            f"15 kare → '{v_kayip}'   14 kare → '{v_iska}'")

    # ── K3: çok fazlı uçuşta en_yakin TÜM fazların minimumu olmalı ──
    faz_a = [_vis_satir(0.0, 9.0, "ok"), _vis_satir(0.1, 3.4, "ok")]
    faz_b = [_vis_satir(5.0, 8.0, "ok"), _vis_satir(5.1, 1.2, "ok")]
    v = gk._vis_metrik([(VIS, faz_a), (VIS, faz_b)])
    kontrol("K3  en_yakin tüm fazların minimumu",
            v["faz_sayisi"] == 2 and abs(v["en_yakin"] - 1.2) < 1e-9
            and v["vurus"] is False,
            f"{v['faz_sayisi']} faz, min(3.40, 1.20) = {v['en_yakin']:.2f} m")

    # ── K4: pose ORANI ARTIK YOK — metrik geri sızarsa uyar ──
    # 2026-08-06'da pose modeli kaldırıldı (bkz. scripts/gcs.sh başlığı), ama
    # bu test 'pose_orani_%' okumaya devam etti ve KeyError ile ÇÖKTÜ. Çöken
    # dosya K5-K10'u hiç çalıştırmıyordu; üstelik testleri `python3 dosya.py |
    # tail` ile koşarsanız çıkış kodu boruda kaybolduğu için sağlam görünür.
    # (2026-08-09'da fark edildi.) Artık ölçüt tersine çevrildi: metriğin
    # OLMADIĞI doğrulanıyor. Pose geri gelirse burası kırmızı yanar ve testi
    # bilinçli olarak yeniden yazmak gerekir.
    karisik = [_vis_satir(0.0, 8.0, "ok", kalite="0.9"),
               _vis_satir(0.1, 8.0, "kanat_dusuk", kalite="0.0"),
               _vis_satir(0.2, 8.0, "tespit_yok", kalite=""),
               _vis_satir(0.3, 8.0, "tespit_yok", kalite="")]
    v = gk._vis_metrik([(VIS, karisik)])
    kontrol("K4  pose metriği kaldırılmış durumda (08-06 pose kaldırıldı)",
            "pose_orani_%" not in v and v["faz_sayisi"] == 1,
            f"anahtarlar: {sorted(v)[:6]}…")

    # ── K5: istasyonda oturma, İLK kez 15 m altına inildikten SONRA ölçülür ──
    # Uzun ilk yaklaşma (30, 20 m) yüzdeyi sulandırmamalı.
    dh = [30.0, 20.0, 14.0, 10.0, 10.0, 9.0, 12.0, 30.0]
    rows = [{"d_h": str(d), "t": str(i * 0.05)} for i, d in enumerate(dh)]
    g = gk._gps_metrik([(GPS, rows)])
    kontrol("K5  oturma yüzdesi ilk yaklaşmayı dışarıda bırakıyor",
            abs(g["oturma_%"] - 100.0 * 4 / 6) < 1e-6
            and abs(g["min_d_h"] - 9.0) < 1e-9 and abs(g["son_d_h"] - 30.0) < 1e-9,
            f"14 m'den sonraki 6 karenin 4'ü 8-12 m bandında → %{g['oturma_%']:.1f}")

    # ── K6: kadraj hatası YALNIZ 'KILIT' karelerinden — kilitsiz kare kirletmemeli ──
    rows = [{"d_h": "10.0", "t": "0.0", "durum": "KILIT", "kadraj_yaw_deg": "3.0"},
            {"d_h": "10.0", "t": "0.1", "durum": "KILIT", "kadraj_yaw_deg": "-4.0"},
            {"d_h": "10.0", "t": "0.2", "durum": "ARAMA", "kadraj_yaw_deg": "100.0"}]
    g = gk._gps_metrik([(GPS, rows)])
    kontrol("K6  kadraj yaw RMS yalnız KİLİT karelerinden",
            abs(g["kadraj_yaw_rms"] - math.sqrt(12.5)) < 1e-9,
            f"RMS(3, −4) = {g['kadraj_yaw_rms']:.4f}  (100° kilitsiz kare atıldı)")

    # ── K7: eski test artefaktı süzgeci sentetik CSV'yi elemeli ──
    sentetik_vis = [_vis_satir(1 / 30, 8.0, "ok")]
    sentetik_gps = [{"tgt_x": "50.0", "tgt_y": "20.0", "tgt_z": "-40.0"}]
    kontrol("K7  sentetik test artefaktları eleniyor",
            gk._sentetik(VIS, sentetik_vis) and gk._sentetik(GPS, sentetik_gps)
            and gk._sentetik(VIS, []),
            "t_ros=1/30 (visual) ve tgt=(50,20,−40) (gps) imzaları + boş dosya")

    # ── K8: ...ama GERÇEK uçuş elenmemeli (sessiz veri kaybı en kötü hata) ──
    kontrol("K8  gerçek uçuş kayıtları ELENMİYOR",
            not gk._sentetik(VIS, [_vis_satir(1234.5, 8.0, "ok")])
            and not gk._sentetik(GPS, [{"tgt_x": "12.3", "tgt_y": "4.5",
                                        "tgt_z": "-30.0"}]),
            "gerçekçi t_ros ve hedef konumu süzgeçten geçiyor")

    # ── K9-K11: FAZ GEÇİŞİ SAĞLIĞI (2026-08-09) ──
    # Kullanıcı "gps kopup duruyo, sürekli gps-görsel geçişi oluyor" dedi ve
    # hiçbir araç bunu raporlamıyordu. Karne faz SAYISINI basıyordu ama 22
    # fazın anormal olduğunu söyleyen bir ölçüt yoktu.
    def _gps_hiz(t0, hizlar):
        """t0'dan başlayan, verilen yatay hızlara sahip GPS fazı satırları."""
        return [{"t": str(t0 + i * 0.05), "vx_cmd": str(h), "vy_cmd": "0.0"}
                for i, h in enumerate(hizlar)]

    # K9: araç durmuş başlıyor ve faz hızına ULAŞAMIYOR → yakalanmalı
    duran = _gps_hiz(0.0, [0.1, 0.5, 1.2, 2.0, 3.1, 4.0])       # 15'e çıkmıyor
    saglikli = _gps_hiz(100.0, [17.0] * 6 + [18.0] * 4)          # baştan hızlı
    c = gk._gecis_metrik({"gps": [(GPS, duran), (GPS, saglikli)], "vis": []})
    kontrol("K9  GPS fazı 'durmuş başladı' ve 'hıza ulaşamadı' ölçülüyor",
            c["gps_faz"] == 2 and c["gps_ulasamadi"] == 1
            and abs(c["gps_ilk_hiz_med"] - (0.1 + 17.0) / 2) < 1e-9,
            f"{c['gps_faz']} faz, ulaşamayan {c['gps_ulasamadi']}, "
            f"ilk hız medyan {c['gps_ilk_hiz_med']:.2f} m/s")

    # K10: kısa faz + fly-past + yeniden-giriş oranları
    # uzun faz: 4 s, devir 19 m (gerçek devir) — fly-past ile biter
    uzun = [_vis_satir(i * 0.04, 19.0 - i * 0.15, "ok") for i in range(100)]
    uzun[-1]["durum"] = "gecildi"
    # kısa faz: 0.8 s, devir 9 m (yeniden-giriş)
    kisa = [_vis_satir(i * 0.04, 9.0 - i * 0.1, "ok") for i in range(20)]
    c = gk._gecis_metrik({"gps": [], "vis": [(VIS, uzun), (VIS, kisa)]})
    kontrol("K10 kısa faz / fly-past / yeniden-giriş oranları",
            c["vis_faz"] == 2 and abs(c["vis_kisa_%"] - 50.0) < 1e-9
            and abs(c["gecildi_%"] - 50.0) < 1e-9
            and abs(c["yeniden_giris_%"] - 50.0) < 1e-9,
            f"kısa %{c['vis_kisa_%']:.0f}  fly-past %{c['gecildi_%']:.0f}  "
            f"yeniden-giriş %{c['yeniden_giris_%']:.0f}")

    # K11: GPS yasası CSV SÜTUNLARINDAN tanınmalı. frpn_guidance çıktısını
    # gps_guidance ile AYNI dosya adıyla yazıyor ve damgası yok — kol yalnız
    # sütun imzasından ayırt edilebiliyor. FRPN A/B'si buna bağlı.
    from tools import ab_gecerli_mi as ab
    kontrol("K11 GPS yasası (frpn / istasyon) sütun imzasından tanınıyor",
            ab._yasa([{"t_go": "1.0", "zem_norm": "0.1"}]) == "frpn"
            and ab._yasa([{"ist_elev_deg": "15.0"}]) == "istasyon"
            and ab._yasa([]) is None,
            "t_go → frpn · ist_elev_deg → istasyon · boş → None")

    return ozet("gudum_karne")


if __name__ == "__main__":
    import sys
    gecen, toplam = main()
    sys.exit(0 if gecen == toplam else 1)
