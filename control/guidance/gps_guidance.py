"""
gps_guidance.py — GPS güdümü (sıfırdan yeniden inşa, görsel-temas odaklı).

AMAÇ (başarı kriteri): Drone'u öyle konumlandır ki hedef sabit-kanatlı İHA
kameranın TAM ORTASINDA, pose modelinin güvenilir çalıştığı menzil bandında
(~10-11 m) ve KARARLI görünsün → supervisor görsel faza devretsin. (Vuruş DEĞİL;
vuruş görsel fazın işi.)

Kadraj merkezi ⇔ gövde-çerçevesinde hedefe bakış: azimut=0, yükseliş=+25°
(kamera tilt'i). Bu hata GPS + drone attitude'undan kapalı formda ölçülür
(guidance_core.hedef_kadraj_hatasi) ve her kare CSV'ye yazılır → merkezleme
başarısı ölçülebilir.

KADEME 1: GEOMETRİK kadraj-noktası takibi. Hedefin hız yönünün D_BEHIND
gerisine + D_BELOW altına (slant RANGE_SET'te +25° yükseliş verecek) bir
istasyon kur; oraya PD hız + hedef-hızı feedforward ile git (feedforward →
kilitlenince kararlı hold). Burun daima gerçek hedefe döner (yaw). Drone hedefin
ALTINDA kalır → gökyüzü arka planı, pose kopmaz.

KADEME 1b (2026-08-04): İSTASYON ARTIK ÖNGÖRÜLÜ. Kademe 1 istasyonu hedefin
ANLIK konumuna kuruyordu, yani saf takip (pure pursuit). Kapalı bir desende
(daire/kare) saf takip yapan ve hedeften yavaş olan bir avcı hedefi ASLA
yakalayamaz — desenin ortasına spirallenir. Ölçüm bunu birebir gösterdi
(gps_guidance_20260801_173612, 1143 s): menzil 89 m'den 82 m'ye indi ve 19
dakika orada kaldı, tek bir görsel devir olmadı. Çare köşeyi KESMEK: hedefin
dönüş hızı (omega) kestirilir, t_go sonrası nerede olacağı sabit-dönüş
(coordinated turn) modeliyle tahmin edilir ve istasyon ORAYA kurulur.
(KADEME 2'de: gerçek attitude'la kadraj hatasını doğrudan kapatma eklenecek.)

Arayüz (supervisor / gcs_server ile aynı sözleşme):
  run_gps_guidance(conn, get_plane, get_iris, stop_event, cfg=Cfg)
    get_plane() -> {x,y,z,yaw,frozen}                (m, NED; GPS-gürültülü hedef)
    get_iris()  -> {x,y,z, roll,pitch,yaw, vx,vy,vz} (m/rad; kendi poz + attitude)
  status["d_h"], status["durum"] supervisor.izci tarafından okunur (DROPOUT dahil).
"""

import csv
import math
import os
import time

from control.guidance.common import (
    clamp, normalize_angle, limit_acceleration, send_velocity,
)
from control.guidance.guidance_core import hedef_kadraj_hatasi


def _env_f(name, default):
    return float(os.environ.get(name, default))


class Cfg:
    LOOP_HZ = 20.0

    # --- KADRAJ GEOMETRİSİ (merkezleme) ---
    CENTER_ELEV_DEG = 25.0    # kamera tilt'i = merkez için gereken LOS yükselişi
    RANGE_SET = _env_f("AVCI_GPS_RANGE", 11.0)   # m; slant menzil setpoint (pose tatlı nokta)
    TRACK_MIN_SPD = 3.0       # m/s; üstünde istasyon HIZ yönünün gerisi (kuyruk), altında LOS gerisi
    LOOKUP_MIN_ALT = 8.0      # m; alçalma tabanı (yere çakılma koruması)

    # --- HIZ KONTROLÜ ---
    KP_H = 0.8                # yatay konum hatası → hız (1/s)
    KD_H = 0.20               # yatay türev sönümleme
    KP_Z = 1.0               # dikey konum hatası → hız (1/s)
    # VZ_MAX 6.0 → 4.0 (2026-08-04 çöküşü). 6 m/s'lik SÜREKLİ iniş, avcıyı
    # kendi pervane akımına düşürüyor (vortex ring state): itki çöker, araç
    # kontrolü kaybeder. Ölçüm (gps_guidance_20260804_130550): vz_cmd 5+ saniye
    # +6.0'da doygun, eğim 41° → 64.8° → 81° → 103.9° (90° geçince ters dönme),
    # gerçek hız 0.8 m/s'ye düştü, avcı düştü.
    # 4 m/s, aynı 45 m'yi 11 s'de indirir — chase için yeterince hızlı, VRS
    # eşiğinin (tipik olarak iniş hızı / pervane indüklenmiş hız > ~0.5) altında.
    # DİKKAT: bu değer avci_copter.parm'daki WP_SPD_DN ile TUTARLI olmalı.
    VZ_MAX = 4.0              # m/s; dikey hız tavanı
    # V_MAX 28→15 (2026-08-01, iki uçuşun ölçümü: gps_guidance_20260801_151839
    # ve _152542; 3674 + 3875 kare, toplam 380 s).
    #
    # 28 m/s KOMUT EDİLEBİLİR AMA UÇULAMAZ, ve fazlası aktif ZARAR veriyordu:
    #   komut medyanı 28.0 (karelerin %84'ü tavanda doygun)
    #   GERÇEKLEŞEN  medyan 8.2 m/s, p95 16.0, p99 19.8 → airframe tavanı ~20
    #   menzil >50 m iken (tam gaz gitmesi gereken yer): gerçekleşen 6.8 m/s,
    #     GERÇEK/KOMUT oranı 0.24
    #   menzil <50 m iken (yavaşlaması gereken yer):     gerçekleşen 9.9 m/s
    # Uzaktayken YAKINDAKİNDEN YAVAŞ olması patolojinin imzası: 28 m/s'lik hız
    # hatası PSC_VELXY_P=2.0 ile ~56 m/s² ivme talebine dönüşüyor, bu ANGLE_MAX
    # ile ulaşılabilir ivmenin (~17 m/s², irtifa korunurken) çok üstünde. Attitude
    # kontrolcüsü tavana yapışıyor, itki vektörü irtifayı korumak için yatışı geri
    # çekiyor → yatış açısında limit çevrimi, ortalama ivme düşüyor. Hata küçükken
    # (yakın menzil) kontrolcü doğrusal bölgede kalıyor ve DAHA İYİ çalışıyor.
    #
    # 15→18 (aynı gün, ikinci tur): 15'te kontrolcü SAĞLIKLI çalıştı —
    # gerçekleşen/komut oranı 0.24'ten 0.78'e çıktı, medyan hız 8.2→11.7 m/s,
    # tepe 15.1 (yani komut tavanına DAYANDI, airframe'de pay kaldı).
    # Ama 11.7 m/s hedefin sürdürülebilir en düşük hızının (11 m/s, AIRSPEED_MIN)
    # ancak 0.7 m/s üstü; menzil kapanmıyor. Hedefi daha da yavaşlatmak stall
    # ettiriyor (ölçüm: 9.6 m/s'de kare dönüşünde düştü), dolayısıyla marj
    # AVCI tarafından açılmalı.
    # 18→20 (aynı gün, üçüncü tur): 18'de avcı komutu TAM uyguluyordu
    # (gerçekleşen/komut = 1.00, sabit 18.0 m/s) — yani tavana dayanmıştı,
    # airframe'de hâlâ pay vardı (V_MAX=28 ölçümünde p99 19.8, tepe 20.5).
    # Ama hedef 16.3 m/s uçuyor; 1.7 m/s marj kare dönüşlerinin salınımını
    # yenemedi: menzil 441→84.6 m'ye indi, orada dibe vurup 97.8'e geri açıldı.
    # 20 ile marj 3.7 m/s'ye çıkar (iki katından fazla).
    #
    # Doygunluk patolojisine geri dönme riski DÜŞÜK, çünkü eşik V_MAX değil hız
    # HATASININ büyüklüğü: PSC_VELXY_P=2.0 ile 20−16.3 = 3.7 m/s hata → 7.4 m/s²
    # ivme talebi, ulaşılabilir ~17 m/s²'nin çok altında. (28'de hata 20 m/s →
    # 40 m/s² talebiyle attitude tavana yapışıp limit çevrimine giriyordu.)
    # Doğrulama: analiz_gps "GERÇEKLEŞEN/KOMUT" oranı 0.9'un altına düşerse
    # patoloji geri gelmiş demektir, V_MAX düşürülmeli.
    # 20 → 17 (2026-08-04). Kopter parametreleri düzeltilince (arıza 9) avcı
    # gerçekten 20 m/s'ye gitmeye başladı — ve bunu SÜRDÜRMEK için gereken
    # sürükleme yatışı, ivme yatışıyla toplanıp ATC_ANGLE_MAX'ı deldi; iki
    # uçuşta da araç takla attı. Hedef artık 13.8 m/s uçuyor (avci_plane.parm
    # AIRSPEED_MAX 16); 17, kapanma için 3.2 m/s marj bırakır ve bunun için
    # gereken sürükleme yatışı ~46°, tavana (65°) pay kalır.
    # DOĞRULAMA: menzil kapanmıyorsa marj yetmiyor demektir — önce HEDEFİ
    # yavaşlat (slider), V_MAX'ı yükseltmek yatış payını yer.
    V_MAX = 17.0             # m/s; yatay hız tavanı
    # avci_copter.parm WP_ACC ile AYNI olmalı: komut, aracın uygulayabildiğinden
    # hızlı değişirse fark doygunluk olarak birikir ve yatış tavana dayanır.
    MAX_ACCEL = 8.0          # m/s²; komut hızı değişim sınırı
    # Yön dönme sınırının altında devre dışı kalacağı hız. Düşük hızda
    # omega_max = YON_ACCEL/|v| patlar (|v|→0'da sonsuz) ve sınır anlamsızlaşır;
    # ayrıca duran araç yönünü serbestçe seçebilmeli (kalkış, hover, ilk yönelme).
    YON_LIMIT_MIN_HIZ = 3.0  # m/s
    # Yön dönüşüne ayrılan yanal ivme bütçesi. MAX_ACCEL'den AYRI ve DAHA
    # BÜYÜK — ölçümle: MAX_ACCEL=8 ile omega_max = 8/17 = 0.47 rad/s = 27 °/s
    # oluyordu, oysa daire uçan hedefte komut yönünün dönme hızı p90 34.6 °/s
    # (gps_guidance_20260801_173612). Yani sınır, NORMAL takibi kesiyordu:
    # görsel faz oranı %77'den %8-18'e, faz süresi 5.4 s'den 0.8-2.4 s'ye düştü
    # (2026-08-04 karşılaştırması). Agresif senaryodaki çöküşü tetikleyen
    # sıçramalar ise bunun kat kat üstündeydi, dolayısıyla daha gevşek bir
    # bütçe o korumayı kaybettirmez.
    # 12 m/s² → 17 m/s'de 40 °/s: dairenin p90'ını (34.6) kapsar, agresif
    # sıçramaları keser. Yatış karşılığı atan(12/9.81) = 50.7°, ATC_ANGLE_MAX
    # 65'in altında. DOĞRULAMA: menzil kapanmıyorsa sınır hâlâ sıkı,
    # takla dönüyorsa gevşek.
    # 12 → 10 (2026-08-04, üçüncü tur): 12 m/s² ile menzil kapandı (310→13.4 m,
    # 45 s) AMA avcı yine takla attı — yatış p99 71°, max 133.6°, ATC_ANGLE_MAX
    # 65'in üstünde. 12 m/s²'lik yön ivmesi tek başına 50.7° yatış ister;
    # üstüne hızı sürdüren sürükleme yatışı (~46°) binince tavan deliniyor.
    # 10 → 45.6° yön yatışı; 17 m/s'de omega_max 33.7 °/s, dairenin p90'ı
    # (34.6) sınırda ama medyanının (15) iki katı — takip sürüyor.
    # BANT: 8 çok sıkı (görsel faz %77→%8), 12 çok gevşek (takla). Ara değer.
    YON_ACCEL = 10.0         # m/s²
    DERIV_EMA = 0.2

    # --- YAW ---
    # YAW_RATE_MAX 120→45 °/s (2026-08-01 ölçümü). GPS fazının GÖREVİ hedefi
    # kameranın ortasında TUTMAK; burnu hızla çevirmek bunun tam tersini yapıyordu.
    #
    # Ölçüm (gps_guidance_20260801_172851.csv, 1300 kare):
    #   komut yaw dönme hızı : med 0 °/s, p90 66, p99 94, max 120  ← tavana dayalı
    #   gerçek yaw dönme hızı: med 30 °/s, p90 96, p99 196, max 368
    # Kamera gövdeye sabit ve HFOV 125°. Hedefin kadrajı kat etme süresi:
    #   120 °/s → 1.04 s (31 kare) | 200 °/s → 0.62 s (19) | 368 °/s → 0.34 s (10)
    # Aynı gün ölçülen en uzun ARDIŞIK pose tespiti: 6, 8, 18 kare. Birebir bu
    # aralık — yani hedef kaçmıyordu, avcı onu KENDİ burun hareketiyle kadrajdan
    # süpürüyordu. Devir kapısı 10 ardışık kare istiyor ve zar zor yetişiyordu.
    #
    # 45 °/s neden yeterli: burun hedefin KERTERİZİNİ takip etmeli, kerteriz
    # değişim hızı = yanal bağıl hız / menzil. Hedef 15 m/s, kritik menzil bandı
    # 20-50 m → 17-43 °/s. 45 bu bandı kapsar. Kazanç: hedefin kadrajda kalma
    # süresi 2.8 s'ye (≈83 kare) çıkar — kapının istediği 10 karenin sekiz katı.
    # Daha düşürmek kerteriz takibini kaybettirir; DOĞRULAMA: analiz_gps'te
    # "kadraj yaw hatası" medyanı büyürse bu değer fazla düşük demektir.
    #
    # 2026-08-04: SABİT TAVAN KALDIRILDI, KERTERİZE BAĞLANDI.
    # 45 °/s doğru ölçülmüştü ama YANLIŞ REJİM için: o gün avcı 8-14 m/s
    # uçuyordu ve 82 m'de takılı kalmıştı; o menzilde gereken kerteriz hızı
    # 4.5-18.7 °/s'dir, 45 bolca yetiyordu. Kopter parametreleri düzeltilip
    # (avci_copter.parm arıza 9) avcı gerçekten yaklaşmaya başlayınca rejim
    # değişti ve aynı tavan bu kez KISITLAYICI oldu.
    #
    # Ölçüm (2026-08-04 circle uçuşu, 3425 kare, dokuz GPS fazı):
    #   komut yaw hızı: med 44.0, p75 46.0, p99 46.0  ← %99 TAVANDA SIKIŞIK
    #   GEREKEN kerteriz hızı, menzil bandına göre:
    #     60+ m   med  4.5  p90  18.7 °/s     |kadraj_yaw| med  7.2°
    #     30-60 m med 10.1  p90  36.0 °/s     |kadraj_yaw| med 20.4°
    #     15-30 m med 18.5  p90  50.1 °/s     |kadraj_yaw| med 20.3°
    #     0-15 m  med 38.5  p90 134.0 °/s     |kadraj_yaw| med 18.2°  ← devir bandı
    # Yani tam devrin olduğu yerde 134 °/s gerekiyor, 45 veriliyordu; kadraj
    # hatası 2.8°'den 37.6°'ye çıktı ve görsel faz hedefi kenarda devraldı.
    #
    # Yeni yasa: tavan, GEREKEN kerteriz hızıyla ölçeklenir.
    #     tavan = clamp(KERTERIZ_PAY·|kerteriz_hızı| + TABAN, TABAN, TAVAN)
    # Bu, eski sabit tavanın çözdüğü patolojiyi DE yapısal olarak önler:
    # 2026-08-01'de burun, kerteriz sakinken 368 °/s savruluyor ve hedefi kendi
    # hareketiyle kadrajdan süpürüyordu. Yeni yasada kerteriz sakinse tavan
    # TABAN'da (20 °/s) kalır — savrulma zaten mümkün değil. Kerteriz gerçekten
    # hızlıysa izin verilir, çünkü izin verilmezse hedef kadrajdan zaten çıkar.
    YAW_DEADBAND = math.radians(3.0)
    # PAY > 1 olmalı: yalnız kerterizi TAKİP etmek birikmiş hatayı kapatmaz,
    # üstüne yetişme payı gerekir. 1.5, 0-15 m bandında 134 → 221 °/s verir.
    KERTERIZ_PAY = _env_f("AVCI_GPS_KERTERIZ_PAY", 1.5)
    # Taban: kerteriz sıfırken bile artık hatayı kapatacak kadar. 60+ m bandının
    # p90'ı 18.7 °/s — 20 o bandı kerteriz terimi olmadan da karşılar.
    YAW_RATE_TABAN = math.radians(20.0)
    # Tavan: ölçülen gerçek yaw hızı p90 104 °/s. 200, 0-15 m bandının p90
    # ihtiyacını (134) karşılar ama 2026-08-01'de görülen 368 °/s savrulmayı
    # keser. DOĞRULAMA: |kadraj_yaw| medyanı yakın menzilde hâlâ büyükse
    # PAY veya TAVAN yükseltilmeli; komut yaw hızı yine %99 tavandaysa aynısı.
    # 200 → 120 (2026-08-04, çöküşlerden sonra). Yaw yetkisi motorların
    # DİFERANSİYEL torkundan gelir ve aynı motorlar yatışı da taşır; 40-55°
    # yatışta 200 °/s yaw talebi attitude payını yiyor. 120, 0-15 m bandının
    # medyan ihtiyacını (38.5 °/s) ve p90'ının (134) neredeyse tamamını
    # karşılar — eski sabit 45'in üç katı — ama motor payını tüketmez.
    YAW_RATE_TAVAN = math.radians(120.0)

    # --- KESME (öngörülü istasyon) ---
    # Neden gerekli: bkz. modül başlığı KADEME 1b. Saf takip, hedeften yavaş
    # bir avcıyla kapalı desende yakınsamaz — 1143 s'lik ölçümde menzil 82 m'de
    # kilitlendi. Öngörü, istasyonu hedefin GELECEK konumuna kurarak köşeyi
    # kestirir; hız üstünlüğü olmadan da menzil kapanır.
    KESME = os.environ.get("AVCI_GPS_KESME", "on").lower() not in ("0", "off", "false")
    # t_go = istasyon mesafesi / V_MAX. Tavan neden 6 s: kestirim hatası t_go
    # ile büyür; hedef 15 m/s'de 6 s'de 90 m gider, bu zaten desenin çapı
    # mertebesinde (ölçülen desen kutusu 133×132 m). Daha uzun öngörü desenin
    # dışına nişan alır.
    T_GO_MAX = _env_f("AVCI_GPS_TGO_MAX", 6.0)          # s
    # Ek tavan: bir çeyrek turdan fazlasını öngörme. omega·t_go bu açıyı aşarsa
    # t_go kısılır. Sabit-dönüş modeli hedefin yatışını SABİT varsayar; kare
    # senaryosunda yatış 2-3 s'de değişiyor, o yüzden yarım turdan azı güvenli.
    TAHMIN_ACI_MAX = math.radians(120.0)
    # omega kestirimi: hız vektörünün dönme hızı, EMA'lı ve fiziksel sınırda
    # kırpılmış. Talon 15 m/s'de 48° yatışla (senaryodaki en sert dönüş)
    # omega = g·tan(48°)/V = 0.73 rad/s ≈ 42 °/s yapar. 60 °/s tavanı bunun
    # üstünde pay bırakır ve gürültülü kareleri fizik dışı değerlere taşımaz.
    OMEGA_EMA = 0.25
    OMEGA_MAX = math.radians(60.0)                       # rad/s

    # --- HEDEF TELEMETRİ FİLTRESİ ---
    POS_EMA = 0.4
    VEL_EMA = 0.3
    HOLD_S = 3.0             # s; hedef telemetri bu kadar donuk kalırsa → DROPOUT

    # --- DURUM / DEVİR ETİKETİ (supervisor kendi GATE_MENZIL=20'yi kullanır) ---
    HANDOFF_RANGE = 20.0    # m; d_h altında durum=KILIT (görsel devir bandı)


# Telemetri/arayüz için son durum (gcs_server + supervisor.izci okur; salt gözlem)
status = {
    "durum": "WARMUP", "d_h": None, "menzil": None,
    "kadraj_yaw_deg": None, "kadraj_elev_deg": None, "none_count": 0,
    # Kuyruk açısı: hedefin KUYRUK yönü ile hedeften bize olan bakış arasındaki
    # açı. 0° = tam arkasındayız (yandanlık 0, pose'un en iyi çalıştığı geometri),
    # 90° = tam yandan (yandanlık 1). supervisor devir kapısında okur.
    "kuyruk_aci_deg": None,
}

# CSV çıktı dizini. AVCI_LEAD_LOG_DIR ile taşınabilir — testler bunu geçici bir
# dizine çevirir, yoksa test koşuları logs/ içine sahte CSV bırakır ve analiz
# scriptleri onları gerçek uçuş sanır (visual_lead.py ile aynı kural).
_LOG_DIR = os.environ.get("AVCI_LEAD_LOG_DIR") or os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

_CSV_ALANLAR = [
    "t", "dt", "durum", "d_h", "menzil",
    "tgt_x", "tgt_y", "tgt_z", "tgt_vx", "tgt_vy", "tgt_vz",
    "iris_x", "iris_y", "iris_z", "iris_roll_deg", "iris_pitch_deg", "iris_yaw_deg",
    "st_x", "st_y", "st_z", "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg",
    "kadraj_yaw_deg", "kadraj_elev_deg", "kadraj_pitch_hata_deg", "u_px", "v_px",
    # ── ÖLÇÜM KOLONLARI (2026-08-04) ──
    # "komut 20 m/s, konum türevi 8.3 m/s, eğim yalnız 14.6°" üçlü çelişkisini
    # ayırmak için eklendi. iris_v* ArduPilot'un KENDİ hız kestirimi
    # (LOCAL_POSITION_NED, yani EKF); konum türevinden farklıysa sorun EKF'te,
    # aynıysa araç komutu gerçekten uygulamıyor demektir. iris_egim_deg ise
    # hız kontrolcüsünün ne kadar itki isteği ürettiğini gösterir: komut ile
    # gerçek arasında 12 m/s fark varken eğim 15°'de kalıyorsa kontrolcü o
    # hatayı GÖRMÜYOR (aksi halde ANGLE_MAX=70°'e dayanırdı).
    "iris_vx", "iris_vy", "iris_vz", "iris_hiz", "iris_egim_deg",
    "v_cmd_mag", "tgt_hiz", "tgt_omega_deg", "t_go_s",
    # Devir kapısının geometri koşulu (supervisor okur) — 0° = tam kuyrukta.
    "kuyruk_aci_deg",
    # Yaw tavanının kerterize bağlanması (2026-08-04): gereken kerteriz hızı ve
    # o karede İZİN VERİLEN tavan. İkisi sürekli eşitse tavan hâlâ kısıtlıyor
    # demektir — KERTERIZ_PAY/YAW_RATE_TAVAN yükseltilmeli.
    "kerteriz_hizi_deg", "yaw_tavan_deg",
]


def hedef_tahmin(x, y, vx, vy, omega, t_go):
    """Sabit-dönüş (coordinated turn) modeliyle hedefin t_go sonraki hali.

    SAF HESAP — IO yok, birim test edilir (tests/test_gps_guidance.py).

    Uçak seviyeli koordineli dönüşte sabit yarıçaplı bir yay çizer: hız
    BÜYÜKLÜĞÜ korunur, yön omega hızıyla döner. Yarıçap R = V/omega.
        psi_t = psi + omega·t
        x(t)  = x + R·( sin(psi_t) − sin(psi) )
        y(t)  = y − R·( cos(psi_t) − cos(psi) )
    omega → 0 limitinde bu ifade düz uçuşa (x + vx·t) yakınsar; sayısal
    kararlılık için küçük omega'da doğrudan düz model kullanılır.

    Dönüş: (x_tahmin, y_tahmin, psi_tahmin) — psi radyan, NED'de kuzeyden saat
    yönü (atan2(vy, vx) ile aynı konvansiyon).
    """
    V = math.hypot(vx, vy)
    psi = math.atan2(vy, vx)
    if V < 1e-6 or t_go <= 0.0:
        return x, y, psi
    if abs(omega) < 1e-3:                       # düz uçuş (R > 15 km)
        return x + vx * t_go, y + vy * t_go, psi
    R = V / omega
    psi_t = psi + omega * t_go
    return (x + R * (math.sin(psi_t) - math.sin(psi)),
            y - R * (math.cos(psi_t) - math.cos(psi)),
            psi_t)


def tgo_hesapla(mesafe, v_max, omega, cfg=Cfg):
    """İstasyona uçuş süresi kestirimi, iki tavanla kırpılmış.

    (1) T_GO_MAX: mutlak tavan (kestirim hatası t_go ile büyür).
    (2) TAHMIN_ACI_MAX: |omega·t_go| bir çeyrek/yarım turu aşmasın — sabit
        yatış varsayımı o kadar uzun geçerli kalmaz.
    SAF HESAP — birim test edilir."""
    if v_max <= 0.0:
        return 0.0
    t = min(max(mesafe, 0.0) / v_max, cfg.T_GO_MAX)
    if abs(omega) > 1e-6:
        t = min(t, cfg.TAHMIN_ACI_MAX / abs(omega))
    return t


def run_gps_guidance(conn, get_plane, get_iris, stop_event, cfg=Cfg):
    loop_period = 1.0 / cfg.LOOP_HZ
    center_elev = math.radians(cfg.CENTER_ELEV_DEG)
    d_behind = cfg.RANGE_SET * math.cos(center_elev)     # yatay standoff (~9.97 m)
    d_below = cfg.RANGE_SET * math.sin(center_elev)      # dikey alt ofset (~4.65 m)

    # hedef kestirimi (EMA pozisyon + sonlu-fark hız)
    est_x = est_y = est_z = None
    vel_x = vel_y = vel_z = 0.0
    last_raw = None
    t_last_fresh = None
    none_count = 0

    de = [0.0, 0.0, 0.0]           # EMA'lı yatay/dikey hata türevi
    e_prev = None
    t_prev_deriv = None

    omega = 0.0                    # hedefin kestirilen dönüş hızı (rad/s)
    psi_prev = None                # önceki hedef hız-yönü (rad)
    bearing_onceki = None          # önceki kerteriz (yaw tavanını ölçeklemek için)
    kerteriz_hizi = 0.0            # |d(kerteriz)/dt| (rad/s)
    yaw_tavan = cfg.YAW_RATE_TABAN # o karede uygulanan yaw hız tavanı (log)

    vx_prev = vy_prev = vz_prev = 0.0
    cmd_yaw = None
    prev_time = None
    loop_count = 0

    os.makedirs(_LOG_DIR, exist_ok=True)
    csv_yol = os.path.join(_LOG_DIR, time.strftime("gps_guidance_%Y%m%d_%H%M%S.csv"))
    f = open(csv_yol, "w", newline="")
    w = csv.DictWriter(f, fieldnames=_CSV_ALANLAR, extrasaction="ignore")
    w.writeheader()

    print("=" * 60)
    print("[GPS] Kadraj güdümü (yeniden inşa) — hedefi kamera merkezine getir")
    print(f"[GPS] setpoint: slant {cfg.RANGE_SET:.1f}m → {d_behind:.1f}m arka + "
          f"{d_below:.1f}m alt (yükseliş {cfg.CENTER_ELEV_DEG:.0f}°) — log: {csv_yol}")
    print("=" * 60)

    def _hover():
        send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)

    try:
        while not stop_event.is_set():
            now = time.monotonic()
            dt = (now - prev_time) if prev_time is not None else loop_period
            dt = clamp(dt, 0.001, 0.2)
            prev_time = now

            iris = get_iris()
            ix, iy, iz = iris["x"], iris["y"], iris["z"]
            iroll = iris.get("roll", 0.0)
            ipitch = iris.get("pitch", 0.0)
            iyaw = iris.get("yaw", 0.0)
            # ArduPilot'un KENDİ hız kestirimi (EKF) — yalnız log/teşhis.
            ivx = iris.get("vx", 0.0)
            ivy = iris.get("vy", 0.0)
            ivz = iris.get("vz", 0.0)
            plane = get_plane()

            # ── 1) TAZELİK + FİLTRE (EMA pozisyon, sonlu-fark hız) ──
            raw = (plane["x"], plane["y"], plane["z"])
            frozen = bool(plane.get("frozen", False))
            fresh = (not frozen) and (raw != last_raw)
            if fresh:
                last_raw = raw
                none_count = 0
                if est_x is None:
                    est_x, est_y, est_z = raw
                else:
                    a = cfg.POS_EMA
                    nx = a * raw[0] + (1 - a) * est_x
                    ny = a * raw[1] + (1 - a) * est_y
                    nz = a * raw[2] + (1 - a) * est_z
                    if t_last_fresh is not None:
                        fdt = now - t_last_fresh
                        if 1e-3 < fdt < 2.0:
                            b = cfg.VEL_EMA
                            vel_x = b * ((nx - est_x) / fdt) + (1 - b) * vel_x
                            vel_y = b * ((ny - est_y) / fdt) + (1 - b) * vel_y
                            vel_z = b * ((nz - est_z) / fdt) + (1 - b) * vel_z
                            # dönüş hızı: hız VEKTÖRÜNÜN dönme oranı. Hedef
                            # duruyorsa/çok yavaşsa yön gürültüdür — okuma.
                            if math.hypot(vel_x, vel_y) >= cfg.TRACK_MIN_SPD:
                                psi = math.atan2(vel_y, vel_x)
                                if psi_prev is not None:
                                    w_ham = normalize_angle(psi - psi_prev) / fdt
                                    w_ham = clamp(w_ham, -cfg.OMEGA_MAX, cfg.OMEGA_MAX)
                                    omega = (cfg.OMEGA_EMA * w_ham
                                             + (1 - cfg.OMEGA_EMA) * omega)
                                psi_prev = psi
                            else:
                                psi_prev = None
                                omega = 0.0
                    est_x, est_y, est_z = nx, ny, nz
                t_last_fresh = now
            else:
                none_count += 1
            status["none_count"] = none_count

            # ── 2) WARMUP / DROPOUT ──
            if est_x is None:
                _hover()
                status.update(durum="WARMUP", d_h=None, menzil=None)
                loop_count += 1
                _sleep(now, loop_period)
                continue
            if none_count * loop_period > cfg.HOLD_S:
                _hover()
                vx_prev = vy_prev = vz_prev = 0.0
                status.update(durum="DROPOUT")
                loop_count += 1
                _sleep(now, loop_period)
                continue

            # ── 3) HATA / MENZİL (hedef kestirimine göre) ──
            ex = est_x - ix
            ey = est_y - iy
            d_h = math.hypot(ex, ey)
            menzil = math.sqrt(ex * ex + ey * ey + (est_z - iz) ** 2)

            # ── 4) KADRAJ NOKTASI (istasyon): ÖNGÖRÜLEN hedefin gerisi + altı ──
            # Saf takip yerine kesme: hedefin t_go sonraki yeri tahmin edilir ve
            # istasyon ORAYA kurulur (bkz. modül başlığı KADEME 1b). t_go,
            # istasyona olan mesafeden türer — yakınken 0'a gider ve davranış
            # eski saf-takiple ÖZDEŞ hale gelir (kilitte kararlılık korunur).
            tgt_spd_h = math.hypot(vel_x, vel_y)
            t_go = 0.0
            ff_x, ff_y = vel_x, vel_y                # feedforward hız vektörü
            if tgt_spd_h >= cfg.TRACK_MIN_SPD:
                if cfg.KESME:
                    # istasyona mesafenin kaba ölçüsü: hedefe olan mesafe eksi
                    # standoff (istasyonun kendisi henüz hesaplanmadı)
                    t_go = tgo_hesapla(max(menzil - cfg.RANGE_SET, 0.0),
                                       cfg.V_MAX, omega, cfg)
                px, py, psi_t = hedef_tahmin(est_x, est_y, vel_x, vel_y, omega, t_go)
                bx, by = -math.cos(psi_t), -math.sin(psi_t)   # ÖNGÖRÜLEN kuyruk yönü
                ff_x = tgt_spd_h * math.cos(psi_t)            # FF de öngörülen yönde
                ff_y = tgt_spd_h * math.sin(psi_t)
            else:
                px, py = est_x, est_y
                if d_h > 1e-6:
                    bx, by = -ex / d_h, -ey / d_h             # LOS gerisi (drone tarafı)
                else:
                    bx, by = 0.0, 0.0
            st_x = px + bx * d_behind
            st_y = py + by * d_behind
            # dikey de öngörülür: hedef tırmanıyorsa istasyonu bugünkü değil
            # varış anındaki irtifasının altına kur
            st_z = est_z + vel_z * t_go + d_below                 # NED: altında (+z aşağı)
            if -st_z < cfg.LOOKUP_MIN_ALT:                        # yere çakılma koruması
                st_z = -cfg.LOOKUP_MIN_ALT

            # ── 5) EMA TÜREV (istasyona hata) ──
            ex_cmd, ey_cmd, ez_cmd = st_x - ix, st_y - iy, st_z - iz
            e_now = (ex_cmd, ey_cmd, ez_cmd)
            if e_prev is not None and t_prev_deriv is not None:
                ddt = now - t_prev_deriv
                if ddt > 1e-3:
                    a = cfg.DERIV_EMA
                    for i in range(3):
                        de[i] = (1 - a) * de[i] + a * (e_now[i] - e_prev[i]) / ddt
            e_prev, t_prev_deriv = e_now, now

            # ── 6) HIZ KOMUTU: hedef-hızı FF + PD ──
            # FF, ÖNGÖRÜLEN hız yönünde (ff_x/ff_y): varış anında hedefin hangi
            # yöne gideceğine göre hizalanırız. Kilitte t_go→0 olduğundan bu
            # anlık hız vektörüne yakınsar, yani hold davranışı değişmez.
            vx = ff_x + cfg.KP_H * ex_cmd + cfg.KD_H * de[0]
            vy = ff_y + cfg.KP_H * ey_cmd + cfg.KD_H * de[1]
            vmag = math.hypot(vx, vy)
            if vmag > cfg.V_MAX and vmag > 1e-6:
                s = cfg.V_MAX / vmag
                vx *= s
                vy *= s
            vz = clamp(vel_z + cfg.KP_Z * ez_cmd, -cfg.VZ_MAX, cfg.VZ_MAX)

            # ── 7) YAW: burun GERÇEK hedefe, tavan KERTERİZ HIZIYLA ölçekli ──
            bearing = math.atan2(ey, ex)
            if cmd_yaw is None:
                cmd_yaw = bearing
            # kerteriz hızı = bakış açısının değişim hızı (rad/s). Bu, burnun
            # hedefte kalabilmesi için EN AZ yapması gereken dönme hızıdır.
            if bearing_onceki is not None and dt > 1e-6:
                kerteriz_hizi = abs(normalize_angle(bearing - bearing_onceki)) / dt
            else:
                kerteriz_hizi = 0.0
            bearing_onceki = bearing
            yaw_tavan = clamp(cfg.KERTERIZ_PAY * kerteriz_hizi + cfg.YAW_RATE_TABAN,
                              cfg.YAW_RATE_TABAN, cfg.YAW_RATE_TAVAN)
            yaw_err = normalize_angle(bearing - cmd_yaw)
            if abs(yaw_err) > cfg.YAW_DEADBAND:
                step = clamp(yaw_err, -yaw_tavan * dt, yaw_tavan * dt)
                cmd_yaw = normalize_angle(cmd_yaw + step)

            # ── 7b) KOMUT YÖNÜ DÖNME SINIRI (2026-08-04, agresif senaryo çöküşü) ──
            # limit_acceleration komutun BÜYÜKLÜK değişimini sınırlar ama yön
            # değişimini değil: |v| sabitken komut bir karede 90° dönebilir ve
            # ivme sınırı bunu "değişim yok" sanır. Oysa v hızında yönü omega
            # hızıyla döndürmek |v|·omega kadar YANAL ivme ister.
            #
            # Agresif senaryoda hedef sert manevra yapınca istasyon noktası
            # sıçrıyor, komut yönü bir anda dönüyor, gereken yanal ivme aracın
            # yapabileceğinin üstüne çıkıyor ve attitude kontrolcüsü tavana
            # yapışıp yetkiyi kaybediyor. Ölçüm (gps_guidance_20260804_155333):
            # yatış medyanı 14.1° (sakin) ama p99 66.1° — ATC_ANGLE_MAX 65'e
            # DAYALI — ve orada roll +102° → −156°, pitch +81° → −72° savruldu,
            # avcı düştü (t+143 s). Yatış > 80° olan 16 kare, hepsi o anda.
            #
            # Çare: yönü, ancak ayrılan yanal ivme bütçesi kadar döndür:
            #     omega_max = YON_ACCEL / |v|
            # Büyüklük dokunulmaz — yalnız yön yavaşlatılır, yani araç hedefe
            # gitmeye devam eder, sadece dönüşü fiziğe uydurulur.
            vmag_yeni = math.hypot(vx, vy)
            vmag_onceki = math.hypot(vx_prev, vy_prev)
            if vmag_yeni > 1e-6 and vmag_onceki > cfg.YON_LIMIT_MIN_HIZ:
                aci_ham = normalize_angle(math.atan2(vy, vx)
                                          - math.atan2(vy_prev, vx_prev))
                aci_tavan = (cfg.YON_ACCEL / vmag_onceki) * dt        # rad
                if abs(aci_ham) > aci_tavan:
                    yeni_aci = (math.atan2(vy_prev, vx_prev)
                                + math.copysign(aci_tavan, aci_ham))
                    vx = vmag_yeni * math.cos(yeni_aci)
                    vy = vmag_yeni * math.sin(yeni_aci)

            # ── 8) İVME SINIRI + GÖNDER ──
            vx, vy, vz = limit_acceleration(
                vx, vy, vz, vx_prev, vy_prev, vz_prev, cfg.MAX_ACCEL, dt)
            vx_prev, vy_prev, vz_prev = vx, vy, vz
            send_velocity(conn, vx, vy, vz, cmd_yaw)

            # ── 9) KADRAJ HATASI (başarı ölçütü) — gerçek attitude'la ──
            kad = hedef_kadraj_hatasi((est_x, est_y, est_z), (ix, iy, iz),
                                      iroll, ipitch, iyaw)

            # ── 10) DURUM ──
            durum = "KILIT" if d_h < cfg.HANDOFF_RANGE else "ARAMA"
            # kuyruk açısı: hedefin kuyruk yönü (−hız) ile hedeften bize olan
            # bakış arasındaki açı. 0° = tam arkasındayız. Devir kapısı bunu
            # okur — yandan devralınan görsel faz 0.3 s'de temas kaybediyor
            # (ölçüm: visual_lead_20260801_173610, yandanlik_f ≈ 0.9-1.0).
            kuyruk_aci = None
            if tgt_spd_h >= cfg.TRACK_MIN_SPD and d_h > 1e-6:
                kuyruk_aci = math.degrees(math.acos(clamp(
                    ((-vel_x / tgt_spd_h) * (-ex / d_h)
                     + (-vel_y / tgt_spd_h) * (-ey / d_h)), -1.0, 1.0)))
            status.update(durum=durum, d_h=round(d_h, 1), menzil=round(menzil, 1),
                          kadraj_yaw_deg=round(math.degrees(kad["yaw_hata"]), 1),
                          kadraj_elev_deg=round(math.degrees(kad["elev"]), 1),
                          kuyruk_aci_deg=(round(kuyruk_aci, 1)
                                          if kuyruk_aci is not None else None))

            w.writerow({
                "t": round(now, 3), "dt": round(dt, 4), "durum": durum,
                "d_h": round(d_h, 2), "menzil": round(menzil, 2),
                "tgt_x": round(est_x, 2), "tgt_y": round(est_y, 2), "tgt_z": round(est_z, 2),
                "tgt_vx": round(vel_x, 2), "tgt_vy": round(vel_y, 2), "tgt_vz": round(vel_z, 2),
                "iris_x": round(ix, 2), "iris_y": round(iy, 2), "iris_z": round(iz, 2),
                "iris_roll_deg": round(math.degrees(iroll), 1),
                "iris_pitch_deg": round(math.degrees(ipitch), 1),
                "iris_yaw_deg": round(math.degrees(iyaw), 1),
                "st_x": round(st_x, 2), "st_y": round(st_y, 2), "st_z": round(st_z, 2),
                "vx_cmd": round(vx, 2), "vy_cmd": round(vy, 2), "vz_cmd": round(vz, 2),
                "yaw_cmd_deg": round(math.degrees(cmd_yaw), 1),
                "kadraj_yaw_deg": round(math.degrees(kad["yaw_hata"]), 2),
                "kadraj_elev_deg": round(math.degrees(kad["elev"]), 2),
                "kadraj_pitch_hata_deg": round(math.degrees(kad["pitch_hata"]), 2),
                "u_px": round(kad["u"], 1) if kad["u"] is not None else "",
                "v_px": round(kad["v"], 1) if kad["v"] is not None else "",
                "iris_vx": round(ivx, 2), "iris_vy": round(ivy, 2),
                "iris_vz": round(ivz, 2), "iris_hiz": round(math.hypot(ivx, ivy), 2),
                "iris_egim_deg": round(math.degrees(math.acos(clamp(
                    math.cos(iroll) * math.cos(ipitch), -1.0, 1.0))), 1),
                "v_cmd_mag": round(math.hypot(vx, vy), 2),
                "tgt_hiz": round(tgt_spd_h, 2),
                "tgt_omega_deg": round(math.degrees(omega), 1),
                "t_go_s": round(t_go, 2),
                "kuyruk_aci_deg": (round(kuyruk_aci, 1)
                                   if kuyruk_aci is not None else ""),
                "kerteriz_hizi_deg": round(math.degrees(kerteriz_hizi), 1),
                "yaw_tavan_deg": round(math.degrees(yaw_tavan), 1),
            })
            f.flush()

            loop_count += 1
            if loop_count % int(cfg.LOOP_HZ * 3) == 0:
                # "komut ne, GERÇEKLEŞEN ne, hedef ne" üçlüsü aynı satırda:
                # menzil kapanmıyorsa sebebin hız mı geometri mi olduğu buradan
                # anlaşılır. gercek < hedef ise avcı fiziksel olarak yetişemiyor
                # demektir ve güdüm ayarı bunu çözemez.
                print(f"[GPS] {durum} d_h={d_h:.1f}m menzil={menzil:.1f}m "
                      f"kadraj(yaw={math.degrees(kad['yaw_hata']):+.0f}°,"
                      f"elev={math.degrees(kad['elev']):+.0f}°/hedef {cfg.CENTER_ELEV_DEG:.0f}°) "
                      f"hiz(komut={math.hypot(vx, vy):.1f} gercek={math.hypot(ivx, ivy):.1f} "
                      f"hedef={tgt_spd_h:.1f}) egim={math.degrees(math.acos(clamp(math.cos(iroll)*math.cos(ipitch), -1.0, 1.0))):.0f}° "
                      f"vz={vz:+.1f} t_go={t_go:.1f}s omega={math.degrees(omega):+.0f}°/s")

            _sleep(now, loop_period)

        send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)
        status.update(durum="DURDU")
        print("[GPS] Stop sinyali — döngü sonlandı.")
    finally:
        f.close()
        print(f"[GPS] log kapatıldı: {csv_yol}")
        _panel_tazele()


def _panel_tazele():
    """Uçuş biter bitmez log panelini yeniden üret ve linkini yazdır.

    Panel HER GPS fazının sonunda tazelenir; en yeni uçuşlar otomatik girer,
    ayrıca elle `python3 tools/gps_log_viz.py` çalıştırmaya gerek kalmaz.
    Panel üretimi uçuşu asla düşürmemeli → tüm hatalar yutulur.
    """
    try:
        from tools.gps_log_viz import panel_uret, _VARSAYILAN_CIKTI
        yol = panel_uret(last=12, out=_VARSAYILAN_CIKTI, sessiz=True)
        if yol:
            print(f"[GPS] Log paneli güncellendi → http://localhost:8000/loglar/"
                  f"{os.path.basename(yol)}")
            print(f"[GPS]   (GCS kapalıysa: file://{os.path.abspath(yol)})")
    except Exception as e:
        print(f"[GPS] Log paneli üretilemedi ({e}) — uçuş etkilenmedi.")


def _sleep(t_start, period):
    elapsed = time.monotonic() - t_start
    if elapsed < period:
        time.sleep(period - elapsed)
