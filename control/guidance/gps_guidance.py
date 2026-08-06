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

KADEME 1 (bu sürüm): GEOMETRİK kadraj-noktası takibi. Hedefin hız yönünün
D_BEHIND gerisine + D_BELOW altına (slant RANGE_SET'te +25° yükseliş verecek)
bir istasyon kur; oraya PD hız + hedef-hızı feedforward ile git (feedforward →
kilitlenince kararlı hold). Burun daima gerçek hedefe döner (yaw). Drone hedefin
ALTINDA kalır → gökyüzü arka planı, pose kopmaz.
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
    CENTER_ELEV_DEG = 25.0    # kamera tilt'i (FİZİKSEL, iris_cam modelinden).
                              # Hedefin kadrajın TAM MERKEZİNDE görünmesi için
                              # gereken LOS yükselişi. Ölçüm/tanı referansı;
                              # istasyon geometrisini ARTIK BELİRLEMİYOR.
    RANGE_SET = _env_f("AVCI_GPS_RANGE", 11.0)   # m; slant menzil setpoint (pose tatlı nokta)

    # İSTASYONUN LOS yükselişi — kamera tilt'inden AYRILDI (2026-08-02).
    #
    # NEDEN AYRILDI: ikisi tek sayıya bağlıydı (25°), dolayısıyla istasyon
    # RANGE_SET·sin25° = 4.65 m ALTTA kuruluyordu. Terminal hücum bu 4.65 m'yi
    # kapatmak zorunda ve ÖLÇÜLDÜ (3 uçuş, kara kutu): ArduPilot dikey hız
    # komutunu WP_ACC_Z = 1.0 m/s² ile rampalıyor — güdüm 8-22 m/s tırmanma
    # istese de. Sıfırdan 4.65 m kapatmak 3.05 s sürer; terminalde eldeki süre
    # 2.4-2.8 s. Yani geometri, aracın dikey ivme bütçesine SIĞMIYORDU:
    #     vurdu   → kalan dikey +0.03 m   (rampayı erken başlatabildiği için)
    #     ıskaladı→ kalan dikey +1.52 m, +2.06 m  (drone hedefin ALTINDAN geçti)
    #
    # 15°'de istasyon RANGE_SET·sin15° = 2.85 m altta, 10.63 m geride:
    #     eldeki süre 10.63 / 3.7 m/s ≈ 2.87 s → 1 m/s² ile 4.13 m tırmanılır
    #     gereken 2.85 m → %45 pay. En hızlı ölçülen kapanmada (4.3 m/s) bile
    #     2.47 s → 3.05 m, yine yeter.
    # BEDELİ: hedef kadraj merkezinde değil, ~10° altında görünür
    # (v_px ≈ 269/480 — hâlâ rahat içeride). G11 bütçeyi test olarak koruyor.
    #
    # ⚠ BU AYRIM HENÜZ KESİNLEŞMEDİ — bkz. UYGULANACAK.md B7.
    # 25° tesadüf değildi, kamera tilt'i o; istasyon 25°'de kurulunca hedef
    # kadrajın TAM MERKEZİNDE oluyordu. Ölçüm 15°'yi destekliyor (terminal
    # `ok` oranı %8.7 → %18.2, kadraj içi %59.8 → %67.0, en yakın menzil
    # medyanı 5.25 → 1.73 m) AMA bu kanıt karışık: algının iyileşmesi büyük
    # ölçüde geometrinin SONUCU (drone seviyeye yakın kalınca hedef kadrajdan
    # geç çıkıyor). Merkez dışı kadrajlamanın KENDİ bedeli izole ölçülmedi.
    #
    # İki şey denenmedi:
    #   (1) 15° taranarak seçilmedi, ivme bütçesi hesabından çıktı — bütçeye
    #       sığan en büyük açı (18°? 20°?) merkeze daha yakın olurdu.
    #   (2) Asıl alternatif: istasyonu 25°'de bırakıp WP_ACC_Z'yi 1.0 → 2.5
    #       yükseltmek. Tutarsa bu ayrım gereksiz hale gelir.
    ISTASYON_ELEV_DEG = _env_f("AVCI_GPS_ISTASYON_ELEV", 15.0)
    TRACK_MIN_SPD = 3.0       # m/s; üstünde istasyon HIZ yönünün gerisi (kuyruk), altında LOS gerisi
    LOOKUP_MIN_ALT = 8.0      # m; alçalma tabanı (yere çakılma koruması)

    # --- HIZ KONTROLÜ ---
    KP_H = _env_f("AVCI_GPS_KP", 0.8)   # yatay konum hatası → hız (1/s)

    # KD_H — bu terim SÖNÜMLEME DEĞİL, LEAD'in kendisidir (2026-08-05 bulgusu).
    # de[] istasyon hatasının türevidir, yani ≈ göreli hız Δv. Yasa açılınca:
    #     v_cmd = v_hedef + KP_H·Δp + KD_H·Δv
    # FRPN'in hız formu da aynı üç terimli yapıda; oradaki karşılığı K_ZEM.
    # Yani "hedefin gideceği yere nişan alma" miktarını bu katsayı belirliyor:
    # küçükse hedefin izini birebir tekrarlarsın, büyükse aşıp salınırsın.
    # Denge analizi: Δv_yeni⊥ = −K·Δv_eski⊥ → K=1 söndürmez, salındırır.
    # F3 taraması (tools/frpn_replay.py --tara) 0.60'ı buldu.
    #
    # UÇUŞTA DOĞRULANDI → VARSAYILAN 0.20'DEN 0.60'A ÇEKİLDİ (2026-08-05).
    # Aynı senaryoda (daire, hedef 14.4-14.6 m/s, dönüş 21.4-21.9°/s), görev
    # başından hizalanmış 30 s'lik dilimlerde oturmuş menzil:
    #     KD_H=0.20 → 34.3 m      KD_H=0.60 → 29.4 m      (FRPN → 31.1 m)
    # Üç koşu da 150 s boyunca ±0.3 m içinde kararlı kaldı, yani fark gürültü
    # değil. Eskisine dönmek için: AVCI_GPS_KD=0.2
    KD_H = _env_f("AVCI_GPS_KD", 0.60)

    # ── İÇ DAİRE NİŞANI (2026-08-05) ──
    # SORUN: istasyon "hedefin hız yönünün gerisi"ne konuyor. Hedef daire
    # çizerken o nokta hedefin KENDİ ÇEMBERİNİN ÜZERİNDEDİR. Drone onu
    # kovaladığı sürece aynı yarıçapta uçmak zorunda, dolayısıyla aynı hıza
    # muhtaç. Ölçüldü (2026-08-05, 6 koşu): drone yarıçapı 38 m = hedef
    # yarıçapı 38 m, menzil 29-34 m'de donuyor.
    #
    # Dairesel kovalamacada zorunlu bağ:  yarıçap = hız / açısal_hız
    # Hedefin açısal hızı sabit olduğuna göre drone'u HIZLANDIRMAK çemberini
    # BÜYÜTÜR. Bu deneyle doğrulandı: V_MAX 18→24 yapılınca drone yarıçapı
    # 38→43 m'ye çıktı ve menzil 29→35-41 m'ye AÇILDI. Yani güç eklemek
    # ters teptik.
    #
    # ÇÖZÜM: istasyonu dönüşün İÇİNE kaydır. Drone daha küçük yarıçapta,
    # DAHA AZ hızla aynı açısal hızı tutturur ve hedefe yaklaşır:
    #     34 m yarıçap → 12.8 m/s gerekir → hedefe ~4 m
    #     30 m yarıçap → 11.3 m/s gerekir → hedefe ~8 m
    # Drone zaten 13-15 m/s yapabiliyor; ekstra güce ihtiyaç YOK.
    #
    # Kayma yönü = merkezcil ivme yönü = hız vektörünün dönüş yönünde 90°'si.
    # Hedef DÜZ uçarken açısal hız ~0 olur ve kayma kendiliğinden sıfırlanır —
    # düz kovalama durumunda regresyon riski yok (kritik: en iyi bilinen
    # sonucumuz bu yolda bozulmamalı).
    #
    # UÇUŞTA ÖLÇÜLDÜ (2026-08-05, aynı senaryoda üç koşu):
    #     kayma   menzil(medyan)  en yakın   drone R − hedef R
    #       0 m       34.1 m        31.3 m        +2 m  (aynı çember)
    #       8 m       22.8 m         6.9 m        −7 m  (İÇERİDE)
    #      14 m        9.8 m         3.2 m       −11 m  (İÇERİDE)
    # Mekanizma doğrulandı: drone artık hedefin çemberinin İÇİNDE uçuyor.
    # 34.1 → 9.8 m; GPS fazının hedefi (görsel faza devredilebilir konum)
    # fazlasıyla tutturuldu. Kayma başına kazanç 8→14 aralığında artıyor
    # (1.41 → 2.17 m/m), yani eğri henüz doymamış; 14 yeter görüldüğü için
    # daha ileri gidilmedi — GPS fazının işi çarpmak değil devretmek.
    # ⚠ Bu SABİT METRE bir kaymadır. Çok dar dairede (uçağın yapabileceği en
    # dar ~24 m yarıçap) fazla içeri iter. Yarıçap-oranlı sürüm sıradaki iş.
    #
    # DÜZ UÇUŞ REGRESYON TESTİ YAPILDI: kare deseninde düz kenarlarda davranış
    # bozulmadı — ölçekleme (ω→0 ⇒ kayma→0) uçuşta da doğrulandı.
    # Kapatmak için: AVCI_GPS_IC=0
    IC_KAYMA = _env_f("AVCI_GPS_IC", 14.0)     # m; dönüş merkezine doğru kayma
    IC_OMEGA_REF = 0.15                        # rad/s; bu dönüş hızında tam kayma
    IC_OMEGA_EMA = 0.15                        # açısal hız kestirimi yumuşatması

    # ── YARIÇAP-ORANLI KAYMA (2026-08-05, sabit-metre sürümünün devamı) ──
    # SABİT METRENİN AÇIĞI: 14 m, bu senaryonun dairesi (hedef R ≈ 52 m) için
    # ölçülmüş doğru değer. Ama hedef DAR bir daire çizerse (uçağın yapabileceği
    # en dar ~24 m yarıçap) aynı 14 m nişanı merkeze fazla yaklaştırır: drone
    # gereğinden içeride uçar, hedefe 14 m kalır — oysa oranlı olsa ~6 m olurdu.
    # Tehlikeli değil ama performans kaybı; ve dar daire yarışmada olası.
    #
    # ÇÖZÜM: kaymayı hedefin DÖNÜŞ YARIÇAPININ oranı yap. Yarıçap zaten
    # elimizde: R = |v_hedef| / |ω|  (ikisini de ölçüyoruz).
    #     kayma = IC_ORAN × R,  IC_KAYMA_MAX ile tavanlı
    #
    # KATSAYI ÖLÇÜMDEN: 2026-08-05 uçuşunda 14 m kayma, hedefin 52.2 m'lik
    # yarıçabının 0.268'iydi → IC_ORAN = 0.27. Böylece bu senaryoda oranlı
    # sürüm sabit sürümle AYNI kaymayı üretir (14.1 m); fark yalnız yarıçap
    # değişince ortaya çıkar — istenen davranış bu.
    #
    # ⚠ TEORİK TAHMİN TUTMADI, ölçüme uyuldu. Geometrik beklenti
    # (1 − v_drone/v_hedef ≈ 0.06) gerçeğin dörtte biriydi; çünkü drone hedefin
    # çemberini birebir izlemiyor ve istasyonun 10.6 m'lik "arka" bileşeni de
    # menzile katkı veriyor. Katsayı teoriden değil uçuştan alınmıştır.
    #
    # VARSAYILAN 0.0 = KAPALI (sabit metre kullanılır). Denemek için:
    #     AVCI_GPS_IC_ORAN=0.27
    IC_ORAN = _env_f("AVCI_GPS_IC_ORAN", 0.0)  # 0 = kapalı, sabit IC_KAYMA geçerli
    IC_KAYMA_MAX = _env_f("AVCI_GPS_IC_MAX", 25.0)   # m; oranlı kaymanın tavanı
    IC_R_MIN = 15.0            # m; bundan dar yarıçap kestirimi güvenilmez sayılır
    KP_Z = 1.0               # dikey konum hatası → hız (1/s)
    VZ_MAX = 6.0              # m/s; dikey hız tavanı (eski 3.5 darboğazı açıldı)
    # V_MAX 20→28 (2026-07-31): telemetri 4→25 Hz düzeltilince hedefin GERÇEK hızı
    # ortaya çıktı — 18-23 m/s (4 Hz'de EMA sönümlemesi 14-15 gösteriyordu). 20 m/s
    # tavanında komut %98 doygundu: hedef 19-23 giderken drone tavanda kalınca
    # yaklaşma hızı ≈ 0, açı hiç kapanmıyordu. Yüksek hızda eski salınımın sebebi
    # 250 ms telemetri faz gecikmesiydi; 25 Hz ile ~40 ms'e indi.
    # 2026-08-01: 28 → 18. 28 m/s'den MAX_ACCEL=12 m/s² ile durma mesafesi
    # v²/2a = 32.7 m, oysa istasyon standoff'u yalnız 10 m yatay — araç
    # geometrik olarak zamanında yavaşlayamıyor, hedefin etrafında savruluyor.
    # ⚠ TODO: main branch 20.0 kullanıyor — merge sonrası 18 vs 20 karşılaştırma testi yapılacak.
    V_MAX = _env_f("AVCI_GPS_V_MAX", 18.0)   # m/s; yatay hız tavanı
    MAX_ACCEL = 12.0         # m/s²; komut hızı değişim sınırı
    DERIV_EMA = 0.2

    # --- YAW ---
    YAW_DEADBAND = math.radians(3.0)
    YAW_RATE_MAX = math.radians(120.0)

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
}

_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

# ⚠ "menzil" ve "tgt_*" KESTİRİMDEN hesaplanır (EMA'lı est_x/y/z), HAM
# telemetriden değil. Bu ayrım 2026-08-06'da kafa karışıklığı yarattı:
# arayüz ham telemetriyi kullandığı için köşelerde CSV 15 m derken panel 21 m
# gösteriyordu ve hangisinin doğru olduğu ayırt edilemiyordu.
# Artık HAM konum ve ona olan mesafe de yazılıyor:
#     tgt_ham_*  : get_plane()'in verdiği konum (EMA'dan ÖNCE)
#     menzil_ham : drone → ham konum mesafesi  (panelin gördüğü sayı)
#     kestirim_gecikme_m : |ham − kestirim|, yani filtrenin ne kadar geriden
#                          geldiği. Manevrada bu büyür; köşe analizinin anahtarı.
_CSV_ALANLAR = [
    "t", "dt", "durum", "d_h", "menzil",
    "menzil_ham", "kestirim_gecikme_m",
    "tgt_ham_x", "tgt_ham_y", "tgt_ham_z",
    "tgt_x", "tgt_y", "tgt_z", "tgt_vx", "tgt_vy", "tgt_vz",
    "iris_x", "iris_y", "iris_z", "iris_roll_deg", "iris_pitch_deg", "iris_yaw_deg",
    "st_x", "st_y", "st_z", "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg",
    "kadraj_yaw_deg", "kadraj_elev_deg", "kadraj_pitch_hata_deg", "u_px", "v_px",
]


def run_gps_guidance(conn, get_plane, get_iris, stop_event, cfg=Cfg):
    loop_period = 1.0 / cfg.LOOP_HZ
    ist_elev = math.radians(cfg.ISTASYON_ELEV_DEG)
    d_behind = cfg.RANGE_SET * math.cos(ist_elev)        # yatay standoff (15°'de ~10.63 m)
    d_below = cfg.RANGE_SET * math.sin(ist_elev)         # dikey alt ofset (15°'de ~2.85 m)

    # hedef kestirimi (EMA pozisyon + sonlu-fark hız)
    est_x = est_y = est_z = None
    vel_x = vel_y = vel_z = 0.0
    tgt_hdg_prev = None       # hedefin hız yönü (rad) — dönüş hızı için
    tgt_omega = 0.0           # hedefin açısal hızı (rad/s, işaretli), EMA'lı
    last_raw = None
    t_last_fresh = None
    none_count = 0

    de = [0.0, 0.0, 0.0]           # EMA'lı yatay/dikey hata türevi
    e_prev = None
    t_prev_deriv = None

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
          f"{d_below:.1f}m alt; yakınlaşınca ofset menzille ORANTILI küçülür, "
          f"yükseliş her menzilde {cfg.ISTASYON_ELEV_DEG:.0f}° kalır "
          f"(kamera tilt'i {cfg.CENTER_ELEV_DEG:.0f}° → hedef merkezin "
          f"{cfg.CENTER_ELEV_DEG - cfg.ISTASYON_ELEV_DEG:.0f}° altında) — log: {csv_yol}")
    _t_terminal = d_behind / 3.7          # ölçülen terminal yatay kapanma hızı
    print(f"[GPS] terminal dikey bütçe: {d_below:.2f} m kapatılacak, "
          f"~{_t_terminal:.2f} s var → 1 m/s² rampayla {0.5*_t_terminal**2:.2f} m "
          f"{'YETER' if 0.5*_t_terminal**2 > d_below else '⚠ YETMEZ'}")
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
                    est_x, est_y, est_z = nx, ny, nz
                # Hedefin AÇISAL HIZI (işaretli) — iç daire nişanı için.
                # Yalnız taze telemetride güncellenir; ara karelerde hız
                # değişmediği için burada hesaplamak zorunlu (her döngüde
                # hesaplansaydı taze olmayan karelerde 0'a sönerdi).
                if t_last_fresh is not None:
                    fdt2 = now - t_last_fresh
                    spd2 = math.hypot(vel_x, vel_y)
                    if 1e-3 < fdt2 < 2.0 and spd2 >= cfg.TRACK_MIN_SPD:
                        hdg = math.atan2(vel_y, vel_x)
                        if tgt_hdg_prev is not None:
                            dh = normalize_angle(hdg - tgt_hdg_prev)
                            w_ham = dh / fdt2
                            if abs(w_ham) < 3.0:      # gürültü ayıklama
                                a_w = cfg.IC_OMEGA_EMA
                                tgt_omega = a_w * w_ham + (1 - a_w) * tgt_omega
                        tgt_hdg_prev = hdg
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

            # ── 4) KADRAJ NOKTASI (istasyon): hedefin gerisi + altı ──
            #
            # SABİT METRE DEĞİL, SABİT AÇI (2026-08-01 dikey ıska düzeltmesi).
            # Eskiden ofset RANGE_SET'ten bir kez hesaplanıp sabit metre olarak
            # kullanılıyordu (d_behind 9.97 m, d_below 4.65 m). Ama sabit metre
            # kapanan menzilde sabit açı DEĞİLDİR — drone RANGE_SET'ten daha
            # yakına girdiğinde aynı 4.65 m giderek büyüyen bir LOS yükselişine
            # dönüşüyordu:
            #     menzil 11 m → 25° (kadraj merkezi, kamera tilt'i)
            #     menzil  8 m → 35°
            #     menzil  6 m → 51°
            #     menzil  4 m → >90°  (kadrajın DIŞI; üst sınır +80.2°)
            # Hedef kadrajın tepesinden çıkıyor, tespit kopuyor, drone altından
            # geçiyordu. Yani tasarım, korumak istediği görsel temasını yakın
            # menzilde kendi bozuyordu.
            #
            # Düzeltme: etkin standoff, menzil RANGE_SET'in altına inince onunla
            # birlikte küçülür. Böylece LOS yükselişi HER menzilde
            # ISTASYON_ELEV_DEG kalır. Uzakta (menzil ≥ RANGE_SET) davranış
            # AYNEN eskisi gibidir.
            # NOT: bu açı 2026-08-02'de kamera tilt'inden (CENTER_ELEV_DEG=25°)
            # AYRILDI ve 15°'ye indirildi — terminalin kapatması gereken dikey
            # mesafe aracın 1 m/s²'lik dikey ivme bütçesine sığmıyordu. Ayrıntı
            # ve ölçüm: Cfg.ISTASYON_ELEV_DEG.
            r_eff = min(menzil, cfg.RANGE_SET)
            d_behind_eff = r_eff * math.cos(ist_elev)
            d_below_eff = r_eff * math.sin(ist_elev)

            tgt_spd_h = math.hypot(vel_x, vel_y)
            if tgt_spd_h >= cfg.TRACK_MIN_SPD:
                bx, by = -vel_x / tgt_spd_h, -vel_y / tgt_spd_h   # hız yönünün gerisi (kuyruk)
            elif d_h > 1e-6:
                bx, by = -ex / d_h, -ey / d_h                     # LOS gerisi (drone tarafı)
            else:
                bx, by = 0.0, 0.0
            st_x = est_x + bx * d_behind_eff
            st_y = est_y + by * d_behind_eff
            st_z = est_z + d_below_eff                            # NED: altında (+z aşağı)

            # ── İÇ DAİRE KAYMASI (bkz. Cfg.IC_KAYMA; varsayılan 0 = kapalı) ──
            # Merkezcil yön: hız birim vektörünün dönüş yönünde 90°'si.
            # NED'de (x kuzey, y doğu) başlık atan2(vy,vx) ARTARKEN hız vektörü
            # x'ten y'ye döner; o dönüşün merkezi (-v̂y, +v̂x) yönündedir.
            # İşaret tgt_omega'dan gelir → sağa ve sola dönüşte doğru taraf.
            ic_kayma = 0.0
            ic_yaricap = None
            if tgt_spd_h >= cfg.TRACK_MIN_SPD:
                olcek = min(1.0, abs(tgt_omega) / cfg.IC_OMEGA_REF)
                if cfg.IC_ORAN > 0.0:
                    # YARIÇAP-ORANLI: R = |v| / |ω|. Dar dairede küçük, geniş
                    # dairede büyük kayma → tek katsayı her yarıçapta doğru.
                    if abs(tgt_omega) > 1e-6:
                        ic_yaricap = tgt_spd_h / abs(tgt_omega)
                        if ic_yaricap >= cfg.IC_R_MIN:
                            ic_kayma = min(cfg.IC_ORAN * ic_yaricap,
                                           cfg.IC_KAYMA_MAX) * olcek
                else:
                    # SABİT METRE (uçuşta doğrulanmış varsayılan)
                    ic_kayma = cfg.IC_KAYMA * olcek
                if ic_kayma > 1e-6:
                    vhx, vhy = vel_x / tgt_spd_h, vel_y / tgt_spd_h
                    isaret = 1.0 if tgt_omega >= 0 else -1.0
                    cx_, cy_ = -vhy * isaret, vhx * isaret     # merkeze doğru
                    st_x += cx_ * ic_kayma
                    st_y += cy_ * ic_kayma
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
            vx = vel_x + cfg.KP_H * ex_cmd + cfg.KD_H * de[0]
            vy = vel_y + cfg.KP_H * ey_cmd + cfg.KD_H * de[1]
            vmag = math.hypot(vx, vy)
            if vmag > cfg.V_MAX and vmag > 1e-6:
                s = cfg.V_MAX / vmag
                vx *= s
                vy *= s
            vz = clamp(vel_z + cfg.KP_Z * ez_cmd, -cfg.VZ_MAX, cfg.VZ_MAX)

            # ── 7) YAW: burun GERÇEK hedefe ──
            bearing = math.atan2(ey, ex)
            if cmd_yaw is None:
                cmd_yaw = bearing
            yaw_err = normalize_angle(bearing - cmd_yaw)
            if abs(yaw_err) > cfg.YAW_DEADBAND:
                step = clamp(yaw_err, -cfg.YAW_RATE_MAX * dt, cfg.YAW_RATE_MAX * dt)
                cmd_yaw = normalize_angle(cmd_yaw + step)

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
            status.update(durum=durum, d_h=round(d_h, 1), menzil=round(menzil, 1),
                          kadraj_yaw_deg=round(math.degrees(kad["yaw_hata"]), 1),
                          kadraj_elev_deg=round(math.degrees(kad["elev"]), 1))

            w.writerow({
                "t": round(now, 3), "dt": round(dt, 4), "durum": durum,
                "d_h": round(d_h, 2), "menzil": round(menzil, 2),
                # HAM telemetri (EMA'dan önce) + ona olan mesafe + filtre gecikmesi
                "tgt_ham_x": round(raw[0], 2), "tgt_ham_y": round(raw[1], 2),
                "tgt_ham_z": round(raw[2], 2),
                "menzil_ham": round(math.sqrt((raw[0] - ix) ** 2 + (raw[1] - iy) ** 2
                                              + (raw[2] - iz) ** 2), 2),
                "kestirim_gecikme_m": round(math.sqrt((raw[0] - est_x) ** 2
                                                      + (raw[1] - est_y) ** 2
                                                      + (raw[2] - est_z) ** 2), 2),
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
            })
            f.flush()

            loop_count += 1
            if loop_count % int(cfg.LOOP_HZ * 3) == 0:
                print(f"[GPS] {durum} d_h={d_h:.1f}m menzil={menzil:.1f}m "
                      f"kadraj(yaw={math.degrees(kad['yaw_hata']):+.0f}°,"
                      f"elev={math.degrees(kad['elev']):+.0f}°/istasyon {cfg.ISTASYON_ELEV_DEG:.0f}°) "
                      f"v=({vx:+.1f},{vy:+.1f},{vz:+.1f}) tgt_v={tgt_spd_h:.1f}")

            _sleep(now, loop_period)

        send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)
        status.update(durum="DURDU")
        print("[GPS] Stop sinyali — döngü sonlandı.")
    finally:
        f.close()
        print(f"[GPS] log kapatıldı: {csv_yol}")


def _sleep(t_start, period):
    elapsed = time.monotonic() - t_start
    if elapsed < period:
        time.sleep(period - elapsed)
