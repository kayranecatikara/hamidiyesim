"""
guidance_core.py — IBVS lead pursuit çekirdeği (PLATFORMDAN BAĞIMSIZ, Adım 1-8).

Pose modelinin 6 keypoint'inden (burun, kuyruk, kanat uçları) hedefin görünür
yönelimini çıkarır ve saf takip yönünün üstüne MENZİLDEN BAĞIMSIZ bir öne nişan
(lead) kaydırması bindirir. Hedefin hızı ve mesafesi ÖLÇÜLMEZ; tek ayar K_LEAD
(≈ hedef_hızı / bizim_hız).

Çıktı bir YÖNDÜR: u_govde (FRD birim vektör) ve ondan türeyen yaw_hata /
pitch_hata. Bu geometri platformdan bağımsızdır — platforma bağlı komut üretimi
adaptördedir (adapter_copter).

Kritik tasarım kuralları (master spec):
  - Kaydırma PİKSEL uzayında DEĞİL, yön vektörü uzayında yapılır (FOV 125°
    geniş: kenarda 1 piksel, merkezdekinin ~çeyreği kadar açı eder).
  - Kamera açıları doğrudan komut olmaz; önce gövde çerçevesine çevrilir
    (25° yukarı montaj tilt'i — atlanırsa sürekli 25° sabit hata).
  - dt daima kare header.stamp farkından gelir, duvar saatinden DEĞİL.
  - PnP/menzil kestirimi güdüme bağlanmaz; menzil_kestirim_m SADECE log.
"""

import math
import os

import numpy as np

from vision import geometry as geo

# ══ Talon fiziksel boyutları (Gazebo collision mesh'ten ölçülmüş, doğrulanmış;
#    fabrika X-UAV Mini Talon ile uyumlu) ══
GOVDE_BOYU_M = 0.81        # X ekseni
KANAT_ACIKLIGI_M = 1.28    # Y ekseni
GOVDE_KANAT_ORANI = 0.633  # 0.81 / 1.28


def _env_f(name, default):
    return float(os.environ.get(name, default))


class Cfg:
    """IBVS lead pursuit ayarları. Kritikler AVCI_IBVS_* env ile override edilir."""
    # ── çekirdek ──
    K_LEAD = _env_f("AVCI_IBVS_K_LEAD", 0.5)      # ≈ hedef_hızı/bizim_hız, tarama 0.0-1.0
    MAX_LEAD_DEG = 35.0
    OLCEK_KAPALI_PX = 6.0    # olcek_px = fx·0.81/R = 134.9/R → 6 px ≈ R 22.5 m (lead yok)
    OLCEK_TAM_PX = 14.0      #                                → 14 px ≈ R 9.6 m (tam lead)
    FILTRE_TAU_S = 0.12      # 30 Hz'te ~3.6 kare; pencere dar, uzun tutma
    MIN_GOVDE_PX = 2.0
    FLIP_DT_TAVAN_S = 0.20   # 30 Hz'te 6 kare
    GECIKME_TAVAN_S = 0.12   # bundan bayat kare atlanır (döngü kullanır)
    YUKSELTI_DUZELT = True   # Adım 3: LOS yükselti düzeltmesi (alttan yaklaşma)
    KAMERA_TILT_DEG = 25.0   # sabit montaj; gimbal gelirse dinamik okunacak
    UNDISTORT_AKTIF = False  # simde distorsiyon yok; gerçek donanımda True + katsayılar
    DIST_KATSAYILARI = None  # cv2.undistortPoints için (k1,k2,p1,p2,k3), simde None
    KPT_CONF_MIN = _env_f("AVCI_POSE_KPT_CONF", 0.5)
    # ── copter adaptörü ──
    # Kapanma hızı hedefin (~15 m/s) ÇOK üstünde olmalı (kullanıcı kararı):
    # görsel faz vurucu fazdır, hedefle hız eşitlemek GPS fazının işidir.
    # Gerçek tavanı ArduCopter param/gövde limiti belirler — canlıda CSV'deki
    # kapanma_hizi_ms ile doğrula. K_LEAD taramasında SABİT tut.
    V_KAPANMA = _env_f("AVCI_IBVS_V_KAPANMA", 25.0)   # m/s
    KP_YAW = 1.2
    # ── TAVANLAR GERİ ALINDI (2026-07-31) ──
    # 2026-07-25'te "limitsiz test" için YAW_HIZ_MAX=1080 / IVME_TAVAN=1000
    # yapılmıştı (pratikte slew ve rampa KAPALI). 141017 uçuşunda bu iki tavanın
    # yokluğu tespit gürültüsünü doğrudan gövdeye geçirdi:
    #   - tek karede 136° yaw komut sıçraması, 4.1 s'te 637° dönüş
    #   - vz_cmd kare-arası 8.2 m/s sıçrama = 247 m/s² talep
    # Tavanlar ayarlı değerlerine döndürüldü; tarama için env ile override edilir.
    # IVME_TAVAN=4 fiziksel bir sınırdır, keyfi değil: quad ileri ivmelenmek için
    # burnunu aşağı eğer, kamera gövdeye +25° bağlı → ~5 m/s² üstünde kamera
    # dünyada aşağı bakar, gökyüzü arka planı kaybolur (bkz. adapter_copter).
    YAW_HIZ_MAX = _env_f("AVCI_IBVS_YAW_HIZ_MAX", 90.0)   # deg/s
    # Ardışık kaç kare AYNI YÖNDE yaw doygunluğuna izin verilir. Aşılırsa yaw
    # susturulur: adım sürekli tavandaysa hata kapanmıyor demektir (algı bayat/
    # hatalı), dönmeye devam etmek yalnız aracı çevirir. 15 kare @30 Hz = 0.5 s
    # → kaçak en fazla ~45° ile sınırlanır (ölçülen: 33.6 TUR).
    YAW_DOYGUN_N = int(_env_f("AVCI_IBVS_YAW_DOYGUN_N", 15))
    # İVME TAVANI YATAY/DİKEY AYRILDI (2026-07-31 — dikey ıska düzeltmesi).
    # Tek 3B tavan, kameranın YATAY kısıtını dikeye de dayatıyordu:
    #   YATAY ivme burun eğimi gerektirir → kamera (+25° sabit) aşağı bakar.
    #     4 m/s² → eğim 22° → kamera +2.8° (gökyüzü zar zor duruyor)
    #     8 m/s² → eğim 39° → kamera −14°  (gökyüzü kayıp, tespit bozulur)
    #   DİKEY ivme burun eğimi GEREKTİRMEZ — yalnız itki artar, kamera sabit.
    #     22° eğimde 10 m/s² → 2.18x hover itkisi → gaz %85 (yaw için pay kalır)
    #     12 m/s² → %94, 15 m/s² → %107 SATÜRE (yaw yetkisi gider)
    # Ölçüm: 4 m/s² tek tavanla dikey PN %27-78 oranında 15° tavanında "yukarı"
    # derken v_doygun %93-99 idi — komut rampada yok oluyor, drone hedefin ~2 m
    # altından geçiyordu. 25 m/s'te hız vektörünün dönme hızı = ivme/hız:
    # 4 m/s² → 9.2°/s (2 sn'de 18°), 10 m/s² → 23°/s (2 sn'de 46°).
    IVME_TAVAN = _env_f("AVCI_IBVS_IVME_TAVAN", 4.0)             # m/s² — YATAY
    IVME_TAVAN_DIKEY = _env_f("AVCI_IBVS_IVME_TAVAN_DIKEY", 10.0)  # m/s² — DİKEY

    # ── LİMİT dt TAVANI (2026-07-31) ──
    # YAW_HIZ_MAX ve IVME_TAVAN birer HIZ limitidir; adaptör bunları kare
    # başına paya çevirirken dt ile çarpıyor. dt ardışık kare zaman damgası
    # farkıdır ve kamera hedefi kaybedince ŞİŞİYOR — tespit dönene kadar
    # process() çağrılmadığı için boşluğun tamamı tek dt'ye biniyor.
    # 160249 uçuşu satır 60: 15 kare kör dalıştan sonra dt=0.825 s → yaw tavanı
    # 90*0.825 = 74.2° oldu ve 74°'lik adımın TAMAMI tek MAVLink mesajında
    # gitti. Araç bunu rampa değil BASAMAK olarak gördü, savruldu.
    # Limit hesabında kullanılan dt bu tavanla kırpılır; boşluktan sonra tek
    # hamle yerine birkaç karede toparlanır. Ham dt filtre/PN'de aynen kalır
    # (oradaki kullanım türev/zaman sabiti, "biriken hak" değil).
    DT_TAVAN_S = _env_f("AVCI_IBVS_DT_TAVAN", 0.1)   # s (~3 kare @30 Hz)

    # ── AZİMUT TEKİLLİĞİ KAPISI (2026-07-31) ──
    # yaw_hata = atan2(u_govde[1], u_govde[0]); hedef kadraj tepesinden çıkarken
    # nişan vektörü dikeye yaklaşır, yatay bileşen (=cos(yükseliş)) sıfıra iner ve
    # atan2 TANIMSIZLAŞIR. 141017 uçuşunda ardışık üç kare: yatay 0.446 → 0.027 →
    # 0.156, yaw_hata +64.7° → −154.2° → +130.8°. KP_YAW bunu doğrudan komuta
    # çevirince drone kendi etrafında döndü.
    # Çözüm: yatay bileşene göre azimut GÜVENİ (0..1) üret, adaptör yaw adımını
    # bununla söndürsün. Hedef tam tepedeyken yaw zaten işe yaramaz — o geometride
    # düzeltmeyi dikey kanal (PN + co-altitude) yapar.
    AZIMUT_TAM_YUKSELIS_DEG   = 60.0   # altında azimut tam güvenilir (güven=1)
    AZIMUT_TEKIL_YUKSELIS_DEG = 75.0   # üstünde tekil sayılır (güven=0, yaw kapalı)
    # ── terminal (kör dalış + vuruş) ──
    # Son ~6 m'de hedef kadraj tepesinden çıkıp tespit kopuyor; GPS'e dönmek
    # yerine son nişan komutunu kısa süre SÜRDÜR (çarpışmayı tamamla).
    TERMINAL_MENZIL = _env_f("AVCI_IBVS_TERMINAL_MENZIL", 8.0)   # m; altında temas
                                                                # koparsa kör dalış
    # Kör dalış KİLİTLİDİR: bir kez girince süresi dolana/vuruşa dek sürer
    # (gürültülü menzil kapanma bayrağını titretiyordu). 0.6 s @ 25 m/s ≈ 15 m
    # — 6 m'lik kör bölgeyi kapatır, ıskada uzağa uçmaz.
    TERMINAL_SURE   = _env_f("AVCI_IBVS_TERMINAL_SURE", 0.6)     # s; kör dalış süresi
    # Hedef telemetrisi ~4-5 Hz, drane 25 m/s → menzil örnekleri ~5 m aralıklı;
    # +araç açıklıkları ~1.3 m. 3 m merkez-merkez ≈ fiziksel temas.
    # 3.0 → 1.5 (2026-08-01): 3 m FİZİKSEL TEMAS DEĞİL, yakın geçiştir. Talon'un
    # kanat açıklığı 1.3 m; drone hedefin 2.9 m altından geçerken "VURULDU"
    # raporlanıyordu. 1.5 m gerçek çarpışmaya çok daha yakın bir ölçüt.
    VURUS_MENZIL    = _env_f("AVCI_IBVS_VURUS_MENZIL", 1.5)      # m; altı = VURULDU

    # ── MENZİL MAKULLÜK KAPISI (2026-07-31 sahte VURULDU düzeltmesi) ──
    # Ground-truth menzil zıplıyor: 193559 uçuşunda 33 ms'de 22.4→6.6 m (479 m/s)
    # örneği geldi ve VURULDU sayıldı; drone aslında altından geçip uzaklaşıyordu.
    # Tavan, kafa-kafaya en kötü kapanmanın üstünde tutulur: bizim 25 m/s +
    # hedef ~15 m/s = 40 m/s; 50 gerçek kapanmayı asla kırpmaz ama 479'u eler.
    MENZIL_HIZ_TAVAN = _env_f("AVCI_IBVS_MENZIL_HIZ_TAVAN", 50.0)  # m/s
    MENZIL_RESENK_N  = 8    # bu kadar ARDIŞIK red sonrası yeni seviyeye senkronize
                            # ol (kalıcı kayma olmuşsa bayat değere kilitlenme)

    # ── DİKEY PN + TERMİNAL CO-ALTITUDE (2026-07-25 alttan-geçiş düzeltmesi) ──
    # Son uçuşta SAF TAKİP, tırmanan hedefin ALTINDAN geçti: menzil kapandıkça
    # hedefin yükseliş açısı 51°→75° (kadraj tavanına) fırladı, drone ~2.4 m
    # altından ıskaladı. İki terim eklendi (adapter_copter uygular):
    #  (a) DİKEY AIM YUMUŞATMA: pose yükseliş açısı ok↔kpt_dusuk kareleri arasında
    #      BİMODAL zıplıyor (~10-18°); ham türev alınırsa PN ±tavana çakılıp vz'yi
    #      chatter'a sokuyor (162804 uçuşu: %24 doygun, 17 vz işaret değişimi, 5.1m
    #      ıska). Önce dikey aim'i tek-kare slew kırpma + EMA ile yumuşat.
    #  (b) DİKEY PN: YUMUŞATILMIŞ yükseliş oranıyla orantılı lead = PN_LEAD_SURE·oran
    #      (anticipasyon; tırmanan hedefe çarpışma rotası, alttan geçişi keser).
    #  (c) TERMİNAL CO-ALTITUDE: menzil eşik altında (KİLİTLİ) sabit yukarı nişan
    #      yanlılığı → drone hedef seviyesine çıkıp kafa-kafaya vursun.
    ELEV_EMA          = 0.4     # dikey aim EMA (bimodal kpt gürültüsü → vz chatter'ı kes)
    ELEV_STEP_MAX_DEG = 10.0    # tek-karede dikey aim slew kırpması (gürültü sıçraması sınırı)
    # ── DİKEY PN GÜÇLENDİRİLDİ (2026-07-31 dikey ıska) ──
    # Terminalde düzeltme FİZİKSEL OLARAK imkânsız: hız vektörünü eğmek yay
    # çizer, R = v²/a. 25 m/s + 10 m/s² → 62 m yarıçap; çarpışma 3-20 m'de
    # oluyor. 62 m'lik yayla 3 m'lik düzeltme yapılamaz. Tek çıkış: kesme
    # rotasına UZAKTAN girip terminalde dönüşe hiç ihtiyaç bırakmamak — bunu
    # yapan şey dikey PN'dir, o yüzden erken ve güçlü davranmalı.
    #   PN_LEAD_SURE 0.4→0.6  : anticipasyon süresi (kesme rotası daha erken kurulur)
    #   PN_DIKEY_MAX_DEG 15→30: gerçek uçuşlarda PN karelerin %27-78'inde 15°
    #                           tavanına ÇAKILIYORDU — kapı daralttığı için
    #                           komut kırpılıyordu
    #   PN_RATE_EMA 0.3→0.45  : oran filtresinin gecikmesi azaltıldı
    # ELEV_EMA/ELEV_STEP_MAX_DEG BİLEREK DEĞİŞMEDİ: onlar bimodal kpt gürültüsüne
    # karşı chatter kalkanı. Lead'in sert aç/kapa titremesi (ayrı görev) durduğu
    # sürece gevşetilmemeli.
    # Dikey lead: yumuşatılmış LOS yükseliş oranıyla orantılı anticipasyon.
    # NOT: klasik PN (γ += N·Δλ) 2026-07-31'de denendi ve GERİ ALINDI —
    # simülasyonda ıska 0.66 → 1.5-2.1 m'ye çıktı, gerekçe adapter_copter
    # `_dikey_pn` docstring'inde. Dikey ıskanın kökü bu yasa değil, GPS
    # fazının istasyon geometrisi (hedefin 4.65 m altında park etmesi).
    PN_LEAD_SURE      = _env_f("AVCI_IBVS_PN_SURE", 0.6)   # s
    PN_DIKEY_MAX_DEG  = _env_f("AVCI_IBVS_PN_MAX_DEG", 30.0)
    PN_RATE_EMA       = 0.45
    # ── KADRAJ TUTMA: "metre altta kal" DEĞİL, "AÇI altta kal" (2026-07-31) ──
    # Dikey ıskanın kökü: GPS fazı hedefin SABİT 4.65 m altında istasyon tutuyor
    # (RANGE_SET·sin(25°)). Sabit METRE, kapanan menzilde sabit AÇI değildir —
    # 11 m'de 25° (kadraj merkezi), 6 m'de 51°, 4 m'de kadrajın DIŞI (+80.2°
    # üst sınır). Hedef kadrajın tepesinden çıkıyor, tespit kopuyor, drone
    # altından geçiyor. GPS fazına dokunmadan görsel faz bunu düzeltebilir:
    # hedef kadraj merkezinden YUKARI kaçtıkça nişanı orantılı olarak yukarı it,
    # yani açıyı KAMERA_TILT_DEG'de tutmaya çalış. Ofset böylece menzille
    # orantılı küçülür ve sabit görüş açısı = çarpışma rotası olur.
    #
    # NOT — bu PN DEĞİLDİR ve PN'in başarısız olduğu yerde çalışır: PN açının
    # DEĞİŞİMİNİ sıfırlar, yani hangi açıdaysa onu KİLİTLER (2026-07-31'de
    # denendi, ıskayı 0.66→2.1 m'ye çıkardı). Buradaki terim açının KENDİSİNİ
    # kadraj merkezine geri çeker.
    # Kazanç kapalı çevrim simülasyonda tarandı: 0.5 en iyi (ıska 0.59→0.55 m,
    # en kötü 1.39→1.15 m). 1.0 ve üstü ZARARLI (1.5'te 0/24 vuruş) — yüksek
    # kazanç yakınsamayla savaşıp salınım üretiyor.
    KP_KADRAJ      = _env_f("AVCI_IBVS_KP_KADRAJ", 0.5)   # oransal kazanç
    KADRAJ_MAX_DEG = 30.0    # düzeltmenin açı tavanı (aşırı komut sınırı)

    # ── GÖRSEL FAZI ERKEN BIRAKMA (2026-07-31) ──
    # Eskiden KAYIP_M ARDIŞIK pose'suz kare görsel fazı bitiriyordu. Tespit
    # kümelenmiş koptuğu için (karelerin %28'i tespit_yok) bu şart sık sağlanıyor:
    # 19:35 uçuşunda görsel faz 4 KEZ başlayıp koptu, her biri 1-1.9 s. Her
    # kopuşta GPS fazı drone'u istasyona (hedefin 4.65 m altına) GERİ ÇEKİYOR —
    # görsel fazın kazandığı irtifa geri alınıyor. Dikey ıskanın asıl dinamiği bu.
    # Yeni ölçüt: son KAYIP_PENCERE karede en az KAYIP_MIN_ISABET tespit varsa
    # temas SÜRÜYOR sayılır. Uzun kopukluk kümelerini tolere eder, gerçekten
    # kaybedince yine bırakır.
    KAYIP_PENCERE    = 40   # kare (~1.3 s @30 Hz)
    KAYIP_MIN_ISABET = 4    # bu pencerede bu kadar tespit varsa temas sürüyor

    TERMINAL_COALT_DEG    = 10.0   # terminalde sabit yukarı nişan yanlılığı
    TERMINAL_COALT_MENZIL = _env_f("AVCI_IBVS_COALT_MENZIL", 12.0)  # m; altında co-altitude (kilitli)


def cfg_copy():
    """Cfg'nin bağımsız bir kopyası (test/tarama için)."""
    import types
    return types.SimpleNamespace(
        **{k: v for k, v in vars(Cfg).items() if not k.startswith("_")})


# ══ Çerçeve dönüşümleri ══

def kamera_to_govde(u_kamera, tilt_rad):
    """OpenCV kamera çerçevesi (X sağ, Y aşağı, Z ileri) → gövde FRD
    (X ileri, Y sağ, Z aşağı). Montaj tilt'i (yukarı pozitif) Ry ile uygulanır.
    TEK dönüşüm noktası — gimbal gelirse yalnız burası dinamikleşir.
    Doğrulama: merkez [0,0,1], tilt 25° → [0.906, 0, -0.423] (+25° pitch hatası)."""
    ham = np.array([u_kamera[2], u_kamera[0], u_kamera[1]], dtype=float)
    c, s = math.cos(tilt_rad), math.sin(tilt_rad)
    Ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return Ry @ ham


def _dcm_govde_dunya(roll, pitch, yaw):
    """Gövde FRD → dünya NED yön kosinüs matrisi (DCM = Rz(ψ)·Ry(θ)·Rx(φ))."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def govde_to_dunya(u_govde, roll, pitch, yaw):
    """Gövde FRD → dünya NED (ArduPilot attitude, radyan)."""
    return _dcm_govde_dunya(roll, pitch, yaw) @ np.asarray(u_govde, dtype=float)


def hedef_kadraj_hatasi(hedef_ned, drone_ned, roll, pitch, yaw):
    """Hedefin drone KAMERASINDAKİ kadraj konumu — SAF geometri (GPS + attitude).

    guidance_core'un ters izdüşümü: dünya-NED hedef/drone pozu + drone attitude'undan
    hedefe bakışın (LOS) gövde-çerçevesi açılarını ve piksel izdüşümünü verir. IBVS
    pose zincirinin AYNI konvansiyonu (NED/FRD, 25° tilt) — hiç ENU/Gazebo karışmaz.

    Kadraj MERKEZİ ⇔ yaw_hata=0 ve elev=+25° (kamera tilt'i) ⇔ (u,v)=(CX,CY).
    Dönüş dict:
      menzil      : slant menzil (m)
      yaw_hata    : gövde azimut hatası (rad, sağ+; 0 = burun hedefte)
      elev        : gövde LOS yükselişi (rad, yukarı+)
      pitch_hata  : elev − tilt (rad; 0 = dikey merkez)
      u, v        : piksel izdüşümü (hedef önde değilse None)
      onde        : hedef kameranın önünde mi (bool)
    """
    los = np.asarray(hedef_ned, dtype=float) - np.asarray(drone_ned, dtype=float)
    menzil = float(np.linalg.norm(los))
    if menzil < 1e-6:
        return {"menzil": 0.0, "yaw_hata": 0.0, "elev": 0.0,
                "pitch_hata": 0.0, "u": None, "v": None, "onde": False}
    u_dunya = los / menzil
    R = _dcm_govde_dunya(roll, pitch, yaw)
    u_govde = R.T @ u_dunya                                  # dünya → gövde (Rᵀ)
    yaw_hata = math.atan2(u_govde[1], u_govde[0])            # sağ +
    elev = math.atan2(-u_govde[2], math.hypot(u_govde[0], u_govde[1]))  # yukarı +
    tilt = math.radians(Cfg.KAMERA_TILT_DEG)
    # gövde → kamera (kamera_to_govde tersi) → piksel (yalnız log/UI)
    c, s = math.cos(tilt), math.sin(tilt)
    ham = np.array([c * u_govde[0] - s * u_govde[2],         # Ry(-tilt) @ u_govde
                    u_govde[1],
                    s * u_govde[0] + c * u_govde[2]])
    u_kam = np.array([ham[1], ham[2], ham[0]])               # [sağ, aşağı, ileri]
    onde = u_kam[2] > 1e-6
    u_px = (geo.CX + geo.FX * u_kam[0] / u_kam[2]) if onde else None
    v_px = (geo.CY + geo.FY * u_kam[1] / u_kam[2]) if onde else None
    return {"menzil": menzil, "yaw_hata": yaw_hata, "elev": elev,
            "pitch_hata": elev - tilt, "u": u_px, "v": v_px, "onde": onde}


def yukselti_duzeltme(eps_rad):
    """Adım 3 düzeltme katsayısı: olcek = olcek_ham / sqrt(1 + sin²(eps)).
    eps = LOS'un yatayla açısı. Hedefin yaklaşık seviyeli uçtuğu varsayılır."""
    s = math.sin(eps_rad)
    return math.sqrt(1.0 + s * s)


# ══ GERÇEK-ROTASYON MODU (AVCI_POSE_KAYNAK=gercek) yardımcıları ══

def _quat_dcm(q):
    """Birim quaternion (w,x,y,z) → gövde→dünya DCM (Gazebo pozları için)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def gercek_geometri(pozlar):
    """sim_truth.pozlar() → hedefin GERÇEK burun yönü (iris gövde FRD'de) + menzil.

    Her şey Gazebo'nun TEK pose mesajından ve GÖRELİ hesaplanır — NED/ENU
    orijin hizalama derdi yok: iki quat da aynı dünya çerçevesinde, vektör
    iris'in kendi gövdesine indirgeniyor. Gazebo gövde çerçevesi FLU
    (x ileri, y sol, z yukarı) → FRD dönüşümü (x, −y, −z).
    Dönüş: {"eksen_frd": np.array birim, "menzil": float m}."""
    ip, pp = pozlar["iris"], pozlar["plane"]
    eksen_dunya = _quat_dcm(pp["quat"]) @ np.array([1.0, 0.0, 0.0])  # hedef burnu
    v = _quat_dcm(ip["quat"]).T @ eksen_dunya                        # iris FLU
    eksen_frd = np.array([v[0], -v[1], -v[2]])
    n = np.linalg.norm(eksen_frd)
    if n > 1e-9:
        eksen_frd = eksen_frd / n
    fark = np.subtract(pp["pos"], ip["pos"])
    return {"eksen_frd": eksen_frd, "menzil": float(np.linalg.norm(fark))}


class LeadPursuitCore:
    """Adım 1-8 + sağlamlık. Saf hesap — IO/MAVLink yok, birim test edilir.

    process() her KABUL EDİLEN karede çağrılır; bayat kare/mod kapıları
    döngünün (visual_lead) işidir."""

    def __init__(self, cfg=Cfg):
        self.cfg = cfg
        self.yandanlik_f = None       # EMA durumu
        self.d_birim_onceki = None    # flip koruması
        self.t_onceki = None          # header.stamp (s)
        self.flip_sayaci = 0

    # keypoint indeksleri (vision/geometry.KEYPOINT_NAMES sırası)
    _I_BURUN, _I_KUYRUK, _I_SOLK, _I_SAGK = 0, 1, 2, 3

    def _undistort(self, pts):
        """UNDISTORT kancası: gerçek donanımda cv2.undistortPoints; simde işlemsiz."""
        if not self.cfg.UNDISTORT_AKTIF:
            return pts
        import cv2
        K = np.array([[geo.FX, 0, geo.CX], [0, geo.FY, geo.CY], [0, 0, 1]])
        dist = np.asarray(self.cfg.DIST_KATSAYILARI or [0, 0, 0, 0, 0], float)
        p = np.asarray(pts, np.float64).reshape(-1, 1, 2)
        und = cv2.undistortPoints(p, K, dist, P=K)
        return und.reshape(-1, 2)

    def process(self, pose, stamp, attitude, gercek=None):
        """
        pose     : pose_detector çıktısı {cx, cy, conf, kpts: 6×(u,v,conf)} (tam kare px)
        stamp    : karenin header.stamp'i (s) — dt BUNDAN hesaplanır
        attitude : (roll, pitch, yaw) radyan (kendi aracımız) veya None
        gercek   : verilirse (AVCI_POSE_KAYNAK=gercek modu) keypoint zinciri
                   ATLANIR; lead geometrisi gercek["eksen_frd"]/["menzil"]'den
                   hesaplanır (bkz. _process_gercek). None = görsel pose yolu.
        Dönüş: tüm ara değerler + u_govde / yaw_hata / pitch_hata + durum + warn listesi.
        """
        if gercek is not None:
            return self._process_gercek(pose, stamp, gercek)
        cfg = self.cfg
        warn = []
        durum = "ok"

        # dt: ardışık kare header.stamp farkı (ASLA duvar saati)
        dt = (stamp - self.t_onceki) if self.t_onceki is not None else None
        self.t_onceki = stamp

        kpts = pose["kpts"]
        pts = self._undistort([(k[0], k[1]) for k in kpts] + [(pose["cx"], pose["cy"])])
        burun, kuyruk = np.array(pts[self._I_BURUN]), np.array(pts[self._I_KUYRUK])
        solk, sagk = np.array(pts[self._I_SOLK]), np.array(pts[self._I_SAGK])
        bbox_cx, bbox_cy = pts[-1]
        guven = float(pose["conf"])

        # ── Adım 1: ham ölçümler ──
        d = burun - kuyruk                    # 2D gövde ekseni vektörü (px)
        a = float(np.hypot(d[0], d[1]))       # gövde projeksiyonu
        b = float(np.hypot(*(solk - sagk)))   # kanat projeksiyonu — SADECE uzunluk

        # ── Adım 2: ham ölçek ──
        olcek_ham = math.sqrt(a * a + (GOVDE_KANAT_ORANI * b) ** 2)

        # d_birim + flip koruması (burun/kuyruk takası lead'i TERS çevirir)
        if a > 1e-9:
            d_birim = d / a
        else:
            d_birim = self.d_birim_onceki if self.d_birim_onceki is not None \
                else np.array([1.0, 0.0])
        if (self.d_birim_onceki is not None and dt is not None
                and dt < cfg.FLIP_DT_TAVAN_S
                and float(np.dot(d_birim, self.d_birim_onceki)) < -0.5):
            d_birim = self.d_birim_onceki     # yön korunur
            self.flip_sayaci += 1
            warn.append("flip")               # sessizce düzeltme YOK — her seferinde logla
        # dt büyükse kontrol ATLANIR: düşük kare hızında gerçek aspect değişimi
        # tek karede olabilir, yanlış alarm üretir.
        self.d_birim_onceki = d_birim.copy()

        # ── Adım 6 ön hazırlık: saf takip yönü u (kamera çerçevesi) ──
        u = np.array([(bbox_cx - geo.CX) / geo.FX,
                      (bbox_cy - geo.CY) / geo.FY, 1.0])
        u = u / np.linalg.norm(u)

        # ── Adım 3: yükselti düzeltmesi (alttan yaklaşma; hedef ~seviyeli varsayımı) ──
        tilt = math.radians(cfg.KAMERA_TILT_DEG)
        u_govde_hedef = kamera_to_govde(u, tilt)     # lead'siz yön, gövde FRD
        eps_deg, duzeltme = 0.0, 1.0
        if cfg.YUKSELTI_DUZELT:
            if attitude is not None:
                u_dunya_hedef = govde_to_dunya(u_govde_hedef, *attitude)
                eps = math.asin(max(-1.0, min(1.0, -float(u_dunya_hedef[2]))))  # NED: -Z yukarı
                eps_deg = math.degrees(eps)
                duzeltme = yukselti_duzeltme(eps)
            else:
                warn.append("attitude_yok")          # sağlamlık #6: düzeltmesiz devam
        olcek = olcek_ham / duzeltme

        # ── Adım 4: yandanlık, kalite, filtre ──
        yandanlik = (a / olcek) if olcek > 1e-9 else 0.0
        kalite = max(0.0, min(1.0, (olcek - cfg.OLCEK_KAPALI_PX)
                              / (cfg.OLCEK_TAM_PX - cfg.OLCEK_KAPALI_PX)))
        # kanat ucu güveni düşükse b'yi bbox'tan UYDURMA — kaliteyi söndür
        if (kpts[self._I_SOLK][2] < cfg.KPT_CONF_MIN
                or kpts[self._I_SAGK][2] < cfg.KPT_CONF_MIN):
            kalite = 0.0
            durum = "kanat_dusuk"
        # burun/kuyruk güveni düşükse d yönü anlamsız — lead'i söndür
        kpt_govde_ok = (kpts[self._I_BURUN][2] >= cfg.KPT_CONF_MIN
                        and kpts[self._I_KUYRUK][2] >= cfg.KPT_CONF_MIN)

        if self.yandanlik_f is None or dt is None:
            self.yandanlik_f = yandanlik
        else:
            alpha = 1.0 - math.exp(-dt / cfg.FILTRE_TAU_S)   # dt'den türetilir, sabit DEĞİL
            self.yandanlik_f = alpha * yandanlik + (1.0 - alpha) * self.yandanlik_f

        # ── Adım 5: lead açısı ──
        carpim = cfg.K_LEAD * guven * kalite * self.yandanlik_f
        if carpim > 0.95:
            durum = "cozumsuz"       # kesme çözümü olmayabilir (hedef bizden hızlı,
            warn.append("cozumsuz")  # tam yandan) — güdüm DURMAZ, sadece işaretlenir
        lead = math.atan(carpim)
        lead = min(lead, math.radians(cfg.MAX_LEAD_DEG))
        if a < cfg.MIN_GOVDE_PX or not kpt_govde_ok:
            lead = 0.0               # deadband: saf takibe düş
            if not kpt_govde_ok:
                durum = "kpt_dusuk"

        # ── Adım 6: yön vektörü uzayında kaydırma (PİKSEL UZAYINDA DEĞİL) ──
        e = np.array([d_birim[0] / geo.FX, d_birim[1] / geo.FY, 0.0])
        e = e - float(np.dot(e, u)) * u
        e_n = np.linalg.norm(e)
        if e_n > 1e-12:
            e = e / e_n
            u_nisan = math.cos(lead) * u + math.sin(lead) * e
            u_nisan = u_nisan / np.linalg.norm(u_nisan)
        else:
            u_nisan = u.copy()       # gövde ekseni bakış yönüyle çakışık — kaydırma tanımsız

        # ── Adım 7: kamera → gövde (aynı fonksiyon, ikinci çağrı) ──
        u_govde = kamera_to_govde(u_nisan, tilt)

        # ── Adım 8: hata açıları (gövde FRD) ──
        # u_govde birim → yatay = cos(yükseliş). Nişan dikeye yaklaştıkça yatay→0
        # ve azimut tanımsızlaşır; azimut_kalite bunu 1→0 arası sürekli ölçer.
        # yaw_hata HAM bırakılır (log dürüst kalsın), sönümlemeyi adaptör yapar.
        yatay = math.hypot(u_govde[0], u_govde[1])
        yaw_hata = math.atan2(u_govde[1], u_govde[0])                          # sağ +
        pitch_hata = math.atan2(-u_govde[2], yatay)                            # yukarı +
        _y_tam = math.cos(math.radians(cfg.AZIMUT_TAM_YUKSELIS_DEG))
        _y_tekil = math.cos(math.radians(cfg.AZIMUT_TEKIL_YUKSELIS_DEG))
        azimut_kalite = max(0.0, min(1.0, (yatay - _y_tekil) / (_y_tam - _y_tekil))) \
            if _y_tam > _y_tekil else 1.0
        if azimut_kalite <= 0.0:
            warn.append("azimut_tekil")   # durum'a dokunma: bu poz kalitesi değil,
                                          # geometri — görünürlük azimut_kalite'de

        return {
            "kaynak": "pose",
            "durum": durum, "warn": warn, "dt": dt,
            "a": a, "b": b, "olcek_ham": olcek_ham,
            "eps_deg": eps_deg, "duzeltme": duzeltme, "olcek": olcek,
            "yandanlik_ham": yandanlik, "yandanlik_f": self.yandanlik_f,
            "kalite": kalite, "guven": guven, "lead_deg": math.degrees(lead),
            "d_birim": d_birim, "u": u, "u_nisan": u_nisan, "u_govde": u_govde,
            "u_govde_hedef": u_govde_hedef,
            "yaw_hata": yaw_hata, "pitch_hata": pitch_hata,
            "yatay_bilesen": yatay, "azimut_kalite": azimut_kalite,
            "yaw_hata_deg": math.degrees(yaw_hata),
            "pitch_hata_deg": math.degrees(pitch_hata),
            "eksen_disi_deg": math.degrees(
                math.acos(max(-1.0, min(1.0, float(u_nisan[2]))))),
            # SADECE LOG — güdüm hesabında KULLANILMAZ:
            "menzil_kestirim_m": (geo.FX * GOVDE_BOYU_M / olcek) if olcek > 1e-9 else 0.0,
            "flip_sayaci": self.flip_sayaci,
        }

    def _process_gercek(self, pose, stamp, gercek):
        """GERÇEK-ROTASYON modu (AVCI_POSE_KAYNAK=gercek): pose modeli DEVRE DIŞI.

        Nişan yönü u yine TESPİT kutusundan (tracker kilidi) gelir — görsel hat
        ölmez. Lead'in YÖNÜ ve MİKTARI ise keypoint yerine Gazebo'dan gelen
        gerçek gövde ekseni + gerçek menzille hesaplanır:
          yandanlık = |eksenin bakış hattına dik bileşeni| = sin(eksen↔LOS)
          (görsel yoldaki a/olcek tam olarak bu niceliğin piksel kestirimidir)
          kalite    = AYNI 6→14 px rampası, gerçek menzilin piksel eşdeğeriyle
          guven     = tespit conf'u (pose conf artık yok)
        Bu bir SİM koltuk değneğidir (pose modeli eğitilene dek); gerçek uçuşta
        yoktur. CSV'de kaynak=gercek damgasıyla ayrılır; truth bayatsa lead
        kapanır, saf takip sürer (durum=gercek_yok)."""
        cfg = self.cfg
        warn = []
        durum = "ok"

        dt = (stamp - self.t_onceki) if self.t_onceki is not None else None
        self.t_onceki = stamp

        guven = float(pose["conf"])
        u = np.array([(pose["cx"] - geo.CX) / geo.FX,
                      (pose["cy"] - geo.CY) / geo.FY, 1.0])
        u = u / np.linalg.norm(u)
        tilt = math.radians(cfg.KAMERA_TILT_DEG)
        u_govde_hedef = kamera_to_govde(u, tilt)

        eksen = gercek.get("eksen_frd")
        menzil = gercek.get("menzil")

        # kalite: pose ölçeği yerine GERÇEK menzil, aynı rampa (px eşdeğeri)
        kalite = 0.0
        if menzil is not None and menzil > 1e-6:
            olcek_esdeger = geo.FX * GOVDE_BOYU_M / menzil
            kalite = max(0.0, min(1.0, (olcek_esdeger - cfg.OLCEK_KAPALI_PX)
                                  / (cfg.OLCEK_TAM_PX - cfg.OLCEK_KAPALI_PX)))

        yandanlik = 0.0
        e = None
        if eksen is None:
            durum = "gercek_yok"      # truth bayat/yok → lead kapalı, saf takip
            warn.append("gercek_yok")
        else:
            # FRD → kamera: kamera_to_govde'nin tersi (hedef_kadraj_hatasi ile
            # aynı yol) — Ry(−tilt) sonra [sağ, aşağı, ileri] sıralaması.
            c, s = math.cos(tilt), math.sin(tilt)
            ham = np.array([c * eksen[0] - s * eksen[2],
                            eksen[1],
                            s * eksen[0] + c * eksen[2]])
            eksen_kam = np.array([ham[1], ham[2], ham[0]])
            e3 = eksen_kam - float(np.dot(eksen_kam, u)) * u
            yandanlik = float(np.linalg.norm(e3))
            if yandanlik > 1e-9:
                e = e3 / yandanlik

        if self.yandanlik_f is None or dt is None:
            self.yandanlik_f = yandanlik
        else:
            alpha = 1.0 - math.exp(-dt / cfg.FILTRE_TAU_S)
            self.yandanlik_f = alpha * yandanlik + (1.0 - alpha) * self.yandanlik_f

        carpim = cfg.K_LEAD * guven * kalite * self.yandanlik_f
        if carpim > 0.95:
            durum = "cozumsuz"
            warn.append("cozumsuz")
        lead = math.atan(carpim)
        lead = min(lead, math.radians(cfg.MAX_LEAD_DEG))
        if e is None:
            lead = 0.0                # yön yok (truth bayat / tam kuyruktan)

        if e is not None and lead > 0.0:
            u_nisan = math.cos(lead) * u + math.sin(lead) * e
            u_nisan = u_nisan / np.linalg.norm(u_nisan)
        else:
            u_nisan = u.copy()

        u_govde = kamera_to_govde(u_nisan, tilt)
        yatay = math.hypot(u_govde[0], u_govde[1])
        yaw_hata = math.atan2(u_govde[1], u_govde[0])
        pitch_hata = math.atan2(-u_govde[2], yatay)
        _y_tam = math.cos(math.radians(cfg.AZIMUT_TAM_YUKSELIS_DEG))
        _y_tekil = math.cos(math.radians(cfg.AZIMUT_TEKIL_YUKSELIS_DEG))
        azimut_kalite = max(0.0, min(1.0, (yatay - _y_tekil) / (_y_tam - _y_tekil))) \
            if _y_tam > _y_tekil else 1.0
        if azimut_kalite <= 0.0:
            warn.append("azimut_tekil")

        return {
            "kaynak": "gercek",
            "durum": durum, "warn": warn, "dt": dt,
            "a": 0.0, "b": 0.0, "olcek_ham": 0.0,
            "eps_deg": 0.0, "duzeltme": 1.0, "olcek": 0.0,
            "yandanlik_ham": yandanlik, "yandanlik_f": self.yandanlik_f,
            "kalite": kalite, "guven": guven, "lead_deg": math.degrees(lead),
            "d_birim": np.array([0.0, 0.0]), "u": u, "u_nisan": u_nisan,
            "u_govde": u_govde, "u_govde_hedef": u_govde_hedef,
            "yaw_hata": yaw_hata, "pitch_hata": pitch_hata,
            "yatay_bilesen": yatay, "azimut_kalite": azimut_kalite,
            "yaw_hata_deg": math.degrees(yaw_hata),
            "pitch_hata_deg": math.degrees(pitch_hata),
            "eksen_disi_deg": math.degrees(
                math.acos(max(-1.0, min(1.0, float(u_nisan[2]))))),
            # gerçek modda kestirim = gerçek menzil (kaynak sütunuyla ayrılır)
            "menzil_kestirim_m": (menzil if menzil is not None else 0.0),
            "flip_sayaci": self.flip_sayaci,
        }
