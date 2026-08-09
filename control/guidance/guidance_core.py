"""
guidance_core.py — BBOX IBVS çekirdeği (PLATFORMDAN BAĞIMSIZ).

Girdi YALNIZCA detection kutusudur: {cx, cy, w, h, conf, bbox}. Kutunun
merkezinden nişan YÖNÜ, kutunun genişliğinden ÖLÇEK (→ kalite + menzil
kestirimi) çıkar. Çıktı bir YÖNDÜR: u_govde (FRD birim vektör) ve ondan türeyen
yaw_hata / pitch_hata. Bu geometri platformdan bağımsızdır — platforma bağlı
komut üretimi adaptördedir (adapter_copter).

── 2026-08-06: POSE MODELİ KALDIRILDI ──
Eskiden YOLO-pose'un 6 keypoint'inden (burun, kuyruk, kanat/vtail uçları)
hedefin görünür yönelimi (yandanlık + gövde ekseni yönü) çıkarılıp saf takip
yönünün üstüne bir ŞEKİL-LEAD'i bindiriliyordu (tek ayar K_LEAD). Bu daldan
vazgeçildi; sökülen kodun tamamı ve geri dönüş yolu:
    POSEA_GERI_DONMEK_ISTERSENIZ/README.md

Lead kayboldu mu? HAYIR, KAYNAĞI DEĞİŞTİ. Şekil-lead'inin yerini adaptördeki
AZİMUT-ORANI LEAD'İ aldı (adapter_copter._yatay_pn) — dikey kanalda zaten
çalışan PN'in yatay eşi. Ölçüldü (08-06, 10 183 `ok` karesi, aynı yumuşatma
sabitleriyle): şekil-lead'i medyan 2.39° / ort 6.18° / p90 18.64°;
azimut-oranı lead'i medyan 1.09° / ort 6.08° / p90 20.0°. Aynı büyüklük bandı.

Kritik tasarım kuralları (master spec):
  - Kaydırma PİKSEL uzayında DEĞİL, yön vektörü uzayında yapılır (FOV 125°
    geniş: kenarda 1 piksel, merkezdekinin ~çeyreği kadar açı eder).
  - Kamera açıları doğrudan komut olmaz; önce gövde çerçevesine çevrilir
    (25° yukarı montaj tilt'i — atlanırsa sürekli 25° sabit hata).
  - dt daima kare header.stamp farkından gelir, duvar saatinden DEĞİL.
  - Menzil kestirimi güdüme bağlanmaz; menzil_kestirim_m SADECE log.

GT MODU (AVCI_GT_ROT=on): process() bir `gt` sözlüğü alırsa ölçümler kameradan
değil Gazebo'nun gerçek pozundan türetilir (bkz. Cfg.GT_ROT). Geri kalan
geometri değişmez — böylece iki mod aynı yasayı, farklı algı ile çalıştırır ve
CSV sütunları birebir kıyaslanabilir. Bu mod SİMÜLASYONA ÖZGÜDÜR.
"""

import math
import os

import numpy as np

from vision import geometry as geo

# ══ Talon fiziksel boyutları (Gazebo collision mesh'ten ölçülmüş, doğrulanmış;
#    fabrika X-UAV Mini Talon ile uyumlu) ══
# NOT: güdüm bu sayıları ARTIK KULLANMIYOR — bbox ölçeği Cfg.BBOX_L_ETKIN_M
# kalibrasyon sabitiyle çalışır (YOLO kutusu gövdeye tam oturmadığı için gerçek
# uzunluklar doğrudan kullanılamaz). Burada referans/araçlar için duruyorlar.
GOVDE_BOYU_M = 0.81        # X ekseni
KANAT_ACIKLIGI_M = 1.28    # Y ekseni
GOVDE_KANAT_ORANI = 0.633  # 0.81 / 1.28


def _env_f(name, default):
    return float(os.environ.get(name, default))


def _env_b(name, default=False):
    v = os.environ.get(name)
    return default if v is None else v.lower() in ("on", "1", "true", "yes")


class Cfg:
    """BBOX IBVS ayarları. Kritikler AVCI_IBVS_* env ile override edilir."""
    # ── ÖLÇEK: bbox GENİŞLİĞİ ──
    # Kutunun görünen genişliği w ≈ fx · L_ETKIN / R. L_ETKIN gerçek bir uzunluk
    # değil, YOLO kutusunun kalibrasyon sabitidir (kutu gövdeye tam oturmaz).
    # 2026-08-06'da ölçüldü: bugünkü tüm `ok` karelerinde (n=1917) w·R/fx
    # medyanı 1.687 m, p10-p90 yayılımı ±%45. Kıyas — sökülen pose ölçeğinin
    # aynı verideki yayılımı ±%55'ti, yani BBOX DAHA KARARLI bir menzil vekili.
    # Denenen alternatifler: h (±%83), sqrt(w·h) (±%58), hypot(w,h) (±%52).
    BBOX_L_ETKIN_M = _env_f("AVCI_IBVS_BBOX_L", 1.687)
    # Kalite rampası bbox pikseline göre yeniden ölçeklendi. Eski eşikler pose
    # ölçeğine (fx·0.81/R) göreydi: 6 px ≈ 22.5 m, 14 px ≈ 9.6 m. Aynı MENZİL
    # bandını bbox genişliğinde tutmak için: fx·1.687/22.5 = 12.5 px ve
    # fx·1.687/9.6 = 29.3 px. Böylece kalite kapısı menzil olarak AYNI yerde.
    OLCEK_KAPALI_PX = _env_f("AVCI_IBVS_OLCEK_KAPALI", 12.5)   # ≈ R 22.5 m (kalite 0)
    OLCEK_TAM_PX = _env_f("AVCI_IBVS_OLCEK_TAM", 29.3)         # ≈ R  9.6 m (kalite 1)
    MIN_BBOX_PX = 2.0        # bundan küçük kutu ölçek olarak kullanılmaz
    GECIKME_TAVAN_S = 0.12   # bundan bayat kare atlanır (döngü kullanır)
    YUKSELTI_DUZELT = True   # LOS yükselti düzeltmesi (alttan yaklaşma)
    KAMERA_TILT_DEG = 25.0   # sabit montaj; gimbal gelirse dinamik okunacak
    UNDISTORT_AKTIF = False  # simde distorsiyon yok; gerçek donanımda True + katsayılar
    DIST_KATSAYILARI = None  # cv2.undistortPoints için (k1,k2,p1,p2,k3), simde None
    # ── GT MODU (AVCI_GT_ROT=on, varsayılan KAPALI) ──
    # Açıkken güdümün TÜM algı girdileri (nişan noktası, kutu ölçeği) Gazebo'nun
    # gerçek pozundan gelir; YOLO komut yoluna hiç girmez (ekran/log için
    # çalışmaya devam eder).
    # ⚠ SİMÜLASYONA ÖZGÜ: gerçek donanımda hedefin pozu bilinmez, bu mod
    # uçurulamaz. Amacı güdüm YASASINI algı hatasından yalıtarak test etmek:
    # GT modunda ıska varsa suç yasada, bbox modunda varsa suç algıdadır.
    # (Env adı AVCI_GT_ROT tarihsel — pose döneminde "GT rotasyon modu"ydu.)
    GT_ROT = _env_b("AVCI_GT_ROT", False)
    # ── copter adaptörü ──
    # Kapanma hızı hedefin (~15 m/s) ÇOK üstünde olmalı (kullanıcı kararı):
    # görsel faz vurucu fazdır, hedefle hız eşitlemek GPS fazının işidir.
    # Gerçek tavanı ArduCopter param/gövde limiti belirler — canlıda CSV'deki
    # kapanma_hizi_ms ile doğrula. K_LEAD taramasında SABİT tut.
    V_KAPANMA = _env_f("AVCI_IBVS_V_KAPANMA", 25.0)   # m/s

    # ── YAKLAŞMA ALT-FAZI (2026-08-05) — bkz. adapter_copter.compute ──
    # Görsel faz devir anından itibaren tek davranıştı: sabit V_KAPANMA ile
    # dalış. Ölçüldü: kapanma hızı menzilden BAĞIMSIZ, 0-2 m'de bile 24.4 m/s;
    # 30 Hz'de kare başına 0.81 m yol = ıska mesafesi medyanının tamamı.
    # Artık devirden sonra önce YAKLAŞMA yapılıyor: yavaşla + hedefle irtifayı
    # eşitle, sonra TERMINAL_MENZIL altında tam hız hücuma geç.
    # V_YAKLASMA=0 → alt-faz kapalı, eski davranış birebir geri gelir.
    # ⚠ V_YAKLASMA HEDEFİN HIZINDAN BELİRGİN YÜKSEK OLMALI — mutlak hız tavanı,
    # göreli değil. İlk denemede 12.0 seçildi ve GT modunu tamamen bozdu:
    # hedef 14.2 m/s (ölçüldü, medyan) uçarken net kapanma −2.2 m/s oldu, yani
    # drone UZAKLAŞTI. 222803 uçuşu: 1070 karenin hepsi yaklaşma alt-fazında,
    # menzil 24.2 → 82.8 m. Pose modu etkilenmedi çünkü orada devir zaten
    # ~6 m'de (TERMINAL_MENZIL altında) oluyor ve alt-faz hiç çalışmıyor.
    # 20.0 → net kapanma +5.8 m/s: yavaşlama faydası korunuyor (25 yerine 20)
    # ama yetişme garanti. Hedef daha hızlı bir uçağa çevrilirse BU DEĞER DE
    # yükseltilmeli; ölçüt: V_YAKLASMA − hedef_hızı ≥ ~4 m/s.
    V_YAKLASMA   = _env_f("AVCI_IBVS_V_YAKLASMA", 20.0)   # m/s; yaklaşma yatay hızı
    VZ_YAKLASMA  = _env_f("AVCI_IBVS_VZ_YAKLASMA", 4.0)   # m/s; yaklaşmada dikey tavan
    # Yaklaşma alt-fazının ÜST sınırı: bundan uzakta yavaşlamanın anlamı yok,
    # tam hızla kapanmak gerekir. GT modunda devir 20+ m'de olabiliyor; üst
    # sınır olmadan araç oradan itibaren yavaş moda giriyordu.
    # 0 → üst sınır yok (her menzilde yaklaşma).
    YAKLASMA_MAX_MENZIL = _env_f("AVCI_IBVS_YAKLASMA_MAX", 18.0)  # m
    KP_IRTIFA    = _env_f("AVCI_IBVS_KP_IRTIFA", 0.6)     # 1/s; irtifa farkı → dikey hız
    # Terminalde dikey tavan: nişan dikeye yaklaşınca V_KAPANMA'nın TAMAMI
    # dikeye geçebiliyordu (25 m/s tırmanma). Bu, ıska sonrası "yukarı fırlama"
    # ve istasyon aşımının doğrudan sebebiydi. 0 → tavan yok (eski davranış).
    VZ_TERMINAL_MAX = _env_f("AVCI_IBVS_VZ_TERMINAL", 12.0)  # m/s

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
    # Susturma KALICI DEĞİL SÜRELİ (2026-08-06 kilitlenme düzeltmesi). Kapı
    # adımı 0 yapınca araç dönmez, dönmeyen araçta hata kapanmaz, kapı hiç
    # açılmaz — ölü kilit. GPS fazında ölçüldü: karelerin %7.8'inde burun >20°
    # sapmışken adım tam 0; en uzun kesintisiz susma 1867 kare = 93 saniye.
    # YAW_SUS_N kare sonra yetki geri verilir; sorun gerçekse kapı yeniden
    # tetiklenir ve dönme oranı 15/(15+60) ≈ %20'ye kısılır (kaçak yine sınırlı).
    YAW_SUS_N = int(_env_f("AVCI_IBVS_YAW_SUS_N", 60))    # 30 Hz → 2.0 s
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
    # ── ⚠ "KAMERA YERE BAKAR" GEREKÇESİ ÖLÇÜMLE ZAYIFLADI (2026-08-08) ──
    # Yukarıdaki 4 m/s² tavanının dayanağı bir TAHMİNDİ: "8 m/s² → eğim 39° →
    # kamera −14° → gökyüzü kayıp, tespit bozulur". Mevcut uçuş loglarıyla
    # sınandı (7830 'ok' karesi, kamera_dunya_pitch_deg ↔ guven), menzil
    # karıştırıcısı da çıkarıldı:
    #     menzil 0-8 m  : kamera <0° → güven 0.89 (n=172) | >10° → 0.86 (n=793)
    #     menzil 8-15 m : kamera <0° → güven 0.88 (n=53)  | >10° → 0.78 (n=1823)
    # Kamera dünyada −25°'ye kadar aşağı bakmış, tespit BOZULMAMIŞ (o bantta
    # güven en yüksek). 8 m/s²'nin gerektirdiği −14° bölgesinde 211 kare var,
    # güven medyanı 0.86.
    # ⚠ Çekince: aşağı bakan kare azınlıkta (259/7830) ve >15 m'de yok — iddia
    # görsel fazın çalıştığı 8-15 m bandında çürüdü, uzakta denenmedi.
    #
    # NEDEN ÖNEMLİ: 08-08 teşhisi görsel fazın dönüşte çökmesini bu tavana
    # bağladı. Nişanın dönme hızı 8-20 m'de 19.2 °/s; hız vektörü ivme/hız
    # kadar dönebiliyor: 4 m/s²'de 10.4 °/s (YETMİYOR), 8 m/s²'de 20.8 °/s
    # (YETER). ArduPilot tarafı engel değil: ATC_ANGLE_MAX 45° → 9.8 m/s².
    # GPS fazı zaten MAX_ACCEL=12 ile uçuyor (o dosya Kayra'nın alanı).
    # Uçuşta sınanacak: AVCI_IBVS_IVME_TAVAN=8
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

    # ── İLK KARE İVME LİMİTİ (2026-08-09) ──
    # Görsel fazın ilk karesinde dt None'dır (kıyaslanacak önceki kare yok) ve
    # eskiden limitleme tümüyle atlanıyordu. Bu her devirde bir kez yaşanıyor:
    # 08-09 uçuşunda altı fazın ilk karesi 12-25 m/s basamak komutuyla başladı.
    # Açıkken nominal dt (DT_TAVAN_S) kullanılır ve adaptörün hız referansı
    # aracın gerçek hızıyla tohumlanır (CopterAdapter.hiz_tohumla).
    # Kill-switch: AVCI_IBVS_ILK_KARE_LIMIT=off → eski (limitsiz) davranış.
    ILK_KARE_LIMIT = _env_b("AVCI_IBVS_ILK_KARE_LIMIT", True)

    # ── FAZ SONU TAM DURUŞ (2026-08-09) ──
    # _bitir() faz biterken 3 kez send_velocity(0,0,0,yaw) gönderiyordu.
    # Ölçülmüş gerekçesi bir YAW kaçağıydı (log 00000108: 14.57 tur) ama komut
    # ötelenmeyi de sıfırlıyor. 08-09 uçuşunda sonucu:
    #     GPS fazı başlangıç hızı medyan 0.16 m/s  (araç durmuş)
    #     15 m/s'e HİÇ ulaşamayan GPS fazı 9/23
    #     rampada geçen süre 47 s / 342 s = %14
    # Kullanıcının "gps kopup duruyo" dediği şey bu. Görev gerçekten bittiğinde
    # (vuruldu/durduruldu) tam duruş DOĞRU; 'kayip' yolunda ise GPS fazı hemen
    # devralıyor, aracı önce durdurmanın bir faydası yok.
    # Kill-switch: AVCI_IBVS_BITIR_TAM_DUR=on → her yolda tam duruş (eski).
    BITIR_TAM_DUR = _env_b("AVCI_IBVS_BITIR_TAM_DUR", False)

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

    # ── YATAY (AZİMUT-ORANI) LEAD — pose şekil-lead'inin YERİNE (2026-08-06) ──
    # Pose kaldırılınca yandanlıktan türeyen lead de gitti. Yerine dikey kanalda
    # zaten çalışan PN'in yatay eşi kondu: LOS azimutunun DEĞİŞİM ORANIYLA
    # orantılı öne nişan. Hedef yanlamasına geçiyorsa azimut döner → nişan öne
    # kayar; hedef tam önümüzde/arkamızdaysa oran ~0 → saf takip. Bu, şekil-lead'i
    # ile aynı FİZİĞİ farklı bir sinyalden okur ve keypoint gerektirmez.
    #
    # SABİTLER DİKEY KANALDAN ALINDI (aynı yumuşatma zinciri: slew-kırpma → EMA →
    # oran EMA). Ölçüm (08-06, 10 183 `ok` karesi, log üstünde yeniden oynatıldı):
    #     şekil-lead'i (pose)   : medyan 2.39°  ort 6.18°  p90 18.64°
    #     azimut-oranı (bu yasa): medyan 1.09°  ort 6.08°  p90 20.00°
    # yani lead bütçesi korunuyor. ⚠ Ölçüm cevap anahtarı azimutuyla yapıldı;
    # canlıda sinyal bbox merkezinden gelir ve daha gürültülüdür — yumuşatma
    # zinciri bu yüzden dikeyle BİREBİR aynı tutuldu.
    #
    # PN_YATAY_SURE=0 → yatay lead kapanır, davranış SAF TAKİP olur (geri dönüş
    # yolu tek env değişkeni).
    PN_YATAY_SURE     = _env_f("AVCI_IBVS_PN_YATAY_SURE", 0.6)   # s
    PN_YATAY_MAX_DEG  = _env_f("AVCI_IBVS_PN_YATAY_MAX", 20.0)   # ° (eski MAX_LEAD_DEG 35'ti)
    # Yatay lead KAPISI — "azimut" (varsayılan) | "olcek" (2026-08-08 öncesi).
    # "olcek" lead'i bbox GENİŞLİĞİNDEN türeyen `kalite` ile söndürür; 13 m
    # üstünde 0'a iner, yani hedef DÖNERKEN lead tamamen kapanır (ölçüm:
    # dönüşte 0/14 görsel faz kapanabildi, uygulanan lead 0.0°/gereken 20°).
    # "azimut" kapıyı azimut_kalite'ye bağlar — sinyal bbox MERKEZİ olduğu için
    # doğru kapı budur. Gerekçenin tamamı: adapter_copter._yatay_pn.
    PN_YATAY_KAPI     = os.environ.get("AVCI_IBVS_PN_YATAY_KAPI", "azimut").lower()
    AZ_STEP_MAX_DEG   = 10.0   # tek-karede azimut slew kırpması (ELEV_STEP_MAX_DEG eşi)
    AZ_EMA            = 0.4    # azimut yumuşatma (ELEV_EMA eşi)
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

    # ── B5: FLY-PAST (hedefi geçme) TESPİTİ — 2026-08-06 ──
    # Drone hedefi geçince "hedefe uç" komutu YUKARI-GERİYİ gösterir: araç
    # tırmanır, hedef kadrajdan çıkar, tespit kopar. Ölçüldü (08-06): ıska
    # sonrası hız vektörü ile burun arasındaki açı medyan 54.4° / p90 138.9°
    # (ıska ÖNCESİ 25.0° / 84.1°) — yani araç yan yan uçuyor ve kamera boşa
    # bakıyor. Toparlaması 10-20 s, o sürede hedef kaçıyor.
    #
    # Ölçüt İŞARET DEĞİL BİRİKEN MESAFE: "menzil şu an büyüyor mu" gürültüde
    # her kare titrer. Bunun yerine bu görsel fazın EN YAKIN noktası tutulur ve
    # ondan FLYPAST_BUYUME_M kadar uzaklaşınca geçmiş sayılırız.
    FLYPAST_MENZIL   = _env_f("AVCI_IBVS_FLYPAST_MENZIL", 8.0)   # m; bu bandın
                                # altına inmeden fly-past aranmaz (uzakta menzil
                                # büyümesi normal bir manevra olabilir)
    FLYPAST_BUYUME_M = _env_f("AVCI_IBVS_FLYPAST_BUYUME", 1.5)   # m; en yakın
                                # noktadan bu kadar uzaklaşırsak GEÇTİK
    # İkinci, bağımsız imza: nişan yönünün gövde ileri bileşeni negatif
    # (|yaw_hata| > 90°) → hedef arkamızda. Kutu hâlâ görülüyorken bile
    # geçmiş olabiliriz; menzil imzası gecikirse bu yakalar.
    FLYPAST_ARKA     = _env_b("AVCI_IBVS_FLYPAST_ARKA", True)

    # ── TERMİNAL YUKARI YANLILIĞI — 08-08'de ŞÜPHELİ HALE GELDİ ──
    # Amacı: "alttan sıyırma yerine seviyeye çıkıp kafa kafaya" (yalnız hedef
    # ÜSTTEyken uygulanır, elev > 0).
    #
    # ÖLÇÜM (08-08 akşam, düz uçuşta 12 görsel faz, gövde yükselti açısı):
    #     her faz İYİ başlıyor (giriş +16…+30° = drone hedefin ALTINDA)
    #     ıskalayanlar son karede −25…−29°  → hedefin ~0.5 m ÜSTÜNDEN geçiyor
    #     vuranlar          son karede  +2…+26° → hafifçe ALTINDAN geçiyor
    #     drone ÜSTTE biten 6 fazın 6'sı da ISKA (0/6)
    # 10° yanlılık 3 m menzilde 0.53 m yukarı itme demek — ölçülen ıska
    # mesafesiyle birebir aynı büyüklük.
    #
    # ŞÜPHELİ GERİ BESLEME: yanlılık yalnız hedef üstteyken açık; araç seviyeye
    # gelince KAPANIYOR ama o ana kadar dikey hız kazanmış oluyor ve aşıyor.
    # Aşınca hedef kadrajın ALT kenarından çıkıyor (kamera gövdeye +25° YUKARI
    # bağlı) → kör dalış → araç son komutunda kalıyor. Ölçüldü: 9 ıskanın
    # 6'sında hedef alt kenardan çıkmış; 3 vuruşta hiç çıkmamış (kör kare 0).
    #
    # ⚠ HENÜZ UÇUŞTA SINANMADI. Env ile açıldı ki A/B yapılabilsin:
    #     AVCI_IBVS_COALT_DEG=0   → yanlılık kapalı
    #     AVCI_IBVS_COALT_DEG=10  → bugünkü davranış (varsayılan, değişmedi)
    TERMINAL_COALT_DEG    = _env_f("AVCI_IBVS_COALT_DEG", 10.0)   # ° yukarı yanlılık
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


class LeadPursuitCore:
    """bbox → nişan yönü. Saf hesap — IO/MAVLink yok, birim test edilir.

    process() her KABUL EDİLEN karede çağrılır; bayat kare/mod kapıları
    döngünün (visual_lead) işidir.

    Sınıf adı tarihsel: lead artık BURADA üretilmiyor, adaptörün azimut-oranı
    kanalında üretiliyor (adapter_copter._yatay_pn). Bu çekirdek SAF TAKİP
    yönünü ve ölçek/kalite sinyallerini verir."""

    def __init__(self, cfg=Cfg):
        self.cfg = cfg
        self.t_onceki = None          # header.stamp (s)

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

    def process(self, det, stamp, attitude, gt=None):
        """
        det      : detection çıktısı {cx, cy, w, h, conf} (tam kare px).
                   GT modunda (gt verilirse) KULLANILMAZ, None olabilir.
        stamp    : karenin header.stamp'i (s) — dt BUNDAN hesaplanır
        attitude : (roll, pitch, yaw) radyan (kendi aracımız) veya None
        gt       : GT MODU girdisi — geometry.bbox_gt_goruntu() çıktısı
                   {uv, w, h, menzil} veya None (normal bbox modu). Verilirse
                   ölçümler kameradan DEĞİL Gazebo gerçek pozundan gelir;
                   geri kalan geometri ikisinde de aynıdır.
        Dönüş: tüm ara değerler + u_govde / yaw_hata / pitch_hata + durum + warn listesi.
        """
        cfg = self.cfg
        warn = []
        durum = "ok"

        # dt: ardışık kare header.stamp farkı (ASLA duvar saati)
        dt = (stamp - self.t_onceki) if self.t_onceki is not None else None
        self.t_onceki = stamp

        if gt is not None:
            # ══ GT MODU — ölçüm yerine GERÇEK pozdan türetme ══
            # Kamera hiç okunmaz: nişan noktası hedefin izdüşümü, kutu ölçeği
            # gerçek menzilden gelir.
            bbox_cx, bbox_cy = gt["uv"]
            guven = 1.0                       # ölçüm değil gerçek → tam güven
            menzil_gt = max(float(gt["menzil"]), 1e-6)
            # Ölçek ÖLÇÜLMEZ, gerçek menzilden türetilir: olcek = fx·L/R. Bbox
            # modundaki ölçekle AYNI birimde çıkar (px), böylece kalite rampası
            # (OLCEK_KAPALI_PX / OLCEK_TAM_PX) iki modda da aynı eşiklerle
            # çalışır — modlar kıyaslanabilir kalır.
            olcek_ham = geo.FX * cfg.BBOX_L_ETKIN_M / menzil_gt
            bbox_w, bbox_h = gt.get("w"), gt.get("h")
        else:
            pts = self._undistort([(det["cx"], det["cy"])])
            bbox_cx, bbox_cy = pts[0]
            guven = float(det.get("conf", 1.0))
            bbox_w = float(det["w"])
            bbox_h = float(det["h"])
            # ── ÖLÇEK: kutunun GENİŞLİĞİ (bkz. Cfg.BBOX_L_ETKIN_M) ──
            olcek_ham = bbox_w
            if olcek_ham < cfg.MIN_BBOX_PX:
                durum = "kutu_kucuk"          # ölçek güvenilmez → kalite sönecek
                warn.append("kutu_kucuk")

        # ── Saf takip yönü u (kamera çerçevesi) ──
        u = np.array([(bbox_cx - geo.CX) / geo.FX,
                      (bbox_cy - geo.CY) / geo.FY, 1.0])
        u = u / np.linalg.norm(u)

        # ── Yükselti düzeltmesi (alttan yaklaşma; hedef ~seviyeli varsayımı) ──
        tilt = math.radians(cfg.KAMERA_TILT_DEG)
        u_govde_hedef = kamera_to_govde(u, tilt)     # saf takip yönü, gövde FRD
        eps_deg, duzeltme = 0.0, 1.0
        if cfg.YUKSELTI_DUZELT:
            if attitude is not None:
                u_dunya_hedef = govde_to_dunya(u_govde_hedef, *attitude)
                eps = math.asin(max(-1.0, min(1.0, -float(u_dunya_hedef[2]))))  # NED: -Z yukarı
                eps_deg = math.degrees(eps)
                # GT modunda ölçek zaten GERÇEK menzilden geliyor; bu düzeltme
                # ÖLÇÜLEN ölçeğin perspektif kısalmasını telafi etmek içindir,
                # gerçeğe uygulanırsa hata EKLER. eps_deg yalnız log'da kalır.
                if gt is None:
                    duzeltme = yukselti_duzeltme(eps)
            else:
                warn.append("attitude_yok")          # sağlamlık #6: düzeltmesiz devam
        olcek = olcek_ham / duzeltme

        # ── Kalite: ölçek rampası (yalnız dikey/yatay PN kazancını söndürür) ──
        kalite = max(0.0, min(1.0, (olcek - cfg.OLCEK_KAPALI_PX)
                              / (cfg.OLCEK_TAM_PX - cfg.OLCEK_KAPALI_PX)))
        if durum == "kutu_kucuk":
            kalite = 0.0

        # SAF TAKİP: nişan = bakış yönü. Öne kaydırmayı adaptörün azimut-oranı
        # kanalı yapar (u_nisan burada bilerek u'ya EŞİT bırakılır ki çekirdek
        # "hedef nerede" sorusundan başka bir şey cevaplamasın).
        u_nisan = u.copy()

        # ── Kamera → gövde (aynı fonksiyon, ikinci çağrı) ──
        u_govde = kamera_to_govde(u_nisan, tilt)

        # ── Hata açıları (gövde FRD) ──
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
            warn.append("azimut_tekil")   # durum'a dokunma: bu kutu kalitesi değil,
                                          # geometri — görünürlük azimut_kalite'de

        return {
            "durum": durum, "warn": warn, "dt": dt,
            "bbox_w": bbox_w, "bbox_h": bbox_h, "olcek_ham": olcek_ham,
            "eps_deg": eps_deg, "duzeltme": duzeltme, "olcek": olcek,
            "kalite": kalite, "guven": guven,
            "u": u, "u_nisan": u_nisan, "u_govde": u_govde,
            "u_govde_hedef": u_govde_hedef,
            "yaw_hata": yaw_hata, "pitch_hata": pitch_hata,
            "yatay_bilesen": yatay, "azimut_kalite": azimut_kalite,
            "yaw_hata_deg": math.degrees(yaw_hata),
            "pitch_hata_deg": math.degrees(pitch_hata),
            "eksen_disi_deg": math.degrees(
                math.acos(max(-1.0, min(1.0, float(u_nisan[2]))))),
            # SADECE LOG — güdüm hesabında KULLANILMAZ:
            # (GT modunda olcek gerçek menzilden türetildiği için bu sütun
            #  menzil_gercek_m'i AYNEN yansıtır; algı sapması ölçülemez.)
            "menzil_kestirim_m": ((geo.FX * cfg.BBOX_L_ETKIN_M / olcek)
                                  if olcek > 1e-9 else 0.0),
            "gt_modu": gt is not None,
        }
