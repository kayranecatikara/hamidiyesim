"""
bbox_ibvs.py — SAF görüntü tabanlı görsel güdüm (IBVS), yalnız bbox.

YARIŞMA KURALI (üstün kısıt, bkz. UYGULANACAK.md D0): görsel temas varken
hedefin GPS'i güdümde KULLANILAMAZ — canlı GPS akışı yasak.

Bu modül iki girdiyle çalışır:
  1. Tespit kutusu (cx, cy, w, h, conf) — her kare, tek canlı kaynak.
  2. Drone'un KENDİ durumu (yaw, kendi hızı) — kendi sensörü, kural serbest.
  3. DONDURULMUŞ TAŞIYICI (ff_hiz): devir ANINDA, yani görsel temas kurulmadan
     ÖNCEKİ son GPS kestiriminden alınan hedef hız vektörü. Görsel faz boyunca
     BİR DAHA OKUNMAZ.
     ⚠ YAPISAL GARANTİ: bu bir SAYI ÜÇLÜSÜ olarak geçilir, callback değil —
     döngünün canlı GPS'e erişimi FİZİKSEL OLARAK YOKTUR. Kural ihlali
     "yapmamayı seçmek"le değil, yapamamakla güvence altında.

Kontrol yasası — SAF TAKİP (pure pursuit) + PI HIZ:
  YAW     : yatay piksel hatası (cx − CX) → burun hedefe döner.
  DİKEY   : hedefin SEVİYE çerçevesindeki yükselişi → hız vektörünün
            YÖNÜ döner (yatayla aynı matematik), büyüklüğü korunur.
  YATAY   : hız DAİMA LOS (burun) YÖNÜNDE; büyüklüğü kutu boyutu hatasına
            PI kontrol:  v = I + K_P·(REF − boyut),  İ += K_I·(REF − boyut)·dt
            İntegral, hedefin hızını GÖRÜNTÜDEN öğrenir — GPS gerekmez.
            ff_hiz yalnız İNTEGRALİN BAŞLANGIÇ DEĞERİ (sıcak start).

⚠ 2026-08-08 İKİ UÇUŞ DERSİ (ikisi de bu tasarımı zorunlu kıldı):

1) Saf kutu-boyutu (P-only, taşıyıcısız) 12 m'de 8 m/s üretiyordu; hedef
   15 m/s → drone geride kaldı, faz 3.5 s'de koptu. P-only'nin kalıcı hata
   sorunu: denge için hız hedefin hızına EŞİT olmalı ama P-only bunu ancak
   sıfır olmayan hatayla üretir.

2) DONDURULMUŞ NED TAŞIYICI + LOS kapanması: faz 160 s sürdü, mesafe medyanı
   7.2 m çıktı — ama GEOMETRİ BOZULDU. Ölçüldü (log 184748, aspect açısı):
       devir anı  7° (tam kuyrukta) → 72 s: 55° → 180 s: 70° → 216 s: kayıp
       mesafe 8.7 → 5.3 → 12.2 → 66.6 m
   Kök neden: taşıyıcı NED'de SABİT. Hedef 0.16°/s gibi ÇOK yavaş dönse bile
   168 s'de 27° birikiyor; hız uyuşmazlığı 2·V·sin(Δ/2) ≈ 6 m/s yana kayma
   üretiyor. LOS'taki 3.8 m/s kapanma bunu yenemiyor → drone yana savruluyor,
   hedefi yandan görüyor, sonra kaybediyor.
   DERS: taşıyıcı NED'de dondurulamaz. Yön DAİMA LOS olmalı (saf takip);
   böylece hedef döndükçe hız vektörü kendiliğinden döner ve kuyruk
   geometrisi korunur. Dondurulmuş GPS ancak İNTEGRAL BAŞLANGICI olarak
   kullanılabilir — o da sadece ilk saniyelerin gecikmesini kesmek için.

Arayüz (supervisor.run_hybrid ile uyumlu):
  run_bbox_ibvs(conn, get_iris, wait_pose, stop_event, cfg, kayip_kare_esik,
                ff_hiz=(vx,vy,vz))
    get_iris() -> {..., "yaw": rad, "vx","vy","vz": m/s}  (drone KENDİ durumu)
    wait_pose(son_seq, timeout) -> {"seq","pose",...}  (pose = bbox kaydı | None)
  Dönüş: 'durduruldu' (stop_event) | 'kayip' (kayip_kare_esik ardışık kutusuz).
"""

import csv
import math
import os
import time

from vision import geometry as geo
from control.guidance.guidance_core import Cfg as GeoCfg
from control.guidance.common import (
    clamp, normalize_angle, send_velocity, limit_acceleration,
)
from control.guidance.kurtarma import Kurtarma


def _env_f(name, default):
    return float(os.environ.get(name, default))


class Cfg:
    LOOP_HZ = 20.0

    CX_NISAN = geo.CX                           # px; yatay merkez (320)

    # ── YAW ──
    # eps_yaw = atan((cx − CX)/FX); komut = iris_yaw + K_YAW·eps. K_YAW=1 tam
    # düzeltme (ArduPilot kendi yaw hızıyla slew eder). <1 yumuşatır.
    K_YAW = _env_f("AVCI_IBVS_KYAW", 1.0)
    YAW_ESIK = math.radians(1.0)   # bu açının altında yaw komutu güncellenmez

    # ── YAW HIZ SINIRI (2026-08-09, TAKLANIN KÖK NEDENİ) ──
    # Yaw komutu her kare "iris_yaw + eps_yaw" olarak YENİDEN kuruluyordu;
    # hız sınırı YOKTU. Hedefin yanından geçerken (terminal sonrası fly-past)
    # kutu kadrajı hızla tarıyor ve komut çılgınlaşıyor. ÖLÇÜLDÜ (5 görsel faz):
    #     medyan 12-38 °/s   p95 238-412 °/s   MAX 876 °/s
    # Aracın yapabildiği ~120 °/s. 876 °/s isteyince yaw doyuyor, motorlar
    # yaw torkuna gidiyor, roll/pitch yetkisi kalmıyor → TAKLA.
    # Taklanın maliyeti ölçüldü: takla yaşamayan koşular 2/2 vurdu, takla
    # yaşayanlar 1/3 — her kurtarma 13+ s sürüyor ve hedef kaçıyor.
    #
    # ÇÖZÜM: gps_guidance'ta zaten olan slew sınırı buraya da konur.
    # ⚠ NORMAL TAKİBİ KISITLAMAZ: medyan 12-38 °/s, sınır 120 °/s — yalnız
    # p95 üstü (fly-past) anlarda bağlar.
    # ⚠ HIZ YÖNÜ ETKİLENMEZ: hız hedefin GERÇEK LOS'u boyunca gider; yalnız
    # BURUN yavaş döner. Multirotor yan uçabilir, nişan bozulmaz.
    # ⚠ Ö7 ÖLÇÜMÜ (2026-08-10): bu sınır kaçamak senaryosunda BAĞLIYOR.
    # "normal takip medyanı 12-38 °/s, sınır bağlamaz" gerekçesiyle konmuştu;
    # 8 m'lik yanal kırılmada ölçülen medyan 53-100 °/s ve karelerin
    # %23-47'sinde komut 120 °/s tavanına yapışıyor. Doymuş bir hız
    # sınırlayıcı kontrol döngüsüne FAZ GECİKMESİ katar — salınımın klasik
    # sebebi. Kullanıcı gözlemi: "karşı tarafa daha çok gidiyor ve salınıyor."
    # Panelden canlı denenebilsin diye derece cinsinden tutulur.
    YAW_RATE_MAX_DEG = _env_f("AVCI_IBVS_YAWRATE", 120.0)   # °/s

    # ⭐ 2026-08-17: env'e bağlandı. ZARF BÜYÜTMESİ yatay bütçeyi 3.3 katına
    # çıkardı (WPNAV_ACCEL 8 → 26 m/s²) ama dikey 3 m/s'de kaldı — 10 kat
    # asimetri. ÖLÇÜLDÜ: `square`'de (sürekli manevra) araç hedefin
    # SİSTEMATİK olarak 1-3 m ALTINDA uçuyor (dz −1.05 / −1.75 / −2.84 m);
    # `duz`'da böyle bir sapma YOK (±1.3 m saçılma). Sürekli yatışta araç
    # alçalıyor ve 3 m/s tavanıyla toparlayamıyor.
    # ⭐ 2026-08-17 KULLANICI İSTEĞİ: "aracın gücünü artırdın, bu hareket
    # kabiliyetini DİKEY için kullanalım." İtki/ağırlık 2.56 → 7.08 çıktı ama
    # dikey bütçe 3 m/s'de kalmıştı (yatay 3.3 katına çıkarken) — 10 kat
    # asimetri. Varsayılan 3 → 8 m/s. Araç tarafındaki tavanlar da birlikte
    # açıldı (avci_copter.parm: WPNAV_SPEED_UP/DN, WPNAV_ACCEL_Z).
    VZ_MAX = _env_f("AVCI_IBVS_VZMAX", 8.0)      # m/s; dikey hız tavanı


    # ── HIZ: PI kontrol, kutu boyutu hatası üzerinden (menzil vekili) ──
    # boyut = sqrt(w·h). Büyük kutu = yakın. hata = REF − boyut (pozitif = uzak).
    #     v_los = I + K_FWD·hata      (LOS yönünde uygulanır — saf takip)
    #     I    += K_I·hata·dt         (hedefin hızını GÖRÜNTÜDEN öğrenir)
    # İntegralin işi tam da P-only'nin yapamadığı şey: denge hızını (hedefin
    # LOS üzerindeki hız bileşeni) kalıcı hatasız üretmek. Hedef hızlanır,
    # yavaşlar ya da DÖNERSE integral kendini yeniden ayarlar — GPS'siz.
    #
    # REF ölçümden (2026-08-08): 12 m'de kutu ≈ 12-14 px → boyut ≈ 1/menzil.
    # 6-7 m tutuş için REF ≈ 25 px.
    # ═══════════════════════════════════════════════════════════════════
    # KUTU → MENZİL ÖLÇÜSÜ (2026-08-19, kullanıcı önerisi)
    # ═══════════════════════════════════════════════════════════════════
    # NEDEN: menzil, iğne deliği kamera bağıntısından çıkar —
    #     p / FX = S / R      (benzer üçgenler)
    #   p  : hedefin kadrajdaki boyu, piksel
    #   FX : odak uzaklığı, piksel  (bizde 166.58)
    #   S  : hedefin GERÇEK boyu, metre
    #   R  : menzil, metre
    # Buradan  R = (FX·S)/p = MENZIL_PX_M / p.
    # ⚠ Bu, S'nin SABİT olduğunu varsayar. Ama bir uçağın görünen boyu
    # hangi taraftan bakıldığına ve YATIŞINA göre değişir — varsayım orada
    # bozulur ve menzil yanlış çıkar.
    #
    # KULLANICI ÖNERİSİ: `p` olarak KÖŞEGEN sqrt(w²+h²) alalım. Gerekçesi
    # şu: kutu eksen-hizalı olduğu için, İNCE BİR ÇUBUK kadrajda θ kadar
    # dönerse  w = L·|cosθ|,  h = L·|sinθ|  olur ve
    #     sqrt(w² + h²) = L·sqrt(cos²θ + sin²θ) = L      → YATIŞTAN BAĞIMSIZ
    # Talon arkadan bakınca büyük ölçüde "ince çubuk"tur (kanat 1.280 m,
    # kuyruk yüksekliği 0.286 m).
    #
    # ÖLÇÜLDÜ (8 uçuş, 5812 kare; gerçek menzil telem'den, YALNIZ ANALİZDE):
    #   görüş açısı dağılımı: %91'i 0-15° (tam arkadan), medyan 1°
    #     → kullanıcının "devirde hedefi hep arkadan görüyoruz" tespiti DOĞRU
    #   0-15° bandında bağıl menzil hatası (p50):
    #     sqrt(w·h)  %22        ← bugünkü
    #     KÖŞEGEN    %14        ← %36 daha iyi
    #     w tek başına %12      ← en iyi AMA 30-60°'de %35'e fırlıyor
    #   Teorik yatış duyarlılığı (0-90°, kanat 1.280 / kuyruk 0.286):
    #     KÖŞEGEN %19  ·  sqrt(w·h) %83  ·  w %359
    #   ⇒ köşegen, hâkim rejimde kazancın çoğunu alıyor ve bozulduğunda
    #     zarifçe bozuluyor. `w` daha iyi ama kırılgan.
    #
    # ⚠ MODEL ÖLÇÜLERİ (mini_talon_vtail collision mesh'ten, doğrulandı):
    #   kanat açıklığı 1.280 m · gövde boyu 0.814 m · yükseklik 0.286 m
    # Kalibre sabitleri MODELDEN DEĞİL ÖLÇÜMDEN alındı (kutu, görsel modeli
    # kaplıyor ve YOLO kutusu gevşek çiziliyor; ampirik sabit doğrusu).
    #
    # ⭐ 2026-08-19 KULLANICI KARARI: varsayılan "kosegen".
    # Kampanya OL, 6 uçuş. Eşleşmiş kıyas (AYNI karelerde iki tahmin, koşular
    # arası değişkenlik SIFIR, 3378 kare): ortanca menzil hatası %22 → %14,
    # 6 uçuşun 6'sında da köşegen kazandı. Uçuş sonucu ölçütleri ayrışmadı
    # (isabet 3/3 → 3/3) — çünkü daha iyi menzilin bugün gidecek yeri yok
    # (bkz. aşağıdaki not). Karar SONUÇ için değil ALTYAPI için verildi:
    # yol haritasının her sonraki adımı (yavaşlama profili, kapanma integrali,
    # aykırı değer kapısı, durum kestirimi) R ve ṙ'ye dayanıyor.
    #
    # ⚠ MENZİLİN BUGÜNKÜ ETKİ ALANI (ölçüldü):
    #   lead sönmesi (LEAD_SONUM)   ✅ tek gerçek yol
    #   hız PI hatası               ⚠ hız doygun — 13325 karenin %58'i tam
    #                                  V_HUCUM'da; hatanın değeri etkisiz
    #   YANAL_K / YAW_MENZIL_REF / KACIS_KD hepsi 0 → kapalı
    #
    # SEÇENEK NEDEN DURUYOR: "w" (yalnız genişlik) 0-15° bandında en iyiydi
    # (%12) ama 30-60°'de %35'e fırlıyor. İleride sınanabilir; seçici o yüzden
    # kalıyor, ölü kod değil aday listesi.
    #
    # AVCI_IBVS_OLCU = "carpim" | "kosegen"
    BOYUT_OLCU = os.environ.get("AVCI_IBVS_OLCU", "kosegen").strip().lower()

    # Ölçüye göre kalibrasyon — C = medyan(p · R_gerçek), 0-15° bandında.
    # ⚠ Bugüne kadar 160.0 kullanılıyordu; ölçülen 185.7. Yani menziller
    # %14 EKSİK tahmin ediliyordu (kendimizi olduğumuzdan yakın sanıyorduk).
    MENZIL_PX_M_CARPIM = _env_f("AVCI_IBVS_C_CARPIM", 185.7)   # px·m
    MENZIL_PX_M_KOSEGEN = _env_f("AVCI_IBVS_C_KOSEGEN", 296.8)  # px·m

    # ── EŞİKLER ARTIK METRE ───────────────────────────────────────────
    # Ölçü değişince piksel eşikleri anlamını yitirir (köşegen ~1.6 kat
    # büyük sayı verir). Metre cinsinden tutulunca ölçü değişimi TEK
    # DEĞİŞKEN olarak kalır: yalnız menzil kestirimi değişir, kurulan
    # hedefler aynı fiziksel yerde durur.
    HUCUM_MENZIL_M = _env_f("AVCI_IBVS_HUCUM_M", 1.0)   # m; PI'nın sıfır noktası
    LEAD_MENZIL_M = _env_f("AVCI_IBVS_LEAD_M", 6.4)     # m; lead tamamen sönene menzil

    K_FWD = _env_f("AVCI_IBVS_KFWD", 0.35)      # (m/s)/px; P kazancı
    K_I = _env_f("AVCI_IBVS_KI", 0.04)          # (m/s)/(px·s); İ kazancı
    I_MIN, I_MAX = 0.0, 24.0       # m/s; integral penceresi (windup koruması)

    # V_MIN 0 (2026-08-08, kullanıcı kararı): GERİ ÇEKİLME YOK. Eski −2 m/s
    # "fren"i, kutu REF'i aşınca drone'u geri itiyordu — kullanıcı düz uçuş
    # koşusunda bunu görüp "fren olmasa vururduk" dedi. Görev vuruş; tutuş
    # mesafesinde beklemek değil.
    V_MIN = 0.0                    # m/s; asla geri gitme



    # ⚠ LEAD MENZİLLE SÖNER (2026-08-09, kullanıcı gözlemi: "çarpacakken
    # birden yukarı itki verip kaçırıyoruz").
    # SORUN: lead = LOS_hızı × sabit süre. Ama LOS dönüş hızı menzil→0 iken
    # PATLAR (1 m'de küçük bir göreli hareket bile devasa açısal hız üretir).
    # Sabit süreyle çarpılınca nişan yukarı savruluyor, drone hedefin ÜSTÜNE
    # çıkıyor, sonra dalmak zorunda kalıyor ve ıskalıyor.
    # Ölçüldü (3 uçuşun son terminal kareleri): dikey hata +13° → +45°
    # büyüyor, yatay hız 10 m/s tabanına yapışıyor, hedef 45° ALTTA kalıyor.
    # ÇÖZÜM: lead, kalan süreyle (≈menzille) ölçeklenir — güdüm literatüründe
    # lead daima t_go ile çarpılır. Menzil vekili kutu boyutu olduğu için
    # ölçek = BOYUT_REF/boyut: uzakta 1.0, temas anında ~0.2.
    LEAD_SURE = _env_f("AVCI_IBVS_LEAD", 0.4)    # s; uzaktaki lead süresi
    LEAD_SONUM = _env_f("AVCI_IBVS_LEAD_SON", 1.0) >= 0.5  # 0 = eski (sabit) yol
    LEAD_EMA = 0.25                              # LOS hızı yumuşatması
    LEAD_MAX_DEG = 25.0                          # °; lead açısı tavanı







    KAPANMA_EMA = _env_f("AVCI_IBVS_KAPANMA_EMA", 0.20)  # kare başına yumuşatma
    # ⭐ 2026-08-17 ZARF BÜYÜTMESİ: bu sayı ESKİ aracın 8 m/s²'lik ivme
    # tavanına göre konmuştu ve komut, onu bile karelerin %35'inde aşıyordu.
    # Araç zarfı büyütüldü (ANGLE_MAX 45°→70°, WPNAV_ACCEL 8→26 m/s²,
    # rotor itkisi ×2.5). Bu tavan 12'de kalırsa zarf büyütmesi HİÇBİR İŞE
    # YARAMAZ — güdüm kendi kendini 12'de kırpar. 26 = yeni araç tavanı.
    # ⚠ Bu bir güdüm YASASI değişikliği DEĞİL; eski aracın yeteneğini
    # yansıtan bir kırpıcının yeni araca göre ölçeklenmesidir.
    #
    # ⛔ 26 DENENDİ VE GERİ ALINDI (2026-08-17, 8 uçuş, düz+kaçamak, n=4/kol):
    #   MAX_ACCEL 26 → en yakın menzil medyanı 2.92 m
    #   MAX_ACCEL 12 → 1.79 m   (TAM AYRIŞMA, p=0.057 — dördü de daha yakın)
    # Sebep: 16 m/s'de 26 m/s², hız vektörünün 93 °/s savrulmasına izin verir.
    # 12'lik sınır terminalde fiilen YUMUŞATICI görevi görüyormuş. Araç zarfı
    # büyüdü diye bu kırpıcıyı da büyütmek TERMİNALİ BOZUYOR — takip ile
    # bitiriş ayrı problemler (bu oturumun tekrar eden bulgusu).
    MAX_ACCEL = _env_f("AVCI_IBVS_MAXACC", 12.0)   # m/s²; komut değişim sınırı

    # ══ YATAY AÇI ROLL/PITCH TELAFİSİ — T1a (2026-08-09) ══
    # KULLANICI GÖZLEMİ: "düz uçuşta ıskalamıyor, hedef manevra yapınca görsel
    # güdüm sapıtıyor, yatayda çok salınım oluyor."
    #
    # KÖK NEDEN — bir çerçeve karışıklığı. Yatay hata şöyle okunuyordu:
    #     eps_yaw = atan((cx − CX)/FX)      ← KAMERA çerçevesi azimutu
    #     los_az  = iris_yaw + eps_yaw      ← "bu SEVİYE azimutudur" varsayımı
    # Bu varsayım YALNIZ roll=0'da doğru. Kamera gövdeye 25° YUKARI vidalı;
    # araç yattığında kamera da yatıyor ve hedefin görüntüdeki YATAY konumu
    # kayıyor. Kodda roll telafisi hiç yoktu (roll okunuyordu ama sadece
    # takla bekçisine gidiyordu).
    #
    # ÖLÇÜLDÜ — 5869 kare GERÇEK uçuş verisi (GPS logunda hedefin gerçek
    # konumu + aracın gerçek duruşu + piksel izdüşümü birlikte var; okunan
    # açı ile gerçek seviye azimutu doğrudan kıyaslandı):
    #     yatış  0-9°  (4450 kare) → yatay okuma hatası ort. 0.6°
    #     yatış 10-19° ( 821 kare) →                        2.4°
    #     yatış 20-29° ( 313 kare) →                       11.0°
    #     yatış 30-39° ( 210 kare) →                       13.9°
    #     yatış 40-49° (  75 kare) →                       10.8°
    # Teori (hedef boresight'ın 20° üstünde): 30°→9.9°, 45°→14.2° — UYUŞUYOR.
    # Araç manevrada gerçekten 43-45°'ye (ANGLE_MAX tavanı) dayanıyor.
    #
    # ⚠ HATANIN İŞARETİ DÖNÜŞE KARŞI: sağa dönerken (sağa yatarken) hedef
    # SOLA kaymış görünüyor → güdüm dönüşü kısıyor → geride kalıyor → sonra
    # aşırı düzeltiyor. Yatay salınımın kaynağı bu.
    #
    # ÇÖZÜM: piksel ışını aracın KENDİ duruşuyla SEVİYE çerçevesine döndürülür
    # (bkz. los_seviye). Girdi drone'un kendi IMU'su — canlı hedef GPS'i yok,
    # yarışma kuralı (D0) temiz.
    # ⚠ T1a KAPSAMI: YALNIZ YATAY kanal. Dikey kanal (piksel_elev + pitch ve
    # tutuştaki eps_elev) BİLEREK dokunulmadan bırakıldı — uçuşta doğrulanmış
    # dikey davranış tek değişkenli testin dışında tutuluyor.
    # ⚠ DÜZ UÇUŞU BOZMAZ: roll<10°'de fark 0.6° (ölçüldü), yani kullanıcının
    # "düz uçuşta ıskalamıyor" dediği davranış pratikte aynı kalır.
    # AVCI_IBVS_ROLL=0 → eski (telafisiz) yol aynen geri gelir.
    ROLL_TELAFI = _env_f("AVCI_IBVS_ROLL", 1.0) >= 0.5

    # ══ Ö1 · KAÇIŞ TELAFİSİ — hız yasasına kapanma hızı geri beslemesi ══
    # KULLANICI GÖZLEMİ (2026-08-10, kendi uçuşu + 16 kaçamak testi):
    # "hedef manevra yaptığı sırada mesafe kapatılamıyor, hedef çok uzağa
    # gidiyor; ne zaman düz gitmeye başlarsa o zaman vuruluyor."
    #
    # ÖLÇÜLDÜ — 10 kaçamak koşusunun İSTİSNASIZ HEPSİNDE, kaçamaktan sonraki
    # 15 s içinde:
    #     drone hızı  7.7-13.9 m/s'ye düşüyor   (hedef 15.4-16.3 m/s)
    #     açılan mesafe 48-147 m
    # Hedeften YAVAŞKEN mesafe matematiksel olarak kapanmaz.
    #
    # KÖK NEDEN: hız yasası saf bir MENZİL düzenleyicisi — "hedef şu an benden
    # uzaklaşıyor mu" girdisi YOK.
    #     hata  = BOYUT_REF − boyut ;  hiz_I += K_I·hata·dt ;  v = hiz_I+K_FWD·hata
    # Yakın geçişte kutu 88-102 px olunca hata = −63…−77 → integral saniyede
    # 3.1 m/s DÜŞÜYOR; normal hata ≈ +15'te ise saniyede 0.6 m/s toparlanıyor.
    # 5:1 asimetri = kullanıcının gördüğü uzun toparlanma. (Kullanıcının uçuş
    # logunda birebir: hiz_I 15.1 → 12.0 iki saniyede, geri çıkması ~5 s.)
    #
    # ÇÖZÜM: ṙ (kapanma hızı) zaten hesaplanıyor — dikey kanal için eklenmişti
    # (bkz. Cfg.KAPANMA). Hız yasasına da girsin:
    #     v_los = hiz_I + K_FWD·hata + KACIS_KD·max(0, −ṙ)
    # ⚠ YALNIZ HIZLANDIRMA YÖNÜ. ṙ>0 (yaklaşırken) terim sıfırdır — kullanıcı
    # freni bilerek kaldırttığı için (V_MIN=0, "geri çekilme yok") bu terim
    # asla yavaşlatma yapmaz.
    # ⚠ TARİHSEL NOT: bu ayrım, terminal fazı varken yazılmıştı
    # ve dikey kanal DOKUNULMADI — tek değişken kuralı.
    # AVCI_IBVS_KD=0 → kapalı (varsayılan; açık değeri ölçüm belirleyecek).
    KACIS_KD = _env_f("AVCI_IBVS_KD", 0.0)      # (m/s)/(m/s); 0 = kapalı
    KACIS_MAX = _env_f("AVCI_IBVS_KDMAX", 10.0)  # m/s; terimin tavanı

    # ══ Ö8 · YANAL KOMUT: AÇI DEĞİL, KAÇIRMA MESAFESİ ══
    # KULLANICI GÖZLEMİ (2026-08-10): "araç tam çarpacakken hedef hafif sağa
    # manevra yaptı; bbox ekranın en sağına geldiği için bizim araç sağa öyle
    # bir manevra yapıyor ki sonra salınım oluyor. En azından hedefin sağa
    # gittiği kadar gitsek ve aynı doğrultuda kalsak."
    #
    # ÖLÇÜLDÜ (O7A, kaçamak yatay, tetik 8 m — temas öncesi son 0.4 s):
    #     t       cx    menzil   eps_yaw   yaw komut değişimi
    #   -0.25    336     1.8 m      5.0°        60 °/s
    #   -0.10    410     1.5 m     24.7°       120 °/s  ← DOYDU
    #    0.00    432     1.3 m     35.1°       120 °/s
    #   +0.05    548     1.5 m     58.3°       122 °/s
    #   +0.15    600     2.5 m     58.0°       118 °/s   (kadraj genişliği 640)
    #
    # KÖK NEDEN: eps_yaw = atan((cx−CX)/FX) geometrik olarak DOĞRU — hedef
    # gerçekten 58° yanda. Ama 1.5 m'de 58°, yalnızca 1.5·sin(58°) = 1.3 m
    # yanal kaçırma demek. Güdüm 58°'lik dönüş emri veriyor: 1.3 metre için.
    # Aynı 58° hata 30 m'de 25 metrelik kaçırmadır — güdüm ikisine AYNI
    # komutu veriyor. Yani AÇIYA tepki veriyor, oysa önemli olan MESAFE.
    # 18 m/s'lik vektörü 58° döndürmek 17.5 m/s'lik hız değişimi ister;
    # MAX_ACCEL=12 ile 1.45 s sürer, oysa geometri 0.08 s bırakıyor →
    # komut doyar, araç savrulur, sonra geri savrulur = SALINIM.
    #
    # ⚠ Ö7 (yaw hız tavanı) bu yüzden hiçbir şey yapmadı: yaw sınırı yalnız
    # BURNU yavaşlatıyor, hız vektörü zaten anında savruluyordu. Salınım
    # burun kanalında değil HIZ kanalındaydı.
    #
    # ÇÖZÜM: hız vektörünün yönü, kalan sürede yanal kaçırmayı kapatmak için
    # gereken yanal hızdan türetilir:
    #     y      = R·sin(eps_yaw)            yanal kaçırma (m)
    #     t_go   = R / ṙ                     kalan süre (s)
    #     v_y    = YANAL_K · y / t_go        gereken yanal hız
    #     eps_eff= asin(v_y / v_los)
    # ve YALNIZ KISAR: eps_hiz = min(|eps_yaw|, |eps_eff|).
    # BURUN tam eps_yaw'da kalır → kamera hedefi kaybetmez. Gövde hedefin
    # gittiği kadar yana kayar → savrulmaz. (Kullanıcının tarifi bu.)
    #
    # Yukarıdaki anda (1.5 m, y=1.3 m, ṙ≈3 m/s, K=3): 58° → ~26°,
    # gereken hız değişimi 17.5 → 8.1 m/s.
    # ⚠ RİSK: uzak menzilde de kısar (30 m'de 20° → ~10°). Uzak menzil şu an
    # çalışıyor (25 m tetikte 4/6 isabet) — gerileme olup olmadığı ÖNCE orada
    # ölçülür. AVCI_IBVS_YANAL=0 → tamamen kapalı (varsayılan).
    YANAL_K = _env_f("AVCI_IBVS_YANAL", 0.0)      # 0 = kapalı; açık ~3.0
    YANAL_RDOT_MIN = 1.5   # m/s; t_go patlamasın diye kapanma tabanı
    YANAL_TGO_MIN = 0.20   # s;   t_go tabanı (0'a bölme + aşırı agresiflik)
    # MENZİL KAPISI — birim testi B45 yakaladı: sınır kapısız haliyle 20 m'de
    # komutu %37'ye düşürüyordu. Uzak menzil ŞU AN ÇALIŞIYOR (25 m tetikte
    # 4/6 isabet); orayı bozmamak için sınır yalnız yakında bağlar ve
    # YUMUŞAK geçer (sert kapı kendi başına sıçrama yaratırdı):
    #   R ≥ MENZIL      → hiç kısmaz (eski davranış birebir)
    #   R ≤ MENZIL/2    → tam kısar
    #   arası           → doğrusal harman
    YANAL_MENZIL = _env_f("AVCI_IBVS_YANAL_M", 12.0)   # m

    # ══ Ö9 · YATAY KANALA SÖNÜMLEME (D terimi) ══
    # KULLANICI GÖZLEMİ (2026-08-11, kendi uçuşu ucus_20260811_185753):
    # "hedefin yaptığı ilk manevrada bizim araç o kadar sağa yönelmese, hafif
    # bir sağa yönelip hedefin direkt arkasında kalsa, salınımı sönümlesek."
    # Kare 5 (4.3 m): ufuk DÜZ, hedef tam ortada, kutu 0.92 — mükemmel.
    # Kare 7 (2 s sonra): ufuk 40° YATIK, drone hedefin ÖBÜR tarafına geçmiş.
    # Mesafe 6.1 → 7.6 → 5.8 → 7.7 → 15.3 → 25.4 m: iki kez gidip geldi,
    # sonra tamamen kaybetti. Tetikleyen manevra HAFİFTİ (aileron 1733).
    #
    # KÖK NEDEN — YAPISAL: yatay kanal SAF ORANSAL bir denetleyici.
    #     yaw_cmd = iris_yaw + K_YAW·eps_yaw        (K_YAW = 1.0, TAM düzeltme)
    # Türev/sönümleme terimi YOK. Gecikmeli bir sistemde saf-P denetleyici
    # ZORUNLU olarak salınır — bu bir ayar değil YAPI eksiği. Araç hedefe
    # doğru dönerken "yeterince döndüm, yavaşla" diyen hiçbir şey yok;
    # hatayı ancak sıfırı geçtikten SONRA fark ediyor.
    #
    # ÇÖZÜM: aracın KENDİ dönüş hızına karşı koyan bir terim (rate feedback):
    #     eps_sonumlu = eps_yaw − SONUM_T · yaw_hizi
    # SONUM_T saniye biriminde: araç ω rad/s dönüyorsa komut ω·SONUM_T kadar
    # geri çekilir. Klasik PD; P-only aşımının ders kitabı çaresi.
    # ⚠ Girdi aracın KENDİ IMU'su (yaw türevi) — canlı hedef GPS'i yok, D0 temiz.
    # ⚠ Düz uçuşta etkisiz: hedef düz giderken drone dönmüyor (ω≈0) → terim 0.
    # AVCI_IBVS_SONUM=0 → kapalı (varsayılan; açık değeri ölçüm belirleyecek).
    SONUM_T = _env_f("AVCI_IBVS_SONUM", 0.0)   # s; 0 = kapalı, açık ~0.30
    SONUM_MAX_DEG = 30.0    # °; sönümleme teriminin tavanı (ters yöne itmesin)

    # ══ Ö5 · DÖNÜŞ-FARKINDA HIZ TAVANI ══
    # KULLANICI ÖLÇÜTÜYLE BULUNDU (2026-08-11): salınım artık hedefin
    # çerçevesindeki YANAL konumdan ölçülüyor (tools/salinim.py). 12 koşuda
    # SAĞA AŞIM 8-47 m — yani drone hedefin arkasında kalıyor ama YANINA
    # 8-47 metre savruluyor. "önde %" ise ~0: sorun boyuna değil, YANAL.
    #
    # FİZİK: dönüş yarıçapı R = V²/a. Aracın a tavanı g·tan(ANGLE_MAX 45°)
    # = 9.81 m/s². 18 m/s'de R = 33 m; hedef (Talon, 15 m/s, 60° yatış)
    # R = 13 m çiziyor. Drone 2.5 kat geniş yay çizdiği için DIŞARI taşıyor.
    # Yatışı artırmak denendi (Ö6) — çalışmadı, kanal zaten doymuş.
    # Geriye tek kaldıraç: HIZI KISMAK. R hızın KARESİYLE düşer:
    #     18 m/s → 33.0 m       12 m/s → 14.7 m       9 m/s → 8.3 m
    #
    # YASA: gereken yanal ivme = V·λ̇ ; bu a_max'ı aşıyorsa hız kısılır.
    #     v_tavan = DONUS_A / λ̇        (λ̇ = LOS azimut oranı, zaten ölçülü)
    # Yalnız KISAR; hızı asla artırmaz. Düz uçuşta λ̇≈0 → tavan sonsuz →
    # etkisiz (kullanıcının doğruladığı düz uçuş davranışı korunur).
    # ⚠ Taban: DONUS_V_MIN altına inmez — hedeften tamamen kopmayalım.
    # AVCI_IBVS_DONUS=0 → kapalı (varsayılan).
    # ⛔ Ö-K (kör devam) 2026-08-15'te ÖLÇÜLDÜ ve ELENDİ: birincil ölçüt düz,
    # en yakın menzil 3.24 → 4.28 m GERİLEDİ. Kod §5.12 uyarınca tamamen
    # çıkarıldı; ölçüm UYGULANACAK.md ve docs/ibvs_sicili.html'de durur.
    DONUS_A = _env_f("AVCI_IBVS_DONUS", 0.0)     # m/s²; 0 = kapalı, açık ~9.0
    DONUS_V_MIN = _env_f("AVCI_IBVS_DONUS_VMIN", 10.0)   # m/s; hız tabanı




    # ══ Ö12 · YAKIN MENZİLDE YAW SLEW TAVANI (KENDİ EKSENİNDE DÖNME ÇARESİ) ══
    # KULLANICI GÖZLEMİ (2026-08-12): "araç manevra limitleri zorlandığında ya
    # da hedefi pas geçtiğinde kendi etrafında dönmeye başlıyor, çok hızlı yaw
    # yapıp olduğu yerde kalıyor, 15 saniyede düzeliyor."
    #
    # ÖLÇÜLDÜ — 30 koşunun 10'unda KURTARMA bekçisi tetiklenmiş. T09'da
    # tetikten hemen önceki kareler:
    #     cx 208 → 222 → 262 → 280 (hedef kadrajı tarıyor, pas geçiş)
    #     yaw komut hızı 122 / 118 / 122 °/s  → YAW_RATE_MAX TAVANINDA
    # Yaw hedefi tavanda SÜREKLİ kaçıyor; aracın GERÇEK yaw hızı 300°/s'yi
    # aşınca kurtarma bekçisi güdümü kesiyor (kurtarma.py) → araç olduğu
    # yerde kalıp dönüyor, hedef bu arada uzaklaşıyor.
    #
    # KÖK NEDEN: menzil küçüldükçe hedefin AÇISAL hızı 1/R ile patlıyor.
    # 8 m'de ~100°/s, 2 m'de ~400°/s. Araç bunu zaten TAKİP EDEMEZ; peşinden
    # gitmeye çalışmak yaw'ı doyurup savurmaktan başka işe yaramıyor.
    #
    # ÇÖZÜM: yaw slew tavanı menzille ölçeklenir — uzakta tam, yakında kısık.
    #     tavan_eff = YAW_RATE_MAX_DEG · clamp(R/YAW_MENZIL_REF, YAW_MIN_KAT, 1)
    #
    # ⚠ NEDEN BAŞKA DURUMU BOZAMAZ (yapısal): yaw slew sınırı YALNIZ BURNU
    # etkiler. Hız vektörü (vx, vy) `hiz_yonu`ndan hesaplanır ve bu sınırdan
    # GEÇMEZ — uçuş yolu, kesişim geometrisi, dikey kanal aynen kalır.
    # Tek risk kameranın hedefi kadrajda tutması; o da zaten pas geçişte
    # kaybediliyordu. Birim testi B67 hız vektörünün değişmediğini bekçiler.
    # ⚠ UZAK MENZİLDE ETKİSİZ: R ≥ YAW_MENZIL_REF iken tavan aynen 120°/s.
    # AVCI_IBVS_YAW_MENZIL=0 → kapalı (varsayılan).
    YAW_MENZIL_REF = _env_f("AVCI_IBVS_YAW_MENZIL", 0.0)  # m; 0 = kapalı, ~15
    YAW_MIN_KAT = _env_f("AVCI_IBVS_YAW_MINKAT", 0.35)    # tavanın alt sınırı



    # Tek fazda hız tavanı. Hedef 15.1 m/s uçuyor → kapanma = V_HUCUM − 15.1.
    # 20 → 4.9 m/s kapanma (son 3 m: 0.61 s). Çok yükseltmek dikey kanala
    # oturma süresi bırakmaz (D2'de ölçüldü: 3 m içinde |dikey| 0.21 → 1.06 m).
    # ⚠ 2026-08-19 KULLANICI KARARI: 20.0 → 18.0. Kullanıcı 18'e çekip
    # uçtu, "bir sorun olmadı, drone hedef araca yaklaşıp vurabiliyor" dedi
    # ve kalıcı olmasını istedi. Hedef 15.1 m/s → kapanma 2.9 m/s.
    # ⚠ GEÇİCİ: menzile bağlı yavaşlama profili gelince bu sabit tavan
    # yerini o profile bırakacak (yol haritası, sıra 2).
    V_HUCUM = _env_f("AVCI_IBVS_V_HUCUM", 18.0)          # m/s


    # Dikey saf takip kazancı. 1.0 = yatayla AYNI (K_YAW da 1.0) — hız
    # vektörü doğrudan hedefe döner. <1 tembelleşir, >1 aşım yapar.
    K_ELEV = _env_f("AVCI_IBVS_KELEV", 1.0)

    # Dikey türev sönümlemesi (tek faz kendi kazancını taşır; terminalinkini
    # ÖDÜNÇ ALMAZ — §5.12: iki özellik aynı alanı paylaşmasın).
    K_VZ_D = _env_f("AVCI_IBVS_KVZD", 0.6)

    # ── TEK FAZ · "KAÇIRACAKSAN YAVAŞLA" (kullanıcının kendi fikri) ──
    # Kullanıcı (2026-08-18): "hedef aracı kaçıracak gibiysek eğer hızı
    # azaltıp öyle dengeli yaklaşsak olmuyor mu?"
    # NEDEN GEREKLİ: dikey tavan VZ_MAX, hız v_los iken vektörün eğilebileceği
    # en dik açı asin(VZ_MAX/v_los)'tur. 8 ve 20 ile bu YALNIZCA 23.6° —
    # hedef daha yukarıdaysa kesişim MATEMATİKSEL OLARAK İMKÂNSIZ, komut
    # kırpılır ve drone altından geçer. Saf takip yasasının sessiz deliği bu.
    # ÇÖZÜM: dikey tavan yetmiyorsa YATAYI KIS — oran düzelsin, vektör
    # hedefi gösterebilsin. Doğrulandı (D3 ölçümü): 45°'de tavan tek başına
    # 38.7°'de takılıyordu, yavaşlamayla 45.0°'ye ulaşıldı.
    # ⚠ Hızı ASLA ARTIRMAZ ve V_HUCUM_MIN altına inmez — yoksa 15.1 m/s uçan
    # hedefe büsbütün yetişemeyiz.
    YAVASLA = _env_f("AVCI_IBVS_YAVASLA", 1.0) >= 0.5
    V_HUCUM_MIN = _env_f("AVCI_IBVS_VHUCUM_MIN", 12.0)   # m/s; yavaşlama tabanı

    # ═══════════════════════════════════════════════════════════════════
    # YAVAŞLAMA PROFİLİ + HEDEF HIZI KESTİRİMİ (2026-08-19, kullanıcı fikri)
    # ═══════════════════════════════════════════════════════════════════
    # KULLANICI: *"Hedefin kadrajda büyümesiyle orantılı şekilde avcı dronun
    # hızını azaltma gibi bir şey yapabilir miyiz? Çünkü hedef araca
    # çarparken hedef araç ile yakın hızlarda olmak aracı kaçırma riskini
    # minimuma indirir, daha dengeli yaklaşmayı mümkün kılar."*
    #
    # ⛔ "KUTU BOYUTUYLA ORANTILI" YAPILMADI — kasıtlı. Kutu 1/R ile gider:
    #   menzil 30→5 m (25 metre!) → kutu hatası 155→128, yani %17 değişim
    #   menzil  2→1 m (tek metre) → kutu hatası  80→  0, yani %100 değişim
    # Yani kutu-orantılı yavaşlama, TAM TEMAS ANINDA ANİ FREN demektir. Ani
    # fren de burnu kaldırıp hedefi kadrajdan çıkarır — sildiğimiz terminal
    # fazının kök nedeni tam buydu. Bunun yerine MENZİLLE DOĞRUSAL profil.
    #
    # YASA — ileri besleme + integral:
    #   kapanma_hedefi = clamp(R / T_GO, KAPANMA_MIN, KAPANMA_MAX)
    #   v_hedef_I     += K_I_KAP · (kapanma_hedefi − kapanma_ölçülen) · dt
    #   v_los          = clamp(v_hedef_I + kapanma_hedefi, V_MIN, V_HUCUM)
    #
    # NEDEN BU BİÇİM: profil DOĞRUDAN eklenir (gecikmesiz, menzille iner);
    # integral yalnız BİLİNMEYENİ öğrenir — hedefin hızı. Dengede
    # kapanma_ölçülen = kapanma_hedefi ve v_hedef_I = hedefin gerçek hızı.
    #
    # ⚠ GÜRÜLTÜ: `kapanma` kutu büyüme hızından gelir ve gürültülüdür
    # (ölçüldü: ardışık kare değişimi p90 %17.5, gerçek değişim ~%2.5 —
    # yedi kat). Bu yüzden YALNIZ İNTEGRAL yolunda kullanılır; integratör
    # zaten alçak geçiren filtredir. Oransal yola konsaydı komut titrerdi.
    #
    # ⚠ ZENON: taban olmasaydı R→0 iken kapanma→0 olur ve ARAÇ HEDEFE ASLA
    # DEĞMEZDİ. KAPANMA_MIN bunu engeller.
    #
    # ⚠ ÖLÇÜLMÜŞ RİSK: yavaş kapanma hedefe kaçma zamanı verir. V_TERMINAL=16
    # (kapanma 0.9 m/s) ile araç 8 SANİYE 6 metrede asılı kalmış ve hiç
    # yaklaşamamıştı. Taban ve T_GO bunun ayarıdır.
    #
    # AVCI_IBVS_YAVASLAMA=1 → açık. Varsayılan KAPALI (ölçülmeden girmez).
    YAVASLAMA = _env_f("AVCI_IBVS_YAVASLAMA", 0.0) >= 0.5

    # Profilin zaman sabiti: kapanma = R / T_GO.
    # 4 s → 20 m'de 5.0 m/s · 10 m'de 2.5 · 5 m'de 1.25 · 2 m'de 0.50
    T_GO = _env_f("AVCI_IBVS_TGO", 4.0)                  # s

    # Kapanma tabanı — TEMASI GARANTİ EDER (Zenon kalkanı).
    # ⚠ AD SEÇİMİ: "KAPANMA_MIN" DEĞİL. O ad silinen terminal fazının
    # "kapanma ölçeği tabanı" alanına aitti; aynı adı farklı anlamla geri
    # getirmek §5.12'nin uyardığı karışıklığı üretirdi (birim testi B1
    # zaten yakaladı).
    KAPANMA_TABAN = _env_f("AVCI_IBVS_KAPANMA_TABAN", 1.5)  # m/s
    # Kapanma tavanı — uzakta imkânsız hız istenmesin.
    KAPANMA_TAVAN = _env_f("AVCI_IBVS_KAPANMA_TAVAN", 6.0)  # m/s

    # Hedef hızı kestiriminin öğrenme hızı. Düşük = sakin ama geç;
    # yüksek = çevik ama gürültüyü içeri alır ve salınabilir.
    K_I_KAP = _env_f("AVCI_IBVS_KI_KAP", 0.8)            # (m/s)/(m/s·s)

    CONF_MIN = _env_f("AVCI_IBVS_CONF", 0.35)   # bunun altı kutu = yok sayılır
    BOYUT_MIN = 6.0                # px; bundan küçük kutu güvenilmez (gürültü)


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

_CSV_ALANLAR = [
    "t", "dt", "durum", "cx", "cy", "w", "h", "boyut", "conf",
    "eps_yaw_deg", "eps_yaw_ham_deg", "nisan_elev_deg",
    "iris_roll_deg", "iris_pitch_deg", "iris_yaw_deg",
    "boyut_hata", "hiz_I", "v_los", "kacis_ek", "gecikme_s", "eps_hiz_deg", "sonum_deg", "donus_tavan", "lead_az_deg", "los_hiz_az", "los_hiz_el",
    "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg", "kayip_sayac",
    "elev_atalet_deg", "kapanma_hedefi", "kapanma_olculen",
]


def kutu_olcusu(w, h, cfg=Cfg):
    """Kutunun MENZİL için kullanılacak tek sayılık boyu (piksel).

    "carpim"  : sqrt(w·h) — geometrik ortalama (2026-08-19 öncesi tek yol)
    "kosegen" : sqrt(w²+h²) — eksen-hizalı kutunun köşegeni

    ⚠ KÖŞEGEN NEDEN: kutu eksen-hizalı olduğu için, kadrajda θ kadar dönmüş
    İNCE BİR ÇUBUK için w=L·|cosθ|, h=L·|sinθ| olur ve köşegen tam L kalır —
    yatıştan BAĞIMSIZ. Talon arkadan bakınca büyük ölçüde ince çubuktur.
    Ölçüldü: 0-15° bandında bağıl menzil hatası %22 → %14.
    """
    w = max(w, 0.0)
    h = max(h, 0.0)
    if cfg.BOYUT_OLCU == "kosegen":
        return math.sqrt(w * w + h * h)
    return math.sqrt(w * h)


def menzil_sabiti(cfg=Cfg):
    """Seçili ölçünün kalibre sabiti C (px·m); R = C / kutu_olcusu."""
    if cfg.BOYUT_OLCU == "kosegen":
        return cfg.MENZIL_PX_M_KOSEGEN
    return cfg.MENZIL_PX_M_CARPIM


def piksel_elev(cy, cfg=Cfg):
    """Kutunun dikey pikselinden GÖVDE çerçevesinde LOS yükselişi (rad, yukarı+).

    Kamera gövdeye KAMERA_TILT (25°) yukarı tilt'li. cy=CY (kadraj merkezi)
    → boresight → yükseliş = +25°. cy büyüdükçe (kadrajda aşağı) yükseliş azalır.
    Doğrulama: cy = CY + FY·tan(25°) ≈ 318 → yükseliş ≈ 0 (seviye hedef).
    """
    tilt = math.radians(GeoCfg.KAMERA_TILT_DEG)
    b = (cy - geo.CY) / geo.FY
    return math.atan2(math.sin(tilt) - math.cos(tilt) * b,
                      math.cos(tilt) + math.sin(tilt) * b)


def los_seviye(cx, cy, roll, pitch, cfg=Cfg):
    """Piksel + aracın KENDİ duruşu → SEVİYE çerçevesinde (azimut, yükseliş).

    Neden gerekli: bkz. Cfg.ROLL_TELAFI. atan((cx−CX)/FX) KAMERA çerçevesinin
    azimutudur; araç yattığında bu, seviye çerçevesindeki gerçek azimut DEĞİLDİR
    (30-40° yatışta 11-14° sapma ölçüldü).

    Zincir — üç adım, hepsi drone'un kendi sensörüyle (canlı GPS YOK):
      1) piksel → kamera ışını      [sağ, aşağı, ileri] = (x, y, 1)
      2) kamera → GÖVDE (FRD)       kamera KAMERA_TILT° yukarı vidalı: Ry(−tilt)
      3) gövde → SEVİYE (yaw hariç) Ry(pitch)·Rx(roll) ile duruş çıkarılır

    Dönüş: (azimut, yükseliş) rad — azimut BURNA GÖRE sağ+, yükseliş yukarı+.
    Yani çağıran seviye çerçevesindeki mutlak yönü `iris_yaw + azimut` ile alır.

    Doğrulama (roll=pitch=0, cx=CX): azimut=0 ve yükseliş = piksel_elev(cy).
    """
    x = (cx - geo.CX) / geo.FX          # kamera sağ  (CX = ana nokta)
    y = (cy - geo.CY) / geo.FY          # kamera aşağı
    t = math.radians(GeoCfg.KAMERA_TILT_DEG)
    ct, st = math.cos(t), math.sin(t)
    # 2) kamera ışını → gövde FRD
    bx = ct + st * y                    # ileri
    by = x                              # sağ
    bz = ct * y - st                    # aşağı
    # 3) gövde → seviye: önce roll, sonra pitch geri alınır
    cr, sr = math.cos(roll), math.sin(roll)
    y1 = by * cr - bz * sr
    z1 = by * sr + bz * cr
    cp, sp = math.cos(pitch), math.sin(pitch)
    x2 = bx * cp + z1 * sp
    z2 = -bx * sp + z1 * cp
    return math.atan2(y1, x2), math.atan2(-z2, math.hypot(x2, y1))


def komut(cx, cy, w, h, iris_yaw, hiz_I, dt, cfg=Cfg,
          los_hiz=(0.0, 0.0), iris_pitch=0.0, iris_vz=0.0,
          kapanma=None, iris_roll=0.0, yaw_hizi=0.0):
    """IBVS kontrol yasası — SAF TAKİP + PI hız (MAVLink yok, CANLI GPS yok).

    Girdi:
      (cx,cy,w,h) : tespit kutusu — TEK canlı kaynak
      iris_yaw    : drone kendi yaw'ı (rad) — kendi sensörü
      hiz_I       : hız integralinin o anki değeri (m/s) — çağıran taşır
      dt          : adım süresi (s)
    Çıktı: (vx_ned, vy_ned, vz, yaw_cmd, hiz_I_yeni, tani)

    Hız DAİMA LOS (burun) yönünde: hedef dönünce hız vektörü de döner —
    dondurulmuş NED taşıyıcının yana savurma hatası yapısal olarak imkânsız.
    """
    boyut = kutu_olcusu(w, h, cfg)
    _C = menzil_sabiti(cfg)

    # YAW: yatay açı hatası → burun hedefe
    # ROLL/PITCH TELAFİSİ (bkz. Cfg.ROLL_TELAFI): araç yattığında kamera
    # azimutu seviye azimutu DEĞİLDİR. Telafili yol pikseli aracın kendi
    # duruşuyla seviye çerçevesine döndürür; hız vektörü de bu yöne gider.
    eps_yaw_ham = math.atan((cx - cfg.CX_NISAN) / geo.FX)
    if cfg.ROLL_TELAFI:
        eps_yaw, _ = los_seviye(cx, cy, iris_roll, iris_pitch, cfg)
    else:
        eps_yaw = eps_yaw_ham
    # LEAD ÖLÇEĞİ: kalan süreyle (≈menzille) söner — bkz. Cfg.LEAD_SONUM.
    # boyut ∝ 1/menzil olduğu için REF/boyut ≈ menzil/menzil_REF.
    lead_olcek = 1.0
    if cfg.LEAD_SONUM and boyut > 1e-6:
        # lead, LEAD_MENZIL_M'nin altında kademeli söner. Piksel yerine
        # METRE eşik: ölçü değişse de aynı fiziksel menzilde söner.
        lead_olcek = clamp((_C / cfg.LEAD_MENZIL_M) / boyut, 0.0, 1.0)
    lead_sure = cfg.LEAD_SURE * lead_olcek
    lead_az = 0.0
    # LEAD: nişanı atalet LOS dönüş hızıyla öne al (bkz. Cfg.LEAD_SURE).
    # M3: kapı kalktı — artık kutu olan her karede (bkz. Cfg.LEAD_ERKEN).
    lead_az = clamp(lead_sure * los_hiz[0],
                    -math.radians(cfg.LEAD_MAX_DEG),
                    math.radians(cfg.LEAD_MAX_DEG))
    # Ö9 SÖNÜMLEME: aracın kendi dönüş hızı komutu geri çeker (bkz. SONUM_T)
    sonum = 0.0
    if cfg.SONUM_T > 0.0:
        sonum = clamp(cfg.SONUM_T * yaw_hizi,
                      -math.radians(cfg.SONUM_MAX_DEG),
                      math.radians(cfg.SONUM_MAX_DEG))
    yaw_cmd = normalize_angle(iris_yaw + cfg.K_YAW * eps_yaw - sonum + lead_az)

    # HIZ: kutu boyutu hatası üzerinden PI (terminalde TAM taahhüt)
    # DENGE KUTUSU = TEMAS KUTUSU (park yok) —
    # hata hep büyük pozitif kalır, PI "kapat" der ve hız V_HUCUM'te oturur.
    # HIZ: kutu boyutu hatasi uzerinden PI — DENGE KUTUSU = TEMAS KUTUSU.
    # Yani "su menzilde dur" diye bir nokta YOK; hata hep buyuk pozitif kalir
    # ve hiz V_HUCUM tavaninda oturur. Sabit kapanma orani.
    # ⚠ Eski sistemde bu BOYUT_REF=25 px idi → 160/25 = 6.4 m'de PARK ederdi;
    # terminal fazi tam da o parki ezmek icin vardi. Park kalkinca faza da
    # gerek kalmadi (bkz. dosya basindaki TEK GORSEL FAZ notu).
    # Denge kutusu = TEMAS kutusu. Piksel karşılığı ölçüye göre kendiliğinden
    # ölçeklenir (HUCUM_MENZIL_M metre olarak sabit kalır).
    hata = (_C / cfg.HUCUM_MENZIL_M) - boyut    # px; + = uzak
    kacis_ek = 0.0
    kapanma_hedefi = None
    if cfg.YAVASLAMA:
        # ── YAVAŞLAMA PROFİLİ + HEDEF HIZI KESTİRİMİ (bkz. Cfg.YAVASLAMA) ──
        # `hiz_I` burada ARTIK "hız integrali" değil, HEDEFİN HIZI
        # KESTİRİMİDİR. İsim aynı kaldı çünkü çağıran onu taşıyor.
        _R = _C / boyut if boyut > 1e-6 else cfg.KAPANMA_TAVAN * cfg.T_GO
        kapanma_hedefi = clamp(_R / cfg.T_GO,
                               cfg.KAPANMA_TABAN, cfg.KAPANMA_TAVAN)
        # ── ANTI-WINDUP (2026-08-19, ilk kampanyada MEKANİZMA KAPISI yakaladı)
        # İlk sürümde integral koşulsuz güncelleniyordu ve ŞİŞTİ: uzakta
        # profil 5.5 m/s kapanma istiyor, V_HUCUM=18 buna izin vermiyor,
        # hata kapanmıyor, `hiz_I` I_MAX'a (24) tırmanıyordu. Yakına gelince
        # kapanma_hedefi 1.5'e düşse bile 24+1.5 yine tavana çarpıyor ve
        # ÖZELLİĞİN HIZA SIFIR ETKİSİ oluyordu (ölçüldü: v_los her menzil
        # bandında tam 18.00).
        # ÇÖZÜM: çıktı doyumdayken, doyumu DERİNLEŞTİREN yönde integrali
        # dondur. Böylece `hiz_I` hedefin gerçek hızında kalır ve menzil
        # düşüp profil daralınca v_los doyumdan ÇIKAR.
        _v_ham = hiz_I + kapanma_hedefi
        if kapanma is not None:
            # İntegral YALNIZ ölçüm varken güncellenir. Kutu kaybolunca
            # `kapanma` bayat kalır; sürüklenmesin diye DONDURULUR.
            _delta = cfg.K_I_KAP * (kapanma_hedefi - kapanma) * dt
            _ust = _v_ham >= cfg.V_HUCUM - 1e-9
            _alt = _v_ham <= cfg.V_MIN + 1e-9
            if not ((_ust and _delta > 0.0) or (_alt and _delta < 0.0)):
                hiz_I = clamp(hiz_I + _delta, cfg.I_MIN, cfg.I_MAX)
        v_los = clamp(hiz_I + kapanma_hedefi, cfg.V_MIN, cfg.V_HUCUM)
    else:
        hiz_I = clamp(hiz_I + cfg.K_I * hata * dt, cfg.I_MIN, cfg.I_MAX)
        # Ö1 KAÇIŞ TELAFİSİ (bkz. Cfg.KACIS_KD): hedef uzaklaşıyorsa (ṙ<0)
        # hızı ANINDA artır — integralin 5 saniyesini bekleme.
        if cfg.KACIS_KD > 0.0 and kapanma is not None and kapanma < 0.0:
            kacis_ek = min(cfg.KACIS_KD * (-kapanma), cfg.KACIS_MAX)
        v_los = clamp(hiz_I + cfg.K_FWD * hata + kacis_ek,
                      cfg.V_MIN, cfg.V_HUCUM)

    # Ö5 DÖNÜŞ TAVANI (bkz. Cfg.DONUS_A): gereken yanal ivme V·λ̇ aracın
    # tavanını aşıyorsa hızı kıs — yarıçap V² ile düştüğü için dönüş sıkışır.
    # ⚠ YALNIZ KISAR. Düz uçuşta λ̇≈0 → tavan çok büyük → etkisiz.
    donus_tavan = None
    if cfg.DONUS_A > 0.0:
        _lam = abs(los_hiz[0])
        if _lam > 1e-3:
            donus_tavan = max(cfg.DONUS_V_MIN, cfg.DONUS_A / _lam)
            if donus_tavan < v_los:
                v_los = donus_tavan

    # ══ Ö8 · YANAL KOMUT AÇIYLA DEĞİL, KAÇIRMA MESAFESİYLE ══
    # Hız vektörünün yönü artık ayrı hesaplanır (bkz. Cfg.YANAL_K).
    # BURUN (yaw_cmd) tam eps_yaw'da kalır — kamera hedefi izlemeye devam eder.
    eps_hiz = eps_yaw
    if cfg.YANAL_K > 0.0 and boyut > 1e-6 and v_los > 0.1:
        _R = _C / boyut                               # menzil (m)
        _y = _R * math.sin(eps_yaw)                   # YANAL KAÇIRMA (m)
        _rdot = max(abs(kapanma) if kapanma is not None else 0.0,
                    cfg.YANAL_RDOT_MIN)
        _tgo = max(_R / _rdot, cfg.YANAL_TGO_MIN)     # kalan süre (s)
        _vy = cfg.YANAL_K * _y / _tgo                 # gereken yanal hız
        _eps_eff = math.asin(clamp(_vy / v_los, -1.0, 1.0))
        if abs(_eps_eff) < abs(eps_yaw):              # YALNIZ KISAR, büyütmez
            # menzil harmanı: uzakta hiç, yakında tam (bkz. YANAL_MENZIL)
            _w = clamp((cfg.YANAL_MENZIL - _R) / (0.5 * cfg.YANAL_MENZIL),
                       0.0, 1.0)
            eps_hiz = eps_yaw + _w * (_eps_eff - eps_yaw)

    # SAF TAKİP: hız LOS yönünde — ama yönü eps_hiz belirler
    hiz_yonu = normalize_angle(iris_yaw + cfg.K_YAW * eps_hiz - sonum + lead_az)
    vx_ned = v_los * math.cos(hiz_yonu)
    vy_ned = v_los * math.sin(hiz_yonu)

    # ── DİKEY = YATAYIN AYNI MATEMATİĞİ ────────────────────────────────
    # Yatay:  hiz_yonu = iris_yaw + K_YAW·eps_yaw    (vektörün YÖNÜ döner)
    # Dikey:  elev_cmd = K_ELEV·elev_los             (vektörün YÖNÜ döner)
    # ve hız vektörünün BÜYÜKLÜĞÜ her iki eksende de v_los kalır.
    # NİŞAN OFSETİ YOK: elev_los doğrudan hedefin SEVİYE çerçevesindeki
    # yükselişi (los_seviye) — roll telafisi her karede içinde, ayrı bir
    # anahtara gerek yok.
    # ⚠ Eski sistemde iki ayrı dikey yasa vardı: seyirde "hedefi CY_NISAN'da
    # TUT" (ufkun 5° yukarısı = R·sin(5°) kadar altta dur), terminalde
    # tan tabanlı KESİŞİM. İkisi arasındaki geçiş, ölçülen bütün bitiriş
    # sorunlarının kaynağıydı. Artık tek yasa var.
    _, _elev_los = los_seviye(cx, cy, iris_roll, iris_pitch, cfg)
    lead_el = clamp(lead_sure * los_hiz[1],
                    -math.radians(cfg.LEAD_MAX_DEG),
                    math.radians(cfg.LEAD_MAX_DEG))
    elev_atalet = _elev_los
    nisan_elev = clamp(cfg.K_ELEV * (_elev_los + lead_el),
                       -math.radians(60.0), math.radians(60.0))
    # KAÇIRACAKSAN YAVAŞLA (bkz. Cfg.YAVASLA): vektörün eğilebileceği en dik
    # açı asin(VZ_MAX/v_los)'tur. Hedef daha dikse kesişim imkânsız ve komut
    # kırpılır → altından geçilir. Tavan yetmiyorsa YATAYI kıs.
    if cfg.YAVASLA:
        _se = abs(math.sin(nisan_elev))
        if _se > 1e-6 and v_los * _se > cfg.VZ_MAX:
            v_los = max(cfg.V_HUCUM_MIN, cfg.VZ_MAX / _se)
    vz_nisan = -v_los * math.sin(nisan_elev)
    # TÜREV SÖNÜMLEMESİ: aracın KENDİ dikey hızı nişanın ötesine geçtiyse
    # komut geri çekilir → hedefin üstünden geçme biter (bkz. Cfg.K_VZ_D).
    vz = clamp(vz_nisan + cfg.K_VZ_D * (vz_nisan - iris_vz),
               -cfg.VZ_MAX, cfg.VZ_MAX)
    # |v| = v_los KORUNSUN: dikey ne kadar aldıysa gerisi yatayadır.
    _yat = math.sqrt(max(v_los * v_los - vz * vz, 0.0))
    vx_ned = _yat * math.cos(hiz_yonu)
    vy_ned = _yat * math.sin(hiz_yonu)

    tani = {"boyut": boyut, "eps_yaw": eps_yaw, "eps_yaw_ham": eps_yaw_ham,
            "hata": hata, "v_los": v_los,
            "eps_hiz": eps_hiz, "sonum": sonum,
            "donus_tavan": donus_tavan,
            "kacis_ek": kacis_ek,
            "lead_az": lead_az, "lead_olcek": lead_olcek,
            "kapanma_hedefi": kapanma_hedefi,
            "nisan_elev": nisan_elev,
            "elev_atalet": elev_atalet}
    return vx_ned, vy_ned, vz, yaw_cmd, hiz_I, tani


def _kutu_gecerli(pose, cfg):
    """pose kaydından geçerli kutu çıkar → (cx,cy,w,h,conf) veya None."""
    if pose is None:
        return None
    conf = pose.get("conf", 0.0)
    if conf < cfg.CONF_MIN:
        return None
    bbox = pose.get("bbox")
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        w, h = (x2 - x1), (y2 - y1)
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    elif pose.get("cx") is not None:
        cx, cy = pose["cx"], pose["cy"]
        w = pose.get("w", 0.0)
        h = pose.get("h", 0.0)
    else:
        return None
    # ⚠ BOYUT_MIN piksel güvenilirliği kapısıdır, menzil değil — bu yüzden
    # HER ZAMAN sqrt(w·h) ile ölçülür (ölçü seçiminden BAĞIMSIZ), yoksa
    # köşegene geçince eşik sessizce gevşerdi.
    if math.sqrt(max(w, 0.0) * max(h, 0.0)) < cfg.BOYUT_MIN:
        return None
    return cx, cy, w, h, conf


def run_bbox_ibvs(conn, get_iris, wait_pose, stop_event, cfg=Cfg,
                  kayip_kare_esik=20, ff_hiz=(0.0, 0.0, 0.0), get_temas=None):
    """bbox IBVS görsel güdüm döngüsü. Kutu akışına kilitli (wait_pose).

    ff_hiz: devir anındaki son GPS hız kestirimi — YALNIZ hız integralinin
    SICAK BAŞLANGIÇ değeri olarak kullanılır (|ff| skaler). ⚠ SAYI ÜÇLÜSÜ,
    callback DEĞİL: döngünün canlı GPS'e erişimi yoktur (D0 yapısal garanti).
    Yön hiçbir zaman ff'ten gelmez — hız daima LOS yönündedir (2026-08-08
    dersi: dondurulmuş NED yönü hedef döndükçe drone'u yana savuruyordu).

    get_temas: Talon'un ÇARPMA SENSÖRÜ (sim_truth.temas) — True dönerse
    'vuruldu' ile biter. ⚠ Bu bir SONUÇ sinyalidir, güdüm girdisi DEĞİL:
    hedefin yerini/hızını taşımaz, yalnız "çarpışma oldu mu" der. Güdüm
    yasası (komut) onu hiç görmez.

    kayip_kare_esik ardışık geçersiz-kutu karesi → 'kayip' döner (görsel temas
    kesildi; supervisor GPS fazına döner). stop_event → 'durduruldu'.
    """
    loop_period = 1.0 / cfg.LOOP_HZ
    son_seq = 0
    kayip_sayac = 0
    # SICAK BAŞLANGIÇ: integral hedefin bilinen seyir hızıyla başlar; böylece
    # ilk saniyelerde "hızı sıfırdan öğrenme" gecikmesi yaşanmaz. Bundan sonra
    # integrali YALNIZ görüntü hatası sürer.
    hiz_I = clamp(math.hypot(float(ff_hiz[0]), float(ff_hiz[1])),
                  cfg.I_MIN, cfg.I_MAX)
    # İvme sınırlayıcı drone'un GERÇEK hızından başlar (kendi sensörü).
    # Sıfırdan başlarsa devir anında 15 m/s'lik seyir "frenlenmiş" gibi
    # rampalanır (12 m/s² ile 1.25 s) — hedef o sırada kaçar.
    _i0 = get_iris()
    vx_p = float(_i0.get("vx", 0.0) or 0.0)
    vy_p = float(_i0.get("vy", 0.0) or 0.0)
    vz_p = float(_i0.get("vz", 0.0) or 0.0)
    son_v_cmd = None       # kutu boşluğunda sürdürülecek son komut
    prev_time = None
    cmd_yaw = None
    kurt = Kurtarma()         # duruş bekçisi (normal uçuşta hiç tetiklenmez)
    # LOS (atalet) açıları ve hızları — lead nişanı için
    los_az_onceki = los_el_onceki = None
    los_hiz = [0.0, 0.0]      # [azimut, yükseliş] rad/s, EMA'lı
    boyut_onceki = None       # kapanma hızı için (bkz. Cfg.KAPANMA)
    kapanma = None            # m/s; görüntüden ölçülen kapanma hızı, EMA'lı
    iyaw_onceki = None        # Ö9 sönümlemesi için yaw türevi (bkz. Cfg.SONUM_T)
    yaw_hizi = 0.0            # rad/s; aracın KENDİ dönüş hızı, EMA'lı

    def _vuruldu():
        if get_temas is None:
            return False
        return get_temas() is True

    os.makedirs(_LOG_DIR, exist_ok=True)
    csv_yol = os.path.join(_LOG_DIR, time.strftime("bbox_ibvs_%Y%m%d_%H%M%S.csv"))
    f = open(csv_yol, "w", newline="")
    w_csv = csv.DictWriter(f, fieldnames=_CSV_ALANLAR, extrasaction="ignore")
    w_csv.writeheader()
    print(f"[IBVS] bbox görsel güdüm başladı — SAF TAKİP + PI hız "
          f"(CANLI GPS YOK, yarışma kuralı). İntegral sıcak başlangıç: "
          f"{hiz_I:.1f} m/s, ölçü={cfg.BOYUT_OLCU}, tavan "
          f"{cfg.V_HUCUM:.0f} m/s, kayıp eşiği={kayip_kare_esik} kare, "
          f"yatay roll/pitch telafisi={'AÇIK' if cfg.ROLL_TELAFI else 'kapalı'}, "
          f"temas sensörü={'VAR' if get_temas is not None else 'yok'} "
          f"— log: {csv_yol}")

    try:
        while not stop_event.is_set():
            kayit = wait_pose(son_seq, timeout=0.5)
            if kayit is None:
                # kare akışı durdu — temas kesildi say (akış yoksa ilerleme yok)
                kayip_sayac += 1
                if kayip_sayac >= kayip_kare_esik:
                    print("[IBVS] kare akışı/temas kesildi → 'kayip'")
                    return "kayip"
                continue
            son_seq = kayit["seq"]
            # ÖLÇÜM (davranışa etkisi YOK): karenin gcs'e gelişinden
            # komut anına kadar geçen süre. Lead süresinin doğru
            # değeri bu gecikmeden çıkar — tahminle konmamalı.
            _wr = kayit.get("wall_recv")
            gecikme_s = (time.time() - _wr) if _wr else None

            if _vuruldu():
                print("[IBVS] ✓✓ VURULDU (Talon çarpma sensörü)")
                send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)
                return "vuruldu"

            now = time.monotonic()
            dt = (now - prev_time) if prev_time is not None else loop_period
            dt = clamp(dt, 0.001, 0.5)
            prev_time = now

            iris = get_iris()
            iyaw = iris.get("yaw", 0.0)
            # Ö9 için aracın KENDİ dönüş hızı (rad/s) — kendi IMU'su, D0 temiz.
            # EMA: yaw gürültüsü sönümleme terimini titretmesin.
            if iyaw_onceki is not None and 1e-3 < dt < 0.5:
                _yr = normalize_angle(iyaw - iyaw_onceki) / dt
                yaw_hizi = 0.3 * _yr + 0.7 * yaw_hizi
            iyaw_onceki = iyaw

            # ── KURTARMA BEKÇİSİ (bkz. kurtarma.py) — takla/kaçak dönmede
            # güdüm komutu kesilir. Terminal kör hücumdan da ÖNCE gelir:
            # kontrolü kaybetmiş araçla hücumu sürdürmek uçuşu bitiriyor.
            # Kayıp sayacı işlemeye devam eder → uzun sürerse GPS'e dönülür.
            if kurt.guncelle(iris.get("roll", 0.0), iris.get("pitch", 0.0),
                             iyaw, now):
                # V2 KUSUR 1: kilitli yaw hedefi. `iyaw` yollarsak "olduğun
                # yerde dur" hedefi araçla birlikte kayar ve dönme hiç durmaz
                # (bkz. kurtarma.py — kullanıcının 154505 kaydında ölçüldü).
                _kyaw = kurt.kilit_yaw if kurt.kilit_yaw is not None else iyaw
                send_velocity(conn, 0.0, 0.0, 0.0, _kyaw)
                vx_p = vy_p = vz_p = 0.0
                son_v_cmd = None
                cmd_yaw = _kyaw
                kayip_sayac += 1
                if kayip_sayac >= kayip_kare_esik:
                    print("[IBVS] kurtarma sırasında temas koptu → 'kayip'")
                    return "kayip"
                w_csv.writerow({"t": round(now, 3), "dt": round(dt, 4),
                                "durum": "KURTARMA",
                                "kayip_sayac": kayip_sayac,
                                "iris_roll_deg": round(
                                    math.degrees(iris.get("roll", 0.0)), 1),
                                "iris_pitch_deg": round(
                                    math.degrees(iris.get("pitch", 0.0)), 1),
                                "iris_yaw_deg": round(math.degrees(iyaw), 1),
                                "yaw_cmd_deg": round(math.degrees(_kyaw), 1)})
                f.flush()
                continue

            kutu = _kutu_gecerli(kayit["pose"], cfg)
            if kutu is None:
                kayip_sayac += 1
                if kayip_sayac >= kayip_kare_esik:
                    print(f"[IBVS] {kayip_kare_esik} ardışık kutusuz kare → 'kayip'")
                    return "kayip"
                # Kutu yok: SON KOMUT sürdürülür (hedefin seyri bir karede
                # değişmez). Sıfır komut vermek kısa bir tespit boşluğunu
                # kalıcı kayba çevirir. İntegral dokunulmaz (bozulmasın).
                if son_v_cmd is not None:
                    send_velocity(conn, *son_v_cmd)
                else:
                    send_velocity(conn, vx_p, vy_p, vz_p, cmd_yaw or iyaw)
                w_csv.writerow({"t": round(now, 3), "dt": round(dt, 4),
                                "durum": "KUTU_YOK", "kayip_sayac": kayip_sayac,
                                "iris_yaw_deg": round(math.degrees(iyaw), 1)})
                f.flush()
                continue

            kayip_sayac = 0
            cx, cy, bw, bh, conf = kutu

            # ── ATALET LOS AÇILARI + HIZLARI (lead nişanı girdisi) ──
            # Piksel hızı DEĞİL: yaw kontrolcüsü kutuyu merkeze çektiği için
            # piksel hızı kendi düzeltmemizi içerir. Atalet açısı = gövde
            # açısı + aracın kendi duruşu → gerçek LOS dönüşü kalır.
            ipitch = iris.get("pitch", 0.0)
            iroll = iris.get("roll", 0.0)
            # ⚠ Bu açı LEAD nişanının girdisi (los_hiz). Telafisiz halinde
            # aracın YATIŞI sahte LOS dönüş hızı üretiyordu — manevrada lead
            # de bozuluyordu. Aynı telafi burada da uygulanır.
            if cfg.ROLL_TELAFI:
                _az_s, _ = los_seviye(cx, cy, iroll, ipitch, cfg)
                los_az = normalize_angle(iyaw + _az_s)
            else:
                los_az = normalize_angle(
                    iyaw + math.atan((cx - cfg.CX_NISAN) / geo.FX))
            # ⚠ T1a: DİKEY BİLEREK DOKUNULMADI (tek değişkenli test).
            los_el = piksel_elev(cy, cfg) + ipitch
            if los_az_onceki is not None and 1e-3 < dt < 0.5:
                a_ = cfg.LEAD_EMA
                los_hiz[0] = (a_ * (normalize_angle(los_az - los_az_onceki) / dt)
                              + (1 - a_) * los_hiz[0])
                los_hiz[1] = (a_ * ((los_el - los_el_onceki) / dt)
                              + (1 - a_) * los_hiz[1])
            los_az_onceki, los_el_onceki = los_az, los_el

            # ── KAPANMA HIZI, GÖRÜNTÜDEN (bkz. Cfg.KAPANMA) ──
            # R = MENZIL_PX_M/boyut  ⇒  ṙ = −dR/dt = R·(dboyut/dt)/boyut
            # GPS YOK: yalnız kutu boyutunun büyüme hızı. Kutu titrediği için
            # EMA'lanır; ilk karede geçmiş yok, None kalır (komut o turda
            # eski davranışa düşer — güvenli taraf).
            boyut_simdi = kutu_olcusu(bw, bh, cfg)
            if (boyut_onceki is not None and boyut_simdi > 1e-6
                    and 1e-3 < dt < 0.5):
                _R = menzil_sabiti(cfg) / boyut_simdi
                _rdot = _R * ((boyut_simdi - boyut_onceki) / dt) / boyut_simdi
                _rdot = clamp(_rdot, -30.0, 30.0)      # gürültü kalkanı
                kapanma = (_rdot if kapanma is None else
                           cfg.KAPANMA_EMA * _rdot
                           + (1.0 - cfg.KAPANMA_EMA) * kapanma)
            boyut_onceki = boyut_simdi
            vx, vy, vz, yaw_hedef, hiz_I, tani = komut(
                cx, cy, bw, bh, iyaw, hiz_I, dt, cfg,
                tuple(los_hiz), ipitch,
                float(iris.get("vz", 0.0) or 0.0),
                kapanma, iroll, yaw_hizi)
            # ── YAW SLEW SINIRI (bkz. Cfg.YAW_RATE_MAX) ──
            # HIZ (vx, vy) yaw_hedef'ten hesaplandı ve DEĞİŞMEZ: nişan hedefin
            # gerçek yönünde kalır. Sınırlanan yalnız BURUNUN dönme hızı.
            if cmd_yaw is None:
                cmd_yaw = iyaw
            # Ö12: yaw slew tavanı menzille ölçeklenir (bkz. Cfg.YAW_MENZIL_REF).
            # YALNIZ BURUN — hız vektörü yukarıda hesaplandı, dokunulmuyor.
            _yaw_tavan = cfg.YAW_RATE_MAX_DEG
            if cfg.YAW_MENZIL_REF > 0.0 and tani["boyut"] > 1e-6:
                _Ryaw = menzil_sabiti(cfg) / tani["boyut"]
                _yaw_tavan *= clamp(_Ryaw / cfg.YAW_MENZIL_REF,
                                    cfg.YAW_MIN_KAT, 1.0)
            yaw_err = normalize_angle(yaw_hedef - cmd_yaw)
            adim = clamp(yaw_err, -math.radians(_yaw_tavan) * dt,
                         math.radians(_yaw_tavan) * dt)
            cmd_yaw = normalize_angle(cmd_yaw + adim)
            yaw_cmd = cmd_yaw

            # ivme sınırı (komut hızı sıçramasın)
            vx, vy, vz = limit_acceleration(vx, vy, vz, vx_p, vy_p, vz_p,
                                            cfg.MAX_ACCEL, dt)
            vx_p, vy_p, vz_p = vx, vy, vz
            son_v_cmd = (vx, vy, vz, yaw_cmd)
            send_velocity(conn, vx, vy, vz, yaw_cmd)

            w_csv.writerow({
                "t": round(now, 3), "dt": round(dt, 4),
                "durum": "GORSEL",
                "cx": round(cx, 1), "cy": round(cy, 1),
                "w": round(bw, 1), "h": round(bh, 1),
                "boyut": round(tani["boyut"], 1), "conf": round(conf, 3),
                "eps_yaw_deg": round(math.degrees(tani["eps_yaw"]), 1),
                # ÖLÇÜM SÜTUNU: telafisiz okuma. Farkı (ham − telafili) roll'e
                # karşı çizince T1a'nın uçuşta ne kadar bağladığı doğrudan
                # görülür. Yalnız log — güdüm bunu kullanmaz.
                "eps_yaw_ham_deg": round(math.degrees(tani["eps_yaw_ham"]), 1),
                "nisan_elev_deg": round(math.degrees(tani["nisan_elev"]), 2),
                "iris_roll_deg": round(math.degrees(iroll), 1),
                "iris_pitch_deg": round(math.degrees(ipitch), 1),
                "iris_yaw_deg": round(math.degrees(iyaw), 1),
                "boyut_hata": round(tani["hata"], 1),
                "hiz_I": round(hiz_I, 2), "v_los": round(tani["v_los"], 2),
                # MEKANİZMA SÜTUNLARI (§5.1): yavaşlama profili çalıştı mı?
                "kapanma_hedefi": ("" if tani["kapanma_hedefi"] is None
                                   else round(tani["kapanma_hedefi"], 2)),
                "kapanma_olculen": ("" if kapanma is None
                                    else round(kapanma, 2)),
                "kacis_ek": round(tani["kacis_ek"], 2),
                "gecikme_s": (round(gecikme_s, 4)
                              if gecikme_s is not None else ""),
                "eps_hiz_deg": round(math.degrees(tani["eps_hiz"]), 1),
                "sonum_deg": round(math.degrees(tani["sonum"]), 2),
                "donus_tavan": ("" if tani["donus_tavan"] is None
                                else round(tani["donus_tavan"], 2)),
                "lead_az_deg": round(math.degrees(tani["lead_az"]), 2),
                "los_hiz_az": round(los_hiz[0], 3), "los_hiz_el": round(los_hiz[1], 3),
                "vx_cmd": round(vx, 2), "vy_cmd": round(vy, 2),
                "vz_cmd": round(vz, 2),
                "yaw_cmd_deg": round(math.degrees(yaw_cmd), 1),
                "kayip_sayac": 0,
                "elev_atalet_deg": ("" if tani.get("elev_atalet") is None
                                    else round(math.degrees(tani["elev_atalet"]), 2)),
            })
            f.flush()

            _elapsed = time.monotonic() - now
            if _elapsed < loop_period:
                time.sleep(loop_period - _elapsed)

        send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)
        return "durduruldu"
    finally:
        f.close()
        print(f"[IBVS] log kapatıldı: {csv_yol}")
