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
  DİKEY   : dikey piksel hatası (cy − CY_NISAN) → tırman/alçal.
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

    # ── KADRAJ NİŞAN NOKTASI ──
    # ⚠ GEOMETRİ (2026-08-08 uçuş dersi): kamera gövdeye 25° YUKARI tilt'li.
    # SEVİYE (co-altitude) bir hedef kadrajda merkezde DEĞİL, AŞAĞIDA görünür:
    #     cy_seviye = CY + FY·tan(25°) = 240 + 166.6·0.466 ≈ 318 px
    # İlk sürümde nişan 210 (üst) alınmıştı — bu "hedefin ~8 m ALTINA dal"
    # demekti: drone vz'yi tavana (+4) yapıştırıp sürekli alçaldı, hedef
    # kadrajın altından (cy→390→dışarı) kaçtı, faz 3.1 s'de koptu.
    # DÜZELTME: nişanı seviye-hedef konumunun hafif ÜSTÜNE al (drone hedefin
    # az altında kalsın — gökyüzü arka planı + terminal pop-up). tan(20°) ile
    # ~10° altı: cy ≈ 240 + FY·tan(20°) ≈ 300.
    _CY_SEVIYE = geo.CY + geo.FY * math.tan(math.radians(20.0))
    CY_NISAN = _env_f("AVCI_IBVS_CY", round(_CY_SEVIYE, 0))  # ≈300 px
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

    # ── DİKEY ──
    # eps_elev = atan((cy − CY_NISAN)/FY); v_z = K_VZ · V_NOM · eps_elev.
    # cy büyük (hedef kadrajda AŞAĞIDA) → hedef boresight'ın altında → ALÇAL
    # (vz>0, NED down+). Nominal hızla ölçekli: hızlı giderken dik açı daha çok
    # dikey hız ister (irtifayı korumak için).
    # K_VZ 1.2 → 0.5, VZ_MAX 4 → 3 (2026-08-08): ilk sürüm dikey hızı çok
    # agresifti (10° hata → 2.5 m/s) ve tavana yapışıp salındı. Nişan doğru
    # yere gelince (≈300) hata küçük kalır; yumuşak kazanç yeter.
    K_VZ = _env_f("AVCI_IBVS_KVZ", 0.5)
    VZ_MAX = 3.0                    # m/s; dikey hız tavanı
    V_NOM = 12.0                   # m/s; dikey ölçekleme için nominal ileri hız

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
    BOYUT_REF = _env_f("AVCI_IBVS_REF", 25.0)   # px; sqrt(w·h) denge boyutu
    K_FWD = _env_f("AVCI_IBVS_KFWD", 0.35)      # (m/s)/px; P kazancı
    K_I = _env_f("AVCI_IBVS_KI", 0.04)          # (m/s)/(px·s); İ kazancı
    I_MIN, I_MAX = 0.0, 24.0       # m/s; integral penceresi (windup koruması)

    # V_MIN 0 (2026-08-08, kullanıcı kararı): GERİ ÇEKİLME YOK. Eski −2 m/s
    # "fren"i, kutu REF'i aşınca drone'u geri itiyordu — kullanıcı düz uçuş
    # koşusunda bunu görüp "fren olmasa vururduk" dedi. Görev vuruş; tutuş
    # mesafesinde beklemek değil.
    V_MIN = 0.0                    # m/s; asla geri gitme
    # 18 → 24 (2026-08-08, kullanıcı kararı): görsel faz KUYRUK takibi yapıyor,
    # GPS fazındaki "hızlanınca çember büyür" tuzağı burada YOK. 18 tavanında
    # komut %83 doygundu → hedefe (15-16 m/s) pay kalmıyordu, mesafe 30 m'de
    # donuyordu. GPS fazının V_MAX'ı 18'de KALIR (orada çember riski gerçek).
    V_TOPLAM_MAX = _env_f("AVCI_IBVS_VMAX", 24.0)   # m/s; yatay hız tavanı

    # ── TERMİNAL HÜCUM (mandal) ──
    # Kutu bu boyutu aşınca (≈ birkaç metre) kontrol "tut" modundan çıkar ve
    # LOS boyunca TAM hızla taahhüt eder; bir kez girilince mandal kilitli
    # kalır (kutu titrese de geri dönmez). Kutu kaybolursa son komut sürer —
    # kör hücum: terminalde hedef kadrajdan çıkabilir, çarpışma tamamlanmalı.
    # 45 → 25 px (2026-08-08, kullanıcı kararı — "1. madde"): 45 px ≈ 3.6 m
    # demekti; 24 m/s'lik hücumla o mesafe 0.15 s'de kapanıyor ve kamera 30 Hz'de
    # yalnız 4-5 kare görüyordu — hedefin son anki kaçışını düzeltecek zaman yok,
    # 7 hücumun 7'si ıska (en yakın 1.5 m). 25 px ≈ 6.4 m'den taahhüt → düzeltmeye
    # ~20 kare kalır. Ölçüm: kutu ≈ 160/menzil (12 m'de 12-14 px, uçuş logu).
    # 25, BOYUT_REF ile aynı: "tutuş mesafesine varınca hücuma geç" demek.
    TERMINAL_BOYUT = _env_f("AVCI_IBVS_TERM", 25.0)  # px; ≈6.4 m
    # ⚠ KÖR HÜCUM SÜRE SINIRI (2026-08-08, pahalı hata): ilk sürümde kör
    # hücumun süresi YOKTU. Drone hedefi ıskalayıp geçti, kutu kayboldu ve
    # son komut 260 s boyunca basıldı — araç 1032 m uzağa düz uçtu, faz hiç
    # 'kayip' dönmedi. Kör hücum çarpışmayı TAMAMLAMAK içindir; bu süre
    # içinde temas gelmezse ıska sayılır ve GPS fazına dönülür.
    TERMINAL_SURE = _env_f("AVCI_IBVS_TERM_SURE", 2.0)   # s

    # ── TERMİNAL NİŞANI: KESİŞİM + LEAD (2026-08-08, kullanıcı "2. madde") ──
    #
    # ÖLÇÜM (term25 uçuşu, en yakın anlar; ıska hedef çerçevesinde ayrıştırıldı):
    #     mesafe 0.9 m → yanal +0.5, DİKEY +0.5
    #     mesafe 0.8 m → yanal  0.0, DİKEY −0.2
    #     mesafe 1.9 m → yanal −0.1, DİKEY −0.8
    # Talon'un çarpışma gövdesi KANATLAR DAHİL (fuselage+left_wing+right_wing),
    # yani 0.8 m'de değmeliydi. Iskanın baskın bileşeni DİKEY.
    #
    # KÖK NEDEN: terminalde bile dikey kanal "TUTUŞ" yasasıydı — hedefi
    # CY_NISAN'da (≈5° yukarıda) tutmaya çalışıyor, yani ALTINDAN geçiyoruz.
    # Kesişim için hız vektörünün hedefe DOĞRU bakması gerekir, hedefi sabit
    # bir açıda tutması değil.
    #
    # ÇÖZÜM (yalnız TERMİNALDE; tutuş davranışı değişmez):
    #   1) KESİŞİM: vz = −v_los·tan(elev_hedef). elev, pikselden ve gövde
    #      pitch'inden çıkar (kamera 25° yukarı tilt'li).
    #   2) LEAD: nişan, ATALET çerçevesindeki LOS DÖNÜŞ HIZIYLA öne alınır
    #      (klasik lead pursuit / PN mantığı):
    #          los_azimut = iris_yaw + eps_yaw      → türevi = LOS hızı
    #          nişan = los + LEAD_SURE · los_hızı
    #      ⚠ Piksel hızı DEĞİL atalet LOS hızı kullanılır: yaw kontrolcüsü
    #      kutuyu merkeze çektiği için piksel hızı kendi düzeltmemizi de
    #      içerir; ona lead vermek düzeltmeyle kavga etmek olurdu.
    #   Düz kuyruk takibinde LOS hızı ≈ 0 → lead ≈ 0, yalnız kesişim kalır.
    # ── TERMİNAL HÜCUM HIZI (2026-08-08, kullanıcı kararı) ──
    # Yaklaşmada tavan 24 m/s KALIR (hedefe yetişmek için gerekli), yalnız
    # HÜCUM hızı 18'e düşer. Gerekçe: 24 m/s'de hedefin yanından 0.06 s'de
    # geçiyoruz — kamera 30 Hz'de son metrede 2 kare görüyor ve temas
    # penceresinden çok hızlı geçiliyor. 18 m/s'de kapanma 3.5 m/s (hedef
    # 14.5) → hem düzeltmeye daha çok kare, hem pencerede daha uzun süre.
    # Hedef 14.5 m/s olduğu için 18 hâlâ yeterli pay bırakır.
    V_TERMINAL = _env_f("AVCI_IBVS_VTERM", 18.0)   # m/s; hücum hızı
    # Dikey bütçe yetmediğinde yatay hız buraya kadar kısılabilir (bkz. komut()).
    V_TERM_MIN = _env_f("AVCI_IBVS_VTERM_MIN", 10.0)   # m/s; hücum hız tabanı

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

    # ══ LEAD ERKEN BAŞLASIN — M3 (2026-08-09) ══
    # Yatay lead `if terminal:` kapısının ARKASINDAYDI. terminal mandalı
    # TERMINAL_BOYUT=25 px ≈ 6.4 m'de kapanır, yani lead ancak son 6 metrede
    # devreye giriyordu. `lead_olcek` de o noktaya kadar zaten 1.0 (sönüm
    # yalnız 6.4 m'nin İÇİNDE başlar) — yani sönüm kusurlu değildi, KAPI
    # kusurluydu.
    #
    # ÖLÇÜLDÜ — 4473 kutulu kare, kendi daire koşularım (2026-08-09):
    #   menzil    |λ̇| med   V med   gereken yanal ivme   tavanı aşan   lead
    #   20-35 m   0.46      19.4     9.0 m/s²             %43           0.0°
    #   13-20 m   0.59      19.6    12.0                  %62           0.0°
    #    8-13 m   1.21      18.3    21.9                  %88           0.0°
    #     5-8 m   1.56      15.9    22.4                  %75           0.0°
    #     0-5 m   0.79      18.0    14.1                  %54           8.4°
    # Tavan = g·tan(ANGLE_MAX 45°) = 9.81 m/s². Gereken ivme = V·λ̇.
    #
    # OKUMASI: 8 m'ye gelindiğinde karelerin %88'i aracın FİZİKSEL olarak
    # üretemeyeceği bir dönüş istiyor — o noktada hiçbir nişan düzeltmesi
    # kurtarmaz. Düzeltmenin ucuz olduğu yer 13-35 m bandı (9-12 m/s²,
    # tavana yakın ama erişilebilir) ve orada lead TAM SIFIR.
    #
    # DEĞİŞİKLİK: yatay lead artık kutu olan HER karede uygulanır. Ölçek,
    # tavan ve LOS hızı kaynağı AYNEN aynı — tek değişen, kapının kalkması.
    # ⚠ KAPSAM: yalnız YATAY. Dikey lead (lead_el) terminal tutuşunda kalıyor;
    # kullanıcının düz uçuşta doğruladığı dikey davranış tek değişken
    # kuralının dışında tutuluyor.
    # DÜZ UÇUŞ RİSKİ DÜŞÜK: lead = LEAD_SURE · λ̇ ve düz takipte λ̇ ≈ 0
    # (ölçüldü: 20-35 m'de bile medyan 0.46 rad/s DÖNÜŞTE; düz koşuda ~0).
    #
    # ⛔ UÇUŞTA ÖLÇÜLDÜ (2026-08-09, 2 koşu / 2038 kutulu kare) — VARSAYILAN
    # KAPALI. Kapı kalkınca kadrajda tutuş gerçekten düzeldi:
    #     yatay hata p90   173.5 → 97.5 px      temas süresi  90 → 143 s
    #     yatay hata med    46.0 → 34.0 px      boyut son/ilk 0.97 → 1.07
    # AMA asıl iş olan YAKLAŞMA bozuldu:
    #     8 m içine giriş   4 kez / 65 kare  →  2 kez / 15 kare
    #     en yakın menzil   2.1 m (isabet)   →  13.2 / 10.0 m
    #     tavanı aşan kare  8-13 m'de %88    →  %95
    # SEBEP: lead karelerin %27'sinde LEAD_MAX_DEG=25° tavanında, medyan 18.7°.
    # Terminal için ayarlanmış tavan sürekli uygulanınca kalıcı nişan sapması
    # oluyor; araç kesişmek yerine hedefi GÖLGE ediyor (paralel koşu).
    # YÖN doğru, GENLİK yanlış. Sıradaki deney: seyir fazına AYRI (küçük)
    # lead tavanı — ~8-10° — terminal tavanı 25°'de kalsın.
    # AVCI_IBVS_LEAD_ERKEN=1 → ölçülen bu davranış geri gelir.
    LEAD_ERKEN = _env_f("AVCI_IBVS_LEAD_ERKEN", 0.0) >= 0.5
    VZ_MAX_TERM = _env_f("AVCI_IBVS_VZT", 5.0)   # m/s; terminalde dikey tavan

    # ── TERMİNAL DİKEY SÖNÜMLEME (2026-08-09, kullanıcı: "son anda üstten
    # geçtik") ──
    # SORUN: terminal dikey kanalı SAF NİŞANLAMA (vz = −v·tan(elev)) — türev/
    # sönümleme terimi YOK. Uzaktayken haklı olarak tırmanma emri veriliyor,
    # araç dikey momentum kazanıyor; hedefe varınca komut azalıyor ama momentum
    # geç sönüyor → hedefin ÜSTÜNDEN geçiliyor.
    # Kullanıcının manuel uçuş kaydından ölçüldü (log 081132, son kareler):
    #     hedef TAM nişanda (dikey hata −2.2°) iken vz komutu −4.2 m/s
    #     ardından kutu kadrajda 294 → 456 px kayıyor = üstünden geçildi
    # ⚠ Lead DEĞİLDİ: aynı karelerde lead 0.09-0.15 s'ye sönmüş ve AŞAĞI
    # yönlüydü (−3° … −13°). Lead sönümü çalışıyor, sebep bu değil.
    #
    # ÇÖZÜM: aracın KENDİ dikey hızıyla türev sönümlemesi.
    #     vz = vz_nişan + K_VZ_D · (vz_nişan − vz_gerçek)
    # Zaten gerekenden hızlı tırmanıyorsak komut azalır/ters döner.
    # Girdi drone'un KENDİ sensörü — yarışma kuralı serbest.
    K_VZ_D = _env_f("AVCI_IBVS_KVZD", 0.6)   # dikey sönümleme kazancı

    # ══ DİKEY KOMUT KAPANMA HIZIYLA ÖLÇEKLENİR (2026-08-09) ══
    # KULLANICI GÖZLEMİ (uçuş kaydı): "tam vuracağı sırada yukarı manevra
    # yapıp aracın üstünden geçiyoruz."
    #
    # KÖK NEDEN — tek bir çarpan. Terminal dikey yasası şuydu:
    #     vz = −v_los · tan(yükseliş)          v_los = DRONE'un hızı (18 m/s)
    # Oysa dikey farkı "varana kadar" kapatmak gerekir; "varana kadar"ki süreyi
    # belirleyen şey KAPANMA hızıdır, drone'un yer hızı değil. Hedef 15 m/s ile
    # kaçtığı için mesafe saniyede 18 m değil ~2 m kapanıyor. Doğrusu:
    #     vz = −ṙ · tan(yükseliş)              ṙ = kapanma hızı
    #
    # ÖLÇÜLDÜ (kullanıcının 4 hücumu, üçünde de aynı):
    #     menzil 3.67 m, dikey fark 0.89 m altta
    #     komut −5.00 m/s   ·   gereken −0.37 m/s   →  13.7 KAT fazla
    # Araç yukarı ivmeleniyor, komut sonra tersine dönüyor ama momentum
    # kalıyor → hedefin üstünden geçiliyor. Gün boyu kovaladığım dikey
    # salınımın açıklaması da bu: mimari değil, çarpan.
    #
    # ṙ GÖRÜNTÜDEN ölçülür (GPS YOK, yarışma kuralı temiz):
    #     R = MENZIL_PX_M / boyut   ⇒   ṙ = −dR/dt = R · (dboyut/dt) / boyut
    # Kutu boyutu titrer → EMA ile yumuşatılır; taban konur ki kapanma
    # durduğunda dikey düzeltme büsbütün ölmesin.
    # AVCI_IBVS_KAPANMA=0 → eski davranış (v_los ile ölçekleme) aynen geri.
    KAPANMA = _env_f("AVCI_IBVS_KAPANMA", 1.0) >= 0.5
    KAPANMA_MIN = _env_f("AVCI_IBVS_KAPANMA_MIN", 1.5)   # m/s; ölçek tabanı
    KAPANMA_EMA = _env_f("AVCI_IBVS_KAPANMA_EMA", 0.20)  # kare başına yumuşatma
    # Kutu boyutu → menzil ölçeği: TERMINAL_BOYUT 25 px ≈ 6.4 m (Cfg yorumu)
    MENZIL_PX_M = 160.0                                  # px·m
    MAX_ACCEL = 12.0               # m/s²; komut hızı değişim sınırı

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
    # ⚠ KAPSAM: yalnız SEYİR (IBVS). Terminal hücum yasası (v=V_TERMINAL)
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
    DONUS_A = _env_f("AVCI_IBVS_DONUS", 0.0)     # m/s²; 0 = kapalı, açık ~9.0
    DONUS_V_MIN = _env_f("AVCI_IBVS_DONUS_VMIN", 10.0)   # m/s; hız tabanı

    # ══ T1b · DİKEY KANALDA ROLL/PITCH TELAFİSİ ══
    # NEDEN ŞİMDİ (2026-08-11 gece ölçümü): kesişim artık 10-40 cm'ye kadar
    # çözülüyor. İki uzun kayıtlı koşunun temas anı bileşenlerine ayrıldı:
    #     R01  yatay 0.33 m   dikey +0.05 m  → İSABET
    #     R02  yatay 0.12 m   dikey −0.11 m  → ıska (zarf sınırında)
    # İsabet zarfı yatayda ±0.65 m ama DİKEYDE +0.29 / −0.13 m — 5 KAT DAR.
    # Yani isabetle ıska arasındaki fark artık SANTİMETRE ve DİKEY eksende.
    #
    # T1a (yatay telafi) uygulanıp uçuşta doğrulandı; DİKEY, tek-değişken
    # kuralı gereği bilerek dokunulmadan bırakılmıştı. Ölçülen okuma hatası
    # dikeyde YATAYDAKİNDEN BÜYÜK: kullanıcının uçuşunda (log 091554) araç
    # 30° yatıktayken ham dikey okuma −22.1° derken telafili okuma +4.8°
    # diyordu — İŞARET TERS, en büyük sapma 33.1°.
    #
    # ÇÖZÜM: eps_elev, ham piksel farkı yerine los_seviye()'nin SEVİYE
    # çerçevesindeki yükseliş çıktısından kurulur. Nişan noktası da aynı
    # çerçeveye taşınır (CY_NISAN'ın seviye karşılığı), böylece hata tanımı
    # değişmez — yalnız okuma düzelir.
    # ⚠ DÜZ UÇUŞTA ETKİSİZ: roll=pitch=0'da los_seviye = piksel_elev, fark 0.
    # AVCI_IBVS_DIKEY_ROLL=0 → eski (telafisiz) dikey yol aynen geri gelir.
    DIKEY_ROLL = _env_f("AVCI_IBVS_DIKEY_ROLL", 0.0) >= 0.5

    # ══ Ö11 · ISKA SONRASI DÖNÜŞ İÇİN YAVAŞLAMA ══
    # ÖLÇÜLDÜ (2026-08-12, S01-S10): "sağa aşım" bir KONTROL SALINIMI DEĞİL.
    # Aşım BEŞ koşuda da tetikten TAM +7 s sonra oluyor ve 66-69 m:
    #     R = V²/(g·tan45°) = 18²/9.81 = 33 m  →  U-dönüşü 2R = 66 m
    # Yani drone hedefi geçiyor ve geri dönmek için MİNİMUM ÇEMBERİNİ çiziyor.
    # Ö5/Ö8/Ö9 (kazanç ve nişan ayarları) bu yüzden işe yaramadı — sınır
    # fiziksel, ayar değil.
    #
    # ÇÖZÜM: yarıçap hızın KARESİYLE düşer.
    #     18 m/s → 2R = 66 m      12 m/s → 29 m      9 m/s → 17 m
    # Geçişten SONRA, dönüşü tamamlayana kadar hız kısılır.
    #
    # TETİK (yalnız kutudan — CANLI GPS YOK, D0 temiz):
    #   kapanma < −DONUS_YAVAS_RDOT  → kutu hızla küçülüyor = hedefi GEÇTİK
    #   |eps_yaw| > DONUS_YAVAS_ACI  → daha çok dönmemiz gerekiyor
    # Koşul DURUM TUTMAZ: dönüş ilerledikçe eps_yaw küçülür ve kendiliğinden
    # serbest bırakır. Hedefe yeniden nişan alınca hız geri gelir.
    # ⚠ Ö5'ten farkı: Ö5 λ̇'ya bakıyordu ve geçiş ANINDA bağlamıyordu;
    # bu doğrudan "geçtik, şimdi dön" durumunu hedefler.
    # ⚠ DÜZ TAKİPTE ETKİSİZ: yaklaşırken kapanma > 0, koşul hiç kurulmaz.
    # AVCI_IBVS_DONUS_YAVAS=0 → kapalı (varsayılan).
    DONUS_YAVAS = _env_f("AVCI_IBVS_DONUS_YAVAS", 0.0)   # m/s; açık ~9.0
    DONUS_YAVAS_RDOT = _env_f("AVCI_IBVS_DY_RDOT", 5.0)  # m/s; uzaklaşma eşiği
    DONUS_YAVAS_ACI = _env_f("AVCI_IBVS_DY_ACI", 45.0)   # °; dönüş gereği eşiği

    # ── KUTU GEÇERLİLİĞİ ──
    CONF_MIN = _env_f("AVCI_IBVS_CONF", 0.35)   # bunun altı kutu = yok sayılır
    BOYUT_MIN = 6.0                # px; bundan küçük kutu güvenilmez (gürültü)


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

_CSV_ALANLAR = [
    "t", "dt", "durum", "cx", "cy", "w", "h", "boyut", "conf",
    "eps_yaw_deg", "eps_yaw_ham_deg", "eps_elev_deg", "eps_elev_ham_deg",
    "iris_roll_deg", "iris_pitch_deg", "iris_yaw_deg",
    "boyut_hata", "hiz_I", "v_los", "kacis_ek", "gecikme_s", "eps_hiz_deg", "sonum_deg", "donus_tavan", "donus_yavas", "lead_az_deg", "los_hiz_az", "los_hiz_el",
    "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg", "kayip_sayac",
]


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


def komut(cx, cy, w, h, iris_yaw, hiz_I, dt, cfg=Cfg, terminal=False,
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
    boyut = math.sqrt(max(w, 0.0) * max(h, 0.0))

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
        lead_olcek = clamp(cfg.BOYUT_REF / boyut, 0.0, 1.0)
    lead_sure = cfg.LEAD_SURE * lead_olcek
    lead_az = 0.0
    # LEAD: nişanı atalet LOS dönüş hızıyla öne al (bkz. Cfg.LEAD_SURE).
    # M3: kapı kalktı — artık kutu olan her karede (bkz. Cfg.LEAD_ERKEN).
    if terminal or cfg.LEAD_ERKEN:
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
    hata = cfg.BOYUT_REF - boyut               # px; + = uzak
    hiz_I = clamp(hiz_I + cfg.K_I * hata * dt, cfg.I_MIN, cfg.I_MAX)
    kacis_ek = 0.0
    if terminal:
        v_los = cfg.V_TERMINAL                 # hücum: fren yok, sabit hız
    else:
        # Ö1 KAÇIŞ TELAFİSİ (bkz. Cfg.KACIS_KD): hedef uzaklaşıyorsa (ṙ<0)
        # hızı ANINDA artır — integralin 5 saniyesini bekleme.
        # ⚠ YALNIZ hızlandırma yönü: ṙ>0 iken (yaklaşırken) terim SIFIR.
        if cfg.KACIS_KD > 0.0 and kapanma is not None and kapanma < 0.0:
            kacis_ek = min(cfg.KACIS_KD * (-kapanma), cfg.KACIS_MAX)
        v_los = clamp(hiz_I + cfg.K_FWD * hata + kacis_ek,
                      cfg.V_MIN, cfg.V_TOPLAM_MAX)

    # Ö11 ISKA SONRASI YAVAŞLAMA (bkz. Cfg.DONUS_YAVAS): hedefi geçtik ve
    # geri dönmemiz gerekiyorsa hızı kıs — dönüş çemberi V² ile daralır.
    donus_yavas = False
    if (cfg.DONUS_YAVAS > 0.0 and kapanma is not None
            and kapanma < -cfg.DONUS_YAVAS_RDOT
            and abs(eps_yaw) > math.radians(cfg.DONUS_YAVAS_ACI)
            and cfg.DONUS_YAVAS < v_los):
        v_los = cfg.DONUS_YAVAS
        donus_yavas = True

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
        _R = cfg.MENZIL_PX_M / boyut                  # menzil (m)
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

    # T1b (bkz. Cfg.DIKEY_ROLL): ham piksel farkı KAMERA çerçevesindedir;
    # araç yattığında bu SEVİYE çerçevesindeki yükseliş DEĞİLDİR.
    eps_elev_ham = math.atan((cy - cfg.CY_NISAN) / geo.FY)  # cy büyük → altta
    eps_elev = eps_elev_ham
    if cfg.DIKEY_ROLL:
        # ⚠ TELAFİ, FARK OLARAK uygulanır — hata TANIMI değişmez.
        # Birim testi B58 şunu yakaladı: seviye yükselişini doğrudan hata
        # yerine koymak, roll=pitch=0'da BİLE komutu 0.51 m/s değiştiriyordu
        # (25° tilt yüzünden piksel farkı ile açı farkı aynı fonksiyon değil).
        # Doğrusu: duruşun getirdiği SAPMAYI çıkarmak.
        # ⚠ YALNIZ ROLL izole edilir; pitch İKİ terimde de aynı bırakılır.
        # Sebep: nişan noktası CY_NISAN, aracın seyir pitch'i (18 m/s'de
        # burun ~28° aşağı) ile BİRLİKTE uçuşta ayarlanmıştı. Pitch'i de
        # telafi etmek nişan noktasını kaydırır — bu ayrı bir değişkendir,
        # bu adımın konusu değil. (İlk sürüm pitch'i de içeriyordu ve
        # terminalde +5.9° kayma üretiyordu; tek-değişken kuralına aykırı.)
        _, _el_roll = los_seviye(cx, cy, iris_roll, iris_pitch, cfg)
        _, _el_norm = los_seviye(cx, cy, 0.0, iris_pitch, cfg)
        # el_roll > el_norm ⇒ hedef sandığımızdan YUKARIDA ⇒ daha çok tırman
        eps_elev = eps_elev_ham - (_el_roll - _el_norm)
    if terminal:
        # KESİŞİM: hız vektörü hedefe DOĞRU baksın (tutuş ofseti değil).
        # elev_atalet = gövde LOS yükselişi + gövde pitch; lead ile öne alınır.
        elev_atalet = piksel_elev(cy, cfg) + iris_pitch
        lead_el = clamp(lead_sure * los_hiz[1],
                        -math.radians(cfg.LEAD_MAX_DEG),
                        math.radians(cfg.LEAD_MAX_DEG))
        nisan_elev = clamp(elev_atalet + lead_el,
                           -math.radians(60.0), math.radians(60.0))

        # ── DİKEY BÜTÇE KISITI (2026-08-09, kullanıcı gözlemi: "dikeyde çok
        # kaçırıyor") ──
        # Hız vektörünün gösterebileceği en dik açı atan(VZ_MAX_TERM/v_los).
        # 18 ve 5 ile bu YALNIZCA 15.5° — hedef daha yukarıdaysa kesişim
        # MATEMATİKSEL OLARAK İMKÂNSIZ, drone altından geçer. Ölçüldü: terminal
        # karelerinin %22-49'unda vz tavana dayanmıştı (yani "daha çok
        # tırmanmam lazım" deyip yapamıyordu).
        # ÇÖZÜM: dikey tavan yetmiyorsa YATAYI KIS — böylece vektör hedefe
        # bakabilir. Yavaşlamak yaklaşmayı geciktirir ama ıskalamaktan iyidir;
        # V_TERM_MIN altına inilmez (hedefi büsbütün kaçırmamak için).
        # AÇIYI DİKEY HIZA ÇEVİREN ÖLÇEK (bkz. Cfg.KAPANMA): kapanma hızı.
        # ⚠ Dikey bütçe kısıtı da AYNI ölçeği kullanmalı — yoksa yatayı,
        # artık var olmayan bir dikey talep yüzünden kısar (yani boşuna
        # frene basar). İki yer tek kavram.
        v_dikey = v_los
        if cfg.KAPANMA and kapanma is not None:
            v_dikey = clamp(kapanma, cfg.KAPANMA_MIN, max(cfg.KAPANMA_MIN, v_los))
        t_ = abs(math.tan(nisan_elev))
        if t_ > 1e-6 and v_dikey * t_ > cfg.VZ_MAX_TERM:
            v_los = max(cfg.V_TERM_MIN, cfg.VZ_MAX_TERM / t_)
            vx_ned = v_los * math.cos(hiz_yonu)
            vy_ned = v_los * math.sin(hiz_yonu)
        vz_nisan = -v_dikey * math.tan(nisan_elev)
        # TÜREV SÖNÜMLEMESİ: aracın kendi dikey hızı nişanın ötesine geçtiyse
        # komut geri çekilir → hedefin üstünden geçme biter (bkz. Cfg.K_VZ_D).
        vz = clamp(vz_nisan + cfg.K_VZ_D * (vz_nisan - iris_vz),
                   -cfg.VZ_MAX_TERM, cfg.VZ_MAX_TERM)
    else:
        # TUTUŞ (değişmedi): hedefi CY_NISAN'da tut
        vz = clamp(cfg.K_VZ * cfg.V_NOM * eps_elev, -cfg.VZ_MAX, cfg.VZ_MAX)

    tani = {"boyut": boyut, "eps_yaw": eps_yaw, "eps_elev": eps_elev,
            "eps_elev_ham": eps_elev_ham,
            "hata": hata, "v_los": v_los, "terminal": terminal,
            "eps_hiz": eps_hiz, "sonum": sonum,
            "donus_tavan": donus_tavan, "donus_yavas": donus_yavas,
            "kacis_ek": kacis_ek,
            "lead_az": lead_az, "lead_olcek": lead_olcek,
            "eps_yaw_ham": eps_yaw_ham}
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
    terminal_mandal = False   # terminal hücum kilidi (bir kez girilince kalır)
    kor_baslangic = None      # kör hücumun başladığı duvar anı (süre sınırı)
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
          f"{hiz_I:.1f} m/s, REF={cfg.BOYUT_REF:.0f}px, tavan "
          f"{cfg.V_TOPLAM_MAX:.0f} m/s, terminal hücum >{cfg.TERMINAL_BOYUT:.0f}px, "
          f"CY_nişan={cfg.CY_NISAN:.0f}, kayıp eşiği={kayip_kare_esik} kare, "
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
                send_velocity(conn, 0.0, 0.0, 0.0, iyaw)
                vx_p = vy_p = vz_p = 0.0
                son_v_cmd = None
                cmd_yaw = iyaw
                kayip_sayac += 1
                if kayip_sayac >= kayip_kare_esik:
                    print("[IBVS] kurtarma sırasında temas koptu → 'kayip'")
                    return "kayip"
                w_csv.writerow({"t": round(now, 3), "dt": round(dt, 4),
                                "durum": "KURTARMA",
                                "kayip_sayac": kayip_sayac,
                                "iris_yaw_deg": round(math.degrees(iyaw), 1)})
                f.flush()
                continue

            kutu = _kutu_gecerli(kayit["pose"], cfg)
            if kutu is None:
                kayip_sayac += 1
                # TERMİNAL: kör hücum — kutu kaybolsa da son komutla devam,
                # AMA SÜRE SINIRLI. Terminalde hedef kadrajdan çıkması NORMAL
                # (çok yakın); GPS'e hemen dönmek çarpışmayı iptal eder. Süre
                # dolarsa ıska sayılır — sınırsız bırakmak aracı kaçırıyor.
                if terminal_mandal:
                    if kor_baslangic is None:
                        kor_baslangic = time.time()
                        print(f"[IBVS] kör hücum başladı — {cfg.TERMINAL_SURE:.1f} s "
                              f"içinde temas gelmezse ıska")
                    gecen = time.time() - kor_baslangic
                    if gecen >= cfg.TERMINAL_SURE:
                        print(f"[IBVS] kör hücum {gecen:.1f} s sürdü, temas yok "
                              f"→ ISKA, 'kayip' (GPS'e dönülüyor)")
                        return "kayip"
                    if son_v_cmd is not None:
                        send_velocity(conn, *son_v_cmd)
                    w_csv.writerow({"t": round(now, 3), "dt": round(dt, 4),
                                    "durum": "TERM_KOR",
                                    "kayip_sayac": kayip_sayac,
                                    "iris_yaw_deg": round(math.degrees(iyaw), 1)})
                    f.flush()
                    continue
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
            kor_baslangic = None       # kutu geri geldi → kör sayaç sıfırlanır
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
            boyut_simdi = math.sqrt(bw * bh)
            if (boyut_onceki is not None and boyut_simdi > 1e-6
                    and 1e-3 < dt < 0.5):
                _R = cfg.MENZIL_PX_M / boyut_simdi
                _rdot = _R * ((boyut_simdi - boyut_onceki) / dt) / boyut_simdi
                _rdot = clamp(_rdot, -30.0, 30.0)      # gürültü kalkanı
                kapanma = (_rdot if kapanma is None else
                           cfg.KAPANMA_EMA * _rdot
                           + (1.0 - cfg.KAPANMA_EMA) * kapanma)
            boyut_onceki = boyut_simdi
            # TERMİNAL MANDALI: kutu eşiği aşınca hücuma taahhüt, geri dönüş yok
            if not terminal_mandal and math.sqrt(bw * bh) >= cfg.TERMINAL_BOYUT:
                terminal_mandal = True
                print(f"[IBVS] ⚡ TERMİNAL HÜCUM (kutu {math.sqrt(bw*bh):.0f}px "
                      f"≥ {cfg.TERMINAL_BOYUT:.0f}) — fren yok, tam taahhüt")
            vx, vy, vz, yaw_hedef, hiz_I, tani = komut(cx, cy, bw, bh, iyaw,
                                                       hiz_I, dt, cfg,
                                                       terminal_mandal,
                                                       tuple(los_hiz), ipitch,
                                                       float(iris.get("vz", 0.0) or 0.0),
                                                       kapanma, iroll,
                                                       yaw_hizi)
            # ── YAW SLEW SINIRI (bkz. Cfg.YAW_RATE_MAX) ──
            # HIZ (vx, vy) yaw_hedef'ten hesaplandı ve DEĞİŞMEZ: nişan hedefin
            # gerçek yönünde kalır. Sınırlanan yalnız BURUNUN dönme hızı.
            if cmd_yaw is None:
                cmd_yaw = iyaw
            yaw_err = normalize_angle(yaw_hedef - cmd_yaw)
            adim = clamp(yaw_err, -math.radians(cfg.YAW_RATE_MAX_DEG) * dt,
                         math.radians(cfg.YAW_RATE_MAX_DEG) * dt)
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
                "durum": "TERMINAL" if terminal_mandal else "IBVS",
                "cx": round(cx, 1), "cy": round(cy, 1),
                "w": round(bw, 1), "h": round(bh, 1),
                "boyut": round(tani["boyut"], 1), "conf": round(conf, 3),
                "eps_yaw_deg": round(math.degrees(tani["eps_yaw"]), 1),
                # ÖLÇÜM SÜTUNU: telafisiz okuma. Farkı (ham − telafili) roll'e
                # karşı çizince T1a'nın uçuşta ne kadar bağladığı doğrudan
                # görülür. Yalnız log — güdüm bunu kullanmaz.
                "eps_yaw_ham_deg": round(math.degrees(tani["eps_yaw_ham"]), 1),
                "eps_elev_deg": round(math.degrees(tani["eps_elev"]), 1),
                "eps_elev_ham_deg": round(math.degrees(tani["eps_elev_ham"]), 1),
                "iris_roll_deg": round(math.degrees(iroll), 1),
                "iris_pitch_deg": round(math.degrees(ipitch), 1),
                "iris_yaw_deg": round(math.degrees(iyaw), 1),
                "boyut_hata": round(tani["hata"], 1),
                "hiz_I": round(hiz_I, 2), "v_los": round(tani["v_los"], 2),
                "kacis_ek": round(tani["kacis_ek"], 2),
                "gecikme_s": (round(gecikme_s, 4)
                              if gecikme_s is not None else ""),
                "eps_hiz_deg": round(math.degrees(tani["eps_hiz"]), 1),
                "sonum_deg": round(math.degrees(tani["sonum"]), 2),
                "donus_tavan": ("" if tani["donus_tavan"] is None
                                else round(tani["donus_tavan"], 2)),
                "donus_yavas": int(tani["donus_yavas"]),
                "lead_az_deg": round(math.degrees(tani["lead_az"]), 2),
                "los_hiz_az": round(los_hiz[0], 3), "los_hiz_el": round(los_hiz[1], 3),
                "vx_cmd": round(vx, 2), "vy_cmd": round(vy, 2),
                "vz_cmd": round(vz, 2),
                "yaw_cmd_deg": round(math.degrees(yaw_cmd), 1),
                "kayip_sayac": 0,
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
