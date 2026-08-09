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
    YAW_RATE_MAX = math.radians(_env_f("AVCI_IBVS_YAWRATE", 120.0))  # rad/s

    # ── DİKEY ──
    # eps_elev = atan((cy − CY_NISAN)/FY); v_z = K_VZ · V_NOM · eps_elev.
    # cy büyük (hedef kadrajda AŞAĞIDA) → hedef boresight'ın altında → ALÇAL
    # (vz>0, NED down+). Nominal hızla ölçekli: hızlı giderken dik açı daha çok
    # dikey hız ister (irtifayı korumak için).
    # K_VZ 1.2 → 0.5, VZ_MAX 4 → 3 (2026-08-08): ilk sürüm dikey hızı çok
    # agresifti (10° hata → 2.5 m/s) ve tavana yapışıp salındı. Nişan doğru
    # yere gelince (≈300) hata küçük kalır; yumuşak kazanç yeter.
    K_VZ = _env_f("AVCI_IBVS_KVZ", 0.5)
    # Tutuş fazının dikey tavanı. ⚠ Aracın ölçülen dikey ivme sınırı 2.5 m/s²
    # (WPNAV_ACCEL_Z=250) — 3 m/s'lik bir komutu uygulaması 1.2 s sürüyor.
    # Terminal tavanıyla (VZ_MAX_TERM) birlikte ayarlanmalı.
    # ⚠ 3.0 → 2.5 (2026-08-09): aracın ÖLÇÜLEN dikey ivme sınırı 2.5 m/s²
    # (WPNAV_ACCEL_Z=250). 3 m/s'lik komutu uygulaması 1.2 s sürüyor ve komut
    # sürekli doygun kalıyordu. Tavanı aracın yapabildiğine indirmek, her
    # salınımın GENLİĞİNİ küçültüyor — kazanan üçlünün parçası.
    VZ_MAX = _env_f("AVCI_IBVS_VZMAX", 2.5)   # m/s; dikey hız tavanı
    V_NOM = 12.0                   # m/s; dikey ölçekleme için nominal ileri hız

    # ══ İRTİFA EŞİTLE, SONRA DAL (2026-08-09, KULLANICI FİKRİ) ══
    #
    # KULLANICI GÖZLEMİ + KANIT (kayıt ucus_20260809_092315): yaklaşmada dikey
    # fark (hedef − drone) bir LİMİT ÇEVRİMİ yapıyordu, genliği büyüyerek:
    #   −1.4 → −0.5 → +1.5 → +1.4 → −0.6 → −1.9 → −0.5 → +2.8 → +6.4 → +7.5
    # En yakın anda (1.50 m) drone hedefin 0.5 m ÜSTÜNDEYDİ → ıska.
    # Kullanıcı: "irtifayı eşitlemeden dalışa geçince hep altından yaklaşıp
    # bir anda üstünden geçiyor". Doğru teşhis: terminal hem yatayı kapatmaya
    # hem dikeyi düzeltmeye çalışınca dikey döngü salınıyor.
    #
    # ÇÖZÜM (kullanıcının önerdiği mekanizma): önce İRTİFAYI EŞİTLE, sonra dal.
    #   1) Tutuş fazının dikey hedefi artık SABİT PİKSEL değil, ATALET
    #      YÜKSELİŞİ: elev = piksel_elev(cy) + gövde_pitch (jiroskop).
    #      Yani drone'un kendi eğimi hesaba katılır — kullanıcının dediği gibi
    #      "kaç derece eğildiği jiroskoptan çekilebilir".
    #   2) Terminal hücum, yükseliş oturmadan AÇILMAZ (aşağıki eşik).
    # Böylece terminal saf yatay bir koşuya dönüşür; dikey iş bitmiş olur.
    #
    # Menzil bilgisine GEREK YOK: açıyı sıfırlamak irtifayı eşitlemekle aynı
    # şeydir (her menzilde geçerli) — kullanıcının "oranı sıfırlamak" dediği.
    #
    # ELEV_HEDEF neden tam 0 değil: 0° hedefi tam ufuk çizgisine oturtur,
    # orası tespit için en kötü zemin (kontrast düşük). 2°'de hedef ufkun az
    # üstünde kalır (gökyüzü zemini korunur) ama 6 m'de yalnız 0.2 m fark eder
    # — pratikte "aynı irtifa".
    ELEV_HEDEF_DEG = _env_f("AVCI_IBVS_ELEV_HEDEF", 2.0)   # °; tutuşta yükseliş

    # ── DİKEY İNTEGRAL (2026-08-09) ──
    # ⚠ NEDEN: irtifa eşitleyici SADECE ORANTILI idi ve yakınsamıyordu —
    # ölçüldü: yükseliş hatası tutuş fazında 8-9° → 8-12° (kapanmıyor, bazen
    # büyüyor). Orantılı kontrol kalıcı hatayı kapatamaz; burada kalıcı hatanın
    # kaynağı hedefin SÜREKLİ TIRMANIŞI (~0.1-0.3 m/s) ve drone'un kendi eğim
    # değişimi. İntegral bu sabit "yükü" öğrenir ve hatayı sıfıra çeker —
    # yatay hızda (hiz_I) zaten işe yarayan çözümün dikey karşılığı.
    # Terminal sırasında DONDURULUR (orada farklı yasa çalışıyor, windup olmasın).
    K_ELEV_I = _env_f("AVCI_IBVS_KELEVI", 4.0)   # (m/s)/(rad·s)
    ELEV_I_MAX = 3.0                             # m/s; integral penceresi

    # ══ PN — ORANTILI SEYRÜSEFER (2026-08-09) ══
    #
    # NEDEN: bugüne kadarki bütün terminal yasaları AÇIYI kontrol ediyordu
    # ("hız vektörünü hedefe doğrult"). Ama çarpışmanın şartı açının belli bir
    # DEĞERDE olması değil, açının SABİT KALMASI (denizcilik kuralı: çarpışma
    # rotasındaki gemi ufuktaki yerini değiştirmez). Kontrol ettiğimiz büyüklük
    # ile başarı ölçütümüz (kutu kayması) farklı şeylerdi; her yamada bir
    # yerden bastırıp başka yerden kaçıyordu:
    #     lead sabit → yukarı savuruyor        (menzille söndürüldü)
    #     nişanlama → dikey momentum salınımı  (türev sönümlemesi eklendi)
    #     irtifa eşitleme → yakınsamıyor       (integral eklendi, kısmi)
    # Hiçbiri tekrarlanabilir üstünlük vermedi.
    #
    # PN bunun yerine doğrudan LOS DÖNÜŞÜNÜ sıfırlar — yani ölçütümüzle AYNI
    # büyüklüğü hedefler. Güdümlü füzelerde 1950'lerden beri standart.
    # Hız formu: hız vektörünün yönü, LOS dönüş hızının N katıyla döner.
    #     χ̇ = N · λ̇        (χ: hız vektörü yönü, λ: LOS yönü)
    # Doğası gereği sönümlüdür: λ̇ küçüldükçe komut da küçülür; nişanlama
    # yasasının aksine "hedefe varınca fren" sorunu yoktur.
    #
    # PN_BETA: saf PN yavaşça sürüklenebilir (λ̇ ölçümü gürültülü). Küçük bir
    # oranla LOS'a geri çekilir — klasik "PN + pursuit bias".
    # ⚠ VARSAYILAN KAPALI (2026-08-09 ölçümü): PN 3 uçuşta 0/3 vurdu ve
    # kaçırmanın asıl sebebi olan dikey salınımı düzeltmedi (yükseliş
    # sürüklenmesi −40°, kontrolle aynı). Ölçüm desteklemediği için varsayılan
    # olmaya hak kazanmadı; AVCI_IBVS_PN=1 ile açılır. Bkz. dikey döngü teşhisi.
    PN = _env_f("AVCI_IBVS_PN", 0.0) >= 0.5      # 1 = orantılı seyrüsefer
    PN_N = _env_f("AVCI_IBVS_PN_N", 3.0)         # seyrüsefer sabiti (klasik 3-5)
    PN_BETA = _env_f("AVCI_IBVS_PN_BETA", 0.10)  # LOS'a geri çekme oranı
    PN_HOLD = _env_f("AVCI_IBVS_PN_HOLD", 0.0) >= 0.5   # tutuş fazında da PN
    # BURUN nereye baksın? Multirotorda hız komutu NED'dir, yaw'dan bağımsız —
    # yani kamerayı hedefe kilitleyip hız vektörünü BAŞKA yöne sürebiliriz.
    #   1 = burun DAİMA hedefte (kutu kadraj merkezinde kalır; görsel temas
    #       yarışma kuralının can damarı olduğu için varsayılan bu)
    #   0 = burun hız vektöründe (eski davranış; hedef kadrajın kenarına kayar)
    PN_YAW_LOS = _env_f("AVCI_IBVS_PN_YAWLOS", 1.0) >= 0.5

    # ══ ALTTAN YAKLAŞMA EĞİLİMİ (2026-08-09) ══
    # ÖLÇÜM (3 kontrol uçuşu, üçünde de aynı): terminalde hedef kadrajın
    # ALTINDAN çıkıyor (cy 316 → 461, kadraj yüksekliği 480) ve dedektör onu
    # kaybediyor; ardından 2 s kör hücum, sonra hedef 58° yandan bulunuyor.
    # GEOMETRİ: kamera gövdeye 25° YUKARI bakıyor. Dikey görüş sınırları:
    #     yukarı  25° + atan(240/166.6) = +80°
    #     aşağı   25° − atan(240/166.6) = −30°
    # Yani hedefin ÜSTÜNE çıktığımız an kör kalıyoruz; altında kalırsak 80°
    # payımız var. Bu eğilim, nişanı LOS'un biraz ALTINA alarak drone'u
    # hedefin altında tutar — hem görsel temas korunur (yarışma kuralının can
    # damarı) hem de sistematik "üstünden geçme" hatası ters yöne çekilir.
    # Maliyeti küçük: 2-6 m menzilde 5° = 0.17-0.52 m. Hedef gövdesi + drone
    # yarıçapı bunu yutar.  0 = kapalı (varsayılan, davranış değişmez).
    TERM_ELEV_BIAS = math.radians(_env_f("AVCI_IBVS_TERM_BIAS", 0.0))

    # ══ TERMİNAL KAPISININ BOŞ GEÇMESİ (2026-08-09, kusur) ══
    # Kapı iki şart ister: kutu ≥ TERMINAL_BOYUT VE yükseliş hatası ≤ eşik.
    # Ama ikinci şart, döngü başında 0.0 ile başlatılan bir değişkene bakıyor.
    # Görsel faz UZAKTAN başlarsa (kutu ~10 px) sorun çıkmıyor. Ancak ıskadan
    # sonra supervisor GPS'e dönüp YAKINDAN yeniden devrediyor (ölçüldü: bir
    # koşuda 5 görsel faz); o girişte kutu zaten 30-50 px olduğu için kapı
    # SAHTE 0° hatayla ilk karede açılıyor — tam da engellemesi gereken durum.
    # 1 = kapı gerçek ölçüm gelene dek KAPALI kalır (doğrusu bu).
    # Varsayılan 0: faz-2 kampanyasının tek-değişken temeli bozulmasın diye;
    # ölçülüp kazanırsa varsayılan çevrilecek.
    # ⚠ VARSAYILAN AÇIK (kazanan üçlünün parçası). Kapı artık gerçek ölçüm
    # gelene dek kapalı kalır; sahte 0° ile açılmaz.
    KAPI_KATI = _env_f("AVCI_IBVS_KAPI_KATI", 1.0) >= 0.5

    # ══ İNTEGRAL DOYMA KORUMASI (anti-windup) — 2026-08-09 ══
    # ÖLÇÜLDÜ (log 112517, tutuş fazı): elev_I ilk yarım saniyede tavana (3.0)
    # yapıştı ve 6 SANİYE orada kaldı; vz komutu −3.0 rayında durdu, aracın
    # gerçek dikey hızı ise ancak 5. saniyede −3.0'a ulaştı. Hata sıfırı
    # geçtiğinde integral hâlâ doluydu → yükseliş +0.6°'den −25.6°'ye savruldu
    # ve terminal tam bu savrulmanın ortasında mandalladı.
    # Klasik integral doyması: çıkış doymuşken integral almaya devam etmek,
    # hatanın işareti dönene kadar boşalamayan bir yük biriktirir.
    # 1 = çıkış doymuşken ve hata doymayı DERİNLEŞTİRİYORKEN integral durur.
    AWU = _env_f("AVCI_IBVS_AWU", 0.0) >= 0.5

    # ══ TERMİNAL MANDALININ BIRAKILMASI (2026-08-09) ══
    # Mandal bir kez kapanınca bir daha açılmıyor. ÖLÇÜLDÜ: terminal fazı
    # 9.6 / 17.3 / 37.3 saniye sürdü — oysa hücumun kendisi ~1-2 saniye.
    # Sebep: ıskadan sonra menzil 10-16 m'ye AÇILIYOR ama araç hâlâ terminal
    # yasasında — tam gaz, nişan LOS'ta, hız PI'si ve irtifa eşitleyici DEVRE
    # DIŞI. O menzilde doğru yasa tutuş yasasıdır.
    # Bu oran, kutu bu eşiğin altına düşerse mandalı BIRAKIR; araç tutuşa
    # döner, irtifayı yeniden eşitler, sonra yeniden mandallar.
    # 0 = kapalı (eski davranış). 0.6 → 15 px ≈ 10.7 m'de bırakır.
    TERM_BIRAK = _env_f("AVCI_IBVS_TERM_BIRAK", 0.0)

    # ══ TERMİNALDE DİKEYİ DONDURMA (2026-08-09) ══
    # Aracın dikey tepkisi ~1.5-2 s gecikmeli; terminal ise ~1-2 s sürüyor.
    # Yani terminal SÜRESİNCE dikey döngü yakınsayamaz — matematiksel olarak
    # mümkün değil. Her düzeltme denemesi, ancak hücum bittikten sonra etki
    # eden bir komut üretir; bu da savrulmadan başka bir şey değildir.
    # Mandal anında irtifa zaten kapıdan geçmiş (hata ≤ eşik) ve eşitleyici
    # yakınsamış durumda. En iyi hamle: O ANDAKİ dikey hızı KORU. Böylece
    # küçük olan ofset 1-2 saniye boyunca küçük kalır.
    # 1 = terminalde dikey hız mandal anındaki değerde dondurulur.
    TERM_VZ_DONDUR = _env_f("AVCI_IBVS_TERM_VZDON", 0.0) >= 0.5

    # ══ TERMİNALDE DİKEYİ TUTUŞ YASASINDA BIRAKMA (2026-08-09) ══
    # Terminal iki şey birden yapıyor ve ikisi AYNI kefeye konmamalı:
    #   (a) FRENİ KALDIRIYOR — tutuşun PI hız denetimi kutu büyüdükçe yavaşlar
    #       (ölçüldü: 2.7 m'de yalnız 1.7 m/s komut). Bu olmadan araç son
    #       metrede frene basar. Terminalin bu kısmı GEREKLİ.
    #   (b) DİKEY YASAYI DEĞİŞTİRİYOR — nazik yükseliş eşitleyicisi yerine
    #       "hız vektörünü LOS'a doğrult" geliyor. Bunun kazancı v_los·tan(),
    #       yani 18 m/s'de devasa: komut karelerin %17-77'sinde raya dayanıyor
    #       ve araç (2.5 m/s² sınırıyla) izleyemiyor. Bu kısım ZARARLI.
    # Bu seçenek (b)'yi geri alır, (a)'yı korur: terminalde hız tam, dikey
    # ise tutuş fazının eşitleyicisiyle sürülür.
    # Fizik neden doğru: sabit bir YÜKSELİŞ AÇISI tutmak, menzil küçüldükçe
    # doğrusal ofseti sıfıra götürür — yani eşitleyici zaten bir kesişim
    # çözümüdür, üstelik kazancı hıza değil açı hatasına bağlı olduğu için
    # raya dayanmaz.
    # ⚠ VARSAYILAN AÇIK (2026-08-09, 39 uçuşluk kampanyayla doğrulandı):
    #     kontrol            0/3 vuruş, dikey kaçırma 0.84 m
    #     bu yasa + tavan 2.5  8/9 vuruş, dikey kaçırma 0.16 m   (p ≈ 0.018)
    # AVCI_IBVS_TERM_DIKEY_TUTUS=0 eski davranışı geri getirir.
    TERM_DIKEY_TUTUS = _env_f("AVCI_IBVS_TERM_DIKEY_TUTUS", 1.0) >= 0.5
    # Terminal kapısı: yükseliş bu bandın içinde DEĞİLSE hücum başlamaz.
    TERMINAL_ELEV_ESIK = _env_f("AVCI_IBVS_TERM_ELEV", 5.0)  # °
    ELEV_ATALET = _env_f("AVCI_IBVS_ELEV_ATALET", 1.0) >= 0.5  # 0 = eski yol

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
    # 10 → 14 (2026-08-09, 9 uçuşluk A/B/C ölçümü): 10 tabanı hedefin hızının
    # (14.5) ALTINDA kalıyordu, dik açılarda geride kalıyorduk.
    V_TERM_MIN = _env_f("AVCI_IBVS_VTERM_MIN", 14.0)   # m/s; hücum hız tabanı

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
    # 0.6 → 0.9 (2026-08-09, 9 uçuşluk A/B/C ölçümü).
    # ⚠ ÖLÇÜMÜN ASIL DERSİ: iki değişiklik TEK BAŞINA İŞE YARAMIYOR, hatta
    # hız tabanı tek başına TABANDAN KÖTÜ. Ancak BİRLİKTE çalışıyorlar:
    #   yapılandırma        kutu kayması medyanı   çarpışma rotasında
    #   taban (10 / 0.6)          49 px                2/8
    #   A: yalnız taban 14        96 px                1/12   ← kötüleşti
    #   B: yalnız sönüm 0.9       56 px                1/5
    #   C: İKİSİ BİRLİKTE         13 px                3/3    ✓
    # Fiziksel anlamı: taban geometrinin bozulmasını engelliyor, sönüm de
    # aşırı-salınımı kesiyor; biri olmadan diğeri yeni bir dengesizlik açıyor.
    K_VZ_D = _env_f("AVCI_IBVS_KVZD", 0.9)   # dikey sönümleme kazancı
    MAX_ACCEL = 12.0               # m/s²; komut hızı değişim sınırı

    # ── KUTU GEÇERLİLİĞİ ──
    CONF_MIN = _env_f("AVCI_IBVS_CONF", 0.35)   # bunun altı kutu = yok sayılır
    BOYUT_MIN = 6.0                # px; bundan küçük kutu güvenilmez (gürültü)


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

_CSV_ALANLAR = [
    "t", "dt", "durum", "cx", "cy", "w", "h", "boyut", "conf",
    "eps_yaw_deg", "eps_elev_deg", "iris_yaw_deg",
    "boyut_hata", "hiz_I", "v_los", "lead_az_deg", "elev_atalet_deg", "elev_I", "los_hiz_az", "los_hiz_el",
    "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg", "kayip_sayac", "gecikme_ms",
    "iris_pitch_deg", "iris_vz", "iris_hiz",
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


def komut(cx, cy, w, h, iris_yaw, hiz_I, dt, cfg=Cfg, terminal=False,
          los_hiz=(0.0, 0.0), iris_pitch=0.0, iris_vz=0.0, elev_I=0.0,
          pn_yon=None, vz_dondurulmus=None):
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
    eps_yaw = math.atan((cx - cfg.CX_NISAN) / geo.FX)
    # LEAD ÖLÇEĞİ: kalan süreyle (≈menzille) söner — bkz. Cfg.LEAD_SONUM.
    # boyut ∝ 1/menzil olduğu için REF/boyut ≈ menzil/menzil_REF.
    lead_olcek = 1.0
    if cfg.LEAD_SONUM and boyut > 1e-6:
        lead_olcek = clamp(cfg.BOYUT_REF / boyut, 0.0, 1.0)
    lead_sure = cfg.LEAD_SURE * lead_olcek
    lead_az = 0.0
    if terminal:
        # LEAD: nişanı atalet LOS dönüş hızıyla öne al (bkz. Cfg.LEAD_SURE)
        lead_az = clamp(lead_sure * los_hiz[0],
                        -math.radians(cfg.LEAD_MAX_DEG),
                        math.radians(cfg.LEAD_MAX_DEG))
    yaw_cmd = normalize_angle(iris_yaw + cfg.K_YAW * eps_yaw + lead_az)

    # HIZ: kutu boyutu hatası üzerinden PI (terminalde TAM taahhüt)
    hata = cfg.BOYUT_REF - boyut               # px; + = uzak
    hiz_I = clamp(hiz_I + cfg.K_I * hata * dt, cfg.I_MIN, cfg.I_MAX)
    if terminal:
        v_los = cfg.V_TERMINAL                 # hücum: fren yok, sabit hız
    else:
        v_los = clamp(hiz_I + cfg.K_FWD * hata, cfg.V_MIN, cfg.V_TOPLAM_MAX)

    # SAF TAKİP: tüm hız LOS/burun yönünde
    vx_ned = v_los * math.cos(yaw_cmd)
    vy_ned = v_los * math.sin(yaw_cmd)

    eps_elev = math.atan((cy - cfg.CY_NISAN) / geo.FY)   # cy büyük → hedef altta
    if terminal and cfg.TERM_DIKEY_TUTUS:
        # Yatay: terminal (fren yok, tam hız). Dikey: tutuş eşitleyicisi.
        elev_atalet = piksel_elev(cy, cfg) + iris_pitch
        hata_elev = elev_atalet - math.radians(cfg.ELEV_HEDEF_DEG)
        elev_I = clamp(elev_I + cfg.K_ELEV_I * hata_elev * dt,
                       -cfg.ELEV_I_MAX, cfg.ELEV_I_MAX)
        vz_nisan = -(cfg.K_VZ * cfg.V_NOM * hata_elev + elev_I)
        vz = clamp(vz_nisan + cfg.K_VZ_D * (vz_nisan - iris_vz),
                   -cfg.VZ_MAX, cfg.VZ_MAX)
        yaw_cmd = normalize_angle(iris_yaw + eps_yaw)
        vx_ned = v_los * math.cos(yaw_cmd)
        vy_ned = v_los * math.sin(yaw_cmd)
        eps_elev = math.atan((cy - cfg.CY_NISAN) / geo.FY)
        tani = {"boyut": boyut, "eps_yaw": eps_yaw, "eps_elev": eps_elev,
                "elev_atalet": elev_atalet, "elev_hata": hata_elev,
                "hata": hata, "v_los": v_los, "terminal": True,
                "lead_az": 0.0, "lead_olcek": 0.0, "elev_I": elev_I}
        return vx_ned, vy_ned, vz, yaw_cmd, hiz_I, tani, elev_I, pn_yon

    if terminal and cfg.PN:
        # ══ PN — hız vektörünün YÖNÜ, LOS dönüşünün N katıyla döner ══
        # pn_yon = [azimut, yükseliş] (atalet, rad). İlk turda LOS'a kilitlenir.
        los_az_simdi = normalize_angle(iris_yaw + eps_yaw)
        los_el_simdi = piksel_elev(cy, cfg) + iris_pitch
        if pn_yon is None:
            pn_yon = [los_az_simdi, los_el_simdi]
        else:
            # χ̇ = N·λ̇  (+ küçük oranla LOS'a geri çekme: sürüklenme koruması)
            pn_yon[0] = normalize_angle(pn_yon[0] + cfg.PN_N * los_hiz[0] * dt)
            pn_yon[1] = pn_yon[1] + cfg.PN_N * los_hiz[1] * dt
            if cfg.PN_BETA > 0:
                pn_yon[0] = normalize_angle(
                    pn_yon[0] + cfg.PN_BETA * normalize_angle(los_az_simdi - pn_yon[0]))
                pn_yon[1] += cfg.PN_BETA * (los_el_simdi - pn_yon[1])
        pn_yon[1] = clamp(pn_yon[1], -math.radians(60.0), math.radians(60.0))

        v_yon = normalize_angle(pn_yon[0])          # HIZ vektörünün yönü
        yaw_cmd = los_az_simdi if cfg.PN_YAW_LOS else v_yon   # BURNUN yönü
        nisan_elev = pn_yon[1] - cfg.TERM_ELEV_BIAS
        # dikey bütçe: vektör bu açıyı gösteremiyorsa yatayı kıs
        t_ = abs(math.tan(nisan_elev))
        if t_ > 1e-6 and v_los * t_ > cfg.VZ_MAX_TERM:
            v_los = max(cfg.V_TERM_MIN, cfg.VZ_MAX_TERM / t_)
        vx_ned = v_los * math.cos(v_yon)
        vy_ned = v_los * math.sin(v_yon)
        vz = clamp(-v_los * math.tan(nisan_elev),
                   -cfg.VZ_MAX_TERM, cfg.VZ_MAX_TERM)
        if cfg.TERM_VZ_DONDUR and vz_dondurulmus is not None:
            vz = vz_dondurulmus
        eps_elev = math.atan((cy - cfg.CY_NISAN) / geo.FY)
        elev_atalet_tani = los_el_simdi
        tani = {"boyut": boyut, "eps_yaw": eps_yaw, "eps_elev": eps_elev,
                "elev_atalet": elev_atalet_tani,
                "elev_hata": elev_atalet_tani - math.radians(cfg.ELEV_HEDEF_DEG),
                "hata": hata, "v_los": v_los, "terminal": True,
                "lead_az": normalize_angle(v_yon - los_az_simdi),
                "lead_olcek": 1.0, "elev_I": elev_I}
        return vx_ned, vy_ned, vz, yaw_cmd, hiz_I, tani, elev_I, pn_yon

    if terminal:
        # KESİŞİM (PN kapalıyken eski yasa): hız vektörü hedefe DOĞRU baksın.
        elev_atalet = piksel_elev(cy, cfg) + iris_pitch
        lead_el = clamp(lead_sure * los_hiz[1],
                        -math.radians(cfg.LEAD_MAX_DEG),
                        math.radians(cfg.LEAD_MAX_DEG))
        nisan_elev = clamp(elev_atalet + lead_el - cfg.TERM_ELEV_BIAS,
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
        t_ = abs(math.tan(nisan_elev))
        if t_ > 1e-6 and v_los * t_ > cfg.VZ_MAX_TERM:
            v_los = max(cfg.V_TERM_MIN, cfg.VZ_MAX_TERM / t_)
            vx_ned = v_los * math.cos(yaw_cmd)
            vy_ned = v_los * math.sin(yaw_cmd)
        vz_nisan = -v_los * math.tan(nisan_elev)
        # TÜREV SÖNÜMLEMESİ: aracın kendi dikey hızı nişanın ötesine geçtiyse
        # komut geri çekilir → hedefin üstünden geçme biter (bkz. Cfg.K_VZ_D).
        vz = clamp(vz_nisan + cfg.K_VZ_D * (vz_nisan - iris_vz),
                   -cfg.VZ_MAX_TERM, cfg.VZ_MAX_TERM)
        if cfg.TERM_VZ_DONDUR and vz_dondurulmus is not None:
            vz = vz_dondurulmus
    elif cfg.ELEV_ATALET:
        # TUTUŞ — İRTİFA EŞİTLEME (bkz. Cfg.ELEV_HEDEF_DEG).
        # Hedef artık sabit bir PİKSEL değil, ATALET yükselişi: gövde eğimi
        # (jiroskop) hesaba katılır. Böylece drone kendi pitch'i yüzünden
        # yanlış irtifaya sürüklenmez — dikey limit çevriminin kaynağı buydu.
        elev_atalet = piksel_elev(cy, cfg) + iris_pitch
        hata_elev = elev_atalet - math.radians(cfg.ELEV_HEDEF_DEG)
        # İNTEGRAL: hedefin sürekli tırmanışı gibi SABİT yükü öğrenir; orantılı
        # terim tek başına kalıcı hatayı kapatamıyordu (bkz. Cfg.K_ELEV_I).
        _I_yeni = clamp(elev_I + cfg.K_ELEV_I * hata_elev * dt,
                        -cfg.ELEV_I_MAX, cfg.ELEV_I_MAX)
        if cfg.AWU:
            # Doymuş çıkışta, hatayı DAHA DA doyuran yönde integral alma.
            _ham = -(cfg.K_VZ * cfg.V_NOM * hata_elev + _I_yeni)
            _doymus = abs(_ham) >= cfg.VZ_MAX
            if not (_doymus and (_ham * -hata_elev) > 0):
                elev_I = _I_yeni
        else:
            elev_I = _I_yeni
        vz_nisan = -(cfg.K_VZ * cfg.V_NOM * hata_elev + elev_I)
        # Terminaldeki ile aynı türev sönümlemesi (salınımı kesen terim).
        vz = clamp(vz_nisan + cfg.K_VZ_D * (vz_nisan - iris_vz),
                   -cfg.VZ_MAX, cfg.VZ_MAX)
    else:
        # ESKİ YOL (AVCI_IBVS_ELEV_ATALET=0): hedefi sabit CY_NISAN pikselinde tut
        vz = clamp(cfg.K_VZ * cfg.V_NOM * eps_elev, -cfg.VZ_MAX, cfg.VZ_MAX)

    # Terminal kapısı için atalet yükselişi (çağıran karar verir)
    elev_atalet_tani = piksel_elev(cy, cfg) + iris_pitch
    tani = {"boyut": boyut, "eps_yaw": eps_yaw, "eps_elev": eps_elev,
            "elev_atalet": elev_atalet_tani, "elev_I": elev_I,
            "elev_hata": elev_atalet_tani - math.radians(cfg.ELEV_HEDEF_DEG),
            "hata": hata, "v_los": v_los, "terminal": terminal,
            "lead_az": lead_az, "lead_olcek": lead_olcek}
    return vx_ned, vy_ned, vz, yaw_cmd, hiz_I, tani, elev_I, pn_yon


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
    # KAPI_KATI: ölçüm gelmeden kapı açılmasın diye 'çok büyük' başlar
    tani_onceki_elev_hata = (99.0 if cfg.KAPI_KATI else 0.0)
    elev_I = 0.0              # dikey integral (irtifa eşitleyici)
    pn_yon = None             # PN hız-vektörü yönü [azimut, yükseliş]
    vz_mandal = None          # mandal anındaki dikey hız (dondurma için)
    irtifa_bekleme_yazildi = False
    # LOS (atalet) açıları ve hızları — lead nişanı için
    los_az_onceki = los_el_onceki = None
    los_hiz = [0.0, 0.0]      # [azimut, yükseliş] rad/s, EMA'lı

    def _vuruldu():
        if get_temas is None:
            return False
        return get_temas() is True

    os.makedirs(_LOG_DIR, exist_ok=True)
    csv_yol = os.path.join(_LOG_DIR, time.strftime("bbox_ibvs_%Y%m%d_%H%M%S.csv"))
    f = open(csv_yol, "w", newline="")
    w_csv = csv.DictWriter(f, fieldnames=_CSV_ALANLAR, extrasaction="ignore")
    w_csv.writeheader()
    if cfg.TERM_DIKEY_TUTUS and (abs(cfg.TERM_ELEV_BIAS) > 1e-9
                                 or cfg.TERM_VZ_DONDUR or cfg.PN):
        print("[IBVS] ⚠ TERM_BIAS / TERM_VZDON / PN yalnız ESKİ terminal "
              "yasasında etkilidir; şu an dikey kanal tutuş yasasında "
              "(AVCI_IBVS_TERM_DIKEY_TUTUS=1). Bu anahtarlar YOK SAYILIYOR — "
              "kullanmak için AVCI_IBVS_TERM_DIKEY_TUTUS=0 verin.")
    print(f"[IBVS] bbox görsel güdüm başladı — SAF TAKİP + PI hız "
          f"(CANLI GPS YOK, yarışma kuralı). İntegral sıcak başlangıç: "
          f"{hiz_I:.1f} m/s, REF={cfg.BOYUT_REF:.0f}px, tavan "
          f"{cfg.V_TOPLAM_MAX:.0f} m/s, terminal hücum >{cfg.TERMINAL_BOYUT:.0f}px, "
          f"CY_nişan={cfg.CY_NISAN:.0f}, kayıp eşiği={kayip_kare_esik} kare, "
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
            # ── TESPİT GECİKMESİ (yalnız ÖLÇÜM, davranış değişmez) ──
            # wall_recv = ham karenin GELDİĞİ an; buraya varana dek YOLO
            # çıkarımı + kuyruk + döngü beklemesi eklendi. Kapanma 5-18 m/s
            # olduğu için 100 ms bile 1-2 m bayatlık demek; dikey döngüdeki
            # faz gecikmesinin kaynağı da bu olabilir. Önce ÖLÇ.
            _wr = kayit.get("wall_recv")
            gecikme_ms = round((time.time() - _wr) * 1000.0, 1) if _wr else None

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
                                "gecikme_ms": gecikme_ms,
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
                                    "gecikme_ms": gecikme_ms,
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
                                "gecikme_ms": gecikme_ms,
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
            los_az = normalize_angle(iyaw + math.atan((cx - cfg.CX_NISAN) / geo.FX))
            los_el = piksel_elev(cy, cfg) + ipitch
            if los_az_onceki is not None and 1e-3 < dt < 0.5:
                a_ = cfg.LEAD_EMA
                los_hiz[0] = (a_ * (normalize_angle(los_az - los_az_onceki) / dt)
                              + (1 - a_) * los_hiz[0])
                los_hiz[1] = (a_ * ((los_el - los_el_onceki) / dt)
                              + (1 - a_) * los_hiz[1])
            los_az_onceki, los_el_onceki = los_az, los_el
            # ── TERMİNAL MANDALI: İKİ ŞART (2026-08-09, kullanıcı fikri) ──
            #  1) kutu yeterince büyük (menzil kapandı)
            #  2) ⚠ İRTİFA OTURDU — atalet yükselişi hedef bandında
            # İkincisi yeni: eskiden irtifa eşitlenmeden hücum başlıyordu ve
            # terminal hem yatayı kapatıp hem dikeyi düzeltmeye çalışınca
            # dikey döngü salınıyordu (ölçüldü: dikey fark ±7 m limit çevrimi,
            # en yakın anda drone hedefin 0.5 m ÜSTÜNDE → ıska).
            # Artık önce irtifa eşitlenir, terminal saf yatay koşuya döner.
            _boy_simdi = math.sqrt(bw * bh)
            if (terminal_mandal and cfg.TERM_BIRAK > 0
                    and _boy_simdi < cfg.TERM_BIRAK * cfg.TERMINAL_BOYUT):
                terminal_mandal = False
                vz_mandal = None
                pn_yon = None            # yeniden mandallayınca LOS'a kilitlensin
                irtifa_bekleme_yazildi = False
                print(f"[IBVS] terminal BIRAKILDI (kutu {_boy_simdi:.0f}px < "
                      f"{cfg.TERM_BIRAK * cfg.TERMINAL_BOYUT:.0f}) — menzil "
                      f"açıldı, tutuşa dönülüyor (irtifa yeniden eşitlenecek)")
            if not terminal_mandal:
                _boy = _boy_simdi
                _eh = math.degrees(abs(tani_onceki_elev_hata))
                if _boy >= cfg.TERMINAL_BOYUT:
                    if _eh <= cfg.TERMINAL_ELEV_ESIK:
                        terminal_mandal = True
                        # Dondurma seçeneği için: hücuma girerken aracın
                        # GERÇEK dikey hızı — eşitleyici yakınsamışken bu,
                        # hedefin tırmanışına oturmuş hızdır.
                        vz_mandal = float(iris.get("vz", 0.0) or 0.0)
                        print(f"[IBVS] ⚡ TERMİNAL HÜCUM (kutu {_boy:.0f}px ≥ "
                              f"{cfg.TERMINAL_BOYUT:.0f}, yükseliş hatası "
                              f"{_eh:.1f}° ≤ {cfg.TERMINAL_ELEV_ESIK:.0f}°) — "
                              f"irtifa oturdu, tam taahhüt")
                    elif not irtifa_bekleme_yazildi:
                        irtifa_bekleme_yazildi = True
                        _eh_yazi = ("henüz ölçüm yok" if _eh > 360.0
                                    else f"yükseliş hatası {_eh:.1f}°")
                        print(f"[IBVS] menzil hazır ama İRTİFA OTURMADI "
                              f"({_eh_yazi}, eşik "
                              f"{cfg.TERMINAL_ELEV_ESIK:.0f}°) — önce eşitleniyor")
            vx, vy, vz, yaw_hedef, hiz_I, tani, elev_I, pn_yon = komut(
                cx, cy, bw, bh, iyaw, hiz_I, dt, cfg, terminal_mandal,
                tuple(los_hiz), ipitch, float(iris.get("vz", 0.0) or 0.0),
                elev_I, pn_yon, vz_mandal)
            tani_onceki_elev_hata = tani.get("elev_hata", 0.0)
            # ── YAW SLEW SINIRI (bkz. Cfg.YAW_RATE_MAX) ──
            # HIZ (vx, vy) yaw_hedef'ten hesaplandı ve DEĞİŞMEZ: nişan hedefin
            # gerçek yönünde kalır. Sınırlanan yalnız BURUNUN dönme hızı.
            if cmd_yaw is None:
                cmd_yaw = iyaw
            yaw_err = normalize_angle(yaw_hedef - cmd_yaw)
            adim = clamp(yaw_err, -cfg.YAW_RATE_MAX * dt, cfg.YAW_RATE_MAX * dt)
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
                "eps_elev_deg": round(math.degrees(tani["eps_elev"]), 1),
                "iris_yaw_deg": round(math.degrees(iyaw), 1),
                "boyut_hata": round(tani["hata"], 1),
                "hiz_I": round(hiz_I, 2), "v_los": round(tani["v_los"], 2),
                "lead_az_deg": round(math.degrees(tani["lead_az"]), 2),
                "elev_atalet_deg": round(math.degrees(tani["elev_atalet"]), 2),
                "elev_I": round(tani.get("elev_I", 0.0), 2),
                "los_hiz_az": round(los_hiz[0], 3), "los_hiz_el": round(los_hiz[1], 3),
                "vx_cmd": round(vx, 2), "vy_cmd": round(vy, 2),
                "vz_cmd": round(vz, 2),
                "yaw_cmd_deg": round(math.degrees(yaw_cmd), 1),
                "kayip_sayac": 0, "gecikme_ms": gecikme_ms,
                # Dikey döngünün teşhisi için: aracın KENDİ eğimi ve dikey hızı.
                # elev_atalet = piksel_elev(cy) + iris_pitch olduğu için pitch
                # yanlışsa yükseliş ölçümü topyekûn kayar; iris_vz ise komutun
                # gerçekten izlenip izlenmediğini gösterir (WPNAV_ACCEL_Z=2.5
                # m/s² tavanı yüzünden ±5 m/s komut fiziksel olarak 4 s sürer).
                "iris_pitch_deg": round(math.degrees(ipitch), 1),
                "iris_vz": round(float(iris.get("vz", 0.0) or 0.0), 2),
                "iris_hiz": round(math.hypot(float(iris.get("vx", 0.0) or 0.0),
                                             float(iris.get("vy", 0.0) or 0.0)), 1),
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
