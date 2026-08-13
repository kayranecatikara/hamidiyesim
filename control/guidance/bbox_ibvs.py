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
from control import ozellik_bayraklari as _ozellikler   # çalışma-anı kill-switch deposu

# ── ŞARTNAME 3-FAZ KAPISI (2026-08-09 entegrasyon) ──
# Terminal hücum (mandal) YALNIZ merkezî görev FSM STRIKE'ında tetiklenir (5 sn
# kümülatif + 3 sn kesintisiz kilit doldu). Böylece tespit/takip/angajman
# disiplini, bbox IBVS ALGORİTMASININ üstünde korunur (kullanıcı isteği: 3 fazlı
# yapı bozulmasın, ama fazlarda çalışan algoritma bbox_ibvs olsun). Kapı KAPALI
# (AVCI_SARTNAME_KAPI=off) ya da get_gorev_state verilmemişse: NATIVE davranış
# (kutu eşiği aşınca hemen hücum) — bbox_ibvs birim testleri bu yolu kullanır.
try:
    from control.mission_fsm import State as _State
except Exception:                       # mission_fsm yoksa (izole test) native
    _State = None
_SARTNAME_KAPI = os.environ.get("AVCI_SARTNAME_KAPI", "on").lower() in ("on", "1")


def _env_f(name, default):
    return float(os.environ.get(name, default))


def _env_bool(name, default=False):
    """Açma/kapama bayrağı — on/off/1/0/true/false kabul eder (bkz. AVCI_IBVS_PN).
    Dosya başı _SARTNAME_KAPI ile aynı sözleşme; sayısal _env_f'in aksine
    "on" gibi metinlerde patlamaz."""
    v = os.environ.get(name)
    if v is None:
        return default
    return v.strip().lower() in ("on", "1", "true", "yes", "evet")


# ── ÇALIŞMA-ANI KİLL-SWITCH KÖPRÜSÜ (2026-08-10) ───────────────────────────
# PN/PRED/INTERCEPT eskiden IMPORT anında env'den okunuyordu; artık ÇALIŞMA
# ANINDA ozellik_bayraklari deposundan okunuyor (gcs_server yeniden başlatmadan
# UI'dan açılıp kapatılabilsin diye). Depo AYNI env ile tohumlu, dolayısıyla
# hiçbir düğmeye dokunulmazsa (hepsi env varsayılanı=off) davranış BYTE-AYNI.
#
# İKİ OKUMA YOLU, TEK GARANTİ:
#  1) VARSAYILAN cfg=Cfg → Cfg.PN/PRED/INTERCEPT bir DESCRIPTOR'dur (aşağıdaki
#     _CanliBayrak): getattr her seferinde depoyu okur → çalışma-anı togglable.
#  2) TEST cfg alt sınıfı (ör. `class _PNon(Cfg): PN = True`) düz sınıf
#     özniteliği atadığı için descriptor'ı GÖLGELER → getattr o sabiti verir.
#     Böylece mevcut birim testleri (per-cfg override) BİRE BİR korunur.
# _ozellik(cfg, anahtar, attr): iki yolu birleştirir — cfg açıkça override
# etmişse onu, etmemişse (canlı descriptor) depoyu okur.

class _CanliBayrak:
    """Cfg üzerinde çalışma-anı kill-switch okuyan descriptor. Yalnız TABAN
    Cfg'de canlıdır; bir alt sınıf düz bool atarsa gölgelenir (native/test yolu)."""
    def __init__(self, anahtar):
        self.anahtar = anahtar

    def __get__(self, obj, objtype=None):
        return _ozellikler.get(self.anahtar)


class _CfgMeta(type):
    """Cfg metasınıfı: sınıf üzerinden (getattr(cfg, 'PN')) descriptor'ın
    çalışmasını sağlar — düz sınıf özniteliği descriptor protokolünü sınıf
    erişiminde çalıştırmaz, metasınıf çalıştırır."""
    PN = _CanliBayrak("pn")
    PRED = _CanliBayrak("pred")
    INTERCEPT = _CanliBayrak("intercept")
    VLEAD = _CanliBayrak("vlead")
    TBOOST = _CanliBayrak("tboost")
    GAINSCHED = _CanliBayrak("gainsched")
    COMMIT = _CanliBayrak("commit")
    DUZTERM = _CanliBayrak("duzterm")


def _ozellik(cfg, anahtar, attr):
    """cfg[attr] override edilmişse onu, edilmemişse çalışma-anı depoyu döner.
    Taban Cfg'de attr canlı descriptor'dır → depo okunur (togglable). Test alt
    sınıfı attr'ı düz atarsa o değer okunur (byte-aynı native/test davranışı)."""
    return bool(getattr(cfg, attr, _ozellikler.get(anahtar)))


class Cfg(metaclass=_CfgMeta):
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
    VZ_MAX_TERM = _env_f("AVCI_IBVS_VZT", 5.0)   # m/s; terminalde dikey tavan

    # ══ PROPORTIONAL-NAVIGATION YATAY LEAD — KİLL-SWITCH (2026-08-09) ══
    # ARAŞTIRMA BULGUSU (arastirma_raporu Aday #2): crosser fly-past'ın kökü,
    # terminal yaw'ın SABİT KERTERİZ (collision triangle) yerine hedefin ŞU ANKİ
    # pikselini kovalaması. Çözüm: yatay LOS DÖNÜŞ HIZINI sıfıra süren PN-tipi
    # lead — burun, hedefin GİDECEĞİ yere döner, geçtiği yere değil.
    #
    # ⚠ EGO-DÜZELTME — KRİTİK BULGU (bu dosyada zaten yapılıyor):
    #   los_hiz[0] = d/dt[ iyaw + atan((cx−CX)/FX) ]  (run_bbox_ibvs, ~587)
    #   Yani LOS azimutu = drone HEADING + kadraj-içi kerteriz. Türevi alınınca
    #   drone'un KENDİ yaw dönüşü ZATEN doğru biçimde içeride — los_hiz[0] atalet
    #   (ego-temiz) LOS hızıdır. Diagnostik doğruladı: kod-oranı gerçek-atalet
    #   oranıyla ondalığına dek eşit; "ham kadraj oranı" (yalnız atan türevi) ise
    #   gerçek − yaw_rate olurdu (kontamine). Ekip _yatay_pn'de de aynısını
    #   world-frame u_dunya azimutundan yapıyor. Dolayısıyla true_los_rate_az
    #   İÇİN yaw_rate'i TEKRAR çıkarmak ÇİFT-DÜZELTME (yanlış) olur.
    #   PN_EGO kancası bu yüzden VARSAYILAN KAPALI: los_hiz[0] zaten ego-temiz;
    #   flag yalnız varsayım yanlışsa uçuşta A/B için (frame_rate − yaw_rate yolu).
    #
    # FORMÜL (yalnız YAW/YATAY kanal; dikey vz yasasına DOKUNULMAZ):
    #   t_go   = clamp(menzil / max(kapanma, KAP_MIN), 0, T_GO_MAX)
    #            menzil = MENZIL_PX_M / boyut  (kutudan; GPS yok)
    #   lead_az = clamp(N · true_los_rate_az · t_go, ±LEAD_MAX_DEG)
    #   N (PN sabiti) ≈ 3.0; native lead (LEAD_SURE·lead_olcek·los_hiz) yerine geçer.
    # KAPALI (AVCI_IBVS_PN=off, VARSAYILAN) → native lead_az bit-aynı korunur.
    # PN artık ÇALIŞMA ANINDA okunur: TABAN Cfg'de _CfgMeta.PN canlı descriptor'ı
    # (ozellik_bayraklari deposu, "pn" env ile tohumlu). Burada düz atanMAZ ki
    # descriptor gölgelenmesin; test alt sınıfı `PN = True` atarsa gölgeler
    # (byte-aynı). Erişim: _ozellik(cfg, "pn", "PN") ya da getattr(cfg, "PN").
    PN_N = _env_f("AVCI_IBVS_PN_N", 3.0)         # PN etkin oransal sabiti (N)
    PN_TGO_MAX = _env_f("AVCI_IBVS_PN_TGO", 3.0)  # s; t_go tavanı (uzak/yavaşta patlamasın)
    PN_KAP_MIN = _env_f("AVCI_IBVS_PN_KAPMIN", 1.5)  # m/s; t_go için kapanma tabanı
    # EGO kancası: los_hiz[0] ZATEN ego-temiz (yukarı). AÇIKKEN yaw_rate BİR
    # KEZ DAHA çıkarılır (frame-rate varsayımı) — normalde YANLIŞ, yalnız A/B için.
    PN_EGO = _env_bool("AVCI_IBVS_PN_EGO", False)

    # ══ KESTİRİM + COAST (PRED) — KİLL-SWITCH (2026-08-09, arastirma Aday #3/#5) ══
    # ÖLÇÜLEN PROBLEM: hedef-kaybı %73.7; tespit karelerin yalnız ~%24'ünde kutu
    # taşıyor. Kör terminal hücumda (TERM_KOR) son komut 2 s DONDURULUYOR — araç
    # düz uçarken CROSSER hedef yana kaçıyor → ıska.
    #
    # ÇÖZÜM (yalnız (cx,cy) BESLEMESİNİ etkiler; dikey/yatay YASAYA dokunmaz):
    #   Görüntü-düzlemi hedef merkezine (cx,cy) hafif alpha-beta (sabit-hız)
    #   kestirimi. Durum = [cx, cy, vcx, vcy] px, px/s. Her geçerli kutuda GÜNCELLE.
    #   Kutu YOKKEN (özellikle kör hücum) İLERİ TAHMİN et (cx += vcx·dt) ve bu
    #   tahmini komut()'a besle → güdüm donan komut yerine hareket eden hedefi
    #   izlemeyi sürdürür. Tahmin ufku sınırlı: PRED_MAXS (0.6 s) sonra tahmine
    #   güvenilmez (donmuş komuta düşülür). Normal takipte ise küçük bir
    #   görüntü-düzlemi LEAD (tahmin edilen hız yönünde öne nişan) uygulanır —
    #   PN'in YERİNE değil, ONUNLA birlikte (PN los_hiz'i okur; bu cx/cy'yi kaydırır).
    # KAPALI (AVCI_IBVS_PRED=off, VARSAYILAN) → native/donmuş davranış BİT-AYNI.
    # PRED artık ÇALIŞMA ANINDA okunur (TABAN Cfg'de _CfgMeta.PRED descriptor'ı,
    # "pred" env ile tohumlu). Düz atanMAZ (descriptor gölgelenmesin); test alt
    # sınıfı `PRED = True` atarsa gölgeler (byte-aynı native/test yolu).
    PRED_MAXS = _env_f("AVCI_IBVS_PRED_MAXS", 0.6)  # s; gerçek tespitsiz coast ufku
    PRED_LEAD = _env_f("AVCI_IBVS_PRED_LEAD", 0.3)  # normal takip görüntü-lead kesri
    # Alpha-beta gözlem kazançları (α konum düzeltmesi, β hız düzeltmesi).
    # α≈0.5/β≈0.1: gürültülü bbox'ta makul yumuşatma + hız izleme (ölçek-bağımsız).
    PRED_ALPHA = _env_f("AVCI_IBVS_PRED_ALPHA", 0.5)
    PRED_BETA = _env_f("AVCI_IBVS_PRED_BETA", 0.1)

    # ══ KESİŞİM-NOKTASI TAAHHÜDÜ (INTERCEPT) — KİLL-SWITCH (2026-08-10) ══
    # ÖLÇÜLEN PROBLEM (en iyi uçuş, log 092502 + candidate_opt960 meta):
    #   en yakın 3.00 m, 0 vuruş; 97 KÖR-hücum epizodu (churn); kör faz 18 m/s
    #   × 2.05 s'ye kadar = ~37 m fly-past. Terminal karelerinin %95'inde yatay
    #   lead 25°'de DOYMUŞ (crosser hedefi kovalamaya yetmiyor); en yakın
    #   karelerde cx=407-436 (kadraj sağ kenarı, eps_yaw 37°), eps_elev −26° —
    #   yaw DA dikey DE hedefin GİTTİĞİ yerin GERİSİNDE. Drone hedefin ŞU ANKİ
    #   yerine nişanlıyor, GİDECEĞİ yere değil → arkasından/yanından geçiyor.
    #
    # ÇÖZÜM (yalnız (cx,cy) BESLEMESİ; dikey/yatay YASAYA dokunulmaz): terminal
    # son-birleşmede, hedefin ŞU ANKİ pikseli yerine TAHMİN EDİLEN KESİŞİM
    # noktasına nişanla. PRED kestiricisinin görüntü-düzlemi hedef hızını
    # (vcx,vcy px/s — AVCI_IBVS_PRED GEREKLİ) ve kalan süreyi kullan:
    #     t_go   = menzil / max(kapanma, KAPMIN)   (menzil = MENZIL_PX_M/boyut)
    #     cx_aim = cx + vcx · t_go · INTERCEPT_K
    #     cy_aim = cy + vcy · t_go · INTERCEPT_K
    # Kayma büyüklüğü INTERCEPT_MAX_PX ile sınırlanır (kadraj yakınında kal).
    # Bu (cx_aim,cy_aim) komut()'a beslendiği için taahhüt edilen dalış hem
    # yaw'ı hem dikeyi hedefin GELECEK konumuna sürer — PN'in salt-yatay,
    # doymuş lead'inden güçlü ve terminale özgü. PRED'in lead_ofset'inin
    # (küçük, sabit ufuklu) yerine geçer: terminalde INTERCEPT, normalde
    # PRED lead_ofset.
    # AYRICA churn/fly-past azaltması: INTERCEPT açıkken kör hücum ufku
    # daha kısa bir tavana (INTERCEPT_KOR_MAXS) bağlanır — 37 m düz taşmayı
    # keser; kör fazda da coast+intercept ile hedefe DOĞRU eğrilmeye devam
    # edilir (donmuş-düz değil).
    # KAPALI (AVCI_IBVS_INTERCEPT=off, VARSAYILAN) → beslenen (cx,cy) hiç
    # kaydırılmaz, kör ufuk TERMINAL_SURE kalır: BİT-AYNI native.
    # INTERCEPT artık ÇALIŞMA ANINDA okunur (TABAN Cfg'de _CfgMeta.INTERCEPT
    # descriptor'ı, "intercept" env ile tohumlu). Düz atanMAZ (descriptor
    # gölgelenmesin); test alt sınıfı `INTERCEPT = True` atarsa gölgeler (byte-aynı).
    INTERCEPT_K = _env_f("AVCI_IBVS_INTERCEPT_K", 1.0)    # t_go lead kesri (1.0=tam kesişim)
    INTERCEPT_MENZIL = _env_f("AVCI_IBVS_INTERCEPT_MENZIL", 6.0)  # m; yalnız bu menzilin ALTINDA uygula
    INTERCEPT_MAX_PX = _env_f("AVCI_IBVS_INTERCEPT_MAX_PX", 200.0)  # px; nişan kayması tavanı
    INTERCEPT_KAPMIN = _env_f("AVCI_IBVS_INTERCEPT_KAPMIN", 1.5)  # m/s; t_go için kapanma tabanı
    INTERCEPT_TGO_MAX = _env_f("AVCI_IBVS_INTERCEPT_TGO", 2.0)  # s; t_go tavanı (yavaş kapanmada patlamasın)
    # Kör hücum ufku INTERCEPT açıkken bu tavanla sınırlanır (37 m fly-past'ı
    # keser). TERMINAL_SURE ile min alınır → yalnızca KISALTIR, uzatmaz.
    INTERCEPT_KOR_MAXS = _env_f("AVCI_IBVS_INTERCEPT_KOR_MAXS", 0.8)  # s; INTERCEPT-on kör ufuk tavanı

    # ══ TEKTE VURUŞ ÖZELLİKLERİ (2026-08-10) — dört bağımsız kill-switch ══
    # Hepsi VARSAYILAN KAPALI → byte-aynı native. Çalışma-anı okunur (TABAN
    # Cfg'de _CfgMeta descriptor'ları: vlead/tboost/gainsched/commit, ilgili env
    # ile tohumlu). Hepsi TERMİNAL-özgü; tutuş fazına dokunmaz. Erişim:
    # _ozellik(cfg, "vlead", "VLEAD") vb.
    #
    # VLEAD — dikey terminal öncülük: terminalde nişan yükselişine sabit +açı
    #   payı ekler → hedefin ÜSTÜNE nişanla. Teşhis edilen dikey ıska ekseni
    #   (yükselen hedef + 25° kamera tilt) için. ⚠ Önceki dikey PN kötüleştirmişti;
    #   bu SADECE terminal + küçük sabit pay + kill-switch (o yüzden ayrı sınanır).
    VLEAD_DEG = _env_f("AVCI_IBVS_VLEAD_DEG", 6.0)   # derece; terminal yukarı nişan payı
    # TBOOST — terminal hız artışı: hücum hızını V_TERMINAL yerine bu değere çıkarır
    #   (hedef kadrajdan kaçmadan son metreyi delmek için). Dikey-bütçe kısıtı
    #   yine geçerli (aşırı dik nişanda yatayı kısar).
    TBOOST_VTERM = _env_f("AVCI_IBVS_TBOOST_VTERM", 23.0)  # m/s; TBOOST-on hücum hızı
    # GAINSCHED — PN kazanç programı: t_go küçüldükçe etkin N büyür →
    #   N_eff = PN_N·(1 + GAINSCHED_K·(1 − t_go/PN_TGO_MAX)). PN AÇIK gerektirir
    #   (PN kapalıyken hiç etki yok). Terminale doğru keskinleşen öncülük.
    GAINSCHED_K = _env_f("AVCI_IBVS_GAINSCHED_K", 1.0)
    # COMMIT — son-saniye taahhüt: t_go ≤ COMMIT_TGO iken yaw hatası ölü bant
    #   altındaysa (küçük piksel titremesi) yaw düzeltmesi SIFIRLANIR → burun
    #   son vektöre taahhüt eder, churn/fly-past titremesi kesilir. Yalnız YATAY
    #   kanal (churn'ün kaynağı). Büyük hata hâlâ düzeltilir (kilitlenip sapmaz).
    COMMIT_TGO = _env_f("AVCI_IBVS_COMMIT_TGO", 0.8)      # s; bu t_go altında taahhüt
    COMMIT_DEADBAND_DEG = _env_f("AVCI_IBVS_COMMIT_DB", 4.0)  # derece; yaw ölü bant

    # DUZTERM — düz kuyruk / pure pursuit: terminalde lead_az'ı ±DUZTERM_LEAD_MAX
    # ile KAPAR (ölçülen kök neden: PN lead ±25°'de doyup işaret değiştirince
    # yaw limit-cycle salınımı → drone teğet geçiyor; log 191011 terminal yaw_cmd
    # std 34.8°, 218 yön değişimi). Küçük tavan (varsayılan 4°) lead'in violent
    # ±25 sıçramasını keser → hedefin ARKASINDAN düz gidip doğrudan çarpar.
    # KAPALI → lead_az normal (±LEAD_MAX_DEG), bit-aynı. Kilit %6 tespitine
    # dokunmaz (ayrı katman). 0.0 = tam pure pursuit (lead yok).
    DUZTERM_LEAD_MAX = _env_f("AVCI_IBVS_DUZTERM_LEAD_MAX", 4.0)  # derece; terminal lead tavanı

    # CAPRAZLEAD — adaptif terminal lead tavanı (kill-switch, env-only, panelde YOK).
    # ÖLÇÜLEN SORUN (kilit_20260811_155735 terminal analizi): yandan geçen hedef
    # 5.9 m'de kadrajı süpürüp çıkıyor (cx 330→201→480), 2.14 s tespit kaybı →
    # kesintisiz kilit resetleniyor (kümülatif kopması) VE yanal ıska — TEK KÖK.
    # DUZTERM lead'i 4°'de sabit kapıyor; yandan geçen hedef DAHA ÇOK lead ister.
    # ÇÖZÜM: LOS açısal hızı (|los_hiz[0]|) yükseldikçe tavanı ek ile aç → burun
    # öne alır, hedef kadrajda kalır. Kuyrukta (LOS hızı ~0) ek=0 → 4° korunur →
    # salınım koruması bozulmaz. KAPALI → _dlmax=DUZTERM_LEAD_MAX (bit-aynı).
    CAPRAZLEAD = _env_bool("AVCI_IBVS_CAPRAZLEAD", False)
    CAPRAZ_LOS_ESIK = _env_f("AVCI_IBVS_CAPRAZ_LOS_ESIK", 0.05)  # rad/s; altı = kuyruk, ek yok
    CAPRAZ_KAZANC = _env_f("AVCI_IBVS_CAPRAZ_KAZANC", 120.0)     # derece/(rad/s)
    CAPRAZ_EK_MAX = _env_f("AVCI_IBVS_CAPRAZ_EK_MAX", 12.0)      # derece; ek tavan üst sınırı

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
    # DİKEY ÖLÇEK HARMANI (2026-08-10, birleşme dikey ıska teşhisi): terminal
    # dikey hız v_dikey = kapanma + DIK_ORAN·(v_los − kapanma). 0 = saf kapanma
    # (byte-aynı; hedef üstten çıkıp ALTINDAN geçiliyordu), 1 = saf yer hızı
    # (hedef alttan çıkıp ÜSTÜNDEN geçiliyordu). Ortası hız vektörünü hedefe
    # oturtur → sub-metre dikey ıskayı kapatır. Ölçümle ayarlanır.
    DIK_ORAN = _env_f("AVCI_IBVS_DIK_ORAN", 0.0)
    KAPANMA_MIN = _env_f("AVCI_IBVS_KAPANMA_MIN", 1.5)   # m/s; ölçek tabanı
    KAPANMA_EMA = _env_f("AVCI_IBVS_KAPANMA_EMA", 0.20)  # kare başına yumuşatma
    # Kutu boyutu → menzil ölçeği: TERMINAL_BOYUT 25 px ≈ 6.4 m (Cfg yorumu)
    MENZIL_PX_M = 160.0                                  # px·m
    MAX_ACCEL = 12.0               # m/s²; komut hızı değişim sınırı

    # ── KUTU GEÇERLİLİĞİ ──
    CONF_MIN = _env_f("AVCI_IBVS_CONF", 0.35)   # bunun altı kutu = yok sayılır
    BOYUT_MIN = 6.0                # px; bundan küçük kutu güvenilmez (gürültü)


class HedefKestirim:
    """Görüntü-düzlemi hedef merkezi (cx,cy) için alpha-beta (sabit-hız) kestirimi.

    Durum: [cx, cy, vcx, vcy]  (px, px/s). Saf Python — yeni bağımlılık yok.

    Alpha-beta, sabit-hız Kalman'ın sabit-kazançlı sadeleştirmesidir: tahmin
    adımı konumu hızla ilerletir, güncelleme adımı ölçüm artığını (residual)
    α (konum) ve β/dt (hız) ile geri besler. Terminal bbox gürültüsünde ucuz ve
    kararlıdır; kutu boşluğunda saf ileri-tahmin (coast) verir.

    Kullanım (run_bbox_ibvs döngüsü):
      est = HedefKestirim(cfg)
      # geçerli kutuda:
      est.guncelle(cx, cy, dt)             # tahmin + ölçüm birleşimi
      # kutu yokken (coast):
      cx_p, cy_p = est.tahmin_ileri(dt)    # yalnız ileri-tahmin, ölçüm yok
    """

    def __init__(self, cfg=Cfg):
        self.cfg = cfg
        self.cx = self.cy = None       # None = henüz başlatılmadı
        self.vcx = self.vcy = 0.0      # px/s

    def hazir(self):
        """Hız kestirimi anlamlı mı? (en az iki ölçüm görüldü)."""
        return self.cx is not None

    def guncelle(self, cx, cy, dt):
        """Geçerli kutu: alpha-beta tahmin+güncelleme. Yeni (cx,cy) durumu döner.

        İlk çağrı yalnız durumu tohumlar (hız 0). Sonraki çağrılar hızı ölçüm
        artığından öğrenir. dt makul aralıkta değilse (akış boşluğu) hız
        güncellenmez, yalnız konum ölçüme çekilir (bayat hızla ekstrapolasyon
        yapıp savrulmayı önler).
        """
        if self.cx is None:            # tohumla
            self.cx, self.cy = float(cx), float(cy)
            self.vcx = self.vcy = 0.0
            return self.cx, self.cy
        if not (1e-3 < dt < 0.5):
            # akış boşluğu — hızı ekstrapole etme, konumu ölçüme oturt
            self.cx, self.cy = float(cx), float(cy)
            return self.cx, self.cy
        # TAHMİN (sabit hız)
        cx_pred = self.cx + self.vcx * dt
        cy_pred = self.cy + self.vcy * dt
        # ÖLÇÜM ARTIĞI
        rx = float(cx) - cx_pred
        ry = float(cy) - cy_pred
        a, b = self.cfg.PRED_ALPHA, self.cfg.PRED_BETA
        self.cx = cx_pred + a * rx
        self.cy = cy_pred + a * ry
        self.vcx += (b / dt) * rx
        self.vcy += (b / dt) * ry
        return self.cx, self.cy

    def tahmin_ileri(self, dt):
        """Kutu YOK: yalnız ileri-tahmin (coast). Durumu ilerletir ve döner.

        Ölçüm olmadığı için hız sabit kalır; konum vcx/vcy·dt kadar ilerler.
        Böylece kör hücumda donmuş komut yerine hedefin GİTTİĞİ yer beslenir.
        """
        if self.cx is None:
            return None
        if 1e-3 < dt < 0.5:
            self.cx += self.vcx * dt
            self.cy += self.vcy * dt
        return self.cx, self.cy

    def lead_ofset(self):
        """Normal takip için görüntü-düzlemi LEAD ofseti (dcx, dcy) px.

        Tahmin edilen hızla, coast ufku (PRED_MAXS) boyunca alınacak yer
        değiştirmenin PRED_LEAD kesri kadar öne nişan. Modest (PN'i ezmez):
        düz kuyruk takibinde v≈0 → ofset≈0.
        """
        if self.cx is None:
            return 0.0, 0.0
        s = self.cfg.PRED_LEAD * self.cfg.PRED_MAXS
        return self.vcx * s, self.vcy * s

    def intercept_ofset(self, boyut, kapanma):
        """TERMİNAL kesişim ofseti (dcx, dcy) px — hedefin GELECEK pikseline nişan.

        Kalan süre t_go = menzil / max(kapanma, KAPMIN) boyunca hedefin
        görüntü-düzlemi hızıyla (vcx,vcy) alacağı yol; INTERCEPT_K kesriyle
        ölçekli. Menzil kutu boyutundan (GPS yok): menzil = MENZIL_PX_M/boyut.
        Kayma büyüklüğü INTERCEPT_MAX_PX ile sınırlanır (kadraj yakınında kal).

        Yalnız INTERCEPT_MENZIL menzilinin ALTINDA anlamlı; çağıran menzil
        kapısını uygular. Hız kestirimi yoksa (tek ölçüm) vcx=vcy=0 → ofset 0.
        Döner: (dcx, dcy, t_go) — t_go tanı/log için.
        """
        if self.cx is None or boyut is None or boyut <= 1e-6:
            return 0.0, 0.0, 0.0
        menzil = self.cfg.MENZIL_PX_M / boyut
        kap = self.cfg.INTERCEPT_KAPMIN
        if kapanma is not None:
            kap = max(self.cfg.INTERCEPT_KAPMIN, float(kapanma))
        t_go = clamp(menzil / kap, 0.0, self.cfg.INTERCEPT_TGO_MAX)
        dcx = self.vcx * t_go * self.cfg.INTERCEPT_K
        dcy = self.vcy * t_go * self.cfg.INTERCEPT_K
        # kayma büyüklüğünü tavanla (yön korunur): gürültülü vcx·t_go savurmasın
        mag = math.hypot(dcx, dcy)
        if mag > self.cfg.INTERCEPT_MAX_PX and mag > 1e-9:
            k = self.cfg.INTERCEPT_MAX_PX / mag
            dcx *= k
            dcy *= k
        return dcx, dcy, t_go


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

_CSV_ALANLAR = [
    "t", "dt", "durum", "cx", "cy", "w", "h", "boyut", "conf",
    "eps_yaw_deg", "eps_elev_deg", "iris_yaw_deg",
    "boyut_hata", "hiz_I", "v_los", "lead_az_deg", "los_hiz_az", "los_hiz_el",
    "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg", "kayip_sayac",
    "lead_kaynak", "yaw_rate_deg",
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
          los_hiz=(0.0, 0.0), iris_pitch=0.0, iris_vz=0.0,
          kapanma=None, yaw_rate=0.0):
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
    _lead_kaynak = "native"
    # Terminal t_go (GAINSCHED/COMMIT/PN ORTAK) — menzil/kapanma, GPS yok.
    # menzil = MENZIL_PX_M/boyut; kapanma None/küçükse PN_KAP_MIN tabanı.
    t_go_term = None
    if terminal:
        _kap_t = cfg.PN_KAP_MIN
        if kapanma is not None:
            _kap_t = max(cfg.PN_KAP_MIN, float(kapanma))
        _menzil_t = (cfg.MENZIL_PX_M / boyut) if boyut > 1e-6 else 0.0
        t_go_term = clamp(_menzil_t / _kap_t, 0.0, cfg.PN_TGO_MAX)
        if _ozellik(cfg, "pn", "PN"):
            # ── PN-TİPİ YATAY LEAD (kill-switch AÇIK; bkz. Cfg.PN) ──
            # true_los_rate_az: los_hiz[0] ZATEN ego-temiz (atalet LOS hızı;
            # bkz. Cfg.PN yorumu ve run_bbox_ibvs ~587). PN_EGO açıksa yaw_rate
            # BİR KEZ DAHA çıkarılır — bu ÇİFT-DÜZELTMEDİR, normalde YANLIŞ,
            # yalnız varsayımı uçuşta sınamak için (VARSAYILAN kapalı).
            true_los_rate_az = los_hiz[0]
            if getattr(cfg, "PN_EGO", False):
                true_los_rate_az = los_hiz[0] - yaw_rate
            t_go = t_go_term
            # GAINSCHED (kill-switch): t_go küçüldükçe etkin N büyür (bkz.
            # Cfg.GAINSCHED_K). KAPALI → N_eff = PN_N (bit-aynı PN).
            _N = cfg.PN_N
            if _ozellik(cfg, "gainsched", "GAINSCHED"):
                _N = cfg.PN_N * (1.0 + cfg.GAINSCHED_K
                                 * clamp(1.0 - t_go / max(cfg.PN_TGO_MAX, 1e-6),
                                         0.0, 1.0))
                _lead_kaynak = "pn+gs"
            else:
                _lead_kaynak = "pn"
            # PN lead = N_eff · λ̇_atalet · t_go  → LOS dönüş hızını sıfıra sürer.
            lead_az = clamp(_N * true_los_rate_az * t_go,
                            -math.radians(cfg.LEAD_MAX_DEG),
                            math.radians(cfg.LEAD_MAX_DEG))
        else:
            # NATIVE (VARSAYILAN, bit-aynı): nişanı atalet LOS dönüş hızıyla öne
            # al (bkz. Cfg.LEAD_SURE). lead_sure = LEAD_SURE·lead_olcek.
            lead_az = clamp(lead_sure * los_hiz[0],
                            -math.radians(cfg.LEAD_MAX_DEG),
                            math.radians(cfg.LEAD_MAX_DEG))
        # DUZTERM (kill-switch): terminal lead'i küçük tavana kap → PN'in ±25°
        # doyup işaret-değiştirme salınımını kes, hedefin arkasından düz git
        # (bkz. Cfg.DUZTERM_LEAD_MAX). KAPALI → lead_az değişmez (bit-aynı).
        if _ozellik(cfg, "duzterm", "DUZTERM"):
            _dlmax = cfg.DUZTERM_LEAD_MAX
            # CAPRAZLEAD (kill-switch): yandan geçen hedefte (|LOS açısal hızı|
            # yüksek) tavanı ek ile aç → burun öne alır, hedef terminalde kadrajdan
            # süpürülüp çıkmaz (kilit kopması + yanal ıska aynı kök; bkz. Cfg).
            # Kuyrukta (LOS hızı < eşik) ek=0 → 4° korunur, salınım koruması aynı.
            # KAPALI → _dlmax=DUZTERM_LEAD_MAX (bit-aynı).
            if _ozellik(cfg, "caprazlead", "CAPRAZLEAD"):
                _ek = clamp((abs(los_hiz[0]) - cfg.CAPRAZ_LOS_ESIK) * cfg.CAPRAZ_KAZANC,
                            0.0, cfg.CAPRAZ_EK_MAX)
                _dlmax = cfg.DUZTERM_LEAD_MAX + _ek
                _lead_kaynak = _lead_kaynak + "+capraz"
            _dl = math.radians(_dlmax)
            lead_az = clamp(lead_az, -_dl, _dl)
            _lead_kaynak = _lead_kaynak + "+duz"
    # COMMIT (kill-switch): son t_go penceresinde küçük yaw hatasını YOK SAY →
    # burun son vektöre taahhüt eder, churn/fly-past titremesi kesilir. Yalnız
    # YATAY kanal; büyük hata hâlâ düzeltilir. KAPALI → eps_yaw_cmd = eps_yaw
    # (bit-aynı). Terminal + t_go ≤ COMMIT_TGO + |eps_yaw| < ölü bant koşulu.
    eps_yaw_cmd = eps_yaw
    _commit_aktif = (terminal and t_go_term is not None
                     and _ozellik(cfg, "commit", "COMMIT")
                     and t_go_term <= cfg.COMMIT_TGO)
    if _commit_aktif and abs(eps_yaw) < math.radians(cfg.COMMIT_DEADBAND_DEG):
        eps_yaw_cmd = 0.0
        _lead_kaynak = _lead_kaynak + "+commit"
    yaw_cmd = normalize_angle(iris_yaw + cfg.K_YAW * eps_yaw_cmd + lead_az)

    # HIZ: kutu boyutu hatası üzerinden PI (terminalde TAM taahhüt)
    hata = cfg.BOYUT_REF - boyut               # px; + = uzak
    hiz_I = clamp(hiz_I + cfg.K_I * hata * dt, cfg.I_MIN, cfg.I_MAX)
    if terminal:
        # TBOOST (kill-switch): hücum hızını yükselt (bkz. Cfg.TBOOST_VTERM).
        # KAPALI → V_TERMINAL (bit-aynı). Dikey-bütçe kısıtı aşağıda yine geçerli.
        v_los = (cfg.TBOOST_VTERM if _ozellik(cfg, "tboost", "TBOOST")
                 else cfg.V_TERMINAL)         # hücum: fren yok, sabit hız
    else:
        v_los = clamp(hiz_I + cfg.K_FWD * hata, cfg.V_MIN, cfg.V_TOPLAM_MAX)

    # SAF TAKİP: tüm hız LOS/burun yönünde
    vx_ned = v_los * math.cos(yaw_cmd)
    vy_ned = v_los * math.sin(yaw_cmd)

    eps_elev = math.atan((cy - cfg.CY_NISAN) / geo.FY)   # cy büyük → hedef altta
    if terminal:
        # KESİŞİM: hız vektörü hedefe DOĞRU baksın (tutuş ofseti değil).
        # elev_atalet = gövde LOS yükselişi + gövde pitch; lead ile öne alınır.
        elev_atalet = piksel_elev(cy, cfg) + iris_pitch
        lead_el = clamp(lead_sure * los_hiz[1],
                        -math.radians(cfg.LEAD_MAX_DEG),
                        math.radians(cfg.LEAD_MAX_DEG))
        nisan_elev = clamp(elev_atalet + lead_el,
                           -math.radians(60.0), math.radians(60.0))
        # VLEAD (kill-switch): terminalde hedefin ÜSTÜNE sabit açı payı
        # (yükselen hedef + 25° kamera tilt telafisi; bkz. Cfg.VLEAD_DEG).
        # KAPALI → nisan_elev değişmez (bit-aynı native). Yalnız terminal.
        if _ozellik(cfg, "vlead", "VLEAD"):
            nisan_elev = clamp(nisan_elev + math.radians(cfg.VLEAD_DEG),
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
            _kap_olcek = clamp(kapanma, cfg.KAPANMA_MIN, max(cfg.KAPANMA_MIN, v_los))
            # HARMAN (bkz. Cfg.DIK_ORAN): 0=kapanma (bit-aynı), 1=yer hızı.
            v_dikey = _kap_olcek + cfg.DIK_ORAN * (v_los - _kap_olcek)
        t_ = abs(math.tan(nisan_elev))
        if t_ > 1e-6 and v_dikey * t_ > cfg.VZ_MAX_TERM:
            v_los = max(cfg.V_TERM_MIN, cfg.VZ_MAX_TERM / t_)
            vx_ned = v_los * math.cos(yaw_cmd)
            vy_ned = v_los * math.sin(yaw_cmd)
        vz_nisan = -v_dikey * math.tan(nisan_elev)
        # TÜREV SÖNÜMLEMESİ: aracın kendi dikey hızı nişanın ötesine geçtiyse
        # komut geri çekilir → hedefin üstünden geçme biter (bkz. Cfg.K_VZ_D).
        vz = clamp(vz_nisan + cfg.K_VZ_D * (vz_nisan - iris_vz),
                   -cfg.VZ_MAX_TERM, cfg.VZ_MAX_TERM)
    else:
        # TUTUŞ (değişmedi): hedefi CY_NISAN'da tut
        vz = clamp(cfg.K_VZ * cfg.V_NOM * eps_elev, -cfg.VZ_MAX, cfg.VZ_MAX)

    tani = {"boyut": boyut, "eps_yaw": eps_yaw, "eps_elev": eps_elev,
            "hata": hata, "v_los": v_los, "terminal": terminal,
            "lead_az": lead_az, "lead_olcek": lead_olcek,
            "lead_kaynak": _lead_kaynak}
    return vx_ned, vy_ned, vz, yaw_cmd, hiz_I, tani


def _besleme_nisan(cx, cy, boyut, kapanma, terminal, est, cfg, coast=False):
    """komut()'a beslenecek NİŞAN pikselini (cx_aim, cy_aim) seç — YASA DIŞI.

    Kutu yasasını (komut) HİÇ değiştirmez; yalnız hangi (cx,cy)'nin besleneceğini
    belirler. Yollar (öncelik sırasıyla):
      1. TERMİNAL + INTERCEPT açık + menzil < INTERCEPT_MENZIL → KESİŞİM ofseti
         (hedefin GELECEK pikseli). Hem yaw hem dikey hedefin gideceği yere sürer.
      2. (coast DEĞİLKEN) Normal takip + PRED açık → küçük görüntü-LEAD ofseti.
      3. Aksi halde → ham (cx,cy) (bit-aynı native/coast).
    INTERCEPT KAPALIYKEN yol 1 hiç seçilmez.

    coast=True: kör-hücum/boşluk yolu. Bu yolda PRED zaten konumu ileri-tahmin
    ettiği için AYRICA lead_ofset EKLENMEZ (yol 2 atlanır) — INTERCEPT kapalı
    coast davranışı bit-aynı kalır (yalnız INTERCEPT açıkken kesişim eklenir).

    Döner: (cx_aim, cy_aim, kaynak) — kaynak ∈ {"native","pred_lead","intercept"}.
    """
    if est is None or not _ozellik(cfg, "pred", "PRED") or not est.hazir():
        return cx, cy, "native"
    # 1) TERMİNAL KESİŞİM (yalnız INTERCEPT açık + yakın menzil)
    if terminal and _ozellik(cfg, "intercept", "INTERCEPT") and boyut and boyut > 1e-6:
        menzil = cfg.MENZIL_PX_M / boyut
        if menzil < cfg.INTERCEPT_MENZIL:
            dcx, dcy, _tgo = est.intercept_ofset(boyut, kapanma)
            return cx + dcx, cy + dcy, "intercept"
    if coast:
        # coast: konum zaten ileri-tahminli; lead_ofset EKLENMEZ (bit-aynı).
        return cx, cy, "native"
    # 2) NORMAL TAKİP görüntü-LEAD (INTERCEPT devre dışıysa terminalde de bu
    #    yola düşülür — INTERCEPT kapalı canlı-yol davranışını bit-aynı korur).
    _dcx, _dcy = est.lead_ofset()
    return cx + _dcx, cy + _dcy, "pred_lead"


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
                  kayip_kare_esik=20, ff_hiz=(0.0, 0.0, 0.0), get_temas=None,
                  get_gorev_state=None):
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
    iyaw_onceki = None        # PN_EGO kancası için drone yaw'ı (opsiyonel)
    # ── KESTİRİM + COAST (bkz. Cfg.PRED) ──
    # est: görüntü-düzlemi (cx,cy) sabit-hız kestiricisi. Kutu boşluğunda
    # (özellikle kör hücum) donmuş komut yerine ileri-tahmin edilen hedefi
    # komut()'a besler. son_gercek_t: son GERÇEK tespitin monotonic anı —
    # coast ufku (PRED_MAXS) bundan ölçülür. son_boyut: coast sırasında komut
    # üretmek için son geçerli kutu boyutu (kapanma/terminal ölçeği için).
    est = HedefKestirim(cfg)
    son_gercek_t = None       # son geçerli kutunun monotonic anı
    son_bw = son_bh = 0.0     # son geçerli kutu genişlik/yükseklik (coast komutu)

    def _vuruldu():
        if get_temas is None:
            return False
        return get_temas() is True

    def _pred_coast_cmd(iris, iyaw, ipitch, dt, now):
        """PRED coast: kutu yokken ileri-tahminle CANLI komut üret (donmuş değil).

        Estimator'ı ilerletir, tahmin edilen (cx,cy)'yi komut()'a besler ve
        ana yoldakiyle AYNI yaw-slew + ivme sınırını uygular. Yeni komut
        dörtlüsünü döner; None dönerse çağıran donmuş davranışa düşer.
        Paylaşılan limitleyici/yaw durumunu (nonlocal) günceller — böylece
        ardışık coast kareleri sürekli ilerler.
        Kutu YASASINI değiştirmez: yalnızca (cx,cy) beslemesi tahminden gelir.
        """
        nonlocal vx_p, vy_p, vz_p, cmd_yaw, hiz_I
        if not _ozellik(cfg, "pred", "PRED") or not est.hazir():
            return None
        # coast ufku: son GERÇEK tespitten bu yana geçen süre PRED_MAXS'i aşarsa
        # tahmine güvenme (donmuş komuta düş).
        if son_gercek_t is not None and (now - son_gercek_t) > cfg.PRED_MAXS:
            return None
        ileri = est.tahmin_ileri(dt)
        if ileri is None:
            return None
        cx_p, cy_p = ileri
        # NİŞAN BESLEMESİ (ana yolla aynı kural): kör hücumda terminal +
        # INTERCEPT + yakın menzil ise coast edilen konumun ÜSTÜNE kesişim
        # ofseti eklenir — donmuş-düz yerine hedefin GELECEK yerine eğrilir.
        # INTERCEPT kapalıysa yol saf coast (tahmin_ileri) kalır: bit-aynı.
        _boyut_coast = math.sqrt(max(son_bw, 0.0) * max(son_bh, 0.0))
        cx_p, cy_p, _ = _besleme_nisan(
            cx_p, cy_p, _boyut_coast, kapanma, terminal_mandal, est, cfg,
            coast=True)
        # komut(): terminal durumu KORUNUR (kör hücumda terminal yasası sürer).
        # Kutu boyutu son geçerli kutudan (kapanma/dikey ölçek için); los_hiz/
        # kapanma canlı döngü durumundan. hiz_I DEĞİŞMEZ (coast'ta integrali
        # sürdürme; hata sinyali gerçek değil) — yerel kopyayla çağrılır.
        vx, vy, vz, yaw_hedef, _I_yeni, _t = komut(
            cx_p, cy_p, son_bw, son_bh, iyaw, hiz_I, dt, cfg,
            terminal_mandal, tuple(los_hiz), ipitch,
            float(iris.get("vz", 0.0) or 0.0), kapanma, 0.0)
        # YAW SLEW (ana yolla aynı): burun sınırla, hız yönü yaw_hedef'ten.
        _cy = cmd_yaw if cmd_yaw is not None else iyaw
        yaw_err = normalize_angle(yaw_hedef - _cy)
        adim = clamp(yaw_err, -cfg.YAW_RATE_MAX * dt, cfg.YAW_RATE_MAX * dt)
        _cy = normalize_angle(_cy + adim)
        # hız yönünü slew'lenmiş yaw'a göre yeniden kur (ana yol yaw_cmd'yi
        # komut içinde kullanır; burada komut yaw_hedef'i döndürdüğü için
        # slew sonrası yönü tazele).
        v_yatay = math.hypot(vx, vy)
        vx = v_yatay * math.cos(_cy)
        vy = v_yatay * math.sin(_cy)
        cmd_yaw = _cy
        vx, vy, vz = limit_acceleration(vx, vy, vz, vx_p, vy_p, vz_p,
                                        cfg.MAX_ACCEL, dt)
        vx_p, vy_p, vz_p = vx, vy, vz
        return (vx, vy, vz, _cy)

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
                    # KÖR UFUK: INTERCEPT açıkken 37 m fly-past'ı kesmek için
                    # ufuk INTERCEPT_KOR_MAXS ile SINIRLANIR (yalnız kısaltır:
                    # TERMINAL_SURE ile min). Kör faz bitince 'kayip' → GPS
                    # yeniden yaklaşır; kısa ufuk churn'ü büyütmez çünkü
                    # INTERCEPT açıkken hedefe DOĞRU eğrildiğimiz için daha az
                    # tespit boşluğu/coast gerekir. INTERCEPT KAPALI → ufuk
                    # TERMINAL_SURE (bit-aynı native).
                    _kor_ufuk = cfg.TERMINAL_SURE
                    if _ozellik(cfg, "intercept", "INTERCEPT"):
                        _kor_ufuk = min(cfg.TERMINAL_SURE, cfg.INTERCEPT_KOR_MAXS)
                    if kor_baslangic is None:
                        kor_baslangic = time.time()
                        print(f"[IBVS] kör hücum başladı — {_kor_ufuk:.1f} s "
                              f"içinde temas gelmezse ıska")
                    gecen = time.time() - kor_baslangic
                    if gecen >= _kor_ufuk:
                        print(f"[IBVS] kör hücum {gecen:.1f} s sürdü, temas yok "
                              f"→ ISKA, 'kayip' (GPS'e dönülüyor)")
                        return "kayip"
                    # PRED AÇIK: donmuş komut yerine TAHMİN edilen hedefe nişanla
                    # (kör hücumda crosser yana kaçarken hedefin GİTTİĞİ yeri
                    # sürdür). PRED KAPALI ya da tahmin süresi dolduysa → donmuş
                    # son_v_cmd (native davranış, bit-aynı).
                    _pred = _pred_coast_cmd(iris, iyaw, iris.get("pitch", 0.0),
                                            dt, now)
                    _durum_kor = "TERM_KOR"
                    if _pred is not None:
                        son_v_cmd = _pred
                        _durum_kor = "TERM_PRED"
                    if son_v_cmd is not None:
                        send_velocity(conn, *son_v_cmd)
                    w_csv.writerow({"t": round(now, 3), "dt": round(dt, 4),
                                    "durum": _durum_kor,
                                    "kayip_sayac": kayip_sayac,
                                    "vx_cmd": round(son_v_cmd[0], 2) if son_v_cmd else None,
                                    "vy_cmd": round(son_v_cmd[1], 2) if son_v_cmd else None,
                                    "vz_cmd": round(son_v_cmd[2], 2) if son_v_cmd else None,
                                    "iris_yaw_deg": round(math.degrees(iyaw), 1)})
                    f.flush()
                    continue
                if kayip_sayac >= kayip_kare_esik:
                    print(f"[IBVS] {kayip_kare_esik} ardışık kutusuz kare → 'kayip'")
                    return "kayip"
                # Kutu yok: SON KOMUT sürdürülür (hedefin seyri bir karede
                # değişmez). Sıfır komut vermek kısa bir tespit boşluğunu
                # kalıcı kayba çevirir. İntegral dokunulmaz (bozulmasın).
                # PRED AÇIK: donmuş komut yerine TAHMİN edilen hedefe köprüle
                # (kısa tespit boşluğunda hedef hareketini sürdür). KAPALI ya da
                # tahmin süresi dolduysa → native (donmuş son_v_cmd), bit-aynı.
                _pred = _pred_coast_cmd(iris, iyaw, iris.get("pitch", 0.0),
                                        dt, now)
                _durum_yok = "KUTU_YOK"
                if _pred is not None:
                    son_v_cmd = _pred
                    _durum_yok = "PRED"
                if son_v_cmd is not None:
                    send_velocity(conn, *son_v_cmd)
                else:
                    send_velocity(conn, vx_p, vy_p, vz_p, cmd_yaw or iyaw)
                w_csv.writerow({"t": round(now, 3), "dt": round(dt, 4),
                                "durum": _durum_yok, "kayip_sayac": kayip_sayac,
                                "vx_cmd": round(son_v_cmd[0], 2) if son_v_cmd else None,
                                "vy_cmd": round(son_v_cmd[1], 2) if son_v_cmd else None,
                                "vz_cmd": round(son_v_cmd[2], 2) if son_v_cmd else None,
                                "iris_yaw_deg": round(math.degrees(iyaw), 1)})
                f.flush()
                continue

            kayip_sayac = 0
            kor_baslangic = None       # kutu geri geldi → kör sayaç sıfırlanır
            cx, cy, bw, bh, conf = kutu

            # ── KESTİRİM GÜNCELLE + COAST DURUMU (bkz. Cfg.PRED) ──
            # Her geçerli kutuda görüntü-düzlemi (cx,cy) kestiricisini güncelle
            # ve son gerçek tespit anını/kutu boyutunu sakla — kutu boşluğunda
            # coast komutu bunları kullanır. PRED KAPALI iken bu tamamen atıldır:
            # est kullanılmaz, komut girdisi ham (cx,cy) kalır (bit-aynı native).
            if _ozellik(cfg, "pred", "PRED"):
                est.guncelle(cx, cy, dt)
            son_gercek_t = now
            son_bw, son_bh = bw, bh

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

            # ── DRONE YAW HIZI (yalnız PN_EGO kancası için, bkz. Cfg.PN) ──
            # los_hiz[0] ZATEN ego-temiz olduğundan bu normalde KULLANILMAZ;
            # yaw_rate=0 komuta gider. PN_EGO açıksa komut() yaw_rate'i los_hiz[0]'
            # dan çıkarır (çift-düzeltme; yalnız A/B doğrulaması için).
            yaw_rate = 0.0
            if iyaw_onceki is not None and 1e-3 < dt < 0.5:
                yaw_rate = normalize_angle(iyaw - iyaw_onceki) / dt
            iyaw_onceki = iyaw

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
            # TERMİNAL MANDALI: hücuma taahhüt, geri dönüş yok.
            # ŞARTNAME KAPISI (bkz. dosya başı): AÇIK + get_gorev_state varsa,
            # terminal YALNIZ FSM STRIKE'ta tetiklenir — box-size şartı ARANMAZ,
            # çünkü FSM zaten 5 sn kümülatif + 3 sn kesintisiz kilidi garanti etti
            # (tespit/takip/angajman disiplini). KAPALI/None → native (kutu eşiği).
            _kapi_aktif = _SARTNAME_KAPI and _State is not None and get_gorev_state is not None
            _box_yakin = (math.sqrt(bw * bh) >= cfg.TERMINAL_BOYUT)
            if _kapi_aktif:
                # İKİSİ BİRDEN (2026-08-09 uçuş bulgusu): FSM STRIKE'ı TEK BAŞINA
                # yeterli sayınca terminal, hedef 139px merkez-dışı + box 18px
                # (uzak) iken tetikleniyordu → bbox_ibvs terminali o geometriden
                # yakınsayamıyor, fly-past. Box eşiği de şart: ENGAGE'de bbox_ibvs
                # merkezleyip yakınlaşsın (box→TERMINAL_BOYUT), SONRA STRIKE terminali.
                _terminal_ok = (get_gorev_state() == _State.STRIKE and _box_yakin)
            else:
                _terminal_ok = _box_yakin
            if not terminal_mandal and _terminal_ok:
                terminal_mandal = True
                print(f"[IBVS] ⚡ TERMİNAL HÜCUM (kutu {math.sqrt(bw*bh):.0f}px, "
                      f"kapı={'FSM-STRIKE' if _kapi_aktif else 'native>=%d px' % cfg.TERMINAL_BOYUT}) "
                      f"— fren yok, tam taahhüt")
            # ── NİŞAN BESLEMESİ (bkz. Cfg.PRED / Cfg.INTERCEPT) ──
            # komut()'a HANGİ (cx,cy)'nin besleneceğini _besleme_nisan seçer
            # (yasa değişmez, yalnız besleme):
            #   • TERMİNAL + INTERCEPT + yakın menzil → KESİŞİM (hedefin gelecek
            #     pikseli; hem yaw hem dikey öne alınır).
            #   • Normal takip + PRED → küçük görüntü-LEAD ofseti (PN'i ezmez).
            #   • INTERCEPT/PRED KAPALI → ham (cx,cy), bit-aynı native.
            cx_giris, cy_giris, _besleme_kaynak = _besleme_nisan(
                cx, cy, boyut_simdi, kapanma, terminal_mandal, est, cfg)
            vx, vy, vz, yaw_hedef, hiz_I, tani = komut(cx_giris, cy_giris, bw, bh,
                                                       iyaw, hiz_I, dt, cfg,
                                                       terminal_mandal,
                                                       tuple(los_hiz), ipitch,
                                                       float(iris.get("vz", 0.0) or 0.0),
                                                       kapanma, yaw_rate)
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
                "los_hiz_az": round(los_hiz[0], 3), "los_hiz_el": round(los_hiz[1], 3),
                "vx_cmd": round(vx, 2), "vy_cmd": round(vy, 2),
                "vz_cmd": round(vz, 2),
                "yaw_cmd_deg": round(math.degrees(yaw_cmd), 1),
                "kayip_sayac": 0,
                "lead_kaynak": tani.get("lead_kaynak", "native"),
                "yaw_rate_deg": round(math.degrees(yaw_rate), 1),
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
