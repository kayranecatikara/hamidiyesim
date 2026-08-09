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

    LEAD_SURE = _env_f("AVCI_IBVS_LEAD", 0.4)    # s; nişanın öne alınma süresi
    LEAD_EMA = 0.25                              # LOS hızı yumuşatması
    LEAD_MAX_DEG = 25.0                          # °; lead açısı tavanı
    VZ_MAX_TERM = _env_f("AVCI_IBVS_VZT", 5.0)   # m/s; terminalde dikey tavan
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
    "boyut_hata", "hiz_I", "v_los", "lead_az_deg", "los_hiz_az", "los_hiz_el",
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


def komut(cx, cy, w, h, iris_yaw, hiz_I, dt, cfg=Cfg, terminal=False,
          los_hiz=(0.0, 0.0), iris_pitch=0.0):
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
    lead_az = 0.0
    if terminal:
        # LEAD: nişanı atalet LOS dönüş hızıyla öne al (bkz. Cfg.LEAD_SURE)
        lead_az = clamp(cfg.LEAD_SURE * los_hiz[0],
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
    if terminal:
        # KESİŞİM: hız vektörü hedefe DOĞRU baksın (tutuş ofseti değil).
        # elev_atalet = gövde LOS yükselişi + gövde pitch; lead ile öne alınır.
        elev_atalet = piksel_elev(cy, cfg) + iris_pitch
        lead_el = clamp(cfg.LEAD_SURE * los_hiz[1],
                        -math.radians(cfg.LEAD_MAX_DEG),
                        math.radians(cfg.LEAD_MAX_DEG))
        nisan_elev = clamp(elev_atalet + lead_el,
                           -math.radians(60.0), math.radians(60.0))
        vz = clamp(-v_los * math.tan(nisan_elev),
                   -cfg.VZ_MAX_TERM, cfg.VZ_MAX_TERM)
    else:
        # TUTUŞ (değişmedi): hedefi CY_NISAN'da tut
        vz = clamp(cfg.K_VZ * cfg.V_NOM * eps_elev, -cfg.VZ_MAX, cfg.VZ_MAX)

    tani = {"boyut": boyut, "eps_yaw": eps_yaw, "eps_elev": eps_elev,
            "hata": hata, "v_los": v_los, "terminal": terminal,
            "lead_az": lead_az}
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
            los_az = normalize_angle(iyaw + math.atan((cx - cfg.CX_NISAN) / geo.FX))
            los_el = piksel_elev(cy, cfg) + ipitch
            if los_az_onceki is not None and 1e-3 < dt < 0.5:
                a_ = cfg.LEAD_EMA
                los_hiz[0] = (a_ * (normalize_angle(los_az - los_az_onceki) / dt)
                              + (1 - a_) * los_hiz[0])
                los_hiz[1] = (a_ * ((los_el - los_el_onceki) / dt)
                              + (1 - a_) * los_hiz[1])
            los_az_onceki, los_el_onceki = los_az, los_el
            # TERMİNAL MANDALI: kutu eşiği aşınca hücuma taahhüt, geri dönüş yok
            if not terminal_mandal and math.sqrt(bw * bh) >= cfg.TERMINAL_BOYUT:
                terminal_mandal = True
                print(f"[IBVS] ⚡ TERMİNAL HÜCUM (kutu {math.sqrt(bw*bh):.0f}px "
                      f"≥ {cfg.TERMINAL_BOYUT:.0f}) — fren yok, tam taahhüt")
            vx, vy, vz, yaw_hedef, hiz_I, tani = komut(cx, cy, bw, bh, iyaw,
                                                       hiz_I, dt, cfg,
                                                       terminal_mandal,
                                                       tuple(los_hiz), ipitch)
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
