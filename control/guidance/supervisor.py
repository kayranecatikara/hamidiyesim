"""
supervisor.py — Faz 4: GPS ↔ görsel güdüm geçişi (hibrit müdahale).

run_hybrid tek görev döngüsüdür (start_chase bunu çalıştırır):

  GPS fazı (gps_guidance) hedefe yaklaşır. Görsel temas oturunca
  (KILIT_PENCERE karenin KILIT_N'inde tespit, conf ≥ POSE_CONF_MIN, VE handoff
  menzili içindeyiz YA DA GPS düşmüş/DROPOUT) → GÖRSEL faza (visual_lead)
  geçilir. Görsel temas kesilirse ya da hedef GEÇİLİRSE (B5 fly-past) → GPS
  fazına dönülür. stop_chase gelene (veya araç vurulana) kadar bu döngü sürer.

  ── 2026-08-06: kilit sinyali pose değil DETECTION güveni ──
  Pose modeli kaldırıldı; "görsel kilit" artık kutunun güvenidir. Eşik adı
  (POSE_CONF_MIN) tarihsel olarak korundu — aynı sayı, aynı anlam.

Menzil kapısının (GATE_KILIT) nedeni: görsel fazın kapanma hızı sabit
(V_KAPANMA); uzaktan erken geçilirse hızlı hedefe yetişilemez. GPS handoff
histerezisi (≤40 m) zaten "yetişilmiş" durumu işaretler. GPS jam/DROPOUT'ta
menzil bilinemez → görsel temas tek başına yeter (jamming fallback).

GT modunda pose kilidini atlamak MÜMKÜNDÜR (`AVCI_GT_KILIT_BYPASS=on`) ama
VARSAYILAN KAPALIDIR — ölçümle çürütüldü, bkz. SupCfg.GT_KILIT_BYPASS.
"""

import math
import os
import threading
import time

from control.guidance import gps_guidance as _ga
from control.guidance.gps_guidance import run_gps_guidance
from control.guidance.guidance_core import Cfg as LeadCfg
from control.guidance.visual_lead import run_visual_lead
from control.guidance.bbox_ibvs import (run_bbox_ibvs as _run_bbox_ibvs,
                                        Cfg as _IbvsCfg)
from control.guidance.common import send_velocity
from control import menzil_tutucu as _tutucu
from control.mission_fsm import State
from control.yaklasma_kontrol import YaklasmaKontrol
from vision.detection_state import (get_kilit_durum, get_gorev_state,
                                    set_yaklasma_karar)

# ── Güdüm YÜRÜTÜCÜSÜ = görev FSM durumunun türevi (tek karar kaynağı) ──
# GPS yürütücüsü: SEARCH/APPROACH + DETECT/TRACK_LOCK + TRACK_LOST.
#   DETECT/TRACK_LOCK neden GPS? "Mesafeyi koru" HAREKETLİ hedefte SIFIR HIZ
#   DEĞİL, hedefin hızıyla eşleşen (bağıl kapanma=0) hızdır. GPS istasyon-tutma
#   hedefin pozunu/hızını bilir → sabit menzili (~10-11 m) korur ve hedefi kadraj
#   merkezine oturtur (açısal hizalama). visual_lead'in kapanma=0 klempi hız
#   vektörünü tamamen sıfırlayıp aracı durduruyordu → hedef uzaklaşıp kilit
#   birikemiyordu (bkz. 2026-08-07 saha bulgusu). Kümülatif/kesintisiz kilit
#   kamera thread'inde birikmeye devam eder; güdüm GPS'te kalır.
# Görsel yürütücü: YALNIZ ENGAGE (kapanma <= V_MAX_ENGAGE) ve STRIKE (tam
#   lead-pursuit) — kapanma klempi visual_lead içinde FSM durumuna göre.
_GPS_SET = frozenset({State.SEARCH, State.APPROACH, State.DETECT,
                      State.TRACK_LOCK, State.TRACK_LOST})
_GORSEL_SET = frozenset({State.ENGAGE, State.STRIKE})

# GÖRSEL FAZ ALGORİTMASI (2026-08-09): FSM görsel fazına (ENGAGE/STRIKE) girince
# hangi görsel güdüm yürütücüsü koşar?
#   bbox_ibvs (VARSAYILAN) → kayramin IBVS algoritması (dikey-ıska çözümlü);
#                            terminal FSM STRIKE'a kapılı (3-faz disiplini korunur).
#   visual_lead            → eski (benim) PN + co-altitude yürütücüm (A/B için).
# 3 fazlı FSM YAPISI HER İKİSİNDE de aynı sürer; değişen yalnız fazların İÇİNDEKİ
# görsel algoritma. bbox_ibvs kutu akışına wait_pose adaptörüyle bağlanır.
_GORSEL_YASA = os.environ.get("AVCI_GORSEL_YASA", "bbox_ibvs").strip().lower()


class SupCfg:
    # ── DEVİR KAPISI: ARDIŞIK → KAYAN PENCERE (2026-07-31) ──
    # Eskiden KILIT_N ARDIŞIK güvenli kare aranıyordu. Tespit gürültülü olduğu
    # için (gerçek uçuşlarda karelerin yalnız %12'si temiz `ok`) bu şart çok geç
    # sağlanıyordu: devir kapısı 20 m'ye ayarlı olmasına rağmen görsel faz
    # 6-10 m'de başlıyordu ve elinde 0.6-1.9 s kalıyordu — hedefin 4.65 m altında
    # devralınan dikey farkı kapatmaya yetmiyor.
    # Kayan pencere aynı güveni verir ama tek bir kötü kare sayacı sıfırlamaz.
    # ⚠ 10 → 7 DENENDİ VE GERİ ALINDI (2026-08-02). DÜŞÜRMEYİN.
    #
    # Gerekçe iyiydi: A5 sonrası 17 geçişte vuranlar görsel faza medyan
    # 11.11 m'de, ıskalayanlar 9.05 m'de girmişti. Kapıyı gevşetmek devri
    # uzaklaştırıp terminale daha çok tırmanma süresi bırakacaktı.
    #
    # Ölçüm bunu ÇÜRÜTTÜ — her ölçütte kötüleşti:
    #
    #                        KILIT_N=10 (5 uçuş)   KILIT_N=7 (1 uçuş)
    #   faz / uçuş                  3.4                  8.0
    #   giriş menzili medyan      10.00 m               9.62 m   ← DÜŞTÜ
    #   en yakın menzil medyan     1.73 m               2.08 m
    #   kor_dalis medyan            %19                  %27
    #   <1.5 s'de kopan faz        2/17                  4/8
    #   vuruş                      3/17                 1/8
    #
    # MEKANİZMA: kapı cılız tespitte de açılıyor. Erken devirler gerçekten
    # oluyor (14.73 m, 10.47 m'de girdi) ama hemen ölüyor — o iki faz 0.9 ve
    # 1.3 s sürdü, birinde kareler %69 kör_dalış, diğerinde %100 tespit_yok.
    # Faz KAYIP_M yiyip GPS'e dönüyor, drone bu arada yaklaşmış oluyor, bir
    # sonraki devir DAHA YAKINDA gerçekleşiyor. Net etki ters.
    #
    # Yani devir menzili ile vuruş arasındaki bağıntı nedensel DEĞİL: ikisi de
    # "tespit o an gerçekten sağlam mı"ya bağlı. Kapıyı gevşetmek sağlamlığı
    # üretmiyor, sadece sağlam sanılan anları çoğaltıyor.
    # Asıl kaldıraç terminal algı sürekliliği (vuran 4 geçişin dördünde de
    # kor_dalis ≤ %3) — bkz. DURUM.md B6.
    KILIT_N = int(os.environ.get("AVCI_HYBRID_KILIT_N", 10))
    KILIT_PENCERE = 15    # kayan pencere boyu (~0.5 s @30 Hz)
    KAYIP_M = 20          # ardışık tespitsiz kare → GPS'e dön (~0.66 s)
    POSE_CONF_MIN = 0.5
    GATE_KILIT = True     # geçiş için menzil kapısı (VEYA GPS DROPOUT — jamming)
    # Devir menzili: GPS handoff bayrağı 40 m'de açılıyor ama orada kutu ~7 px,
    # pose güvenilmez (uzakta devralınca hedef hemen kaçtı — 2026-07-24 log).
    # 20 m'de kutu ~7 px hâlâ küçük; pose asıl 10-12 m'de sağlam. GPS istasyonu
    # 10 m; kapı 20 → GPS yaklaşırken pose kilidini bu banda çeker.
    GATE_MENZIL = float(os.environ.get("AVCI_HYBRID_GATE_MENZIL", 20.0))

    # ── GT MODUNDA GÖRSEL KİLİDİ ATLA — DENENDİ VE GERİ ALINDI (2026-08-04) ──
    # Gerekçe mantıklıydı: GT modunda güdüm pose'a bakmıyor, o hâlde geçişi
    # pose'un tutması anlamsız. Kilit sinyali "GT akışı canlı mı"ya çevrildi.
    #
    # ÖLÇÜM ÇÜRÜTTÜ (uçuş 164352 = kilit VAR, 172103 = kilit YOK, ikisi de GT):
    #
    #                              kilit VAR    kilit YOK
    #   görsel faza giriş medyanı    6.6 m       19.6 m    ← kapıya yapıştı
    #   en yakın menzil              0.68 m       2.41 m
    #   GPS istasyonda oturma        33.7%         0.4%
    #   GPS kadraj yaw RMS           35.7°       116.8°
    #   biten faz                  3 ıska/4 kayıp  13/13 KAYIP
    #
    # MEKANİZMA: pose kilidi farkında olmadan bir GECİKME görevi görüyormuş.
    # Kilit ~6 m'de oturuyor, devir orada oluyordu. Kilit kalkınca devir 20 m
    # kapısına yapıştı; görsel faz hedefe yetişemeyeceği menzilde devralıp
    # hemen kaybediyor. Dahası GPS fazı artık istasyonuna hiç oturamıyor
    # (%33.7 → %0.4) — 20 m'de devir alındığı için 8-12 m bandına hiç girmiyor.
    # SupCfg başındaki KILIT_N=7 deneyi de aynı sonucu vermişti: kapıyı
    # gevşetmek sağlamlık üretmiyor, sağlam sanılan anları çoğaltıyor.
    #
    # Açmadan önce V_KAPANMA'yı düşürmek gerekir (bkz. TODO.md §1): 25 m/s'te
    # 20 m'den devralmanın düzeltme bütçesi zaten yok.
    GT_KILIT_BYPASS = os.environ.get(
        "AVCI_GT_KILIT_BYPASS", "off").lower() in ("on", "1", "true")




# Telemetri/arayüz için son durum (gcs_server okur; salt gözlem)
status = {"faz": "GPS", "gecis_sayisi": 0, "kilit_sayac": 0, "son_sebep": None}


def _kopru(parent_event, child_event):
    """parent set olunca child'ı da set eder (faz thread'i ana stop'u duysun)."""
    def izle():
        while not parent_event.is_set() and not child_event.is_set():
            parent_event.wait(0.5)
        if parent_event.is_set():
            child_event.set()
    threading.Thread(target=izle, daemon=True).start()


def _geri_cek(conn, get_plane, get_iris, hiz):
    """Hedeften UZAĞA yatay geri-çekilme (NED), yaw hedefe dönük (kamera hedefi
    görmeye devam etsin). gps_guidance'a DOKUNMAZ — ayrı send_velocity.
    r_eff=min(menzil,RANGE_SET) RANGE_SET ile uzaklaştıramadığı için gereklidir."""
    p, i = get_plane(), get_iris()
    if p is None or i is None:
        return
    dx, dy = i["x"] - p["x"], i["y"] - p["y"]        # hedef→drone (uzaklaşma yönü)
    n = math.hypot(dx, dy)
    if n < 1e-6:
        return
    yaw = math.atan2(p["y"] - i["y"], p["x"] - i["x"])   # drone→hedef (kamera dönük)
    send_velocity(conn, hiz * dx / n, hiz * dy / n, 0.0, yaw)


def _oran_regulasyon(conn, get_plane, get_iris, regul_stop, hz=20.0):
    """APPROACH/TRACK_LOCK'ta hedefin karedeki ORANINI ORAN_SETPOINT'e regüle
    eder (menzil değil oran). Yaklaşma: gps_guidance.Cfg.RANGE_SET'i düşürür
    (gps kapanır). Geri çekilme: _geri_cek (sarmalayıcı send_velocity). Oran
    HAM'ın EMA'sından ölçülür (kilit hattı ham kalır; bkz. #2/#3). Kararı log
    için yayınlar. gps_guidance ile eşzamanlı koşar; geri çekilirken RANGE_SET
    büyütülür → gps ~0 tutar, geri komut baskın olur."""
    yk = YaklasmaKontrol()
    periyot = 1.0 / hz
    son_t = time.time()
    while not regul_stop.is_set():
        simdi = time.time()
        dt, son_t = simdi - son_t, simdi
        st = get_gorev_state()
        aktif = st in (State.APPROACH, State.TRACK_LOCK)
        kd = get_kilit_durum() or {}
        kx, ky = kd.get("kaplama_x"), kd.get("kaplama_y")
        bbox_var = (kd.get("ah_kutu") is not None) and (kx is not None)
        oran_ham = max(kx or 0.0, ky or 0.0) if bbox_var else None
        karar = yk.adim(oran_ham, bbox_var, _ga.status.get("menzil"),
                        _ga.Cfg.RANGE_SET, dt, aktif=aktif)
        if aktif:
            _ga.Cfg.RANGE_SET = karar.range_set          # setattr — dosyaya dokunmaz
            if karar.komut == "geri" and karar.geri_hiz > 0.0:
                _geri_cek(conn, get_plane, get_iris, karar.geri_hiz)
        set_yaklasma_karar(karar)
        regul_stop.wait(periyot)
    set_yaklasma_karar(None)


def run_hybrid(conn, get_plane, get_iris, wait_kare, get_plane_truth,
               stop_event, sup_cfg=SupCfg, lead_cfg=LeadCfg, get_temas=None,
               get_menzil=None, get_gt=None):
    # gecis_sayisi BURADA SIFIRLANMAZ (2026-08-05): gcs_server'ın güdüm modu
    # seçici döngüsü run_hybrid'i görev boyunca defalarca çağırıyor; burada
    # sıfırlamak sayacı her çağrıda 0'a düşürüp arayüzde hep boş gösteriyordu.
    # Sayaç GÖREV başına anlamlı → start_chase/start_visual sıfırlıyor.
    status.update(faz="GPS", kilit_sayac=0, son_sebep=None)
    # Marj geri beslemeli mesafe tutucu (Adım 7) — bir kez başlar; YALNIZ VISUAL'de
    # RANGE_SET'i ayarlar (gps_guidance.py'ye dokunmadan sınıf niteliği yazımıyla).
    _tutucu.calistir(
        stop_event,
        get_faz=lambda: status["faz"],
        get_marj=lambda: (get_kilit_durum() or {}).get("marj"),
        get_range=lambda: _ga.Cfg.RANGE_SET,
        set_range=lambda r: setattr(_ga.Cfg, "RANGE_SET", r),
    )
    # ── FSM-GÜDÜMLÜ DİSPATCH (2026-08-07) ──
    # Eski devir kapısı (KILIT_N ardışık/kayan pencere + menzil) KALDIRILDI:
    # "hedefi ilk gördüğü anda dalıyor" hatasının kaynağı buydu — kapı yalnız
    # "gördüm + yakınım"a bakıyor, kümülatif/kesintisiz kilidi BEKLEMİYORDU.
    # Artık YÜRÜTÜCÜ görev FSM durumunun türevi: FSM SEARCH/APPROACH/TRACK_LOST
    # iken GPS yaklaşma; DETECT'ten itibaren görsel (kapanma klempi visual_lead
    # içinde FSM durumuna göre). GATE_KILIT/KILIT_N/GT_KILIT_BYPASS artık kullanılmaz.
    while not stop_event.is_set():
        st = get_gorev_state()

        # ══ GPS YÜRÜTÜCÜSÜ ══ (FSM SEARCH/APPROACH/TRACK_LOST veya durum yok) ══
        if st is None or st in _GPS_SET:
            status["faz"] = "GPS"
            faz_stop = threading.Event()
            _kopru(stop_event, faz_stop)
            # Oran regülasyonu gps ile EŞZAMANLI koşar (APPROACH/TRACK_LOCK'ta
            # aktif): oranı ORAN_SETPOINT'e getirene dek yaklaş/geri çekil.
            regul_stop = threading.Event()
            _kopru(stop_event, regul_stop)
            threading.Thread(target=_oran_regulasyon,
                             args=(conn, get_plane, get_iris, regul_stop),
                             daemon=True).start()

            def gps_izci():
                # FSM görsel kümeye (ENGAGE/STRIKE) girince GPS döngüsünü kır.
                while not faz_stop.is_set():
                    s = get_gorev_state()
                    status["kilit_sayac"] = 0
                    if s in _GORSEL_SET:
                        faz_stop.set()
                        return
                    faz_stop.wait(0.05)

            threading.Thread(target=gps_izci, daemon=True).start()
            print(f"[SUPERVISOR] GPS yürütücüsü (FSM: {st.value if st else 'SEARCH'})"
                  f" — oran regülasyonu aktif; görsel faza ENGAGE'de geçilir")
            run_gps_guidance(conn, get_plane, get_iris, faz_stop)
            regul_stop.set()               # oran regülasyon thread'ini durdur
            if stop_event.is_set():
                break
            continue

        # ══ GÖRSEL YÜRÜTÜCÜ ══ (FSM DETECT/TRACK_LOCK/ENGAGE/STRIKE) ══
        status["faz"] = "VISUAL"
        status["gecis_sayisi"] += 1
        print(f"[SUPERVISOR] ✓ GÖRSEL YÜRÜTÜCÜ (FSM: {st.value}) — kapanma klempi "
              f"FSM türevi (geçiş #{status['gecis_sayisi']})")
        gorsel_stop = threading.Event()
        _kopru(stop_event, gorsel_stop)

        def gorsel_izci():
            # FSM GPS kümesine düşerse (TRACK_LOST vb.) görsel yürütücüyü durdur.
            while not gorsel_stop.is_set():
                s = get_gorev_state()
                if s is not None and s in _GPS_SET:
                    gorsel_stop.set()
                    return
                gorsel_stop.wait(0.05)

        threading.Thread(target=gorsel_izci, daemon=True).start()
        # ── GÖRSEL FAZ ALGORİTMASI DİSPATCH (bkz. _GORSEL_YASA) ──
        # 3 fazlı FSM yürütücüyü buraya getirdi (ENGAGE/STRIKE); İÇİNDE koşan
        # görsel algoritma seçilir. bbox_ibvs: kutu akışını wait_pose adaptörüyle
        # alır (det→pose), integralini drone'un o anki hızıyla sıcak başlatır,
        # terminal hücumu FSM STRIKE'a kapılar (get_gorev_state).
        if _GORSEL_YASA == "bbox_ibvs":
            _i = get_iris() or {}
            _ff = (float(_i.get("vx", 0.0) or 0.0),
                   float(_i.get("vy", 0.0) or 0.0), 0.0)

            def _wait_pose(son_seq, timeout=0.5):
                k = wait_kare(son_seq, timeout)
                if k is None:
                    return None
                return {"seq": k["seq"], "pose": k.get("det"),
                        "stamp": k.get("stamp"), "wall_recv": k.get("wall_recv"),
                        "lock": k.get("lock")}

            sebep = _run_bbox_ibvs(conn, get_iris, _wait_pose, gorsel_stop,
                                   cfg=_IbvsCfg, kayip_kare_esik=sup_cfg.KAYIP_M,
                                   ff_hiz=_ff, get_temas=get_temas,
                                   get_gorev_state=get_gorev_state)
        else:
            sebep = run_visual_lead(conn, wait_kare, get_plane_truth, gorsel_stop,
                                    cfg=lead_cfg, kayip_kare_esik=sup_cfg.KAYIP_M,
                                    get_temas=get_temas, get_menzil=get_menzil,
                                    get_gt=get_gt, get_gorev_state=get_gorev_state)
        status["son_sebep"] = sebep
        if sebep == "vuruldu":
            status["faz"] = "VURULDU"
            print("[SUPERVISOR] ✓✓ HEDEF VURULDU — görev tamamlandı.")
            return
        if stop_event.is_set():
            break
        # kayip/durduruldu → FSM durumuna göre yeniden değerlendir (TRACK_LOST→GPS).
        print(f"[SUPERVISOR] görsel yürütücü bitti (sebep={sebep}) → FSM'e göre devam")
        time.sleep(0.05)                          # thrash önleme (aynı karede yeniden girme)

    status["faz"] = "DURDU"
    print("[SUPERVISOR] Hibrit güdüm sonlandı.")
