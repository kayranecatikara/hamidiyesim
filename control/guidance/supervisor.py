"""
supervisor.py — Faz 4: GPS ↔ görsel güdüm geçişi (hibrit müdahale).

run_hybrid tek görev döngüsüdür (start_chase bunu çalıştırır):

  GPS fazı (gps_guidance) hedefe yaklaşır. Görsel temas oturunca
  (ARDIŞIK KILIT_N karede tespit — conf eşiği D0 gereği KALDIRILDI, VE handoff
  menzili içindeyiz YA DA GPS düşmüş/DROPOUT) → GÖRSEL faza (bbox_ibvs)
  geçilir. Görsel temas kesilirse ya da hedef GEÇİLİRSE (B5 fly-past) → GPS
  fazına dönülür. stop_chase gelene (veya araç vurulana) kadar bu döngü sürer.

  ── Kilit sinyali: DETECTION güveni ──
  "Görsel kilit" kutunun güvenidir (KILIT_CONF_MIN). Bu eşiğin eski adı ve
  keypoint dönemindeki karşılığı için: POSEA_GERI_DONMEK_ISTERSENIZ/README.md

Menzil kapısının (GATE_KILIT) nedeni: görsel fazın kapanma hızı sabit
(V_KAPANMA); uzaktan erken geçilirse hızlı hedefe yetişilemez. GPS handoff
histerezisi (≤40 m) zaten "yetişilmiş" durumu işaretler. GPS jam/DROPOUT'ta
menzil bilinemez → görsel temas tek başına yeter (jamming fallback).

GT modunda görsel kilidi atlamak MÜMKÜNDÜR (`AVCI_GT_KILIT_BYPASS=on`) ama
VARSAYILAN KAPALIDIR — ölçümle çürütüldü, bkz. SupCfg.GT_KILIT_BYPASS.
"""

import collections
import os
import threading

from control import carpisma_state          # GERÇEK temas — GPS fazı vuruşu
from control.guidance import gps_guidance as _ga
from control.guidance.gps_guidance import run_gps_guidance
from control.guidance.guidance_core import Cfg as LeadCfg
from control.guidance.bbox_ibvs import run_bbox_ibvs, Cfg as IbvsCfg

# GÖRSEL FAZ: TEK YASA — SAF bbox IBVS (GPS'siz, D0 yarışma kuralına uygun).
# ⚠ 2026-08-10: eski `lead` kolu (visual_lead + adapter_copter) ARŞİVLENDİ →
# POSEA_GERI_DONMEK_ISTERSENIZ/gudum_anlik_goruntu/. Uçmayan bir kolu ayakta
# tutmak hangi kodun gerçek olduğunu belirsizleştiriyordu ve bir kez pahalıya
# patladı: TODO'da sıradaki dört A/B'nin anahtarları o koldaydı, uçsaydık
# dört uçuş boşa giderdi. AVCI_VISUAL anahtarı da kalktı.


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
    # ⚠ KAYIP_M BİR EŞİK DEĞİL, BAYRAK — 2026-08-09'da anlaşıldı.
    # Buradaki 20 değeri HİÇBİR YERDE KULLANILMIYOR. `run_visual_lead`'e
    # `kayip_kare_esik=` diye geçiliyor ama orada yalnız `is not None` diye
    # okunuyor (visual_lead.py:423,513,620) — anlamı "hibrit moddayız, temas
    # kaybında GPS'e dön". Eski yorumu ("ardışık tespitsiz kare ~0.66 s")
    # yanlıştı ve analizleri yanlış yöne sürüyordu.
    # GERÇEK kayıp ölçütü KAYAN PENCEREDİR, ardışık sayaç değil:
    #     guidance_core.KAYIP_PENCERE = 40 kare (~1.3 s @30 Hz)
    #     guidance_core.KAYIP_MIN_ISABET = 4
    #   → son 40 karede 4'ten az tespit varsa faz düşer.
    # Değeri değiştirmek DAVRANIŞI DEĞİŞTİRMEZ; eşiği ayarlamak isteyen
    # guidance_core'daki o iki sabite bakmalı.
    KAYIP_M = 20

    # ══ D0 KURAL UYUMU — DEVİR ÖLÇÜTÜ SADELEŞTİRİLDİ ══
    # Kayra'nın `kayramin_super_gudumu` dalından alındı (2b8d68c, 2026-08-10).
    # Kullanıcı tespiti: "görsel temas sağlandıktan sonra GPS'ten güdüm
    # üretmek yasak; üst üste N kare dedektör algıladıysa görsel güdüme geç."
    #
    # ⚠ ASIL SEBEP — KURAL İHLALİ RİSKİ (bu depoda doğrulandı):
    # Eski KILIT_CONF_MIN=0.5 iken dedektörün kendi kabul eşiği
    # bbox_ibvs.Cfg.CONF_MIN=0.35'ti. Arada kalan 0.35-0.50 bandında GÖRSEL
    # TEMAS VAR sayılıyor ama supervisor devretmiyordu → görsel temas varken
    # GPS güdümü sürüyordu. D0 tam olarak bunu yasaklıyor.
    #
    # İkinci sapma: kayan pencere (son 15'in 10'u) "üst üste 10" değildi.
    # Bu şart GEVŞEKTİ, yani devri erkene alıyordu (kural lehine, ama ölçüte
    # sadık değil).
    #
    # ⚠ RİSK (Kayra'nın kendi notu, aynen taşındı): "üst üste N" gürültülü
    # tespitte GEÇ sağlanabilir — kayan pencere tam bu yüzden konmuştu
    # (2026-07-31). Devir menzili ölçülecek; kötüleşirse ARDISIK=0 yapılır.
    # Geri dönüş: AVCI_HYBRID_ARDISIK=0 → kayan pencere
    #             AVCI_HYBRID_CONF=0.5  → eski ekstra güven eşiği
    KILIT_ARDISIK = os.environ.get("AVCI_HYBRID_ARDISIK", "1") == "1"
    KILIT_CONF_MIN = float(os.environ.get("AVCI_HYBRID_CONF", 0.0))

    # ── MENZİL KAPISI KAPATILDI (2026-08-08, D0 YARIŞMA KURALI) ──
    #
    # Kural: görsel temas kurulunca (tespit sürekliliği) GPS ile güdüm YASAK.
    # Menzil kapısı tam bunu ihlal ediyordu: 30 m'de hedef kesintisiz
    # görülürken kapı "henüz 20 m değil" deyip GPS güdümünü SÜRDÜRÜYORDU.
    # Kapıyı 20 → 12 m'ye çekmek ihlali BÜYÜTÜR (GPS'te daha uzun kalınır);
    # 2026-08-08'de bunu yaptım, kullanıcı yakaladı — yanlış refleksti.
    #
    # DOĞRU ÇÖZÜM: kapıyı kaldır, görsel fazı uzak menzilde de ÇALIŞIR yap.
    # Uzakta çalışamamasının kök nedeni kapı değil HIZDI: saf kutu-boyutu
    # modeli 8 m/s üretiyordu, hedef 15 m/s. Dondurulmuş taşıyıcı (bkz.
    # bbox_ibvs) bunu kapatıyor → devir artık her menzilde yaşanabilir.
    #
    # Geri açmak (deney amaçlı): AVCI_HYBRID_GATE=1
    GATE_KILIT = os.environ.get("AVCI_HYBRID_GATE", "0") == "1"
    GATE_MENZIL = float(os.environ.get("AVCI_HYBRID_GATE_MENZIL", 20.0))

    # ── GPS FAZINDA VURUŞ RAPORLAMA (2026-08-09) ──
    # Vuruş görsel fazdan sonra olur; ama avcı hedefe kameranın göremeyeceği
    # kadar yaklaşınca faz 'kayip' ile biter ve ÇARPMA GPS fazına düşer.
    # O pencerede olan temas eskiden hiçbir yerde raporlanmıyordu.
    # Ölçüt YALNIZ Gazebo contact sensörü (carpisma_state); yakınlık yedeği
    # BİLEREK yok — GPS fazı hedefin 8-10 m gerisinde durmak üzere kurulu.
    GPS_VURUS = os.environ.get("AVCI_GPS_VURUS", "on").lower() not in ("0", "off")

    # ── GT MODUNDA GÖRSEL KİLİDİ ATLA — DENENDİ VE GERİ ALINDI (2026-08-04) ──
    # Gerekçe mantıklıydı: GT modunda güdüm tespite bakmıyor, o hâlde geçişi
    # tespitin tutması anlamsız. Kilit sinyali "GT akışı canlı mı"ya çevrildi.
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
    # MEKANİZMA: görsel kilit farkında olmadan bir GECİKME görevi görüyormuş.
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
status = {
    # Faz bitiş sebepleri (TEŞHİS — panel/log okur; davranışa etkisi yok)
    "gps_faz_bitis": None, "gorsel_faz_bitis": None,"faz": "GPS", "gecis_sayisi": 0, "kilit_sayac": 0, "son_sebep": None}


def _kopru(parent_event, child_event):
    """parent set olunca child'ı da set eder (faz thread'i ana stop'u duysun)."""
    def izle():
        while not parent_event.is_set() and not child_event.is_set():
            parent_event.wait(0.5)
        if parent_event.is_set():
            child_event.set()
    threading.Thread(target=izle, daemon=True).start()


def run_hybrid(conn, get_plane, get_iris, wait_kare, get_plane_truth,
               stop_event, sup_cfg=SupCfg, lead_cfg=LeadCfg, get_temas=None,
               get_menzil=None, get_gt=None):
    # gecis_sayisi BURADA SIFIRLANMAZ (2026-08-05): gcs_server'ın güdüm modu
    # seçici döngüsü run_hybrid'i görev boyunca defalarca çağırıyor; burada
    # sıfırlamak sayacı her çağrıda 0'a düşürüp arayüzde hep boş gösteriyordu.
    # Sayaç GÖREV başına anlamlı → start_chase/start_visual sıfırlıyor.
    status.update(faz="GPS", kilit_sayac=0, son_sebep=None)
    # Kilit sinyali GT akışına devredilsin mi (varsayılan HAYIR — ölçümle
    # çürütüldü, bkz. SupCfg.GT_KILIT_BYPASS).
    gt_modu = bool(getattr(lead_cfg, "GT_ROT", False) and get_gt is not None
                   and sup_cfg.GT_KILIT_BYPASS)
    if gt_modu:
        print("[SUPERVISOR] ⚠ GÖRSEL KİLİT ATLANIYOR (AVCI_GT_KILIT_BYPASS=on) — "
              "geçiş yalnız menzil/DROPOUT kapısına kalır. ÖLÇÜMDE KÖTÜLEŞTİRDİ: "
              "devir 6.6→19.6 m, en yakın 0.68→2.41 m, 13/13 faz kayıp.")

    while not stop_event.is_set():
        # ══ GPS FAZI ══ (gps_guidance kendi 20 Hz döngüsünde; izci tespit akışını sayar)
        status["faz"] = "GPS"
        faz_stop = threading.Event()
        _kopru(stop_event, faz_stop)
        tetik = {"gorsel": False, "vuruldu": False}

        def izci():
            pencere = collections.deque(maxlen=sup_cfg.KILIT_PENCERE)
            ardisik = 0
            son_seq = 0
            while not faz_stop.is_set():
                # ── GPS FAZINDA VURUŞ (2026-08-09, kullanıcı isteği) ──
                # Vuruş hep görsel fazdan sonra olur — AMA avcı hedefe öyle
                # yaklaşır ki kamera onu göremez, faz 'kayip' ile biter ve
                # ÇARPMA GPS fazına düşer. 08-09 uçuşunda 22 görsel fazın
                # 17'si fly-past ile bitti, bir kısmı 0.43-1.1 m'ye kadar
                # kapatmıştı. Bu pencerede gerçekleşen temas hiçbir yerde
                # raporlanmıyordu — görev sonsuza kadar dönüyordu.
                #
                # ⚠ ÖLÇÜT YALNIZ GERÇEK TEMAS: kaynak_var() dinleyicinin
                # ÇALIŞTIĞINI, temas_var() Gazebo contact sensöründen GERÇEK
                # çarpma geldiğini söyler. Yakınlık YEDEĞİ burada BİLEREK YOK:
                # GPS fazı zaten hedefin 8-10 m gerisinde durmak üzere
                # kurulu, mesafeye bakmak sahte vuruş üretirdi (görsel fazda
                # tek koşuda 6 sahte vuruş ölçülmüştü — bkz. _vurus_oldu).
                # Kullanıcı şartı buydu: "vuruldu saysın EĞER GERÇEKTEN
                # ÇARPMA VE HASAR TESPİTİ DOĞRU ÇALIŞIYORSA".
                if (sup_cfg.GPS_VURUS and carpisma_state.kaynak_var()
                        and carpisma_state.temas_var()):
                    tetik["vuruldu"] = True
                    faz_stop.set()
                    return
                kayit = wait_kare(son_seq, timeout=0.5)
                if kayit is None:
                    continue
                son_seq = kayit["seq"]
                det = kayit["det"]
                # GT modunda kilit sinyali tespit DEĞİL, GT akışının canlılığıdır.
                if gt_modu:
                    gorulen = get_gt() is not None
                else:
                    gorulen = (det is not None
                               and det.get("conf", 0.0) >= sup_cfg.KILIT_CONF_MIN)
                # D0: ARDIŞIK sayım — tek bir tespitsiz kare sayacı sıfırlar.
                # ARDISIK=0 iken eski kayan pencere davranışı (bkz. SupCfg).
                if sup_cfg.KILIT_ARDISIK:
                    ardisik = (ardisik + 1) if gorulen else 0
                    sayac = ardisik
                else:
                    pencere.append(gorulen)
                    sayac = sum(pencere)      # kayan pencerede güvenli kare sayısı
                status["kilit_sayac"] = sayac
                if sayac >= sup_cfg.KILIT_N:
                    d_h = _ga.status.get("d_h")
                    yakin = (d_h is not None and d_h < sup_cfg.GATE_MENZIL)
                    dropout = _ga.status.get("durum") == "DROPOUT"  # jamming fallback
                    kapi = (not sup_cfg.GATE_KILIT) or yakin or dropout
                    if kapi:
                        tetik["gorsel"] = True
                        faz_stop.set()          # gps_guidance döngüsünü kır
                        return

        threading.Thread(target=izci, daemon=True).start()
        print(f"[SUPERVISOR] GPS fazı "
              f"({'GT akışı' if gt_modu else 'görsel kilit'}: "
              + (f"{sup_cfg.KILIT_N} ARDIŞIK kare" if sup_cfg.KILIT_ARDISIK
                 else f"{sup_cfg.KILIT_N}/{sup_cfg.KILIT_PENCERE} kayan kare")
              + f", conf≥{sup_cfg.KILIT_CONF_MIN:.2f}"
              + f"{' + handoff/DROPOUT kapısı' if sup_cfg.GATE_KILIT else ''})")
        run_gps_guidance(conn, get_plane, get_iris, faz_stop)

        # ── FAZ BİTİŞ SEBEBİ (2026-08-10, TEŞHİS — davranışa etkisi YOK) ──
        # NEDEN VAR: 08-10'da hibritte güdümün 69-314 s boyunca TAMAMEN
        # durduğu üç olay yaşandı ve sebebi ancak log kazarak daraltılabildi
        # (bkz. TODO §0). Fazın neden bittiği hiçbir yere yazılmıyordu.
        # Artık hem stdout'a hem status'a düşer; bir sonraki donmada cevap
        # tek satırda görünür.
        _sebep_gps = ("vuruldu"   if tetik["vuruldu"]
                      else "devir" if tetik["gorsel"]
                      else "durduruldu" if stop_event.is_set()
                      else "BİLİNMEYEN")
        status["gps_faz_bitis"] = _sebep_gps
        print(f"[SUPERVISOR] GPS fazı bitti — sebep: {_sebep_gps}")
        if _sebep_gps == "BİLİNMEYEN":
            print("[SUPERVISOR] ⚠ GPS fazı KENDİLİĞİNDEN bitti (ne devir, ne "
                  "vuruş, ne durdurma). run_hybrid burada SONLANIYOR — görev "
                  "döngüsü yeniden kurmazsa araca komut gitmez. TODO §0.")

        if tetik["vuruldu"]:
            status["faz"] = "VURULDU"
            status["son_sebep"] = "vuruldu_gps"
            print("[SUPERVISOR] ✓✓ HEDEF VURULDU (GPS fazında gerçek temas) — "
                  "görev tamamlandı.")
            return

        if stop_event.is_set() or not tetik["gorsel"]:
            break

        # ══ GÖRSEL FAZ ══ (temas kesilene ya da stop'a kadar)
        status["faz"] = "VISUAL"
        status["gecis_sayisi"] += 1
        print(f"[SUPERVISOR] ✓ GÖRSEL TEMAS — görsel güdüme geçildi "
              f"(geçiş #{status['gecis_sayisi']})")
        # bbox IBVS — CANLI GPS girmez (yarışma kuralı D0).
        # get_plane_truth/get_menzil/get_gt KASITEN geçilmez.
        #
        # DONDURULMUŞ TAŞIYICI: hedefin son GPS hız kestirimi BURADA, yani
        # görsel faz BAŞLAMADAN önce bir kez okunur ve sayı olarak geçilir.
        # Görsel döngü canlı GPS'e erişemez (callback değil, üçlü sayı).
        ff = (_ga.status.get("tgt_vx") or 0.0,
              _ga.status.get("tgt_vy") or 0.0,
              _ga.status.get("tgt_vz") or 0.0)
        print(f"[SUPERVISOR] taşıyıcı donduruldu: "
              f"({ff[0]:+.1f},{ff[1]:+.1f},{ff[2]:+.1f}) m/s "
              f"— görsel faz boyunca GPS'e bir daha bakılmayacak")
        # get_temas: Talon çarpma sensörü — SONUÇ sinyali (vuruş kararı),
        # güdüm girdisi değil; hedefin yerini/hızını taşımaz.
        # ⚠ Köprü adları: wait_kare / kayit["det"] (KARE köprüsü). Bu depoda
        # tek isim düzlemi budur; eski köprü adlarıyla yazılmış dış kod
        # gelirse çevrilir, yoksa çalışma anında KeyError verir.
        sebep = run_bbox_ibvs(conn, get_iris, wait_kare, stop_event,
                              cfg=IbvsCfg, kayip_kare_esik=sup_cfg.KAYIP_M,
                              ff_hiz=ff, get_temas=get_temas)
        status["son_sebep"] = sebep
        status["gorsel_faz_bitis"] = sebep
        print(f"[SUPERVISOR] görsel faz bitti — sebep: {sebep}")
        if sebep == "vuruldu":
            status["faz"] = "VURULDU"
            print("[SUPERVISOR] ✓✓ HEDEF VURULDU — görev tamamlandı.")
            return
        if sebep == "kayip":
            print("[SUPERVISOR] Görsel temas kesildi → GPS fazına dönülüyor")
            continue
        break                                    # durduruldu

    status["faz"] = "DURDU"
    print("[SUPERVISOR] Hibrit güdüm sonlandı.")
