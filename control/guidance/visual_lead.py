"""
visual_lead.py — IBVS lead pursuit döngüsü (olay güdümlü, kameraya kilitli).

Sabit Hz'te DÖNMEZ: kamera 30 Hz, kare geldikçe işler (sabit döngü kare tekrarı
ve bayat veri üretir). Her yeni pose kaydında:
  bayat kare kapısı → GUIDED kontrolü → guidance_core.process → adaptör → CSV.

Saat tasarımı (sim/duvar saati birbirine karıştırılmaz):
  dt        = ardışık kare header.stamp farkı (sim saati) — filtre/rampa bunu kullanır
  gecikme_s = time.time() - wall_recv (karenin gcs'e geliş duvar anı; komut anında)
  gecikme > GECIKME_TAVAN_S → kare atlanır, komut GÖNDERİLMEZ, son komut korunur.

Kullanım (gcs_server): run_visual_lead(conn, wait_pose, get_plane_truth, stop_event)
  wait_pose        : vision.detection_state.wait_new_pose
  get_plane_truth  : hedefin GERÇEK NED pozu (çerçeve-ofset düzeltmeli) — SADECE
                     menzil_gercek/kapanma_hizi ve CEVAP ANAHTARI sütunları için,
                     güdüme girmez (bkz. _cevap_anahtari).
"""

import collections
import csv
import math
import os
import time

from control import carpisma_state
from control import mav_common
from control.guidance.adapter_copter import CopterAdapter
from control.guidance.common import send_velocity
from control.guidance.guidance_core import (Cfg, LeadPursuitCore, govde_to_dunya,
                                            hedef_kadraj_hatasi)
from vision import geometry as geo

_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs")

_CSV_ALANLAR = [
    "t_ros", "dt", "gecikme_s", "bbox", "a", "b", "olcek_ham", "eps_deg",
    "duzeltme", "olcek", "yandanlik_ham", "yandanlik_filtreli", "kalite",
    "lead_deg", "u_nisan_x", "u_nisan_y", "u_nisan_z",
    "u_govde_x", "u_govde_y", "u_govde_z", "yaw_hata_deg", "pitch_hata_deg",
    "yatay_bilesen", "azimut_kalite", "yaw_adim_deg",
    "vx_cmd", "vy_cmd", "vz_cmd", "yaw_cmd_deg", "v_doygun", "yaw_doygun",
    "durum", "flip_sayaci", "eksen_disi_deg", "govde_yukselti_deg",
    "menzil_kestirim_m", "menzil_gercek_m", "menzil_ham_m", "menzil_red",
    "menzil_kaynak",   # "gz" (zaman hizalı) | "telem" (yedek) — bkz. _menzil_olc
    "kapanma_hizi_ms", "mod",
    "pitch_body_deg", "kamera_dunya_pitch_deg", "pn_dikey_deg", "coalt_deg",
    "los_elev_deg", "los_elev_rate_dps", "gama_deg",
    "kadraj_hata_deg", "kadraj_duz_deg",
    # ── CEVAP ANAHTARI (bkz. _cevap_anahtari) — ölçüm, güdüme GİRMEZ ──
    "gercek_yaw_deg", "gercek_elev_deg", "gercek_menzil_ham_m",
    "gercek_u_px", "gercek_v_px", "gercek_onde", "gercek_kadraj_ici",
    "pose_yaw_sapma_deg", "pose_elev_sapma_deg", "pose_menzil_sapma_m",
    # Hangi bayraklarla uçuldu — yalnız İLK satırda dolu (bkz. _yapilandirma).
    "yapilandirma",
]

# durum kodları (CSV): ok / cozumsuz / kanat_dusuk / kpt_dusuk / tespit_yok /
#                      bayat / mod_hata / attitude_yok / kor_dalis / vuruldu


class _ArasState:
    """Kendi aracımızın MAVLink durumunu conn'dan non-blocking drenajla günceller
    (visual thread aktifken iris telem worker durur — port deseni gcs'te)."""
    def __init__(self):
        self.attitude = None      # (roll, pitch, yaw) rad
        self.mode = None          # HEARTBEAT custom_mode
        self.pos = None           # (x, y, z) NED, iris çerçevesi

    def drenaj(self, conn):
        while True:
            msg = conn.recv_match(
                type=["ATTITUDE", "HEARTBEAT", "LOCAL_POSITION_NED"],
                blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "ATTITUDE":
                self.attitude = (msg.roll, msg.pitch, msg.yaw)
            elif t == "HEARTBEAT" and msg.get_srcSystem() != 255:
                self.mode = msg.custom_mode
            elif t == "LOCAL_POSITION_NED":
                self.pos = (msg.x, msg.y, msg.z)


def _menzil_hesapla(get_plane_truth, iris_pos):
    """Hedefin gerçek NED pozu ile iris arası mesafe (m) veya None.
    NOT: sim ground-truth; gerçek uçuşta yerini yakınlık/menzil sensörü alır."""
    if iris_pos is None or get_plane_truth is None:
        return None
    p = get_plane_truth()
    if p is None:
        return None
    return math.sqrt((p["x"] - iris_pos[0]) ** 2 + (p["y"] - iris_pos[1]) ** 2
                     + (p["z"] - iris_pos[2]) ** 2)


def _cevap_anahtari(satir, hedef_ned, aras, res=None):
    """CEVAP ANAHTARI — hedefin GERÇEK açıları, yalnız ÖLÇÜM için.

    ⚠ BU SÜTUNLAR GÜDÜME GİRMEZ VE GİRMEMELİDİR. Hedefin telemetrisi bir
    simülasyon kolaylığıdır; gerçek harekâtta düşman uçağın konumu bilinmez —
    görsel güdümün varlık sebebi zaten budur. Buradaki değerler komut yoluna
    bağlanırsa drone hedefi vurur ama kamera hattının çalışıp çalışmadığı bir
    daha ölçülemez, ve bu daldaki tüm kazanç ayarları (KP_KADRAJ taraması, faz
    kapıları, menzil kapısı) dayandıkları gürültü varsayımını kaybeder.
    Fonksiyon bu yüzden yalnız `satir` sözlüğüne yazar, hiçbir şey döndürmez.

    Ne ölçer: pose zincirinin LEAD'SİZ hedef kestirimi (res["u_govde_hedef"])
    ile saf geometrinin verdiği gerçek yön yan yana → fark = ALGI hatası.
    Lead'li nişan (u_govde) bilerek hedeften kaydırılmıştır, kıyas ona yapılmaz.

    Neden ayrıca piksel: gercek_kadraj_ici, hedefin o karede kameranın görüş
    alanında OLUP OLMADIĞINI söyler. "tespit_yok" satırlarında bu sütun,
    modelin mi kaçırdığını yoksa hedefin kadrajdan çıktığını mı ayırt eder —
    dikey ıska probleminin (sabit 4.65 m ofset) doğrudan kanıtı.

    ZAMAN NOTU: telemetri ile kare AYNI ana ait değildir (kare gecikme_s kadar
    bayat, telemetri ayrı hızda akar). Sapma sütunları bu kadar zamanlama
    gürültüsü taşır; analizde gecikme_s ile süzülmeli.
    """
    if hedef_ned is None or aras.pos is None or aras.attitude is None:
        return
    kad = hedef_kadraj_hatasi(hedef_ned, aras.pos, *aras.attitude)
    satir["gercek_yaw_deg"] = round(math.degrees(kad["yaw_hata"]), 2)
    satir["gercek_elev_deg"] = round(math.degrees(kad["elev"]), 2)
    satir["gercek_menzil_ham_m"] = round(kad["menzil"], 3)
    satir["gercek_onde"] = int(kad["onde"])
    if kad["u"] is not None:
        satir["gercek_u_px"] = round(kad["u"], 1)
        satir["gercek_v_px"] = round(kad["v"], 1)
        satir["gercek_kadraj_ici"] = int(0 <= kad["u"] < geo.IMG_W
                                         and 0 <= kad["v"] < geo.IMG_H)
    else:
        satir["gercek_kadraj_ici"] = 0        # hedef kameranın arkasında

    if res is None:
        return                                 # tespit yok → kıyaslanacak kestirim yok
    uh = res["u_govde_hedef"]                  # LEAD'SİZ saf hedef yönü (kamera zinciri)
    kam_yaw = math.atan2(float(uh[1]), float(uh[0]))
    kam_elev = math.atan2(-float(uh[2]), math.hypot(float(uh[0]), float(uh[1])))
    # yaw farkı ±180°'e sarılır (±179° ile ∓179° arası 2°'dir, 358° değil)
    d_yaw = (kam_yaw - kad["yaw_hata"] + math.pi) % (2 * math.pi) - math.pi
    satir["pose_yaw_sapma_deg"] = round(math.degrees(d_yaw), 2)
    satir["pose_elev_sapma_deg"] = round(math.degrees(kam_elev - kad["elev"]), 2)
    if kad["menzil"] > 1e-6:
        satir["pose_menzil_sapma_m"] = round(
            res["menzil_kestirim_m"] - kad["menzil"], 2)


class _MenzilKapisi:
    """Menzil sinyaline MAKULLÜK kapısı — fiziksel olarak imkânsız sıçramaları eler.

    Neden: ground-truth menzil zıplıyor. 193559 uçuşunda 33 ms'de 22.4 → 6.6 m
    (= 479 m/s) örneği geldi ve kod bunu VURULDU saydı; oysa drone hedefin
    altından geçip uzaklaşıyordu. 2026-07-31 uçuşlarında doğrulanan 7 vuruşun
    1'i bu şekilde SAHTE. Sinyal yalnız log değil — kör dalış tetiğini
    (_terminal_giris_ok) ve terminal co-altitude kilidini de besliyor, yani
    yalan menzil yanlış anda yanlış terminal hamlesi yaptırıyor.

    Kapı: iki örnek arasındaki değişim MENZIL_HIZ_TAVAN·dt'yi aşarsa örnek
    REDDEDİLİR, son geçerli değer korunur. Üst üste çok reddedersek gerçekten
    kopmuşuzdur (ör. çerçeve ofseti yeniden kalibre oldu) — o zaman yeni değere
    SENKRONİZE ol, yoksa bayat bir değere sonsuza dek kilitleniriz.

    Bu kapı SEMPTOMU keser; verinin neden zıpladığı ayrı bir iş (bkz. Görev 4,
    baş şüpheli gcs_server._frame_off dikey kalibrasyonu).

    ZAMAN NOTU: kapı GEÇEN SÜREYİ dışarıdan alır, saat tutmaz. Kare döngüsü sim
    saatinden (header.stamp) türetilmiş dt verir, kör dalış döngüsü duvar
    saatinden — dosyanın saat ayrımı kuralı böylece bozulmaz."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.son = None           # son GEÇERLİ menzil
        self.red_sayaci = 0       # toplam reddedilen örnek (CSV'ye gider)
        self._ardisik_red = 0

    def ekle(self, d, dt):
        """Yeni ham örnek + son örnekten bu yana geçen süre (s).
        Dönüş: (gecerli_menzil, kabul_edildi_mi)."""
        if d is None:
            return self.son, False
        if self.son is None or dt is None or dt <= 0:
            self.son = d
            self._ardisik_red = 0
            return d, True
        if abs(d - self.son) > self.cfg.MENZIL_HIZ_TAVAN * dt:
            self.red_sayaci += 1
            self._ardisik_red += 1
            if self._ardisik_red < self.cfg.MENZIL_RESENK_N:
                return self.son, False          # son geçerli değeri koru
            # ısrarla sapıyor → gerçekten koptuk, yeni seviyeye senkronize ol
        self.son = d
        self._ardisik_red = 0
        return d, True


_VURUS_ETIKET = {
    "gercek_temas": "GERÇEK TEMAS",
    # Yedek yolun raporu bilerek çirkin: log'a bakan biri bunun fiziksel temas
    # DEĞİL, kaynak yokluğundan düşülen tahmin olduğunu görmeli.
    "yedek_menzil": "yakınlık YEDEĞİ (temas kaynağı yok — gerçek temas DEĞİL)",
}


def _m_yaz(m):
    return f"{m:.2f}" if m is not None else "?"


def _vurus_oldu(menzil, cfg, temas_kaynagi=carpisma_state):
    """A5 — VURUŞ ÖLÇÜTÜ. Dönüş: (vuruldu_mu, gerekce).

    Öncelik sırası ve nedeni:

    1. GERÇEK TEMAS (Gazebo contact sensörü). Tek dürüst ölçüt.
    2. Temas kaynağı ÇALIŞIYOR ama temas yok → VURUŞ DEĞİL, menzile bakılmaz.
       A5'in bütün amacı bu satır. Eski kod 1.5 m altını vuruş sayıyordu;
       ölçüldü — tek koşuda 0.61 / 0.69 / 0.75 / 1.06 / 1.16 / 1.20 m
       yaklaşma vardı, yani 6 SAHTE vuruş. Dahası sahte vuruş güdümü
       DURDURUYOR: drone tam hızla giderken komutsuz kalıp savruluyordu
       (00000108: vuruş sonrası 15 s boyunca ~350 °/s dönüş, 21.8 → 2.0 m).
    3. Temas kaynağı YOKSA (gz-transport kurulu değil / AVCI_HASAR=0 /
       dünyada contact-system eksik) yakınlık YEDEĞİ devreye girer. Yoksa
       o kurulumlarda vuruş hiç raporlanamaz.

    Not: 0.61 m'de bile Gazebo temas sensörü tetiklenmiyor (ölçüldü) — gerçek
    temas için ~0.3 m daha kapatmak gerekiyor. Bu ölçüt bunu YARATMIYOR,
    GÖRÜNÜR KILIYOR."""
    if temas_kaynagi.temas_var():
        return True, "gercek_temas"
    if temas_kaynagi.kaynak_var():
        return False, ""
    if menzil is not None and menzil < cfg.VURUS_MENZIL:
        return True, "yedek_menzil"
    return False, ""


def run_visual_lead(conn, wait_pose, get_plane_truth, stop_event, cfg=Cfg,
                    kayip_kare_esik=None, get_temas=None, get_menzil=None,
                    get_gt=None):
    """kayip_kare_esik verilirse (supervisor hibrit modu): bu kadar ARDIŞIK
    pose'suz kare → 'kayip' döner (görsel temas kesildi, GPS'e dönülecek).
    Dönüş: 'durduruldu' (stop_event) | 'kayip' (temas kaybı) | 'vuruldu'
    (GERÇEK fiziksel temas — bkz. _vurus_oldu; yakınlık yalnız temas kaynağı
    yoksa yedek). Terminal: menzil < TERMINAL_MENZIL iken ve kapanırken temas
    koparsa GPS'e DÖNMEZ, son nişan komutunu TERMINAL_SURE boyunca sürdürür (kör
    dalış — hedef kadraj tepesinden çıkınca çarpışmayı tamamlamak için).

    get_gt: GT ROTASYON MODU kaynağı (geometry.rot_gt_goruntu çıktısını döndüren
    çağrılabilir) — yalnız cfg.GT_ROT açıkken kullanılır. Açıkken güdümün algı
    girdisi tamamen bu olur; pose komut yoluna girmez ve "görsel temas"ın anlamı
    'pose var mı' yerine 'GT akışı canlı mı'ya döner (GT her karede vardır, yani
    pose kaybı artık GPS'e döndürmez)."""
    core = LeadPursuitCore(cfg)
    adapter = CopterAdapter(cfg)          # yalnız copter platformu
    gt_modu = bool(getattr(cfg, "GT_ROT", False) and get_gt is not None)
    if gt_modu:
        print("[LEAD] ⚠ GT ROTASYON MODU (AVCI_GT_ROT=on) — güdüm girdileri "
              "Gazebo GERÇEK pozundan; pose modeli komut yolunda DEĞİL. "
              "Bu mod simülasyona özgüdür, gerçek donanımda uçurulamaz.")
    elif getattr(cfg, "GT_ROT", False) and get_gt is None:
        print("[LEAD] ⚠ AVCI_GT_ROT=on ama GT kaynağı verilmedi "
              "(sim_truth kapalı/gz yok) — pose moduyla devam ediliyor.")

    def _menzil_olc():
        """Menzil ölçümü — (metre|None, kaynak_adı).

        ⚠ 2026-08-04'e kadar bu fonksiyon TANIMLIYDI AMA HİÇ ÇAĞRILMIYORDU;
        menzil doğrudan telemetriden (`_menzil_hesapla`) alınıyordu. Bedeli
        ölçüldü: telemetri menzili iki ayrı akışın zaman hizasız farkı ve
        loglarda karelerin %37'sinde bir önceki kareyle AYNI değeri taşıyor
        (en uzun donma 12 kare = 0.4 s; 25 m/s'te 10 m yol). Yani "en yakın
        menzil" istatistiği gerçek en yakın anı kaçırabiliyordu ve bir vuruş
        anında sütun 25.6 m'de donmuş görünüyordu.

        sim_truth.menzil() iki aracın pozunu Gazebo'nun TEK mesajından okur —
        zaman hizalı, fizik doğruluğunda. Öncelik onda; yoksa telemetriye düşer.
        """
        if get_menzil is not None:
            m = get_menzil()
            if m is not None:
                return m, "gz"
        return _menzil_hesapla(get_plane_truth, aras.pos), "telem"

    def _temas_vurus():
        """(vuruldu_mu, akış_canlı_mı). Temas akışı canlıyken vuruş kararının
        tek sahibi fiziksel temastır (sahte menzil-vuruşlarına karşı)."""
        if get_temas is None:
            return False, False
        t = get_temas()
        if t is True:
            print("[LEAD] ✓ VURULDU (FİZİKSEL TEMAS — Gazebo contact)")
            return True, True
        return False, t is not None

    if get_temas is not None and get_temas() is not None:
        print("[LEAD] Vuruş kararı: FİZİKSEL TEMAS sensörü (menzil eşiği devre dışı)")

    aras = _ArasState()
    son_seq = 0            # _pose_seq 0'dan başlar; ilk GERÇEK kareyi bekle
    menzil_kapi = _MenzilKapisi(cfg)   # imkânsız menzil sıçramalarını eler
    menzil_onceki = None
    t_menzil_onceki = None
    t_terminal_menzil = None   # kör dalışta menzil örnekleri arası DUVAR saati
    bayat_sayaci = 0
    kayip_sayaci = 0       # ardışık pose'suz kare (yalnız log/gözlem)
    temas_pencere = collections.deque(maxlen=cfg.KAYIP_PENCERE)   # kayan temas penceresi
    son_kayit_wall = time.time()
    son_v_cmd = None       # son gönderilen (vx,vy,vz,yaw) — kör dalışta sürdürülür
    kapaniyor = False      # menzil azalıyor mu (yalnız log/gözlem)
    terminal_baslangic = None   # kör dalış başlangıç duvar anı
    terminal_latch = False      # kör dalış KİLİDİ (girince süre/vuruşa dek sürer)
    terminal_min = None         # kör dalış boyunca görülen en yakın menzil
    coalt_latch = False         # terminal co-altitude KİLİDİ (menzil bir kez eşik
                                # altına inince kilitli; ground-truth menzil gürültüsü
                                # yukarı yanlılığı titretmesin)
    hedef_ned = None            # hedefin gerçek NED pozu — YALNIZ cevap anahtarı logu

    os.makedirs(_LOG_DIR, exist_ok=True)
    csv_yol = os.path.join(_LOG_DIR,
                           time.strftime("visual_lead_%Y%m%d_%H%M%S.csv"))
    f = open(csv_yol, "w", newline="")
    w = csv.DictWriter(f, fieldnames=_CSV_ALANLAR, extrasaction="ignore")
    w.writeheader()
    # YAPILANDIRMA DAMGASI (2026-08-04): hangi bayraklarla uçulduğu logdan
    # okunabilsin. Olmadığı için A2/A3/A4 koşuları birbirinden ayırt edilemedi
    # ve deney tekrarlanmak zorunda kaldı — takip açık/kapalı CSV'de hiçbir ize
    # bırakmıyordu. İlk satıra yazılır (sonraki satırlarda boş).
    _yapilandirma = ",".join([
        f"GT={'on' if getattr(cfg, 'GT_ROT', False) else 'off'}",
        f"POSE={os.environ.get('AVCI_POSE', 'on')}",
        f"KILIT_BYPASS={os.environ.get('AVCI_GT_KILIT_BYPASS', 'off')}",
        f"TRACKER={os.environ.get('AVCI_TRACKER', 'off')}",
        f"LOCK={os.environ.get('AVCI_LOCK', 'on')}",
        f"V_KAPANMA={cfg.V_KAPANMA}",
        f"K_LEAD={cfg.K_LEAD}",
        f"IVME={cfg.IVME_TAVAN}/{cfg.IVME_TAVAN_DIKEY}",
    ])
    _yapilandirma_yazildi = False
    print(f"[LEAD] IBVS lead pursuit başladı (copter, "
          f"K_LEAD={cfg.K_LEAD}, V_KAPANMA={cfg.V_KAPANMA}) — log: {csv_yol}")
    print(f"[LEAD] YAPILANDIRMA: {_yapilandirma}")

    def _satir(row, res=None):
        # Cevap anahtarı HER satıra yazılır (tespit_yok/kör dalış dahil — asıl
        # sorulacak soru zaten "kaçırdığımız karede hedef neredeydi"). Ölçüm
        # amaçlı; güdüm bu değerleri okumaz, bkz. _cevap_anahtari.
        nonlocal _yapilandirma_yazildi
        if not _yapilandirma_yazildi:
            row["yapilandirma"] = _yapilandirma
            _yapilandirma_yazildi = True
        _cevap_anahtari(row, hedef_ned, aras, res)
        w.writerow(row)
        f.flush()

    def _terminal_giris_ok():
        """Kör dalışa GİRİŞ: hibrit modda, son komut var, menzil eşik altında.
        Kapanma bayrağı aranmaz — bu kadar yakında temas koparsa hedef önümüzde,
        ileri commit doğru (gürültülü menzil kapanma bayrağını titretiyordu)."""
        return (kayip_kare_esik is not None and son_v_cmd is not None
                and menzil_onceki is not None
                and menzil_onceki < cfg.TERMINAL_MENZIL)

    def _terminal_adim():
        """Kilitli kör dalış bir adımı. Dönüş: 'vuruldu'/'kayip'/None (sürüyor)."""
        nonlocal terminal_baslangic, terminal_latch, terminal_min, t_terminal_menzil
        if not terminal_latch:
            terminal_latch = True
            terminal_baslangic = time.time()
            terminal_min = menzil_onceki
            t_terminal_menzil = None
            print(f"[LEAD] KÖR DALIŞ (KİLİTLİ) — menzil ~{menzil_onceki:.1f} m, "
                  f"son nişan {cfg.TERMINAL_SURE:.1f} s sürdürülüyor")
        send_velocity(conn, *son_v_cmd)
        aras.drenaj(conn)
        # Kör dalışta kare akışı YOK → geçen süre DUVAR saatinden ölçülür.
        # Kapıdan geçirilmezse sahte bir menzil örneği burada VURULDU raporlar
        # (193559 uçuşu tam böyle bitti: 10.4 m'den 2.8 m'ye "düştü").
        simdi = time.time()
        dt_t = (simdi - t_terminal_menzil) if t_terminal_menzil else None
        t_terminal_menzil = simdi
        m, _kabul = menzil_kapi.ekle(_menzil_olc()[0], dt_t)
        if m is not None:
            terminal_min = min(terminal_min, m) if terminal_min is not None else m
        vuruldu, gerekce = _vurus_oldu(m, cfg)
        if vuruldu:
            print(f"[LEAD] ✓ VURULDU — {_VURUS_ETIKET[gerekce]} "
                  f"(menzil {_m_yaz(m)} m)")
            return "vuruldu"
        if time.time() - terminal_baslangic > cfg.TERMINAL_SURE:
            vuruldu, gerekce = _vurus_oldu(terminal_min, cfg)
            if vuruldu:
                print(f"[LEAD] ✓ VURULDU — {_VURUS_ETIKET[gerekce]} "
                      f"(en yakın {terminal_min:.2f} m)")
                return "vuruldu"
            en_yakin = f"{terminal_min:.2f}" if terminal_min is not None else "?"
            print(f"[LEAD WARN] kör dalış bitti — en yakın {en_yakin} m, ISKA")
            return "kayip"
        return None

    try:
        while not stop_event.is_set():
            kayit = wait_pose(son_seq, timeout=0.5)
            if kayit is None:                 # yeni kare yok (timeout / akış durdu)
                # Kilitli kör dalıştaysak veya girişe uygunsak son nişanı sürdür.
                if terminal_latch or _terminal_giris_ok():
                    sonuc = _terminal_adim()
                    if sonuc:
                        return sonuc
                    time.sleep(0.02)
                    continue
                if (kayip_kare_esik is not None
                        and time.time() - son_kayit_wall > 1.0):
                    print("[LEAD WARN] kare akışı kesildi (>1 s) — temas kaybı")
                    return "kayip"
                continue
            son_seq = kayit["seq"]
            son_kayit_wall = time.time()
            pose, stamp = kayit["pose"], kayit["stamp"]
            wall_recv = kayit["wall_recv"]
            # GT modunda algı girdisi burada tazelenir; pose artık okunmaz.
            gt = get_gt() if gt_modu else None

            aras.drenaj(conn)
            satir = {"t_ros": stamp, "flip_sayaci": core.flip_sayaci,
                     "mod": aras.mode}
            hedef_ned = None      # her karede tazelenir; gelmezse sütun BOŞ kalır
                                  # (bayat ground-truth sessizce loglanmasın)

            # Fiziksel temas kontrolü (akış canlıysa vuruş kararının tek sahibi)
            vurus, temas_canli = _temas_vurus()
            if vurus:
                # Menzili BU satıra da yaz: temas kontrolü menzil bloğundan önce
                # olduğu için vuruş satırı eskiden BOŞ menzille kaydediliyordu.
                # 191258 uçuşunda vuruş anı sütunda 25.6 m (donmuş telemetri)
                # görünüyordu ve gerçek geometri logdan okunamıyordu.
                _m, _k = _menzil_olc()
                if _m is not None:
                    satir["menzil_ham_m"] = round(_m, 3)
                    satir["menzil_gercek_m"] = round(_m, 3)
                    satir["menzil_kaynak"] = _k
                satir["durum"] = "vuruldu"
                _satir(satir)
                return "vuruldu"

            # Cevap anahtarı sütunlarının kaynağı AYRI: hedefin NED pozu
            # telemetriden gelir (açı kıyasları o çerçevede yapılıyor).
            # MENZİL ise artık _menzil_olc() ile ölçülür — zaman hizalı gz
            # önce, telemetri yalnız yedek (bkz. _menzil_olc dokümantasyonu).
            if get_plane_truth is not None:
                p = get_plane_truth()
                if p is not None:
                    hedef_ned = (p["x"], p["y"], p["z"])   # cevap anahtarı kaynağı

            # menzil_gercek + kapanma hızı + terminal durum takibi
            d_ham, _mkaynak = _menzil_olc()
            if d_ham is not None:
                satir["menzil_kaynak"] = _mkaynak
                # MAKULLÜK KAPISI: imkânsız sıçrama sahte VURULDU üretiyordu.
                # dt sim saatinden (kare damgası) — duvar saatiyle karıştırma.
                dt_menzil = ((stamp - t_menzil_onceki)
                             if (stamp and t_menzil_onceki) else None)
                d, kabul = menzil_kapi.ekle(d_ham, dt_menzil)
                satir["menzil_ham_m"] = round(d_ham, 3)
                satir["menzil_red"] = menzil_kapi.red_sayaci
                if not kabul:
                    print(f"[LEAD WARN] menzil örneği reddedildi "
                          f"({menzil_onceki:.1f} → {d_ham:.1f} m, "
                          f"toplam {menzil_kapi.red_sayaci})")
                satir["menzil_gercek_m"] = round(d, 3)
                kapaniyor = (menzil_onceki is not None and d < menzil_onceki)
                if menzil_onceki is not None and stamp and t_menzil_onceki \
                        and stamp > t_menzil_onceki:
                    satir["kapanma_hizi_ms"] = round(
                        -(d - menzil_onceki) / (stamp - t_menzil_onceki), 2)
                menzil_onceki, t_menzil_onceki = d, stamp
                # VURUŞ: gerçek fiziksel temas (yakınlık yalnız yedek) — A5
                vuruldu, gerekce = _vurus_oldu(d, cfg)
                if vuruldu:
                    satir["durum"] = "vuruldu"
                    _satir(satir)
                    print(f"[LEAD] ✓ VURULDU — {_VURUS_ETIKET[gerekce]} "
                          f"(menzil {_m_yaz(d)} m)")
                    return "vuruldu"

            # "Algı girdisi var mı": pose modunda pose, GT modunda GT akışı.
            girdi_var = (gt is not None) if gt_modu else (pose is not None)
            if not girdi_var:                 # bu karede tespit yok
                # Kilitli kör dalış: son nişanı sürdür, GPS'e DÖNME (kayip sayma).
                if terminal_latch or _terminal_giris_ok():
                    satir["durum"] = "kor_dalis"
                    (satir["vx_cmd"], satir["vy_cmd"],
                     satir["vz_cmd"], yaw_r) = son_v_cmd
                    satir["yaw_cmd_deg"] = round(math.degrees(yaw_r), 1)
                    _satir(satir)
                    sonuc = _terminal_adim()
                    if sonuc:
                        return sonuc
                    continue
                satir["durum"] = "tespit_yok"
                _satir(satir)
                kayip_sayaci += 1
                temas_pencere.append(False)

                # KAYAN PENCERE: ardışık sayaç yerine "son N karede en az M tespit".
                # Ardışık ölçüt kümelenmiş kopukluklarda görsel fazı erken
                # bırakıyordu ve GPS fazı drone'u istasyona geri çekiyordu.
                if (kayip_kare_esik is not None
                        and len(temas_pencere) >= cfg.KAYIP_PENCERE
                        and sum(temas_pencere) < cfg.KAYIP_MIN_ISABET):
                    print(f"[LEAD WARN] görsel temas kesildi (son "
                          f"{cfg.KAYIP_PENCERE} karede {sum(temas_pencere)} tespit "
                          f"< {cfg.KAYIP_MIN_ISABET})")
                    return "kayip"
                continue
            kayip_sayaci = 0                  # algı girdisi var → temas sürüyor
            temas_pencere.append(True)
            terminal_baslangic = None         # temas döndü → kör dalış sıfırla
            terminal_latch = False
            terminal_min = None

            # bayat kare kapısı (duvar saati — aynı saat cinsinden ölçüm)
            gecikme = (time.time() - wall_recv) if wall_recv else 0.0
            satir["gecikme_s"] = round(gecikme, 4)
            if pose is not None:              # GT modunda pose olmayabilir
                satir["bbox"] = "|".join(str(v) for v in pose["bbox"])
            if gecikme > cfg.GECIKME_TAVAN_S:
                bayat_sayaci += 1
                satir["durum"] = "bayat"
                print(f"[LEAD WARN] bayat kare atlandı ({gecikme*1000:.0f} ms "
                      f"> {cfg.GECIKME_TAVAN_S*1000:.0f} ms, toplam {bayat_sayaci})")
                _satir(satir)
                continue                      # komut GÖNDERME, son komut korunur

            # GUIDED kontrolü. mod HENÜZ BİLİNMİYORSA (None) komut KESME: HEARTBEAT
            # ~1 Hz, ilk saniye None kalıyor; GPS fazından yeni geldik, zaten
            # GUIDED'iz. Yalnız POZİTİF olarak GUIDED-dışı görürsek blokla.
            if aras.mode is not None and aras.mode != mav_common.COPTER_MODE_GUIDED:
                satir["durum"] = "mod_hata"
                print(f"[LEAD ERROR] mod GUIDED değil (custom_mode={aras.mode}) "
                      f"— komut gönderilmiyor")
                _satir(satir)
                continue

            res = core.process(pose, stamp, aras.attitude, gt=gt)
            for warntip in res["warn"]:
                print(f"[LEAD WARN] {warntip} (kare t={stamp:.3f})")

            satir.update({
                "dt": res["dt"], "a": round(res["a"], 2), "b": round(res["b"], 2),
                "olcek_ham": round(res["olcek_ham"], 2),
                "eps_deg": round(res["eps_deg"], 2),
                "duzeltme": round(res["duzeltme"], 4),
                "olcek": round(res["olcek"], 2),
                "yandanlik_ham": round(res["yandanlik_ham"], 4),
                "yandanlik_filtreli": round(res["yandanlik_f"], 4),
                "kalite": round(res["kalite"], 3),
                "lead_deg": round(res["lead_deg"], 2),
                "u_nisan_x": round(float(res["u_nisan"][0]), 5),
                "u_nisan_y": round(float(res["u_nisan"][1]), 5),
                "u_nisan_z": round(float(res["u_nisan"][2]), 5),
                "u_govde_x": round(float(res["u_govde"][0]), 5),
                "u_govde_y": round(float(res["u_govde"][1]), 5),
                "u_govde_z": round(float(res["u_govde"][2]), 5),
                "yaw_hata_deg": round(res["yaw_hata_deg"], 2),
                "pitch_hata_deg": round(res["pitch_hata_deg"], 2),
                "yatay_bilesen": round(res["yatay_bilesen"], 4),
                "azimut_kalite": round(res["azimut_kalite"], 3),
                "durum": res["durum"], "flip_sayaci": res["flip_sayaci"],
                "eksen_disi_deg": round(res["eksen_disi_deg"], 2),
                "govde_yukselti_deg": round(res["pitch_hata_deg"], 2),
                "menzil_kestirim_m": round(res["menzil_kestirim_m"], 2),
            })

            if aras.attitude is not None:
                mevcut_yaw = aras.attitude[2]
                # terminal co-altitude: menzil eşik altına bir kez inince KİLİTLE
                # (menzil_onceki = ground-truth; gerçek donanımda yakınlık sensörü).
                # Kilit, ground-truth menzil gürültüsünün bayrağı titretmesini önler.
                if (menzil_onceki is not None
                        and menzil_onceki < cfg.TERMINAL_COALT_MENZIL):
                    coalt_latch = True
                cmd = adapter.command(conn, res["u_govde"], res["yaw_hata"],
                                      aras.attitude, res["dt"], mevcut_yaw,
                                      kalite=res["kalite"], terminal=coalt_latch,
                                      azimut_kalite=res["azimut_kalite"])
                # kör dalışta sürdürülecek son nişan komutu
                son_v_cmd = (cmd["v_cmd"][0], cmd["v_cmd"][1], cmd["v_cmd"][2],
                             cmd["yaw_cmd"])
                satir.update({
                    "vx_cmd": round(cmd["v_cmd"][0], 2),
                    "vy_cmd": round(cmd["v_cmd"][1], 2),
                    "vz_cmd": round(cmd["v_cmd"][2], 2),
                    "yaw_cmd_deg": round(math.degrees(cmd["yaw_cmd"]), 1),
                    "v_doygun": int(cmd["v_doygun"]),
                    "yaw_doygun": int(cmd["yaw_doygun"]),
                    "pn_dikey_deg": round(cmd["pn_dikey_deg"], 2),
                    "coalt_deg": round(cmd["coalt_deg"], 2),
                    "yaw_adim_deg": round(cmd["yaw_adim_deg"], 2),
                    "los_elev_deg": round(cmd["los_elev_deg"], 2),
                    "los_elev_rate_dps": round(cmd["los_elev_rate_dps"], 1),
                    "gama_deg": round(cmd["gama_deg"], 2),
                    "kadraj_hata_deg": round(cmd["kadraj_hata_deg"], 2),
                    "kadraj_duz_deg": round(cmd["kadraj_duz_deg"], 2),
                })
                # quad'a özgü izleme: burun eğimi + kameranın DÜNYAYA göre bakışı
                # (ivme tavanı aşılırsa kamera yere bakmaya başlar — Cfg.IVME_TAVAN)
                pitch_body = math.degrees(aras.attitude[1])
                satir["pitch_body_deg"] = round(pitch_body, 2)
                kam = govde_to_dunya(
                    [math.cos(math.radians(cfg.KAMERA_TILT_DEG)), 0.0,
                     -math.sin(math.radians(cfg.KAMERA_TILT_DEG))],
                    *aras.attitude)
                satir["kamera_dunya_pitch_deg"] = round(
                    math.degrees(math.asin(max(-1.0, min(1.0, -float(kam[2]))))), 2)
            else:
                satir["durum"] = satir["durum"] if res["warn"] else "attitude_yok"

            _satir(satir, res)     # res → pose kestirimi ile gerçek yan yana
        return "durduruldu"
    finally:
        f.close()
        print(f"[LEAD] IBVS lead pursuit durdu — log kapatıldı: {csv_yol}")
