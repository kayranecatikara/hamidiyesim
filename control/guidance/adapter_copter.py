"""
adapter_copter.py — IBVS lead pursuit COPTER adaptörü (Adım 9, platforma bağlı).

Çekirdeğin ürettiği nişan yönünü (u_govde) multirotor komutuna çevirir.
SET_ATTITUDE_TARGET KULLANILMAZ — multirotorda attitude komutu bu iş için
yanlış araç (burun yukarı = tırmanış değil geri yavaşlama). Bunun yerine:
SET_POSITION_TARGET_LOCAL_NED, sadece HIZ + yaw aktif (common.send_velocity,
GPS hatlarıyla aynı kanıtlanmış yol). Gaz/eğim ArduCopter'ın iç kontrolcüsünün
işi — gaz politikası YOK (yedek attitude-yolu politikası docs/GUIDANCE_ROADMAP.md).

Quad'ın sabit kanada göre avantajı: nereye uçtuğun (v) ile nereye baktığın (yaw)
bağımsız — yaw hedefi kadrajda tutarken hız vektörü kesme rotasında kalır.

İvme tavanı gerekçesi: quad ileri ivmelenmek için burnunu aşağı eğer; kamera
gövdeye +25° bağlı olduğundan ~5 m/s² üstünde kamera dünyada AŞAĞI bakmaya
başlar (gökyüzü arka planı kaybolur, yer karmaşası tespit modeline girer).
IVME_TAVAN=4 m/s² bu zarfın içinde kalır; hız komutu rampayla uygulanır.

Limit dt tavanı gerekçesi: YAW_HIZ_MAX ve IVME_TAVAN birer HIZ limitidir, kare
başına paya çevrilirken dt ile çarpılır. dt tespit kesildiğinde şişer (boşluğun
tamamı tek dt'ye biner) ve biriken hak tek karede harcanır — 160249 uçuşunda
dt=0.825 s → 74°'lik yaw adımı tek mesajda gitti. Limit hesabı Cfg.DT_TAVAN_S
ile kırpılmış dt kullanır; ham dt yalnız türev/zaman sabiti yerlerinde kalır.

Yaw kapısı gerekçesi: yaw_hata gövde azimutudur (atan2), hedef kadraj tepesine
çıkınca TANIMSIZLAŞIR — küçük gürültü ±180° savurur ve quad kendi etrafında
döner (141017 uçuşu: 4.1 s'te 637°). guidance_core her karede azimut_kalite
(0..1) üretir, yaw adımı onunla çarpılır: tepedeki hedefte yaw susar, düzeltmeyi
dikey kanal (PN + co-altitude) yapar.
"""

import math

import numpy as np

from control.guidance.common import (
    clamp, limit_acceleration_split, normalize_angle, send_velocity)
from control.guidance.guidance_core import Cfg, govde_to_dunya


class CopterAdapter:
    """u_govde → NED hız vektörü + slew-limitli yaw komutu."""

    def __init__(self, cfg=Cfg):
        self.cfg = cfg
        self.v_onceki = (0.0, 0.0, 0.0)
        self.elev_f = None            # yumuşatılmış LOS yükseliş açısı λ (rad)
        self.elev_onceki = None       # önceki λ (kare-arası Δλ için)
        self.elev_rate_f = 0.0        # EMA'lı λ̇ (rad/s) — yalnız log/gözlem
        self.gama = None              # komut edilen uçuş yolu yükselişi (rad, log)
        self.kadraj_duz = 0.0         # kadraj tutma düzeltmesi (rad, log)
        # Yaw kaçak kapısı durumu (bkz. compute): kaç ardışık karedir adım
        # tavanda OLUP hata kapanmıyor, ve kıyas için son |yaw_hata|.
        self._yaw_doygun_n = 0
        self._yaw_hata_ref = None
        self._yaw_sus_n = 0           # kaç karedir SUSTURULMUŞ (süreli susma)
        # Yatay (azimut-oranı) lead durumu — bkz. _yatay_pn
        self.az_f = None              # yumuşatılmış LOS azimutu (rad)
        self.az_onceki = None         # önceki azimut (kare-arası Δaz için)
        self.az_rate_f = 0.0          # EMA'lı azimut oranı (rad/s)
        self.yatay_lead = 0.0         # son uygulanan yatay lead (rad, log)
        # v_onceki aracın gerçek hızıyla tohumlandı mı (bkz. hiz_tohumla).
        # İlk karede limitleme YALNIZ tohumlandıysa açılır: yanlış referanstan
        # (0,0,0) limitlemek basamak komutundan DAHA kötü — sahte fren olur.
        self._tohumlandi = False

    def hiz_tohumla(self, v):
        """İvme limitleyicisinin referansını aracın GERÇEK hızına kur.

        Adaptör her görsel fazda YENİDEN kurulur ve `v_onceki` (0,0,0)'dan
        başlar. Ama araç durmuyor: GPS fazından 18 m/s'e varan hızla geliyor.
        Bu yanlış başlangıç durumu iki ayrı hataya yol açıyordu —
        limitleme atlanınca basamak komutu, limitleme açılınca sahte fren.
        Faz başında bir kez çağrılır; `None` gelirse (telemetri henüz yok)
        dokunmaz ve False döner — o durumda ilk kare limitlenmez.
        """
        if v is None:
            return False
        self.v_onceki = (float(v[0]), float(v[1]), float(v[2]))
        self._tohumlandi = True
        return True

    def _yatay_pn(self, u_dunya, dt, kalite, azimut_kalite=1.0):
        """AZİMUT-ORANI LEAD — pose şekil-lead'inin yerine (2026-08-06).

        Nişan yönünü YATAY düzlemde, LOS azimutunun değişim oranıyla orantılı
        kadar öne döndürür. Yükseliş açısına DOKUNMAZ (dikey kanal ayrı).
        Dönüş: (u_dunya_yeni, lead_rad).

        NEDEN BU SİNYAL: pose kaldırılınca yandanlıktan türeyen lead de gitti.
        Hedefin yanlamasına geçişi LOS azimutunu döndürür; o dönüşün oranı
        "hedef ne kadar hızlı yanımızdan geçiyor"un doğrudan ölçüsüdür ve
        keypoint gerektirmez. Saf takip + bu terim = lead pursuit.

        YUMUŞATMA ZİNCİRİ DİKEY KANALLA BİREBİR AYNI (slew-kırpma → EMA → oran
        EMA) ve bu bilinçli: ham azimut oranı çok gürültülü (08-06 ölçümü,
        n=10 183: |oran| medyan 15.8 °/s ama p90 187 °/s — kuyruk tamamen
        gürültü). Zincir uygulandığında lead dağılımı sökülen şekil-lead'iyle
        aynı banda oturuyor (ort 6.08° vs 6.18°).

        AZİMUT DAİRESELDİR: farklar normalize_angle'dan geçirilir, yoksa
        ±180° geçişinde tek karede sahte 360°/dt oranı üretilir.

        ── KAPI: ölçek kalitesi DEĞİL, azimut kalitesi (2026-08-08) ──
        Lead eskiden `kalite` ile çarpılıyordu. `kalite` bbox GENİŞLİĞİnden
        türer (menzil vekili, 22.5 m'de 0 / 9.6 m'de 1) — oysa buradaki sinyal
        bbox MERKEZİdir ve menzilden bağımsız güvenilirdir. Kadraj tutma terimi
        aynı gerekçeyle zaten `kalite` ile çarpılmıyor (bkz. _dikey_pn (3)).

        ÖLÇÜM (08-08, 4 oturum, 60 görsel faz): görsel faz hedef DÜZ uçarken
        39/39 kapanıyor, DÖNERKEN 0/14. Dönüş karelerinde (n=348) tespit
        sağlamdı — güven 0.79, bbox merkez sapması 1.8° — ama:

            gerçek menzil 15.3 m → kalite 0.00
            |az_rate| 71.8 °/s → gereken lead 20° (tavan)
            FİİLEN UYGULANAN LEAD: 0.0°

        Yani lead tam ihtiyaç duyulan anda kapalıydı; hedef kadrajdan yandan
        süpürülüp çıkıyor, 37 kare tespitsiz kalınca faz GPS'e dönüyordu
        (tespit_yok karelerinin %91'inde hedef GERÇEKTEN kadraj dışında).

        azimut_kalite bu uçuşlarda medyan 1.000 — pratikte "ölçek kapısını
        kaldır, tekil geometri (nişan dikeye yaklaşınca) koruması kalsın"
        demek. Gürültü koruması değişmedi: AZ_STEP_MAX + AZ_EMA + PN_RATE_EMA
        + ±PN_YATAY_MAX tavanı.

        ⚠ Dönüşte gereken lead 43°, tavan 20° — DOYAR. Tavanı yükseltmek AYRI
        bir deneydir, bu koşuya karıştırılmadı.

        Eski davranışa dönüş: AVCI_IBVS_PN_YATAY_KAPI=olcek
        """
        cfg = self.cfg
        elev = math.asin(max(-1.0, min(1.0, -float(u_dunya[2]))))
        az_ham = math.atan2(float(u_dunya[1]), float(u_dunya[0]))

        if self.az_f is None:
            self.az_f = az_ham
        else:
            step = clamp(normalize_angle(az_ham - self.az_f),
                         -math.radians(cfg.AZ_STEP_MAX_DEG),
                         math.radians(cfg.AZ_STEP_MAX_DEG))
            self.az_f = normalize_angle(self.az_f + cfg.AZ_EMA * step)
        az = self.az_f

        lead = 0.0
        if (cfg.PN_YATAY_SURE > 0.0 and dt is not None and 0.0 < dt <= 0.2
                and self.az_onceki is not None):
            rate = normalize_angle(az - self.az_onceki) / dt
            self.az_rate_f = (cfg.PN_RATE_EMA * rate
                              + (1.0 - cfg.PN_RATE_EMA) * self.az_rate_f)
            kapi = kalite if cfg.PN_YATAY_KAPI == "olcek" else azimut_kalite
            lead = clamp(
                cfg.PN_YATAY_SURE * self.az_rate_f * max(0.0, min(1.0, kapi)),
                -math.radians(cfg.PN_YATAY_MAX_DEG),
                math.radians(cfg.PN_YATAY_MAX_DEG))
        self.az_onceki = az
        self.yatay_lead = lead

        az_yeni = normalize_angle(az + lead)
        ce = math.cos(elev)
        return (np.array([ce * math.cos(az_yeni), ce * math.sin(az_yeni),
                          -math.sin(elev)]), lead)

    def _dikey_pn(self, u_dunya, dt, kalite, terminal, kadraj_elev=None):
        """Aim yönünü dikey düzlemde yukarı döndür. Sıra:
          (1) dikey aim YUMUŞAT (slew-kırpma + EMA) — bimodal kpt gürültüsünü kes,
          (2) lead: yumuşatılmış yükseliş ORANIYLA orantılı anticipasyon,
          (3) terminalde sabit co-altitude yanlılığı.
        |u| korunur (azimut sabit, yalnız yükseliş açısı değişir). Dönüş:
        (u_dunya_yeni, pn_lead_rad, coalt_rad).

        GERÇEK PN DENENDİ VE GERİ ALINDI (2026-07-31). `γ += N·Δλ` biçiminde
        klasik oransal seyrüsefer uygulandı; kapalı çevrim simülasyonda ıska
        0.66 m → 1.5-2.1 m'ye ÇIKTI. Sebep: devir anında λ zaten doğal olarak
        azalıyor (drone hedefe yakınsıyor), PN bunu "sıfırlanacak LOS hızı"
        sanıp yakınsamayla savaşıyor ve γ eksiye (dalışa) gidiyor — hedef
        yukarıdayken. PN küçük sapmaları düzeltmek için tasarlanmıştır, büyük
        bir başlangıç ofsetini kapatmak için değil.

        ASIL SORUN BURADA DEĞİL: ölçüm (193548 uçuşu) drone'un dikey komutu
        sadıkla uyguladığını gösteriyor (komut −0.32 m/s, gerçekleşen
        −0.39 m/s). Komutun KENDİSİ küçük, çünkü GPS fazı istasyonda
        (hedefin 4.65 m altında) durmak üzere tasarlanmış ve görsel faz
        devraldığında (6-10 m) farkı kapatacak zaman kalmıyor. Düzeltme GPS
        fazının istasyon geometrisine ait."""
        cfg = self.cfg
        elev_ham = math.asin(max(-1.0, min(1.0, -float(u_dunya[2]))))   # yukarı +
        az = math.atan2(float(u_dunya[1]), float(u_dunya[0]))

        # (1) λ YUMUŞATMA: tek-kare slew kırpma + EMA. Δλ bunun üstünden alınır,
        # ham keypoint gürültüsü doğrudan γ'ya integre edilmesin.
        if self.elev_f is None:
            self.elev_f = elev_ham
        else:
            step = clamp(elev_ham - self.elev_f,
                         -math.radians(cfg.ELEV_STEP_MAX_DEG),
                         math.radians(cfg.ELEV_STEP_MAX_DEG))
            self.elev_f += cfg.ELEV_EMA * step
        elev = self.elev_f                      # λ

        # (2) DİKEY LEAD: yumuşatılmış yükseliş oranıyla orantılı anticipasyon
        pn_lead = 0.0
        if dt is not None and 0.0 < dt <= 0.2 and self.elev_onceki is not None:
            rate = (elev - self.elev_onceki) / dt
            self.elev_rate_f = (cfg.PN_RATE_EMA * rate
                                + (1.0 - cfg.PN_RATE_EMA) * self.elev_rate_f)
            pn_lead = clamp(
                cfg.PN_LEAD_SURE * self.elev_rate_f * max(0.0, min(1.0, kalite)),
                -math.radians(cfg.PN_DIKEY_MAX_DEG),
                math.radians(cfg.PN_DIKEY_MAX_DEG))
        self.elev_onceki = elev

        # (3) KADRAJ TUTMA — "metre altta kal" değil "AÇI altta kal".
        # kadraj_elev = nişanın GÖVDE çerçevesindeki yükselişi; kadraj merkezi
        # KAMERA_TILT_DEG'e karşılık gelir. Hedef merkezden yukarı kaçtıysa
        # (sabit metre ofset, menzil kapandıkça bunu ZORUNLU olarak yapar)
        # nişanı orantılı yukarı it → drone daha çok tırmanır → dikey ofset
        # menzille orantılı küçülür → sabit görüş açısı = çarpışma rotası.
        # kalite ile ÇARPILMAZ: kalite keypoint'lerden gelir ve lead'i kapatır;
        # buradaki sinyal bbox merkezidir, keypoint'lerden bağımsız güvenilir.
        # SİMETRİK: hedef merkezin altındaysa nişan aşağı iner (gereksiz
        # tırmanmayı önler, kadrajı ortada tutar).
        kadraj_duz = 0.0
        if kadraj_elev is not None:
            kadraj_duz = clamp(
                cfg.KP_KADRAJ * (kadraj_elev - math.radians(cfg.KAMERA_TILT_DEG)),
                -math.radians(cfg.KADRAJ_MAX_DEG),
                math.radians(cfg.KADRAJ_MAX_DEG))
        self.kadraj_duz = kadraj_duz
        self.gama = elev + pn_lead + kadraj_duz    # yalnız log/gözlem

        # (4) co-altitude: yalnız terminalde ve hedef ÜSTTEyken (aşağıdaki hedefte
        # yukarı yanlılık yanlış) — alttan sıyırma yerine seviyeye çıkıp kafa-kafaya.
        coalt = math.radians(cfg.TERMINAL_COALT_DEG) if (terminal and elev > 0.0) else 0.0
        elev_yeni = self.gama + coalt
        ce = math.cos(elev_yeni)
        u_yeni = np.array([ce * math.cos(az), ce * math.sin(az), -math.sin(elev_yeni)])
        return u_yeni, pn_lead, coalt

    def compute(self, u_govde, yaw_hata, attitude, dt, mevcut_yaw,
                kalite=1.0, terminal=False, azimut_kalite=1.0, menzil=None):
        """Saf hesap (test edilir; göndermez).
        attitude: (roll, pitch, yaw) radyan. dt: kare aralığı (s).
        kalite: pose kalitesi 0..1 (düşükse PN söner). terminal: menzil eşik altı
        (co-altitude yanlılığı aktif). azimut_kalite: 0..1, nişan dikeye yaklaşınca
        yaw_hata tanımsızlaşır (guidance_core) — yaw adımı bununla sönümlenir.
        menzil: hedefe gerçek mesafe (m) — YAKLAŞMA alt-fazı için; None ise
        alt-faz devre dışı, davranış eskisiyle birebir aynı.
        Dönüş: dict(v_cmd, yaw_cmd, u_dunya, v_doygun, yaw_doygun, pn_dikey_deg,
        coalt_deg, yaw_adim_deg, alt_faz)."""
        cfg = self.cfg
        # Kadraj tutma için nişanın GÖVDE çerçevesindeki yükselişi gerekir —
        # hedefin sensörde nereye düştüğünü bu belirler (dünya yükselişi değil;
        # aradaki fark aracın kendi pitch'i). Kadraj merkezi = KAMERA_TILT_DEG.
        ug = np.asarray(u_govde, dtype=float)
        ug_n = ug / np.linalg.norm(ug)
        kadraj_elev = math.asin(clamp(-float(ug_n[2]), -1.0, 1.0))

        u_dunya = govde_to_dunya(u_govde, *attitude)
        u_dunya = u_dunya / np.linalg.norm(u_dunya)
        # SIRA ÖNEMLİ: yatay kanal yükselişe dokunmaz, dikey kanal azimuta
        # dokunmaz — ama dikey kanal azimutu yeniden okuduğu için yatay lead
        # ÖNCE uygulanmalı, yoksa dikey adım lead'siz azimutu geri yazar.
        u_dunya, yatay_lead = self._yatay_pn(u_dunya, dt, kalite, azimut_kalite)
        u_dunya, pn_lead, coalt = self._dikey_pn(u_dunya, dt, kalite, terminal,
                                                 kadraj_elev=kadraj_elev)
        # ── YAKLAŞMA ALT-FAZI (2026-08-05) ──
        # Görsel faz eskiden tek davranıştı: devir anından itibaren sabit
        # V_KAPANMA ile hedefe dalıyordu. Ölçüldü — kapanma hızı menzilden
        # BAĞIMSIZ: 0-2 m bandında bile medyan 24.4 m/s. 30 Hz'de bu kare
        # başına 0.81 m yol; ıska mesafesi medyanı 0.65-0.85 m, yani tam bir
        # karelik yol. Nişan kusursuz olsa bile araç hedefi iki kare arasında
        # atlıyordu.
        #
        # Ayrıca dikey bileşen v_hedef = V_KAPANMA · u_dunya tanımından geliyor:
        # nişan 30° yukarıysa 12.5 m/s tırmanma. Dikey hıza AYRI TAVAN YOKTU
        # (IVME_TAVAN_DIKEY yalnız değişim hızını sınırlar), araç istasyonun
        # 5-8 m üstüne fırlıyordu.
        #
        # Yeni davranış — görsel faz ikiye ayrıldı:
        #   YAKLAŞMA (menzil > TERMINAL_MENZIL): yatay hız V_YAKLASMA'ya
        #     düşürülür, dikey hız hedefle İRTİFA FARKINI kapatmaya ayrılır
        #     (VZ_YAKLASMA tavanlı). Amaç: terminale hedefle aynı seviyede,
        #     düz ve yavaş girmek.
        #   TERMINAL (menzil <= TERMINAL_MENZIL): eski davranış, tam V_KAPANMA.
        #
        # GPS fazına DOKUNULMAZ — istasyon geometrisi olduğu gibi kalır; bu
        # ayrışma yalnız devir anından sonra başlar.
        alt_faz = "terminal"
        yaklasma = (menzil is not None and menzil > cfg.TERMINAL_MENZIL
                    and cfg.V_YAKLASMA > 0
                    # Üst sınır: çok uzakta yavaşlama yerine tam hız kapanma
                    # (GT modunda devir 20+ m'de olabiliyor — bkz. Cfg notu).
                    and (cfg.YAKLASMA_MAX_MENZIL <= 0
                         or menzil <= cfg.YAKLASMA_MAX_MENZIL))
        if yaklasma:
            alt_faz = "yaklasma"
            yatay_n = math.hypot(float(u_dunya[0]), float(u_dunya[1]))
            if yatay_n > 1e-6:
                # Yatay yön korunur, büyüklük V_YAKLASMA'ya çekilir.
                vx = cfg.V_YAKLASMA * float(u_dunya[0]) / yatay_n
                vy = cfg.V_YAKLASMA * float(u_dunya[1]) / yatay_n
            else:
                vx = vy = 0.0
            # Dikey: hedefin bize göre yükseklik farkı ≈ −u_dunya[2] · menzil
            # (NED: z aşağı pozitif, u_dunya[2] negatifse hedef YUKARIDA).
            # Bunu KP_IRTIFA kazancıyla kapat, VZ_YAKLASMA ile tavanla.
            dikey_fark = -float(u_dunya[2]) * menzil          # + ise hedef yukarıda
            vz = clamp(-cfg.KP_IRTIFA * dikey_fark,
                       -cfg.VZ_YAKLASMA, cfg.VZ_YAKLASMA)     # NED: -vz = tırmanma
            v_hedef = np.array([vx, vy, vz])
        else:
            v_hedef = cfg.V_KAPANMA * u_dunya
            # Terminalde de dikey hızın kendi tavanı var: nişan dikeye
            # yaklaşınca V_KAPANMA'nın tamamı dikeye geçebiliyordu (25 m/s).
            if cfg.VZ_TERMINAL_MAX > 0 and abs(v_hedef[2]) > cfg.VZ_TERMINAL_MAX:
                v_hedef = np.array([v_hedef[0], v_hedef[1],
                                    math.copysign(cfg.VZ_TERMINAL_MAX, v_hedef[2])])

        # LİMİT dt'si: ham dt kare boşluğunda şişiyor (tespit kesilince
        # process() çağrılmıyor, boşluğun tamamı tek dt'ye biniyor). Hız/ivme
        # tavanları kare başına paya çevrilirken bu şişmiş dt kullanılırsa
        # "biriken hak" tek karede harcanıyor ve limit koruma sağlamıyor —
        # 160249 uçuşunda tek karede 74° yaw adımı. Ham dt _dikey_pn'de aynen
        # kaldı: orada dt bir türev paydası, harcanacak bir pay değil.
        dt_lim = min(dt, cfg.DT_TAVAN_S) if (dt is not None and dt > 0) else None

        # ── İLK KARE (dt=None) — 2026-08-09 ────────────────────────────────
        # Eskiden dt None ise limitleme TÜMÜYLE atlanıyordu ve komut doğrudan
        # v_hedef oluyordu. dt her GÖRSEL FAZIN ilk karesinde None'dır, yani
        # bu "nadir kenar durum" değil, her devirde bir kez yaşanan kuraldı.
        # Ölçüldü (08-09 uçuşu, altı fazın ilk karesi):
        #     25.00 · 12.13 · 25.00 · 20.16 · 23.87 · 21.72 m/s   (hepsi dt='')
        # Fazlar 0.3-5 s sürdüğü için bu basamak fazın kendisiyle aynı
        # mertebedeydi.
        #
        # ⚠ Limitlemeyi açmak TEK BAŞINA yetmez: v_onceki (0,0,0)'dan başlıyor
        # ama araç DURMUYOR — GPS fazından 18 m/s ile geliyor. Sıfırdan
        # limitlemek bu kez yapay bir yavaşlama üretirdi. Çözüm iki parçalı:
        #   (a) v_onceki aracın GERÇEK hızıyla tohumlanır (hiz_tohumla)
        #   (b) burada nominal dt ile limitleyici çalıştırılır
        # Böylece ne basamak kalır ne de sahte fren.
        #
        # ⚠ TOHUMLAMA ŞARTI: telemetri ilk karede henüz gelmemişse v_onceki
        # hâlâ (0,0,0)'dır. O referanstan limitlemek 20 m/s yerine 0.4 m/s
        # komut ederdi — basamaktan DAHA kötü, sahte fren. Tohumlanmadıysa
        # eski (limitsiz) davranış korunur.
        if dt_lim is None and cfg.ILK_KARE_LIMIT and self._tohumlandi:
            dt_lim = cfg.DT_TAVAN_S

        if dt_lim is None:
            v_cmd = tuple(v_hedef)
            v_doygun = False
        else:
            # Yatay/dikey AYRI tavan: yatayı kamera kısıtlar (burun eğimi),
            # dikeyi yalnız itki bütçesi. Ayrıntı: common.limit_acceleration_split
            v_cmd = limit_acceleration_split(
                v_hedef[0], v_hedef[1], v_hedef[2], *self.v_onceki,
                cfg.IVME_TAVAN, cfg.IVME_TAVAN_DIKEY, dt_lim)
            v_doygun = (abs(v_cmd[0] - v_hedef[0]) + abs(v_cmd[1] - v_hedef[1])
                        + abs(v_cmd[2] - v_hedef[2])) > 1e-9
        self.v_onceki = v_cmd

        # Yaw: mevcut heading üstüne KP'li adım, YAW_HIZ_MAX ile slew-limitli
        # (agresif yaw quad'ı savurur, kamerayı bulanıklaştırır).
        # AZİMUT KAPISI: nişan dikeye yaklaştığında yaw_hata atan2 tekilliğinden
        # ötürü ±180° savrulur; azimut_kalite ile söndürülür (1=güvenilir, 0=kapalı).
        #
        # NOT — "mutlak hedefe slew" denendi ve GERİ ALINDI (2026-08-01):
        # cmd_yaw'ı kalıcı durumda tutup hedef_yaw = mevcut_yaw + yaw_hata'ya
        # slew etmek, GPS fazındaki desene benziyor ama BURADA daha kötü.
        # Kapalı çevrim ölçümü (algı hatayı güncellemediği arıza koşulunda,
        # 30 s): mevcut kod 1.0 tur, "mutlak" biçim 7.4 tur döndü. Sebep:
        # mevcut biçim komutu her karede aracın GERÇEK başlığına yeniden
        # demirler, yani komut asla actual+adim'den fazla öne geçemez — bu bir
        # güvenlik özelliğidir. Kalıcı cmd_yaw bu demirlemeyi kaybeder.
        adim_ham = cfg.KP_YAW * yaw_hata * clamp(azimut_kalite, 0.0, 1.0)
        tavan = math.radians(cfg.YAW_HIZ_MAX) * (dt_lim if dt_lim else 1.0 / 30.0)
        adim = clamp(adim_ham, -tavan, tavan)
        yaw_doygun = abs(adim_ham) > tavan

        # ── SÜREKLİ DOYGUNLUK KAPISI (2026-08-01 dönme sınırlaması) ──
        # Ölçüm: yaw_doygun karelerin %91-100'ünde 1 ve adımlar TEK YÖNLÜ
        # (bir logda 28 negatif / 4 pozitif). Adım sürekli tavandaysa yaw_hata
        # kapanmıyor demektir — yani algı, dönüşe rağmen aynı hatayı bildiriyor
        # (bayat/hatalı ölçüm). O durumda dönmeye devam etmek hatayı kapatmaz,
        # yalnız aracı çevirir: araç 443 s'de 33.6 tur döndü, DesYaw dönüşü
        # birebir takip etti (yani komut buydu) ve motorlar HİÇ doymadı (%0.0).
        # Ortalama 91.5 °/s ≈ YAW_HIZ_MAX(90) — tam olarak "her karede tavan
        # adımı" imzası.
        # Ölçüt DOYGUNLUK DEĞİL, HATANIN KAPANMAMASI. Büyük ama meşru bir dönüş
        # de doygundur (60°'lik hata ~20 kare tavanda kalır) — onu kesmemeliyiz.
        # Ayırt edici soru: tavan adımı komut ettik, hata buna karşılık AZALDI mı?
        #   azaldıysa  → döngü kapanıyor, algı sağlıklı, sayaç sıfırlanır
        #   azalmadıysa→ dönüşe rağmen aynı hata bildiriliyor (bayat/hatalı ölçüm)
        # Eşik: komut edilen adımın en az dörtte biri kadar hata azalması.
        if yaw_doygun:
            ilerleme = (None if self._yaw_hata_ref is None
                        else self._yaw_hata_ref - abs(yaw_hata))
            if ilerleme is not None and ilerleme > 0.25 * abs(adim):
                self._yaw_doygun_n = 0        # hata kapanıyor → yetki tam
            else:
                self._yaw_doygun_n += 1
            self._yaw_hata_ref = abs(yaw_hata)
        else:
            self._yaw_doygun_n = 0
            self._yaw_hata_ref = None
        if self._yaw_doygun_n > cfg.YAW_DOYGUN_N:
            adim = 0.0            # ölçüm loop'u kapatmıyor → yaw'ı sustur
            # ⚠ SÜRELİ SUSMA (2026-08-06) — gps_guidance ile aynı kilit burada
            # da vardı: kapı adımı 0 yapar → araç dönmez → hata kapanmaz →
            # kapı hiç açılmaz. GPS fazında ölçüldü: karelerin %7.8'inde burun
            # >20° sapmışken adım tam 0, en uzun kesintisiz susma 93 SANİYE.
            # Susma artık süreli: YAW_SUS_N kare sonra yetki geri verilir.
            # Sorun gerçekse kapı yeniden tetiklenir → dönme oranı
            # YAW_DOYGUN_N/(YAW_DOYGUN_N+YAW_SUS_N) kadar kısılır, kilit olmaz.
            self._yaw_sus_n += 1
            if self._yaw_sus_n >= cfg.YAW_SUS_N:
                self._yaw_doygun_n = 0
                self._yaw_hata_ref = None
                self._yaw_sus_n = 0
        else:
            self._yaw_sus_n = 0

        # ±π'ye sarmala: mevcut_yaw+adim aksi halde ±3.8 rad'a çıkabiliyordu
        yaw_cmd = normalize_angle(mevcut_yaw + adim)

        return {"v_cmd": v_cmd, "yaw_cmd": yaw_cmd, "u_dunya": u_dunya,
                "v_doygun": v_doygun, "yaw_doygun": yaw_doygun,
                "alt_faz": alt_faz,
                "yatay_lead_deg": math.degrees(yatay_lead),
                "az_rate_dps": math.degrees(self.az_rate_f),
                "pn_dikey_deg": math.degrees(pn_lead),
                "coalt_deg": math.degrees(coalt),
                "yaw_adim_deg": math.degrees(adim),
                # PN gözlem üçlüsü: λ (LOS), λ̇ (başarı ölçütü — sıfıra yakınsa
                # çarpışma rotasındayız), γ (komut edilen uçuş yolu açısı)
                "los_elev_deg": math.degrees(self.elev_f or 0.0),
                "los_elev_rate_dps": math.degrees(self.elev_rate_f),
                "gama_deg": math.degrees(self.gama if self.gama is not None else 0.0),
                # kadraj tutma: hedefin kadraj MERKEZİNE göre yeri ve düzeltme.
                # kadraj_hata_deg 0 ⇒ hedef tam merkezde (= açı korunuyor).
                "kadraj_hata_deg": math.degrees(kadraj_elev
                                                - math.radians(cfg.KAMERA_TILT_DEG)),
                "kadraj_duz_deg": math.degrees(self.kadraj_duz)}

    def command(self, conn, u_govde, yaw_hata, attitude, dt, mevcut_yaw,
                kalite=1.0, terminal=False, azimut_kalite=1.0, menzil=None):
        """Hesapla + gönder. Dönen dict CSV loguna girer."""
        out = self.compute(u_govde, yaw_hata, attitude, dt, mevcut_yaw,
                            kalite=kalite, terminal=terminal,
                            azimut_kalite=azimut_kalite, menzil=menzil)
        send_velocity(conn, out["v_cmd"][0], out["v_cmd"][1], out["v_cmd"][2],
                      out["yaw_cmd"])
        return out
