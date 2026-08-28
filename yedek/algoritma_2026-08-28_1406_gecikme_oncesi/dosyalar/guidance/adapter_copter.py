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
                kalite=1.0, terminal=False, azimut_kalite=1.0):
        """Saf hesap (test edilir; göndermez).
        attitude: (roll, pitch, yaw) radyan. dt: kare aralığı (s).
        kalite: pose kalitesi 0..1 (düşükse PN söner). terminal: menzil eşik altı
        (co-altitude yanlılığı aktif). azimut_kalite: 0..1, nişan dikeye yaklaşınca
        yaw_hata tanımsızlaşır (guidance_core) — yaw adımı bununla sönümlenir.
        Dönüş: dict(v_cmd, yaw_cmd, u_dunya, v_doygun, yaw_doygun, pn_dikey_deg,
        coalt_deg, yaw_adim_deg)."""
        cfg = self.cfg
        # Kadraj tutma için nişanın GÖVDE çerçevesindeki yükselişi gerekir —
        # hedefin sensörde nereye düştüğünü bu belirler (dünya yükselişi değil;
        # aradaki fark aracın kendi pitch'i). Kadraj merkezi = KAMERA_TILT_DEG.
        ug = np.asarray(u_govde, dtype=float)
        ug_n = ug / np.linalg.norm(ug)
        kadraj_elev = math.asin(clamp(-float(ug_n[2]), -1.0, 1.0))

        u_dunya = govde_to_dunya(u_govde, *attitude)
        u_dunya = u_dunya / np.linalg.norm(u_dunya)
        u_dunya, pn_lead, coalt = self._dikey_pn(u_dunya, dt, kalite, terminal,
                                                 kadraj_elev=kadraj_elev)
        v_hedef = cfg.V_KAPANMA * u_dunya

        # LİMİT dt'si: ham dt kare boşluğunda şişiyor (tespit kesilince
        # process() çağrılmıyor, boşluğun tamamı tek dt'ye biniyor). Hız/ivme
        # tavanları kare başına paya çevrilirken bu şişmiş dt kullanılırsa
        # "biriken hak" tek karede harcanıyor ve limit koruma sağlamıyor —
        # 160249 uçuşunda tek karede 74° yaw adımı. Ham dt _dikey_pn'de aynen
        # kaldı: orada dt bir türev paydası, harcanacak bir pay değil.
        dt_lim = min(dt, cfg.DT_TAVAN_S) if (dt is not None and dt > 0) else None

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

        # ±π'ye sarmala: mevcut_yaw+adim aksi halde ±3.8 rad'a çıkabiliyordu
        yaw_cmd = normalize_angle(mevcut_yaw + adim)

        return {"v_cmd": v_cmd, "yaw_cmd": yaw_cmd, "u_dunya": u_dunya,
                "v_doygun": v_doygun, "yaw_doygun": yaw_doygun,
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
                kalite=1.0, terminal=False, azimut_kalite=1.0):
        """Hesapla + gönder. Dönen dict CSV loguna girer."""
        out = self.compute(u_govde, yaw_hata, attitude, dt, mevcut_yaw,
                            kalite=kalite, terminal=terminal,
                            azimut_kalite=azimut_kalite)
        send_velocity(conn, out["v_cmd"][0], out["v_cmd"][1], out["v_cmd"][2],
                      out["yaw_cmd"])
        return out
