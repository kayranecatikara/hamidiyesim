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
"""

import math

import numpy as np

from control.guidance.common import clamp, limit_acceleration, send_velocity
from control.guidance.guidance_core import Cfg, govde_to_dunya


class CopterAdapter:
    """u_govde → NED hız vektörü + slew-limitli yaw komutu."""

    def __init__(self, cfg=Cfg):
        self.cfg = cfg
        self.v_onceki = (0.0, 0.0, 0.0)
        self.elev_f = None            # yumuşatılmış dikey aim yükseliş açısı (rad)
        self.elev_onceki = None       # dikey PN: önceki (yumuşatılmış) yükseliş açısı
        self.elev_rate_f = 0.0        # EMA'lı yükseliş oranı (rad/s)

    def _dikey_pn(self, u_dunya, dt, kalite, terminal):
        """Aim yönünü dikey düzlemde yukarı döndür. Sıra:
          (1) dikey aim YUMUŞAT (slew-kırpma + EMA) — bimodal kpt gürültüsünü kes,
          (2) PN lead: yumuşatılmış yükseliş ORANIYLA orantılı anticipasyon,
          (3) terminalde sabit co-altitude yanlılığı.
        |u| korunur (azimut sabit, yalnız yükseliş açısı değişir). Dönüş:
        (u_dunya_yeni, pn_lead_rad, coalt_rad)."""
        cfg = self.cfg
        elev_ham = math.asin(max(-1.0, min(1.0, -float(u_dunya[2]))))   # yukarı +
        az = math.atan2(float(u_dunya[1]), float(u_dunya[0]))

        # (1) DİKEY AIM YUMUŞATMA: tek-kare slew kırpma (gürültü sıçramasını sınırla)
        # + EMA. Ham türev yerine bunun türevi PN'e gider → chatter'sız.
        if self.elev_f is None:
            self.elev_f = elev_ham
        else:
            step = clamp(elev_ham - self.elev_f,
                         -math.radians(cfg.ELEV_STEP_MAX_DEG),
                         math.radians(cfg.ELEV_STEP_MAX_DEG))
            self.elev_f += cfg.ELEV_EMA * step
        elev = self.elev_f

        # (2) DİKEY PN: yumuşatılmış yükseliş oranıyla orantılı lead
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

        # (3) co-altitude: yalnız terminalde ve hedef ÜSTTEyken (aşağıdaki hedefte
        # yukarı yanlılık yanlış) — alttan sıyırma yerine seviyeye çıkıp kafa-kafaya
        coalt = math.radians(cfg.TERMINAL_COALT_DEG) if (terminal and elev > 0.0) else 0.0
        elev_yeni = elev + pn_lead + coalt
        ce = math.cos(elev_yeni)
        u_yeni = np.array([ce * math.cos(az), ce * math.sin(az), -math.sin(elev_yeni)])
        return u_yeni, pn_lead, coalt

    def kapanma_hizi(self, menzil_m, yandanlik, lead_rad):
        """LOS kapısı: bu geometride hedefi kadrajda tutabileceğimiz en yüksek
        kapanma hızı (m/s). SAF HESAP — birim test edilir.

        Bakış hattının dönme hızı ≈ (V_hedef·yandanlik + v·sin(lead)) / menzil.
        V_hedef ≈ K_LEAD·v olduğundan (K_LEAD tanımı gereği hız oranıdır):
            omega_LOS ≈ v · (K_LEAD·yandanlik + sin(lead)) / menzil
        Aracın izleyebildiği omega'ya eşitleyip v'yi çekeriz. Payda küçükse
        (kuyruk takibi) kısıt yoktur — tam V_KAPANMA döner.

        menzil_m None/≤0 ise (kestirim yok) kapı UYGULANMAZ: yanlış bir sıfır
        yüzünden aracı durdurmak, hızlı gitmekten daha kötü.
        Gerekçe ve ölçümler: guidance_core.Cfg.LOS_KAPISI notu."""
        cfg = self.cfg
        if not getattr(cfg, "LOS_KAPISI", True):
            return cfg.V_KAPANMA
        if menzil_m is None or menzil_m <= 0.0:
            return cfg.V_KAPANMA
        payda = (cfg.K_LEAD * max(0.0, min(1.0, yandanlik))
                 + abs(math.sin(lead_rad)))
        if payda < 1e-3:                       # tam kuyruk: LOS neredeyse dönmüyor
            return cfg.V_KAPANMA
        omega = math.radians(cfg.LOS_YAW_IZLENEBILIR) * cfg.LOS_PAY    # rad/s
        v = omega * menzil_m / payda
        return max(cfg.V_KAPANMA_MIN, min(cfg.V_KAPANMA, v))

    def compute(self, u_govde, yaw_hata, attitude, dt, mevcut_yaw,
                kalite=1.0, terminal=False,
                menzil_m=None, yandanlik=0.0, lead_rad=0.0):
        """Saf hesap (test edilir; göndermez).
        attitude: (roll, pitch, yaw) radyan. dt: kare aralığı (s).
        kalite: pose kalitesi 0..1 (düşükse PN söner). terminal: menzil eşik altı
        (co-altitude yanlılığı aktif).
        menzil_m/yandanlik/lead_rad: LOS kapısı girdileri — hepsi POSE'dan türer,
        ground truth DEĞİL (CLAUDE.md §8). menzil_m None ise kapı kapalıdır.
        Dönüş: dict(v_cmd, yaw_cmd, u_dunya, v_doygun, yaw_doygun,
        pn_dikey_deg, coalt_deg, v_kapanma)."""
        cfg = self.cfg
        u_dunya = govde_to_dunya(u_govde, *attitude)
        u_dunya = u_dunya / np.linalg.norm(u_dunya)
        u_dunya, pn_lead, coalt = self._dikey_pn(u_dunya, dt, kalite, terminal)
        v_kapanma = self.kapanma_hizi(menzil_m, yandanlik, lead_rad)
        v_hedef = v_kapanma * u_dunya

        if dt is None or dt <= 0:
            v_cmd = tuple(v_hedef)
            v_doygun = False
        else:
            v_cmd = limit_acceleration(v_hedef[0], v_hedef[1], v_hedef[2],
                                       *self.v_onceki, cfg.IVME_TAVAN, dt)
            v_doygun = (abs(v_cmd[0] - v_hedef[0]) + abs(v_cmd[1] - v_hedef[1])
                        + abs(v_cmd[2] - v_hedef[2])) > 1e-9
        # DİKEY TAVAN: aracın yapabildiğinin üstünü komutlamak dikey aşıma ve
        # (ölçüldü) çöküşe yol açıyor. Yatay bileşen korunur — nişan düzleşir.
        vz_tavan = getattr(cfg, "VZ_TAVAN", None)
        if vz_tavan is not None and abs(v_cmd[2]) > vz_tavan:
            v_cmd = (v_cmd[0], v_cmd[1], clamp(v_cmd[2], -vz_tavan, vz_tavan))
            v_doygun = True
        self.v_onceki = v_cmd

        # Yaw: mevcut heading üstüne KP'li adım, YAW_HIZ_MAX ile slew-limitli
        # (agresif yaw quad'ı savurur, kamerayı bulanıklaştırır)
        adim_ham = cfg.KP_YAW * yaw_hata
        tavan = math.radians(cfg.YAW_HIZ_MAX) * (dt if dt else 1.0 / 30.0)
        adim = clamp(adim_ham, -tavan, tavan)
        yaw_doygun = abs(adim_ham) > tavan
        yaw_cmd = mevcut_yaw + adim

        return {"v_cmd": v_cmd, "yaw_cmd": yaw_cmd, "u_dunya": u_dunya,
                "v_doygun": v_doygun, "yaw_doygun": yaw_doygun,
                "pn_dikey_deg": math.degrees(pn_lead),
                "coalt_deg": math.degrees(coalt),
                "v_kapanma": v_kapanma}

    def command(self, conn, u_govde, yaw_hata, attitude, dt, mevcut_yaw,
                kalite=1.0, terminal=False,
                menzil_m=None, yandanlik=0.0, lead_rad=0.0):
        """Hesapla + gönder. Dönen dict CSV loguna girer."""
        out = self.compute(u_govde, yaw_hata, attitude, dt, mevcut_yaw,
                            kalite=kalite, terminal=terminal,
                            menzil_m=menzil_m, yandanlik=yandanlik,
                            lead_rad=lead_rad)
        send_velocity(conn, out["v_cmd"][0], out["v_cmd"][1], out["v_cmd"][2],
                      out["yaw_cmd"])
        return out
