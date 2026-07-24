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

    def compute(self, u_govde, yaw_hata, attitude, dt, mevcut_yaw):
        """Saf hesap (test edilir; göndermez).
        attitude: (roll, pitch, yaw) radyan. dt: kare aralığı (s).
        Dönüş: dict(v_cmd, yaw_cmd, u_dunya, v_doygun, yaw_doygun)."""
        cfg = self.cfg
        u_dunya = govde_to_dunya(u_govde, *attitude)
        u_dunya = u_dunya / np.linalg.norm(u_dunya)
        v_hedef = cfg.V_KAPANMA * u_dunya

        if dt is None or dt <= 0:
            v_cmd = tuple(v_hedef)
            v_doygun = False
        else:
            v_cmd = limit_acceleration(v_hedef[0], v_hedef[1], v_hedef[2],
                                       *self.v_onceki, cfg.IVME_TAVAN, dt)
            v_doygun = (abs(v_cmd[0] - v_hedef[0]) + abs(v_cmd[1] - v_hedef[1])
                        + abs(v_cmd[2] - v_hedef[2])) > 1e-9
        self.v_onceki = v_cmd

        # Yaw: mevcut heading üstüne KP'li adım, YAW_HIZ_MAX ile slew-limitli
        # (agresif yaw quad'ı savurur, kamerayı bulanıklaştırır)
        adim_ham = cfg.KP_YAW * yaw_hata
        tavan = math.radians(cfg.YAW_HIZ_MAX) * (dt if dt else 1.0 / 30.0)
        adim = clamp(adim_ham, -tavan, tavan)
        yaw_doygun = abs(adim_ham) > tavan
        yaw_cmd = mevcut_yaw + adim

        return {"v_cmd": v_cmd, "yaw_cmd": yaw_cmd, "u_dunya": u_dunya,
                "v_doygun": v_doygun, "yaw_doygun": yaw_doygun}

    def command(self, conn, u_govde, yaw_hata, attitude, dt, mevcut_yaw):
        """Hesapla + gönder. Dönen dict CSV loguna girer."""
        out = self.compute(u_govde, yaw_hata, attitude, dt, mevcut_yaw)
        send_velocity(conn, out["v_cmd"][0], out["v_cmd"][1], out["v_cmd"][2],
                      out["yaw_cmd"])
        return out
