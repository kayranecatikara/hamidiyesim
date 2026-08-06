"""Kilit kriteri — SAF geometri modülü.

Tek bir bbox'ın (merkez cx,cy ve boyut w,h) şartname kilit koşulunu sağlayıp
sağlamadığını değerlendirir. I/O yok, thread yok, durum yok — sadece geometri.
bbox=None durumunu ÇAĞIRAN taraf yönetir.

Kurallar (bkz. CLAUDE.md — Geometri):
- merkez (cx,cy) AV = [0.25W, 0.75W] × [0.10H, 0.90H] içinde,
- boyut_ok = w >= ESIK_BILDIRIM·W VEYA h >= ESIK_BILDIRIM·H (eksenlerden en az biri),
- kilit = merkez_av_icinde VE boyut_ok,
- marj = max(w/(ESIK_BILDIRIM·W), h/(ESIK_BILDIRIM·H)).

Tüm eşikler config/kilit_sabitler.py'den gelir; piksel değerleri W,H'den türetilir.
"""

from dataclasses import dataclass

from config.kilit_sabitler import ESIK_BILDIRIM, av_sinirlari


@dataclass
class KilitKriter:
    merkez_av_icinde: bool
    boyut_ok: bool
    kilit: bool
    marj: float


def kriter_degerlendir(cx, cy, w, h, W, H):
    """bbox merkezini ve boyutunu kaynak kare (W,H) çerçevesinde değerlendirir.

    Döner: KilitKriter(merkez_av_icinde, boyut_ok, kilit, marj).
    """
    x_min, x_max, y_min, y_max = av_sinirlari(W, H)
    merkez_av_icinde = (x_min <= cx <= x_max) and (y_min <= cy <= y_max)

    min_en = ESIK_BILDIRIM * W
    min_boy = ESIK_BILDIRIM * H
    boyut_ok = (w >= min_en) or (h >= min_boy)

    marj = max(w / min_en, h / min_boy)

    kilit = merkez_av_icinde and boyut_ok

    return KilitKriter(
        merkez_av_icinde=merkez_av_icinde,
        boyut_ok=boyut_ok,
        kilit=kilit,
        marj=marj,
    )
