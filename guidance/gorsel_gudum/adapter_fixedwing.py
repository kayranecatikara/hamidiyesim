"""
adapter_fixedwing.py — IBVS lead pursuit SABİT KANAT adaptörü (STUB).

Henüz uygulanmadı; çağrılırsa NotImplementedError fırlatır (sessizce geçmez).

Uygulanacağında (özet — ayrıntı docs/GUIDANCE_ROADMAP.md):
  - Komut yolu: SET_ATTITUDE_TARGET, roll_cmd/pitch_cmd → kuaterniyon,
    yaw mevcut heading'de tutulur (sabit kanat yana kayamaz, YATARAK döner:
    yatay hata SAPMA değil YATIŞ komutuna gider).
  - type_mask ile gövde açısal hızları ignore, sadece attitude + thrust.
  - Gaz: ArduPlane'de TECS yönetir; attitude hedefi verirken gaz politikası
    ayrı karardır (terminal fazda tipik: sabit yüksek GAZ_TERMINAL). TECS ile
    mod/parametre etkileşimi uygulanmadan önce ayrıca incelenecek.
"""


class FixedWingAdapter:
    def __init__(self, cfg=None):
        self.cfg = cfg

    def compute(self, *args, **kwargs):
        raise NotImplementedError(
            "Sabit kanat adaptörü henüz uygulanmadı (PLATFORM='fixedwing'). "
            "Copter için AVCI_PLATFORM=copter kullanın.")

    def command(self, *args, **kwargs):
        raise NotImplementedError(
            "Sabit kanat adaptörü henüz uygulanmadı (PLATFORM='fixedwing'). "
            "Copter için AVCI_PLATFORM=copter kullanın.")
