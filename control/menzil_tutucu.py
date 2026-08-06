"""control/menzil_tutucu.py — Marj geri beslemeli mesafe tutucu (Adım 7).

"Öğrenilmiş standoff" katmanı: kilit/takip (VISUAL) fazında, hedefin görüntüdeki
görünür boyutunu (marj) MARJ_REF..MARJ_UST bandında tutacak biçimde GPS istasyon
menzil referansını (gps_guidance.Cfg.RANGE_SET) YAVAŞ dış döngüde ayarlar.

- Kontrol geri beslemesi: detection_state kilit durumundaki 'marj' (görüntü).
- Mesafe ölçümü (telemetri/gözlem): hedef GPS'inden gelen menzil (PnP değil).
- Faz kapısı: YALNIZ VISUAL'de aktif. ANGAJMAN (sistematik kapanış) ve GPS'te
  PASİF — referansa dokunmaz. Etki, hibrit döngü GPS'e döndüğünde standoff
  olarak gerçekleşir (VISUAL'de gps döngüsü koşmaz).
- Adım sınırlı (ani sıçrama yok) + alt/üst clamp.

SAF karar mantığı (MenzilTutucu.adim) I/O'suzdur ve test edilebilir; thread
koşucusu (calistir) callable'larla ayrık tutulur — gps_guidance/supervisor
import EDİLMEZ, dolayısıyla dokunulmazlara dokunulmaz.
"""

import threading
import time

from config.kilit_sabitler import (
    MARJ_REF,
    MARJ_UST,
    MENZIL_ADIM_M,
    MENZIL_REF_ALT,
    MENZIL_REF_UST,
    TUTUCU_HZ,
)

# Telemetri (gcs/UI okur; Adım 8 overlay'i bunu gösterecek)
status = {"aktif": False, "menzil_ref": None, "marj": None}

# calistir tek sefer başlasın (run_hybrid defalarca çağrılıyor)
_baslatildi = False
_baslat_lock = threading.Lock()


class MenzilTutucu:
    """Marj → menzil referansı SAF kontrol yasası. Durum: yalnız menzil_ref."""

    def __init__(self, baslangic_ref,
                 marj_ref=MARJ_REF, marj_ust=MARJ_UST,
                 adim=MENZIL_ADIM_M, alt=MENZIL_REF_ALT, ust=MENZIL_REF_UST):
        self.ref = float(baslangic_ref)
        self.marj_ref = marj_ref
        self.marj_ust = marj_ust
        self.adim = adim
        self.alt = alt
        self.ust = ust

    def adim_uygula(self, faz, marj):
        """Bir dış-döngü adımı. Dönüş: (aktif: bool, yeni_ref: float|None).

        yeni_ref None ise referans DEĞİŞMEDİ (bant içi / sinyal yok / pasif).
        """
        if faz != "VISUAL":
            return (False, None)               # GPS / ANGAJMAN / diğer → pasif
        if marj is None:
            return (True, None)                # görüntü sinyali yok → dokunma
        if marj < self.marj_ref:
            yeni = self.ref - self.adim        # hedef küçük/uzak → yaklaş
        elif marj > self.marj_ust:
            yeni = self.ref + self.adim        # hedef büyük/yakın → uzaklaş
        else:
            return (True, None)                # bant içi → dokunma
        yeni = min(self.ust, max(self.alt, yeni))   # clamp
        if yeni == self.ref:
            return (True, None)                # clamp'te sabitlendi (ör. ALT'ta)
        self.ref = yeni
        return (True, yeni)


def calistir(stop_event, get_faz, get_marj, get_range, set_range, hz=TUTUCU_HZ):
    """Tutucu dış döngüsünü daemon thread'de başlatır (bir kez).

    get_faz()   -> supervisor faz etiketi (str)
    get_marj()  -> güncel marj (float|None)
    get_range() -> güncel menzil referansı (float)   [başlangıç için]
    set_range(r)-> menzil referansını yaz (gps_guidance.Cfg.RANGE_SET)
    """
    global _baslatildi
    with _baslat_lock:
        if _baslatildi:
            return
        _baslatildi = True

    tutucu = MenzilTutucu(get_range())
    status["menzil_ref"] = tutucu.ref
    periyot = 1.0 / hz

    def _dongu():
        while not stop_event.is_set():
            marj = get_marj()
            aktif, yeni = tutucu.adim_uygula(get_faz(), marj)
            if yeni is not None:
                set_range(yeni)
            status["aktif"] = aktif
            status["marj"] = marj
            status["menzil_ref"] = tutucu.ref
            stop_event.wait(periyot)

    threading.Thread(target=_dongu, daemon=True).start()
    print(f"[TUTUCU] Marj geri beslemeli mesafe tutucu başladı "
          f"(bant {MARJ_REF:.2f}-{MARJ_UST:.2f}, ref₀ {tutucu.ref:.1f} m, "
          f"clamp {MENZIL_REF_ALT:.0f}-{MENZIL_REF_UST:.0f} m, {hz:.1f} Hz)")
