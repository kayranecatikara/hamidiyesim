"""control/yaklasma_kontrol.py — Oran regülasyonlu APPROACH kontrolü (SAF, durumlu).

Mesafeyi SABİT tutmak yerine, hedefin karedeki ORANINI ORAN_SETPOINT'e regüle
eder: oran düşükse yaklaş (gps_guidance.Cfg.RANGE_SET'i düşür → gps kapanır),
oran yüksekse geri çekil (sarmalayıcı send_velocity — gps_guidance'a dokunmadan;
r_eff=min(menzil,RANGE_SET) yüzünden RANGE_SET'i büyütmek uzaklaştıramaz).

Ölçüm: HAM oranın EMA'sı (ORAN_EMA_TAU). Nişan/coast kutusu DEĞİL — coast'ta
tahmin üretir, mesafe kontrolü tahmine bağlanmaz. bbox yoksa EMA güncellenmez.

Bant latch (sınır titreşimini önler): oran banttan çıkınca düzeltme başlar ve
sınıra değince değil ORAN_SETPOINT'e varınca biter. Bant içindeyken komut yok.

Emniyet: EMA oran >= AH_ORAN_TAVAN → geri çekil (birincil). R_MIN_GUVENLI yalnız
bbox kaybında (menzil güvenilmez) yaklaşmayı durduran yedek taban.
Yalnız APPROACH/TRACK_LOCK'ta etkin (ENGAGE/STRIKE'ta pasif).
"""

from dataclasses import dataclass
from typing import Optional

from config.kilit_sabitler import AYAR, MENZIL_REF_UST

_SETPOINT_EPS = 0.003          # |oran-setpoint| bunun altındaysa "vardı" (latch bırakır)


def _clamp(v, alt, ust):
    return alt if v < alt else (ust if v > ust else v)


@dataclass
class YaklasmaKarar:
    range_set: float          # yeni RANGE_SET setpoint (m) — gps kapanma knob'u
    komut: str                # "yaklas" | "geri" | "yok"
    geri_hiz: float           # geri-çekilme hız büyüklüğü (m/s); "geri" değilse 0
    sebep: str                # durma/komut sebebi (log)
    oran_ema: Optional[float] # filtreli oran (log)
    oran_hatasi: Optional[float]  # setpoint - ema (log)


# Sebep sabitleri (log/karşılaştırma tek kaynak)
SEBEP_BANT = "bant ici"
SEBEP_YAKLAS = "yaklasiyor"
SEBEP_GERI = "uzaklasiyor"
SEBEP_TAVAN = "oran tavani (emniyet)"
SEBEP_RMIN = "R_MIN (bbox kaybi yedegi)"
SEBEP_PASIF = "pasif"
SEBEP_ORANYOK = "oran yok"


class YaklasmaKontrol:
    """Durumlu regülatör: EMA + bant latch. Tek tüketiciden çağrılır."""

    def __init__(self, cfg=AYAR):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self._ema = None
        self._duzeltiyor = False     # bant dışına çıkınca True; setpoint'e varınca False

    def _ema_guncelle(self, oran_ham, bbox_var, dt):
        if bbox_var and oran_ham is not None:
            if self._ema is None:
                self._ema = float(oran_ham)
            elif dt > 0.0:
                a = dt / (self.cfg.ORAN_EMA_TAU + dt)
                self._ema += a * (float(oran_ham) - self._ema)
        return self._ema

    def adim(self, oran_ham, bbox_var, menzil, range_set, dt, aktif=True):
        """oran_ham: max(kaplama_x,kaplama_y) ham (bbox varsa). bbox_var: bbox
        bu karede var mı (EMA güncelleme kapısı). menzil: ölçülen menzil (m|None).
        range_set: mevcut RANGE_SET. dt: tik (s). aktif: APPROACH/TRACK_LOCK mı."""
        cfg = self.cfg
        ema = self._ema_guncelle(oran_ham, bbox_var, dt)

        if not aktif:
            self._duzeltiyor = False
            return YaklasmaKarar(range_set, "yok", 0.0, SEBEP_PASIF, ema, None)

        # ── BBOX YOK (bu kare) → körlemesine yaklaşma YOK; R_MIN yedeği devrede ──
        # Oran güvenilmez; RANGE_SET'i R_MIN altına indirme, mevcutta tut. Menzil
        # (GNSS) de güvenilmez ama tek elimizdeki taban odur.
        if not bbox_var or ema is None:
            self._duzeltiyor = False
            yeni = _clamp(range_set, cfg.R_MIN_GUVENLI, MENZIL_REF_UST)
            sebep = SEBEP_RMIN if ema is not None else SEBEP_ORANYOK
            return YaklasmaKarar(yeni, "yok", 0.0, sebep, ema, None)

        # ── BBOX VAR → oran regülasyonu. Emniyet ORAN TAVANI'dır (menzil/R_MIN
        # DEĞİL — GNSS güvenilmez). RANGE_SET tabanı RANGE_SET_MIN. ──
        hata = cfg.ORAN_SETPOINT - ema        # + → uzak (yaklaş), - → yakın (geri)

        # BİRİNCİL EMNİYET: tavanı aşınca koşulsuz geri çekil
        if ema >= cfg.AH_ORAN_TAVAN:
            self._duzeltiyor = True
            return YaklasmaKarar(MENZIL_REF_UST, "geri", cfg.V_MAX_RETREAT,
                                 SEBEP_TAVAN, ema, hata)

        # BANT LATCH: dışına çıkınca başla, ORAN_SETPOINT'e varınca bırak
        if not self._duzeltiyor:
            if ema < cfg.ORAN_BANT_ALT or ema > cfg.ORAN_BANT_UST:
                self._duzeltiyor = True
        elif abs(hata) <= _SETPOINT_EPS:
            self._duzeltiyor = False

        if not self._duzeltiyor:              # bant içi / setpoint'te → tut
            hedef_rs = menzil if menzil is not None else range_set
            yeni = _clamp(hedef_rs, cfg.RANGE_SET_MIN, MENZIL_REF_UST)
            return YaklasmaKarar(yeni, "yok", 0.0, SEBEP_BANT, ema, hata)

        if hata > 0.0:                        # oran düşük → YAKLAŞ
            # RANGE_SET'i RANGE_SET_MIN'e doğru düşür → gps kapanır. R_MIN yok:
            # oran tavanı (0.10) daha yakına inmeyi zaten durdurur.
            yeni = _clamp(range_set - cfg.V_MAX_APPROACH * dt,
                          cfg.RANGE_SET_MIN, MENZIL_REF_UST)
            return YaklasmaKarar(yeni, "yaklas", 0.0, SEBEP_YAKLAS, ema, hata)
        else:                                 # oran yüksek → GERİ ÇEKİL
            # RANGE_SET'i büyüt ki gps kapanma komutu vermesin (min() → gps tutar),
            # geri hareketi sarmalayıcı send_velocity üretir.
            return YaklasmaKarar(MENZIL_REF_UST, "geri", cfg.V_MAX_RETREAT,
                                 SEBEP_GERI, ema, hata)
