"""
=============================================================
  GÜDÜM SUPERVISOR — GPS ↔ Görsel geçiş yöneticisi
=============================================================
DURUM: İSKELET (Faz 0). Yapı ve geçiş politikası burada tanımlı; karar eşikleri
ve görsel hat Faz 3 (IBVS) + Faz 4'te devreye girecek. Şu an update() daima GPS
fazını döndürür — mevcut davranış korunur.

Onaylanan geçiş politikası:
  GPS → VISUAL : (a) GÖRSEL TEMAS — N ardışık kararlı tespit (conf ≥ eşik), VEYA
                 (b) GPS SAĞLIKSIZ — jamming seviyesi yüksek / telemetri donmuş
  VISUAL → GPS : GÖRSEL KAYIP — M ardışık karede tespit yok (fallback; drone kör
                 kalmasın)

Entegrasyon (Faz 4): gcs_server._chase_thread supervisor.update()'i her döngüde
çağırır; dönen faza göre get_plane() callback'i ya GPS telemetrisi ya da IBVS
görsel pozisyon tahmini üretir (bkz. docs/GUIDANCE_ROADMAP.md, "en temiz dikiş").
"""

from enum import Enum


class GuidancePhase(Enum):
    GPS = "gps"        # tam state (telemetri) ile güdüm — gps_chase / gps_strike
    VISUAL = "visual"  # kamera bbox (IBVS) ile güdüm — visual_guidance (Faz 3)


# Geçiş eşikleri (Faz 4'te ayarlanacak) — @20 Hz döngü varsayımı
DEFAULT_LOCK_FRAMES = 10    # ~0.5 s kararlı tespit → görsel temas onayı
DEFAULT_LOSS_FRAMES = 20    # ~1.0 s tespit yok → GPS'e fallback
DEFAULT_CONF_MIN    = 0.50  # tespit güven eşiği (YOLO conf)
DEFAULT_JAM_MAX     = 0.60  # üstünde GPS "sağlıksız" sayılır


class GuidanceSupervisor:
    """
    GPS ve görsel güdüm hatları arasında geçişi yöneten durum makinesi.

    İSKELET: sinyal okuma kancaları hazır; update() Faz 4'e kadar daima GPS döner.

    Args:
        get_detection : () -> dict|None   son YOLO/tespit sonucu (conf, bbox...)
        get_gps_health: () -> dict        {"jam": float 0-1, "frozen": bool}
    """

    def __init__(self, get_detection=None, get_gps_health=None,
                 lock_frames=DEFAULT_LOCK_FRAMES, loss_frames=DEFAULT_LOSS_FRAMES,
                 conf_min=DEFAULT_CONF_MIN, jam_max=DEFAULT_JAM_MAX):
        self._get_detection = get_detection
        self._get_gps_health = get_gps_health
        self.lock_frames = lock_frames
        self.loss_frames = loss_frames
        self.conf_min = conf_min
        self.jam_max = jam_max

        self.phase = GuidancePhase.GPS
        self._lock_streak = 0   # ardışık kararlı tespit sayacı
        self._loss_streak = 0   # ardışık tespitsiz kare sayacı

    # ── sinyal kancaları (Faz 4'te update() bunları kullanacak) ──

    def _visual_contact(self):
        """N ardışık karede conf ≥ eşik tespit var mı? (görsel temas)"""
        if self._get_detection is None:
            return False
        det = self._get_detection()
        if det is not None and det.get("conf", 0.0) >= self.conf_min:
            self._lock_streak += 1
            self._loss_streak = 0
        else:
            self._lock_streak = 0
            self._loss_streak += 1
        return self._lock_streak >= self.lock_frames

    def _gps_unhealthy(self):
        """GPS jamming/freeze nedeniyle güvenilmez mi?"""
        if self._get_gps_health is None:
            return False
        h = self._get_gps_health()
        return h.get("frozen", False) or h.get("jam", 0.0) >= self.jam_max

    def _visual_lost(self):
        """M ardışık karede tespit kayboldu mu? (fallback tetiği)"""
        return self._loss_streak >= self.loss_frames

    # ── ana karar ──

    def update(self):
        """
        Aktif güdüm fazını döndürür.

        Faz 0 İSKELET: daima GPS. Faz 4'te aşağıdaki state machine devreye girer:
            if phase == GPS and (_visual_contact() or _gps_unhealthy()):
                phase = VISUAL
            elif phase == VISUAL and _visual_lost():
                phase = GPS
        """
        return self.phase
