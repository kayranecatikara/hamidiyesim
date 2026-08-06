# vision/detection_state.py
# Kamera thread'i ile güdüm döngüleri arasında tespit sonuçlarını paylaşan
# thread-safe köprü (gcs_server yazar; control.guidance okur).
#
# ── 2026-08-06: POSE KÖPRÜSÜ → KARE KÖPRÜSÜ ──
# Eskiden olay güdümlü köprü YOLO-pose çıktısını taşıyordu (set_pose_detection /
# wait_new_pose) ve güdüm keypoint'lerden lead açısı türetiyordu. Pose modeli
# kullanımdan kaldırıldı (bkz. POSEA_GERI_DONMEK_ISTERSENIZ/README.md); görsel
# güdüm artık YALNIZ bbox ile çalışıyor. Köprü de aynı kareyi taşımaya devam
# ediyor, yalnız yükü pose değil DETECTION sözlüğü:
#     {cx, cy, w, h, conf, bbox, track_id?}
# Fonksiyon adları bilerek değişti (set_frame_detection / wait_new_frame): eski
# adlar kalsaydı pose taşıdıkları sanılırdı.
import threading

_lock = threading.Lock()
_last_detection = None

def set_detection(det):
    """Store the latest detection result (dict or None)."""
    global _last_detection
    with _lock:
        _last_detection = det

def get_detection():
    """Retrieve the latest detection result (dict or None)."""
    with _lock:
        return _last_detection


# ── HybridSORT takip durumu (gcs yazar; şimdilik overlay/inceleme için) ──
_last_tracks = None


def set_tracks(tracks):
    """Son takip sonucu: M×8 ndarray [x1,y1,x2,y2,id,conf,cls,det_ind] veya None."""
    global _last_tracks
    with _lock:
        _last_tracks = tracks


def get_tracks():
    """Son takip sonucunu döndürür (M×8 ndarray veya None)."""
    with _lock:
        return _last_tracks


# ── KARE köprüsü: olay güdümlü tüketiciler (visual_lead) için seq + Condition ──
_kare_cond = threading.Condition(_lock)
_last_kare_det = None
_kare_seq = 0
_kare_stamp = None      # kare header.stamp (s) — dt bundan hesaplanır
_kare_wall = None       # karenin geliş duvar anı (time.time) — gecikme ölçümü
_kare_lock = None       # aynı karenin kilit durumu (TargetLock.step çıktısı | None)


def set_frame_detection(det, stamp=None, wall_recv=None, lock=None):
    """O KAREYE ait detection sonucunu (dict veya None) + kare zamanlarını +
    kilit durumunu yayınla. Her KARE için çağrılır (det None olsa bile) —
    bekleyenler uyandırılır, böylece güdüm döngüsü kareye kilitli kalır."""
    global _last_kare_det, _kare_seq, _kare_stamp, _kare_wall, _kare_lock
    with _kare_cond:
        _last_kare_det = det
        _kare_stamp = stamp
        _kare_wall = wall_recv
        _kare_lock = lock
        _kare_seq += 1
        _kare_cond.notify_all()


def get_frame_detection():
    """Son karenin detection sonucunu döndürür (dict veya None)."""
    with _kare_cond:
        return _last_kare_det


def wait_new_frame(son_seq, timeout=0.5):
    """son_seq'ten YENİ bir kare kaydı gelene dek bekler (kareye kilitli döngü
    için — sabit Hz'te dönmek kare tekrarı/bayat veri üretir).
    Dönüş: {'seq','det','stamp','wall_recv','lock'} veya timeout'ta None."""
    with _kare_cond:
        if _kare_seq == son_seq:
            _kare_cond.wait(timeout)
        if _kare_seq == son_seq:
            return None
        return {"seq": _kare_seq, "det": _last_kare_det,
                "stamp": _kare_stamp, "wall_recv": _kare_wall,
                "lock": _kare_lock}
