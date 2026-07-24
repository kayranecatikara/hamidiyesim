# vision/detection_state.py
# Thread-safe wrapper for sharing detection results with chase_algorithm
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


# ── Pose durumu: olay güdümlü tüketiciler (visual_lead) için seq + Condition ──
_pose_cond = threading.Condition(_lock)
_last_pose = None
_pose_seq = 0
_pose_stamp = None      # kare header.stamp (s) — dt bundan hesaplanır
_pose_wall = None       # karenin geliş duvar anı (time.time) — gecikme ölçümü


def set_pose_detection(pose, stamp=None, wall_recv=None):
    """Store the latest pose result (dict with 'kpts', or None) + kare zamanları.
    Her KARE için çağrılır (pose None olsa bile) — bekleyenler uyandırılır."""
    global _last_pose, _pose_seq, _pose_stamp, _pose_wall
    with _pose_cond:
        _last_pose = pose
        _pose_stamp = stamp
        _pose_wall = wall_recv
        _pose_seq += 1
        _pose_cond.notify_all()


def get_pose_detection():
    """Retrieve the latest pose result (dict or None)."""
    with _pose_cond:
        return _last_pose


def wait_new_pose(son_seq, timeout=0.5):
    """son_seq'ten YENİ bir kare kaydı gelene dek bekler (kareye kilitli döngü
    için — sabit Hz'te dönmek kare tekrarı/bayat veri üretir).
    Dönüş: {'seq','pose','stamp','wall_recv'} veya timeout'ta None."""
    with _pose_cond:
        if _pose_seq == son_seq:
            _pose_cond.wait(timeout)
        if _pose_seq == son_seq:
            return None
        return {"seq": _pose_seq, "pose": _last_pose,
                "stamp": _pose_stamp, "wall_recv": _pose_wall}
