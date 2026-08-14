# vision/detection_state.py
# Kamera thread'i ile güdüm döngüleri arasında tespit/pose sonuçlarını paylaşan
# thread-safe köprü (gcs_server yazar; control.guidance okur).
import threading
import time

_lock = threading.Lock()
_last_detection = None
# ── TESPİT ZAMAN DAMGASI (2026-08-14) ──────────────────────────────────
# NEDEN: arayüz "kutu ekranda duruyor" ile "tespit GÜNCEL" ayrımını
# yapamıyordu. Kutu videoya sunucuda gömülüyor; kare eskiyse operatör bunu
# göremezdi. Yaş ölçülebilir olmalı ki STALE/LOST dürüstçe gösterilebilsin.
#
# EKLEMELİ: mevcut set_detection/get_detection davranışı DEĞİŞMEDİ; yalnız
# yeni bir damga tutulur ve yeni bir okuyucu eklenir. Güdüm bu alanı
# kullanmaz, davranışı etkilenmez.
_last_detection_ts = None      # son BAŞARILI tespit anı (time.time)

def set_detection(det):
    """Store the latest detection result (dict or None)."""
    global _last_detection, _last_detection_ts
    with _lock:
        _last_detection = det
        if det is not None:
            _last_detection_ts = time.time()

def get_detection():
    """Retrieve the latest detection result (dict or None)."""
    with _lock:
        return _last_detection

def get_detection_ts():
    """Son BAŞARILI tespitin zaman damgası (yoksa None). Salt gözlem."""
    with _lock:
        return _last_detection_ts


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


# ── Pose durumu: olay güdümlü tüketiciler (visual_lead) için seq + Condition ──
_pose_cond = threading.Condition(_lock)
_last_pose = None
_pose_seq = 0
_pose_stamp = None      # kare header.stamp (s) — dt bundan hesaplanır
_pose_wall = None       # karenin geliş duvar anı (time.time) — gecikme ölçümü
_pose_lock = None       # aynı karenin kilit durumu (TargetLock.step çıktısı | None)


def set_pose_detection(pose, stamp=None, wall_recv=None, lock=None):
    """Store the latest pose result (dict with 'kpts', or None) + kare zamanları
    + o karenin kilit durumu (supervisor sayacı kimlik sürekliliğini görsün).
    Her KARE için çağrılır (pose None olsa bile) — bekleyenler uyandırılır."""
    global _last_pose, _pose_seq, _pose_stamp, _pose_wall, _pose_lock
    with _pose_cond:
        _last_pose = pose
        _pose_stamp = stamp
        _pose_wall = wall_recv
        _pose_lock = lock
        _pose_seq += 1
        _pose_cond.notify_all()


def get_pose_detection():
    """Retrieve the latest pose result (dict or None)."""
    with _pose_cond:
        return _last_pose


def wait_new_pose(son_seq, timeout=0.5):
    """son_seq'ten YENİ bir kare kaydı gelene dek bekler (kareye kilitli döngü
    için — sabit Hz'te dönmek kare tekrarı/bayat veri üretir).
    Dönüş: {'seq','pose','stamp','wall_recv','lock'} veya timeout'ta None."""
    with _pose_cond:
        if _pose_seq == son_seq:
            _pose_cond.wait(timeout)
        if _pose_seq == son_seq:
            return None
        return {"seq": _pose_seq, "pose": _last_pose,
                "stamp": _pose_stamp, "wall_recv": _pose_wall,
                "lock": _pose_lock}
