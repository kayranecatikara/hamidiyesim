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
