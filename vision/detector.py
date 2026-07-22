"""
vision/detector.py — YOLO tabanlı Talon detector (color_detector.py'nin yerine).

detect_talon(frame_bgr) → {cx, cy, w, h, conf, bbox} veya None.
Çıktı sözleşmesi color_detector ile AYNI → set_detection / downstream değişmez.
Model: vision/models/avci_yolo.pt (train_yolo.py çıktısı). GPU varsa otomatik kullanır.
"""

import os

_MODEL_PATH = os.environ.get(
    "AVCI_YOLO_MODEL",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "vision", "models", "avci_yolo.pt"),
)
_CONF_MIN = float(os.environ.get("AVCI_YOLO_CONF", "0.35"))

_model = None


def load(model_path=None):
    """Modeli yükle (bir kez). İlk çağrıda ~1-2 sn (ağırlık + CUDA warmup)."""
    global _model
    if _model is None:
        from ultralytics import YOLO
        path = model_path or _MODEL_PATH
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"YOLO modeli yok: {path}\n"
                f"Önce eğit: python3 -m vision.train_yolo")
        _model = YOLO(path)
    return _model


def detect_talon(frame_bgr, conf=None):
    """En yüksek güvenli Talon tespitini döndürür (dict) ya da None."""
    model = load()
    res = model.predict(frame_bgr, conf=(conf if conf is not None else _CONF_MIN),
                        verbose=False)[0]
    boxes = res.boxes
    if boxes is None or len(boxes) == 0:
        return None
    i = int(boxes.conf.argmax())                 # en güvenli tespit
    x1, y1, x2, y2 = (int(v) for v in boxes.xyxy[i].tolist())
    return {
        "cx": (x1 + x2) // 2,
        "cy": (y1 + y2) // 2,
        "w": x2 - x1,
        "h": y2 - y1,
        "conf": float(boxes.conf[i]),
        "bbox": (x1, y1, x2, y2),
    }


def draw_overlay(frame_bgr, det):
    """Tespit kutusunu + güveni çizer (det None ise kareyi değiştirmeden döner)."""
    import cv2
    if det is None:
        return frame_bgr
    f = frame_bgr.copy()
    x1, y1, x2, y2 = det["bbox"]
    cv2.rectangle(f, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(f, f"HEDEF {det['conf']:.2f}", (x1, max(y1 - 6, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return f
