"""
vision/pose_detector.py — YOLO-pose tabanlı Talon keypoint tespiti.

detect_pose(frame_bgr) → {cx, cy, w, h, conf, bbox, kpts} veya None.
  kpts: geometry.KEYPOINT_NAMES sırasında 6 eleman, her biri (u, v, conf).
Detection modelinin (detector.py) YANINDA çalışır, yerine geçmez.
Model: vision/models/avci_pose.pt (train_yolo_pose.py çıktısı). GPU otomatik.
"""

import os

from vision.geometry import KEYPOINT_NAMES

_MODEL_PATH = os.environ.get(
    "AVCI_POSE_MODEL",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "vision", "models", "avci_pose.pt"),
)
_CONF_MIN = float(os.environ.get("AVCI_POSE_CONF", "0.35"))
_KPT_CONF_MIN = float(os.environ.get("AVCI_POSE_KPT_CONF", "0.5"))

_model = None

# Overlay: keypoint renkleri (BGR, KEYPOINT_NAMES sırası) + iskelet
_KPT_COLORS = [(0, 0, 255), (255, 0, 0), (0, 255, 0),
               (0, 255, 255), (255, 0, 255), (255, 255, 0)]
_SKELETON = [(0, 1), (2, 3), (1, 4), (1, 5)]   # gövde, kanat, V-tail'ler


def load(model_path=None):
    """Modeli yükle (bir kez). İlk çağrıda ~1-2 sn (ağırlık + CUDA warmup)."""
    global _model
    if _model is None:
        from ultralytics import YOLO
        path = model_path or _MODEL_PATH
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"Pose modeli yok: {path}\n"
                f"Önce eğit: python3 -m vision.train_yolo_pose")
        _model = YOLO(path)
    return _model


def detect_pose(frame_bgr, conf=None, imgsz=None):
    """En yüksek güvenli Talon pozunu döndürür (dict) ya da None."""
    model = load()
    kw = {"imgsz": imgsz} if imgsz else {}
    res = model.predict(frame_bgr, conf=(conf if conf is not None else _CONF_MIN),
                        verbose=False, **kw)[0]
    boxes = res.boxes
    if boxes is None or len(boxes) == 0 or res.keypoints is None:
        return None
    i = int(boxes.conf.argmax())                 # en güvenli tespit
    x1, y1, x2, y2 = (int(v) for v in boxes.xyxy[i].tolist())
    kxy = res.keypoints.xy[i].tolist()
    kcf = (res.keypoints.conf[i].tolist()
           if res.keypoints.conf is not None else [1.0] * len(kxy))
    kpts = [(float(u), float(v), float(c)) for (u, v), c in zip(kxy, kcf)]
    return {
        "cx": (x1 + x2) // 2,
        "cy": (y1 + y2) // 2,
        "w": x2 - x1,
        "h": y2 - y1,
        "conf": float(boxes.conf[i]),
        "bbox": (x1, y1, x2, y2),
        "kpts": kpts,
    }


# Krop penceresi: bbox'ın bu katı kadar, [min, max] piksel aralığına sıkıştırılır
_CROP_FACTOR = 4.0
_CROP_MIN = 128
_CROP_MAX = 448


def detect_pose_in_bbox(frame_bgr, bbox, conf=None, imgsz="native"):
    """Pose'u yalnız detection bbox'ının çevresindeki krop pencerede çalıştırır.
    bbox: detection modelinin (x1, y1, x2, y2) kutusu. Dönen koordinatlar TAM
    KARE pikselindedir. Kutu çevresinde _CROP_FACTOR katı kare pencere alınır —
    hem hesap yükü düşer hem model sahnenin kalanına yanlış nokta atamaz.
    imgsz='native': krop kendi boyutunda işlenir (büyütmek eğitim dağılımının
    dışına çıkarıyor — val ölçümünde pose oranını %100→%74'e düşürdü)."""
    import numpy as np
    H, W = frame_bgr.shape[:2]
    x1, y1, x2, y2 = bbox
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    size = max(x2 - x1, y2 - y1) * _CROP_FACTOR
    size = int(min(max(size, _CROP_MIN), _CROP_MAX, W, H))
    x0 = int(min(max(cx - size / 2, 0), W - size))
    y0 = int(min(max(cy - size / 2, 0), H - size))
    crop = np.ascontiguousarray(frame_bgr[y0:y0 + size, x0:x0 + size])
    if imgsz == "native":                 # kropu büyütmeden, kendi boyutunda işle
        imgsz = ((size + 31) // 32) * 32
    pose = detect_pose(crop, conf=conf, imgsz=imgsz)
    if pose is None:
        return None
    bx1, by1, bx2, by2 = pose["bbox"]
    pose["bbox"] = (bx1 + x0, by1 + y0, bx2 + x0, by2 + y0)
    pose["cx"] += x0
    pose["cy"] += y0
    pose["kpts"] = [(u + x0, v + y0, c) for (u, v, c) in pose["kpts"]]
    return pose


def draw_overlay(frame_bgr, pose):
    """Keypoint + iskeleti çizer (pose None ise kareyi değiştirmeden döner).
    Bbox'ı çizmez — o detection overlay'inin işi; ikisi üst üste çalışır."""
    import cv2
    if pose is None:
        return frame_bgr
    f = frame_bgr.copy()
    kpts = pose["kpts"]
    for a, b in _SKELETON:
        if kpts[a][2] >= _KPT_CONF_MIN and kpts[b][2] >= _KPT_CONF_MIN:
            cv2.line(f, (int(kpts[a][0]), int(kpts[a][1])),
                     (int(kpts[b][0]), int(kpts[b][1])), (220, 220, 220), 1)
    for i, (u, v, c) in enumerate(kpts):
        if c >= _KPT_CONF_MIN:
            cv2.circle(f, (int(u), int(v)), 3, _KPT_COLORS[i], -1)
    return f
