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
# ★ 0.45 → 0.25 (2026-07-30 UÇUŞ ölçümü). Eski değer 3-15 m verisiyle
# eğitilmiş modele göre seçilmişti ve uzakta hedefi ELİYORDU: 40 m'de
# sabit takipte aynı uçuş verisi eşiğe göre şöyle okunuyor —
#     conf≥0.45 → %6.7    conf≥0.35 → %63.6    conf≥0.25 → %92.9
# Kesinti süresi de buna bağlı: 0.35'te en uzun kesinti 1.13 s, 0.25'te
# 0.20 s. "25 m'de kesiyor" diye aylarca modelde aranan sorunun kaynağı
# modelin menzili değil, bu eşikti.
_CONF_MIN = float(os.environ.get("AVCI_YOLO_CONF", "0.25"))

# GPU KULLANIMI — ÖLÇÜM: predict() cihaz belirtilmediği için YOLO
# CPU'da çalışıyordu ve 8 çekirdeği doldurup (%793 CPU) Gazebo'yu
# aç bırakıyordu → simülasyon %61 hızda koşuyor, her şey gecikmeli
# görünüyordu. Kart (RTX 4060) boşta duruyordu. Artık GPU'da çalışır.
_ETIKET = "YOLO-TESPIT"

try:
    import torch
    if torch.cuda.is_available():
        _CIHAZ = 0
        _CIHAZ_AD = torch.cuda.get_device_name(0)
    else:
        _CIHAZ = 'cpu'
        _CIHAZ_AD = None
except Exception:
    _CIHAZ = 'cpu'
    _CIHAZ_AD = None

# Seçilen cihaz AÇIKÇA yazılır. Sebebi yaşanmış bir arıza: NVIDIA sürücüsü
# kernel modülüyle uyumsuz kaldığında torch sessizce CPU'ya düşüyordu; kimse
# fark etmediği için günlerce "model çok yavaş / arayüz geç" diye başka yerde
# hata arandı. Ölçüm: GPU 3.6 ms/kare (277 FPS), CPU 58.2 ms/kare (17 FPS) —
# 16 KAT fark. Bir daha sessizce düşmesin diye CPU hâli UYARI olarak basılır.
if _CIHAZ == 'cpu':
    print(f"[{_ETIKET}] !!! UYARI: GPU YOK, CPU'da calisiyor (~16x YAVAS) !!!")
    print(f"[{_ETIKET}] !!! kontrol: nvidia-smi  /  torch.cuda.is_available() !!!")
else:
    print(f"[{_ETIKET}] GPU aktif: {_CIHAZ_AD}")

_model = None

# ÇIKARIM ÇÖZÜNÜRLÜĞÜ — modelin EĞİTİLDİĞİ boyutla aynı olmak ZORUNDA.
# predict()'e imgsz verilmezse ultralytics 640 varsayar. Uzak/küçük hedef için
# 1280'de eğitilmiş bir model 640'ta çalıştırılırsa kazanılan çözünürlük geri
# atılır ve model uzaktan göremez — üstelik hata vermez, sessizce kötü çalışır.
# Bu yüzden boyut ağırlık dosyasından (train_args.imgsz) OKUNUR.
# AVCI_YOLO_IMGSZ ile elle geçersiz kılınabilir.
_IMGSZ_ZORLA = os.environ.get("AVCI_YOLO_IMGSZ")
_imgsz = None


def _model_imgsz(model):
    """Ağırlık dosyasına gömülü eğitim çözünürlüğü; okunamazsa 640."""
    if _IMGSZ_ZORLA:
        return int(_IMGSZ_ZORLA)
    for kaynak in (getattr(model, "overrides", None) or {},
                   (getattr(model, "ckpt", None) or {}).get("train_args") or {}):
        try:
            v = kaynak.get("imgsz")
        except AttributeError:
            continue
        if isinstance(v, (int, float)) and v:
            return int(v)
    return 640


def load(model_path=None):
    """Modeli yükle (bir kez). İlk çağrıda ~1-2 sn (ağırlık + CUDA warmup)."""
    global _model, _imgsz
    if _model is None:
        from ultralytics import YOLO
        path = model_path or _MODEL_PATH
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"YOLO modeli yok: {path}\n"
                f"Önce eğit: python3 -m vision.train_yolo")
        _model = YOLO(path)
        _imgsz = _model_imgsz(_model)
        print(f"[{_ETIKET}] model: {os.path.basename(path)}  "
              f"cikarim imgsz={_imgsz}  conf>={_CONF_MIN}")
    return _model


def detect_talon(frame_bgr, conf=None):
    """En yüksek güvenli Talon tespitini döndürür (dict) ya da None."""
    model = load()
    res = model.predict(frame_bgr, device=_CIHAZ, imgsz=_imgsz,
                        conf=(conf if conf is not None else _CONF_MIN),
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
