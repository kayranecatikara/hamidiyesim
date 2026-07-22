"""
vision/train_yolo.py — Otomatik etiketli veriyle YOLO nano modeli eğitir.

Kullanım:
  python3 -m vision.train_yolo --epochs 100
  # farklı taban model:
  python3 -m vision.train_yolo --model yolo11n.pt --epochs 150

Çıktı: en iyi ağırlık → vision/models/avci_yolo.pt (gcs entegrasyonu bunu bekleyecek).
GPU (torch+CUDA) otomatik kullanılır.
"""

import argparse
import os
import shutil

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="vision/datasets/talon/dataset.yaml")
    ap.add_argument("--model", default="yolo11n.pt",
                    help="taban ağırlık (nano). Örn: yolo11n.pt (varsayılan). "
                         "Ultralytics ismi otomatik indirir.")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--name", default="talon", help="eğitim run adı")
    ap.add_argument("--out", default="vision/models/avci_yolo.pt")
    args = ap.parse_args()

    if not os.path.exists(args.data):
        raise SystemExit(f"dataset.yaml yok: {args.data}\n"
                         f"Önce veri topla: python3 -m vision.capture_dataset")

    print(f"[TRAIN] Taban model: {args.model}  data: {args.data}  epochs: {args.epochs}")
    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        name=args.name,
    )

    best = os.path.join(results.save_dir, "weights", "best.pt")
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    shutil.copy(best, args.out)
    print(f"[TRAIN] Eğitim bitti. En iyi ağırlık → {args.out}")
    print(f"[TRAIN] Metrikler/grafikler: {results.save_dir}")


if __name__ == "__main__":
    main()
