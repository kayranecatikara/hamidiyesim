#!/usr/bin/env python3
"""
tools/pose_sirali_egit.py — Piksel-sıralı etiketlerle pose modeli eğitir.

Kullanım:
  python3 tools/pose_sirali_egit.py on     # 20 epoch ön test (~15 dk)
  python3 tools/pose_sirali_egit.py tam    # 100 epoch tam eğitim

Ön test kanat açıklığı oranını ölçer: 0.05'te kalırsa fikir yanlıştır,
belirgin yükselirse tam eğitime geçilir. Amaç boşuna 40 dk harcamamak.

ultralytics göreli project yolunu runs/pose/ altına gömüyor — bu yüzden
mutlak yol verilir (2026-07-30'da bu yüzden çıktı kaybolmuştu).
"""

import os
import sys

from ultralytics import YOLO

KOK = "/home/zeylo/projects/avci_sim"
VERI = f"{KOK}/vision/datasets/talon_pose_sirali/dataset.yaml"
CIKTI = f"{KOK}/vision/runs_pose"

mod = sys.argv[1] if len(sys.argv) > 1 else "on"
if mod == "on":
    ad, epoch = "pose_sirali_on", 20
else:
    ad, epoch = "pose_sirali", 100

m = YOLO("yolo11n-pose.pt")
m.train(
    data=VERI,
    epochs=epoch,
    imgsz=192,
    batch=32,
    device=0,
    project=CIKTI,
    name=ad,
    exist_ok=True,
    cache="ram",          # dataloader kilitlenmesine karşı (2026-07-30)
    workers=4,
    patience=20,
    pose=12.0,
    kobj=1.0,
    degrees=0.0,
    scale=0.5,
    flipud=0.0,
    # flip_idx [0,1,3,2,5,4] piksel sırasıyla uyumlu → aynalama güvenli
    fliplr=0.5,
    mosaic=1.0,
    plots=True,
    verbose=True,
)
print(f"\nBİTTİ → {CIKTI}/{ad}/weights/best.pt")
