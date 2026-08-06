#!/usr/bin/env python3
"""
tools/pose_kanat_olc.py — Bir pose modelinin keypoint kalitesini ölçer.

Asıl ölçüt KANAT AÇIKLIĞI ORANI: model iki kanadı ne kadar ayırabiliyor?
  oran = ortanca(model açıklığı) / ortanca(gerçek açıklık)
  1.0'a yakın = doğru      |      0'a yakın = ikisini üst üste koyuyor

Ölçülmüş referans (eski, "sol/sağ" anlamlı etiketler):
  avci_pose_krop.pt     0.05      avci_pose_krop_v2.pt  0.05  (fliplr=0)
Kanat noktası hatası her ikisinde ~40 px = açıklığın yarısı, yani model
belirsizlik altında iki ihtimalin ortasını tahmin ediyordu.

Kullanım: python3 tools/pose_kanat_olc.py <model.pt> [dataset_dizini] [N]
"""

import glob
import os
import sys

import cv2
import numpy as np
from ultralytics import YOLO

ADLAR = ["burun", "kuyruk", "kanat_A", "kanat_B", "vtail_A", "vtail_B"]


def olc(model_yolu, veri_dizini, n_kare=500):
    m = YOLO(model_yolu)
    m.to("cuda")
    imgs = sorted(glob.glob(os.path.join(veri_dizini, "images/val/*")))[:n_kare]

    hata = [[] for _ in range(6)]
    g_kanat, t_kanat, g_vt, t_vt = [], [], [], []
    bulundu = 0

    for p in imgs:
        lp = p.replace("/images/", "/labels/").rsplit(".", 1)[0] + ".txt"
        if not os.path.exists(lp):
            continue
        s = open(lp).read().split()
        if len(s) < 23:
            continue
        kp = np.array(s[5:23], dtype=float).reshape(6, 3)
        im = cv2.imread(p)
        if im is None:
            continue
        H, W = im.shape[:2]
        r = m.predict(im, verbose=False, conf=0.10)[0]
        if r.keypoints is None or len(r.keypoints.data) == 0:
            continue
        pk = r.keypoints.data[0].cpu().numpy()
        bulundu += 1
        gt = np.stack([kp[:, 0] * W, kp[:, 1] * H], 1)
        for i in range(6):
            if kp[i, 2] > 0:
                hata[i].append(float(np.hypot(*(pk[i, :2] - gt[i]))))
        if kp[2, 2] > 0 and kp[3, 2] > 0:
            g_kanat.append(float(np.hypot(*(gt[2] - gt[3]))))
            t_kanat.append(float(np.hypot(*(pk[2, :2] - pk[3, :2]))))
        if kp[4, 2] > 0 and kp[5, 2] > 0:
            g_vt.append(float(np.hypot(*(gt[4] - gt[5]))))
            t_vt.append(float(np.hypot(*(pk[4, :2] - pk[5, :2]))))

    print(f"\n  ══ {os.path.basename(model_yolu)} — {bulundu}/{len(imgs)} karede tespit ══")
    print(f"    {'nokta':10s} {'ortanca':>9s} {'%90':>9s}")
    for i, a in enumerate(ADLAR):
        if hata[i]:
            h = np.array(hata[i])
            print(f"    {a:10s} {np.median(h):8.1f}px {np.percentile(h,90):8.1f}px")

    oran = None
    if g_kanat:
        g, t = np.median(g_kanat), np.median(t_kanat)
        oran = t / g
        print(f"\n    KANAT açıklığı  gerçek {g:6.1f}px → model {t:6.1f}px"
              f"   ORAN {oran:.2f}")
    if g_vt:
        g, t = np.median(g_vt), np.median(t_vt)
        print(f"    VTAIL açıklığı  gerçek {g:6.1f}px → model {t:6.1f}px"
              f"   ORAN {t/g:.2f}")
    return oran


if __name__ == "__main__":
    model = sys.argv[1]
    veri = sys.argv[2] if len(sys.argv) > 2 else \
        "/home/zeylo/projects/avci_sim/vision/datasets/talon_pose_sirali"
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 500
    o = olc(model, veri, n)
    if o is not None:
        print(f"\n    referans (eski modeller): 0.05")
        print(f"    → {'BAŞARILI, tam eğitime değer' if o > 0.5 else ('kısmi iyileşme' if o > 0.2 else 'DEĞİŞMEDİ, fikir yanlış')}")
