"""
vision/capture_pose_dataset.py — Gazebo statik world'de OTOMATİK ETİKETLİ YOLO-POSE verisi.

Detection pipeline'ıyla (capture_dataset.py) aynı mekanik: kamera + hedef her karede
rastgele konumlanır; pozlar bilindiğinden bbox VE 6 keypoint (burun, kuyruk, sol/sağ
kanat ucu, sol/sağ V-tail ucu) projeksiyonla otomatik etiketlenir. Manuel etiket yok.

Etiket formatı (YOLO-pose): "0 cx cy w h  x1 y1 v1 ... x6 y6 v6" (normalize).
v=2 görünür, v=0 kadraj dışı/arkada (x=y=0).

Kullanım:
  # 1) Ayrı terminalde statik world (detection'la AYNI world):
  export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models
  gz sim -r sim/gazebo_harmonic/worlds/dataset_capture.sdf
  # 2) Veri topla:
  python3 -m vision.capture_pose_dataset --count 5000 --debug-overlay
"""

import argparse
import os
import random
import time

import cv2

from gz.transport13 import Node

from vision import geometry as geo
from vision.capture_dataset import (
    CAMERA, CAM_TOPIC, TARGET,
    FrameGrabber, _set_pose, random_camera_pose, sample_target_pose,
)

# Keypoint pikselden okunabilsin diye bbox alt sınırı detection'dan büyük
MIN_POSE_BOX_PX = 16.0

# Debug overlay: keypoint renkleri (BGR) + iskelet çizgileri
_KPT_COLORS = [(0, 0, 255), (255, 0, 0), (0, 255, 0),
               (0, 255, 255), (255, 0, 255), (255, 255, 0)]
_SKELETON = [(0, 1), (2, 3), (1, 4), (1, 5)]   # gövde, kanat, V-tail'ler


def write_dataset_yaml(out_dir):
    abs_out = os.path.abspath(out_dir)
    yaml_path = os.path.join(out_dir, "dataset.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"path: {abs_out}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write(f"kpt_shape: [{len(geo.KEYPOINT_NAMES)}, 3]\n")
        f.write(f"flip_idx: {geo.KEYPOINT_FLIP_IDX}\n")
        f.write("nc: 1\n")
        f.write("names:\n  0: talon\n")
    return yaml_path


def kpts_to_yolo(kpts):
    """(6,3) piksel [u,v,vis] → normalize YOLO-pose string parçası."""
    parts = []
    for u, v, vis in kpts:
        if vis < 1:
            parts.append("0 0 0")
        else:
            parts.append(f"{u / geo.IMG_W:.6f} {v / geo.IMG_H:.6f} {int(vis)}")
    return " ".join(parts)


def draw_debug(frame, bb, kpts):
    dbg = frame.copy()
    cv2.rectangle(dbg, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), (0, 255, 0), 1)
    for a, b in _SKELETON:
        if kpts[a][2] >= 1 and kpts[b][2] >= 1:
            cv2.line(dbg, (int(kpts[a][0]), int(kpts[a][1])),
                     (int(kpts[b][0]), int(kpts[b][1])), (200, 200, 200), 1)
    for i, (u, v, vis) in enumerate(kpts):
        if vis >= 1:
            cv2.circle(dbg, (int(u), int(v)), 3, _KPT_COLORS[i], -1)
            cv2.putText(dbg, geo.KEYPOINT_NAMES[i], (int(u) + 4, int(v) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, _KPT_COLORS[i], 1)
    return dbg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=5000, help="hedef örnek sayısı")
    ap.add_argument("--out", default="vision/datasets/talon_pose")
    ap.add_argument("--val-split", type=float, default=0.15)
    ap.add_argument("--settle", type=float, default=0.20,
                    help="set_pose sonrası render bekleme (s)")
    ap.add_argument("--min-kpts", type=int, default=3,
                    help="karede en az bu kadar keypoint görünür olmalı")
    ap.add_argument("--dist-min", type=float, default=None,
                    help="hedef mesafe alt sınırı m (varsayılan capture_dataset: 3)")
    ap.add_argument("--dist-max", type=float, default=None,
                    help="hedef mesafe üst sınırı m (varsayılan 15)")
    ap.add_argument("--dist-exp", type=float, default=None,
                    help="mesafe dağılım üssü: 2=yakın ağırlıklı (vars.), 1=düzgün")
    ap.add_argument("--min-box", type=float, default=MIN_POSE_BOX_PX,
                    help=f"bbox alt sınırı px (varsayılan {MIN_POSE_BOX_PX}; "
                         f"uzak mesafe partisinde düşür)")
    ap.add_argument("--debug-overlay", action="store_true",
                    help="ilk 20 karede bbox+keypoint'leri çizip ayrı kaydet")
    args = ap.parse_args()

    # Mesafe ayarlarını örnekleyicinin (capture_dataset) modül sabitlerine uygula
    import vision.capture_dataset as _cd
    if args.dist_min is not None:
        _cd.DIST_MIN = args.dist_min
    if args.dist_max is not None:
        _cd.DIST_MAX = args.dist_max
    if args.dist_exp is not None:
        _cd.DIST_EXP = args.dist_exp
    print(f"[POSE] Mesafe: {_cd.DIST_MIN}-{_cd.DIST_MAX} m (üs {_cd.DIST_EXP}), "
          f"min kutu {args.min_box}px")

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)
    if args.debug_overlay:
        os.makedirs(os.path.join(args.out, "debug"), exist_ok=True)

    node = Node()
    grabber = FrameGrabber(node)

    print(f"[POSE] Kamera bekleniyor ({CAM_TOPIC})...")
    t0 = time.time()
    while grabber.snapshot()[0] is None and time.time() - t0 < 15:
        time.sleep(0.3)
    if grabber.snapshot()[0] is None:
        print("[POSE] HATA: kameradan görüntü gelmedi. Gazebo (dataset_capture.sdf) çalışıyor mu?")
        return
    print(f"[POSE] Kamera hazır. {len(geo.KEYPOINT_NAMES)} keypoint: "
          f"{', '.join(geo.KEYPOINT_NAMES)}")

    # Yarıda kesilen toplamaya kaldığı yerden devam et (isim çakışması/çift
    # etiket olmasın): mevcut en büyük indeksin bir sonrasından numaralandır.
    existing = [f for sp in ("train", "val")
                for f in os.listdir(os.path.join(args.out, "images", sp))]
    saved = 1 + max((int(f.split("_")[-1].split(".")[0]) for f in existing),
                    default=-1)
    if saved:
        print(f"[POSE] Mevcut {saved} örnek bulundu — {args.count} hedefine devam ediliyor.")

    attempts = 0
    max_attempts = args.count * 4
    while saved < args.count and attempts < max_attempts:
        attempts += 1
        iris_pos, iris_rpy = random_camera_pose()
        tpos, trpy = sample_target_pose(iris_pos, iris_rpy)
        if tpos is None:
            continue

        if not _set_pose(node, CAMERA, iris_pos, iris_rpy):
            time.sleep(0.02); continue
        if not _set_pose(node, TARGET, tpos, trpy):
            time.sleep(0.02); continue
        time.sleep(args.settle)

        bb = geo.target_bbox(tpos, trpy, iris_pos, iris_rpy)
        if bb is None:
            continue
        if (bb[2] - bb[0]) < args.min_box or (bb[3] - bb[1]) < args.min_box:
            continue
        kpts = geo.target_keypoints(tpos, trpy, iris_pos, iris_rpy)
        if (kpts[:, 2] >= 1).sum() < args.min_kpts:
            continue
        frame, _ = grabber.snapshot()
        if frame is None:
            continue

        split = "val" if random.random() < args.val_split else "train"
        name = f"talon_pose_{saved:05d}"
        cv2.imwrite(os.path.join(args.out, "images", split, name + ".jpg"), frame)
        cx, cy, w, h = geo.bbox_to_yolo(bb)
        with open(os.path.join(args.out, "labels", split, name + ".txt"), "w") as f:
            f.write(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {kpts_to_yolo(kpts)}\n")

        if args.debug_overlay and saved < 20:
            cv2.imwrite(os.path.join(args.out, "debug", name + "_pose.jpg"),
                        draw_debug(frame, bb, kpts))

        saved += 1
        if saved % 100 == 0:
            print(f"[POSE]   {saved}/{args.count}  (deneme {attempts})")

    yaml_path = write_dataset_yaml(args.out)
    print(f"[POSE] Bitti: {saved} örnek kaydedildi ({attempts} deneme).")
    print(f"[POSE] dataset.yaml: {yaml_path}")
    if args.debug_overlay:
        print(f"[POSE] Etiket doğrulama görselleri: {args.out}/debug/")


if __name__ == "__main__":
    main()
