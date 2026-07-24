"""
vision/capture_dataset.py — Gazebo statik world'de OTOMATİK ETİKETLİ YOLO verisi toplar.

Her karede HEM drone/kamera HEM hedef rastgele konum+rotasyona konur. Pozlar
bilindiğinden hedefin 2D bbox'ı projeksiyonla (konum+rotasyon+FOV 125° → piksel)
otomatik hesaplanır (vision/geometry.py). Uçuş/güdüm gerekmez.

Kullanım:
  # 1) Ayrı terminalde statik world:
  export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models
  gz sim -r sim/gazebo_harmonic/worlds/dataset_capture.sdf
  # 2) Veri topla:
  python3 -m vision.capture_dataset --count 2000
"""

import argparse
import math
import os
import random
import threading
import time

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image as GzImage

from vision import geometry as geo

# ── Sabitler (dataset_capture.sdf ile eşleşmeli) ──
WORLD = "dataset"
TARGET = "mini_talon"          # hedef model adı
CAMERA = "camera_rig"          # drone/kamera model adı
CAM_TOPIC = "/iris_cam/image"
SET_POSE_SVC = f"/world/{WORLD}/set_pose"

# Drone (kamera) örnekleme — HAVADA bir hacimde, her yöne bakar
CAM_XY_RANGE = 30.0            # x,y ∈ [-30, 30] m
CAM_Z_RANGE = (15.0, 45.0)    # irtifa m
CAM_ROLL_MAX = math.radians(25)   # drone roll eğimi (yaw tam tur)
# Pitch: yukarı az (gökyüzü arka plan), AŞAĞI geniş (zemin arka plan — model yerdeki
# hedefi de öğrensin). +pitch = burun aşağı (zemin), -pitch = yukarı (gökyüzü).
CAM_PITCH_UP = math.radians(20)
CAM_PITCH_DOWN = math.radians(55)
CAM_PITCH_HORIZON = math.radians(20)   # bu pitch'in üstü ≈ gökyüzü, altı ≈ zemin
SKY_FRACTION = 0.70                     # karelerin ~%70'i gökyüzü arka plan olsun

# Hedef örnekleme (kameranın FOV'una göre)
DIST_MIN, DIST_MAX = 3.0, 15.0   # yakın ağırlıklı (**DIST_EXP). FOV 125° geniş olduğundan
                                 # hedef yakında bile orta boyut; bu yüzden agresif yakın.
DIST_EXP = 2.0                   # 2=yakın ağırlıklı, 1=düzgün dağılım
PIX_MARGIN = 0.12
MIN_BOX_PX = 10.0             # bundan küçük bbox'ları at
MIN_TARGET_ALT = 0.5         # hedef yere yakın da olabilir (zemin arka plan); yer altı at
ROLL_MAX = math.radians(65)  # hedef oryantasyon çeşitliliği
PITCH_MAX = math.radians(45)


def _rpy_to_quat(roll, pitch, yaw):
    """RPY → quaternion (x, y, z, w), Gazebo pose ile tutarlı (ZYX)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class FrameGrabber:
    """gz-transport kamera aboneliği; en son kareyi thread-safe verir."""
    def __init__(self, node):
        self._lock = threading.Lock()
        self._frame = None
        self._id = 0
        node.subscribe(GzImage, CAM_TOPIC, self._cb)

    def _cb(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            f = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            with self._lock:
                self._frame = f
                self._id += 1
        except Exception as e:
            print(f"[CAP] kare hatası: {e}")

    def snapshot(self):
        with self._lock:
            if self._frame is None:
                return None, 0
            return self._frame.copy(), self._id


def _set_pose(node, name, pos, rpy):
    """Bir modeli (name) verilen world konum+rpy'ye taşı (gz set_pose)."""
    qx, qy, qz, qw = _rpy_to_quat(*rpy)
    req = Pose()
    req.name = name
    req.position.x, req.position.y, req.position.z = float(pos[0]), float(pos[1]), float(pos[2])
    req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = qx, qy, qz, qw
    try:
        _, ok = node.request(SET_POSE_SVC, req, Pose, Boolean, 500)
    except Exception:
        ok = False
    return ok


def random_camera_pose():
    """Drone/kamera için rastgele world konum + rpy (havada, her yöne bakar)."""
    pos = np.array([
        random.uniform(-CAM_XY_RANGE, CAM_XY_RANGE),
        random.uniform(-CAM_XY_RANGE, CAM_XY_RANGE),
        random.uniform(*CAM_Z_RANGE),
    ])
    # Pitch: %70 gökyüzü (yukarı/ufuk), %30 zemin (belirgin aşağı bakış)
    if random.random() < SKY_FRACTION:
        pitch = random.uniform(-CAM_PITCH_UP, CAM_PITCH_HORIZON)
    else:
        pitch = random.uniform(CAM_PITCH_HORIZON, CAM_PITCH_DOWN)
    rpy = (
        random.uniform(-CAM_ROLL_MAX, CAM_ROLL_MAX),     # roll
        pitch,
        random.uniform(-math.pi, math.pi),               # yaw (tam tur)
    )
    return pos, rpy


def sample_target_pose(iris_pos, iris_rpy):
    """Verilen kamera pozu için, FOV içinde ve YER ÜSTÜNDE hedef (world_pos, rpy).
    Bulunamazsa (nadir) (None, None)."""
    cam_pos, R_cam = geo.camera_world_pose(iris_pos, iris_rpy)
    world = None
    for _ in range(20):
        d = DIST_MIN + (DIST_MAX - DIST_MIN) * random.random() ** DIST_EXP
        u = random.uniform(PIX_MARGIN, 1.0 - PIX_MARGIN) * geo.IMG_W
        v = random.uniform(PIX_MARGIN, 1.0 - PIX_MARGIN) * geo.IMG_H
        Xo = (u - geo.CX) / geo.FX * d
        Yo = (v - geo.CY) / geo.FY * d
        cand = cam_pos + R_cam @ np.array([d, -Xo, -Yo])   # X_link=d, Y_link=-Xo, Z_link=-Yo
        if cand[2] >= MIN_TARGET_ALT:
            world = cand
            break
    if world is None:
        return None, None
    trpy = (
        random.uniform(-ROLL_MAX, ROLL_MAX),
        random.uniform(-PITCH_MAX, PITCH_MAX),
        random.uniform(-math.pi, math.pi),
    )
    return world, trpy


def write_dataset_yaml(out_dir):
    abs_out = os.path.abspath(out_dir)
    yaml_path = os.path.join(out_dir, "dataset.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"path: {abs_out}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("nc: 1\n")
        f.write("names:\n  0: talon\n")
    return yaml_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=2000, help="hedef örnek sayısı")
    ap.add_argument("--out", default="vision/datasets/talon")
    ap.add_argument("--val-split", type=float, default=0.15)
    ap.add_argument("--settle", type=float, default=0.20,
                    help="set_pose sonrası render bekleme (s) — kamera+hedef taşınır")
    ap.add_argument("--debug-overlay", action="store_true",
                    help="ilk 20 karede bbox'ı çizip ayrı kaydet (etiket doğrulama)")
    args = ap.parse_args()

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)
    if args.debug_overlay:
        os.makedirs(os.path.join(args.out, "debug"), exist_ok=True)

    node = Node()
    grabber = FrameGrabber(node)

    print(f"[CAP] Kamera bekleniyor ({CAM_TOPIC})...")
    t0 = time.time()
    while grabber.snapshot()[0] is None and time.time() - t0 < 15:
        time.sleep(0.3)
    if grabber.snapshot()[0] is None:
        print("[CAP] HATA: kameradan görüntü gelmedi. Gazebo (dataset_capture.sdf) çalışıyor mu?")
        return
    print("[CAP] Kamera hazır. Drone + hedef rastgele konumlanarak veri toplanıyor...")

    saved = 0
    attempts = 0
    max_attempts = args.count * 4
    while saved < args.count and attempts < max_attempts:
        attempts += 1
        iris_pos, iris_rpy = random_camera_pose()
        tpos, trpy = sample_target_pose(iris_pos, iris_rpy)
        if tpos is None:
            continue

        # Her iki modeli de yeni pozlarına taşı
        if not _set_pose(node, CAMERA, iris_pos, iris_rpy):
            time.sleep(0.02); continue
        if not _set_pose(node, TARGET, tpos, trpy):
            time.sleep(0.02); continue
        time.sleep(args.settle)                       # kamera+hedef render bekle

        bb = geo.target_bbox(tpos, trpy, iris_pos, iris_rpy)
        if bb is None:
            continue
        if (bb[2] - bb[0]) < MIN_BOX_PX or (bb[3] - bb[1]) < MIN_BOX_PX:
            continue
        frame, _ = grabber.snapshot()
        if frame is None:
            continue

        split = "val" if random.random() < args.val_split else "train"
        name = f"talon_{saved:05d}"
        cv2.imwrite(os.path.join(args.out, "images", split, name + ".jpg"), frame)
        cx, cy, w, h = geo.bbox_to_yolo(bb)
        with open(os.path.join(args.out, "labels", split, name + ".txt"), "w") as f:
            f.write(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")

        if args.debug_overlay and saved < 20:
            dbg = frame.copy()
            cv2.rectangle(dbg, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), (0, 255, 0), 2)
            cv2.imwrite(os.path.join(args.out, "debug", name + "_bbox.jpg"), dbg)

        saved += 1
        if saved % 100 == 0:
            print(f"[CAP]   {saved}/{args.count}  (deneme {attempts})")

    yaml_path = write_dataset_yaml(args.out)
    print(f"[CAP] Bitti: {saved} örnek kaydedildi ({attempts} deneme).")
    print(f"[CAP] dataset.yaml: {yaml_path}")
    if args.debug_overlay:
        print(f"[CAP] Etiket doğrulama görselleri: {args.out}/debug/")


if __name__ == "__main__":
    main()
