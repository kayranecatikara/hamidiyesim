"""
vision/capture_runway_negatives.py — PİST/ZEMİN hard-negative karesi toplar (uçuşsuz).

Amaç: modelin pist işaretlerini/zemin dokularını "HEDEF" sanmasını engellemek
(canlı simde pist başlangıcı conf 0.57 ile FP verdi). capture_dataset.py ile aynı
statik world (dataset_capture.sdf) kullanılır ama:
  - mini_talon 600m uzağa park edilir (hiçbir karede görünmez → etiketler BOŞ),
  - camera_rig pistin etrafında rastgele DÜŞÜK irtifa pozlarına konur ve pist
    üzerindeki rastgele bir noktaya BAKTIRILIR (FP'nin doğduğu bakış geometrisi),
  - karelerin bir kısmı genel zemin/ufuk görüntüsüdür (çim, ufuk çizgisi).

ÖNEMLİ: Canlı sim çalışırken topic çakışmasın diye AYRI GZ_PARTITION ile çalıştırın
(hem gz sim hem bu script aynı partition'ı görmeli):

  export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models
  DISPLAY=:1 GZ_PARTITION=negcap gz sim -s -r sim/gazebo_harmonic/worlds/dataset_capture.sdf &
  GZ_PARTITION=negcap python3 -m vision.capture_runway_negatives --count 650

Sonra yeniden eğit: python3 -m vision.train_yolo --epochs 100
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

WORLD = "dataset"
TARGET = "mini_talon"
CAMERA = "camera_rig"
CAM_TOPIC = "/iris_cam/image"
SET_POSE_SVC = f"/world/{WORLD}/set_pose"

# Kamera gövdesine montaj: pitch -25° (YUKARI). Işını δ kadar aşağı indirmek
# için gövde pitch'i δ+25° aşağı olmalı (vision/geometry.py ile tutarlı).
CAM_MOUNT_UP = math.radians(25)

# Pist: 1500x100m, model içi görsel 90° dönük → uzun eksen dünya Y'si.
RUNWAY_AIM_X = 22.0        # pist şeridi genişliği içinde hedef nokta |x|
RUNWAY_AIM_Y = 220.0       # pist boyunca hedef nokta |y|
CAM_XY_X = 70.0            # kamera konumu |x|
CAM_XY_Y = 260.0           # kamera konumu |y|
CAM_Z = (1.0, 30.0)        # DÜŞÜK irtifa ağırlıklı (FP yerde/alçakta doğdu)
GROUND_VIEW_FRACTION = 0.30   # karelerin ~%30'u pist yerine genel zemin/ufuk


def _rpy_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class FrameGrabber:
    def __init__(self, node):
        self._lock = threading.Lock()
        self._frame = None
        self._id = 0
        node.subscribe(GzImage, CAM_TOPIC, self._cb)

    def _cb(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
            f = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            with self._lock:
                self._frame = f
                self._id += 1
        except Exception as e:
            print(f"[NEG-RW] kare hatası: {e}")

    def snapshot(self):
        with self._lock:
            if self._frame is None:
                return None, 0
            return self._frame.copy(), self._id


def _set_pose(node, name, pos, rpy):
    qx, qy, qz, qw = _rpy_to_quat(*rpy)
    req = Pose()
    req.name = name
    req.position.x, req.position.y, req.position.z = map(float, pos)
    (req.orientation.x, req.orientation.y,
     req.orientation.z, req.orientation.w) = qx, qy, qz, qw
    try:
        _, ok = node.request(SET_POSE_SVC, req, Pose, Boolean, 500)
    except Exception:
        ok = False
    return ok


def random_negative_pose():
    """Kamera gövdesi için pist-bakışlı (veya genel zemin) rastgele poz üretir."""
    z = CAM_Z[0] + (CAM_Z[1] - CAM_Z[0]) * (random.random() ** 2)  # alçak ağırlıklı
    pos = np.array([
        random.uniform(-CAM_XY_X, CAM_XY_X),
        random.uniform(-CAM_XY_Y, CAM_XY_Y),
        z,
    ])
    if random.random() < GROUND_VIEW_FRACTION:
        # Genel zemin/ufuk: rastgele yaw, ufka yakın→belirgin aşağı bakış
        yaw = random.uniform(-math.pi, math.pi)
        depression = math.radians(random.uniform(-2.0, 45.0))
    else:
        # Pist üzerindeki rastgele bir noktaya bak
        aim = np.array([
            random.uniform(-RUNWAY_AIM_X, RUNWAY_AIM_X),
            random.uniform(-RUNWAY_AIM_Y, RUNWAY_AIM_Y),
            0.0,
        ])
        d = aim - pos
        horiz = math.hypot(d[0], d[1])
        if horiz < 3.0:            # tam tepesindeyse ufka doğru kaydır
            d[1] += 30.0
            horiz = math.hypot(d[0], d[1])
        yaw = math.atan2(d[1], d[0])
        depression = math.atan2(pos[2] - aim[2], horiz)
    body_pitch = depression + CAM_MOUNT_UP      # montaj yukarı bakışını telafi et
    roll = random.uniform(-math.radians(20), math.radians(20))
    return pos, (roll, body_pitch, yaw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=650)
    ap.add_argument("--out", default="vision/datasets/talon")
    ap.add_argument("--val-split", type=float, default=0.15)
    ap.add_argument("--prefix", default="negrw")
    ap.add_argument("--debug-dir", default="",
                    help="verilirse ilk 5 kare buraya da kopyalanır (göz kontrolü)")
    args = ap.parse_args()

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)
    if args.debug_dir:
        os.makedirs(args.debug_dir, exist_ok=True)

    node = Node()
    grabber = FrameGrabber(node)

    print(f"[NEG-RW] Kamera bekleniyor ({CAM_TOPIC})...")
    t0 = time.time()
    while grabber.snapshot()[0] is None and time.time() - t0 < 20:
        time.sleep(0.3)
    if grabber.snapshot()[0] is None:
        raise SystemExit("[NEG-RW] HATA: kamera karesi gelmedi. "
                         "gz sim (dataset_capture.sdf) aynı GZ_PARTITION'da mı?")

    # Talon'u sahneden çıkar — hiçbir karede görünmemeli (etiketler boş)
    if not _set_pose(node, TARGET, (600.0, 600.0, 5.0), (0, 0, 0)):
        raise SystemExit("[NEG-RW] HATA: mini_talon taşınamadı (set_pose)")
    print("[NEG-RW] Talon 600m uzağa park edildi. Toplama başlıyor...")
    time.sleep(0.5)

    from vision import geometry as geo
    TALON_PARK = (600.0, 600.0, 5.0)
    KENAR_PAYI = 30.0     # px; kadraj kenarına bu kadar yakını da reddet

    def talon_kadrajda(cam_pos_, cam_rpy_):
        """Park edilmiş Talon'un MERKEZİ kadraja (pay dahil) projekte oluyor mu?
        (poz + 25° tilt + FOV 125°). target_bbox kullanılmaz: 2px alt sınırı
        uzaktaki beneği 'görünmez' sayar, oysa 5px benek bile etiket zehirler."""
        cp, R = geo.camera_world_pose(cam_pos_, cam_rpy_)
        u, v, valid = geo.project_points(
            np.array([TALON_PARK], dtype=float), cp, R)
        return bool(valid[0]) and (-KENAR_PAYI <= u[0] <= geo.IMG_W + KENAR_PAYI
                                   and -KENAR_PAYI <= v[0] <= geo.IMG_H + KENAR_PAYI)

    saved = 0
    while saved < args.count:
        pos, rpy = random_negative_pose()
        # GEOMETRİK KORUMA: negatif karede hedef (park halinde bile) görünemez.
        if talon_kadrajda(pos, rpy):
            continue
        if not _set_pose(node, CAMERA, pos, rpy):
            time.sleep(0.1)
            continue
        # Taşınma sonrası TAZE kare bekle. En az 8 kare (~270 ms): set_pose
        # ışınlanmasında render, kamera pozundan ÖNCE gövde pozunu güncelleyip
        # kameranın KENDİ GÖVDESİNİ eski bakıştan görüntüleyebiliyor (17/650
        # karede hayalet drone çıktı, 7'si etiket zehirledi). Uzun bekleme
        # bu yarışı kapatır (pozitif toplayıcıdaki settle=0.20s'nin karşılığı).
        _, id0 = grabber.snapshot()
        t1 = time.time()
        frame = None
        while time.time() - t1 < 1.5:
            f, fid = grabber.snapshot()
            if fid >= id0 + 8:
                frame = f
                break
            time.sleep(0.02)
        if frame is None:
            continue

        split = "val" if random.random() < args.val_split else "train"
        name = f"{args.prefix}_{saved:05d}"
        cv2.imwrite(os.path.join(args.out, "images", split, name + ".jpg"), frame)
        open(os.path.join(args.out, "labels", split, name + ".txt"), "w").close()
        if args.debug_dir and saved < 5:
            cv2.imwrite(os.path.join(args.debug_dir, name + ".jpg"), frame)

        saved += 1
        if saved % 50 == 0:
            print(f"[NEG-RW]   {saved}/{args.count}")

    print(f"[NEG-RW] Bitti: {saved} pist/zemin negatifi → {args.out}")
    print("[NEG-RW] Yeniden eğit: python3 -m vision.train_yolo --epochs 100")


if __name__ == "__main__":
    main()
