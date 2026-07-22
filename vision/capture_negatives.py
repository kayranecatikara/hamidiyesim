"""
vision/capture_negatives.py — CANLI simden HARD-NEGATIVE (etiketsiz) kare toplar.

Amaç: modelin kendi pervanemizi/kanadımızı "talon" sanmasını engellemek. Canlı
uçuşta (pervaneler DÖNERKEN) /iris_cam/image'den kare kaydeder, YOLO etiketi BOŞ
bırakılır → ultralytics bu kareleri "arka plan / hedef yok" örneği sayar.

ÖNEMLİ: Toplama sırasında Talon kadrajda OLMAMALI (talon'lu kare boş etiketle
kaydedilirse eğitimi zehirler). Ya talon'suz uçuş yap ya da talon'dan uzağa bak.

Kullanım:
  # 1) Normal simülasyonu başlat (ArduPilot + Gazebo, avci_harmonic), iris'i uçur.
  # 2) Talon kadrajda değilken:
  python3 -m vision.capture_negatives --count 500
  # 3) Yeniden eğit: python3 -m vision.train_yolo --epochs N  (aynı dataset klasörü)
"""

import argparse
import os
import random
import threading
import time

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GzImage

CAM_TOPIC = "/iris_cam/image"


class FrameGrabber:
    """gz-transport kamera aboneliği; en son kareyi ve sayacını thread-safe verir."""
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
            print(f"[NEG] kare hatası: {e}")

    def snapshot(self):
        with self._lock:
            if self._frame is None:
                return None, 0
            return self._frame.copy(), self._id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=500, help="toplanacak negatif kare sayısı")
    ap.add_argument("--interval", type=float, default=0.4,
                    help="kareler arası bekleme (s) — art arda aynı görüntüyü kaydetmemek için")
    ap.add_argument("--out", default="vision/datasets/talon")
    ap.add_argument("--val-split", type=float, default=0.15)
    ap.add_argument("--prefix", default="neg", help="dosya adı öneki (mevcutlarla çakışmasın)")
    args = ap.parse_args()

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)

    node = Node()
    grabber = FrameGrabber(node)

    print(f"[NEG] Kamera bekleniyor ({CAM_TOPIC})...")
    t0 = time.time()
    while grabber.snapshot()[0] is None and time.time() - t0 < 15:
        time.sleep(0.3)
    if grabber.snapshot()[0] is None:
        print("[NEG] HATA: kameradan görüntü gelmedi. Simülasyon çalışıyor mu?")
        return

    print(f"[NEG] Toplama başladı: {args.count} kare, {args.interval}s aralıkla.")
    print("[NEG] DİKKAT: Talon kadrajda olmasın!")

    saved = 0
    last_id = 0
    while saved < args.count:
        frame, fid = grabber.snapshot()
        if frame is None or fid == last_id:      # yeni kare gelmedi (sim duraklamış olabilir)
            time.sleep(0.05)
            continue
        last_id = fid

        split = "val" if random.random() < args.val_split else "train"
        name = f"{args.prefix}_{saved:05d}"
        cv2.imwrite(os.path.join(args.out, "images", split, name + ".jpg"), frame)
        # Boş etiket dosyası → YOLO için "bu karede hedef yok" (hard negative)
        open(os.path.join(args.out, "labels", split, name + ".txt"), "w").close()

        saved += 1
        if saved % 50 == 0:
            print(f"[NEG]   {saved}/{args.count}")
        time.sleep(args.interval)

    print(f"[NEG] Bitti: {saved} negatif kare eklendi → {args.out}")
    print("[NEG] Şimdi yeniden eğit: python3 -m vision.train_yolo")


if __name__ == "__main__":
    main()
