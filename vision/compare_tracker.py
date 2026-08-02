"""
vision/compare_tracker.py — "yalnız detection" vs "detection + HybridSORT" karşılaştırması
(ground-truth'lu kontrollü deney).

Statik dataset world'de (dataset_capture.sdf) KAMERA SABİT durur, Talon pürüzsüz bir
yörüngede adım adım uçurulur (set_pose). Pozlar bilindiğinden her karenin gerçek bbox'ı
projeksiyon la bilinir (vision/geometry.py) → iki boru hattı AYNI kareler üzerinde
güdüm-ilgili metriklerle ölçülür:

  A) YALNIZ DETECTION (bugünkü güdüm girdisi): best_det(detect_all(f)) — her karede
     en yüksek güvenli kutu, kimlik yok.
  B) DETECTION + TRACKER (kilitli-ID politikası): tracker çıktısından bir ID'ye
     kilitlenilir; o ID bu karede eşleşmediyse Kalman TAHMİN kutusu (coast köprüsü)
     kullanılır (en çok --max-coast kare); track ölürse yeniden kilitlenilir.

Yörünge fazları (güdümün gerçek zorlukları): yakın geçiş → uzaklaşma (tespit kopmaları
başlar) → uzak seyir → kadraj DIŞINA çıkıp geri girme → yaklaşma → hızlı yanal manevra.

Kullanım:
  # 1) Ayrı partition'da başsız world:
  export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models
  GZ_PARTITION=trkcmp gz sim -s -r sim/gazebo_harmonic/worlds/dataset_capture.sdf &
  # 2) Yakala + değerlendir (varsayılan ikisi birden):
  GZ_PARTITION=trkcmp python3 -m vision.compare_tracker
  # Yakalanmış kareler üzerinde yalnız değerlendirme (sim gerekmez):
  python3 -m vision.compare_tracker --eval-only
"""

import argparse
import math
import os
import time

import cv2
import numpy as np

OUT_DIR = "vision/datasets/trkcmp"          # kareler + GT (gitignore'lu)
IOU_DOGRU = 0.2                              # kutu-GT örtüşmesi bunun üstüyse "doğru hedef"
IOU_YANLIS = 0.1                             # kutu var ama GT ile bunun altıysa "yanlış hedef"


# ═══════════════════════════ YAKALAMA ═══════════════════════════

CAM_POS = np.array([0.0, 0.0, 20.0])
CAM_RPY = (0.0, 0.0, 0.0)                    # kamera gövdeye göre zaten 25° yukarı bakar


def _traj_duv(n):
    """Yörünge: adım i → (mesafe d, piksel u, piksel v). Pürüzsüz, fazlı.
    u/v kamera kadraj koordinatı (640×480); u>640 kadraj dışı demektir."""
    pts = []
    for i in range(n):
        t = i / max(n - 1, 1)
        if t < 0.20:                                  # yakın geçiş (d=8): sağdan sola
            s = t / 0.20
            d, u, v = 8.0, 450 - 250 * s, 240 + 20 * math.sin(6 * s)
        elif t < 0.43:                                # uzaklaşma 8→32 m: kopmalar başlar
            s = (t - 0.20) / 0.23
            d, u, v = 8 + 24 * s, 200 + 150 * s, 240 + 30 * math.sin(4 * s)
        elif t < 0.57:                                # uzak seyir (32-35 m)
            s = (t - 0.43) / 0.14
            d, u, v = 32 + 3 * math.sin(3 * s), 350 - 100 * s, 230 + 15 * math.sin(5 * s)
        elif t < 0.70:                                # kadraj DIŞINA çık ve geri gir
            s = (t - 0.57) / 0.13
            d = 30.0
            u = 250 + 700 * math.sin(math.pi * s)     # tepe ~950 → kadraj dışı
            v = 235.0
        elif t < 0.90:                                # yaklaşma 30→10 m
            s = (t - 0.70) / 0.20
            d, u, v = 30 - 20 * s, 250 + 70 * s, 235 + 25 * math.sin(4 * s)
        else:                                         # hızlı yanal manevra (d=10)
            s = (t - 0.90) / 0.10
            d, u, v = 10.0, 320 + 200 * math.sin(4 * math.pi * s), 240.0
        pts.append((d, u, v))
    return pts


def capture(n_frames, settle_frames):
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image as GzImage  # noqa: F401 (FrameGrabber içinde)
    from vision import geometry as geo
    from vision.capture_dataset import FrameGrabber, _set_pose, CAMERA, TARGET

    node = Node()
    grabber = FrameGrabber(node)
    print("[CMP] Kamera bekleniyor (/iris_cam/image)...")
    t0 = time.time()
    while grabber.snapshot()[0] is None and time.time() - t0 < 20:
        time.sleep(0.3)
    if grabber.snapshot()[0] is None:
        raise RuntimeError("Kamera görüntüsü yok — gz sim (dataset_capture.sdf) "
                           "aynı GZ_PARTITION'da çalışıyor mu?")

    # Kamera sabit yerine otur; ilk ışınlanmanın render'ı otursun diye uzun bekle
    _set_pose(node, CAMERA, CAM_POS, CAM_RPY)
    time.sleep(1.0)

    cam_pos, R_cam = geo.camera_world_pose(CAM_POS, CAM_RPY)

    def duv_to_world(d, u, v):
        Xo = (u - geo.CX) / geo.FX * d
        Yo = (v - geo.CY) / geo.FY * d
        return cam_pos + R_cam @ np.array([d, -Xo, -Yo])

    traj = _traj_duv(n_frames)
    world_pts = [duv_to_world(*p) for p in traj]

    os.makedirs(os.path.join(OUT_DIR, "frames"), exist_ok=True)
    gt_rows = []                                     # [i, d, x1, y1, x2, y2] (yoksa -1)
    kayip_gt = 0
    for i, (wp, (d, _u, _v)) in enumerate(zip(world_pts, traj)):
        # Yönelim: hız yönüne hizalı (pürüzsüz uçuş görüntüsü)
        nxt = world_pts[min(i + 1, n_frames - 1)]
        vel = nxt - wp
        if np.linalg.norm(vel[:2]) > 1e-6:
            yaw = math.atan2(vel[1], vel[0])
        else:
            yaw = 0.0
        pitch = -math.atan2(vel[2], max(np.linalg.norm(vel[:2]), 1e-6))
        pitch = max(-0.5, min(0.5, pitch))
        rpy = (0.0, pitch, yaw)

        _set_pose(node, TARGET, wp, rpy)
        # Render'ın hedefin YENİ pozunu içermesi için taze kare bekle
        _, fid0 = grabber.snapshot()
        deadline = time.time() + 2.0
        while time.time() < deadline:
            frame, fid = grabber.snapshot()
            if fid >= fid0 + settle_frames:
                break
            time.sleep(0.005)

        bb = geo.target_bbox(wp, rpy, CAM_POS, CAM_RPY)
        cv2.imwrite(os.path.join(OUT_DIR, "frames", f"{i:05d}.jpg"),
                    frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if bb is None:
            gt_rows.append([i, d, -1, -1, -1, -1]); kayip_gt += 1
        else:
            gt_rows.append([i, d, *[float(x) for x in bb]])
        if (i + 1) % 100 == 0:
            print(f"[CMP]   yakalama {i + 1}/{n_frames}")

    np.savez(os.path.join(OUT_DIR, "gt.npz"), gt=np.array(gt_rows, dtype=np.float64))
    print(f"[CMP] Yakalama bitti: {n_frames} kare ({kayip_gt} karede hedef kadraj dışı) → {OUT_DIR}")


# ═══════════════════════════ DEĞERLENDİRME ═══════════════════════════

def _iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def _center(b):
    return np.array([(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0])


class Metrik:
    """Bir boru hattının GT'ye karşı sayaçları."""
    def __init__(self, ad):
        self.ad = ad
        self.kutu = 0          # kutu üretilen kare (GT görünürken)
        self.dogru = 0         # IoU >= IOU_DOGRU
        self.yanlis = 0        # kutu var ama IoU < IOU_YANLIS
        self.iou_top = 0.0
        self.merkez_top = 0.0
        self.jit = []          # hareket-telafili titreşim |Δkutu − ΔGT|
        self.dis_kutu = 0      # GT kadraj DIŞIYKEN kutu üretme (riskli davranış)
        self._onceki = None    # (kutu_merkez, gt_merkez)

    def kare(self, box, gt):
        if gt is None:
            if box is not None:
                self.dis_kutu += 1
            self._onceki = None
            return
        if box is None:
            self._onceki = None
            return
        self.kutu += 1
        i = _iou(box, gt)
        if i >= IOU_DOGRU:
            self.dogru += 1
            self.iou_top += i
            c, g = _center(box), _center(gt)
            self.merkez_top += float(np.linalg.norm(c - g))
            if self._onceki is not None:
                dc = c - self._onceki[0]
                dg = g - self._onceki[1]
                self.jit.append(float(np.linalg.norm(dc - dg)))
            self._onceki = (c, g)
        else:
            self._onceki = None
            if i < IOU_YANLIS:
                self.yanlis += 1


def evaluate(max_coast):
    from vision import detector
    from vision.tracker import TalonTracker, TargetLock

    data = np.load(os.path.join(OUT_DIR, "gt.npz"))["gt"]
    n = len(data)
    gt_gorunur = int((data[:, 2] >= 0).sum())
    print(f"[CMP] Değerlendirme: {n} kare ({gt_gorunur} karede GT görünür), "
          f"max_coast={max_coast}")

    tr = TalonTracker()
    kilit = TargetLock(tr, max_coast=max_coast)   # gcs'deki politikanın kendisi
    A = Metrik("A yalnız detection")
    B = Metrik("B det+tracker(kilit)")
    b_taze = b_byte = b_coast = 0  # B kutusunun kaynağı (GT görünürken)
    a_yok_b_var = 0               # A None derken B kutu verdi (GT görünürken)

    for row in data:
        i = int(row[0])
        gt = None if row[2] < 0 else tuple(row[2:6])
        frame = cv2.imread(os.path.join(OUT_DIR, "frames", f"{i:05d}.jpg"))

        dets = detector.detect_all(frame)
        a_box = None
        d = detector.best_det(dets)
        if d is not None:
            a_box = tuple(float(v) for v in d["bbox"])

        out = tr.update(dets, frame)
        lock = kilit.step(out, d)
        b_box, kaynak = None, None
        if lock is not None:
            b_box = lock["bbox"]
            kaynak = ("coast" if lock["kaynak"] == "tahmin"
                      else ("taze" if lock["conf"] >= 0.45 else "byte"))

        A.kare(a_box, gt)
        B.kare(b_box, gt)
        if gt is not None and b_box is not None:
            if kaynak == "taze":
                b_taze += 1
            elif kaynak == "byte":
                b_byte += 1
            elif kaynak == "coast":
                b_coast += 1
            if a_box is None:
                a_yok_b_var += 1

    def rapor(m):
        drm = m.dogru or 1
        return (f"  kutu üretilen kare : {m.kutu}/{gt_gorunur}\n"
                f"  doğru hedef (IoU≥{IOU_DOGRU}) : {m.dogru}/{gt_gorunur}"
                f"  (%{100.0 * m.dogru / max(gt_gorunur, 1):.1f})\n"
                f"  YANLIŞ hedef (IoU<{IOU_YANLIS}) : {m.yanlis}\n"
                f"  ort IoU / merkez hatası : {m.iou_top / drm:.3f} / {m.merkez_top / drm:.1f} px\n"
                f"  titreşim (|Δkutu−ΔGT| ort) : "
                f"{np.mean(m.jit) if m.jit else float('nan'):.2f} px\n"
                f"  GT kadraj dışıyken kutu : {m.dis_kutu}")

    print("\n════════ SONUÇ ════════")
    print(f"[A] YALNIZ DETECTION (bugünkü güdüm girdisi)\n{rapor(A)}")
    print(f"\n[B] DETECTION + TRACKER (kilitli ID, max_coast={max_coast})\n{rapor(B)}")
    print(f"\n[B ayrıntı] kutu kaynağı: taze(conf≥0.45)={b_taze}, "
          f"BYTE düşük-conf={b_byte}, coast köprüsü={b_coast}")
    print(f"[B ayrıntı] kilitlenme sayısı: {kilit.relock_sayisi} (1 = hiç kopmadı)")
    print(f"[KAZANÇ] A'nın kutu veremediği, B'nin verdiği kare: {a_yok_b_var} "
          f"(GT görünürken)")
    return {"A": A, "B": B, "a_yok_b_var": a_yok_b_var,
            "relock": kilit.relock_sayisi,
            "b_taze": b_taze, "b_byte": b_byte, "b_coast": b_coast}


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="detection vs detection+tracker (GT'li)")
    ap.add_argument("--frames", type=int, default=600)
    ap.add_argument("--settle-frames", type=int, default=2,
                    help="set_pose sonrası beklenecek taze kare sayısı")
    ap.add_argument("--max-coast", type=int, default=15,
                    help="kilitli ID için Kalman köprüsünün kare sınırı")
    ap.add_argument("--capture-only", action="store_true")
    ap.add_argument("--eval-only", action="store_true")
    args = ap.parse_args()

    if not args.eval_only:
        capture(args.frames, args.settle_frames)
    if not args.capture_only:
        evaluate(args.max_coast)
