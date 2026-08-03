"""
vision/hybridsort_video.py — HybridSORT'u video dosyası üzerinde offline çalıştırır
(YOLO tespit + takip + ID'li çıktı videosu). Canlı gcs pipeline'ıyla AYNI
sarmalayıcıyı ve parametreleri kullanır (vision/tracker.py) — offline'da iyi
çalışan ayar canlıda da aynı davranır.

Kullanım:
    python3 -m vision.hybridsort_video --video test.mp4
    python3 -m vision.hybridsort_video --video test.mp4 \
        --model vision/models/avci_yolo.pt --output cikis.mp4 --debug

Not: Orijinal script tespitleri det_thresh(0.3) altında kesip tracker'a veriyordu;
burada model low_thresh(0.1) eşiğiyle çalışır ve 0.1-0.3 bandı da tracker'a gider —
HybridSort'un BYTE düşük-skor eşleşmesi (use_byte) ancak böyle malzeme bulur.
Yüksek güvenli tespitler iki yaklaşımda da birebir aynıdır.
"""

import argparse
import os
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from vision.tracker import TalonTracker, TRACKER_PARAMS, draw_tracks

_DEFAULT_MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              "vision", "models", "avci_yolo.pt")


def load_reid_model(weights_path: str, device: str):
    from boxmot.reid.core.reid import ReID
    p = Path(weights_path)
    if not p.exists():
        raise FileNotFoundError(f"ReID ağırlık dosyası bulunamadı: {p}")
    print(f"[INFO] ReID modeli yükleniyor: {p}")
    return ReID(path=p, device=device)


def yolo_to_detections(results, thresh: float) -> np.ndarray:
    """YOLO → BoxMOT formatı: [x1, y1, x2, y2, conf, cls]"""
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return np.empty((0, 6), dtype=np.float32)
    xyxy  = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy().reshape(-1, 1)
    cls   = boxes.cls.cpu().numpy().reshape(-1, 1)
    dets  = np.concatenate([xyxy, confs, cls], axis=1).astype(np.float32)
    return dets[dets[:, 4] >= thresh]


class FfmpegWriter:
    def __init__(self, path: str, fps: float, width: int, height: int):
        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo", "-vcodec", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}",
            "-r", str(fps),
            "-i", "pipe:0",
            "-vcodec", "libx264",
            "-pix_fmt", "yuv420p",
            "-crf", "18",
            path,
        ]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL)

    def write(self, frame: np.ndarray):
        self.proc.stdin.write(frame.tobytes())

    def release(self):
        self.proc.stdin.close()
        self.proc.wait()


def run(model_path: str, video_path: str, output_path: str,
        reid_weights: str | None = None, show: bool = False, debug: bool = False,
        max_frames: int = 0):

    from ultralytics import YOLO

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[INFO] Cihaz  : {device}")

    model      = YOLO(model_path)
    reid_model = load_reid_model(reid_weights, device) if reid_weights else None
    tracker    = TalonTracker(reid_model=reid_model,
                              with_reid=reid_model is not None)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Video açılamadı: {video_path}")

    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if max_frames:
        total = min(total, max_frames)

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    writer = FfmpegWriter(output_path, fps, width, height)

    print(f"[INFO] Video  : {video_path}  ({width}x{height} @ {fps:.1f} fps, {total} kare)")
    print(f"[INFO] Çıktı  : {output_path}")
    print(f"[INFO] ReID   : {'açık → ' + reid_weights if reid_weights else 'kapalı'}")
    if debug:
        print("[INFO] Debug modu açık — ilk 5 kare detaylı loglanacak")

    feed_thresh = TRACKER_PARAMS["low_thresh"]   # BYTE bandı dahil tracker'a git
    frame_idx   = 0
    n_det_kare  = n_trk_kare = 0
    ids_gorulen = set()
    t0 = time.perf_counter()

    while True:
        ret, frame = cap.read()
        if not ret or (max_frames and frame_idx >= max_frames):
            break
        frame_idx += 1

        results = model(frame, conf=feed_thresh, verbose=False)
        dets    = yolo_to_detections(results, feed_thresh)
        tracks  = tracker.update(dets, frame)

        if len(dets):
            n_det_kare += 1
        if len(tracks):
            n_trk_kare += 1
            ids_gorulen.update(int(t[4]) for t in tracks)

        # ── Debug: ilk 5 kareyi detaylı logla ──────────────────────────────
        if debug and frame_idx <= 5:
            print(f"\n── Kare {frame_idx} ──────────────────────────")
            if len(dets) > 0:
                print(f"  Tespitler ({len(dets)} adet):")
                for d in dets:
                    print(f"    x1={d[0]:.0f} y1={d[1]:.0f} x2={d[2]:.0f} y2={d[3]:.0f}"
                          f"  conf={d[4]:.3f}  cls={int(d[5])}")
            else:
                print("  Tespit yok (model hiçbir şey görmedi)")

            if len(tracks) > 0:
                print(f"  Takipler ({len(tracks)} adet):")
                for t in tracks:
                    print(f"    ID={int(t[4])}  conf={t[5]:.3f}  cls={int(t[6])}"
                          f"  bbox=[{t[0]:.0f},{t[1]:.0f},{t[2]:.0f},{t[3]:.0f}]")
            else:
                print(f"  Takip yok  (track_thresh={TRACKER_PARAMS['track_thresh']},"
                      f" min_hits={TRACKER_PARAMS['min_hits']})")
        # ───────────────────────────────────────────────────────────────────

        annotated = draw_tracks(frame.copy(), tracks)

        if show:
            cv2.imshow("HybridSORT", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        writer.write(annotated)

        if frame_idx % 100 == 0:
            print(f"  [{frame_idx:>5}/{total}]  tespit:{len(dets):>3}  takip:{len(tracks):>3}")

    cap.release()
    writer.release()
    if show:
        cv2.destroyAllWindows()

    sure = time.perf_counter() - t0
    print(f"\n[✓] Tamamlandı → {output_path}")
    print(f"    {frame_idx} kare / {sure:.1f} s = {frame_idx / max(sure, 1e-9):.1f} fps işleme")
    print(f"    Tespitli kare : {n_det_kare}/{frame_idx}")
    print(f"    Takipli kare  : {n_trk_kare}/{frame_idx}   ID'ler: {sorted(ids_gorulen)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HybridSORT + YOLO (BoxMOT)")
    parser.add_argument("--model",        default=_DEFAULT_MODEL,
                        help=f"YOLO ağırlıkları (varsayılan: {_DEFAULT_MODEL})")
    parser.add_argument("--video",        required=True)
    parser.add_argument("--output",       default=None,
                        help="Çıktı videosu (varsayılan: <video>_tracked.mp4)")
    parser.add_argument("--reid-weights", default=None)
    parser.add_argument("--show",         action="store_true")
    parser.add_argument("--max-frames",   type=int, default=0,
                        help="En fazla bu kadar kare işle (0 = hepsi)")
    parser.add_argument("--debug",        action="store_true",
                        help="İlk 5 kareyi detaylı logla")
    args = parser.parse_args()

    out = args.output or str(Path(args.video).with_name(Path(args.video).stem + "_tracked.mp4"))
    run(
        model_path=args.model,
        video_path=args.video,
        output_path=out,
        reid_weights=args.reid_weights,
        show=args.show,
        debug=args.debug,
        max_frames=args.max_frames,
    )
