#!/usr/bin/env python3
"""Vitrin dunyasindaki kameralardan PNG ekran goruntusu alir.

gz sim'in kendi ekran goruntusu ozelligi GUI gerektiriyor; burada dogrudan
kamera sensorunun Image topic'ine abone olup PNG yaziyoruz - headless calisir.

Kullanim:
    ./51_capture_shots.py                          # varsayilan iki kamera
    ./51_capture_shots.py --topic /showcase/wide --cikti foo.png
"""
import argparse
import threading
import time
from pathlib import Path

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PILImage

ROOT = Path(__file__).resolve().parent.parent
VARSAYILAN = [
    ("/showcase/wide", "vitrin_genis.png"),
    ("/showcase/closeup", "vitrin_yakin.png"),
]


class Grabber:
    def __init__(self):
        self.msg: Image | None = None
        self.event = threading.Event()

    def cb(self, msg: Image) -> None:
        if self.msg is None:
            self.msg = msg
            self.event.set()


def to_png(msg: Image, path: Path) -> bool:
    w, h = msg.width, msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    kanal = raw.size // (w * h)
    if kanal < 3:
        print(f"  HATA: beklenmeyen piksel bicimi (kanal={kanal})")
        return False
    arr = raw.reshape(h, w, kanal)[:, :, :3]
    PILImage.fromarray(arr, "RGB").save(path)
    return True


def grab(topic: str, out: Path, timeout: float) -> bool:
    g = Grabber()
    node = Node()
    if not node.subscribe(Image, topic, g.cb):
        print(f"  HATA: {topic} abone olunamadi")
        return False
    if not g.event.wait(timeout):
        print(f"  HATA: {topic} uzerinden goruntu gelmedi ({timeout:.0f} sn)")
        return False
    ok = to_png(g.msg, out)
    if ok:
        print(f"  [OK] {topic} -> {out}  ({g.msg.width}x{g.msg.height})")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="Vitrin ekran goruntusu")
    ap.add_argument("--topic")
    ap.add_argument("--cikti")
    ap.add_argument("--klasor", default=str(ROOT / "docs" / "goruntuler"))
    ap.add_argument("--timeout", type=float, default=25.0)
    args = ap.parse_args()

    klasor = Path(args.klasor)
    klasor.mkdir(parents=True, exist_ok=True)

    isler = ([(args.topic, args.cikti or "goruntu.png")] if args.topic
             else VARSAYILAN)

    print("Goruntu aliniyor:")
    basari = 0
    for topic, dosya in isler:
        # Her kamera icin ayri Node: ayni Node'da iki abone bazen
        # ilk mesajdan sonra digerini beslemeyi geciktiriyor
        if grab(topic, klasor / dosya, args.timeout):
            basari += 1
        time.sleep(0.3)

    print(f"\n{basari}/{len(isler)} goruntu alindi -> {klasor}")
    return 0 if basari else 1


if __name__ == "__main__":
    raise SystemExit(main())
