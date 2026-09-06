#!/usr/bin/env python3
"""Ag atisi ve yakalama sekansini kare kare goruntuler (aksiyon cekimi).

net_capture_test dunyasindaki takip kamerasina abone olur, ateslemeden once
baslar ve verilen araliklarla PNG yazar. Boylece:
  ates oncesi -> ag ucusta -> hedefe carpma -> yakalama sonrasi
kareleri elde edilir.

Sunucuyu BU SCRIPT baslatmaz; ayri terminalde acik olmali:
    source scripts/env.sh
    gz sim -s -r --headless-rendering worlds/net_capture_test.sdf

Kullanim:  ./52_action_shots.py --kare 6 --aralik 0.35
"""
import argparse
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node
from PIL import Image as PILImage

ROOT = Path(__file__).resolve().parent.parent


class Son:
    """Son gelen kareyi tutar."""

    def __init__(self):
        self.msg: Image | None = None
        self.lock = threading.Lock()

    def cb(self, msg: Image) -> None:
        with self.lock:
            self.msg = msg

    def yaz(self, path: Path) -> bool:
        with self.lock:
            m = self.msg
        if m is None:
            return False
        raw = np.frombuffer(m.data, dtype=np.uint8)
        kanal = raw.size // (m.width * m.height)
        if kanal < 3:
            return False
        arr = raw.reshape(m.height, m.width, kanal)[:, :, :3]
        PILImage.fromarray(arr, "RGB").save(path)
        return True


def main() -> int:
    ap = argparse.ArgumentParser(description="Ag atisi aksiyon cekimi")
    ap.add_argument("--topic", default="/capture_test/view")
    ap.add_argument("--kare", type=int, default=6, help="kac kare")
    ap.add_argument("--aralik", type=float, default=0.35, help="kareler arasi sn")
    ap.add_argument("--hiz", type=float, default=20.0, help="ag cikis hizi")
    ap.add_argument("--model", default="bullet_net_interceptor",
                    help="ates topic'i bundan turetilir")
    ap.add_argument("--klasor", default=str(ROOT / "docs" / "goruntuler"))
    args = ap.parse_args()

    klasor = Path(args.klasor)
    klasor.mkdir(parents=True, exist_ok=True)

    son = Son()
    node = Node()
    if not node.subscribe(Image, args.topic, son.cb):
        print(f"HATA: {args.topic} abone olunamadi")
        return 1

    yakalandi = threading.Event()
    node.subscribe(StringMsg, "/net/captured", lambda m: yakalandi.set())

    # Ilk kare gelene kadar bekle
    for _ in range(50):
        if son.msg is not None:
            break
        time.sleep(0.2)
    if son.msg is None:
        print(f"HATA: {args.topic} uzerinden goruntu gelmedi (sunucu acik mi?)")
        return 1

    print("Aksiyon cekimi:")
    son.yaz(klasor / "atis_0_once.png")
    print(f"  [OK] atis_0_once.png")

    subprocess.run(["python3", str(ROOT / "scripts" / "fire_net.py"),
                    "--hiz", str(args.hiz), "--model", args.model],
                   capture_output=True, text=True)

    for i in range(1, args.kare + 1):
        time.sleep(args.aralik)
        ad = f"atis_{i}_ucus.png"
        if son.yaz(klasor / ad):
            print(f"  [OK] {ad}")

    if yakalandi.wait(timeout=10):
        time.sleep(0.6)
        son.yaz(klasor / "atis_9_yakalandi.png")
        print("  [OK] atis_9_yakalandi.png  <- YAKALAMA GERCEKLESTI")
    else:
        print("  not: yakalama olayi gelmedi")

    print(f"\nGoruntuler: {klasor}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
