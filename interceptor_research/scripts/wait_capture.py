#!/usr/bin/env python3
"""/net/captured topic'inde yakalama olayi bekler.

NEDEN LOG DEGIL TOPIC:
  Once sunucu logu grep'lenerek yakalama yoklaniyordu. gz sim'in stdout'u
  dosyaya yazarken BLOK TAMPONLU oldugu icin satirlar surec bitene kadar
  diske dusmuyor - kosum sirasinda log bos gorunuyor, 5/5 "kacirdi"
  raporlaniyordu. Yakalama olayi zaten NetCapturePlugin tarafindan
  topic'e basiliyor; dogru arayuz bu.

Cikis kodu: 0 = yakalandi, 1 = zaman asimi

Kullanim:  ./wait_capture.py --sure 25
"""
import argparse
import threading
import time

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


def main() -> int:
    ap = argparse.ArgumentParser(description="Yakalama olayi bekleyici")
    ap.add_argument("--topic", default="/net/captured")
    ap.add_argument("--sure", type=float, default=25.0, help="zaman asimi (sn)")
    args = ap.parse_args()

    event = threading.Event()
    yakalanan = {"ad": ""}

    def on_capture(msg: StringMsg) -> None:
        yakalanan["ad"] = msg.data
        event.set()

    node = Node()
    if not node.subscribe(StringMsg, args.topic, on_capture):
        print(f"HATA: {args.topic} abone olunamadi")
        return 1

    t0 = time.time()
    if event.wait(timeout=args.sure):
        print(f"YAKALANDI: {yakalanan['ad']}  ({time.time() - t0:.1f} sn sonra)")
        return 0

    print(f"YAKALAMA YOK ({args.sure:.0f} sn beklendi)")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
