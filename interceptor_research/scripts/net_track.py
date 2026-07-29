#!/usr/bin/env python3
"""Agin yorungesini izler; menzil/hiz olcer (Asama 4 dogrulamasi).

gz-transport Python ile /world/<dunya>/dynamic_pose/info (Pose_V) dinlenir.
Metin ciktisini regex'le ayristirmak yerine protobuf alanlari okunur -
CLI ciktisinda poz bloklari ic ice ve isim/konum eslesmesi kayabiliyordu.

Zaman ekseni SIM zamanidir (poz basliginin damgasi), duvar saati degil;
RTF 1'in altinda oldugu icin ikisi ayni sey degil.

Kullanim:
    ./net_track.py --sure 6
    ./net_track.py --sure 6 --csv out.csv
"""
import argparse
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

NET_MODEL = "net_cone"


class Tracker:
    def __init__(self, model: str):
        self.model = model
        self.samples: list[tuple[float, float, float, float]] = []
        self.lock = threading.Lock()

    def on_pose(self, msg: Pose_V) -> None:
        # Sim zamani MESAJ basliginda; tek tek poz basliklarinin damgasi bos gelir
        # (ilk denemede t hep 0.00 cikmisti).
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name != self.model:
                continue
            with self.lock:
                self.samples.append((t, p.position.x, p.position.y, p.position.z))


def main() -> int:
    ap = argparse.ArgumentParser(description="Ag yorunge izleyici")
    ap.add_argument("--dunya", default="net_test")
    ap.add_argument("--model", default=NET_MODEL)
    ap.add_argument("--sure", type=float, default=6.0, help="izleme suresi (duvar saati, sn)")
    ap.add_argument("--csv", help="ornekleri CSV'ye yaz")
    ap.add_argument("--hedef-menzil", type=float, default=15.0)
    args = ap.parse_args()

    tracker = Tracker(args.model)
    node = Node()
    topic = f"/world/{args.dunya}/dynamic_pose/info"
    if not node.subscribe(Pose_V, topic, tracker.on_pose):
        print(f"HATA: {topic} abone olunamadi")
        return 1

    time.sleep(args.sure)

    with tracker.lock:
        samples = list(tracker.samples)

    if not samples:
        print(f"HATA: '{args.model}' icin poz ornegi alinamadi.")
        print("  dynamic_pose/info yalnizca HAREKET EDEN modelleri yayinlar -")
        print("  ag hic kimildamadiysa burasi bos kalir.")
        return 1

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as f:
            f.write("t_sim_s,x_m,y_m,z_m\n")
            for t, x, y, z in samples:
                f.write(f"{t:.4f},{x:.4f},{y:.4f},{z:.4f}\n")
        print(f"CSV: {args.csv} ({len(samples)} ornek)")

    t0, x0, y0, z0 = samples[0]
    tlast, xlast, ylast, zlast = samples[-1]
    xmax = max(s[1] for s in samples)
    zmax = max(s[3] for s in samples)
    zmin = min(s[3] for s in samples)

    # Cikis hizi: ilk 0.2 sn sim zamanindaki ortalama yatay hiz
    early = [s for s in samples if s[0] - t0 <= 0.2]
    v0 = float("nan")
    if len(early) >= 2:
        dt = early[-1][0] - early[0][0]
        if dt > 1e-6:
            dx = early[-1][1] - early[0][1]
            dy = early[-1][2] - early[0][2]
            v0 = (dx * dx + dy * dy) ** 0.5 / dt

    yol = ((xlast - x0) ** 2 + (ylast - y0) ** 2) ** 0.5
    menzil = xmax - x0

    print(f"\n=== AG YORUNGESI ({len(samples)} ornek, {tlast - t0:.2f} sn SIM zamani) ===")
    print(f"  baslangic      : x={x0:8.3f}  y={y0:7.3f}  z={z0:7.3f}")
    print(f"  son            : x={xlast:8.3f}  y={ylast:7.3f}  z={zlast:7.3f}")
    print(f"  ileri menzil   : {menzil:8.3f} m")
    print(f"  yatay yol      : {yol:8.3f} m")
    print(f"  tepe / dip z   : {zmax:8.3f} / {zmin:.3f} m")
    print(f"  cikis hizi     : {v0:8.2f} m/s")

    durum = "GECTI" if menzil >= args.hedef_menzil else "KALDI"
    print(f"\n  Hedef menzil {args.hedef_menzil} m -> {durum} ({menzil:.2f} m)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
