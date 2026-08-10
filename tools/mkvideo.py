#!/usr/bin/env python3
"""tools/mkvideo.py — ucus_kaydi kayıt dizinini (frames/ + meta.csv) tek videoya
birleştirir (cv2 ile; ffmpeg gerektirmez). Her kareye telemetri bindirir
(mesafe, faz, hız) ki video analizinde ne olduğu okunabilsin.

Kullanım: python3 tools/mkvideo.py <kayit_dizini> <cikti.mp4> [fps]
"""
import csv
import glob
import os
import sys

import cv2


def main():
    d = sys.argv[1]
    out = sys.argv[2]
    fps = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0

    frames = sorted(glob.glob(os.path.join(d, "frames", "f*.jpg")))
    if not frames:
        print(f"KARE YOK: {d}/frames")
        return 1

    # meta.csv: kare -> satır (mesafe, chase_aktif, plane_spd, iris_spd)
    meta = {}
    mpath = os.path.join(d, "meta.csv")
    if os.path.exists(mpath):
        with open(mpath) as f:
            for r in csv.DictReader(f):
                try:
                    meta[int(r["kare"])] = r
                except (KeyError, ValueError):
                    pass

    first = cv2.imread(frames[0])
    if first is None:
        print(f"KARE OKUNAMADI: {frames[0]}")
        return 1
    h, w = first.shape[:2]
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    fnt = cv2.FONT_HERSHEY_SIMPLEX

    for fp in frames:
        img = cv2.imread(fp)
        if img is None:
            continue
        # kare no dosya adından (f0007.jpg -> 7)
        try:
            kno = int(os.path.basename(fp)[1:-4])
        except ValueError:
            kno = -1
        m = meta.get(kno, {})
        dist = m.get("mesafe", "?")
        aktif = m.get("chase_aktif", "?")
        pspd = m.get("plane_spd", "?")
        ispd = m.get("iris_spd", "?")
        etiket = (f"#{kno} mesafe={dist}m chase={aktif} "
                  f"p_spd={pspd} i_spd={ispd}")
        cv2.rectangle(img, (0, 0), (w, 22), (0, 0, 0), -1)
        cv2.putText(img, etiket, (5, 16), fnt, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
        vw.write(img)

    vw.release()
    print(f"video: {out}  ({len(frames)} kare, {fps:.0f} fps, {w}x{h})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
