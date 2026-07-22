#!/usr/bin/env python3
"""
run_drone_square.py — Avcı drone kalkar ve KARE yörünge çizer (negatif veri uçuşu).

Amaç: hard-negative kare toplarken (vision/capture_negatives.py) drone'un kendi
kendine düzgün bir desende uçması. Her kenarda burun gidiş yönüne döner; köşe
dönüşlerindeki yatış/dönüş sırasında pervaneler kameraya girer — asıl istenen de bu.

Kullanım (simülasyon + SITL çalışırken):
    cd ~/projects/avci_sim
    python3 -m control.run_drone_square                # varsayılan: 25m irtifa, 40m kenar
    python3 -m control.run_drone_square --alt 30 --side 60
Ctrl+C → olduğu yerde LAND.
"""

import argparse
import math
import signal
import sys
import time

sys.path.insert(0, "/home/kayra/projects/avci_sim")

from control.drone_functions import (
    connect_drone,
    get_conn,
    takeoff_to_z,
    land_drone,
    _send_position_setpoint,
    _get_current_pos,
)

CORNER_TOL = 3.0        # köşeye "vardı" sayılacak mesafe (m)
CORNER_TIMEOUT = 60.0   # bir köşeye varamazsa sıradakine geç (s)
SETPOINT_HZ = 5.0


def _stop(sig, frame):
    print("\n[SQUARE] Ctrl+C — iniş yapılıyor...")
    try:
        land_drone()
    except Exception:
        pass
    sys.exit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alt", type=float, default=25.0, help="uçuş irtifası (m)")
    ap.add_argument("--side", type=float, default=40.0, help="kare kenarı (m)")
    ap.add_argument("--center-x", type=float, default=0.0,
                    help="kare merkezi, kalkış noktasına göre kuzey (m)")
    ap.add_argument("--center-y", type=float, default=0.0,
                    help="kare merkezi, kalkış noktasına göre doğu (m)")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    z = -abs(args.alt)   # NED: yukarı = eksi
    h = args.side / 2.0
    corners = [
        (args.center_x + h, args.center_y + h),
        (args.center_x + h, args.center_y - h),
        (args.center_x - h, args.center_y - h),
        (args.center_x - h, args.center_y + h),
    ]

    print("=" * 50)
    print(f"[SQUARE] Kare yörünge: kenar {args.side}m, irtifa {args.alt}m")
    print("[SQUARE] Ctrl+C ile bitir (otomatik iniş).")
    print("=" * 50)

    connect_drone()
    if not takeoff_to_z(target_z=z):
        print("[SQUARE] Kalkış başarısız!")
        return
    conn = get_conn()

    lap = 0
    idx = 0
    while True:
        tx, ty = corners[idx]
        nxt = corners[(idx + 1) % 4]
        # Burun gidiş yönüne dönük (köşede bir sonraki kenara doğru döner)
        yaw = math.atan2(nxt[1] - ty, nxt[0] - tx)

        t0 = time.time()
        while True:
            _send_position_setpoint(conn, tx, ty, z, yaw=yaw)
            time.sleep(1.0 / SETPOINT_HZ)
            px, py, _ = _get_current_pos(conn)
            if math.hypot(px - tx, py - ty) < CORNER_TOL:
                break
            if time.time() - t0 > CORNER_TIMEOUT:
                print(f"[SQUARE] Köşe {idx + 1} zaman aşımı, devam ediliyor.")
                break

        idx = (idx + 1) % 4
        if idx == 0:
            lap += 1
            print(f"[SQUARE] Tur {lap} tamamlandı.")


if __name__ == "__main__":
    main()
