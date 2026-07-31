#!/usr/bin/env python3
"""
run_drone_hover.py — Drone kalkış + hareket + yaw demo scripti.

Kullanım:
    cd ~/projects/avci_sim
    python3 -m control.demos.run_drone_hover

Ön koşullar:
    - Simülasyon çalışıyor olmalı
    - iris kontrol portu 14541 erişilebilir olmalı
"""

import time
import signal
import sys

import os
# Depo kökünü bu dosyanın konumundan türet (control/demos/ -> depo kökü)
sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from control.drone_functions import (
    connect_drone,
    takeoff_to_z,
    hover,
    move_forward,
    move_backward,
    move_right,
    move_left,
    yaw_right,
    yaw_left,
    land_drone,
    print_status,
)


def shutdown(sig, frame):
    print("\n[DEMO] Durduruldu (Ctrl+C)")
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)


def main():
    print("=" * 50)
    print("[DEMO] Drone Hover + Hareket Demo")
    print("=" * 50)

    # 1. Bağlan
    connect_drone()

    # 2. Kalkış
    success = takeoff_to_z(target_z=-2.0)
    if not success:
        print("[DEMO] Kalkış başarısız!")
        return

    # 3. Hover
    print("[DEMO] 3s hover...")
    hover(duration=3.0)

    # 4. Hareketler
    print("[DEMO] 2m ileri...")
    move_forward(distance=2.0)
    hover(duration=2.0)

    print("[DEMO] 2m sağa...")
    move_right(distance=2.0)
    hover(duration=2.0)

    print("[DEMO] 2m sola...")
    move_left(distance=2.0)
    hover(duration=2.0)

    print("[DEMO] 2m geri...")
    move_backward(distance=2.0)
    hover(duration=2.0)

    # 5. Yaw
    print("[DEMO] 90° sağa yaw...")
    yaw_right(degrees=90.0)
    hover(duration=2.0)

    print("[DEMO] 90° sola yaw...")
    yaw_left(degrees=90.0)
    hover(duration=2.0)

    # 6. Durum
    print_status()

    # 7. İniş
    print("[DEMO] İniş...")
    land_drone()

    print("[DEMO] Demo tamamlandı")


if __name__ == "__main__":
    main()
