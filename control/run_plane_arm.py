#!/usr/bin/env python3
"""
run_plane_arm.py — Plane keepalive + arm demo scripti.

Kullanım:
    cd ~/projects/avci_sim
    python -m control.run_plane_arm

Ön koşullar:
    - Simülasyon çalışıyor olmalı
    - plane portu 14541 üzerinde erişilebilir olmalı
"""

import time
import signal
import sys

sys.path.insert(0, "/home/kayra/projects/avci_sim")

from control.plane_functions import (
    connect_plane,
    start_gcs_keepalive,
    arm_plane,
    set_throttle,
    fly_forward,
    print_status,
    stop_gcs_keepalive,
    THROTTLE_CRUISE,
)


def shutdown(sig, frame):
    print("\n[DEMO] Durduruldu (Ctrl+C)")
    stop_gcs_keepalive()
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)


def main():
    print("=" * 50)
    print("[DEMO] Plane Keepalive + ARM")
    print("=" * 50)

    # 1. Bağlan
    connect_plane()

    # 2. Keepalive + ARM
    result = arm_plane(warmup_duration=3.0)
    if result is None or result[1] != 0:
        print("[DEMO] ARM başarısız!")
        print("[DEMO] Keepalive aktif tutuluyor. Ctrl+C ile çık.")
        while True:
            time.sleep(1)

    # 3. Basit throttle
    print("[DEMO] Throttle test: cruise 5s")
    set_throttle(THROTTLE_CRUISE, duration=5.0)

    # 4. Durum
    print_status()

    print("[DEMO] Demo tamamlandı. Keepalive devam ediyor. Ctrl+C ile çık.")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
