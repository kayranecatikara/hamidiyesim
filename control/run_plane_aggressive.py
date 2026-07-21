#!/usr/bin/env python3
"""
run_plane_aggressive.py — Plane agresif manevra demo scripti.

Kullanım:
    cd ~/projects/avci_sim
    python -m control.run_plane_aggressive

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
    arm_plane,
    stop_gcs_keepalive,
)
from control.plane_patterns import (
    takeoff_then_stabilize,
    aggressive_maneuver_1,
    aggressive_maneuver_2,
    aggressive_maneuver_3,
    loiter,
)


def shutdown(sig, frame):
    print("\n[DEMO] Durduruldu (Ctrl+C)")
    stop_gcs_keepalive()
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)


def main():
    print("=" * 50)
    print("[DEMO] Plane Agresif Manevralar")
    print("=" * 50)

    # 1. Bağlan + ARM
    connect_plane()
    result = arm_plane(warmup_duration=3.0)
    if result is None or result[1] != 0:
        print("[DEMO] ARM başarısız!")
        return

    # 2. Takeoff + Stabilize
    takeoff_then_stabilize()

    # 3. Agresif manevralar
    print("[DEMO] Manevra 1: Tırmanış-Dalış-Toparlanma")
    aggressive_maneuver_1()

    print("[DEMO] Manevra 2: Keskin S-dönüşü")
    aggressive_maneuver_2()

    print("[DEMO] Manevra 3: Spiral tırmanış")
    aggressive_maneuver_3()

    # 4. Loiter
    loiter(duration=5.0)

    print("[DEMO] Demo tamamlandı. Keepalive devam ediyor. Ctrl+C ile çık.")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
