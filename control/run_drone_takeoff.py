#!/usr/bin/env python3
"""
run_drone_takeoff.py — Drone kalkış + hover demo scripti.

Kullanım:
    cd ~/projects/avci_sim
    python -m control.run_drone_takeoff

Ön koşullar:
    - Simülasyon çalışıyor olmalı
    - iris portu 14540 üzerinde erişilebilir olmalı
"""

import time
import signal
import sys

# Projenin kök dizininden çalıştığımızdan emin ol
sys.path.insert(0, "/home/kayra/projects/avci_sim")

from control.drone_functions import (
    connect_drone,
    takeoff_to_z,
    hover,
    print_status,
)


def shutdown(sig, frame):
    print("\n[DEMO] Durduruldu (Ctrl+C)")
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)


def main():
    print("=" * 50)
    print("[DEMO] Drone Takeoff + Hover")
    print("=" * 50)

    # 1. Bağlan
    connect_drone()

    # 2. Kalkış
    success = takeoff_to_z(target_z=-2.0)
    if not success:
        print("[DEMO] Kalkış başarısız!")
        return

    # 3. Hover
    print("[DEMO] 10 saniye hover...")
    hover(duration=10.0, target_z=-2.0)

    # 4. Durum yazdır
    print_status()

    print("[DEMO] Demo tamamlandı. Ctrl+C ile çık.")
    # Sürekli setpoint göndermeye devam et
    try:
        while True:
            hover(duration=5.0, target_z=-2.0)
    except KeyboardInterrupt:
        print("\n[DEMO] Durduruldu")


if __name__ == "__main__":
    main()
