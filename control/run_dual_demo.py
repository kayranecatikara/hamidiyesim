#!/usr/bin/env python3
"""
run_dual_demo.py — İki araçlı eş zamanlı demo.

Bu script drone ve plane'i paralel thread'lerde çalıştırır.
Her araç kendi bağımsız görevini yapar.

Kullanım:
    cd ~/projects/avci_sim
    python -m control.run_dual_demo

Ön koşullar:
    - Simülasyon çalışıyor olmalı
    - iris portu 14540, plane portu 14541 üzerinde erişilebilir olmalı
"""

import time
import signal
import sys
import threading

sys.path.insert(0, "/home/kayra/projects/avci_sim")


def drone_task():
    """Drone görevi: kalkış + hover + basit hareket."""
    from control.drone_functions import (
        connect_drone,
        takeoff_to_z,
        hover,
        move_forward,
        move_right,
        print_status,
    )

    print("[DUAL-DRONE] Drone görevi başlıyor")
    connect_drone()

    success = takeoff_to_z(target_z=-2.0)
    if not success:
        print("[DUAL-DRONE] Kalkış başarısız!")
        return

    hover(duration=3.0)
    move_forward(distance=2.0)
    hover(duration=3.0)
    move_right(distance=2.0)
    hover(duration=3.0)

    print_status()
    print("[DUAL-DRONE] Drone görevi tamamlandı")

    # Hover loop — sürekli setpoint
    try:
        while True:
            hover(duration=5.0)
    except Exception:
        pass


def plane_task():
    """Plane görevi: keepalive + arm + kare deseni."""
    from control.plane_functions import (
        connect_plane,
        arm_plane,
    )
    from control.plane_patterns import (
        takeoff_then_stabilize,
        draw_square,
        loiter,
    )

    print("[DUAL-PLANE] Plane görevi başlıyor")
    connect_plane()

    result = arm_plane(warmup_duration=3.0)
    if result is None or result[1] != 0:
        print("[DUAL-PLANE] ARM başarısız!")
        return

    takeoff_then_stabilize()
    draw_square()
    loiter(duration=5.0)

    print("[DUAL-PLANE] Plane görevi tamamlandı")

    # Keepalive devam eder (daemon thread olduğu için)
    while True:
        time.sleep(1)


def shutdown(sig, frame):
    print("\n[DUAL] Durduruldu (Ctrl+C)")
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)


def main():
    print("=" * 50)
    print("[DUAL] İki Araçlı Demo")
    print("=" * 50)

    t_drone = threading.Thread(target=drone_task, daemon=True, name="drone")
    t_plane = threading.Thread(target=plane_task, daemon=True, name="plane")

    t_drone.start()
    t_plane.start()

    print("[DUAL] Her iki thread başlatıldı. Ctrl+C ile çık.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[DUAL] Durduruldu")


if __name__ == "__main__":
    main()
