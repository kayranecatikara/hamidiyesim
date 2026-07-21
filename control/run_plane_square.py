#!/usr/bin/env python3
"""
run_plane_square.py — Plane kare deseni demo scripti.

Kullanım:
    cd ~/projects/avci_sim
    python -m control.run_plane_square

Ön koşullar:
    - Simülasyon çalışıyor olmalı
    - plane portu 14541 üzerinde erişilebilir olmalı

GCS Server'daki /api/plane_throttle endpoint'inden throttle değerini okur.
Frontend'deki slider ile hız ayarlanabilir.
"""

import time
import signal
import sys
import json
import urllib.request

sys.path.insert(0, "/home/kayra/projects/avci_sim")

from control.plane_functions import (
    connect_plane,
    arm_plane,
    stop_gcs_keepalive,
    send_manual_control,
    THROTTLE_CRUISE,
    THROTTLE_FULL,
)
from control.plane_patterns import (
    takeoff_then_stabilize,
    loiter,
)

# GCS'ten throttle değerini oku
def get_throttle_from_gcs():
    """GCS Server'daki throttle API'sinden güncel değeri okur."""
    try:
        req = urllib.request.urlopen("http://127.0.0.1:8000/api/plane_throttle", timeout=0.1)
        data = json.loads(req.read().decode())
        return data.get("throttle", THROTTLE_CRUISE)
    except Exception:
        return THROTTLE_CRUISE  # bağlantı yoksa varsayılan



CONTROL_RATE = 0.05  # 20 Hz


def fly_forward_dynamic(duration: float, abort_event=None):
    """Düz ileri uçuş — her adımda GCS'ten throttle günceller."""
    t0 = time.time()
    while time.time() - t0 < duration:
        if abort_event and abort_event.is_set():
            break
        thr = get_throttle_from_gcs()
        send_manual_control(throttle=thr)
        time.sleep(CONTROL_RATE)


def turn_right_dynamic(intensity: int = 500, duration: float = 2.0, abort_event=None):
    """Sağa dönüş — her adımda GCS'ten throttle günceller."""
    t0 = time.time()
    while time.time() - t0 < duration:
        if abort_event and abort_event.is_set():
            break
        thr = get_throttle_from_gcs()
        send_manual_control(yaw=intensity, throttle=thr)
        time.sleep(CONTROL_RATE)


def draw_square_dynamic(side_duration: float = 3.0, turn_duration: float = 2.0,
                        turn_intensity: int = 500, abort_event=None):
    """Kare desenini sürekli çizer — throttle her iterasyonda GCS'ten okunur."""
    print(f"[PATTERN] Dinamik Kare: side={side_duration}s turn={turn_duration}s")
    for i in range(4):
        if abort_event and abort_event.is_set():
            break
        print(f"[PATTERN] Kenar {i+1}/4")
        fly_forward_dynamic(duration=side_duration, abort_event=abort_event)
        if abort_event and abort_event.is_set():
            break
        print(f"[PATTERN] Dönüş {i+1}/4")
        turn_right_dynamic(intensity=turn_intensity, duration=turn_duration, abort_event=abort_event)
    print("[PATTERN] Kare tamamlandı")


def shutdown(sig, frame):
    print("\n[DEMO] Durduruldu (Ctrl+C)")
    stop_gcs_keepalive()
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    print("=" * 50)
    print("[DEMO] Plane Kare Deseni (Dinamik Throttle)")
    print("=" * 50)

    # 1. Bağlan + ARM
    connect_plane()
    result = arm_plane(warmup_duration=3.0)
    if result is None or result[1] != 0:
        print("[DEMO] ARM başarısız!")
        return

    # 2. Takeoff + Stabilize
    takeoff_then_stabilize()

    # 3. Sürekli kare çiz — throttle GCS slider'dan okunur
    while True:
        draw_square_dynamic(side_duration=3.0, turn_duration=2.0)
        print("[DEMO] Yeni kare başlıyor...")


if __name__ == "__main__":
    main()
