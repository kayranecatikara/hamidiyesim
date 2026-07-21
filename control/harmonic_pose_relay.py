#!/usr/bin/env python3
"""
harmonic_pose_relay.py — ArduPlane telemetrisini Gazebo Harmonic'teki hedef
uçağa (mini_talon_target) aktarır.

Gazebo Classic sürümünün (cessna_pose_relay.py) Harmonic karşılığı. Fark:
gazebo_ros /set_entity_state yerine gz-transport /world/<world>/set_pose service.

ArduPlane (built-in SITL) kendi fiziğinde uçar; Gazebo'da görünmez. Bu node,
LOCAL_POSITION_NED + ATTITUDE telemetrisini okuyup hedef modeli gz set_pose ile
konumlandırır → avcı drone'un (gazebo-iris) kamerası hareketli hedefi görür.

Koordinat dönüşümü — ArduPilot NED → Gazebo ENU:
    Gazebo x (East)  =  NED y (East)   (+ ayrım ofseti)
    Gazebo y (North) =  NED x (North)
    Gazebo z (Up)    = -NED z (Down)
    Gazebo yaw (ENU) =  pi/2 - NED yaw

Kullanım:
    source /opt/ros/humble/setup.bash   # (gz python yollari icin gerekmez ama zararsiz)
    python3 -m control.harmonic_pose_relay [world_name] [port]
"""

import math
import sys
import time

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from pymavlink import mavutil

WORLD = sys.argv[1] if len(sys.argv) > 1 else "avci"
RELAY_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 14552
MODEL_NAME = "mini_talon_target"
UPDATE_HZ = 30.0

# iris ve ArduPlane aynı SITL home'unda başladığından yerde üst üste binmesini
# önlemek için hedefi Gazebo'da yatayda bu kadar ayır (görsel).
VISUAL_OFFSET_EAST_M = 25.0
VISUAL_OFFSET_NORTH_M = 0.0


def main():
    node = Node()
    service = f"/world/{WORLD}/set_pose"
    print(f"[RELAY] gz set_pose service: {service}", flush=True)
    print(f"[RELAY] ArduPlane telemetri dinleniyor (udpin:{RELAY_PORT})...", flush=True)
    conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{RELAY_PORT}", source_system=239)

    yaw = 0.0
    got = False
    count = 0
    period = 1.0 / UPDATE_HZ

    while True:
        t0 = time.monotonic()

        # Kuyruktaki güncel mesajları çek (bayat veri birikmesin)
        pos = None
        for _ in range(20):
            m = conn.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
            if m is None:
                break
            if m.get_type() == "ATTITUDE":
                yaw = m.yaw
            else:
                pos = m

        if pos is not None:
            # NED -> Gazebo ENU (+ ayrım ofseti)
            gx = pos.y + VISUAL_OFFSET_EAST_M
            gy = pos.x + VISUAL_OFFSET_NORTH_M
            gz = max(-pos.z, 0.2)
            gyaw = math.pi / 2.0 - yaw

            req = Pose()
            req.name = MODEL_NAME
            req.position.x = float(gx)
            req.position.y = float(gy)
            req.position.z = float(gz)
            req.orientation.z = math.sin(gyaw / 2.0)
            req.orientation.w = math.cos(gyaw / 2.0)

            try:
                ok, _ = node.request(service, req, Pose, Boolean, 300)
            except Exception as e:
                ok = False
                if count % 60 == 0:
                    print(f"[RELAY] set_pose hatası: {e}", flush=True)

            count += 1
            if not got and ok:
                got = True
                print(f"[RELAY] İlk pozisyon aktarıldı: Gazebo=({gx:.1f},{gy:.1f},{gz:.1f})", flush=True)
            elif count % (int(UPDATE_HZ) * 5) == 0:
                print(f"[RELAY] Hedef Gazebo konumu: ({gx:.1f},{gy:.1f},{gz:.1f}) "
                      f"yaw={math.degrees(gyaw):.0f}°", flush=True)

        elapsed = time.monotonic() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
