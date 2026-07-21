#!/usr/bin/env python3
"""
cessna_pose_relay.py — ArduPlane telemetrisini Gazebo'daki Cessna modeline aktarır.

ArduPlane (built-in SITL) kendi fiziğinde uçar; Gazebo'da görünmez. Bu node,
ArduPlane'in LOCAL_POSITION_NED + ATTITUDE telemetrisini okuyup Gazebo'daki
'cessna_target' modelinin konum/yönelimini /gazebo/set_entity_state ile günceller.
Böylece avcı drone'un (gazebo-iris) kamerası hareketli Cessna'yı görür.

Koordinat dönüşümü — ArduPilot NED → Gazebo ENU:
    Gazebo x (East)  =  NED y (East)
    Gazebo y (North) =  NED x (North)
    Gazebo z (Up)    = -NED z (Down)
    Gazebo yaw (ENU) =  pi/2 - NED yaw

Not: gazebo-iris copter ve ArduPlane aynı SITL varsayılan home'unu kullandığından
NED origin'leri Gazebo world origin ile hizalıdır.

Kullanım:
    source /opt/ros/humble/setup.bash
    python3 -m control.cessna_pose_relay          # varsayılan port 14552
"""

import math
import sys

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from pymavlink import mavutil

RELAY_PORT = 14552
MODEL_NAME = "cessna_target"
UPDATE_HZ = 30.0

# iris (avcı drone) ve ArduPlane aynı SITL home'unda (0,0) başlar; bu yüzden
# ikisi yerde üst üste görünür. Cessna'yı Gazebo'da yatayda bu kadar ayırırız
# (görsel/başlangıç ayrımı — hedef drone'un yanında durur).
VISUAL_OFFSET_EAST_M = 25.0
VISUAL_OFFSET_NORTH_M = 0.0


class CessnaPoseRelay(Node):
    def __init__(self, port=RELAY_PORT):
        super().__init__("cessna_pose_relay")
        self.cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.get_logger().info("Gazebo /gazebo/set_entity_state bekleniyor...")
        if not self.cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("set_entity_state service yok — Gazebo + "
                                    "gazebo_ros_state plugin çalışıyor mu?")
            raise RuntimeError("service yok")

        self.get_logger().info(f"ArduPlane telemetri dinleniyor (udpin:{port})...")
        self.conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}",
                                               source_system=246)
        self._yaw = 0.0
        self._got_pos = False
        self.timer = self.create_timer(1.0 / UPDATE_HZ, self._tick)
        self._count = 0

    def _tick(self):
        # Kuyruktaki güncel mesajları çek (bayat veri birikmesin)
        pos = None
        for _ in range(20):
            m = self.conn.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
            if m is None:
                break
            if m.get_type() == "ATTITUDE":
                self._yaw = m.yaw
            else:
                pos = m

        if pos is None:
            return

        # NED -> Gazebo ENU (+ başlangıç ayrım ofseti)
        gx = pos.y + VISUAL_OFFSET_EAST_M
        gy = pos.x + VISUAL_OFFSET_NORTH_M
        gz = -pos.z
        gyaw = math.pi / 2.0 - self._yaw

        req = SetEntityState.Request()
        req.state.name = MODEL_NAME
        req.state.reference_frame = "world"
        req.state.pose.position.x = float(gx)
        req.state.pose.position.y = float(gy)
        req.state.pose.position.z = float(max(gz, 0.2))  # yere gömülmesin
        req.state.pose.orientation.z = math.sin(gyaw / 2.0)
        req.state.pose.orientation.w = math.cos(gyaw / 2.0)
        self.cli.call_async(req)

        self._count += 1
        if not self._got_pos:
            self._got_pos = True
            self.get_logger().info(
                f"İlk pozisyon aktarıldı: Gazebo=({gx:.1f},{gy:.1f},{gz:.1f})")
        elif self._count % (int(UPDATE_HZ) * 5) == 0:
            self.get_logger().info(
                f"Cessna Gazebo konumu: ({gx:.1f},{gy:.1f},{gz:.1f}) yaw={math.degrees(gyaw):.0f}°")


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else RELAY_PORT
    rclpy.init()
    node = CessnaPoseRelay(port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
