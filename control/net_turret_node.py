#!/usr/bin/env python3
"""Ağ atma + taret mekanizmasının merkezi ROS 2 yönetim düğümü.

Mekanizma TAMAMEN ROS 2 üzerinden yönetilir. Bu düğüm üst-seviye ROS arayüzü
sunar; alt seviyedeki Gazebo (gz-transport) topic'leri `ros_gz_bridge` ile
köprülenir (bkz. ros/net_turret_bridge.yaml). Yani:

    (kullanıcı / görev kodu)
        │  ROS 2
        ▼
    net_turret_node ──▶ /avci/turret/yaw_cmd   (Float64, rad) ─┐
                    ──▶ /avci/turret/pitch_cmd (Float64, rad) ─┤ ros_gz_bridge
                    ──▶ /avci/net/fire         (Float64, m/s) ─┘   │
                    ◀── /avci/net/captured     (String)  ◀─────────┘
                                                              (Gazebo)

ROS ARAYÜZÜ
-----------
Abone olunan (girişler):
  /avci/turret/aim_deg   geometry_msgs/Vector3   x=pan°, y=tilt°  (nişan al)
  /avci/net/fire_speed   std_msgs/Float64        çıkış hızı m/s   (ateşle)

Servis:
  ~/fire (std_srvs/Trigger)   varsayılan hızla ağı at

Yayınlanan (çıkışlar, bridge'e gider):
  /avci/turret/yaw_cmd    std_msgs/Float64  (rad)
  /avci/turret/pitch_cmd  std_msgs/Float64  (rad)
  /avci/net/fire          std_msgs/Float64  (m/s)

Geri bildirim:
  /avci/net/captured      std_msgs/String   (bridge'ten) -> loglanır

Çalıştırma:
  ros2 run ... yerine paket olmadığı için doğrudan:
    python3 -m control.net_turret_node
  (önce köprü ayakta olmalı: ros2 launch ros/net_turret.launch.py)
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger

# Taret eklem sınırları (iris_cam/model.sdf ile aynı)
PAN_MIN_DEG, PAN_MAX_DEG = 0.0, 360.0
TILT_MIN_DEG, TILT_MAX_DEG = 0.0, 180.0   # 0=ileri (paralel), 90=yukarı, 180=geri


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class NetTurretNode(Node):
    def __init__(self):
        super().__init__("net_turret_node")

        self.declare_parameter("default_fire_speed", 20.0)

        # --- Çıkışlar (ros_gz_bridge üzerinden Gazebo'ya) ---
        self.yaw_pub = self.create_publisher(Float64, "/avci/turret/yaw_cmd", 10)
        self.pitch_pub = self.create_publisher(Float64, "/avci/turret/pitch_cmd", 10)
        self.fire_pub = self.create_publisher(Float64, "/avci/net/fire", 10)

        # --- Üst seviye ROS girişleri ---
        self.create_subscription(Vector3, "/avci/turret/aim_deg", self.on_aim, 10)
        self.create_subscription(Float64, "/avci/net/fire_speed", self.on_fire_speed, 10)

        # --- Geri bildirim ---
        self.create_subscription(String, "/avci/net/captured", self.on_captured, 10)

        # --- Servis: ateşle ---
        self.create_service(Trigger, "~/fire", self.srv_fire)

        self.captured_model = None
        self.get_logger().info(
            "net_turret_node hazır | aim: /avci/turret/aim_deg (Vector3 x=pan°,y=tilt°) "
            "| ateş: /avci/net/fire_speed (Float64) veya servis ~/fire")

    # ---------------------------------------------------------------
    def aim_deg(self, pan_deg: float, tilt_deg: float):
        """Taret açılarını derece cinsinden alır, kırpar, rad olarak yayınlar."""
        p = clamp(pan_deg, PAN_MIN_DEG, PAN_MAX_DEG)
        t = clamp(tilt_deg, TILT_MIN_DEG, TILT_MAX_DEG)
        if (p, t) != (pan_deg, tilt_deg):
            self.get_logger().warn(
                f"nişan sınır dışı, kırpıldı: pan {pan_deg:+.1f}->{p:+.1f}, "
                f"tilt {tilt_deg:+.1f}->{t:+.1f}")
        self.yaw_pub.publish(Float64(data=math.radians(p)))
        self.pitch_pub.publish(Float64(data=math.radians(t)))
        self.get_logger().info(f"nişan: pan {p:+.1f}°, tilt {t:+.1f}°")

    def fire(self, speed: float):
        """Ağı verilen çıkış hızıyla atar."""
        self.fire_pub.publish(Float64(data=float(speed)))
        self.get_logger().info(f"ATEŞ: {speed:.1f} m/s")

    # ---------------------------------------------------------------
    def on_aim(self, msg: Vector3):
        self.aim_deg(msg.x, msg.y)

    def on_fire_speed(self, msg: Float64):
        self.fire(msg.data)

    def on_captured(self, msg: String):
        self.captured_model = msg.data
        self.get_logger().info(f"YAKALANDI: '{msg.data}' ağa kilitlendi")

    def srv_fire(self, request, response):
        speed = self.get_parameter("default_fire_speed").value
        self.fire(speed)
        response.success = True
        response.message = f"ağ atıldı ({speed:.1f} m/s)"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NetTurretNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
