#!/usr/bin/env python3
"""Avcı drone'un namlusundan ağı atar — ROS 2 üzerinden.

Ağ atma mekanizması TAMAMEN ROS 2 ile yönetilir. Bu araç ROS 2 topic'ine
(std_msgs/Float64) yayın yapar; ros_gz_bridge bunu Gazebo'daki gz.msgs.Double
/avci/net/fire topic'ine köprüler ve NetLauncherPlugin ağı atar.

ÖN KOŞUL: köprü ayakta olmalı ->  ros2 launch ros/net_turret.launch.py

Kullanım:
    python3 scripts/fire_net.py               # varsayılan 20 m/s
    python3 scripts/fire_net.py --hiz 25
    # eşdeğer saf ROS komutu:
    ros2 topic pub -1 /avci/net/fire std_msgs/msg/Float64 "{data: 20.0}"
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

FIRE_TOPIC = "/avci/net/fire"


def main() -> int:
    ap = argparse.ArgumentParser(description="Ağ fırlatma (ROS 2)")
    ap.add_argument("--hiz", type=float, default=20.0,
                    help="namlu çıkış hızı (m/s). 20 m/s -> ~22-26 m menzil.")
    ap.add_argument("--topic", default=FIRE_TOPIC)
    args = ap.parse_args()

    rclpy.init()
    node = Node("fire_net_cli")
    pub = node.create_publisher(Float64, args.topic, 10)

    # Abone (bridge) bağlanana kadar kısa bekle
    for _ in range(50):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    if pub.get_subscription_count() == 0:
        node.get_logger().warn(
            f"{args.topic} üzerinde abone yok — köprü çalışıyor mu? "
            "(ros2 launch ros/net_turret.launch.py). Yine de yayınlanıyor.")

    pub.publish(Float64(data=args.hiz))
    # mesajın gitmesi için birkaç tur
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"ATEŞ -> {args.topic}  (çıkış hızı {args.hiz} m/s)")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
