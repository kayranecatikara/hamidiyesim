#!/usr/bin/env python3
"""Avcı drone'un burnundaki tareti nişanlar — ROS 2 üzerinden.

Taret TAMAMEN ROS 2 ile yönetilir. Bu araç açı komutlarını (radyan) ROS 2
topic'lerine (std_msgs/Float64) yayınlar; ros_gz_bridge bunları Gazebo'daki
gz JointPositionController cmd_pos topic'lerine köprüler.

ÖN KOŞUL: köprü ayakta olmalı ->  ros2 launch ros/net_turret.launch.py

Kullanım:
    python3 scripts/turret_aim.py <pan_derece> <tilt_derece>
    python3 scripts/turret_aim.py 20 -10       # 20° sağa, 10° yukarı
    # eşdeğer saf ROS komutu (radyan):
    ros2 topic pub -1 /avci/turret/yaw_cmd std_msgs/msg/Float64 "{data: 0.349}"

Açı sözleşmesi:
    pan  : 0..360°   (Z ekseni, tam tur)
    tilt : 0..180°   (0=ileri/şase paralel, 90=tam yukarı, 180=geri/şase paralel;
                      asla şasenin altına bakmaz)
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

YAW_TOPIC = "/avci/turret/yaw_cmd"
PITCH_TOPIC = "/avci/turret/pitch_cmd"
PAN_MIN_DEG, PAN_MAX_DEG = 0.0, 360.0
TILT_MIN_DEG, TILT_MAX_DEG = 0.0, 180.0   # 0=ileri (paralel), 90=yukarı, 180=geri


def clamp(val, lo, hi, adi):
    if val < lo or val > hi:
        print(f"  UYARI: {adi} {val:+.1f}° sınır dışı, [{lo}, {hi}] aralığına kırpıldı")
    return max(lo, min(hi, val))


def main() -> int:
    ap = argparse.ArgumentParser(description="Taret nişanlama (ROS 2)")
    ap.add_argument("pan", type=float, help="pan derece")
    ap.add_argument("tilt", type=float, help="tilt derece")
    args = ap.parse_args()

    pan_deg = clamp(args.pan, PAN_MIN_DEG, PAN_MAX_DEG, "pan")
    tilt_deg = clamp(args.tilt, TILT_MIN_DEG, TILT_MAX_DEG, "tilt")

    rclpy.init()
    node = Node("turret_aim_cli")
    yaw_pub = node.create_publisher(Float64, YAW_TOPIC, 10)
    pitch_pub = node.create_publisher(Float64, PITCH_TOPIC, 10)

    # bridge bağlanana kadar bekle
    for _ in range(50):
        if yaw_pub.get_subscription_count() > 0 and pitch_pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Nişan: pan {pan_deg:+.2f}°, tilt {tilt_deg:+.2f}°")
    yaw_pub.publish(Float64(data=math.radians(pan_deg)))
    pitch_pub.publish(Float64(data=math.radians(tilt_deg)))
    print(f"  {YAW_TOPIC}   <- {math.radians(pan_deg):+.4f} rad")
    print(f"  {PITCH_TOPIC} <- {math.radians(tilt_deg):+.4f} rad")

    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
