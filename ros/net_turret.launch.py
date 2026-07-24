#!/usr/bin/env python3
"""Ağ + taret mekanizması için ROS 2 launch dosyası.

Başlatır:
  1) ros_gz_bridge parameter_bridge  (ros/net_turret_bridge.yaml ile)
     -> ROS 2 <-> Gazebo topic köprüsü
  2) net_turret_node                 (control/net_turret_node.py)
     -> üst seviye ROS yönetim düğümü (aim_deg, fire servisi, capture log)

ÖN KOŞUL: Gazebo + world zaten çalışıyor olmalı
          (bash scripts/start_harmonic.sh).

Çalıştırma:
  cd ~/projects/avcisim_eklentimli
  source /opt/ros/humble/setup.bash
  source /opt/ros/humble/share/ros_gz/... yok; ros_gz_bridge apt ile gelir
  ros2 launch ros/net_turret.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Bu dosyanın (ros/) bir üstü = proje kökü
_ROS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_ROOT = os.path.dirname(_ROS_DIR)
_BRIDGE_YAML = os.path.join(_ROS_DIR, "net_turret_bridge.yaml")


def generate_launch_description():
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="net_turret_bridge",
        parameters=[{"config_file": _BRIDGE_YAML}],
        output="screen",
    )

    # Yönetim düğümü: paket olmadığı için python modülü olarak çalıştırılır.
    manager = ExecuteProcess(
        cmd=["python3", "-m", "control.net_turret_node"],
        cwd=_PROJ_ROOT,
        output="screen",
    )

    return LaunchDescription([bridge, manager])
