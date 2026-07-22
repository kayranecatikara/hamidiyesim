#!/bin/bash
# Sadece gcs_server (web GCS backend) yeniden başlatır — SITL/Gazebo'ya dokunmaz.
# Tam daemon: nohup + setsid → kendi session'ında, SIGHUP/grup sinyallerinden bağımsız.
PROJ="/home/aysenur/projects/avci_sim"
cd "$PROJ" || exit 1
pkill -f 'control.gcs_server' 2>/dev/null
sleep 1
AVCI_GZ_CAMERA=1 nohup setsid python3 -m control.gcs_server > "$PROJ/logs/gcs_server.log" 2>&1 < /dev/null &
# başlatıcı hemen çıkar; süreç arka planda kalır
exit 0
