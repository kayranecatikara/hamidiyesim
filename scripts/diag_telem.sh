#!/bin/bash
# Telemetri hızı tanısı — restart SONRASI çalıştır.
# 1) Canlı süreçleri gösterir (mavproxy gerçekten streamrate=25 ile açık mı?)
# 2) 14551'den ham MAVLink hızını ölçer.
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "======================================================================"
echo " CANLI SÜREÇLER (mavproxy --streamrate ne? gz/SITL ayakta mı?)"
echo "======================================================================"
echo "--- mavproxy ---"
pgrep -af "mavproxy" || echo "  (mavproxy YOK — SITL sim_vehicle ile başlamamış!)"
echo "--- sim_vehicle ---"
pgrep -af "sim_vehicle" || echo "  (sim_vehicle YOK)"
echo "--- gz sim / gazebo ---"
pgrep -af "gz sim" || echo "  (gz sim YOK — SITL Gazebo FDM'e bağlanamaz!)"
echo
echo "======================================================================"
echo " HAM MAVLINK HIZI (14551, 8 sn)"
echo "======================================================================"
python3 "$PROJ/tools/mav_rate.py" --port 14551 --secs 8
