#!/bin/bash
# =============================================================
#  AVCI SİM — Gazebo HARMONIC tam sistem başlatıcı
# =============================================================
# Doğru sırayla, tek seferde başlatır (benim otonom test döngümdeki
# çoklu-restart karışıklığını önlemek için — sen bunu tek çalıştırırsın).
#
#   1) Gazebo Harmonic (avci world: iris_cam + mini_talon_vtail hedef)
#   2) ArduCopter SITL (gazebo-iris --model JSON) — iris Harmonic FDM 9002
#   3) ArduPlane SITL (mini_talon --model JSON:9012) — Talon Harmonic FDM 9012
#      Talon Gazebo'da GERÇEKTEN uçar (relay YOK); gcs "kare çiz" ile kontrol.
#
# Ardından AYRI terminallerde:
#   - python3 -m control.gcs_server      (web GCS + gz kamera + chase/strike)
#   - bash scripts/start_mission_planner.sh
#
# Kullanım:
#   bash scripts/start_harmonic.sh            # GUI (NVIDIA render, önerilen)
#   GZ_HEADLESS=1 bash scripts/start_harmonic.sh   # görüntüsüz
#   bash scripts/start_harmonic.sh stop       # durdur

PROJ="$HOME/projects/avci_sim"
AP="$HOME/ardupilot"
APGZ="$HOME/ardupilot_gazebo"
LOG="$PROJ/logs"; mkdir -p "$LOG"
WORLD="$PROJ/sim/gazebo_harmonic/worlds/avci_harmonic.sdf"

stop_all() {
    for pat in 'cessna_pose_relay' 'model JSON' 'model plane' '[s]im_vehicle' '[m]avproxy' '[g]z sim' '[r]uby.*gz'; do
        pkill -9 -f "$pat" 2>/dev/null
    done
    sleep 3
}

if [ "${1:-}" = "stop" ]; then
    echo "[HARMONIC] Durduruluyor..."; stop_all; echo "[HARMONIC] Durduruldu."; exit 0
fi

echo "[HARMONIC] Eski süreçler temizleniyor..."; stop_all

# Ortam — Harmonic plugin + model yolları + NVIDIA render
source /opt/ros/humble/setup.bash 2>/dev/null
export GZ_SIM_SYSTEM_PLUGIN_PATH="$APGZ/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$PROJ/sim/gazebo_harmonic/models:$APGZ/models:$APGZ/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# 1) Gazebo Harmonic
if [ "${GZ_HEADLESS:-0}" = "1" ]; then
    echo "[HARMONIC] Gazebo (headless) başlatılıyor..."
    unset DISPLAY
    nohup gz sim -s -r --headless-rendering -v2 "$WORLD" > "$LOG/gz_harmonic.log" 2>&1 &
else
    echo "[HARMONIC] Gazebo (GUI, NVIDIA render) başlatılıyor..."
    export DISPLAY="${DISPLAY:-:1}"
    nohup gz sim -r -v2 "$WORLD" > "$LOG/gz_harmonic.log" 2>&1 &
fi

echo "[HARMONIC] Gazebo FDM portu (9002) bekleniyor..."
for i in $(seq 1 30); do ss -ln 2>/dev/null | grep -q ':9002' && break; sleep 1; done
sleep 3

# 2) ArduCopter SITL (Harmonic JSON FDM)
echo "[HARMONIC] ArduCopter (gazebo-iris --model JSON) başlatılıyor..."
( cd "$AP" && nohup python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    -I0 --sysid 5 --no-rebuild --add-param-file="$PROJ/sim/ardupilot_params/avci_copter.parm" \
    --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=10" > "$LOG/copter_harmonic.log" 2>&1 & )

# 3) ArduPlane SITL (Gazebo mini_talon_vtail — GERÇEK uçuş, fdm 9012)
#    Talon artık Gazebo'da ArduPilotPlugin ile uçuyor; relay YOK.
echo "[HARMONIC] ArduPlane (gazebo mini_talon --model JSON:9012) başlatılıyor..."
( cd "$AP" && nohup python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane \
    --model JSON:127.0.0.1:9012 \
    -I1 --sysid 2 --no-rebuild --add-param-file="$PROJ/sim/ardupilot_params/avci_plane.parm" \
    --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=10" > "$LOG/plane_harmonic.log" 2>&1 & )

echo "[HARMONIC] SITL'lerin açılması bekleniyor (25s)..."
sleep 25

# (Relay kaldırıldı — Talon Gazebo'da gerçekten uçtuğu için gerek yok.
#  gcs_server hedefi 14542'den kontrol eder: /api/command/plane/square)

echo "=================================================================="
echo "[HARMONIC] Tam sistem hazır."
echo "  Loglar: $LOG/{gz_harmonic,copter_harmonic,plane_harmonic,harmonic_relay}.log"
echo "  Şimdi AYRI terminallerde:"
echo "    cd ~/projects/avci_sim && source /opt/ros/humble/setup.bash && python3 -m control.gcs_server"
echo "    bash ~/projects/avci_sim/scripts/start_mission_planner.sh"
echo "=================================================================="
