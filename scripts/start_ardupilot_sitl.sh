#!/bin/bash
# =============================================================
#  AVCI SIM — ArduPilot SITL başlatıcı (PX4'ün yerini alır)
# =============================================================
# İki araç başlatır:
#   ArduCopter (avcı drone): instance 0, sysid 5, out udp:14541 (+14550 varsayılan)
#   ArduPlane (hedef uçak):  instance 1, sysid 2, out udp:14542 + udp:14550
#
# Port haritası (PX4 dönemiyle uyumlu tutuldu):
#   14541 — drone kontrol/telemetri (drone_functions, chase, strike)
#   14542 — plane kontrol/telemetri (plane_functions, patterns)
#   14550 — gcs_server broadcast (özel web GCS; udpin ile bind eder)
#   14551 — Mission Planner broadcast (MP bu portu UDP "listen" ile açar)
#   Her iki araç da 14550 VE 14551'e yollar → iki GCS de ikisini birden görür.
#   NOT: gcs_server ve Mission Planner AYNI portu paylaşamaz (UDP tek bind);
#   bu yüzden her birine ayrı port verildi.
#
# Kullanım:  bash scripts/start_ardupilot_sitl.sh
# Durdurma: bash scripts/start_ardupilot_sitl.sh stop

ARDUPILOT_DIR="$HOME/ardupilot"
PROJ_DIR="$HOME/projects/avci_sim"
LOG_DIR="$PROJ_DIR/logs"
COPTER_PARAMS="$PROJ_DIR/sim/ardupilot_params/avci_copter.parm"
PLANE_PARAMS="$PROJ_DIR/sim/ardupilot_params/avci_plane.parm"
mkdir -p "$LOG_DIR"

stop_all() {
    # [x] köşeli parantez hilesi: pattern kendi komut satırıyla eşleşmesin
    pkill -f 'bin/ardu[c]opter'
    pkill -f 'bin/ardu[p]lane'
    pkill -f '[s]im_vehicle.py'
    pkill -f '[m]avproxy'
    sleep 2
}

if [ "$1" = "stop" ]; then
    echo "[SITL] Tüm ArduPilot SITL süreçleri durduruluyor..."
    stop_all
    echo "[SITL] Durduruldu."
    exit 0
fi

echo "[SITL] Eski süreçler temizleniyor..."
stop_all

echo "[SITL] ArduCopter başlatılıyor (instance 0, sysid 5, udp:14541)..."
cd "$ARDUPILOT_DIR" || exit 1
nohup python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f quad \
    -I0 --sysid 5 --no-rebuild \
    --add-param-file="$COPTER_PARAMS" \
    --out udp:127.0.0.1:14541 \
    --out udp:127.0.0.1:14550 \
    --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=10" \
    > "$LOG_DIR/copter_sitl.log" 2>&1 &

echo "[SITL] ArduPlane başlatılıyor (instance 1, sysid 2, udp:14542)..."
nohup python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane \
    -I1 --sysid 2 --no-rebuild \
    --add-param-file="$PLANE_PARAMS" \
    --out udp:127.0.0.1:14542 \
    --out udp:127.0.0.1:14550 \
    --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=10" \
    > "$LOG_DIR/plane_sitl.log" 2>&1 &

echo "[SITL] Araçların açılması bekleniyor (~20s)..."
sleep 20
echo "[SITL] Hazır olmalı. Loglar: $LOG_DIR/{copter,plane}_sitl.log"
echo "[SITL] Kontrol: python3 -m control.run_drone_takeoff  /  python -m control.gcs_server"
