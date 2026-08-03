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
PROJ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$PROJ_DIR/logs"
COPTER_PARAMS="$PROJ_DIR/sim/ardupilot_params/avci_copter.parm"
PLANE_PARAMS="$PROJ_DIR/sim/ardupilot_params/avci_plane.parm"
mkdir -p "$LOG_DIR"

# ⚠ Eskiden burada `pkill -f '[s]im_vehicle.py'` vardı ve yanındaki yorum
# "köşeli parantez hilesi kendini öldürmeyi önler" diyordu. YANLIŞ: o hile
# yalnız `ps | grep` için geçerli. `pkill -f` ÇAĞIRAN KABUĞUN komut satırına
# bakar ve `[s]im_vehicle` regex'i orada yazan düz `sim_vehicle`'a uyar.
# Bkz. scripts/start_harmonic.sh — aynı hata orada ölçüldü (2026-08-02).
SITL_DESEN='bin/arducopter|bin/arduplane|sim_vehicle\.py|mavproxy'

_atalar() {
    local p=$$
    while [ -n "$p" ] && [ "$p" -gt 1 ] 2>/dev/null; do
        echo "$p"; p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
    done
}

_hedef_pidler() {
    local korunan; korunan=" $(_atalar | tr '\n' ' ') "
    local pid
    for pid in $(pgrep -f "$SITL_DESEN" 2>/dev/null); do
        case "$korunan" in *" $pid "*) continue ;; esac
        echo "$pid"
    done
}

# 0 = temiz durdu, 1 = hayatta kalan var
stop_all() {
    local pidler sinyal
    for sinyal in TERM TERM KILL; do
        pidler=$(_hedef_pidler)
        [ -z "$pidler" ] && break
        kill -"$sinyal" $pidler 2>/dev/null
        sleep 2
    done
    pidler=$(_hedef_pidler)
    if [ -n "$pidler" ]; then
        echo "[SITL] ✗ ÖLDÜRÜLEMEDİ — hâlâ ayakta:"
        ps -o pid=,etime=,args= -p "$(echo $pidler | tr ' ' ',')" 2>/dev/null | cut -c1-110
        return 1
    fi
    return 0
}

if [ "$1" = "stop" ]; then
    echo "[SITL] Tüm ArduPilot SITL süreçleri durduruluyor..."
    if stop_all; then
        echo "[SITL] ✓ Durduruldu — geriye süreç kalmadı."
        exit 0
    fi
    exit 1
fi

echo "[SITL] Eski süreçler temizleniyor..."
if ! stop_all; then
    echo "[SITL] ✗ Eski süreçler temizlenemedi — YENİSİ BAŞLATILMIYOR."
    exit 1
fi

echo "[SITL] ArduCopter başlatılıyor (instance 0, sysid 5, udp:14541)..."
cd "$ARDUPILOT_DIR" || exit 1
nohup python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f quad \
    -I0 --sysid 5 --no-rebuild \
    --add-param-file="$COPTER_PARAMS" \
    --out udp:127.0.0.1:14541 \
    --out udp:127.0.0.1:14550 \
    --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=25" \
    > "$LOG_DIR/copter_sitl.log" 2>&1 &

echo "[SITL] ArduPlane başlatılıyor (instance 1, sysid 2, udp:14542)..."
nohup python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane \
    -I1 --sysid 2 --no-rebuild \
    --add-param-file="$PLANE_PARAMS" \
    --out udp:127.0.0.1:14542 \
    --out udp:127.0.0.1:14550 \
    --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=25" \
    > "$LOG_DIR/plane_sitl.log" 2>&1 &

echo "[SITL] Araçların açılması bekleniyor (~20s)..."
sleep 20
echo "[SITL] Hazır olmalı. Loglar: $LOG_DIR/{copter,plane}_sitl.log"
echo "[SITL] Kontrol: python3 -m control.demos.run_drone_takeoff  /  python3 -m control.gcs_server"
