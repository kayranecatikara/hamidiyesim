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
# Artik arayuzu (web GCS) de kendisi baslatir ve tarayiciyi otomatik acar.
# Mission Planner (istege bagli) hala ayri:  bash scripts/start_mission_planner.sh
#
# Kullanım:
#   bash scripts/start_harmonic.sh            # her sey + tarayici otomatik acilir
#   GZ_GUI=1 bash scripts/start_harmonic.sh   # 3B Gazebo penceresi de acilir (Intel GPU, agir)
#   NO_GCS=1 bash scripts/start_harmonic.sh   # arayuzu baslatma (sadece sim)
#   NO_BROWSER=1 bash scripts/start_harmonic.sh   # arayuzu baslat ama tarayici acma
#   bash scripts/start_harmonic.sh stop       # durdur

PROJ="$HOME/projects/avci_sim"
AP="$HOME/ardupilot"
APGZ="$HOME/ardupilot_gazebo"
LOG="$PROJ/logs"; mkdir -p "$LOG"
WORLD="$PROJ/sim/gazebo_harmonic/worlds/avci_harmonic.sdf"

stop_all() {
    for pat in 'cessna_pose_relay' 'model JSON' 'model plane' '[s]im_vehicle' '[m]avproxy' '[g]z sim' '[r]uby.*gz' 'control.gcs_server'; do
        pkill -9 -f "$pat" 2>/dev/null
    done
    sleep 3
}

# Tarayiciyi acabilecek ilk araci bul ve URL'yi ac (arka planda).
open_browser() {
    local url="$1"
    [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && { echo "[HARMONIC] Ekran yok, tarayici acilmiyor: $url"; return; }
    for b in xdg-open firefox google-chrome chromium chromium-browser; do
        if command -v "$b" >/dev/null 2>&1; then
            nohup "$b" "$url" >/dev/null 2>&1 &
            echo "[HARMONIC] Tarayici acildi ($b): $url"
            return
        fi
    done
    echo "[HARMONIC] Tarayici bulunamadi; elle ac: $url"
}

if [ "${1:-}" = "stop" ]; then
    echo "[HARMONIC] Durduruluyor..."; stop_all; echo "[HARMONIC] Durduruldu."; exit 0
fi

echo "[HARMONIC] Eski süreçler temizleniyor..."; stop_all

# Ortam — Harmonic plugin + model yolları + NVIDIA render
source /opt/ros/humble/setup.bash 2>/dev/null
# MAVProxy pip --user ile ~/.local/bin'e kurulu; sim_vehicle.py onu PATH'te arar.
# Bu satır olmadan "No such file or directory: 'mavproxy.py'" -> SITL çöker.
export PATH="$HOME/.local/bin:$PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$APGZ/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$PROJ/sim/gazebo_harmonic/models:$APGZ/models:$APGZ/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# 1) Gazebo Harmonic
# Varsayılan: HEADLESS render (kamera web arayüzüne akar, 3B pencere gerekmez).
# NVIDIA surucusu yuklu degil; render Intel GPU ile yapiliyor. 3B pencere de
# istersen GZ_GUI=1 ile acilir (Intel'de calisir ama agirdir, akis takilabilir).
if [ "${GZ_GUI:-0}" = "1" ]; then
    echo "[HARMONIC] Gazebo (GUI) başlatılıyor..."
    export DISPLAY="${DISPLAY:-:0}"
    nohup gz sim -r -v2 "$WORLD" > "$LOG/gz_harmonic.log" 2>&1 &
else
    echo "[HARMONIC] Gazebo (headless render) başlatılıyor..."
    nohup gz sim -s -r --headless-rendering -v2 "$WORLD" > "$LOG/gz_harmonic.log" 2>&1 &
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

# 4) Arayuz (web GCS) + kamera — arka planda baslat, sonra tarayiciyi ac.
URL="http://localhost:8000/ui/index.html"
if [ "${NO_GCS:-0}" = "1" ]; then
    echo "[HARMONIC] NO_GCS=1 -> arayuz baslatilmadi."
else
    echo "[HARMONIC] Arayuz (web GCS) başlatılıyor..."
    ( cd "$PROJ" && AVCI_GZ_CAMERA=1 nohup python3 -m control.gcs_server > "$LOG/gcs_server.log" 2>&1 & )

    echo "[HARMONIC] Web sunucusu (port 8000) bekleniyor..."
    up=0
    for i in $(seq 1 40); do
        ss -ltn 2>/dev/null | grep -q ':8000' && { up=1; break; }
        sleep 0.5
    done
    if [ "$up" = "1" ]; then
        echo "[HARMONIC] Arayuz hazir: $URL"
        [ "${NO_BROWSER:-0}" = "1" ] || open_browser "$URL"
    else
        echo "[HARMONIC] UYARI: port 8000 acilmadi, gcs_server.log'a bak."
    fi
fi

echo "=================================================================="
echo "[HARMONIC] Tam sistem hazır."
echo "  Arayuz : $URL"
echo "  Loglar : $LOG/{gz_harmonic,copter_harmonic,plane_harmonic,gcs_server}.log"
echo "  Mission Planner (istege bagli): bash ~/projects/avci_sim/scripts/start_mission_planner.sh"
echo "  Durdurmak icin: bash ~/projects/avci_sim/scripts/start_harmonic.sh stop"
echo "=================================================================="
