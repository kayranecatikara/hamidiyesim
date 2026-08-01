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

# Depo kökü, script'in kendi konumundan türetilir — böylece depo başka bir
# dizine klonlandığında da kendi world/model/param/log dosyalarını kullanır.
# (Eskiden $HOME/projects/avci_sim sabit yazılıydı; başka bir yoldan
#  çalıştırıldığında script yine o dizinden okumaya çalışıyordu.)
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ardupilot / ardupilot_gazebo depoları makineden makineye farklı yerde:
# ekipte $HOME/ardupilot, bu makinede $HOME/Masaüstü/ardupilot. Sabit yazılırsa
# `cd "$AP" && ...` sessizce başarısız olur, `&&` zinciri kopar ve SITL HİÇ
# başlamaz — script yine de "başlatılıyor" yazıp normal biter, arıza yalnız
# MAVLink portunun hiç açılmamasıyla belli olur (2026-08-01'de tam bunu yedik).
# AVCI_AP_DIR / AVCI_APGZ_DIR ile elle de verilebilir.
_bul() {   # $1: env değeri, kalanlar: aday yollar → ilk var olanı yaz
    local elle="$1"; shift
    if [ -n "$elle" ]; then echo "$elle"; return; fi
    for aday in "$@"; do [ -d "$aday" ] && { echo "$aday"; return; }; done
    echo "$1"                       # bulunamadı → ilk aday (hata mesajı için)
}
AP="$(_bul "${AVCI_AP_DIR:-}" "$HOME/ardupilot" "$HOME/Masaüstü/ardupilot" "$HOME/Desktop/ardupilot")"
APGZ="$(_bul "${AVCI_APGZ_DIR:-}" "$HOME/ardupilot_gazebo" "$HOME/Masaüstü/ardupilot_gazebo" "$HOME/Desktop/ardupilot_gazebo")"
APT="$AP/Tools/autotest"

if [ ! -d "$AP" ]; then
    echo "[HARMONIC] HATA: ardupilot deposu bulunamadı (bakılan: $AP)."
    echo "[HARMONIC] Doğru yolu ver:  AVCI_AP_DIR=/yol/ardupilot bash scripts/start_harmonic.sh"
    exit 1
fi
if [ ! -d "$APGZ/build" ]; then
    echo "[HARMONIC] UYARI: $APGZ/build yok — Gazebo ArduPilotPlugin'i bulamayabilir."
    echo "[HARMONIC]         (AVCI_APGZ_DIR ile yol verilebilir.)"
fi
echo "[HARMONIC] ardupilot=$AP"
echo "[HARMONIC] ardupilot_gazebo=$APGZ"

LOG="$PROJ/logs"; mkdir -p "$LOG"
WORLD="$PROJ/sim/gazebo_harmonic/worlds/avci_harmonic.sdf"

stop_all() {
    for pat in 'model JSON' 'model plane' '[s]im_vehicle' '[m]avproxy' '[g]z sim' '[r]uby.*gz'; do
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

# NVIDIA PRIME render offload (Optimus dizüstü: Intel iGPU + GTX).
# Bu değişkenler olmadan `prime-select on-demand` modunda Gazebo Intel iGPU'da
# render eder (log'da "libEGL: failed to create dri2 screen", nvidia-smi'de %0).
# ÖNEMLİ: kamera sensörünü render eden `gz sim server`'dır ve YOLO'yu o besler —
# iGPU'da kare hızı düşer, görsel fazın tüm zamanlaması değişir.
# Kapatmak için: GZ_NVIDIA=0 bash scripts/start_harmonic.sh
if [ "${GZ_NVIDIA:-1}" = "1" ] && command -v nvidia-smi >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __VK_LAYER_NV_optimus=NVIDIA_only
    NV_EGL_JSON=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
    [ -f "$NV_EGL_JSON" ] && export __EGL_VENDOR_LIBRARY_FILENAMES="$NV_EGL_JSON"
    echo "[HARMONIC] NVIDIA PRIME offload açık ($(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null))"
fi

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
#
# İlk iki --add-param-file ZORUNLU. Güncel ArduPilot'ta sim_vehicle.py artık
# SITL'e --defaults göndermiyor; SITL frame varsayılanlarını gömülü
# vehicleinfo.json'dan --model anahtarına göre çözüyor. Burada --model JSON
# verildiği için arama anahtarı "JSON" oluyor ve o anahtarın frame varsayılanı
# yok (-f gazebo-iris yalnızca sim_vehicle.py tarafını ilgilendiriyor). Bu iki
# dosya olmadan FRAME_CLASS/FRAME_TYPE tanımsız kalıyor:
#   AP: Frame: UNSUPPORTED
#   AP: PreArm: Motors: Check frame class and type
# ve iris motorlarını yapılandıramadığı için NAV_TAKEOFF başarısız oluyor —
# kovalama görevi hiç başlamıyor. (copter.parm ayrıca INS_ACCOFFS/INS_ACCSCAL
# kalibrasyon işaretlerini ve MOT_THST_HOVER'ı da getiriyor.)
# SIRA ÖNEMLİ: avci_copter.parm en sonda kalmalı ki proje değerleri
# (ANGLE_MAX, WPNAV_SPEED, FS_*) üstte kalsın.
echo "[HARMONIC] ArduCopter (gazebo-iris --model JSON) başlatılıyor..."
( cd "$AP" && nohup python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    -I0 --sysid 5 --no-rebuild \
    --add-param-file="$APT/default_params/copter.parm" \
    --add-param-file="$APT/default_params/gazebo-iris.parm" \
    --add-param-file="$PROJ/sim/ardupilot_params/avci_copter.parm" \
    --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=25" > "$LOG/copter_harmonic.log" 2>&1 & )

# 3) ArduPlane SITL (Gazebo mini_talon_vtail — GERÇEK uçuş, fdm 9012)
#    Talon artık Gazebo'da ArduPilotPlugin ile uçuyor; relay YOK.
echo "[HARMONIC] ArduPlane (gazebo mini_talon --model JSON:9012) başlatılıyor..."
( cd "$AP" && nohup python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane \
    --model JSON:127.0.0.1:9012 \
    -I1 --sysid 2 --no-rebuild --add-param-file="$PROJ/sim/ardupilot_params/avci_plane.parm" \
    --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=25" > "$LOG/plane_harmonic.log" 2>&1 & )

# SITL'lerin gerçekten ayağa kalktığını BEKLE ve DOĞRULA. Eskiden burada kör bir
# `sleep 25` vardı; SITL hiç başlamasa bile script "hazır" diyordu.
#
# NOT: 14541/14542 portlarını BURADA aramak yanlıştır. MAVProxy `--out udp:...`
# ile GİDEN soket açar (efemeral yerel port); o portları DİNLEYEN taraf
# gcs_server'dır. Dolayısıyla ölçüt süreçler + SITL banner'ıdır.
echo "[HARMONIC] SITL'ler bekleniyor..."
for i in $(seq 1 60); do
    pgrep -f 'bin/arducopter' >/dev/null \
        && pgrep -f 'bin/arduplane' >/dev/null \
        && grep -q 'AP: Frame:' "$LOG/copter_harmonic.log" 2>/dev/null \
        && grep -q 'ArduPlane V' "$LOG/plane_harmonic.log" 2>/dev/null && break
    sleep 1
done

_eksik=""
pgrep -f 'bin/arducopter' >/dev/null || _eksik="$_eksik arducopter"
pgrep -f 'bin/arduplane'  >/dev/null || _eksik="$_eksik arduplane"
pgrep -f 'mavproxy'       >/dev/null || _eksik="$_eksik mavproxy"
if [ -n "$_eksik" ]; then
    echo "[HARMONIC] HATA: şu süreçler ayağa kalkmadı →$_eksik"
    echo "[HARMONIC] Bak: $LOG/copter_harmonic.log ve $LOG/plane_harmonic.log"
    echo "[HARMONIC] gcs_server bu haliyle araca BAĞLANAMAZ."
else
    _frame="$(grep -m1 'AP: Frame:' "$LOG/copter_harmonic.log" 2>/dev/null)"
    echo "[HARMONIC] ✓ iki SITL de ayakta — ${_frame:-'AP: Frame: (satır yok!)'}"
    case "$_frame" in
        *UNSUPPORTED*) echo "[HARMONIC] UYARI: frame UNSUPPORTED — iris kalkamaz "
                       echo "[HARMONIC]         (--add-param-file sırasını kontrol et)";;
    esac
fi

# (Relay kaldırıldı — Talon Gazebo'da gerçekten uçtuğu için gerek yok.
#  gcs_server hedefi 14542'den kontrol eder: /api/command/plane/square)

echo "=================================================================="
echo "[HARMONIC] Tam sistem hazır."
echo "  Loglar: $LOG/{gz_harmonic,copter_harmonic,plane_harmonic,harmonic_relay}.log"
echo "  Şimdi AYRI terminallerde:"
echo "    cd $PROJ; source /opt/ros/humble/setup.bash; export AVCI_GZ_CAMERA=1; python3 -m control.gcs_server"
echo "      (AVCI_GZ_CAMERA=1 ŞART — yoksa ROS2 moduna düşer, kamera görüntüsü GELMEZ)"
echo "      (ayırıcı ';' — '&&' kullanma: zincirdeki ilk sıfır-dışı çıkış her şeyi sessizce keser)"
echo "    bash $PROJ/scripts/start_mission_planner.sh"
echo "=================================================================="
