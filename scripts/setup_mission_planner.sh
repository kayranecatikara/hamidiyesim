#!/bin/bash
# =============================================================
#  Mission Planner indirici / kurulum yardımcısı
# =============================================================
# MissionPlanner-latest.zip'i indirir ve tools/mission_planner altına açar.
# (mono runtime ayrıca kurulmalıdır — bkz. start_mission_planner.sh)

MP_DIR="$HOME/projects/avci_sim/tools/mission_planner"
URL="https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip"
mkdir -p "$MP_DIR"
cd "$MP_DIR" || exit 1

if [ -f "MissionPlanner.exe" ]; then
    echo "[MP] Zaten kurulu: $MP_DIR/MissionPlanner.exe"
    exit 0
fi

echo "[MP] İndiriliyor: $URL"
wget -q --show-progress -O MissionPlanner-latest.zip "$URL" || { echo "İndirme başarısız"; exit 1; }
echo "[MP] Açılıyor..."
unzip -q -o MissionPlanner-latest.zip -d .
[ -f "MissionPlanner.exe" ] && echo "[MP] Kurulum tamam: $MP_DIR/MissionPlanner.exe" || echo "[MP] HATA: exe bulunamadı"
