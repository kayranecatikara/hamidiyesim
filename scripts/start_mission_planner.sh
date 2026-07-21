#!/bin/bash
# =============================================================
#  Mission Planner başlatıcı (QGroundControl'ün yerini alır)
# =============================================================
# Mission Planner bir GUI uygulamasıdır ve mono runtime gerektirir.
# ArduPilot SITL'e UDP 14551 portundan bağlanır (gcs_server 14550'yi kullanır,
# çakışmayı önlemek için MP'ye ayrı port verildi).
#
# Kullanım:  bash scripts/start_mission_planner.sh

MP_DIR="$HOME/projects/avci_sim/tools/mission_planner"

if ! command -v mono >/dev/null 2>&1; then
    echo "=================================================================="
    echo " HATA: mono runtime kurulu değil."
    echo " Mission Planner Linux'ta mono ile çalışır. Önce şunu çalıştırın:"
    echo ""
    echo "   sudo apt-get update"
    echo "   sudo apt-get install -y mono-complete libgdiplus"
    echo ""
    echo " (Bu oturumda '! sudo apt-get install -y mono-complete libgdiplus'"
    echo "  yazarak da çalıştırabilirsiniz.)"
    echo "=================================================================="
    exit 1
fi

if [ ! -f "$MP_DIR/MissionPlanner.exe" ]; then
    echo "HATA: $MP_DIR/MissionPlanner.exe bulunamadı."
    echo "Önce indirin:  scripts/setup_mission_planner.sh"
    exit 1
fi

echo "[MP] Mission Planner başlatılıyor (mono)..."
echo "[MP] Bağlantı için: sağ üstte UDP seç → port 14551 → Connect"
echo "[MP] Her iki araç (Copter sysid 5, Plane sysid 2) üst menüden seçilebilir."
cd "$MP_DIR" || exit 1

# Ubuntu 22.04'te mono, versiyon-suz libdl.so'yu bulamıyor (libdl artık libc'ye
# entegre). Yerel bir symlink ile çözülür — aksi halde native çağrılar (seri
# port vb.) DllNotFoundException verir.
export LD_LIBRARY_PATH="$MP_DIR/native_libs:$LD_LIBRARY_PATH"
export DISPLAY="${DISPLAY:-:1}"
export MONO_LOG_LEVEL="${MONO_LOG_LEVEL:-warning}"

exec mono MissionPlanner.exe
