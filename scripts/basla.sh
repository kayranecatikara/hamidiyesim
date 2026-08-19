#!/usr/bin/env bash
# basla.sh — TEK KOMUTLA sim (HEADLESS, ekran GEREKTİRMEZ) + GCS.
#   Bu makinede mkur.sh GUI Gazebo'yu :1 ekranında açıp çöküyordu; bu script
#   headless çalışır (Xvfb gerekmez). Önce çalışan her şeyi kapatır (çift-sim
#   çakışmasını önler), sonra Gazebo + ArduCopter + ArduPlane + GCS başlatır.
#
# Kullanım:
#   bash scripts/basla.sh                    # başlat
#   bash scripts/start_harmonic.sh stop      # kapat
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

echo "[1/3] eski sim kapatılıyor..."
bash "$REPO/scripts/start_harmonic.sh" stop >/dev/null 2>&1
sleep 1

# Gazebo kamera render'ını GPU'ya zorla (NVIDIA EGL). Yoksa Mesa/DRI'ye
# düşüp CPU'da (yazılım) render ediyor ("failed to create dri2 screen").
# glvnd'yi yalnız NVIDIA EGL vendor'ına kilitler → offscreen render NVIDIA GPU'da.
if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
fi

echo "[2/3] sim başlatılıyor (headless, ~60-90 s)..."
GZ_HEADLESS=1 bash "$REPO/scripts/start_harmonic.sh" > "$REPO/logs/basla_sim.log" 2>&1
if ! grep -q "Tam sistem hazır" "$REPO/logs/basla_sim.log"; then
    echo "  ✗ sim hazır olmadı — bak: tail -40 $REPO/logs/basla_sim.log"
    exit 1
fi
echo "  ✓ sim hazır (Gazebo + Copter + Plane)"

echo "[3/3] GCS başlatılıyor..."
setsid nohup bash -c '
    cd "'"$REPO"'"
    source /opt/ros/humble/setup.bash 2>/dev/null
    export AVCI_GZ_CAMERA=1 AVCI_GORSEL=on
    exec python3 -m control.gcs_server
' > "$REPO/logs/gcs.log" 2>&1 < /dev/null & disown

for i in $(seq 1 40); do
    sleep 2
    curl -sf -m 3 http://127.0.0.1:8000/api/guidance_mode >/dev/null 2>&1 && break
done
if ! curl -sf -m 3 http://127.0.0.1:8000/api/guidance_mode >/dev/null 2>&1; then
    echo "  ✗ GCS yanıt vermiyor — bak: tail -40 $REPO/logs/gcs.log"
    exit 1
fi

echo "════════════════════════════════════════════════"
echo "  ✓ HAZIR → http://localhost:8000"
echo "  Kapat:  bash scripts/start_harmonic.sh stop"
echo "════════════════════════════════════════════════"
