#!/usr/bin/env bash
# scripts/gcs.sh — GCS'i (Terminal B) tek komutla başlatır.
#
#   bash scripts/gcs.sh          # GT rotasyon modu AÇIK (varsayılan)
#   bash scripts/gcs.sh pose     # pose modeli güdümde (eski davranış)
#
# Yaptıkları: ROS ortamı + kamera/tarayıcı değişkenleri + 8000 portunu boşalt.
# Elle export etmeye gerek yok; yeni terminalde de doğru çalışır.

set -u
cd "$(dirname "$0")/.." || exit 1

MOD="${1:-gt}"
case "$MOD" in
  gt)   export AVCI_GT_ROT=on  ;;
  pose) export AVCI_GT_ROT=off ;;
  *) echo "kullanım: bash scripts/gcs.sh [gt|pose]" >&2; exit 2 ;;
esac

# ROS setup.bash tanımsız değişken okuyor (AMENT_TRACE_SETUP_FILES) — `set -u`
# altında patlıyor. Yalnız source süresince kapatılır.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export AVCI_GZ_CAMERA=1        # Harmonic kamerası gz-transport'tan okunur
export AVCI_NO_BROWSER=1       # otomatik tarayıcı açma (MESA takılmasını önler)

fuser -k 8000/tcp 2>/dev/null   # "Address already in use" olmasın
sleep 0.3

echo "[gcs.sh] mod=$MOD  AVCI_GT_ROT=$AVCI_GT_ROT  → http://localhost:8000"
exec python3 -m control.gcs_server
