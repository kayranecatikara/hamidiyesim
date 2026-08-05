#!/usr/bin/env bash
# scripts/gcs.sh — GCS'i (Terminal B) tek komutla başlatır.
#
# DENEY ADIMLARI (bkz. DENEY.md) — her adım öncekinden TEK DEĞİŞKEN farklı:
#
#   bash scripts/gcs.sh A0    TABAN: GT poz + pose modeli KAPALI + takip KAPALI
#   bash scripts/gcs.sh A1    A0 + pose modeli AÇIK (yalnız ekran/kilit için)
#   bash scripts/gcs.sh A2    A1 + pose kilidi kapısı AÇIK
#   bash scripts/gcs.sh A3    A2 + HybridSORT takip AÇIK
#   bash scripts/gcs.sh A4    A3 + kilitli-ID politikası AÇIK
#   bash scripts/gcs.sh pose  GERÇEK SİSTEM: GT yok, güdüm pose modelinden
#
# Argümansız çalıştırmak A0 demektir.
# Ortak: ROS ortamı + kamera değişkenleri + 8000 portu temizliği.

set -u
cd "$(dirname "$0")/.." || exit 1

ADIM="${1:-A0}"

# Adım tanımları: her satır bir öncekine TEK bir şey ekler.
#   AVCI_GT_ROT          güdüm girdisi Gazebo gerçek pozundan
#   AVCI_POSE            pose modeli yüklensin mi (GT'de güdüme girmez)
#   AVCI_GT_KILIT_BYPASS GPS→görsel geçişte pose kilidi kapısı atlansın mı
#   AVCI_TRACKER         HybridSORT kareler arası takip
#   AVCI_LOCK            kilitli-ID hedef politikası (tracker'a bağlı)
case "$ADIM" in
  A0)   export AVCI_GT_ROT=on  AVCI_POSE=off AVCI_GT_KILIT_BYPASS=on  AVCI_TRACKER=off AVCI_LOCK=off ;;
  A1)   export AVCI_GT_ROT=on  AVCI_POSE=on  AVCI_GT_KILIT_BYPASS=on  AVCI_TRACKER=off AVCI_LOCK=off ;;
  A2)   export AVCI_GT_ROT=on  AVCI_POSE=on  AVCI_GT_KILIT_BYPASS=off AVCI_TRACKER=off AVCI_LOCK=off ;;
  A3)   export AVCI_GT_ROT=on  AVCI_POSE=on  AVCI_GT_KILIT_BYPASS=off AVCI_TRACKER=on  AVCI_LOCK=off ;;
  A4)   export AVCI_GT_ROT=on  AVCI_POSE=on  AVCI_GT_KILIT_BYPASS=off AVCI_TRACKER=on  AVCI_LOCK=on  ;;
  pose) export AVCI_GT_ROT=off AVCI_POSE=on  AVCI_GT_KILIT_BYPASS=off AVCI_TRACKER=off AVCI_LOCK=off ;;
  *) echo "kullanım: bash scripts/gcs.sh [A0|A1|A2|A3|A4|pose]  (bkz. DENEY.md)" >&2
     exit 2 ;;
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

echo "[gcs.sh] ADIM=$ADIM  GT=$AVCI_GT_ROT  POSE=$AVCI_POSE  " \
     "KILIT_BYPASS=$AVCI_GT_KILIT_BYPASS  TRACKER=$AVCI_TRACKER  LOCK=$AVCI_LOCK"
echo "[gcs.sh] → http://localhost:8000    (uçuş sonrası: python3 tools/gudum_karne.py)"
exec python3 -m control.gcs_server
