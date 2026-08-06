#!/usr/bin/env bash
# scripts/gcs.sh — GCS'i (Terminal B) tek komutla başlatır.
#
# 2026-08-06: POSE MODELİ KALDIRILDI. Görsel güdüm artık YALNIZ detection
# kutusuyla çalışıyor (bkz. POSEA_GERI_DONMEK_ISTERSENIZ/README.md), o yüzden
# eski A0-A4 adımlarındaki AVCI_POSE değişkeni yok.
#
# MODLAR:
#   bash scripts/gcs.sh bbox   GERÇEK SİSTEM: güdüm YOLO kutusundan (varsayılan)
#   bash scripts/gcs.sh gt     TEŞHİS: güdüm girdisi Gazebo GERÇEK pozundan
#   bash scripts/gcs.sh takip  bbox + HybridSORT takip + kilitli-ID politikası
#
# Argümansız çalıştırmak `bbox` demektir.
# Ortak: ROS ortamı + kamera değişkenleri + 8000 portu temizliği.

set -u
cd "$(dirname "$0")/.." || exit 1

ADIM="${1:-bbox}"

# Değişkenler:
#   AVCI_GT_ROT          güdüm girdisi Gazebo gerçek pozundan (SİMÜLASYONA ÖZGÜ)
#   AVCI_GT_KILIT_BYPASS GPS→görsel geçişte görsel kilit kapısı atlansın mı
#                        (ölçümle çürütüldü — açmayın, bkz. supervisor.SupCfg)
#   AVCI_TRACKER         HybridSORT kareler arası takip
#   AVCI_LOCK            kilitli-ID hedef politikası (tracker'a bağlı)
case "$ADIM" in
  bbox)  export AVCI_GT_ROT=off AVCI_GT_KILIT_BYPASS=off AVCI_TRACKER=off AVCI_LOCK=off ;;
  gt)    export AVCI_GT_ROT=on  AVCI_GT_KILIT_BYPASS=off AVCI_TRACKER=off AVCI_LOCK=off ;;
  takip) export AVCI_GT_ROT=off AVCI_GT_KILIT_BYPASS=off AVCI_TRACKER=on  AVCI_LOCK=on  ;;
  *) echo "kullanım: bash scripts/gcs.sh [bbox|gt|takip]  (bkz. DENEY.md)" >&2
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

echo "[gcs.sh] MOD=$ADIM  ALGI=$([ "$AVCI_GT_ROT" = on ] && echo 'GT (gerçek poz)' || echo 'bbox (YOLO)')" \
     " KILIT_BYPASS=$AVCI_GT_KILIT_BYPASS  TRACKER=$AVCI_TRACKER  LOCK=$AVCI_LOCK"
echo "[gcs.sh] → http://localhost:8000    (uçuş sonrası: python3 tools/gudum_karne.py)"
exec python3 -m control.gcs_server
