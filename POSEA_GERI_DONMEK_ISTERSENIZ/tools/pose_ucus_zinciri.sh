#!/usr/bin/env bash
# =====================================================================
#  pose_ucus_zinciri.sh — Veri çekimi bitince eğitimi otomatik başlatır.
#
#  Yeni veri setinin farkı: krop penceresi UÇUŞTAKİ GİBİ detection
#  kutusundan alınıyor (eskiden geometry'nin gerçek kutusundan alınıyordu).
#  Ölçüm: model val'de 2.2 px hata yaparken uçuşta 14-28 px yapıyordu;
#  tek fark krop penceresinin kaynağıydı.
#
#  Piksel sıralaması artık capture içinde yapılıyor — ayrı dönüştürme adımı
#  gerekmez.
# =====================================================================
set -u
AVCI=$HOME/projects/avci_sim
cd "$AVCI" || exit 1
VERI="$AVCI/vision/datasets/talon_pose_ucus"

log() { echo "[$(date '+%H:%M:%S')] $*"; }

log "veri çekiminin bitmesi bekleniyor..."
while pgrep -f '[c]apture_pose_dataset' >/dev/null; do sleep 30; done

N=$(find "$VERI/images" -name '*.jpg' 2>/dev/null | wc -l)
log "çekim bitti — $N örnek"
if [ "$N" -lt 5000 ]; then
  log "✗ örnek sayısı çok az, eğitim başlatılmıyor"
  exit 1
fi

# Gazebo'yu kapat: eğitim GPU'yu tek başına kullansın
pkill -9 -f '[g]z sim' 2>/dev/null
sleep 5

log "eğitim başlıyor (100 epoch)"
python3 - <<'PY'
from ultralytics import YOLO
KOK = "/home/zeylo/projects/avci_sim"
m = YOLO("yolo11n-pose.pt")
m.train(
    data=f"{KOK}/vision/datasets/talon_pose_ucus/dataset.yaml",
    epochs=100, imgsz=192, batch=32, device=0,
    project=f"{KOK}/vision/runs_pose", name="pose_ucus", exist_ok=True,
    cache="ram", workers=4, patience=20,
    pose=12.0, kobj=1.0, degrees=0.0, scale=0.5,
    flipud=0.0, fliplr=0.5, mosaic=1.0, plots=True, verbose=True,
)
PY

W="$AVCI/vision/runs_pose/pose_ucus/weights/best.pt"
if [ -f "$W" ]; then
  cp -f "$AVCI/vision/models/avci_pose.pt" "$AVCI/vision/models/avci_pose_ONCEKI2.pt" 2>/dev/null
  cp -f "$W" "$AVCI/vision/models/avci_pose_ucus.pt"
  log "✓ model kuruldu: vision/models/avci_pose_ucus.pt"
  python3 tools/pose_kanat_olc.py "$AVCI/vision/models/avci_pose_ucus.pt" "$VERI" 2>&1 \
    | grep -vE "Warning|warnings.warn|Ultralytics"
else
  log "✗ ağırlık oluşmadı"
fi
log "zincir bitti"
