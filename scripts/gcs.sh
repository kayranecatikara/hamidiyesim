#!/usr/bin/env bash
# scripts/gcs.sh — GCS'i (Terminal B) tek komutla başlatır.
#
# Görsel güdüm YALNIZ detection kutusuyla çalışır; eski A0-A4 adımlarındaki
# keypoint değişkenleri yoktur (bkz. POSEA_GERI_DONMEK_ISTERSENIZ/README.md).
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
  *) echo "kullanım: bash scripts/gcs.sh [bbox|gt|takip]  (bkz. docs/SIMULASYON_CALISTIRMA.md)" >&2
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

# ── UÇUŞ BEKÇİSİ (tools/ucus_bekci.py) ──────────────────────────────────────
# Sağlık bandı dışına SÜREKLİ çıkan durumu canlı yakalar (hedef 12 m'ye alçaldı /
# araç yerin altına savruldu gibi hatalar eskiden ancak SONRADAN fark ediliyordu).
# Burada başlatılır çünkü bekçi panelin HTTP API'sini okur — start_harmonic.sh
# çalışırken panel henüz yok.
#
# Salt GÖZLEM: ihlalde tek satır basar ve kendi çıkar; gcs_server'a dokunmaz,
# uçuşu kesmez. Kararı okuyan verir (o koşunun verisi geçersiz sayılır).
# Kapatmak için: AVCI_BEKCI=0
BEKCI_PID=""
if [ "${AVCI_BEKCI:-1}" != "0" ]; then
  # Panel açılana kadar bekçinin ilk denemeleri boşa düşer; kendi 10 s API
  # toleransı var, ayrıca kalkış payı (2. argüman) bunu zaten kapsıyor.
  python3 tools/ucus_bekci.py 86400 60 2>&1 | sed 's/^/[BEKCI] /' &
  BEKCI_PID=$!
  echo "[gcs.sh] uçuş bekçisi AÇIK (kapatmak için AVCI_BEKCI=0)"
fi
# exec KULLANILMIYOR: bekçiyi temizleyebilmek için kabuk hayatta kalmalı.
trap '[ -n "$BEKCI_PID" ] && kill "$BEKCI_PID" 2>/dev/null' EXIT INT TERM
python3 -m control.gcs_server
