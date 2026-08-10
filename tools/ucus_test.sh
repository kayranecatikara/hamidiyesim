#!/usr/bin/env bash
# tools/ucus_test.sh — TEK bir uçuş deneyini tekrarlanabilir biçimde koşar.
# gcs_server + Gazebo + SITL ZATEN çalışıyor olmalı (varyant kodu/env'i çağıran
# tarafından ayarlanır; bu script gcs_server'ı YENİDEN BAŞLATMAZ).
#
# Kullanım: bash tools/ucus_test.sh <LABEL> <SURE_SN> <KAYIT_KOK> <GCS_LOG> [senaryo]
#   LABEL     : deney adı (kayıt dizini + video adı)
#   SURE_SN   : chase süresi (angajman doğrulandıktan sonra)
#   KAYIT_KOK : kare/video kök dizini (scratchpad)
#   GCS_LOG   : çalışan gcs_server'ın stdout log dosyası (VURULDU sayımı için)
#   senaryo   : circle (varsayılan) | square | duz | circle_s ...
#
# Çıktı: <KAYIT_KOK>/<LABEL>/{frames,meta.csv} + <KAYIT_KOK>/<LABEL>.mp4
#        ve SONUÇ satırı (VURULDU, terminal, en yakın mesafe, angajman).
set -u
API=http://127.0.0.1:8000
LABEL="${1:?LABEL gerekli}"; SURE="${2:?SURE gerekli}"; KOK="${3:?KAYIT_KOK}"
GCSLOG="${4:?GCS_LOG}"; SENARYO="${5:-circle}"
DIR="$KOK/$LABEL"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO" || exit 1

echo "[TEST:$LABEL] === başlıyor $(date +%H:%M:%S) senaryo=$SENARYO süre=${SURE}s ==="

# 1) Sağlık
if [ "$(curl -s -o /dev/null -w '%{http_code}' $API/api/scenario_status)" != "200" ]; then
  echo "[TEST:$LABEL] SONUÇ: BASARISIZ (gcs_server API yanıt vermiyor)"; exit 2
fi

# 2) Uçağı uçur (aktif değilse) + hız>12 bekle
act=$(curl -s $API/api/scenario_status | python3 -c "import sys,json;print(json.load(sys.stdin).get('active'))" 2>/dev/null)
if [ "$act" != "True" ]; then
  curl -s -X POST $API/api/command/plane/scenario/$SENARYO >/dev/null; sleep 2
fi
for i in $(seq 1 20); do
  spd=$(curl -s $API/api/debug/telem | python3 -c "import sys,json;print(round(json.load(sys.stdin)['telemetry_state']['plane']['speed'],1))" 2>/dev/null)
  python3 -c "exit(0 if float('${spd:-0}')>12 else 1)" 2>/dev/null && break
  sleep 3
done
echo "[TEST:$LABEL] uçak hız=${spd:-?} m/s"

# 3) Zaman damgası (bu uçuşa ait logları ayırmak için)
T0=$(date +%s)
vur0=$(grep -c "VURULDU" "$GCSLOG" 2>/dev/null; true)

# 4) Kayıt başlat
mkdir -p "$DIR"
python3 tools/ucus_kaydi.py "$DIR" "$((SURE+60))" > "$DIR/kayit.log" 2>&1 &
KPID=$!
sleep 1

# 5) Chase başlat + angajman doğrula (40s içinde yaklaşma/faz değişimi)
curl -s -X POST $API/api/command/iris/start_chase >/dev/null
echo "[TEST:$LABEL] chase başladı, angajman bekleniyor..."
engaged=no
for i in $(seq 1 20); do
  sleep 2
  cs=$(curl -s $API/api/chase_status)
  d=$(echo "$cs" | python3 -c "import sys,json;d=json.load(sys.stdin);print(round(float(d.get('distance') or 999),1))" 2>/dev/null)
  a=$(echo "$cs" | python3 -c "import sys,json;print(json.load(sys.stdin).get('active'))" 2>/dev/null)
  # yaklaşma işareti: mesafe < 40 m ya da faz VISUAL
  fz=$(echo "$cs" | python3 -c "import sys,json;print(json.load(sys.stdin).get('supervisor',{}).get('faz'))" 2>/dev/null)
  if [ "$a" = "True" ] && python3 -c "exit(0 if float('${d:-999}')<40 else 1)" 2>/dev/null; then engaged=yes; break; fi
done
echo "[TEST:$LABEL] angajman=$engaged (mesafe=${d:-?}m faz=${fz:-?})"

# 6) Koş
sleep "$SURE"

# 7) Durdur
curl -s -X POST $API/api/command/iris/stop_chase >/dev/null
kill "$KPID" 2>/dev/null
sleep 1

# 8) Metrikler (grep -c zaten 0 basar; exit 1'i yut, çift-0 olmasın)
vurN=$(grep -c "VURULDU" "$GCSLOG" 2>/dev/null; true)
vur=$(( ${vurN:-0} - ${vur0:-0} ))
term=$(grep -c "TERMİNAL HÜCUM" "$GCSLOG" 2>/dev/null; true); term=${term:-0}
kor=$(grep -c "kör hücum başladı" "$GCSLOG" 2>/dev/null; true); kor=${kor:-0}
# meta.csv'den en yakın mesafe (chase aktifken)
minmes=$(awk -F',' 'NR>1 && $5=="True" && $4+0>0 {if(mn==0||$4<mn)mn=$4} END{printf "%.1f",mn+0}' "$DIR/meta.csv" 2>/dev/null)
kare=$(ls "$DIR/frames/" 2>/dev/null | wc -l)

# 9) Video
python3 tools/mkvideo.py "$DIR" "$KOK/$LABEL.mp4" 8 >/dev/null 2>&1 && vid="$KOK/$LABEL.mp4" || vid="(video yok)"

echo "[TEST:$LABEL] === SONUÇ ==="
echo "[TEST:$LABEL] angajman=$engaged  VURULDU=$vur  terminal=$term  kör_ıska=$kor  en_yakın_meta=${minmes}m  kare=$kare"
echo "[TEST:$LABEL] kayıt=$DIR  video=$vid  başlangıç_epoch=$T0"
