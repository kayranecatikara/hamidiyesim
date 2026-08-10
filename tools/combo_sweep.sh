#!/usr/bin/env bash
# tools/combo_sweep.sh — PN/PRED/INTERCEPT kombinasyonlarını tek tek uçurur.
# gcs ZATEN çalışıyor olmalı (env'siz — özellikler API/panelden runtime kontrol).
# Her combo: API'den bayrakları set et → uçuş harness → en yakın mesafe ölç.
# Combolar arası gcs restart YOK (runtime toggle).
# Kullanım: bash tools/combo_sweep.sh <KAYIT_KOK> <GCS_LOG> <SURE> [senaryo]
set -u
API=http://127.0.0.1:8000
KOK="${1:?}"; GCSLOG="${2:?}"; SURE="${3:-70}"; SEN="${4:-duz}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"; cd "$REPO" || exit 1

# combo adı : pn,pred,intercept
combos="none:0,0,0 pn:1,0,0 pred:0,1,0 pnpred:1,1,0 hepsi:1,1,1"

setflag(){ curl -s -X POST $API/api/ozellikler -H 'Content-Type: application/json' \
  -d "{\"anahtar\":\"$1\",\"deger\":$2}" >/dev/null; }

echo "COMBO_SWEEP başlıyor $(date +%H:%M:%S) senaryo=$SEN süre=$SURE"
echo "=================================================================="
for c in $combos; do
  ad="${c%%:*}"; bits="${c#*:}"
  IFS=',' read pn pred icept <<< "$bits"
  [ "$pn" = 1 ] && setflag pn true || setflag pn false
  [ "$pred" = 1 ] && setflag pred true || setflag pred false
  [ "$icept" = 1 ] && setflag intercept true || setflag intercept false
  sleep 1
  durum=$(curl -s $API/api/ozellikler | python3 -c "import sys,json;print(json.load(sys.stdin)['durum'])" 2>/dev/null)
  echo ">>> COMBO=$ad  bayraklar=$durum"
  bash tools/ucus_test.sh "combo_$ad" "$SURE" "$KOK" "$GCSLOG" "$SEN" 2>&1 | grep -E "SONUÇ|angajman=" | tail -2
  echo "---"
done
echo "=================================================================="
echo "COMBO_SWEEP bitti $(date +%H:%M:%S)"
