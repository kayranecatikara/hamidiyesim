#!/usr/bin/env bash
# tools/ab_vlead.sh — base(pn+pred) vs vlead(pn+pred+vlead) DÖNÜŞÜMLÜ A/B.
# Standout bulguyu (vlead 6.2→1.0m) doğrular. gcs+sim ZATEN çalışıyor.
# Her uçuş öncesi geçerlilik kapısı (uçak bant dışıysa duz yenile+bekle).
# Kullanım: bash tools/ab_vlead.sh <KAYIT_KOK> <GCS_LOG> <SURE> [tur]
set -u
API=http://127.0.0.1:8000
KOK="${1:?}"; GCSLOG="${2:?}"; SURE="${3:-60}"; TUR="${4:-2}"; SEN=duz
REPO="$(cd "$(dirname "$0")/.." && pwd)"; cd "$REPO" || exit 1

setflag(){ curl -s -X POST $API/api/ozellikler -H 'Content-Type: application/json' \
  -d "{\"anahtar\":\"$1\",\"deger\":$2}" >/dev/null; }
plane_spd(){ curl -s $API/api/debug/telem | python3 -c \
  "import sys,json;print(round(json.load(sys.stdin)['telemetry_state']['plane']['speed'],1))" 2>/dev/null; }
kapi(){
  local spd; spd=$(plane_spd)
  python3 -c "exit(0 if 12<=float('${spd:-0}')<=20 else 1)" 2>/dev/null && { echo "  [kapı] OK spd=$spd"; return 0; }
  echo "  [kapı] bant dışı spd=$spd → duz yenile"; curl -s -X POST $API/api/command/plane/scenario/$SEN >/dev/null; sleep 3
  for i in $(seq 1 15); do spd=$(plane_spd); python3 -c "exit(0 if 12<=float('${spd:-0}')<=20 else 1)" 2>/dev/null && { echo "  [kapı] oturdu spd=$spd"; return 0; }; sleep 3; done
  echo "  [kapı] UYARI oturmadı spd=$spd"; return 1
}
kol(){ # $1=ad $2=vlead(true/false)
  setflag pn true; setflag pred true; setflag intercept false
  setflag vlead "$2"; setflag tboost false; setflag gainsched false; setflag commit false
  sleep 1
  echo ">>> $1  vlead=$2"
  kapi
  bash tools/ucus_test.sh "$1" "$SURE" "$KOK" "$GCSLOG" "$SEN" 2>&1 | grep -E "SONUÇ|angajman=" | tail -2
  echo "---"
}

echo "AB_VLEAD başlıyor $(date +%H:%M:%S) tur=$TUR süre=$SURE"
echo "=================================================================="
for t in $(seq 1 "$TUR"); do
  kol "ab_base_$t"  false
  kol "ab_vlead_$t" true
done
echo "=================================================================="
echo "AB_VLEAD bitti $(date +%H:%M:%S)"
