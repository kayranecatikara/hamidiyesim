#!/usr/bin/env bash
# tools/temiz_kampanya.sh — HER KOL İÇİN TAZE SİM (ilk-uçuş) kampanyası.
# İrtifa-kayması confound'unu ortadan kaldırır: her kol kendi taze siminde
# İLK uçuş olarak koşar → hepsi aynı düşük irtifadan başlar, kıyaslanabilir.
#
# Her kol: harmonic stop → start → gcs → sağlık-bekle → bayrak set → 1 uçuş.
# Kullanım: bash tools/temiz_kampanya.sh <KAYIT_KOK> <SURE> <TUR>
set -u
API=http://127.0.0.1:8000
KOK="${1:?}"; SURE="${2:-60}"; TUR="${3:-1}"; SEN=duz
REPO="$(cd "$(dirname "$0")/.." && pwd)"; cd "$REPO" || exit 1
SB="$KOK"

# kol : pn,pred,intercept,vlead,tboost,gainsched,commit
kollar="\
base:1,1,0,0,0,0,0 \
vlead:1,1,0,1,0,0,0 \
tboost:1,1,0,0,1,0,0 \
gainsched:1,1,0,0,0,1,0 \
commit:1,1,0,0,0,0,1"

setflag(){ curl -s -X POST $API/api/ozellikler -H 'Content-Type: application/json' \
  -d "{\"anahtar\":\"$1\",\"deger\":$2}" >/dev/null; }
b(){ [ "$1" = 1 ] && echo true || echo false; }

taze_sim(){   # taze Gazebo+SITL+gcs; sağlık dönene kadar bekle (retry'li)
  bash scripts/start_harmonic.sh stop >/dev/null 2>&1
  sleep 2
  GZ_HEADLESS=1 bash scripts/start_harmonic.sh > "$SB/km_harmonic.log" 2>&1
  source /opt/ros/humble/setup.bash 2>/dev/null; export AVCI_GZ_CAMERA=1
  nohup python3 -m control.gcs_server > "$SB/km_gcs.log" 2>&1 &
  for i in $(seq 1 25); do
    [ "$(curl -s -o /dev/null -w '%{http_code}' $API/api/scenario_status 2>/dev/null)" = "200" ] && break
    sleep 2
  done
  # uçak/iris arm + uçak hız bandı bekle
  for i in $(seq 1 25); do
    spd=$(curl -s $API/api/debug/telem | python3 -c "import sys,json;print(round(json.load(sys.stdin)['telemetry_state']['plane']['speed'],1))" 2>/dev/null)
    python3 -c "exit(0 if 12<=float('${spd:-0}')<=20 else 1)" 2>/dev/null && { echo "  [sim] hazır spd=$spd"; return 0; }
    sleep 3
  done
  echo "  [sim] UYARI uçak bant dışı spd=${spd:-?}"; return 1
}

echo "TEMİZ_KAMPANYA başlıyor $(date +%H:%M:%S) tur=$TUR süre=$SURE (her kol taze sim)"
echo "=================================================================="
for t in $(seq 1 "$TUR"); do
  for k in $kollar; do
    ad="${k%%:*}"; bits="${k#*:}"
    IFS=',' read pn pred icept vlead tb gs cm <<< "$bits"
    echo ">>> TUR$t KOL=$ad — taze sim kuruluyor $(date +%H:%M:%S)"
    taze_sim || echo "  [sim] sağlıksız, kol yine de denenecek"
    setflag pn $(b $pn); setflag pred $(b $pred); setflag intercept $(b $icept)
    setflag vlead $(b $vlead); setflag tboost $(b $tb); setflag gainsched $(b $gs); setflag commit $(b $cm)
    sleep 1
    echo "  bayraklar=$(curl -s $API/api/ozellikler | python3 -c 'import sys,json;print(json.load(sys.stdin)["durum"])' 2>/dev/null)"
    bash tools/ucus_test.sh "km_${ad}_t${t}" "$SURE" "$KOK" "$SB/km_gcs.log" "$SEN" 2>&1 | grep -E "SONUÇ|angajman=" | tail -2
    echo "---"
  done
done
bash scripts/start_harmonic.sh stop >/dev/null 2>&1
echo "=================================================================="
echo "TEMİZ_KAMPANYA bitti $(date +%H:%M:%S)"
