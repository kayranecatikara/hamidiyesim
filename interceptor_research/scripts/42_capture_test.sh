#!/usr/bin/env bash
# Yakalama testi (Asama 5): agi at, hedefe carptir, NetCapturePlugin'in
# hedefi aga kilitledigini dogrula. N kez tekrarlayip basari oranini basar.
#
# Kullanim:  ./42_capture_test.sh [deneme_sayisi] [cikis_hizi]
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/env.sh"

WORLD="net_capture_test"
N="${1:-5}"
HIZ="${2:-20}"
LOG="${TMPDIR:-/tmp}/net_capture.log"

basari=0
echo "Yakalama testi: $N deneme, cikis hizi $HIZ m/s"
echo

for i in $(seq "$N"); do
  gz sim -s -r --headless-rendering -v 3 "$ROOT/worlds/$WORLD.sdf" > "$LOG" 2>&1 &
  pid=$!

  for _ in $(seq 30); do
    gz topic -l 2>/dev/null | grep -q "/world/$WORLD/stats" && break
    sleep 0.5
  done

  sleep 1
  # Yakalama bekleyicisini ATESTEN ONCE baslat ki olayi kacirmasin
  TMP_OUT="${TMPDIR:-/tmp}/capture_wait_$i.txt"
  python3 "$ROOT/scripts/wait_capture.py" --sure 25 > "$TMP_OUT" 2>&1 &
  wpid=$!
  sleep 0.5

  python3 "$ROOT/scripts/fire_net.py" --hiz "$HIZ" > /dev/null 2>&1

  # Yakalama olayini TOPIC'ten bekle (log tamponlu, guvenilmez - bkz.
  # scripts/wait_capture.py aciklamasi). Izleyici atesten ONCE baslatilmali.
  sonuc="YOK"
  if wait $wpid; then
    sonuc="VAR"
  fi

  kill $pid 2>/dev/null; wait $pid 2>/dev/null

  if [[ "$sonuc" == "VAR" ]]; then
    basari=$((basari+1))
    printf "  deneme %d: %s\n" "$i" "$(cat "$TMP_OUT")"
  else
    printf "  deneme %d: kacirdi\n" "$i"
  fi
done

echo
echo "SONUC: $basari/$N yakalama  ($(python3 -c "print(f'{100*$basari/$N:.0f}')")%)"
echo "Referans: Fortem F700 ilk atis isabeti ~%85 (ticari_referanslar/README.md)"
