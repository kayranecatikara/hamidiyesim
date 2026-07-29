#!/usr/bin/env bash
# Uctan uca dogrulama: sunucuyu ac, tareti nisanla, agi at, menzili olc, kapat.
# Kullanim:  ./40_verify.sh [pan_derece] [tilt_derece] [cikis_hizi]
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/env.sh"

PAN="${1:-0}"
TILT="${2:--10}"
HIZ="${3:-18}"
LOG="${TMPDIR:-/tmp}/interceptor_verify.log"

cleanup() {
  [[ -n "${SIM_PID:-}" ]] && kill "$SIM_PID" 2>/dev/null
  wait "$SIM_PID" 2>/dev/null
}
trap cleanup EXIT

echo "=== 1) Gazebo sunucusu baslatiliyor ==="
gz sim -s -r --headless-rendering -v 3 "$ROOT/worlds/$IR_WORLD.sdf" > "$LOG" 2>&1 &
SIM_PID=$!

for _ in $(seq 30); do
  if gz topic -l 2>/dev/null | grep -q "/world/$IR_WORLD/stats"; then break; fi
  sleep 0.5
done
if ! gz topic -l 2>/dev/null | grep -q "/world/$IR_WORLD/stats"; then
  echo "HATA: sunucu ayaga kalkmadi. Log: $LOG"; tail -20 "$LOG"; exit 1
fi
echo "  sunucu hazir (pid $SIM_PID)"

echo
echo "=== 2) Taret nisanlama: pan ${PAN}, tilt ${TILT} derece ==="
python3 "$ROOT/scripts/turret_aim.py" "$PAN" "$TILT"
sleep 5
echo "  --- olculen ---"
python3 "$ROOT/scripts/turret_state.py" "$IR_WORLD"

echo
echo "=== 3) Ag atisi (cikis hizi hedefi ${HIZ} m/s) ==="
# Yorunge izleyici ve yakalama bekleyici ATESTEN ONCE baslar
python3 "$ROOT/scripts/net_track.py" --dunya "$IR_WORLD" --sure 8 \
        --csv "$ROOT/docs/bench_raw/net_trajectory.csv" &
TRACK_PID=$!
CAP_OUT="${TMPDIR:-/tmp}/verify_capture.txt"
python3 "$ROOT/scripts/wait_capture.py" --sure 25 > "$CAP_OUT" 2>&1 &
CAP_PID=$!
sleep 1

python3 "$ROOT/scripts/fire_net.py" --hiz "$HIZ"
wait $TRACK_PID

echo
echo "=== 4) Yakalama ==="
# Log DEGIL topic yoklanir: gz sim'in dosyaya yazdigi stdout blok tamponlu,
# kosum sirasinda bos gorunuyor ve yakalama olmadigi saniliyordu.
if wait $CAP_PID; then
  echo "  $(cat "$CAP_OUT")"
else
  echo "  $(cat "$CAP_OUT")"
  echo "  not: net_test dunyasinda interceptor YERDE; ag alcak irtifadan"
  echo "       atilip sekiyor, isabet kaotik. Deterministik yakalama testi:"
  echo "       scripts/42_capture_test.sh"
fi

echo
echo "Sunucu logu: $LOG"
