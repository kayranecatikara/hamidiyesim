#!/usr/bin/env bash
# Ag balistigi: farkli cikis hizi / namlu acisi kombinasyonlarinda menzil olcer.
# Her atis icin sunucu yeniden baslatilir (ag baslangic konumuna donsun).
#
# Atis YONU artik disaridan verilmiyor - NetLauncherPlugin namlunun gercek
# yonelimini okuyor. Aci taramasi icin ag askisi egiliyor (tilt_world.py).
#
# Kullanim:  ./41_range_test.sh
#            HIZLAR="12 18 24" ACILAR="-10 -20" ./41_range_test.sh
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/env.sh"

WORLD="net_ballistics"
HIZLAR="${HIZLAR:-15 20 25}"
ACILAR="${ACILAR:-0 -10 -20}"
OUT="$ROOT/docs/bench_raw/menzil_taramasi.csv"
LOG="${TMPDIR:-/tmp}/net_range.log"
TMP="${TMPDIR:-/tmp}"

mkdir -p "$(dirname "$OUT")"
echo "cikis_hizi_ms,tilt_derece,menzil_m,ucus_suresi_s,tepe_z_m,olculen_v0_ms" > "$OUT"

printf "%-10s %-8s %-12s %-10s %-10s %s\n" \
       "hiz(m/s)" "tilt" "menzil(m)" "sure(s)" "tepe z(m)" "olculen v0"
printf -- "---------------------------------------------------------------------\n"

for hiz in $HIZLAR; do
  for aci in $ACILAR; do
    wfile="$TMP/${WORLD}_a${aci}.sdf"
    if ! python3 "$ROOT/scripts/tilt_world.py" "$ROOT/worlds/$WORLD.sdf" "$wfile" "$aci"; then
      echo "  [ATLA] hiz=$hiz aci=$aci : dunya uretilemedi"; continue
    fi

    gz sim -s -r --headless-rendering -v 1 "$wfile" > "$LOG" 2>&1 &
    pid=$!

    for _ in $(seq 30); do
      gz topic -l 2>/dev/null | grep -q "/world/$WORLD/stats" && break
      sleep 0.5
    done

    trace="$ROOT/docs/bench_raw/traj_h${hiz}_a${aci}.csv"
    python3 "$ROOT/scripts/net_track.py" --dunya "$WORLD" --sure 6 --csv "$trace" \
        > /dev/null 2>&1 &
    tpid=$!
    sleep 1

    python3 "$ROOT/scripts/fire_net.py" --hiz "$hiz" > /dev/null 2>&1

    wait $tpid
    kill $pid 2>/dev/null; wait $pid 2>/dev/null

    read -r menzil sure tepe v0 < <(python3 "$ROOT/scripts/traj_stats.py" "$trace")
    printf "%-10s %-8s %-12s %-10s %-10s %s\n" "$hiz" "$aci" "$menzil" "$sure" "$tepe" "$v0"
    echo "$hiz,$aci,$menzil,$sure,$tepe,$v0" >> "$OUT"
  done
done

echo
echo "Sonuclar: $OUT"
