#!/usr/bin/env bash
# Her aday govdeyi bosuna bir Harmonic dunyasinda headless acar,
# yuklenme hatalarini ve gercek zaman faktorunu (RTF) olcer.
# Sonuclar: docs/bench_raw/<aday>.log  +  docs/bench_raw/<aday>.json
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/docs/bench_raw"
TMP="${TMPDIR:-/tmp}/interceptor_bench"
ITERATIONS="${ITERATIONS:-3000}"   # 1ms adim -> 3 sn sim zamani

mkdir -p "$OUT" "$TMP"

# Kendi modellerimiz ONCE gelir (donusturulmus bagimliliklar Classic
# orijinallerini golgelesin), ardindan kaynak repolarin mesh/materyal dizinleri.
export GZ_SIM_RESOURCE_PATH="\
$ROOT/models/interceptors:\
$ROOT/models/net_launchers:\
$ROOT/models/targets:\
$ROOT/repos/mrs_uav_gazebo_simulator/models:\
$ROOT/repos/d2dtracker_sim/models:\
$ROOT/repos/iq_sim/models:\
${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/kubra/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

CANDIDATES=()
for d in "$ROOT"/models/interceptors/cand_*/; do
  CANDIDATES+=("$(basename "$d")")
done

echo "Kiyas tezgahi: ${#CANDIDATES[@]} aday, $ITERATIONS iterasyon (${ITERATIONS}ms sim zamani)"
echo

for cand in "${CANDIDATES[@]}"; do
  world="$TMP/bench_${cand}.sdf"
  sed "s/@CANDIDATE@/${cand}/g" "$ROOT/worlds/bench.sdf.in" > "$world"

  log="$OUT/${cand}.log"
  printf "  %-20s " "$cand"

  start=$(date +%s.%N)
  timeout 120 gz sim -s -r --headless-rendering -v 3 \
      --iterations "$ITERATIONS" "$world" > "$log" 2>&1
  rc=$?
  end=$(date +%s.%N)

  wall=$(python3 -c "print(f'{$end - $start:.2f}')")
  simtime=$(python3 -c "print(f'{$ITERATIONS/1000:.2f}')")
  rtf=$(python3 -c "print(f'{($ITERATIONS/1000)/max($end - $start, 1e-9):.3f}')")

  # Hata siniflandirmasi ("|| echo 0" tek satirlik sayi bozar, tr ile temizliyoruz)
  errors=$(grep -ciE "^\[?Err|Error Code|Unable to find|Failed to load|Exception" "$log" 2>/dev/null | tr -d '\n')
  errors=${errors:-0}
  # JSON'a gomulecek: ANSI kacislari, satir sonlari ve tirnaklar temizlenir
  sanitize() { sed 's/\x1b\[[0-9;]*m//g' | tr '\n\r"' '   ' | tr -s ' '; }
  missing=$(grep -oE "Unable to find uri\[model://[^]]+\]" "$log" 2>/dev/null | sort -u | sanitize)
  plugin_fail=$(grep -oE "Failed to load system plugin [^:]*|Unable to find shared library [^ ]*" "$log" 2>/dev/null | sort -u | head -3 | sanitize)

  if [[ $rc -eq 124 ]]; then
    status="ZAMAN_ASIMI"
  elif [[ $rc -ne 0 ]]; then
    status="CIKIS_$rc"
  elif grep -q "Failed to load a world" "$log"; then
    status="YUKLENMEDI"
  elif [[ ${errors:-0} -gt 0 ]]; then
    status="HATALI"
  else
    status="YUKLENDI"
  fi

  # Dunya yuklenmediyse simulasyon hic donmemistir; RTF anlamsiz olur.
  if [[ "$status" == "YUKLENMEDI" || "$status" == "CIKIS_"* ]]; then
    rtf="null"
  fi

  cat > "$OUT/${cand}.json" <<EOF
{
  "aday": "$cand",
  "durum": "$status",
  "cikis_kodu": $rc,
  "hata_sayisi": $errors,
  "eksik_modeller": "$missing",
  "eklenti_hatalari": "$plugin_fail",
  "duvar_saati_sn": $wall,
  "sim_zamani_sn": $simtime,
  "rtf": $rtf
}
EOF

  printf "%-12s  RTF=%-6s  hata=%-3s %s\n" "$status" "$rtf" "$errors" "${missing:-}"
done

echo
echo "Ham loglar: $OUT/"
echo "Rapor uretmek icin: python3 $ROOT/scripts/13_report.py"
