#!/bin/bash
# ============================================================================
# Gece otonom uçuş test kampanyası sürücüsü (~12 saat).
# YALNIZ env — asıl güdüm/kontrol/config kodu DEĞİŞTİRİLMEZ.
# matris.txt'i okur, her kolu tam-restart ile koşar, arşivler, raporu yeniler.
# setsid ile başlat: logout'a dayanıklı. Bitişte araçları durdurur.
# ============================================================================
set -u
PROJ=/home/aysenur/projects/hamidiyesim
GK=$PROJ/tools/gece_kampanya
OUT=$PROJ/logs/gece_kampanya
MATRIS=$GK/matris.txt
ILERLEME=$OUT/ilerleme.log
BUTCE_SN=${BUTCE_SN:-43200}     # sert zaman bütçesi (12 saat)
MAX_UCUS=${MAX_UCUS:-200}       # uçuş tavanı (backstop)
REC=${REC:-200}                 # kayıt süresi/uçuş
cd "$PROJ" || exit 1
mkdir -p "$OUT"

il() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$ILERLEME"; }

# ---- taban config (kazanan) + opsiyonel ek env ----
gcs_env() {  # $1=vmax $2=cap $3=ekenv("-" ya da "k=v,k=v")
  export AVCI_GZ_CAMERA=1 AVCI_DUZ_BEKLEME=0
  export AVCI_IBVS_PN=on AVCI_IBVS_PRED=on AVCI_IBVS_TBOOST=on AVCI_IBVS_DUZTERM=on
  export AVCI_YAPISKANLIK=on AVCI_KILIT_KAYIP_SN_ON=3.5
  export AVCI_GPS_V_MAX="$1"
  export AVCI_IBVS_CAPRAZLEAD="$2"
  if [ "${3:--}" != "-" ]; then
    local kv
    for kv in ${3//,/ }; do export "$kv"; done
  fi
}

restart_sim() {
  pkill -f 'control[.]gcs_server' 2>/dev/null
  bash scripts/start_harmonic.sh stop > "$OUT/.stop.log" 2>&1
  sleep 2
  GZ_HEADLESS=1 setsid bash scripts/start_harmonic.sh > "$OUT/.boot.log" 2>&1 &
  local i
  for i in $(seq 1 80); do
    grep -q 'Tam sistem hazır' "$OUT/.boot.log" 2>/dev/null && return 0
    sleep 2
  done
  return 1
}

baslat_gcs() {  # $1=vmax $2=cap $3=ekenv
  set +u; source /opt/ros/humble/setup.bash 2>/dev/null; set -u
  gcs_env "$1" "$2" "$3"
  setsid python3 -m control.gcs_server > "$OUT/.gcs.log" 2>&1 &
  local i
  for i in $(seq 1 60); do
    ss -tln 2>/dev/null | grep -q ':8000' && grep -q 'YOLO detector hazır' "$OUT/.gcs.log" 2>/dev/null && return 0
    sleep 1
  done
  return 1
}

arsivle() {  # $1=dir $2=vmax $3=cap $4=kacamak $5=durum $6=sebep $7=ekenv
  local dir="$1" vmax="$2" cap="$3" kac="$4" durum="$5" sebep="$6" ekenv="$7"
  local pat f
  for pat in kilit bbox_ibvs bildirim gps_guidance; do
    f=$(ls -t "$PROJ"/logs/${pat}_*.csv 2>/dev/null | head -1)
    [ -n "$f" ] && cp "$f" "$dir/" 2>/dev/null
  done
  # gcs stdout logu (bu gcs örneğinin [SUPERVISOR] geçiş satırları — titreme metriği)
  cp "$OUT/.gcs.log" "$dir/gcs.log" 2>/dev/null
  [ -d "$dir/frames" ] && python3 tools/mkvideo.py "$dir" "$dir/ucus.mp4" 5 > /dev/null 2>&1
python3 - "$dir" "$vmax" "$cap" "$kac" "$durum" "$sebep" "$ekenv" <<'PY'
import csv, json, sys, statistics, os
d,vmax,cap,kac,durum,sebep,ekenv=sys.argv[1:8]
pm=None
p=os.path.join(d,"kacamak.csv")
if os.path.exists(p):
    pl=[float(r["plane_spd"]) for r in csv.DictReader(open(p))
        if r.get("plane_spd") not in (None,"")]
    pl=[x for x in pl if x>1.0]
    if pl: pm=round(statistics.median(pl),1)
if durum=="OK" and pm is not None and not (6.0<=pm<=25.0):
    durum="GECERSIZ"; sebep=f"plane_med={pm} band dışı"
json.dump(dict(vmax=int(vmax),caprazlead=cap,kacamak=kac,ekenv=ekenv,
               durum=durum,sebep=sebep,plane_med=pm),
          open(os.path.join(d,"meta.json"),"w"), ensure_ascii=False, indent=2)
print(durum)
PY
}

kol_kos() {  # $1=seq $2=blok $3=vmax $4=kacamak $5=cap $6=ekenv
  local seq="$1" blok="$2" vmax="$3" kac="$4" cap="$5" ekenv="${6:--}"
  local tag; tag=$(echo "$ekenv" | tr -cd 'A-Za-z0-9'); tag=${tag: -14}; [ "$ekenv" = "-" ] && tag=def
  local ad; ad=$(printf "%03d_%s_v%s_%s_%s_%s" "$seq" "$blok" "$vmax" "$kac" "$cap" "$tag")
  local dir="$OUT/$ad"
  local deneme durum="" sebep=""
  for deneme in 1 2; do
    rm -rf "$dir"
    if ! restart_sim; then sebep="sim_hazir_olmadi"; continue; fi
    if ! baslat_gcs "$vmax" "$cap" "$ekenv"; then sebep="gcs_kalkmadi"; continue; fi
    timeout $((REC+180)) python3 tools/kacamak_testi.py "$dir" "$kac" 25 "$REC" \
        > "$dir.kos.log" 2>&1
    mkdir -p "$dir"; mv "$dir.kos.log" "$dir/kos.log" 2>/dev/null
    if [ -f "$dir/olay.json" ]; then durum="OK"; sebep=""; break
    else durum="BASARISIZ"; sebep="olay_yok(deneme$deneme)"; fi
  done
  durum=$(arsivle "$dir" "$vmax" "$cap" "$kac" "${durum:-BASARISIZ}" "${sebep:-?}" "$ekenv")
  local eny imha
  eny=$(python3 -c "import json;print(json.load(open('$dir/olay.json')).get('en_yakin'))" 2>/dev/null || echo "-")
  imha=$(python3 -c "import json;print(json.load(open('$dir/olay.json')).get('imha'))" 2>/dev/null || echo "-")
  il "$ad → durum=$durum en_yakın=${eny}m imha=$imha"
  python3 "$GK/analiz.py" >> "$OUT/.analiz.log" 2>&1
}

# ============================ ANA AKIŞ ============================
il "==== GECE KAMPANYA BAŞLADI (bütçe=${BUTCE_SN}s ≈ $((BUTCE_SN/3600))sa, tavan=${MAX_UCUS}) ===="

# Düz/yok duman testi kullanıcı istemiyorsa atlanabilir. Manevralı kampanyada
# ilk matris kolu altyapı doğrulamasını da üstlenir.
if [ "${SKIP_SMOKE:-0}" != "1" ]; then
  il "duman testi (VMAX=24 yok)..."
  kol_kos 0 SMOKE 24 yok off -
  S=$OUT/000_SMOKE_v24_yok_off_def
if [ ! -f "$S/olay.json" ]; then
    il "DUMAN TESTİ BAŞARISIZ — altyapı sorunlu, kampanya durduruluyor."
    bash scripts/start_harmonic.sh stop > /dev/null 2>&1; exit 1
  fi
  if ! python3 -c "import json,sys;sys.exit(0 if json.load(open('$S/olay.json')).get('tetiklendi') else 1)" 2>/dev/null; then
    il "DUMAN TESTİ: tetiklenmedi (hedefe yaklaşamadı) — durduruluyor."
    bash scripts/start_harmonic.sh stop > /dev/null 2>&1; exit 1
  fi
  il "duman testi OK — tam matrise geçiliyor."
fi

# --- MATRİSİ GENİŞLET ve KOŞ ---
seq=1; satir_no=0
while IFS= read -r satir; do
  case "$satir" in ''|'#'*) continue;; esac
  satir_no=$((satir_no+1))
  read -r blok vmax kacamaklar cap tekrar ekenv <<< "$satir"
  ekenv=${ekenv:--}
  IFS=',' read -ra kaclar <<< "$kacamaklar"
  for ((tur=1; tur<=tekrar; tur++)); do
    for kac in "${kaclar[@]}"; do
      if [ "$cap" = "ab" ]; then
        if (( tur % 2 == 1 )); then caps=(off on); else caps=(on off); fi
      else caps=("$cap"); fi
      for c in "${caps[@]}"; do
        if (( SECONDS >= BUTCE_SN )); then il "ZAMAN BÜTÇESİ DOLDU — yeni uçuş yok."; break 4; fi
        if (( seq > MAX_UCUS )); then il "UÇUŞ TAVANI — duruluyor."; break 4; fi
        kol_kos "$seq" "$blok" "$vmax" "$kac" "$c" "$ekenv"
        seq=$((seq+1))
      done
    done
  done
  # ── ARA RAPOR: bu config (matris satırı) bitti → snapshot (kullanıcı isteği) ──
  ara="$OUT/ARA_RAPOR_$(printf '%02d' "$satir_no")_${blok}.md"
  cp "$OUT/RAPOR_KARSILASTIRMA.md" "$ara" 2>/dev/null
  il "── ara rapor yazıldı: $(basename "$ara") (config: $blok $kacamaklar cap=$cap ekenv=$ekenv) ──"
done < "$MATRIS"

il "==== KAMPANYA BİTTİ — $((seq-1)) kol koşuldu. Araçlar durduruluyor. ===="
bash scripts/start_harmonic.sh stop > /dev/null 2>&1
python3 "$GK/analiz.py" genel >> "$OUT/.analiz.log" 2>&1
il "GENEL RAPOR: $OUT/GENEL_RAPOR.md · canlı: $OUT/RAPOR_KARSILASTIRMA.md"
