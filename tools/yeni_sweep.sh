#!/usr/bin/env bash
# tools/yeni_sweep.sh — TEKTE VURUŞ yeni özelliklerini base=pn+pred üzerine
# TEK DEĞİŞKEN A/B ile tarar. gcs+sim ZATEN çalışıyor olmalı (env'siz;
# bayraklar API/panel deposundan runtime set edilir, combolar arası gcs
# restart YOK). Base başta VE sonda koşulur → sim drift'i bracketlenir.
#
# ⚠ GEÇERLİLİK KAPISI (geçen sweep dersi: bozuk simde chase hiç kurulmadı,
# uçak 190m'ye tırmandı, çöp veri): her uçuş öncesi uçak hız bandında (12-20)
# değilse duz senaryosu YENİDEN verilir + seviyeye oturması beklenir. Uçuş
# angajman=no dönerse o kol GEÇERSİZ işaretlenir (bir kez retry).
#
# Kullanım: bash tools/yeni_sweep.sh <KAYIT_KOK> <GCS_LOG> <SURE> [senaryo]
set -u
API=http://127.0.0.1:8000
KOK="${1:?}"; GCSLOG="${2:?}"; SURE="${3:-70}"; SEN="${4:-duz}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"; cd "$REPO" || exit 1

# ad : pn,pred,intercept,vlead,tboost,gainsched,commit
combos="\
base1:1,1,0,0,0,0,0 \
vlead:1,1,0,1,0,0,0 \
tboost:1,1,0,0,1,0,0 \
gainsched:1,1,0,0,0,1,0 \
commit:1,1,0,0,0,0,1 \
allnew:1,1,0,1,1,1,1 \
base2:1,1,0,0,0,0,0"

setflag(){ curl -s -X POST $API/api/ozellikler -H 'Content-Type: application/json' \
  -d "{\"anahtar\":\"$1\",\"deger\":$2}" >/dev/null; }

plane_spd(){ curl -s $API/api/debug/telem | python3 -c \
  "import sys,json;print(round(json.load(sys.stdin)['telemetry_state']['plane']['speed'],1))" 2>/dev/null; }

# GEÇERLİLİK KAPISI: uçak hız bandında değilse senaryo yenile + bekle
saglik_kapisi(){
  local spd; spd=$(plane_spd)
  if python3 -c "exit(0 if 12<=float('${spd:-0}')<=20 else 1)" 2>/dev/null; then
    echo "  [kapı] uçak OK (spd=${spd})"; return 0
  fi
  echo "  [kapı] uçak bant DIŞI (spd=${spd}) → duz yenile, seviyeye bekle"
  curl -s -X POST $API/api/command/plane/scenario/$SEN >/dev/null; sleep 3
  for i in $(seq 1 15); do
    spd=$(plane_spd)
    python3 -c "exit(0 if 12<=float('${spd:-0}')<=20 else 1)" 2>/dev/null && \
      { echo "  [kapı] seviyeye oturdu (spd=${spd})"; return 0; }
    sleep 3
  done
  echo "  [kapı] UYARI: uçak seviyeye oturmadı (spd=${spd}) — kol yine de koşulur, sonuç şüpheli"
  return 1
}

echo "YENİ_SWEEP başlıyor $(date +%H:%M:%S) senaryo=$SEN süre=$SURE (base=pn+pred)"
echo "=================================================================="
for c in $combos; do
  ad="${c%%:*}"; bits="${c#*:}"
  IFS=',' read pn pred icept vlead tboost gsched commit <<< "$bits"
  setflag pn $([ "$pn" = 1 ] && echo true || echo false)
  setflag pred $([ "$pred" = 1 ] && echo true || echo false)
  setflag intercept $([ "$icept" = 1 ] && echo true || echo false)
  setflag vlead $([ "$vlead" = 1 ] && echo true || echo false)
  setflag tboost $([ "$tboost" = 1 ] && echo true || echo false)
  setflag gainsched $([ "$gsched" = 1 ] && echo true || echo false)
  setflag commit $([ "$commit" = 1 ] && echo true || echo false)
  sleep 1
  durum=$(curl -s $API/api/ozellikler | python3 -c "import sys,json;print(json.load(sys.stdin)['durum'])" 2>/dev/null)
  echo ">>> KOL=$ad  bayraklar=$durum"
  saglik_kapisi
  bash tools/ucus_test.sh "kol_$ad" "$SURE" "$KOK" "$GCSLOG" "$SEN" 2>&1 | grep -E "SONUÇ|angajman=" | tail -2
  echo "---"
done
echo "=================================================================="
echo "YENİ_SWEEP bitti $(date +%H:%M:%S)"
