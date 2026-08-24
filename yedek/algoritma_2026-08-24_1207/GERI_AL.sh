#!/usr/bin/env bash
# 2026-08-24 12:07 anındaki güdüm algoritmasını GERİ YÜKLER.
# Kullanım:  bash yedek/algoritma_2026-08-24_1207/GERI_AL.sh
# Uyarı: bu tarihten SONRA yapılan tüm güdüm değişikliklerini siler.
set -euo pipefail
Y="$(cd "$(dirname "$0")" && pwd)"
R="$(cd "$Y/../.." && pwd)"
cd "$R"
echo "[geri-al] hedef depo: $R"
read -r -p "Bu tarihten sonraki degisiklikler SILINECEK. Devam? (evet/hayir) " c
[ "$c" = "evet" ] || { echo "iptal"; exit 1; }
rm -rf control/guidance/__pycache__
cp -a "$Y/dosyalar/guidance/." control/guidance/
cp -a "$Y/dosyalar/ayar_konsolu.py"        control/ayar_konsolu.py
cp -a "$Y/dosyalar/gcs_server.py"          control/gcs_server.py
cp -a "$Y/dosyalar/run_plane_scenario.py"  control/run_plane_scenario.py
cp -a "$Y/dosyalar/gcs_ui/index.html"      control/gcs_ui/index.html
cp -a "$Y/dosyalar/gcs_ui/script.js"       control/gcs_ui/script.js
cp -a "$Y/dosyalar/ardupilot_params/avci_copter.parm" sim/ardupilot_params/avci_copter.parm
echo "[geri-al] TAMAM. Sim ayaktaysa yeniden kur: bash scripts/kapat.sh && AVCI_TEMIZ=1 bash scripts/mkur.sh m"
