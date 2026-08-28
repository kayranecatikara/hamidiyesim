#!/usr/bin/env bash
# Ö-KF (görüntü gecikmesi telafisi) ÖNCESİNDEKİ güdüm algoritmasını GERİ YÜKLER.
# Kullanım:  bash yedek/algoritma_2026-08-28_1406_gecikme_oncesi/GERI_AL.sh
# Uyarı: Ö-KF ve sonrasındaki tüm güdüm değişikliklerini siler.
set -euo pipefail
Y="$(cd "$(dirname "$0")" && pwd)"
R="$(cd "$Y/../.." && pwd)"
cd "$R"
echo "[geri-al] hedef depo: $R"
echo "[geri-al] geri yüklenecek nokta: $(head -2 "$Y/git_head.txt" | tail -1)"
read -r -p "Ö-KF ve sonraki degisiklikler SILINECEK. Devam? (evet/hayir) " c
[ "$c" = "evet" ] || { echo "iptal"; exit 1; }
rm -rf control/guidance/__pycache__ tests/__pycache__
# ⚠ §5.12: silinen özellik TAMAMEN silinir — Ö-KF'nin kendi modülü ve testi de.
rm -f control/guidance/gecikme_kf.py tests/test_gecikme_kf.py
cp -a "$Y/dosyalar/guidance/."             control/guidance/
cp -a "$Y/dosyalar/ayar_konsolu.py"        control/ayar_konsolu.py
cp -a "$Y/dosyalar/gcs_server.py"          control/gcs_server.py
cp -a "$Y/dosyalar/run_plane_scenario.py"  control/run_plane_scenario.py
cp -a "$Y/dosyalar/gcs_ui/index.html"      control/gcs_ui/index.html
cp -a "$Y/dosyalar/gcs_ui/script.js"       control/gcs_ui/script.js
cp -a "$Y/dosyalar/tests/test_bbox_ibvs.py" tests/test_bbox_ibvs.py
cp -a "$Y/dosyalar/UYGULANACAK.md"         UYGULANACAK.md
cp -a "$Y/dosyalar/ardupilot_params/avci_copter.parm" sim/ardupilot_params/avci_copter.parm
echo "[geri-al] dogrulama:"
PYTHONPATH=. python3 tests/test_bbox_ibvs.py | tail -2
echo "[geri-al] TAMAM. Sim ayaktaysa yeniden kur:"
echo "          bash scripts/kapat.sh && AVCI_TEMIZ=1 bash scripts/mkur.sh m"
