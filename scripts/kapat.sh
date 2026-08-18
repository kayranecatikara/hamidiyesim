#!/bin/bash
# ============================================================================
# kapat.sh — SİMÜLASYONU KOMPLE KAPAT
#   Gazebo + iki SITL + MAVProxy + gcs_server + senaryo süreci.
#
# Kullanım:  bash scripts/kapat.sh
#
# ⚠ Test bitince araçları havada KONTROLSÜZ BIRAKMA (CLAUDE.md §9) — simi
#   komple kapat.
# ⚠ 'pkill -f' KENDİ KABUĞUNU ÖLDÜREBİLİR (exit 144): komut satırında yazılan
#   desen, çalışan kabuğun komut satırında da göründüğü için kendini eşler.
#   Bu yüzden ayrı bir script DOSYASINDA duruyor — inline çalıştırma.
#
# Önce TERM (araçlara temiz kapanma şansı), 3 sn sonra KILL (takılan varsa).
# ============================================================================
DESEN='gz sim|gz-sim-server|gz-sim-gui|sim_vehicle|mavproxy|arducopter|arduplane|model JSON|control.gcs_server|run_plane_scenario'
pkill -TERM -f "$DESEN" 2>/dev/null
sleep 3
pkill -KILL -f "$DESEN" 2>/dev/null
sleep 2
echo "kapatıldı"
