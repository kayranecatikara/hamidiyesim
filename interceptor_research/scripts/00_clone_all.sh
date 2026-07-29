#!/usr/bin/env bash
# Interceptor arastirmasi icin kaynak repolari repos/ altina siğ klonlar.
# Tekrar calistirilabilir: mevcut klonlari atlar.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOS="$ROOT/repos"
mkdir -p "$REPOS"

# repo_url | hedef_dizin | branch | ekstra_git_bayraklari
REPO_LIST=(
  "https://github.com/mzahana/d2dtracker_sim|d2dtracker_sim|main|"
  "https://github.com/Lexicon121/Strix-Interceptor|Strix-Interceptor|main|"
  "https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8|PX4-ROS2-Gazebo-YOLOv8|main|"
  "https://github.com/ctu-mrs/mrs_uav_gazebo_simulator|mrs_uav_gazebo_simulator|ros2|"
  "https://github.com/Zhefan-Xu/uav_simulator|uav_simulator|main|"
  "https://github.com/arijit-dasgupta/UAVProjectileCatcher|UAVProjectileCatcher|master|--filter=blob:none"
  "https://github.com/ctu-mrs/mrs_uav_gazebo_simulator|mrs_uav_gazebo_simulator_master|master|"
  "https://github.com/Intelligent-Quads/iq_sim|iq_sim|master|"
  # Arastirma sirasinda cikan ek kaynaklar
  "https://github.com/ctu-mrs/mrs_gazebo_common_resources|mrs_gazebo_common_resources|master|"
  "https://github.com/ctu-mrs/mrs_gazebo_extras_resources|mrs_gazebo_extras_resources|master|"
  "https://github.com/Bochicchio3/MBZIRC-2020-Challenge|MBZIRC-2020-Challenge|master|"
)

ok=0; skip=0; fail=0
for entry in "${REPO_LIST[@]}"; do
  IFS='|' read -r url dir branch extra <<< "$entry"
  dest="$REPOS/$dir"

  if [[ -d "$dest/.git" ]]; then
    echo "[ATLA] $dir zaten var"
    skip=$((skip+1)); continue
  fi

  echo "[KLON] $dir  ($url @ $branch)"
  # shellcheck disable=SC2086
  if git clone --depth 1 --branch "$branch" $extra "$url" "$dest" 2>&1 | tail -2; then
    ok=$((ok+1))
  else
    echo "[HATA] $dir klonlanamadi"
    fail=$((fail+1))
  fi
done

echo
echo "===== OZET ====="
echo "klonlanan: $ok  |  atlanan: $skip  |  hatali: $fail"
du -sh "$REPOS"/* 2>/dev/null | sort -h
