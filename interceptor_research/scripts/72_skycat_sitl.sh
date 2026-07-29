#!/bin/bash
# skycat_tvbs icin ArduPilot SITL (ArduPlane) baslatir.
#
# Instance 1 kullaniliyor: bu makinede baska bir SITL oturumu instance 0'i
# (TCP 5760 + FDM 9002) tutabiliyor. Instance 1 -> TCP 5770, FDM 9012.
# Model SDF'indeki <fdm_port_in>9012</fdm_port_in> bununla eslesiyor.
#
# Kullanim:
#   source scripts/env.sh
#   gz sim -r worlds/skycat_runway.sdf     # once Gazebo
#   ./scripts/72_skycat_sitl.sh            # sonra SITL
#   ./scripts/71_skycat_flight.py          # sonra ucus sekansi (tcp:5770)
set -euo pipefail

IR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CALISMA="${IR_SITL_DIR:-/tmp/skycat_sitl}"
mkdir -p "$CALISMA"
cd "$CALISMA"

exec "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" \
  -v ArduPlane \
  --model JSON \
  --add-param-file="$IR_ROOT/config/skycat_tvbs.param" \
  -I1 \
  --no-mavproxy \
  "$@"
