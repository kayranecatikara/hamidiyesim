#!/usr/bin/env bash
#===============================================================================
# run_simulation.sh
#
# Launches the Gazebo Garden/Harmonic world containing the bullet_interceptor
# model and then starts ArduPilot SITL (ArduCopter) bound to it over the
# JSON/FDM link on UDP 9002 (SITL -> Gazebo) and 9003 (Gazebo -> SITL).
#
# Usage:
#   ./run_simulation.sh                 # Gazebo GUI + SITL + MAVProxy console/map
#   ./run_simulation.sh --headless      # no Gazebo GUI (server only)
#   ./run_simulation.sh --no-console    # SITL without MAVProxy console/map
#   ./run_simulation.sh --wipe          # wipe SITL EEPROM before loading params
#   ./run_simulation.sh --gazebo-only   # start Gazebo and stop (no SITL)
#   ./run_simulation.sh --sitl-only     # start SITL and stop (Gazebo already up)
#
# Environment overrides:
#   ARDUPILOT_HOME        default: $HOME/ardupilot
#   ARDUPILOT_GAZEBO_HOME default: $HOME/ardupilot_gazebo
#   SIM_HOME              default: 40.1281,32.9953,950,0   (lat,lon,alt,heading)
#===============================================================================

set -euo pipefail

#-------------------------------------------------------------------------------
# Paths
#-------------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_HOME="${ARDUPILOT_GAZEBO_HOME:-$HOME/ardupilot_gazebo}"

WORLD_FILE="$SCRIPT_DIR/worlds/interceptor_world.sdf"
PARAM_FILE="$SCRIPT_DIR/config/interceptor_params.param"
MODEL_DIR="$SCRIPT_DIR/models"

SIM_HOME="${SIM_HOME:-40.1281,32.9953,950,0}"
FDM_PORT_IN=9002
FDM_PORT_OUT=9003

#-------------------------------------------------------------------------------
# Argument parsing
#-------------------------------------------------------------------------------
HEADLESS=0
USE_CONSOLE=1
WIPE_EEPROM=0
RUN_GAZEBO=1
RUN_SITL=1

for arg in "$@"; do
  case "$arg" in
    --headless)     HEADLESS=1 ;;
    --no-console)   USE_CONSOLE=0 ;;
    --wipe)         WIPE_EEPROM=1 ;;
    --gazebo-only)  RUN_SITL=0 ;;
    --sitl-only)    RUN_GAZEBO=0 ;;
    -h|--help)      sed -n '3,22p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)              echo "Unknown option: $arg  (use --help)" >&2; exit 1 ;;
  esac
done

#-------------------------------------------------------------------------------
# Pretty logging
#-------------------------------------------------------------------------------
C_RESET=$'\033[0m'; C_INFO=$'\033[1;36m'; C_OK=$'\033[1;32m'
C_WARN=$'\033[1;33m'; C_ERR=$'\033[1;31m'

info()  { printf '%s[ INFO ]%s %s\n' "$C_INFO" "$C_RESET" "$*"; }
ok()    { printf '%s[  OK  ]%s %s\n' "$C_OK"   "$C_RESET" "$*"; }
warn()  { printf '%s[ WARN ]%s %s\n' "$C_WARN" "$C_RESET" "$*"; }
die()   { printf '%s[ FAIL ]%s %s\n' "$C_ERR"  "$C_RESET" "$*" >&2; exit 1; }

#-------------------------------------------------------------------------------
# Pre-flight checks
#-------------------------------------------------------------------------------
info "Interceptor simulation bootstrap"
info "Project root      : $SCRIPT_DIR"

command -v gz >/dev/null 2>&1 || die "'gz' not found. Install Gazebo Garden or Harmonic."

GZ_VERSION="$(gz sim --versions 2>/dev/null | head -n1 || true)"
info "Gazebo Sim version: ${GZ_VERSION:-unknown}"

[[ -f "$WORLD_FILE" ]] || die "World file missing: $WORLD_FILE"
[[ -f "$PARAM_FILE" ]] || die "Param file missing: $PARAM_FILE"
[[ -f "$MODEL_DIR/bullet_interceptor/model.sdf" ]] \
  || die "Model missing: $MODEL_DIR/bullet_interceptor/model.sdf"

SIM_VEHICLE="$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py"
if (( RUN_SITL )); then
  [[ -x "$SIM_VEHICLE" || -f "$SIM_VEHICLE" ]] \
    || die "sim_vehicle.py not found at $SIM_VEHICLE (set ARDUPILOT_HOME)."
fi

#-------------------------------------------------------------------------------
# Environment: model + plugin search paths
#-------------------------------------------------------------------------------
export GZ_SIM_RESOURCE_PATH="$MODEL_DIR:$SCRIPT_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_HOME/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

# Garden and older releases also honour the IGN_* names.
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$GZ_SIM_SYSTEM_PLUGIN_PATH"

# Extra model libraries shipped with ardupilot_gazebo, if present.
if [[ -d "$ARDUPILOT_GAZEBO_HOME/models" ]]; then
  export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds"
  export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"
fi

info "GZ_SIM_RESOURCE_PATH      = $GZ_SIM_RESOURCE_PATH"
info "GZ_SIM_SYSTEM_PLUGIN_PATH = $GZ_SIM_SYSTEM_PLUGIN_PATH"

if ! ls "$ARDUPILOT_GAZEBO_HOME/build/"libArduPilotPlugin.* >/dev/null 2>&1; then
  warn "libArduPilotPlugin not found in $ARDUPILOT_GAZEBO_HOME/build"
  warn "Build it with:"
  warn "  cd $ARDUPILOT_GAZEBO_HOME && mkdir -p build && cd build && cmake .. && make -j4"
fi

#-------------------------------------------------------------------------------
# Port hygiene: a stale gz or SITL still holding 9002/9003 breaks lock-step.
#-------------------------------------------------------------------------------
for port in "$FDM_PORT_IN" "$FDM_PORT_OUT"; do
  if command -v ss >/dev/null 2>&1 && ss -lun 2>/dev/null | grep -q ":$port\b"; then
    warn "UDP port $port is already in use - a previous run may still be alive."
  fi
done

#-------------------------------------------------------------------------------
# Cleanup on exit
#-------------------------------------------------------------------------------
GZ_PID=""
cleanup() {
  if [[ -n "$GZ_PID" ]] && kill -0 "$GZ_PID" 2>/dev/null; then
    info "Stopping Gazebo (pid $GZ_PID)..."
    kill "$GZ_PID" 2>/dev/null || true
    wait "$GZ_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

#-------------------------------------------------------------------------------
# 1) Start Gazebo
#-------------------------------------------------------------------------------
if (( RUN_GAZEBO )); then
  GZ_ARGS=(-r -v4 "$WORLD_FILE")
  if (( HEADLESS )); then
    GZ_ARGS=(-s -r -v4 "$WORLD_FILE")
    info "Starting Gazebo (headless server) with world: $(basename "$WORLD_FILE")"
  else
    info "Starting Gazebo (GUI) with world: $(basename "$WORLD_FILE")"
  fi

  gz sim "${GZ_ARGS[@]}" &
  GZ_PID=$!
  ok "Gazebo launched (pid $GZ_PID)"

  # Give the physics server and the ArduPilot plugin time to bind the FDM ports.
  info "Waiting for the ArduPilot FDM socket on UDP $FDM_PORT_IN ..."
  for _ in $(seq 1 30); do
    if command -v ss >/dev/null 2>&1 && ss -lun 2>/dev/null | grep -q ":$FDM_PORT_IN\b"; then
      ok "FDM socket is up."
      break
    fi
    kill -0 "$GZ_PID" 2>/dev/null || die "Gazebo exited during startup - check the log above."
    sleep 1
  done
fi

if (( ! RUN_SITL )); then
  ok "Gazebo-only mode. Press Ctrl+C to stop."
  wait "$GZ_PID"
  exit 0
fi

#-------------------------------------------------------------------------------
# 2) Start ArduPilot SITL
#-------------------------------------------------------------------------------
SITL_ARGS=(
  -v ArduCopter
  -f gazebo-iris
  --model JSON
  --console
  --map
  --add-param-file="$PARAM_FILE"
  --custom-location="$SIM_HOME"
  --out=udp:127.0.0.1:14550
  --out=udp:127.0.0.1:14551
)

if (( ! USE_CONSOLE )); then
  SITL_ARGS=("${SITL_ARGS[@]/--console}")
  SITL_ARGS=("${SITL_ARGS[@]/--map}")
fi

if (( WIPE_EEPROM )); then
  SITL_ARGS+=(-w)
  info "EEPROM will be wiped before the parameter file is applied."
fi

info "Starting ArduPilot SITL (ArduCopter, JSON backend)"
info "  sim_vehicle.py ${SITL_ARGS[*]}"
info "  FDM: SITL -> Gazebo udp://127.0.0.1:$FDM_PORT_IN"
info "       Gazebo -> SITL udp://127.0.0.1:$FDM_PORT_OUT"
info "  GCS: udp://127.0.0.1:14550 (QGroundControl / Mission Planner)"

cd "$ARDUPILOT_HOME"
python3 "$SIM_VEHICLE" "${SITL_ARGS[@]}"

#-------------------------------------------------------------------------------
# Quick MAVProxy test sequence once the vehicle is up:
#
#   param show FRAME_CLASS       # expect 1
#   mode GUIDED
#   arm throttle
#   takeoff 30
#   # 500 m dash north at 55 m/s (~200 km/h):
#   velocity 55 0 0
#
# Nose camera stream (separate terminal):
#   gz topic -e -t /bullet_interceptor/nose_camera/image
#   # or view it live:
#   gz sim -g   # then Visualization -> Image Display -> pick the camera topic
#-------------------------------------------------------------------------------
