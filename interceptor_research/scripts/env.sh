# Ortak Gazebo ortam degiskenleri. Kullanim:  source scripts/env.sh
# (Bu dosya calistirilmaz, source edilir.)

_IR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export GZ_SIM_RESOURCE_PATH="\
$_IR_ROOT/models/interceptors:\
$_IR_ROOT/models/showcase:\
$_IR_ROOT/models/net_launchers:\
$_IR_ROOT/models/targets:\
$_IR_ROOT/models/platforms:\
/home/kubra/ardupilot_gazebo/models:\
$_IR_ROOT/repos/mrs_uav_gazebo_simulator/models:\
$_IR_ROOT/repos/d2dtracker_sim/models:\
$_IR_ROOT/repos/iq_sim/models:\
${GZ_SIM_RESOURCE_PATH:-}"

# ArduPilotPlugin + ParachutePlugin buradan geliyor;
# NetCapturePlugin derlendiginde plugins/build de eklenir.
export GZ_SIM_SYSTEM_PLUGIN_PATH="\
/home/kubra/ardupilot_gazebo/build:\
$_IR_ROOT/plugins/build:\
${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

export IR_ROOT="$_IR_ROOT"
export IR_MODEL="avci_net_interceptor"
export IR_WORLD="net_test"
