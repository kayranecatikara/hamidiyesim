# Ortak Gazebo ortam degiskenleri. Kullanim:  source scripts/env.sh
# (Bu dosya calistirilmaz, source edilir.)

_IR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ardupilot_gazebo'nun yeri. Baska bir yerdeyse source etmeden once ayarlayin:
#   export ARDUPILOT_GAZEBO_ROOT=/yol/ardupilot_gazebo
# Yoksa sorun degil: bullet_net_test / net_test dunyalari ArduPilotPlugin
# olmadan da acilir (Gazebo uyari verir, taret ve ag calismaya devam eder).
# Sadece SITL ile UCUS icin gerekir.
: "${ARDUPILOT_GAZEBO_ROOT:=$HOME/ardupilot_gazebo}"

export GZ_SIM_RESOURCE_PATH="\
$_IR_ROOT/models/interceptors:\
$_IR_ROOT/models/net_launchers:\
$_IR_ROOT/models/targets:\
$ARDUPILOT_GAZEBO_ROOT/models:\
${GZ_SIM_RESOURCE_PATH:-}"

# ArduPilotPlugin + ParachutePlugin buradan geliyor;
# NetCapturePlugin/NetLauncherPlugin derlendiginde plugins/build de eklenir.
export GZ_SIM_SYSTEM_PLUGIN_PATH="\
$ARDUPILOT_GAZEBO_ROOT/build:\
$_IR_ROOT/plugins/build:\
${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

export IR_ROOT="$_IR_ROOT"
export IR_MODEL="bullet_net_interceptor"
export IR_WORLD="bullet_net_test"
