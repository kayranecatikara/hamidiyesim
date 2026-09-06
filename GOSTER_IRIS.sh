#!/usr/bin/env bash
# vakkas-entegre'nin taretli irisini (iris_cam) GUI'de acar.
#   ./GOSTER_IRIS.sh
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
W=avci                      # world adi; dosya adi avci_harmonic.sdf
: "${ARDUPILOT_GAZEBO_ROOT:=$HOME/ardupilot_gazebo}"

export GZ_SIM_RESOURCE_PATH="$ROOT/sim/gazebo_harmonic/models:$ARDUPILOT_GAZEBO_ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_ROOT/build:$ROOT/plugins/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# Acik kosumlari kapat. "gz sim" aslinda bir ruby sureci; comm=gz ile
# eslesmiyor, bu yuzden tam komut satirina bakiyoruz.
pkill -f "^gz sim" 2>/dev/null
for _ in $(seq 10); do pgrep -f "^gz sim" >/dev/null || break; sleep 0.5; done

echo ">> Gazebo aciliyor (iris_cam / $W)..."
gz sim -r "$ROOT/sim/gazebo_harmonic/worlds/avci_harmonic.sdf" >/tmp/gz_iris.log 2>&1 &

for _ in $(seq 60); do
  gz topic -l 2>/dev/null | grep -q "/world/$W/stats" && break
  sleep 0.5
done
if ! gz topic -l 2>/dev/null | grep -q "/world/$W/stats"; then
  echo "HATA: dunya acilmadi. Log: /tmp/gz_iris.log"; tail -20 /tmp/gz_iris.log; exit 1
fi

# GUI duraklatilmis aciliyor; calistir
gz service -s "/world/$W/control" --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false' >/dev/null 2>&1

# Pencereyi buyut
if command -v wmctrl >/dev/null 2>&1; then
  sleep 2
  WID=$(wmctrl -l | grep -i "Gazebo Sim" | head -1 | cut -d' ' -f1)
  [ -n "$WID" ] && wmctrl -i -r "$WID" -b add,maximized_vert,maximized_horz 2>/dev/null
fi

# Kamerayi drone'a cevir. Bu dunyada /gui/move_to/pose tutmuyor;
# model adiyla /gui/move_to calisiyor (world'deki entity adi iris_with_ardupilot).
sleep 4
gz service -s /gui/move_to --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean --timeout 5000 --req 'data: "iris_with_ardupilot"' >/dev/null 2>&1

cat <<'MSG'

=== HAZIR ===
Pencerede: taretli iris (iris_cam), tepesinde taret + namlu + kamera,
onunde ag konisi, 12 m ileride hedef ucak (mini_talon_vtail).

Taret/ates icin ROS 2 kopruye ihtiyac var (ayri terminal):
  export PYTHONPATH=$HOME/projects/hamidiyesim:$PYTHONPATH
  ros2 launch ros/net_turret.launch.py

sonra:
  python3 scripts/turret_aim.py 90 45      # pan 90, tilt 45 derece
  python3 scripts/fire_net.py --hiz 20

Taret limitleri (bu tasarim): pan 0-360, tilt 0-180 (0=ileri, 90=yukari).
Ag TEK ATIMLIK -- tekrar icin ./GOSTER_IRIS.sh yeniden calistir.
MSG
