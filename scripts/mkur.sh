#!/bin/bash
# ============================================================================
# mkur.sh — TEK KOMUTLA SİM KURULUMU
#   Gazebo Harmonic + ArduCopter SITL (avcı) + ArduPlane SITL (hedef) + GCS
#
# Kullanım:  [AVCI_* env...] bash scripts/mkur.sh <etiket>
#   <etiket> log dosyalarına ek olarak yazılır (gz_<etiket>.log gibi).
#
# ⚠ BORUYA BAĞLAMA (CLAUDE.md §9) — arka plandaki sim süreçleri yüzünden boru
#   EOF almaz, script asılı kalır. Çıktıyı dosyaya yaz:
#       bash scripts/mkur.sh m > ~/.avci_sim/log/kur_m.log 2>&1
#
# "sim hazır HH:MM:SS" satırını yazınca hazırdır (~90 sn).
# Kapatmak için: bash scripts/kapat.sh
# ============================================================================

# Yollar — hepsi env ile ezilebilir. Varsayılanlar bu makinenin düzeni.
REPO=${AVCI_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
ARDUPILOT=${ARDUPILOT_DIR:-$HOME/ardupilot}
AP_GAZEBO=${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}
LOG=${AVCI_LOG_DIR:-$HOME/.avci_sim/log}

mkdir -p $LOG
T=${1:-m}
cd $REPO
set +u; source /opt/ros/humble/setup.bash; set +u
export GZ_SIM_SYSTEM_PLUGIN_PATH=$AP_GAZEBO/build
export GZ_SIM_RESOURCE_PATH=$REPO/sim/gazebo_harmonic/models:$AP_GAZEBO/models:$AP_GAZEBO/worlds
export DISPLAY=:1
nohup gz sim -r -v2 sim/gazebo_harmonic/worlds/avci_harmonic.sdf > $LOG/gz_$T.log 2>&1 &
sleep 6
APT=$ARDUPILOT/Tools/autotest
( cd $ARDUPILOT && nohup script -qfec "python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --sysid 5 --no-rebuild -w --add-param-file=$APT/default_params/copter.parm --add-param-file=$APT/default_params/gazebo-iris.parm --add-param-file=$REPO/sim/ardupilot_params/avci_copter.parm --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --mavproxy-args='--streamrate=25'" /dev/null > $LOG/cop_$T.log 2>&1 & )
( cd $ARDUPILOT && nohup script -qfec "python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 -I1 --sysid 2 --no-rebuild --add-param-file=$REPO/sim/ardupilot_params/avci_plane.parm --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --mavproxy-args='--streamrate=25'" /dev/null > $LOG/pla_$T.log 2>&1 & )
# Kör 'sleep' YOK — aracın kendi çıktısındaki EKF+GPS kilidi beklenir (~50 sn).
for i in $(seq 1 90); do
  grep -qa "EKF3 IMU0 is using GPS" $LOG/cop_$T.log 2>/dev/null && grep -qa "EKF3 IMU0 is using GPS" $LOG/pla_$T.log 2>/dev/null && break
  sleep 3
done
export AVCI_GZ_CAMERA=1 AVCI_GORSEL=on
nohup python3 -m control.gcs_server > $LOG/gcs_$T.log 2>&1 &
for i in $(seq 1 60); do curl -sf -m 2 http://127.0.0.1:8000/api/guidance_mode >/dev/null 2>&1 && break; sleep 2; done
curl -s -X POST http://127.0.0.1:8000/api/guidance_mode -H "Content-Type: application/json" -d '{"mode":"hybrid"}'
curl -s -X POST http://127.0.0.1:8000/api/hasar/sifirla >/dev/null
echo " | sim hazır $(date +%H:%M:%S)"
