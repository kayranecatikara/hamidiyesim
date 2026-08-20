#!/bin/bash
# ============================================================================
# mkur.sh — TEK KOMUTLA SİM KURULUMU   (VARSAYILAN: HEADLESS)
#   Gazebo Harmonic + ArduCopter SITL (avcı) + ArduPlane SITL (hedef) + GCS
#
# Kullanım:  [env...] bash scripts/mkur.sh [etiket]
#   [etiket]   log dosyası eki (gz_<etiket>.log gibi). Varsayılan: m
#   AVCI_GUI=1 Gazebo penceresini aç. Varsayılan headless
#              (--headless-rendering: pencere yok ama KAMERALAR RENDER EDİLİR).
#
# ⚠ BORUYA BAĞLAMA (CLAUDE.md §9) — arka plandaki sim süreçleri yüzünden boru
#   EOF almaz, script asılı kalır. Çıktıyı dosyaya yaz:
#       bash scripts/mkur.sh m > ~/.avci_sim/log/kur_m.log 2>&1
#
# ÇIKIŞ KODU: 0 = hazır ("sim hazır HH:MM:SS" satırı). 1 = kurulamadı; son
# satır neyin eksik kaldığını söyler. Kör "hazır" YOK.
# Kapatmak için: bash scripts/kapat.sh
# ============================================================================

# Yollar — hepsi env ile ezilebilir. Varsayılanlar bu makinenin düzeni.
REPO=${AVCI_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
ARDUPILOT=${ARDUPILOT_DIR:-$HOME/ardupilot}
AP_GAZEBO=${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}
LOG=${AVCI_LOG_DIR:-$HOME/.avci_sim/log}

mkdir -p "$LOG"
T=${1:-m}

# --- 0) ÖNCE TEMİZ Mİ DİYE BAK -------------------------------------------
# En sık "kurulmuyor" sebebi: önceki simden kalan süreçler. 9002/8000 dolu
# olduğu için yeni Gazebo FDM'i ve yeni gcs_server sessizce çöker, script
# 4.5 dk bekleyip boşuna döner. Baştan söyle.
DESEN='gz sim|gz-sim-server|sim_vehicle|/arducopter|/arduplane|control\.gcs_server'
if pgrep -f "$DESEN" >/dev/null 2>&1; then
  echo "HATA: sim zaten ayakta — önce 'bash scripts/kapat.sh' çalıştır."
  pgrep -af "$DESEN" | cut -c1-110
  exit 1
fi
if ss -lnt 2>/dev/null | grep -q ':8000 '; then
  echo "HATA: 8000 portu dolu (panel). Kapat: fuser -k 8000/tcp"
  exit 1
fi

cd "$REPO" || { echo "HATA: depo kökü yok: $REPO"; exit 1; }
set +u; source /opt/ros/humble/setup.bash; set +u
export GZ_SIM_SYSTEM_PLUGIN_PATH=$AP_GAZEBO/build
export GZ_SIM_RESOURCE_PATH=$REPO/sim/gazebo_harmonic/models:$AP_GAZEBO/models:$AP_GAZEBO/worlds
DUNYA=sim/gazebo_harmonic/worlds/avci_harmonic.sdf

# --- 1) GAZEBO ------------------------------------------------------------
if [ "${AVCI_GUI:-0}" = "1" ]; then
  # Pencereli. DISPLAY'i SABİTLEME (§ 'Unable to open display ":1"') — makinede
  # ne varsa o kullanılır.
  echo "[mkur] Gazebo pencereli — DISPLAY=${DISPLAY:-(boş!)}"
  nohup gz sim -r -v2 "$DUNYA" > "$LOG/gz_$T.log" 2>&1 &
else
  # Headless. -s yalnız sunucu, --headless-rendering kameraları ekransız
  # render eder. Bu bayrak OLMADAN kamera topic'leri boş kalır ve görsel
  # güdüm hiç çalışmaz — headless'ın tek doğru hâli budur.
  echo "[mkur] Gazebo headless (--headless-rendering)"
  unset DISPLAY
  nohup gz sim -s -r --headless-rendering -v2 "$DUNYA" > "$LOG/gz_$T.log" 2>&1 &
fi

# Kör 'sleep 6' YOK — ArduCopter'ın bağlanacağı FDM portu (9002) beklenir.
# 9002 açılmadan SITL başlatılırsa araç Gazebo'ya hiç bağlanamaz ve düşer.
for i in $(seq 1 60); do
  ss -lnu 2>/dev/null | grep -q ':9002 ' && break
  sleep 1
done
if ! ss -lnu 2>/dev/null | grep -q ':9002 '; then
  echo "HATA: Gazebo FDM portu (9002) 60 sn'de açılmadı. Bak: $LOG/gz_$T.log"
  exit 1
fi

# --- 2) İKİ SITL ----------------------------------------------------------
# 'script -qfec' sarmalı zorunlu: MAVProxy TTY'siz çıkar (§9).
APT=$ARDUPILOT/Tools/autotest
( cd "$ARDUPILOT" && nohup script -qfec "python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --sysid 5 --no-rebuild -w --add-param-file=$APT/default_params/copter.parm --add-param-file=$APT/default_params/gazebo-iris.parm --add-param-file=$REPO/sim/ardupilot_params/avci_copter.parm --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --mavproxy-args='--streamrate=25'" /dev/null > "$LOG/cop_$T.log" 2>&1 & )
( cd "$ARDUPILOT" && nohup script -qfec "python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 -I1 --sysid 2 --no-rebuild --add-param-file=$REPO/sim/ardupilot_params/avci_plane.parm --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --mavproxy-args='--streamrate=25'" /dev/null > "$LOG/pla_$T.log" 2>&1 & )

# Kör 'sleep' YOK — aracın kendi çıktısındaki EKF+GPS kilidi beklenir (~50 sn).
for i in $(seq 1 90); do
  grep -qa "EKF3 IMU0 is using GPS" "$LOG/cop_$T.log" 2>/dev/null && grep -qa "EKF3 IMU0 is using GPS" "$LOG/pla_$T.log" 2>/dev/null && break
  sleep 3
done
grep -qa "EKF3 IMU0 is using GPS" "$LOG/cop_$T.log" 2>/dev/null || { echo "HATA: ArduCopter GPS kilidi gelmedi. Bak: $LOG/cop_$T.log"; exit 1; }
grep -qa "EKF3 IMU0 is using GPS" "$LOG/pla_$T.log" 2>/dev/null || { echo "HATA: ArduPlane GPS kilidi gelmedi. Bak: $LOG/pla_$T.log"; exit 1; }

# --- 3) GCS ---------------------------------------------------------------
# AVCI_NO_BROWSER=1 şart: gcs_server açılıştan 2 sn sonra webbrowser.open()
# çağırıyor; ekransız/SSH oturumunda bu açılışı kilitliyor.
export AVCI_GZ_CAMERA=1 AVCI_GORSEL=on AVCI_NO_BROWSER=1
nohup python3 -m control.gcs_server > "$LOG/gcs_$T.log" 2>&1 &
for i in $(seq 1 60); do curl -sf -m 2 http://127.0.0.1:8000/api/guidance_mode >/dev/null 2>&1 && break; sleep 2; done
curl -sf -m 2 http://127.0.0.1:8000/api/guidance_mode >/dev/null 2>&1 || { echo "HATA: panel (8000) cevap vermedi. Bak: $LOG/gcs_$T.log"; exit 1; }

# --- 4) KAMERA KAPISI -----------------------------------------------------
# Panelin cevap vermesi YETMEZ: görsel güdümün kalbi kameralar. Headless'ta
# render bozuksa API 200 döner ama kutu hiç gelmez — uçuş boşa gider.
# ⚠ İKİ ÖN KAMERAYI İSMEN ARA. Dış görüş (chase) kameraları eklendikten
# sonra logda 4 tane "ilk görüntü" satırı olabiliyor; kaba sayım, ön kamera
# hiç gelmeden de dolabilir. Güdümün ihtiyacı olan bu ikisidir.
#
# ⏱ PENCERE NİYE BU KADAR GENİŞ — ÖLÇÜLDÜ (2026-08-18):
#   talon ön / avcı dış / talon dış  →  gcs başlangıcından  6.0 s
#   IRIS ÖN                          →                     31.0 s   (5 kat)
# Iris ön akışı YOLO'dan geçiyor (model yükleme + CUDA ısınması); diğer üçü
# ham geçiyor. Eskiden pencere 80 s'ydi ve DEĞİŞKENLİK yüzünden bazı koşularda
# dolmuyordu: script "render yok" diye HATA basıyordu, oysa sim sapasağlam
# kalkmıştı. "Sim açılmıyor" şikâyetinin sebebi buydu. 180 s = ölçülenin ~6 katı.
ON_KAM() { grep -qa "Iris kamerasından ilk görüntü" "$LOG/gcs_$T.log" 2>/dev/null \
        && grep -qa "Talon (hedef İHA) kamerasından ilk görüntü" "$LOG/gcs_$T.log" 2>/dev/null; }
for i in $(seq 1 90); do
  ON_KAM && break
  sleep 2
done
if ! ON_KAM; then
  echo "HATA: ön kamera (iris/YOLO) 180 sn'de gelmedi. Bak: $LOG/gcs_$T.log"
  # ⚠ Süreçler ÖLDÜRÜLMEZ (log/teşhis dursun, sim gecikmeli hazır olabilir) —
  #   ama bunu SÖYLEMEK zorunlu: aksi hâlde kullanıcı tekrar deniyor ve
  #   "sim zaten ayakta" kapısına çarpıyor, iki hata üst üste kafa karıştırıyor.
  echo "      ⚠ SÜREÇLER AYAKTA BIRAKILDI. Panel yine de çalışıyor olabilir:"
  echo "        http://127.0.0.1:8000   ·   kontrol: grep 'ilk görüntü' $LOG/gcs_$T.log"
  echo "      ⚠ TEKRAR DENEMEDEN ÖNCE:  bash scripts/kapat.sh"
  exit 1
fi

# --- 5) KİP + SAYAÇ -------------------------------------------------------
curl -s -X POST http://127.0.0.1:8000/api/guidance_mode -H "Content-Type: application/json" -d '{"mode":"hybrid"}' >/dev/null
curl -s -X POST http://127.0.0.1:8000/api/hasar/sifirla >/dev/null
echo "sim hazır $(date +%H:%M:%S)"
