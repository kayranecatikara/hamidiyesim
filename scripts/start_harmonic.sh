#!/bin/bash
# =============================================================
#  AVCI SİM — Gazebo HARMONIC tam sistem başlatıcı
# =============================================================
# Doğru sırayla, tek seferde başlatır (benim otonom test döngümdeki
# çoklu-restart karışıklığını önlemek için — sen bunu tek çalıştırırsın).
#
#   1) Gazebo Harmonic (avci world: iris_cam + mini_talon_vtail hedef)
#   2) ArduCopter SITL (gazebo-iris --model JSON) — iris Harmonic FDM 9002
#   3) ArduPlane SITL (mini_talon --model JSON:9012) — Talon Harmonic FDM 9012
#      Talon Gazebo'da GERÇEKTEN uçar (relay YOK); gcs "kare çiz" ile kontrol.
#
# Ardından AYRI terminallerde:
#   - python3 -m control.gcs_server      (web GCS + gz kamera + chase/strike)
#   - bash scripts/start_mission_planner.sh
#
# Kullanım:
#   bash scripts/start_harmonic.sh            # GUI (NVIDIA render, önerilen)
#   GZ_HEADLESS=1 bash scripts/start_harmonic.sh   # görüntüsüz
#   bash scripts/start_harmonic.sh stop       # durdur (doğrulayarak)
#   bash scripts/start_harmonic.sh durum      # ne ayakta? (hiçbir şeyi öldürmez)

# Depo kökü, script'in kendi konumundan türetilir — böylece depo başka bir
# dizine klonlandığında da kendi world/model/param/log dosyalarını kullanır.
# (Eskiden $HOME/projects/avci_sim sabit yazılıydı; başka bir yoldan
#  çalıştırıldığında script yine o dizinden okumaya çalışıyordu.)
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AP="$HOME/ardupilot"
APT="$AP/Tools/autotest"
APGZ="$HOME/ardupilot_gazebo"
LOG="$PROJ/logs"; mkdir -p "$LOG"
WORLD="$PROJ/sim/gazebo_harmonic/worlds/avci_harmonic.sdf"

# ─────────────────────────────────────────────────────────────
#  SÜREÇ AVI — kendini öldürmeyen, doğrulayan
# ─────────────────────────────────────────────────────────────
# ⚠ ÖLÇÜLDÜ (2026-08-02): eski hâli `pkill -9 -f 'model JSON' ...` döngüsüydü.
# `pkill -f` deseni ÇAĞIRAN KABUĞUN komut satırında da arar. Bu satır
#     pkill -9 -f 'gz sim|sim_vehicle|mavproxy|...' ; bash scripts/start_harmonic.sh
# çalıştırıldığında pkill kendi kabuğunu öldürüyor → satırın geri kalanı HİÇ
# çalışmıyor. Aynı şekilde `stop` içindeki döngü 3. desende ('sim_vehicle')
# kabuğu öldürünce 4-6. desenler ('mavproxy', 'gz sim', 'ruby.*gz') hiç
# çalışmıyordu → gz sim ve mavproxy AYAKTA kalıyordu.
# Üstelik eski kod koşulsuz "Durduruldu." yazıyordu: süreçler yaşarken bile
# kullanıcı durduğunu sanıyordu. Bu belge boyunca "Ctrl+C işe yaramıyor"
# sanılan durumların bir kısmı aslında BUYDU.
#
# Çözüm: pgrep ile aday çıkar → kendi PID'imizi ve TÜM ATA süreçlerimizi
# listeden düş → öldür → TEKRAR BAK → hâlâ varsa hata ver.
#
# Ayrıca -9 yerine önce TERM: ArduPilot SIGKILL yerse kara kutu (.BIN) yarım
# kalıyor ve `PSCD`/`ATT` ölçümleri son saniyeleri kaybediyor.

AVCI_DESEN='gz sim|gz-sim-(server|gui)|ruby.*gz sim|sim_vehicle|mavproxy|/arducopter|/arduplane|model JSON|control\.gcs_server|run_plane_scenario'

# Kendi PID'imiz + tüm ata zincirimiz (kabuk, terminal, ssh, ...)
_atalar() {
    local p=$$
    while [ -n "$p" ] && [ "$p" -gt 1 ] 2>/dev/null; do
        echo "$p"
        p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
    done
}

# Öldürülecek PID'ler — ata zinciri HARİÇ
_hedef_pidler() {
    local korunan
    korunan=" $(_atalar | tr '\n' ' ') "
    local pid
    for pid in $(pgrep -f "$AVCI_DESEN" 2>/dev/null); do
        case "$korunan" in *" $pid "*) continue ;; esac
        echo "$pid"
    done
}

_listele() {
    local pidler="$1"
    [ -z "$pidler" ] && return 0
    ps -o pid=,etime=,args= -p "$(echo $pidler | tr ' ' ',')" 2>/dev/null | cut -c1-110
}

# 0 = temiz durdu, 1 = bir şeyler hayatta kaldı
stop_all() {
    local pidler sinyal
    for sinyal in TERM TERM KILL; do
        pidler=$(_hedef_pidler)
        [ -z "$pidler" ] && break
        kill -"$sinyal" $pidler 2>/dev/null
        sleep 2
    done
    pidler=$(_hedef_pidler)
    if [ -n "$pidler" ]; then
        echo "[HARMONIC] ✗ ÖLDÜRÜLEMEDİ — hâlâ ayakta:"
        _listele "$pidler"
        return 1
    fi
    return 0
}

if [ "${1:-}" = "durum" ] || [ "${1:-}" = "status" ]; then
    pidler=$(_hedef_pidler)
    if [ -z "$pidler" ]; then
        echo "[HARMONIC] Temiz — simülasyona ait süreç yok."
    else
        echo "[HARMONIC] Ayakta olan süreçler:"
        _listele "$pidler"
    fi
    echo "[HARMONIC] Dinlenen portlar:"
    ss -lnup 2>/dev/null | grep -E ':(9002|9012|14541|14542|14550|14551|8000)\b' | sed 's/^/    /' \
        || echo "    (yok)"
    exit 0
fi

if [ "${1:-}" = "stop" ]; then
    echo "[HARMONIC] Durduruluyor..."
    if stop_all; then
        echo "[HARMONIC] ✓ Durduruldu — geriye süreç kalmadı."
        exit 0
    fi
    echo "[HARMONIC] Elle: kill -9 <yukarıdaki PID'ler>"
    exit 1
fi

# ── yeniden: durdur + baştan kur ────────────────────────────────────────────
# NEDEN VAR: VURUŞ SONRASI SİM KULLANILAMAZ HALE GELİYOR. Drone hedefe çarpınca
# uçak enkazı Gazebo fiziğiyle savruluyor (bir kez yerin 1738 m altına gitti) ve
# ArduPlane o durumdan dönmüyor. Panelden senaryo durdurup başlatmak YETMEZ —
# bozukluk model tarafında. Aynı sebeple A/B kolları, biri vurduysa AYNI sim
# oturumunda koşulamaz; ikinci kol taze simde koşulur.
if [ "${1:-}" = "yeniden" ] || [ "${1:-}" = "restart" ]; then
    echo "[HARMONIC] Yeniden kuruluyor (vuruş sonrası sim bozulur — bkz. yorum)"
    stop_all || true
    sleep 2
    shift
    set -- "$@"
fi

# Faz zamanlaması: "bu kadar beklemek şart mı" sorusu ölçümle cevaplanabilsin.
_T0=$SECONDS
_sure() { echo "[HARMONIC] ⏱  $1: +$((SECONDS-_T0)) s"; }

echo "[HARMONIC] Eski süreçler temizleniyor..."
if ! stop_all; then
    echo "[HARMONIC] ✗ Eski süreçler temizlenemedi — YENİSİ BAŞLATILMIYOR."
    echo "[HARMONIC]   (üstteki süreçler ayaktayken port çakışması olur ve"
    echo "[HARMONIC]    bayat telemetri yeni koşuya karışır)"
    exit 1
fi

# Ortam — Harmonic plugin + model yolları + NVIDIA render
source /opt/ros/humble/setup.bash 2>/dev/null
export GZ_SIM_SYSTEM_PLUGIN_PATH="$APGZ/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$PROJ/sim/gazebo_harmonic/models:$APGZ/models:$APGZ/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# 1) Gazebo Harmonic
# ⚠ Aşağıdaki üç başlatmada `setsid ... < /dev/null &` deseni ZORUNLU.
# Eskiden `( cd "$AP" && nohup ... & )` yazıyordu; bu iki `bash
# start_harmonic.sh` alt-kabuğunu hayatta bırakıyordu ve onların fd 1/2'si
# ÇAĞIRANIN borusuna bakıyordu (ölçüldü: /proc/<pid>/fd/1 -> pipe:[56526]).
# Sonuç: `bash scripts/start_harmonic.sh | tee kurulum.log` gibi bir şey
# ASLA dönmüyordu — script bitse bile boru kapanmıyordu.
# setsid: yeni oturum → terminale bağlı değil, Ctrl+C ulaşmaz, terminal
# kapansa da yaşar. </dev/null: stdin'i terminalden koparır.
if [ "${GZ_HEADLESS:-0}" = "1" ]; then
    echo "[HARMONIC] Gazebo (headless) başlatılıyor..."
    unset DISPLAY
    setsid gz sim -s -r --headless-rendering -v2 "$WORLD" \
        > "$LOG/gz_harmonic.log" 2>&1 < /dev/null &
else
    echo "[HARMONIC] Gazebo (GUI, NVIDIA render) başlatılıyor..."
    export DISPLAY="${DISPLAY:-:1}"
    setsid gz sim -r -v2 "$WORLD" > "$LOG/gz_harmonic.log" 2>&1 < /dev/null &
fi

_sure "Gazebo başlatıldı"
echo "[HARMONIC] Gazebo FDM portu (9002) bekleniyor..."
for i in $(seq 1 30); do ss -ln 2>/dev/null | grep -q ':9002' && break; sleep 1; done
_sure "Gazebo FDM portu açıldı"
sleep 3

# 2) ArduCopter SITL (Harmonic JSON FDM)
#
# İlk iki --add-param-file ZORUNLU. Güncel ArduPilot'ta sim_vehicle.py artık
# SITL'e --defaults göndermiyor; SITL frame varsayılanlarını gömülü
# vehicleinfo.json'dan --model anahtarına göre çözüyor. Burada --model JSON
# verildiği için arama anahtarı "JSON" oluyor ve o anahtarın frame varsayılanı
# yok (-f gazebo-iris yalnızca sim_vehicle.py tarafını ilgilendiriyor). Bu iki
# dosya olmadan FRAME_CLASS/FRAME_TYPE tanımsız kalıyor:
#   AP: Frame: UNSUPPORTED
#   AP: PreArm: Motors: Check frame class and type
# ve iris motorlarını yapılandıramadığı için NAV_TAKEOFF başarısız oluyor —
# kovalama görevi hiç başlamıyor. (copter.parm ayrıca INS_ACCOFFS/INS_ACCSCAL
# kalibrasyon işaretlerini ve MOT_THST_HOVER'ı da getiriyor.)
# SIRA ÖNEMLİ: avci_copter.parm en sonda kalmalı ki proje değerleri
# (ANGLE_MAX, WPNAV_SPEED, FS_*) üstte kalsın.
echo "[HARMONIC] ArduCopter (gazebo-iris --model JSON) başlatılıyor..."
# Logları ÖNCE boşalt: hazır-bekleme döngüsü bu dosyalarda işaret arıyor,
# önceki koşunun işareti kalırsa anında "hazır" sanır.
: > "$LOG/copter_harmonic.log"; : > "$LOG/plane_harmonic.log"
( cd "$AP" && exec setsid python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    -I0 --sysid 5 --no-rebuild \
    --add-param-file="$APT/default_params/copter.parm" \
    --add-param-file="$APT/default_params/gazebo-iris.parm" \
    --add-param-file="$PROJ/sim/ardupilot_params/avci_copter.parm" \
    --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=25" ) > "$LOG/copter_harmonic.log" 2>&1 < /dev/null &

# 3) ArduPlane SITL (Gazebo mini_talon_vtail — GERÇEK uçuş, fdm 9012)
#    Talon artık Gazebo'da ArduPilotPlugin ile uçuyor; relay YOK.
echo "[HARMONIC] ArduPlane (gazebo mini_talon --model JSON:9012) başlatılıyor..."
( cd "$AP" && exec setsid python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane \
    --model JSON:127.0.0.1:9012 \
    -I1 --sysid 2 --no-rebuild --add-param-file="$PROJ/sim/ardupilot_params/avci_plane.parm" \
    --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
    --mavproxy-args="--daemon --streamrate=25" ) > "$LOG/plane_harmonic.log" 2>&1 < /dev/null &

# SITL'ler GERÇEKTEN hazır olana kadar bekle.
#
# Eskiden burada kör bir `sleep 25` vardı. İki sorunu vardı: makine yavaşsa
# 25 s yetmiyor ve script "hazır" derken SITL hâlâ açılıyor oluyordu
# (gcs_server bağlanamıyor); makine hızlıysa boşa bekleniyordu.
# Artık aracın kendi çıktısındaki "EKF3 IMU0 is using GPS" işareti aranıyor —
# bu satır göründüğünde EKF oturmuş, GPS kilidi var, MAVLink akıyor demektir.
# Bu, Terminal B'yi (gcs_server) ardışık çalıştırmayı GÜVENLİ kılar: script
# hazır olmadan dönmez ve hazır olamazsa çıkış kodu 1 verir.
_bekle_hazir() {
    local log="$1" ad="$2" i
    for i in $(seq 1 90); do
        if grep -q 'EKF3 IMU0 is using GPS' "$log" 2>/dev/null; then
            echo "[HARMONIC]   ✓ $ad EKF+GPS kilidi (+$((SECONDS-_T0)) s)"
            return 0
        fi
        sleep 1
    done
    echo "[HARMONIC]   ✗ $ad 90 s içinde hazır olmadı — bak: tail -30 $log"
    return 1
}

echo "[HARMONIC] SITL'lerin hazır olması bekleniyor (EKF+GPS kilidi)..."
hazir_ok=0
_bekle_hazir "$LOG/copter_harmonic.log" ArduCopter || hazir_ok=1
_bekle_hazir "$LOG/plane_harmonic.log"  ArduPlane  || hazir_ok=1
if [ "$hazir_ok" != "0" ]; then
    echo "[HARMONIC] ✗ SITL hazır işareti gelmedi — sistem KULLANILABİLİR DEĞİL."
    echo "[HARMONIC]   Temizlemek için: bash scripts/start_harmonic.sh stop"
    exit 1
fi

# (Relay kaldırıldı — Talon Gazebo'da gerçekten uçtuğu için gerek yok.
#  gcs_server hedefi 14542'den kontrol eder: /api/command/plane/square)

# Koşulsuz "hazır" yazmak yanıltıcıydı: SITL açılmasa da aynı satır çıkıyordu.
# Üç bileşen de gerçekten ayakta mı, bakarak söyle.
eksik=""
pgrep -f 'gz sim|gz-sim-server' >/dev/null 2>&1 || eksik="$eksik Gazebo"
pgrep -f 'sim_vehicle.py -v ArduCopter' >/dev/null 2>&1 || eksik="$eksik ArduCopter"
pgrep -f 'sim_vehicle.py -v ArduPlane'  >/dev/null 2>&1 || eksik="$eksik ArduPlane"

echo "=================================================================="
if [ -n "$eksik" ]; then
    echo "[HARMONIC] ✗ BAŞLAMADI:$eksik"
    echo "  Neden için:  tail -30 $LOG/{gz_harmonic,copter_harmonic,plane_harmonic}.log"
    echo "  Temizlemek:  bash scripts/start_harmonic.sh stop"
    echo "=================================================================="
    exit 1
fi
echo "[HARMONIC] ✓ Tam sistem hazır (Gazebo + ArduCopter + ArduPlane ayakta) — toplam $((SECONDS-_T0)) s."
echo "[HARMONIC]   Sürenin ~%95'i ArduPilot'un kendi EKF+GPS kilidi — kısaltılamaz,"
echo "[HARMONIC]   kalkış zaten ondan önce mümkün değil. Gazebo birkaç saniyede hazır."
echo "[HARMONIC]   gcs_server'ı ŞİMDİ başlatın (~5 s sürer). ÖNCE başlatmayın: ölçüldü —"
echo "[HARMONIC]   (1) bu script başlarken onu da öldürür, (2) Gazebo'dan önce açılan"
echo "[HARMONIC]   kamera aboneliği geri gelmiyor, 'ilk görüntü' satırları hiç çıkmıyor."
echo "  Kontrol:   bash scripts/start_harmonic.sh durum"
echo "  Durdurmak: bash scripts/start_harmonic.sh stop"
echo "  Loglar: $LOG/{gz_harmonic,copter_harmonic,plane_harmonic,harmonic_relay}.log"
echo "  Şimdi AYRI terminallerde:"
echo "    cd $PROJ && source /opt/ros/humble/setup.bash && export AVCI_GZ_CAMERA=1 && python3 -m control.gcs_server"
echo "      (AVCI_GZ_CAMERA=1 ŞART — yoksa ROS2 moduna düşer, kamera görüntüsü GELMEZ)"
echo "    bash $PROJ/scripts/start_mission_planner.sh"
echo "=================================================================="
