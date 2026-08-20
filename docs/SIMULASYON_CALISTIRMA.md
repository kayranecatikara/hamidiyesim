# SİMÜLASYON ÇALIŞTIRMA

Depo: `~/projects/avci_sim` · ArduPilot: `~/ardupilot` · Gazebo eklentisi:
`~/ardupilot_gazebo` · Log ve koşu scriptleri: `~/.avci_sim`

---

## 1 · Tek komut

**Yeniden kur** — ayakta ne varsa kapatır, sonra kurar. Günlük kullanımda
tek ihtiyacın olan budur:

```bash
cd ~/projects/avci_sim && AVCI_TEMIZ=1 bash scripts/mkur.sh m > ~/.avci_sim/log/kur_m.log 2>&1; tail -1 ~/.avci_sim/log/kur_m.log
```

**Kur** — ayakta sim YOKKEN. (Varsa "sim zaten ayakta" der; o zaman yukarıdaki `AVCI_TEMIZ=1`'li satırı kullanın.)

```bash
cd ~/projects/avci_sim && bash scripts/mkur.sh m > ~/.avci_sim/log/kur_m.log 2>&1; tail -1 ~/.avci_sim/log/kur_m.log
```

Satırın `; tail -1 …` kısmı kurulumun bir parçası değildir — logun son satırını okur, `sim hazır HH:MM:SS` yazıyorsa hazırdır.

**Kapat** — her şeyi indirir (Gazebo + iki SITL + MAVProxy + panel):

```bash
cd ~/projects/avci_sim && bash scripts/kapat.sh
```

**Pencereli Gazebo** (varsayılan headless):

```bash
cd ~/projects/avci_sim && AVCI_TEMIZ=1 AVCI_GUI=1 bash scripts/mkur.sh m > ~/.avci_sim/log/kur_m.log 2>&1; tail -1 ~/.avci_sim/log/kur_m.log
```

Kurulan: Gazebo + ArduCopter (avcı) + ArduPlane (hedef) + panel. **~90 sn.**
Panel: **http://127.0.0.1:8000**

Son satır `sim hazır HH:MM:SS` ise hazırdır. **Çıkış kodu:** 0 = hazır ·
1 = kurulamadı, son satır sebebi söyler. Kör "hazır" yok.

Tek kelimeye indirmek istersen `~/.bashrc`'ye — bunlarda `cd` yok, script
yolu mutlak verildiği için **her dizinden** çalışırlar (`mkur.sh` içeride
kendisi depo köküne geçiyor):

```bash
alias simkur='AVCI_TEMIZ=1 bash ~/projects/avci_sim/scripts/mkur.sh m > ~/.avci_sim/log/kur_m.log 2>&1; tail -1 ~/.avci_sim/log/kur_m.log'
alias simkapat='bash ~/projects/avci_sim/scripts/kapat.sh'
```

| değişken | ne yapar |
|---|---|
| `AVCI_TEMIZ=1` | önce `kapat.sh`, süreçler ve 8000 portu boşalınca kurar |
| `AVCI_GUI=1` | Gazebo penceresini açar (varsayılan `--headless-rendering`) |
| `[etiket]` | 1. argüman; log adlarına ek (`gz_m.log`). Varsayılan `m` |

⚠ `AVCI_TEMIZ` varsayılan olarak **kapalı**: koşan bir kampanyayı ya da
havadaki aracı sessizce düşürmesin diye susturma isteyerek yazılır.

⚠ `mkur.sh`'i **boruya bağlamayın** (`| tail`, `| grep`). Arka plandaki sim
süreçleri yüzünden boru EOF almaz, script asılı kalır. Çıktıyı dosyaya yazın —
yukarıdaki satırlarda `>` sonrası ayrı komutla `tail` alınmasının sebebi budur.

---

## 2 · Uçuş komutları

```bash
cd ~/projects/avci_sim
API=http://127.0.0.1:8000

curl -s -X POST $API/api/command/plane/scenario/duz    # hedefi kaldır + senaryo
curl -s -X POST $API/api/command/iris/start_chase      # avcıyı takibe başlat
curl -s -X POST $API/api/command/plane/stop_scenario   # senaryoyu durdur
curl -s -X POST $API/api/command/iris/stop_chase       # takibi durdur
```

**Senaryolar:** `duz` · `square` · `elips_gorev` · `aggressive` · `circle` ·
`circle_xl` (~96 m) · `circle_l` (~71 m) · `circle_s` (~41 m)

**Kayıt** — saniyede 1 kare + eşli telemetri:

```bash
cd ~/projects/avci_sim && python3 tools/ucus_kaydi.py logs/kayit/<ad> <süre_sn>
```

**Kaçamak testi** — düz uçuşta buluşma anında tetiklenen manevra:

```bash
cd ~/projects/avci_sim && python3 tools/kacamak_testi.py logs/kacamak/<ad> <kacamak> <tetik_m> <kayit_s> <senaryo>
```

Kaçamak türleri: `yatay` · `dikey_yukari` · `dikey_asagi` · `capraz` ·
`hizlan` · `yok` (taban koşusu)

**Video üret:**

```bash
cd ~/projects/avci_sim && ffmpeg -framerate 5 -i logs/kayit/<ad>/frames/f%04d.jpg -c:v libx264 -pix_fmt yuv420p logs/<ad>.mp4
```

---

## 3 · Doğrulama

```bash
curl -sL -o /dev/null -w 'panel: %{http_code}\n' http://127.0.0.1:8000/   # 200 bekle
```

Dört kamera akışı (ikisi ön, ikisi dış görüş):

```bash
for v in iris plane iris_chase talon_chase; do curl -s -o /dev/null -w "$v: %{http_code}\n" --max-time 5 http://127.0.0.1:8000/api/video_feed/$v; done
```

```bash
curl -s http://127.0.0.1:8000/api/debug/telem | python3 -m json.tool | head -20
```

**Hazır olma işaretleri**

| işaret | nerede | ne demek |
|---|---|---|
| `ss -lnu \| grep :9002` | kabuk | Gazebo FDM portu açık, SITL bağlanabilir |
| `AP: Frame: QUAD/X` | ArduCopter | motorlar yapılandırıldı |
| `AP: EKF3 IMU0 is using GPS` | iki SITL | GPS kilidi geldi (~50 sn) |
| `Iris kamerasından ilk görüntü` | GCS logu | ön kamera akıyor (~31 sn, YOLO ısınması) |
| `dış görüş kamerasından ilk görüntü` | GCS logu | AVD/TLD pencereleri dolacak (~6 sn) |

```bash
grep 'ilk görüntü' ~/.avci_sim/log/gcs_m.log     # dört satır bekleriz
```

---

## 4 · Elle çalıştırma (5 terminal)

Bir şey ters gittiğinde hangi bileşenin sorunlu olduğunu görmek için.
Her adımın hazır işaretini görmeden sonrakine geçmeyin.

### Terminal 1 — Gazebo

```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
unset DISPLAY
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
gz sim -s -r --headless-rendering -v4 sim/gazebo_harmonic/worlds/avci_harmonic.sdf
```

`-s` yalnız sunucu · `-r` duraklatmadan başlat · `--headless-rendering`
pencere yok ama **kameralar render edilir** (görsel güdümün şartı).

### Terminal 2 — ArduCopter (avcı, FDM 9002)

```bash
cd ~/ardupilot
APT=$HOME/ardupilot/Tools/autotest
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  -I0 --sysid 5 --no-rebuild -w \
  --add-param-file=$APT/default_params/copter.parm \
  --add-param-file=$APT/default_params/gazebo-iris.parm \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_copter.parm \
  --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 \
  --mavproxy-args="--streamrate=25"
```

⚠ Üç `--add-param-file` de zorunlu ve **sıra önemli**; `avci_copter.parm` en
sonda kalmalı. İlk ikisi olmadan `FRAME_CLASS`/`FRAME_TYPE` tanımsız kalır
(`AP: Frame: UNSUPPORTED`) ve kalkış başarısız olur.

### Terminal 3 — ArduPlane (hedef, FDM 9012)

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 \
  -I1 --sysid 2 --no-rebuild \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_plane.parm \
  --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 \
  --mavproxy-args="--streamrate=25"
```

### Terminal 4 — GCS

```bash
cd ~/projects/avci_sim
export AVCI_GZ_CAMERA=1 AVCI_GORSEL=on AVCI_NO_BROWSER=1
python3 -m control.gcs_server
```

`AVCI_NO_BROWSER=1` şart: sunucu açılıştan 2 sn sonra tarayıcı açmayı dener,
ekransız oturumda bu kilitler.

`AVCI_GZ_CHASE_CAM=0` verilirse dış görüş (chase) kameraları hiç açılmaz —
iki ek 640×480@15 Hz render; zayıf GPU'da kapatmak isteyebilirsiniz.

### Terminal 5 — pencereli Gazebo (isteğe bağlı)

```bash
cd ~/projects/avci_sim
bash scripts/start_harmonic.sh
# ya da yalnız Terminal 1'i şöyle değiştirin:
gz sim -r -v4 sim/gazebo_harmonic/worlds/avci_harmonic.sdf
```

⚠ `unset DISPLAY` satırını çıkarın ama `export DISPLAY=:1` **yazmayın** —
mevcut değerinizi kullanın. Ekranınızı `ls /tmp/.X11-unix/` ile doğrulayın.

---

## 5 · Sorun giderme

| belirti | sebep | çözüm |
|---|---|---|
| `mkur.sh`: "sim zaten ayakta" | önceki simden kalan süreçler | `AVCI_TEMIZ=1` ile kurun (ya da `bash scripts/kapat.sh`) |
| `mkur.sh`: "8000 portu dolu" | eski panel süreci | `fuser -k 8000/tcp` |
| `mkur.sh`: "9002 açılmadı" | Gazebo kalkmadı | `~/.avci_sim/log/gz_<etiket>.log` |
| `mkur.sh`: "GPS kilidi gelmedi" | SITL hatası | `~/.avci_sim/log/cop_*.log`, `pla_*.log` |
| `mkur.sh`: "ön kamera gelmedi" | render bozuk / YOLO yavaş | `grep 'ilk görüntü' ~/.avci_sim/log/gcs_*.log` |
| Panelde AVD/TLD boş | Gazebo dış görüş sensörleri olmadan başlamış | simi yeniden kurun (sensörler dünya yüklenirken okunur) |
| Araç uçmuyor, `Frame: UNSUPPORTED` | param dosyası sırası yanlış | Terminal 2'deki üç dosyayı sırayla verin |
| Kutu hiç gelmiyor | `--headless-rendering` yok | bayrağı ekleyin; yalnız `-s` yetmez |
| Kabuk exit 144 ile ölüyor | `pkill -f` kendi kabuğunu eşliyor | deseni elle yazmayın, `kapat.sh` kullanın |
| Panel donuyor | `gcs_server`'ı tek başına yeniden başlattınız | tam restart yapın |

⚠ **`pkill -9 -f 'gz sim|sim_vehicle|...'` elle yazmayın** — komut satırınız
desenin kendisini içerdiği için kabuğunuzu öldürür. `scripts/kapat.sh`
kullanın; desen ayrı dosyada durduğu için bu tuzağa düşmez. `AVCI_TEMIZ=1` de
aynı sebeple `kapat.sh`'i ayrı dosyadan çağırır.

⚠ Test bitince araçları havada **kontrolsüz bırakmayın**; simi komple kapatın.

⚠ `/tmp` gecelik temizlenebilir. Kritik veriyi `logs/` altına arşivleyin.

---

## 6 · Kampanya koşusu

Kol seçimi:

| kol neyi değiştiriyor | yöntem | teyit |
|---|---|---|
| güdüm alanı (`Cfg.*`) | `AVCI_*` env + tam restart | log CSV'sindeki mekanizma sütunu ≠ 0 |
| araç parametresi (`ATC_*`, `PSC_*`) | MAVLink `PARAM_SET` + geri okuma | okunan değer beklenene eşit |

Her koşu **tam restart** ister — kolun koşu boyunca değişmediğinden ancak
böyle emin olunur.

### Şablon — `~/.avci_sim/kosu.sh`

```bash
#!/bin/bash
# Kullanım: bash kosu.sh <ad> <kol:K|D> [senaryo] [kayit_s]
set -u
AD=$1; KOL=$2; SEN=${3:-aggressive}; KAYIT=${4:-240}
API=http://127.0.0.1:8000
REPO=$HOME/projects/avci_sim
D=$REPO/logs/kayit/$AD
cd $REPO || exit 1

# KOL — güdüm alanıysa env ile
[ "$KOL" = "D" ] && export AVCI_GPS_VZ_ALCALMA=2.0 || export AVCI_GPS_VZ_ALCALMA=6.0

AVCI_TEMIZ=1 bash scripts/mkur.sh "$AD" > ~/.avci_sim/log/kur_$AD.log 2>&1
grep -qa "sim hazır" ~/.avci_sim/log/kur_$AD.log || {
  echo "[$AD] HATA: sim kurulamadı"; tail -3 ~/.avci_sim/log/kur_$AD.log; exit 1; }

T0=$(date +%s)
curl -s -m 10 -X POST $API/api/command/plane/scenario/$SEN > /dev/null
curl -s -m 10 -X POST $API/api/command/iris/start_chase   > /dev/null
mkdir -p "$D"
python3 tools/ucus_kaydi.py "$D" "$KAYIT" > ~/.avci_sim/log/kayit_$AD.log 2>&1
bash scripts/kapat.sh > /dev/null 2>&1

# Faz her GPS↔görsel geçişinde YENİ log açar — koşu başına tek dosya YOKTUR.
for p in $REPO/logs/gps_guidance_*.csv; do
  [ -e "$p" ] || continue
  [ "$(stat -c %Y "$p")" -ge "$T0" ] && cp "$p" "$D/"
done
# Kara kutu — duruş/motor teşhisi burada
for b in $(ls -t $HOME/ardupilot/logs/*.BIN 2>/dev/null | head -4); do
  [ "$(stat -c %Y "$b")" -ge "$T0" ] || continue
  head -c 200000 "$b" | grep -qa "ArduCopter" && { cp "$b" "$D/kopter.BIN"; break; }
done
echo "$KOL $SEN" > "$D/KOL.txt"
echo "[$AD] bitti"
```

### Araç parametresi yazma + teyit

```bash
python3 - <<'EOF'
from pymavlink import mavutil
import time, sys
AD, DEGER = 'ATC_RAT_RLL_P', 0.135
m = mavutil.mavlink_connection('udp:127.0.0.1:14541'); m.wait_heartbeat(timeout=20)
m.mav.param_set_send(m.target_system, 1, AD.encode(),
                     float(DEGER), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
m.mav.param_request_read_send(m.target_system, 1, AD.encode(), -1)
t0 = time.time()
while time.time() - t0 < 10:
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg and msg.param_id.strip('\x00') == AD:
        ok = abs(msg.param_value - DEGER) <= abs(DEGER) * 0.02
        print(f"{AD} = {msg.param_value}  {'TEYİT' if ok else 'TUTMADI'}")
        sys.exit(0 if ok else 1)
print("PARAM_VALUE gelmedi — koşu GEÇERSİZ"); sys.exit(1)
EOF
```

⚠ `MAV_PARAM_TYPE_REAL32` kullanın; `INT32` ondalık değerleri sıfırlayabilir.

⚠⚠ **Bu firmware'de olmayan parametreler var** (ArduCopter 4.8-dev, ölçüldü
2026-08-19): `ATC_ACCEL_R_MAX` · `ATC_ACCEL_P_MAX` · `ATC_ACCEL_Y_MAX` ·
`ATC_SLEW_YAW`. `avci_copter.parm` bu satırları yazıyor, ArduPilot **sessizce
yok sayıyor**. Bir parametreyi kola koymadan önce yukarıdaki geri-okumayla
var olduğunu doğrulayın; yoksa deney kolu fiilen kontrol koşusu olur.

---

## 7 · Analiz araçları

| araç | ne yapar |
|---|---|
| `tools/ucus_kaydi.py` | kare + eşli telemetri kaydı (`meta.csv`) |
| `tools/kacamak_testi.py` | tetiklenmiş kaçamak A/B kampanyası |
| `tools/ucus_bekci.py` | uçuş sırasında geçerlilik bandı denetimi |
| `tools/vurus_kalitesi.py` | vuruşu KONTROLLÜ / ŞANS diye sınıflar |
| `tools/tf_analiz.py` | tek faz kampanyası ölçütleri |
| `tools/takla_analiz.py` | KURTARMA payı, takla sayısı, duruş zarfı |

**Log yerleri**

| ne | nerede |
|---|---|
| GPS güdüm CSV | `logs/gps_guidance_*.csv` |
| Görsel güdüm CSV | `logs/bbox_ibvs_*.csv` |
| Kurulum/çalışma logları | `~/.avci_sim/log/` |
| ArduPilot kara kutusu | `~/ardupilot/logs/*.BIN` |
