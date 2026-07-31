# SİMÜLASYON ÇALIŞTIRMA KOMUTLARI

Kurulum tamamlandıktan sonra TÜM sistemi bu dosyadaki komutlarla çalıştırırsınız.
Her blok **ayrı bir terminalde**, buradaki **sırayla** başlatılır.

> **En hızlı yol:** [Tek script ile başlatma](#hızlı-alternatif--tek-script)
> bölümüne geçin. Aşağıdaki elle adımlar, bir şey ters gittiğinde hangi
> bileşenin sorunlu olduğunu görmek için.

---

## 0. Ön kontrol (ilk çalıştırmada bir kez)

```bash
cd ~/projects/avci_sim

# Hangi X ekranındasınız?  (DISPLAY değerini not edin — Terminal 1'de gerekecek)
echo "DISPLAY = $DISPLAY"
ls /tmp/.X11-unix/          # X0 varsa ekranınız :0, X1 varsa :1

# Bağımlılıklar yerinde mi?
ls ~/ardupilot/build/sitl/bin/arducopter ~/ardupilot/build/sitl/bin/arduplane
ls ~/ardupilot_gazebo/build/libArduPilotPlugin.so
gz sim --versions            # 8.x yazmalı
```

---

## Temizlik

Boş bir terminalde (çalışan bileşenlerin olduğu terminalde değil):

```bash
pkill -9 -f 'gz sim|sim_vehicle|mavproxy|arducopter|arduplane|control.gcs_server|run_plane_scenario'
sleep 3

# Portların gerçekten boşaldığını doğrula (çıktı BOŞ olmalı):
ss -lntup | grep -E ':9002|:9012|:1454|:1455|:8000'
```

> **⚠️ `run_plane_scenario` neden listede?** Hedef senaryosu (Kare/Daire/Agresif)
> `gcs_server` tarafından **ayrı bir süreç** olarak başlatılır. GCS çökerse ya da
> düzgün kapanmazsa bu süreç arkada kalır ve **uçağa RC komutu göndermeye devam
> eder**. Yeni oturumda uçağın kendi kendine hareket etmesinin ya da
> komutlarınızın tutmamasının en sık sebebi budur.

---

## TERMİNAL 1 — Gazebo Harmonic (ilk açılır, ~15 sn)

```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
gz sim -r -v4 sim/gazebo_harmonic/worlds/avci_harmonic.sdf
```

> **⚠️ `DISPLAY` ayarlamayın — mevcut değerinizi kullanın.** Daha önce burada
> `export DISPLAY=:1` satırı vardı; tek ekranlı makinelerde (`/tmp/.X11-unix/`
> içinde yalnız `X0` varsa) bu satır Gazebo'nun **hiç açılmamasına** sebep olur:
> ```
> Unable to open display ":1"
> ```
> Yalnızca GUI'niz gerçekten ikinci bir X ekranındaysa (`X1` soketi varsa)
> `export DISPLAY=:1` ekleyin. Emin değilseniz Adım 0'daki `ls /tmp/.X11-unix/`
> çıktısına bakın.

**Hazır olduğunun işareti** (yeni bir terminalde, aynı `GZ_SIM_RESOURCE_PATH` ile):

```bash
ss -lnup | grep -E ':9002|:9012'      # İKİ satır da görünmeli
gz topic -l | grep image              # /iris_cam/image ve /talon_cam/image
```

> Terminal 2'ye geçmeden **9002 portunun açıldığını görün.** Erken başlatırsanız
> ArduCopter Gazebo'ya bağlanamaz ve araç hiç uçmaz.

---

## TERMİNAL 2 — ArduCopter (avcı iris, FDM 9002)

```bash
cd ~/ardupilot
APT=$HOME/ardupilot/Tools/autotest
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  -I0 --sysid 5 --no-rebuild \
  --add-param-file=$APT/default_params/copter.parm \
  --add-param-file=$APT/default_params/gazebo-iris.parm \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_copter.parm \
  --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
  --mavproxy-args="--daemon --streamrate=25"
```

**Hazır olduğunun işaretleri:**
```
AP: Frame: QUAD/X                    ← bu satırı MUTLAKA görün
AP: EKF3 IMU0 is using GPS
```

> **⚠️ İlk iki `--add-param-file` neden zorunlu:** Güncel ArduPilot'ta
> `sim_vehicle.py` artık SITL'e `--defaults` göndermiyor; SITL frame
> varsayılanlarını gömülü `vehicleinfo.json`'dan **`--model` anahtarına göre**
> çözüyor. Burada `--model JSON` verildiği için arama anahtarı `JSON` oluyor ve
> o anahtarın frame varsayılanı yok (`-f gazebo-iris` yalnızca `sim_vehicle.py`
> tarafını ilgilendirir). Bu iki dosya olmadan `FRAME_CLASS`/`FRAME_TYPE`
> tanımsız kalır:
> `AP: Frame: UNSUPPORTED` + `AP: PreArm: Motors: Check frame class and type`
> → iris motorlarını yapılandıramaz, `NAV_TAKEOFF` başarısız olur, kovalama
> görevi hiç başlamaz. Sıra da önemlidir: `avci_copter.parm` en sonda olmalı ki
> kendi ayarları (ANGLE_MAX, WPNAV_SPEED, FS_*) üstte kalsın.

---

## TERMİNAL 3 — ArduPlane (hedef Talon, Gazebo'da gerçek uçuş, FDM 9012)

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 \
  -I1 --sysid 2 --no-rebuild \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_plane.parm \
  --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
  --mavproxy-args="--daemon --streamrate=25"
```

**Hazır olduğunun işareti:** `AP: EKF3 IMU0 is using GPS`

---

## TERMİNAL 4 — GCS Server (kamera + web arayüz + görev)

```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export AVCI_GZ_CAMERA=1
fuser -k 8000/tcp 2>/dev/null
python3 -m control.gcs_server
```

**Hazır olduğunun işaretleri** — bu dört satırı görün:
```
[GCS] YOLO detector hazır (avci_yolo.pt)
[GCS] YOLO pose hazır (avci_pose.pt)
[GCS] ✓ Iris kamerasından ilk görüntü!
[GCS] ✓ Talon (hedef İHA) kamerasından ilk görüntü!
```

Web arayüz: <http://localhost:8000> (otomatik açılır)

> Log'da `MESA-LOADER: failed to open ...` satırları görebilirsiniz — bunlar
> `webbrowser` modülünün tarayıcıyı açarken ürettiği zararsız uyarılardır,
> simülasyonu etkilemez.

---

## TERMİNAL 5 — Mission Planner (isteğe bağlı)

```bash
cd ~/projects/avci_sim/tools/mission_planner
mono MissionPlanner.exe        # UDP 14551'den bağlanın
```

> Önceki sürümde burada `export LD_LIBRARY_PATH=".../native_libs:..."` satırı
> vardı; `native_libs` klasörü Mission Planner dağıtımında **bulunmuyor**, bu
> yüzden kaldırıldı. Eksik kütüphane hatası alırsanız
> `sudo apt-get install -y mono-complete libgdiplus` çalıştırın.

---

## Hızlı alternatif — tek script

Terminal 1-2-3'ü tek komutla başlatır (Gazebo + iki SITL), aralarındaki
beklemeleri ve port kontrolünü kendisi yapar:

```bash
cd ~/projects/avci_sim
bash scripts/start_harmonic.sh              # GUI (mevcut DISPLAY'inizi kullanır)
GZ_HEADLESS=1 bash scripts/start_harmonic.sh # görüntüsüz
bash scripts/start_harmonic.sh stop          # hepsini durdur
```

Ardından **Terminal 4** (gcs_server) ve gerekiyorsa **Terminal 5**
(Mission Planner) ayrı terminallerde başlatılır.

Script'in logları: `logs/gz_harmonic.log`, `logs/copter_harmonic.log`,
`logs/plane_harmonic.log`

---

## Doğrulama — her şey ayakta mı?

Sistem çalışırken boş bir terminalde:

```bash
cd ~/projects/avci_sim

# 1) Süreçler
ps aux | grep -E 'gz sim|arducopter|arduplane|gcs_server' | grep -v grep

# 2) MAVLink gerçekten akıyor mu? (sysid 5 = iris, sysid 2 = Talon)
python3 - <<'EOF'
from pymavlink import mavutil
import time
for port in (14541, 14542, 14550, 14551):
    c = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}', source_system=252)
    seen, son = {}, time.time() + 4
    while time.time() < son:
        m = c.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if m: seen[m.get_srcSystem()] = seen.get(m.get_srcSystem(), 0) + 1
    c.close()
    print(f"  {port}: {seen or 'HEARTBEAT YOK'}")
EOF

# 3) Web arayüz + video
curl -s -o /dev/null -w '  arayuz: %{http_code}\n' http://127.0.0.1:8000/
curl -s -o /dev/null -w '  iris video: %{http_code}\n' --max-time 5 http://127.0.0.1:8000/api/video_feed/iris
curl -s -o /dev/null -w '  talon video: %{http_code}\n' --max-time 5 http://127.0.0.1:8000/api/video_feed/plane

# 4) Telemetri
curl -s http://127.0.0.1:8000/api/debug/telem | python3 -m json.tool | head -20
```

**Beklenen çıktı:**

| Kontrol | Beklenen |
|---------|----------|
| Port 14541 | `{5: N}` — yalnız iris |
| Port 14542 | `{2: N}` — yalnız Talon |
| Port 14550 | `{2: N, 5: M}` — ikisi de (gcs_server) |
| Port 14551 | `{2: N, 5: M}` — ikisi de (Mission Planner) |
| Arayüz / video | `307` (yönlendirme) / `200` |
| Telemetri | iris ≈ `(0, 0, -0.19)`, Talon ≈ `(0, 12, -0.12)` — world'deki spawn konumları |

---

## Sorun giderme

| Belirti | Sebep | Çözüm |
|---------|-------|-------|
| Gazebo penceresi hiç açılmıyor, `Unable to open display ":1"` | `DISPLAY` yanlış ekrana ayarlanmış | `DISPLAY` satırını **silin**; `ls /tmp/.X11-unix/` ile gerçek ekranınızı doğrulayın |
| `AP: Frame: UNSUPPORTED` | İlk iki `--add-param-file` verilmemiş | Terminal 2 komutunu eksiksiz kopyalayın (sıra dâhil) |
| ArduCopter Gazebo'ya bağlanmıyor, araç düşüyor | Terminal 2, Gazebo hazır olmadan başlatılmış | Terminal 1'de `ss -lnup \| grep 9002` iki satır verene kadar bekleyin |
| Uçak kendi kendine hareket ediyor / komutlar tutmuyor | Önceki oturumdan kalan `run_plane_scenario` süreci | Temizlik komutunu çalıştırın (artık bu süreci de öldürüyor) |
| `[GCS] Iris kamerasından ilk görüntü` satırı hiç gelmiyor | `AVCI_GZ_CAMERA=1` verilmemiş veya Gazebo kapalı | Değişkeni export edin; `gz topic -l \| grep image` ile topic'leri doğrulayın |
| `Address already in use` (8000) | Eski gcs_server hâlâ çalışıyor | `fuser -k 8000/tcp` |
| Port 14550'de sysid 5 iki kat fazla heartbeat | `sim_vehicle.py` varsayılan olarak zaten 14550'ye yolluyor, komutta bir kez daha var | Zararsız — yalnızca çift trafik |
