# AVCI SİM — Teknofest Avcı İHA Hava-Hava Müdahale Simülasyonu

BismillahirRahmanirRahim.

Bir **avcı multikopterin** (iris) sabit kanatlı bir **hedef İHA'yı** (mini Talon)
tespit edip kovaladığı ve kamikaze müdahale ettiği hava-hava simülasyonu.
Her iki araç da **ArduPilot SITL** ile **Gazebo Harmonic** fiziğinde gerçekten uçar;
kontrol, görüntü işleme ve görev arayüzü tek bir web tabanlı Yer Kontrol İstasyonu
(gcs_server) üzerinden yönetilir.

> **Stack:** ArduPilot (ArduCopter + ArduPlane) · Gazebo Harmonic (gz-sim 8) ·
> gz-transport · ROS 2 Humble · OpenCV · FastAPI · pymavlink · Mission Planner

---

## Mimari

| Araç | Rol | Model | ArduPilot | FDM portu | Kontrol portu | Kamera |
|------|-----|-------|-----------|-----------|---------------|--------|
| **iris** | Avcı drone | `iris_cam` | ArduCopter (`-I0`, sysid 5) | 9002 | 14541 | `/iris_cam/image` |
| **Talon** | Hedef İHA | `mini_talon_vtail` | ArduPlane (`-I1`, sysid 2) | 9012 | 14542 | `/talon_cam/image` |

- **iris** GUIDED modda pozisyon setpoint'leriyle uçar; kamerasından hedefi
  (HSV renk tespiti) bulur, SPRINT→APPROACH→LOCK→STRIKE durum makinesiyle kovalar,
  Oransal Seyrüsefer (Proportional Navigation) ile müdahale eder.
- **Talon** ArduPlane ile Gazebo'da uçar; gcs arayüzündeki **"Kare Çiz"** butonu
  onu otonom kalkırıp (TAKEOFF modu) kare rota çizdirir.
- **gcs_server** iki kamerayı da gz-transport'tan okur, MJPEG olarak web arayüzüne
  akıtır, telemetriyi toplar ve görev komutlarını yollar → `http://localhost:8000`

---

## Sistem Gereksinimleri

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic (gz-sim 8)
- Python 3.10+
- (Önerilen) NVIDIA GPU — Gazebo kamera render'ı için

---

## Kurulum

### 1) Gazebo Harmonic + gz-transport (Python)
```bash
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update
sudo apt-get install -y gz-harmonic python3-gz-transport13 python3-gz-msgs10
```

### 2) ROS 2 Humble
Resmi kılavuz: <https://docs.ros.org/en/humble/Installation.html>
(cv_bridge kamera köprüsü Classic fallback içindir; Harmonic'te gz-transport
doğrudan kullanılır ama `rclpy` yine de import edilir.)

### 3) ArduPilot SITL
```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
./waf plane
```

### 4) ArduPilot Gazebo eklentisi (Harmonic)
```bash
sudo apt-get install -y libgz-sim8-dev rapidjson-dev
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

### 5) Bu depo + Python paketleri
```bash
git clone <bu-depo-url> ~/projects/avci_sim
cd ~/projects/avci_sim
pip install -r requirements.txt
```

### 6) Mission Planner
```bash
sudo apt-get install -y mono-complete libgdiplus
bash scripts/setup_mission_planner.sh   # binary'yi tools/mission_planner/ altına indirir
```

### 7) Ortam değişkenleri (her Gazebo terminaline ya da ~/.bashrc'ye)
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
```

---

## Çalıştırma

Her komut **ayrı bir terminalde**. Sırayla başlatın (önce Gazebo).

**Temizlik** (boş bir terminalde):
```bash
pkill -9 -f 'gz sim|sim_vehicle|mavproxy|arducopter|arduplane|control.gcs_server'; sleep 3
```

**Terminal 1 — Gazebo Harmonic** (açılması ~15 sn)
```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
gz sim -r -v4 sim/gazebo_harmonic/worlds/avci_harmonic.sdf
```

**Terminal 2 — ArduCopter (avcı iris, FDM 9002)**
```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --sysid 5 --no-rebuild --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_copter.parm --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
```

**Terminal 3 — ArduPlane (hedef Talon, FDM 9012)**
```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 -I1 --sysid 2 --no-rebuild --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_plane.parm --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
```

**Terminal 4 — GCS Server (kamera + web arayüz + görev)**
```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export AVCI_GZ_CAMERA=1
python3 -m control.gcs_server
```

**Terminal 5 — Mission Planner**
```bash
cd ~/projects/avci_sim/tools/mission_planner
export LD_LIBRARY_PATH="$PWD/native_libs:$LD_LIBRARY_PATH"
mono MissionPlanner.exe    # UDP 14551
```

> GUI ikinci bir X ekranındaysa Terminal 1'de `export DISPLAY=:1` ekleyin.

---

## Kullanım

1. `http://localhost:8000` otomatik açılır (YKİ — Taktik Saha Ekranı).
2. Kamera görünümü: **AVCI DRONE** sekmesi = iris kamerası, **HEDEF İHA** sekmesi = Talon burun kamerası.
3. **Kare Çiz** → Talon kalkıp kare rota çizer. **Manuel Mod** → RC benzeri kontrol.
4. iris kamerası hedefi görünce tespit + kilit overlay'i çizilir; chase/strike telemetriyle sürülür.
5. **GPS Karıştırma** kaydırıcısı video/telemetri bozulmasını simüle eder.

---

## Port Haritası

| Port | Kullanım |
|------|----------|
| 9002 / 9012 | Gazebo ↔ SITL FDM (iris / Talon) |
| 14541 / 14542 | iris / Talon kontrol (pymavlink) |
| 14550 | gcs_server telemetri (udpin) |
| 14551 | Mission Planner |
| 8000 | Web arayüz + MJPEG |

---

## Proje Yapısı

```
avci_sim/
├── control/               # MAVLink kontrol + gcs_server (web/kamera/görev)
│   ├── mav_common.py          # Ortak ArduPilot MAVLink altyapısı
│   ├── drone_functions.py     # iris (ArduCopter) kontrol
│   ├── plane_functions.py     # Talon (ArduPlane) kontrol
│   ├── plane_patterns.py      # Kalkış + kare deseni
│   ├── chase_algorithm.py     # SPRINT→APPROACH→LOCK→STRIKE
│   ├── strike_algorithm.py    # Oransal Seyrüsefer (PN)
│   └── gcs_server.py          # FastAPI + gz-transport kamera + görev API
├── vision/                # OpenCV HSV hedef tespiti
├── sim/
│   ├── gazebo_harmonic/       # Harmonic world + modeller (iris_cam, mini_talon_vtail)
│   └── ardupilot_params/      # avci_copter.parm, avci_plane.parm
├── scripts/               # start_harmonic.sh, setup/start_mission_planner.sh
├── tools/mission_planner/ # (git'e dahil değil — setup script ile kurulur)
├── requirements.txt
└── docs/ARDUPILOT_MIGRATION.md
```

---

## Notlar

- **Harici bağımlılıklar** (`~/ardupilot`, `~/ardupilot_gazebo`) bu depoda değil,
  yukarıdaki kurulum adımlarıyla ayrıca kurulur.
- iris ve Talon **ayrı FDM portları** (9002/9012) kullanır — aynı world'de çakışmaz.
- Talon V-kuyruk servo eşlemesi (SERVO2/4 = Sol/Sağ V-Tail) `avci_plane.parm`'dadır.
- Ayrıntılı geçiş notları: `docs/ARDUPILOT_MIGRATION.md`.

---

## 🤖 Claude Code ile Sıfırdan Otomatik Kurulum

Elle kurulumla uğraşmak istemiyorsanız: temiz bir **Ubuntu 22.04** makinede bu depoyu
`~/projects/avci_sim`'e klonlayın, sonra aşağıdaki prompt'u **Claude Code**'a yapıştırın.
Claude Code bütün bağımlılıkları (ROS 2, Gazebo Harmonic, ArduPilot, ardupilot_gazebo,
Python paketleri, Mission Planner) kurar, her adımı doğrular ve sistemi çalıştırır.
Sizin tek yapmanız gereken, istendiğinde **sudo şifrenizi** girmek.

````text
Sen "AVCI SİM" projesini Ubuntu 22.04 üzerinde SIFIRDAN kuran otonom bir DevOps
kurulum ajanısın. Aşağıdaki adımları SIRAYLA uygula. HER adımı komut çıktısıyla
DOĞRULA; bir adım başarısız olursa nedenini bul, düzelt, sonra devam et. Yalnızca
sudo şifresi / interaktif giriş gerektiğinde kullanıcıya sor — gerisini kendin yap.

PROJE BAĞLAMI:
Teknofest Avcı İHA hava-hava müdahale simülasyonu. Avcı multikopter (iris/ArduCopter)
ile hedef sabit kanat (mini Talon/ArduPlane); ikisi de Gazebo Harmonic fiziğinde
ArduPilot SITL ile GERÇEKTEN uçar. Kontrol, kamera işleme ve görev arayüzü tek bir
web GCS'de (control/gcs_server.py, http://localhost:8000).

ÖN KOŞUL:
Ubuntu 22.04, internet erişimi, sudo yetkisi. BOŞ bir dizinde başlıyor olabilirsin;
depoyu ADIM 0 kendisi klonlar. Kurulum boyunca zaten kurulu bileşenleri (klasör/binary
mevcutsa) tekrar kurma, atla.

--- ADIM 0: DEPOYU KLONLA ---
Depo yoksa klonla, varsa güncelle:
[ -d ~/projects/avci_sim/.git ] && (cd ~/projects/avci_sim && git pull) || git clone https://github.com/kayranecatikara/hamidiyesim.git ~/projects/avci_sim
cd ~/projects/avci_sim
DOĞRULA: ls ~/projects/avci_sim/control/gcs_server.py çıktı vermeli.

--- ADIM 1: SİSTEM PAKETLERİ ---
sudo apt-get update
sudo apt-get install -y git build-essential cmake python3-pip wget curl lsb-release gnupg unzip
DOĞRULA: gcc --version ve cmake --version çıktı vermeli.

--- ADIM 2: ROS 2 HUMBLE ---
Kontrol: [ -f /opt/ros/humble/setup.bash ] && echo "ROS2 VAR" || echo "KURULACAK"
Yoksa resmi Debian kurulumunu uygula (locale UTF-8; universe deposu; ROS 2 apt anahtarı
+ deposu; sudo apt-get install -y ros-humble-desktop ros-humble-cv-bridge python3-colcon-common-extensions).
Rehber: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
DOĞRULA: source /opt/ros/humble/setup.bash && ros2 --version

--- ADIM 3: GAZEBO HARMONIC + gz-transport (Python) ---
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update
sudo apt-get install -y gz-harmonic python3-gz-transport13 python3-gz-msgs10
DOĞRULA: gz sim --versions  ->  8.x yazmalı.

--- ADIM 4: ARDUPILOT SITL (ArduCopter + ArduPlane) ---
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
./waf plane
DOĞRULA: ls ~/ardupilot/build/sitl/bin/arducopter ~/ardupilot/build/sitl/bin/arduplane

--- ADIM 5: ARDUPILOT GAZEBO EKLENTİSİ (Harmonic) ---
sudo apt-get install -y libgz-sim8-dev rapidjson-dev
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
DOĞRULA: ls ~/ardupilot_gazebo/build/libArduPilotPlugin.so

--- ADIM 6: PYTHON PAKETLERİ ---
cd ~/projects/avci_sim
pip install -r requirements.txt
DOĞRULA: python3 -c "import cv2,numpy,fastapi,uvicorn,pymavlink; print('PY OK')"

--- ADIM 7: MISSION PLANNER ---
sudo apt-get install -y mono-complete libgdiplus
bash ~/projects/avci_sim/scripts/setup_mission_planner.sh
DOĞRULA: ls ~/projects/avci_sim/tools/mission_planner/MissionPlanner.exe

--- ADIM 8: ORTAM DEĞİŞKENLERİ ---
~/.bashrc'de yoksa şu iki satırı ekle:
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds

--- ADIM 9: KURULUM DOĞRULAMASI (kanıtla) ---
Ortam değişkenlerini export ettikten ve /opt/ros/humble/setup.bash source ettikten sonra:
(a) Gazebo world + iki kamera:
    gz sim'i arka planda başlat (gz sim -s -r <world> &), 9 sn bekle,
    gz topic -l | grep -E 'iris_cam/image|talon_cam/image'  -> İKİSİ de görünmeli.
    (world: ~/projects/avci_sim/sim/gazebo_harmonic/worlds/avci_harmonic.sdf)
    ÖNEMLİ: gz sim'i öldürürken 'kill <PID>' kullan; gz sim'i başlatan komutun içinde
    'pkill -f "gz sim"' KULLANMA — pkill komut satırında 'gz sim' geçtiği için kendi
    shell'ini de öldürür.
(b) SITL bağlantısı: Gazebo açıkken README "Terminal 2" komutuyla ArduCopter'ı başlat,
    çıktıda "EKF3 ... active" / GPS kilidi gör.
(c) GCS kamera: AVCI_GZ_CAMERA=1 ile control.gcs_server'ı başlat; log'da
    "Iris kamerasından ilk görüntü" ve "Talon kamerasından ilk görüntü" satırlarını gör.

--- BİTİRİŞ ---
README'nin "Çalıştırma" bölümündeki 5 terminali sırayla başlat ve http://localhost:8000
arayüzünde HEM "AVCI DRONE" HEM "HEDEF İHA" sekmelerinde kamera görüntüsünün geldiğini
doğrula. Kullanıcıya kısa bir kurulum özeti + neyin doğrulandığını raporla.
````
