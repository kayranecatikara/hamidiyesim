# AVCI SİM — CLAUDE MASTER PROMPT

BismillahirRahmanirRahim.

Bu prompt, "Avcı Drone Simülasyonu" projesinin tüm teknik bağlamını içerir. Bu projeyi devralıp aşağıda belirtilen 3 büyük migrasyon görevini gerçekleştireceksin. Projenin her dosyasını, her algoritmasını, her bağlantı noktasını biliyormuş gibi devam et.

---

## 1) PROJENİN AMACI

Teknofest 2026 "Avcı İHA" yarışması için otonom hava-hava müdahale simülasyonu. Bir avcı drone (multicopter — iris), düşman sabit kanatlı bir hedef uçağı (şu an "plane" olarak adlandırılan model) tespit edip takip ediyor ve vurma (kamikaze) manevrası yapıyor.

**Mevcut çalışan sistem:**
- PX4 + Gazebo Classic + ROS 2 Humble + pymavlink
- İki araç: iris (multicopter, sysid=5) + plane (fixed-wing, sysid=2)
- Kamera: Her iki araçta ROS kamera sensörü (iris_cam, plane_cam)
- GCS: FastAPI + WebSocket tabanlı özel web arayüzü (port 8000)
- Takip algoritması: 4-fazlı state machine (SPRINT→APPROACH→LOCK→STRIKE)
- Vurma algoritması: Proportional Navigation güdümü
- Renk tabanlı hedef tespiti (HSV maskeleme)

**YAPILACAK 3 BÜYÜK DEĞİŞİKLİK:**

| # | Mevcut | Hedef | Neden |
|---|--------|-------|-------|
| 1 | PX4 Autopilot | **ArduPilot (ArduCopter + ArduPlane)** | Daha stabil SITL, daha iyi fixed-wing desteği |
| 2 | Talon modeli (plane) | **Cessna modeli** | Talon mesh'i kötü, Cessna zaten PX4 repo'sunda mevcut |
| 3 | QGroundControl | **Mission Planner** | Yarışma gereksinimleri |

---

## 2) SİSTEM ENVANTER BİLGİSİ

### 2.1 Sistemde Kurulu Olan Şeyler

```
OS: Linux (Ubuntu)
ROS 2: Humble
Gazebo: Classic (gazebo11)
Python: 3.x (pymavlink, opencv, fastapi, uvicorn, cv_bridge, numpy)
PX4-Autopilot: ~/projects/avci_sim/PX4-Autopilot (release/1.16)
ArduPilot: ~/ardupilot (ZATEN KURULU VE BUILD EDİLMİŞ)
  - sim_vehicle.py: ~/ardupilot/Tools/autotest/sim_vehicle.py
  - arduplane binary: ~/ardupilot/build/sitl/bin/arduplane
  - arducopter binary: ~/ardupilot/build/sitl/bin/arducopter
  - MAVProxy: pip ile kurulu (v1.8.71)
Micro-XRCE-DDS-Agent: ~/projects/avci_sim/Micro-XRCE-DDS-Agent (PX4↔ROS2 bridge)
Cessna mesh: ~/projects/avci_sim/PX4-Autopilot/Tools/simulation/jmavsim/jMAVSim/models/cessna.obj + cessna.mtl
Cessna SDF (Gazebo): ~/projects/avci_sim/PX4-Autopilot/Tools/simulation/gz/models/rc_cessna/model.sdf
Mission Planner: KURULU DEĞİL — mono runtime da yok
```

### 2.2 ArduPilot Frame Seçenekleri (sim_vehicle.py)

ArduPlane frameleri: `plane`, `plane-3d`, `plane-dspoilers`, `plane-elevon`, `plane-ice`, `plane-jet`, `gazebo-zephyr`, `jsbsim`, `glider`, vb.
ArduCopter frameleri: `quad`, `hexa`, `octa`, `X`, `gazebo-iris`, vb.

### 2.3 Proje Kök Dizini: `~/projects/avci_sim/`

---

## 3) TÜM DOSYALAR VE GÖREVLERİ

### 3.1 `control/` — Ana Kontrol Katmanı

| Dosya | Satır | Görev |
|-------|-------|-------|
| `mav_common.py` | 269 | Ortak MAVLink altyapısı: `connect_mavlink()`, `GCSKeepalive`, `arm()`, `disarm()`, `set_mode()`, `wait_ack()`, `get_local_position()`, `get_attitude()`. **PX4 custom mode sabitleri burada** (OFFBOARD=6, MANUAL=1, STABILIZED=7, AUTO=4). ArduPilot'a geçişte bu sabitlerin **tamamı değişmeli**. |
| `drone_functions.py` | 345 | Iris multicopter kontrol: `connect_drone(port=14541)`, `takeoff_to_z()`, `set_offboard_mode()`, `hold_position()`, `move_forward/backward/left/right/up/down()`, `yaw_left/right()`, `land_drone()`. **PX4 OFFBOARD modu kullanıyor** → ArduPilot'ta GUIDED moda çevrilmeli. `_send_position_setpoint()` → `SET_POSITION_TARGET_LOCAL_NED` ile çalışıyor, ArduPilot GUIDED modda da bu mesajı destekler ama typemask ve frame farklılıkları var. |
| `plane_functions.py` | 342 | Fixed-wing kontrol: `connect_plane(port=14542)`, `start_gcs_keepalive()`, `arm_plane()`, `send_manual_control()` (RC override ile), `set_throttle()`, `set_heading()`, `set_pitch()`, `set_roll()`, `fly_forward()`, `turn_left/right()`, `climb()`, `descend()`, `loiter()`. **PX4'ün MANUAL(1) ve STABILIZED(7) modlarını kullanıyor** → ArduPilot'ta MANUAL, STABILIZE, FBWA, FBWB, GUIDED modlarına çevrilmeli. |
| `plane_patterns.py` | 324 | Scripted manevralar: `takeoff_then_stabilize()`, `draw_square()`, `draw_rectangle()`, `circle()`, `zigzag()`, `aggressive_maneuver_1/2/3()`, `demo_basic/aggressive/mixed()`. PX4 AUTO TAKEOFF modunu (main=4, sub=2) kullanıyor → ArduPilot'ta `MAV_CMD_NAV_TAKEOFF` veya TAKEOFF moduna çevrilmeli. |
| `chase_algorithm.py` | 446 | **TAKİP ALGORİTMASI** (v2). State machine: SPRINT→APPROACH→LOCK→STRIKE. Pinhole kamera modeli ile mesafe tahmini. Aspect angle hesabı. **X-UAV Talon'a özel sabitler var**: `PLANE_WINGSPAN_M=1.718`, `PLANE_FUSELAGE_M=1.20`. Kamera FOV=125°, tilt=25° yukarı. PID kontrolcü (mod-bazlı Kp). `SET_POSITION_TARGET_LOCAL_NED` ile setpoint gönderiyor. **Cessna'ya geçişte wingspan/fuselage sabitleri güncellenecek.** |
| `strike_algorithm.py` | 276 | **VURMA ALGORİTMASI**. Proportional Navigation güdümü. İki faz: APPROACH (PN) → TERMINAL (pure pursuit). EMA filtre + closing velocity hesabı. Velocity setpoint gönderiyor. **Autopilot-agnostic** — sadece MAVLink velocity komutu, PX4'e özel bir şey yok. |
| `gcs_server.py` | 977 | **ANA GCS SUNUCUSU**. FastAPI + WebSocket. Tüm sistemi orkestre eder. İçerdiği özellikler: telemetri toplama (14550 port, 14541 port), kamera stream (ROS2→MJPEG), GPS karıştırma simülasyonu, video parazit simülasyonu, PnP pose tahmini simülasyonu, manuel kontrol (RC override), kare uçuş komutu, chase/strike mod yönetimi, plane throttle slider. **Talon visual offset sabitleri var** (VISUAL_OFFSET_BX=0.58, BY=-0.811, BZ=0.04) — Cessna'da bu değişecek. Port 14550 üzerinden plane MAVLink dinliyor, sysid ile FixedWing/Quadrotor ayırıyor. |
| `gcs_ui/index.html` | 13374 bytes | GCS web arayüzü HTML |
| `gcs_ui/script.js` | 27099 bytes | GCS frontend JS — telemetri gösterimi, chase/strike kontrolleri, joystick, sliders |
| `gcs_ui/style.css` | 21361 bytes | GCS frontend stil |

### 3.2 `vision/` — Görüntü İşleme

| Dosya | Satır | Görev |
|-------|-------|-------|
| `color_detector.py` | 81 | Talon renk tespiti: HSV maskeleme (beyaz/gri), morfolojik operasyonlar, kontur analizi. `detect_talon()` → bbox + merkez döndürür. **"UCAK" etiketi kullanıyor, Talon'un beyaz rengine göre ayarlı** — Cessna'nın rengine göre HSV değerleri güncellenecek. |
| `detection_state.py` | 18 | Thread-safe detection sonuç paylaşımı: `set_detection()`, `get_detection()`. |

### 3.3 Demo/Test Scriptleri (`control/` içinde)

| Dosya | Görev |
|-------|-------|
| `run_drone_takeoff.py` | Drone kalkış + hover testi |
| `run_drone_hover.py` | Drone hareket + yaw testi |
| `run_plane_arm.py` | Plane keepalive + arm testi |
| `run_plane_square.py` | Plane kare deseni testi |
| `run_plane_aggressive.py` | Plane agresif manevralar testi |
| `run_dual_demo.py` | İki araç eş zamanlı test |

### 3.4 Diğer Dosyalar (Kök dizin)

| Dosya | Görev |
|-------|-------|
| `iris_hover.py` | Eski basit hover scripti |
| `arm_plane_direct.py` | Eski plane arm denemesi |
| `fix_failsafes.sh` | PX4 failsafe parametre düzeltme |
| `fix_params.sh` | PX4 parametre düzeltme |
| `fix_plane_params.py` | PX4 plane parametre düzeltme |
| `health_check.py` | Sistem sağlık kontrolü |
| `merge_plane_to_stl.py` | Talon mesh birleştirme |
| `prop_nose_align.py` | Talon pervane hizalama |

### 3.5 Model/SDF Dosyaları

```
PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
  iris/iris.sdf.jinja        ← Drone modeli (ROS kamera eklenmiş)
  iris/iris.sdf.jinja.bak    ← Yedek
  plane/plane.sdf.jinja      ← Talon modeli (ROS kamera eklenmiş)  
  plane/plane.sdf.jinja.bak  ← Yedek
```

---

## 4) MEVCUT PORT HARİTASI (PX4 SITL)

| Araç | Onboard MAVLink | Sim TCP | Source Sys ID | Kamera Namespace |
|------|-----------------|---------|---------------|------------------|
| iris | udpin:127.0.0.1:14541 | 4561 | 250 (script), 5 (PX4) | /iris_cam |
| plane | udpin:127.0.0.1:14542 | 4562 | 251 (script), 2 (PX4) | /plane_cam |
| GCS broadcast | udpin:0.0.0.0:14550 | — | — | — |

**ArduPilot SITL'de portlar farklı olacak:**
- ArduPilot SITL varsayılan: 5760 (MAVProxy), 5762-5763 (ekstra GCS), 14550 (UDP broadcast)
- İki araç için `--instance` ile port offsetleri: instance 0 → 5760, instance 1 → 5770 vb.
- Veya `--master` ile özel port belirlenebilir

---

## 5) MEVCUT ÇALIŞMA AKIŞI (PX4)

```bash
# Terminal 1: Simülasyon
pkill -f 'px4|gzserver|gzclient|gazebo|MicroXRCEAgent' || true
cd ~/projects/avci_sim/PX4-Autopilot
bash Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s "iris:1,plane:1" -w empty

# Terminal 2: XRCE Agent (PX4 → ROS 2 bridge)
~/projects/avci_sim/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888

# Terminal 3: ROS 2
source /opt/ros/humble/setup.bash
source ~/projects/avci_sim/ros2_ws/install/setup.bash

# Terminal 4: GCS Server
cd ~/projects/avci_sim
python -m control.gcs_server
```

---

## 6) KRİTİK ALGORİTMA DETAYLARI

### 6.1 Chase Algorithm (chase_algorithm.py)

**State Machine:**
```
SPRINT (>30m)  → Tam hız ile hedefe koş, KP=7.0
APPROACH (10-30m) → Azalan hızla yaklaş, KP=5.0  
LOCK (d_lock ±30%) → Hedefin arkasında stabilize, KP=2.0
STRIKE (<5m) → Kamikaze dalışı, KP=7.0
```

**Pinhole Kamera Modeli:**
- Lock mesafesi dinamik: `d = visible_width / (2 * target_ratio * tan(FOV/2))`
- Aspect angle'a göre görünen genişlik değişir (kanat vs gövde)
- Kamera 25° yukarı tiltli → vertical offset hesabı var

**PID + Feedforward:** Hedef hızı feedforward olarak ekler, PID hatayı düzeltir.

**Cessna'ya geçişte değişecekler:**
- `PLANE_WINGSPAN_M` = Cessna kanat açıklığı
- `PLANE_FUSELAGE_M` = Cessna gövde uzunluğu
- Kamera FOV ve tilt aynı kalabilir (drone tarafı)

### 6.2 Strike Algorithm (strike_algorithm.py)

**Proportional Navigation:**
- LOS (Line of Sight) rate hesabı
- `a_cmd = N * Vc * LOS_rate` (N=4.0)
- İki faz: APPROACH (PN güdüm) → TERMINAL (<3m, pure pursuit)
- **Autopilot-agnostic** — sadece velocity setpoint

### 6.3 Vision (color_detector.py)

- HSV maskeleme: Hue=[0,180], Sat=[0,25], Val=[120,255] → beyaz/gri nesneler
- Bit dilimleme (gürültü azaltma), dilate, morphological close
- En büyük kontur = hedef uçak
- **Cessna'nın rengi farklı olabilir** → HSV değerleri güncellenecek

---

## 7) MİGRASYON TALİMATLARI

### GÖREV 1: PX4 → ArduPilot Geçişi

**Etkilenen dosyalar ve değişiklikler:**

**A) `mav_common.py` — MOD SABİTLERİ:**
```python
# PX4 (ESKİ):
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
PX4_CUSTOM_MAIN_MODE_AUTO = 4

# ArduPilot (YENİ) — MAV_CMD_DO_SET_MODE ile custom_mode:
# ArduCopter modları:
COPTER_MODE_STABILIZE = 0
COPTER_MODE_GUIDED = 4
COPTER_MODE_LOITER = 5
COPTER_MODE_RTL = 6
COPTER_MODE_LAND = 9

# ArduPlane modları:
PLANE_MODE_MANUAL = 0
PLANE_MODE_STABILIZE = 2
PLANE_MODE_FBWA = 5
PLANE_MODE_FBWB = 6
PLANE_MODE_GUIDED = 15
PLANE_MODE_LOITER = 12
PLANE_MODE_RTL = 11
PLANE_MODE_TAKEOFF = 13
```

**B) `mav_common.py` — `set_mode()` fonksiyonu:**
PX4'te `MAV_CMD_DO_SET_MODE` + `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` + `main_mode` + `sub_mode` gönderiliyor.
ArduPilot'ta: `MAV_CMD_DO_SET_MODE` + `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` + `custom_mode` (tek parametre, sub_mode yok).

**C) `drone_functions.py`:**
- `set_offboard_mode()` → `set_guided_mode()` olacak
- ArduPilot GUIDED modda `SET_POSITION_TARGET_LOCAL_NED` destekleniyor
- ARM: ArduPilot'ta force arm için `MAV_CMD_COMPONENT_ARM_DISARM` param2=2989 (PX4'te 21196 idi)
- Takeoff: ArduPilot'ta `MAV_CMD_NAV_TAKEOFF` komutu veya `GUIDED` modda position setpoint

**D) `plane_functions.py`:**
- PX4 MANUAL(1) → ArduPilot MANUAL(0)
- PX4 STABILIZED(7) → ArduPilot STABILIZE(2) veya FBWA(5)
- RC override aynı MAVLink mesajı, ArduPilot'ta da çalışır
- GCS keepalive: ArduPilot'ta da gerekli olabilir

**E) `plane_patterns.py`:**
- `takeoff_then_stabilize()`: PX4 AUTO.TAKEOFF (main=4, sub=2) → ArduPilot TAKEOFF mode (13)

**F) `gcs_server.py`:**
- Port 14550 hâlâ kullanılabilir (ArduPilot da 14550'de broadcast yapar)
- sysid tespiti aynı (MAV_TYPE_FIXED_WING vs QUADROTOR)
- PX4-spesifik mode importları değişecek

**G) Simülasyon Başlatma:**
```bash
# ESKİ (PX4):
cd ~/projects/avci_sim/PX4-Autopilot
bash Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s "iris:1,plane:1" -w empty

# YENİ (ArduPilot) — İki ayrı terminal:
# Terminal A — ArduCopter (Avcı drone):
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --instance 0 -I 0

# Terminal B — ArduPlane (Hedef uçak — Cessna):
cd ~/ardupilot  
python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --instance 1 -I 1
```

**H) XRCE Agent artık GEREKMİYOR** (ArduPilot → MAVLink doğrudan pymavlink ile konuşur, ROS 2 için mavros veya MAVROS2 kullanılabilir ama zorunlu değil).

### GÖREV 2: Talon → Cessna Model Değişikliği

**Cessna kaynakları (mevcut):**
- OBJ mesh: `~/projects/avci_sim/PX4-Autopilot/Tools/simulation/jmavsim/jMAVSim/models/cessna.obj` (145308 satır, detaylı mesh)
- MTL: `~/projects/avci_sim/PX4-Autopilot/Tools/simulation/jmavsim/jMAVSim/models/cessna.mtl`
- Gazebo SDF (yeni Gazebo için): `~/projects/avci_sim/PX4-Autopilot/Tools/simulation/gz/models/rc_cessna/model.sdf`

**Yapılacaklar:**
1. Cessna OBJ mesh'ini Gazebo Classic'e uygun SDF modeline çevir (veya ArduPilot SITL'in kendi default plane modelini kullan)
2. `chase_algorithm.py` sabitlerini güncelle:
   - `PLANE_WINGSPAN_M` → Cessna kanat açıklığı (yaklaşık 1.12m RC Cessna için)
   - `PLANE_FUSELAGE_M` → Cessna gövde uzunluğu (yaklaşık 1.1m)
3. `gcs_server.py` visual offset sabitlerini güncelle veya kaldır
4. `color_detector.py` HSV değerlerini Cessna'nın rengine göre ayarla
5. ArduPlane SITL zaten kendi default plane modelini Gazebo'da gösterir — önce bununla başla

### GÖREV 3: QGroundControl → Mission Planner

**Mission Planner Linux'ta kurulumu:**
```bash
# Mono runtime gerekli
sudo apt-get install mono-complete
# Mission Planner indir
# Veya AppImage versiyonu kullan
```

**Entegrasyon:**
- Mission Planner aynı MAVLink protokolünü kullanır
- ArduPilot SITL'den 14550 portuna UDP broadcast gelir
- Mission Planner'ı UDP 14550'ye bağla
- Mevcut özel GCS web arayüzü (FastAPI) Mission Planner'ın YERİNE değil YANINDA çalışacak
- Mission Planner: Görev planlama, parametre ayarı, log analizi
- Özel GCS: Chase/Strike kontrol, kamera stream, GPS jamming simülasyonu

---

## 8) ÇALIŞMA PRENSİPLERİ

1. **ÇALIŞAN SİSTEMİ BOZMA.** Önce ArduPilot SITL'i ayrı test et, çalıştığını doğrula, sonra mevcut kodları adapte et.
2. **Modüler ilerle.** Önce `mav_common.py`'deki sabitleri ArduPilot'a çevir, sonra `drone_functions.py`, sonra `plane_functions.py`.
3. **Her adımda doğrula.** "Muhtemelen çalışır" deme — log, ACK, telemetri ile kanıtla.
4. **Heredoc kullanma.** Uzun terminal blokları bozulabiliyor — dosya oluşturarak/düzenleyerek ilerle.
5. **Büyük refactor yapma.** Önce minimal çalışan geçişi yap.

---

## 9) UYGULAMA SIRASI

```
ADIM 1: ArduPilot SITL'i tek araçla test et
  → sim_vehicle.py ile ArduCopter başlat
  → pymavlink ile bağlan, heartbeat al, arm et, GUIDED modda kaldır
  → DOĞRULA: kalkış başarılı mı?

ADIM 2: ArduPlane SITL'i tek araçla test et  
  → sim_vehicle.py ile ArduPlane başlat
  → pymavlink ile bağlan, arm et, takeoff modunda kaldır
  → DOĞRULA: uçak havada mı?

ADIM 3: mav_common.py'yi ArduPilot'a adapte et
  → Mod sabitlerini değiştir
  → set_mode() fonksiyonunu güncelle
  → arm() fonksiyonunda force param2'yi güncelle (21196 → 2989)
  → DOĞRULA: arm + mode change ACK alıyor mu?

ADIM 4: drone_functions.py'yi ArduPilot'a adapte et
  → OFFBOARD → GUIDED
  → takeoff_to_z() mantığını güncelle
  → Position setpoint'in ArduPilot GUIDED'da çalıştığını doğrula
  → DOĞRULA: drone kalktı mı, hover yapıyor mu, hareket ediyor mu?

ADIM 5: plane_functions.py'yi ArduPilot'a adapte et
  → PX4 mod numaralarını ArduPlane modlarına çevir
  → RC override'ın çalıştığını doğrula
  → DOĞRULA: plane arm oldu mu, throttle çalışıyor mu?

ADIM 6: İki aracı aynı anda çalıştır
  → İki instance, farklı portlar
  → Her ikisine de pymavlink ile bağlan
  → Telemetri oku, her ikisi de çalışıyor mu?
  → DOĞRULA: iki araç da havada

ADIM 7: Chase/Strike algoritmasını test et
  → chase_algorithm.py zaten MAVLink velocity/position setpoint kullanıyor
  → ArduPilot GUIDED modda da bu mesajlar çalışır
  → Cessna sabitlerini güncelle
  → DOĞRULA: drone plane'i takip ediyor mu?

ADIM 8: GCS server'ı adapte et
  → Port mapping'i güncelle
  → Mode importlarını güncelle
  → DOĞRULA: web arayüzde telemetri görünüyor mu?

ADIM 9: Mission Planner kurulumu
  → mono-complete kur
  → Mission Planner indir ve çalıştır
  → ArduPilot SITL'e UDP 14550 ile bağla
  → DOĞRULA: Mission Planner'da her iki araç görünüyor mu?

ADIM 10: Cessna modeli ince ayar
  → Renk tespiti HSV değerlerini güncelle
  → chase_algorithm.py wingspan/fuselage sabitlerini güncelle
  → DOĞRULA: kamera stream'de Cessna tespit ediliyor mu?
```

---

## 10) SON HEDEF

Tüm migrasyon tamamlandığında şu senaryo çalışmalı:

1. ArduPilot SITL'de iki araç (ArduCopter + ArduPlane/Cessna) başlatılır
2. Mission Planner'da her iki araç görünür
3. Özel GCS web arayüzünde (port 8000) telemetri, kamera akışı çalışır
4. "Chase Başlat" butonuna basılınca avcı drone kalkar
5. Drone, Cessna'yı kamerayla tespit eder (renk tespiti)
6. Takip algoritması Cessna'nın arkasına kilitlenir
7. "Strike" butonuna basılınca PN güdüm ile kamikaze dalışı yapar
8. Drone Cessna'ya çarpana kadar (dist < 0.05m) izler

**Bu prompttaki tüm bilgileri biliyormuş gibi projeye devam et. Adım adım ilerle, her adımda doğrula.**
