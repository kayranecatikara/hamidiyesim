# AVCI SİM — Proje Yapısı

> Bu dosya, 30 Temmuz 2026 temizliği sonrası güncellenmiştir.
> Kod dosyalarının **detaylı açıklamaları** için:
> `~/Masaüstü/AVCI_SIM_DOKUMANTASYON/` klasörü.
> Depo içi dokümanlar için: [docs/](docs/)

---

---

## 1. Klasör ağacı

```
avci_sim/
├── control/                      # Uçuş kontrolü + güdüm + yer kontrol istasyonu
│   ├── mav_common.py                 # Ortak MAVLink altyapısı (ArduPilot)
│   ├── sim_truth.py                  # Gazebo gerçek poz + temas (vuruş doğrusu)
├── tools/gudum_karne.py              # Uçuş karnesi: güdüm metrik raporu/kıyası
├── tools/parm_denetle.py             # .parm dosyaları SITL'e gerçekten uygulandı mı
│   ├── drone_functions.py            # iris (ArduCopter) kontrolü
│   ├── plane_functions.py            # Talon (ArduPlane) kontrolü
│   ├── plane_patterns.py             # Talon manevra desenleri
│   ├── run_plane_scenario.py         # Hedef senaryoları (GCS bunu başlatır)
│   ├── arm_diag.py                   # ARM reddi teşhis aracı
│   ├── gcs_server.py                 # FastAPI + kamera + görev API
│   ├── gcs_ui/                       # Web arayüzü
│   │   ├── index.html
│   │   ├── script.js
│   │   └── style.css
│   ├── guidance/                     # ★ HİBRİT GÜDÜM (projenin kalbi)
│   │   ├── guidance_core.py              # IBVS çekirdeği (platformdan bağımsız)
│   │   ├── adapter_copter.py             # Çekirdek çıktısı → copter komutu
│   │   ├── visual_lead.py                # Görsel faz döngüsü
│   │   ├── gps_guidance.py               # GPS fazı döngüsü
│   │   ├── supervisor.py                 # Faz geçiş denetleyicisi
│   │   └── common.py                     # Paylaşılan matematik + MAVLink
│   └── demos/                        # Elle çalıştırılan bağımsız uçuş demoları
│       ├── run_drone_takeoff.py
│       ├── run_drone_hover.py
│       ├── run_drone_square.py
│       ├── run_plane_arm.py
│       ├── run_plane_aggressive.py
│       └── run_dual_demo.py
├── vision/                       # Görüntü işleme + model eğitimi
│   ├── geometry.py                   # Kamera projeksiyonu + Talon 3D geometrisi
│   ├── detector.py                   # YOLO tespiti (bbox)
│   ├── pose_detector.py              # YOLO-pose (6 keypoint)
│   ├── tracker.py                    # HybridSORT takip (boxmot, kareler arası ID)
│   ├── hybridsort_video.py           # Takibi video dosyasında offline çalıştırma
│   ├── compare_tracker.py            # GT'li "detection vs detection+tracker" deneyi
│   ├── detection_state.py            # Thread-safe tespit köprüsü
│   ├── capture_dataset.py            # Detection verisi (otomatik etiketli)
│   ├── capture_pose_dataset.py       # Pose verisi (otomatik etiketli)
│   ├── capture_negatives.py          # Hard-negative (canlı sim, pervane)
│   ├── capture_runway_negatives.py   # Hard-negative (pist/zemin)
│   ├── train_yolo.py                 # Detection eğitimi
│   ├── train_yolo_pose.py            # Pose eğitimi
│   ├── models/                       # avci_yolo.pt, avci_pose.pt
│   └── demo_model_comparison/        # 20 örnek çıkarım görüntüsü
├── sim/
│   ├── gazebo_harmonic/
│   │   ├── worlds/
│   │   │   ├── avci_harmonic.sdf         # Ana görev dünyası
│   │   │   └── dataset_capture.sdf       # Statik veri toplama dünyası
│   │   └── models/
│   │       ├── iris_cam/                 # Avcı drone + kamera
│   │       ├── mini_talon_vtail/         # Hedef İHA (uçan, tam detaylı)
│   │       ├── iris_with_standoffs/      # Veri toplama kamera taşıyıcı
│   │       └── mini_talon_target/        # Veri toplama hedefi (sade)
│   └── ardupilot_params/
│       ├── avci_copter.parm              # Avcı drone parametre yaması
│       └── avci_plane.parm               # Hedef uçak parametre yaması
├── scripts/
│   ├── start_harmonic.sh             # ★ Ana başlatıcı (Gazebo + 2 SITL)
│   ├── start_ardupilot_sitl.sh       # Sadece SITL (Gazebo'suz)
│   ├── setup_mission_planner.sh      # Mission Planner indirici
│   └── start_mission_planner.sh      # Mission Planner başlatıcı
├── tests/
│   ├── test_visual_lead.py           # Görsel hat kabul testleri (T1-T30)
│   └── test_gps_guidance.py          # GPS fazı kabul testleri (G1-G9)
├── tools/
│   ├── gps_log_viz.py                # Uçuş CSV'leri → interaktif HTML panel
│   └── mission_planner/              # MP binary (depoda değil)
├── docs/
│   ├── SIMULASYON_CALISTIRMA.md      # Çalıştırma komutları (5 terminal)
│   ├── GUIDANCE.md                   # Güdüm mimarisi (gerçeklik)
│   ├── GUIDANCE_ROADMAP.md           # Güdüm yol haritası (plan/gerekçe)
│   ├── GPS_LOGGING.md                # CSV log formatı + görselleştirme
│   ├── COLAB_TRAINING.md             # Colab GPU'sunda eğitim
│   └── ARDUPILOT_MIGRATION.md        # PX4 → ArduPilot geçişi (tarihsel)
├── logs/                         # Çalışma çıktıları (depoda değil)
├── README.md
├── requirements.txt
└── .gitignore
```

---

## 2. Klasörlerin işlevi

| Klasör | İşlevi | Depoda mı? |
|--------|--------|------------|
| **`control/`** | Araçları MAVLink ile uçurur, güdüm algoritmalarını çalıştırır, web GCS'i sunar. Projenin beyni ve kasları. | ✅ |
| **`control/guidance/`** | İki fazlı hibrit güdüm paketi. Bir aracın hedefi *nasıl* vuracağını belirleyen tüm matematik burada. | ✅ |
| **`control/gcs_ui/`** | Web arayüzünün ön yüzü. `gcs_server` bunu statik dosya olarak servis eder. | ✅ |
| **`control/demos/`** | Görev akışının parçası değil — tek tek fonksiyon denemek, öğrenmek ve veri toplama uçuşu yapmak için. | ✅ |
| **`vision/`** | Kameradan hedefi bulan ve yönelimini çıkaran katman + bu modelleri üreten veri/eğitim hattı. | ✅ (dataset hariç) |
| **`sim/`** | Simülasyonun fiziksel dünyası: Gazebo world/model dosyaları ve ArduPilot uçuş parametreleri. | ✅ |
| **`scripts/`** | Sistemi doğru sırayla ayağa kaldıran ve harici araçları kuran kabuk betikleri. | ✅ |
| **`tests/`** | Gazebo/SITL gerektirmeyen saf mantık kabul testleri. Kod değiştirmeden önce çalıştırılır. | ✅ |
| **`tools/`** | Yardımcı araçlar: uçuş log görselleştiricisi ve Mission Planner. | Kısmen |
| **`docs/`** | Mimari, çalıştırma, güdüm ve loglama dokümanları. | ✅ |
| **`logs/`** | Uçuş CSV'leri ve süreç logları — her çalıştırmada üretilir. | ❌ |

---

## 3. Dosya envanteri (satır sayılarıyla)

### control/ — 3.086 satır Python + 2.187 satır arayüz

| Dosya | Satır | Rol |
|-------|------:|-----|
| `gcs_server.py` | 1170 | Web GCS: FastAPI + kamera + telemetri + görev API |
| `drone_functions.py` | 414 | iris (ArduCopter) kontrolü |
| `mav_common.py` | 377 | Ortak MAVLink altyapısı |
| `sim_truth.py` | 165 | Gazebo gerçek poz + temas olayı (vuruş doğrusu) |
| `plane_functions.py` | 375 | Talon (ArduPlane) kontrolü |
| `arm_diag.py` | 329 | ARM reddi teşhisi |
| `plane_patterns.py` | 317 | Talon manevra desenleri |
| `run_plane_scenario.py` | 317 | Hedef uçuş senaryoları |
| `__init__.py` | 16 | Paket dokümantasyonu |
| `gcs_ui/script.js` | 990 | Arayüz mantığı |
| `gcs_ui/style.css` | 939 | Arayüz tasarımı |
| `gcs_ui/index.html` | 258 | Arayüz iskeleti |

### control/guidance/ — 1.166 satır

| Dosya | Satır | Rol |
|-------|------:|-----|
| `guidance_core.py` | 344 | IBVS lead pursuit çekirdeği (platformdan bağımsız) |
| `visual_lead.py` | 319 | Görsel faz döngüsü (olay güdümlü) |
| `gps_guidance.py` | 292 | GPS fazı döngüsü (kadraj merkezleme) |
| `adapter_copter.py` | 123 | Copter komut adaptörü |
| `supervisor.py` | 114 | Faz geçiş denetleyicisi |
| `common.py` | 74 | Paylaşılan matematik + `send_velocity` |
| `__init__.py` | 20 | Paket dokümantasyonu |

### control/demos/ — 580 satır

| Dosya | Satır | Rol |
|-------|------:|-----|
| `run_dual_demo.py` | 125 | İki araç eş zamanlı demo |
| `run_drone_square.py` | 109 | Kare yörünge (negatif veri uçuşu) |
| `run_drone_hover.py` | 102 | Kalkış + hareket + yaw |
| `run_plane_aggressive.py` | 81 | Agresif manevralar |
| `run_plane_arm.py` | 74 | Keepalive + ARM |
| `run_drone_takeoff.py` | 71 | Kalkış + hover |
| `__init__.py` | 18 | Paket dokümantasyonu + kullanım |

### vision/ — 1.358 satır

| Dosya | Satır | Rol |
|-------|------:|-----|
| `geometry.py` | 288 | Kamera projeksiyonu + Talon 3D geometrisi |
| `capture_dataset.py` | 247 | Detection verisi toplama |
| `capture_runway_negatives.py` | 224 | Pist/zemin hard-negative |
| `capture_pose_dataset.py` | 193 | Pose verisi toplama |
| `compare_tracker.py` | 298 | GT'li detection vs detection+tracker deneyi |
| `tracker.py` | 285 | HybridSORT sarmalayıcı + kilitli-ID politikası (TargetLock) |
| `hybridsort_video.py` | 205 | HybridSORT'u videoda offline çalıştırma |
| `pose_detector.py` | 120 | YOLO-pose çıkarımı |
| `detector.py` | 114 | YOLO detection çıkarımı (+detect_all/best_det) |
| `capture_negatives.py` | 110 | Pervane hard-negative |
| `detection_state.py` | 74 | Thread-safe tespit köprüsü |
| `train_yolo.py` | 55 | Detection eğitimi |
| `train_yolo_pose.py` | 53 | Pose eğitimi |

### tests/, tools/, scripts/, sim/

| Dosya | Satır | Rol |
|-------|------:|-----|
| `tests/test_visual_lead.py` | 439 | Görsel hat kabul testleri (T1-T30) |
| `tests/test_gps_guidance.py` | 143 | GPS fazı kabul testleri (G1-G9) |
| `tools/gps_log_viz.py` | 293 | Uçuş CSV → interaktif HTML panel |
| `scripts/start_harmonic.sh` | 111 | Ana sistem başlatıcı |
| `scripts/start_ardupilot_sitl.sh` | 71 | SITL başlatıcı (Gazebo'suz) |
| `scripts/start_mission_planner.sh` | 45 | Mission Planner başlatıcı |
| `scripts/setup_mission_planner.sh` | 22 | Mission Planner indirici |
| `sim/gazebo_harmonic/worlds/avci_harmonic.sdf` | 171 | Ana görev dünyası |
| `sim/gazebo_harmonic/worlds/dataset_capture.sdf` | 83 | Veri toplama dünyası |
| `sim/ardupilot_params/avci_copter.parm` | 31 | Avcı drone parametre yaması |
| `sim/ardupilot_params/avci_plane.parm` | 27 | Hedef uçak parametre yaması |

### docs/ — 844 satır

| Dosya | Satır | Durum |
|-------|------:|-------|
| `GUIDANCE.md` | 197 | GERÇEKLİK — kodda çalışan sistem |
| `GUIDANCE_ROADMAP.md` | 193 | PLAN — tasarım gerekçeleri |
| `ARDUPILOT_MIGRATION.md` | 150 | TARİHSEL — PX4 geçişi |
| `SIMULASYON_CALISTIRMA.md` | 107 | GÜNCEL — çalıştırma komutları |
| `GPS_LOGGING.md` | 102 | GÜNCEL — log formatı |
| `COLAB_TRAINING.md` | 95 | GÜNCEL — bulut eğitimi |

**Toplam:** ~11.400 satır (Python + arayüz + SDF + doküman)

---

## 4. Depoya girmeyen dosyalar (`.gitignore`)

| Yol | Neden |
|-----|-------|
| `__pycache__/`, `*.pyc` | Python derleme çıktısı |
| `.venv/`, `venv/`, `env/` | Sanal ortamlar |
| `tools/mission_planner/` | ~400 MB binary — `setup_mission_planner.sh` indirir |
| `logs/` | Her çalıştırmada üretilen uçuş CSV'leri ve süreç logları |
| `vision/datasets/` | Üretilen YOLO eğitim verisi (binlerce görüntü) |
| `runs/`, `*.pt` | Eğitim çıktıları |
| `eeprom.bin`, `mav.parm` | ArduPilot SITL geçici dosyaları |

**İstisna:** eğitilmiş `vision/models/avci_yolo.pt` ve `avci_pose.pt` depoya
**girer** (~5.5 MB) — dataset girmez ama kullanılabilir model girer.

---

## 5. Harici bağımlılıklar (bu depoda değil)

| Bağımlılık | Konum | Kurulum |
|------------|-------|---------|
| ArduPilot SITL | `~/ardupilot` | `./waf copter && ./waf plane` |
| ArduPilot Gazebo eklentisi | `~/ardupilot_gazebo` | `cmake && make` |
| ROS 2 Humble | `/opt/ros/humble` | apt |
| Gazebo Harmonic (gz-sim 8) | sistem | apt (`gz-harmonic`) |
| gz-transport Python | sistem | apt (`python3-gz-transport13`) |
| Mission Planner | `tools/mission_planner/` | `setup_mission_planner.sh` |

**Zorunlu ortam değişkenleri:**
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
```
