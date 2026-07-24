# Ağ Fırlatmalı Taretli Drone — Entegrasyon (branch: `vakkas-entegre`)

Bu branch, avcı drone'a **2 eksenli taret + namludan ağ fırlatma** mekanizmasını
ekler. **SADECE taretli araçla ilgili dosyaları** içerir; güdüm kodları, arayüz
(gcs_server / web UI) ve yapay zeka modelleri **bilerek dışarıda bırakılmıştır.**

> **Amaç:** Diğer ekip arkadaşları bu branch'i kendi branch'lerine merge ettiğinde
> **yalnızca taretli aracı** alsınlar; kendi (daha güncel) güdüm/arayüz/AI kodları
> **etkilenmesin.**

Branch, herkesin ortak atası olan commit'ten (`5e2be6f`) açıldı ve yalnızca
aşağıdaki dosyaları değiştirir/ekler. Bu yüzden merge ettiğinizde **sadece bu
dosyalar** gelir; sizin diğer değişiklikleriniz olduğu gibi kalır.

---

## Merge nasıl yapılır

```bash
git fetch origin
git checkout <kendi-branchiniz>
git merge origin/vakkas-entegre
```

Gelen dosyalar (aşağıdaki liste) dışında hiçbir şeyiniz değişmez. Eğer drone
modelinizi (`iris_cam` / `iris_with_standoffs`) veya `avci_harmonic.sdf` world'ünü
kendiniz de değiştirdiyseniz, yalnızca o dosyalarda çakışma (conflict) çözersiniz.

---

## Bu branch'in İÇERDİĞİ dosyalar (taret mekanizması)

| Dosya | Ne |
|---|---|
| `plugins/NetLauncherPlugin.{cc,hh}` | Ağı namludan atan C++ gz-sim eklentisi |
| `plugins/NetCapturePlugin.{cc,hh}` | Temas eden hedefi ağa kilitleyen eklenti |
| `plugins/CMakeLists.txt` | Eklenti derleme dosyası |
| `sim/gazebo_harmonic/models/net_cone/` | Atılan ağ modeli |
| `sim/gazebo_harmonic/models/iris_cam/model.sdf` | **Taretli drone** (taret + namlu + kamera taret üzerinde) |
| `sim/gazebo_harmonic/models/iris_with_standoffs/model.sdf` | Kamera base_link'ten alınıp tarete taşındı |
| `sim/gazebo_harmonic/worlds/avci_harmonic.sdf` | World'e Contact + NetCapturePlugin + net_cone eklendi |
| `ros/net_turret_bridge.yaml` | ROS 2 ↔ Gazebo köprü eşlemesi |
| `ros/net_turret.launch.py` | Köprü + yönetim düğümü launch |
| `control/net_turret_node.py` | Ağ/taret ROS 2 yönetim düğümü |
| `scripts/turret_aim.py` · `scripts/fire_net.py` | ROS 2 taret nişan / ağ atma araçları |

## İÇERMEDİĞİ (bilerek — kendi sisteminizi kullanın)

- Arayüz: `control/gcs_server.py`, `control/gcs_ui/`
- Güdüm: `control/chase_algorithm.py`, `strike_algorithm.py`, `drone_functions.py`, …
- Yapay zeka / görüntü: `vision/` (modeller, tespit)
- Sistem başlatıcılar: `run_all.sh`, `start_harmonic.sh`, vb.

---

## Kurulum

### 1) Eklentileri derle (bir kez)
```bash
cd plugins
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j$(nproc)
# -> libNetLauncherPlugin.so, libNetCapturePlugin.so
```
Gazebo'yu başlatırken plugin yolunu ekleyin:
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:<repo>/plugins/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=<repo>/sim/gazebo_harmonic/models:$GZ_SIM_RESOURCE_PATH
```

### 2) ROS köprüsü (Humble + Harmonic)
```bash
sudo apt-get install -y ros-humble-ros-gzharmonic-bridge
```

---

## Kullanım (ROS 2 üzerinden — tamamen ROS)

Gazebo + world çalışırken:
```bash
# köprü + yönetim düğümü
export PYTHONPATH=<repo>:$PYTHONPATH
ros2 launch ros/net_turret.launch.py

# nişanla / ateşle
python3 scripts/turret_aim.py 90 45      # pan 90°, tilt 45°
python3 scripts/fire_net.py --hiz 20     # ağı at
# ya da saf ROS:
ros2 topic pub -1 /avci/turret/aim_deg geometry_msgs/msg/Vector3 "{x: 90, y: 45}"
ros2 service call /net_turret_node/fire std_srvs/srv/Trigger
```

### ROS arayüzü
| Topic / Servis | Tip | Yön |
|---|---|---|
| `/avci/turret/aim_deg` | `geometry_msgs/Vector3` (x=pan°, y=tilt°) | nişan → |
| `/avci/net/fire_speed` | `std_msgs/Float64` (m/s) | ateş → |
| `/net_turret_node/fire` | `std_srvs/Trigger` | varsayılan hızla ateş |
| `/avci/net/captured` | `std_msgs/String` | ← yakalanan model |

---

## Taret özellikleri

- **Konum:** drone'un yatay merkezinde (x=0, y=0), üst bölümünde.
- **Kamera tarete entegre:** taret nereye dönerse kamera da oraya bakar
  (topic `iris_cam/image` aynı kaldı).
- **PAN:** 0–360° (tam tur).
- **TILT:** 0–180° — 0=ileri (şase paralel), 90=tam yukarı, 180=geri (şase paralel);
  asla şasenin altına bakmaz.
- **Ağ:** tek atımlık; yeniden atış için world'ü yeniden yükleyin.

---

## Kendi world'ünüz varsa (avci_harmonic.sdf'i kullanmıyorsanız)

World'ünüze şu üçünü ekleyin:

```xml
<!-- 1) Temas sistemi (net_cone temas sensörü için) -->
<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

<!-- 2) Yakalama: ağ hedefe değince kilitler (target_model = kendi hedefiniz) -->
<plugin filename="NetCapturePlugin" name="avci::NetCapturePlugin">
  <net_model>net_cone</net_model>
  <net_link>net_link</net_link>
  <target_model>mini_talon</target_model>
  <capture_topic>/avci/net/captured</capture_topic>
  <min_speed>1.0</min_speed>
</plugin>

<!-- 3) Ağ konisi — drone'un namlu ucunda başlasın (drone kalkış pozunuza göre poz) -->
<include>
  <uri>model://net_cone</uri>
  <name>net_cone</name>
  <pose degrees="true">0 0.06 0.29 0 0 90</pose>
</include>
```

Taret + namlu + kamera bloğu `iris_cam/model.sdf` içindedir (`turret_base_link` →
`turret_yaw_joint` → `turret_pitch_joint` → `muzzle_link` + kamera). Kendi drone
modelinize taşımak isterseniz bu bloğu oradan alın.
