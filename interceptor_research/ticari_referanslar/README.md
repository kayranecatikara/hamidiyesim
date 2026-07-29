# Ticari Ağ Atan Interceptor Sistemleri — Teknik Referans

Aradığımız 5 kalemin **SDF/3D model arama sonucu ve kullanılabilir teknik verisi**.

> **Özet sonuç:** Bu sistemlerin hiçbirinin SDF/URDF modeli public değil.
> Hepsi kapalı kaynak ticari ya da yayınlanmamış akademik ürün.
> Bu doküman, kendi modelimizi **gerçekçi parametrelerle** kurmak için
> açık kaynaklardan derlenen sayısal verileri tutar.

---

## 1. Fortem DroneHunter F700

**SDF durumu:** ✗ Yok. Kapalı kaynak, Fortem Technologies ürünü.

| Parametre | Değer | Modelimize etkisi |
|---|---|---|
| Faydalı yük | ~2.3 kg (5 lb) | Taret + ağ toplam kütle bütçesi |
| Çekme kapasitesi | ~5.9 kg (13 lb) hedef | Yakalama sonrası taşıma senaryosu |
| Ağ tipleri | 3 adet: **tether net** (küçük Grup-1 hedefi bağlayıp indir), **DrogueNet** (paraşütlü, ağır hedef için), standart | `net_cone` → `net_mesh` → `net_drogue` varyantları |
| İsabet oranı | İlk atışta ~%85 (hedeflerin %15'i kaçıyor) | Görev testinde başarı eşiği: 10 denemede ≥8 |
| İkinci atış | Genelde hazır | Çoklu ağ şarjörü opsiyonu |
| Tespit | Radar (multi-km) | Bizde kamera + YOLOv8 |
| Otonomi | Tam otonom takip + yakalama | Hedef durum makinesi tasarımı |

**Kaynaklar:**
- <https://fortemtech.com/products/dronehunter-f700/>
- <https://www.defensedaily.com/fortems-f700-dronehunter-open-architecture-autonomous-drone-drone-combat/unmanned-systems/>
- <https://www.aviationtoday.com/2020/04/02/fortems-f700-dronehunter-open-architecture-autonomous-drone-drone-combat/>

---

## 2. Delft Dynamics — DroneCatcher

**SDF durumu:** ✗ Yok. Kapalı kaynak (Delft Dynamics B.V., Hollanda).

**Bizim tasarımımıza en yakın sistem** — pnömatik ağ tabancalı multikopter.

| Parametre | Değer | Modelimize etkisi |
|---|---|---|
| Ağ tabancası | **Pnömatik** | Fırlatma = tek seferlik impuls (sürekli itki değil) |
| **Menzil** | **20 m'ye kadar** | `fire_net.py` impuls kalibrasyonunun hedefi |
| Toplam kütle | < 6 kg | Aday gövde seçiminde üst sınır |
| İleri hız | 20 m/s | Yaklaşma fazı hız profili |
| Nişanlama | Kamera + **lazer mesafe ölçer** + track & trace | Taret nişan çözümü: görüntü + menzil |
| Yakalama sonrası | Hedefi **kabloyla** güvenli bölgeye taşır; ağırsa "kontrollü paraşüt" gibi davranır | `NetCapturePlugin` sonrası davranış modu |
| Diğer | Katlanır karbon fiber kol | Görsel detay, fizik için önemsiz |

**Kaynaklar:**
- <https://dronecatcher.nl/>
- <https://www.delftdynamics.nl/?portfolio=dronecatcher>
- <https://newatlas.com/dronecatcher/55056/>
- <https://dronelife.com/2017/10/23/video-dronecatcher-downs-naughty-uavs-net/>

---

## 3. Airspace Systems — Interceptor (Alpha / Defender)

**SDF durumu:** ✗ Yok. Kapalı kaynak, **ABD patentli**, US Army kontratı altında geliştirilmiş.

| Parametre | Değer |
|---|---|
| Mitigasyon | Fiziksel yakalama **veya** ramming (hard-kill) |
| Otonomi | Tam otonom tespit → takip → mitigasyon, üst seviye süpervizör kontrolü |
| Hedef yelpazesi | Ticari + modifiye drone platformları |
| Teknik detay | Yayınlanmamış (patent koruması) |

**Not:** Bu sistemin ram/kamikaze modu `avci_sim` projesindeki mevcut yaklaşıma denk.
Ağ tarafı için kullanılabilir sayısal veri yok.

**Kaynak:** <https://www.airspace.co/products-interceptor>

---

## 4. CTU MRS "Net Launcher Plugin" (mrs_gazebo_common_resources)

**SDF/plugin durumu:** ✗ **Böyle bir plugin mevcut değil.**

`ctu-mrs` organizasyonunun 200+ public reposu tarandı.
`mrs_gazebo_common_resources/src/` altındaki tüm eklentiler:

```
2dlidar_plugin.cpp        gps_plugin.cpp              rangefinder_plugin.cpp
3dlidar_plugin.cpp        light_plugin.cpp            realsense_plugin.cpp
camera_plugin.cpp         link_static_tf_publisher    safety_led_plugin.cpp
dynamic_model_plugin.cpp  livox_points_plugin.cpp     servo_camera_plugin.cpp
magnetometer_plugin.cpp   motor_speed_republisher     parachute_plugin.cpp
                                                      water_gun_plugin.cpp
```

Ağ fırlatıcı yok. **En yakın 3 public muadil (bizim şablonlarımız):**

| Dosya | Repo | Neden değerli |
|---|---|---|
| `src/sensor_and_model_plugins/water_gun_plugin.cpp` | `mrs_gazebo_common_resources` | UAV üzerinde bir noktadan **mermi/parçacık spawn edip fırlatma** deseni (offset_x/y/z ile namlu ucu hesabı) |
| `src/sensor_and_model_plugins/parachute_plugin.cpp` | `mrs_gazebo_common_resources` | Komutla payload ayırma |
| **`src/link_attacher.cpp`** | `mrs_gazebo_extras_resources` | **Çalışma anında iki model arasında joint yaratma/koparma servisi** — yakalama (ağın hedefe kilitlenmesi) mantığının referansı |

⚠️ Üçü de **Gazebo Classic + ROS1**. Harmonic'te doğrudan derlenmez; mantık şablonu olarak kullanılır.
Harmonic karşılığını `ardupilot_gazebo/src/ParachutePlugin.cc`'den türeteceğiz
(orada `_ecm.CreateEntity()` + `components::DetachableJoint` deseni zaten var).

**MBZIRC 2020 bağlantısı:** CTU MRS, MBZIRC 2020 Challenge 1'i (uçan hedeften top
yakalama) gövde altı ağ ile kazandı. O yarışmanın ağlı drone kodu/modeli
**yayınlanmadı**. Devamı olan çalışma: EPN güdüm yöntemi, arXiv **2405.13542**
("Towards Safe Mid-Air Drone Interception", IEEE RA-L 2024) — ağ taşıyan uçan robot,
IMM tabanlı durum kestirimi. Kod public değil ama güdüm matematiği referansımız.

---

## 5. MBZIRC Drone Hunter Gazebo Modeli

**SDF durumu:** ⚠️ Resmî model public değil, **ama arena/dünya modelleri var.**

Bulunan en yakın public kaynak:
[`Bochicchio3/MBZIRC-2020-Challenge`](https://github.com/Bochicchio3/MBZIRC-2020-Challenge)
— MBZIRC 2020'de 26 takım arasında 4. olan ekibin **ArduPilot + Gazebo + ROS**
entegrasyon rehberi (`repos/MBZIRC-2020-Challenge/`).

İçindekiler:
```
MBZIRC/MBZIRC/MBZIRC.world
MBZIRC/MBZIRC/MBZIRColo_base/model.sdf          # arena
MBZIRC/MBZIRC/MBZIRColo_demo/model.sdf
MBZIRC/MBZIRC/MBZIRColo_front_camera/model.sdf  # kameralı drone
MBZIRC/MBZIRC/MBZIRColo_fake_camera/model.sdf
MBZIRC/MBZIRC/MBZIRColo_hope_camera/model.sdf
MBZIRC/MBZIRC/gimbal_small_2d/model.sdf         # 2 eksenli gimbal — taret referansı
```

**Değeri:** Ağ yok, ama (a) yığını bizimkiyle aynı (ArduPilot+Gazebo+ROS),
(b) `gimbal_small_2d` taret için ikinci bir referans, (c) arena dünyası hazır test sahnesi.

İlgili akademik kaynak (ağ mekanizmasının tarifi):
*"Autonomous capture of agile flying objects using UAVs: The MBZIRC 2020 challenge"*,
Robotics and Autonomous Systems 149 (2021).

---

## Tasarım hedeflerimiz (yukarıdaki verilerden türetilen)

| Büyüklük | Hedef değer | Dayanağı |
|---|---|---|
| Interceptor toplam kütle | ≤ 6 kg | DroneCatcher |
| Taret + ağ yükü | ≤ 2.3 kg | Fortem F700 faydalı yük |
| Ağ menzili | ≥ 15 m (hedef 20 m) | DroneCatcher pnömatik ağ tabancası |
| Ağ çıkış hızı | 15–20 m/s | 20 m menzil + balistik düşüş |
| Ağ kütlesi | ~0.15–0.3 kg | Menzil/impuls dengesi |
| Yaklaşma hızı | ≤ 20 m/s | DroneCatcher ileri hız |
| Yakalama başarısı | 10 denemede ≥ 8 | Fortem %85 ilk atış isabeti |
| Taret ekseni | 2 (pan + tilt) | DroneCatcher / gimbal_small_2d |
