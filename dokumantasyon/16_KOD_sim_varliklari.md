# 16 — `sim/` Simülasyon Varlıkları

> Gazebo dünyaları, araç modelleri ve ArduPilot parametre yamaları.
> Bunlar "kod" değil ama sistemin davranışını kodun kendisi kadar belirler.

---

## 1. Gazebo dünyaları — `sim/gazebo_harmonic/worlds/`

### `avci_harmonic.sdf` — Ana görev dünyası

**Satır:** 171 · **Kullanım:** canlı görev

```bash
gz sim -r -v4 sim/gazebo_harmonic/worlds/avci_harmonic.sdf
```

**İçindekiler:**

| Öğe | Konum | Not |
|-----|-------|-----|
| `iris_cam` | `(0, 0, 0.195)`, yaw 90° | Avcı drone — FDM 9002 |
| `mini_talon` (`mini_talon_vtail`) | `(12, 0, 0.195)`, yaw 90° | Hedef İHA — FDM 9012 |
| `runway` | origin | Pist mesh'i (`airfield.dae`) |
| `grass_field` | zemin | Çim saha |
| `axes` | origin | X/Y/Z eksen işaretleri (kırmızı/yeşil/mavi çubuklar) — yön hatalarını gözle görmek için |
| `sun` | `(0, 0, 10)` | Yönlü ışık |
| `navsat_sensor` | — | GPS simülasyonu |

**Fizik:** 1 ms adım, `real_time_factor = 1.0`.

**Kritik:** Her iki araç da kendi `ArduPilotPlugin`'i ve **ayrı FDM portu** ile
bağlanır (9002 / 9012). Bu ayrım sayesinde iki SITL instance'ı aynı world'de
çakışmadan uçar.

---

### `dataset_capture.sdf` — Statik veri toplama dünyası

**Satır:** 83 · **Kullanım:** YOLO verisi toplama

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models
gz sim -r sim/gazebo_harmonic/worlds/dataset_capture.sdf
```

**Farkı:** Fizik ve ArduPilot **yok**. Sabit bir kamera taşıyıcısı ve statik bir
hedef var; `capture_*.py` betikleri bunları `/world/dataset/set_pose` ile
ışınlar.

| Öğe | Konum | Not |
|-----|-------|-----|
| `camera_rig` (`iris_with_standoffs`) | `(0, 0, 20)`, **static** | `base_link` 20 m'de; kamera ofseti `(0.10, 0, 0.05)`, pitch −25° (yukarı). `capture_dataset.py`'deki `IRIS_POS=(0,0,20)`, `IRIS_YAW=0` ile **birebir** eşleşir. |
| `mini_talon` (`mini_talon_target`) | `(20, 0, 14)`, yaw 1.57 | Etiketlenecek hedef |
| `runway` | origin | Kameranın altında — hedef bazen pist önünde görünsün diye |

> **Neden arka plan ana world'e eşlendi?** Aynı gökyüzü tonu (0.8 gri), aynı
> güneş ışığı, aynı pist kullanılıyor. Amaç **domain gap'i kapatmak**: eğitim
> verisi ile canlı uçuşun görsel dağılımı farklı olursa model canlıda çuvallar.

---

## 2. Gazebo modelleri — `sim/gazebo_harmonic/models/`

Her modelde `model.config` (meta veri) + `model.sdf` (geometri, sensör, eklenti)
+ `meshes/` (DAE görsel, STL collision) bulunur.

### `iris_cam/` — Avcı drone

**Kullanan:** `avci_harmonic.sdf`

- `iris_with_standoffs` modelini `<include>` ile alır, üzerine kamera ve
  ArduPilot eklentisi ekler
- 4 rotor (`(±0.13, ±0.22, 0.216)` konumlarında), her biri kendi joint'iyle
- **Kamera:** 640×480, HFOV 125°, `base_link`'e göre `(0.10, 0, 0.05)`,
  pitch **−0.4363 rad = 25° yukarı** → topic `/iris_cam/image`
- **`ArduPilotPlugin`** → `fdm_port_in = 9002`

> Kameranın 25° yukarı tilt'i tüm güdüm matematiğinin dayandığı sabittir
> (`guidance_core.Cfg.KAMERA_TILT_DEG`, `geometry.CAM_TILT_RAD`). SDF'de
> değiştirilirse iki koddaki değer de değişmelidir.

### `mini_talon_vtail/` — Hedef İHA (uçan, tam detaylı)

**Kullanan:** `avci_harmonic.sdf`

- **7 link** — gövde, sol/sağ kanat, sol/sağ V-kuyruk, ön güverte, tekerlek
- **6 hareketli joint:** `left_aileron_joint`, `right_aileron_joint`,
  `left_ruddervator_joint`, `right_ruddervator_joint`, `motor_joint`,
  `main_wheel_joint`
- **Sensörler:** IMU, NavSat (GPS), burun kamerası (`talon_camera`, HFOV 1.8 rad ≈ 103°)
- **`ArduPilotPlugin`** → `fdm_port_in = 9012`
- Ayrı collision STL'leri (`*_collision.stl`) — `vision/geometry.py` bunları okur

> Talon **gerçekten uçar**: ArduPlane SITL kontrol yüzeylerini ve motoru sürer,
> Gazebo aerodinamiği hesaplar. Eskiden telemetri "relay" ile hareket
> ettiriliyordu; o yaklaşım kaldırıldı.

### `iris_with_standoffs/` — Veri toplama kamera taşıyıcısı

**Kullanan:** `dataset_capture.sdf` (`camera_rig` adıyla)

Sade Iris gövdesi + mesh'ler (`iris.dae`, `iris_prop_cw/ccw.dae`,
`iris_collision.stl`). ArduPilot eklentisi **yok** — statik olarak konumlanır.

### `mini_talon_target/` — Veri toplama hedefi

**Kullanan:** `dataset_capture.sdf`

Talon'un sade sürümü: gövde, sol/sağ kanat, sol/sağ kuyruk, ön güverte
mesh'leri. Hareketli yüzey, sensör ve eklenti yok — sadece görünüm ve geometri
gerekiyor.

---

## 3. ArduPilot parametre yamaları — `sim/ardupilot_params/`

Bu dosyalar SITL başlatılırken `--add-param-file` ile yüklenir. **Ölçülerek
belirlenmiş** değerler içerirler; her satırın bir gerekçesi vardır.

### `avci_copter.parm` — Avcı drone (31 satır)

**Amaç:** Kamikaze avcı drone'un yüksek hızlı, agresif takip yapabilmesi.

```ini
ANGLE_MAX 7000          # 70° maksimum eğim

WPNAV_SPEED 3000        # 30 m/s yatay (cm/s)
WPNAV_SPEED_UP 3000     # 30 m/s tırmanış
WPNAV_SPEED_DN 3000     # 30 m/s alçalma
WPNAV_ACCEL 2000        # 20 m/s² ivme
LOIT_SPEED 3000
PSC_VELXY_P 2.0         # yatay hız kontrolcü kazancı

FS_GCS_ENABLE 0         # SITL'de GCS failsafe kapalı
FS_THR_ENABLE 0         # throttle failsafe kapalı
```

**Gerekçeler (SITL'de ölçülerek):**

| Parametre | Neden |
|-----------|-------|
| `ANGLE_MAX` | Varsayılan 30° ile "+" quad yatayda **~10.3 m/s'de takılıyordu** (aerodinamik terminal hız). 55°'de ~18.6 m/s'e çıkıyor — takibin ihtiyacı olan hız bandına giriyor. |
| `WPNAV_*` | GUIDED/pozisyon-hedefli setpoint'lerdeki hız ve ivme tavanlarını açar. Bunlar kapalıyken güdüm 25 m/s komut verse bile araç çıkmaz. |
| `FS_GCS_ENABLE 0` | Chase thread'i GCS keepalive göndermiyor; failsafe açık kalsaydı güdüm ortasında RTL tetiklenirdi. |

> ⚠️ **Mevcut değerler "limitsiz test" ayarıdır** (2026-07-25): güdüm kodunun
> gerçek davranışını görmek için firmware tavanları sonuna kadar açıldı.
> **Önceki ayarlı değerler:** `ANGLE_MAX 5500`, `WPNAV_SPEED 2200`,
> `WPNAV_SPEED_UP/DN 1000`, `WPNAV_ACCEL 600`, `LOIT_SPEED 2200`.
> Aynı karar `guidance_core.Cfg`'de de var (`IVME_TAVAN`, `YAW_HIZ_MAX`).

### `avci_plane.parm` — Hedef uçak (27 satır)

**Amaç:** Hedefi avcının yakalayabileceği hızda tutmak + V-kuyruk servo eşlemesi.

```ini
TRIM_ARSPD_CM 1500      # cruise 15 m/s
ARSPD_FBW_MIN 1200      # min 12 m/s
ARSPD_FBW_MAX 2200      # max 22 m/s
WP_LOITER_RAD 80        # dar loiter yarıçapı

SERVO1_FUNCTION 4       # Aileron
SERVO2_FUNCTION 79      # Sol V-Tail
SERVO3_FUNCTION 70      # Throttle
SERVO4_FUNCTION 80      # Sağ V-Tail
```

**Gerekçeler:**

| Parametre | Neden |
|-----------|-------|
| `TRIM_ARSPD_CM` vd. | Varsayılan SITL plane LOITER'da **22-29 m/s** yapıyordu; avcı drone ~18.6 m/s max hıza sahipti → hedef asla yakalanamıyordu. |
| `WP_LOITER_RAD 80` | Hedef daha dar bir bölgede kalsın, avcı sürekli kovalamak zorunda kalmasın. |
| `SERVO*_FUNCTION` | **V-kuyruk mixing.** ArduPilot resmi `model.sdf`'in gerektirdiği çıkış eşlemesi. ArduPlane bu fonksiyonlarla elevator + rudder komutlarını sol/sağ ruddervator'a mixler. Yanlış eşlemede uçak kalkışta yalpalar veya hiç dönmez. |

---

## 4. Kalkış için zorunlu ek parametre dosyaları

ArduCopter başlatılırken **`avci_copter.parm`'dan önce** iki ArduPilot dosyası
daha yüklenmelidir:

```bash
APT=$HOME/ardupilot/Tools/autotest
--add-param-file=$APT/default_params/copter.parm
--add-param-file=$APT/default_params/gazebo-iris.parm
--add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_copter.parm
```

> **Atlanırsa:** `FRAME_CLASS` / `FRAME_TYPE` tanımsız kalır, konsolda
> `AP: Frame: UNSUPPORTED` çıkar ve **iris kalkamaz**. Bu hatanın kanıt kaydı
> `logs/hamidiyesim-KANIT-copter-UNSUPPORTED.log` dosyasındadır.

---

## 5. Kaynak yolu (`GZ_SIM_RESOURCE_PATH`)

Gazebo modelleri bulmak için bu değişkeni kullanır:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
```

Üç yol da gereklidir:
1. **Bu depo** — `iris_cam`, `mini_talon_vtail`, `iris_with_standoffs`, `mini_talon_target`
2. **ardupilot_gazebo modelleri** — `runway` ve diğer ortak varlıklar
3. **ardupilot_gazebo world'leri** — dolaylı bağımlılıklar

`GZ_SIM_SYSTEM_PLUGIN_PATH` ise `libArduPilotPlugin.so`'yu bulmak içindir; yoksa
world yüklenir ama araçlar SITL'e hiç bağlanmaz.
