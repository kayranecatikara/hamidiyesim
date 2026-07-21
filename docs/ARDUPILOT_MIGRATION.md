# AVCI SİM — ArduPilot Migrasyonu (Tamamlandı)

Bu doküman, projenin **PX4 → ArduPilot**, **Talon → Cessna**, **QGroundControl →
Mission Planner** geçişini ve yeni çalıştırma akışını özetler.

## Ne Değişti

| Konu | Eski (PX4) | Yeni (ArduPilot) |
|------|-----------|------------------|
| Otopilot | PX4 1.16 | ArduCopter + ArduPlane (SITL, 4.6.x) |
| Drone modu | OFFBOARD (6) | GUIDED (4) |
| Plane modları | MANUAL(1), STABILIZED(7), AUTO.TAKEOFF | MANUAL(0), FBWA(5), TAKEOFF(13), LOITER(12) |
| Force ARM magic | 21196 | **2989** (disarm hâlâ 21196) |
| set_mode | main_mode + sub_mode | tek `custom_mode` |
| Telemetri | PX4 kendiliğinden yollar | `REQUEST_DATA_STREAM` ile istenir |
| Hedef uçak | Talon (beyaz) | Cessna (rc_cessna mesh, beyaz görsel) |
| Yer istasyonu | QGroundControl | Mission Planner (kurulu: `tools/mission_planner/`) |
| Görselleştirme | PX4+Gazebo Classic | ArduPilot + Gazebo Classic (`ardupilot_gazebo` plugin) |

## Port Haritası

| Port | Amaç |
|------|------|
| 14541 | Drone (ArduCopter) kontrol/telemetri — `drone_functions`, chase, strike |
| 14542 | Plane (ArduPlane) kontrol/telemetri — `plane_functions` |
| 14550 | **gcs_server** yayını (özel web GCS, `udpin` bind) |
| 14551 | **Mission Planner** yayını (MP bu portu UDP "listen" ile açar) |
| 9002/9003 | ArduPilotPlugin ↔ Gazebo FDM köprüsü (yalnızca Gazebo modu) |

> gcs_server ve Mission Planner **aynı** UDP portunu paylaşamaz (tek bind);
> bu yüzden her birine ayrı port verildi. Her iki araç da her iki porta yollar.

## Çalıştırma

### 1) SITL (built-in fizik — chase/strike/GCS için yeterli)
```bash
bash scripts/start_ardupilot_sitl.sh          # başlat (iki araç)
bash scripts/start_ardupilot_sitl.sh stop     # durdur
```
Bu, `sim/ardupilot_params/avci_copter.parm` ve `avci_plane.parm` yamalarını
otomatik yükler.

### 2) Özel Web GCS
```bash
source /opt/ros/humble/setup.bash
python3 -m control.gcs_server        # http://localhost:8000
```

### 3) Mission Planner (mono gerektirir — bir kez kurulum)
```bash
# ! sudo apt-get install -y mono-complete libgdiplus     (oturumda ! ile)
bash scripts/start_mission_planner.sh
# MP açılınca: sağ üst → UDP → port 14551 → Connect
# Üst menüden Copter (sysid 5) / Plane (sysid 2) arasında geçiş yapılır.
```

### 4) Gazebo + kamera (kamera-tabanlı tespit için)
```bash
bash scripts/start_gazebo.sh              # GUI
bash scripts/start_gazebo.sh headless     # sadece render + kamera
# Kamera ROS2 topic: /iris_cam/front_camera/image_raw
```
Gazebo fiziğinde uçan iris için SITL'i Gazebo frame ile başlatın:
```bash
cd ~/ardupilot && python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter -f gazebo-iris -I0 --sysid 5 --no-rebuild \
    --add-param-file=~/projects/avci_sim/sim/ardupilot_params/avci_copter.parm \
    --out udp:127.0.0.1:14541
```

## Önemli Parametre Bulguları (SITL'de ölçülerek)

- **ANGLE_MAX = 5500 (55°)** — Varsayılan 30° ile "+"quad yatayda ~10.3 m/s'de
  aerodinamik olarak takılıyordu; chase'in 19.5 m/s hedefi için 55° gerekti
  (~18.6 m/s'e çıkıyor). `avci_copter.parm` içinde.
- **TRIM_ARSPD_CM = 1500** — Hedef uçak varsayılanda LOITER'da 22-29 m/s
  uçuyordu; avcı drone yakalayamıyordu. `avci_plane.parm` ile ~15 m/s'e çekildi.
- **FS_GCS_ENABLE/FS_THR_ENABLE = 0** — Chase thread'i keepalive göndermediği
  için failsafe'ler sim'de kapatıldı.

## Doğrulanan Senaryolar

1. ✅ ArduCopter GUIDED kalkış/iniş (NAV_TAKEOFF)
2. ✅ ArduPlane TAKEOFF + uçuş + LOITER
3. ✅ `mav_common` arm/mode/telemetri (ACK'lerle)
4. ✅ `drone_functions` kalkış/hareket/yaw/iniş
5. ✅ `plane_functions`/`plane_patterns` arm/RC-override/FBWA dönüş
6. ✅ İki araç eş zamanlı, 14550 & 14551'de ikisi de görünür
7. ✅ Chase 164m→19.8m kapattı (SPRINT→APPROACH→LOCK→STRIKE); Strike PN 133m→2.9m
8. ✅ gcs_server telemetri + chase/strike API (LOCK'a 4.3m'ye kadar kilit)
9. ✅ Mission Planner 14551'de her iki aracı yayınlıyor (port katmanı)
10. ✅ Gazebo iris kamerası Cessna'yı tespit ediyor; gcs_server MJPEG uçtan uca

## Gazebo Modu (kamera dahil tam sistem)

iris'i Gazebo fiziğinde uçurup kamera görüntüsü almak için `-f gazebo-iris`
kullanılır; hedef uçak (Cessna) ArduPlane built-in fizikte uçar ve
`cessna_pose_relay` node'u onu Gazebo'da hareket ettirir.

Bileşenler (sırayla, her biri ayrı terminal):
1. Gazebo: `gazebo --verbose sim/gazebo/worlds/avci_arducopter.world`
   (GAZEBO_PLUGIN/MODEL/RESOURCE_PATH + ROS2 + `/usr/share/gazebo/setup.sh`)
2. ArduCopter: `sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --sysid 5 ...`
3. ArduPlane: `sim_vehicle.py -v ArduPlane -f plane -I1 --sysid 2 ... --out udp:14552`
4. Cessna relay: `python3 -m control.cessna_pose_relay`
5. gcs_server, 6. Mission Planner

Doğrulanan bileşenler:
- ✅ iris Gazebo fiziğinde kalkıyor (gazebo-iris, EKF Gazebo sensörüyle)
- ✅ Kamera `/iris_cam/front_camera/image_raw` ~20-30 Hz
- ✅ `cessna_pose_relay`: ArduPlane NED → Gazebo ENU, `/gazebo/set_entity_state`
- ✅ chase frame-hizalı (iris Gazebo + plane built-in aynı SITL home)
- ✅ Detaylı Cessna mesh'i (jMAVSim `cessna.obj`, 49k yüz) — kamera %8.8 kaplama

Notlar:
- `cessna_pose_relay` içinde `VISUAL_OFFSET_EAST_M=25` — iki araç aynı home'da
  başladığından yerde üst üste binmesini önler (görsel ayrım).
- Cessna mesh oryantasyonu doğru; uçuş yönü relay yaw'ıyla aktarılır. Chase'te
  ters/yan uçarsa relay yaw'ına ±π offset eklenebilir.

## Kalan / Manuel Adımlar

- **Mono kurulumu** (sudo): Mission Planner GUI için. Kuruldu (`mono-complete`).
- **Gazebo kamera-chase geometrisi**: chase iris'i hedeften yukarı konumlar
  (`vert_offset`); kamera 25° yukarı tiltli. Kamerada hedefi tam merkezlemek
  için bu geometri ince ayarı yapılabilir (chase GPS-tabanlı çalıştığından
  kritik değil).

## Yedek

PX4 dönemi `control/` ve `vision/` modülleri `backup_px4_stack/` altında.
