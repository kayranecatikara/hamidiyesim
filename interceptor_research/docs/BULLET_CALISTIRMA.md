# Mermi gövdeli taretli interceptor'ı çalıştırma

`bullet_net_interceptor` — dikey mermi gövde, **tepesinde** 2 eksenli taret,
namlu ileri bakar, ağ ileri atılır.

Bu doküman **yalnızca bu drone'u** çalıştırmak içindir. Projenin geri kalanı
(aday gövde kıyası, `repos/` altındaki 11 kaynak repo, showcase) **gerekmez** —
`scripts/00_clone_all.sh` çalıştırmayın, 3 GB indirmenize gerek yok.

---

## Gereken

| | Sürüm | Not |
|---|---|---|
| Gazebo Harmonic | gz-sim 8.14 | `gz sim --versions` |
| gz-transport | 13 | Python bağları: `python3-gz-transport13` |
| Python | 3.10+ | `numpy`, `Pillow` (sadece görüntü almak için) |
| cmake + g++ | — | eklentileri derlemek için |
| ArduPilot SITL | ArduCopter | **isteğe bağlı** — sadece uçmak için |

`ardupilot_gazebo` kurulu değilse sorun değil. Gazebo şu **hatayı** basar:

```
[Err] [SystemLoader.cc:92] Failed to load system plugin [ArduPilotPlugin] :
      Could not find shared library.
```

ama dünya yüklenmeye devam eder ve **taret + ağ + yakalama çalışır** — test
edildi: taret komut −8.00° → ölçülen −8.02°, `YAKALANDI: target_box` (1.6 sn).
Sadece motorlar dönmez, yani SITL ile **uçuş** yapılamaz.

---

## Kurulum

```bash
git clone -b incelenmeli-vakkas-entegre \
    https://github.com/kayranecatikara/hamidiyesim.git
cd hamidiyesim/interceptor_research

# Eklentileri derle (NetLauncherPlugin + NetCapturePlugin) — bir kez
cd plugins && mkdir -p build && cd build && cmake .. && make -j4 && cd ../..

# Ortam
source scripts/env.sh
```

`ardupilot_gazebo` başka bir yerdeyse, `source` etmeden önce:
```bash
export ARDUPILOT_GAZEBO_ROOT=/yol/ardupilot_gazebo
```

Model deposuda hazır geliyor; yeniden üretmek isterseniz:
```bash
python3 scripts/21_build_bullet_interceptor.py
```
Bu script `models/interceptors/cand_bullet/` (ham gövde, burnunda koni) alır,
koniyi kaldırır, taret bloğunu enjekte eder ve
`models/interceptors/bullet_net_interceptor/` üretir.

---

## Çalıştırma

**Görsel (GUI):**
```bash
source scripts/env.sh
gz sim -r worlds/bullet_net_test.sdf
```

Ayrı bir terminalde (yine `source scripts/env.sh` sonrası):
```bash
# Tareti çevir — pan ±100°, tilt -60°..+30° (negatif = yukarı)
python3 scripts/turret_aim.py 0 -8 --model bullet_net_interceptor

# Ağı at
python3 scripts/fire_net.py --hiz 20 --model bullet_net_interceptor
```

**Başsız (ölçüm):**
```bash
source scripts/env.sh
gz sim -s -r --headless-rendering worlds/bullet_net_test.sdf &

python3 scripts/turret_aim.py 0 -8 --model bullet_net_interceptor
python3 scripts/turret_state.py bullet_net_test bullet_net_interceptor   # açıyı oku
python3 scripts/net_track.py --dunya bullet_net_test --sure 6            # yörünge
python3 scripts/wait_capture.py --sure 20 &                              # yakalamayı bekle
python3 scripts/fire_net.py --hiz 20 --model bullet_net_interceptor
```

**Görüntü almak:**
```bash
python3 scripts/52_action_shots.py --topic /action/view \
        --model bullet_net_interceptor --kare 6 --aralik 0.10
```

---

## Beklenen çıktı

Bu makinede ölçülenler (Gazebo Harmonic 8.14, 1 ms adım):

```
taret        komut -8.00°  ->  ölçülen -8.03°
ağ başlangıç x=0.119  z=0.621
ağ bitiş     x=27.955  ileri menzil 27.84 m
yakalama     YAKALANDI: target_box  (~1.6 sn)
```

Sapma görürseniz muhtemel sebep: `plugins/build` derlenmemiş (ağ namludan
ayrılmaz) veya `GZ_SIM_RESOURCE_PATH` eksik (`net_cone` bulunamaz).

---

## Arayüz

```
/model/bullet_net_interceptor/joint/turret_yaw_joint/0/cmd_pos     gz.msgs.Double, radyan
/model/bullet_net_interceptor/joint/turret_pitch_joint/0/cmd_pos   gz.msgs.Double, radyan
/bullet_net_interceptor/net/fire                                   gz.msgs.Double, çıkış hızı m/s
/net/captured                                                      gz.msgs.StringMsg, yakalanan model
/bullet_net_interceptor/nose_camera/image                          burun kamerası
```

Script kullanmadan, doğrudan:
```bash
gz topic -t /model/bullet_net_interceptor/joint/turret_pitch_joint/0/cmd_pos \
         -m gz.msgs.Double -p 'data: -0.14'
gz topic -t /bullet_net_interceptor/net/fire -m gz.msgs.Double -p 'data: 20'
```

---

## Uçurmak isterseniz (doğrulanmadı)

`config/bullet_net_interceptor.param` ArduCopter parametrelerini taşır, FDM
portu **9032/9033**. Kütle bütçesinden türetildi ama **SITL'de sınanmadı** —
bu projede henüz hiçbir model uçurulmadı. Ayrıntı ve uyarılar:
[`../README.md`](../README.md) → "2. tasarım (mermi gövde) — kütle bütçesi" ve
"Bilinen sınırlar".

```bash
sim_vehicle.py -v ArduCopter --model JSON \
    --add-param-file=config/bullet_net_interceptor.param
```
