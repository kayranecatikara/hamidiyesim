# SİMÜLASYON ÇALIŞTIRMA KOMUTLARI

Kurulum tamamlandıktan sonra TÜM sistemi bu dosyadaki komutlarla çalıştırırsınız.
Her blok **ayrı bir terminalde**, buradaki **sırayla** başlatılır.

---

## Temizlik

Boş bir terminalde (çalışan bileşenlerin olduğu terminalde değil):

```bash
pkill -9 -f 'gz sim|sim_vehicle|mavproxy|arducopter|arduplane|control.gcs_server'; sleep 3
```

---

## TERMİNAL 1 — Gazebo Harmonic (ilk açılır, ~15 sn bekle)

```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
export DISPLAY=:1
gz sim -r -v4 sim/gazebo_harmonic/worlds/avci_harmonic.sdf
```

> Not: `DISPLAY=:1` GUI'nin ikinci X ekranında olduğu makineler içindir; tek
> ekranlı kurulumda bu satırı atlayabilirsiniz.

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
  --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
```

**Hazır olduğunun işareti:** `AP: Frame: QUAD/X` ve `AP: ArduPilot Ready`.

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
python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 -I1 --sysid 2 --no-rebuild --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_plane.parm --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
```

---

## TERMİNAL 4 — GCS Server (kamera + web arayüz + görev)

```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export AVCI_GZ_CAMERA=1
fuser -k 8000/tcp
python3 -m control.gcs_server
```

Web arayüz: <http://localhost:8000> — YOLO detector otomatik yüklenir
(`vision/models/avci_yolo.pt`).

---

## TERMİNAL 5 — Mission Planner

```bash
cd ~/projects/avci_sim/tools/mission_planner
export LD_LIBRARY_PATH="$HOME/projects/avci_sim/tools/mission_planner/native_libs:$LD_LIBRARY_PATH"
mono MissionPlanner.exe
```

---

## Hızlı alternatif — tek script

Yukarıdaki 1-2-3. terminalleri tek komutla başlatmak için:

```bash
cd ~/projects/avci_sim
bash scripts/start_harmonic.sh          # durdurmak için: bash scripts/start_harmonic.sh stop
```

Ardından Terminal 4 (gcs_server) ve gerekiyorsa Terminal 5 (Mission Planner)
yine ayrı terminallerde başlatılır.
