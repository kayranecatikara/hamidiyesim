# SİMÜLASYON ÇALIŞTIRMA

## Hızlı başlatma

İki terminal, iki komut. **Sıra önemli: önce A, A bitince B.**

**Terminal A** — Gazebo + iki SITL (~50 s, komut satırı geri gelince hazır)

    cd ~/projects/hamidiyesim
    GZ_HEADLESS=1 bash scripts/start_harmonic.sh

**Terminal B** — GCS

    cd ~/projects/hamidiyesim
    bash scripts/gcs.sh              # bbox güdümü (varsayılan, GERÇEK SİSTEM)
    bash scripts/gcs.sh gt           # teşhis: algı girdisi Gazebo gerçek pozundan
    bash scripts/gcs.sh takip        # bbox + HybridSORT + kilitli-ID

`gcs.sh` **uçuş bekçisini de başlatır** (`tools/ucus_bekci.py`): sağlık bandı
dışına sürekli çıkan durumu canlı yakalar ve `[BEKCI] İHLAL: ...` basar. Salt
gözlemdir, uçuşu kesmez — o koşunun verisini geçersiz saymak okuyanın kararı.
Kapatmak için `AVCI_BEKCI=0 bash scripts/gcs.sh takip`.

`gcs.sh` ROS ortamını, `AVCI_GZ_CAMERA`/`AVCI_NO_BROWSER` değişkenlerini ve
8000 portu temizliğini kendi yapar — elle `export` etmeye gerek yok.

Arayüz: <http://localhost:8000>

**Durdurma:**

    bash scripts/start_harmonic.sh stop     # Terminal A (Ctrl+C İŞE YARAMAZ)
    bash scripts/start_harmonic.sh durum    # ne çalışıyor — hiçbir şeyi öldürmez
                                            # Terminal B: Ctrl+C yeterli

---

### Neden 50 s ve neden sıra önemli

- Sürenin ~46 s'i ArduPilot SITL'in kendi açılışı (parametre indirme, EKF, GPS
  kilidi). Kısaltılamaz: GPS kilidi olmadan araç arm edilmiyor. Script kör
  `sleep` yapmaz, `EKF3 IMU0 is using GPS` satırını bekler; hazır olamazsa
  çıkış kodu 1 verir.
- B'yi A'dan önce başlatırsanız `start_harmonic.sh` açılış temizliği onu
  öldürür; öldürmese bile Gazebo'dan önce açılan gz kamera aboneliği geri
  gelmez ve kamera hiç görüntü vermez.
- `A && B` zincirlemesi çalışır ama komut satırını bloklar; ayrı terminal daha
  kullanışlı.
- Terminal A'da Ctrl+C işe yaramaz: süreçler `setsid` ile ayrı oturumda.

> ⚠ **`pkill -9 -f 'gz sim|sim_vehicle|mavproxy'` KULLANMAYIN** — desen çağıran
> kabuğun kendi komut satırında da eşleşir ve kabuğunuzu öldürür. "Durdurdum ama
> hâlâ çalışıyor" şikâyetinin kaynağı buydu. `stop`/`durum` güvenlidir.

---

## Hazır olma işaretleri

Terminal B'de bu satırları görün:

    [GCS] YOLO detector hazır (avci_yolo.pt)
    [GCS] ✓ Iris kamerasından ilk görüntü!
    [GCS] ✓ Talon (hedef İHA) kamerasından ilk görüntü!
    [GCS] gz-transport kamera dinleniyor (/iris_cam/image, Harmonic) — en-son-kare-kazanır

Kamera hattı 10 s'de bir sağlık satırı basar; **düşme oranı sürekli yüksekse**
işleme kameraya yetişmiyordur (gecikme birikmez ama kare kaybedilir):

    [GZ-CAM] 30.0 kare/s geldi, 28.4 işlendi, 16 düştü (%5) — gecikme birikmiyor

GT modunda ayrıca (görev başlatınca):

    [LEAD] ⚠ GT MODU (AVCI_GT_ROT=on) — güdüm girdileri Gazebo GERÇEK kutusundan

Kamera satırları gelmiyorsa render edilmiyordur — Sorun giderme'ye bakın.
(`--headless-rendering` pencereyi kapatır ama kameraları render etmeye devam
eder; yalnız `gz sim -s` verilirse kamera topic'leri boş kalır.)

---

## Ayrı terminallerde (detaylı)

Bir şey ters gittiğinde hangi bileşenin sorunlu olduğunu görmek için. Sıra
önemli — her adımın hazır işaretini görmeden sonrakine geçmeyin.

> **Burada `--daemon` YOK, bilerek.** Elle başlatmanın amacı ArduCopter/ArduPlane
> çıktısını canlı görmek; `--daemon` MAVProxy'yi arka plana atıp o çıktıyı
> gizliyor. Telemetri hızı `--streamrate=25` ile korunuyor.
> `scripts/start_harmonic.sh` içinde `--daemon` **duruyor** — orada süreçler
> `nohup` ile arka planda başlatılıyor, çıktı `logs/*.log` dosyalarına yazılıyor.

### Terminal 1 — Gazebo (headless)

    cd ~/projects/hamidiyesim
    source /opt/ros/humble/setup.bash
    unset DISPLAY
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
    export GZ_SIM_RESOURCE_PATH=$HOME/projects/hamidiyesim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
    gz sim -s -r --headless-rendering -v4 sim/gazebo_harmonic/worlds/avci_harmonic.sdf

`-s` yalnız sunucu, `-r` duraklatmadan başlat, `--headless-rendering` kameraları
ekransız render et.

**Hazır işareti** (başka terminalde): `ss -lnup | grep -E ':9002|:9012'` iki
satır vermeli. 9002 açılmadan Terminal 2'ye geçmeyin, yoksa ArduCopter Gazebo'ya
bağlanamaz ve araç hiç uçmaz.

### Terminal 2 — ArduCopter (avcı iris, FDM 9002)

    cd ~/ardupilot
    APT=$HOME/ardupilot/Tools/autotest
    python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
      -I0 --sysid 5 --no-rebuild \
      --add-param-file=$APT/default_params/copter.parm \
      --add-param-file=$APT/default_params/gazebo-iris.parm \
      --add-param-file=$HOME/projects/hamidiyesim/sim/ardupilot_params/avci_copter.parm \
      --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
      --mavproxy-args="--streamrate=25"

**Hazır işareti:** `AP: Frame: QUAD/X` ve `AP: EKF3 IMU0 is using GPS`

Üç `--add-param-file` de zorunlu ve **sıra önemli** — `avci_copter.parm` en sonda
kalmalı. İlk ikisi olmadan `FRAME_CLASS`/`FRAME_TYPE` tanımsız kalır
(`AP: Frame: UNSUPPORTED`), motorlar yapılandırılamaz, kalkış başarısız olur.

### Terminal 3 — ArduPlane (hedef Talon, FDM 9012)

    cd ~/ardupilot
    python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 \
      -I1 --sysid 2 --no-rebuild \
      --add-param-file=$HOME/projects/hamidiyesim/sim/ardupilot_params/avci_plane.parm \
      --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
      --mavproxy-args="--streamrate=25"

**Hazır işareti:** `AP: EKF3 IMU0 is using GPS`

### Terminal 4 — GCS

    cd ~/projects/hamidiyesim
    bash scripts/gcs.sh

### Terminal 5 — Mission Planner (isteğe bağlı, ekran gerektirir)

    cd ~/projects/hamidiyesim/tools/mission_planner
    mono MissionPlanner.exe        # UDP 14551'den bağlanın

---

## Pencereli (GUI) çalıştırma

Tek script:

    cd ~/projects/hamidiyesim
    bash scripts/start_harmonic.sh

Elle çalıştıracaksanız yalnız Terminal 1 değişir:

    gz sim -r -v4 sim/gazebo_harmonic/worlds/avci_harmonic.sdf

`unset DISPLAY` satırını çıkarın ama **`export DISPLAY=:1` de yazmayın** —
mevcut değerinizi kullanın. Tek ekranlı makinede sabit `:1` Gazebo'nun hiç
açılmamasına sebep olur (`Unable to open display ":1"`). Ekranınızı
`ls /tmp/.X11-unix/` ile doğrulayın.

---

## Doğrulama

Sistem çalışırken boş bir terminalde:

    cd ~/projects/hamidiyesim
    ps aux | grep -E 'gz sim|arducopter|arduplane|gcs_server' | grep -v grep
    curl -s -o /dev/null -w 'arayuz: %{http_code}\n' http://127.0.0.1:8000/
    curl -s -o /dev/null -w 'iris video: %{http_code}\n' --max-time 5 http://127.0.0.1:8000/api/video_feed/iris
    curl -s -o /dev/null -w 'talon video: %{http_code}\n' --max-time 5 http://127.0.0.1:8000/api/video_feed/plane
    curl -s http://127.0.0.1:8000/api/debug/telem | python3 -m json.tool | head -20

Arayüz `307`, videolar `200` dönmeli. Video `200` değilse headless rendering
çalışmıyordur.

### Simülasyon hızı (RTF) — ölçerken dikkat

    gz topic -e -t /stats -n 2

**SITL'LER BAĞLIYKEN ÖLÇÜN.** İki araç modeli de `<lock_step>1</lock_step>`
kullanıyor (`models/iris_cam/model.sdf`, `models/mini_talon_vtail/model.sdf`):
ArduPilotPlugin her fizik adımında SITL'den FDM paketi bekleyip bloklar. Yalnız
`gz sim` açıkken alınan RTF **anlamsızdır** — plugin her adımda boşa bekler,
sayı gerçek sistemin üçte birine kadar düşer. SITL bağlanıp sonra ölürse sim
tamamen donar ve `/stats` yayını kesilir.

Mesajdaki `real_time_factor` alanına da tek başına güvenmeyin — kayan pencere
olduğu için oynak (1.17 gibi 1'in üstünde değerler görülebilir). Güvenilir
yöntem iki örnek arasındaki **fark**:

    RTF = (sim_time₂ − sim_time₁) / (real_time₂ − real_time₁)

Ölçümden önce ~60 sn bekleyin; açılış maliyeti kümülatif oranı aşağı çeker.

**Sağlıklı değerler** (2026-08-01, RTX 3060, headless, 30 s pencere): SITL'ler
bağlı ve boşta `RTF 0.994` / kamera `30.2 FPS`; `gcs_server` + YOLO çalışırken
`RTF 0.982` / `29.7 FPS` (GPU %14). Yani YOLO yükü simülasyonu yavaşlatmıyor.
Çapraz kontrol: ölçülen FPS ≈ `30 × RTF` olmalı (kamera 30 Hz'e ayarlı).

MAVLink akışını da görmek isterseniz:

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

Beklenen: 14541 → `{5: N}` (yalnız iris), 14542 → `{2: N}` (yalnız Talon),
14550 ve 14551 → ikisi de. Telemetride iris ≈ `(0, 0, -0.19)`,
Talon ≈ `(0, 12, -0.12)` (spawn konumları).

---

## Sorun giderme

| Belirti | Çözüm |
|---------|-------|
| Kamera topic'i var ama `✓ Iris kamerasından ilk görüntü!` hiç gelmiyor | `--headless-rendering` unutulmuş — Terminal 1 komutuna ekleyin |
| Video endpoint'i `200` dönmüyor | Aynı sebep, ya da `AVCI_GZ_CAMERA=1` verilmemiş. Önce `gz topic -e -t /iris_cam/image -n 1` ile Gazebo tarafını doğrulayın |
| `libEGL` + `OGRE EXCEPTION` + `Segmentation fault` (render penceresi 11 denemede açılamıyor) | GPU/EGL erişimi yok — genelde NVIDIA sürücüsü yarım kurulu (`dpkg -l \| grep nvidia` çıktısında `iF`/`iU` satırları). Asıl çözüm sürücüyü onarmak; `nvidia-smi` çalışıyorsa hazırsınız. **`LIBGL_ALWAYS_SOFTWARE=1` İŞE YARAMAZ** — Ogre EGL cihazını ismen seçtiği için libEGL reddeder (*"Not allowed to force software rendering when API explicitly selects a hardware device"*). Geçici çözüm: `export MESA_LOADER_DRIVER_OVERRIDE=kms_swrast` (ayrıntı aşağıda) |
| Yalnız `libEGL warning: egl: failed to create dri2 screen` — arkasından Ogre hatası YOK | **Zararsız.** GPU sağlıklıyken de çıkar: EGL önce dri2'yi dener, olmayınca NVIDIA cihazına düşer ve render çalışır. Arıza işareti bu satır değil, arkasından gelen `OGRE EXCEPTION` + `Segmentation fault` |
| GCS açılışta takılıyor, `MESA-LOADER` uyarıları | `export AVCI_NO_BROWSER=1` |
| Uzak makineden arayüze erişemiyorum | SSH tüneli: `ssh -L 8000:localhost:8000 kullanici@makine` |
| `AP: Frame: UNSUPPORTED` | Terminal 2'deki üç `--add-param-file` eksik veya sırası bozuk |
| ArduCopter bağlanmıyor, araç düşüyor | Terminal 2, Gazebo hazır olmadan başlatılmış — 9002'yi bekleyin |
| Uçak kendi kendine hareket ediyor / komutlar tutmuyor | Önceki oturumdan kalan `run_plane_scenario` süreci — temizlik komutunu çalıştırın |
| `Address already in use` (8000) | `fuser -k 8000/tcp` (`gcs.sh` bunu zaten yapar) |
| `HybridSORT yüklenemedi (No module named 'boxmot')` | `pip install boxmot==19.0.0` — sürüm sabit, 20+ numpy≥2 dayatıp torch/cv2 kurulumunu bozar |
| GT modu açık ama loglarda `duzeltme` 1.0 değil | Bayrak geçmemiş. `gcs.sh` kullanın; elle çalıştırıyorsanız `AVCI_GT_ROT=on` **aynı satırda** olmalı |
| GUI'de `Unable to open display ":1"` | `export DISPLAY=:1` satırını silin |

### GPU'suz makinede çalıştırma (`kms_swrast`)

Ekran kartı yoksa ya da sürücü bozuksa Mesa'nın yazılım rasterleştiricisi
kullanılabilir — sistemde zaten kurulu, ek paket gerekmez:

    export MESA_LOADER_DRIVER_OVERRIDE=kms_swrast
    GZ_HEADLESS=1 bash scripts/start_harmonic.sh

Doğrulandı: Gazebo çökmeden koşuyor ve kamera topic'leri gerçekten kare
yayınlıyor (`/iris_cam/image`, `/talon_cam/image`).

> **Yalnız geliştirme ve duman testi için.** Yazılım render'ı GPU'ya göre kat
> kat yavaştır; simülasyon gerçek zamanın çok altına düşer. Bu modda alınan
> uçuş logları GPU'lu koşulardaki sayılarla **karşılaştırılamaz** — ıska
> ölçümü, kazanç taraması veya faz kapısı ayarı için kullanmayın. Kodun
> çalıştığını görmek için uygundur.

Sürücü onarıldığında `MESA_LOADER_DRIVER_OVERRIDE` satırını kaldırın.
