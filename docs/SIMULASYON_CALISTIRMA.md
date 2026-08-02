# SİMÜLASYON ÇALIŞTIRMA

## Hızlı başlatma (headless)

İki terminal yeter. Sırayla:

**Terminal A** — Gazebo + iki SITL (~50 sn sürer, bitince kendi kendine döner)

    cd ~/projects/avci_sim
    GZ_HEADLESS=1 bash scripts/start_harmonic.sh

Script zaten kendi başında eski süreçleri temizliyor; temizleyemezse **yenisini
başlatmaz** ve neyin ayakta kaldığını PID'leriyle yazar.

> **Beklemek zorunda mısınız? Hayır — script sizin yerinize bekliyor.**
> Eskiden burada kör bir `sleep 25` vardı ve "SITL'lerin açılması bekleniyor
> (25s)" yazıyordu. Ölçüldü (2026-08-02): araçların EKF+GPS kilidi ~50 s'de
> geliyor, yani script 25 s'de "hazır" derken SITL **hâlâ açılıyordu**.
> Artık aracın kendi çıktısındaki `EKF3 IMU0 is using GPS` satırı bekleniyor.
> Script hazır olmadan dönmez, olamazsa **çıkış kodu 1** verir.
>
**Bu kadar beklemek şart mı? Evet — ve kısaltılamaz.** Faz faz ölçüldü
(2026-08-02, script bunu her koşuda kendi basıyor):

| faz | süre |
|---|---:|
| Gazebo açılması + FDM portu | **~4 s** |
| ArduPilot EKF + GPS kilidi | **~46 s** |
| **Terminal A toplam** | **~50 s** |
| Terminal B (`gcs_server`, iki kamera görüntüsü dahil) | **~5 s** |

Sürenin %90'ı ArduPilot SITL'in kendi açılışı: 1421 parametrenin FTP ile
inmesi, EKF ilklenmesi, tilt hizalaması, GPS origin. Bizim eklediğimiz bir
gecikme değil ve **kalkış zaten bundan önce mümkün değil** — GPS kilidi
olmadan arm edilmiyor. `SIM_GPS1_LCKTIME` ve `GPS1_DELAY_MS` zaten 0.

> ⚠ **Terminaller AYNI ANDA başlatılamaz, sıra önemli.** Ölçüldü:
> `gcs_server` Gazebo'dan **önce** başlatılırsa (1) `start_harmonic.sh`'ın
> açılıştaki temizliği onu öldürür, (2) öldürmese bile Gazebo'dan önce açılan
> gz kamera aboneliği geri gelmiyor — `✓ ... ilk görüntü` satırları hiç
> çıkmıyor, arayüzde kamera kararıyor. Önce A, sonra B.
>
> Zincirlemek (`A && B`) çalışır ama bir faydası yok: toplam süreyi ~5 s
> uzatır ve `gcs_server` aynı terminalde ön planda çalıştığı için **komut
> satırı geri gelmez, çıktı akmaya devam eder**. Ayrı terminaller daha iyi.

**Terminal B** — GCS (Terminal A "Tam sistem hazır" yazdıktan sonra)

    cd ~/projects/avci_sim
    source /opt/ros/humble/setup.bash
    export AVCI_GZ_CAMERA=1
    export AVCI_NO_BROWSER=1
    fuser -k 8000/tcp 2>/dev/null
    python3 -m control.gcs_server

Sonra tarayıcıda: <http://localhost:8000>

**Durdurmak için:**

    cd ~/projects/avci_sim
    bash scripts/start_harmonic.sh stop        # Terminal A tarafı
                                               # Terminal B: Ctrl+C yeterli

Gerçekten durdu mu — **kontrol edin**, bu komut hiçbir şeyi öldürmez:

    bash scripts/start_harmonic.sh durum

`stop` artık **doğrulayarak** çalışıyor: öldürür, tekrar bakar, hâlâ ayakta
kalan varsa `✓ Durduruldu` yerine PID listesi basar ve **çıkış kodu 1** döner.

> ⚠ **Terminal A'da Ctrl+C İŞE YARAMAZ.** Script Gazebo'yu ve iki SITL'i
> `setsid ... &` ile başlatıp kendisi çıkar; süreçler ayrı bir oturumda olduğu
> için Ctrl+C onlara hiç ulaşmaz. Ölçüldü: Ctrl+C sonrası `gz sim`,
> 2× `arduplane`, 2× `sim_vehicle`, 4× `mavproxy` hâlâ ayaktaydı.
> **Terminal B (gcs_server) ön planda çalıştığı için orada Ctrl+C doğrudur.**

> ⚠⚠ **`pkill -9 -f 'gz sim|sim_vehicle|mavproxy|...'` KULLANMAYIN.**
> Bu belgede eskiden böyle bir satır vardı ve **kendi kabuğunuzu öldürüyordu**:
> `pkill -f` deseni *çağıran kabuğun komut satırında* da arar, o satırda da
> `sim_vehicle`, `gz sim`, `mavproxy` kelimeleri geçiyor. Sonuç ölçüldü
> (2026-08-02): pkill kabuğu öldürüyor → `sleep 3 && ... start_harmonic.sh`
> **hiç çalışmıyor**, ya da `stop` döngüsü ortasında kesiliyor ve geri kalan
> desenler ('mavproxy', 'gz sim') uygulanmadan kalıyor. Ekranda hata yok,
> süreçler ayakta. "Durdurdum ama hâlâ çalışıyor" şikâyetinin kaynağı buydu.
> `stop`/`durum` kendi ata süreçlerini listeden düşer; güvenli olan onlardır.

---

## Hazır olma işaretleri

Terminal B'de bu dört satırı görün:

    [GCS] YOLO detector hazır (avci_yolo.pt)
    [GCS] YOLO pose hazır (avci_pose.pt)
    [GCS] ✓ Iris kamerasından ilk görüntü!
    [GCS] ✓ Talon (hedef İHA) kamerasından ilk görüntü!

Son iki satır gelmiyorsa kamera render edilmiyordur — Sorun giderme'ye bakın.

> **Headless ≠ render yok.** Kameralar bu projede güdümün kalbi: YOLO tespit ve
> pose `/iris_cam/image`'dan besleniyor. `--headless-rendering` bayrağı pencereyi
> kapatır ama kameraları ekransız render etmeye devam eder. Sadece `gz sim -s`
> verilirse kamera topic'leri boş kalır ve görsel güdüm hiç çalışmaz.

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

    cd ~/projects/avci_sim
    source /opt/ros/humble/setup.bash
    unset DISPLAY
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
    export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
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
      --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_copter.parm \
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
      --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_plane.parm \
      --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
      --mavproxy-args="--streamrate=25"

**Hazır işareti:** `AP: EKF3 IMU0 is using GPS`

### Terminal 4 — GCS

Yukarıdaki "Terminal B" ile aynı.

### Terminal 5 — Mission Planner (isteğe bağlı, ekran gerektirir)

    cd ~/projects/avci_sim/tools/mission_planner
    mono MissionPlanner.exe        # UDP 14551'den bağlanın

---

## Pencereli (GUI) çalıştırma

Tek script:

    cd ~/projects/avci_sim
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

    cd ~/projects/avci_sim
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
| `Address already in use` (8000) | `fuser -k 8000/tcp` |
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
