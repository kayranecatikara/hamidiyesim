# 13 — `gcs_server.py` Yer Kontrol İstasyonu

**Yol:** `control/gcs_server.py` · **1170 satır**
**Rol:** Projenin ana çalıştırılabilir süreci. Kamera, telemetri, görev kontrolü
ve web arayüzü tek yerde.

```bash
source /opt/ros/humble/setup.bash
export AVCI_GZ_CAMERA=1
python3 -m control.gcs_server        # → http://localhost:8000
```

---

## Thread mimarisi

```
uvicorn ana thread          → HTTP + WebSocket (asyncio)
gz_iris_camera_thread       → iris kamerası 30 Hz + YOLO + pose
gz_talon_camera_thread      → Talon burun kamerası 30 Hz (tespit YOK)
mavlink_listener (asyncio)  → plane telemetrisi, UDP 14550
_iris_telem_worker          → iris telemetrisi, UDP 14541  ← güdümde DURUR
_chase_thread               → kalkış + supervisor.run_hybrid
_visual_thread              → kalkış + run_visual_lead (izole test)
_manual_control_thread      → FBWA RC override, 10 Hz
[alt süreç] run_plane_scenario → hedef uçuş senaryosu
```

### Port paylaşım deseni

Tek UDP portunu iki dinleyici bind edemez. Güdüm başlarken:

```python
stop_iris_telem()          # worker'ı durdur, 14541 serbest kalsın
time.sleep(0.3)
conn = df_connect_drone(port=14541)     # güdüm kendi bağlantısını açar
...
finally:
    start_iris_telem()     # güdüm bitince worker geri gelir
```

Güdüm sırasında telemetriyi güdüm döngüsünün kendi `_ArasState` sınıfı okur.

---
---

# Telemetri katmanı

## `_process_mavlink_msg(msg, vehicle_name)`

Gelen MAVLink mesajını `telemetry_state`'e yazar.

İşlenen mesaj tipleri:

| Mesaj | Yazılan alanlar |
|-------|-----------------|
| `LOCAL_POSITION_NED` | `x, y, z, vx, vy, vz, speed` |
| `ATTITUDE` | `roll, pitch, yaw` (dereceye çevrilir) |
| `GLOBAL_POSITION_INT` | `lat, lon, alt_amsl` → sonra `_frame_off_update()` |
| `HEARTBEAT` | `armed, mode` |

---

## `_frame_off_update()` — çerçeve ofseti kalibrasyonu

**Sorun:** iris ve Talon **ayrı SITL instance'ları**. Her birinin
`LOCAL_POSITION_NED` origin'i kendi EKF'sine göredir. İkisini aynı çerçevede
karşılaştırmadan güdüm yapılamaz.

```python
ip = telemetry_state["iris"]
pp = telemetry_state["plane"]
if ip["lat"] == 0.0 or pp["lat"] == 0.0:
    return                                    # iki GPS de gelmeden kalibre etme

rel_n = (pp["lat"] - ip["lat"]) * _M_PER_DEG
rel_e = (pp["lon"] - ip["lon"]) * _M_PER_DEG * math.cos(math.radians(ip["lat"]))
sn = (ip["x"] + rel_n) - _plane_local_raw["x"]
se = (ip["y"] + rel_e) - _plane_local_raw["y"]
```

**Mantık:** GPS lat/lon **mutlak** ve ortaktır. Plane'in iris çerçevesindeki
gerçek konumu GPS'ten hesaplanır; ham LOCAL değeriyle farkı = EKF origin ofseti.

```python
if _frame_off["samples"] == 0:
    _frame_off.update(n=sn, e=se, d=sd)       # ilk örnek doğrudan
else:
    a = 0.1
    _frame_off["n"] = (1 - a) * _frame_off["n"] + a * sn    # EMA
    ...
_frame_off["samples"] += 1
if not _frame_off["ok"] and _frame_off["samples"] >= 20:
    _frame_off["ok"] = True                   # 20 örnekten sonra güvenilir
```

Docstring: *"Ofset sabittir (orijinler hareket etmez); EMA yalnız GPS gürültüsünü
süzer."*

### Dikey ofset neden sıfırlandı — kök neden düzeltmesi

```python
sd = 0.0    # AMSL KULLANMA
```

Koddaki yorum (2026-07-25) tam bir hata analizi:

> *"İki SITL'in EKF orijin İRTİFALARI farklı (araç-tipi varsayılan home alt'ları
> ~12.7m ayrık; start_harmonic.sh --home vermiyor). GPS lat/lon gerçek yatay
> konumu doğru verirken alt_amsl bu 12.7m sahte ofseti taşıyordu: kamera+ham
> yerel-z 'hedef ALTTA' derken AMSL 'ÜSTTE' diyordu → güdüm drone'u hedefin
> üstüne çıkarıp görsel teması ENGELLİYORDU."*

**Belirti:** Drone hedefin üstüne çıkıyor, kamera hedefi göremiyor, görsel faza
hiç geçilmiyor.
**Kök neden:** Sahte 12.7 m dikey ofset.
**Çözüm:** İki araç da aynı düz zemine spawn olduğu için dikey origin farkı
gerçekte **sıfırdır**. Yatay GPS kalibrasyonu korunur, dikey doğrudan ham
yerel-z ile karşılaştırılır.

---

## `mavlink_listener()` (asyncio) / `_iris_telem_worker()` (thread)

| Fonksiyon | Port | Araç |
|-----------|------|------|
| `mavlink_listener` | `udpin:0.0.0.0:14550` | Plane (ana GCS broadcast) |
| `_iris_telem_worker` | `udpin:0.0.0.0:14541` | iris |

`start_iris_telem()` / `stop_iris_telem()` worker'ı yönetir.

`_read_iris_telem_from_conn(conn)` — güdüm aktifken chase/visual bağlantısı
üzerinden iris telemetrisini okuyup `telemetry_state`'e yazar. Non-blocking.

---
---

# Simülasyon efektleri

## `_apply_gps_noise(clean_x, clean_y, clean_z, clean_yaw)`

GPS jamming/spoofing simülasyonu. Tek kaydırıcı (`0.0-1.0`) dört rejimi sürer.

```python
if lvl <= 0.001:
    _noisy_plane_telem = {..., "frozen": False}       # temiz
    return

if lvl >= 0.999:
    _noisy_plane_telem["frozen"] = True               # tamamen donuk
    return

# Freeze olasılığı (quadratic: %50 seviyede %25 freeze)
freeze_prob = lvl * lvl
if random.random() < freeze_prob:
    _noisy_plane_telem["frozen"] = True
    return

noise_std = lvl * 20.0
nx = clean_x + random.gauss(0, noise_std)
ny = clean_y + random.gauss(0, noise_std)
nz = clean_z + random.gauss(0, noise_std * 0.3)       # irtifada daha az gürültü
nyaw = clean_yaw + random.gauss(0, lvl * 30)

if lvl > 0.7 and random.random() < 0.15:              # spoofing
    jump = 30.0 * lvl
    ...
```

| Seviye | Davranış |
|--------|----------|
| 0-30% | Hafif gürültü (±2 m), veri akar |
| 30-70% | Orta gürültü (±10 m) + freeze olasılığı |
| 70-99% | Şiddetli (±20 m) + %70 freeze + **büyük atlamalar** (spoofing) |
| 100% | Tamamen donuk (son bilinen konum) |

### Üç tasarım kararı

**1. Kuadratik freeze olasılığı (`lvl²`):** Doğrusal olsaydı %10 seviyede bile
%10 donma olurdu — "hafif gürültü" rejimi hiç yaşanmazdı. Kuadratik ile %50
seviyede %25 donma olur, düşük seviyeler temiz kalır.

**2. Dikey gürültü 0.3 katı:** Gerçek GPS'te dikey hata yataydan azdır
(uydu geometrisi). Simülasyon bunu yansıtır.

**3. `frozen` bayrağı:** Donmuş veri işaretlenir. `gps_guidance` bunu görüp
`DROPOUT` durumuna geçer → **supervisor görsel faza kaçar**. Kaydırıcı tam da
bu jamming yedeğini test etmek için var — güdüm zincirinin uçtan uca sınavı.

---

## Video parazit (`_video_noise_level`)

`process_iris_frame` içinde uygulanır. Kaydırıcı 0.0-1.0. Görüntüye gürültü
ekler — görsel güdümün bozuk görüntüde nasıl davrandığını test etmek için.

---
---

# Kamera katmanı

## `gz_iris_camera_thread()`

```python
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.image_pb2 import Image as GzImage
except Exception as e:
    print(f"[GCS] gz-transport Python yok, Harmonic kamera atlandı: {e}")
    return                                        # zarif düşüş

def cb(msg):
    wall_recv = time.time()                       # ← DUVAR saati
    stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9   # ← SİM saati
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    process_iris_frame(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR),
                       stamp=stamp, wall_recv=wall_recv)

topic = os.environ.get("AVCI_GZ_CAMERA_TOPIC", "/iris_cam/image")
node = GzNode()
node.subscribe(GzImage, topic, cb)
while True:
    time.sleep(1)                                 # node'u canlı tut
```

### İki saat burada damgalanır

| Damga | Kaynak | Kullanıldığı yer |
|-------|--------|------------------|
| `stamp` | `msg.header.stamp` (**sim saati**) | `guidance_core` dt hesabı, filtreler, PN türevi |
| `wall_recv` | `time.time()` (**duvar saati**) | `visual_lead` bayat kare tespiti |

Bu ayrım tüm güdüm zincirinin doğruluğunun temelidir — karıştırılırsa filtre
zaman sabitleri ve gecikme ölçümü bozulur.

### `RGB2BGR` dönüşümü
gz-transport RGB verir, OpenCV BGR bekler. Atlanırsa renk kanalları ters olur ve
YOLO modeli (BGR ile eğitildiği için) çuvallar.

### `while True: sleep(1)`
gz-transport `subscribe` non-blocking'dir; thread canlı kalmazsa node yok edilir
ve abonelik düşer.

---

## `process_iris_frame(img, stamp=None, wall_recv=None)`

```python
# İki model de TEMİZ kare üzerinde çıkarım yapar; overlay'ler sonra çizilir
# (detection kutusu çizilmiş kare pose'a girerse çıkarımı bozar).
det = pose = None
if _yolo_detector is not None:
    det = _yolo_detector.detect_talon(img)
    set_detection(det)
if _pose_detector is not None:
    pose = _pose_detector.detect_pose(img)
    set_pose_detection(pose, stamp=stamp, wall_recv=wall_recv)   # ← visual_lead uyanır
# ... overlay çiz ... video parazit ... JPEG kodla
```

**Sıra kritik:**
1. **Temiz** kare → detection
2. **Temiz** kare → pose
3. `set_pose_detection` → `Condition.notify_all()` → `visual_lead` uyanır
4. Overlay çizimi (detection kutusu + pose keypoint/iskelet)
5. Video parazit
6. JPEG → `latest_frames["iris"]`

Overlay çıkarımdan **sonra** gelmezse: detection kutusu çizilmiş kare pose
modeline girer ve keypoint tahminleri bozulur.

---

## `gz_talon_camera_thread()` / `process_plane_frame(img)`

```python
def process_plane_frame(img):
    """Hedef İHA (Talon) burun kamerası: ham görüntü → MJPEG. Iris'ten farkı:
    tespit/overlay YOK (bu hedefin kendi görüşü, avcının değil)."""
    _, buf = cv2.imencode('.jpg', img)
    if latest_frames["plane"]["data"] is None:
        print("[GCS] ✓ Talon (hedef İHA) kamerasından ilk görüntü!")
    latest_frames["plane"]["data"] = buf.tobytes()
    latest_frames["plane"]["id"] += 1
```

Çok daha basit — tespit yapılmaz. Bu kamera operatöre hedefin bakış açısını
göstermek için, güdüme girmez.

---

## `class CameraSubscriber(Node)` / `ros2_spin_thread()`

**Gazebo Classic fallback.** `rclpy` üzerinden `sensor_msgs/Image` dinler,
`cv_bridge` ile OpenCV'ye çevirir. Harmonic'te kullanılmaz ama kod korunmuş.

---

## `generate_mjpeg()` / `video_feed(vehicle)`

```python
@app.get("/api/video_feed/{vehicle}")
def video_feed(vehicle: str):
    return StreamingResponse(generate_mjpeg(vehicle),
        media_type="multipart/x-mixed-replace; boundary=frame")
```

`multipart/x-mixed-replace` — tarayıcının `<img>` etiketiyle doğrudan
tükettiği MJPEG akış formatı. JavaScript gerekmez.

`generate_mjpeg` bir üreteç (generator): `latest_frames[vehicle]["id"]`
değişimini izler, yeni kare geldikçe yollar.

---
---

# Görev kontrolü — Hedef İHA

## `_stop_scenario_proc()`

```python
if _scenario_proc is not None:
    try:
        os.killpg(os.getpgid(_scenario_proc.pid), signal.SIGTERM)
    except Exception:
        pass
    _scenario_proc.wait()
_scenario_proc = None
_scenario_name = None
# Emniyet: GCS yeniden başlatıldıysa elde referansı olmayan süreç kalmış olabilir
subprocess.run(['pkill', '-9', '-f', 'run_plane_scenario'], capture_output=True)
```

**İki katmanlı temizlik:**
1. Bilinen sürecin **süreç grubuna** `SIGTERM` (betiğin `_sig_handler`'ı temiz
   çıkış yapar)
2. `pkill` — GCS çöküp yeniden başlarsa referans kaybolmuş olur, bu ağ hepsini
   yakalar

> Bu `pkill` olmasa bile **çalıştırma dokümanındaki temizlik komutu** artık
> `run_plane_scenario`'yu içeriyor — canlı testte 6 saatlik bir hayalet süreç
> bulunmuştu.

---

## `start_plane_scenario(name)`

```python
if name not in _SCENARIO_NAMES:
    return {"status": "error", "message": f"Bilinmeyen senaryo: {name}"}
if _manual_active:                 # manuel kontrol açıksa kapat
    _manual_active = False
    time.sleep(0.3)
_stop_scenario_proc()              # önceki senaryo (varsa) dursun

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_scenario_proc = subprocess.Popen(
    ["python3", "-m", "control.run_plane_scenario", name],
    cwd=project_root,
    start_new_session=True,        # kendi süreç grubu → killpg ile temiz öldürme
)
```

**`start_new_session=True` neden:** Alt süreç kendi süreç grubunu alır.
`os.killpg` ile hem betiği hem onun açtığı alt süreçleri tek seferde öldürebiliriz.

**Neden alt süreç, thread değil:** Senaryo süresiz döngüde RC override'ı 10-20 Hz
besler ve blokan `time.sleep` kullanır. Aynı süreçte thread olsaydı GIL ve
bloklamalar kamera/telemetri akışını bozardı. Ayrıca ayrı süreç `pkill` ile
kesin öldürülebilir.

---

## `_manual_control_thread()` / `command_plane_manual(...)`

Uçağı FBWA'da klavye/joystick kontrolüne devralır (10 Hz RC override).
Arayüzdeki değerler `/api/command/plane/manual` ile thread'e iletilir.

FBWA seçimi bilinçli: RC girdileri **açı hedefi** olarak yorumlanır ("sağa %30
yatış"), MANUAL'deki ham servo pozisyonu değil — kullanıcı uçağı kolayca
kaybetmez.

---
---

# Görev kontrolü — Avcı drone

## `_chase_thread()` — hibrit güdüm

```python
stop_iris_telem()                      # 14541 serbest kalsın
time.sleep(0.3)
conn = df_connect_drone(port=14541)
success = df_takeoff(target_z=-5.0)    # GUIDED + NAV_TAKEOFF
if not success: return

# ---- _chase_active'i stop_event'e bağla ----
chase_stop = threading.Event()
def watch_active():
    while _chase_active:
        time.sleep(0.1)
    chase_stop.set()
threading.Thread(target=watch_active, daemon=True).start()

# ---- ALGORİTMAYI ÇAĞIR ----
if os.environ.get("AVCI_HYBRID", "on").lower() in ("off", "0"):
    _run_gps_guidance(conn, get_plane, get_iris, chase_stop)    # saf GPS
else:
    def get_plane_truth():
        t = telemetry_state["plane"]
        return {"x": t["x"], "y": t["y"], "z": t["z"]}
    _run_hybrid(conn, get_plane, get_iris, wait_new_pose,
                get_plane_truth, chase_stop)

# ---- DURDURMA → HOVER ----
df_hover(duration=3.0)
```

### `watch_active` köprüsü
`stop_chase` endpoint'i basit bir bayrağı (`_chase_active`) indirir. Güdüm
katmanı ise `threading.Event` bekler. Bu küçük thread ikisini birbirine bağlar.

### `get_plane_truth` neden ayrı callback
Hedefin **gerçek** (gürültüsüz) pozu. `visual_lead` bunu **yalnız** menzil logu
ve vuruş tespiti için kullanır — güdüm hesabına girmez. Gerçek donanımda yerini
yakınlık sensörü alır.

Güdüme giren `get_plane` ise `_apply_gps_noise`'dan geçmiş gürültülü veridir.

---

## `_visual_thread()` — izole görsel güdüm

```python
_chase_active = False       # aynı porta erişen GPS chase'i durdur
stop_iris_telem()
conn = df_connect_drone(port=14541)
success = df_takeoff(target_z=-5.0)
_run_visual_lead(conn, wait_new_pose, get_plane_truth, _visual_stop_event)
```

Supervisor **yok** — doğrudan görsel döngü. Hata ayıklarken hangi fazın sorunlu
olduğunu ayırmak için: GPS fazı devre dışı, sadece IBVS koşar.

---
---

# API uç noktaları

## Hedef İHA
| Metod | Yol | İşlev |
|-------|-----|-------|
| POST | `/api/command/plane/scenario/{name}` | `square`/`circle`/`aggressive` başlat |
| POST | `/api/command/plane/stop_scenario` | Senaryoyu durdur |
| GET | `/api/scenario_status` | Aktif senaryo adı (buton senkronu) |
| POST | `/api/command/plane/start_manual` | Klavye kontrolüne devral |
| POST | `/api/command/plane/manual` | Anlık PWM değerleri |
| POST | `/api/command/plane/stop_manual` | Manuel modu bitir |
| GET/POST | `/api/plane_throttle` | Gaz kaydırıcısı |

## Avcı drone
| Metod | Yol | İşlev |
|-------|-----|-------|
| POST | `/api/command/iris/start_chase` | **Hibrit güdüm** |
| POST | `/api/command/iris/stop_chase` | Durdur |
| GET | `/api/chase_status` | Faz, menzil, kilit sayacı, geçiş sayısı |
| POST | `/api/command/iris/start_visual` | İzole görsel güdüm |
| POST | `/api/command/iris/stop_visual` | Durdur |

## Efektler ve veri
| Metod | Yol | İşlev |
|-------|-----|-------|
| GET/POST | `/api/gps_noise` | GPS karıştırma (0.0-1.0) |
| GET/POST | `/api/video_noise` | Video parazit (0.0-1.0) |
| GET | `/api/telemetry/pnp` | PnP poz tahmini |
| GET | `/api/debug/telem` | Anlık telemetri + MAVLink istatistikleri |
| GET | `/api/video_feed/{vehicle}` | MJPEG akışı (`iris` / `plane`) |
| WS | `/ws` | Telemetri akışı |

---

## Ortam değişkenleri

| Değişken | Varsayılan | Etkisi |
|----------|-----------|--------|
| `AVCI_GZ_CAMERA` | — | `1` → gz-transport kamera thread'leri (Harmonic) |
| `AVCI_GZ_CAMERA_TOPIC` | `/iris_cam/image` | Alternatif topic |
| `AVCI_HYBRID` | `on` | `off`/`0` → hibrit yerine saf GPS fazı |
| `AVCI_DETECTOR` | `yolo` | Detection modelini yükle |
| `AVCI_POSE` | `on` | Pose modelini yükle |
| `AVCI_YOLO_MODEL` / `AVCI_POSE_MODEL` | `vision/models/*.pt` | Alternatif ağırlık |
| `AVCI_YOLO_CONF` | 0.45 | Detection güven eşiği |
| `AVCI_IBVS_*` | bkz. `guidance_core.Cfg` | Güdüm ayarları |
| `AVCI_GPS_RANGE` | 11.0 | GPS fazı menzil setpoint |
| `AVCI_HYBRID_GATE_MENZIL` | 20.0 | Faz geçiş kapısı |
