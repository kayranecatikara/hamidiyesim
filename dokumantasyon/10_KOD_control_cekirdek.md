# 10 — `control/` Çekirdek Kontrol Katmanı

> MAVLink altyapısı ve araç kontrol API'si. Fonksiyon bazında açıklama.
>
> **Kapsam:** `mav_common.py` · `drone_functions.py` · `plane_functions.py` · `plane_patterns.py`

---

# `mav_common.py`

**Yol:** `control/mav_common.py` · **377 satır**
**Rol:** Tüm araçların paylaştığı düşük seviye ArduPilot MAVLink altyapısı.

**Tasarım ilkesi:** Her fonksiyon **açık bir bağlantı nesnesi alır**, global tutmaz.
Bu sayede aynı Python sürecinde birden fazla araca bağlanılabilir
(`run_dual_demo` tam olarak bunu yapar).

---

## Sabitler

### Mod numaraları

```python
COPTER_MODE_STABILIZE = 0    PLANE_MODE_MANUAL    = 0
COPTER_MODE_ALT_HOLD  = 2    PLANE_MODE_CIRCLE    = 1
COPTER_MODE_GUIDED    = 4    PLANE_MODE_STABILIZE = 2
COPTER_MODE_LOITER    = 5    PLANE_MODE_FBWA      = 5
COPTER_MODE_RTL       = 6    PLANE_MODE_FBWB      = 6
COPTER_MODE_LAND      = 9    PLANE_MODE_CRUISE    = 7
                             PLANE_MODE_AUTO      = 10
                             PLANE_MODE_RTL       = 11
                             PLANE_MODE_LOITER    = 12
                             PLANE_MODE_TAKEOFF   = 13
                             PLANE_MODE_GUIDED    = 15
```

> ArduCopter ve ArduPlane **ayrı numara uzayları** kullanır. Aynı sayı iki araçta
> farklı mod demektir (4 = Copter GUIDED, Plane'de tanımsız). Sabitlerin ayrı
> önekle tutulmasının sebebi budur.

### Magic sayılar

| Sabit | Değer | Anlamı |
|-------|------:|--------|
| `FORCE_ARM_MAGIC` | **2989** | `MAV_CMD_COMPONENT_ARM_DISARM` param2 — prearm kontrollerini atlar |
| `FORCE_DISARM_MAGIC` | **21196** | Uçuşta bile disarm eder |
| `GCS_SOURCE_SYSTEM` | **255** | Bağlantının kaynak sysid'si |
| `DEFAULT_STREAM_RATE_HZ` | 10 | Telemetri akış frekansı |

> ⚠️ **PX4 tuzağı:** PX4'te force ARM 21196'ydı. ArduPilot'ta 21196 gönderilirse
> araç **disarm** olur. Bu, `arm_diag.py`'de bulduğum gerçek hatanın kaynağıydı.

---

## `connect_mavlink(port, source_system=255, source_component=0, protocol="udpin", ip="127.0.0.1", timeout=15, stream_rate_hz=10)`

**Dönüş:** `mavutil.mavlink_connection` nesnesi
**Hata:** heartbeat 15 sn içinde gelmezse `TimeoutError`

### Ne yapar
Verilen port üzerinde MAVLink bağlantısı kurar, **otopilot** heartbeat'ini bekler
ve telemetri akışını açar.

### Nasıl çalışır — kritik filtre

```python
while time.time() - t0 < timeout:
    msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
    if msg is None:
        continue
    # Yalnızca otopilot bileşeninden gelen heartbeat'i kabul et
    if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
        continue
    if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
        continue
    conn.target_system = msg.get_srcSystem()
    conn.target_component = msg.get_srcComponent()
    break
```

**İki aşamalı filtre neden gerekli:** Aynı porta birden fazla şey yayın yapar —
otopilot, MAVProxy, diğer GCS'ler, hatta bizim kendi keepalive'ımız. Filtre
olmasaydı `target_system` yanlış bir kaynağa kilitlenir ve **gönderilen tüm
komutlar boşa giderdi**. Belirti: komutlar sessizce hiçbir şey yapmaz, hata da
vermez.

Son adım: ArduPilot telemetriyi kendiliğinden yollamaz, `request_streams()`
çağrılır.

---

## `request_streams(conn, rate_hz=10)`

Tüm telemetri akışlarını istenen frekansta talep eder.

```python
conn.mav.request_data_stream_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz, 1,   # 1 = başlat
)
```

> **Neden zorunlu:** PX4 telemetriyi otomatik yollardı, ArduPilot **istek
> üzerine** yollar. Bu çağrı atlanırsa `LOCAL_POSITION_NED` ve `ATTITUDE` hiç
> gelmez — güdüm döngüleri veri bekleyip sonsuza kadar bloklanır.

---

## `set_message_interval(conn, message_id, hz)`

Tek bir mesajın yayın frekansını ayarlar. Akışın tamamı yerine belirli bir mesajı
hızlandırmak için.

```python
interval_us = int(1e6 / hz) if hz > 0 else -1    # -1 = mesajı durdur
conn.mav.command_long_send(..., MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                           message_id, interval_us, 0, 0, 0, 0, 0)
```

Örnek — pozisyonu 30 Hz istemek:
```python
set_message_interval(conn, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30)
```

---

## `send_gcs_heartbeat(conn)`

Tek bir GCS heartbeat paketi gönderir.

```python
conn.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0)
```

`MAV_AUTOPILOT_INVALID` bilinçli: biz bir otopilot değil, yer istasyonuyuz.

---

## `class GCSKeepalive`

Arka planda sürekli GCS heartbeat gönderen thread yöneticisi.

| Üye | İşlev |
|-----|-------|
| `__init__(conn, interval=0.1)` | 10 Hz varsayılan |
| `start()` | Daemon thread başlatır (zaten çalışıyorsa işlemsiz) |
| `stop()` | Bayrağı indirir, thread'i `join(timeout=2.0)` ile bekler |
| `_loop()` | `while self._running: send_gcs_heartbeat(); sleep(interval)` |

**Neden gerekli — iki sebep:**
1. **GCS failsafe:** ArduPilot bir süre GCS heartbeat'i almazsa failsafe
   tetikler (RTL/LAND). Uçuş ortasında bu görevi bitirir.
2. **RC override zaman aşımı:** RC override paketleri 3 sn içinde yenilenmezse
   düşer. Keepalive, modlar arası geçişteki boşlukta bağlantıyı canlı tutar.

`daemon=True` seçimi: ana program çıkarken thread'in asılı kalmaması için.

---

## `wait_ack(conn, command_id=None, timeout=5.0)`

**Dönüş:** `(command, result)` tuple veya `None`

Gönderilen bir komutun `COMMAND_ACK` yanıtını bekler.

```python
t0 = time.time()
while time.time() - t0 < timeout:
    msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
    if msg is None:
        continue
    if command_id is None or msg.command == command_id:
        return (msg.command, msg.result)
return None
```

`command_id=None` verilirse **herhangi bir** ACK kabul edilir. Belirli bir komut
verilirse diğerleri atlanır — birden fazla komut aynı anda uçuşta olabilir.

`result == 0` (`MAV_RESULT_ACCEPTED`) başarı demektir. Çağıranlar bu kontrolü
`result and result[1] == 0` kalıbıyla yapar.

---

## `is_armed(conn)`

Son heartbeat'in `base_mode` alanındaki `MAV_MODE_FLAG_SAFETY_ARMED` bitini
okur.

```python
return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
```

Heartbeat 2 sn içinde gelmezse `False` döner — "bilmiyorum" ile "armlı değil"
ayrımı yapılmaz (basitleştirme).

---

## `arm(conn, force=False, retries=1, retry_interval=2.0)`

**Dönüş:** son `wait_ack` sonucu

### Nasıl çalışır

```python
p2 = FORCE_ARM_MAGIC if force else 0        # 2989
for attempt in range(1, max(1, retries) + 1):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, p2, 0, 0, 0, 0, 0,               # param1=1 (arm), param2=magic
    )
    result = wait_ack(conn, MAV_CMD_COMPONENT_ARM_DISARM)
    if result and result[1] == MAV_RESULT_ACCEPTED:
        return result
    if attempt < retries:
        time.sleep(retry_interval)
return result
```

### Neden retry var
EKF oturana kadar prearm kontrolleri **geçici olarak** reddeder ("EKF3 waiting
for GPS config"). Kalkış fonksiyonları `retries=10, retry_interval=2.0` ile
çağırır — yani 20 saniyeye kadar sabırla dener. Tek denemede vazgeçilseydi
soğuk başlangıçta hiç ARM olunamazdı.

---

## `disarm(conn, force=False)`

`arm()`'ın aynası: `param1=0`, `param2 = 21196` (force ise).

> Force disarm **uçuşta bile** çalışır — araç düşer. Simülasyonda acil durdurma
> için kullanılır, gerçek uçuşta dikkatli olunmalıdır.

---

## `set_mode(conn, custom_mode, confirm_timeout=3.0)`

### Nasıl çalışır — iki aşama

**1. Komut gönder:**
```python
conn.mav.command_long_send(
    conn.target_system, conn.target_component,
    MAV_CMD_DO_SET_MODE, 0,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,   # param1: custom mode kullanacağız
    custom_mode,                          # param2: hedef mod
    0, 0, 0, 0, 0,
)
result = wait_ack(conn, MAV_CMD_DO_SET_MODE)
```

**2. Heartbeat'ten teyit et:**
```python
while time.time() - t0 < confirm_timeout:
    hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
    if (hb is not None
            and hb.get_srcSystem() == conn.target_system
            and hb.get_srcComponent() == conn.target_component
            and hb.custom_mode == custom_mode):
        return result        # mod gerçekten değişti
```

### Neden çift kontrol
`COMMAND_ACK` yalnızca "komutu aldım" der; **modun gerçekten değiştiğini
garanti etmez**. ArduPilot komutu kabul edip sonra prearm/koşul sebebiyle moda
geçmeyebilir. Heartbeat'teki `custom_mode` alanı tek güvenilir kanıttır.

> **ArduPilot farkı:** PX4'te mod ayarı `main_mode` + `sub_mode` ikilisiydi.
> ArduPilot'ta tek `custom_mode` alanı var — bu fonksiyon o basitleştirmeyi
> yansıtıyor.

---

## `get_local_position(conn, timeout=2.0)` / `get_attitude(conn, timeout=2.0)`

| Fonksiyon | Mesaj | Dönüş |
|-----------|-------|-------|
| `get_local_position` | `LOCAL_POSITION_NED` | `{x, y, z, vx, vy, vz}` (m, m/s, NED) |
| `get_attitude` | `ATTITUDE` | `{roll, pitch, yaw, rollspeed, pitchspeed, yawspeed}` (rad, rad/s) |

Mesaj gelmezse `None`.

> **NED hatırlatması:** `z` **aşağı pozitiftir**. 5 metre irtifa `z = -5.0`
> demektir. Kod boyunca `abs(target_z)` ve `-5.0` kalıplarını bu yüzden
> görürsünüz.

---

## `drain_messages(conn, timeout=0.5)`

```python
while True:
    msg = conn.recv_match(blocking=False)
    if msg is None:
        break
```

Kuyruktaki tüm mesajları okuyup atar.

### Neden gerekli — iki farklı sebep

**1. Bayat veri:** `recv_match` kuyruğun **başındaki** mesajı döndürür. Kuyrukta
20 eski `LOCAL_POSITION_NED` birikmişse okuduğunuz konum saniyeler öncesine
aittir. `get_drone_position()` ve `_get_current_pos()` bu yüzden önce drenaj
yapar.

**2. Soket tamponu taşması:** `send_manual_control` içindeki yorum bunu açıklar
— komut döngüleri hiç okuma yapmazsa soket tamponu dolar, telemetri bayatlayıp
kayıplı hâle gelir (SITL'de ölçülmüş).

---

## `timestamp_ms()` / `log_telemetry(conn, duration, interval)`

- `timestamp_ms()` — `int(time.time() * 1000) & 0xFFFFFFFF`. MAVLink zaman
  alanları 32-bit olduğu için maskelenir.
- `log_telemetry()` — belirli süre pozisyon yazdıran hata ayıklama yardımcısı.

---
---

# `drone_functions.py`

**Yol:** `control/drone_functions.py` · **414 satır**
**Rol:** Avcı drone'un (iris / ArduCopter) yüksek seviye kontrolü.

**Tasarım:** Modül düzeyinde tek bağlantı (`_conn`) tutar — demolar ve GCS için
kolaylık. `mav_common`'ın aksine burada durum vardır.

---

## Sabitler ve typemask

```python
DRONE_PORT = 14541
DRONE_SYS_ID = GCS_SOURCE_SYSTEM   # 255 — SYSID_MYGCS ile eşleşmeli
SETPOINT_RATE = 0.1                # saniye — setpoint gönderim aralığı (10 Hz)
DEFAULT_TAKEOFF_Z = -2.0           # NED: yukarı = negatif

# bit0-2: pozisyon, bit3-5: hız, bit6-8: ivme, bit9: force,
# bit10: yaw, bit11: yaw_rate   (set edilen bit = IGNORE)
TYPEMASK_POSITION = 0b0000_1111_1111_1000   # sadece pozisyon (yaw serbest)
TYPEMASK_POS_YAW  = 0b0000_1011_1111_1000   # pozisyon + yaw
```

### Typemask nasıl okunur
`SET_POSITION_TARGET_LOCAL_NED` mesajında **set edilen bit = o alanı YOKSAY**
demektir (ters mantık — sık yapılan hata kaynağı).

| Maske | bit10 (yaw) | Anlamı |
|-------|-------------|--------|
| `TYPEMASK_POSITION` = `...1111_1111_1000` | **1** (set) | yaw yoksayılır → heading serbest |
| `TYPEMASK_POS_YAW` = `...1011_1111_1000` | **0** (temiz) | yaw uygulanır |

Alt üç bit (`000`) pozisyonun **kullanılacağını** söyler.

> **PX4 sürümündeki hata:** Eski kodda yaw biti yanlışlıkla hep set ediliyordu —
> `yaw` parametresi veriliyor ama araç hiç dönmüyordu. Bu sürümde düzeltildi.

---

## `connect_drone(port=14541)` / `get_conn()`

```python
_conn = connect_mavlink(port, source_system=DRONE_SYS_ID)
```

`get_conn()` bağlantı yoksa `RuntimeError("Önce connect_drone() çağrılmalı")`
fırlatır — sessiz `None` hatası yerine net mesaj.

---

## `_send_position_setpoint(conn, x, y, z, yaw=None)`

Tek bir pozisyon setpoint'i gönderir. Modülün en alt seviye çıkışı.

```python
if yaw is None:
    type_mask = TYPEMASK_POSITION      # heading serbest
    yaw_val = 0.0
else:
    type_mask = TYPEMASK_POS_YAW       # heading komutlanır
    yaw_val = yaw

conn.mav.set_position_target_local_ned_send(
    timestamp_ms(),
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    type_mask,
    x, y, z,           # pozisyon (m, NED)
    0.0, 0.0, 0.0,     # vx, vy, vz — yoksayılır
    0.0, 0.0, 0.0,     # afx, afy, afz — yoksayılır
    yaw_val, 0.0,      # yaw (rad), yaw_rate — yoksayılır
)
```

> Güdüm paketi (`guidance/common.send_velocity`) **bu fonksiyonu kullanmaz** —
> aynı MAVLink mesajını farklı typemask ile (hız + yaw) gönderir. İki farklı
> setpoint sözleşmesi bilinçli olarak ayrı tutulmuştur.

---

## `arm_drone(force_sim=True)` / `disarm_drone(force=True)` / `set_guided_mode()`

İnce sarmalayıcılar:

```python
def arm_drone(force_sim=True):
    return arm(get_conn(), force=force_sim, retries=10, retry_interval=2.0)
```

`retries=10` — EKF oturana kadar 20 saniyeye kadar dener.

```python
set_offboard_mode = set_guided_mode   # geriye dönük uyumluluk takma adı
```
PX4 döneminden kalan çağrılar kırılmasın diye bırakılmış.

---

## `_wait_gps_ready(conn, timeout=60.0)`

```python
while time.time() - t0 < timeout:
    msg = conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
    if msg and msg.fix_type >= 3:     # 3 = 3D fix
        return True
print("[DRONE] GPS fix beklenirken timeout — yine de devam ediliyor")
return False
```

**Neden force ARM'dan önce:** Force ARM prearm kontrollerini **atlar** — EKF
oturmadan da arm eder. O durumda araç kalkar ama konum kestirimi saçmadır ve
uçuş kontrolsüz olur. Bu bekleme, force ARM'ın bu tehlikeli yan etkisini
dengeler.

Timeout'ta program durmaz, uyarıp devam eder (simülasyonda GPS bazen geç gelir).

---

## `takeoff_to_z(target_z=-2.0, timeout=30.0)`

**Dönüş:** `True` = kalkış başarılı

Modülün en karmaşık fonksiyonu. Beş adım + bir özel durum.

### Adım 0 — zaten havada mı?

```python
drain_messages(conn)
cur = get_local_position(conn, timeout=1.0)
already_airborne = cur is not None and cur["z"] < -1.5

if already_airborne:
    set_mode(conn, COPTER_MODE_GUIDED)
    cx, cy = cur["x"], cur["y"]
    while time.time() - t0 < timeout:
        _send_position_setpoint(conn, cx, cy, target_z)   # yatayda sabit kal
        pos = get_local_position(conn, timeout=0.2)
        if pos and abs(pos["z"] - target_z) < 0.3:
            return True
        time.sleep(SETPOINT_RATE)
    return False
```

**Neden gerekli:** ArduCopter **uçan bir aracı** `NAV_TAKEOFF` ile kaldırmaz —
komutu reddeder. Önceki bir görevden havada kalmışsa (ör. chase durdurulup
yeniden başlatıldıysa) takeoff atlanır, doğrudan hedef irtifaya setpoint
gönderilir. Eşik `-1.5 m` — yerdeki gürültüyü havadan ayırır.

### Adım 1-5 — normal kalkış

```
1. _wait_gps_ready(conn)                       # 3D fix
2. set_mode(conn, COPTER_MODE_GUIDED)          # başarısızsa → False
3. arm_drone(force_sim=True)                   # başarısızsa → False
   time.sleep(0.5)                             # ARM'ın oturması için
4. command_long_send(MAV_CMD_NAV_TAKEOFF, ..., param7=takeoff_alt)
   wait_ack(conn, MAV_CMD_NAV_TAKEOFF)
5. while: pos okuyup |z - target_z| < 0.3 olana kadar bekle
```

**Sıra neden bu:** GUIDED **ARM'dan önce** gelmelidir — ArduCopter bazı modlarda
arm olmayı reddeder. `NAV_TAKEOFF` ise ARM'dan sonra, çünkü disarmlı araç kalkış
komutunu kabul etmez.

`param7 = takeoff_alt` — `NAV_TAKEOFF` irtifayı **pozitif metre** olarak alır,
NED `z` değil. Bu yüzden `abs(target_z)`.

---

## `hold_position(x, y, z, duration=5.0, yaw=None)` / `hover(duration, target_z)`

```python
t0 = time.time()
while time.time() - t0 < duration:
    _send_position_setpoint(conn, x, y, z, yaw=yaw)
    time.sleep(SETPOINT_RATE)          # 10 Hz
```

**Neden sürekli gönderim:** GUIDED modda setpoint'in **yenilenmesi** gerekir.
Tek seferlik gönderim zaman aşımına uğrar (`GUID_TIMEOUT`) ve araç hover'a
düşer. 10 Hz güvenli bir tekrar oranıdır.

`hover()` mevcut konumu okuyup `hold_position`'a devreder; okuyamazsa
`(0, 0, target_z)` kullanır.

---

## Hareket fonksiyonları

Hepsi aynı iskelet: **mevcut konumu oku → hedefi hesapla → git**.

```python
def move_forward(distance=2.0, speed=1.0):
    cx, cy, cz = _get_current_pos(conn)
    return _move_to(cx + distance, cy, cz, speed)
```

| Fonksiyon | Yön | NED etkisi |
|-----------|-----|-----------|
| `move_forward(d)` | kuzey | `x + d` |
| `move_backward(d)` | güney | `x - d` |
| `move_right(d)` | doğu | `y + d` |
| `move_left(d)` | batı | `y - d` |
| `move_up(d)` | yukarı | `z - d` |
| `move_down(d)` | aşağı | `z + d` |

> ⚠️ **"forward" gövde yönü değil, kuzeydir.** NED dünya çerçevesinde çalışır;
> drone'un burnu nereye bakarsa baksın `move_forward` kuzeye gider.

### `_move_to(x, y, z, speed=1.0, yaw=None)`

```python
while time.time() - t0 < timeout:          # timeout = 30 s
    _send_position_setpoint(conn, x, y, z, yaw=yaw)
    pos = get_local_position(conn, timeout=0.2)
    if pos:
        dist = math.sqrt((pos["x"]-x)**2 + (pos["y"]-y)**2 + (pos["z"]-z)**2)
        if dist < 0.3:
            return True
    time.sleep(SETPOINT_RATE)
return False
```

**Kapalı döngü:** Hedefe **0.3 m** yaklaşınca biter — süreyle tahmin edilmez.

> **`speed` parametresi kullanılmıyor.** İmzada var ama gövdede yok; hız
> ArduCopter'ın `WPNAV_SPEED` parametresiyle belirlenir. İleride
> `set_message_interval` benzeri bir parametre ayarıyla bağlanabilir.

### `_get_current_pos(conn)` / `_get_current_yaw(conn)`

```python
drain_messages(conn)      # bayat kuyruk verisi yerine güncel değeri oku
pos = get_local_position(conn, timeout=1.0)
return (pos["x"], pos["y"], pos["z"]) if pos else (0.0, 0.0, DEFAULT_TAKEOFF_Z)
```

Drenaj olmadan okunan konum saniyeler eski olabilir — hareket hedefi yanlış
hesaplanır.

---

## `yaw_right(degrees=90.0, hold_time=2.0)` / `yaw_left(...)`

```python
current_yaw = _get_current_yaw(conn)
target_yaw = current_yaw + math.radians(degrees)      # left'te −
cx, cy, cz = _get_current_pos(conn)
hold_position(cx, cy, cz, duration=hold_time, yaw=target_yaw)
```

Yaw komutu **konum tutmayla birlikte** gönderilir — sadece yaw gönderilse
pozisyon serbest kalır ve drone sürüklenir.

---

## `land_drone()` / telemetri fonksiyonları

- `land_drone()` — `set_mode(conn, COPTER_MODE_LAND)`. İniş profilini
  ArduCopter yönetir.
- `get_drone_position()` / `get_drone_attitude()` — drenaj + oku.
- `print_status()` — konum ve açıları derece cinsinden yazdırır.

---
---

# `plane_functions.py`

**Yol:** `control/plane_functions.py` · **375 satır**
**Rol:** Hedef İHA'nın (Talon / ArduPlane) yüksek seviye kontrolü.

**Temel fark:** Multikopter **pozisyon** setpoint'iyle sürülür; sabit kanat
**RC override** ile — yani sanal bir kumanda kolu gibi.

---

## Sabitler

```python
PLANE_PORT = 14542
PLANE_SYS_ID = GCS_SOURCE_SYSTEM   # 255 — RC override için SYSID_MYGCS ile eşleşmeli
CONTROL_RATE = 0.1                 # 10 Hz kontrol döngüsü

THROTTLE_IDLE   = 0
THROTTLE_CRUISE = 600
THROTTLE_FULL   = 900
```

Girdi aralıkları: `pitch`/`roll`/`yaw` → `-1000..+1000`, `throttle` → `0..1000`.

---

## `connect_plane(port=14542)` / `get_conn()`

`drone_functions` ile aynı desen. Ek olarak modül `_keepalive` referansı tutar.

---

## `start_gcs_keepalive()` / `stop_gcs_keepalive()`

```python
if _keepalive is not None:
    _keepalive.stop()               # eskisini temizle
_keepalive = GCSKeepalive(conn, interval=0.1)
_keepalive.start()
```

**Neden sabit kanatta kritik:** RC override paketi 3 sn yenilenmezse düşer.
Düştüğü an uçak kontrolsüz kalır — multikopter gibi hover'a geçemez, düşer.

---

## `arm_plane(warmup_duration=3.0)`

Beş adımlı, sırası önemli:

```
1. GCS keepalive başlat (zaten yoksa)
2. Kısa warmup (3 sn) — EKF/telemetri otursun
3. _wait_gps_ready(conn) — 3D fix
4. arm(conn, force=True, retries=10, retry_interval=2.0)
5. set_mode(conn, PLANE_MODE_MANUAL)   ← ARM'dan SONRA
```

### Adım 5 neden ARM'dan sonra

```python
if result and result[1] == 0:
    # RC override'ın motoru doğrudan sürebilmesi için MANUAL moda al.
    set_mode(conn, PLANE_MODE_MANUAL)
```

MANUAL modda RC override **doğrudan servolara** işler — otopilot araya girmez.
Ancak ArduPlane MANUAL'de arm olmayı zorlaştırabilir, bu yüzden önce arm edilip
sonra moda geçilir.

> Not: `run_plane_scenario` bu fonksiyonu kullanmaz — kendi TAKEOFF → FBWA
> akışını uygular. `arm_plane` daha çok demo ve teşhis içindir.

---

## `send_manual_control(pitch=0, roll=0, throttle=0, yaw=0)`

Modülün kalbi. Tüm hareket fonksiyonları buna indirgenir.

```python
# Alım tamponunu boşalt: komut döngüleri hiç okuma yapmazsa soket tamponu
# dolar, telemetri bayatlayıp kayıplı hale gelir (SITL'de ölçüldü).
drain_messages(conn)

rc_roll     = int(1500 + (roll / 2))       # -1000..+1000 → 1000..2000
rc_pitch    = int(1500 + (pitch / 2))
rc_throttle = int(1000 + throttle)         #     0..1000  → 1000..2000
rc_yaw      = int(1500 + (yaw / 2))

conn.mav.rc_channels_override_send(
    conn.target_system, conn.target_component,
    rc_roll,      # CH1: Aileron
    rc_pitch,     # CH2: Elevator
    rc_throttle,  # CH3: Throttle
    rc_yaw,       # CH4: Rudder
    0, 0, 0, 0    # kullanılmayan kanallar (0 = override yok)
)
```

### Ölçek dönüşümü
- Kontrol yüzeyleri: `-1000..+1000` **/2** → `±500` → `1500±500` = `1000..2000`
- Throttle: `0..1000` **+1000** → `1000..2000` (tek yönlü, orta nokta yok)

### İki önemli not

**1. `drain_messages` neden burada:** Bu fonksiyon 10 Hz çağrılır ve hiç okuma
yapmaz. Soket alım tamponu dolarsa işletim sistemi paket düşürmeye başlar —
telemetri kayıplı hâle gelir. Yorumda "SITL'de ölçüldü" denmesi bunun tahmin
değil gözlem olduğunu gösteriyor.

**2. Pitch işareti düzeltildi:** Kod yorumu şunu söylüyor —
> *"RC2 YÜKSEK PWM = burun yukarı (canlı SITL'de doğrulandı; eski yorum tersti)"*

Yani `pitch > 0` → burun yukarı. Önceki yorum tersini iddia ediyordu.

**3. `0` yazılan kanallar:** MAVLink'te RC override'da `0` = "bu kanalı override
etme". `65535` ise "override'ı serbest bırak" demektir. Burada `0` kullanılıyor,
yani CH5-8 hiç dokunulmadan bırakılıyor.

---

## Eksen yardımcıları

Hepsi aynı kalıp — belirli süre boyunca sabit komut:

```python
def set_throttle(throttle, duration=1.0):
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(throttle=throttle)
        time.sleep(CONTROL_RATE)
```

| Fonksiyon | Sürdüğü eksen |
|-----------|---------------|
| `set_throttle(throttle, duration)` | Gaz |
| `set_heading(yaw, throttle, duration)` | Rudder + gaz |
| `set_pitch(pitch, throttle, duration)` | Elevator + gaz |
| `set_roll(roll, throttle, duration)` | Aileron + gaz |

---

## Manevra fonksiyonları

| Fonksiyon | Ne yapar |
|-----------|----------|
| `fly_forward(throttle, duration)` | Düz uçuş (tüm yüzeyler nötr) |
| `turn_left(intensity, throttle, duration)` | Sola yatış |
| `turn_right(intensity, throttle, duration)` | Sağa yatış |
| `climb(pitch_intensity, throttle, duration)` | Burun yukarı + yüksek gaz |
| `descend(pitch_intensity, throttle, duration)` | Burun aşağı + düşük gaz |
| `loiter(duration)` | Cruise throttle ile düz uçuş |

### `loiter()` neden gerçek LOITER değil

Docstring açıklıyor: gerçek LOITER modu için
`mav_common.set_mode(conn, PLANE_MODE_LOITER)` kullanılabilir; o modda otopilot
kendi daire rotasını uçurur ve **RC override'ı yok sayar**. Buradaki `loiter()`
ise RC override kontrolünü elde tutar.

> **Sabit kanat kuralı:** "Dur ve bekle" diye bir şey yoktur. Hız düşerse stall
> olur. Bu yüzden her fonksiyon en azından cruise throttle sürdürür.

---
---

# `plane_patterns.py`

**Yol:** `control/plane_patterns.py` · **317 satır**
**Rol:** `plane_functions` üzerine kurulu, senaryolaştırılmış manevra desenleri.

**Ön koşul:** `connect_plane()` ve `start_gcs_keepalive()` önceden çağrılmış olmalı.

---

## `takeoff_then_stabilize(throttle, climb_duration, stabilize_duration)`

Otonom kalkış — iki mod arasında geçiş:

```
1. set_mode(TAKEOFF=13)   → ArduPlane'in kendi kalkış mantığı:
                             motoru açar, uçağı kaldırır, TKOFF_ALT'a tırmanır
2. climb_duration bekle
3. set_mode(FBWA=5)       → RC override ile desen uygulanabilir hâle gel
4. stabilize_duration boyunca düz uçuş
```

**Neden TAKEOFF modu, elle throttle değil:** Sabit kanat kalkışı hassastır —
yeterli hız toplanmadan burun kaldırılırsa stall olur. ArduPlane'in TAKEOFF
modu bunu doğru yapar (`TKOFF_ALT`, `TKOFF_LVL_ALT` parametreleriyle).

**Neden FBWA'ya geçiliyor:** FBWA (Fly By Wire A) modunda RC girdileri **açı
hedefi** olarak yorumlanır — "sağa %30 yatış" gibi. MANUAL'de ise doğrudan
servo pozisyonudur ve uçak kolayca kontrolden çıkar.

---

## Desen fonksiyonları

| Fonksiyon | Parametreler | Nasıl çalışır |
|-----------|--------------|---------------|
| `draw_square(side_duration, turn_duration, throttle, turn_intensity)` | Kenar süresi, dönüş süresi | 4 × (düz uçuş + dönüş) |
| `draw_rectangle(long_side, short_side, turn_duration, throttle, turn_intensity)` | İki farklı kenar | 2 × (uzun + dönüş + kısa + dönüş) |
| `circle(duration, turn_intensity, throttle)` | Süre, yatış | Sürekli sabit yatış |
| `zigzag(segments, segment_duration, turn_duration, throttle, turn_intensity)` | Segment sayısı | Sırayla sola-sağa |

### Kare neden süre tabanlı
Docstring açıklıyor: *"Fixed-wing GPS'siz ortamda kesin mesafe kontrolü zor
olduğundan"* kenar uzunluğu süreyle belirlenir.

> ⚠️ **Bu yaklaşımın sınırı:** Rüzgâr veya hız değişimi kenarları eşitsiz yapar,
> dönüşler tam 90° olmaz. `run_plane_scenario.py` bu sorunu **pusula tabanlı
> dönüşle** çözer (gerçek heading değişimini bekler) — canlı görevde kullanılan
> odur. Buradaki desenler daha çok demo içindir.

---

## Agresif manevralar

| Fonksiyon | Profil |
|-----------|--------|
| `aggressive_maneuver_1()` | Hızlı tırmanış → dik dalış → toparlanma |
| `aggressive_maneuver_2()` | Keskin S-dönüşü (sol-sağ ardışık) |
| `aggressive_maneuver_3()` | Spiral tırmanış (sürekli yatış + burun yukarı) |

Bunlar **avcı drone'un takip yeteneğini test etmek** için — hedef öngörülemez
hareket ederse güdüm ne yapıyor?

---

## Hazır demo dizileri

| Fonksiyon | İçerik |
|-----------|--------|
| `demo_basic()` | takeoff → kare → loiter |
| `demo_aggressive()` | takeoff → üç agresif manevra |
| `demo_mixed()` | takeoff → zigzag → çember → kare → agresif |
