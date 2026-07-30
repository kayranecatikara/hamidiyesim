# 11 — Uçuş Senaryoları, Teşhis ve Demolar

> **Kapsam:** `run_plane_scenario.py` · `arm_diag.py` · `control/demos/`

---
---

# `run_plane_scenario.py`

**Yol:** `control/run_plane_scenario.py` · **317 satır**
**Rol:** Hedef İHA'nın canlı görevdeki uçuş senaryoları.
**Önem:** `gcs_server`'ın **alt-süreç olarak başlattığı** tek betik budur.

```bash
python3 -m control.run_plane_scenario square       # kare
python3 -m control.run_plane_scenario circle       # daire
python3 -m control.run_plane_scenario aggressive   # rastgele agresif
```

---

## Sabitler ve global durum

```python
AIRBORNE_ALT_M = 15.0   # Bu irtifanın üstünde armlıysak kalkış ATLANIR
CONTROL_RATE = 0.05     # 20 Hz komut döngüsü

_abort = False          # SIGINT/SIGTERM bayrağı
_att = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "ok": False}   # _pump doldurur
_pos = {"z": 0.0}
```

`_att` ve `_pos` modül düzeyinde tutulur çünkü `_pump()` her yerden çağrılır ve
en güncel telemetriyi buraya yazar.

---

## `_sig_handler(_sig, _frame)`

```python
global _abort
_abort = True
```

`SIGINT` ve `SIGTERM` yakalanır. **Kritik:** `gcs_server` senaryoyu durdururken
önce `terminate()` (SIGTERM) yollar — bu handler sayesinde döngüler temiz çıkar
ve son komut (nötr yüzey + cruise gaz) gönderilir. Handler olmasaydı süreç
komut ortasında ölür, uçak son yatış komutuyla asılı kalırdı.

---

## `_pump(conn)`

```python
while True:
    msg = conn.recv_match(blocking=False)
    if msg is None:
        return
    t = msg.get_type()
    if t == "ATTITUDE":
        _att.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, ok=True)
    elif t == "LOCAL_POSITION_NED":
        _pos["z"] = msg.z
```

### Neden var — docstring'den

> *"plane_functions.send_manual_control her çağrıda drain_messages ile HER ŞEYİ
> çöpe atıyordu — heading tabanlı dönüş için attitude'u burada yakalıyoruz.
> Tamponu boşaltmak ayrıca telemetrinin bayatlamasını da önler."*

`drain_messages` mesajları **okuyup atar**. `_pump` ise **okuyup saklar**. İkisi
de tamponu boşaltır ama `_pump` veriyi kaybetmez. Pusula tabanlı dönüş bu veriye
muhtaçtır.

`_att["ok"]` bayrağı: hiç attitude gelmediyse `turn_by` bunu anlar ve kısa süre
bekler.

---

## `_rc(conn, roll=0, pitch=0, throttle=0, yaw=0)`

```python
conn.mav.rc_channels_override_send(
    conn.target_system, conn.target_component,
    int(1500 + roll / 2),       # CH1: Aileron
    int(1500 + pitch / 2),      # CH2: Elevator (YÜKSEK PWM = burun yukarı,
                                #      canlı SITL'de doğrulandı)
    int(1000 + throttle),       # CH3: Throttle
    int(1500 + yaw / 2),        # CH4: Rudder
    0, 0, 0, 0,
)
```

`plane_functions.send_manual_control` ile **aynı eşleme**, ama `drain_messages`
çağırmaz (o işi `_pump` yapar). Bu ayrım, betiğin attitude okuyabilmesinin tek
sebebi.

---

## `gcs_throttle()`

```python
_thr_cache = {"val": THROTTLE_CRUISE, "t": 0.0}

def gcs_throttle():
    now = time.time()
    if now - _thr_cache["t"] > 0.5:              # 0.5 sn önbellek
        _thr_cache["t"] = now
        try:
            req = urllib.request.urlopen(
                "http://127.0.0.1:8000/api/plane_throttle", timeout=0.2)
            _thr_cache["val"] = json.loads(req.read().decode()).get(
                "throttle", THROTTLE_CRUISE)
        except Exception:
            pass                                  # GCS yoksa son değerde kal
    return _thr_cache["val"]
```

**Neden önbellekli:** Komut döngüsü 20 Hz. Her karede HTTP isteği atmak saniyede
20 istek demektir — hem GCS'i yorar hem `timeout=0.2` gecikmesi kontrol
döngüsünü bozar. 0.5 sn önbellekle saniyede 2 istek yapılır.

**Neden sessiz `except`:** GCS kapalıyken senaryo **durmamalı** — son bilinen
değerle uçmaya devam eder. Betik GCS'e bağımlı değil, sadece ondan faydalanır.

---

## `hold(conn, duration, roll=0, pitch=0, throttle=None, yaw=0)`

```python
t0 = time.time()
while not _abort and time.time() - t0 < duration:
    _pump(conn)
    thr = gcs_throttle() if throttle is None else throttle
    _rc(conn, roll=roll, pitch=pitch, throttle=thr, yaw=yaw)
    time.sleep(CONTROL_RATE)
```

Tüm senaryoların yapı taşı. `throttle=None` → GCS kaydırıcısı; sayı verilirse o
kullanılır (agresif manevralar kendi gazını belirler).

`not _abort` kontrolü döngü koşulunda — durdurma sinyali anında etkili olur.

---

## `_angdiff(a, b)`

```python
d = a - b
while d > math.pi:  d -= 2 * math.pi
while d < -math.pi: d += 2 * math.pi
return d
```

Açı farkını `[-π, π]`'ye sarar. 350° → 10° farkı `-340°` yerine `+20°` olur.

---

## `turn_by(conn, deg, bank=650, timeout=20.0)`

**Betiğin en önemli fonksiyonu.**

```python
_pump(conn)
if not _att["ok"]:
    hold(conn, 1.0)          # attitude henüz gelmediyse kısa bekle
    _pump(conn)

target = _att["yaw"] + math.radians(deg)
roll_cmd = bank if deg > 0 else -bank

t0 = time.time()
while not _abort and time.time() - t0 < timeout:
    _pump(conn)
    if _att["ok"] and abs(_angdiff(target, _att["yaw"])) < math.radians(10):
        break                                    # 10° tolerans
    _rc(conn, roll=roll_cmd, pitch=180, throttle=gcs_throttle())
    time.sleep(CONTROL_RATE)
```

### Üç tasarım kararı

**1. Neden roll, rudder değil?**
Silinen eski `run_plane_square.py` zaman bazlı **rudder** dönüşü kullanıyordu.
FBWA modunda rudder tek başına dönüş üretmez — uçak yalpalar ama heading
değişmez, kare bozuk çıkardı. Sabit kanat **yatarak** döner.

**2. Neden `pitch=180` (hafif up-elevator)?**
Docstring: *"Dönüşte hafif up-elevator irtifa kaybını azaltır."* Uçak yattığında
kaldırma kuvvetinin dikey bileşeni azalır → irtifa kaybeder. Hafif burun yukarı
bunu telafi eder.

**3. Neden 10° tolerans?**
Docstring: *"FBWA kanatları düzeltirken kalan momentum farkı kapatır."* Tam 90°
beklenirse uçak hedefi aşar (overshoot); 80°'de bırakılınca kanat düzeltme
sırasındaki momentum kalan 10°'yi tamamlar.

`timeout=20.0`: dönüş bir şekilde tamamlanamazsa (stall, aşırı rüzgâr) betik
sonsuza kadar takılmaz.

---

## `_read_vehicle_state(conn, wait=1.5)`

**Dönüş:** `(armed: bool, irtifa_m: float)`

```python
armed = False
t0 = time.time()
while time.time() - t0 < wait:
    msg = conn.recv_match(type=["HEARTBEAT", "LOCAL_POSITION_NED", "ATTITUDE"],
                          blocking=True, timeout=0.3)
    if msg is None: continue
    t = msg.get_type()
    if t == "HEARTBEAT" and msg.get_srcSystem() == conn.target_system:
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    elif t == "LOCAL_POSITION_NED":
        _pos["z"] = msg.z
    elif t == "ATTITUDE":
        _att.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, ok=True)
return armed, -_pos["z"]        # NED z negatif → irtifa pozitif
```

### Neden var — gerçek bir kaza raporu

Docstring aynen şunu anlatıyor:

> *"Senaryo geçişinde kritik: önceki senaryo öldürülüp yenisi başlarken araç
> HAVADA. Eski akış havadaki uçağa yerden kalkış prosedürü uyguluyordu
> (warmup + GPS bekleme sırasında RC failsafe → arm_plane'in MANUAL moda alması
> → gaz trim'e düşüp dalış → havada TAKEOFF) ve araç yere çakılıyordu."*

**Hata zinciri adım adım:**
1. Kullanıcı "Kare"den "Daire"ye geçer → GCS eski süreci öldürür, yenisini başlatır
2. Yeni süreç `arm_plane(warmup_duration=3.0)` çağırır
3. 3 saniyelik warmup + GPS bekleme boyunca **RC override gönderilmez**
4. 3 sn sonra RC override **düşer** → failsafe
5. `arm_plane` MANUAL moda alır → gaz trim değerine düşer
6. Uçak dalışa geçer
7. Havadaki uçağa TAKEOFF modu uygulanır → çakılma

Bu fonksiyon zinciri **1. adımda** kırar.

---

## `takeoff(conn, climb_time=8.0)`

```python
set_mode(conn, PLANE_MODE_TAKEOFF)          # 13
t0 = time.time()
while not _abort and time.time() - t0 < climb_time:
    _pump(conn)                              # telemetri akmaya devam etsin
    time.sleep(0.2)
set_mode(conn, PLANE_MODE_FBWA)             # 5
hold(conn, 2.0)                              # kısa düz uçuşla stabilize
```

TAKEOFF modunda **RC override gönderilmez** — ArduPlane kendi kalkış profilini
uygular (`TKOFF_ALT`'a kadar). Yalnız `_pump` çağrılır ki telemetri bayatlamasın.

---

## Senaryolar

### `scenario_square(conn)`

```python
side = 5.0
while not _abort:
    hold(conn, side)              # 5 sn düz uçuş
    turn_by(conn, 90)             # pusula tabanlı 90° dönüş
```

Süresiz döner — GCS süreci öldürene kadar.

### `scenario_circle(conn)`

```python
# roll=500 → ~22° yatış: ~18 m/s'de ~80m yarıçaplı daire
while not _abort:
    hold(conn, 0.5, roll=500, pitch=150)
```

Sürekli sabit yatış. `pitch=150` irtifa kaybını telafi eder. Yorumdaki yarıçap
hesabı: `r = v² / (g·tan φ)` → `18² / (9.81 · tan 22°) ≈ 80 m`.

### `scenario_aggressive(conn)`

```python
maneuvers = ["climb", "dive", "bank_l", "bank_r", "s_turn", "spiral"]
while not _abort:
    m = random.choice(maneuvers)
    ...
    hold(conn, random.uniform(1.0, 2.0))    # her manevradan sonra toparlanma
```

| Manevra | Komut |
|---------|-------|
| `climb` | `pitch=500..800`, `throttle=FULL`, 1.5-3.0 sn |
| `dive` | `pitch=-350..-600`, `throttle=200`, 1.0-2.0 sn |
| `bank_l/r` | `roll=±600..900`, `pitch=200`, `throttle=FULL` |
| `s_turn` | `roll=-750` 1.5 sn → `roll=+750` 1.5 sn |
| `spiral` | `roll=450`, `pitch=450`, `throttle=FULL`, 3-5 sn |

#### İrtifa emniyeti

```python
elif m == "dive":
    if -_pos["z"] > 40.0:
        hold(conn, ..., pitch=-random.randint(350, 600), throttle=200)
    else:
        print("[SCN] İrtifa düşük — dalış yerine tırmanış")
        hold(conn, 2.0, pitch=500, throttle=THROTTLE_FULL)
```

40 m altındaysa dalış **tırmanışa çevrilir**. Rastgele manevra üreteci
tehlikeli bir dizi (dalış + dalış) seçebilir; bu kontrol yere çakılmayı önler.

---

## `main()`

### Üç başlangıç yolu

```python
armed, alt = _read_vehicle_state(conn)

if armed and alt > AIRBORNE_ALT_M:
    # ── HAVADA DEVRALMA ──
    _rc(conn, throttle=gcs_throttle())            # override akışı HEMEN başlasın
    set_mode(conn, PLANE_MODE_FBWA, confirm_timeout=0)
    hold(conn, 1.0)
elif armed:
    # ── ARMLI AMA YERDE ──
    takeoff(conn)
else:
    # ── SIFIRDAN ──
    result = arm_plane(warmup_duration=3.0)
    if result is None or result[1] != 0:
        print("[SCN] ARM başarısız!"); return
    takeoff(conn)

SCENARIOS[name](conn)
```

#### Havada devralma yolundaki iki ince ayrıntı

**1. `_rc(...)` mod değişiminden ÖNCE:**
Yorum: *"önceki RC override 3 sn içinde düşmeden FBWA + desen devralır."*
Önceki süreç öldüğü an override akışı kesilir; 3 sn sonra failsafe tetiklenir.
Yeni süreç **hemen** bir override paketi göndererek sayacı sıfırlar.

**2. `confirm_timeout=0`:**
`set_mode` normalde heartbeat'ten teyit bekler (3 sn'ye kadar). O bekleme
sırasında override gönderilmez → failsafe riski. Havada teyidi beklemeye
zaman yok, mod komutu gönderilip devam edilir.

### Temiz kapanış

```python
SCENARIOS[name](conn)
_rc(conn, throttle=THROTTLE_CRUISE)     # nötr yüzey + cruise gaz
stop_gcs_keepalive()
```

Yorum: *"Durduruldu → nötr yüzey + cruise gazla bırak (manuel mod hemen
devralır)."* Uçak son yatış komutuyla asılı kalmaz.

---
---

# `arm_diag.py`

**Yol:** `control/arm_diag.py` · **329 satır**
**Rol:** Bir araç ARM olmuyorsa sebebini bulan teşhis aracı.

> Bu dosya temizlikte **PX4'ten ArduPilot'a çevrildi**. Ayrıntı:
> [90_TEMIZLIK_KAYDI.md](90_TEMIZLIK_KAYDI.md)

```bash
python3 -m control.arm_diag                   # Talon (sysid 2, port 14542)
python3 -m control.arm_diag --iris            # iris  (sysid 5, port 14541)
python3 -m control.arm_diag --port 14550 --sysid 2
python3 -m control.arm_diag --gevset          # arming kontrollerini gevşet
```

---

## Sabitler

```python
MY_SYS = 251                    # GCS kaynak sysid (SYSID_MYGCS=255 ile çakışmasın)
FORCE_ARM_MAGIC = 2989          # ArduPilot force ARM (force DISARM = 21196)
PLANE_MODE_MANUAL = 0           # ArduPlane MANUAL (PX4'te 1'di)
COPTER_MODE_STABILIZE = 0

DEFAULT_PLANE = {"port": 14542, "sysid": 2, "ad": "Talon (ArduPlane)"}
DEFAULT_IRIS  = {"port": 14541, "sysid": 5, "ad": "iris (ArduCopter)"}
```

**`MY_SYS = 251` neden 255 değil:** Teşhis aracı çalışırken gerçek GCS de
çalışıyor olabilir. Aynı sysid'den iki kaynak paket yollarsa MAVLink yönlendirme
karışır.

---

## Tablo sabitleri

### `OKUNACAK_PARAMLAR`

ARM'ı etkileyen 12 ArduPilot parametresi:

| Parametre | Ne anlatır |
|-----------|-----------|
| `ARMING_CHECK` | Hangi ön kontroller açık (bit maskesi) |
| `ARMING_REQUIRE` | Throttle arming zorunluluğu (plane) |
| `ARMING_RUDDER` | Rudder ile arm izni |
| `BRD_SAFETY_DEFLT` | Safety switch varsayılanı |
| `AHRS_EKF_TYPE` / `EK3_ENABLE` | Aktif EKF (3 = EKF3) |
| `GPS_TYPE` / `SIM_GPS_DISABLE` | GPS yapılandırması |
| `RC_PROTOCOLS` | RC girdisi |
| `SYSID_MYGCS` | Komut kabul edilen GCS sysid'si |
| `FS_GCS_ENABL` | GCS failsafe |
| `BATT_MONITOR` | Batarya izleme |

### `ARMING_CHECK_BITLERI`

20 bitlik maskeyi okunur isimlere çevirir:

```python
ARMING_CHECK_BITLERI = {
    1 << 0: "Tümü (ALL)",      1 << 1: "Barometre",
    1 << 2: "Pusula",           1 << 3: "GPS kilidi",
    1 << 4: "INS (gyro/accel)", 1 << 5: "Parametreler",
    1 << 6: "RC kanalları",     1 << 7: "Kart voltajı",
    1 << 8: "Batarya seviyesi", 1 << 10: "Log kaydı",
    1 << 11: "Safety switch",   1 << 12: "GPS yapılandırması",
    1 << 13: "Sistem",          1 << 14: "Görev",
    1 << 15: "Rangefinder",     1 << 16: "Kamera",
    1 << 17: "Yardımcı yetki",  1 << 18: "Görüş konumu",
    1 << 19: "FFT",
}
```

### `SENSOR_BITLERI`

`MAV_SYS_STATUS_SENSOR` maskeleri (3D Gyro, Baro, Pitot, GPS, ESC, RC alıcısı,
Ön-arm kontrolü `0x02000000` dâhil).

### `GEVSETME_PARAMLARI`

```python
GEVSETME_PARAMLARI = {
    "ARMING_CHECK": 0,        # tüm ön kontroller kapalı
    "ARMING_REQUIRE": 0,      # throttle arming zorunlu değil
    "FS_GCS_ENABL": 0,        # GCS failsafe kapalı
    "BRD_SAFETY_DEFLT": 0,    # safety switch devre dışı
}
```

> ⚠️ Yalnız `--gevset` ile yazılır. Gerçek uçuşta bunları kapatmak emniyet
> mekanizmalarını devre dışı bırakır.

---

## Fonksiyonlar

### `baglan(port, sysid, ad)`

```python
while time.time() < deadline:                    # 15 sn
    msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg and msg.get_srcSystem() == sysid:
        conn.target_system = sysid
        conn.target_component = msg.get_srcComponent()
        return conn
raise TimeoutError(f"{ad} bulunamadı (sysid={sysid}, port={port}). "
                   f"SITL çalışıyor mu? Port haritası: scripts/start_ardupilot_sitl.sh")
```

Hata mesajı **yol gösterir** — hangi dosyaya bakılacağını söyler.

### `param_oku(conn, ad, timeout=3.0)`

`PARAM_REQUEST_READ` gönderip `PARAM_VALUE` bekler. Parametre yoksa `None`
(farklı araç tiplerinde farklı parametreler var).

### `param_yaz(conn, ad, deger)`

```python
conn.mav.param_set_send(..., bname, float(deger), MAV_PARAM_TYPE_INT32)
while time.time() - t < 2.0:
    m = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
    if m and m.param_id.strip('\x00') == ad:
        uygulandi = int(m.param_value)
        isaret = "✓" if uygulandi == int(deger) else "✗"
        print(f"  {isaret} {ad:20s} = {uygulandi}  (hedef={deger})")
        return uygulandi
print(f"  ✗ {ad:20s} → ACK yok (parametre yok olabilir)")
```

**Yazar ve geri okuyarak doğrular.** ArduPilot bir parametreyi sessizce
reddedebilir (aralık dışı, salt-okunur, o araçta yok). Geri okuma tek güvenilir
teyittir.

### `parametreleri_raporla(conn)`

Parametreleri okur, `ARMING_CHECK`'i bit bit çözer:

```python
if arming_check == 0:
    print("  (hiçbiri — tüm ön kontroller KAPALI)")
elif arming_check & 1:
    print("  TÜMÜ açık (bit 0 = ALL)")
else:
    for mask, etiket in ARMING_CHECK_BITLERI.items():
        if arming_check & mask:
            print(f"  • {etiket}")
```

Bit 0 özel: set ise **tüm** kontroller açıktır, diğer bitlere bakılmaz.

### `sys_status_raporla(conn)`

```python
sorunlu = mevcut & etkin & ~saglik
```

**Üçlü maske mantığı:** Bir sensör ancak **mevcut** (present) ve **etkin**
(enabled) ise sağlığı anlamlıdır. Takılı olmayan bir sensörün "sağlıksız"
görünmesi normaldir — bu maske onları eler.

### `arm_dene(conn, plane)`

```python
mod = PLANE_MODE_MANUAL if plane else COPTER_MODE_STABILIZE
conn.mav.set_mode_send(conn.target_system,
                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mod)
time.sleep(1.0)

conn.mav.command_long_send(..., MAV_CMD_COMPONENT_ARM_DISARM, 0,
                           1.0, float(FORCE_ARM_MAGIC), 0, 0, 0, 0, 0)

deadline = time.time() + 5
red_sebepleri = []
while time.time() < deadline:
    gcs_heartbeat(conn)                       # failsafe tetiklenmesin
    msg = conn.recv_match(type=['STATUSTEXT', 'COMMAND_ACK'], blocking=True, timeout=0.3)
    ...
    if "PreArm" in metin or "Arm" in metin:
        red_sebepleri.append(metin)
```

**Bu fonksiyonun değeri:** ArduPilot red sebebini `COMMAND_ACK`'te **söylemez** —
sadece `result != 0` döner. Gerçek sebep `STATUSTEXT` içinde `PreArm: ...`
metni olarak gelir. Bu döngü 5 saniye dinleyip hepsini toplar.

### `main()`

```python
varsayilan = DEFAULT_IRIS if args.iris else DEFAULT_PLANE
port = args.port if args.port is not None else varsayilan["port"]
sysid = args.sysid if args.sysid is not None else varsayilan["sysid"]
plane = not args.iris

conn = baglan(port, sysid, varsayilan["ad"])
for _ in range(15): gcs_heartbeat(conn); time.sleep(0.2)   # 3 sn
parametreleri_raporla(conn)
sys_status_raporla(conn)
if args.gevset: kontrolleri_gevset(conn)
arm_dene(conn, plane)
```

**Sıra mantıklı:** Önce heartbeat (failsafe sakinleşsin), sonra **salt-okunur**
teşhis, en son (istenirse) değiştirme ve deneme.

---
---

# `control/demos/` — Bağımsız uçuş demoları

**Toplam:** 580 satır · Görev akışının parçası **değil**.

> Bu 6 betik temizlikte `control/` kökünden buraya taşındı ve içlerindeki
> `/home/kayra/...` sabit yolu göreli yolla değiştirildi.

Ortak yapı — hepsinde aynı iskelet:

```python
import os
# Depo kökünü bu dosyanın konumundan türet (control/demos/ -> depo kökü)
sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

def shutdown(sig, frame):
    print("\n[DEMO] Durduruldu (Ctrl+C)")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)
```

**Üç kez `dirname`:** `control/demos/dosya.py` → `control/demos/` →
`control/` → depo kökü.

---

## `run_drone_takeoff.py` (71 satır)

En basit demo — giriş noktası.

```python
connect_drone()                                   # port 14541
success = takeoff_to_z(target_z=-2.0)
if not success:
    print("[DEMO] Kalkış başarısız!"); return
hover(duration=10.0, target_z=-2.0)
print_status()
try:
    while True:
        hover(duration=5.0, target_z=-2.0)        # setpoint akışı sürsün
except KeyboardInterrupt:
    print("\n[DEMO] Durduruldu")
```

**Sonsuz hover döngüsü neden:** GUIDED modda setpoint akışı kesilirse araç
zaman aşımına uğrar. Demo bitince bile setpoint göndermeye devam eder.

---

## `run_drone_hover.py` (102 satır)

`drone_functions`'ın tüm hareket API'sini gösterir: kalkış → ileri/geri/sağ/sol
hareket → yaw dönüşleri → hover.

---

## `run_drone_square.py` (109 satır)

**Asıl amacı veri toplamak.** Docstring:

> *"Hard-negative kare toplarken (vision/capture_negatives.py) drone'un kendi
> kendine düzgün bir desende uçması. Her kenarda burun gidiş yönüne döner; köşe
> dönüşlerindeki yatış/dönüş sırasında pervaneler kameraya girer — asıl istenen
> de bu."*

```bash
python3 -m control.demos.run_drone_square                # 25 m irtifa, 40 m kenar
python3 -m control.demos.run_drone_square --alt 30 --side 60
```

Ctrl+C → olduğu yerde LAND.

**Neden pervaneler kadraja girsin istiyoruz:** YOLO modeli kendi pervanemizi
"talon" sanıyordu. Bu uçuşta toplanan etiketsiz kareler modele "bu hedef değil"
diye öğretilir.

---

## `run_plane_arm.py` (74 satır)

Talon keepalive + ARM + düz uçuş. ARM sorunlarını izole etmek için — sorun
varsa `arm_diag.py`'ye geçilir.

---

## `run_plane_aggressive.py` (81 satır)

`plane_patterns`'in üç agresif manevrasını sırayla uçurur:
`takeoff_then_stabilize` → `aggressive_maneuver_1/2/3` → `loiter`.

---

## `run_dual_demo.py` (125 satır)

**İki aracı paralel thread'lerde eş zamanlı uçurur.**

```python
def drone_task():
    from control.drone_functions import connect_drone, takeoff_to_z, hover
    connect_drone()      # 14541
    ...

def plane_task():
    from control.plane_functions import connect_plane, arm_plane
    from control.plane_patterns import ...
    connect_plane()      # 14542
    ...

threading.Thread(target=drone_task).start()
threading.Thread(target=plane_task).start()
```

**Mimari değeri:** `mav_common`'ın "her fonksiyon `conn` alır, global tutmaz"
tasarımının **neden** böyle olduğunu gösteren örnek. İki araç aynı süreçte,
farklı portlarda, birbirini etkilemeden çalışır.

> Not: `drone_functions` ve `plane_functions` modül düzeyinde `_conn` tutar —
> bu yüzden her araç için **ayrı modül** kullanılır. Aynı modülle iki drone'a
> bağlanılamaz; o durumda `mav_common` doğrudan kullanılmalıdır.

---

## `control/demos/__init__.py` (18 satır)

Paket dokümantasyonu — hangi demonun ne yaptığı ve çalıştırma komutları.
Temizlikte eklendi.

```bash
python3 -m control.demos.run_drone_takeoff      # kalkış + hover
python3 -m control.demos.run_drone_hover        # kalkış + hareket + yaw
python3 -m control.demos.run_drone_square       # kare (negatif veri uçuşu)
python3 -m control.demos.run_plane_arm          # keepalive + ARM
python3 -m control.demos.run_plane_aggressive   # agresif manevralar
python3 -m control.demos.run_dual_demo          # iki araç eş zamanlı
```
