# 17 — `scripts/`, `tests/`, `tools/`, `logs/` ve Kök Dosyalar

---

## 1. `scripts/` — Başlatma ve kurulum betikleri

### `start_harmonic.sh` — Ana başlatıcı

**Satır:** 111 · **Rol:** Tüm simülasyon yığınını **doğru sırada, tek seferde**
ayağa kaldırır.

```bash
bash scripts/start_harmonic.sh              # GUI (NVIDIA render, önerilen)
GZ_HEADLESS=1 bash scripts/start_harmonic.sh # görüntüsüz
bash scripts/start_harmonic.sh stop         # hepsini durdur
```

**Akış:**

```
0. stop_all()  → eski süreçleri temizle
       pkill kalıpları: 'model JSON', 'model plane', '[s]im_vehicle',
                        '[m]avproxy', '[g]z sim', '[r]uby.*gz'
1. Ortam        → ROS 2 source + GZ_SIM_SYSTEM_PLUGIN_PATH + GZ_SIM_RESOURCE_PATH
2. Gazebo       → avci_harmonic.sdf (GUI veya headless)
                  ⏳ FDM portu 9002 açılana kadar bekle (max 30 sn) + 3 sn
3. ArduCopter   → sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON
                  -I0 --sysid 5, 3 parametre dosyası, out 14541/14550/14551
4. ArduPlane    → sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012
                  -I1 --sysid 2, avci_plane.parm, out 14542/14550/14551
5. ⏳ 25 sn bekle → SITL'lerin açılması
6. Kullanıcıya  → "şimdi ayrı terminallerde gcs_server ve Mission Planner"
```

**Loglar:** `logs/gz_harmonic.log`, `logs/copter_harmonic.log`,
`logs/plane_harmonic.log`

#### Betikteki en önemli açıklama — üç parametre dosyası neden zorunlu

> Güncel ArduPilot'ta `sim_vehicle.py` artık SITL'e `--defaults` göndermiyor;
> SITL frame varsayılanlarını gömülü `vehicleinfo.json`'dan **`--model`
> anahtarına göre** çözüyor. Burada `--model JSON` verildiği için arama anahtarı
> `"JSON"` oluyor ve o anahtarın frame varsayılanı **yok** (`-f gazebo-iris`
> yalnızca `sim_vehicle.py` tarafını ilgilendiriyor).
>
> Bu iki dosya olmadan `FRAME_CLASS`/`FRAME_TYPE` tanımsız kalıyor:
> ```
> AP: Frame: UNSUPPORTED
> AP: PreArm: Motors: Check frame class and type
> ```
> ve iris motorlarını yapılandıramadığı için `NAV_TAKEOFF` başarısız oluyor —
> **kovalama görevi hiç başlamıyor**.
>
> `copter.parm` ayrıca `INS_ACCOFFS`/`INS_ACCSCAL` kalibrasyon işaretlerini ve
> `MOT_THST_HOVER`'ı da getiriyor.
>
> **SIRA ÖNEMLİ:** `avci_copter.parm` **en sonda** kalmalı ki proje değerleri
> (`ANGLE_MAX`, `WPNAV_SPEED`, `FS_*`) üstte kalsın.

> **Neden `pkill` kalıplarında köşeli parantez var?** `'[s]im_vehicle'` yazımı,
> `pkill`'in kendi komut satırını eşleştirip kendini öldürmesini önler.

---

### `start_ardupilot_sitl.sh`

**Satır:** 71 · **Rol:** Sadece iki SITL aracını başlatır (**Gazebo'suz** —
ArduPilot'un built-in fiziğiyle).

```bash
bash scripts/start_ardupilot_sitl.sh          # başlat
bash scripts/start_ardupilot_sitl.sh stop     # durdur
```

Kamera gerekmeyen kontrol/telemetri testleri için kullanışlıdır. **Port
haritasının kaynak dokümantasyonu da bu betiğin başındadır.**

---

### `setup_mission_planner.sh`

**Satır:** 22 · **Rol:** Mission Planner'ı indirir ve `tools/mission_planner/`
altına açar.

- Kaynak: `https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip`
- ~400 MB → depoya girmez (`.gitignore`)
- `MissionPlanner.exe` zaten varsa **atlar** (idempotent)

---

### `start_mission_planner.sh`

**Satır:** 45 · **Rol:** Mission Planner'ı mono ile başlatır (UDP 14551).

mono kurulu değilse anlaşılır bir hata ve kurulum komutu (`mono-complete`,
`libgdiplus`) yazdırır.

---

## 2. `tests/` — Kabul testleri

**Felsefe:** Gazebo/SITL gerektirmeyen **saf mantık** testleri. Sentetik veri
üreteciyle güdüm matematiğini doğrular. Kural: **kod değişikliği önce buradan
geçmeli**, sonra Gazebo'ya çıkmalı.

### `test_visual_lead.py`

**Satır:** 439 · **30 senaryo (T1-T30)** · Çalıştırma:
`python3 -m tests.test_visual_lead`

**Sentetik üreteç (master spec):**
```
a = fx · GOVDE_BOYU_M    / R · sin(aspect)      # gövde projeksiyonu
b = fx · KANAT_ACIKLIGI_M / R · cos(aspect)     # kanat projeksiyonu
```
Bilinen menzil ve yönelimden keypoint'ler türetilir, çekirdeğe verilir, çıktının
beklenen olup olmadığı kontrol edilir.

| Test grubu | Kapsam |
|------------|--------|
| T1-T16, T21 | `guidance_core` — menzil bağımsızlık, tilt telafisi, görüş zarfı, yükselti düzeltmesi sınırları, flip koruması |
| T17-T20, T27-T29 | `adapter_copter` — hız/yaw sözleşmeleri, dikey PN, co-altitude, aim yumuşatma |
| T22-T23 | `supervisor` geçiş zinciri |
| T24-T26 | `visual_lead` terminal davranışı — kör dalış süresi, ıska, gürültülü menzilde kilit |

**Örnek çıktı:**
```
PASS  T27 dikey PN tırmanan hedefte aim'i yukarı kaydırır  girdi=24° çıkış=34.29° pn=11.79°
PASS  T28 terminal co-altitude yukarı yanlılık             base=20.00° term=30.00° Δ=10.00°
SONUÇ: 30/30 geçti — HEPSİ GEÇTİ ✓
```

### `test_gps_guidance.py`

**Satır:** 143 · **9 senaryo (G1-G9)** · Çalıştırma:
`python3 -m tests.test_gps_guidance`

| Test | Kapsam |
|------|--------|
| G1-G6 | `hedef_kadraj_hatasi` matematiği — merkez, yatay-önde, sağda, pitch sapması, arkada, menzil |
| G7 | **Tasarım tutarlılığı:** geometrik kadraj noktasında drone → hedef gerçekten MERKEZDE mi (`yaw≈0`, `elev≈25°`, `menzil≈RANGE_SET`) |
| G8 | İstasyon hedefin hız yönünün **gerisinde** ve **altında** mı |
| G9 | **Döngü duman testi** (sahte bağlantı): komut üretiyor mu, hold'da `vx ≈ hedef hızı` mı, durum ve kadraj alanları dolu mu |

> **G7 özellikle değerli:** GPS fazının tüm iddiası "bu geometrik noktada hedef
> kadrajın merkezindedir" — bu test o iddiayı bağımsız olarak doğrular.

**Bilinen boşluk:** `supervisor`ın gerçek döngüsü ve `common` doğrudan testsiz.

---

## 3. `tools/`

### `gps_log_viz.py`

**Satır:** 293 · **Rol:** Uçuş CSV loglarını **tek dosyalık, kendine yeten
interaktif HTML panele** çevirir.

```bash
python3 tools/gps_log_viz.py                      # en yeni 6 log
python3 tools/gps_log_viz.py --last 8             # en yeni 8 log
python3 tools/gps_log_viz.py logs/a.csv logs/b.csv
python3 tools/gps_log_viz.py --last 4 -o rapor.html --open
```

**Panelde her uçuş için:**

| Görselleştirme | Ne gösterir |
|----------------|-------------|
| Kuşbakışı yörünge | Drone vs hedef izleri — takibin şekli |
| Kamera nişangâhı | Hedefin `(u, v)` piksel izi — merkezleme başarısı |
| Menzil zaman serisi | `d_h` — yaklaşma profili |
| Kadraj açıları | `elev` / `yaw` hataları zaman içinde |
| Otomatik yorum | Veriden türetilen özet |

**Teknik:** Veri JSON olarak HTML şablonuna gömülür (`__DATA__` yer tutucusu),
CSS'te açık/koyu tema (`prefers-color-scheme`) desteği var. Çıktı tamamen
**self-contained** — internet/CDN gerektirmez, çift tıklayıp tarayıcıda açılır.

`_HEDEF_NOKTA` sabitiyle veri seyreltilir (uzun uçuşlarda HTML şişmesin).

### `tools/mission_planner/`

Mission Planner binary'si (mono ile çalışan .NET GUI). `setup_mission_planner.sh`
ile indirilir, **depoya dahil değildir**.

---

## 4. `logs/` — Çalışma zamanı çıktıları

Depoya girmez. İki tür dosya vardır.

### Uçuş CSV'leri (analiz için)

#### `gps_guidance_<tarih>_<saat>.csv` — 29 kolon

| Kolon grubu | Kolonlar |
|-------------|----------|
| Zaman/durum | `t`, `dt`, `durum` |
| Menzil | `d_h` (yatay), `menzil` (slant) |
| Hedef kestirimi | `tgt_x/y/z`, `tgt_vx/vy/vz` |
| Kendi durumumuz | `iris_x/y/z`, `iris_roll_deg`, `iris_pitch_deg`, `iris_yaw_deg` |
| İstasyon | `st_x/y/z` |
| Komut | `vx_cmd`, `vy_cmd`, `vz_cmd`, `yaw_cmd_deg` |
| **Başarı ölçütü** | `kadraj_yaw_deg`, `kadraj_elev_deg`, `kadraj_pitch_hata_deg`, `u_px`, `v_px` |

#### `visual_lead_<tarih>_<saat>.csv` — 41 kolon

| Kolon grubu | Kolonlar |
|-------------|----------|
| Zaman | `t_ros`, `dt`, `gecikme_s` |
| Ham ölçüm | `bbox`, `a`, `b`, `olcek_ham`, `eps_deg`, `duzeltme`, `olcek` |
| Türetilmiş | `yandanlik_ham`, `yandanlik_filtreli`, `kalite`, `lead_deg` |
| Nişan yönü | `u_nisan_x/y/z`, `u_govde_x/y/z`, `yaw_hata_deg`, `pitch_hata_deg` |
| Komut | `vx_cmd`, `vy_cmd`, `vz_cmd`, `yaw_cmd_deg`, `v_doygun`, `yaw_doygun` |
| Durum | `durum`, `flip_sayaci`, `mod` |
| Doğrulama | `menzil_kestirim_m` (log), `menzil_gercek_m` (ground truth), `kapanma_hizi_ms` |
| Quad izleme | `pitch_body_deg`, `kamera_dunya_pitch_deg`, `pn_dikey_deg`, `coalt_deg` |

> **`kamera_dunya_pitch_deg` neden var?** İvme tavanı aşılırsa quad burnunu
> aşağı eğer ve kamera **dünyada** aşağı bakmaya başlar. Bu kolon, gökyüzü arka
> planının ne zaman kaybedildiğini uçuş sonrası tespit etmeyi sağlar.

Format ayrıntısı: `docs/GPS_LOGGING.md`.

### Süreç logları (hata ayıklama)

`gz_harmonic.log`, `sitl_copter.log`, `copter_harmonic.log`, `plane_harmonic.log`,
`gcs_server.log`, `mission_planner.log`, `start_harmonic_run.log` ve tek
seferlik teşhis logları (`copter_frame_fix.log`,
`hamidiyesim-KANIT-copter-UNSUPPORTED.log` — `Frame: UNSUPPORTED` hatasının
kanıt kaydı).

---

## 5. Kök dizin dosyaları

### `README.md` (363 satır)

Proje tanıtımı, mimari tablosu, sistem gereksinimleri, adım adım kurulum,
çalıştırma komutları, kullanım, port haritası, proje yapısı şeması, notlar ve
doküman dizini.

Ayrıca **sıfırdan otomatik kurulum için hazır bir Claude Code prompt'u** içerir:
temiz bir Ubuntu 22.04 makinede depoyu klonlayıp bu prompt'u yapıştırmak, tüm
bağımlılıkları (ROS 2, Gazebo Harmonic, ArduPilot, ardupilot_gazebo, Python
paketleri, Mission Planner) kurup her adımı doğrulamaya yeter.

### `requirements.txt`

```
opencv-python>=4.5      # görüntü işleme
numpy>=1.21             # matris/vektör
fastapi>=0.100          # web API
uvicorn>=0.20           # ASGI sunucu
pydantic>=2.0           # istek gövdesi doğrulama
pymavlink>=2.4          # MAVLink
ultralytics>=8.3        # YOLO (eğitim + çıkarım) — torch ayrıca gerekli (CUDA'lı)
```

> **apt ile gelenler pip'te yok:** `rclpy`, `cv_bridge`, `sensor_msgs`,
> `gazebo_msgs` (ROS 2 Humble) ve `gz.transport13` / `gz.msgs10`
> (`python3-gz-transport13`).

### `.gitignore`

| Girdi | Neden |
|-------|-------|
| `__pycache__/`, `*.py[cod]`, `*.egg-info/` | Python çıktıları |
| `.venv/`, `venv/`, `env/` | Sanal ortamlar |
| `tools/mission_planner/` | ~400 MB binary |
| `logs/`, `*.log`, `*.tlog`, `*.BIN`, `*.bak` | Çalışma çıktıları |
| `eeprom.bin`, `mav.parm`, `*.parm.bak` | ArduPilot SITL geçicileri |
| `*.swp`, `.vscode/`, `.idea/`, `.DS_Store` | Editör/OS |
| `vision/datasets/`, `vision/models/*`, `runs/`, `*.pt` | YOLO üretilen veri |

**İstisna satırları:**
```gitignore
!vision/models/avci_yolo.pt
!vision/models/avci_pose.pt
```
Eğitilmiş güncel modeller (~5.5 MB) depoya **girer**; dataset girmez. Böylece
depoyu klonlayan biri veri toplamadan/eğitmeden sistemi çalıştırabilir.

---

## 6. `docs/` — Doküman dizini

| Doküman | Durum | İçerik |
|---------|-------|--------|
| `SIMULASYON_CALISTIRMA.md` | GÜNCEL | Kopyala-yapıştır 5 terminal komut bloğu. **Sistemi çalıştırmak için ilk bakılacak dosya.** |
| `GUIDANCE.md` | GERÇEKLİK | `control/guidance/` paketinde şu an çalışan sistemin mimarisi: veri akışı, modül tablosu, iki faz, config yüzeyi, frame sözleşmeleri, test/log |
| `GUIDANCE_ROADMAP.md` | PLAN | Hibrit güdümün faz faz tasarımı ve **gerekçeleri**. "Neden böyle tasarlandı" sorusunun cevabı. Faz 1-4 uygulandı. |
| `GPS_LOGGING.md` | GÜNCEL | CSV log formatının kolon kolon anlamı + `gps_log_viz.py` kullanımı |
| `COLAB_TRAINING.md` | GÜNCEL | Veriyi yerelde toplayıp eğitimi Colab GPU'sunda yapma akışı |
| `ARDUPILOT_MIGRATION.md` | TARİHSEL | PX4 → ArduPilot geçişi. ArduPilot'a özgü sözleşmelerin (force ARM magic, mod numaraları, `SYSID_MYGCS`, port haritası) **neden böyle olduğunu** açıkladığı için saklanıyor. |
