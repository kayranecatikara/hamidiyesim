# 90 — Temizlik Kaydı (30 Temmuz 2026)

> Bu belge, dokümantasyon hazırlanırken yapılan depo temizliğini kaydeder.
> **Hiçbir değişiklik commit edilmedi** — hepsi çalışma ağacında duruyor,
> `git status` ile görülebilir, `git checkout` ile geri alınabilir.

**Özet:** 26 dosya değişti · **+492 satır** / **−1.301 satır**

---

## 1. Silinen dosyalar (7)

| Dosya | Satır | Neden silindi |
|-------|------:|---------------|
| `control/px4_shell.py` | 153 | **PX4 dönemi.** MAVLink `SERIAL_CONTROL` üzerinden PX4 SITL shell'ine bağlanıp `commander check` çalıştırıyordu. ArduPilot'ta bu mekanizmanın karşılığı yok. |
| `control/fix_accel_bias.py` | 175 | **PX4 dönemi.** Kullandığı tüm parametreler (`CAL_ACC*`, `COM_ARM_IMU_ACC`, `EKF2_*`) PX4'e ait. |
| `control/cessna_pose_relay.py` | 117 | **Gazebo Classic + Cessna dönemi.** ArduPlane telemetrisini `gazebo_ros /set_entity_state` ile Cessna modeline aktarıyordu. Hem Classic hem Cessna aşıldı. |
| `control/harmonic_pose_relay.py` | 109 | **Kullanım dışı.** Yukarıdakinin Harmonic karşılığıydı. Talon artık Gazebo'da `ArduPilotPlugin` ile (FDM 9012) **gerçekten uçuyor** — relay'e gerek kalmadı. |
| `control/guidance/gps_approach.py` | 370 | **Ölü kod.** Hiçbir yerden import edilmiyordu; `AVCI_HYBRID=off` yolu bile `gps_guidance`'ı çağırıyor. Yerini kadraj merkezleme odaklı `gps_guidance.py` aldı. |
| `control/run_plane_square.py` | 121 | **Aşıldı.** Zaman bazlı rudder dönüşü kullanıyordu; FBWA'da rudder tek başına dönüş üretmediği için kare bozuk çıkıyordu. `run_plane_scenario.py`'nin pusula (ATTITUDE yaw) tabanlı sürümü yerini aldı. |
| `sim/ardupilot/copter_avci.parm` | 7 | **Aşıldı.** `sim/ardupilot_params/avci_copter.parm` (31 satır, gerekçeli) yerini aldı. Boş kalan `sim/ardupilot/` klasörü de kaldırıldı. |

---

## 2. Taşınan dosyalar (6)

Demo betikleri `control/` kökünden **`control/demos/`** altına taşındı ve
yeni bir `control/demos/__init__.py` (kullanım kılavuzlu) eklendi.

| Eski yol | Yeni yol |
|----------|----------|
| `control/run_drone_takeoff.py` | `control/demos/run_drone_takeoff.py` |
| `control/run_drone_hover.py` | `control/demos/run_drone_hover.py` |
| `control/run_drone_square.py` | `control/demos/run_drone_square.py` |
| `control/run_plane_arm.py` | `control/demos/run_plane_arm.py` |
| `control/run_plane_aggressive.py` | `control/demos/run_plane_aggressive.py` |
| `control/run_dual_demo.py` | `control/demos/run_dual_demo.py` |

**Yeni çalıştırma:**
```bash
python3 -m control.demos.run_drone_takeoff     # (eskiden: control.run_drone_takeoff)
```

`control/run_plane_scenario.py` **taşınmadı** — `gcs_server` onu alt-süreç
olarak `python3 -m control.run_plane_scenario` ile başlattığı için kökte kaldı.

---

## 3. Düzeltilen sabit yollar (7 dosya)

Yedi betikte şu satır vardı:

```python
sys.path.insert(0, "/home/kayra/projects/avci_sim")     # ← başka makinenin yolu
```

Yerine dosya konumundan türetilen göreli yol kondu:

```python
import os
# Depo kökünü bu dosyanın konumundan türet (control/demos/ -> depo kökü)
sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
```

Depoda `/home/kayra` referansı **kalmadı**.

---

## 4. `arm_diag.py` — PX4'ten ArduPilot'a çevrildi

Bu dosya "sysid'i düzelt, kullanmaya devam edelim" diye ele alındı — ancak
incelemede **sysid'den çok daha fazlasının PX4'e ait olduğu** görüldü:

| Sorun | Eski (PX4) | Yeni (ArduPilot) |
|-------|-----------|------------------|
| Force ARM magic | `21196` | **`2989`** (21196 ArduPilot'ta force **DISARM**'dır — havada disarm riski) |
| MANUAL mod no | `1` | **`0`** (ArduPlane) |
| Parametre listesi | `COM_*`, `CBRK_*`, `EKF2_*`, `NAV_DLL_ACT`, `SYS_HAS_GPS` | `ARMING_CHECK`, `ARMING_REQUIRE`, `ARMING_RUDDER`, `BRD_SAFETY_DEFLT`, `AHRS_EKF_TYPE`, `EK3_ENABLE`, `GPS_TYPE`, `SIM_GPS_DISABLE`, `RC_PROTOCOLS`, `SYSID_MYGCS`, `FS_GCS_ENABL`, `BATT_MONITOR` |
| Hedef araç | Sabit `sysid=3`, port `14550` | **CLI argümanlı**: `--iris` (5/14541) veya varsayılan Talon (2/14542), `--port`/`--sysid` ile override |
| Davranış | Parametreleri **koşulsuz yazıyordu** | **Varsayılan salt-okunur**; yazma açık `--gevset` bayrağına bağlandı |

**Eklenenler:**
- `ARMING_CHECK` bit maskesini 20 ayrı kontrole (Barometre, Pusula, GPS kilidi,
  INS, RC kanalları, Safety switch…) çözen tablo
- `MAV_SYS_STATUS_SENSOR` bit maskelerini Türkçe isimlere çeviren tablo
- ARM reddi sebeplerinin `STATUSTEXT`'ten (`PreArm: ...`) toplanıp özetlenmesi

```bash
python3 -m control.arm_diag                   # Talon
python3 -m control.arm_diag --iris            # iris
python3 -m control.arm_diag --gevset          # arming kontrollerini gevşet (SITL/test)
```

---

## 5. Kod içi referans düzeltmeleri

| Dosya | Değişiklik |
|-------|-----------|
| `control/__init__.py` | "PX4 simülasyonu için…" → ArduPilot; güncel modül + alt paket listesi |
| `control/guidance/__init__.py` | `gps_approach` girdisi → `gps_guidance` (yeni açıklamasıyla) |
| `control/guidance/common.py` | Docstring'de `gps_approach` → `gps_fazı` |
| `control/gcs_server.py` | Silinen `run_plane_square` için `pkill` satırı kaldırıldı; `chase_algorithm` docstring'i → `supervisor.run_hybrid`; yanlış port yorumu (`14551=plane, 14540=iris`) → `14550=plane, 14541=iris` |
| `control/run_plane_scenario.py` | Silinen dosyaya atıf parantez içine alınarak tarihsel not hâline getirildi |
| `vision/detection_state.py` | `chase_algorithm` referanslı İngilizce yorum → güncel Türkçe açıklama |
| `scripts/start_harmonic.sh` | `pkill` kalıplarından `cessna_pose_relay` çıkarıldı |
| `scripts/start_ardupilot_sitl.sh` | `control.run_drone_takeoff` → `control.demos.run_drone_takeoff` |
| Demo docstring'leri | Kullanım yolları `control.demos.*` oldu; yanlış port numaraları (14540/14541) gerçek değerlerle (14541/14542) değiştirildi |

---

## 6. Doküman güncellemeleri

### `README.md`
- **Mimari bölümü** yeniden yazıldı: "HSV renk tespiti + SPRINT→APPROACH→LOCK→STRIKE
  + PN" → YOLO + YOLO-pose + iki fazlı hibrit güdüm
- **Proje yapısı şeması** güncellendi: artık var olmayan `chase_algorithm.py` /
  `strike_algorithm.py` çıkarıldı; `guidance/`, `demos/`, `vision/`, `tools/`,
  `tests/` ağacı eklendi
- **Kullanım bölümü** güncellendi: senaryo butonları, hibrit güdüm, log
  görselleştirme, GPS jamming yedeği
- **Doküman dizini tablosu** eklendi
- Test çalıştırma hatırlatması eklendi

### `docs/GUIDANCE.md`
- Tüm `gps_approach` referansları `gps_guidance` ile değiştirildi (akış şeması,
  modül tablosu, config tablosu dâhil)
- **"GPS fazı" bölümü tamamen yeniden yazıldı** — kadraj merkezleme yaklaşımı:
  geometrik istasyon noktası, "geri" yönü seçimi, feedforward + PD, kadraj
  hatası ölçümü, DROPOUT, kademe notu
- Test bölümüne `test_gps_guidance.py` (G1-G9) eklendi, log görselleştirme
  bağlantısı kondu
- `AVCI_GPS_RANGE` ortam değişkeni eklendi

### `docs/ARDUPILOT_MIGRATION.md`
- Başa **"TARİHSEL KAYIT"** uyarısı: bu doküman güncel çalıştırma talimatı değil;
  neden saklandığı (ArduPilot sözleşmelerinin gerekçeleri) açıklandı
- "Gazebo Modu" bölümü **"ARTIK GEÇERSİZ"** olarak işaretlendi ve relay
  dosyalarının kaldırıldığı not düşüldü

### `docs/GUIDANCE_ROADMAP.md`
- Başlık durumu "PLAN (onay bekliyor)" → **"Faz 1-4 UYGULANDI"**; belgenin artık
  *tasarım gerekçesi kaydı* olduğu belirtildi
- "2. Mevcut Durum" bölümü **"(plan yazıldığı andaki kod — TARİHSEL)"** olarak
  işaretlendi
- Faz 4 açıklamasındaki `gps_approach` atfına güncel karşılığı eklendi

---

## 7. Doğrulama

Tüm değişikliklerden sonra:

```
✓ Sözdizimi        : tüm .py dosyaları temiz
✓ Import           : control.guidance, control.demos, control.arm_diag yükleniyor
✓ test_gps_guidance:  9/9 geçti
✓ test_visual_lead : 30/30 geçti
✓ Ölü referans     : kodda (py/sh/js/html) hiç kalmadı
```

`docs/` içinde 14 adet `gps_approach` / `cessna_pose_relay` / `chase_algorithm`
geçişi **bilinçli olarak** duruyor — hepsi uyarı bandıyla işaretlenmiş tarihsel
bölümlerin içinde.

---

## 8. Çalıştırma dokümanı düzeltmesi (canlı testle bulundu)

Sistem gerçekten başlatılıp uçtan uca test edildi. `docs/SIMULASYON_CALISTIRMA.md`
üç sebeple çalışmıyordu:

| # | Sorun | Kanıt | Çözüm |
|---|-------|-------|-------|
| 1 | Terminal 1'de **koşulsuz** `export DISPLAY=:1` | Makinede yalnız `/tmp/.X11-unix/X0` var; `:1` yok → Gazebo `Unable to open display ":1"` ile hiç açılmıyor | Satır kaldırıldı; Adım 0'a `ls /tmp/.X11-unix/` kontrolü eklendi |
| 2 | Temizlik komutu `run_plane_scenario`'yu öldürmüyordu | Başlangıçta **6 saat** (`05:59:22`) çalışan bir `run_plane_scenario square` süreci bulundu, 14542'yi tutuyordu | Hem `docs/` hem `README.md` temizlik komutuna eklendi |
| 3 | Terminal 5'te var olmayan `native_libs` yolu | `tools/mission_planner/native_libs` klasörü yok | `LD_LIBRARY_PATH` satırı kaldırıldı |

**Ayrıca eklendi:** ön kontrol adımı, her terminal için "hazır olduğunun
işareti", kopyala-yapıştır doğrulama scripti (4 portta heartbeat + video +
telemetri) ve 7 satırlık sorun giderme tablosu.

### Canlı test sonuçları

```
✓ Gazebo Harmonic        FDM 9002 + 9012 açık
✓ ArduCopter SITL        AP: Frame: QUAD/X  (UNSUPPORTED hatası YOK)
✓ ArduPlane SITL         EKF3 IMU0 is using GPS
✓ Kamera topic'leri      /iris_cam/image + /talon_cam/image
✓ gcs_server             YOLO detector + pose yüklendi, iki kameradan görüntü geldi
✓ Web arayüz             HTTP 307, iki video akışı HTTP 200
✓ Port haritası          14541→sysid 5 · 14542→sysid 2 · 14550/14551→ikisi de
```

---

## 9. Commit edildi ve push'landı

```
commit ae753b1  (main → origin/main)
29 dosya değişti, +897 / −1319 satır
```

```bash
cd ~/projects/avci_sim
git log --oneline -1          # ae753b1
git show ae753b1 --stat       # değişiklik özeti
```

**Geri almak için** (commit push'landığı için `revert` tercih edilmeli):
```bash
git revert ae753b1            # değişiklikleri tersine çeviren yeni commit
git push origin main
```

**Tek bir dosyayı geri almak için:**
```bash
git checkout ae753b1~1 -- control/arm_diag.py   # örnek: arm_diag'ın eski hâli
```
