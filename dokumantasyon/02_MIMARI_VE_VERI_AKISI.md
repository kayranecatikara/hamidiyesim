# 02 — Mimari ve Veri Akışı

> Sistem uçtan uca nasıl çalışır: süreçler, portlar, kare akışı, görev akışı.

---

## 1. Süreç haritası

Sistem **5 ayrı süreçten** oluşur; her biri kendi terminalinde çalışır.

| # | Süreç | Görevi | Bağlandığı yerler |
|---|-------|--------|-------------------|
| 1 | **Gazebo Harmonic** | Fizik motoru + render. İki aracı ve iki kamerayı simüle eder. | FDM 9002 (iris), 9012 (Talon); gz-transport topic'leri |
| 2 | **ArduCopter SITL** | iris'in otopilotu. Gazebo'dan sensör alır, motor komutu döner. | FDM 9002 ↔ Gazebo; UDP 14541/14550/14551 çıkış |
| 3 | **ArduPlane SITL** | Talon'un otopilotu. | FDM 9012 ↔ Gazebo; UDP 14542/14550/14551 çıkış |
| 4 | **gcs_server** | Web arayüzü + kamera + telemetri + görev kontrolü. | gz-transport (kamera), UDP 14550/14541, HTTP 8000 |
| 5 | **Mission Planner** | Standart ArduPilot yer istasyonu (isteğe bağlı). | UDP 14551 |

---

## 2. Port haritası

```
┌─────────────────────┐
│  Gazebo Harmonic    │
│  ┌───────────────┐  │   FDM 9002    ┌──────────────────┐
│  │   iris_cam    │◄─┼───────────────┤  ArduCopter SITL │
│  └───────┬───────┘  │               │  (-I0, sysid 5)  │
│          │          │               └────────┬─────────┘
│  ┌───────┴───────┐  │   FDM 9012             │ UDP
│  │mini_talon_vtail│◄─┼──────────────┐        │ 14541 ─────┐
│  └───────┬───────┘  │              │        │ 14550 ──┐  │
│          │          │     ┌────────┴──────┐ │ 14551 ─┐│  │
└──────────┼──────────┘     │ ArduPlane SITL│ │        ││  │
           │                │ (-I1, sysid 2)│─┘        ││  │
   gz-transport             └───────────────┘          ││  │
   /iris_cam/image                UDP 14542 ───────────┘│  │
   /talon_cam/image               UDP 14550 ────────────┤  │
           │                      UDP 14551 ────────────┼┐ │
           │                                            ││ │
           ▼                                            ▼▼ ▼
    ┌──────────────────────────────────────────────────────────┐
    │                    gcs_server.py                          │
    │  · kamera oku → YOLO + pose → overlay → MJPEG            │
    │  · telemetri topla (plane: 14550, iris: 14541)           │
    │  · görev API + WebSocket → tarayıcı                       │
    └────────────────────────┬─────────────────────────────────┘
                             │ HTTP 8000
                             ▼
                    http://localhost:8000
                    (gcs_ui — Avcı Operasyon Merkezi)

                    UDP 14551 ──► Mission Planner
```

| Port | Kullanım | Kim bind eder |
|------|----------|---------------|
| 9002 / 9012 | Gazebo ↔ SITL FDM (iris / Talon) | ArduPilot eklentisi |
| 14541 | iris kontrol + telemetri | `drone_functions`, `gcs_server` |
| 14542 | Talon kontrol | `plane_functions`, `run_plane_scenario` |
| 14550 | gcs_server ana telemetri (her iki araç yayın yapar) | `gcs_server` |
| 14551 | Mission Planner (her iki araç yayın yapar) | Mission Planner |
| 8000 | Web arayüz + MJPEG akışı | `gcs_server` (uvicorn) |

> **Neden 14550 ve 14551 ayrı?** UDP'de bir portu tek süreç bind edebilir.
> `gcs_server` ve Mission Planner aynı anda çalışsın diye her birine ayrı port
> verildi; SITL ikisine birden yayın yapar.

---

## 3. Kamera kare akışı (30 Hz)

```
Gazebo iris_cam sensörü
   │  gz-transport /iris_cam/image (Image mesajı, header.stamp = SİM saati)
   ▼
gz_iris_camera_thread()  ──── wall_recv = time.time() damgası vurulur
   │
   ▼
process_iris_frame(img, stamp, wall_recv)
   │
   ├──► detector.detect_talon(TEMİZ kare)   → bbox + conf   → set_detection()
   ├──► pose_detector.detect_pose(TEMİZ kare) → 6 keypoint  → set_pose_detection(pose, stamp, wall_recv)
   │                                                             │
   ├──► overlay çiz (bbox + keypoint + iskelet)                  │  Condition.notify_all()
   ├──► video parazit simülasyonu uygula                         │
   └──► JPEG kodla → latest_frames["iris"]                       │
              │                                                   ▼
              ▼                                        visual_lead döngüsü
       MJPEG akışı → tarayıcı                          (kareye kilitli, uyanır)
```

**Kritik ayrıntılar:**

- **İki model de TEMİZ kare üzerinde çalışır.** Overlay'ler sonradan çizilir —
  detection kutusu çizilmiş kare pose modeline girerse çıkarımı bozar.
- **İki farklı saat kullanılır ve karıştırılmaz:**
  - `stamp` (kare header'ı, **sim saati**) → `dt` hesabı, filtreler, PN türevi
  - `wall_recv` (**duvar saati**) → bayat kare tespiti (`gecikme_s`)
- **Döngü sabit Hz'te dönmez.** `wait_new_pose(son_seq)` bir `threading.Condition`
  üzerinde bekler; kare geldikçe uyanır. Sabit Hz'te dönen bir döngü aynı kareyi
  tekrar işler ve bayat veriyle komut üretir.

---

## 4. Görev akışı (start_chase → vuruş)

```
Kullanıcı → POST /api/command/iris/start_chase
   │
   ▼
_chase_thread (gcs_server)
   ├── iris telemetri worker'ını durdur (port serbest kalsın)
   ├── conn = connect_drone(14541)
   ├── takeoff_to_z(-5.0)              # GUIDED + NAV_TAKEOFF
   └── supervisor.run_hybrid(conn, get_plane, get_iris, wait_pose, get_plane_truth, stop)
          │
          ▼
   ┌───────────────────── HİBRİT DÖNGÜ ─────────────────────┐
   │                                                          │
   │  ══ GPS FAZI ══  gps_guidance.run_gps_guidance (20 Hz)  │
   │   · hedefin gerisine+altına geometrik istasyon kur       │
   │   · hedef-hızı feedforward + PD ile oraya git            │
   │   · burun daima gerçek hedefe                            │
   │   · her kare: kadraj hatası ölç → CSV                    │
   │                                                          │
   │   izci() alt-thread'i pose akışını sayar:                │
   │     10 ardışık güvenli pose karesi (conf ≥ 0.5)          │
   │       VE (d_h < 20 m  VEYA  GPS DROPOUT)                 │
   │            │                                             │
   │            ▼ faz_stop.set()                              │
   │  ══ GÖRSEL FAZ ══  visual_lead.run_visual_lead           │
   │   · kare geldikçe: guidance_core.process → adapter       │
   │   · menzilden bağımsız lead → hız + yaw komutu           │
   │   · menzil < 8 m ve temas koparsa → KÖR DALIŞ (kilitli)  │
   │   · menzil < 3 m → VURULDU ✓                             │
   │            │                                             │
   │            ├── "vuruldu"  → görev tamamlandı, çık        │
   │            ├── "kayip"    → GPS fazına dön ──────────────┤
   │            └── "durduruldu" → çık                        │
   └──────────────────────────────────────────────────────────┘
```

### Faz geçiş kuralları

| Geçiş | Koşul | Gerekçe |
|-------|-------|---------|
| GPS → Görsel | 10 ardışık pose karesi (conf ≥ 0.5) **VE** (`d_h < 20 m` **VEYA** GPS DROPOUT) | Görsel fazın kapanma hızı sabit (25 m/s); uzaktan erken geçilirse hızlı hedefe yetişilemez. DROPOUT'ta menzil bilinemez → görsel temas tek başına yeter (jamming yedeği). |
| Görsel → GPS | 20 ardışık pose'suz kare **VEYA** kare akışı > 1 sn durursa | Temas kaybı; GPS ile yeniden yaklaş. |
| Görsel → Bitiş | Menzil < 3 m | Fiziksel temas = VURULDU. |
| Kör dalış | Menzil < 8 m iken temas koparsa | Son ~6 m'de hedef kadraj tepesinden çıkıp tespit kopar; GPS'e dönmek yerine son nişan 0.6 sn sürdürülür. |

---

## 5. Koordinat sistemleri ve dönüşümler

Projede **dört ayrı çerçeve** var; karışması en sık hata kaynağı olduğu için
tüm dönüşümler tek dosyada (`guidance_core.py`) toplandı.

| Çerçeve | Eksenler | Nerede |
|---------|----------|--------|
| **Kamera (OpenCV)** | X sağ, Y aşağı, Z ileri | `pose_detector`, `geometry` |
| **Gövde (FRD)** | X ileri, Y sağ, Z aşağı | Güdüm çekirdeği çıktısı (`u_govde`) |
| **Dünya (NED)** | X kuzey, Y doğu, Z aşağı | ArduPilot telemetrisi, hız komutları |
| **Gazebo (ENU)** | X doğu, Y kuzey, Z yukarı | SDF dosyaları, `set_pose` servisi |

**Dönüşüm zinciri (görsel faz):**

```
piksel (u,v)
   │  intrinsics (FX, FY, CX, CY)
   ▼
kamera birim vektörü [sağ, aşağı, ileri]
   │  kamera_to_govde(u, tilt=25°)      ← Ry rotasyonu, TEK nokta
   ▼
gövde FRD (u_govde)
   │  govde_to_dunya(u, roll, pitch, yaw)  ← DCM = Rz·Ry·Rx
   ▼
dünya NED birim vektörü
   │  × V_KAPANMA (25 m/s)
   ▼
hız komutu (vx, vy, vz) → SET_POSITION_TARGET_LOCAL_NED
```

> **25° tilt neden kritik?** Kamera gövdeye 25° yukarı bakacak şekilde
> sabitlenmiştir. `kamera_to_govde` bu tilt'i uygulamazsa güdüme **sürekli 25°
> sabit hata** girer — drone hedefin sürekli altını nişanlar. Doğrulama testi:
> kadraj merkezi `[0,0,1]` + tilt 25° → `[0.906, 0, -0.423]`.

---

## 6. Güdüme giren ve girmeyen veriler

Bu ayrım bilinçlidir: simülasyonda kolay erişilen ama gerçek donanımda
olmayacak veriler güdüm hesabına sokulmaz.

| Veri | Kaynak | Güdüme girer mi? |
|------|--------|------------------|
| Hedefin GPS telemetrisi (gürültülü) | `get_plane()` → `_apply_gps_noise` | ✅ GPS fazında |
| Kendi pozisyon + attitude | `LOCAL_POSITION_NED`, `ATTITUDE` | ✅ Her iki fazda |
| Pose keypoint'leri | `pose_detector` | ✅ Görsel fazda |
| Hedefin **gerçek** NED pozu | `get_plane_truth()` | ❌ **Sadece log + vuruş tespiti** |
| Menzil kestirimi (piksel ölçeğinden) | `guidance_core` | ❌ **Sadece log** |

> Gerçek donanımda `get_plane_truth`'un yerini bir yakınlık/menzil sensörü alır.
> Menzil kestiriminin güdüme bağlanmaması bilinçli bir tasarım kuralı: lead
> açısı **menzilden bağımsız** çalışır, tek ayarı `K_LEAD`'dir.

---

## 7. Test → simülasyon sırası

```
1. Saf mantık testleri (Gazebo YOK, saniyeler)
      python3 -m tests.test_visual_lead      → 30/30 geçmeli
      python3 -m tests.test_gps_guidance     →   9/9 geçmeli
                    │
                    ▼
2. Gazebo + SITL (docs/SIMULASYON_CALISTIRMA.md — 5 terminal)
                    │
                    ▼
3. Uçuş → logs/*.csv üretilir
                    │
                    ▼
4. Analiz:  python3 tools/gps_log_viz.py --last 6 --open
```

Kural: **kod değişikliği önce testlerden geçmeli**, sonra Gazebo'ya çıkmalı.
Testler sentetik keypoint üreteciyle çalışır — Gazebo açmadan matematiği
doğrular.
