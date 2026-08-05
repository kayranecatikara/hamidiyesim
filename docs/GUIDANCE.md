# Güdüm Sistemi — Mevcut Mimari

> **Durum:** GERÇEKLİK (kodda şu an çalışan sistem). Vizyon/plan için ayrıca
> [`GUIDANCE_ROADMAP.md`](GUIDANCE_ROADMAP.md).
> **Kapsam:** `control/guidance/` paketi + `control/gcs_server.py` giriş noktaları.

Avcı drone (ArduCopter, quad), hedef İHA'yı (Talon, ArduPlane) hava-havada
kovalayıp vurur. Güdüm iki fazlı: **GPS fazı** hedefi kamera kadrajının merkezine
ve pose'un çalıştığı menzil bandına oturtur, **görsel IBVS** kilitlenip vurur.
Bir denetleyici (supervisor) iki faz arasında geçiş yapar.

---

## 1. Veri akışı

```mermaid
flowchart TD
    UI["POST /api/command/iris/start_chase"] --> CT["_chase_thread (gcs_server)"]
    CT -->|kalkış + callback'ler| SUP["supervisor.run_hybrid"]
    CT -.->|AVCI_HYBRID=off| GA0["gps_guidance (tek başına)"]

    SUP -->|GPS fazı| GA["gps_guidance.run_gps_guidance"]
    SUP -->|görsel faz| VL["visual_lead.run_visual_lead"]
    SUP -. izci: pose akışını sayar .-> GA

    GA -->|hız+yaw| SV["common.send_velocity → MAVLink GUIDED"]
    VL --> CORE["guidance_core.LeadPursuitCore (pose→lead)"]
    CORE --> AD["adapter_copter.CopterAdapter (yön→hız)"]
    AD --> SV

    UIV["POST /api/command/iris/start_visual"] -.->|bağımsız test| VL
```

**Varsayılan görev yolu:** `start_chase` → `_chase_thread` (port bağlantısı +
kalkış) → `supervisor.run_hybrid` → GPS ↔ görsel geçişli döngü → `CopterAdapter`
veya `gps_guidance` → `common.send_velocity` → ArduCopter GUIDED.

**Yan giriş noktaları:**
- `AVCI_HYBRID=off` → hibrit yerine saf `gps_guidance` (GPS'i tek başına test için).
- `POST /api/command/iris/start_visual` → `_visual_thread` → `run_visual_lead`
  doğrudan (görsel hattı izole test; supervisor yok).

Hedefin GERÇEK NED pozu (`get_plane_truth`) yalnız `menzil_gercek` **logu** ve
terminal vuruş tespiti için; güdüm hesabına GİRMEZ (gerçek donanımda yakınlık
sensörü yerini alır). Güdüm, hedefin GPS-gürültülü telemetrisini (`get_plane`)
ve kameranın pose çıktısını kullanır.

### Tespit → kilit → pose zinciri (gcs `process_iris_frame`, 2026-08-02)

Her karede tek YOLO çıkarımı: `detect_all(conf=0.1)` ham tespitleri hem
**HybridSORT** takipçisine (`vision/tracker.py`, `AVCI_TRACKER=off` kapatır)
hem `best_det`'e verir. `set_detection`'a giden kutu **kilitli-ID politikası**
(`TargetLock`, `AVCI_LOCK=off` → eski "en yüksek conf" seçimi): kilitli track
eşleşmişse onun kutusu (`det["track_id"]` eklenir; anlık FP hedefi tek karede
çalamaz), kilit coast'taysa `det=None` (Kalman tahmini yalnız pose KROPUNA
gider), kilit yoksa ham `best_det`. Korumalar: kilit yalnız conf≥0.5 onaylı
track'e; sıçrama koruması (kutu tek karede >3× köşegen atlarsa kilit düşer);
çelişki koruması (güçlü tespit 10 kare başka yerdeyse tazelenir). Ayrıca
`control/sim_truth.py` gz'den gerçek menzil + fiziksel TEMAS olayını dinler —
yalnız GÖZLEM (terminal `[TRUTH]` satırları); güdüm kararlarına bağlı değildir.

---

## 2. Modüller (`control/guidance/`)

| Dosya | Sorumluluk | Girdi → Çıktı |
|---|---|---|
| `common.py` | Paylaşılan matematik + tek MAVLink göndericisi | `clamp/normalize_angle/vec3_len/limit_acceleration`, `send_velocity(conn, vx,vy,vz, yaw)` |
| `guidance_core.py` | **Platformdan bağımsız** IBVS çekirdeği (pose→lead) + frame dönüşümleri | pose 6 keypoint + attitude → `u_govde` (nişan yönü FRD) + `yaw_hata/pitch_hata` + kalite |
| `adapter_copter.py` | Copter adaptörü: nişan yönü → multirotor komutu | `u_govde` → NED hız (`V_KAPANMA`) + slew-limitli yaw; dikey yumuşatma/PN/co-altitude; ivme sınırı |
| `gps_guidance.py` | GPS fazı: kadraj merkezleme (geometrik istasyon + PD hız + hedef-hızı feedforward) | `get_plane/get_iris` → hız+yaw setpoint (20 Hz) + CSV log |
| `visual_lead.py` | Görsel IBVS döngüsü (kameraya kilitli, olay güdümlü) + CSV log + terminal kör-dalış | pose akışı → çekirdek → adaptör → `send_velocity` |
| `supervisor.py` | Faz 4: GPS ↔ görsel geçiş denetleyicisi | tek görev döngüsü `run_hybrid` |
| `__init__.py` | Paket dokümantasyonu | — |

Bağımlılık grafiği (yaprak → tepe): `common` ve `guidance_core` tabandır;
`adapter_copter` ikisine dayanır; `visual_lead` çekirdek+adaptör+common;
`gps_guidance` common+çekirdek(`hedef_kadraj_hatasi`);
`supervisor` gps_guidance+visual_lead+çekirdek(Cfg).

---

## 3. İki faz

### GPS fazı (`gps_guidance.py`)
**Amacı vuruş değil, kadraj merkezleme.** Başarı ölçütü: hedef kameranın tam
ortasında, pose modelinin güvenilir çalıştığı menzil bandında (~10-11 m) ve
kararlı görünsün → supervisor görsel faza devretsin. 20 Hz döngü:

- **Geometrik kadraj noktası (istasyon):** slant menzil `RANGE_SET` (11 m) ve
  `ISTASYON_ELEV_DEG` (15°) bir istasyon noktası belirler: hedefin
  `RANGE_SET·cos15° ≈ 10.63 m` **gerisi** + `RANGE_SET·sin15° ≈ 2.85 m` **altı**.
  Hedef kadrajda, merkezin ~10° altında görünür (v≈269/480 px — test G7 hem
  açıyı hem kenar payını doğrular).

  > **İstasyon açısı kamera tilt'inden neden ayrı?** 2026-08-02'ye kadar tek
  > sayıydı (25°) ve istasyon 4.65 m altta kuruluyordu. Üç uçuşun kara kutusu
  > gösterdi ki ArduPilot dikey hız komutunu `WP_ACC_Z = 1.0 m/s²` ile
  > rampalıyor — güdüm 8-22 m/s tırmanma istese de. Sıfırdan 4.65 m kapatmak
  > 3.05 s sürer, terminalde 2.4-2.8 s var. Sonuç: drone hedefin **altından**
  > geçiyordu (kalan dikey +1.52 m ve +2.06 m; vurabilen tek koşuda +0.03 m).
  > 15°'de kapatılacak mesafe 2.85 m'ye iniyor ve ivme bütçesine sığıyor
  > (test **G11** bunu koruyor). Ayar: `AVCI_GPS_ISTASYON_ELEV`.
- **"Geri" yönünün seçimi:** hedefin yatay hızı `TRACK_MIN_SPD` (3 m/s) üstündeyse
  **hız yönünün gerisi** (kuyruk takibi), altındaysa **LOS gerisi** (drone tarafı)
  — duran/yavaş hedefte hız yönü gürültülü olduğu için.
- **Alttan bakış (look-up):** istasyon hedefin altında olduğundan hedef gökyüzü
  önünde siluet kalır, kamera 25° yukarı tilt'li → pose tespiti kopmaz.
  (Pose dataset'i de bu geometriyle toplandı.) `LOOKUP_MIN_ALT` (8 m) yere
  çakılmayı önler.
- **Hız komutu:** hedef-hızı **feedforward** + istasyon hatasına **PD**
  (`KP_H/KD_H` yatay, `KP_Z` dikey). Feedforward sayesinde kilitlenince drone
  hedefin hızıyla birlikte gider — kararlı hold, sürekli kovalama salınımı yok.
  `V_MAX`, `VZ_MAX` ve `MAX_ACCEL` ile sınırlanır.
- **Yaw:** burun daima **gerçek hedefe** döner (istasyona değil), rate-limitli.
- **Kadraj hatası ölçümü:** her karede `guidance_core.hedef_kadraj_hatasi` ile
  drone'un GERÇEK attitude'undan azimut/yükseliş hatası ve piksel (u,v) kapalı
  formda hesaplanıp CSV'ye yazılır → merkezleme başarısı ölçülebilir.
- **Tazelik:** hedef telemetrisi `HOLD_S` (3 sn) donuk kalırsa `DROPOUT` — hover
  edilir ve supervisor jamming fallback'i devreye alır.
- **Devir etiketi:** `d_h < HANDOFF_RANGE` (20 m) → `durum=KILIT`; supervisor bunu
  görsel kilit sayacıyla birleştirir.

> **Kademe notu:** Bu sürüm KADEME 1 — istasyon geometrik olarak kurulur.
> KADEME 2'de ölçülen kadraj hatası doğrudan geri beslemeye girecek.

### Görsel IBVS (`visual_lead.py` + `guidance_core.py` + `adapter_copter.py`)
Sabit Hz'te DÖNMEZ — kamera karesi (30 Hz) geldikçe işler. Her kabul edilen karede:
1. **Çekirdek (`LeadPursuitCore.process`, Adım 1-8):** 6 pose keypoint'inden hedefin
   görünür yönelimini çıkarır, saf takip yönünün üstüne **menzilden bağımsız** bir
   öne-nişan (lead) bindirir. Tek ayar `K_LEAD ≈ hedef_hızı/bizim_hız`. Çıktı bir
   **yöndür** (`u_govde`, FRD), menzil ölçülmez (`menzil_kestirim_m` yalnız log).
2. **Adaptör (`CopterAdapter.compute`, Adım 9):** yönü dünya-NED hız vektörüne çevirir
   (`V_KAPANMA` sabit kapanma hızı), dikey düzlemde yumuşatma + PN lead + terminal
   co-altitude uygular, ivme sınırı ve slew-limitli yaw ekler, `send_velocity` ile gönderir.
   `SET_ATTITUDE_TARGET` kullanılmaz (multirotorda yanlış araç).
3. **Terminal kör-dalış:** menzil `TERMINAL_MENZIL` (8 m) altında iken temas koparsa
   GPS'e DÖNMEZ; son nişan komutunu `TERMINAL_SURE` boyunca sürdürür (hedef kadraj
   tepesinden çıkınca çarpışmayı tamamlar). `VURUS_MENZIL` (3 m) altı = VURULDU.

### Geçiş mantığı (`supervisor.run_hybrid`)
Tek görev döngüsü. `izci()` alt-thread'i pose akışını sayar:
- **GPS → görsel:** `KILIT_N` (10) ardışık güvenli pose karesi (conf ≥ `POSE_CONF_MIN`)
  **VE** kapı: `d_h < GATE_MENZIL` (20 m) **YA DA** GPS `DROPOUT` (jamming fallback).
- **görsel → GPS:** `KAYIP_M` (20) ardışık pose'suz kare veya kare akışının durması.
- **Bitiş:** `run_visual_lead` `"vuruldu"` dönerse görev tamamlanır (`faz=VURULDU`);
  `stop_chase` gelirse durur.

Menzil kapısının nedeni: görsel fazın kapanma hızı sabit; uzaktan erken geçilirse
hızlı hedefe yetişilemez. Pose asıl 10-12 m'de sağlam; kapı 20 m bandı seçer.

---

## 4. Config yüzeyi

Üç ayrı config; **bilinçli olarak** ayrı tutuluyor (birleştirme rebuild fazına ertelendi):

| Sınıf | Dosya | Alan (domain) | Örnek parametreler |
|---|---|---|---|
| `Cfg` | `guidance_core.py:40` | **IBVS/görsel** | `K_LEAD`, `V_KAPANMA`, `KP_YAW`, `IVME_TAVAN`, `TERMINAL_MENZIL/SURE`, `VURUS_MENZIL`, `ELEV_EMA`, `PN_LEAD_SURE`, `TERMINAL_COALT_*` |
| `Cfg` | `gps_guidance.py:43` | **GPS fazı (kadraj)** | `RANGE_SET`, `CENTER_ELEV_DEG`, `TRACK_MIN_SPD`, `LOOKUP_MIN_ALT`, `KP_H/KD_H`, `KP_Z/VZ_MAX`, `V_MAX/MAX_ACCEL`, `HANDOFF_RANGE`, `POS_EMA/VEL_EMA`, `HOLD_S` |
| `SupCfg` | `supervisor.py:28` | **geçiş** | `KILIT_N`, `KAYIP_M`, `POSE_CONF_MIN`, `GATE_KILIT`, `GATE_MENZIL` |

> **Not:** İki config sınıfı da `Cfg` adında; `supervisor` içine `LeadCfg` olarak
> import edilir. İsim çakışması yalnız kozmetik (Python ayırır). Birleştirme/yeniden
> adlandırma bilinçli olarak **rebuild fazına** bırakıldı (config o zaman yeniden tasarlanacak).

**Ortam değişkenleri** (canlı ayar; `_env_f` = float okuyucu):
`AVCI_IBVS_K_LEAD`, `AVCI_IBVS_V_KAPANMA`, `AVCI_IBVS_TERMINAL_MENZIL`,
`AVCI_IBVS_TERMINAL_SURE`, `AVCI_IBVS_VURUS_MENZIL`, `AVCI_IBVS_PN_SURE`,
`AVCI_IBVS_COALT_MENZIL`, `AVCI_POSE_KPT_CONF`, `AVCI_GT_ROT` (guidance_core);
`AVCI_GPS_RANGE` (gps_guidance: slant menzil setpoint);
`AVCI_HYBRID_GATE_MENZIL` (supervisor); `AVCI_HYBRID` (gcs_server: hibrit/saf-GPS seçimi).

---

## 5. Frame ve geometri sözleşmeleri

Tüm dönüşümler `guidance_core.py`'de tek noktada:
- **`kamera_to_govde(u, tilt)`** — OpenCV kamera (X sağ, Y aşağı, Z ileri) → gövde
  FRD (X ileri, Y sağ, Z aşağı). Montaj **25° yukarı tilt** Ry ile uygulanır
  (atlanırsa sürekli 25° sabit hata). Doğrulama: merkez → `[0.906, 0, -0.423]`.
- **`govde_to_dunya(u, roll, pitch, yaw)`** — gövde FRD → dünya NED, DCM = Rz·Ry·Rx.
- **Look-up düzeltmesi** (`yukselti_duzeltme`) — LOS'un yatayla açısına göre ölçek
  düzeltmesi (alttan yaklaşmada gövde/kanat izdüşümü kısalmasını telafi eder).
- Kamera intrinsics tek dış bağımlılık: `vision.geometry` (`FX/FY/CX/CY`).

Görüş zarfı (640×480, HFOV 125°): kadraj tepesi ≈ +80° yükseliş, tabanı ≈ −30°,
merkez = boresight +25° (kamera tilt'i).

---

## 5b. GT rotasyon modu (`AVCI_GT_ROT=on`) — teşhis aracı

Varsayılan **kapalı**. Açıkken görsel güdümün **tüm algı girdileri** pose
modelinden değil Gazebo'nun gerçek pozundan gelir:

| Girdi | Pose modu (varsayılan) | GT modu |
|---|---|---|
| Nişan noktası | detection bbox merkezi | hedefin izdüşümü (`rot_gt_goruntu.uv`) |
| Gövde ekseni yönü (`d_birim`) | burun−kuyruk keypoint'leri | gerçek yönelimin izdüşümü |
| Yandanlık | `a/olcek` (izdüşüm oranı) | gerçek \|sin θ\| (gövde ekseni ↔ LOS) |
| Ölçek (`olcek`) | keypoint uzunlukları | `fx·L/R`, gerçek menzilden |
| Güven/kalite kapıları | keypoint conf | devre dışı (güven = 1.0) |

**Adım 3-8 geometrisi iki modda da aynıdır** — değişen yalnız Adım 1-2'nin
nereden beslendiğidir. Bu kasıtlı: aynı yasa, farklı algı → CSV sütunları
birebir kıyaslanabilir. Iska varsa suç GT modunda **yasada**, pose modunda
**algıdadır**.

Akış: `sim_truth.pozlar()` (iki araç TEK gz mesajından, zaman hizalı) →
`geometry.rot_gt_goruntu()` → `gcs_server._gt_rot_girdi()` → `visual_lead` →
`guidance_core.process(..., gt=...)`.

**Davranış farkları** (bilinçli):
- `pose is None` karesi artık güdümü durdurmaz — "görsel temas"ın anlamı
  *pose var mı* yerine *GT akışı canlı mı*dır. Pose kaybı GPS'e döndürmez.
- Yükselti düzeltmesi uygulanmaz (`duzeltme = 1.0`): o düzeltme **ölçülen**
  ölçeğin perspektif kısalmasını telafi eder, gerçeğe uygulanırsa hata ekler.
- `menzil_kestirim_m` sütunu artık gerçek menzili aynen yansıtır; GT modunda
  **algı sapması ölçülemez** (cevap anahtarı sütunları anlamsızlaşır).
- Supervisor'ın GPS→görsel geçiş kapısı **hâlâ pose kilidine bakar** (pose
  modeli GT modunda da yüklü kalır). Saf GT koşusu için `/start_visual`
  uç noktası supervisor'ı atlar.

> ⚠ **Simülasyona özgüdür, gerçek donanıma taşınamaz.** Gerçek harekâtta hedefin
> pozu bilinmez — görsel güdümün varlık sebebi budur. Bu mod bir teşhis aracıdır,
> teslim edilecek güdüm değildir.

**Bilinen ayrışma:** hedef bank yaparken pose yandanlığı gerçeğin altında kalır
(`a/olcek` "hedef seviyeli uçuyor" varsayar; 45° bankta 1.00 yerine 0.73 ölçer —
`tests/test_visual_lead.py` T53b). GT modu bu manevra körlüğünü ortadan kaldırır;
iki modun ıska farkı buradan gelebilir.

---

## 6. Test ve loglar

- **Görsel hat testleri:** `python3 -m tests.test_visual_lead` — 30 senaryo (T1-T30);
  çoğunlukla `guidance_core` (T1-16,21) ve `adapter_copter` (T17-20,27-29), ayrıca
  `supervisor` geçiş zinciri (T22-23) ve `visual_lead` terminal (T24-26).
- **GPS fazı testleri:** `python3 -m tests.test_gps_guidance` — 9 senaryo (G1-G9):
  kadraj hatası matematiği (G1-G6), istasyon geometrisinin merkezleme tutarlılığı
  (G7-G8) ve sahte bağlantıyla döngü duman testi (G9).
- **GT rotasyon modu testleri:** aynı dosyada T49-T54 — kadraj merkezi (T49),
  pose'suz çalışma (T50), gerçek menzilden ölçek (T51), yandanlık (T52),
  **GT yolu ≈ kusursuz pose yolu** (T53: Δyaw < 2°, Δyandanlık < 0.02),
  yatık hedefte bilinçli ayrışma (T53b) ve `gt=None` regresyonu (T54).
- **Ölçüm aracı testleri:** `python3 -m tests.olcum_araclari` — 22 senaryo;
  güdümü değil `tools/` altındaki ölçüm araçlarını denetler: kara kutu
  geometrisi ve zaman hizalama (E1-E8), CSV karne metrikleri (K1-K8),
  parametre denetimi (P1-P6). Araçlar uçuş kaydı okur ve kayıtlar depoda
  yoktur; test edilen kısım içlerindeki **saf hesap**.
- **Bilinen boşluk:** `supervisor`ın gerçek döngüsü ve `common` doğrudan testsiz.
- **Uçuş logları:** her faz kendi CSV'sini yazar —
  `logs/visual_lead_<zaman>.csv` (pose ölçümleri, nişan yönü, komutlar,
  `pn_dikey_deg`, `coalt_deg`, `menzil_gercek_m`) ve
  `logs/gps_guidance_<zaman>.csv` (istasyon, komutlar, kadraj açıları, `u_px/v_px`).
  `visual_lead` CSV'sinin varlığı görsel faza gerçekten geçildiğini gösterir.
  Görselleştirme: `python3 tools/gps_log_viz.py --last 6 --open`
  (format: [`GPS_LOGGING.md`](GPS_LOGGING.md)).

---

## 7. Sonraki faz (kapsam dışı)

Bu doküman **temizlenmiş mevcut sistemi** anlatır. Bir sonraki adım: güdüm yasalarını
**sıfırdan, en basit algoritmadan gelişmişine** doğru yeniden inşa etmek (ör. saf
takip → oransal-seyrüsefer/PN → tam kesme). O aşamada:
- İki `Cfg` sınıfı tek tutarlı config'e birleştirilecek.
- `common.py` / `guidance_core.py` frame ve MAVLink tabanı yeniden kullanılacak
  (kanıtlanmış, testli).
- Terminal alttan-geçiş/ıska sorunu temiz tabandan ele alınacak.
