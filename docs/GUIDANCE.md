# Güdüm Sistemi — Mevcut Mimari

> **Durum:** GERÇEKLİK (kodda şu an çalışan sistem). Vizyon/plan için ayrıca
> [`GUIDANCE_ROADMAP.md`](GUIDANCE_ROADMAP.md).
> **Kapsam:** `control/guidance/` paketi + `control/gcs_server.py` giriş noktaları.

Avcı drone (ArduCopter, quad), hedef İHA'yı (Talon, ArduPlane) hava-havada
kovalayıp vurur. Güdüm iki fazlı: **GPS yaklaşma** ile menzile girilir, **görsel
IBVS** ile kilitlenip vurulur. Bir denetleyici (supervisor) iki faz arasında geçiş yapar.

---

## 1. Veri akışı

```mermaid
flowchart TD
    UI["POST /api/command/iris/start_chase"] --> CT["_chase_thread (gcs_server)"]
    CT -->|kalkış + callback'ler| SUP["supervisor.run_hybrid"]
    CT -.->|AVCI_HYBRID=off| GA0["gps_approach (tek başına)"]

    SUP -->|GPS fazı| GA["gps_approach.run_gps_approach"]
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
veya `gps_approach` → `common.send_velocity` → ArduCopter GUIDED.

**Yan giriş noktaları:**
- `AVCI_HYBRID=off` → hibrit yerine saf `gps_approach` (GPS'i tek başına test için).
- `POST /api/command/iris/start_visual` → `_visual_thread` → `run_visual_lead`
  doğrudan (görsel hattı izole test; supervisor yok).

Hedefin GERÇEK NED pozu (`get_plane_truth`) yalnız `menzil_gercek` **logu** ve
terminal vuruş tespiti için; güdüm hesabına GİRMEZ (gerçek donanımda yakınlık
sensörü yerini alır). Güdüm, hedefin GPS-gürültülü telemetrisini (`get_plane`)
ve kameranın pose çıktısını kullanır.

---

## 2. Modüller (`control/guidance/`)

| Dosya | Sorumluluk | Girdi → Çıktı |
|---|---|---|
| `common.py` | Paylaşılan matematik + tek MAVLink göndericisi | `clamp/normalize_angle/vec3_len/limit_acceleration`, `send_velocity(conn, vx,vy,vz, yaw)` |
| `guidance_core.py` | **Platformdan bağımsız** IBVS çekirdeği (pose→lead) + frame dönüşümleri | pose 6 keypoint + attitude → `u_govde` (nişan yönü FRD) + `yaw_hata/pitch_hata` + kalite |
| `adapter_copter.py` | Copter adaptörü: nişan yönü → multirotor komutu | `u_govde` → NED hız (`V_KAPANMA`) + slew-limitli yaw; dikey yumuşatma/PN/co-altitude; ivme sınırı |
| `gps_approach.py` | GPS yaklaşma yasası (kuyruk-standoff + göreli fren + alttan bakış) | `get_plane/get_iris` → hız+yaw setpoint (20 Hz) |
| `visual_lead.py` | Görsel IBVS döngüsü (kameraya kilitli, olay güdümlü) + CSV log + terminal kör-dalış | pose akışı → çekirdek → adaptör → `send_velocity` |
| `supervisor.py` | Faz 4: GPS ↔ görsel geçiş denetleyicisi | tek görev döngüsü `run_hybrid` |
| `__init__.py` | Paket dokümantasyonu | — |

Bağımlılık grafiği (yaprak → tepe): `common` ve `guidance_core` tabandır;
`adapter_copter` ikisine dayanır; `visual_lead` çekirdek+adaptör+common; `gps_approach`
common; `supervisor` gps_approach+visual_lead+çekirdek(Cfg).

---

## 3. İki faz

### GPS yaklaşma (`gps_approach.py`)
Eski kanıtlanmış `ana_kontrol.py` güdüm yasasının portu. 20 Hz döngü:
- **Kuyruk-standoff:** komut istasyonu, hedefin **hız yönünün** `APPROACH_STANDOFF`
  (10 m) gerisinde → drone yandan yetişse bile arkaya süzülür, hedef hep kadrajda.
- **Göreli fren:** hız tavanı `min(V_CAP_FAR, hedef_hızı + kapanma_payı(d))`;
  kapanma payı `BRAKE_DIST` altında `V_CLOSE_FAR→V_CLOSE_NEAR` iner (overshoot yok,
  ama tavan hedefin kendi hızının altına inmez).
- **Alttan bakış (look-up):** avcı hedefin `APPROACH_ALT_OFFSET` (≈5 m) **altında**
  uçar → hedef gökyüzü önünde siluet, kamera 25° yukarı tilt'li → kadraj oturur,
  pose tespiti kopmaz. (Pose dataset'i de bu geometriyle toplandı.)
- **Handoff histerezisi:** `d_h < HANDOFF_RANGE` → KILIT bayrağı; supervisor bunu
  görsel kilit sayacıyla birleştirir.

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
| `Cfg` | `gps_approach.py:62` | **GPS yaklaşma** | `APPROACH_STANDOFF`, `V_CAP_FAR/CLOSE_*`, `BRAKE_DIST`, `KP_H/KD_H`, `KP_Z_POS/VZ_MAX`, `HANDOFF_RANGE`, `POS_EMA/VEL_EMA` |
| `SupCfg` | `supervisor.py:28` | **geçiş** | `KILIT_N`, `KAYIP_M`, `POSE_CONF_MIN`, `GATE_KILIT`, `GATE_MENZIL` |

> **Not:** İki config sınıfı da `Cfg` adında; `supervisor` içine `LeadCfg` olarak
> import edilir. İsim çakışması yalnız kozmetik (Python ayırır). Birleştirme/yeniden
> adlandırma bilinçli olarak **rebuild fazına** bırakıldı (config o zaman yeniden tasarlanacak).

**Ortam değişkenleri** (canlı ayar; `_env_f` = float okuyucu):
`AVCI_IBVS_K_LEAD`, `AVCI_IBVS_V_KAPANMA`, `AVCI_IBVS_TERMINAL_MENZIL`,
`AVCI_IBVS_TERMINAL_SURE`, `AVCI_IBVS_VURUS_MENZIL`, `AVCI_IBVS_PN_SURE`,
`AVCI_IBVS_COALT_MENZIL`, `AVCI_POSE_KPT_CONF` (guidance_core);
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

## 6. Test ve loglar

- **Testler:** `tests/test_visual_lead.py` (elle yazılı, `python3 -m tests.test_visual_lead`).
  30 senaryo; çoğunlukla `guidance_core` (T1-16,21) ve `adapter_copter` (T17-20,27-29),
  ayrıca `supervisor` geçiş zinciri (T22-23) ve `visual_lead` terminal (T24-26).
  `gps_approach`, `supervisor` gerçek döngüsü ve `common` doğrudan testsiz (bilinen boşluk).
- **Uçuş logu:** `visual_lead.py` her görsel fazda `logs/visual_lead_<zaman>.csv` yazar
  (pose ölçümleri, nişan yönü, komutlar, `pn_dikey_deg`, `coalt_deg`, `menzil_gercek_m`).
  CSV'nin varlığı görsel faza gerçekten geçildiğini gösterir.

---

## 7. Sonraki faz (kapsam dışı)

Bu doküman **temizlenmiş mevcut sistemi** anlatır. Bir sonraki adım: güdüm yasalarını
**sıfırdan, en basit algoritmadan gelişmişine** doğru yeniden inşa etmek (ör. saf
takip → oransal-seyrüsefer/PN → tam kesme). O aşamada:
- İki `Cfg` sınıfı tek tutarlı config'e birleştirilecek.
- `common.py` / `guidance_core.py` frame ve MAVLink tabanı yeniden kullanılacak
  (kanıtlanmış, testli).
- Terminal alttan-geçiş/ıska sorunu temiz tabandan ele alınacak.
