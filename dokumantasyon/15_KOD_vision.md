# 15 — `vision/` Görüntü İşleme ve Model Eğitimi

> ⚠ **2026-08-10:** keypoint dönemine ait bölümler (keypoint tanımı,
> `talon_keypoints`/`occluded_mask`/`target_keypoints`, `pose_detector.py`,
> `capture_pose_dataset.py`, köprünün eski tarafı) buradan çıkarıldı →
> [`POSEA_GERI_DONMEK_ISTERSENIZ/docs/15_KOD_vision_keypoint_bolumleri.md`](../POSEA_GERI_DONMEK_ISTERSENIZ/docs/15_KOD_vision_keypoint_bolumleri.md).
> Bu dosya yalnız CANLI `vision/` hattını anlatır.

> Hedefi kamerada bulan ve yönelimini çıkaran katman + bu modelleri üreten
> otomatik etiketleme ve eğitim hattı. Fonksiyon bazında açıklama.

---

## Hattın tamamı

```
  geometry.py           ← projeksiyon matematiği (etiketlemenin temeli)
       │
       ├──► capture_dataset.py          → detection verisi (bbox)
       ├──► capture_negatives.py        → hard-negative (kendi pervanemiz)
       └──► capture_runway_negatives.py → hard-negative (pist/zemin)
                    │
                    ▼   vision/datasets/ (depoda değil)
                    │
              train_yolo.py
                    │
                    ▼
           models/avci_yolo.pt
                    │
                    ▼
              detector.py
                    │
                    ▼
          detection_state.py  ← thread-safe KARE köprüsü
                    ▼
          control/guidance/  (bbox_ibvs uyanır)
```

> **Manuel etiketleme yoktur.** Simülasyonda hem kameranın hem hedefin konumu
> bilindiği için bbox projeksiyonla otomatik hesaplanır.

---
---

# `geometry.py`

**Yol:** `vision/geometry.py` · **288 satır**
**Rol:** Kamera projeksiyonu + Talon'un 3B geometrisi. Otomatik etiketlemenin
temeli, güdüm çekirdeğinin tek görü bağımlılığı.

---

## Kamera parametreleri

```python
IMG_W, IMG_H = 640, 480
HFOV_RAD = 2.18166                                  # 125°
FX = FY = (IMG_W / 2.0) / math.tan(HFOV_RAD / 2.0)  # ≈ 166.6
CX, CY = IMG_W / 2.0, IMG_H / 2.0                   # 320, 240

CAM_OFFSET_POS = np.array([0.10, 0.0, 0.05])        # base_link'e göre kamera
CAM_TILT_RAD = -0.4363                              # kamera pitch (negatif = yukarı)
```

**`FX` türetimi:** Pinhole modelde `FX = (genişlik/2) / tan(HFOV/2)`.
`320 / tan(62.5°) ≈ 166.6`.

> **Bu değerler SDF ile senkron tutulmalıdır.** `iris_cam/model.sdf`'te kamera
> değiştirilirse burası da değişmeli — yoksa etiketler ve güdüm birbirinden kayar.

> **İşaret dikkati:** `CAM_TILT_RAD` **negatif** (SDF pitch konvansiyonu:
> negatif = yukarı). `guidance_core.Cfg.KAMERA_TILT_DEG` ise **+25** (pozitif =
> yukarı). İki dosya farklı konvansiyon kullanır; aynı fiziksel açıyı anlatırlar.

---

## Rotasyon yardımcıları

```python
def rot_rpy(roll, pitch, yaw):
    """Gazebo pose RPY → rotasyon matrisi: Rz(yaw)·Ry(pitch)·Rx(roll)."""
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)
```

`_rot_x`, `_rot_y`, `_rot_z` standart sağ-el rotasyon matrisleri.

---

## `_stl_vertices(path)`

```python
with open(path, "rb") as f:
    f.seek(80)                    # 80 baytlık başlığı atla
    ...
```

Binary STL formatı: 80 bayt başlık + 4 bayt üçgen sayısı + her üçgen için
50 bayt (normal + 3 vertex + 2 bayt attribute).

**Dönüş:** `(N, 3)` vertex dizisi.

---

## `talon_aabb()` / `talon_box_corners()`

`talon_aabb()` tüm collision mesh'lerin birleşik eksen-hizalı sınır kutusunu
verir; `talon_box_corners()` bunun 8 köşesini.

> ⚠️ Docstring uyarıyor: *"açılı araçta gevşek bbox verir — target_bbox artık
> talon_vertices() kullanır."* Bu iki fonksiyon geriye dönük uyumluluk için
> duruyor.

---

## `talon_vertices()`

```python
"""Tüm collision mesh vertex'leri (base_link frame, (N,3)), cache'li.
8-köşe AABB yerine bunları projekte etmek SIKI bbox verir: araç hangi açıda
olursa olsun..."""
```

**Neden AABB değil:** AABB'nin 8 köşesini projekte etmek, 45° açıyla duran bir
araçta gerçek silüetten **çok daha büyük** bir kutu verir. Tüm vertex'leri
projekte edip min/max almak sıkı sonuç üretir.

Cache'li — mesh dosyaları her çağrıda okunmaz.

---

## `camera_world_pose(iris_pos, iris_rpy)`

**Dönüş:** `(kamera_world_konum, R_cam)`

Drone'un world pozundan kameranın world konumu ve rotasyonunu hesaplar:
`CAM_OFFSET_POS` gövde çerçevesinde uygulanır, `CAM_TILT_RAD` rotasyona eklenir.

---

## `project_points(P_world, cam_pos, R_cam)`

**Dönüş:** `(u, v, front)` — piksel koordinatları ve "kameranın önünde mi" maskesi

Standart pinhole projeksiyonu:
1. World noktalarını kamera çerçevesine taşı
2. `u = CX + FX·x/z`, `v = CY + FY·y/z`
3. `front = z > 0`

`front` maskesi kritik: kameranın **arkasındaki** noktalar da matematiksel
olarak bir piksele düşer (ters işaretli), maskesiz etiketler saçmalar.

---

## `target_bbox(target_pos, target_rpy, iris_pos, iris_rpy, margin_px)`

**Dönüş:** `(x1, y1, x2, y2)` piksel veya görünmüyorsa `None`

```
1. talon_vertices() → hedefin gövde çerçevesindeki tüm vertex'leri
2. target_rpy ile döndür, target_pos ile ötele → world
3. project_points → piksel + front maskesi
4. Görünür noktaların min/max'ı = bbox
5. margin_px kadar genişlet
```

---

## `bbox_to_yolo(bbox)`

```python
(x1, y1, x2, y2) piksel → (cx, cy, w, h) normalize [0..1]
```

---
---

# `detector.py`

**Yol:** `vision/detector.py` · **67 satır**
**Rol:** YOLO tabanlı Talon tespiti (bounding box).

## Yapılandırma

```python
_MODEL_PATH = os.environ.get("AVCI_YOLO_MODEL", ".../vision/models/avci_yolo.pt")

# 0.45: val pozitiflerinde min conf 0.48 (0.45 altında pozitif YOK) — zayıf
# pist/zemin FP'lerini keser, gerçek hedef kaybettirmez (2026-07-24 ölçümü).
_CONF_MIN = float(os.environ.get("AVCI_YOLO_CONF", "0.45"))
```

**Eşik gerekçesi ölçüme dayanıyor:** Doğrulama kümesindeki gerçek pozitiflerin
en düşük güveni 0.48'di. 0.45 eşiği bu tabanın hemen altında — yanlış
pozitifleri keser, gerçek hedefi kaçırmaz.

## `load(model_path=None)`

```python
global _model
if _model is None:
    from ultralytics import YOLO
    path = model_path or _MODEL_PATH
    if not os.path.exists(path):
        raise FileNotFoundError(...)
    _model = YOLO(path)
```

**Tembel yükleme (lazy loading):** `ultralytics` importu ve CUDA warmup ~1-2 sn
sürer. Modül import edilirken değil, ilk kullanımda yüklenir.

## `detect_talon(frame_bgr, conf=None)`

**Dönüş:** `{"cx", "cy", "w", "h", "conf", "bbox"}` veya `None`

En yüksek güvenli tespiti döndürür. Çıktı sözleşmesi eski `color_detector.py`
(HSV renk tabanlı) ile **birebir aynı** tutulmuş — bu sayede YOLO'ya geçiş
aşağı akışta hiçbir değişiklik gerektirmedi.

## `draw_overlay(frame_bgr, det)`

Kutu + güven değeri çizer. `det=None` ise kareyi **değiştirmeden** döner —
çağıran tarafta `if det:` kontrolü gerekmez.

---
---

# `detection_state.py`

**Yol:** `vision/detection_state.py` · **57 satır**
**Rol:** Kamera thread'i ile güdüm döngüleri arasındaki thread-safe köprü.

## Detection tarafı — basit kilit

```python
_lock = threading.Lock()
_last_detection = None

def set_detection(det):
    global _last_detection
    with _lock:
        _last_detection = det

def get_detection():
    with _lock:
        return _last_detection
```

# Veri toplama betikleri

## `capture_dataset.py` (247 satır)

**Detection verisi — otomatik etiketli.**

```bash
# Terminal 1 — statik world
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models
gz sim -r sim/gazebo_harmonic/worlds/dataset_capture.sdf
# Terminal 2
python3 -m vision.capture_dataset --count 2000
```

### `class FrameGrabber`

```python
def _cb(self, msg):     # gz-transport callback — kareyi sakla
def snapshot(self):     # en son kareyi döndür
```

### `_set_pose(node, name, pos, rpy)`

`gz set_pose` servisiyle modeli taşır. `_rpy_to_quat` ile RPY → quaternion
(ZYX sırası, Gazebo ile tutarlı).

### `random_camera_pose()`

Kamera için rastgele world konum + rpy üretir.

| Sabit | İşlev |
|-------|-------|
| `CAM_XY_RANGE`, `CAM_Z_RANGE` | Konum aralığı |
| `CAM_ROLL_MAX` | Maksimum yatış |
| `CAM_PITCH_UP/DOWN/HORIZON` | Bakış açısı dağılımı |
| `SKY_FRACTION` | Gökyüzü arka planlı kare oranı |

**`SKY_FRACTION` neden:** Canlı görevde drone hedefe **alttan** bakar (gökyüzü
arka plan). Eğitim verisi de bu dağılımı yansıtmalı, yoksa domain gap oluşur.

### `sample_target_pose(iris_pos, iris_rpy)`

**Dönüş:** `(world_pos, rpy)` veya bulunamazsa `(None, None)`

Verilen kamera pozu için **FOV içinde ve yer üstünde** hedef üretir.

| Sabit | İşlev |
|-------|-------|
| `DIST_EXP` | Mesafe dağılımı üsteli — yakın/uzak karelerin oranı |
| `MIN_BOX_PX` | Çok küçük hedefi ele (etiketlenemez) |
| `MIN_TARGET_ALT` | Hedef yer altına düşmesin |
| `PIX_MARGIN` | Kadraj kenarına yapışık hedefleri ele |
| `ROLL_MAX`, `PITCH_MAX` | Hedefin yönelim aralığı |

### `main()` akışı

```
her kare için:
  1. random_camera_pose()   → camera_rig'i taşı
  2. sample_target_pose()   → mini_talon'u taşı
  3. kısa bekle             (render otursun)
  4. FrameGrabber.snapshot()
  5. geometry.target_bbox() → etiket
  6. görüntü + .txt etiketi kaydet
```

**Uçuş/güdüm gerekmez** — fizik durdurulmuş, modeller ışınlanıyor. Bu yüzden
veri toplama çok hızlıdır.

---

## `capture_negatives.py` (110 satır)

**Hard-negative — kendi pervanemiz.**

**Sorun:** Model kendi aracımızın pervanesini "talon" sanıyordu.

**Çözüm:** Canlı simülasyonda (pervaneler **dönerken**) `/iris_cam/image`'den
kare kaydedilir, YOLO etiketi **boş** bırakılır → ultralytics bunları
"arka plan / hedef yok" örneği sayar.

```bash
# 1) Simülasyonu başlat, iris'i uçur (control.demos.run_drone_square)
# 2) Talon kadrajda DEĞİLKEN:
python3 -m vision.capture_negatives --count 500
# 3) Yeniden eğit (aynı dataset klasörü)
```

> ⚠️ **Kritik:** Toplama sırasında Talon kadrajda **olmamalıdır**. Talon'lu bir
> kare boş etiketle kaydedilirse eğitimi **zehirler** — model gerçek hedefi
> "arka plan" olarak öğrenir.

**Neden `run_drone_square` ile birlikte:** Köşe dönüşlerindeki yatış sırasında
pervaneler kadraja girer — yanlış pozitifin doğduğu tam geometri.

---

## `capture_runway_negatives.py` (224 satır)

**Hard-negative — pist ve zemin.**

**Sorun:** Canlı simde pist başlangıcı **0.57 güvenle** yanlış pozitif veriyordu
(eşik 0.45'in üstünde — yani güdüme giriyordu).

**Çözüm — üç önlem:**

```python
TARGET = ...        # mini_talon 600 m uzağa park edilir → hiçbir karede görünmez
RUNWAY_AIM_X, RUNWAY_AIM_Y      # pist üzerinde rastgele bakış noktası
CAM_XY_X, CAM_XY_Y, CAM_Z       # alçak irtifa kamera konumları
GROUND_VIEW_FRACTION            # karelerin bir kısmı genel zemin/ufuk
```

### `random_negative_pose()`

Docstring: *"Kamera gövdesi için pist-bakışlı (veya genel zemin) rastgele poz
üretir."*

Yanlış pozitifin doğduğu **bakış geometrisi birebir yeniden üretilir** — alçak
irtifa, piste doğru bakış. Rastgele pozlar toplamak yetmezdi; sorunlu görüntüyü
hedefli üretmek gerekiyordu.

### Ayrı partition zorunluluğu

```bash
export GZ_SIM_RESOURCE_PATH=...
DISPLAY=:1 GZ_PARTITION=negcap gz sim -s -r sim/gazebo_harmonic/worlds/dataset_capture.sdf &
GZ_PARTITION=negcap python3 -m vision.capture_runway_negatives --count 400
```

Canlı simülasyon çalışırken topic'ler çakışmasın diye. **Hem `gz sim` hem betik
aynı partition'ı görmelidir.**

---
---

# Eğitim betikleri

## `train_yolo.py` (55 satır)

```python
from ultralytics import YOLO
model = YOLO(args.model)                    # taban model
model.train(data=..., epochs=args.epochs, ...)
shutil.copy(best_weights, "vision/models/avci_yolo.pt")
```

```bash
python3 -m vision.train_yolo --epochs 100
python3 -m vision.train_yolo --model yolo11n.pt --epochs 150   # farklı taban
```

En iyi ağırlığı `vision/models/` altına kopyalar — `detector.py` ve
`detector.py` oradan okur. GPU (torch + CUDA) otomatik kullanılır.

Bulut GPU'sunda eğitim: `docs/COLAB_TRAINING.md`.

---

# Model ve örnek çıktılar

| Yol | İçerik |
|-----|--------|
| `vision/models/avci_yolo.pt` | Eğitilmiş detection ağırlığı (**depoya dahil**) |
| `vision/demo_model_comparison/` | 20 örnek çıkarım: `det_bbox_*` kareleri (normal, `_close_`, `_zoom_` varyantları) |
| `vision/datasets/` | Üretilen eğitim verisi (**depoda değil**) |

`.gitignore`'daki istisna satırları sayesinde modeller depoya girer — depoyu
klonlayan biri veri toplamadan/eğitmeden sistemi çalıştırabilir.
