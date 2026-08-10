# `vision/` — KEYPOINT DÖNEMİ BÖLÜMLERİ (arşiv)

⛔ Bunlar canlı sistemi anlatmaz. 2026-08-10'da `dokumantasyon/15_KOD_vision.md`
içinden buraya alındı: anlattıkları dosya ve fonksiyonların tamamı bu arşiv
klasöründe (`vision/pose_detector.py`, `vision/capture_pose_dataset.py`,
`vision/geometry_keypoints.py`, `vision/train_yolo_pose.py`).

Canlı `vision/geometry.py` yalnız kamera iç parametreleri, 3B kutu ve
projeksiyon içerir; `vision/detection_state.py` yalnız KARE köprüsüdür.

---

## Keypoint tanımı

```python
KEYPOINT_NAMES = ["burun", "kuyruk", "sol_kanat", "sag_kanat", "sol_vtail", "sag_vtail"]
KEYPOINT_FLIP_IDX = [0, 1, 3, 2, 5, 4]   # yatay flip augmentasyonunda sol↔sağ eşleşmesi
```

**`KEYPOINT_FLIP_IDX` kritik:** Eğitimde yatay çevirme augmentasyonu
yapıldığında sol kanat ile sağ kanat etiketleri de takas edilmelidir. Yapılmazsa
model sağ/sol ayrımını **hiç öğrenemez** — flip edilmiş karelerde "sol kanat"
etiketi sağ kanadın üzerinde durur, model çelişkili veri görür.

Burun (0) ve kuyruk (1) simetrik olduğu için yerinde kalır.

---

---

## `talon_keypoints()`

**Dönüş:** `(6, 3)` — `base_link` çerçevesinde 3B keypoint konumları.

```python
def part(name):
    return _stl_vertices(os.path.join(_MESH_DIR, f"mini_talon_{name}_collision.stl"))

fus = part("fuselage")
lw, rw = part("left_wing"), part("right_wing")
lt, rt = part("left_tail"), part("right_tail")

_TALON_KPTS = np.array([
    fus[fus[:, 0].argmax()],                     # burun      → gövdenin en +X'i
    fus[fus[:, 0].argmin()],                     # kuyruk     → en −X'i
    lw[lw[:, 1].argmax()],                       # sol kanat  → en +Y'si
    rw[rw[:, 1].argmin()],                       # sağ kanat  → en −Y'si
    lt[(np.abs(lt[:, 1]) + lt[:, 2]).argmax()],  # sol V-tail ucu
    rt[(np.abs(rt[:, 1]) + rt[:, 2]).argmax()],  # sağ V-tail ucu
])
```

### Neden `argmax`/`argmin` — el ile koordinat değil

Docstring: *"Parça collision STL'lerinin uç noktalarından türetilir (mesh
değişirse kendiliğinden günceller)."*

Keypoint konumları elle yazılsaydı, mesh güncellendiğinde **sessizce yanlış**
olurlardı. Bu yöntemde geometri kaynaktan okunur.

**V-tail ucu formülü:** `|Y| + Z` maksimize edilir — V-kuyruk hem yana hem
yukarı uzanır, tek eksende uç nokta aramak yanlış vertex seçerdi.

---

## `talon_triangles()`

```python
_TALON_TRIS = talon_vertices().reshape(-1, 3, 3).astype(float)
```

Docstring: *"STL vertex dizilimi üçgen-sıralı olduğundan reshape yeterli."*
STL dosyasında vertex'ler zaten üçgen üçgen sıralıdır.

---

## `occluded_mask(cam_pos, target_pos, R_t)`

**Dönüş:** `(6,)` bool — hangi keypoint'ler aracın kendi gövdesi arkasında kalıyor.

**Yöntem:** Möller–Trumbore ışın-üçgen kesişim algoritması, **vektörize**.

```python
cam_b = R_t.T @ (cam_pos - target_pos)      # kamerayı hedefin gövde çerçevesine taşı
kpts = talon_keypoints()
tris = talon_triangles()
v0 = tris[:, 0]
e1 = tris[:, 1] - v0                         # üçgen kenar vektörleri
e2 = tris[:, 2] - v0
s = cam_b - v0
q = np.cross(s, e1)

for k, kp in enumerate(kpts):
    d = kp - cam_b
    t_max = np.linalg.norm(d)                # keypoint'e olan mesafe
    d = d / t_max                            # ışın yönü (birim)
    p = np.cross(d, e2)
    det = (e1 * p).sum(1)
    ok = np.abs(det) > 1e-12                 # ışına paralel üçgenleri ele
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    u = (s * p).sum(1) * inv                 # baryzentrik koordinatlar
    v = (d * q).sum(1) * inv
    t = (e2 * q).sum(1) * inv                # ışın boyunca mesafe
    hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6) & (t < t_max - _OCCL_EPS)
    out[k] = bool(hit.any())
```

### Kesişim koşulları

| Koşul | Anlamı |
|-------|--------|
| `u >= 0, v >= 0, u+v <= 1` | Kesişim noktası üçgenin **içinde** |
| `t > 1e-6` | Kesişim kameranın **önünde** (arkada değil) |
| `t < t_max - _OCCL_EPS` | Kesişim keypoint'e **varmadan önce** |

### `_OCCL_EPS = 0.02` neden var

Kod yorumu:
> *"Işın kpt'ye varmadan bu mesafeden fazla önce mesh'e çarparsa örtülü say;
> kpt'nin kendi yüzey üçgenlerinin sahte 'çarpma' vermemesi için tampon."*

Keypoint mesh'in **yüzeyinde** durur. Tampon olmadan kendi üçgeni "engel"
sayılır ve her keypoint hep örtülü görünür.

### Neden gövde çerçevesinde
Hesap hedefin `base_link` çerçevesinde yapılır — mesh verisi zaten o
çerçevededir. Alternatif (mesh'i her karede dünyaya taşımak) çok daha pahalı
olurdu.

---

---

## `target_keypoints(target_pos, target_rpy, iris_pos, iris_rpy)`

**Dönüş:** `(6, 3)` — her satır `[u, v, vis]`

```python
R_t = rot_rpy(*target_rpy)
world = target_pos + talon_keypoints() @ R_t.T      # gövde → world
cam_pos, R_cam = camera_world_pose(iris_pos, iris_rpy)
u, v, front = project_points(world, cam_pos, R_cam)
occl = occluded_mask(cam_pos, target_pos, R_t)

vis = front & ~occl & (u >= 0) & (u < IMG_W) & (v >= 0) & (v < IMG_H)
out[vis, 0] = u[vis]
out[vis, 1] = v[vis]
out[vis, 2] = 2                                      # görünür

---

# `pose_detector.py`

**Yol:** `vision/pose_detector.py` · **120 satır**
**Rol:** YOLO-pose ile 6 keypoint tespiti. **Güdüm çekirdeğinin girdisi.**

## `detect_pose(frame_bgr, conf=None, imgsz=None)`

**Dönüş:**
```python
{"cx", "cy", "w", "h", "conf", "bbox",
 "kpts": [(u, v, conf), ...]}      # KEYPOINT_NAMES sırasında 6 eleman
```

Detection modelinin **yanında** çalışır, yerine geçmez:
- `detector.py` → "hedef var mı, nerede"
- `pose_detector.py` → "hangi yöne bakıyor"

## `detect_pose_in_bbox(frame_bgr, bbox, conf=None, imgsz=None)`

```python
_CROP_FACTOR, _CROP_MIN, _CROP_MAX      # krop penceresi büyütme
```

Detection kutusunun çevresini büyütüp **yalnız o pencerede** pose çalıştırır,
sonucu tam kare koordinatına geri çevirir.

**Neden:** Hedef uzaktayken kadrajın küçük bir bölümünü kaplar. 640×480 karede
40×30 piksellik bir hedefin keypoint'lerini çıkarmak zordur. Kroplayıp
büyütmek etkin çözünürlüğü artırır.

## `draw_overlay(frame_bgr, pose)`

```python
_KPT_COLORS, _SKELETON     # keypoint renkleri ve bağlantı çizgileri
```

Docstring: *"Bbox'ı çizmez — o detection overlay'inin işi; ikisi üst üste
çalışır."* İki overlay aynı kare üzerinde çalıştığı için bbox iki kez
çizilmesin diye.

---
---

---

## Pose tarafı — olay güdümlü

```python
_pose_cond = threading.Condition(_lock)     # AYNI kilidi paylaşır
_last_pose = None
_pose_seq = 0
_pose_stamp = None      # kare header.stamp (s) — dt bundan hesaplanır
_pose_wall = None       # karenin geliş duvar anı — gecikme ölçümü
```

### `set_pose_detection(pose, stamp=None, wall_recv=None)`

```python
with _pose_cond:
    _last_pose = pose
    _pose_stamp = stamp
    _pose_wall = wall_recv
    _pose_seq += 1
    _pose_cond.notify_all()
```

Docstring: *"Her KARE için çağrılır (pose None olsa bile) — bekleyenler
uyandırılır."*

**Neden `pose=None` de sinyallenir:** Tüketici "bu karede tespit yok" bilgisini
de almalı. Kayıp sayacı (`kayip_sayaci`) ve kör dalış kararı buna dayanır.
Sadece başarılı tespitte sinyal verilseydi, temas kaybı **hiç fark edilmezdi** —
`visual_lead` sonsuza kadar bekler, supervisor GPS'e dönemezdi.

### `wait_new_pose(son_seq, timeout=0.5)`

```python
with _pose_cond:
    if _pose_seq == son_seq:
        _pose_cond.wait(timeout)
    if _pose_seq == son_seq:
        return None                      # timeout — yeni kare gelmedi
    return {"seq": _pose_seq, "pose": _last_pose,
            "stamp": _pose_stamp, "wall_recv": _pose_wall}
```

**Çift kontrol deseni:** `wait()` öncesi ve sonrası. Öncesi — kare zaten
gelmişse boşuna beklemez. Sonrası — timeout ile gerçek uyanmayı ayırt eder.

### Neden `Condition`, `Queue` değil

Tüketici **en son** kareyi ister, birikmiş kuyruğu değil. `Queue` kullanılsaydı
yavaş bir tüketici bayat kareleri sırayla işlerdi — güdümde ölümcül.

### `seq` numarasının işlevi
Tüketici hangi kareyi işlediğini bilir: aynı kareyi iki kez işlemez, kaçırdığını
anlar.

---
---

---

## `capture_pose_dataset.py` (193 satır)

**Pose verisi — otomatik etiketli.** Detection hattıyla **aynı mekanik**
(aynı world, aynı poz üreteci — `capture_dataset`'ten import eder).

### `kpts_to_yolo(kpts)`

```
(6,3) piksel [u,v,vis] → normalize YOLO-pose string parçası
```

**Etiket formatı:**
```
0  cx cy w h   x1 y1 v1   x2 y2 v2   ...   x6 y6 v6      (hepsi normalize)
```
`v=2` görünür, `v=0` kadraj dışı/örtülü (o durumda `x=y=0`).

### `MIN_POSE_BOX_PX`

Detection'dan **daha büyük** bir eşik. Keypoint'ler bbox'tan daha çok
çözünürlük ister — 20 piksellik bir kutuda 6 keypoint ayırt edilemez.

### `draw_debug(frame, bb, kpts)` / `--debug-overlay`

Etiketleri görselleştirip kaydeder — etiketleme doğruluğunu **gözle** kontrol
etmek için. Otomatik etiketlemede en büyük risk sessiz hatadır; bu bayrak
onu görünür kılar.

```bash
python3 -m vision.capture_pose_dataset --count 5000 --debug-overlay
```

---

---

## `target_keypoints` — `vis` kuralı (artık parça)

# görünmeyenler (0, 0, 0) kalır — YOLO-pose kuralı
```

### `vis` dört koşulun kesişimi
1. `front` — kameranın önünde
2. `~occl` — **öz-örtülü değil**
3. `u ∈ [0, 640)` — yatay kadraj içinde
4. `v ∈ [0, 480)` — dikey kadraj içinde

### Neden örtülü nokta etikete konmaz

Docstring: *"örtülü nokta etikete KONMAZ (eğitimi bozmasın)."*

Talon yandan görüldüğünde uzak kanat ucu gövdenin arkasında kalır. O keypoint'i
`vis=2` etiketlemek modele *"gövdenin içinden keypoint kestir"* diye öğretirdi.
Model bunu öğrenemez, sadece gürültü ekler.

`vis=0` olan noktalar için `u=v=0` yazılır — YOLO-pose formatının kuralı.

---
