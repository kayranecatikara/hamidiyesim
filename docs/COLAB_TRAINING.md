# Colab'de YOLO Eğitimi (Talon detection)

Veri toplama yerelde (Gazebo), eğitim Colab'de (bulut GPU). Model çıktısı:
`best.pt` → yerelde `vision/models/avci_yolo.pt` olarak kullanılır.

---

## 1) Yerelde veri topla (Gazebo çalışırken)

```bash
cd ~/projects/avci_sim
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models
gz sim -r sim/gazebo_harmonic/worlds/dataset_capture.sdf     # Terminal 1
```
```bash
cd ~/projects/avci_sim && source /opt/ros/humble/setup.bash   # Terminal 2
python3 -m vision.capture_dataset --count 2000
```

## 1b) (Önerilir) Hard-negative kareler ekle

Model kendi pervanemizi hedef sanmasın diye, CANLI simde (pervaneler dönerken,
Talon kadrajda DEĞİLKEN) etiketsiz kare topla — aynı dataset klasörüne eklenir:

```bash
python3 -m vision.capture_negatives --count 500
```

## 2) Veriyi zip'le

```bash
cd ~/projects/avci_sim
zip -r talon_dataset.zip vision/datasets/talon
# → talon_dataset.zip (Colab'e yüklenecek)
```

## 3) Colab: yeni notebook aç → Runtime > Change runtime type > **GPU** (T4/A100)

Aşağıdaki hücreleri sırayla çalıştır:

**Hücre 1 — ultralytics:**
```python
!pip install ultralytics -q
```

**Hücre 2 — dataset yükle:** (açılan pencereden `talon_dataset.zip` seç)
```python
from google.colab import files
files.upload()
!unzip -q talon_dataset.zip -d /content/
```

**Hücre 3 — dataset.yaml (Colab yolu):**
```python
yaml = """path: /content/vision/datasets/talon
train: images/train
val: images/val
nc: 1
names:
  0: talon
"""
open('/content/talon.yaml', 'w').write(yaml)
```

**Hücre 4 — eğit:** (GPU'da; A100'de batch'i 64 yapabilirsin)
```python
from ultralytics import YOLO
model = YOLO('yolo11n.pt')          # nano; istersen 'yolo11s.pt' (biraz daha güçlü)
model.train(data='/content/talon.yaml', epochs=100, imgsz=640, batch=32)
```

**Hücre 5 — sonuçları gör + modeli indir:**
```python
from IPython.display import Image, display
display(Image('runs/detect/train/results.png'))                 # precision/recall eğrileri
display(Image('runs/detect/train/val_batch0_pred.jpg'))         # örnek tahminler
from google.colab import files
files.download('runs/detect/train/weights/best.pt')             # → avci_yolo.pt olarak kaydet
```

## 4) Yerelde modeli yerine koy

İndirilen `best.pt`'yi projeye taşı:
```bash
mkdir -p ~/projects/avci_sim/vision/models
mv ~/Downloads/best.pt ~/projects/avci_sim/vision/models/avci_yolo.pt
```

---

**Notlar**
- Dataset büyükse (2000+ kare) zip yükleme yavaş olabilir; alternatif: zip'i Google Drive'a
  koyup Colab'de `from google.colab import drive; drive.mount('/content/drive')` ile bağla.
- `results.png`'de mAP/precision/recall yükseliyorsa eğitim iyi. Düşükse epoch/veri artır.
- Model `vision/models/avci_yolo.pt` yolunda; detector'ın gcs'ye entegrasyonu ayrı adım.
