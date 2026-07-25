# PROJE HARİTASI — Hangi Kod Nerede?

> Bu dosya, kod mimarisi yeniden düzenlemesinin (eski `control/` dağıtıldı)
> haritasıdır. Her klasörün görevi ve her taşınan dosyanın eski→yeni yolu
> aşağıdadır.

## Klasörler (ilk bakışta)

| Klasör | Görevi |
|---|---|
| `gcs/` | Web Yer Kontrol İstasyonu — FastAPI sunucusu + web arayüzü. Çalıştır: `python3 -m gcs.gcs_server` |
| `vehicle_control/` | Araç kontrol katmanı — MAVLink ile iris (drone) ve Talon (uçak) komutları |
| `guidance/` | Avcı güdüm algoritmaları, türe göre klasörlü: `gps_gudum/` (yaklaşma/chase/strike), `gorsel_gudum/` (IBVS çekirdek+döngü+adaptörler), `hibrit_gudum/` (supervisor), `ortak/` (common) |
| `vision/` | Çalışma zamanı görüntü işleme — YOLO/pose tespiti, geometri, tespit durumu + eğitilmiş modeller |
| `vision_training/` | Veri toplama ve model eğitimi — dataset capture + YOLO eğitim betikleri |
| `demos/` | Tek başına çalışan demo/test uçuş betikleri (`python3 -m demos.run_...`) |
| `diagnostics/` | Tanı/kalibrasyon araçları (ARM reddi teşhisi, ivmeölçer bias düzeltme) |
| `legacy/` | Eski dönem araçları (PX4 ve Gazebo Classic kalıntıları — aktif akışta kullanılmaz) |
| `sim/` | Gazebo Harmonic world + araç modelleri + ArduPilot parametre dosyaları |
| `tests/` | Birim testleri |
| `scripts/` | Başlatma/kurulum kabuk betikleri |
| `docs/` | Rehber dokümanlar (SIMULASYON_CALISTIRMA.md, GUIDANCE_ROADMAP.md...) |
| `tools/` | Mission Planner kurulumu (git dışı) |

## Eski → Yeni Dosya Haritası

| Eski yol | Yeni yol | Ne iş yapar |
|---|---|---|
| `control/gcs_server.py` | `gcs/gcs_server.py` | Web GCS: FastAPI + kamera akışı + görev API |
| `control/gcs_ui/` | `gcs/gcs_ui/` | Web arayüz (HTML/CSS/JS) |
| `control/mav_common.py` | `vehicle_control/mavlink_common.py` | Ortak MAVLink altyapısı (bağlantı, mod, arm) |
| `control/drone_functions.py` | `vehicle_control/drone_functions.py` | iris (ArduCopter) kontrol fonksiyonları |
| `control/plane_functions.py` | `vehicle_control/plane_functions.py` | Talon (ArduPlane) kontrol fonksiyonları |
| `control/plane_patterns.py` | `vehicle_control/plane_patterns.py` | Talon kalkış + kare/daire desen uçuşları |
| `control/guidance/*` | `guidance/<tür>/...` | Güdüm paketi — gps_gudum/, gorsel_gudum/, hibrit_gudum/, ortak/ altına dağıtıldı |
| `control/arm_diag.py` | `diagnostics/arm_diag.py` | Plane ARM reddi teşhis aracı |
| `control/fix_accel_bias.py` | `diagnostics/fix_accel_bias.py` | "High Accelerometer Bias" ARM engeli düzeltici |
| `control/px4_shell.py` | `legacy/px4_shell.py` | PX4 dönemi SITL shell erişimi (artık kullanılmıyor) |
| `control/cessna_pose_relay.py` | `legacy/cessna_pose_relay.py` | Gazebo Classic dönemi hedef pose aynalama |
| `control/harmonic_pose_relay.py` | `legacy/harmonic_pose_relay.py` | Harmonic'te kukla-hedef pose aynalama (gerçek fizik akışıyla gereksizleşti) |
| `control/run_*.py` (8 adet) | `demos/run_*.py` | Demo/test uçuş betikleri |
| `vision/capture_dataset.py` | `vision_training/capture_dataset.py` | YOLO dataset toplama (Gazebo'dan) |
| `vision/capture_pose_dataset.py` | `vision_training/capture_pose_dataset.py` | Pose (keypoint) dataset toplama |
| `vision/capture_negatives.py` | `vision_training/capture_negatives.py` | Negatif örnek toplama |
| `vision/capture_runway_negatives.py` | `vision_training/capture_runway_negatives.py` | Pist/zemin hard-negative toplama |
| `vision/train_yolo.py` | `vision_training/train_yolo.py` | YOLO detection eğitimi |
| `vision/train_yolo_pose.py` | `vision_training/train_yolo_pose.py` | YOLO pose eğitimi |
| `vision/demo_model_comparison/` | `vision_training/demo_model_comparison/` | Model karşılaştırma örnek görüntüleri |

`vision/` içinde kalanlar (çalışma zamanında GCS'nin kullandıkları):
`detector.py`, `pose_detector.py`, `geometry.py`, `detection_state.py`, `models/`.

## Değişen Çalıştırma Komutu

- ESKİ: `python3 -m control.gcs_server`
- YENİ: `python3 -m gcs.gcs_server`

(Gazebo, SITL ve Mission Planner komutları değişmedi. Temizlik komutundaki
süreç deseni de `gcs.gcs_server` olarak güncellendi.)
