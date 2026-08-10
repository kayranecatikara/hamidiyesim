# POSE'A GERİ DÖNMEK İSTERSENİZ

**Tarih:** 2026-08-06 · **Sökülmeden önceki commit:** `51c088b`

Görsel güdüm **YOLO-pose modelini bıraktı**, artık yalnız **detection kutusu
(bbox)** ile çalışıyor. Bu klasör, pose dalına ait *her şeyi* — kod, model,
araç, belge ve sökülmeden önceki güdüm hattının anlık görüntüsünü — bir arada
tutar. Amaç: karar geri alınırsa sıfırdan yazmadan dönebilmek.

---

## 1) Neden bırakıldı

Karar kullanıcının. Ölçümler kararı destekliyor ama tek başına zorunlu kılmıyor:

| ölçüm | sonuç |
|---|---|
| Algı kusursuz yapıldığında (GT modu) isabet | **değişmedi** — darboğaz algı değil |
| Pose yaw sapması (3 m üstü, medyan) | 1.04° — model zaten doğruydu |
| Menzil vekili olarak yayılım | pose ölçeği ±%55 · **bbox genişliği ±%45** |
| Kanat ucu keypoint'i | karelerin %99.8'inde iki kanadı da gövde merkezine koyuyordu (ortalamaya kaçma) |
| Pose'un kattığı lead | medyan 2.39° · ort 6.18° · p90 18.64° |

Yani pose'un tek gerçek katkısı **lead açısıydı** ve o katkı da başka bir
sinyalden üretilebiliyordu (aşağıda).

## 2) Lead nereye gitti — kaybolmadı, kaynağı değişti

Pose'un **şekil-lead'i** (yandanlık × K_LEAD) yerine adaptörde
**azimut-oranı lead'i** var: `control/guidance/adapter_copter.py` → `_yatay_pn`.
Dikey kanalda zaten çalışan PN'in yatay eşi; LOS azimutunun değişim oranıyla
orantılı öne nişan alır ve keypoint istemez.

Aynı 10 183 kare üzerinde yeniden oynatılarak ölçüldü:

| | medyan | ort | p90 |
|---|---:|---:|---:|
| şekil-lead'i (pose) | 2.39° | 6.18° | 18.64° |
| azimut-oranı lead'i | 1.09° | 6.08° | 20.00° |

Saf takibe dönmek isterseniz tek değişken yeter: `AVCI_IBVS_PN_YATAY_SURE=0`.

## 3) Bu klasörde ne var

```
vision/pose_detector.py          YOLO-pose çıkarımı (kropta) + overlay
vision/train_yolo_pose.py        eğitim betiği
vision/capture_pose_dataset.py   Gazebo'dan keypoint'li veri toplama
vision/krop.py                   eğitim ve çıkarımın ORTAK krop mantığı
models/avci_pose.pt              eğitilmiş ağırlık
tools/pose_kanat_olc.py          kanat keypoint hatası ölçümü
tools/pose_sirali_egit.py        sıralı eğitim zinciri
tools/pose_ucus_zinciri.sh       veri→eğitim→uçuş zinciri
tools/pose_vs_gt_viz.py          pose vs ground-truth görselleştirme
tools/etiket_piksel_sirala.py    simetrik keypoint çiftlerini piksel sırasına çevirme
docs/POSE_VERI_VE_EGITIM.md      veri toplama + eğitim belgesi
gudum_anlik_goruntu/             SÖKÜLMEDEN ÖNCEKİ güdüm hattı (aşağıda)
```

### `gudum_anlik_goruntu/` — dönüşün asıl anahtarı

Bunlar **çalışan kopya değil, referans**. `51c088b` commit'indeki hâlleriyle
duruyorlar; bugünkü dosyalarla yan yana koyup farkı okuyabilirsiniz.

| dosya | içinde ne vardı |
|---|---|
| `guidance_core.py` | keypoint → gövde ekseni + yandanlık → şekil-lead'i (Adım 1-8), flip koruması, `K_LEAD`/`MAX_LEAD_DEG`/`FILTRE_TAU_S`, `KPT_CONF_MIN` |
| `adapter_copter.py` | `_yatay_pn` YOKTU (lead çekirdekten geliyordu) |
| `visual_lead.py` | **pose vs gerçek kıyas blokları** (aşağıdaki tablo) ve o CSV sütunları |
| `supervisor.py` | kilit sayacı `pose["conf"]` okuyordu |
| `detection_state.py` | `set_pose_detection` / `wait_new_pose` köprüsü |
| `test_visual_lead.py` | T1-T21 pose testleri + T53b manevra körlüğü + T60 PnP + T61 keypoint |

### Pose ↔ gerçek kıyası nerede yapılıyordu

Ayrı bir analiz aracında değil, **uçuş sırasında** `visual_lead.py`'nin kare
döngüsünde, `core.process(...)` çağrısından hemen sonra. Üç blok vardı; üçü de
yalnız CSV'ye yazıyordu, güdüme girmiyordu:

| blok | `gudum_anlik_goruntu/visual_lead.py` satır | gerçek kaynak | pose kaynağı | CSV sütunları |
|---|---|---|---|---|
| Keypoint konum kıyası | 582-620 | `geo.target_keypoints(...)` | `pose["kpts"]` | `kpt_hata_*`, `kpt_hata_ort`, `kpt_gercek_px`, `kpt_pose_px` |
| 3B rotasyon (PnP) | 622-642 | `gt_olcum["hedef_rpy"]` | `geo.pose_rpy_cozum(...)` | `gt_roll/pitch/yaw_deg`, `pose_roll/pitch/yaw_deg`, `*_sapma_deg`, `pnp_kpt` |
| Yandanlık + gövde ekseni | 644-658 | `gt_olcum["yandanlik"]`, `gt["d_birim"]` | `res["yandanlik_ham"]`, `res["d_birim"]` | `gt_yandanlik`, `gt_d_aci_deg`, `pose_d_aci_deg`, `yandanlik_sapma`, `d_aci_sapma_deg` |

Kritik ayrıntı: `gt_olcum = gt if gt is not None else (get_gt() if get_gt else None)`
— **GT modu kapalıyken bile** her karede ölçülüyordu, böylece pose ile gerçek
aynı kare üzerinde yan yana loglanabiliyordu.

## 4) Bilerek TAŞINMAYANLAR

Bunlar ana ağaçta kaldı; arşivdeki araçlar geri getirildiğinde çalışsınlar diye:

- `vision/geometry.py` → `talon_keypoints`, `target_keypoints`, `occluded_mask`,
  `KEYPOINT_NAMES`, `KEYPOINT_FLIP_IDX`, `pose_rpy_cozum`, `rot_gt_goruntu`.
  Bunlar saf geometri; model değil. `pose_rpy_cozum` ve `rot_gt_goruntu` artık
  canlı yolda **hiç çağrılmıyor**.
- `tools/gudum_rapor.py` (2026-08-10'da buraya taşındı) → pose sütunlarını hâlâ okuyabiliyor, yani **eski
  logları** açabiliyorsunuz. Yeni loglarda o bölümler boş kalır.
- `docs/COLAB_TRAINING.md` → detection eğitimi (pose'a özgü değil).

## 5) Geri dönmek için — adım adım

1. `git checkout 51c088b -- control/guidance/ vision/detection_state.py tests/test_visual_lead.py`
   (ya da `gudum_anlik_goruntu/` içindekileri elle yerine koyun).
2. `POSEA_GERI_DONMEK_ISTERSENIZ/vision/*` → `vision/`,
   `models/avci_pose.pt` → `vision/models/`, `tools/*` → `tools/`,
   `docs/POSE_VERI_VE_EGITIM.md` → `docs/`.
3. `control/gcs_server.py`: pose yükleme bloğunu (`AVCI_POSE`), krop çıkarımını,
   `_pose_detector.draw_overlay` çağrısını ve `set_pose_detection(pose, ...)`
   köprüsünü geri koyun. Bugünkü karşılıkları `set_frame_detection(det, ...)`
   ve `wait_new_frame`.
4. `scripts/gcs.sh`: `AVCI_POSE` değişkenini ve A0-A4 adımlarını geri getirin.
5. `PYTHONPATH=. python3 tests/test_visual_lead.py` — arşivdeki test dosyası
   T1-T21 pose testlerini içerir, hepsi geçmeli.

**⚠ Kısmi dönüş yapmayın.** Pose'u geri açıp lead'i azimut-oranından almaya
devam ederseniz lead İKİ KEZ binmiş olur (`guidance_core` + `_yatay_pn`).
Pose'a dönerken `AVCI_IBVS_PN_YATAY_SURE=0` yapın ya da `_yatay_pn` çağrısını
kaldırın.

---

## DURUM.md'den taşınan ölçümler (2026-08-10)

Aşağıdaki iki bölüm DURUM.md'de "TARİHSEL" etiketiyle duruyordu; canlı sistemi
anlatmadığı için buraya alındı. Pose kaldırma kararının gerekçesi bunlardır.

### Pose modeli ASLINDA İYİYDİ (2026-08-05) — TARİHSEL

> ⚠ Aşağıdaki iki bölüm **pose dönemine** aittir ve artık canlı sistemi
> anlatmaz (pose 2026-08-06'da kaldırıldı). Kararın gerekçesi olarak
> saklanıyor. Araçlar: `POSEA_GERI_DONMEK_ISTERSENIZ/`

Uzun süre "kötü pose modeli" varsayıldı. Ölçüldü — yanlış:

| pose modu, hedef önde, menzil > 3 m | değer |
|---|---|
| |yaw sapması| medyanı | **1.04°** |
| p90 | 4.71° |
| örneklem | 1885 kare |

3 m'nin altında sapma patlıyor (medyan 83°) ama bu **model hatası değil**:
hedef kadrajı taşırıyor, nişan vektörü dikeye yaklaşıyor ve azimut
tanımsızlaşıyor (`guidance_core` bunu `azimut_kalite` ile zaten söndürüyor).

Grafik: `python3 POSEA_GERI_DONMEK_ISTERSENIZ/tools/pose_vs_gt_viz.py`

### Algı darboğaz DEĞİL — ve GT modu neden daha kötü (TARİHSEL)

`AVCI_GT_ROT=on` güdümün algı girdisini Gazebo'nun gerçek pozuna çevirir
(teşhis modu, gerçek donanımda uçurulamaz). Kusursuz algıyla isabet **artmadı**.

Sebebi ölçüldü — güdümün fiilen çalıştığı menzil:

| | medyan | p90 | > 15 m kare oranı |
|---|---|---|---|
| **pose modu** | **5.1 m** | 7.4 m | ~%0 |
| **GT modu** | 17.3 m | 84.0 m | **%42** |

Pose modelinin "uzakta göremiyor" olması bir kusur değil, **doğru faz sınırını
çizen bir filtre**: yaklaşmayı GPS fazı yapar, görsel faz yalnız son metrelerde
devreye girer. GT modunda algı hiç kopmadığı için görsel faz 84 m'ye kadar
devrede kalıyor ve yaklaşmayı da o üstleniyor — ama görsel faz bunun için
tasarlanmadı (sabit `V_KAPANMA`, istasyon tutmaz, hedef hızına uyum sağlamaz).

Yan kanıt: `kalite` medyanı pose modunda 1.00, GT modunda 0.36.
