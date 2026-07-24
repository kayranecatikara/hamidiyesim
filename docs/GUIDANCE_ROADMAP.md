# AVCI SİM — Güdüm Yol Haritası: GPS + Görsel (IBVS) Hibrit Müdahale

> Durum: **PLAN** (uygulama onayı bekliyor). Bu belge kodun mevcut analizine
> dayanır; her faz onaylandıkça uygulanır.

## 1. Bağlam ve Hedef

Amaç: avcı drone'un hedef hava aracını **iki aşamalı** güderek vurması:

1. **GPS güdümü** — başlangıçta hedefin telemetri (GPS) konumuna göre yaklaş.
   İki aracın da tam konum+hız+rotasyonu bilindiği için güçlü, öngörülü bir
   algoritma kurulabilir (mevcut chase + strike).
2. **Görsel güdüm** — drone hedefle **görsel temas** sağlayınca (kamera + YOLO
   tespiti) GPS hattından çıkıp **IBVS** (Image-Based Visual Servoing) hattına
   geç. Kısmi bilgi (yalnız bbox) olduğu için farklı, ayrı bir algoritma.

İki hat **ayrı dosyalarda** olacak — hem düzen, hem de matematiksel doğaları
farklı olduğu için. Bir **supervisor** hangi hattın aktif olduğunu ve geçişleri
yönetir. Ek fayda: GPS karıştırma (jamming) altında görsel hatta düşerek
dayanıklılık (mevcut `_apply_gps_noise` senaryosu tam bunu test eder).

## 2. Mevcut Durum (kod analizi — 3 keşif ajanı bulgusu)

- **chase + strike %100 GPS girdili.** `chase_algorithm.py` ve
  `strike_algorithm.py` gerçek piksele/bbox'a hiç dokunmuyor.
- **chase'deki "kamera" matematiği gerçek değil** — GPS geometrisinden türetilen
  *simüle pinhole* (hedefi kadrajda %7.5 oranında tutacak ideal duruş mesafesini
  hesaplar: `compute_optimal_lock_distance`, `CAM_FOV_DEG=125`, `CAM_TILT_DEG=25`).
- **strike zaten saf GPS** (Proportional Navigation) — refactor'da neredeyse
  değişmeden geçer.
- **Saf görsel güdüm KODDA YOK** — sıfırdan yazılacak.
- **Detection güdüme beslenmiyor:** `set_detection()` her karede yazılıyor ama
  `get_detection()` hiç okunmuyor (`vision/detection_state.py` thread-safe, hazır,
  ama "write-only"). IBVS'in doğal tüketim noktası burası.
- **En temiz entegrasyon dikişi:** chase hedef konumunu yalnız `get_plane()`
  callback'inden alıyor (`gcs_server.py:651-654`). Supervisor'ı oraya koyunca
  `run_chase` imzası hiç değişmez.
- **Ortam:** sistem `python3`'te `torch 2.5.1+cu121` (CUDA) ve `cv2` **kurulu**;
  tek eksik **`ultralytics`**. Model ağırlığı yok (eğitilecek).
- **Kamera intrinsics** (640×480, HFOV 125°): `fx=fy≈166.6`, `cx=320`, `cy=240`,
  **25° yukarı tilt** (piksel merkezi fiziksel ufkun 25° üstü).
- **Portlar (gerçek):** hedef=14550, iris=14541 (koddaki bazı yorumlar bayat).

## 3. Hedef Mimari

```
control/guidance/
├── guidance_common.py      # ortak yardımcılar (EMA, clamp, vec3, accel-limit, ts)
│                           #   — şu an chase+strike'ta BİREBİR kopyalı, buraya çıkar
├── gps_guidance.py         # SAF GPS: mevcut chase (takip/lock) + strike (terminal PN)
├── visual_guidance.py      # YENİ — IBVS: bbox → piksel hatası → drone hız setpoint
└── guidance_supervisor.py  # state machine: GPS ↔ görsel geçiş + jam/fallback

vision/
├── detector.py             # YOLO wrapper — detect() → {cx,cy,w,h,conf,bbox}
├── autolabel.py            # Gazebo ground-truth → otomatik bbox etiketleme
├── train_yolo.py           # Ultralytics eğitim scripti
├── datasets/  (gitignore)  # üretilen eğitim verisi
└── models/    (gitignore)  # eğitilen ağırlık (avci_yolo.pt)
```

Setpoint sözleşmesi korunur: GPS hattı pos+vel+yaw (`_TYPEMASK_POS_VEL_YAW`),
görsel hat sadece vel+yaw (strike'ın `_TYPEMASK_VEL_YAW` deseni).

## 4. Fazlar

### Faz 0 — Refactor: GPS hattını izole et + supervisor iskeleti
- `control/guidance/` paketi. `guidance_common.py`'ye ortak yardımcıları çıkar
  (chase/strike'taki kopyalanmış `_clamp`, `_vec3_len`, EMA sınıfları, accel-limit).
- `chase_algorithm.py` → `guidance/gps_guidance.py` (mantık aynı; pinhole "çerçeveleme"
  GPS hattının parçası olarak KALIR — gerçek kamera değil zaten).
- `strike_algorithm.py` → `guidance/gps_guidance.py` içine terminal faz olarak taşı.
- `guidance_supervisor.py`: iskelet — şimdilik yalnız GPS hattını çağırır, geçiş
  kancaları boş. `gcs_server._chase_thread` supervisor'ı çağıracak (minimal değişiklik).
- **Regresyon hedefi:** mevcut GPS chase davranışı birebir korunur.
- Yan düzeltme: `LOCK_DIST_MIN_MULT` kullanılmıyor (kod↔yorum tutarsız) — netleştir.

### Faz 1 — GPS güdümü iyileştir (Faz 0 sonrası, görsel tarafla paralel)
- Tam state'i sömür: hedef hız/rotasyonu zaten var; PN'i sağlamlaştır, lock bandını
  düzelt, tahmin ufkunu hedef manevrasına göre uyarla.
- Bu faz "daha iyi GPS vuruşu" — görsel taraf gelişirken bağımsız ilerler.

### Faz 2 — Detection modeli (YOLO + Gazebo oto-etiketleme)
- `vision/autolabel.py`: drone uçarken her karede **ground-truth** üret →
  iris kamera pozu (pose + 25° tilt) + intrinsics + hedefin 3B konumu/boyutu
  (`mini_talon` gövde/kanat) → 8 köşeyi görüntüye projekte et → 2B bbox → YOLO
  formatı (normalize cx,cy,w,h). Hedef pozu **gz-transport `/world/avci/pose/info`**
  (en doğru ground-truth) ya da telemetriden.
- **Domain randomization:** ışık, hedef açısı/mesafesi, arka plan, (mevcut) video
  paraziti — gerçeğe/gürültüye transfer için.
- `vision/train_yolo.py`: Ultralytics YOLO **nano** (kurulumda en güncel stabil
  nano ağırlık sabitlenir), `dataset.yaml` + `model.train()`.
- `vision/detector.py`: YOLO wrapper; `detect(frame) → {cx,cy,w,h,conf,bbox}`
  **aynı sözlük** — böylece `set_detection`/`draw_overlay`/downstream değişmez;
  tek fark `conf` artık gerçek olasılık.
- `gcs_server.py:701`: `detect_cessna` yerine detector (env `AVCI_DETECTOR=yolo|hsv`
  ile seçilebilir; HSV yedek kalır). `requirements.txt += ultralytics`.
- Pürüz: `process_iris_frame`'de det'e **frame boyutu** eklenmeli (piksel→açı
  normalizasyonu için).

### Faz 3 — Görsel güdüm (IBVS lead pursuit v2) — UYGULANDI (2026-07-23)
Eski bbox-merkezleme IBVS'i (`visual_guidance.py`) ve supervisor iskeleti KALDIRILDI;
yerine pose modelinin keypoint'lerinden **menzil bağımsız lead pursuit**:
- `guidance_core.py` (platformdan bağımsız, Adım 1-8): `d = burun−kuyruk` gövde
  projeksiyonu a, kanat projeksiyonu b → `olcek = sqrt(a² + (0.633·b)²)` (menzil
  sadeleşir) → `yandanlik = a/olcek` → `lead = atan(K_LEAD·guven·kalite·yandanlik_f)`.
  Kaydırma YÖN VEKTÖRÜ uzayında (FOV 125°'de piksel uzayı yanlış), kamera→gövde
  dönüşümü 25° tilt ile tek fonksiyonda, LOS yükselti düzeltmesi
  (`olcek_ham/sqrt(1+sin²eps)`, alttan yaklaşmada %23'e varan lead kaybını önler).
  Sağlamlık: burun/kuyruk flip koruması, kalite kapısı (6→22.5m / 14→9.6m px),
  deadband, kanat-ucu güven kapısı, çözümsüzlük işareti (çarpım>0.95).
- `adapter_copter.py` (Adım 9, AKTİF): u_govde → dünya-NED, `v = V_KAPANMA·u_dunya`
  + ivme rampası (4 m/s² — üstünde burun eğimi kamerayı yere baktırır) + slew-limitli
  yaw. Komut yolu `common.send_velocity` (SET_POSITION_TARGET_LOCAL_NED hız+yaw).
  **SET_ATTITUDE_TARGET kullanılmaz** — multirotorda attitude komutu yanlış araç
  (burun yukarı = tırmanış değil geri yavaşlama); gaz/eğim ArduCopter'ın işi.
- `adapter_fixedwing.py` (STUB): sabit kanatta SET_ATTITUDE_TARGET + yatarak dönüş.
  **Yedek gaz politikası (uygulanınca):** hover gazını SABİT VARSAYMA, GUIDED'da
  havada tutup çıkış gazını ölçerek HOVER_GAZ belirle;
  `gaz = clip(HOVER_GAZ/cos(egim) + irtifa_pid, 0.15, 0.85)`; `egim>60°` → komut
  reddet + WARN. ArduPlane'de gaz TECS'indir; attitude hedefiyle TECS etkileşimi
  uygulanmadan önce ayrıca incelenecek. ArduCopter'ın SET_ATTITUDE_TARGET gaz
  alanını yorumlayışı sürüme/moda göre değişir — varsayma, SITL'de ölç.
- `visual_lead.py`: OLAY GÜDÜMLÜ döngü (sabit Hz yok, kare geldikçe;
  `detection_state.wait_new_pose`). dt = kare header.stamp farkı (duvar saati
  DEĞİL); bayat kare kapısı (gecikme>0.12s → komut yok); GUIDED kontrolü; her
  kare CSV log (`logs/visual_lead_*.csv`): ölçüm zinciri + komutlar + eps/duzeltme
  + menzil_kestirim (SADECE log, güdüme girmez) + menzil_gercek + pitch_body +
  kameranın dünyaya göre bakışı.
- Kabul kriterleri: `tests/test_visual_lead.py` T1-T21 (menzil bağımsızlık, tilt
  telafisi, görüş zarfı, yükselti düzeltmesi sınır testleri, adaptör sözleşmeleri).

### Faz 4 — Supervisor: geçiş + fallback — UYGULANDI (2026-07-23)
- `control/guidance/supervisor.py` `run_hybrid`:
  `GPS (gps_approach) → (görsel kilit) → VISUAL (visual_lead) → (temas kaybı) → GPS ...`
  stop_chase'e kadar döner.
- **GPS→görsel:** KILIT_N=10 ardışık pose karesi (conf ≥ 0.5) **VE**
  (handoff ≤40 m **VEYA** GPS DROPOUT). Menzil kapısının nedeni: görsel fazın
  kapanma hızı sabit (V_KAPANMA) — uzaktan erken geçilirse hızlı hedefe yetişilemez;
  jam/DROPOUT'ta menzil bilinemez, görsel temas tek başına yeter.
- **görsel→GPS fallback:** KAYIP_M=20 ardışık pose'suz kare veya kare akışının
  >1 s durması → `run_visual_lead` "kayip" döner, GPS fazı yeniden başlar.
- `gcs_server`: `start_chase` VARSAYILAN olarak hibriti çalıştırır
  (`AVCI_HYBRID=off` → saf GPS, `AVCI_GPS_LAW=v2` → eski chase). `/api/chase_status`
  `supervisor` alanı döndürür: `{faz: GPS|VISUAL|DURDU, gecis_sayisi, kilit_sayac}`.
  `start_visual` endpoint'i yalnız-görsel test için duruyor.
- Test: `tests/test_visual_lead.py` T22 — geçiş zinciri
  GPS→VISUAL→(kayıp)→GPS→VISUAL→durdur sahte fazlarla doğrulanır.

## 5. Kritik Tasarım Kararları (öneriler)

| Konu | Öneri | Gerekçe |
|------|-------|---------|
| IBVS derinlik | **Hibrit** (bbox oranı + son GPS prior) | Saf bbox metrik derinlik veremez |
| Geçiş tetiği | Görsel temas **VEYA** GPS bozulması | Hem senaryo hem jamming dayanıklılığı |
| Model dağıtımı | `.pt` **gitignore** + eğitim/indirme scripti | Repo şişmesin (36M'de kalsın) |
| Oto-etiket kaynağı | gz `/world/avci/pose/info` | En doğru ground-truth |
| YOLO sınıfı | nano | Gerçek-zaman (30 Hz kamera), CUDA hazır |

## 6. Doğrulama (her faz)

- **Faz 0:** refactor sonrası GPS chase davranışı regresyonsuz — canlı SITL+Gazebo
  chase testi (mevcut `start_chase`).
- **Faz 2:** oto-etiketleri overlay ile görsel doğrula (bbox hedefe oturuyor mu);
  eğitilen modelin ayrık test karelerinde precision/recall.
- **Faz 3:** IBVS döngüsü hedefi kadraj merkezine çekiyor mu (sim'de piksel hatası
  → 0); menzil kestirimi GPS gerçeğiyle kıyas.
- **Faz 4:** GPS→görsel geçiş anı; jamming senaryosunda görsele düşüş; tespit kaybında
  GPS'e geri dönüş; uçtan uca müdahale.

## 7. Sıra ve Bağımlılıklar

Faz 0 (temel) → Faz 2 (detection, kritik yol, en uzun) → Faz 3 (IBVS, Faz 2'ye
bağlı) → Faz 4 (supervisor, hepsini birleştirir). Faz 1 (GPS iyileştirme) Faz 0
sonrası bağımsız ilerler.
