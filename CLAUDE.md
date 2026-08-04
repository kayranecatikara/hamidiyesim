# AVCI SİM — Claude Çalışma Kuralları

Bu dosya Claude Code'un her oturumda okuduğu proje talimatıdır.

---

## 1. TEMEL KURAL: Simülasyonu Claude ÇALIŞTIRMAZ

**Claude simülasyon testi başlatmaz, uçuş yapmaz, senaryo tetiklemez.**

| Kim | Ne yapar |
|-----|----------|
| **Kullanıcı** | Gazebo'yu, SITL'leri, gcs_server'ı başlatır. Uçuşları uçar. Senaryo ve chase butonlarına basar. |
| **Claude** | `logs/` altındaki dosyaları **okur**, analiz eder, bulguyu raporlar, kod yazar. |

Claude'un çalıştırmasına izin verilen tek şey: **salt-okuma analiz scriptleri**
(`tools/analiz/*.py`) ve testler (`tests/test_visual_lead.py`,
`tests/test_gps_guidance.py`). Bunlar simülasyona bağlanmaz.

**Claude asla şunları çalıştırmaz:** `scripts/start_harmonic.sh`, `gz sim`,
`sim_vehicle.py`, `python3 -m control.gcs_server`, `curl` ile `/api/command/...`
uçları, `control/demos/run_*.py` demo scriptleri.

Bir uçuş verisi gerekiyorsa Claude **kullanıcıdan ister** ve tam olarak ne yapması
gerektiğini yazar (hangi senaryo, ne kadar süre, hangi ayarlar).

---

## 2. Kullanıcı: sistemi nasıl çalıştırırım

Tam komut listesi: `docs/SIMULASYON_CALISTIRMA.md`. Kısa yol:

### Temizlik (yeniden başlatmadan önce)
```bash
pkill -9 -f 'gz sim|sim_vehicle|mavproxy|arducopter|arduplane|control.gcs_server'; sleep 3
```

### Terminal 1 — Gazebo + iki SITL (tek komut)
```bash
cd ~/Masaüstü/avci_sim
bash scripts/start_harmonic.sh
```
Bu script Gazebo Harmonic'i, ArduCopter'ı (iris, sysid 5) ve ArduPlane'i
(Talon, sysid 2) doğru sırayla başlatır. Durdurmak için:
`bash scripts/start_harmonic.sh stop`

**NVIDIA notu:** Bu makine Optimus (Intel iGPU + GTX 1650 Ti) ve
`prime-select` = `on-demand`. Script PRIME offload değişkenlerini kendisi set eder;
o olmadan Gazebo Intel iGPU'da render eder. Kapatmak için:
`GZ_NVIDIA=0 bash scripts/start_harmonic.sh`

Doğrulama: `nvidia-smi` çıktısında `gz sim server` ve `gz sim gui` görünmeli.
`gz sim server` GPU'da olmalı — kamera sensörü render'ı odur, YOLO'yu o besler.

### Terminal 2 — GCS Server
```bash
cd ~/Masaüstü/avci_sim
source /opt/ros/humble/setup.bash
export AVCI_GZ_CAMERA=1
fuser -k 8000/tcp || true      # port boşsa fuser 1 döner — || true zinciri korur
python3 -m control.gcs_server
```
Web arayüz: <http://localhost:8000>

> **Tek satırda yapıştıracaksan `&&` KULLANMA.** `fuser -k 8000/tcp` port boşken
> çıkış kodu 1 döndürür ve `&&` zinciri orada sessizce kopar — hiçbir çıktı
> vermeden hiçbir şey çalışmaz. Ayırıcı olarak `;` kullan:
> ```bash
> export AVCI_GZ_CAMERA=1; fuser -k 8000/tcp; python3 -m control.gcs_server
> ```

> **Çıktıyı dosyaya yönlendiriyorsan `python3 -u` kullan.** Yönlendirilmiş
> stdout blok-tamponludur; `print()` ile yazılan `[GCS]`/`[TRUTH]`/`[SUPERVISOR]`
> satırları tamponda kalır, yalnız uvicorn'un `logging` çıktısı görünür ve
> sistem çökmüş gibi görünür. Terminalde doğrudan çalıştırırken gerekmez.

> **Süreç öldürürken köşeli parantez hilesi:** `pkill -f 'control.gcs_server'`
> komutun KENDİ kabuğunu da eşleştirip öldürür. `pkill -f '[c]ontrol.gcs_server'`
> yaz (depodaki scriptler de bunu yapıyor).

### Terminal 3 (opsiyonel) — Mission Planner
```bash
bash ~/Masaüstü/avci_sim/scripts/start_mission_planner.sh
```

### Hazır olduğunu nasıl anlarım
`gcs_server` logunda şunlar görünmeli:
```
[GCS] YOLO detector hazır (avci_yolo.pt)
[GCS] YOLO pose hazır (avci_pose.pt)
[GCS] gz-transport kamera dinleniyor (/iris_cam/image, Harmonic)
[TRUTH] Gazebo ground truth dinleniyor (/world/avci/dynamic_pose/info; ...)
[GCS] Plane sys_id=2 comp_id=1 tespit edildi.
[FRAME] Plane→iris NED çerçeve ofseti kalibre edildi: ...
```
`copter_harmonic.log` içinde `AP: Frame: QUAD/X` olmalı (`UNSUPPORTED` değil).

**`[TRUTH]` satırı yoksa** doğruluk kolonları boş kalır ve analiz scriptleri
"ground truth yalnız n/m karede var" uyarısı verir — uçuş yine de geçerlidir,
ama Adım 4b'nin asıl sorularını cevaplayamayız. Bu satırı görmeden chase başlatma.

---

## 3. Test uçuşu protokolü

Analiz verisi üreten uçuş şöyle yapılır:

1. Sistemi yukarıdaki gibi başlat, `[FRAME] ... kalibre edildi` satırını bekle.
2. Web arayüzden **hedef senaryosunu** başlat. Talon kalkar ve deseni uçar.
3. Talon **stabil irtifaya oturana kadar bekle** (~20-30 sn). Erken chase
   başlatmak GPS fazını hedefin tırmanışına kilitler, veri temsili olmaz.
4. **Chase başlat.** iris kalkar, GPS fazı devreye girer.
5. `[SUPERVISOR] ✓ GÖRSEL TEMAS` satırını gördüğünde görsel faz devraldı demektir.
6. `[SUPERVISOR] ✓✓ HEDEF VURULDU` ya da ıska olana kadar bırak. Iska olursa
   sistem GPS fazına döner ve tekrar dener — 2-3 devir döngüsü iyi veri verir.
7. Chase'i durdur. **Her chase bir `logs/gps_guidance_<tarih>_<saat>.csv` (GPS
   fazı) ve her görsel faz bir `logs/visual_lead_<tarih>_<saat>.csv` üretir.**

### İstenen uçuş seti (Adım 4)

| # | Senaryo | Neden |
|---|---------|-------|
| 1 | `square` (kare) | Düz uçuş + keskin dönüşler — temel durum |
| 2 | `circle` (daire) | Sürekli sabit yandanlık — lead yasasının asıl testi |
| 3 | `aggressive` | Rastgele manevra — dayanıklılık, flip riski |

İsteğe bağlı 4. uçuş: arayüzden **GPS jamming** kaydırıcısını %100'e al →
DROPOUT fallback yolu (görsel temas tek başına devir kapısı) test edilir.

Uçuş bittiğinde Claude'a sadece şunu söylemen yeter: *"3 uçuş yaptım, loglar hazır."*
Claude `logs/` altını kendi okur.

---

## 4. Log haritası

| Dosya | İçeriği |
|---|---|
| `logs/visual_lead_*.csv` | **Asıl analiz verisi.** Görsel faz her karede bir satır: keypoint ölçümleri, ölçek, yandanlık, lead açısı, komut, menzil + **Gazebo doğruluk kolonları** (`*_gercek`). Bir görsel faz = bir dosya. |
| `logs/gps_guidance_*.csv` | GPS fazı 20 Hz: hedef kestirimi, istasyon noktası, hız komutu, kadraj hatası (`kadraj_yaw_deg` / `kadraj_elev_deg`). Merkezleme başarısının ölçüsü. |
| `logs/gcs_server.log` | Kamera/YOLO durumu, faz geçişleri (`[SUPERVISOR]`), GPS fazı periyodik durumu (`[GPS]`), ground truth (`[TRUTH]`), vuruş/ıska (`[LEAD]`). |
| `logs/copter_harmonic.log` | ArduCopter SITL — frame tipi, PreArm hataları, mod geçişleri. |
| `logs/plane_harmonic.log` | ArduPlane SITL — hedefin kalkışı, mod geçişleri. |
| `logs/gz_harmonic.log` | Gazebo — plugin el sıkışması, render hataları, SDF uyarıları. |

`visual_lead_*.csv` içindeki `durum` kolonu kritik: `ok` / `cozumsuz` /
`kanat_dusuk` / `kpt_dusuk` / `tespit_yok` / `bayat` / `mod_hata` /
`attitude_yok` / `kor_dalis` / `vuruldu`.

`gps_guidance_*.csv` içindeki `durum`: `WARMUP` / `TAKIP` / `KILIT` (d_h < 20 m,
görsel devir bandı) / `DROPOUT` (hedef telemetrisi 3 s donuk — GPS jamming yolu).

Doğruluk kolonları (`vision/dogruluk.py`, yalnız `AVCI_TRUTH` açıkken dolar):
`menzil_gercek_gz_m`, `aspect_gercek_deg`, `yandanlik_gercek`,
`eksen_aci_hata_deg`, `burun_kuyruk_takas`, `kpt_hata_px_*`. Referans Gazebo'nun
tam pozudur (`control/gz_truth.py`), MAVLink telemetrisi değil.

---

## 5. Gazebo'ya cisim bırakırsam bbox içine alır mı?

**HAYIR.**

Her iki model de **tek sınıflı**: `avci_yolo.pt` ve `avci_pose.pt` →
sınıflar `{0: 'talon'}`. Modeller yalnız Mini Talon render'larıyla eğitildi
(`vision/capture_dataset.py`, `vision/capture_pose_dataset.py`).

Gazebo'ya küp, küre, başka bir araç modeli bıraktığında:
- Kamera görüntüsünde **görünür** (Gazebo render'ı normal çalışır),
- Ama **tespit edilmez, bbox çizilmez, keypoint atanmaz**,
- Güdüm için o cisim **yoktur** — iris onu ne hedef sayar ne engel.

Tek istisna: Talon'a benzeyen bir siluet yanlış-pozitif üretebilir.
`vision/detector.py` içindeki `_CONF_MIN = 0.45` eşiği tam da bunu kesmek için
ölçülerek seçilmiş (val pozitiflerinde min güven 0.48; 0.45 altında gerçek
pozitif yok).

**Başka bir cismi tespit ettirmek istersen** tek yol: o cismi de içeren yeni bir
veri seti toplayıp modeli çok sınıflı yeniden eğitmek
(`vision/capture_dataset.py` + `vision/train_yolo.py`). Ayrı bir iş kalemi.

---

## 6. Mimari — hızlı harita

```
Gazebo Harmonic (fizik + kamera render, NVIDIA)
  ├─ FDM 9002 ↔ ArduCopter SITL (iris, sysid 5)   → MAVLink 14541
  ├─ FDM 9012 ↔ ArduPlane  SITL (Talon, sysid 2)  → MAVLink 14542
  └─ gz-transport /iris_cam/image, /talon_cam/image
                              ↓
                      gcs_server (:8000)
                        YOLO tespit → pose → güdüm → MAVLink komut
```

Güdüm zinciri:
```
supervisor.run_hybrid          faz geçişi (GPS ↔ görsel)
  ├─ gps_guidance.py           GPS fazı: kadraj-merkezleme istasyonu, alttan bakış
  │                            (hedefi kameranın ORTASINA getirir — vuruş değil)
  └─ visual_lead.py            görsel faz: kareye kilitli döngü
       └─ guidance_core.py     IBVS lead pursuit (saf hesap)
            └─ adapter_copter  → NED hız + yaw komutu

control/gz_truth.py            Gazebo tam pozu → vision/dogruluk.py → CSV
                               (SADECE analiz; güdüme girmez)
```

Devir kapısı: 10 ardışık güvenli pose karesi **VE** (menzil ≤ 20 m **VEYA**
GPS DROPOUT). Temas 20 kare koparsa GPS'e dönülür.

Ayrıntılı güdüm belgesi: `docs/GUIDANCE.md`. Kod gezintisi: `dokumantasyon/`
(HTML site: `dokumantasyon/site/index.html`).

---

## 7. Geliştirici uçları (arayüzde yok)

- `POST /api/command/iris/start_visual` — GPS fazını atlayıp **doğrudan** IBVS
  görsel güdümünü çalıştırır. Lead yasasını izole test etmenin tek yolu.
  `stop_visual` ile durdurulur. *(Kullanıcı çalıştırır, Claude değil.)*
- `GET /api/debug/telem` — anlık telemetri + MAVLink istatistikleri.
- `GET /api/telemetry/pnp` — menzil/poz kestirim telemetrisi.

Ortam değişkenleriyle ayar:

| Değişken | Nerede | Ne yapar |
|---|---|---|
| `AVCI_IBVS_K_LEAD` | `guidance_core.Cfg` | lead kazancı (≈ hedef_hızı/bizim_hız) |
| `AVCI_IBVS_V_KAPANMA` | `guidance_core.Cfg` | görsel faz kapanma hızı (25 m/s) |
| `AVCI_IBVS_TERMINAL_MENZIL` / `_SURE` | `guidance_core.Cfg` | kör dalış eşiği/süresi |
| `AVCI_IBVS_VURUS_MENZIL` | `guidance_core.Cfg` | vuruş sayılan mesafe (3 m) |
| `AVCI_IBVS_COALT_MENZIL` / `AVCI_IBVS_PN_SURE` | `guidance_core.Cfg` | terminal co-altitude, dikey PN |
| `AVCI_POSE_KPT_CONF` | `guidance_core.Cfg` | keypoint güven eşiği |
| `AVCI_HYBRID_GATE_MENZIL` | `supervisor.SupCfg` | görsel devir menzili (20 m) |
| `AVCI_GPS_RANGE` | `gps_guidance.Cfg` | GPS istasyon slant menzili (11 m) |
| `AVCI_HYBRID=off` | `gcs_server` | hibridi kapat, saf GPS fazı koş |
| `AVCI_TRUTH=off` | `gcs_server` | Gazebo ground truth kapat (doğruluk kolonları boşalır) |
| `AVCI_LEAD_LOG_DIR` | her iki güdüm | CSV çıktı dizinini taşı (testler kullanır) |

---

## 8. Kod kuralları

- **Dil:** Yorumlar, docstring'ler, log mesajları ve değişken adları **Türkçe**.
  Mevcut kodun üslubu bu; koru.
- **`guidance_core.py` saftır:** IO yok, MAVLink yok, sadece hesap. Birim test
  edilebilirliği (`tests/test_visual_lead.py`) buna bağlı — bozma.
- **Parametre gerekçesi:** Her `Cfg` değerinin yanında **neden o değer** olduğu
  yazılı olmalı; mümkünse hangi ölçüme dayandığı (tarih + gözlem).
- **ArduPilot parametre adı yazarken ÖNCE firmware'de var olduğunu doğrula.**
  ArduPilot bilinmeyen adı **sessizce yok sayar** — hata vermez, log'a yazmaz.
  `sim/ardupilot_params/avci_plane.parm`'daki `TRIM_ARSPD_CM` / `ARSPD_FBW_MIN` /
  `ARSPD_FBW_MAX` bu yüzden uzun süre hiçbir şey yapmadı (yeni adlar
  `AIRSPEED_CRUISE` / `AIRSPEED_MIN` / `AIRSPEED_MAX`, ve **birim cm/s değil m/s**).
  Doğrulama: `grep '"AD"' ~/Masaüstü/ardupilot/ArduPlane/Parameters.cpp` — grup
  parametreleri (`TKOFF_ALT` → `mode_takeoff.cpp`, `ARSPD_USE` → `AP_Airspeed`)
  ayrı dosyada. Kesin yol: SITL'in kaydettiği `~/Masaüstü/ardupilot/mav_2_1.parm`
  dökümünde ada bak — orada yoksa uçakta da yoktur.
- **Ground-truth güdüme girmez:** `menzil_gercek*`, `*_gercek` kolonları yalnız
  log/analiz içindir. Güdüm hesabına sızdırmak simülasyonu kendi kendini
  kandıran hale getirir.
- **Test:** `python3 -m tests.test_visual_lead` ve `python3 -m tests.test_gps_guidance`
  — pytest DEĞİL, kendi koşucuları var (sonda `SONUÇ: n/n geçti` yazar). İkisi de
  CSV'lerini geçici dizine yazar, `logs/` kirlenmez (`AVCI_LEAD_LOG_DIR`).
  **Uyarı:** `run_visual_lead` imzasını değiştirirsen `tests/test_visual_lead.py`
  içindeki `fake_visual*` stub'larını da güncelle — uyumsuz imza supervisor
  thread'inde sessizce ölür, T22/T23 gizemli biçimde düşer.

---

## 9. Yürüyen iş planı

Tam plan: `/home/melike/.claude/plans/gleaming-chasing-charm.md`

- [x] **Adım 1** — Bu dosya (`CLAUDE.md`)
- [x] **Adım 2** — `control/gz_truth.py`: Gazebo tam pozundan gerçek-referans
- [x] **Adım 3** — `vision/dogruluk.py` + `visual_lead.py` CSV doğruluk kolonları
      (sentetik doğrulama: T30-T33, menzil sapması < %2)
- [x] **Adım 5a** — `tools/analiz/` scriptleri yazıldı (sentetik veriyle sınandı)
- [x] **Adım 7** — Ölü kod temizliği: `gps_chase.py`, `gps_strike.py`,
      `adapter_fixedwing.py` + strike endpoint'leri + `AVCI_GPS_LAW` dalı silindi
      (531 satır; `guidance/` 1873 → 1342 satır)
- [x] **Adım 4** — 1. tur test uçuşu (`square`, 2 devir, 0/2 vuruş)
- [x] **Adım 5b** — `docs/GUDUM_DOGRULUK_ANALIZI.md` — hipotez çürüdü, gerçek
      arıza: hedef devirden ~0.6 s sonra kadrajdan çıkıyor
- [~] **Adım 6a** — Düzeltmelerimiz (devir hız sürekliliği, LOS kapısı) 2026-08-01
      merge'inde **düştü**: ekip `gps_approach.py`'yi silip `gps_guidance.py`
      yazdı, `visual_lead.py`'ye dikey PN + terminal co-altitude ekledi. Aynı
      arızanın (hedef kadrajdan çıkıyor) farklı bir çözümü — uçuşta sınanmadı.
      Eski hal `git stash@{0}`'da duruyor.
- [x] **Merge** — `origin/main` 15 commit alındı (2026-08-01): GPS güdümü yeniden
      inşa, demolar `control/demos/`'a, telemetri 4→25 Hz, `dokumantasyon/` sitesi
- [x] **Doğruluk yeniden bağlandı** — `gz_truth` → `dogruluk` → yeni
      `visual_lead.py` CSV'si (34/34 test geçiyor)
- [x] **Arıza zinciri çözüldü** (2026-08-01) — tam kayıt:
      `docs/GUDUM_ARIZA_ZINCIRI_20260801.md`. Altı arıza, hepsi ölçümle:
      `start_harmonic.sh` SITL'i hiç başlatmıyordu · Gazebo iGPU'da render
      ediyordu · `V_MAX=28` doygunluk patolojisi (gerçekleşen/komut 0.24) ·
      senaryolar FBWA'da irtifa tutmuyordu · `avci_plane.parm`'daki hız
      parametreleri ölü isimdi · hedefin hız talebi airframe'in üstündeydi.
      Sonuç: avcı 18.0 m/s sabit (oran 1.00), hedef 58 m'de ±0.4 m.
- [x] **Adım 4b** — 2. tur test uçuşları yapıldı (2026-08-04, beş uçuş, `circle`)
- [x] **Arıza 9-16** (2026-08-04) — tam kayıt: `docs/GUDUM_ARIZA_ZINCIRI_20260804.md`
      Sekiz arıza daha, hepsi ölçümle: **ArduCopter parametrelerinin HİÇBİRİ
      uygulanmıyordu** (adlar+birimler değişmişti; `WP_ACC` varsayılanı 2.5 m/s²
      avcıyı 14.6° yatışa hapsediyordu) · saf takip kapalı desende yakınsamıyordu
      (dönüş tuzağı) · hedefin gaz slider'ı yakalanamaz banda eşleniyordu ·
      devir yandan yapılıyordu (LOS açısal hızı 155 °/s, araç 90) · yaw tavanı
      sabit olamaz (kerterize bağlandı) · görsel faz avcıyı hedefin 41 m üstüne
      çıkarıyordu · 6 m/s sürekli iniş girdap halkasına sokuyordu ·
      `ATC_ANGLE_MAX` çalışma noktası olmuştu, güvenlik tavanı değil.
      Sonuç: menzil 82 m platosundan medyan 16.6 m'ye, karelerin %55'i 20 m
      altında; 6 görsel devir (en uzunu 6.3 s); 240 s çöküşsüz.
- [ ] **Vuruş** ← ŞU AN BURADAYIZ. Uçuş ayakta kalıyor ama hedef vurulmuyor.
      Ölçülen kalan arıza: `|kadraj_yaw|` p90 43.5° — devir anında kadraj hâlâ
      bozulabiliyor, menzil ara sıra 70 m'ye geri açılıyor.
- [ ] **Adım 6b** — GPS fazının kuyruğa yakınsaması (aspect 90° → ~180°);
      yeni `gps_guidance.py` bunu KADEME 1 geometrisiyle zaten hedefliyor —
      uçuş verisiyle doğrulanacak
- [ ] **Adım 8** — `visual_lead.py` terminal mantığı sadeleştirme
- [ ] **Adım 9** — `guidance/` + `vision/` belgeleme, README güncelleme
      (ekibin `dokumantasyon/` + `docs/GUIDANCE.md` çalışmasıyla birleştir)

### Analiz scriptleri (uçuştan sonra)
```bash
python3 -m tools.analiz.analiz_gps        # GPS fazı: menzil neden kapanmıyor
python3 -m tools.analiz.analiz_devir      # devir geometrisi — ANA SORU
python3 -m tools.analiz.analiz_yonelim    # yönelim doğruluğu
python3 -m tools.analiz.analiz_menzil     # menzil doğruluğu
python3 -m tools.analiz.parm_dogrula      # ÖNCE BUNU: parametreler UYGULANDI MI
```
`analiz_gps` `gps_guidance_*.csv` okur, diğerleri `visual_lead_*.csv`.
Beş şeyi ayrıştırır: yasa doğru yeri mi gösteriyor / komut uygulanıyor mu /
avcı hedeften hızlı mı / dikey kanal / kadraj. Doygunluk patolojisini,
dönüş tuzağını ve EKF↔konum türevi uyuşmazlığını otomatik teşhis eder.

> **`parm_dogrula` her uçuştan önce çalıştırılmalı.** ArduPilot bilinmeyen
> parametre adını sessizce yok sayar; bu tuzağa İKİ KEZ düşüldü (2026-08-01
> ArduPlane hız adları, 2026-08-04 ArduCopter hareket adlarının TAMAMI — avcı
> `WP_ACC`'nin varsayılanı olan 2.5 m/s² ivme tavanıyla uçuyordu). Script her
> `.parm` adını SITL'in kendi dökümüyle karşılaştırır, ölü adı ve uygulanmamış
> değeri raporlar, olası yeni adı önerir.

> **Kod değiştirdikten sonra YALNIZ `gcs_server`'ı yeniden başlatmak yetmez.**
> Hedef uçak havada kalır, senaryo komutuyla uçmaya devam eder ve dakikalar
> içinde kilometrelerce uzaklaşır; GPS fazı hiç WARMUP'tan çıkamaz ve uçuş
> sessizce geçersiz olur (2026-08-04 uçuş 2). Tam temizlik:
> `bash scripts/start_harmonic.sh stop` → hepsini yeniden başlat.

### Gözetimsiz uçuş koşucusu
```bash
python3 -m tools.analiz.otonom_test square 500 240   # senaryo, gaz, chase süresi
```
**Bu script simülasyona KOMUT GÖNDERİR** — §1'in istisnası, yalnız kullanıcı
açıkça gözetimsiz test istediğinde çalıştırılır. Gaz ayarlar, senaryoyu
başlatır, hedefin irtifası **oturana kadar bekler**, hızını ölçer, chase'i
başlatır, menzil/kilit/faz izler, durdurur ve arızayı sebebiyle raporlar.
Argümansız çağrılırsa `logs/` içindeki EN YENİ CSV'yi alır; dosya yolu/glob da
verilebilir. Salt okuma — Claude bunları çalıştırabilir.

### Açık analiz soruları (Adım 5'te cevaplanacak)

1. Pose modelinin yönelim kestirimi gerçekle ne kadar uyuşuyor?
2. `menzil_kestirim_m` gerçek menzile göre ne kadar sapıyor, hangi bantta güvenilir?
3. Burun/kuyruk keypoint'leri ne sıklıkla takas oluyor (lead'i ters çevirir)?
4. GPS fazı görsel fazı gerçekten `yandanlik ≈ 0` kör noktasında mı devrediyor?
