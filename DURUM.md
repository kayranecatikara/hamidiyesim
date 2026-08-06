# DURUM — sistem şu an nasıl çalışıyor

Bu belge **mevcut hâli** anlatır: hangi ayarlar aktif, ne ölçüldü, hangi fikir
çürütüldü. **Yapılacak iş buraya yazılmaz** → [TODO.md](TODO.md).
GPS güdümünün kararlı referans hâli → [KARARLI_HAL.md](KARARLI_HAL.md).
Çalıştırma komutları → `docs/SIMULASYON_CALISTIRMA.md`.

---

## 1. Bir bakışta

Avcı drone (ArduCopter quad) hedef İHA'yı (Talon, ArduPlane) hava-havada
kovalayıp **fiziksel temasla** vurur. Güdüm iki fazlı, aralarında bir
denetleyici (supervisor) geçiş yapar:

```
GPS fazı (gps_guidance)          görsel faz (visual_lead)
hedefin GPS telemetrisiyle   →   kameradan DETECTION KUTUSUYLA
istasyona oturur, kamerayı       saf takip + azimut-oranı lead'i
hedefe merkezler                 ile terminal hücum
        ↑                                  │
        └── temas koparsa VEYA hedef ──────┘
            geçilirse (B5 fly-past)
```

**2026-08-06 — POSE MODELİ KALDIRILDI.** Görsel güdüm artık YOLO-pose'un 6
keypoint'ini kullanmıyor, yalnız detection kutusunu (`cx, cy, w, h, conf`)
okuyor. Pose'a ait tüm kod/model/araç/belge ve sökülmeden önceki güdüm hattının
anlık görüntüsü: **[POSEA_GERI_DONMEK_ISTERSENIZ/](POSEA_GERI_DONMEK_ISTERSENIZ/README.md)**

**Vuruş ölçütü:** Gazebo temas sensörü (fiziksel çarpışma). Yakınlık (1.5 m)
yalnız temas kaynağı yoksa yedek. Bu bilinçli bir seçim — bkz. §4.

**Güncel başarı (2026-08-05, en iyi dönem):** 30 görsel fazda 8 vuruş (%27),
en yakın menzil medyanı 0.65 m. ⚠ Bu rakam POSE dönemine ait; bbox güdümüyle
henüz uçulmadı, yeni taban ilk uçuşlarda ölçülecek.

---

## 2. Aktif yapılandırma

### Güdüm ayarları (kod)

| ayar | değer | dosya | ne yapar |
|---|---|---|---|
| `RANGE_SET` | 11.0 m | `gps_guidance.Cfg` | istasyonun hedefe slant menzili |
| `ISTASYON_ELEV_DEG` | **15.0°** | `gps_guidance.Cfg` | istasyonun LOS yükselişi (25°'den geri alındı) |
| `CENTER_ELEV_DEG` | 25.0° | `gps_guidance.Cfg` | kameranın fiziksel tilt'i (referans) |
| `KP_H` / `KD_H` | 0.8 / **0.60** | `gps_guidance.Cfg` | konum kazancı / **LEAD** (sönümleme değil, §3) |
| `IC_KAYMA` | **14.0 m** | `gps_guidance.Cfg` | iç daire nişanı — dönüş merkezine kayma |
| `IC_ORAN` | 0.0 (kapalı) | `gps_guidance.Cfg` | yarıçap-oranlı kayma; 0.27 ile açılır |
| `V_MAX` | 18.0 m/s | `gps_guidance.Cfg` | GPS fazı yatay hız tavanı |
| `VZ_MAX` / `MAX_ACCEL` | 6.0 / 12.0 | `gps_guidance.Cfg` | dikey hız / ivme tavanı |
| `YAW_RATE_MAX` | 120 °/s | `gps_guidance.Cfg` | GPS fazı dönüş hızı |
| `YAW_DOYGUN_N` | 15 kare | `gps_guidance.Cfg` | yaw susturma eşiği |
| `YAW_SUS_N` | **40 kare (2 s)** | `gps_guidance.Cfg` | susma SÜRESİ — kilitlenmeyi keser (§3) |
| `BBOX_L_ETKIN_M` | **1.687 m** | `guidance_core.Cfg` | kutu ölçeği kalibrasyonu (w ≈ fx·L/R) |
| `OLCEK_KAPALI/TAM_PX` | **12.5 / 29.3** | `guidance_core.Cfg` | kalite rampası (≈ 22.5 m → 9.6 m) |
| `PN_YATAY_SURE` | **0.6 s** | `guidance_core.Cfg` | azimut-oranı lead'i (şekil-lead'in yerine) |
| `PN_YATAY_MAX_DEG` | **20°** | `guidance_core.Cfg` | yatay lead tavanı |
| `YAW_SUS_N` | **60 kare (2 s)** | `guidance_core.Cfg` | görsel fazda yaw susma süresi |
| `FLYPAST_MENZIL/BUYUME` | **8.0 / 1.5 m** | `guidance_core.Cfg` | B5 hedefi geçme tespiti |
| `V_KAPANMA` | 25.0 m/s | `guidance_core.Cfg` | görsel faz sabit kapanma hızı |
| `V_YAKLASMA` | 20.0 m/s | `guidance_core.Cfg` | yaklaşma alt-fazı yatay hızı (8-18 m) |
| `IVME_TAVAN` | 4.0 / 10.0 | `guidance_core.Cfg` | yatay / dikey ivme tavanı |
| `TERMINAL_MENZIL` | 8.0 m | `guidance_core.Cfg` | altında kör dalış mümkün |
| `VURUS_MENZIL` | 1.5 m | `guidance_core.Cfg` | yalnız temas sensörü yoksa yedek |
| `KILIT_N` / `KILIT_PENCERE` | 10 / 15 | `supervisor.SupCfg` | geçiş için görsel kilit (tespit güveni) |
| `GATE_MENZIL` | 20.0 m | `supervisor.SupCfg` | geçiş için yatay menzil kapısı |

### ArduPilot parametreleri (`sim/ardupilot_params/avci_copter.parm`)

Dosya **`gps_kararli_hal` dalından olduğu gibi alındı** (2026-08-06, kullanıcı
kararı). Kararlı dalın uçuşta kullandığı zarf:

| parametre | değer | kararlı daldaki yazımı | not |
|---|---|---|---|
| `ATC_ANGLE_MAX` | 45° | `ANGLE_MAX 4500` | yatay ivme tavanı ≈ 9.8 m/s² |
| `WP_SPD` | 25 m/s | `WPNAV_SPEED 2500` | GUIDED yatay hız tavanı |
| `WP_ACC` | 8 m/s² | `WPNAV_ACCEL 800` | yatay ivme |
| `WP_SPD_UP` / `WP_SPD_DN` | 6 / 4 m/s | `WPNAV_SPEED_UP/DN 600/400` | dikey hız |
| `WP_ACC_Z` | **2.5 m/s²** | `WPNAV_ACCEL_Z 250` | dikey ivme (1'den yükseldi) |
| `WP_JERK` | 4 m/s³ | `WPNAV_JERK 4` | ivme tavanını jerk boğmasın |
| `PSC_NE_VEL_P` | 2.0 | `PSC_VELXY_P 2.0` | yatay hız halkası |
| `WP_YAW_BEHAVIOR` | 0 | aynı | firmware yaw'a karışmaz, tek yetkili güdüm |

**Değerler kararlı dalın uçuşta doğruladığı değerlerdir; yalnız AD ve BİRİM
çevrildi.** Kararlı dal 08-04 tarihli bir dökümle `WPNAV_*` şemasını
kanıtlıyordu; bu makinedeki firmware (`ad28bb78d2`) o adları **tanımıyor**.
`~/ardupilot/mav_5_1.parm` tam bir döküm (1421 satır, hiç set etmediğimiz
`ATC_ANG_YAW_P`, `MOT_THST_HOVER`, `PSC_*` de içinde) ve orada `^WPNAV` **0
satır**, `^WP_` 12 satır. Ad çevrilmeseydi bu 8 satır sessizce yok sayılır,
araç firmware varsayılanlarıyla (10 m/s, 2.5 m/s², 30°) uçardı.

> **Doğrulama zorunlu:** SITL yeniden başladıktan sonra
> `python3 tools/parm_denetle.py` → 11/11 ✓ olmalı. Değilse uçuş ölçümü
> geçersizdir. **TODO.md madde 0'ın ilk kutusu.**

> **Genel kural:** bu firmware ailesinde ArduPlane ve ArduCopter **farklı** ad
> şemaları kullanıyor ve sürüm değiştikçe kayıyor. Tanımadığı adı ArduPilot
> **sessizce yok sayar** — bu tuzağa üç kez düşüldü.

### Ortam değişkenleri (varsayılanlar)

| değişken | varsayılan | etkisi |
|---|---|---|
| `AVCI_TRACKER` | **off** | HybridSORT takip (bkz. §5) |
| `AVCI_GT_ROT` | off | GT teşhis modu (bkz. §3) |
| `AVCI_GPS_RANGE` | 11.0 | `RANGE_SET` override |
| `AVCI_HYBRID_GATE_MENZIL` | 20.0 | menzil kapısı override |
| `AVCI_GPS_GUDUM` | **istasyon** | `frpn` ile FRPN+IMM yasasına geçilir (§5) |
| `AVCI_GPS_KD` | 0.60 | lead kazancı; 0.2 eski davranış |
| `AVCI_GPS_IC` | 14.0 | iç daire kayması; 0 kapatır |
| `AVCI_GPS_IC_ORAN` | 0.0 | 0.27 ile yarıçap-oranlı kaymaya geçilir |

---

## 3. Ölçülmüş gerçekler

### ⚑ YAW SUSTURMA KİLİDİ — "avcı neden düz gitmiyor" (2026-08-06)

Kullanıcının gözlemi: *"Talon −19° başlıkla düz uçuyor, avcı −22°'de."*
Ölçüldü ve **iki ayrı şey** çıktı:

**(a) İyi geçişlerdeki 3°** — bu normal ve tasarım gereği. `YAW_DEADBAND = 3°`:
hata 3°'nin altındaysa adım sıfırlanır ve komut aracın mevcut başlığına
demirlenir, yani 3°'ye kadar artık hata **bilerek** kapatılmaz (sürekli küçük
yaw komutu kamerayı titretirdi). Kararlı iki uçuşta ölçülen:
kadraj azimut hatası medyanı **−0.25°**, karelerin %93'ü |·| < 3°, yaw adımı
karelerin %89'unda tam 0. **Burun hedefte.** 11 m'de 3° = 0.58 m yanal.

**(b) ASIL SORUN — susturma kapısı bir ÖLÜ KİLİTTİ.** Yaw doygunluk koruması
(`YAW_DOYGUN_N`) hata kapanmıyorsa adımı 0 yapıyordu. Ama adım 0 → araç dönmez
→ hata kapanmaz → kapı **hiç açılmaz**. Çıkış koşulu "hata tavanın altına
insin"di; dönmeyen araçta hata kendiliğinden inmez.

| ölçüm (124 346 GPS karesi, 08-05/08-06) | değer |
|---|---|
| burun >20° sapmış AMA yaw adımı tam 0 | **%7.8** |
| en uzun kesintisiz susma | **1867 kare = 93 saniye** (log 142313) |
| o uçuşta iris başlığı − hedef başlığı | 119°, yaw hızı p90 **0.0 °/s** |
| tüm bandda \|kadraj_yaw\| medyanı | **52.4°** (LOS − hedef başlığı yalnız 1.33°) |

Son satır kritik: **geometri doğruydu** (drone hedefin kuyruk hattındaydı,
istasyona yanal hata medyanı 0.17 m) — bozuk olan tek şey burnun nereye
baktığıydı.

**Düzeltme:** susma artık KALICI değil SÜRELİ. `YAW_SUS_N` kare sonra yetki
geri verilir; sorun gerçekse kapı yeniden tetiklenir. Net etki: sürekli dönme
yerine oranı `YAW_DOYGUN_N/(YAW_DOYGUN_N+YAW_SUS_N)` kadar kısılmış dönme.
Kapalı çevrim testinde (bayat algı, 30 s) kaçak 7.5 tur → **1.5 tur**; kilit
93 s → **2 s**. Testler: G13/G15 (GPS), T45/T45b (görsel).

> Asıl koruma zaten **demirleme**: komut her karede aracın gerçek başlığından
> üretilir (`cmd_yaw = iyaw + adim`), yani asla actual+adım'ı aşamaz. Doygunluk
> kapısı ikinci kattır ve hiçbir zaman kalıcı olmamalıydı.

### ⚑ GPS FAZI — `gps_kararli_hal` dalından entegre edildi (2026-08-06)

Ayrı bir dalda GPS güdümü uçuşta doğrulanmış bir kararlı hâle getirilmişti;
o daldaki **yalnız GPS'e ait** değişiklikler bu dala alındı. Tam kayıt ve
uçuş sonuçları: **[KARARLI_HAL.md](KARARLI_HAL.md)**. Üç ölçülmüş bulgu:

**1. `KD_H` bir sönümleme değil, LEAD'in kendisiymiş.** `de[]` istasyon
hatasının türevi, yani ≈ göreli hız. Yasa açılınca
`v_cmd = v_hedef + KP_H·Δp + KD_H·Δv` — FRPN'in hız formuyla aynı üç terimli
yapı. Yani lead zaten vardı, sadece ~3 kat zayıftı. 0.20 → 0.60: oturmuş
menzil **34.3 → 29.4 m** (aynı senaryo, 150 s boyunca ±0.3 m kararlı).

**2. İç daire nişanı — en büyük tek kazanç.** Dairesel kovalamacada zorunlu
bir bağ var: `yarıçap = hız / açısal_hız`. İstasyon "hedefin hız yönünün
gerisi"ne konuyordu; o nokta hedefin **kendi çemberinin üzerinde**. Drone onu
kovaladığı sürece aynı yarıçapta uçmak zorunda, dolayısıyla aynı hıza muhtaç —
ve hedeften hızlı değilse asla yaklaşamaz. Bu yüzden `V_MAX` 18→24 yapılınca
menzil **açılmıştı** (drone yarıçapı 38→43 m). Çözüm istasyonu dönüşün İÇİNE
kaydırmak:

| kayma | menzil (medyan) | en yakın | drone R − hedef R |
|---:|---:|---:|---:|
| 0 m | 34.1 m | 31.3 m | +2 m (aynı çember) |
| 8 m | 22.8 m | 6.9 m | −7 m (içeride) |
| **14 m** | **9.8 m** | **3.2 m** | **−11 m (içeride)** |

Kayma hedefin açısal hızıyla ölçekleniyor → **düz uçuşta tam sıfır**, düz
kovalama davranışı bozulmuyor (kare deseninin düz kenarlarında doğrulandı).

**3. Gecikme birikmesi — üç ayrı hatta aynı hata.** Kamera callback'i tüm işi
senkron yapıyordu (medyan 21.8 ms) ama kamera 30 Hz (bütçe 33.3 ms); bütçe
aşılınca gz-transport kuyruğu hiç boşalmıyordu. Aynı hata telemetride: her
turda tek mesaj + 5 ms uyku = tavan 200 msg/s, oysa iki araç ~300-500 msg/s
yayın yapıyor. İzole ölçüm (30 Hz üretici, 40 ms işleme): eski desen
80 → **579 ms** (sürekli büyüyor), yeni desen 19 → **19 ms** (sabit).
Bayat telemetri doğrudan hedef hız kestirimini zehirlediği için bu GPS'i de
ilgilendiriyor.

**Alınmayanlar:** kararlı daldaki `guidance_core` / `visual_lead` / arayüz
değişiklikleri **alınmadı** (görsel faz bu dalda baştan yazıldı ve pose
söküldü). Kararlı dalın GPS yaw'ı hâlâ eski birikimli `cmd_yaw`'dı; bu dalın
demirleme + süreli susma düzeltmesi **korundu**.

### BBOX ÖLÇEĞİ pose ölçeğinden DAHA KARARLI (2026-08-06)

Menzil vekili olarak hangi sinyal daha az saçılıyor (n=1917 `ok` karesi,
`w·R/fx` = etkin uzunluk, p10-p90 yayılımı):

| ölçüt | etkin L medyanı | yayılım |
|---|---:|---:|
| **bbox genişliği (w)** | **1.687 m** | **±%45** |
| bbox hypot(w,h) | 1.936 m | ±%52 |
| pose ölçeği (sökülen) | 1.343 m | ±%55 |
| bbox sqrt(w·h) | 1.245 m | ±%58 |
| bbox yüksekliği (h) | 0.904 m | ±%83 |

Kutu genişliği seçildi (`BBOX_L_ETKIN_M = 1.687`). Kalite rampası aynı MENZİL
bandını koruyacak şekilde yeniden ölçeklendi (12.5 px ≈ 22.5 m, 29.3 px ≈ 9.6 m).

### LEAD KAYBOLMADI — kaynağı değişti (2026-08-06)

Pose'un şekil-lead'i (yandanlık × K_LEAD) gitti; yerine `adapter_copter._yatay_pn`
azimut-oranı lead'i geldi. Aynı 10 183 kare üzerinde yeniden oynatıldı:

| | medyan | ort | p90 |
|---|---:|---:|---:|
| şekil-lead'i (pose) | 2.39° | 6.18° | 18.64° |
| azimut-oranı lead'i | 1.09° | 6.08° | 20.00° |

Ham azimut oranı çok gürültülü (|oran| medyan 15.8 °/s ama p90 187 °/s), o
yüzden dikey kanalın yumuşatma zinciri (slew-kırpma → EMA → oran EMA) birebir
uygulandı. Saf takibe dönüş: `AVCI_IBVS_PN_YATAY_SURE=0`.

### ISKA SONRASI YAN UÇUŞ — B5'in gerekçesi (2026-08-06)

Hız vektörü ile burun arasındaki açı:

| | n | medyan | p90 | >60° oranı |
|---|---:|---:|---:|---:|
| ıska ÖNCESİ (yaklaşırken) | 4811 | 25.0° | 84.1° | %28 |
| **ıska SONRASI (geçtikten sonra)** | 3269 | **54.4°** | **138.9°** | **%47** |

Sebep yapısal: quad'da yaw ve hız bağımsız. Hedefi geçince nişan vektörü
arkayı gösterir → drone geriye uçar, ama yaw tavanı 90 °/s olduğu için burun
yetişemez. Kamera boşa bakar, tespit kopar, toparlaması 10-20 s sürer.
**B5 uygulandı** (fly-past tespiti + faz sonu komut sıfırlama) — bkz. §2.

### KALKIŞ AŞIMI hedefin tırmanışına bağlı (2026-08-06)

61 taze kalkış (GPS logu iris yerdeyken başlıyor), ilk 25 s:

| kovalamaya başlarken hedef nerede | aşım (drone hedefin kaç m üstüne çıkıyor) |
|---|---|
| hedef zaten seyirde (z < −12 m) | medyan ≈ **0 m**, çoğu negatif |
| **hedef hâlâ pistte/tırmanışta (z > −7 m)** | **+7 ile +17.9 m** |

Genel: medyan +0.9 m · ortalama +2.4 m · max **+17.9 m** · %39'u > 2 m.

Yani sorun kalkışın kendisi değil, **hedef tırmanırken kovalamaya başlamak**.
`_chase_thread` `plane_z`'yi BİR KEZ okuyup kalkış irtifası seçiyor; hedef
sonra 15 m daha tırmanıyor, GPS fazı P kontrolüyle (`KP_Z=1.0`, `VZ_MAX=6`)
peşinden gidiyor ve `WP_ACC_Z` rampası yüzünden zamanında duramayıp üstüne
çıkıyor. İş TODO.md'de.

---

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

### Kök neden: dikey bütçe (2026-08-02, üç uçuş, kara kutuyla)

Üç uçuş da terminale aynı geometriyle giriyordu: yatay ~12.5 m, dikey +4.65 m.
ArduPilot dikey hız komutunu **1.0 m/s²** ile rampalıyordu (`WP_ACC_Z`
varsayılanı). Güdüm 8-22 m/s tırmanma istiyor ama hız tavanı hiç görülmüyor —
yani **hız değil, İVME sınırlıyor**.

| | **A (vurdu)** | B (ıska) | C (ıska) |
|---|---:|---:|---:|
| görsel faza giriş menzili | **10.32 m** | 7.65 m | 9.16 m |
| faz başından en yakın ana | **2.64 s** | 2.38 s | 2.77 s |
| kapatılan dikey | **4.25 m** | 2.70 m | 2.42 m |
| **en yakın anda kalan dikey** | **+0.03 m** | **+1.52 m** | **+2.06 m** |
| sonuç | **GERÇEK TEMAS** | alttan geçti | alttan geçti |

Sıfırdan 4.65 m kapatmak 1 m/s²'de 3.05 s sürer; üçünün de eldeki süresi
altında. A yalnızca **rampayı erken başlattığı** için yetişti.

4-2 m bandında (algı çöküşünün **sonuç** olduğunun kanıtı):

| | A | B | C |
|---|---:|---:|---:|
| `gercek_kadraj_ici` | **%69** | %21 | %15 |
| son 1.5 s'de `kor_dalis` kare | **1/46** | 30/46 | 29/46 |

**Asıl kusur SİMETRİSİZLİK** (DENEY.md silinirken buraya taşındı, 2026-08-06):
20 m/s tırmanma **komutu** verebiliyoruz ama onu yalnız 1 m/s² ile
**söndürebiliyoruz**. Görsel faz hedefe alttan yaklaştığı için sürekli tırmanma
komutu verir (karelerin %61-82'si negatif `vz_cmd`); temas kopunca kontrol GPS
fazına döner ama drone hâlâ hızla tırmanıyordur:

| kalan tırmanma | durması | bu sırada yükselme |
|---|---|---|
| 2 m/s | 2 s | 2 m |
| 4 m/s | 4 s | **8 m** |
| 7 m/s | 7 s | **24 m** |

Gözlenen 15-25 m'lik istasyon aşımları tam olarak bu. Çözüm iki uçtan biri:
`adapter_copter`'da dikey komut tavanını düşür (bkz. TODO B9), **veya**
`WP_ACC_Z`/`PSC_ACCZ` yükselt — yani komut yetkisiyle frenleme yetkisini
eşitle. `AVCI_GPS_RANGE=8`'in işe yaramasının sebebi de buydu: istasyon
yakınlaşınca görsel faz daha yakında devralıyor, dikey savrulma birikmiyor.

### Yaw kaçağı: kök neden ve düzeltme (2026-08-05)

GPS fazının yaw komutu **kendi kendini besleyen bir birikimdi**
(`cmd_yaw += clamp(bearing − cmd_yaw)`), aracın gerçek başlığına hiç
bakmıyordu. Araç yetişemezse komut kaçıyor, hata kapanmıyor, drone fırıldak
gibi dönüp itkiyi aşağı çeviriyor ve yerçekiminden hızlı düşüyordu
(15.8 m/s ölçüldü). Tipik iz: yaw hızı 28 → 90 → 402 → 710 → **1237 °/s**,
komut ise yalnız 240 °/s.

Düzeltme (`gps_guidance`, testler G12/G13): komut her karede aracın **gerçek
başlığına demirlendi** + hata kapanmıyorsa yaw susturuluyor. Uçuşta doğrulandı:

| | koruma öncesi | koruma sonrası |
|---|---|---|
| \|yaw_cmd − gerçek_yaw\| en büyük | 179.9° | **6.3°** (31/31 uçuş) |
| tepe yaw hızı medyanı | 202 °/s | **134 °/s** |

⚠ Koruma **en kötü durumları bitirmedi** — bir uçuşta hâlâ 3590 °/s tepe
görüldü. Kalan kısım fly-past sonrası (TODO B5).

### Karar: vuruş ölçütü = gerçek fiziksel temas

İki seçenek vardı: 1.5 m yakınlığı vuruş saymak (ekranda güzel, gerçekte
ıskaladığımızı bilmeyiz) veya gerçek temas (dürüst ama çoğu denemede vuramayız).

**Gerçek temas seçildi.** Eski halin "başarısı" kısmen erken durmadan
geliyordu: drone 1.5 m'ye gelince "VURULDU" deyip güdüm DURUYOR, hedefin
yanından geçtikten sonrasıyla hiç yüzleşmiyordu. Gerçek temas ölçütü bu sorunu
**yaratmadı, görünür kıldı**.

---

## 4. Çalışma kuralı ve öğrenilen dersler

**Tek seferde tek değişken → testler → uç → ölç → yaz.** Bu kural bir kere
sekiz grup değişikliğin bir arada uçurulması üzerine kondu: bazıları işe
yaradı, biri ölçülebilir zarar verdi, hangisinin ne yaptığı ayırt edilemedi.

- **"Araç komutu uygulamıyor" demeden önce kara kutuya bak.** Bu teşhis bir kez
  kondu ve çürütüldü: alçalma emredilen anlarda `PSCD.DVD +6.36` iken
  `VD +6.43`, takip hatası 0.1 m/s. Araç kusursuz uyguluyordu.
- **Ölçmeden değer değiştirme.** `ATC_ANG_YAW_P 4.5 → 3.0` iyi niyetle yapıldı,
  düzeltmeye çalıştığı şeyi 4 katına çıkardı.
- **Bozuk veriyle ölçüm yapma.** Hedefin hızı bir süre `tgt_vx` sütunundan
  17.5 m/s sanıldı; o sütun zaten bozuk olduğu kanıtlanan kestirimdi. Gerçek
  değer kara kutudan **14.0 m/s** çıktı.
- **CSV'ye tek başına güvenme.** Bir vuruşta CSV 3.20 m derken kara kutu
  0.21 m diyordu. Geometri sorularının dürüst kaynağı iki aracın kara kutusu
  (`tools/gecis_analiz.py`).
- **Ölçüm aletini önce doğrula.** `menzil_gercek_m` uzun süre telemetriden
  geliyordu ve karelerin **%37'sinde donuktu** (en uzun donma 0.4 s = 25 m/s'te
  10 m yol). Kaynak zaman hizalı gz'ye çevrildi; `menzil_kaynak` sütunu hangi
  kaynağın kullanıldığını yazar.
- **Aynı anda iki şey değiştirme.** 2026-08-05'te `ISTASYON_ELEV_DEG` ve
  `WP_ACC_Z` birlikte değişti; kötüleşti ama hangisi yüzünden bilinmiyor.

---

## 5. Bitenler

- [x] **A1 — `ATC_ANG_YAW_P 3.0` kaldırıldı** (varsayılan 4.5'e dönüldü).
      Sabit-başlık dilimlerinde takip hatası std: 3.0 → 11.88°/8.53°;
      4.5 → **1.32°/1.43°**.
- [x] **A5 — Gerçek çarpışma tespiti.** 1.5 m yakınlık artık vuruş sayılmıyor;
      tek kaynak Gazebo temas sensörü. Bir koşuda 6 sahte vuruş raporlanırdı.
      Sensör gövde+kanat+kuyrukta, tekerlek hariç (pistte sürekli yere değer).
- [x] **Dikey ıska, 1. tur** — sabit 4.65 m ofset yerine
      `r_eff = min(menzil, RANGE_SET)`; LOS yükselişi her menzilde sabit (G10).
- [x] **Dikey ıska, 2. tur** — `ISTASYON_ELEV_DEG` kamera tilt'inden ayrıldı.
      ⚠ 2026-08-05'te 25°'ye geri döndürüldü, ölçüm kötüledi (TODO madde 0).
- [x] **Kendi etrafında dönme (görsel faz)** — `yaw_hata` kapanmıyorken komut
      her karede bir tavan adımı daha ekliyordu. `adapter_copter`'a "hata
      kapanmıyorsa yaw'ı sustur" kapısı (T44/T45). Seyirde dönme 27 °/s → ~0.
- [x] **Yaw kaçağı (GPS fazı)** — aynı koruma `gps_guidance`'a taşındı
      (G12/G13). Bkz. §3.
- [x] **`WP_YAW_BEHAVIOR` 2 → 0** — firmware yaw komutu olmayan anlarda burnu
      gidiş yönüne çeviriyordu.
- [x] **Parametre adları yanlıştı** — 9 parametrenin 7'si SITL'e hiç
      uygulanmıyordu. `tools/parm_denetle.py` tekrarını önlüyor.
- [x] **GT rotasyon modu** (`AVCI_GT_ROT`) — teşhis aracı, algının darboğaz
      olmadığını ölçtü. Testler T49-T54.
- [x] **Menzil kaynağı** — `_menzil_olc()` bağlandı: zaman hizalı gz önce,
      telemetri yedek. `menzil_kaynak` sütunu. Testler T55/T55b.
- [x] **Yapılandırma damgası** — `visual_lead` CSV'sinin ilk satırına hangi
      bayraklarla uçulduğu yazılıyor (GT/POSE/TRACKER/GPS_RANGE/...).
- [x] **HybridSORT kurulumu** — `boxmot` hiç kurulu değildi, takipçi bugüne dek
      **hiç çalışmadı**; projedeki tüm eski ölçümler takipçisiz. Kuruldu ama
      varsayılan **kapalı** bırakıldı (ölçüm tabanı sessizce değişmesin).
      A2/A3 kıyası: takip açıkken vuruş %17 → %5.
- [x] **Sahte PnP paneli kaldırıldı** — ground-truth'a yapay gürültü ekleyip
      "tahmin" diye gösteriyordu.
- [x] **Hedef telemetrisi = cevap anahtarı** — güdüme bağlanmadı, ölçüm için
      10 sütun eklendi.
- [x] **Arayüz** — Ayşenur'un taktik saha ekranı birleştirildi; mesafedeki
      sabit +8 m ofset hatası kaldırıldı, kaynak gz'ye çevrildi, geçiş sayacı
      geri getirildi.
- [x] **Ölçüm araçları test altına alındı** — `tests/olcum_araclari/`.
- [x] **Talon manuel modda kalkmıyor** — ARM + TAKEOFF adımları eklendi.
- [x] **GCS telemetrisi donuyor** — `mavlink_listener`'a `else` dalı.
- [x] **Başlatma/durdurma script hataları** — kendi kabuğunu öldüren `pkill`,
      sahte "hazır" bildirimi.

---

## 6. Tekrar denenmeyecekler (ölçümle çürütüldü)

| fikir | nerede yazılı | ne oldu |
|---|---|---|
| `ATC_ANG_YAW_P` 4.5 → 3.0 | `avci_copter.parm` | yaw takip hatası 1.4° → 8.5-11.9° |
| `supervisor.KILIT_N` 10 → 7 | `supervisor.py` SupCfg | faz/uçuş 3.4 → 8.0, her ölçüt kötüleşti |
| GT modunda pose kilidini atla | `supervisor.py` `GT_KILIT_BYPASS` | devir 6.6 → 19.6 m, 13/13 faz kayıp |
| hedef hızına ivme kapısı | TODO A4 | hız kestiriminin oturmasını da engelledi |
| "araç komutu uygulamıyor" teşhisi | bu belge §4 | kara kutu çürüttü; takip hatası 0.1 m/s |
| `pkill` köşeli parantez hilesi | `dokumantasyon/17_KOD_*.md` | kendi kabuğunu öldürüyor |

**`KILIT_N` denemesinin ayrıntısı** (mantıklı görünüyordu): devir menzili ile
vuruş arasında güçlü bağıntı vardı (vuranlar 11.11 m'de, ıskalayanlar 9.05 m'de
devraldı); kapıyı gevşetip devri uzaklaştırmak denendi. Her ölçütte kötüleşti:
giriş menzili medyanı 10.00 → 9.62 m (**düştü**), en yakın menzil 1.73 → 2.08 m,
vuruş 3/17 → 1/8. Mekanizma: kapı cılız tespitte de açılıyor, erken devir
gerçekten oluyor ama 0.9-1.3 s'de ölüyor, GPS'e dönülüyor, drone bu arada
yaklaşıyor, sonraki devir **daha yakında** oluyor.
**Ders:** devir menzili ↔ vuruş bağıntısı **nedensel değil**; ikisi de "tespit
o an gerçekten sağlam mı"ya bağlı. Kapı sağlamlık üretmiyor.

**Kural:** "araç komutu uygulamıyor" demeden önce MUTLAKA kara kutuda
`PSCD.DVD` vs `PSCD.VD` (dikey) veya `ATT.DesYaw` vs `ATT.Yaw` (yaw) karşılaştır.

---

## 7. Sözlük

| terim | anlamı |
|---|---|
| **kara kutu** | ArduPilot'un kendi uçuş kaydı (`~/ardupilot/logs/*.BIN`). Aracın gördüğü attitude, motor çıkışları, kontrolcü hedefleri. Bizim CSV'lerimizden bağımsız — "araç komutu uyguladı mı" sorusunun tek dürüst kaynağı. |
| **istasyon** | GPS fazının drone'a "şurada dur" dediği hayali nokta; hedefle birlikte hareket eder. Sabit metre DEĞİL sabit AÇI: hedeften `RANGE_SET` uzakta, LOS yükselişi `ISTASYON_ELEV_DEG`. Amacı vurmak değil, kamerayı hedefe oturtmak: alttan bakış (gökyüzü fonu), pose'un çalıştığı menzil bandı, hedefin hızına uyum. |
| **istasyon aşımı** | Drone'un istasyonun ÜSTÜNE çıkması. Kötü çünkü yukarıdan bakınca hedefin fonu yer olur, gökyüzü silueti kaybolur. Sebebi: görsel faz alttan yaklaştığı için sürekli tırmanma emrediyor; temas kopunca kontrol GPS'e drone hâlâ tırmanırken dönüyor. |
| **faz** | GPS fazı (uzaktan yaklaşma, `gps_guidance`) ↔ görsel faz (terminal hücum, `visual_lead`). Geçişi `supervisor` yönetir. |
| **geçiş sayısı** | GPS→görsel kaç kez geçildi. 1 ideal; yüksek sayı görsel temasın kopup kopup kurulduğunu gösterir. |
| **`ok` oranı** | `visual_lead` her kareye `durum` etiketi yazar. `ok` = pose hedefi temiz gördü, keypoint'ler güvenilir. Diğerleri: `kpt_dusuk`, `tespit_yok`, `kor_dalis`, `bayat`. |
| **min medyan** | Her görsel fazın en yakın menzili alınır, hepsinin medyanı. "Tipik bir fazda hedefe ne kadar yaklaşıyoruz" — tek kötü faz ortalamayı bozduğu için medyan kullanılır. |
| **fly-past** | Drone hedefe temas etmeden yanından geçmesi. Sonrasında "hedefe uç" komutu yukarı-geriyi gösterir → kontrolsüz tırmanma. Bkz. TODO B5. |
| **yaw susturma** | Dönüş komutu verildiği hâlde hata kapanmıyorsa (bayat/hatalı ölçüm), dönmeye devam etmek yalnız aracı çevirir. 15 kare sonra yaw komutu sıfırlanır. |
| **cache-buster** | `script.js?v=15` sonundaki sürüm numarası. Olmadan tarayıcı eski dosyayı önbellekten servis eder ve düzeltmeler görünmez. Arayüz dosyası değişince artırılır. |
