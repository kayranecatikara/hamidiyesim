# TODO — yapılacak işlerin tamamı

Bu dosya **yalnız yapılacak işleri** tutar. Sistemin şu anki hali, ölçülmüş
gerçekler ve çürütülmüş fikirler → **[DURUM.md](DURUM.md)**.
Tek değişkenli deney adımları → **[DENEY.md](DENEY.md)**.

**Çalışma kuralı:** tek seferde tek değişken → testler → **uç** → ölç →
*Sonuç:* satırına yaz → tikle. (Gerekçesi DURUM.md'de.)

---

## ⚑ Öncelik sırası (2026-08-06 güncellendi)

| # | iş | neden şimdi |
|---|---|---|
| **0** | [25° + `WP_ACC_Z=3` geri al](#0--acil-son-değişikliği-geri-al) | ölçüldü, **kötüleştirdi** |
| **1** | [B10 — kalkışta hedefin üstüne çıkma](#b10--kalkışta-hedefin-üstüne-çıkma) | **her uçuşun başında bozuluyor** (max +17.9 m) |
| **2** | [B1 — görsel faza irtifa tabanı](#b1--görsel-faza-irtifa-tabanı) | çakılmayı doğrudan keser |
| **3** | [Terminal kontrol yetkisi](#terminal-fazda-kontrol-yetkisi) | son metrede 25 m/s = kare başına 0.81 m |
| **4** | [B8 — frenleme eğrisi](#b8--frenleme-eğrisi-mesafeye-bağlı-hız-tavanı) | "18 m/s çok yavaş" — **kullanıcı şimdilik erteledi** |
| 5+ | aşağıdaki diğer maddeler | |

### ✅ 2026-08-06'da BİTENLER (ölçümleri DURUM.md §3'te)

- **Pose modeli kaldırıldı** → görsel güdüm yalnız bbox. Arşiv + geri dönüş
  yolu: [`POSEA_GERI_DONMEK_ISTERSENIZ/`](POSEA_GERI_DONMEK_ISTERSENIZ/README.md)
- **B5 — fly-past + faz sonu komut sıfırlama** (aşağıda ✓)
- **B9 — dikey hız tavanı** → `VZ_TERMINAL_MAX = 12` (2026-08-05'te girmişti)
- **Yaw susturma kilidi** → süreli susma (`YAW_SUS_N`). Ölçüm: karelerin
  %7.8'inde burun >20° sapmışken adım 0, en uzun susma **93 saniye**.

---

## 0 — ACİL: son değişikliği geri al

**Ölçüldü ve kötüleştirdi.** `ISTASYON_ELEV_DEG` 15° → 25° ve `WP_ACC_Z` 1 → 3
aynı anda değiştirildi (2026-08-05 18:20). Aynı `GPS_RANGE=11` ile önce/sonra:

| | C: 15° + ACC_Z=1 | D: 25° + ACC_Z=3 |
|---|---|---|
| görsel faz | 9 | 10 |
| vuruş | 2 (%22) | **1 (%10)** |
| <1.5 m geçiş | 5/9 | **3/10** |
| en yakın menzil medyanı | 1.11 m | **2.04 m** |
| istasyon aşımı medyanı | −5.4 m | **−8.3 m** |
| yere çakılan uçuş | %27 | **%30** |

Beklenen kazanç (aşımın küçülmesi) **gerçekleşmedi, tersine büyüdü**.

- [ ] `gps_guidance.Cfg.ISTASYON_ELEV_DEG` → **15.0**
- [ ] `avci_copter.parm` → **`WP_ACC_Z 1`**
- [ ] `tests/test_gps_guidance.py` G11 → **`A_DIKEY = 1.0`** (araç yeteneğiyle
      birlikte hareket etmeli, yoksa test 25° geometriyi yanlışlıkla onaylar)
- [ ] İkisi **birlikte** geri alınmalı; sonra tek tek denenecek.
      *Sonuç:*

> **Ders:** iki değişken aynı anda değiştirildi, hangisinin suçlu olduğu
> ayrılamadı. Geri aldıktan sonra önce yalnız `WP_ACC_Z=3` (15° ile), sonra
> yalnız 25° (ACC_Z=1 ile) denenmeli.

---

## Yüksek öncelik

### B10 — Kalkışta hedefin üstüne çıkma
`control/gcs_server.py` → `_chase_thread` · YENİ (2026-08-06, kullanıcı isteği)

*Gözlem:* "kalkışta drone uçak ile aynı mesafeye gelmeye çalışıyor ama
çoğunlukla geçiyor."

*Ölçüldü — doğru, ama sebebi kalkışın kendisi değil.* 61 taze kalkış
(GPS logu iris yerdeyken başlıyor), ilk 25 s içinde drone hedefin kaç metre
ÜSTÜNE çıkıyor:

| kovalamaya başlarken hedef nerede | aşım |
|---|---|
| hedef zaten seyirde (`tgt_z < −12 m`) | medyan ≈ **0 m**, çoğu negatif (altında kalıyor) |
| **hedef hâlâ pistte / tırmanışta (`tgt_z > −7 m`)** | **+7.0 … +17.9 m** |

Genel: medyan **+0.9 m** · ortalama **+2.4 m** · en kötü **+17.9 m** ·
%39'u 2 m'yi aşıyor.

*Mekanizma:*

```python
plane_z = telemetry_state["plane"]["z"]          # BİR KEZ okunuyor
target_z = plane_z if plane_z < -1.0 else -5.0
success = df_takeoff(target_z=target_z)
```

Hedef o an pistteyse kalkış irtifası −5 m seçiliyor. Talon sonra 15-20 m daha
tırmanıyor; GPS fazı P kontrolüyle peşinden gidiyor (`KP_Z=1.0`, `VZ_MAX=6`,
hedef tırmanma hızı ileri-beslemeli) ve `WP_ACC_Z` rampası yüzünden zamanında
duramayıp üstüne çıkıyor.

*Aday çözümler (karar gerekiyor):*
- [ ] **(a) Kalkışı geciktir** — hedefin tırmanma hızı ~0'a inene kadar bekle,
      sonra o anki irtifayı hedefle. `_chase_thread` içinde; **GPS fazına
      dokunmaz**, en güvenli seçenek.
- [ ] **(b) Kalkış irtifasını canlı tut** — `target_z`'yi tek sefer yerine
      kalkış boyunca tazele. Yine yalnız `_chase_thread`.
- [ ] **(c) Dikey frenleme eğrisi** — `vz` tavanını kalan dikey mesafeye bağla
      (B8'in dikey eşi). ⚠ Bu **GPS fazına dokunur**, kullanıcı izni gerekir.

*Ölçüt:* hedef pistteyken başlayan uçuşlarda aşım < 3 m; seyirdeki hedefte
davranış bozulmamalı.
*Sonuç:*

### B9 — Dikey hız bileşenine ayrı tavan ✅ UYGULANDI (2026-08-05)
`control/guidance/adapter_copter.py` → `v_hedef` üretimi.
Uygulama: `VZ_TERMINAL_MAX = 12 m/s` (`guidance_core.Cfg`), test **T56**.
Gerekçe aşağıda kayıt için duruyor.

*Neden — "dikeyde kaçışı yapmamalıyız" isteğinin kök nedeni.* Araştırıldı:
sorun **hız**, ivme değil.

```python
v_hedef = cfg.V_KAPANMA * u_dunya          # adapter_copter.py:154
```

`u_dunya` birim vektör, `V_KAPANMA = 25`. Yani dikey bileşen doğrudan
**`25 · sin(yükseliş)`**:

| nişan yükselişi | emredilen tırmanma |
|---|---|
| 15° | 6.5 m/s |
| 30° | **12.5 m/s** |
| 60° | **21.7 m/s** |

**Dikey hız için ayrı tavan YOK.** `IVME_TAVAN_DIKEY = 10` var ama o hızın ne
kadar *büyüyeceğini* değil, ne kadar hızlı *değişeceğini* sınırlıyor.
ArduPilot'un `WP_SPD_UP = 5 m/s` tavanı da GUIDED hız komutuna **uygulanmıyor**
(ölçüldü: gerçekleşen tırmanma p99 = 9.4 m/s).

*Ölçüm (08-05, pose modu, `durum=ok` kareler):*
- karelerin **%79'unda** tırmanma emrediliyor
- büyüklük: medyan **5.8 m/s**, p90 **12.1**, tepe **25.0 m/s**

Drone hedefe alttan yaklaştığı için nişan sürekli yukarıyı gösteriyor; temas
kopunca kontrol GPS fazına araç **hâlâ tırmanırken** dönüyor ve istasyonun
5-8 m üstüne fırlıyor.

*Nasıl:* `v_hedef` hesaplandıktan sonra dikey bileşeni ayrı bir tavanla kırp —
yatayı bozmadan. Yeni ayar `VZ_KAPANMA_MAX` (öneri: 6-8 m/s, `VZ_MAX=6` ile
tutarlı). Yönü koru, yalnız büyüklüğü kırp:

```python
if abs(v_hedef[2]) > cfg.VZ_KAPANMA_MAX:
    v_hedef[2] = math.copysign(cfg.VZ_KAPANMA_MAX, v_hedef[2])
```

⚠ **Bedeli ölçülmeli:** dikey hızı kısmak "dikey ıska"yı geri getirebilir
(DURUM.md §3, terminalde kapatılamayan dikey mesafe). O yüzden tavan
`TERMINAL_MENZIL` altında gevşetilebilir — önce sabit tavanla ölç.

*Ölçüt:* istasyon aşımı medyanı |−5.4 m| belirgin küçülmeli; terminalde
"kalan dikey" büyümemeli; vuruş oranı düşmemeli.
*Sonuç:*

### B1 — Görsel faza irtifa tabanı
`control/guidance/visual_lead.py` (veya `adapter_copter`)

*Neden:* GPS fazında `LOOKUP_MIN_ALT = 8 m` yere çakılma koruması var
(`gps_guidance.py`); **görsel fazda hiç yok**. Kara kutu: irtifa 8.0 → 0.2 m,
4 m/s alçalışla, ardından `|roll| > 90°`.
*Nasıl:* dikey komut, drone tabana yaklaştıkça **yumuşak** kırpılacak. Sert
kesme terminal dalışı bozar.
*Ölçüt:* zemine çarpma 3/3 → 0.
*Sonuç:*

### B5 — Fly-past davranışı ✅ UYGULANDI (2026-08-06)
`control/guidance/visual_lead.py` → `_bitir`, `_flypast`, `_terminal_adim`

*Neden:* drone hedefi geçince "hedefe uç" komutu **yukarı-geriyi** gösterir,
drone tırmanır, hedef kadrajdan çıkar, tespit kopar. **Her ıskadan sonra
5-7 m yukarı fırlıyor, toparlaması 10-20 s.**

⚑ **Ölçüldü — asıl kusur burada.** Log `00000108`: vuruş anından itibaren
kesintisiz **14.57 tur dönüş (~350 °/s)**, irtifa 21.8 → 2.0 m. `DesYaw` aynı
rampayı izliyor, yani **komut kesilmemiş**. Güdüm CSV'yi kapatıyor ama
**son yaw-hızı komutu araçta yaşamaya devam ediyor** (MAVLink hız komutu
kalıcıdır; göndermeyi bırakmak "dur" demek değildir).

*Yapılanlar:*
- [x] **Faz biterken `send_velocity(conn, 0,0,0, mevcut_yaw)`** — `_bitir()`,
      HER `return` yolunda (vuruldu/kayip/durduruldu/bayat akış). UDP kaybına
      karşı 3 kez gönderilir. Yaw olarak **mevcut** başlık kullanılır (hedef
      başlık değil: dönüşü durdurmak istiyoruz, yenisini başlatmak değil).
      Test **T62**.
- [x] **Fly-past tespiti — iki bağımsız imza** (`_flypast`):
      (a) MENZİL DÖNDÜ: bu görsel fazın en yakın noktası `FLYPAST_MENZIL`
          (8 m) bandındaysa ve oradan `FLYPAST_BUYUME_M` (1.5 m) uzaklaştıysak.
          Ölçüt anlık işaret değil BİRİKEN mesafe — gürültüde titremez.
      (b) HEDEF ARKADA: `u_govde[0] < 0` (|yaw_hata| > 90°).
      Testler **T60** (tetikleniyor) ve **T61** (yanlış alarm yok).
- [x] **Kör dalış erken kesme** — `_terminal_adim` içinde menzil en yakın
      noktadan `FLYPAST_BUYUME_M` büyürse süre dolmadan biter. Kör dalış
      "hedef önümüzde" varsayar; menzil büyüyorsa varsayım çökmüştür ve
      sürdürülen komut bizi uzaklaştırıyordur.
- [x] Davranış: `"kayip"` dönülür → supervisor GPS istasyon geometrisine
      döner; CSV'ye `durum=gecildi` yazılır.
- [ ] "Yeniden hücum" sayacı — **yapılmadı**, gerekirse sonra.
*Ölçüt:* geçiş sonrası irtifa aşımı ve zemine çarpma 0. **Uçuşla doğrulanacak.**
*Sonuç:*

> 2026-08-05'te eklenen GPS yaw kaçağı koruması (G12/G13) bu maddenin
> **faz içi** kısmını çözmüştü; 2026-08-06'da faz bitişi ve fly-past eklendi.

### B8 — Frenleme eğrisi: mesafeye bağlı hız tavanı
`control/guidance/gps_guidance.py` → `V_MAX` kullanımı

⏸ **ERTELENDİ (2026-08-06, kullanıcı kararı):** *"neyse 18'de kalsın şimdilik,
sonra bakarız."* Kod DEĞİŞMEDİ, `V_MAX` hâlâ sabit 18 m/s.

⚠ Bu madde 2026-08-05'te "uygulayalım" denmesine rağmen o turda yapılmadı —
onun yerine görsel fazın `V_YAKLASMA`'sı (12 → 20) düzeltildi. İkisi AYRI
ayardır: `V_MAX` GPS fazının (150 m'de görülen 18 m/s), `V_YAKLASMA` görsel
fazın 8-18 m bandındaki hızı.

*Neden — "drone neden hâlâ 18 m/s?" sorusunun cevabı:* `V_MAX` **tek bir sabit
tavan**. Geçmişi: 20 → 28 → 18.
- 28'de araç istasyona zamanında yavaşlayamıyordu: 28 m/s'den 12 m/s² ile durma
  mesafesi `v²/2a = 32.7 m`, istasyon standoff'u ise yalnız 10 m → hedefin
  etrafında savruluyordu.
- 18'e çekildi, savrulma bitti ama **uzakta çok yavaş** — yetişme gecikiyor.

Sabit tavanla ikisi aynı anda çözülemez. Çözüm tavanı **kalan mesafeye bağlamak**:

```
V_MAX_etkin = min(V_MAX_UZAK, sqrt(2 · MAX_ACCEL · kalan_mesafe))
```

12 m/s² ile: 40 m → 28 m/s · 20 m → 21.9 · 10 m → 15.5 · 5 m → 11.0.
Uzakta hızlı gelir, istasyona yaklaşırken kendiliğinden yavaşlar, **tam
istasyonda durur**.

*Ölçüt:* istasyona oturma oranı artmalı, overshoot (min `d_h`) küçülmeli,
hedefe yetişme süresi kısalmalı.
*Sonuç:*

### Terminal fazda kontrol yetkisi
`guidance_core.Cfg.V_KAPANMA` / `IVME_TAVAN`

*Şüphe:* drone hedefe ~1 m'ye geliyor ve ıskalıyor — nişan hatası değil,
**fizik sınırı**. `V_KAPANMA=25` m/s, `IVME_TAVAN=4` m/s² ile:

| yanal hata | düzeltme süresi | bu menzilde bitmiş olmalı |
|---|---|---|
| 0.5 m | 0.50 s | 12.5 m |
| 1.0 m | 0.71 s | **17.7 m** |
| 2.0 m | 1.00 s | 25.0 m |

Görsel faza giriş medyanı ~6 m. Orada kalan 1 m'lik yanal hata
**düzeltilemez**. Dönüş yarıçapı 25 m/s'te yatayda 156 m, dikeyde 62 m.

- [ ] `AVCI_IBVS_V_KAPANMA=15`, sonra `=10` ile karne al.
      *Uyarı:* `IVME_TAVAN=4` keyfi değil — quad ileri ivmelenmek için burnunu
      eğer, kamera gövdeye +25° bağlı, 5 m/s² üstünde kamera yere bakar.
*Sonuç:*

### B7 — İstasyon açısı: 15° mi, 25° mi, arası mı?
`gps_guidance.Cfg.ISTASYON_ELEV_DEG`

⚠ **25° denendi ve kötüleştirdi (bkz. madde 0).** Ama deney kirliydi:
`WP_ACC_Z` de aynı anda değişti. Karar hâlâ verilmedi.

*Şüphe:* 25° tesadüf değil — kamera tilt'i o. İstasyon 25°'de kurulunca hedef
kadrajın **tam merkezinde** oluyor. 15°'de merkezin ~10° altında.

*Elde olan (10 faz @25° vs 17 faz @15°, ESKİ ölçüm):*

| | 25° | 15° |
|---|---:|---:|
| `ok` oranı (tüm faz) | %24.0 | **%32.0** |
| `ok` oranı (menzil < 8 m) | %8.7 | **%18.2** |
| hedef kadraj içi | %59.8 | **%67.0** |
| en yakın menzil medyanı | 5.25 m | **1.73 m** |

⚠ Bu tablo şüpheyi çürütmüyor, **karışık**: algının iyileşmesi büyük ölçüde
geometrinin sonucu (drone seviyeye yakın kalınca hedef kadrajdan geç çıkıyor).
Merkez dışı kadrajlamanın **kendi bedeli hâlâ bilinmiyor**.

- [ ] 15° taranarak seçilmedi, ivme bütçesinden çıktı — **18° ve 20° hiç
      denenmedi**. Bütçeye sığan en büyük açı merkeze daha yakın olurdu.
- [ ] Tek değişkenli tekrar: önce yalnız `WP_ACC_Z=3` (15° ile), sonra yalnız
      25° (ACC_Z=1 ile).
*Ölçüt — 15°'nin haline göre bozulmamalı:* en yakın menzil medyanı ≤ 1.73 m ·
terminal `ok` oranı ≥ %18 · kadraj içi ≥ %67 · dikey artık |·| ≤ 0.9 m.
Bunları tutturan **en yüksek** açı kazanır.
*Sonuç:*

### A8 — Görsel kilit ⚠ B1 ve B2 OLMADAN UYGULAMAYIN
*Neden bekliyor:* bir kez uygulandı, kör uçuş %64'e çıktı ve drone zemine
çakıldı. İrtifa tabanı (B1) ve dikey sönümleme (B2) olmadan tekrarlanırsa aynı
sonuç beklenir.
*Sonuç:*

### B2 — `kilit_kor` sırasında dikey komutu sönümle
A8 ile birlikte. Kör dalışta dikey komut serbest kalıyor; sönümlenmeli.
*Sonuç:*

---

## Orta öncelik

### Pose'un manevra körlüğü
*Ölçüldü (T53b):* hedef **bank yaparken** pose yandanlığı gerçeğin altında
kalıyor — 45° yatıkta 1.00 yerine **0.73**. Sebep: `yandanlik = a/olcek`
"hedef seviyeli uçuyor" varsayıyor. Sonuç: manevra yapan hedefte lead eksik.

- [ ] Pose'un 5. ve 6. keypoint'i (V-tail uçları) **hiç kullanılmıyor**
      (`guidance_core` yalnız 0-3 indekslerini okuyor). Bank açısı bu ikisinin
      kanat ekseni etrafındaki asimetrisinden kestirilebilir.
*Sonuç:*

### A2 — MAVLink kuyruk boşaltma
`gcs_server.py` → `mavlink_listener`. Döngü her 5 ms'de **tek** mesaj okuyor →
tavan 200 msg/s. İki araç × 4 tip × 25 Hz ≈ 200/s, tam sınırda.
*Sonuç:*

### A3 — Hedef hızı aracın KENDİ saatinden
`gcs_server.py` + `gps_guidance.py`. Hız, GCS'e varış zamanından türetiliyor;
araç saatinden türetilmeli.
*Sonuç:*

### A4 — Hedef sıçrama kapısı (emniyet ağı)
`gps_guidance.py` → `_HedefKapisi`. İmkânsız telemetri sıçraması güdüme
girmemeli (menzil kapısının hedef pozu için olan eşi).
*Sonuç:*

### B3 — Kilit süresini kısalt
B2'ye alternatif, daha kaba çözüm.
*Sonuç:*

### B4 — `coalt` kapsamını daralt
Düşük öncelik.
*Sonuç:*

### B6 — Terminal algı kalitesi ⚠ kapsamı daraldı
*Eski gerekçe çürütüldü (2026-08-04):* algı kusursuz yapıldığında (GT modu)
isabet değişmedi. Genel "algıyı iyileştir" işi **rafa kalktı**; geriye kalan
gerçek algı işi yukarıdaki manevra körlüğü.

---

## Küçük / bağımsız işler

- [ ] **`LOOKUP_MIN_ALT` kararı** — şu an 8 m sabit taban. Hedef alçalırsa
      drone takip edemez; hedef irtifasına göreli mi olmalı?
- [ ] **`ATC_ANGLE_MAX` kademeli artır** — 45'te. 50-55 denenebilir; 55'te
      yalpalama izlenmeli.
- [ ] **Görsel faza geçiş kapısı** — şu an "son 15 karenin 10'unda tespit
      conf ≥ 0.5". `KILIT_N=7` denendi, kötüleşti (bkz. DURUM.md). Kilit
      **zaman aşımı** (menzil kapısı içinde N saniye kilit gelmezse devri
      zorla) hiç denenmedi.
- [ ] ~~**Lead'in yumuşak geçişi**~~ — `kpt_dusuk` diye bir durum kalmadı
      (pose kaldırıldı). Yeni karşılığı `kutu_kucuk`; orada zaten yalnız
      `kalite` sönüyor, nişan sıçraması olmuyor.
- [ ] **Menzil verisi neden zıplıyor** — kapı semptomu kesti, kök neden duruyor.
      (2026-08-04'te kaynak `sim_truth`'a çevrildi, telemetri yalnız yedek —
      ama telemetrinin kendi zıplaması araştırılmadı.)
- [ ] **GPS fazında vuruş tespiti yok** — hasar modülü bağımsız izliyor ama
      GPS fazı kendi içinde vuruş raporlamıyor.
- [ ] **Hasar modülünü arayüze bağla** — `/api/hasar` var, panelde gösterilmiyor.
- [ ] **Video kayıt butonları** — başlat/durdur/kayıt dosyası.
- [ ] **RTF'i tam sistemde tekrar ölç** — `gcs_server` + YOLO yükü altında
      0.982 ölçülmüştü; takipçi ve yeni model sonrası tekrar.
- [ ] **A6 — Tanılama endpoint'i** — `/api/debug/hedef_telem`.

---

## Tekrar denenebilecekler (bir kez denendi, koşulları değişti)

Bunlar geri alınmıştı ama gerekçeleri artık geçersiz olabilir. **Yalnız
ilgili kök neden çözüldükten sonra** denenmeli.

- [ ] **Gerçek PN (`γ += N·Δλ`)** — klasik oransal seyrüsefer. Mevcut yasa
      açı-tabanlı; gerçek PN dönüş oranını LOS dönüş oranına bağlar.
- [ ] **Dikey PN'i güçlendirme** (tavan 15°→30°, süre 0.4→0.6 s) — eski
      sonuç: PN yeni tavana da %79 oranında çakıldı. *Ölçüt:* tavana çakılma
      %79'un belirgin altına inmeli.
- [ ] **`KP_KADRAJ ≥ 1.0`** — kadraj tutma kazancını yükseltme.
- [ ] **Yaw'ı mutlak hedefe slew etme** — kalıcı `cmd_yaw` durumu tutup GPS
      fazında mutlak hedefe yönelme. ⚠ **2026-08-05'te tam tersi yapıldı**
      (komut aracın gerçek başlığına demirlendi, kaçak bitti). Bu madde artık
      **karşı yönde**; denenecekse çok dikkatli.
