# TODO — yapılacak işlerin tamamı

Bu dosya **yalnız yapılacak işleri** tutar. Sistemin şu anki hali, ölçülmüş
gerçekler ve çürütülmüş fikirler → **[DURUM.md](DURUM.md)**.
Tek değişkenli deney adımları → **[DENEY.md](DENEY.md)**.

**Çalışma kuralı:** tek seferde tek değişken → testler → **uç** → ölç →
*Sonuç:* satırına yaz → tikle. (Gerekçesi DURUM.md'de.)

---

## ⚑ Öncelik sırası (2026-08-05 akşamı güncellendi)

| # | iş | neden şimdi |
|---|---|---|
| **0** | [25° + `WP_ACC_Z=3` geri al](#0--acil-son-değişikliği-geri-al) | ölçüldü, **kötüleştirdi** |
| **1** | [B1 — görsel faza irtifa tabanı](#b1--görsel-faza-irtifa-tabanı) | çakılmayı doğrudan keser |
| **2** | [B5 — fly-past + faz sonu komut sıfırlama](#b5--fly-past-davranışı) | ıska sonrası toparlayamama |
| **3** | [B8 — frenleme eğrisi](#b8--frenleme-eğrisi-mesafeye-bağlı-hız-tavanı) | "18 m/s çok yavaş" sorusunun cevabı |
| **4** | [Terminal kontrol yetkisi](#terminal-fazda-kontrol-yetkisi) | 1 m'de ıskalamanın fizik sınırı |
| 5+ | aşağıdaki diğer maddeler | |

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

### B1 — Görsel faza irtifa tabanı
`control/guidance/visual_lead.py` (veya `adapter_copter`)

*Neden:* GPS fazında `LOOKUP_MIN_ALT = 8 m` yere çakılma koruması var
(`gps_guidance.py`); **görsel fazda hiç yok**. Kara kutu: irtifa 8.0 → 0.2 m,
4 m/s alçalışla, ardından `|roll| > 90°`.
*Nasıl:* dikey komut, drone tabana yaklaştıkça **yumuşak** kırpılacak. Sert
kesme terminal dalışı bozar.
*Ölçüt:* zemine çarpma 3/3 → 0.
*Sonuç:*

### B5 — Fly-past davranışı
`control/guidance/visual_lead.py` (+ muhtemelen `supervisor.py`)

*Neden:* drone hedefi geçince "hedefe uç" komutu **yukarı-geriyi** gösterir,
drone tırmanır, hedef kadrajdan çıkar, tespit kopar. **Her ıskadan sonra
5-7 m yukarı fırlıyor, toparlaması 10-20 s.**

⚑ **Ölçüldü — asıl kusur burada.** Log `00000108`: vuruş anından itibaren
kesintisiz **14.57 tur dönüş (~350 °/s)**, irtifa 21.8 → 2.0 m. `DesYaw` aynı
rampayı izliyor, yani **komut kesilmemiş**. Güdüm CSV'yi kapatıyor ama
**son yaw-hızı komutu araçta yaşamaya devam ediyor** (MAVLink hız komutu
kalıcıdır; göndermeyi bırakmak "dur" demek değildir).

*Ne lazım:*
- [ ] **Faz biterken `send_velocity(conn, 0,0,0, mevcut_yaw)` gönder** — en
      küçük ve en etkili parça. Her `return` yolunda (vuruldu/kayip/durduruldu).
- [ ] Fly-past tespiti: menzil < ~3 m VE **büyüyor**, ya da `u_govde[0] < 0`.
- [ ] Davranış: terminal hamleyi bırak, tırmanmayı kes, kontrollü şekilde
      istasyon geometrisine dön. Kör tırmanışı sürdürme.
- [ ] Gerekirse "yeniden hücum" sayacı.
*Ölçüt:* geçiş sonrası irtifa aşımı ve zemine çarpma 0.
*Sonuç:*

> 2026-08-05'te eklenen GPS yaw kaçağı koruması (G12/G13) bu maddenin
> **faz içi** kısmını çözdü. Kalan: **faz bitişi** ve fly-past manevrası.

### B8 — Frenleme eğrisi: mesafeye bağlı hız tavanı
`control/guidance/gps_guidance.py` → `V_MAX` kullanımı

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
- [ ] **Görsel faza geçiş kapısı** — şu an "son 15 karenin 10'unda pose
      conf ≥ 0.5". `KILIT_N=7` denendi, kötüleşti (bkz. DURUM.md). Kilit
      **zaman aşımı** (menzil kapısı içinde N saniye kilit gelmezse devri
      zorla) hiç denenmedi.
- [ ] **Lead'in yumuşak geçişi** — `kpt_dusuk`'ta sert 0'lanıyor, ~15° nişan
      sıçraması yaratıyor. Rampalı geçiş.
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
