# TODO — yapılacak işlerin tamamı

Sistemin şu anki hâli ve ölçülmüş gerçekler → **[DURUM.md](DURUM.md)**.
GPS güdümünün kararlı referansı → **[KARARLI_HAL.md](KARARLI_HAL.md)**.

**Çalışma kuralı:** tek seferde tek değişken → testler → **uç** → ölç →
*Sonuç:* satırına yaz → tikle.

**İŞ BÖLÜMÜ (kullanıcı kararı 08-08):** GPS tarafı **Kayra'da**
(`gps_guidance.py`, `frpn*.py`) — oraya DOKUNULMAZ, şart olursa önce
kullanıcıya sorulur. Görsel güdüm **bizde** (`guidance_core`, `visual_lead`,
`adapter_copter`, `supervisor`, arayüz, `tools/`).

**Bu dosya başlık başlık ayrılmıştır** (2026-08-09). Bir madde yalnız TEK bir
başlıkta bulunur; kronolojik §0/§0b/§0c… bölümleri dağıtıldı.

---

## ⚑ Öncelik sırası

| # | iş | neden şimdi | bölüm |
|---|---|---|---|
| **1** | **Faz geçişi A/B'si** — kod hazır, uçuş bekliyor | 08-09'da ölçülen en büyük tek kusur; diğer tüm A/B'lerin ölçütünü de bozuyor | [🔜](#1--faz-geçişi-transientleri-ab-uçuş-bekliyor) |
| **2** | **FRPN A/B** | kullanıcı isteği; GPS koduna dokunmadan, tek ortam değişkeniyle | [🔜](#2--frpn-ab) |
| **3** | **Geçersiz A/B'leri tekrarla** (COALT · PN_KAPI · PN_MAX · IVME) | dördü de dairede yapıldı, hedef tırmandığı için ölçüm çöp | [🔜](#3--geçersiz-ab-leri-tekrarla) |
| **4** | **Terminal dikey geometri** — drone hedefin ÜSTÜNDE kalırsa ıska **0/6** | düz uçuşta ölçüldü, dar ve çözülebilir | [🔜](#4--terminal-dikey-geometri-drone-üstte-kalıyor) |
| **5** | **Faz salınımının kalanı** (histerezis / cooldown) | 1 numaradan sonra yeniden ölçülmeli — belki kendiliğinden düzelir | [🔜](#5--faz-salınımının-kalanı) |

---

## ✅ UYGULANANLAR

Kodda ve ölçülmüş. Kanıtı yanında; ayrıntı DURUM.md'de.

### 2026-08-09

- **Faz geçişi transientleri kesildi.** İki ayrı kusur, ikisi de bizim alanda:
  - `adapter_copter`: ilk karede `dt=None` olduğu için ivme limiti tümüyle
    atlanıyordu → her görsel faz **12-25 m/s basamak** komutuyla başlıyordu
    (08-09'da altı fazın altısında ölçüldü). Artık `LOCAL_POSITION_NED`'deki
    gerçek hız okunup limitleyici ondan **tohumlanıyor**, sonra nominal dt ile
    limitleniyor. Tohumlanamazsa (telemetri yok) eski davranış korunur —
    yanlış referanstan limitlemek basamaktan kötü olurdu (sahte fren).
    Kill-switch `AVCI_IBVS_ILK_KARE_LIMIT=off`. Testler T69/T70.
  - `visual_lead._bitir`: faz sonunda 3 kez `send_velocity(0,0,0)`. Ölçülmüş
    gerekçesi bir **yaw** kaçağıydı (log `00000108`: 14.57 tur); sıfır
    ötelenme onun bedeliydi, amacı değil. Artık `vuruldu`/`durduruldu` →
    tam duruş (doğru), `kayip` → ötelenme korunur, yalnız yaw dondurulur.
    Kill-switch `AVCI_IBVS_BITIR_TAM_DUR=on`. Testler T62/T67/T68.
- **GPS fazında vuruş raporlanıyor** (kullanıcı isteği). Vuruş hep görsel
  fazdan sonra olur — ama avcı hedefe kameranın göremeyeceği kadar yaklaşınca
  faz `kayip` ile biter ve **çarpma GPS fazına düşer**; o pencerede olan temas
  hiçbir yerde raporlanmıyordu, görev sonsuza kadar dönüyordu. `supervisor`'ın
  `izci` thread'i artık GPS fazı boyunca `carpisma_state`'i izliyor.
  **Ölçüt yalnız Gazebo contact sensörü** (`kaynak_var()` + `temas_var()`);
  yakınlık yedeği BİLEREK yok — GPS fazı hedefin 8-10 m gerisinde durmak üzere
  kurulu, mesafeye bakmak sahte vuruş üretirdi. GPS koduna dokunulmadı.
  Kill-switch `AVCI_GPS_VURUS=off`. Testler S1-S5 (`tests/test_supervisor.py`,
  supervisor'ın ilk test dosyası).
- **`ab_gecerli_mi.py` yapılandırma damgalarını da kıyaslıyor.** İki kol
  sınanan değişkenden başka bir alanda da farklıysa A/B tek değişkenli
  değildir. İlk yakaladığı: 08-09'un iki koşusunda `TRACKER`/`LOCK` bir kolda
  on, diğerinde off'tu. Tavsiye metni artık teşhise ÖZGÜ (eskiden her sorunda
  "DÜZ senaryo kullan" diyordu; kullanıcı düz uçtuğu hâlde bunu gördü).
- **Damga yalanı düzeltildi.** `visual_lead.py`'de `GPS_RANGE` varsayılanı elle
  `'11.0'` yazılıydı; gerçek değer `gps_guidance.Cfg.RANGE_SET`'te 08-08'de
  8'e inmişti. Araç 8 ile uçarken **log 11 yazıyordu**. Artık kaynağından
  okunuyor, ikinci varsayılan yok (bekçi: T71).
- **T62 fiilen BOŞ bir testti.** Hız alanı diye paketin `[5:8]`'ine bakıyordu;
  orası KONUM alanı ve `send_velocity` onu sabit `0.0` gönderiyor — ne
  gönderilirse gönderilsin geçiyordu. 14.57 turluk kusurun bekçisi
  çalışmıyormuş. Doğru dilim `8:11`.
- **`test_gudum_karne.py` 3 gündür çöküyordu** (`pose_orani_%`, 08-06'da
  kaldırılan metrik) → K5-K8 hiç koşmuyordu. Düzeltildi.
- **Araçlar faz salınımını raporluyor.** `gudum_karne.py` yeni `[GEÇİŞ]`
  bölümü: faz sayısı, GPS fazı başlangıç hızı, faz hızına ulaşamayan oranı,
  kısa faz / fly-past / yeniden-giriş oranları + eşik aşılınca uyarı.
- **`tools/ab_gecerli_mi.py`** — A/B kollarının kıyaslanabilirliğini kıyastan
  ÖNCE denetler; **uçuş** bazlı gruplar (`gudum_karne.ucuslar()`), GPS yasasını
  CSV sütunlarından tanır (`t_go` → frpn, `ist_elev_deg` → istasyon).
- **`tools/yaw_spektrum.py`** — `.BIN`'den gyro/açı spektrumu, COPTER kaydını
  kendi bulur (`son`/`son2`).
- **`AVCI_PARM_EK`** kancası (`start_harmonic.sh`) — ArduPilot parametre A/B'si
  artık dosya düzenlemeden, tek ortam değişkeniyle.
- **`LOG_BITMASK 442367`** (ATTITUDE_FAST + IMU_FAST) — teşhis; varsayılan
  kayıt hızı 8 Hz'lik yaw çevrimini 2 Hz'e alias'lıyordu.
- **`supervisor.KAYIP_M` ölü sabit** olduğu belgelendi (gerçek eşik
  `guidance_core.KAYIP_PENCERE=40 / KAYIP_MIN_ISABET=4`).

### 2026-08-08

- **`RANGE_SET` 11 → 8** (C1, Kayra'nın dalından cherry-pick `6d7854b`).
  Daire 15.1 → **13.3 m**, düz 13-14 → **10.3 m** [p10 9.8, p90 10.8].
  Yan etki: dönüşte kadraj −9.4 → −15.2° geriledi.
- **Yatay lead kapısı `kalite` → `azimut_kalite`.** Mekanizma çalıştı
  (lead 0° → 20°), **sonucu değiştirmedi** — ama o A/B geçersiz (aşağı bak).
  Kill-switch `AVCI_IBVS_PN_YATAY_KAPI=olcek`. Testler T64-T66.
- **`AVCI_IBVS_COALT_DEG`** env'e açıldı (eskiden sabit 10°).
- Arayüze **"Düz Uç"** butonu + `start_harmonic.sh yeniden` komutu.

### 2026-08-07 ve öncesi

- **`gps_kararli_hal` TAM merge** (`2125e23`). Pose **kesin olarak çıkarıldı**,
  görsel yığın bu dalda kaldı, GPS tarafı kararlı daldan geldi.
  Merge'ün üç **sessiz** tuzağı yakalandı (pose aygıtının çakışmasız sızması,
  `gcs_server`'da iki davranış geri alması) — `pytest` bunları YAKALAMADI,
  dosyaların kendi koşucuları yakaladı.
- **Pose modeli kaldırıldı** (08-06) → görsel güdüm yalnız bbox.
  Arşiv: [`POSEA_GERI_DONMEK_ISTERSENIZ/`](POSEA_GERI_DONMEK_ISTERSENIZ/README.md)
- **B5 — fly-past tespiti + faz sonu komut sıfırlama** (08-06). İki bağımsız
  imza: menzil döndü (`FLYPAST_MENZIL=8`, `FLYPAST_BUYUME_M=1.5`) ve hedef
  arkada (`u_govde[0] < 0`). Testler T60/T61/T63.
- **B9 — dikey hız tavanı** `VZ_TERMINAL_MAX = 12` (08-05). Test T56.
- **Yaw susturma kilidi** `YAW_SUS_N` — süreli susma. Ölçüm: karelerin
  %7.8'inde burun >20° sapmışken adım 0, en uzun susma **93 s**.
- **Daire çapı butonları** arayüze (`circle_xl` ⌀96 · `circle_l` ⌀71 ·
  `circle` ⌀55 · `circle_s` ⌀41).
- **Uçuş bekçisi** (`tools/ucus_bekci.py`) `gcs.sh`'e bağlandı.

---

## ❌ ÇÜRÜTÜLENLER — tekrar denemeyin

Gerekçesi ölçümle çürütülmüş fikirler. **Yeniden önermeden önce buraya bakın.**

| fikir | ne oldu | tarih |
|---|---|---|
| **`V_KAPANMA` 25 → 12 m/s** | menzil 16 → **89 m**'ye açıldı (hedef 21 m/s, mutlak hız tavanı yetmiyor). `V_YAKLASMA` yorumunda **zaten yazılıydı**, okunmadan denendi | 08-08 |
| **`ATC_ANG_YAW_P` 4.5 → 3.0** | yaw hatası std 1.36° → **4.94°**, toplam **11.96 TUR** sürüklenme. Kazancı düşürmek komut edilen başlığı yakalamayı yavaşlattı | 08-05 |
| **Dönüş ileri beslemesi** (`v_ist = v_hedef + ω×r`) | formül doğru (G14b ile birebir) ama daire 15.1 → **23.0 m açıldı**. Eski `v_hedef` fazlalığı kazara faydalı lead'miş. `AVCI_GPS_FF_DONUS` kapalı | 08-08 |
| **Yarıçap-oranlı iç daire kayması** (`IC_ORAN=0.27`) | dört çapta ölçüldü, **sabit 14 m her yerde kazandı**. Oranlı sürüm ⌀96'da fazla (25 m), ⌀41'de az (11 m) kaydırıyor | kararlı dal |
| **`AVCI_GT_KILIT_BYPASS=on`** | **13/13 KAYIP**. GT modunda görsel kilidi atlamak mantıklı görünüyordu, uçuşta çöktü | 08-04 |
| **`YAW_HIZ_MAX` 1080 °/s** | tespit gürültüsü doğrudan gövdeye geçti | 07-25 |
| **`KP_KADRAJ` ≥ 1.0** | 1.5'te **0/24** vuruş | — |
| **`PN_SURE = 0`** | geometri İYİLEŞTİ (kilitli %9 → %25) ama vuruşa dönüşmedi (1/16) | 08-07 |
| **Kesişme noktası nişanı** | masa üstü doğrulamada saf takipten **hiç** iyi çıkmadı | 08-08 |
| **"Kamera yere bakar, tespit bozulur"** (`IVME_TAVAN=4` gerekçesi) | 8 m/s² ile araç 13° burun aşağı gitti, tespit güveni 0.81 → **0.80**. Korku ÇÜRÜDÜ (ama kapanma da olmadı) | 08-08 |
| **`ATC_RAT_YAW_NEF` ile 8 Hz çentiği** | **frekans değil, filtre bankası indeksi** (`AC_PID.cpp:183`, `@Range 0 8`). Gerçek çentik `FILT1_TYPE=1` + `@RebootRequired` ister, tek `.parm` geçişinde kurulamaz. *Uçuş harcamadan, kaynak okunarak elendi* | 08-09 |
| **Devir menzili ALT sınırı** | 1.87 m'de alınan devir uzaktan verilmiş tuhaf bir karar değil: önceki GPS fazının izi drone'un hedefin **yanında durduğunu** gösteriyor (`d_h 2.87 → 8.9 → 3.49`). Semptoma dokunur, sebebe değil. **Kullanıcı reddetti** | 08-09 |

---

## ⛔ KAPALI YOLLAR

### Dönüş problemi — bu daldan çözülemez

Görsel faz dönüşte **84 °/s** LOS talebi üretiyor; araç 8 m/s²'de 17 °/s
dönebiliyor. Kapatmak **32 m/s² = 3.3 g** ister, quad tavanı 1 g.

| | LOS'un dönme hızı |
|---|---|
| GPS fazı (15-25 m) | 28 °/s |
| **görsel faz (11-19 m)** | **84 °/s** |
| aracın yapabildiği (8 m/s²) | 17 °/s |

**Talebi güdüm KENDİ üretiyor:** aynı uçuş, aynı hedef, benzer menzil — GPS
hedefle birlikte döndüğü için 28 °/s, görsel faz hedefin üstüne dalıp 84 °/s'lik
talebi kendi yaratıp sonra yetişemiyor. Dört deney (lead kapısı · lead tavanı ·
kapanma hızı · ivme tavanı) hepsi "talebe daha iyi yetiş" dedi.
**Yapılması gereken talebi küçültmek** — ve o, devir anındaki geometriyi
düzeltmekle olur: Kayra'nın geometrik devir kapısı hattı (D0, `1c00deb`).

Dönüş yarıçapı = V²/a. Hedefe yetişmek >21 m/s ister; 21 m/s'te drone'un dönüş
yarıçapı **110 m**, hedefin **27.5 m**. **Hiçbir SABİT hız ikisini birden
sağlamıyor.**

⚠ **Bu daldan yeni dönüş deneyi açılmayacak** — ölçüm yapılır, Kayra'ya iletilir.

### Donanım — masada yok

Gimbal, kamera montajı, kamera açısı **asla önerilmez** (kullanıcı kararı
08-08). Kamera gövdeye **+25° YUKARI** sabit; çözüm hep yazılımda.

### GPS güdümü — Kayra'da

`gps_guidance.py`, `frpn*.py` ve tüm `AVCI_GPS_*` / `AVCI_FRPN_*` değişkenleri.
Ortam değişkeniyle **denemek** serbest, **kod değiştirmek** değil.

---

## 🔜 SIRADAKİ İŞLER

### 1 — Faz geçişi transientleri A/B (uçuş bekliyor)

Kod uygulandı (bkz. UYGULANANLAR), A/B'si yapılmadı.

- A: `AVCI_IBVS_ILK_KARE_LIMIT=off AVCI_IBVS_BITIR_TAM_DUR=on` (eski davranış)
- B: varsayılan (yeni)

*Ölçüt* (`gudum_karne` `[GEÇİŞ]` bölümü): GPS fazı başlangıç hızı **0.16 m/s'ten
belirgin yukarı**, 15 m/s'e ulaşamayan oran **%39'un altına**, faz sayısı,
en yakın menzil, vuruş.
*Sonuç:*

### 2 — FRPN A/B

FRPN bu dalda **hiç uçmadı** (111 GPS logunun 111'i istasyon). Kararlı daldaki
tek ölçüm: 31.1 m (istasyon yasası 29.4 m) — ama o **eski zarfla** alındı.

- A: (varsayılan) istasyon yasası, `RANGE_SET` 8
- B: `AVCI_GPS_GUDUM=frpn` — FRPN, `RANGE_SET` **11 (sabit kodlu)**

⚠ Karıştırıcı: `frpn.py:120` `RANGE_SET = 11.0` sabit, `AVCI_GPS_RANGE`
okumuyor. B kaybederse cevap "hayır", karıştırıcıyı çözmeye gerek yok.
B kazanırsa üçüncü kol `AVCI_GPS_RANGE=11` ile istasyon → yasa mı menzil mi
ayrılır. Kol logdan doğrulanır (`head -1`'de `t_go` → FRPN).
*Sonuç:*

### 3 — Geçersiz A/B'leri tekrarla

**08-08'de dairede yapılan DÖRT A/B GEÇERSİZ.** Sebep: hedef uçak daire
senaryosunda irtifa tutmuyor (**+35…+92 m/dk**, hiç oturmuyor); düz uçuşta
başta tırmanıp sonra oturuyor (65 m ve 134 m'de tam 0.0). Aynı oturumdaki
kollar bu yüzden farklı irtifada uçtu:

| A/B | A kolu | B kolu | fark |
|---|---|---|---|
| IVME | 84.8 m | 219.5 m | **134.6 m** |
| PN_KAPI | 52.5 m | 227.6 m | **175 m** |

Hepsi "değişmedi" sonucu vermişti; o sonuçlara güvenilemez.

- [ ] `AVCI_IBVS_COALT_DEG` 0 vs 10 — **düz uçuşta**, tek fazlık vuruş kanıt
      değil. *Sonuç:*
- [ ] `AVCI_IBVS_PN_YATAY_KAPI` azimut vs olcek. *Sonuç:*
- [ ] `AVCI_IBVS_PN_YATAY_MAX` 20 vs 60. *Sonuç:*
- [ ] `AVCI_IBVS_IVME_TAVAN` 4 vs 8. *Sonuç:*

**Yöntem (zorunlu):**

1. **DÜZ senaryo** — dairede hedef hiç oturmuyor (+35…+92 m/dk).
2. **Gaz slider'ı iki kolda AYNI değerde.** (08-09'da öğrenildi:) hedefin
   irtifasını slider belirliyor — `scenario_duz` `pitch=0` ile seviye uçuyor,
   fazla itki tırmanışa gidiyor (`run_plane_scenario.py:207-215` →
   `gcs_throttle()`). Aynı gün iki koşu bu yüzden 19 m ve 74 m'de uçtu.
   Ölçülen tavan **133.8 m**.
3. **Aynı `gcs.sh` preset'i** — `bbox` `TRACKER`/`LOCK`'u off'a, `takip` on'a
   ZORLAR (`gcs.sh:28-30`). 08-09'un iki koşusu bu yüzden de ayrışmıştı.
4. Her kol için `bash scripts/start_harmonic.sh yeniden`
   (**Ctrl+C YOK** — süreçler `setsid` ile ayrılmış, ulaşmıyor; komut zaten
   kendi içinde durduruyor).
5. Kıyastan önce `tools/ab_gecerli_mi.py` **YEŞİL** demeli; ArduPilot kolu
   varsa `tools/parm_denetle.py` ile uygulandığı doğrulanmalı.

### 4 — Terminal dikey geometri: drone ÜSTTE kalıyor

Son görülen karedeki gövde-çerçevesi yükselti açısı:

| | vuruş | ıska |
|---|---|---|
| drone hedefin **ÜSTÜNDE** (6 faz) | **0** | **6** |
| drone hedefin **ALTINDA** (6 faz) | **3** | 3 |

Üstteki 6 fazın hepsinde kadraj hatası **−50…−54°**. 9 ıskanın **6'sı ALT
kenardan** çıkıyor; üç vuruşta hiç çıkmıyor.

**Mekanizma:** kamera +25° YUKARI → sistem baştan "alttan yaklaş" üzerine kurulu
(GPS istasyonu da altta). Drone bir kez üste çıkarsa kamera onu göremez → kör
dalış → ıska. Üste çıkmanın kaynağı fly-past: hedefi geçince "hedefe uç" komutu
yukarı-geriyi gösteriyor.

Bağlı maddeler:

- [ ] **B2 — `kilit_kor` sırasında dikey komutu sönümle.** Kör dalışta dikey
      komut serbest kalıyor; 1'in doğrudan mekanizması. *Sonuç:*
- [ ] **B1 — görsel faza irtifa tabanı.** GPS'te `LOOKUP_MIN_ALT = 8 m` var,
      görsel fazda **hiç yok**. Kara kutu: irtifa 8.0 → 0.2 m, ardından
      `|roll| > 90°`. Sert kesme terminal dalışı bozar, **yumuşak** kırpılmalı.
      *Ölçüt:* zemine çarpma 3/3 → 0. *Sonuç:*
- [ ] **B10 — kalkışta hedefin üstüne çıkma.** 61 taze kalkış: hedef pistteyken
      başlayanlarda aşım **+7.0…+17.9 m**; seyirdeki hedefte medyan ≈ 0.
      Sebep `_chase_thread`'de `target_z` **bir kez** okunuyor.
      Adaylar: (a) hedefin tırmanması bitene kadar bekle *(en güvenli, GPS'e
      dokunmaz)* · (b) `target_z`'yi kalkış boyunca tazele · (c) dikey frenleme
      eğrisi *(⚠ GPS'e dokunur, izin gerekir)*. *Sonuç:*
- [ ] **B6 — terminal algı SÜREKLİLİĞİ** ⚠ raftan indi. Vuran fazlarda kör
      dalış %0/%0/%44, ıskalayanlarda medyan %11 — vuranlar hedefi temasa kadar
      hiç kaybetmedi. Eski çürütme (GT modu isabeti değiştirmedi) algının
      DOĞRULUĞUNU sınamıştı, SÜREKLİLİĞİNİ değil. *Sonuç:*

### 5 — Faz salınımının kalanı

1 numaralı iş bittikten **sonra** yeniden ölçülecek — transientler kesilince
salınım kendiliğinden azalabilir.

Ölçülen (08-09, 7,5 dk): **23 GPS + 22 görsel faz**, 17'si fly-past ile bitti,
0 vuruş. GPS fazlarının 12'sinde `durum` **%100 KILIT** (faz boyunca
`d_h < 20`, devir kapısı hiç kapanmadı), süre medyan 2.78 s. Devir menzili
ikili: yarısı ~19.6 m (gerçek devir), yarısı ~9.6 m (yeniden-giriş).

**Kök neden:** giriş kapısı `d_h < GATE_MENZIL (20 m)`
(`supervisor.py:161-171`); çıkış fly-past ve tanımı gereği `en_yakin < 8 m`
iken tetikleniyor (`visual_lead.py:443-450`) → **çıkış anında kapı zaten
açık**. Yeniden girişi geciktiren tek şey `izci`'nin sıfırdan doldurduğu
10/15 karelik pencere (~2.8 s). Histerezis, cooldown, minimum faz süresi
kodda **yok**.

Kalan adaylar (devir alt sınırı **reddedildi**, bkz. ÇÜRÜTÜLENLER):

- [ ] **Çıkış histerezisi** — fly-past sonrası `d_h > ~30 m` olmadan yeniden
      devir yok. Kök nedene doğrudan cevap. Riski: hedef yakında kalırsa
      görsel faz hiç açılmayabilir. *Sonuç:*
- [ ] **Cooldown** — fly-past sonrası N saniye devir kilidi. Menzilden
      bağımsız, basit. Riski: sabit süre her geometriye uymaz. *Sonuç:*

---

## ⏸ ERTELENENLER

### Yaw limit-çevrimi — 8.0 Hz (kullanıcı isteğiyle ertelendi)

Ölçüldü ve kaldıracı hazır; **vuruş oranını düzeltmesi beklenmediği için**
sıraya alınmadı.

Gerçek frekans **8.0 Hz** — eski "1.1-2.0 Hz" ve "4.3 Hz" rakamları örnekleme
kurbanıydı (ATT yalnız 10 Hz kaydediliyordu, 8 Hz'i 2 Hz'e alias'lıyordu).

| kanal | ölçüm |
|---|---|
| gyroZ tepe | **7.99 Hz**, genlik 80 °/s, 6-10 Hz bandı 68.7 °/s |
| ATT açı std | 1.18-1.60° → 80/(2π·8) = 1.6° ile **birebir** |
| üç eksen | gyroX 55.7 · gyroY 22.2 · gyroZ 70.4 °/s |
| ivmeölçerler | 0.02-0.59 m/s² → dönme var, ötelenme yok |

**HIZ halkasında doğuyor**, üç kanıtla: `RATE.YDes` 5.8 °/s ↔ `RATE.Y` 60 °/s
(10 kat) · açı halkası en fazla 7 °/s besleyebilir · `RCOU` aynı çizgiyi
taşıyor (motorlar sürüyor, Gazebo fiziği değil). Sakin seyirde en güçlü,
agresif manevrada zayıflıyor — limit çevrimi imzası.

- [ ] **`ATC_RAT_YAW_P` 0.30 → 0.15.** Halka kazancını yarıya indirir, faz
      EKLEMEZ. Bedeli yaw hız bant genişliği — ama hedef yalnız 5.8 °/s
      istiyor, marj devasa. `ATC_ANG_YAW_P` denemesinden farkı bu: orası DIŞ
      halkaydı ve marjı yoktu.
      Komut: `AVCI_PARM_EK="ATC_RAT_YAW_P 0.15" bash scripts/start_harmonic.sh yeniden`
      Ölçüt: `tools/yaw_spektrum.py son2` → **6-10 Hz** sütunu. <0.5× tuttu,
      ~1.0 etkisiz, >1.2 kötüleşti (geri al).
      ⚠ Bu dosya avcının ortak ayarı, GPS fazını da etkiler. Kullanıcı kararı:
      denenecek, iyileşme yoksa geri alınacak, iyileşirse Kayra'ya anlatılacak.
      *Sonuç:*

### Diğer ertelenenler

- **B8 — frenleme eğrisi** (`V_MAX`'ı kalan mesafeye bağla). Kullanıcı kararı
  08-06: *"neyse 18'de kalsın şimdilik."* ⚠ GPS'e dokunur.
- **B7 — istasyon açısı 15° / 18° / 20° / 25°.** 25° denendi ve kötüleştirdi
  ama deney kirliydi (`WP_ACC_Z` de aynı anda değişti). 18° ve 20° **hiç
  denenmedi**. ⚠ GPS'e dokunur.
- **`WP_ACC_Z` 2.5 vs 1** ve **`ISTASYON_ELEV_DEG` 25°** — tek değişkenli
  ayrıştırma. ⚠ GPS'e dokunur.
- **A8 — görsel kilit.** ⚠ **B1 ve B2 OLMADAN UYGULAMAYIN**: bir kez uygulandı,
  kör uçuş %64'e çıktı ve drone zemine çakıldı.
- **B3 — kilit süresini kısalt** (B2'ye kaba alternatif).
- **B4 — `coalt` kapsamını daralt.**
- **Gerçek PN (`γ += N·Δλ`)**, **dikey PN'i güçlendirme** (eski sonuç: PN yeni
  tavana da %79 oranında çakıldı), **yaw'ı mutlak hedefe slew etme**
  (⚠ 08-05'te tam TERSİ yapıldı, komut aracın gerçek başlığına demirlendi).
- **`ATC_ANGLE_MAX` 45 → 50-55** — yalpalama izlenmeli.
- **Görsel faza geçiş kapısı zaman aşımı** — `KILIT_N=7` denendi, kötüleşti;
  "menzil kapısı içinde N saniye kilit gelmezse devri zorla" **hiç denenmedi**.

---

## ❓ AÇIK SORULAR — ölçülmemiş

- **`DURUM.md` bayat satırları** (08-09'da bulundu): `:107` `AVCI_TRACKER`
  varsayılanını **off** diyor, kod **on**. `:109` `AVCI_GPS_RANGE` **11.0**
  diyor, kod **8.0**. Belge güncellenmeli.
- **`LOOKUP_MIN_ALT`** şu an 8 m sabit taban. Hedef alçalırsa drone takip
  edemez; hedef irtifasına göreli mi olmalı?
- **Menzil verisi neden zıplıyor** — kapı semptomu kesti, kök neden duruyor.
  Telemetrinin kendi zıplaması araştırılmadı.
- **`gps_guidance.HANDOFF_RANGE=20` ile `SupCfg.GATE_MENZIL=20` ayrı tanımlı** —
  biri env'den değişirse CSV etiketi gerçek kararla sessizce ayrışır.
- **35 m'den dar daireler** hiç denenmedi — orada `IC_KAYMA=14 m` yarıçapın
  %40'ı olur, radyal bedel baskın olabilir.
- **A2 — MAVLink kuyruk boşaltma.** (İris tarafında `BATCH=500` drenajı var;
  `mavlink_listener` tarafı kontrol edilmedi.)
- **A3 — hedef hızı aracın KENDİ saatinden** (şu an GCS varış zamanından).
- **A4 — hedef sıçrama kapısı** (menzil kapısının hedef pozu için olan eşi).
- **A6 — tanılama endpoint'i** `/api/debug/hedef_telem`.
- **Hasar modülünü arayüze bağla** (`/api/hasar` var, panelde yok).
- **Video kayıt butonları.**
- **RTF'i tam sistemde tekrar ölç** (0.982 ölçülmüştü; takipçi ve yeni model
  sonrası tekrar).
