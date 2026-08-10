# TODO — yapılacak işlerin tamamı

Sistemin şu anki hâli ve ölçülmüş gerçekler → **[DURUM.md](DURUM.md)**.
GPS güdümünün kararlı referansı → **[KARARLI_HAL.md](KARARLI_HAL.md)**.
Görsel güdümün güncel deney günlüğü → **[UYGULANACAK.md](UYGULANACAK.md)**.

> ## ⚠ 2026-08-10 — DAL VE YASA DEĞİŞTİ, ÖNCE BUNU OKU
>
> `kayramin_super_gudumu` bu dala merge edildi (21 commit) ve **bundan sonra
> onun hattından devam ediliyor.** İki sonucu var, ikisi de bu dosyanın yarısını
> etkiliyor:
>
> **1. Aktif görsel yasa artık `bbox_ibvs.py`** (`AVCI_VISUAL` varsayılanı
> `bbox`). `visual_lead.py` + `adapter_copter.py` + `guidance_core`'un komut
> yolu **uçmuyor** — yalnız `AVCI_VISUAL=lead` ile açılan alternatif kol.
> Ölçüldü: `bbox_ibvs` `guidance_core`'dan **yalnız `KAMERA_TILT_DEG`**'i
> okuyor, başka hiçbir şeyini değil.
>
> ⛔ Dolayısıyla `AVCI_IBVS_COALT_DEG`, `AVCI_IBVS_PN_YATAY_KAPI`,
> `AVCI_IBVS_PN_YATAY_MAX`, `AVCI_IBVS_IVME_TAVAN`,
> `AVCI_IBVS_ILK_KARE_LIMIT`, `AVCI_IBVS_BITIR_TAM_DUR` **uçan koda
> değmiyor.** Bunlarla A/B koşmak uçuş harcamaktır. Hepsi
> `guidance_core`/`adapter_copter` içinde. Aktif yasanın kendi anahtarları
> ayrı ve panelde: `AVCI_IBVS_ROLL`, `AVCI_IBVS_KAPANMA`,
> `AVCI_IBVS_LEAD_ERKEN`, `AVCI_IBVS_KD`, `AVCI_IBVS_KVZD`…
>
> **2. Çalışma yöntemi artık [CLAUDE.md](CLAUDE.md)'de ve bağlayıcı**
> (Kayra `1b458f3`, kullanıcı kararı). Özeti: her özellik **en az 4 uçuşla**
> test edilir (kare + video + log çaprazlanır) · ölçütler koşmadan **önce**
> ilan edilir · A/B **dönüşümlü** koşulur · **eski log replay'i, yalnız CSV
> istatistiği ve tek/iki koşu KANIT SAYILMAZ** · eklenen her davranış
> anahtarının **panelde aç/kapa düğmesi** olur (CLAUDE.md §5).

**Çalışma kuralı:** tek seferde tek değişken → testler → **uç** → ölç →
*Sonuç:* satırına yaz → tikle. Ayrıntısı CLAUDE.md'de.

**İŞ BÖLÜMÜ (08-08 kullanıcı kararı, 08-10'da daraldı):** GPS tarafı
**Kayra'da** (`gps_guidance.py`, `frpn*.py`) — oraya DOKUNULMAZ.
Görsel güdümün **aktif yasası da (`bbox_ibvs.py`) artık Kayra'nın hattı**;
oraya dokunmadan önce onun UYGULANACAK.md'deki deney sırasına bakılır.
Bizde net kalanlar: **arayüz**, **`tools/`**, **senaryolar
(`run_plane_scenario.py`)**, **`supervisor` faz hakemliği**, testler.

**Bu dosya başlık başlık ayrılmıştır** (2026-08-09). Bir madde yalnız TEK bir
başlıkta bulunur; kronolojik §0/§0b/§0c… bölümleri dağıtıldı.

---

## ⚑ Öncelik sırası

Sıra ÖNEM sırasıdır; sağdaki sütun dosyadaki bölüm numarasıdır.

| önem | iş | neden şimdi | bölüm |
|---|---|---|---|
| **1** | **Dönüş bütçesi** — Ö6 (`ANGLE_MAX` 45→55°) · Ö5 (dönüş-farkında hız tavanı) | Kayra'nın son ölçümü: hedef 2.1× hızlı, **2.5× dar** dönüyor. Eskiden "bu daldan çözülemez" denen kapalı yol; **bu dal artık o hat** | §1 |
| **2** | **Ö1 kaçış telafisi kararı** | ölçüm bitti, birincil ölçüt yanlış seçildiği için karar kullanıcıya bırakıldı; varsayılan KAPALI bekliyor | §2 |
| **3** | **M2 tespit eşiği histerezisi** (yakala 0.35 / tut 0.20) | 0.15 sabit eşik takibi 3× iyileştirdi, vuruş getirmedi; histerezis hiç denenmedi | §3 |
| **4** | **Görsel faza irtifa tabanı (B1)** — `bbox_ibvs`'te **hiç yok** | GPS'te `LOOKUP_MIN_ALT=8 m` var; kara kutuda irtifa 8.0 → 0.2 m, ardından `\|roll\|>90°` | §6 |
| **5** | **FRPN A/B** | kullanıcı isteği; GPS koduna dokunmadan, tek ortam değişkeniyle | §4 |
| **6** | **Faz salınımı** — yeni yasada YENİDEN ölçülmeli | eldeki ölçüm `visual_lead` dönemine ait, artık geçersiz | §7 |

---

## ✅ UYGULANANLAR

Kodda ve ölçülmüş. Kanıtı yanında; ayrıntı DURUM.md'de.

### 2026-08-10 — `kayramin_super_gudumu` merge'ü ile GELENLER

Kayra'nın hattından; kanıtları onun commit mesajlarında ve UYGULANACAK.md'nin
üst bölümünde. **Bu dalda artık uçan kod budur.**

- **T1a — yatay roll/pitch telafisi** (`AVCI_IBVS_ROLL`, varsayılan AÇIK).
  Kök neden bir çerçeve karışıklığıydı: `los_az = iris_yaw + eps_yaw`
  varsayımı yalnız roll=0'da doğru. 5869 gerçek kare: yatış 30-39°'de açı
  hatası **13.9°**, düz uçuşta 0.6° — kullanıcının "düz uçuşta ıskalamıyor,
  manevrada sapıtıyor" gözleminin birebir sayısı. Telafiden sonra ağırlıklı
  hata **2.0° → 0.0°**. Altı taze uçuşla doğrulandı (6/6 ölçüt lehine).
  ⚠ Manevrayı **çözmüyor**, yalnız bir katmanını kaldırıyor.
- **Dikey komut kapanma hızıyla ölçeklenir** (`AVCI_IBVS_KAPANMA`).
  `vz = −v_drone·tan(elev)` → `vz = −ṙ·tan(elev)`. Dikey farkı kapatma
  süresini drone'un yer hızı değil KAPANMA hızı belirler; hedef kaçtığı için
  fark saniyede 18 m değil ~2 m kapanıyordu → komut **13.7 kat** fazlaydı.
  ṙ görüntüden ölçülüyor (kutu büyüme oranı), GPS girmiyor.
- **Terminal dikey sönümleme** (`K_VZ_D=0.6`). Terminal dikey kanalı saf
  nişanlamaydı, türev terimi yoktu → dikey momentum geç sönüyor, "son anda
  üstten geçme". Desen ölçüldü: son 8 karede kutu kayması **+162 px → +98/
  +136/−1/+6 px**.
- **Dikey bütçe kısıtı** — yatay komut, dikey talebi karşılayacak kadar kısılır
  ki hız vektörü hedefe bakabilsin.
- **Lead menzille söner** (`AVCI_IBVS_LEAD_SON`). Sabit 0.4 s'lik lead,
  menzil→0'da patlayan LOS hızıyla çarpılınca nişanı yukarı savuruyordu.
- **Faz girişi yaw sıçraması + YAW SLEW SINIRI** — takla bitti, bir ölçümde
  5/5 ilk denemede vuruş. (Bu düzeltmenin GPS ayağı bizim yaw çıpalama
  kolumuzun faz girişine uygulanmış hâli; bkz. aşağıda.)
- **Terminal hücum hızı 24 → 18 m/s**, **terminal eşiği 45 → 25 px**.
- **`kurtarma.py`** — takla/kaçak dönme bekçisi: güdüm komutlarını keser.
- **Panelden CANLI özellik aç/kapa** (`/api/gudum_ozellikleri` + 🎛 bölümü).
  `bbox_ibvs.Cfg` bir sınıf ve döngü her karede `cfg.<ALAN>` okuyor → sunucu
  sınıf niteliğini değiştirince **bir sonraki kareden** geçerli; sim yeniden
  kurulmuyor. Yeni özellikte yalnız `gcs_server._OZELLIKLER`'e satır eklenir.
- **`tools/vurus_orani.py`** ve **`tools/kacamak_testi.py`** — ikincisi
  "hedefe sürekli daire çizdirmek verimsiz test" tespitinin cevabı: hedef düz
  uçar, drone temiz kuyruk yaklaşması kurar, eşikte (25 m) hedef belirli bir
  kaçamak yapar. `yok` kolu her kampanyada taban olarak koşulur.
- **Ö1 kaçış telafisi** (`AVCI_IBVS_KD`, varsayılan **KAPALI**) — ölçüldü,
  kararı bekliyor (bkz. SIRADAKİ §2).

### 2026-08-10 — merge'ü YAPARKEN bizim onardıklarımız

- **POSE → KARE köprüsü çevirisi.** Kayra'nın dalında köprü hâlâ pose
  devrindeydi (`wait_pose` / `kayit["pose"]`); bu dalda pose çıkarılmış,
  köprü `wait_kare` / `kayit["det"]` olmuştu. Git bunu **çakışma olarak
  görmedi**: merge olduğu gibi bırakılsaydı `bbox_ibvs` ilk karede `KeyError`,
  `supervisor` tanımsız `wait_pose` adına çarpardı — yani **görsel faz hiç
  çalışmazdı** ve testler sahteleri kendi adlarıyla kurduğu için bunu
  YAKALAMAZDI. Üç dosya tek isim düzlemine çevrildi.
- **`frpn_guidance.status` sözleşmesi.** Kayra `gps_guidance.status`'a
  `tgt_vx/vy/vz` ekledi (görsel devirde "dondurulmuş taşıyıcı" hızı); FRPN'e
  eklenmeyince iki GPS yasasının sözleşmesi ayrıştı (test D2 düştü). FRPN'e
  de eklendi, kendi kestiricisinden dolduruldu.
- **`test_supervisor.py`** artık bbox yasasını da sahteliyor — yalnız
  `run_visual_lead` sahtelendiği için supervisor GERÇEK bbox döngüsünü
  çağırıyor ve S5 sessizce düşüyordu.
- **`_daire_sureli` kapalı çevrime bağlandı** (aşağıdaki irtifa tutucunun
  devamı) — Kayra'nın yeni 15 s bekleme dairesi açık çevrimdi ve düz fazın
  irtifa hedefini her koşuda 9-23 m farklı yerde kilitliyordu.
- **🎛 özellik paneli bu arayüze taşındı** — arayüz bu dalda tümüyle
  değiştiği için Kayra'nın HTML/CSS'i uymuyordu; JS kendiliğinden birleşti,
  kap + stil bu arayüzün değişkenlerine bağlandı.

### 2026-08-09

- **Senaryolara KAPALI ÇEVRİM irtifa tutucu** (`_irtifa_pitch`, PD).
  FBWA'da elevator irtifayı değil PİTCH AÇISINI komut eder; `pitch=0` seviye
  AÇI demek, seviye UÇUŞ değil. Fazla gazla uçak seviye burunla tırmanıyordu
  (**dairede +35…+92 m/dk, hiç oturmuyor**; düzde +13…+59). Bu tek başına bir
  senaryo kusuru değil, **08-08'deki dört A/B'yi birden geçersiz kılan
  karıştırıcının kendisiydi** (kollar 134-175 m farklı irtifada uçtu).
  Yük faktörü payı taban olarak korunur, üstüne PD düzeltmesi biner; gaza
  dokunulmaz. `AVCI_SCN_ALT` ile hedef zorlanabilir → iki kol AYNI irtifada.
- **GPS yaw ÇIPALAMA deney kolu** (`AVCI_GPS_YAW_CIPA`, varsayılan **KAPALI**).
  Komut her karede aracın gerçek başlığından üretilir. 751 log / 380 004 sağlam
  kare: `|yaw_cmd − yaw_gerçek|` medyan 1.0°, p95 62° — 08-05'teki kaçak
  imzası YOK. Ama komut donmuşken araç tam tur atıyor (1.3 s'de 360°, tepe
  486 °/s). Çıpalama bu kusur için tasarlanmadı; cevabı yalnız A/B verir.
  ⚠ **Merge'den sonra kısmen konusuz kaldı:** Kayra faz girişinde
  `cmd_yaw = iyaw` yaptı, yani iki kol artık aynı yerden başlıyor. Geriye
  yalnız "HER karede demirlemek ek fayda veriyor mu" sorusu kaldı.
- **`ab_gecerli_mi.py` "iki kol aynı" durumunda ÇÖKÜYORDU**
  (`sorunlar_damga` atanmadan okunuyordu → `UnboundLocalError`). Tam da en sık
  karşılaşılan hâl.
- **Faz geçişi transientleri kesildi.** İki ayrı kusur:
  - `adapter_copter`: ilk karede `dt=None` → ivme limiti tümüyle atlanıyordu,
    her görsel faz **12-25 m/s basamak** komutuyla başlıyordu. Artık
    `LOCAL_POSITION_NED`'den tohumlanıyor. `AVCI_IBVS_ILK_KARE_LIMIT=off`.
  - `visual_lead._bitir`: faz sonunda 3 kez `send_velocity(0,0,0)`. Artık
    `kayip` → ötelenme korunur, yalnız yaw dondurulur.
    `AVCI_IBVS_BITIR_TAM_DUR=on`. Testler T62/T67/T68/T69/T70.

  ⛔ **08-10 NOTU: ikisi de artık UÇMAYAN yasada.** `bbox_ibvs` kendi ivme
  limitleyicisini `common.limit_acceleration` ile çağırıyor ve
  `adapter_copter`'ı hiç kullanmıyor. Kod duruyor, A/B'si **iptal**.

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

### ~~Dönüş problemi — bu daldan çözülemez~~ → **08-10: BU YOL AÇILDI**

⚠ Bu bölüm `visual_lead` dönemine ait ve **hükmü artık geçersiz.** Gerekçesi
"talebi küçültmek gerek, o da Kayra'nın D0 hattıyla olur" idi — **bu dal artık
o hat.** Kayra da aynı duvara kendi tarafından çarptı ve sayısını koydu
(`93ea734`, ω = g·tan(yatış)/V):

| | hız | yatış | dönüş yarıçapı | dönüş hızı |
|---|---|---|---|---|
| AVCI | 18 m/s | 45° | 33.0 m | 31 °/s |
| **HEDEF** | 15 m/s | 60° | **13.2 m** | **65 °/s** |

Hedef **2.1× hızlı, 2.5× dar** dönüyor. Çıkan iş SIRADAKİ §1'dedir (Ö6 · Ö5).
Aşağıdaki eski ölçüm tarihsel kayıt olarak duruyor:

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

~~⚠ Bu daldan yeni dönüş deneyi açılmayacak — ölçüm yapılır, Kayra'ya iletilir.~~
**08-10: kalktı.** Bu dal Kayra'nın hattıyla birleşti; dönüş deneyi artık
burada açılır (SIRADAKİ §1).

### Donanım — masada yok

Gimbal, kamera montajı, kamera açısı **asla önerilmez** (kullanıcı kararı
08-08). Kamera gövdeye **+25° YUKARI** sabit; çözüm hep yazılımda.

### GPS güdümü — Kayra'da

`gps_guidance.py`, `frpn*.py` ve tüm `AVCI_GPS_*` / `AVCI_FRPN_*` değişkenleri.
Ortam değişkeniyle **denemek** serbest, **kod değiştirmek** değil.

---

## 🔜 SIRADAKİ İŞLER

> Aşağıdaki her iş CLAUDE.md'deki döngüye tabidir: **ölçüt önce ilan edilir,
> en az 4 uçuş, dönüşümlü A/B, video + log çaprazlanır.** Yeni eklenen her
> anahtar `gcs_server._OZELLIKLER`'e de yazılır (panelden aç/kapa — CLAUDE.md §5).

### 1 — Dönüş bütçesi (eski kapalı yol, artık açık)

Kayra'nın son uçuş analizi (`93ea734`) darboğazın **hız yasası değil dönüş
bütçesi** olduğunu gösterdi: manevrada `v_los` 18.0 m/s'de SABİT kalıyor —
yani güdüm yavaşlama istemiyor — ama ulaşılan hız 10-12.6 m/s'ye düşüyor.
**Çöküş komut değil FİZİK.**

- [ ] **Ö6 — `ATC_ANGLE_MAX` 45 → 55°.** İtki payı **2.56×** var (ölçüldü).
      45°'de yarıçap 33.0 m; 55°'de ≈ 23.1 m'ye iner.
      ⚠ Bu bir ArduPilot parametresi → `AVCI_PARM_EK` ile geçirilir ve
      `tools/parm_denetle.py` ile uygulandığı DOĞRULANIR.
      ⚠ Eski "`ATC_ANGLE_MAX` 45 → 50-55, yalpalama izlenmeli" maddesinin
      halefi budur (ERTELENENLER'den buraya taşındı).
      *Ölçüt (önceden ilan):* kaçamak sonrası en yakın menzil medyanı, isabet,
      ≤1 m'ye inen koşu sayısı — hepsi `tools/kacamak_testi.py` `yatay` +
      `capraz` kollarında, `yok` tabanına karşı. *Sonuç:*
- [ ] **Ö5 — dönüş-farkında hız tavanı** (`v ≤ a_max/λ̇`). Hızı LOS dönüş
      talebine bağlar: talep büyüdükçe hız kısılır, yarıçap küçülür.
      Ö6'dan SONRA ve ondan ayrı ölçülür (tek değişken kuralı). *Sonuç:*

⚠ **Ö6 önce.** Ö5 yazılım, Ö6 tek parametre; ucuz olan önce elenir/kabul edilir.

### 2 — Ö1 kaçış telafisi: KARAR BEKLİYOR

Kod hazır, testleri var (B38-B42), panelde düğmesi var, **varsayılan KAPALI**.
8 uçuş 4'e 4 dönüşümlü koşuldu ve sonuç **bölündü**:

| ölçüt | kontrol | Ö1 | kazanan |
|---|---|---|---|
| maks açılan mesafe *(birincil)* | 107.7 m | 116.9 m | kontrol |
| drone min hız *(birincil)* | 12.8 m/s | 11.2 m/s | kontrol |
| en yakın menzil medyanı | 0.68 m | **0.51 m** | Ö1 |
| isabet | 2/4 | **3/4** | Ö1 |
| ≤1 m'ye gelen koşu | 3/4 | **4/4** | Ö1 |

Önceden ilan edilen kural Ö1'i **eliyor**; tüm ikincil ölçütler ve video Ö1
lehine (kontrol koşusunda en yakın geçiş karesinde hedef kadrajda **yok** —
drone kör gidiyor; Ö1'de kutu kilitli).

⚠ Kayra'nın kendi notu: **birincil ölçüt seçimi hatalıydı** — "maks açılan
mesafe" savrulmayı ölçüyor, kesişim kalitesini değil. Sonuca bakıp ölçüt
değiştirmek CLAUDE.md §4'e aykırı olduğu için tek taraflı düzeltilmedi.

- [ ] **Kullanıcı kararı:** (a) doğru birincil ölçütü **şimdi** ilan edip 4'e 4
      yeniden koş, (b) mevcut veriye bakıp varsayılanı aç, (c) kapalı bırak.
      *Sonuç:*

### 3 — M2 tespit eşiği histerezisi

Temas kopuşlarının **%100'ünde hedef hâlâ kadrajın İÇİNDE**; kopuştan önceki
5 karede güven medyanı 0.39, min 0.35 = `CONF_MIN`. Dedektör görüyor, güdüm
eşikte atıyor. Sabit 0.15 ile ölçülen: yatay hata 50.2/44.5 → **17.0/15.5 px**,
temas süresi 37/53 → **88/111 s**. **Vuruş yok.**

Varsayılan yapılmadı çünkü (a) düz uçuş gerilemesi ölçülmedi, (b) düz eşik
yerine histerezis olmalı.

- [ ] **Histerezis:** yakala **0.35**, tut **0.20**. Kod yok, yazılacak.
      Panelde düğmesi olacak (CLAUDE.md §5).
      *Ölçüt (önceden ilan):* düz uçuşta isabet GERİLEMESİN (taban koşusu
      zorunlu) + kaçamak kollarında temas süresi ve en yakın menzil.
      *Sonuç:*

### 4 — FRPN A/B

FRPN bu dalda **hiç uçmadı** (111 GPS logunun 111'i istasyon). Kararlı daldaki
tek ölçüm: 31.1 m (istasyon yasası 29.4 m) — ama o **eski zarfla** alındı.

- A: (varsayılan) istasyon yasası, `RANGE_SET` 8
- B: `AVCI_GPS_GUDUM=frpn` — FRPN, `RANGE_SET` **11 (sabit kodlu)**

⚠ Karıştırıcı: `frpn.py:120` `RANGE_SET = 11.0` sabit, `AVCI_GPS_RANGE`
okumuyor. B kaybederse cevap "hayır", karıştırıcıyı çözmeye gerek yok.
B kazanırsa üçüncü kol `AVCI_GPS_RANGE=11` ile istasyon → yasa mı menzil mi
ayrılır. Kol logdan doğrulanır (`head -1`'de `t_go` → FRPN).
*Sonuç:*

### 5 — Geçersiz A/B'ler: İPTAL + kalıcı yöntem dersi

**08-08'de dairede yapılan DÖRT A/B GEÇERSİZ.** Sebep: hedef uçak daire
senaryosunda irtifa tutmuyor (**+35…+92 m/dk**, hiç oturmuyor); düz uçuşta
başta tırmanıp sonra oturuyor (65 m ve 134 m'de tam 0.0). Aynı oturumdaki
kollar bu yüzden farklı irtifada uçtu:

| A/B | A kolu | B kolu | fark |
|---|---|---|---|
| IVME | 84.8 m | 219.5 m | **134.6 m** |
| PN_KAPI | 52.5 m | 227.6 m | **175 m** |

Hepsi "değişmedi" sonucu vermişti; o sonuçlara güvenilemez.

**⛔ 08-10: BU DÖRT A/B İPTAL — tekrarlanmayacak.** Dördünün de anahtarı
`guidance_core`/`adapter_copter` içinde, yani **artık uçmayan yasada**
(`AVCI_IBVS_COALT_DEG`, `PN_YATAY_KAPI`, `PN_YATAY_MAX`, `IVME_TAVAN`).
Aktif yasa `bbox_ibvs` bunların hiçbirini okumuyor. Koşulursa iki kol da
birebir aynı kodu uçurur ve "değişmedi" sonucu bu kez GERÇEKTEN doğrudur —
ama hiçbir şey öğretmez. `AVCI_VISUAL=lead` ile eski yasaya dönülürse
geçerlilikleri geri gelir; şu an öyle bir plan yok.

**Bu bölümün KALICI değeri, karıştırıcının kendisi ve çözümü:**

- Kök neden **çözüldü** (08-09, bkz. UYGULANANLAR): senaryolara kapalı çevrim
  irtifa tutucu eklendi. `AVCI_SCN_ALT=<m>` ile iki kol AYNI irtifada uçar.
  Yani bundan sonraki A/B'ler bu tuzağa düşmez.
- **Yöntem (her A/B için hâlâ zorunlu):**
  1. **DÜZ senaryo** ya da `tools/kacamak_testi.py` — dairede hedef oturmuyor.
  2. **`AVCI_SCN_ALT` iki kolda AYNI** (yeni; eskiden "gaz slider'ı aynı olsun"
     denip elde tutulmaya çalışılıyordu, tutmuyordu).
  3. **Aynı `gcs.sh` preset'i** — `bbox` `TRACKER`/`LOCK`'u off'a, `takip` on'a
     ZORLAR (`gcs.sh:28-30`). 08-09'un iki koşusu bu yüzden ayrışmıştı.
  4. Her kol için `bash scripts/start_harmonic.sh yeniden`
     (**Ctrl+C YOK** — süreçler `setsid` ile ayrılmış, ulaşmıyor).
     ⚠ Panelden aç/kapa edilebilen anahtarlarda bu adım **gerekmiyor**
     (CLAUDE.md §5) — ama A/B'de kolların başka hiçbir şeyde ayrışmaması için
     yine de temiz kurulum tercih edilir.
  5. Kıyastan önce `tools/ab_gecerli_mi.py` **YEŞİL** demeli; ArduPilot kolu
     varsa `tools/parm_denetle.py` ile uygulandığı doğrulanmalı.

### 6 — Görsel faza irtifa tabanı (B1) — artık `bbox_ibvs`'te

⚠ Bu bölümün eski hâli `visual_lead` ölçümlerine dayanıyordu. **Teşhisin
büyük kısmı Kayra tarafından bbox yasasında ÇÖZÜLDÜ** (terminal dikey
sönümleme + kapanma hızıyla ölçekleme + dikey bütçe — bkz. UYGULANANLAR).
Geriye **tek** madde kaldı ve o hâlâ açık:

- [ ] **B1 — görsel faza irtifa tabanı.** GPS'te `LOOKUP_MIN_ALT = 8 m` var,
      `bbox_ibvs`'te **hiç yok** (kodda arandı: irtifa/zemin kısıtı yok).
      Kara kutu: irtifa 8.0 → 0.2 m, ardından `|roll| > 90°`. Sert kesme
      terminal dalışı bozar, **yumuşak** kırpılmalı. Panelde düğmesi olacak.
      *Ölçüt (önceden ilan):* zemine çarpma 3/3 → 0, isabet GERİLEMESİN.
      *Sonuç:*
- [ ] **B6 — terminal algı SÜREKLİLİĞİ** ⚠ eski ölçüm `visual_lead`'e ait;
      bbox yasasında **yeniden ölçülmeli** (kör dalış oranı `TERM_KOR`
      durumundan okunuyor). *Sonuç:*

Aşağıdaki teşhis tarihsel kayıt; sayıları `visual_lead` dönemine ait:

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

- [x] ~~**B2 — `kilit_kor` sırasında dikey komutu sönümle.**~~ **08-10: Kayra
      bunu bbox yasasında yaptı** — terminal dikey sönümleme (`K_VZ_D=0.6`,
      `AVCI_IBVS_KVZD`) + dikey komutun kapanma hızıyla ölçeklenmesi. Teşhis
      birebir aynıydı ("dikey momentum geç sönüyor, üstünden geçiliyor").
      *Sonuç: desen ölçüldü, son 8 karede kutu kayması +162 px → +98/+136/−1/+6.*
- [ ] **B10 — kalkışta hedefin üstüne çıkma.** 61 taze kalkış: hedef pistteyken
      başlayanlarda aşım **+7.0…+17.9 m**; seyirdeki hedefte medyan ≈ 0.
      Sebep `_chase_thread`'de `target_z` **bir kez** okunuyor.
      Adaylar: (a) hedefin tırmanması bitene kadar bekle *(en güvenli, GPS'e
      dokunmaz)* · (b) `target_z`'yi kalkış boyunca tazele · (c) dikey frenleme
      eğrisi *(⚠ GPS'e dokunur, izin gerekir)*. *Sonuç:*
- (B6 yukarı taşındı — bu bölümün başındaki açık maddeler listesine.)

### 7 — Faz salınımı: YENİDEN ölçülmeli

Eldeki ölçüm (08-09, 7,5 dk): **23 GPS + 22 görsel faz**, 17'si fly-past ile
bitti, 0 vuruş. GPS fazlarının 12'sinde `durum` %100 KILIT, süre medyan 2.78 s.
Devir menzili ikili: yarısı ~19.6 m (gerçek devir), yarısı ~9.6 m
(yeniden-giriş). Kök neden şöyle konmuştu: giriş kapısı `d_h < GATE_MENZIL`,
çıkış fly-past ve `en_yakin < 8 m` iken tetikleniyor → çıkış anında kapı zaten
açık; histerezis/cooldown/minimum faz süresi kodda yok.

⚠ **08-10: bu ölçüm ve kök neden analizi ARTIK GEÇERSİZ.** İki sebeple:

1. Ölçüm `visual_lead` yasasıyla alındı; çıkışın (fly-past) kaynağı olarak
   `visual_lead.py:443-450` gösteriliyor — **o kod artık uçmuyor.**
   `bbox_ibvs`'in kendi çıkış koşulları var (kutu kaybı → `kayip`,
   `TERM_KOR` süre sınırı).
2. Kök neden olarak gösterilen giriş kapısı `d_h < GATE_MENZIL` **varsayılan
   olarak KAPALI** (`GATE_KILIT = AVCI_HYBRID_GATE, varsayılan "0"`), ve
   UYGULANACAK.md'ye göre menzil kapısı D0 ihlali olduğu için bilerek
   kaldırılmıştı. Yani "kapı çıkışta zaten açık" cümlesinin öznesi yok.

- [ ] **Önce ÖLÇ, sonra karar ver:** bbox yasasıyla bir seyir uçuşunda faz
      sayısı, faz süresi dağılımı, devir menzili, `TERM_KOR` oranı.
      Araç: `tools/gudum_karne.py` `[GEÇİŞ]` bölümü.
      Salınım hâlâ varsa aşağıdaki iki aday geçerli; yoksa bölüm kapanır.
      *Sonuç:*

Adaylar (devir alt sınırı **reddedildi**, bkz. ÇÜRÜTÜLENLER):

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
- **A8 — görsel kilit.** ⚠ **B1 OLMADAN UYGULAMAYIN**: bir kez uygulandı,
  kör uçuş %64'e çıktı ve drone zemine çakıldı. (B2 artık uygulandı; kalan
  ön şart B1 = irtifa tabanı, SIRADAKİ §6.)
- ~~**B3 — kilit süresini kısalt** (B2'ye kaba alternatif).~~ **08-10: konusuz
  kaldı**, B2 bbox yasasında yapıldı.
- **B4 — `coalt` kapsamını daralt.** ⚠ `coalt` `guidance_core`'da, yani uçmayan
  yasada — `AVCI_VISUAL=lead`'e dönülmedikçe geçersiz.
- **Gerçek PN (`γ += N·Δλ`)** ⚠ **M3 ile kesişiyor:** Kayra aynı fikri
  "lead'i menzille söndürmek yerine LOS oranıyla ölçekle" diye önerdi, kodladı
  (`AVCI_IBVS_LEAD_ERKEN`) ve 6 uçuşla **nötr** buldu — varsayılan kapalı.
  Yeni bir PN önerisi bu sonucu geçersiz kılan bir gerekçe getirmeli.
- **Dikey PN'i güçlendirme** (eski sonuç: PN yeni tavana da %79 oranında
  çakıldı), **yaw'ı mutlak hedefe slew etme** (⚠ 08-05'te tam TERSİ yapıldı;
  ayrıca 08-10'da Kayra faz girişinde bunu zaten yaptı).
- ~~**`ATC_ANGLE_MAX` 45 → 50-55**~~ → **SIRADAKİ §1'e (Ö6) terfi etti.**
- **Görsel faza geçiş kapısı zaman aşımı** — `KILIT_N=7` denendi, kötüleşti;
  "N saniye kilit gelmezse devri zorla" **hiç denenmedi**.

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
  (Etkisi sınırlı: `GATE_KILIT` varsayılan **kapalı**, kapı hiç bakılmıyor.)
- **Kayra'nın hattıyla yapısal ayrışmamız iki yerde:** (a) bu dalın arayüzü
  tümüyle farklı (Ayşenur sürümü), (b) pose köprüsü bu dalda kaldırıldı,
  onunkinde duruyor. Her senkronda ikisi de elle çözülüyor.
  Açık soru: bu iki farkı ona geri mi verelim (tek hatta buluşmak), yoksa
  taşıma maliyetini sürdürelim mi? **Kullanıcı kararı gerekiyor.**
- **35 m'den dar daireler** hiç denenmedi — orada `IC_KAYMA=14 m` yarıçapın
  %40'ı olur, radyal bedel baskın olabilir.
- **A2 — MAVLink kuyruk boşaltma.** (İris tarafında `BATCH=500` drenajı var;
  `mavlink_listener` tarafı kontrol edilmedi.)
- **A3 — hedef hızı aracın KENDİ saatinden** (şu an GCS varış zamanından).
- **A4 — hedef sıçrama kapısı** (menzil kapısının hedef pozu için olan eşi).
- **A6 — tanılama endpoint'i** `/api/debug/hedef_telem`.
- **Hasar modülünü arayüze bağla** (`/api/hasar` var, panelde yok).
- ~~**Video kayıt butonları.**~~ **✅ 08-10'da geldi** (Kayra `c2f04d6`;
  bu arayüze taşındı). Saniyede 1 kare + tam durum satırı →
  `logs/kayit/ucus_<tarih>/`; kayıtta güdüm modu ve manuel kumanda konumları
  da var.
- **RTF'i tam sistemde tekrar ölç** (0.982 ölçülmüştü; takipçi ve yeni model
  sonrası tekrar).
