# TODO — yapılacak işlerin tamamı

Bu dosya **yalnız yapılacak işleri** tutar. Sistemin şu anki hali, ölçülmüş
gerçekler ve çürütülmüş fikirler → **[DURUM.md](DURUM.md)**.
GPS güdümünün kararlı referans hâli → **[KARARLI_HAL.md](KARARLI_HAL.md)**.

**Çalışma kuralı:** tek seferde tek değişken → testler → **uç** → ölç →
*Sonuç:* satırına yaz → tikle. (Gerekçesi DURUM.md'de.)

---

## ⚑ Öncelik sırası (2026-08-06 güncellendi)

| # | iş | neden şimdi |
|---|---|---|
| **0** | [`gps_kararli_hal` TAM merge](#0--gps_kararli_hal-tam-merge-kullanıcı-kararı) | kullanıcı kararı — seçmeli entegrasyon yetmedi |
| **1** | [LOS kayması — görsel fazın asıl kusuru](#1--los-kayması-görsel-fazın-asıl-kusuru) | **ölçüldü: ıskaların ortak imzası** |
| **2** | [B10 — kalkışta hedefin üstüne çıkma](#b10--kalkışta-hedefin-üstüne-çıkma) | **her uçuşun başında bozuluyor** (max +17.9 m) |
| **3** | [B1 — görsel faza irtifa tabanı](#b1--görsel-faza-irtifa-tabanı) | çakılmayı doğrudan keser |
| **4** | [Terminal kontrol yetkisi](#terminal-fazda-kontrol-yetkisi) | son metrede 25 m/s = kare başına 0.81 m |
| **5** | [B8 — frenleme eğrisi](#b8--frenleme-eğrisi-mesafeye-bağlı-hız-tavanı) | "18 m/s çok yavaş" — **kullanıcı şimdilik erteledi** |
| 6+ | aşağıdaki diğer maddeler | |

### ✅ 2026-08-06'da BİTENLER (ölçümleri DURUM.md §3'te)

- **Pose modeli kaldırıldı** → görsel güdüm yalnız bbox. Arşiv + geri dönüş
  yolu: [`POSEA_GERI_DONMEK_ISTERSENIZ/`](POSEA_GERI_DONMEK_ISTERSENIZ/README.md)
- **B5 — fly-past + faz sonu komut sıfırlama** (aşağıda ✓)
- **B9 — dikey hız tavanı** → `VZ_TERMINAL_MAX = 12` (2026-08-05'te girmişti)
- **Yaw susturma kilidi** → süreli susma (`YAW_SUS_N`). Ölçüm: karelerin
  %7.8'inde burun >20° sapmışken adım 0, en uzun susma **93 saniye**.

---

## 0 — `gps_kararli_hal` TAM merge (kullanıcı kararı)

**Karar (2026-08-07, kullanıcı):** seçmeli entegrasyon yeterli olmadı; dal
bütün olarak çekilip bu branch'le birleştirilecek. Düzgün çalışmazsa
[§1'deki PN deneyleri](#1--los-kayması-görsel-fazın-asıl-kusuru) yedek plan.

**Şu an ne var:** commit `c6869f2` ile **yalnız GPS güdümü** alındı
(`gps_guidance.py`, FRPN paketi kapalı, senaryo çapları, kamera/telemetri
gecikme düzeltmesi, `avci_copter.parm`). Görsel faz, arayüz ve
`guidance_core.py` bu daldaki hâlinde kaldı.

**Tam merge'ün ek olarak getireceği** (`git diff --stat HEAD origin/gps_kararli_hal`,
toplam 55 dosya / +4252 −8993):

| dosya | fark | ne demek |
|---|---|---|
| `guidance_core.py` | 457 satır | ortak güdüm çekirdeği baştan farklı |
| `visual_lead.py` | 335 satır (çoğu silme) | görsel faz **pose'lu** sürüme döner |
| `gcs_ui/script.js` + `style.css` + `index.html` | ~3700 satır | arayüz tamamen değişir |
| `supervisor.py` | 84 satır | `get_gercek` dahil faz hakemliği |
| `vision/geometry.py`, `detection_state.py` | 180 satır | pose geometrisi geri gelir |
| `tests/test_visual_lead.py` | 745 satır | testler pose sürümüne göre |
| `tools/los_kayma.py`, `gudum_rapor.py` | −760 | **bu daldaki ölçüm araçları silinir** |

**⚠ Merge'de kaybedilmemesi gerekenler** (kararlı dalda karşılığı YOK):

1. **Yaw çıpalama + süreli susma (`YAW_SUS_N=40`).** Kararlı dal hâlâ eski
   kendini-biriktiren `cmd_yaw`'u kullanıyor. Merge sırasında üzerine
   yazılırsa 08-06'da düzeltilen burun sapması geri gelir.
2. **`tools/los_kayma.py`** — §1'in ölçütü. Silinmesin.
3. **`avci_copter.parm` ad şeması.** Kararlı dal `WPNAV_*`/`ANGLE_MAX`/
   `PSC_VELXY_P` kullanıyor; bu makinenin firmware'inde bu adlar YOK
   (ArduPilot bilinmeyen adı sessizce yutar). Bu daldaki SI adları
   (`WP_SPD`, `WP_ACC`, `ATC_ANGLE_MAX`, `PSC_NE_VEL_P`…) korunmalı —
   değerler zaten aynı.
4. **Pose kaldırma kararı.** Kararlı dal pose'lu; `AVCI_POSE_KAYNAK=gercek`
   ve "görsel faz kapalı" ayarlarının bu dalda karşılığı yok. Merge sonrası
   pose geri gelecek mi, yoksa yine bbox'ta mı kalınacak — **merge'e
   başlamadan karar ver**, yoksa iki sistem yarı yarıya karışır.

**⚠ Yukarıdaki tablo YANILTICI (2026-08-07'de merge sırasında anlaşıldı).**
`git diff --stat HEAD origin/gps_kararli_hal` bir İKİ-NOKTA farkıdır, merge
önizlemesi değil: orada "silinecek" görünen satırların çoğu aslında **bu dalın
kendi eklemeleri**. Gerçek gelen yük çok küçük — kararlı dal ortak atadan
(`5bc5f8d`) bu yana yalnız **22 dosya** değiştirmiş ve `c6869f2` onların
çoğunu (frpn, hedef_kestirim, zarf_olc, run_plane_scenario, testler) zaten
almıştı. Fiilî çakışma: **12 dosya / 57 hunk**.

**Sıra:**

- [x] Merge öncesi bu dalı etiketle. *Sonuç:* `merge_oncesi_20260807`
      → dönüş: `git reset --hard merge_oncesi_20260807`
- [x] `git merge origin/gps_kararli_hal` — çakışmalar çözüldü.
      *Sonuç:* aşağıdaki "Merge nasıl çözüldü" tablosu.
- [x] Testler. *Sonuç:* **HEPSİ GEÇTİ** — `pytest tests/ -q` 17/17; ayrıca
      dosyaların kendi koşucuları (pytest bunları ÇALIŞTIRMIYOR, ayrı
      çağrılmalı: `PYTHONPATH=. python3 tests/test_*.py`):
      visual_lead 66/66 · gps_guidance 24/24 · frpn 26/26 ·
      frpn_guidance 15/15 · hedef_kestirim 13/13.
- [ ] Aşağıdaki taban ölçümünü yap (SITL gerekiyor — **yapılmadı**). *Sonuç:*

### Merge nasıl çözüldü (2026-08-07)

**Pose kararı (kullanıcı, merge öncesi):** pose **kesin olarak çıkarıldı**,
takip bbox ile. Dolayısıyla görsel yığın bu dalın hâlinde kaldı; GPS tarafı
kararlı daldan geldi.

| dosya | karar |
|---|---|
| `visual_lead.py`, `guidance_core.py`, `supervisor.py` | **bizim** (bbox/GT) |
| `gps_guidance.py` | **kararlı dal** — farkı yalnız yorum + yeni CSV sütunları (`tgt_ham_*`, `kestirim_gecikme_m`); davranış aynı. `IC_KAYMA` varsayılanı 14 olduğu için "varsayılan 0" diyen yorumda bizimki tutuldu |
| `avci_copter.parm` | **bizim ad şeması** (SI). Değerler zaten aynıydı |
| `gcs_server.py`, arayüz | **bizim** |
| testler | **bizim** (G16/G17 numaralandırması çakışmayı önlüyor) |
| `KARARLI_HAL.md` | §3b **kararlı daldan alındı** — aşağıya bak |

**⚠ TODO'nun 4 maddesi eksikti — merge'ün SESSİZ tuzakları:**

Git yalnız "iki tarafın da değiştirdiği" yeri çakışma sayar. Bir dosyayı
**yalnız karşı taraf** değiştirmişse (veya silmişse) merge onu sessizce
uygular. Bu merge'de üç yerde oldu:

1. **`guidance_core.py`'ye pose aygıtı çakışmasız sızdı** — `gercek_geometri`,
   `_quat_dcm`, `_process_gercek` ve `process()` içine `if gercek is not
   None:` sevki. Bizim imzada `gercek`/`pose` diye değişken yok → çalışma
   anında `NameError`. Söküldü.
2. **`visual_lead.py:334`** — `kaynak_ad = "gercek" if get_gercek ...`
   aynı şekilde sızmıştı. **`pytest` bunu YAKALAMADI** (17 test geçti);
   dosyanın kendi koşucusu yakaladı. Söküldü.
3. **`gcs_server.py`'de iki DAVRANIŞ GERİ ALMASI** — ortak atada olan,
   bu dalın koruduğu, kararlı dalın SİLDİĞİ kod merge'de siliniyordu:
   - manuel modun yerden **ARM + TAKEOFF**'u ve irtifaya göre FBWA→FBWB
     geçişi (08-01'de "manuel mod yerdeki uçağı hiç kaldıramıyor" diye
     düzeltilmişti) — geri gelirdi.
   - **`_GORSEL_ACIK = off`** — kararlı dalın "görsel fazı tamamen kapat"
     kararı. Bu dalın bütün yönüne aykırı (hibrit güdüm, B5, §1).
   Dosya bu dalın hâline döndürüldü; kararlı dalın `gcs_server`'a katacağı
   şey (kamera/telemetri kuyruk düzeltmesi) `c6869f2`'de zaten alınmıştı.

**Boşuna korkulan:** `tools/los_kayma.py` ve `gudum_rapor.py` **silinmedi**
(uyarı #2 gereksizmiş) — ortak atada yoklardı, yalnız bu dal eklemişti, merge
korudu. `YAW_SUS_N=40` de otomatik korundu: `adapter_copter.py`'yi yalnız bu
dal değiştirmiş, çakışma bile olmadı (`gps_guidance.py`'deki eşi doğrulandı,
test G15 geçiyor).

**Merge'de KAZANILANLAR:**

- `KARARLI_HAL.md` **§3b** — iç daire kayması neden SABİT: oranlı sürüm dört
  çapta (⌀96/71/55/41) ölçülüp **elendi**; sabit 14 m her yerde kazandı.
  ⇒ aşağıdaki "`IC_ORAN=0.27` A/B" maddesi **artık gereksiz**, cevabı bu.
- **Daire çapı butonları arayüze eklendi** (`#cap`) — eskiden yalnız curl'dü.

### Merge sonrası taban ölçümü

Aynı anda çok şey değiştiği için ilk iş **tek bir taban ölçümü**, ayarlama
değil. Değişenler (ayrıntı: [KARARLI_HAL.md](KARARLI_HAL.md), DURUM.md §3):

| ne | eski | yeni |
|---|---|---|
| `KD_H` (lead) | 0.20 | **0.60** |
| iç daire nişanı | yok | **14 m** (yalnız dönüşte) |
| `ISTASYON_ELEV_DEG` | 25° | **15°** |
| araç zarfı | 15 m/s, 5 m/s², `WP_ACC_Z 1` | **25 m/s, 8 m/s², 2.5 m/s², 45°** |
| kamera/telemetri kuyruğu | birikiyordu | en-son-veri-kazanır |

- [ ] **İLK: SITL'i yeniden başlat, sonra `python3 tools/parm_denetle.py`**
      → COPTER **11/11 ✓** olmalı. Parm dosyası kararlı daldan alındı, adları
      bu firmware'in şemasına çevrildi (değerler aynı). Bir satır bile
      ✗ ise araç o parametrede firmware varsayılanında kalmıştır ve uçuş
      ölçümü kararlı dalla karşılaştırılamaz.
      *Sonuç:*
- [ ] Düz uçuş senaryosu — regresyon kontrolü (iç daire kayması düzde 0
      olmalı, `ic_kayma_m` CSV sütunu doğrular). *Sonuç:*
- [ ] Daire senaryosu (`circle`, ⌀55 m) — oturmuş menzil. Kararlı dalın
      karşılığı 15-16 m. *Sonuç:*
- [ ] Görsel faz vuruş oranı — pose dönemindeki taban %27 (30 fazda 8).
      *Sonuç:*

> Kötü çıkarsa geri dönüş: `AVCI_GPS_KD=0.2 AVCI_GPS_IC=0` tek uçuşta eski
> yasaya döndürür (parm dosyası hariç).

### Sonra ayrılacak değişkenler

- [x] ~~`IC_ORAN=0.27` (yarıçap-oranlı kayma) A/B~~ — **merge'de cevaplandı,
      yapmaya gerek yok.** Kararlı dal bunu dört çapta ölçmüş ve oranlı sürümü
      elemiş (KARARLI_HAL.md §3b): sabit 14 m ⌀96/71/55/41'in hepsinde kazandı;
      oranlı sürüm ⌀96'da fazla (25 m), ⌀41'de az (11 m) kaydırıyor. Kod duruyor
      ama kapalı (`AVCI_GPS_IC_ORAN=0.27` ile açılır).
      *Sonuç:* **sabit 14 m kalıyor.** Açık kalan: 35 m'den dar daireler hiç
      denenmedi — orada 14 m yarıçapın %40'ı olur, radyal bedel baskın olabilir.
      > Daire çapı butonları artık arayüzde (Hedef İHA → **Daire Çapı**).
      > curl karşılığı hâlâ geçerli:
      > `curl -X POST localhost:8000/api/command/plane/scenario/circle_s`
      > Çaplar: `circle_xl` ⌀96 · `circle_l` ⌀71 · `circle` ⌀55 · `circle_s` ⌀41
- [ ] `WP_ACC_Z` 2.5 vs 1 — 08-05'te 3 denenip kötüleşmişti ama
      `ISTASYON_ELEV_DEG=25` ile birlikte değişmişti, ayrılamadı. Şimdi 15°
      sabitken tek başına denenebilir. *Sonuç:*
- [ ] `ISTASYON_ELEV_DEG` 25° — yalnız başına (ACC_Z sabitken). *Sonuç:*
- [ ] `AVCI_GPS_GUDUM=frpn` — kararlı dalda 31.1 m (istasyon yasası 29.4 m).
      Yeni zarfla tekrar bakılabilir. *Sonuç:*

> **Ders (08-05'ten):** iki değişken aynı anda değiştirildi, hangisinin suçlu
> olduğu ayrılamadı. Bu entegrasyonda da çok şey birden değişti — bu yüzden
> önce TEK bir taban ölçülür, sonra tek tek ayrılır.

---

## 0d — DÖRDÜNCÜ DENEY DE ÇÜRÜDÜ + vuruşun asıl ayrımı (2026-08-08 akşam)

### Deney 4: ivme tavanı 4 → 8 m/s²  ❌

Aynı oturumda A/B (ivme 8: 15 faz, ivme 4: 24 faz), aynı GPS hattı, ⌀55 daire:

| ölçüt | ivme 8 | ivme 4 |
|---|---|---|
| komut ivmesi | 8.0 (tavanda) | 4.0 (tavanda) |
| **hız vektörünün dönüşü** | **17.3 °/s** | 8.3 °/s |
| **tespit güveni** | **0.80** | 0.81 |
| gövde pitch p10 | −13.2° | −7.7° |
| 3 m altına kapanan | **0/15** | 0/24 |

✅ **"Kamera yere bakar, tespit bozulur" korkusu UÇUŞTA DA ÇÜRÜDÜ.** Araç 13°
burun aşağı gitti, güven 0.81 → 0.80. `IVME_TAVAN` yorumundaki eski gerekçe
artık ölçümle nitelenmiş durumda (kod yorumuna işlendi).

❌ Ama çeviklik iki katına çıkmasına rağmen sonuç değişmedi.

### ⚠ KENDİ SAYIMI DÜZELTTİM

Daha önce "gereken 19.2 °/s" demiştim — **yanlıştı.** O sayı GPS fazının
loglarından ölçülmüştü; GPS drone'u hedefle BİRLİKTE döndürdüğü için orada LOS
yavaş kayıyor. Aynı uçuşta, benzer menzilde:

| | LOS'un dönme hızı |
|---|---|
| GPS fazı (15-25 m) | **28 °/s** |
| **görsel faz (11-19 m)** | **84 °/s** |
| aracın yapabildiği (8 m/s²) | 17 °/s |

84 °/s için gereken ivme **32 m/s² = 3.3 g** — quad'ın fiziksel tavanı 9.8.
Bu yol KESİN kapalı.

### Asıl bulgu: talebi güdüm KENDİ üretiyor

Aynı uçuş, aynı hedef, benzer menzil — GPS 28 °/s, görsel 84 °/s. Farkı yaratan
menzil değil, **drone'un kendi hareketi.** GPS hedefle birlikte dönüyor (hedefin
açısal hareketi kendi hareketiyle sadeleşiyor); görsel faz hedefin üstüne
dalıyor ve 84 °/s'lik talebi kendisi üretip sonra ona yetişemiyor.

Dört deney (lead kapısı · lead tavanı · kapanma hızı · ivme tavanı) hepsi
"talebe daha iyi yetiş" demeye çalıştı. Yapılması gereken **talebi küçültmek.**

### DÜZ vs DÖNÜŞ — tam ayrım (55 faz, 18:00 sonrası)

| hedef | LOS hızı | `ok` oranı | en yakın menzil |
|---|---|---|---|
| dönüşte (ω≈19 °/s), 41 faz | **60-113 °/s** | %44-56 | hiç 9.4 m altına inmedi |
| düzde (ω≈1 °/s), 14 faz | **1.2-2.5 °/s** | %88-99 | **0.3-1.3 m** |

### 🎯 VURUŞUN AYRIMI: terminal algı sürekliliği

Düz uçuşta 3 vuruş / 9 ıska:

| | `ok` oranı | **kör dalış** | en yakın |
|---|---|---|---|
| VURANLAR | %99 / %99 / %44 | **%0 / %0** / %44 | 0.87 / 0.92 / 0.31 m |
| ISKALAYANLAR | %88 medyan | **%11 medyan** | 1.17 m medyan |

**Vuran fazlarda drone hedefi temasa kadar hiç kaybetmedi.** Iskalayanlar son
anda körleşti. `185411` **0.56 m**'ye indi ama vuramadı; `185800` **0.87 m**'de
vurdu — yani mesafe değil, SON ANDA GÖRÜP nişanı düzeltebilmek belirleyici.
(Bu, DURUM.md B6'daki "vuran 4 geçişin dördünde de kor_dalis ≤ %3" bulgusunun
bağımsız tekrarı.)

⚠ Denenen ama KURULAMAYAN hipotez: "ıska ≈ menzil × tan(terminal dikey kadraj
hatası)". Korelasyon r = +0.49, n = 8 — kanıt sayılmaz, veri yetersiz.

### Devir menzili deseni

~20 m'de başlayan fazlar iyi (ok %88-99), 5-14 m'de başlayanlar kötü
(ok %20-34). İkinciler bir geçiş sonrası yeniden giriş. "Birkaç geçiş sonra
vuruyordu" gözleminin kaynağı bu: her seferinde bir ~20 m girişi bekleniyor.

---

## 0c — C1: RANGE_SET 11 → 8 (kayramin_super_gudumu'ndan çekildi, 6d7854b)

Kayra'nın dalında ÜÇ otonom uçuşla ölçüldü, buraya cherry-pick edildi:

- **Dönüş ileri beslemesi (v_ist = v_hedef + ω×r): ELENDİ.** Formül doğru
  (G14b sayısal türevle birebir) ama daire 15.1 → **23.0 m açıldı**. Mekanizma:
  "doğru" FF, komut hızını düşürüp aracın dönen çerçevedeki takip gecikmesini
  telafisiz bırakıyor; eski v_hedef fazlalığı kazara faydalı lead'miş.
  Varsayılan **KAPALI** (`AVCI_GPS_FF_DONUS`), G14a regresyon koruması.
- **RANGE_SET 11 → 8: KABUL.** Daire 15.1 → **13.3 m**, düz 13-14 → **10.3 m**
  [p10 9.8, p90 10.8]. Eski "11→5 etkisiz" bulgusu doygunluk dönemine aitti.
- Yan etki: dönüşte kadraj −9.4 → −15.2° geriledi (dikey ofset RANGE_SET'e
  bağlı, tutuş menzili değişmiyor). Kayra'nın sıradaki hamleleri: d_below'u
  gerçek menzille ölçekle; dönüşte arka bileşeni daralt.

**ALINMAYANLAR** (bu dalda karşılığı yok): dinamik istasyon yükselişi
(`ELEV_DINAMIK`) ve onun G13 testleri — burada yükseliş SABİT 15°.
`UYGULANACAK.md` de alınmadı; bu dalın tek takip belgesi TODO.md.

⚠ **§0b'nin görsel faz tabanı GEÇERSİZ.** O ölçümler RANGE_SET=11 ile alındı;
görsel faza devir ~19 m'de oluyordu. İstasyon 8 m'ye inince devir daha da
yakında olacak ve nişanın dönme hızı YAKINDA daha kötü (8-20 m: 19.2 °/s,
araç 22 m/s'te ancak 10.4 °/s). GPS fazı için iyileşme, görsel faz için
muhtemelen kötüleşme — görsel faz ölçümleri yeniden alınmalı.

---

## 0b — Yatay lead kapısı: ölçek → azimut (UYGULANDI 2026-08-08, UÇUŞ ÖLÇÜMÜ BEKLİYOR)

**Bulgu (08-08, 4 oturum, 60 görsel faz).** Görsel faz hedef DÜZ uçarken
39/39 kapanıyor, DÖNERKEN **0/14**. Ayrım ikili — hedefin açısal hızı
kapatanlarda medyan **1.2 °/s**, kapatamayanlarda **19.1 °/s**.

**Kök neden.** Yatay lead `kalite` ile çarpılıyordu; `kalite` bbox
GENİŞLİĞİNDEN türer (menzil vekili, 22.5 m'de 0 / 9.6 m'de 1). Dönüş
karelerinde (n=348) ölçülen:

| | DÖNÜŞ (kapatamayan) | DÜZ (kapatan) |
|---|---|---|
| gerçek menzil | 15.3 m | 11.1 m |
| kalite (ölçek kapısı) | **0.00** | 0.49 |
| \|az_rate\| | 71.8 °/s | 1.6 °/s |
| gereken lead (0.6·ω, 20° tavan) | **20.0°** | 1.0° |
| **uygulanan lead** | **0.0°** | 0.3° |
| bbox merkez sapması (gt) | 1.84° | 0.33° |
| tespit güveni | 0.79 | 0.76 |

Tespit sağlamdı; lead tam ihtiyaç duyulan anda kapalıydı. `tespit_yok`
karelerinin **%91'inde hedef GERÇEKTEN kadraj dışında** — dedektör hatası değil,
nişanlama hatası. Aynı gerekçe kadraj tutma terimi için zaten kabul edilmişti
(`adapter_copter._dikey_pn` (3): "kalite ile ÇARPILMAZ ... sinyal bbox
merkezidir").

**Yapılan.** `_yatay_pn` kapısı `kalite` → `azimut_kalite`. Gürültü koruması
değişmedi (AZ_STEP_MAX + AZ_EMA + PN_RATE_EMA + ±20° tavan). Tekil geometri
koruması duruyor (T66). Kill-switch: `AVCI_IBVS_PN_YATAY_KAPI=olcek`.
Testler: T64/T65/T66, 69/69 geçti. CSV damgasında `PN_KAPI=`.

### ❌ UÇUŞ ÖLÇÜMÜ: DEĞİŞİKLİK İŞE YARAMADI (08-08 14:28, A/B tek oturum)

Aynı oturumda A (azimut, 11 faz) → B (olcek, 12 faz), ⌀55 daire, ω≈19 °/s:

| ölçüt | A: yeni | B: eski | taban |
|---|---|---|---|
| **uygulanan lead** | **20.0°** | **0.0°** | 0.0° |
| 3 m altına kapanan faz | **0/11** | **0/12** | 0/14 |
| faz süresi (medyan) | 2.3 s | 2.3 s | 3.1 s |
| dönüşte `ok` kare oranı | %49 | %48 | %44-59 |
| en yakın menzil (medyan) | 13.6 m | 12.8 m | 12.6 m |
| kapanma hızı | 2.9 m/s | 1.5 m/s | — |
| \|yaw_hata\| | 17.9° | 21.0° | — |

Mekanizma tasarlandığı gibi çalıştı (lead 0° → 20°), **hiçbir sonuç ölçütü
değişmedi.** Kapı gerçek bir kusurdu ama dönüşteki başarısızlığın sebebi
DEĞİLDİ. Kapanma hızı ve yaw hatası doğru yönde kıpırdadı — sonucu çevirmedi.

### Asıl kısıt: YAW HIZI DOYGUNLUĞU

Aynı karelerde ölçüldü:

- `yaw_doygun` **%98-100** — yaw komutu neredeyse her karede tavanda
- `v_doygun` %97, komut hızı 22.9 m/s, ama **kapanma hızı yalnız 1.5-2.9 m/s**
- LOS'un gövdedeki azimut hızı **78-82 °/s**, güdüm tavanı
  `YAW_HIZ_MAX = 90 °/s` — arada marj yok

Yani yaw yetkisinin TAMAMI hedefin açısal hızına yetişmeye gidiyor, duran
18-21°'lik hatayı kapatmaya bir şey kalmıyor. Hedef eksenden ~20° sapmış
duruyor ve kadrajdan süpürülüyor (`ok` oranı %48).

⚠ `YAW_HIZ_MAX` yükseltmek serbest bir düğme DEĞİL — 2026-07-25'te 1080
denendi ve GERİ ALINDI (tespit gürültüsü doğrudan gövdeye geçti, bkz.
`guidance_core.KP_YAW` üstündeki not).

### Üç deney, üçü de çürüdü (08-08, hepsi ⌀55 daire, hedef ω≈19 °/s)

| # | değişiklik | uygulandı mı | 3 m altına kapanan |
|---|---|---|---|
| 1 | lead kapısı `kalite` → `azimut_kalite` | evet (lead 0° → 20°) | **0/11** (kontrol 0/12) |
| 2 | `PN_YATAY_MAX` 20° → 60° | evet (lead 20° → 47-58°) | **0/13** |
| 3 | `V_KAPANMA` 25 → 12 m/s | evet (hız 22 → 12 m/s) | **0/20** |

Deney 3'ün "iyileşme" gibi görünen sayıları (faz süresi 2.4→3.0 s, `ok`
%51→%60) YANILTICI: uzun fazlarda menzil 16 → 34 → **89 m**'ye açılıyor. Hedef
uzaklaşıyor, kamera geniş açılı olduğu için uzaktaki hedef kadrajda rahat
duruyor. `V_YAKLASMA` yorumunda bu zaten yazılıydı ("İlk denemede 12.0 seçildi
ve GT modunu tamamen bozdu ... menzil 24.2 → 82.8 m") — **önermeden önce
okunmadı, ders budur.**

### KÖK NEDEN: ivme tavanı × sabit kamera (KUTU)

Yatay ivme `IVME_TAVAN=4 m/s²` ile sınırlı ve bu **kamera kısıtı**: quad
ivmelenmek için burnunu eğer, kamera gövdeye +25° sabit → ~5 m/s² üstünde
kamera yere bakar, gökyüzü arka planı gider, tespit bozulur.
⚠ **Donanım değişikliği masada yok (kullanıcı kararı 08-08)** — gimbal cevabı
verilmeyecek.

Dönüş yarıçapı = V²/a. Ölçülenler:

| | değer |
|---|---|
| hedefe yetişmek için gereken hız (mutlak) | > 21 m/s |
| 21 m/s'te drone'un dönüş yarıçapı | **110 m** |
| hedefin dönüş yarıçapı | **27.5 m** |
| hız vektörünün dönebildiği hız @22 m/s | 7.3 °/s |
| hız vektörünün dönebildiği hız @12 m/s | 17.4 °/s |
| LOS'un döndüğü hız | 46-90 °/s |

**Hiçbir SABİT hız ikisini birden sağlamıyor.** Yavaş → döner ama geride kalır;
hızlı → yetişir ama dönemez.

### Kapı: kavis değil, KESİŞME

Kısıt yalnız **sürekli kavis çizilirse** bağlayıcı. Bugünkü yasa hedefin ŞU ANKİ
yerine nişan alıyor; hedef daire çizince drone da kavis çizmek zorunda kalıyor.
**Düz çizgi ivme gerektirmez** — hedefin OLACAĞI yere düz uçulursa tavan hiç
devreye girmez. GPS fazı zaten bunu yapıyor (`IC_KAYMA` ile dönüşün içine).

Denenen 1 ve 2 bunun ucuz taklidiydi: sabit bir lead AÇISI, kesişme NOKTASINA
nişan almakla aynı şey değil. Üçü de bu yüzden tutmadı.

**Sıradaki iş:** hedefin dönüş yarıçapını YALNIZ KAMERADAN kestirip nişanı
kesişme noktasına koymak. Uçuştan önce çevrimdışı doğrulanacak — gerçek uçuş
yörüngeleri + ölçülmüş algı gürültüsüyle.

---

## 1 — LOS kayması: görsel fazın asıl kusuru

**Yedek plan.** Önce [§0'daki tam merge](#0--gps_kararli_hal-tam-merge-kullanıcı-kararı)
denenecek; düzelmezse buradaki deneyler sırayla açılır.

### Ölçülen gerçek (2026-08-07)

Çarpışma rotasının tanımı: **menzil kapanırken görüş hattı (LOS) açısı
DEĞİŞMEZ.** Açı dönüyorsa çarpma olmaz — ne kadar yaklaştığın önemsiz.
`tools/los_kayma.py` bunu 12 m → 2 m bandında ölçüyor.

| yapılandırma | kilitli (<5°) | kayma medyanı | vuruş |
|---|---|---|---|
| varsayılan (`PN_SURE=0.6`, `PN_MAX=30`) | %9 | 12.2° | 1/32 |
| `PN_SURE=0` | %25 | 8.5° | 1/16 |

Vuran fazlar 0.8° kayıyor, ıskalayanlar 8.5°. **En yakın menzil ikisinde de
0.3-0.9 m — o sayı ayırt etmiyor, bu ayırt ediyor.**

> **Çürütülen fikir:** "terminal dikey PN patlaması ıskalara sebep oluyor,
> `PN_SURE=0` düzeltir." Kullanıcı uçurdu, düzelmedi (`pn_dikey` her yerde
> max 0.0, yine 1/19). PN=0 geometriyi **iyileştirdi** (%9 → %25) ama vuruşa
> dönüşmedi.

> **Açık uyarı:** kilitli geometri **gerekli ama yeterli değil** — 3 faz
> kilitliydi (3.0°, 2.6°, 4.4°) ve yine ıskaladı. Son 0.3 m'de başka bir şey
> daha var (bkz. [terminal kontrol yetkisi](#terminal-fazda-kontrol-yetkisi)).

### Deney matrisi

Her uçuştan sonra: `python3 tools/los_kayma.py --son 15`

- [ ] `AVCI_IBVS_PN_SURE=0.6 AVCI_IBVS_PN_MAX_DEG=5 bash scripts/gcs.sh`
      *Sonuç:*
- [ ] `AVCI_IBVS_PN_SURE=2.5 AVCI_IBVS_PN_MAX_DEG=5 bash scripts/gcs.sh`
      ← **asıl aday** (PN erken ve yumuşak: açıyı geç patlatmadan durdurur)
      *Sonuç:*
- [ ] `AVCI_IBVS_PN_SURE=4.0 AVCI_IBVS_PN_MAX_DEG=5 bash scripts/gcs.sh`
      *Sonuç:*

**Karar kuralı:** "kilitli" oranı %25'in üstüne çıkıyorsa yön doğru, aynı
eksende devam et. Vuruş sayısına bakma — örneklem küçük, gürültülü.

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
