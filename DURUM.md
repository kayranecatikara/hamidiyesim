# DURUM — güdüm iş listesi ve gerekçeleri

Eskiden üç ayrı belgeydi (`DEVAM.md`, `TODO.md`, `UYGULANACAK.md`); 2026-08-04'te
burada birleşti. Her maddenin **ölçülmüş gerekçesi** yanında duruyor — bu belge
bir "ne yapalım" listesi değil, "neden bunu yapalım, neyi denemeyelim" listesi.

**Güncel öncelik [TODO.md](TODO.md)'de.** 2026-08-04'te algının darboğaz
olmadığı ölçüldü; oradaki maddeler yeni şüpheliyi kovalıyor ve şu an
buradakilerden önce gelir. Bir madde bitince buradaki "Bitenler"e taşınır.

Çalıştırma ve ölçüm komutları: `docs/SIMULASYON_CALISTIRMA.md`.

---

## Çalışma kuralı ve öğrenilen dersler

**Tek seferde tek değişken → testler → uç → ölç → sonucu bu belgeye yaz.**
Bu kural bir kere sekiz grup değişikliğin bir arada uçurulması üzerine kondu:
bazıları işe yaradı, biri ölçülebilir zarar verdi (görsel kilit: kör uçuş %64,
drone hedefin üstüne çıkıp zemine çakıldı), hangisinin ne yaptığı ayırt
edilemedi.

**Dersler — tekrarlanmasın:**

- **"Araç komutu uygulamıyor" demeden önce kara kutuya bak.** Bu teşhis bir kez
  kondu ve çürütüldü: alçalma emredilen anlarda `PSCD.DVD +6.36` iken
  `VD +6.43`, takip hatası 0.1 m/s. Araç kusursuz uyguluyordu. Hata, 5
  saniyelik bir CSV penceresine bakılmasıydı — araç o sırada mevcut bir
  tırmanışı tersine çeviriyormuş.
- **Ölçmeden değer değiştirme.** `ATC_ANG_YAW_P 4.5 → 3.0` iyi niyetle
  yapıldı, düzeltmeye çalıştığı şeyi 4 katına çıkardı.
- **Bozuk veriyle ölçüm yapma.** Hedefin hızı bir süre `tgt_vx` sütunundan
  17.5 m/s sanıldı; o sütun zaten bozuk olduğu kanıtlanan kestirimdi. Gerçek
  değer ArduPlane kara kutusundan **14.0 m/s** çıktı.
- **CSV'ye tek başına güvenme.** `menzil_gercek_m` MAVLink telemetrisinden
  geliyor, EKF çerçeve ofsetinden etkileniyor ve en yakın anı geriden
  gösteriyor — bir vuruşta CSV 3.20 m derken kara kutu 0.21 m diyordu.
  Geometri sorularının dürüst kaynağı iki aracın kara kutusu; onu hizalayıp
  gerçek geometriyi çıkaran araç `tools/gecis_analiz.py`.

### Kök neden: dikey bütçe (2026-08-02, üç uçuş, kara kutuyla)

Üç uçuş da terminale **aynı** geometriyle giriyordu: yatay ~12.5 m, dikey
+4.65 m (o zamanki istasyon ofseti). ArduPilot dikey hız komutunu **1.0 m/s²**
ile rampalıyor (`PSCD.DVD`'nin pozitif eğim medyanı üç uçuşta da tam 1.00).
Güdüm 8-22 m/s tırmanma istiyor ama hız tavanı (`WP_SPD_UP = 5.0`) hiç
görülmüyor — yani **hız değil, İVME sınırlıyor**. Komutun büyüklüğü alakasız.

Sıfırdan 4.65 m kapatmak 1 m/s²'de **3.05 s** sürer. Elde olan süre:

| | **A (vurdu)** | B (ıska) | C (ıska) |
|---|---:|---:|---:|
| görsel faza giriş menzili (3B) | **10.32 m** | 7.65 m | 9.16 m |
| faz başından en yakın ana | **2.64 s** | 2.38 s | 2.77 s |
| kapatılan dikey | **4.25 m** | 2.70 m | 2.42 m |
| gereken dikey | 4.28 m | 4.22 m | 4.48 m |
| **en yakın anda kalan dikey** | **+0.03 m** | **+1.52 m** | **+2.06 m** |
| sonuç | **GERÇEK TEMAS** | alttan geçti | alttan geçti |

Üçünün de süresi 3.05 s'nin altında. A yalnızca **rampayı erken başlattığı**
için yetişti. Kısır döngü: dikey geride kalır → hedef kadrajın tepesinden çıkar
→ tespit ölür → `kor_dalis` komutu dondurur → düzeltme büsbütün biter. Ama
halkanın başı dikey bütçe.

4-2 m bandında ölçülen (algı çöküşünün **sonuç** olduğunun kanıtı):

| 4-2 m bandı | A | B | C |
|---|---:|---:|---:|
| `gercek_kadraj_ici` | **%69** | %21 | %15 |
| son 1.5 s'de `kor_dalis` kare | **1/46** | 30/46 | 29/46 |
| `pn_dikey_deg` | −1.7° | +21.9° | **+30.0° (tavanda)** |

Not: 0.61 m'de bile Gazebo temas sensörü tetiklenmedi. Gerçek temas için
~0.3 m daha kapatmak gerekiyor.

### Karar: vuruş ölçütü = gerçek fiziksel temas

İki seçenek vardı: **(a)** 1.5 m yakınlığı vuruş saymak — ekranda güzel
görünür, görev "başarılı" biter, ama gerçekte ıskaladığımızı bilmeyiz.
**(b)** Gerçek fiziksel temas — dürüst, ama şu an çoğu denemede vuramıyoruz.

**(b) seçildi.** Eski halin "başarısı" kısmen erken durmadan geliyordu: drone
1.5 m'ye gelince "VURULDU" deyip güdüm DURUYOR, hedefin yanından geçtikten
sonra ne olacağıyla hiç yüzleşmiyordu. Gerçek temas ölçütü bu sorunu
**yaratmadı, görünür kıldı**. Vuruş oranının düşmesi hata değil, dürüstlük.

---

## Bitenler

- [x] **A1 — `ATC_ANG_YAW_P 3.0` satırı kaldırıldı** (varsayılan 4.5'e dönüldü).
      Temiz A/B (4 kopter kaydı), sabit-başlık dilimlerinde takip hatası std:
      3.0 → 11.88° ve 8.53°; 4.5 → **1.32°**, 4.64°, **1.43°**. Bozulmanın
      karakteri de değişti: 3.0'da kalıcı sapma (karelerin %32'si >20°), 4.5'te
      yalnız anlık sıçrama (%2). Motor doygunluğu yok.
      ⚠ Bu maddenin 2. ölçütü ("toplam dönüş < 1 tur") **yanlışmış** — dönme
      P kazancıyla ilgisiz, `DesYaw`'ın kendisi dönerken oluyor, yani güdümün
      komut ettiği dönme. Bkz. B5.

- [x] **A5 — Gerçek çarpışma tespiti.** 1.5 m yakınlık artık vuruş sayılmıyor;
      tek kaynak Gazebo temas sensörü. Ölçülmüştü: bir koşuda 0.61/0.69/0.75/
      1.06/1.16/1.20 m yaklaşma vardı → **6 sahte vuruş** raporlanırdı. Dahası
      sahte vuruş güdümü DURDURUYOR, drone tam hızla komutsuz kalıp savruluyordu.
      Sensör gövde+kanat+kuyrukta, **tekerlek dahil değil** (pistte sürekli yere
      değer). Karşı taraf iris değilse imha sayılmaz. `VURUS_MENZIL` (1.5 m)
      artık yalnız temas kaynağı yoksa devreye giren **yedek**.
      Ek olarak: `start_chase`/`start_visual` temas mandalını sıfırlıyor
      (mandal latch'li olduğu için önceki denemenin teması yeni fazın ilk
      karesinde "vuruldu" dedirtiyordu).

- [x] **Dikey ıska, 1. tur** — sabit 4.65 m ofset yerine
      `r_eff = min(menzil, RANGE_SET)`; LOS yükselişi her menzilde sabit
      (test G10, sapma 0.00°). ⚠ Bu YETMEDİ: açı sabitlendi ama istasyonun
      KENDİSİ hâlâ 25°'deydi.

- [x] **Dikey ıska, 2. tur** — `ISTASYON_ELEV_DEG` kamera tilt'inden ayrıldı,
      25° → 15°: kapatılacak dikey **4.65 → 2.85 m**. Test G11 bütçeyi koruyor.
      ⚠ Bu değişikliğin bedeli izole ölçülmedi — **açık soru B7**.

- [x] **Başlatma/durdurma script hataları** — kendi kabuğunu öldüren `pkill`,
      sahte "hazır" bildirimi, boru sızıntısı.

- [x] **A7 — Belge güncellemesi.** "Araç dikey komutu uygulamıyor" yanlış
      teşhisi ve ondan çıkan kural artık "Çalışma kuralı ve öğrenilen
      dersler"de.

- [x] **Hasar modülü tetiklenebilir hale geldi** — `main` merge'i temas
      sensörlerini SDF'ye geri getirdi, `AVCI_HASAR` varsayılanı 1.
      (Eski `TODO.md`'de "revert edildi, tetiklenemez" yazıyordu; artık geçersiz.)

- [x] **GCS telemetrisi donuyor** — `mavlink_listener`'a `else` dalı; 14550'den
      gelen quadrotor paketleri `telemetry_state["iris"]`'e yazılıyor.
      Uçuşta doğrulandı.

- [x] **Parametre adları yanlıştı** — 9 parametrenin 7'si SITL'e hiç
      uygulanmıyordu (firmware SI birimine geçip yeniden adlandırmış).
      `tools/parm_denetle.py` tekrarını önlüyor.

- [x] **Ölçüm araçları test altına alındı** — `tests/olcum_araclari/`
      (22 senaryo, `python3 -m tests.olcum_araclari`). Gerekçe: bu araçlar
      "vurduk mu, nereden ıskaladık" sorusunun hakemi; sessiz bir hesap hatası
      ona dayanan her kararı bozar ve fark edilmez. Kritik olanlar: kara kutu
      DİKEY işaret sözleşmesi (E4 — tersine dönerse her "alttan/üstten geçti"
      hükmü tersine döner), telemetri boşluğuna uydurma enterpolasyon yapılmaması
      (E3), sentetik süzgecin gerçek uçuşu elememesi (K8), ve parametre
      denetiminde 45 ↔ 45.000000 biçim farkının sahte alarm üretmemesi (P3).

- [x] **Kendi etrafında dönme** — kök neden: `yaw_hata` kapanmıyorken komut her
      karede bir tavan adımı daha ekliyordu (90 °/s sürekli dönme).
      `adapter_copter`'a "hata kapanmıyorsa yaw'ı sustur" kapısı eklendi
      (T44/T45). Seyirde dönme 27 °/s → ~0, yaw takip hatası 30.5° → 4.5°.

- [x] **`WP_YAW_BEHAVIOR` 2 → 0** — firmware, yaw komutu olmayan anlarda burnu
      gidiş yönüne çeviriyordu.

- [x] **Sahte PnP paneli** — ground-truth'a yapay gürültü ekleyip "tahmin" diye
      gösteriyordu. Yerine gerçek görüş kestirimi + ground-truth + faz kapıları.

- [x] **Hedef telemetrisi = cevap anahtarı** — güdüme bağlanmadı, ölçüm için
      10 sütun eklendi.

- [x] **Talon manuel modda kalkmıyor** — ARM + TAKEOFF adımları hiç yoktu.
      Eklendi (yerde FBWA, 15 m üstünde FBWB). Uçuşta doğrulandı.

---

## Sıradaki işler

Sırayla git, atlama. Her madde: uygula → testler → **uç** → ölç →
*Sonuç:* satırına yaz → tikle.

> ⚠ **Önce [TODO.md](TODO.md).** 2026-08-04'te **algının darboğaz olmadığı**
> ölçüldü (kusursuz algıyla da isabet değişmedi). Oradaki maddeler terminal
> fazdaki kontrol yetkisini kovalıyor ve buradakilerden önce gelir.

### Şimdi karara bağlanacak

- [ ] **B7 — AÇIK SORU: istasyon açısını kamera tilt'inden ayırmak doğru muydu?**
      `control/guidance/gps_guidance.py` → `ISTASYON_ELEV_DEG`

      ⚑ **2026-08-05: 25°'YE GERİ DÖNÜLDÜ, ölçüm bekleniyor.** Aşağıdaki
      "iki şey denenmedi" listesindeki **(2). şık uygulandı**: istasyon 25°'de
      bırakılıp `WP_ACC_Z` 1 → 3 yükseltildi. 15°'ye inilmesinin TEK gerekçesi
      dikey ivme bütçesiydi; 4.65 m artık ~1.76 s'de kapanıyor (eskiden 3.05 s,
      terminalde eldeki süre 2.4-2.8 s) — bütçe 25°'ye rahat sığıyor.
      İkisi BİRLİKTE değerlendirilecek: `WP_ACC_Z=3` geri alınırsa istasyon da
      15'e dönmeli, yoksa dikey ıska geri gelir.
      *Ölçüt:* terminalde "kalan dikey" ≈ 0, alttan geçme bitmeli; hedef kadraj
      merkezinde görünmeli (15°'de merkezin ~10° altındaydı).
      *Sonuç:*

      *Şüphe:* 25° tesadüf değildi — kamera tilt'i o. İstasyon 25°'de kurulunca
      hedef kadrajın TAM MERKEZİNDE oluyordu. 15°'ye inince hedef merkezin ~10°
      altına düştü. Pose modelinin merkez dışı performansı, lens distorsiyonu ve
      hedefin gökyüzü yerine zemin önünde görünmeye başlaması bedel olabilir.

      *Elde olan (10 faz @25° vs 17 faz @15°):*

      | | 25° | 15° |
      |---|---:|---:|
      | `ok` oranı (tüm faz) | %24.0 | **%32.0** |
      | `ok` oranı (menzil < 8 m) | %8.7 | **%18.2** |
      | hedef kadraj içi | %59.8 | **%67.0** |
      | en yakın menzil medyanı | 5.25 m | **1.73 m** |
      | vuruş | 1/10 | 3/17 |

      ⚠ **Bu tablo şüpheyi ÇÜRÜTMÜYOR, karışık.** Algının iyileşmesi büyük
      ölçüde geometrinin SONUCU: drone hedefin seviyesine yakın kalınca hedef
      kadrajdan geç çıkıyor, tespit doğal olarak uzuyor. Ölçülen şey "merkez
      dışı kadrajlama zararsız" değil, "net etki olumlu". Merkez dışı olmanın
      kendi bedeli **hâlâ bilinmiyor**.

      *Bilinmeyenler:* (1) 15° taranarak seçilmedi, dikey ivme bütçesi
      hesabından çıktı — 18° ve 20° hiç denenmedi. (2) **Asıl alternatif hiç
      denenmedi:** istasyonu 25°'de bırakıp `WP_ACC_Z`'yi 1.0 → 2.5-3.0
      yükseltmek. İşe yararsa hem merkez kadrajlama hem dikey kapanma birlikte
      elde edilir, bu ayrım gereksizleşir.

      *Nasıl karara bağlanır — sırayla, tek değişken:*
      - **Adım 1:** `ISTASYON_ELEV_DEG=25` geri + `avci_copter.parm`'a
        `WP_ACC_Z 2.5`. Tek uçuş. ⚠ `WP_ACC_Z` global bir kopter parametresi —
        kalkışı ve istasyon tutmayı da etkiler; irtifa aşımı/salınım için
        `PSCD.DVD` vs `VD` bakılacak.
      - **Adım 2:** tutmazsa açıyı tara: 20°, 18°. Bütçeye sığan en büyük açı
        seçilir (test G11 sınırı zaten kontrol ediyor).

      *Ölçüt — 15°'nin şu anki haline göre bozulmamalı:* en yakın menzil
      medyanı ≤ 1.73 m · terminal `ok` oranı ≥ %18 · kadraj içi ≥ %67 · dikey
      artık medyanı |·| ≤ 0.9 m. Bunları tutturan **en yüksek** istasyon açısı
      kazanır.
      *Sonuç:*

### Yüksek öncelik

- [ ] **B8 — Frenleme eğrisi: mesafeye bağlı hız tavanı** · YENİ (2026-08-05)
      `control/guidance/gps_guidance.py` → `V_MAX` kullanımı
      *Neden:* `V_MAX` tek bir sabit tavan (şu an 18). 28 iken araç istasyona
      zamanında yavaşlayamayıp savruluyordu (28 m/s'den 12 m/s² ile durma
      mesafesi v²/2a = 32.7 m, istasyon standoff'u ise 10 m); o yüzden 18'e
      çekildi. Ama 18 uzakta ÇOK YAVAŞ — hedefe yetişmeyi geciktiriyor.
      Sabit tavanla ikisi aynı anda çözülemez.
      *Nasıl:* tavanı kalan mesafeye bağla —
          V_MAX_etkin = min(V_MAX_UZAK, sqrt(2 · MAX_ACCEL · kalan_mesafe))
      12 m/s² ile: 40 m → 28 m/s, 20 m → 21.9, 10 m → 15.5, 5 m → 11.0.
      Uzakta hızlı gelir, istasyona yaklaşırken kendiliğinden yavaşlar ve
      TAM istasyonda durur.
      *Ölçüt:* istasyona oturma oranı artmalı, overshoot (min d_h) küçülmeli,
      hedefe yetişme süresi kısalmalı.
      *Sonuç:*

- [ ] **B1 — Görsel faza irtifa tabanı** · A8'den ÖNCE
      `control/guidance/visual_lead.py` (veya `adapter_copter`)
      *Neden:* GPS fazında `LOOKUP_MIN_ALT = 8 m` yere çakılma koruması var
      (`gps_guidance.py:86`); **görsel fazda hiç yok**. Son üç uçuşun üçünde de
      takla = zemine çarpma; kara kutu: irtifa 8.0 → 0.2 m, 4 m/s alçalışla,
      ardından `|roll| > 90°`.
      *Nasıl:* dikey komut, drone tabana yaklaştıkça **yumuşak** kırpılacak.
      Sert kesme terminal dalışı bozar.
      *Ölçüt:* zemine çarpma **3/3 → 0**.
      *Sonuç:*

- [ ] **B5 — Fly-past davranışı** · A5 ile birlikte anlamlı
      `control/guidance/visual_lead.py` (+ muhtemelen `supervisor.py`)
      *Neden:* eskiden drone 1.5 m'ye gelince "VURULDU" deyip duruyordu;
      ıskalayıp geçtikten sonrası hiç yaşanmıyordu. A5 o erken durmayı
      kaldırınca ortaya çıkan davranış: drone hedefi geçer, "hedefe uç" komutu
      artık **yukarı-geriyi** gösterir, drone tırmanır, hedef kadrajdan çıkar,
      tespit kopar. **Her ıskadan sonra drone 5-7 m yukarı fırlıyor, toparlaması
      10-20 s.** Vuruş oranını değil görev süresini etkiliyor.
      *Ne lazım:*
      - Tespit: menzil çok küçüldü (< ~3 m) VE artık **büyüyor**, ya da hedef
        gövde çerçevesinde arkaya düştü (`u_govde[0] < 0`).
      - Davranış: terminal hamleyi bırak, tırmanmayı kes, kontrollü şekilde
        istasyon geometrisine dön. Kör tırmanışı **sürdürme**.
      - Gerekirse "yeniden hücum" sayacı.

      ⚑ **Ayrıca ölçüldü — "vuruldu"dan sonrası çok daha kötü.** Log `00000108`
      (temiz koşu, 1.07 m'de vuruldu). Vuruş anından itibaren kara kutu:

      | t (uçuş) | yaw | irtifa |
      |---:|---|---:|
      | 39.5 s | sabit 350° | 21.5 m |
      | 40–55 s | **kesintisiz dönüş, net 14.57 tur, ~350 °/s** | 21.8 → 2.0 m |
      | 55–70 s | dönüş sönüyor | 2.0 m sabit |

      `DesYaw` de aynı rampayı ~45° gerisinden izliyor — araç kusursuz
      uyguluyor, **komut kesilmemiş**. Güdüm "VURULDU" deyip CSV'yi kapatıyor
      ama son **yaw-hızı komutu araçta yaşamaya devam ediyor**. Mod hep GUIDED,
      mod değişimi yok. → B5'e **"faz biterken yaw-hızı komutunu SIFIRLA"**
      maddesi eklenmeli.
      *Ölçüt:* geçiş sonrası irtifa aşımı ve zemine çarpma 0.
      *Sonuç:*

- [ ] **B6 — Terminal algı kalitesi** · ⚠ **ÖNCELİĞİ DÜŞTÜ, kapsamı daraldı**
      *Eski gerekçe:* `ok` oranı %50 üstündeyken en yakın menzil 0.99-1.12 m
      (vuruş), %30 altındayken 2.4-12.7 m (ıska) — "asıl darboğaz algı"
      sanılıyordu.
      ⚠ **Bu gerekçe 2026-08-04'te çürütüldü.** Algı kusursuz yapıldığında
      (GT modu) isabet değişmedi: vuruş %18 → %11, en yakın mesafe medyanı
      0.92 → 0.98 m. `ok` oranı ↔ isabet bağıntısı nedensel değilmiş; ikisi de
      geometrinin sonucu. Genel "algıyı iyileştir" işi olarak **rafa kalktı**.
      *Geriye kalan gerçek algı işi:* pose'un manevra körlüğü — hedef bank
      yaparken yandanlık gerçeğin altında ölçülüyor (45° yatıkta 1.00 yerine
      0.73, test T53b). Ayrıntı ve çözüm önerisi `TODO.md` §5'te.

### Geri alınmıştı, tekrar uygulanacak (kodda ŞU AN YOK)

Bunlar bir kez yazıldı, sonra depo `b55953d`'ye döndürülünce kayboldu.
Kontrol edildi (2026-08-04): hiçbiri kodda yok.

- [ ] **A2 — MAVLink kuyruk boşaltma** · `control/gcs_server.py` → `mavlink_listener`
      *Neden:* döngü her 5 ms'de **tek** mesaj okuyor → tavan 200 msg/s. İki
      araç × 4 mesaj tipi × 25 Hz ≈ 200/s, tam sınırda. Kuyruk birikip mesajlar
      TOPLU teslim ediliyor (varış aralığı medyan 0.050 s ama **max 0.30 s**).
      Tur başına kuyruk boşaltılacak (üst sınır 400).
      *Ölçüt:* `duzensizlik_orani` **5.9 → 1.2**.
      *Sonuç:*

- [ ] **A3 — Hedef hızı aracın KENDİ saatinden** · `gcs_server.py` + `gps_guidance.py`
      *Neden:* hız `Δkonum / Δvarış` ile hesaplanıyor. Mesajlar toplu gelince
      0.25 s'de biriken hareket 0.05 s'lik aralığa bölünüp **~100 m/s sahte
      hız** üretiyor (ölçülen max 106.8; gerçek hız **14.0 m/s**). Bu sahte hız
      güdüme FEEDFORWARD giriyor (`vx = vel_x + KP_H·hata`) → araç şiddetle
      pitch'liyor.
      *Nasıl:* `LOCAL_POSITION_NED.time_boot_ms` → `telemetry_state["plane"]
      ["t_boot_ms"]` → `_noisy_plane_telem` → `gps_guidance` hız paydası.
      Damga yoksa duvar saatine düşen yedek yol.
      *Ölçüt:* `ham_konum_hizi.max` 106.8 → ~17 m/s, `imkansiz_40ustu` 0.
      *Sonuç:*

- [ ] **A4 — Hedef sıçrama kapısı (emniyet ağı)** · `gps_guidance.py` → `_HedefKapisi`
      *Neden:* A2+A3 kök nedeni çözüyor; bu, ölçüm zincirinde başka yerde
      bozulma olursa güdüm korumasız kalmasın diye. Desen
      `visual_lead._MenzilKapisi` ile aynı (T38/T38b ile testli).
      `HEDEF_HIZ_TAVAN = 35 m/s`, `HEDEF_RESENK_N = 8`, CSV'ye `hedef_red`.
      ⚠ Bir **ivme kapısı** denenmiş ve KALDIRILMIŞTI — hız kestiriminin
      sıfırdan oturmasını da engelliyordu. Tekrar eklemeyin.
      *Ölçüt:* `hedef_red` ~0 kalmalı. *Test:* 11/11 → 14/14
      *Sonuç:*

- [ ] **A6 — Tanılama endpoint'i** · `gcs_server.py` → `/api/debug/hedef_telem`
      *Neden:* A2/A3'ün kök nedenini bu ayırt etti. Salt gözlem, güdüme
      dokunmaz. İleride tekrar lazım olacak. `AVCI_IRIS_14550` bayrağı da bunda.
      *Sonuç:*

- [ ] **A8 — Görsel kilit** ⚠ **B1 ve B2 OLMADAN UYGULAMAYIN**
      `supervisor.py` + `visual_lead.py`
      *Ne yapar:* kısa tespit kopmalarında GPS'e dönülmez, son nişan komutu
      sürdürülür (`kilit_kor`); 10 s hiç tespit gelmezse pes edilir.
      *Kanıtlanan FAYDA:* faz girişi 23 → 9, ortalama süre 3.5 s → 8.9 s.
      *Ölçülen ZARAR:* karelerin **%64'ü kör uçuş** ve dondurulan komut
      TIRMANIŞ:

      | durum | kare | medyan dikey komut |
      |---|---:|---:|
      | `ok` (tespit var) | 509 | **+10.14** = aşağı |
      | `kilit_kor` (tespit yok) | 1539 | **−12.43** = yukarı |

      Tespit tam da drone hedefe doğru tırmanırken kopuyor, yani kilit
      **kaybın sebebi olan komutu** 10 s sürdürüyor → drone hedefin üstüne
      çıkıyor, zemine çakılıyor.
      *Sonuç:*

- [ ] **B2 — `kilit_kor` sırasında dikey komutu sönümle** · A8 ile birlikte
      Dondurulan komutun **dikey** bileşeni zamanla sıfıra çekilir; yatay ve yaw
      korunur (nişan yönü bilgisi hâlâ değerli).
      *Ölçüt:* `kilit_kor` karelerinde dikey komut medyanı −12.4 → 0'a yakın.
      *Sonuç:*

- [ ] **B3 — Kilit süresini kısalt** · B2'ye alternatif, daha kaba
      `supervisor.SupCfg.GORSEL_KILIT_SURE` 10 s → 1-2 s. B2 çalışırsa gerekmez.
      *Sonuç:*

- [ ] **B4 — `coalt` kapsamını daralt** · düşük öncelik
      `guidance_core.TERMINAL_COALT_DEG = 10°` yukarı yanlılık **1064 karede**
      aktifti; tırmanışı büyüten etkenlerden biri. `coalt_latch` menzil eşiğine
      bir kez girince kilitleniyor. B1+B2 sonrası hâlâ sorun varsa bakılır.
      *Sonuç:*

### Küçük / bağımsız işler

- [ ] **`LOOKUP_MIN_ALT` kararı** — şu an 8 m sabit taban
      (`gps_guidance.py:86`). Hedef yere düşüp sürünürken avcı 8 m'de asılı
      kalıyor, inemiyor. Hedefin irtifasına göre uyarlanmalı mı, yoksa "hedef
      yerdeyse görev bitti" mi sayılmalı?
- [ ] **`ATC_ANGLE_MAX`'i kademeli artır** — 45'te. 50-55 denenebilir; 55'te
      yatay ivme 14 m/s². Her denemede kara kutudan motor doygunluğu ve toplam
      yaw dönüşü kontrol edilmeli. (`sim/ardupilot_params/avci_copter.parm`)
- [ ] **Görsel faza geçiş kapısı** — şu an "son 15 karenin 10'unda pose
      `conf≥0.5`" VE yatay mesafe < 20 m. Bağlayıcı olan pose kilidi; bbox
      20 m'de görünüyor ama pose geç kilitleniyor. Girişi tespit (bbox)
      güvenine bağlamak denenebilir. (`supervisor.py:70-79`)
- [ ] **Lead'in yumuşak geçişi** — `kpt_dusuk`'ta sert 0'lanıyor, ~15° nişan
      zıplaması (58 geçiş, ort 10.8°, max 24.9°). (`guidance_core.process`)
- [ ] **Menzil verisi neden zıplıyor** — kapı semptomu kesti, kök neden duruyor.
      Baş şüpheli `gcs_server._frame_off` dikey kalibrasyonu (`sd = 0.0`
      varsayımı). Artık ölçülebilir: `menzil_ham_m` ile `gercek_menzil_ham_m`
      yan yana loglanıyor.
- [ ] **GPS fazında vuruş tespiti yok** — hasar modülü artık bağımsız izliyor
      ama faz durumu hâlâ "VURULDU" demiyor. (`gps_guidance.py`)
- [ ] **Hasar modülünü arayüze bağla** — `/api/hasar` endpoint'i var, panelde
      gösterilmiyor.
- [ ] **Video kayıt butonları** — başlat / durdur / kayıt dosyası. Iris kamera
      akışı zaten MJPEG olarak `latest_frames["iris"]` üzerinden servis ediliyor
      (`gcs_server.process_iris_frame`). Kayıt, o kareleri `cv2.VideoWriter` ile
      dosyaya yazan bir thread olur; dosyalar `logs/` altına zaman damgalı düşer.
- [ ] **RTF'i tam sistemde tekrar ölç** — `gcs_server` + YOLO yükü altında 0.982
      ölçüldü ama uçuş sırasında (görsel faz aktifken) ölçülmedi.

---

## Tekrar denenmeyecekler (ölçümle çürütüldü)

Hepsi denendi ve **ölçümle çürütüldü**. Gerekçeleri ilgili dosyada yorum
olarak duruyor; tekrar denemeden önce o yorumu oku.

| fikir | nerede yazılı | ne oldu |
|---|---|---|
| `ATC_ANG_YAW_P` 4.5 → 3.0 | `sim/ardupilot_params/avci_copter.parm:83` | yaw takip hatası 1.4° → 8.5-11.9° |
| `supervisor.KILIT_N` 10 → 7 | `control/guidance/supervisor.py:51` | faz/uçuş 3.4 → 8.0, her ölçüt kötüleşti |
| hedef hızına ivme kapısı | bu belge, A4 | hız kestiriminin sıfırdan oturmasını da engelledi |
| "araç komutu uygulamıyor" teşhisi | bu belge, "Öğrenilen dersler" | kara kutu çürüttü; takip hatası 0.1 m/s |
| `pkill` köşeli parantez hilesi | `dokumantasyon/17_KOD_scripts_tools_tests.md` | `pkill -f` için geçersiz, kendi kabuğunu öldürüyor |

**`KILIT_N` denemesinin ayrıntısı** (çünkü mantıklı görünüyordu): devir menzili
ile vuruş arasında güçlü bir bağıntı vardı (vuranlar 11.11 m'de, ıskalayanlar
9.05 m'de devraldı), kapıyı gevşetip devri uzaklaştırmak denendi. Her ölçütte
kötüleşti: faz/uçuş 3.4 → 8.0, giriş menzili medyanı 10.00 → 9.62 m (**düştü**),
en yakın menzil medyanı 1.73 → 2.08 m, vuruş 3/17 → 1/8. Mekanizma: kapı cılız
tespitte de açılıyor, erken devir gerçekten oluyor ama 0.9-1.3 s'de ölüyor,
GPS'e dönülüyor, drone bu arada yaklaşıyor, sonraki devir DAHA YAKINDA oluyor.
**Ders:** devir menzili ↔ vuruş bağıntısı nedensel değil; ikisi de "tespit o an
gerçekten sağlam mı"ya bağlı. Kapı sağlamlık üretmiyor.

**Kural:** "araç komutu uygulamıyor" demeden önce MUTLAKA kara kutuda
`PSCD.DVD` vs `PSCD.VD` (dikey) veya `ATT.DesYaw` vs `ATT.Yaw` (yaw)
karşılaştır.

---

## Tekrar denenebilecekler

Bunlar denenip geri alındı — ama o ölçümler sistemin bugünkünden çok daha kötü
olduğu dönemde alındı. Başarısızlıkların sebebi bu fikirler değil, altlarındaki
bozuk zemin olabilir. Aradan düzelenler: yaw sürekli dönmesi (27 °/s → ~0),
firmware parametrelerinin hiç uygulanmaması (araç 30° eğim tavanıyla uçuyordu),
dikey ıska geometrisi, temiz tespit oranı (%12 → %65).

Her denemede **tek değişken** değiştirin ve eski sayıyla karşılaştırın.

- [ ] **Gerçek PN (`γ += N·Δλ`)** — klasik oransal seyrüsefer.
      *Eski sonuç:* testleri geçti ama kapalı çevrimde ıska **0.66 → 1.5-2.1 m**.
      Sebep: drone hedefe yakınsarken LOS açısı zaten doğal olarak azalıyor;
      PN bunu "sıfırlanacak LOS hızı" sanıp yakınsamayla savaşıyordu.
      *Neden şimdi farklı olabilir:* o ölçümler dikey ıska geometrisi
      düzeltilmeden önce alındı; PN'in "büyük başlangıç ofsetini kapatma" yükü
      artık yok.
      *Ölçüt:* ıska 0.55 m'nin altına inmeli.

- [ ] **Dikey PN'i güçlendirme** (tavan 15°→30°, süre 0.4→0.6 s)
      *Eski sonuç:* PN yeni tavana da %79 oranında çakıldı.
      *Neden şimdi farklı olabilir:* doygunluğun sebebi büyük olasılıkla aracın
      ivme tavanının 5.7 m/s²'de kilitli olmasıydı (`ATC_ANGLE_MAX=30`
      varsayılanı). Artık 45° → 9.8 m/s².
      *Ölçüt:* tavana çakılma oranı %79'un belirgin altına inmeli.

- [ ] **`KP_KADRAJ ≥ 1.0`** — kadraj tutma kazancını yükseltme.
      *Eski sonuç:* yüksek kazanç yakınsamayla savaşıp salınım üretti.
      Tarama: 0.0 → ıska 0.59 m, 0.5 → **0.55 m (seçilen)**, 1.0 → 1.59 m,
      1.5 → 4.90 m. Vuruş: 1.0'da 20/24, 1.5'te **0/24**.
      *Neden şimdi farklı olabilir:* o salınımın bir kısmı yaw kaçağından ve
      tespit kopukluğundan geliyor olabilir; ikisi de düzeldi.

- [ ] **Yaw'ı mutlak hedefe slew etme** — kalıcı `cmd_yaw` durumu tutup GPS
      fazındaki desene benzetme.
      *Eski sonuç:* arıza koşulunda mevcut biçim 1.0 tur dönerken bu biçim
      **7.4 tur** döndü. Mevcut biçim komutu her karede aracın gerçek başlığına
      yeniden demirliyor — bu bir **güvenlik özelliği**.
      *Neden şimdi farklı olabilir:* artık yaw kaçak kapısı var.
      *Ölçüt:* T44/T45 geçmeli, kaçak 45°'nin altında kalmalı.

---

## Sözlük

| terim | anlamı |
|---|---|
| **kara kutu** | ArduPilot'un kendi uçuş kaydı (`~/ardupilot/logs/*.BIN`). Aracın gördüğü attitude, motor çıkışları, kontrolcü hedefleri. Bizim CSV'lerimizden bağımsız — "araç komutu uyguladı mı" sorusunun tek dürüst kaynağı. |
| **istasyon** | GPS fazının drone'a "şurada dur" dediği hayali nokta. Sabit metre DEĞİL sabit AÇI: hedeften `RANGE_SET` (11 m) uzakta, LOS yükselişi `ISTASYON_ELEV_DEG` (15°) → hedefin **10.63 m gerisi + 2.85 m altı** (25°'de 9.97 m + 4.65 m olurdu). Drone hedefi değil bu noktayı takip eder. |
| **faz** | GPS fazı (uzaktan yaklaşma, `gps_guidance`) ↔ görsel faz (terminal hücum, `visual_lead`). Geçişi `supervisor` yönetir. |
| **geçiş sayısı** | GPS→görsel kaç kez geçildi. 1 ideal; yüksek sayı görsel temasın kopup kopup kurulduğunu gösterir. |
| **`ok` oranı** | `visual_lead` her kamera karesine `durum` etiketi yazar. `ok` = pose modeli hedefi temiz gördü, keypoint'ler güvenilir. Diğerleri: `kpt_dusuk`, `tespit_yok`, `kor_dalis`. "%51 ok" = karelerin yarısında güdüm sağlam veriyle çalışıyor. |
| **fly-past** | Drone hedefe temas etmeden yanından geçmesi. Sonrasında "hedefe uç" komutu yukarı-geriyi gösterir → kontrolsüz tırmanma. Bkz. B5. |
