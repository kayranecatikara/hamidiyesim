# UYGULANACAK — teker teker, ölçerek

Aşağıdakiler **tek tek** uygulanacak; her maddeden sonra uçulup ölçülecek,
sonuç maddenin altına yazılacak. Bir madde bitmeden diğerine geçilmeyecek.

**Neden böyle:** bir keresinde 8 grup değişiklik bir arada uçuruldu. Bazıları
ölçümle işe yaradı, biri ölçülebilir zarar verdi (görsel kilit: kör uçuş %64,
drone hedefin üstüne çıkıp zemine çakıldı). Hangisinin ne yaptığı ayırt
edilemedi.

**Kural:** her adımda `python3 -m tests.test_visual_lead` ve
`python3 -m tests.test_gps_guidance` çalıştırılır.

---

> Başka bir makinede/dalda devam edeceksen önce **[DEVAM.md](DEVAM.md)**:
> dal senkronu, sistem başlatma, ölçüm araçları, laptop'ta ayrıca gerekenler.

## DURUM — 2026-08-02 22:30 (yeni oturum buradan devam etsin)

**Depo:** `kubra_masaustu`, temiz, `origin` ile eşit. Son commit `f5737ca`.
Açık PR: **#4** (`gh pr view 4`). Testler: **53/53** ve **12/12**.

**Bitenler:** A1 ✓ · A5 ✓ · dikey ıska 2. tur (istasyon 25° → 15°) ✓ ·
başlatma/durdurma script hataları ✓
**Sıradakiler:** A2, A3, A4, A6, A7 hâlâ **kodda YOK** · B1, B2, B3, B4, B5,
B6 hiç uygulanmadı · A8 (görsel kilit) B1+B2 olmadan uygulanmayacak

**Ölçülen son hal** (A5 + 15° istasyon, 17 geçiş):
en yakın menzil medyanı **1.73 m**, vuruş **3/17**, faz/uçuş **3.4**.
Vuruşu belirleyen tek güçlü değişken: vuran 4 geçişin dördünde de
`kor_dalis` ≤ **%3**, ıskalayanlarda medyan %19-27.

**Bir sonraki iş — üçünden biri:**
- **B7 (açık soru)** — istasyon açısını kamera tilt'inden ayırmak doğru muydu?
  Ölçümler olumlu ama karışık; merkez dışı kadrajlamanın kendi bedeli izole
  edilmedi ve asıl alternatif (`WP_ACC_Z` yükseltmek) hiç denenmedi.
  B6'dan önce karara bağlanmalı — B6 algıyla uğraşacak, algının geometriden
  ne kadar etkilendiği belirsizken çalışmak boşa gider.
- **B6 (terminal algı sürekliliği)** — asıl kaldıraç. Hedefin son 1-2 s'de
  kadrajda kalması. `kpt_dusuk` terminalde %30-60.
- **B5 (fly-past)** — her ıskadan sonra drone 5-7 m yukarı fırlıyor,
  toparlaması 10-20 s. Vuruş oranını değil görev süresini etkiliyor. A5
  sonrası artık HER ıskada yaşanıyor. Ek olarak ölçüldü: faz biterken yaw
  **hız** komutu iptal edilmiyor (bkz. B5 altındaki ⚑ notu).

⚠ **Tekrar denenmeyecekler** (ölçümle çürütüldü, gerekçeleri ilgili yerlerde):
`ATC_ANG_YAW_P` düşürmek · `supervisor.KILIT_N` düşürmek · hedef hızına
ivme kapısı · "araç dikey/yaw komutunu uygulamıyor" teşhisi

---

## Yeni oturuma başlıyorsan

**Sırayla git, atlama.** Her madde: uygula → testler → **uç** → ölç →
*Sonuç:* satırına yaz → tikle. Bir madde bitmeden diğerine geçme. Bu kural
var çünkü hepsi bir arada uygulanınca hangisinin ne yaptığı ayırt edilemedi.

**Ölçüm yöntemi — CSV'ye tek başına güvenme.** Geometri sorularının dürüst
kaynağı iki aracın kara kutusu: her iki `.BIN`'den `POS` (Lat/Lng/Alt) alınıp
`GPS.GWk`+`GPS.GMS` ile ortak saate hizalanır, sonra aradaki yatay/dikey
mesafe hesaplanır. CSV'deki `menzil_gercek_m` EKF çerçeve ofsetinden
etkileniyor ve en yakın anı geriden gösteriyor.

**Sistemi başlatma** (iki terminal, ayrıntı `docs/SIMULASYON_CALISTIRMA.md`):

```bash
# Terminal A
GZ_HEADLESS=1 bash scripts/start_harmonic.sh    # eski surecleri kendi temizler
# durdurmak : bash scripts/start_harmonic.sh stop    (Ctrl+C ISE YARAMAZ)
# kontrol    : bash scripts/start_harmonic.sh durum  (hicbir seyi oldurmez)
# ELLE pkill -9 -f 'gz sim|sim_vehicle|...' KULLANMA — kendi kabugunu oldurur.

# Terminal B
source /opt/ros/humble/setup.bash && export AVCI_GZ_CAMERA=1 AVCI_NO_BROWSER=1
fuser -k 8000/tcp 2>/dev/null; python3 -m control.gcs_server
```

### Sözlük

| terim | anlamı |
|---|---|
| **kara kutu** | ArduPilot'un kendi uçuş kaydı (`~/ardupilot/logs/*.BIN`). "Araç komutu uyguladı mı" sorusunun tek dürüst kaynağı. |
| **istasyon** | GPS fazının "şurada dur" dediği hayali nokta. Sabit metre DEĞİL, sabit AÇI: hedeften `RANGE_SET`(11 m) uzakta, LOS yükselişi `ISTASYON_ELEV_DEG`. 2026-08-02'de bu açı kamera tilt'inden (25°) ayrılıp **15°**'ye indirildi → 10.63 m geride + **2.85 m** altta (eskiden 9.97 m + 4.65 m). Sebep: terminalin kapatacağı dikey mesafe aracın 1 m/s²'lik dikey ivme bütçesine sığmıyordu. Drone hedefi değil bu noktayı takip eder. B5'teki "istasyona dön" = ıskaladıktan sonra kontrollü şekilde bu bekleme noktasına dönüp yeni hücuma hazırlanmak. |
| **`ok` oranı** | `visual_lead` her kareye `durum` yazar. `ok` = pose temiz, keypoint'ler güvenilir. Diğerleri `kpt_dusuk` / `tespit_yok` / `kor_dalis`. "%51 ok" = karelerin yarısında güdüm sağlam veriyle çalışıyor. |
| **faz** | GPS fazı (yaklaşma) ↔ görsel faz (terminal hücum). Geçişi `supervisor` yönetir. |
| **fly-past** | Drone hedefe temas etmeden yanından geçmesi. Sonrasında "hedefe uç" komutu yukarı-geriyi gösterir → kontrolsüz tırmanma. Bkz. B5. |

### Bu oturumda öğrenilen — tekrarlamayın

- **"Araç komutu uygulamıyor" demeden önce kara kutuya bakın.** Bu teşhis bir
  kez kondu ve çürütüldü: alçalma emredilen anlarda `PSCD.DVD +6.36` iken
  `VD +6.43`, takip hatası 0.1 m/s. Araç kusursuz uyguluyordu.
- **Tek seferde tek değişken.** Üç kez birden fazla şey değiştirildi ve
  hangisinin ne yaptığı ayırt edilemedi.
- **Ölçmeden değer değiştirmeyin.** `ATC_ANG_YAW_P 4.5 → 3.0` iyi niyetle
  yapıldı, yaw takip hatasını 1.36° → 4.94°'ye çıkardı.
- **Bozuk veriyle ölçüm yapmayın.** Hedefin hızı bir süre `tgt_vx` sütunundan
  17.5 m/s sanıldı; o sütun zaten bozuk olduğu kanıtlanan kestirimdi. Gerçek
  değer ArduPlane kara kutusundan **14.0 m/s** çıktı.

---

## Önerilen sıra

```
A1 → A2 → A3 → A4 → A6    kanıtlı / düşük risk (birlikte uçulabilir)
A7                         belge, uçuş gerektirmez
B1                         güvenlik (irtifa tabanı) — kendi uçuşu
A5 + B5                    gerçek temas + fly-past davranışı — BİRLİKTE
A8 + B2                    görsel kilit + dikey sönümleme; B1 olmadan ASLA
B6                         terminal algı kalitesi (asıl darboğaz, uzun iş)
B3 / B4                    yalnız gerekirse
```

**A5 ve B5 neden birlikte:** A5 erken durmayı kaldırıyor, B5 ondan sonra ne
olacağını tanımlıyor. A5 tek başına uygulanırsa drone hedefi geçtikten sonra
kontrolsüz tırmanır — kilit olmasa bile.

**B1 neden A5'ten önce:** görsel fazda yere çakılma koruması yok. Fly-past
denemeleri sırasında drone alçalabilir; taban olmadan zemine girer.

---

## KARAR: (b) — gerçek temas ölçütü

2026-08-02, push haliyle 18 görsel faz ölçüldü. İki seçenek vardı:

**(a)** 1.5 m yakınlığı vuruş saymak — ekranda güzel görünür, görev "başarılı"
biter, ama gerçekte ıskaladığımızı bilmeyiz.
**(b)** Gerçek fiziksel temas — dürüst, ama şu an çoğu denemede vuramıyoruz.

**(b) seçildi.** Gerekçe: push halinin "başarısı" kısmen erken durmadan
geliyor. Drone 1.5 m'ye gelince "VURULDU" deyip güdüm DURUYOR; hedefin
yanından geçtikten sonra ne olacağıyla hiç yüzleşmiyor. Gerçek temas ölçütü
bu sorunu **yaratmadı, görünür kıldı**.

### Ölçülen gerçek: terminal isabet tespit kalitesine bağlı

18 görsel fazın en yakın yaklaşmaları (push hali, hiç değişiklik yok):

| en yakın menzil | `ok` oranı | vuruldu |
|---:|---:|:---:|
| **0.99 m** | %51 | ✓ |
| **1.12 m** | %68 | ✓ |
| 2.42 – 3.13 m | %6 – 22 | — |
| 4.23 – 6.80 m | %5 – 50 | — |
| 10.24 – 12.66 m | %0 – 27 | — |

`ok` oranı %50'nin üstündeyken 1 m altına iniliyor; %30'un altındayken 2-12 m
ıskalanıyor. ~~**Asıl darboğaz terminal algı kalitesi.**~~

### ⚑ DÜZELTME (2026-08-02, A5 sonrası 3 uçuş): darboğaz algı DEĞİL, DİKEY BÜTÇE

Yukarıdaki "asıl darboğaz algı" çıkarımı **korelasyonu nedenle karıştırmış**.
A5'ten sonra üç uçuş kara kutuyla (iki aracın `POS` mesajları, GPS haftası
saatiyle hizalanmış) ölçüldü. Üçü de terminale **aynı** geometriyle giriyor:
yatay ~12.5 m, dikey **+4.65 m** (istasyon ofseti), hedef düz uçuyor.

**Kök neden: ArduPilot dikey hız komutunu 1.0 m/s² ile rampalıyor.**
Ölçüldü — `PSCD.DVD`'nin pozitif eğim medyanı üç uçuşta da tam **1.00 m/s²**
(`WP_ACC_Z = 1.0`). Güdüm 8-22 m/s tırmanma istiyor, tavan (`WP_SPD_UP = 5.0`)
hiç görülmüyor (en yüksek DVD 2.13-2.76) — yani **hız değil, İVME sınırlıyor.**
Araç kusursuz uyguluyor: DVD↔VD takip hatası 0.1 m/s, gaz hiç %95'i aşmıyor,
RCOU doygun değil. Komutun büyüklüğü tamamen alakasız.

Sıfırdan 4.65 m kapatmak 1 m/s²'de **3.05 s** sürer. Elde olan süre:

| | **A (vurdu)** | B (ıska) | C (ıska) |
|---|---:|---:|---:|
| görsel faza giriş menzili (3B) | **10.32 m** | 7.65 m | 9.16 m |
| faz başından en yakın ana | **2.64 s** | 2.38 s | 2.77 s |
| yatay 9 m'de DVD (tırmanma) | **+0.46** | +0.40 → 0.37 düştü | +0.19 → 0.09 düştü |
| kapatılan dikey | **4.25 m** | 2.70 m | 2.42 m |
| gereken dikey | 4.28 m | 4.22 m | 4.48 m |
| **en yakın anda kalan dikey** | **+0.03 m** | **+1.52 m** | **+2.06 m** |
| sonuç | **GERÇEK TEMAS** | alttan geçti | alttan geçti |

Üçünün de süresi 3.05 s'nin altında. A yalnızca **rampayı erken başlattığı**
için yetişti: görsel faza 10.3 m'de girdi ve 9 m'ye geldiğinde zaten
0.46 m/s tırmanıyordu (≈1.2 m'lik avans). B 7.65 m'de, C 9.16 m'de girdi ve
rampaları başta duraksadı.

**Algı çöküşü SONUÇ, sebep değil.** Drone altta kaldıkça hedef kadrajın
üstünden çıkıyor — 4-2 m bandında ölçülen:

| 4-2 m bandı | A | B | C |
|---|---:|---:|---:|
| `gercek_kadraj_ici` | **%69** | %21 | %15 |
| `gercek_v_px` (görüntü yüksekliği 480) | **336** | 20 | −1 |
| son 1.5 s'de `kor_dalis` kare | **1/46** | 30/46 | 29/46 |
| `pn_dikey_deg` | −1.7° | +21.9° | **+30.0° (tavanda)** |

Kısır döngü: dikey geride kalır → hedef kadrajın tepesinden çıkar → tespit
ölür → `kor_dalis` komutu dondurur → düzeltme büsbütün biter. Ama halkanın
başı dikey bütçe.

**Bunun üç maddeye etkisi:**
- **B6** yeniden çerçevelenmeli: algıyı düzeltmek dikey bütçeyi düzeltmez.
  Önce dikey, sonra algı — sıra bu.
- ⚠ **DENENDİ VE GERİ ALINDI: `supervisor.KILIT_N` 10 → 7.** Devir menzili ile
  vuruş arasında güçlü bir bağıntı vardı (vuranlar 11.11 m'de, ıskalayanlar
  9.05 m'de devraldı), kapıyı gevşetip devri uzaklaştırmak denendi. Her
  ölçütte kötüleşti: faz/uçuş 3.4 → 8.0, giriş menzili medyanı 10.00 → 9.62 m
  (**düştü**), en yakın menzil medyanı 1.73 → 2.08 m, `kor_dalis` medyanı
  %19 → %27, 1.5 s'den kısa kopan faz 2/17 → 4/8, vuruş 3/17 → 1/8.
  Mekanizma: kapı cılız tespitte de açılıyor, erken devir gerçekten oluyor
  (14.73 ve 10.47 m) ama 0.9-1.3 s'de ölüyor, GPS'e dönülüyor, drone bu arada
  yaklaşıyor, sonraki devir DAHA YAKINDA oluyor.
  **Ders:** devir menzili ↔ vuruş bağıntısı nedensel değil; ikisi de "tespit
  o an gerçekten sağlam mı"ya bağlı. Kapı sağlamlık üretmiyor.
- Yeni aday: `WP_ACC_Z` (1.0 m/s²) terminalde geçici olarak yükseltilebilir
  mi, ya da istasyonun 4.65 m'lik dikey ofseti küçültülebilir mi? İkisi de
  **ölçülmeden değiştirilmeyecek** — bkz. `ATC_ANG_YAW_P` dersi.
- Devir menzili (`supervisor.GATE_MENZIL`/kilit koşulu) doğrudan dikey süreyi
  belirliyor: A 10.3 m'de devraldı ve vurdu, B 7.65 m'de devraldı ve 1.5 m
  alttan geçti. Erken devir = daha çok tırmanma süresi.

Not: 0.61 m'de bile Gazebo temas sensörü tetiklenmedi (ölçüldü). Yani gerçek
temas için ~0.3 m daha kapatmak gerekiyor.

---

## A) Push sonrası yapılanlar — geri alındı, tekrar uygulanacak

- [ ] **A1 — `ATC_ANG_YAW_P 3.0` satırını kaldır**
      `sim/ardupilot_params/avci_copter.parm`
      *Neden:* push'ta bu satır vardı ve zararlıydı. Ölçüm — 4.5'te yaw takip
      hatası std **1.36°**, seyirde dönme ~0 °/s; 3.0'da std **4.94°** ve
      **11.96 tur** dönme. Kazancı düşürmek aracın komut edilen başlığı
      yakalamasını yavaşlatıyor; düzeltmeye çalıştığımız şeyi 4 katına çıkardı.
      Satırı silmek varsayılan 4.5'e döndürür.
      *Ölçüt:* kara kutuda `ATT.Yaw − ATT.DesYaw` std'si ~1.4°; toplam yaw
      dönüşü 1 turun altı.
      *Sonuç:* **UYGULANDI — 1. ölçüt geçti, 2. ölçüt yanlışmış.**
      Kara kutuda `ATC_ANG_YAW_P = 4.5` doğrulandı (log 105/107/108).
      Aynı oturumda temiz A/B (4 kopter kaydı) — sabit-başlık (DesYaw ±5°,
      ≥8 s) dilimlerinde takip hatası std:

      | log | P | dilim | süre | std | \|max\| |
      |---|---|---:|---:|---:|---:|
      | 00000100 | 3.0 | 3 | 45 s | 11.88° | 47.5° |
      | 00000103 | 3.0 | 2 | 54 s | 8.53° | 45.0° |
      | 00000105 | 4.5 | 3 | 46 s | **1.32°** | 3.8° |
      | 00000107 | 4.5 | 5 | 133 s | 4.64° | 44.6° |
      | 00000108 | 4.5 | 1 | 22 s | **1.43°** | 2.5° |

      Bozulmanın **karakteri** de değişti: 3.0'da kalıcı sapma (log 100,
      36-45 s: ortalama hata −15.3°, karelerin %32'si >20° — araç başlığı
      yakalayamıyor), 4.5'te yalnız anlık sıçrama (log 107, 136-222 s:
      ortalama +0.60°, %2'si >20°). Motor doygunluğu yok (RCOU max 1866).

      ⚠ **2. ölçüt (toplam dönüş < 1 tur) kullanılamaz — P ile ilgisi yok.**
      Net dönme: 3.0 → 0.102 ve 0.150 tur/s; 4.5 → 0.176, 0.045, 0.238 tur/s.
      Korelasyon yok. Belgedeki "3.0 → 11.96 tur" bir P-kazancı etkisi değilmiş:
      dönme sabit-başlık dilimlerinin DIŞINDA, `DesYaw`'ın kendisi dönerken
      oluyor → güdümün komut ettiği dönme. Bkz. aşağıdaki B5 notu.

- [ ] **A2 — MAVLink kuyruk boşaltma**
      `control/gcs_server.py` → `mavlink_listener`
      *Neden:* döngü her 5 ms'de **tek** mesaj okuyordu → tavan 200 msg/s.
      İki araç × 4 mesaj tipi × 25 Hz ≈ 200/s, tam sınırda. Kuyruk birikip
      mesajlar TOPLU teslim ediliyordu (varış aralığı medyan 0.050 s ama
      **max 0.30 s**). Tur başına kuyruk boşaltılacak (üst sınır 400).
      *Ölçüt:* `/api/debug/hedef_telem` → `varis_araligi_s.duzensizlik_orani`
      **5.9 → 1.2**.
      *Sonuç:*

- [ ] **A3 — Hedef hızı aracın KENDİ saatinden**
      `control/gcs_server.py` + `control/guidance/gps_guidance.py`
      *Neden:* hız `Δkonum / Δvarış` ile hesaplanıyordu. Mesajlar toplu
      gelince 0.25 s'de biriken hareket 0.05 s'lik aralığa bölünüp **~100 m/s
      sahte hız** üretiyordu (ölçülen max 106.8; Talon'un gerçek hızı
      **medyan 14.0 m/s**, ArduPlane kara kutusu GPS.Spd ile doğrulandı).
      Bu sahte hız güdüme FEEDFORWARD olarak giriyor
      (`vx = vel_x + KP_H·hata`) → araç şiddetle pitch'liyor.
      *Nasıl:* `LOCAL_POSITION_NED.time_boot_ms` →
      `telemetry_state["plane"]["t_boot_ms"]` → `_noisy_plane_telem` →
      `gps_guidance` hız paydası. Damga yoksa duvar saatine düşen yedek yol.
      *Ölçüt:* `ham_konum_hizi.max` **106.8 → ~17 m/s**, `imkansiz_40ustu` 0,
      medyan ~14 (gerçek hızla uyuşmalı).
      *Sonuç:*

- [ ] **A4 — Hedef sıçrama kapısı (emniyet ağı)**
      `control/guidance/gps_guidance.py` → `_HedefKapisi` + testler G11-G13
      *Neden:* A2+A3 kök nedeni çözüyor; bu, ölçüm zincirinde başka bir yerde
      bozulma olursa güdüm korumasız kalmasın diye. Desen
      `visual_lead._MenzilKapisi` ile aynı (kanıtlı, T38/T38b ile testli):
      imkânsız sıçrama reddedilir, son geçerli değer korunur, ısrarlı redde
      yeniden senkronize olunur.
      `HEDEF_HIZ_TAVAN = 35 m/s` (gerçek tepe 21.8'in belirgin üstü),
      `HEDEF_RESENK_N = 8`, CSV'ye `hedef_red` sütunu.
      ⚠ Ayrıca bir **ivme kapısı** denendi ve KALDIRILDI — hız kestiriminin
      sıfırdan oturmasını da engelliyordu (G9 yakaladı). Tekrar eklemeyin.
      *Ölçüt:* `hedef_red` ~0 kalmalı. 0 değilse kök neden geri gelmiş demektir.
      *Test:* 11/11 → 14/14
      *Sonuç:*

- [ ] **A5 — Gerçek çarpışma tespiti**
      `sim/gazebo_harmonic/worlds/avci_harmonic.sdf` (contact-system eklentisi)
      `sim/gazebo_harmonic/models/mini_talon_vtail/model.sdf` (`carpisma_sensoru`)
      `control/carpisma_state.py` (YENİ) · `control/gcs_server.py` (hasar modülü)
      `control/guidance/visual_lead.py` (`_vurus_oldu`) · testler T46-T48
      *Neden:* eski hâli **1.5 m yakınlığı** vuruş sayıyordu. Ölçüldü — bir
      koşuda 0.61 / 0.69 / 0.75 / 1.06 / 1.16 / 1.20 m yaklaşma vardı, yani
      **6 sahte vuruş** raporlanırdı. Yakınlık çarpışma değildir. Dahası sahte
      vuruş güdümü DURDURUYOR; drone tam hızla giderken komutsuz kalıp
      savruluyordu.
      *Ayrıntı:* sensör gövde+kanat+kuyrukta; **tekerlek DAHİL DEĞİL** (pistte
      sürekli yere değiyor, her kalkışta sahte çarpışma üretirdi). Karşı taraf
      iris değilse imha sayılmaz (zemine çarpma elenir). `VURUS_MENZIL` (1.5 m)
      artık yalnız temas kaynağı yoksa devreye giren **yedek** ölçüt.
      *Ölçüt:* ıskalayıp yanından geçince "VURULDU" **yazmamalı** ve hedef
      düşmemeli; gerçekten çarpınca `VURULDU — GERÇEK TEMAS` yazmalı ve hedef
      düşmeli. Açılışta `[HASAR] GERÇEK çarpışma dinleniyor` satırı görünmeli.
      *Test:* +3 (T46-T48)
      *Sonuç:* **UYGULANDI (kullanıcı isteğiyle sıradan önce çekildi).**
      Testler 50/50 → **53/53**, GPS 11/11 bozulmadı.

      Yerinde doğrulandı (uçuş değil, sistem ayaktayken):
      - `gz topic -l` → `/world/avci/model/mini_talon/link/base_link/sensor/`
        `carpisma_sensoru/contact` **var** (varsayılan `_HASAR_TOPIC` ile birebir).
      - Topic dinlendi: uçak pistte dururken
        `mini_talon::base_link::fuselage_collision ↔ grass_field::link::collision`
        akıyor. Karşı tarafta `iris` geçmediği için süzgeç doğru şekilde
        **imha saymıyor**.
      - `gcs_server` açılışı: `[HASAR] GERÇEK çarpışma dinleniyor: ...` +
        `vuruş ölçütü = fiziksel temas`.
      - `GET /api/debug/carpisma` → `{"temas":false, ..., "kaynak_hazir":true}`.

      Ek olarak (belgede yoktu, gerekliydi): `start_chase` ve `start_visual`
      artık temas mandalını sıfırlıyor. Mandal latch'li olduğu için önceki
      denemede gelen temas, yeni görsel fazın İLK karesinde "vuruldu"
      dedirtiyordu.
      ⚠ **Uçuşta beklenen:** 18 fazın yalnız 2'si 1.5 m altına iniyordu ve
      0.61 m'de bile temas sensörü tetiklenmemişti. Yani bu maddeden sonra
      "VURULDU" oranının **düşmesi** normal — hata değil, dürüstlük. Asıl iş
      B6 (terminal algı) ve B5 (geçiş sonrası davranış).

- [ ] **A6 — Tanılama endpoint'i + `AVCI_IRIS_14550` bayrağı**
      `control/gcs_server.py` → `/api/debug/hedef_telem`
      *Neden:* A2/A3'ün kök nedenini bu ayırt etti (varış düzensizliği mi,
      ham veri mi, `_frame_off` mu, GPS gürültü slider'ı mı). Salt gözlem,
      güdüme dokunmaz. İleride tekrar lazım olacak.
      *Sonuç:*

- [ ] **A7 — TODO.md güncellemeleri**
      Özellikle **"⚠ YANLIŞ TEŞHİS"** bölümü: *"araç dikey komutu
      uygulamıyor"* iddiası kara kutu `PSCD` ile çürütüldü — alçalma emredilen
      anlarda DVD +6.36 iken VD +6.43, takip hatası ~0.1 m/s, ve "aşağı
      emredildi ama yukarı gidiyor" örneği **0/3648**. Hata: 5 saniyelik bir
      CSV penceresine bakılmış, araç o sırada mevcut bir tırmanışı tersine
      çeviriyormuş, aracın kendi kaydına bakılmamış.
      **Kural:** "araç komutu uygulamıyor" demeden önce MUTLAKA kara kutuda
      `PSCD` (dikey) veya `ATT.DesYaw` (yaw) ile istenen–gerçekleşen
      karşılaştırılacak.
      *Uçuş gerektirmez.*
      *Sonuç:*

- [ ] **A8 — Görsel kilit** ⚠ **B1 ve B2 OLMADAN UYGULAMAYIN**
      `control/guidance/supervisor.py` + `control/guidance/visual_lead.py`
      + testler T49-T50
      *Ne yapar:* kısa tespit kopmalarında GPS'e dönülmez, son nişan komutu
      sürdürülür (`kilit_kor` durumu); 10 s hiç tespit gelmezse pes edilir.
      *Kanıtlanan FAYDA:* faz girişi **23 → 9**, ortalama süre **3.5 s → 8.9 s**
      (en uzun 27.8 s).
      *Ölçülen ZARAR:* karelerin **%64'ü `kilit_kor`** (kör uçuş) ve dondurulan
      komut TIRMANIŞ:

      | durum | kare | medyan dikey komut |
      |---|---:|---:|
      | `ok` (tespit var) | 509 | **+10.14** = aşağı |
      | `kilit_kor` (tespit yok) | 1539 | **−12.43** = yukarı |

      Tespit tam da drone hedefe doğru tırmanırken kopuyor (hedef kadrajdan
      çıkıyor), yani kilit **kaybın sebebi olan komutu** 10 s sürdürüyor.
      Sonuç: drone hedefin üstüne çıkıyor, toparlayamıyor, zemine çakılıyor.
      *Ölçüt:* faz girişi < 5; `kilit_kor` karelerinde dikey komut medyanı
      0'a yakın (B2 ile); zemine çarpma 0 (B1 ile).
      *Test:* +2 (T49-T50)
      *Sonuç:*

---

## B) Öneriler — henüz hiç uygulanmadı

- [ ] **B1 — Görsel faza irtifa tabanı** · öncelik **YÜKSEK**, A8'den ÖNCE
      `control/guidance/visual_lead.py` (veya `adapter_copter`)
      *Neden:* GPS fazında `LOOKUP_MIN_ALT = 8 m` yere çakılma koruması var
      (`gps_guidance.py:50`); **görsel fazda hiç yok**. Son üç uçuşun üçünde de
      takla = zemine çarpma; kara kutu: irtifa **8.0 → 0.2 m**, 4 m/s alçalışla,
      ardından `|roll| > 90°`. Kilit olsun olmasın bu bağımsız bir eksiklik.
      *Nasıl:* dikey komut, drone tabana yaklaştıkça **yumuşak** kırpılacak.
      Sert kesme terminal dalışı bozar; taban yaklaşımında oransal sönümleme.
      *Ölçüt:* zemine çarpma **3/3 → 0**; en düşük irtifa tabanın altına
      inmemeli.
      *Sonuç:*

- [ ] **B2 — `kilit_kor` sırasında dikey komutu sönümle** · A8 ile birlikte
      `control/guidance/visual_lead.py`
      *Neden:* kör uçarken kaybın SEBEBİ olan tırmanışı sürdürmek yanlış.
      *Nasıl:* dondurulan komutun **dikey** bileşeni zamanla sıfıra çekilir;
      yatay ve yaw korunur (nişan yönü bilgisi hâlâ değerli).
      *Ölçüt:* `kilit_kor` karelerinde dikey komut medyanı **−12.4 → 0'a yakın**.
      *Sonuç:*

- [ ] **B3 — Kilit süresini kısalt** · B2'ye alternatif, daha kaba
      `supervisor.SupCfg.GORSEL_KILIT_SURE` 10 s → 1-2 s.
      Mevcut kör dalış (`_terminal_adim` / `TERMINAL_SURE`) zaten kısa
      tutulmuş; faz seviyesindeki kilidi 10 s yapmak o dersi görmezden
      gelmekti. B2 çalışırsa gerekmez.
      *Sonuç:*

- [ ] **B5 — FLY-PAST DAVRANIŞI** · öncelik **YÜKSEK**, A5 ile birlikte
      `control/guidance/visual_lead.py` (+ muhtemelen `supervisor.py`)
      *Neden:* push halinde drone 1.5 m'ye gelince "VURULDU" deyip **duruyor**;
      ıskalayıp yanından geçtikten sonrası hiç yaşanmıyor. A5 (gerçek temas)
      o erken durmayı kaldırınca ortaya çıkan davranış şu: drone hedefi geçer,
      "hedefe uç" komutu artık **yukarı-geriyi** gösterir, drone tırmanır,
      hedef kadrajdan çıkar, tespit kopar. Kilit varsa kör tırmanışa dönüşür;
      kilit yoksa bile kontrolsüz bir yukarı hamle olur.
      **A5 tek başına uygulanırsa bu sorun kilit olmasa da gelir.**
      *Ne lazım:* "geçtim" durumunun tespiti ve ondan sonrası için ayrı bir
      davranış. Kaba taslak — uygulamadan önce ölçülecek:
      - Tespit: menzil çok küçüldü (< ~3 m) VE artık **büyüyor**, ya da hedef
        gövde çerçevesinde arkaya düştü (`u_govde[0] < 0`).
      - Davranış: terminal hamleyi bırak, tırmanmayı kes, kontrollü şekilde
        istasyon geometrisine dön. Kör tırmanışı **sürdürme**.
      - Gerekirse "yeniden hücum" sayacı: kaç kez denendi, ne zaman vazgeç.
      *Ölçüt:* geçiş sonrası irtifa aşımı ve zemine çarpma 0; drone kontrollü
      şekilde yeni bir yaklaşmaya geçebilmeli.

      ⚑ **2026-08-02'de ÖLÇÜLDÜ — "vuruldu"dan sonrası çok daha kötü.**
      Log `00000108` (temiz koşu, 1.07 m'de `vuruldu`). Vuruş anından itibaren
      kara kutu:

      | t (uçuş) | yaw | irtifa |
      |---:|---|---:|
      | 39.5 s | sabit 350° | 21.5 m |
      | 40–55 s | **kesintisiz dönüş, net 14.57 tur, ~350 °/s** | 21.8 → 2.0 m |
      | 55–70 s | dönüş sönüyor | 2.0 m sabit |

      `DesYaw` de aynı rampayı ~45° gerisinden izliyor — yani araç kusursuz
      uyguluyor, **komut kesilmemiş**. `ATT.DesYaw`'ın sürekli rampa çizmesi
      bir yaw **HIZ** komutunun iptal edilmemiş olduğunu gösteriyor: güdüm
      "VURULDU" deyip CSV'yi kapatıyor (o andan sonra ne gps_guidance ne
      visual_lead kaydı var), ama son yaw-hızı komutu araçta yaşamaya devam
      ediyor. Araç 15 s boyunca ~350 °/s dönerek 21.8 m'den 2.0 m'ye düşüyor.
      Mod hep GUIDED (4), mod değişimi yok.
      **Bu A1'in yan etkisi değil:** dönme 3.0'lı koşularda da vardı
      (log 100/103: 0.10-0.15 tur/s).
      → B5 çözümüne "faz biterken yaw-hızı komutunu SIFIRLA" maddesi eklenmeli;
      A5 (gerçek temas) bu davranışı daha da uzun süre görünür kılacak.
      *Sonuç:*

- [ ] **B6 — Terminal algı kalitesi** · asıl darboğaz
      *Neden:* ölçüm net — `ok` oranı %50 üstündeyken en yakın menzil 0.99-1.12 m
      (vuruş), %30 altındayken 2.4-12.7 m (ıska). 18 fazın yalnız 2'si 1.5 m
      altına indi. Gerçek temas için ~0.3 m daha kapatmak gerekiyor ve bunu
      sağlayacak şey daha iyi/kararlı pose.
      *Nereye bakılacak:* `kpt_dusuk` oranı terminalde %27-46 — keypoint güveni
      düşüyor. Yeni eklenen cevap anahtarı sütunları (`pose_elev_sapma_deg`,
      `pose_yaw_sapma_deg`, `gercek_kadraj_ici`) algı hatasını doğrudan ölçüyor;
      bunlar A5 sonrası loglarda incelenip hatanın açı mı, ölçek mi, yoksa
      kadraj mı olduğu ayrıştırılmalı.
      *Ölçüt:* terminal `ok` oranı %50 üstüne çıkmalı; en yakın menzil
      dağılımının medyanı 1 m altına inmeli.
      *Sonuç:*

- [ ] **B7 — AÇIK SORU: istasyon açısını kamera tilt'inden ayırmak doğru muydu?**
      · öncelik **ORTA**, B6'dan önce karara bağlanmalı
      `control/guidance/gps_guidance.py` → `ISTASYON_ELEV_DEG`

      *Şüphe:* 25° tesadüf değildi — kamera tilt'i o. İstasyon 25°'de
      kurulunca hedef kadrajın TAM MERKEZİNDE oluyordu. 15°'ye indirince
      hedef merkezin ~10° altına düştü. Pose modelinin merkez dışı
      performansı, lens distorsiyonu, ve hedefin gökyüzü yerine zemin
      önünde görünmeye başlaması (daha yatık bakış) bedel olabilir.
      Değişiklik ölçüme dayanıyordu ama **bedeli izole ölçülmedi**.

      *Elde olan (2026-08-02, 10 faz @25° vs 17 faz @15°):*

      | | 25° | 15° |
      |---|---:|---:|
      | `ok` oranı (tüm faz) | %24.0 | **%32.0** |
      | `ok` oranı (menzil < 8 m) | %8.7 | **%18.2** |
      | hedef kadraj içi | %59.8 | **%67.0** |
      | pose kalite medyanı | 1.00 | 1.00 |
      | `pose_elev_sapma` medyanı | 3.10° | 3.51° |
      | en yakın menzil medyanı | 5.25 m | **1.73 m** |
      | vuruş | 1/10 | 3/17 |

      ⚠ **Bu tablo şüpheyi ÇÜRÜTMÜYOR, çünkü karışık.** Algının iyileşmesi
      büyük ölçüde geometrinin SONUCU: drone hedefin seviyesine yakın
      kalınca hedef kadrajdan geç çıkıyor, tespit doğal olarak uzuyor.
      Yani ölçülen şey "merkez dışı kadrajlama zararsız" değil, "net etki
      olumlu". Merkez dışı olmanın kendi bedeli **hâlâ bilinmiyor**.

      *Bilinmeyenler:*
      1. 15° taranarak seçilmedi — dikey ivme bütçesi hesabından çıktı.
         18° ve 20° hiç denenmedi. Bütçeye sığan **en büyük** açı hangisi?
      2. **Asıl alternatif hiç denenmedi:** istasyonu 25°'de bırakıp
         `WP_ACC_Z`'yi (1.0 → 2.5-3.0) yükseltmek. İşe yararsa hem merkez
         kadrajlama hem dikey kapanma birlikte elde edilir ve bu ayrım
         gereksiz hale gelir.

      *Nasıl karara bağlanır — sırayla, tek değişken:*
      - **Adım 1:** `ISTASYON_ELEV_DEG=25` geri + `avci_copter.parm`'a
        `WP_ACC_Z 2.5`. Tek uçuş. ⚠ `WP_ACC_Z` global bir kopter
        parametresi — kalkışı ve istasyon tutmayı da etkiler; irtifa
        aşımı/salınım için `PSCD.DVD` vs `VD` bakılacak.
      - **Adım 2:** Adım 1 tutmazsa açıyı tara: 20°, 18°. Bütçeye sığan
        en büyük açı seçilir (test G11 sınırı zaten kontrol ediyor).

      *Ölçüt — 15°'nin şu anki haline göre bozulmamalı:* en yakın menzil
      medyanı ≤ 1.73 m · terminal `ok` oranı ≥ %18 · kadraj içi ≥ %67 ·
      dikey artık medyanı |·| ≤ 0.9 m. Bunları tutturan **en yüksek**
      istasyon açısı kazanır (merkez kadrajlamaya en yakın olan).
      *Ölçüm aracı:* `python3 tools/gecis_analiz.py`
      *Sonuç:*

- [ ] **B4 — `coalt` kapsamını daralt** · düşük öncelik
      `guidance_core.TERMINAL_COALT_DEG = 10°` yukarı yanlılık **1064 karede**
      aktifti; tırmanışı büyüten etkenlerden biri. `coalt_latch` menzil eşiğine
      bir kez girince kilitleniyor. B1+B2 sonrası hâlâ sorun varsa bakılır.
      *Sonuç:*

---

## Ölçüm komutları

```bash
# Uçuş SONRASI — geçişlerin gerçek geometrisi (iki aracın kara kutusundan).
# CSV'deki menzil EKF ofsetinden etkileniyor; bu araç dürüst kaynağa bakar.
python3 tools/gecis_analiz.py            # en son uçuş
python3 tools/gecis_analiz.py 126 127    # belirli BIN'ler
python3 tools/gecis_analiz.py --liste    # son 10 uçuş

# Uçuş sırasında — gerçek temas kaynağı sağlam mı (A5)
curl -s localhost:8000/api/debug/carpisma | python3 -m json.tool

# Uçuş sırasında — hedef telemetrisi sağlığı (A6 uygulanınca)
curl -s localhost:8000/api/debug/hedef_telem | python3 -m json.tool

# Uçuş sırasında — faz ve kapılar
curl -s localhost:8000/api/telemetry/pnp | python3 -m json.tool

# Uçuş sonrası — parametreler gerçekten uygulandı mı
python3 tools/parm_denetle.py
```

Kara kutu ölçümleri (`~/ardupilot/logs/*.BIN`): yaw için `ATT.Yaw` vs
`ATT.DesYaw`; dikey için `PSCD.DVD` (istenen) vs `PSCD.VD` (gerçekleşen);
motor doygunluğu için `RCOU`.
