# Güdüm Arıza Zinciri — 2026-08-01

**Soru:** "Avcı hedefi neden takip edemiyor, neden hiç göremiyoruz?"

Cevap tek bir hata değil, **birbirine bağlı beş arıza** çıktı. Hepsi ölçümle
tespit edildi; bu belge her birinin kanıtını ve düzeltmesini kaydeder.

Veri: `logs/gps_guidance_20260801_*.csv` (yedi uçuş, ~7500 kare) ve
`logs/plane_harmonic.log`. Referans zemin Gazebo tam pozu (`control/gz_truth.py`),
MAVLink telemetrisi değil.

---

## Özet tablo

| # | Arıza | Kanıt | Düzeltme |
|---|---|---|---|
| 1 | `start_harmonic.sh` SITL'i hiç başlatmıyor | `cd "$AP"` sessizce başarısız; MAVLink portu hiç açılmıyor | Aday yol taraması + hata verip durma |
| 2 | Gazebo Intel iGPU'da render ediyor | `nvidia-smi`'de `gz sim` yok, log'da `failed to create dri2 screen` | NVIDIA PRIME bloğu geri kondu |
| 3 | `V_MAX = 28` doygunluk patolojisi | Komut %84 tavanda, gerçekleşen/komut **0.24** | `V_MAX = 18` |
| 4 | Senaryolar FBWA'da irtifa tutmuyor | Hedef 65 m'den yere süzülüyor, pitch −1.6…−2.8° | FBWA → **FBWB** |
| 5 | `avci_plane.parm`'daki hız ayarları ÖLÜ | `TRIM_ARSPD_CM` vb. bu firmware'de yok, sessizce yok sayılıyor | `AIRSPEED_*` (m/s) |
| 6 | Hedefin hız talebi airframe'in üstünde | 14 m/s 29° yatışta da düştü; 11.5 m/s 32°'de tuttu | `AIRSPEED_MIN 11` / `CRUISE 12` |
| 7 | **Chase hiç başlamıyordu** | `takeoff_to_z` 30 s'de 57 m'ye çıkamıyor → `[CHASE] Kalkış başarısız!` | `timeout=120`, `tolerans=1.5` |

Sonuç zinciri: hedef ya düşüyor ya avcıdan hızlı → menzil 87–215 m'de kalıyor →
o menzilde Talon görüntüde **0.5–2 piksel** → YOLO tespit edemiyor → 10 ardışık
pose karesi hiç birikmiyor → **görsel faza devir kapısı hiç açılmıyor.**
"Hedefi göremiyoruz" ayrı bir arıza değil, bu zincirin son halkası.

---

## 1. `scripts/start_harmonic.sh` — SITL sessizce hiç başlamıyordu

Merge `PROJ`'u script konumundan türetecek şekilde düzeltmiş ama `AP` ve `APGZ`
sabit kalmış:

```bash
AP="$HOME/ardupilot"          # bu makinede depo $HOME/Masaüstü/ardupilot
( cd "$AP" && nohup python3 Tools/autotest/sim_vehicle.py ... )
```

`cd` başarısız olunca `&&` zinciri kopuyor, sim_vehicle **hiç çalışmıyor**, ama
script "başlatılıyor" yazıp normal bitiyor. Arıza yalnız MAVLink portunun hiç
açılmamasıyla belli oluyor.

**Düzeltme:** aday yollar taranıyor (`$HOME/ardupilot`, `$HOME/Masaüstü/...`,
`$HOME/Desktop/...`), `AVCI_AP_DIR` ile elle verilebiliyor, bulunamazsa
**hata verip duruyor**. Ayrıca sondaki kör `sleep 25`, süreç + `AP: Frame:`
banner doğrulamasıyla değiştirildi.

> Not: 14541/14542 portlarını beklemek YANLIŞ ölçüttür. MAVProxy `--out udp:...`
> ile **giden** soket açar; o portları dinleyen taraf `gcs_server`'dır.

## 2. NVIDIA PRIME offload bloğu düşmüştü

Merge, PRIME değişkenlerini set eden bloğu silmiş; yorumlarda "NVIDIA render"
yazıyor ama hiçbir şey set edilmiyordu. Kamera sensörünü render eden
`gz sim server`'dır ve YOLO'yu o besler — iGPU'da kare hızı düşer.

**Kanıt:** `nvidia-smi` çıktısında `gz sim` yok, `gz_harmonic.log`'da
`libEGL warning: egl: failed to create dri2 screen`.
**Düzeltmeden sonra:** `gz sim server` 306–333 MiB, `gz sim gui` 396–646 MiB,
EGL hatası sıfır.

## 3. `V_MAX = 28` — ulaşılamaz değil, ZARARLI

İki uçuş, 380 saniye, 7549 kare:

| Ölçüm | Değer |
|---|---|
| Komut hızı | medyan 28.0 m/s, karelerin **%84'ü tavanda** |
| Gerçekleşen | medyan **8.2 m/s**, p99 19.8 |
| Menzil >50 m iken (tam gaz olmalı) | gerçekleşen **6.8 m/s**, oran **0.24** |
| Menzil <50 m iken (yavaşlamalı) | gerçekleşen **9.9 m/s** |

**Uzaktayken yakındakinden yavaş.** Bu ters ilişki patolojinin imzası:
28 m/s'lik hız hatası `PSC_VELXY_P = 2.0` ile ~56 m/s² ivme talebine dönüşüyor;
irtifa korunurken ulaşılabilir ivme ~17 m/s². Attitude kontrolcüsü tavana
yapışıyor, itki vektörü irtifayı korumak için yatışı geri çekiyor → yatış
açısında limit çevrimi. Hata küçükken kontrolcü doğrusal bölgede kalıyor ve
**daha iyi** çalışıyor.

Yasa sağlam: komut yönü istasyon noktasını **8.3°** hatayla gösteriyor, gerçek
hareket komutu izliyor (medyan 13.8° sapma), gövde-çerçeve bug'ı yok
(yaw-düzeltmeli sapma 89.6°), irtifa hatası **0.14 m**.

**Düzeltme ve doğrulama:**

| V_MAX | Gerçekleşen medyan | Gerçekleşen/komut |
|---|---|---|
| 28 | 8.2 m/s | 0.24 |
| 15 | 11.7 m/s | 0.78 |
| **18** | **18.0 m/s** | **1.00** |

## 4. Senaryolar FBWA'da irtifa tutmuyordu

FBWA'da nötr elevator **"0° pitch AÇISI"** komutudur, "irtifayı koru" değil.
Uçak seviye uçuş için pozitif pitch ister; 0°'de sürekli batar.

**Kanıt:** irtifa 65 → 55 → 45 → 35 → 24 → 14 → yer; ölçülen pitch −1.6…−2.8°.
Ani düşüş değil, kararlı süzülüş — stall değil, irtifa tutamama.
Aynı dosyada tutarsızlık: `scenario_circle` telafi için `pitch=150` veriyordu,
`scenario_square`'in düz kenarları **0** veriyordu.

**Düzeltme:** senaryolar **FBWB**'ye alındı (TECS irtifayı kilitler). FBWB'de
elevator artık *tırmanma hızı* komutu olduğu için tüm pitch biasları kaldırıldı
ve dikey otoriteyi korumak için `FBWB_CLIMB_RATE 8` eklendi.

## 5. `avci_plane.parm`'daki hız ayarları ÖLÜ isimdi

ArduPilot bilinmeyen parametre adını **sessizce yok sayar** — hata vermez.

| Dosyadaki | Durum | Doğrusu | Birim |
|---|---|---|---|
| `TRIM_ARSPD_CM 1500` | ölü | `AIRSPEED_CRUISE` | cm/s → **m/s** |
| `ARSPD_FBW_MIN 1200` | ölü | `AIRSPEED_MIN` | cm/s → **m/s** |
| `ARSPD_FBW_MAX 2200` | ölü | `AIRSPEED_MAX` | cm/s → **m/s** |

Dosyanın başındaki *"hedefi avcının yakalayabileceği hızda tut"* yorumu bir
niyet beyanıydı; uygulanmıyordu. Uçak firmware varsayılanlarıyla uçuyordu.

FBWB'de gaz slider'ı doğrudan hız komutudur
(`ArduPlane/navigation.cpp:177`):

```
slider ≤ %50 → hız = AIRSPEED_MIN … AIRSPEED_CRUISE arası doğrusal
slider ≥ %50 → hız = AIRSPEED_CRUISE … AIRSPEED_MAX arası doğrusal
```

Yani hedefin inebileceği **en düşük hız = AIRSPEED_MIN**.

**Ders:** parametre dosyasına ad yazarken firmware'de var olduğu doğrulanmalı.
Bu depodaki her ad tek tek `ArduPlane/Parameters.cpp` ve
`ArduPlane/mode_takeoff.cpp` karşısında sınandı (`TKOFF_ALT` geçerliymiş —
`TKOFF_` grup öneki ile).

---

## 6. Hedefin uçabilir hız bandı — asıl kısıt HIZ TALEBİ

Uzun süre yatış açısını suçladım. Ölçüm tablosu bunu çürüttü:

| Talep edilen hız | Yatış | Sonuç |
|---|---|---|
| 9.6 m/s | 42° | düştü (stall bandı) |
| 11.2 m/s | 42° | düştü |
| 14 m/s | 42° | düştü |
| **14 m/s** | **29°** | **düştü** ← yatış düşürüldü, yine düştü |
| **11.5 m/s** | **32°** (`circle`) | **58 m'de 24 s ±0.1 m ✓** |
| **12 m/s** | **29°** (`square`) | **58 m'de 30 s −0.57 m ✓** |

14 m/s'nin 29° yatışta da düşmesi belirleyici oldu: sorun yatış değil.
`TRIM_THROTTLE 45` — bu airframe makul gazda ~11-12 m/s seyrediyor. Daha
yükseğini talep etmek TECS'i **irtifayı hıza takas** etmeye zorluyor; uçak
süzülerek iniyor. Alçalma FBWB'ye geçer geçmez başlıyordu, dönüşlerden bağımsız.

**Düzeltme:** `AIRSPEED_MIN 11`, `AIRSPEED_CRUISE 12`. Yan düzeltmeler:
`turn_by` yatışı 650 → 450 (~29°, yük faktörü 1.35 → 1.14) ve `ARSPD_USE 1`
(sensör vardı, kalibre oluyordu, ama TECS kullanmıyordu; FBWB'nin
"slider = hedef hız" eşlemesi de bu sensörü gerektiriyor).

Hedefi yavaşlatmak artık bir kayıp değil: `V_MAX = 18` ile avcı 18.0 m/s'yi
sabit tutuyor, yani 12 m/s'lik hedefe karşı **6 m/s kapanma marjı** var —
bugüne kadarki en geniş marj.

---

## 7. Chase hiç başlamıyordu — `takeoff_to_z` timeout'u

En sinsi arıza. `_chase_thread` sırası:

```python
target_z = plane_z if plane_z < -1.0 else -5.0   # HEDEFİN irtifasına kalk
success = df_takeoff(target_z=target_z)
if not success:
    print("[CHASE] Kalkış başarısız!"); return       # ← chase HİÇ başlamıyor
```

`takeoff_to_z` varsayılanları **30 s timeout** ve **0.3 m tolerans**'tı. Bu
değerler chase 5 m'ye kalkarken yazılmıştı. Artık hedefin irtifasına
(ölçülen uçuşlarda **57 m**) kalkıyor ve bu makinede sim gerçek zamanın
~%60'ında koşuyor — 30 s duvar saati ≈ 18 s sim. Kopter 57 m'ye o sürede
çıkamıyor, fonksiyon `False` dönüyor, chase thread'i sessizce çıkıyor.

Dışarıdan görünen: kopter havalanıyor (kalkış komutu gitmiş), 55-58 m'ye
tırmanıyor, sonra öylece duruyor. `chase_status` `active: false` diyor ama
arayüzde bunun karşılığı yok. **"Avcı hedefi takip etmiyor" şikâyetinin bir
kısmı buydu** — takip eden bir şey hiç başlamamıştı.

Teşhis ancak `gcs_server` çıktısı dosyaya yönlendirilince mümkün oldu
(`python3 -u ... 2>&1 | tee logs/gcs_server.log`); terminalde kalan
`[CHASE] Kalkış başarısız!` satırı hiç görülmüyordu.

**Düzeltme:** `takeoff_to_z(timeout=120.0, tolerans=1.5)`. Tolerans güdümün
istasyon hassasiyetinin (4.65 m dikey ofset) çok altında, bir şey kaybettirmez.

---

## Menzil ↔ görülebilirlik

Kamera 640×480, HFOV 125° → `FX = 166.6`. Talon gövdesi 0.81 m:

| Menzil | Gövde | Not |
|---|---|---|
| 10 m | 13.5 px | pose sağlam |
| 20 m | 6.7 px | görsel devir kapısı (`GATE_MENZIL`) |
| 40 m | 3.4 px | tespit sınırı |
| 80 m | 1.7 px | tespit yok |
| 215 m | 0.6 px | ekranda hedef fiziksel olarak yok |

Bugünkü uçuşların menzil medyanı 87–215 m'ydi. Bu yüzden `kilit_sayac` hep 0
kaldı ve `visual_lead_*.csv` hiç üretilmedi.

---

## 8. ÇÖZÜLEMEYEN: hedefin hızı talep edilenden bağımsız

Bu tek başına bir arıza değil, **kapatılamamış bir kısıt** — kaydı burada
dursun ki aynı yollar tekrar denenmesin.

**Belirti:** hedefe komutlanan hız ne olursa olsun uçak ~15.3 m/s seyrediyor.

| `AIRSPEED_CRUISE` | Gerçekleşen |
|---|---|
| 14 | 16.3 m/s |
| 12 | 15.4 m/s |
| 10 | 14.2 m/s |

Talebin ancak yarısı geçiyor.

**Kesin ölçüm** (MAVLink `VFR_HUD`, 1195 örnek, 50 s seyir, talep 10 m/s):

```
BİLDİRİLEN hava hızı : 15.26 m/s
YER hızı (GPS)       : 15.26 m/s
```

İkisi **birebir aynı** ve rüzgâr yok (`SIM_WIND_SPD 0`) → **hız sensörü sağlam,
ölçek doğru.** Yani sorun sensörde değil: TECS hız talebini uygulamıyor.
(`ARSPD_USE 1`, `ARSPD_TYPE 2`, `AIRSPEED_STALL 8`, `STALL_PREVENTION 1`.)

**Denenen ve başarısız olan çare:** `THR_MAX 40` ile gazı fiziksel olarak kısmak.
Uçak **kalkamadı**, yerde kaldı. Sebep: `TKOFF_THR_MAX 0` "sınırsız" DEĞİL,
"THR_MAX'ı kullan" demek — kalkış da 40'a kısılıyor. Denenecekse
`TKOFF_THR_MAX 100` ile birlikte ve ayrı bir ayar turu olarak denenmeli.

**Alınan karar:** hız bandı uçağın GERÇEKTE uçtuğu değere çekildi
(`AIRSPEED_MIN 12`, `AIRSPEED_CRUISE 15`). Dosya artık uçağın yaptığı işi tarif
ediyor; "komutlanan" ile "uçulan" arasında yanıltıcı fark yok.

**Sonucu:** avcı (12.8-18 m/s, kare dönüşleri hızı yiyor) hedefe (15.3 m/s)
karşı kalıcı bir kapanma marjı bulamıyor. Menzil 24-359 m arasında dönüşlerle
senkron salınıyor; en iyi yaklaşma **28.9 m**. Görsel devir kapısı (20 m) hiç
açılmadı.

---

## Sabitlenen halin sonucu — günün en iyi uçuşu

Hız bandı gerçeğe çekildikten sonra (`AIRSPEED_MIN 12`, `AIRSPEED_CRUISE 15`,
`V_MAX 20`) yapılan doğrulama uçuşu, gün boyunca yapılan **ayarların en iyisini**
verdi. Yani hedefi yavaşlatmaya çalışmak (9-12 m/s talebi) sonucu KÖTÜLEŞTİRİYORDU:

| Uçuş | En yakın menzil | Pose kilidi |
|---|---|---|
| `AIRSPEED_CRUISE 12`, `V_MAX 18` | 39.0 m | 0 |
| `AIRSPEED_CRUISE 10`, `V_MAX 20` | 28.9 m | 1 |
| **`AIRSPEED_CRUISE 15`, `V_MAX 20`** | **5.54 m** | **4** |

Tam ölçüm (4129 kare):

```
menzil            min 5.54 m,  20 m altında 24 kare
komut yönü hatası 6.4° medyan, %100'ü 30° içinde
dikey istasyon    0.05 m medyan hata
kadraj nişanı     1.76° medyan
avcı hızı         14.4 m/s medyan (tepe 19.9)
hedef hızı        15.8 m/s medyan
```

**Devir kapısına ne kadar kaldı:** kapı 10 ARDIŞIK güvenli pose karesi VE
menzil ≤ 20 m istiyor. Menzil koşulu 24 karede sağlandı, pose kilidi 4'e çıktı.
Yani eksik olan menzil değil, **pose tespitinin sürekliliği**: 5-20 m'de Talon
görüntüde 13-27 piksel, YOLO görebilmeli. 4'te kalması hedefin kadraja girip
hızla çıktığını düşündürüyor (avcı yanından geçiyor). Bir sonraki adım bunu
`visual_lead` CSV'sinden doğrulamak.

---

## Devir kapısı neden açılmıyor — ÖLÇÜLDÜ

Bu soru gün boyu cevaplanamadı çünkü **GPS fazı sırasındaki pose tespitleri
hiçbir yere yazılmıyordu.** `supervisor.izci`'ye sayaç eklendi
(`pose_toplam` / `pose_var` / `pose_guvenli` / `kilit_en_uzun`, 10 saniyede bir
`[SUPERVISOR] pose: …` satırı). İlk uçuşta cevap çıktı:

```
pose: 0/259   kare tespit (%0), 0 güvenli,  en uzun ardışık 0/10,  d_h=201m
pose: 2/510   kare tespit (%0), 2 güvenli,  en uzun ardışık 2/10,  d_h=98m
pose: 19/743  kare tespit (%3), 19 güvenli, en uzun ardışık 18/10, d_h=174m
pose: 19/1509 kare tespit (%1), 19 güvenli, en uzun ardışık 18/10, d_h=132m
```

**Okunacak üç şey:**

1. **Pose tarafı sağlam.** 19 tespitin **19'u da güvenli** (`conf ≥ 0.5`) ve
   **18'i ARDIŞIK** — kapının istediği 10'un neredeyse iki katı. Yani
   "YOLO göremiyor" hipotezi yanlış; gördüğünde sağlam görüyor.
2. **Tespit tek bir pencerede oldu.** Sayaç 19'da donuyor: hedef bir kez
   kadraja girdi, 18 kare kaldı, çıktı ve bir daha girmedi.
3. **Kapı bu yüzden açılmıyor:** iki koşul (10 ardışık pose VE menzil ≤ 20 m)
   **aynı anda** sağlanmıyor. Kilit yakalandığında menzil hâlâ 20 m'nin
   üstündeydi.

Yani arıza "tespit edemiyoruz" değil, **"tespit ile yakınlık çakışmıyor"**.
Avcı hedefin yanından hızla geçiyor; kadrajda kaldığı ~0.6 saniyede kilit
doluyor ama o an istenen menzilde değil.

**Bundan çıkan iki somut yol:**

- **Kapıyı gevşet.** `AVCI_HYBRID_GATE_MENZIL` (varsayılan 20) ölçülen tespit
  menziline çekilsin. 18 ardışık güvenli kare gerçekten o menzilde toplandıysa
  görsel faz oradan başlayabilir. ÖNCE doğrulanmalı: güven yüksek olması pose'un
  DOĞRU olduğunu göstermez — bunu `visual_lead` CSV'sinin doğruluk kolonları
  söyler (`kpt_hata_px_*`, `eksen_aci_hata_deg`).
- **Geçişi yavaşlat.** Avcı hedefin yanından geçmek yerine kuyrukta kalabilirse
  tespit penceresi saniyelere çıkar. Bu da GPS fazının hız marjı sorununa geri
  bağlanır.

---

## IBVS'e nereden devam edilmeli

Görsel fazı GPS fazından bağımsız ölçmek için `POST /api/command/iris/start_visual`
var ve kalkış irtifası artık hedefe bağlı (bu oturumda düzeltildi). **Ama tek
başına yetmiyor:** görsel faz yatay mesafeyi kapatamaz. Denendi — drone hedefin
2.5 km gerisinde kaldı, üretilen `visual_lead` CSV'sinin 736 karesinin tamamı
`tespit_yok`. (CSV yapısı doğrulandı: **70 kolon, 24'ü doğruluk kolonu.**)

Yani IBVS'i çalıştırmanın ön koşulu avcıyı hedefin **yakınına koymak**. Üç yol:

1. **Konumlandırma adımı ekle** — `start_visual` önce GPS istasyon noktasına
   (hedefin `RANGE_SET·cos25°` gerisi, `·sin25°` altı) pozisyon setpoint'iyle
   gitsin, oraya varınca IBVS'i başlatsın. GPS fazının hız marjı sorununu
   tamamen atlar; IBVS'i izole ölçmenin en temiz yolu.
2. **Hedefi gerçekten yavaşlat** — `TKOFF_THR_MAX 100` + `THR_MAX ~35` çifti.
3. **Senaryoya hız kontrolü koy** — `run_plane_scenario` gaz yerine doğrudan
   hedef hızı sürsün (kapalı çevrim, ölçülen yer hızına göre).

1 numara IBVS çalışması için en hızlı yol; 2 ve 3 GPS fazını da düzeltir.

---

## Açık kalan iş

1. **Avcının serbest düşüşü.** Hedef dalınca güdüm `VZ_MAX = 6 m/s` alçalma
   komutluyor ama drone **18.2 m/s** ile düşüyor (87 m → 4.9 m, 4.5 s).
   `LOOKUP_MIN_ALT` istasyon noktasını koruyor, düşüşün kendisini değil.
   Yatık gövdede itki kısılınca dikey bileşen ağırlığın altına düşüyor.
   `aggressive` senaryosunda tekrarlanabilir.
2. **Adım 6b** — GPS fazının kuyruğa yakınsaması, artık ölçülebilir durumda.
3. **Adım 4b verisi** — görsel faz devreye girince `visual_lead_*.csv` üretilecek;
   doğruluk kolonları (`*_gercek`) yeniden bağlandı ve sentetik olarak doğrulandı
   (T30–T33, menzil sapması < %2).
