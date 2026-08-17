# Ö-T · Terminal mandalını süreyle bırak — ölçüt ilanı

**Tarih:** 2026-08-17 (kampanya koşulmadan ÖNCE yazıldı, §4)
**Kol anahtarı:** `AVCI_IBVS_TERM_BIRAK_S` · panel: "Ö-T · Terminal mandalını SÜREYLE bırak"
**Kontrol:** 0.0 (kapalı) · **Deney:** 4.0 s

---

## Neden — kullanıcının 2026-08-17 uçuşu

261 saniyelik koşu, 5208 karenin **tamamı** TERMINAL fazında:

| ölçüm | değer |
|---|---|
| menzil | medyan 8.7 m, **min 6.1 m** (261 s'de altına hiç inmedi) |
| kapanma hızı | **0.00 m/s** |
| `eps_elev` (dikey hata) | **+0.7°** — dikeyde sorun YOK |
| irtifa farkı (kara kutu) | **−0.09 m** — neredeyse aynı seviye |
| `v_los` komutu | 16.0 sabit |
| Ö-T sayacı | 0.00 → **kapalıydı** |

Kara kutu geçişleri: menzil 6.16 m, **yatay 6.15 m, dikey −0.39 m**.
Mesafenin tamamı yatayda ve kapanmıyor.

### Kök neden — hız marjı

| | m/s |
|---|---|
| komut edilen (`v_los`) | 16.00 |
| avcının **gerçekleşen** hızı (kara kutu) | 15.35 |
| hedefin hızı | 15.15 |
| **kapanma kapasitesi** | **+0.20** |

6 metreyi 0.20 m/s ile kapatmak 30 saniye sürer; aradaki en ufak salınım
o marjı yiyor → pratikte hiç kapanmıyor.

### Aynı oturumun kendi kontrol grubu

| koşu tipi | komut hızı | en yakın menzil |
|---|---|---|
| 9 kısa koşu (mandal kurulmamış) | **21-23 m/s** | **1.3-1.5 m** |
| son koşu (mandal kilitli) | **16.0 m/s** | **6.1 m** |

Mandal kurulmayan koşularda 1.3 m'ye giriyor; kilitlenende 6.1'de takılıyor.

### Mekanizma

Kutu 25 px'i aşınca (~6.4 m) mandal kilitleniyor, hız `V_TERMINAL`=16'ya
sabitleniyor. Mandal ancak menzil `TERM_BIRAK_M`=20 m'yi aşarsa bırakılıyor.
6-9 m bandında sıkışınca 20 m'ye hiç çıkamıyor → **sonsuz kilit**.

Ö-T: mandal kilitliyken **en iyi menzil** `TERM_BIRAK_S` saniye boyunca
`TERM_BIRAK_EPS`=0.5 m'den fazla iyileşmezse mandal bırakılır ve seyir yasası
(24 m/s tavan) geri gelir.

⚠ İlk tasarım "anlık kapanma < 0.5 m/s" idi ve ÇÜRÜDÜ: kapanma çok gürültülü
(p10 −16.8 / p90 +16.4), karelerin %44.9'unda eşiği aşıp sayacı sıfırlıyordu.
En iyi menzil MONOTON olduğu için gürültüden etkilenmez.

Çevrimdışı oynatma (kullanıcının 3 takılan koşusu): yeni mantık **11, 14 ve
38 kez** tetiklerdi; eski mantık sıfır kez.

---

## ÖLÇÜTLER

### Birincil — NET TANIM (D-V dersi)
**Koşu başına en iyi menzilin medyanı** (kara kutudan, `tools/gecis_analiz.py`).

⚠ "Tüm geçişlerin medyanı" KULLANILMAZ: D-V kampanyasında bu ölçütün kol
başına geçiş sayısından etkilendiği görüldü (86 vs 47 geçiş) ve çok geçiş
üreten kolu cezalandırdı. Koşu-başı okuma bu karıştırıcıdan bağımsız.

Karar: deney kolunun medyanı kontrolünkinden **küçük** olmalı.

### İkincil
1. **Temas sayısı** — kara kutuda ≤0.5 m geçiş
2. **Takılma süresi** — 6-12 m bandında geçen terminal karesi oranı
3. **En iyi tekil geçiş**

### Mekanizma kapısı (§5.1)
Deney kolunda:
- `term_kapanmasiz` sütunu **sıfırdan farklı** olmalı, VE
- gcs günlüğünde **"terminal mandalı SÜREYLE bırakıldı"** satırı düşmeli

İkisi de yoksa özellik çalışmamıştır → o koşu **veri değildir**.

⚠ Bu kapı `vz_cmd` ya da `eps_elev` gibi SONUÇ ölçütleri üzerinden
okunmaz. D-V'de mekanizma kapısını sonuç ölçütüyle karıştırma, D-N'de ise
kendi kendine referanslı ölçüt kullanma hatası yapıldı; ikisi de
tekrarlanmıyor.

### Geçerlilik eşi (§5.2)
| ölçüt | kötü sebeple iyileşir mi | zorunlu eş |
|---|---|---|
| en yakın menzil | **evet** — savrulup şans eseri yaklaşma | salınım + görsel temas |
| takılma süresi azalması | **evet** — hedefi büsbütün kaybedip uzaklaşmak da takılmayı bitirir | geçiş sayısı + görsel temas |
| temas | evet — dengesiz araç şans eseri çarpar | salınım |

**Görsel temas %60 altına inen kol GÜVENİLMEZ** sayılır.

### Salınım (kullanıcı kuralı)
`cx` işaret değişimi/s · `|roll|` p90 · roll işaret değişimi/s

### Geçerlilik (§4)
Hedef 20-250 m irtifa, 6-25 m/s bandında. Dışına çıkan koşu SAYILMAZ.

---

## ETKİ ALANI TABLOSU (§5.10)

| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Terminal takılması | doğrudan hedef | `duz` — ana ölçüm |
| **Temas anı hızı** | mandal bırakılınca 24 m/s'ye çıkar → temas penceresinden HIZLI geçip ıskalayabilir. Mandalın var oluş sebebi buydu | `duz` — temas sayısı + en yakın menzil |
| Kör hücum penceresi | mandal düşünce kör hücum de kapanır | `duz` — kutusuz kare oranı |
| Manevrada | dairede mandal zaten kurulmuyor (6.4 m'ye inilemiyor) | `circle` — REGRESYON (zaman kalırsa) |
| Seyir fazı | **etkilenmez** — mandal yokken kod bu daldan geçmiyor | birim testi T1 |

**Cevaplanacak soru:** "takılmayı çözdü ama temas anını bozdu mu?"

---

## KOŞU PLANI

- `duz` senaryo, hibrit güdüm, **dönüşümlü** (K,D,K,D,…), hedef başına 2 koşu
- Diğer dört deney (D-V, D-V2, D-N, D-S) **iki kolda da KAPALI** — tek değişken
- İrtifa tutucu **iki kolda da AÇIK** (hedef sabit irtifada uçsun)
- Ölçüm penceresi: angajman (menzil<40 m) sonrası **150 s**
  (takılmanın oluşup Ö-T'nin tetiklenmesi için D-V/D-N'deki 120 s'den uzun)
- Hedef vurulup düşerse sim kendini kurar, kampanya devam eder

## KARAR KURALI (önceden ilan)

- Birincil + en az bir ikincil deney kolu lehine → **GİRER**
- Birincil kötüleşir **veya** temas sayısı düşerse → **GİRMEZ**
- Bölünürse **kullanıcıya** — ölçüt değiştirilmez (§5.6)
- n<4/kol → **ara veri**, hüküm kurulmaz (§5.4)

---

## ⚑ ARA BULGU — mekanizma AMAÇLADIĞI İŞİ yapıyor (kampanya sürerken)

"Mekanizma ateşledi mi" sorusunun ötesi: bırakma olayından SONRA menzil
gerçekten kapanıyor mu? Her bırakma anı (`term_kapanmasiz` ≥3.9 → 0) ve
sonraki 5 saniye ölçüldü:

| ölçüm | değer |
|---|---|
| bırakma olayı | 11 |
| bırakma anındaki menzil | medyan 8.8 m |
| sonraki 5 s'deki en iyi menzil | medyan **7.1 m** |
| **kazanım** | **+1.18 m** |
| 0.5 m'den fazla kapatan olay | **8/11 = %73** |

⇒ Mandal bırakılınca araç seyir yasasına dönüp gerçekten yaklaşıyor.

⚠ Bu **tek başına "Ö-T iyi" demek DEĞİLDİR.** Cevaplanmamış üç soru:
1. Yaklaştıktan sonra mandal yeniden kurulup tekrar takılıyor mu?
2. Temas anını bozuyor mu (24 m/s ile hızlı geçip ıskalama riski)?
3. Kontrol koluna göre net kazanç ne?

Bunlar kampanya bitince birincil ölçütle cevaplanacak.

## ⚠ ARA BULGU 2 — Ö-T DÖNGÜ yaratıyor, takılmayı KIRIYOR ama ÇÖZMÜYOR

Her bırakma olayı ve o ana kadarki en iyi menzil izlendi:

| koşu | olay | bırakma menzili | o ana kadarki EN İYİ |
|---|---|---|---|
| 155541 | #1 | 14.5 m | 3.4 m |
| | #2 | 6.4 m | 3.4 m |
| | #3 | 8.3 m | 3.4 m |
| | #4 | 9.7 m | **3.4 m** ← hiç ilerlemedi |
| 162301 | #1 | 7.9 m | 3.9 m |
| | #2 | 8.5 m | 3.9 m |
| | #3 | 10.3 m | **2.7 m** ← biraz ilerledi |
| 163036 | #1 | 7.9 m | 4.4 m |
| | #2 | 8.8 m | **4.4 m** ← ilerlemedi |

**Desen:** bırak → hızlan → yaklaş (~3-4 m) → mandal yeniden kurul → takıl →
bırak … Döngü çalışıyor ama **her turda AYNI menzile kadar iniyor**, daha
yakınına değil.

⇒ Ö-T "6-9 m'de sonsuz kilit"i kırıyor (araç artık 3-4 m'ye inebiliyor)
ama temasa (≤0.5 m) vardırmıyor.

### Bu yeni bir soru doğuruyor
Araç 3-4 m'ye iniyor, oradan neden ilerleyemiyor? Terminal hızı 16, hedef
15.15 → kapanma 0.85 m/s; 3.4 m'den 0.5 m'ye inmek 3.4 saniye ister.
O süre içinde ne oluyor — hedefin yanından mı geçiyor, dikey/yanal hata mı
devreye giriyor? **Kampanya sonrası incelenecek.**

⚠ Bu bulgu Ö-T'yi elemez de onaylamaz. Birincil ölçüt (koşu-başı en iyi
menzil) kontrol koluyla kıyaslanınca karar verilecek.

## ⚑⚑ ARA BULGU 3 — "3-4 m'den neden ilerleyemiyor" sorusunun cevabı

Menzil 2-5 m'ye indiğinde sonraki 2 saniyede ne olduğu sayıldı (n=80):

| sonuç | oran |
|---|---|
| **UZAKLAŞTI** | **%52** |
| **KUTU KAYBOLDU** | **%30** |
| yaklaştı (0.5-2 m) | %18 |
| TEMAS | **%0** |

Kutu kaybolmadan önceki son kare (menzil <8 m, n=345):

| ölçüm | değer |
|---|---|
| menzil | 3.3 m |
| kutu boyutu | 48 px |
| **kadraj kenarında** | **%71** |
| üst kenar | %23 |
| alt kenar | %28 |
| sağ kenar | %15 |
| sol kenar | %9 |

⇒ **Dikey kenarlar (%51) yatay kenarların (%24) iki katı.** Son 3 metrede
hedef kadrajın ÜSTÜNDEN veya ALTINDAN çıkıyor → kutu kayboluyor → kör hücum
→ ıska.

### Teşhis zinciri tamamlandı

1. Terminal mandalı 6-9 m'de sonsuz kilit yaratıyordu → **Ö-T bunu kırdı**
2. Araç artık 3-4 m'ye iniyor
3. **Ama 2-5 m bandında %52 uzaklaşıyor, %30 kutuyu kaybediyor**
4. Kutu kaybının yarısı **dikey** kadraj kenarlarında
5. Sonuç: temas %0

⇒ **Sıradaki iş: son 3 metrede hedefi kadrajda tutmak** — özellikle dikeyde.
Bu, gecenin "dikey ıska baskın" bulgusunun devamı ve muhtemelen aynı kökten.

⚠ Ö-T bu sorunu çözmez, çözmesi de beklenmez — o mandal kilidini hedefliyor
ve orada işini yaptı. Ama Ö-T olmadan bu banda hiç girilemiyordu; yani Ö-T
**bir sonraki darboğazı görünür kıldı.**

---

# SONUÇ — Ö-T, 19 koşu (7 açık / 12 kapalı), 8 sim oturumu

## Mekanizma kapısı: GEÇTİ (7/7)
`term_kapanmasiz` açık kolun 7 koşusunun 7'sinde de sıfırdan farklı
(3.96, 3.96, 3.97, 1.47, 3.96, 0.6, 3.56). Kapalı kolda 11/11 sıfır.
gcs günlüğünde **15** kez "SÜREYLE bırakıldı". Özellik kesinlikle çalıştı.

## BİRİNCİL ÖLÇÜT: KÖTÜLEŞTİ

| ölçüt | kapalı (n=12) | açık (n=7) |
|---|---|---|
| **★ koşu-başı en iyi menzil medyanı** | **0.71 m** | **1.02 m** |
| en iyi tekil geçiş | 0.18 m | 0.15 m |
| **≤0.5 m TEMAS** | **5** | **1** |
| ≤2 m geçiş | 17 | 19 |
| \|dikey\| ıska medyanı | **0.12 m** | 0.87 m |

## İKİNCİL: TAKILMA ARTTI — mekanizmanın hedefinin TERSİ

| ölçüt | kapalı | açık |
|---|---|---|
| **takılma (6-12 m bandı)** | **%7** | **%37** |
| `cx` işaret değişimi | 0.39/s | 1.54/s |
| \|roll\| p90 | 12.4° | 17.5° |
| görsel temas | %67.0 | %63.0 |

⚠ **Ö-T takılmayı ÇÖZMEDİ, ARTIRDI.** Sebep, ara bulgu 2'de görülen döngü:
bırak → hızlan → yaklaş → mandal yeniden kurul → takıl → bırak… Bu döngü
aracı 6-12 m bandında **tutuyor**. Kapalı kolda araç ya o banda hiç
girmiyor ya da geçip gidiyor.

## HÜKÜM: GİRMEZ

Karar kuralı (önceden ilan): *"Birincil kötüleşir **veya** temas sayısı
düşerse → GİRMEZ"*. **İkisi de gerçekleşti:**
- birincil 0.71 → 1.02 m (kötüleşti)
- temas 5 → 1 (düştü)

Ayrıca dikey ıska 0.12 → 0.87 m kötüleşti ve salınım dört kat arttı.

`AVCI_IBVS_TERM_BIRAK_S` varsayılanı **0.0'da (kapalı) kalır.**

⇒ §5.12 gereği elenen özellik koddan **tamamen çıkarılmalı** (Cfg alanları,
kod bloğu, CSV sütunu, panel düğmesi, birim testi). Bu silme işlemi güdüm
kodunu değiştirdiği için **kullanıcı onayına bırakıldı** — özellik varsayılan
kapalı olduğu için bekletmek zarar vermiyor.

---

## ⚑⚑ ASIL BULGU — SİSTEM ZATEN ÇALIŞIYOR, TAKILMA İSTİSNA

Kontrol kolunun (mevcut sistem, hiçbir deney açık değil) sayıları:

| ölçüt | değer |
|---|---|
| koşu-başı en iyi menzil medyanı | **0.71 m** |
| ≤0.5 m TEMAS | **5** (12 koşuda) |
| \|dikey\| ıska | **0.12 m** |
| takılma | **%7** |

**12 koşunun çoğunda takılma YOK ve 5 temas var.**

⇒ Kullanıcının 2026-08-17'de gördüğü 261 saniyelik takılma **istisnai bir
geometri**, sistemin normal davranışı değil. Takılma bazı karşılaşma
geometrilerinde oluşuyor, hepsinde değil.

⚠ Bu, gecenin dikey bulgusuyla da tutarlı: kontrol kolunda |dikey| ıska
0.12 m — yani dikey eksen bu kampanyada zaten iyiydi. Dikey sorunu
"her zaman" değil "bazı koşularda" var.

**Doğru sıradaki iş, takılmayı kovalamak değil:** son 3 metrede hedefin
kadrajdan çıkması (%30 kutu kaybı, yarısı dikey kenarlarda) ve 2-5 m
bandında %52 uzaklaşma. Ara bulgu 3'e bakınız.
