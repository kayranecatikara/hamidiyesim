# Ö5 KAREDE — DÖNÜŞ-FARKINDA HIZ TAVANI, TASARIM ZARFI İÇİNDE

> **Ölçütler ve karar kuralı KOŞMADAN ÖNCE yazıldı (CLAUDE.md §4).**
> Yazım tarihi 2026-08-16, ilk uçuştan önce.

---

## 0 · NİYE ELENMİŞ BİR ÖZELLİK YENİDEN ÖLÇÜLÜYOR (§5.13)

Ö5 (`v ≤ DONUS_A/λ̇`) 2026-08-11'de **10 uçuşla ölçüldü ve GİRMEDİ**:
sağa aşım 14.6 → 14.2 m (berabere), en yakın menzil 1.89 → **3.23 m
kötüleşti**. O kampanyanın tamamı `duz` + kaçamak senaryosundaydı.

§5.13 şunu sorar: **"bu özelliğin devreye girip TAMAMLANMASI için senaryoda
ne bulunmalı?"** Ö5 yalnız λ̇ büyükken bağlar. D kampanyasının 24 uçuşluk
20 Hz logundan tetik oranı ölçüldü (çevrimdışı, `DONUS_A = 9.81`):

| senaryo | tavan BAĞLAR | komut hızı medyanı | **10 m/s tabanına dayanma** |
|---|---|---|---|
| **kare** | **%56.5** | 16.19 m/s | **%23.8** |
| düz (Ö5'in elendiği yer) | **%15.5** | 17.97 m/s | %6.3 |

**Ö5 tetiğinin %16 çalıştığı senaryoda elendi.** Karede %57 — 3.6 kat.
Bu, CLAUDE.md'nin "elenmiş listeyi tekrar önerme; öneriyorsan neden bizim
ölçümümüzden farklı sonuç vereceğini açıkla" şartını karşılayan gerekçedir.

**Kök neden ölçümü de tam bunu istiyor** (D kampanyası §1): karede LOS
dönüş hızı medyanı 42-57 °/s; aracın verebildiği ω_max = g·tan(45°)/16 =
**35 °/s**. Kutulu karelerin **%65-80'i takip edilemez** durumda. Jerk bu
formülde yok — kaldıraçlar `V` ve `ANGLE_MAX`. Ö5 tam olarak `V` kaldıracıdır.

**Girdi sinyali sağlam mı — kontrol edildi:** `los_hiz_az` EMA'lı; ardışık
20 Hz örneklerde işaret değiştirme oranı **%4.9**. Gürültü tek yönlü bir
kısıcıyı sürekli tetikleyecek düzeyde DEĞİL.

---

## 1 · TEK DEĞİŞKEN VE DEĞERİ

```
KONTROL : DONUS_A = 0      (bugünkü varsayılan — kapalı)
DENEY   : DONUS_A = 9.81   DONUS_V_MIN = 10 (DEĞİŞMİYOR)
```

**9.81 ayarlanmış bir sayı değil, FİZİKTEN türetildi:**
`g · tan(ANGLE_MAX = 45°) = 9.81 m/s²` — aracın üretebileceği en büyük
yanal ivme. Kural şu hâle gelir: *"üretemeyeceğin yanal ivmeyi isteme."*
Kazanç arama turu yapılmayacak; bu değer kaybederse özellik kaybeder.

Diğer değerler çevrimdışı bakıldı ve NİYE seçilmedikleri yazılıdır:
`6.0` → %74 bağlar, komut hızı medyanı **10.14 m/s** (taban) — hedef
15.1 m/s uçarken kalıcı geri kalma; `12.0` → 50.7° yatış ister, ANGLE_MAX
45°'in üstünde, yani fiziksel olarak ulaşılamayan bir bütçe.

⚠ Diğer koşullar D kampanyasının bıraktığı yerde: **`PSC_JERK_XY = 10`**
her iki kolda da aynı.

---

## 2 · MEKANİZMA KAPISI (§5.1)

`donus_tavan` sütunu (`bbox_ibvs_*.csv`) zaten var ve Ö5 kapalıyken BOŞ,
açıkken dolu yazılıyor. Bir koşu VERİ NOKTASI sayılmaz, şu ikisi olmadıkça:

1. Deney kolunda `donus_tavan` dolu olan kare oranı **%40'ın üstünde**
   (çevrimdışı öngörü %56.5; %40 altına düşerse özellik ölçülmemiştir).
2. Kontrol kolunda `donus_tavan` sütunu **tamamen boş**.

Ayrıca deney kolunda **`v_los` medyanı kontrol kolundan DÜŞÜK** olmalı —
tavan bağlıyorsa hız düşer; düşmüyorsa kapı kapalıdır.

---

## 3 · ÖLÇÜTLER

### BİRİNCİL · **60 m içinde geçen süre** (kare, koşu medyanı)

Kaynak `meta.csv` (1 Hz). Yüksek = iyi.

*Niye bu:* kullanıcının hedefi *"hedef manevra yaparken arasındaki mesafeyi
çok açmasın, geride kalmasın"*. Ö5'in ilan edilmiş RİSKİ tam da bunun
tersi: hızı kısarak geride kalmak. Ölçüt, özelliğin **en zayıf** olduğu
yerden sorar — kendi lehine seçilmiş bir ölçüt değildir (§5.5, §5.6).

*§5.2 geçerlilik eşi:* **medyan mesafe** (düşük = iyi). İkisi ters yöne
giderse hüküm kurulmaz.

*⛔ D kampanyasında öğrenildi:* "en yakın menzil" birincil YAPILMAZ —
isabetsiz senaryoda savrulup şans eseri yaklaşmayı ödüllendiriyor (§5.2).
İkincil olarak, 20 Hz kutu boyutundan raporlanır (§5.3).

### İKİNCİL

| # | ölçüt | kaynak | niye |
|---|---|---|---|
| İK-1 | medyan mesafe | meta 1 Hz | BİRİNCİL'in geçerlilik eşi |
| İK-2 | **rota salınımı ψ̇ işaret değişimi/s** | **`telem.csv` 10 Hz, KOŞULSUZ** | kullanıcının şikâyeti; kutu oranı yanlılığından bağımsız (D §7.2) |
| İK-3 | \|ψ̇\| p90 | telem 10 Hz | dönüş sertliği |
| İK-4 | görsel fazda kutu oranı | bbox 20 Hz | temas korundu mu; **her kutu-tabanlı sayının geçerlilik eşi** |
| İK-5 | en yakın menzil (20 Hz kutudan) | bbox 20 Hz | bilgi |
| İK-6 | dönüş yarıçapı medyanı | meta 1 Hz | mekanizmanın amacı: R küçülmeli |
| İK-7 | `v_los` medyanı + taban (%10 m/s) oranı | bbox 20 Hz | **ilan edilen riskin doğrudan ölçüsü** |
| İK-8 | isabet | olay.json | karede taban 0/12 — n=4'te ayırması BEKLENMİYOR |

---

## 4 · ETKİ ALANI TABLOSU (§5.10)

| etkilenebilecek davranış | neden etkilenebilir | hangi senaryoda sınanır |
|---|---|---|
| **düz, sakin kuyruk takibinde isabet** | Ö5 düzde de %15.5 oranında bağlıyor; 2026-08-11 ölçümü orada **en yakın menzili 1.89 → 3.23 m kötüleştirmişti**. Üstelik o ölçüm jerk 5'te yapıldı, bugünkü taban jerk 10 — taban değişti, tekrar sınanmalı. | **`duz` + `yatay`/`capraz`, n=4/kol** |
| **hedefe yetişememe (kalıcı yavaşlık)** | karelerin %24'ünde hız 10 m/s tabanına dayanıyor, hedef 15.1 m/s | **kare — İK-7 ve BİRİNCİL doğrudan ölçüyor** |
| **dikey kanal** | — | **YAPISAL:** Ö5 yalnız `v_los`'u (yatay LOS hızı) kısar; `vz` ayrı hesaplanır ve bu tavandan geçmez. Birim testi B56 "Ö5 hızı ASLA artırmaz" zaten var; **B-yeni ile dikey denklik de kanıtlanacak** (regresyon testinden güçlüdür). |
| **sürekli dönüşte (daire)** | dairede λ̇ sabit ve büyük → tavan KALICI bağlar → araç sürekli yavaş kalabilir. Bu Ö11'de yaşanan hatanın aynısı. | **`circle`, n=2/kol — karar vermez, model çürütür** |

**"Hedeflenen yeri iyileştirdi ama başka bir yeri bozdu mu?"** — düz ve
daire koşulmadan cevaplanmaz; koşulmazsa rapor değil **eksik listesi**dir.

---

## 5 · KOŞU PLANI (§4 dönüşümlü, §5.9 tür-eşli)

**Kare (kazanım) — 8 uçuş:**
```
F01_K_kare  F02_O_kare  F03_K_kare  F04_O_kare
F05_K_kare  F06_O_kare  F07_K_kare  F08_O_kare
```
**Düz (regresyon) — 8 uçuş, tür-eşli:**
```
F09_K_duz_yatay   F10_O_duz_yatay   F11_K_duz_capraz  F12_O_duz_capraz
F13_K_duz_yatay   F14_O_duz_yatay   F15_K_duz_capraz  F16_O_duz_capraz
```
**Daire (model çürütme) — 4 uçuş, KARAR VERMEZ:**
```
F17_K_daire  F18_O_daire  F19_K_daire  F20_O_daire
```

⚠ `E*` adları Ö-B kampanyasında dolu; üzerine-yazma koruması (§5.7) ilk
denemede uyardı, prefix `F` yapıldı. Hiçbir eski koşu kaybedilmedi.
Tür dağılımı her kolda: 4 kare + 2 yatay + 2 çapraz + 2 daire. **Eşit.**

---

## 6 · KARAR KURALI — sonuca bakmadan ilan edildi

**GİRER** (varsayılan `DONUS_A = 9.81` olur), ÜÇÜ birden sağlanırsa:
1. Karede BİRİNCİL (60 m içinde süre) **artar**, ve
2. İK-1 (medyan mesafe) **artmaz** (yakınlaşma gerçek), ve
3. Düz regresyonda deney kolu isabeti kontrol kolundan **az değil**.

**ÇIKAR**, şunlardan biri olursa:
- Karede BİRİNCİL **azalır** (özelliğin ilan edilen riski gerçekleşti:
  hız kısıldı, geride kalındı), **veya**
- Düz regresyonda deney kolu **0/4** alırken kontrol ≥2/4, **veya**
- Mekanizma kapısı kapalı (§2).

**KULLANICIYA**: geri kalan her hâl — özellikle karede kazanıp düzde
kaybettiği bölünmüş sonuç. Ölçütle oynanmaz (§5.6).

**Dairede gerileme** kararı tek başına değiştirmez ama **raporlanır** ve
"kalıcı yavaşlık" modeli çürütülmediyse açıkça yazılır.

---

## 7 · SONUÇLAR — 20 uçuş, 2026-08-16

### 7.0 · Mekanizma kapısı (§2) — SONUNA KADAR AÇIK

| | kontrol | Ö5 | ilan edilen şart |
|---|---|---|---|
| `donus_tavan` dolu kare oranı (kare) | **%0.0** | **%94.1** | deneyde >%40, kontrolde 0 ✓ |
| `v_los` medyanı (kare) | 20.25 | **15.63** m/s | deneyde DÜŞÜK ✓ |
| 10 m/s tabanına dayanma (kare) | %0 | **%25.5** | öngörü %23.8 ✓ |

Çevrimdışı öngörü tetik oranını %56.5 demişti, gerçek %94.1 — çünkü öngörü
loglanmış `v_los`'u (≈16) kullanmıştı; kontrol kolunun gerçek `v_los`'u
kare IBVS fazında 20.25 m/s'ye çıkıyor, tavan o yüzden çok daha sık bağlıyor.
**Öngörü yönü doğru, büyüklüğü küçüktü** — kapı yine de açık.

### 7.1 · KARE (kazanım) — n=4/kol

| ölçüt | KONTROL | Ö5 | |
|---|---|---|---|
| **BİRİNCİL · 60 m içinde süre** | 114.0 s | **142.5 s** | +%25, p=0.086 |
| İK-1 medyan mesafe (geçerlilik eşi) | 62.3 m | **55.6 m** | ✓ **aynı yönde** |
| İK-6 dönüş yarıçapı | 98.0 m | 90.4 m | −%8 |
| İK-5 en yakın menzil | 4.0 m | 3.8 m | berabere |
| İK-4 kutu oranı | %34.1 | %29.8 | hafif düşük |
| İK-2 ψ̇ işaret değişimi/s | 0.274 | 0.331 | **hafif KÖTÜ**, p=0.086 |
| isabet | 0/4 | 0/4 | ayırması beklenmiyordu |

Birincil arttı, geçerlilik eşi aynı yöne gitti. **Karede kazanım gerçek.**
⚠ Ama beklenen mekanizma (yarıçap küçülmesi) sadece %8 — hız %23 kısıldığı
hâlde. R ∝ V² bekletisi %41 düşüş isterdi; gelmedi. Yani kazanım
"daha dar dönüyoruz"dan çok "yavaşlayınca hedefi kaybetmiyoruz"dan geliyor.

### 7.2 · DÜZ (regresyon, §5.10) — n=4/kol, TÜR-EŞLİ

| | KONTROL | Ö5 |
|---|---|---|
| **yatay** (ani yanal kırılma) | **2/2** | **0/2** — 2.03 / 2.32 m |
| **çapraz** | **2/2** | **2/2** — 0.88 / 0.53 m |
| **TOPLAM** | **4/4** | **2/4** |
| medyan mesafe | 35.5 m | **24.2 m** (daha yakın) |
| kutu oranı | %65.1 | **%85.1** (daha iyi temas) |
| ψ̇ işaret değişimi/s | 1.839 | **1.493** (daha sakin) |

**§5.9 tür-içi kıyas şart:** kayıp tamamen `yatay` kaçamağında.
Çaprazda hiçbir şey kaybedilmiyor, hatta en yakın menzil daha iyi.

⚠ **2026-08-11 Ö5 kampanyası BAĞIMSIZ OLARAK DOĞRULANDI:** o zaman
"en yakın menzil 1.89 → 3.23 m kötüleşti" ölçülmüştü. Bugün yatay kolunda
1.4/1.3 → 2.03/2.32 m. **Aynı imza, farklı kampanya, farklı jerk tabanı.**

### 7.3 · DAİRE — İLAN EDİLEN MODELİM YARISI YANLIŞ ÇIKTI

İlan etmiştim: *"dairede λ̇ sabit ve büyük → tavan KALICI bağlar → araç
sürekli yavaş kalabilir. Bu Ö11'de yaşanan hatanın aynısı."*

**Yavaşlık kısmı DOĞRU, sonuç kısmı YANLIŞ:**

| | KONTROL | Ö5 |
|---|---|---|
| tavan bağlar | %0 | **%99.1** |
| 10 m/s tabanına dayanma | %0 | **%39.2** |
| `v_los` medyanı | 19.27 | **11.92 m/s** (hedef 15.1!) |
| 60 m içinde süre | 107.5 s | **173.5 s** |
| medyan mesafe | 64.8 m | **50.3 m** |
| dönüş yarıçapı | 50.7 m | **38.1 m** |
| en yakın menzil | 3.7 m | 5.7 m (kötü) |
| ψ̇ işaret değişimi/s | 0.265 | **0.122** (yarı yarıya sakin) |

Araç hedeften **kalıcı olarak yavaş** uçuyor (11.9 vs 15.1 m/s) ve buna
rağmen **geride kalmıyor, daha yakın duruyor.** Sebebi: dairede kaybettiğimiz
şey hız değil, GEOMETRİ — yavaşlayınca dönüş yarıçapı 50.7 → 38.1 m'ye
düşüyor ve içeriden kestirme yapılıyor. "Yavaş kalırsa yetişemez" sezgim
bu senaryoda yanlıştı; açıkça düzeltiyorum.

### 7.4 · BÜTÜN SENARYOLARDA TEK BİR İMZA

Üç senaryoda da aynı şey oluyor: **Ö5 takibi belirgin biçimde iyileştiriyor,
bitirişi bozuyor.**

| | takip (medyan mesafe / kutu / sakinlik) | bitiriş (isabet / en yakın) |
|---|---|---|
| kare | ✓ 62.3→55.6 m, 60m süre +%25 | — 0/4 ↔ 0/4 |
| düz | ✓ 35.5→24.2 m, kutu %65→%85, ψ̇ 1.84→1.49 | ✗ **4/4 → 2/4** |
| daire | ✓ 64.8→50.3 m, ψ̇ 0.27→0.12 | ✗ en yakın 3.7→5.7 m |

*Video teyidi (§2 adım 4-6):* F09 (kontrol, İSABET) f0136 hedef merkezde,
kutu ~110 px, ufuk düz → f0137 çarpma. F10 (Ö5, IŞKA 2.03 m) f0127 hedef
merkezde, kutu ~100 px, ufuk düz, **kontrol kolundan bile SAKİN** → f0128
hedef büyümüş ama araç yanından geçiyor, uçak sağlam. **Ö5 kusursuz takip
edip bitiremiyor.** Bu bir kontrol kusuru değil, kapanma hızı yokluğudur:
16 m/s tavanla 15.1 m/s hedefe kalan kapanma 0.9 m/s.

### 7.5 · KARAR KURALININ UYGULANMASI

**GİRER** üç şart ister:
1. Karede birincil artar → **✓** (114.0 → 142.5 s)
2. İK-1 medyan mesafe artmaz → **✓** (62.3 → 55.6 m)
3. Düz regresyonda isabet kontrolden az değil → **✗ (4/4 → 2/4)**

→ **GİRMEZ.**

**ÇIKAR** şartlarından hiçbiri oluşmadı: birincil azalmadı, düz kolu 0/4
değil (2/4), mekanizma kapısı açık.

→ **KULLANICIYA.** İlan ederken yazdığım *"özellikle karede kazanıp düzde
kaybettiği bölünmüş sonuç"* hâli aynen gerçekleşti. Ölçütle oynanmadı (§5.6).

**Varsayılan `DONUS_A = 0` (kapalı) BIRAKILDI** — kullanıcı karar verene
kadar davranış değişmiyor. Panel düğmesi duruyor.

### 7.6 · RAPORDAN ÖNCE ÜÇ SORU (§5.8)

1. **Özellik çalıştı mı?** Evet, fazlasıyla — %94-99 tetik, `v_los`
   20.25 → 15.63 (kare), 19.27 → 11.92 (daire).
2. **Ölçütüm kötü bir sebeple mi iyileşti?** Karede birincil arttı ve
   geçerlilik eşi (medyan mesafe) AYNI yöne gitti; ayrıca kutudan bağımsız
   ψ̇ ölçütü de kullanıldı. Düzde ise kutu oranı Ö5 kolunda daha YÜKSEK
   (%85 vs %65) — yani "isabeti kaybetti çünkü hedefi göremedi" açıklaması
   **elenir**; hedefi daha iyi görüyordu ve yine de vuramadı.
3. **n kaç?** Kare 4/kol, düz 4/kol (§5.4 alt sınırı, hüküm kurulur),
   daire 2/kol (model çürütme, karar vermez — öyle de kullanıldı).

### 7.7 · BUNDAN SONRASI İÇİN KALICI BULGU

Ö5'in imzası — *takip düzelir, bitiriş bozulur* — üç senaryoda tekrarlandı
ve iki ayrı kampanyada (2026-08-11 ve bugün) aynı çıktı. Sebep tek satırda:

> `V_TERMINAL = 16 m/s`, hedef 15.1 m/s → kalan kapanma **0.9 m/s**.
> Ö5 hızı 12-16 m/s'ye kısınca bu marj **sıfırlanıyor ya da negatife
> dönüyor**; araç hedefi mükemmel izler ama asla yetişemez.

Bu, "hızı kısmak" ailesindeki HER özelliğin (Ö5, Ö11, Ö-B) neden aynı
duvara çarptığını açıklar. Bir sonraki adım bu ailenin devamı olmamalı;
kısmayı **menzile göre kapatan** bir yapı (uzakta kıs, terminalde bırak)
ya da doğrudan `ANGLE_MAX`/`V_TERMINAL` kanalı sınanmalıdır.
