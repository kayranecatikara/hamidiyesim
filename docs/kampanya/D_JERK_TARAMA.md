# D KAMPANYASI — `PSC_JERK_XY` TARAMASI (5 / 10 / 15)

> **Ölçütler ve karar kuralı KOŞMADAN ÖNCE yazıldı (CLAUDE.md §4).**
> Yazım tarihi 2026-08-16, ilk uçuştan önce.

---

## 0 · NEDEN BU KAMPANYA — kullanıcının kendi uçuşu

Kullanıcı `PSC_JERK_XY = 15`'i kendi makinesinde uçurdu
(`logs/kayit/ucus_20260816_144037`, 69 kare) ve şunu bildirdi:

> "hedef hava aracı manevra yaptığı esnada bizim dronda çok çevik bir hareket
> göstererek hedef araca doğru yöneliyor, hedef hava aracıyla arasındaki
> mesafeyi çok açmıyor, geride kalmıyor — **bu iyi**.
> AMA ... drone daha **kontrolsüz hareketler** yapmaya başladı ve **salınım**
> var. ... Bu ivme şeyini de 5'ten direkt 15'e çektik ya, acaba biraz
> azalabilir mi, 10 falan olabilir mi, **en iyi değerine çekelim**."

Kampanyanın tek sorusu budur: **5, 10, 15 — hangisi?**

---

## 1 · KULLANICI UÇUŞUNUN ÇÖZÜMLEMESİ (kampanyanın girdisi)

`logs/kayit/ucus_20260816_144037` + `logs/bbox_ibvs_20260816_144045.csv`
(1489 satır, 20 Hz, 74 s).

**Yaklaşma iyi:** 59 m → 3.2 m (gerçek 3B), 21 saniyede. Isabet zarfı
±0.65 m; ıska. Bu bacak kullanıcının "geride kalmıyor" dediği şeydir.

**Sonra 47 saniye boyunca LİMİT ÇEVRİMİ:** araç 6.4-15.9 m bandında kaldı,
bir daha hiç kapatamadı. 20 Hz logda çevrimin imzası nettir — yatış
**±40°** (ANGLE_MAX 45° tavanına dayanıyor), periyot **4-5 s**:

```
t(s)  41.0  41.6  42.1  42.6  43.1  43.6  44.1  44.6  45.1
roll   +37   +42   +35    +0   -35   -38   -21   +19   +29
cx    -33   -50   -27    -4   +32   +38   +30    -8   -30
```

`eps_yaw` bu sırada ±5° — yani **güdüm hatası küçük, araç yine de tavana
dayanıyor.** Salınımı üreten şey hata büyüklüğü değil.

**Kök neden — GEOMETRİ, kazanç değil (§5.11).**
LOS'u merkezde tutmak için gereken dönüş hızı λ̇; aracın verebildiği
ω_max = g·tan(ANGLE_MAX)/V. 16 m/s ve 45°'de **ω_max = 35 °/s**.
C kampanyasının 20 Hz loglarından ölçülen λ̇ (kutu olan kareler):

| senaryo | menzil | \|λ̇\| medyan | \|λ̇\| p90 | ω_max | **λ̇ > ω_max olan kare** | λ̇_p90 için gereken V |
|---|---|---|---|---|---|---|
| kare · jerk 5  | 10-20 m | 57.3 °/s | 138.5 °/s | 27.6 °/s | **%76** | 4.1 m/s |
| kare · jerk 15 | 10-20 m | 41.8 °/s | 108.9 °/s | 27.0 °/s | **%65** | 5.2 m/s |
| kare · jerk 15 | 0-10 m  | 73.5 °/s | 151.9 °/s | 35.0 °/s | **%71** | 3.7 m/s |
| duz · jerk 5   | 10-20 m |  4.7 °/s |  30.1 °/s | 27.2 °/s | %12 | 18.7 m/s |
| duz · jerk 15  | 10-20 m | 11.0 °/s |  35.0 °/s | 35.1 °/s | %10 | 16.1 m/s |

**Karede, kutu olan karelerin %65-80'inde LOS araçtan HIZLI dönüyor.**
Düzde bu oran %10-12. Düzün çalışıp karenin çalışmamasının sebebi budur.
Hiçbir jerk değeri bunu düzeltemez: jerk, ivmenin ne kadar ÇABUK
kurulacağını belirler, TAVANINI değil. Tavan `ANGLE_MAX` ve `V`'dir.

⚠ **Ölçütün zayıf yanı, peşinen söylenir:** `los_hiz_az` kutu merkezinin
hareketinden türetiliyor; içinde tespit gürültüsü ve aracın kendi dönüşü de
var. p90 değerleri bu yüzden şişkin olabilir. Ama medyan (57 / 42 °/s) bile
ω_max'ın (27 °/s) **1.5-2 katı** — sonuç yarıya bölünse de ayakta kalır.

**Bu kampanyanın kapsamı:** yukarıdaki tespit, jerk taramasının SONUCUNU
değiştirmez, YORUMUNU belirler. Tarama "hangi jerk en iyi uzlaşma" sorusunu
cevaplar; "salınım tamamen çözülür mü" sorusunu **cevaplamaz** — o soru
`V`/`ANGLE_MAX` kanalına aittir ve ayrı bir adımdır (bkz. §8).

---

## 2 · MEKANİZMA KAPISI (§5.1)

Bir koşu VERİ NOKTASI sayılmaz, şu ikisi birden sağlanmadıkça:

1. `kosuD.sh` param yazımından sonra **araçtan geri okunan** `PSC_JERK_XY`,
   hedeflenen seviyeye **±0.5 m/s³** içinde. (Script sağlamazsa `exit 4`.)
2. 20 Hz `iris_roll_deg`'den türetilen **jerk p90**, seviyeyle birlikte
   artıyor. C kampanyasında ölçüldü: 5 → 7.26, 15 → 14.57 m/s³.
   10 seviyesi bu ikisinin ARASINA düşmezse, o seviye **ölçülmemiştir**.

⚠ Sert tavan `0.5·√(a_max·snap_max) = 19.41 m/s³` (AC_PosControl:448).
15 tavanın altında; geri okuma yine de zorunlu.

---

## 3 · ÖLÇÜTLER — kullanıcının cümlesinden türetildi (§5.5)

### BİRİNCİL-A · yakınlık = "mesafeyi çok açmıyor, geride kalmıyor"

**Koşunun medyan mesafesi (kare senaryosu), koşular arası medyan.**
Kaynak `meta.csv` (1 Hz). Düşük = iyi.

*§5.3 örnekleme kontrolü:* ölçülen şey bir koşunun TAMAMININ merkezi
eğilimi (dakikalar mertebesi); 1 Hz bunun 100 katından hızlı. Geçer.
*(Anlık "en yakın menzil" bu kaynaktan ALINMAZ — 1 Hz onun için yasak.)*

*§5.2 geçerlilik eşi:* medyan mesafe, araç hedefi kaybedip GPS'e dönerek
kestirme uçtuğunda da düşebilir. Eşi: **60 m içinde geçen süre** (İK-1) —
ikisi aynı yöne gitmezse hüküm kurulmaz.

### BİRİNCİL-B · salınım = "kontrolsüz hareketler ve salınım"

**`cx` işaret değişimi / s, 10-20 m MENZİL KOVASI İÇİNDE.**
Kaynak `bbox_ibvs_*.csv` (20 Hz), menzil = `160/boyut`. Düşük = iyi.

*Niye kova içinde:* menzil koşullamadan yapılan kıyas, hedefe HİÇ
yaklaşamayan kolu "sakin" gösterir. C kampanyasında bu tuzak somuttu:
jerk 5, karelerinin yalnız %0.3'ünü 10 m altında geçirdi; jerk 15 %3.2.
Kaba kıyas kolları değil, **menzil dağılımını** ölçer (§5.9'un menzil
karşılığı). 10-20 m kovası her iki kolda karelerin ~%78'ini tutuyor —
kıyas oraya kurulur. Diğer kovalar da raporlanır.

*§5.2 geçerlilik eşi:* salınım YALNIZ kutu olan karelerde sayılabilir;
hedefi daha çok kaybeden koşu daha sakin görünür. Eşi: **görsel fazda kutu
oranı**. Bir seviyenin kutu oranı **%60 altındaysa** o seviyenin salınım
sayısı **GÜVENİLMEZ** ilan edilir ve karara sokulmaz.

### İKİNCİL (bilgi; tek başına karar vermez)

| # | ölçüt | kaynak | niye |
|---|---|---|---|
| İK-1 | 60 m içinde geçen süre | meta 1 Hz | BİRİNCİL-A'nın geçerlilik eşi |
| İK-2 | \|yatış\| p90, 10-20 m kovası | bbox 20 Hz | mekanizmanın şiddeti |
| İK-3 | yatış işaret değişimi / s, kova içi | bbox 20 Hz | C'de kolları AYIRMADI (0.363 / 0.343) — burada da ayırmazsa doğrulanmış olur |
| İK-4 | \|cx\| p90, kova içi | bbox 20 Hz | salınımın GENLİĞİ (BİRİNCİL-B frekansı) |
| İK-5 | en yakın menzil (20 Hz kutudan) | bbox 20 Hz | §5.3'e uygun tek yakınlık anı ölçütü |
| İK-6 | isabet | olay.json | karede taban 0/8 — n=4'te ayırması BEKLENMİYOR |
| İK-7 | KURTARMA sayısı | bbox `durum` | görsel temas kesintisi |
| İK-8 | jerk p90 | bbox 20 Hz | mekanizma kapısı (§2) |

---

## 4 · ETKİ ALANI TABLOSU (§5.10 — zorunlu, koddan ÖNCE)

| etkilenebilecek davranış | neden etkilenebilir | hangi senaryoda sınanır |
|---|---|---|
| **düz, sakin kuyruk takibinde isabet** | C ölçümü: terminalin son 2 s'sinde yanal ivme 0.57 → 3.88 m/s² (6.8×), hedefin merkezden sapması 8.5 → 21.5 px. Çevik araç, temas anında piksel gürültüsü kovalıyor. İsabet 4/4 → 0/4 düştü. | **`duz` + `yatay`/`capraz` kaçamak — kazanan seviye vs 5, n=4** |
| **dikey kanal (tırmanma/dalış)** | — | **YAPISAL GARANTİ, koşu gerekmez.** `PSC_JERK_XY` yalnız yatay düzlemi kurar (`set_max_speed_accel_NE_cm`); dikey `PSC_JERK_Z` ayrıdır ve dokunulmuyor. |
| **sürekli dönüşte (daire) takip** | jerk ivmenin DEĞİŞİMİNİ sınırlar; sabit yarıçaplı dönüşte ivme sabittir → etkisiz olmalı | **C kampanyasında ÖLÇÜLDÜ ve öngörü tuttu** (en yakın 3.90 → 3.30 m, fark yok). Tekrar koşulmaz; jerk 10, 5 ile 15 arasında olduğu için arada kalır. |
| **görsel temas kesintisi (KURTARMA)** | daha sert manevra hedefi kadrajdan çıkarabilir | **kare** (İK-7) |

**"Hedeflenen yeri iyileştirdi ama başka bir yeri bozdu mu?"** — düz
regresyonu koşulmadan bu soruya cevap verilmez; koşulmazsa rapor değil,
**eksik listesi** sunulur.

---

## 5 · KOŞU PLANI (§4 dönüşümlü)

**Kare (kazanım) — 12 uçuş, 5/10/15 dönüşümlü 4 tur:**

```
D01_J05_kare  D02_J10_kare  D03_J15_kare
D04_J05_kare  D05_J10_kare  D06_J15_kare
D07_J05_kare  D08_J10_kare  D09_J15_kare
D10_J05_kare  D11_J10_kare  D12_J15_kare
```

Her koşu: `bash ~/.avci_sim/kosuD.sh <ad> yok <jerk> 8 240 square`
(kaçamak `yok`; kare senaryosunun köşeleri zaten manevradır).

**Düz (regresyon) — tür-eşli (§5.9), kazanan seviye belli olunca 4 uçuş.**

> **PLAN DEĞİŞİKLİĞİ (uçuşlar sırasında, kare sonucu görüldükten sonra):**
> karar kuralı Adım 2 **iki aday** bıraktı (10 ve 15), tek kazanan çıkmadı.
> Bu yüzden düz regresyonu tek seviye yerine **üç seviyenin hepsiyle**
> koşuldu — bu bir daraltma değil GENİŞLETMEDİR, seviye seçilerek
> kiraz toplanmadı. Ve n=2/seviye §5.4'ün altında kaldığı için ikinci tur
> eklendi → **n=4/seviye, her seviyede 2 yatay + 2 çapraz.**

```
D13_J05_yatay   D14_J10_yatay   D15_J15_yatay
D16_J05_capraz  D17_J10_capraz  D18_J15_capraz
D19_J05_yatay   D20_J10_yatay   D21_J15_yatay
D22_J05_capraz  D23_J10_capraz  D24_J15_capraz
```

n = 4/seviye (kare → hüküm kurulur), 4/seviye (düz → isabet kapısı).

⚠ n=4 §5.4'ün alt sınırıdır. Üç seviyeli permütasyon testinde
ulaşılabilen en küçük p, 4+4'te 0.057'dir. **p < 0.05 aranmaz**;
karar kuralı ayrımın YÖNÜNE ve büyüklüğüne bakar, p yalnız raporlanır.

---

## 6 · KARAR KURALI — sonuca bakmadan ilan edildi (§4)

Adımlar sırayla uygulanır; sonuca bakıp ölçüt seçmek yasaktır.

**Adım 0 — geçerlilik.** Mekanizma kapısından (§2) geçmeyen koşu atılır ve
yerine yenisi uçulur. Kutu oranı %60 altındaki seviyenin BİRİNCİL-B sayısı
GÜVENİLMEZ ilan edilir.

**Adım 1 — yakınlıkta ayrım var mı?** Üç seviyenin medyan mesafesi
permütasyon testinde ayrılmıyorsa (en iyi ile en kötü arası **p > 0.3**
VEYA fark **< %10**), yakınlık ekseni kararsız sayılır → **Adım 3'e geç**.

**Adım 2 — en iyi seviye.** Medyan mesafesi EN DÜŞÜK seviyenin **%10'u
içinde** kalan seviyeler "yakınlıkta eşdeğer" kabul edilir. Bunların
arasından **BİRİNCİL-B (cx dgs/s, 10-20 m) EN DÜŞÜK olanı SEÇİLİR.**

> *Niye böyle:* kullanıcı 2026-08-12'de açıkça *"biraz mesafe açılsa ama
> salınım olmasa okeydir"* dedi. Kural o cümleyi uygular: yakınlıkta
> eşdeğer olanlar arasında sakinlik kazanır.

**Adım 3 — kararsızlık.** Adım 1 kararsız çıkarsa veya iki eksen ters
yönde ayrılıp Adım 2 tek aday bırakmazsa → **KULLANICIYA**, ölçütle
oynanmaz (§5.6).

**Adım 4 — regresyon kapısı (§5.10).** Seçilen seviye düz regresyonda
**0/2** isabet alırken jerk 5 **≥1/2** alırsa: bu bir GERİLEMEDİR, gizlenmez.
Seçim iptal EDİLMEZ, ölçüsüyle raporlanır ve **kararı kullanıcı verir.**

**Varsayılan.** Seçilen seviye `sim/ardupilot_params/avci_copter.parm`'a
`PSC_JERK_XY` satırı olarak yazılır. Seçim 5 çıkarsa satır YAZILMAZ
(firmware varsayılanı zaten 5) ve C adayı §7 kuralı uyarınca ÇIKAR.

---

## 7 · SONUÇLAR — 24 uçuş (12 kare + 12 düz), 2026-08-16

### 7.0 · Mekanizma kapısı (§2) — GEÇTİ

24 koşunun 24'ünde `PSC_JERK_XY` **araçtan geri okundu** ve hedefe eşitti.
Bir koşu (D15, ilk deneme) araç `PARAM_VALUE` cevabı vermediği için
**reddedildi ve uçurulmadı** — dizin yaratılmadı, aynı adla tekrarlandı.

Jerk p90 (20 Hz `iris_roll_deg`'den, kare kolu koşu medyanı):

| seviye | jerk p90 (ölçülen) | beklenen sıra |
|---|---|---|
| 5  | **6.73** m/s³ | — |
| 10 | **10.49** m/s³ | 5 ile 15 arasında ✓ |
| 15 | **13.33** m/s³ | — |

Monoton ve ara seviye gerçekten arada. Kapı açık.

### 7.1 · KARE (kazanım senaryosu) — n=4/seviye

| seviye | isabet | **medyan mesafe** | 60 m süre | en yakın (20 Hz) | kutu % | cx dgs/s | \|cx\| p90 | yatış p90 | ψ̇ dgs/s |
|---|---|---|---|---|---|---|---|---|---|
| 5  | 0/4 | **66.0 m** |  96 s | 6.5 m | 30.2 | 0.142 | 81 px | 25.8° | 0.203 |
| 10 | 0/4 | **59.5 m** | 122 s | 2.1 m | 38.6 | 0.514 | 55 px | 35.8° | 0.240 |
| 15 | 0/4 | **54.5 m** | 134 s | 2.4 m | 41.5 | 0.496 | 62 px | 37.3° | 0.257 |

Permütasyon (n=4+4'te ulaşılabilen en küçük p = 0.057):

| kıyas | medyan mesafe | 60 m süre | cx dgs/s |
|---|---|---|---|
| 5 vs 10 | 6.45 m, p=0.086 | 25.0 s, p=0.086 | 0.372, p=0.057 |
| 5 vs 15 | 11.45 m, **p=0.057** | 37.5 s, **p=0.057** | 0.354, p=0.057 |
| 10 vs 15 | 5.00 m, p=0.229 | 12.5 s, p=0.200 | 0.018, p=0.800 |

### 7.2 · ⚠ BİRİNCİL-B GEÇERSİZ ÇIKTI — ilan edilen eşik tuttu

**Kutu oranı üç seviyede de %60'ın ALTINDA** (30.2 / 38.6 / 41.5).
§3'te ilan edilen kural gereği **BİRİNCİL-B (cx dgs/s) GÜVENİLMEZ** ilan
edilir ve karara SOKULMAZ. Kural sonucu görmeden yazılmıştı; şimdi tam da
uyardığı şey oldu: `cx` yalnız kutu olan karelerde sayılabiliyor ve jerk 5,
10-20 m kovasında **430 kare** verirken jerk 15 **806 kare** veriyor. Jerk
5'in "sakinliği", hedefi daha çok kaybetmesiyle iç içe geçmiş durumda —
ayrıştırılamaz.

**Kutudan BAĞIMSIZ ölçüm yapıldı** (§5.2'nin gerektirdiği çözüm; ölçüt
kapı kapandıktan SONRA eklendi ve bu açıkça belirtilir): `telem.csv`
**10 Hz ve koşulsuz** yazılıyor. Aracın kendi konum izinden rota açısı ψ
türetildi; ψ̇ işaret değişimi = savrulma. Salınım periyodu 4-5 s → ~45
örnek/periyot (§5.3 fazlasıyla geçer).

| seviye | ψ̇ işaret değişimi/s | \|ψ̇\| medyan | \|ψ̇\| p90 |
|---|---|---|---|
| 5  | 0.203 | 7.4 °/s | 15.7 °/s |
| 10 | 0.240 | 10.2 °/s | 23.8 °/s |
| 15 | 0.257 | 11.6 °/s | 24.3 °/s |

p: 5-10 **0.286**, 5-15 **0.400**, 10-15 **0.571** — **hiçbiri ayrılmıyor.**
Tek koşular 0.081 ile 0.331 arasında saçılıyor; kol farkı bu saçılmanın
içinde kayboluyor.

> **Bulgu:** jerk arttıkça araç **daha SIK dönüş değiştirmiyor** (frekans
> ayrılmıyor); **her dönüşü daha SERT yapıyor** (|ψ̇| p90 15.7 → 24.3 °/s).
> Ayrıca `|cx|` p90 81 → 55/62 px'e **DÜŞÜYOR** — hedef kadrajda merkeze
> daha YAKIN tutuluyor, sadece merkezi daha sık geçiyor.
> Kullanıcının gördüğü salınım gerçektir, ama ölçüm onu **"uçuş yolu
> düzensizleşti"** değil **"dönüşler sertleşti, gövdeye sabit kamera
> onlarla birlikte savruluyor"** diye tarif ediyor.

*Video teyidi (§2 adım 6):* jerk **5** koşusunda da (D10, f0023) ufuk ~40°
yatık. ±40° yatış jerk 15'e özgü DEĞİL — karede her seviye tavana dayanıyor.
§1'deki "geometri, kazanç değil" tespitiyle örtüşüyor.

### 7.3 · DÜZ (regresyon, §5.10) — n=4/seviye, tür-eşli (2 yatay + 2 çapraz)

| seviye | yatay | çapraz | **TOPLAM isabet** | en yakın medyan |
|---|---|---|---|---|
| 5  | 2/2 | 2/2 | **4/4** | 0.65 m |
| 10 | 2/2 | 1/2 | **3/4** | 1.29 m |
| 15 | 0/2 | 2/2 | **2/4** | 1.64 m |

**Gerileme gerçek ve monoton.** C kampanyasının düz verisiyle birleşince
(jerk 5: 4/4, jerk 15: 0/4) → **jerk 5 = 8/8, jerk 15 = 2/8.**

**Gerilemenin mekanizması — temasın son 2 saniyesi** (tepe kutu her koşuda
104-127 px, yani görsel temas tam; bu ölçüt kutu oranı yanlılığından
ETKİLENMEZ):

| seviye | nişan sapması \|cx−320\| | yanal ivme |
|---|---|---|
| 5  | **7 px** | 0.44 m/s² |
| 10 | **8 px** | 1.19 m/s² |
| 15 | **30 px** | 4.49 m/s² |

**Jerk 10 sakin terminali KORUYOR (8 ≈ 7 px); jerk 15 KORUMUYOR (30 px).**
Bu, düzdeki isabet kaybının doğrudan sebebidir ve 10 ile 15'i, kutu
yanlılığından bağımsız bir ölçütle ayıran tek sayıdır.

*Video teyidi (§2 adım 4-6):* D19 (jerk 5, İSABET) f0128 → hedef tam
merkezde, kutu düzgün büyüyor, ufuk düz → f0129 çarpma. **KONTROLLÜ vuruş.**
D21 (jerk 15, IŞKA 0.54 m) f0216 hedef merkezde ufuk 25° yatık → f0217
hedef kadrajın **alt kenarına** kayıyor, ufuk 45° → f0218 hedef merkeze
dönüyor ama araç yanından geçiyor. Son saniyedeki sert yatış nişanı
hedeften kaydırdı. Log (30 px, 4.49 m/s²) ile kare birebir uyuşuyor.

### 7.4 · KARAR KURALININ UYGULANMASI

- **Adım 0:** mekanizma kapısı ✓. Kutu oranı <%60 → **BİRİNCİL-B geçersiz.**
- **Adım 1:** yakınlıkta ayrım VAR (5 vs 15: %17 fark, p=0.057). ✓
- **Adım 2:** en iyi medyan mesafe 54.5 m (jerk 15). %10 bandı ≤ 59.95 m →
  **eşdeğer adaylar: {15 (54.5), 10 (59.5)}**; jerk 5 (66.0) elenir.
  İlan edilen ayırıcı ölçüt (BİRİNCİL-B) Adım 0'da geçersiz oldu →
  **tek aday kalmadı.**
- **Adım 3:** → **KULLANICIYA.** Ölçütle oynanmaz (§5.6).
- **Adım 4 (regresyon):** hiçbir seviye "0/2 iken 5 ≥1/2" kapısına
  takılmadı; ama gerileme monoton ve **gizlenmiyor**: düzde 4/4 → 3/4 → 2/4.

**JERK 5 ELENDİ** (karede medyan mesafe %17 daha kötü, kutu oranı en düşük,
en yakın 6.5 m). **10 ile 15 arası karar kullanıcınındır.**

**Yapay zekânın önerisi: `PSC_JERK_XY = 10`.** Gerekçe:
1. Karede birincilin (medyan mesafe) %10'u içinde — kullanıcının beğendiği
   "geride kalmıyor" kazanımının pratikte tamamını tutuyor (60 m içinde
   süre 96 → 122 s, jerk 15'in 134 s'ine karşı).
2. Düzde terminal nişan sapması 8 px — jerk 5'in 7 px'i ile aynı; jerk 15
   30 px'e çıkıyor. İsabet 3/4 vs 2/4.
3. Yanlılıktan bağımsız salınım ölçütünde (ψ̇) 10 ile 15 **ayrılmıyor**
   (p=0.571) — yani 15'i seçmek sakinlik kazandırmıyor, sadece düzdeki
   isabeti daha çok yiyor.

Varsayılan `sim/ardupilot_params/avci_copter.parm`'a `PSC_JERK_XY 10`
olarak yazıldı. Panel düğmesi **10 ↔ 15** olarak bırakıldı ki kullanıcı
açık kalan tek soruyu kendi uçuşunda sınayabilsin.

### 7.5 · RAPORDAN ÖNCE ÜÇ SORU (§5.8)

1. **Özellik çalıştı mı?** Evet — 24/24 geri okuma, jerk p90 monoton
   6.73 / 10.49 / 13.33.
2. **Ölçütüm kötü bir sebeple mi iyileşti?** BİRİNCİL-B için **EVET** —
   ilan edilen kutu-oranı eşi kapıyı kapattı, ölçüt karardan çıkarıldı ve
   yerine kutudan bağımsız (`telem.csv` 10 Hz) ölçüm kondu. BİRİNCİL-A
   (medyan mesafe) için hayır: geçerlilik eşi (60 m içinde süre) aynı yöne
   gitti (96 → 122 → 134 s).
3. **n kaç?** Kare 4/seviye, düz 4/seviye — §5.4'ün alt sınırı. Hüküm
   kurulabilir; ama "10 mu 15 mi" farkı n=4'te ayrılmadığı için karar
   kullanıcıya bırakıldı, sayı zorlanmadı.

---

## 8 · BU KAMPANYANIN CEVAPLAMADIĞI SORU

§1'de ölçüldü: karede kutulu karelerin %65-80'inde λ̇ > ω_max. Jerk taraması
bu oranı değiştiremez — ω_max = g·tan(ANGLE_MAX)/V, ve jerk bu formülde yok.

Salınımın KÖKÜNE giden iki kanal var, ikisi de bu kampanyanın DIŞINDA:

- **V'yi λ̇ ile kısmak** — `Cfg.DONUS_A` (Ö5) tam bu işi yapıyor ve şu an
  **0 = kapalı**. Ö5 `duz`+kaçamakta ölçülüp elenmişti; §1 tablosu düzde
  tetik oranının yalnız **%10-12** olduğunu gösteriyor — yani Ö5 **kendi
  tasarım zarfının dışında** sınanmış olabilir (§5.13). Karede tetik oranı
  %65-80. Bu, Ö5'i yeniden ölçmek için CLAUDE.md'nin kabul ettiği türden
  bir gerekçedir; ama **bu kampanyanın konusu değildir** (§4 tek değişken).
- **ANGLE_MAX'i açmak** — Ö6'da denenmiş, mekanizma kapısı kapalı çıkmıştı
  (araç 45°'e bile dayanmıyordu). Bugün dayanıyor (yatış p90 36-43°), yani
  o ölçüm ARTIK GEÇERSİZ; yeniden sınanabilir.

Sıra kullanıcınındır: D kampanyası biter, sonra tek değişkenli bir sonraki
adım seçilir.
