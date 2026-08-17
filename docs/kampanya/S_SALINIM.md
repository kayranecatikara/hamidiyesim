# S KAMPANYASI — YATAY SALINIMI ÇÖZ, ÇEVİKLİĞİ KISMA

> **Ölçütler ve karar kuralı KOŞMADAN ÖNCE yazıldı (CLAUDE.md §4).**
> Yazım tarihi 2026-08-16, ilk uçuştan önce.

---

## 0 · KULLANICININ İSTEĞİ — birebir alıntı (§5.5)

> *"biz bu ivme şeysini 15 yaptık ve aracın manevra kabiliyeti arttı ve hedef
> araca doğru daha iyi yönelebiliyoruz, yalnız bu ivme şeysini artırmamız
> salınımın çok olmasını sağladı ve bu salınımın çok olması da takibi komple
> bozuyor. Senden isteğim şu: **bu ivme şeysini bozmadan salınımı azaltmamız
> lazım.** Artık orda bir PID ayarı mı, ne yaparsan yap, şu salınımı kes ve
> aracın ivme şeysini kısma."*

İki şart var ve **ikisi de bağlayıcı**:
1. Yatay salınım azalacak.
2. `PSC_JERK_XY` / ivme bütçesi **kısılmayacak** — çeviklik kazanımı duracak.

Şart 2 bu kampanyada bir **ELEME KAPISI**dır (§6), süsleme değil.

---

## 1 · TEŞHİS — salınımın ölçülen sebebi ivme DEĞİL, KOMUT

D kampanyasının 24 uçuşluk 20 Hz logundan, hız vektörü komutunun
(`hiz_yonu = iris_yaw + K_YAW·eps_hiz − sonum + lead_az`) dönüş hızı ve
onun istediği yanal ivme hesaplandı:

| kare | jerk 5 | **jerk 10** | jerk 15 |
|---|---|---|---|
| komut dönüş hızı medyan | 24.0 | **24.0** | 23.5 °/s |
| komut dönüş hızı p90 | 78.0 | **80.0** | 76.5 °/s |
| istenen yanal ivme medyan | 8.36 | **8.43** | 8.38 m/s² |
| istenen yanal ivme p90 | 26.1 | **26.3** | 26.7 m/s² |
| aracın fiziksel tavanı `g·tan45°` | 9.81 | 9.81 | 9.81 m/s² |
| **komut dönüş tavanını AŞIYOR** | %41.7 | **%43.4** | %44.0 |
| komut `MAX_ACCEL=12`'yi aşıyor | %33.2 | %34.8 | %35.9 |

**Üç jerk seviyesinde BİREBİR AYNI.** Yani komut talebi jerk'ten bağımsızdır;
jerk yalnız aracın o talebi ne kadar takip edebildiğini değiştirir.

**Mekanizma:** aşan her derece **windup**tır. `limit_acceleration` komutu
kırpar; güdüm kırpıldığını BİLMEZ (geri besleme yok); hata birikir; araç
aşırı düzeltir; ters yöne savrulur. Gecikmeli döngüde limit çevriminin ders
kitabı tarifi.

**Gecikme daha önce ölçülmüştü:** kamera→komut 73 ms, komut→gerçekleşen yaw
300 ms, hız vektörünü 30° döndürmek 780 ms. Yatay kanal **saf-P** —
türev terimi yok (`SONUM_T = 0`).

**Literatür aynı yeri gösteriyor:**
- Saf takip N=1 nötr kararlıdır ve **LOS hızını sönümleyen terimi yoktur**;
  gerçek-zamanlı hataya dayalı **PD şeması** yakınsamayı hızlandırıp takip
  gecikmesini azaltır. ([ScienceDirect, Pure Pursuit optimizasyonu](https://www.sciencedirect.com/science/article/pii/S1110016825005113), [Grokipedia · pursuit guidance](https://grokipedia.com/page/pursuit_guidance))
- Look-ahead küçükken yakınsama hızlıdır **ama SALINIR**; büyütmek salınımı
  bastırır, yakınsamayı yavaşlatır — yani bir komut-agresifliği takasıdır.
- Görsel servolamada ölçüm gecikmesi kontrol bant genişliğini doğrudan
  sınırlar; **Smith öngörücüsü** gecikmeyi kapalı döngüden ayırmak için
  kullanılan standart yöntemdir. ([MDPI Sensors 24(17):5546](https://www.mdpi.com/1424-8220/24/17/5546), [ScienceDirect · Smith Predictor](https://www.sciencedirect.com/topics/engineering/smith-predictor))
- Gecikmeli görsel servolamada Kalman tabanlı **durum öngörüsü + kısıt
  farkındalığı** (MPC) yaklaşımı, kontrol/durum kısıtları altında kapalı
  döngü kararlılığını korumak için kullanılıyor. ([arXiv 2605.22443](https://arxiv.org/pdf/2605.22443))

Bizim `lead_az` terimi zaten "LOS'u 0.4 s ileri taşı" biçiminde bir öngörü
yapıyor — ama **öngörünün diğer yarısı eksik:** aracın kendi emredilmiş ama
henüz gerçekleşmemiş dönüşü hesaba katılmıyor. Adaylar tam bu boşluğu
kapatıyor.

---

## 2 · ADAYLAR — üçü de HIZA/İVMEYE DOKUNMUYOR (yapısal kanıtlı)

### S1 · KOMUT DÖNÜŞ HIZI TAVANI (ileri besleme sınırı) — `AVCI_IBVS_SLEW=1.0`

```
ω_max = g·tan(ANGLE_MAX 45°) / v_los           (canlı, hıza bağlı)
|Δhiz_yonu| ≤ SLEW_KAT · ω_max · dt
```
"Yapamayacağın dönüşü emretme." `SLEW_KAT = 1.0` ayarlanmış değil,
**fizikten gelir**. Mekanizma sütunu `slew_kirp_deg`.

⭐ **Birim testi B67:** 200 girdi kombinasyonunda `v_los` farkı **0.00e+00**.
Kullanıcının 2. şartı artık söz değil, **matematiksel garanti**.
B68: küçük hatada hiç kırpmaz — sakin takip aynen korunur.

### S2 · Ö9 SÖNÜMLEME, D TERİMİ — `AVCI_IBVS_SONUM=0.30`

`yaw_cmd −= SONUM_T·ω` (ω = aracın kendi yaw hızı). Saf-P kanala türev ekler.

**Niye yeniden (§5.13):** Ö9 2026-08-11'de 12 uçuşla ölçüldü, aracı
ölçülebilir biçimde SAKİNLEŞTİRDİ (yatış p90 21° → 14°, 6'ya 6 tutarlı) ama
**birincil ölçüt DEĞERLENDİRİLEMEDİ**: salınım yalnız kutulu karelerde
sayılıyordu, görsel temas %60 eşiğini geçen koşu sayısı yetmedi (§5.2).
**O engel artık yok** — `telem.csv` 10 Hz koşulsuz kaynağı D kampanyasında
kuruldu. Ayrıca o ölçüm **jerk 5**'teydi; bugünkü taban **jerk 10**.

### S3 · ANTI-WINDUP GERİ BESLEME (back-calculation) — `AVCI_IBVS_AW=1.0`

```
borç    = hiz_yonu_istenen(t−1) − uygulanan_yon(t−1)
hiz_yonu ← hiz_yonu − AW_K · borç        (tavan 40°)
```
S1 kırpmayı ÖNLER; S3 olan kırpmayı GERİ BESLER. Komut `MAX_ACCEL`'i
karelerin %35'inde aşıyor ve o karelerde gönderilen yön ile istenen yön
ayrışıyor — güdüm bunu görmüyor. Mekanizma sütunu `aw_geri_deg`.
⭐ Birim testi B70: `v_los` farkı 0.00e+00.

### Ortak yapısal garanti — B65

Üç anahtar da KAPALIYKEN komut **bit bit** eskisiyle aynı (90 kombinasyon,
fark 0.00e+00). Kill-switch'ler temiz.

---

## 3 · ÖLÇÜTLER

### BİRİNCİL · **ψ̇ işaret değişimi / s** — rota salınımı

Kaynak **`telem.csv` (10 Hz, KOŞULSUZ yazılır)**, gerçek menzil < 60 m.
Aracın kendi konum izinden rota açısı ψ türetilir; ψ̇'nın işaret değiştirme
sıklığı = savrulma. **Düşük = iyi.**

*Niye bu kaynak:* kutu tabanlı `cx` salınımı **§5.2 tuzağına açıktır** —
hedefi daha çok kaybeden kol daha sakin görünür. D kampanyasında bu tuzak
somut olarak yakalandı ve BİRİNCİL-B geçersiz ilan edildi. `telem.csv` her
karede yazılıyor → seçilim yok. Salınım periyodu 4-5 s, 10 Hz → ~45
örnek/periyot (§5.3 fazlasıyla geçer).

*§5.2 geçerlilik eşi:* salınım, araç hedefi bırakıp düz uçarsa da düşer.
Eşleri: **60 m içinde geçen süre** ve **medyan mesafe**. Takip bozulduysa
sakinlik SAHTEDİR ve karara girmez.

### ⛔ KISIT KAPISI · "aracın ivme şeysini kısma"

**GERÇEKLEŞEN yanal ivme p90** (20 Hz `iris_roll_deg` → `g·tan(roll)`).

> **Bir kol, yanal ivme p90'ı kontrol kolunun %85'inin ALTINA düşürürse,
> salınımda ne kazanırsa kazansın ELENİR.**

Bu kullanıcının 2. şartının ölçülebilir hâlidir ve sonuca bakmadan yazıldı.
*Beklenti:* S1 yalnız İMKÂNSIZ komutları kestiği için gerçekleşen ivme
**belirgin düşmemeli** — çünkü o kareler zaten kırpılıyordu. Düşerse, S1
gerçekten çevikliği kısıyor demektir ve kapıya takılır.

### İKİNCİL

| # | ölçüt | kaynak | niye |
|---|---|---|---|
| İK-1 | 60 m içinde geçen süre | meta 1 Hz | birincilin geçerlilik eşi |
| İK-2 | medyan mesafe | meta 1 Hz | birincilin geçerlilik eşi |
| İK-3 | \|ψ̇\| p90 | telem 10 Hz | salınımın GENLİĞİ |
| İK-4 | görsel fazda kutu oranı | bbox 20 Hz | "takibi komple bozuyor" iddiasının ölçüsü |
| İK-5 | `cx` işaret değişimi/s + \|cx\| p90 | bbox 20 Hz | ⚠ kutu-yanlı; **yalnız kutu oranı eşitse** yorumlanır |
| İK-6 | en yakın menzil (20 Hz kutudan) | bbox 20 Hz | §5.3'e uygun yakınlık |
| İK-7 | isabet | olay.json | karede taban 0/12; ayırması beklenmiyor |
| İK-8 | KURTARMA sayısı | bbox `durum` | görsel temas kesintisi |
| İK-9 | komut dönüş hızı p90 (yeniden kurulmuş) | bbox 20 Hz | teşhisin doğrudan takibi |

---

## 4 · MEKANİZMA KAPISI (§5.1)

Bir koşu VERİ NOKTASI sayılmaz, şu ikisi birden olmadıkça:

| kol | mekanizma sütunu | şart |
|---|---|---|
| S1 | `slew_kirp_deg` | sıfırdan farklı kare oranı **> %20** |
| S2 | `sonum_deg` | sıfırdan farklı kare oranı **> %20** |
| S3 | `aw_geri_deg` | sıfırdan farklı kare oranı **> %20** |
| K | üçü birden | **tam 0** |

Ayrıca `kosuS.sh` her koşuda üç anahtarı da sunucudan **geri okuyup teyit
eder**; edemezse `exit 4` — koşu uçurulmaz.

---

## 5 · ETKİ ALANI TABLOSU (§5.10)

| etkilenebilecek davranış | neden etkilenebilir | hangi senaryoda sınanır |
|---|---|---|
| **düz, sakin kuyruk takibinde isabet** | temasın son saniyesinde LOS hızlı döner; S1 tam orada kırpabilir ve nişanı geciktirebilir | **`duz` + `yatay`/`capraz`, n=4/kol** |
| **hızlı hedef manevrasına tepki gecikmesi** | S1 komut dönüşünü sınırlar → gerçek bir keskin dönüşe geç kalınabilir | **kare (kazanım) + `duz`+kaçamak** — birincil ve İK-1 doğrudan ölçüyor |
| **dikey kanal** | — | **YAPISAL:** S1/S3 yalnız yatay komut açısına dokunur; `vz` ayrı hesaplanır. B65 (kapalıyken bit bit) + B67/B70 (`v_los` değişmez) bunu kanıtlıyor. **Uçuş gerekmez.** |
| **hız / ivme / jerk bütçesi** | — | **YAPISAL: B67 ve B70**, `v_los` farkı 0.00e+00. Ayrıca KISIT KAPISI gerçekleşen ivmeyi uçuşta da ölçüyor. |
| **sürekli dönüşte (daire)** | dairede λ̇ büyük ve sabit → S1 kalıcı kırpabilir, araç dönüşe geç kalabilir | **`circle`, n=2/kol — karar vermez, model çürütür** |
| **görsel temas kesintisi** | sakinleşen araç kamerayı daha az savurur → BEKLENTİ İYİLEŞME | kare + düz (İK-4, İK-8) |

**"Hedeflenen yeri iyileştirdi ama başka bir yeri bozdu mu?"** — düz ve
daire koşulmadan cevaplanmaz.

---

## 6 · KOŞU PLANI

**FAZ 1 — ELEME (8 uçuş, kare):** n=2/kol. **ARA VERİ, karar değil (§5.4).**
Amaç: mekanizma kapısı + KISIT KAPISI. Kapıdan geçemeyen kol elenir.

```
S01_K   S02_S1   S03_S2   S04_S3
S05_K   S06_S1   S07_S2   S08_S3
```

**FAZ 2 — ANA (+8 uçuş, kare):** Faz 1 koşuları AYNI koşullarda uçtuğu için
veri noktası olarak SAYILIR; her kalan kola 2 tur daha eklenir → **n=4/kol**.

```
S09_K   S10_<kalan>  S11_<kalan>  S12_<kalan>
S13_K   S14_<kalan>  S15_<kalan>  S16_<kalan>
```

**FAZ 3 — REGRESYON (12 uçuş):** kazanan kol vs kontrol.
```
düz  : S17_K_yatay  S18_W_yatay  S19_K_capraz S20_W_capraz
       S21_K_yatay  S22_W_yatay  S23_K_capraz S24_W_capraz   (n=4/kol, tür-eşli)
daire: S25_K  S26_W  S27_K  S28_W                            (n=2/kol)
```

⚠ **`PSC_JERK_XY = 10` HER KOLDA AYNI** — tek değişken kuralı (§4) ve
kullanıcının 2. şartı.

---

## 7 · KARAR KURALI — sonuca bakmadan ilan edildi

**Adım 0 — KAPILAR.** Mekanizma kapısından (§4) geçmeyen koşu atılır.
**KISIT KAPISI**: yanal ivme p90 < kontrolün %85'i olan kol **ELENİR**
(kullanıcı şartı 2), salınımdaki kazanımına bakılmaksızın.

**Adım 1 — KAZANIR** bir kol, ÜÇÜ birden sağlanırsa:
1. BİRİNCİL (ψ̇ işaret değişimi/s) kontrolden **düşük**, ve
2. İK-1 (60 m içinde süre) kontrolden **belirgin düşük DEĞİL** (>%85'i), ve
3. İK-4 (kutu oranı) kontrolden **belirgin düşük DEĞİL** (>%85'i).

**Adım 2 — ELENİR** bir kol, şunlardan biri olursa: birincil **artarsa**,
veya İK-1/İK-4 %85 eşiğinin altına düşerse, veya mekanizma/kısıt kapısına
takılırsa.

**Adım 3 — BİRDEN FAZLA KOL KAZANIRSA:** birincili en düşük olan seçilir;
aradaki fark %15'in altındaysa **BİRLEŞİK KOL** (ör. S1+S3) ek olarak
ölçülür — ama yalnız süre kalırsa.

**Adım 4 — REGRESYON (§5.10):** kazanan kol düzde isabeti kontrolün altına
düşürürse bu bir GERİLEMEDİR, gizlenmez; ölçüsüyle raporlanır ve **kararı
kullanıcı verir.**

**Hiçbir kol kazanmazsa** bu açıkça yazılır ve "salınım bu üç kanaldan
çözülmedi" denir — sonuç uydurulmaz (§5.6).

---

## 8 · DALGA 3 — YAKIN MENZİL + DOZ-TEPKİ (koşmadan ÖNCE ilan edildi)

**Niye üçüncü dalga:** Dalga 1 (S1/S2/S3, güdüm komutu) ve Dalga 2 (W1/W2,
ArduPilot hız denetleyicisi PID'i) birincilde −%9…−%11 aralığında kaldı ve
**hepsinde p > 0.30** — kontrol kolunun kendi saçılması (0.224-0.285) kol
farkını yutuyor. Sonuca bakıldığında sebebi ortaya çıktı ve bu bir
**ÖLÇÜM TASARIMI HATASIDIR, sonuç değil:**

`telem.csv`'den menzil kovası kova ölçülen ψ̇ (post-hoc teşhis):

| kova | K | S1 | S2 | W1 | W2 | koşu başına süre |
|---|---|---|---|---|---|---|
| **0-15 m** | **0.427** | 0.389 | **0.241** | 0.343 | 0.311 | **8 s** |
| 15-30 m | 0.148 | 0.272 | 0.195 | 0.128 | 0.237 | 15-25 s |
| 30-60 m | 0.257 | 0.221 | 0.241 | 0.302 | 0.227 | ~90 s |
| 60+ m | 0.428 | 0.357 | 0.334 | 0.414 | 0.321 | ~95 s |

Kullanıcının şikâyet ettiği salınım **yakın menzil** olayıdır (kendi uçuşu:
%76 TERMINAL, 7-16 m, 47 s limit çevrimi). `square` koşuları o bantta 240
saniyenin yalnız **8'ini** geçiriyor — birincil ölçüt o 8 saniyeyi 200
saniyelik uzak menzil içinde eritti. §5.13'ün tarif ettiği hatanın ölçüt
tarafındaki karşılığı.

**Senaryo seçimi ölçüldü** (<15 m'de geçen medyan süre): `square` 6-7 s,
`circle` 5 s, **`duz`+kaçamak 13-23 s**. Dalga 3 `duz`+kaçamakta koşulur —
ayrıca orada **isabet** ölçütü de çalışıyor.

### Dalga 3 · TEK DEĞİŞKEN ve DOZ-TEPKİ

```
K    : SONUM_T = 0      (kontrol)
S2-A : SONUM_T = 0.30   (Dalga 1'de denenen değer)
S2-B : SONUM_T = 0.60
```
0.60'ın gerekçesi ÖLÇÜM: döngü limit çevrimi periyodu 4-5 s → ω ≈ 1.4 rad/s;
integratör + ölü zaman modelinde 180° faz için τ ≈ π/(2ω) ≈ **1.1 s**.
Ölçülen ölü zamanlar (kamera→komut 73 ms, komut→yaw 300 ms, hız vektörü
30° dönüşü 780 ms) bu mertebeyi doğruluyor. 0.30 s bu ölü zamanın dörtte
biri — **muhtemelen küçük kalmıştı.**

### Dalga 3 ÖLÇÜTLERİ

**BİRİNCİL:** ψ̇ işaret değişimi/s, **gerçek menzil < 30 m**
(`telem.csv` 10 Hz, koşulsuz). Bu bantta koşu başına 39-49 s veri var.
**Düşük = iyi.**

**KISIT KAPISI (değişmedi):** gerçekleşen yanal ivme p90 < kontrolün %85'i
→ kol ELENİR (kullanıcı şartı).

**Geçerlilik eşleri:** isabet (tür-eşli), kutu oranı, en yakın menzil.
Sakinlik "hedefi bıraktığı için" gelmişse karara girmez.

**DOZ-TEPKİ ŞARTI:** 0.60, 0.30'dan daha çok sönümlemeli. Monoton
değilse nedensellik iddiası ZAYIFLAR ve rapor bunu açıkça yazar.

### Dalga 3 KOŞU PLANI (§5.9 tür-eşli)
```
SD01_K_yatay   SD02_A_yatay   SD03_B_yatay
SD04_K_capraz  SD05_A_capraz  SD06_B_capraz
SD07_K_yatay   SD08_A_yatay   SD09_B_yatay
SD10_K_capraz  SD11_A_capraz  SD12_B_capraz
```
n=4/kol (2 yatay + 2 çapraz). `PSC_JERK_XY=10`, `PSC_VELXY_*` varsayılan.

---

## 9 · SONUÇLAR — 40 uçuş, 3 dalga, 5 aday, 2026-08-16/17

### 9.0 · TEK CÜMLEYLE

**Salınım bu beş kanalın hiçbiriyle çözülmedi.** İki aday kullanıcının kendi
şartına takılıp elendi; üçü hiçbir şeyi bozmadı ama kazanımları da
istatistiksel olarak ayrışmadı. Uydurulmuş bir kazanan yok (§5.6).

### 9.1 · DALGA 1 — güdüm komutu (kare, n=4/kol + kontrol n=5)

| kol | ψ̇ (birincil) | **KISIT ivme p90** | 60 m süre | kutu% | sonuç |
|---|---|---|---|---|---|
| K | 0.265 | %100 | 127 s | 32.0 | — |
| **S1** slew tavanı | 0.243 (p=0.484) | **%81** | **81 s (p=0.024)** | 47.4 | ⛔ **ELENDİ** |
| **S2** D terimi | 0.242 (p=0.524) | %104 ✓ | 121 s ✓ | 39.1 | kaldı |
| **S3** anti-windup | 0.329 (KÖTÜ) | **%81** | 92 s | 31.2 | ⛔ **ELENDİ** |

**S1 en güçlü teorik gerekçeye sahip adaydı ve kullanıcının kendi şartını
ihlal etti:** dört koşusunun dördü de kontrolün altında yanal ivme üretti
(tam ayrışma, p=0.024) ve 60 m içindeki süreyi %36 düşürdü. Salınımdaki
kazanımı ise gürültü. Teşhis doğruydu (komut karelerin %43'ünde imkânsız
dönüş istiyor) ama **çare işe yaramadı** — araç o aşırı komutu görünen o ki
üretken biçimde kullanıyor: doyma, ulaşılabilir en hızlı tepkinin kendisi.

### 9.2 · DALGA 2 — ArduPilot hız denetleyicisi PID'i (kare)

| kol | ψ̇ | KISIT ivme | 60 m süre | sonuç |
|---|---|---|---|---|
| K | 0.265 | %100 | 127 s | — |
| **W1** `PSC_VELXY_D` 0.5→1.2 | 0.278 (p=0.563) | %109 | 123 s | fayda YOK |
| **W2** `PSC_VELXY_P` 2.0→1.2 | 0.237 (p=0.304) | %97 ✓ | 127 s | fayda gürültüde |

### 9.3 · ⚠ ÖLÇÜM TASARIMI HATAM — dalga 3'ün sebebi

Dalga 1-2'nin tamamı −%9…−%11 ve p>0.30'da kaldı. Sebebi sonradan ortaya
çıktı ve **benim ölçüt tasarımımdaki hatadır**: birincili "60 m içi" diye
tanımlamıştım; salınım ise menzile göre üç kat değişiyor ve kullanıcının
şikâyet ettiği bant koşunun yalnız **8 saniyesi**. O 8 saniye, 200 saniyelik
uzak menzil içinde eridi. (Kova tablosu §8'de.)

Ayrıca ölçüt seçiminde ikinci bir kusur: `|roll| p90` ile `ivme p90`
**aynı büyüklüktür** (`a = g·tan roll`). "Yatış genliğini düşür" ile
"ivmeyi kıs" ayırt edilemez; salınım genlik değil **işaret değiştirme**
demektir. Birincil (ψ̇ işaret değişimi) bunu doğru ölçüyor, KISIT kapısı ise
genliği — ikisi ayrı tutulmalı, karıştırılmamalı.

### 9.4 · DALGA 3 — doğru rejim: `duz`+kaçamak, yakın menzil (n=8/kol)

Birincil koşmadan önce yeniden ilan edildi: **ψ̇ işaret değişimi/s,
gerçek menzil < 30 m** (koşu başına 29-43 s veri).

| ölçüt | K (n=8) | **A · SONUM=0.30 (n=8)** | B · 0.60 (n=4) |
|---|---|---|---|
| **BİRİNCİL ψ̇ <30 m** | 1.450 | **1.174 (−%19)** p=0.308 | 1.401 (−%3) p=0.794 |
| **KISIT ivme p90** | %100 | **%102** ✓ p=0.646 | %95 ✓ |
| kutu oranı | 60.6% | **69.9% (+%15)** p=0.309 | 56.4% |
| **isabet** (tür-eşli) | **4/8** (yatay 2/4, çapraz 2/4) | **7/8** (yatay 3/4, çapraz 4/4) | 3/4 |
| Fisher (isabet, K vs A) | — | **p=0.282** | — |
| mekanizma `sonum_deg` | %0 | %99.1 | %95.2 |

**⛔ İLAN EDİLEN DOZ-TEPKİ ŞARTI DÜŞTÜ.** 0.60, 0.30'dan daha AZ sönümledi
(1.401 vs 1.174). Monoton olmadığı için **nedensellik iddiası zayıftır** ve
"S2 salınımı azaltıyor" cümlesi bu veriyle kurulamaz.

**⚠ n ARTIRMA KARARININ ŞEFFAFLIĞI:** n=4'te isabetin 2/4 → 4/4 olduğunu
GÖRDÜKTEN SONRA n'i 8'e çıkardım — yani özelliğin LEHİNE yönde. Bu, kanıtı
zayıflatan bir karardır ve saklanmıyor. Durma kuralı önceden sabitlendi
(kol başına tam 4 uçuş daha, bakıp uzatma yok) ve ona uyuldu.

### 9.5 · VİDEO BACAĞI (§2 adım 4-6)

`logs/s_SD13_K_yatay.mp4` (K, IŞKA 1.57 m) ve `logs/s_SD14_A_yatay.mp4`
(A, İSABET 1.35 m) kare kare incelendi. Temas öncesi diziler **birbirine
çok benziyor**: ikisinde de hedef merkezde, ikisinde de son karede ~30-35°
yatış. **Video bu kampanyada iki kolu gözle AYIRMADI.** Logla çelişki yok
(§2 adım 6 ✓) ama video ayırt edici kanıt üretmedi — bu da raporlanır.

### 9.6 · RAPORDAN ÖNCE ÜÇ SORU (§5.8)

1. **Özellik çalıştı mı?** Evet, beşi de: S1 %79.5, S2 %96-99, S3 %89.9
   mekanizma oranı; W1/W2 araçtan geri okundu. Kapı hiçbir kolda kapalı değil.
2. **Ölçütüm kötü bir sebeple mi iyileşti?** S2 için hayır — kutu oranı
   ARTIYOR (%60.6 → %69.9), yani "hedefi kaybettiği için sakin görünüyor"
   açıklaması elenir. S1 için EVET yönünde bir sinyal vardı (kutu oranı
   %47.4'e çıkarken 60 m içi süre %36 düşmüştü) ve zaten kısıt kapısında
   elendi.
3. **n kaç?** Dalga 1-2: 3-5/kol. Dalga 3: 8/kol (K, A), 4 (B).
   Hüküm kurmaya yeter; **ama etki büyüklüğü n=8'de bile ayrışmıyor.**

### 9.7 · KARAR

İlan edilen kural (§7 Adım 1) S2/A için üç şartı da sağlıyor: birincil
düşük, İK-1 düşmemiş, İK-4 yükselmiş. Ama **hiçbiri istatistiksel olarak
ayrışmıyor ve doz-tepki şartı düşmüş durumda.** Bu yüzden:

> **KARAR: KULLANICIYA.** `SONUM_T = 0.30` varsayılan YAPILMADI, kapalı
> bırakıldı. Panelde düğmesi duruyor.

Lehine olan: dört bağımsız gösterge (salınım −%19, kutu +%15, isabet
4/8→7/8, ve 2026-08-11 Ö9 kampanyasında yatış p90 21°→14°) aynı yöne
bakıyor ve **hiçbir maliyeti ölçülmedi** (ivme %102, yani kullanıcının
şartını ihlal etmiyor).
Aleyhine olan: hiçbiri tek başına anlamlı değil, doz-tepki tutmadı, n
artırma kararı lehte alındı.

### 9.8 · SIRADAKİ ADIM İÇİN KALICI BULGU

Salınımın kaynağı ölçüldü ve **kapatılamadı**: komut, aracın dönüş tavanını
karelerin %43'ünde aşıyor; bunu ileri beslemeyle sınırlamak (S1) ve geri
beslemeyle telafi etmek (S3) **ikisi de aracı yavaşlattı**. Yani aşırı komut
zararlı bir artık değil, doygun bir sistemin en hızlı tepkisi.

Geriye kalan tek yapısal kaldıraç, D ve Ö5 kampanyalarının da işaret ettiği
yer: **ω_max = g·tan(ANGLE_MAX)/V**. Salınımı komut tarafından değil ARAÇ
tarafından çözmek gerekiyor — `ANGLE_MAX`'i açmak (Ö6 ölçümü artık geçersiz:
o zaman araç 45°'e dayanmıyordu, bugün 36-43° yatışla dayanıyor) tavanı
büyüten tek kanaldır ve henüz geçerli biçimde sınanmamıştır.
