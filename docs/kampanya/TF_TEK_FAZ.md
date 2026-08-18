# KAMPANYA TF — TEK FAZ (terminal fazı kaldırıldı)

**Tek değişken:** `AVCI_IBVS_TEK_FAZ` (konsol: `TEK_FAZ`) · 0 / 1
**8 uçuş**, `duz` senaryosu, n=4/kol (her kolda 3 `yatay` + 1 `yok`, §5.9 eşit),
dönüşümlü · 2026-08-18

> **Kullanıcı fikri:** *"terminal fazı diye bir şey neden var ki? Sistem iki
> fazdan oluşsa: GPS güdüm fazı ve görsel güdüm fazı. GPS'in amacı drone'u
> hedefin belli bir metre uzağında istasyona getirmek zaten. Görsel güdümün
> amacı da her saniye hedefle arasındaki göreli mesafeyi kapatıp aynı zamanda
> hedefi kadrajda ortalamak olmalı. Mesafeyi kapata kapata en sonunda çarpar
> zaten. Sonra terminalde yok hız limiti farklı yok o farklı diye çok sıkıntı
> çektik."*

---

## 1 · TEŞHİS — terminal fazı ne yapıyordu

Mandal atıldığı anda **dokuz şey birden** değişiyordu: hız yasası, dikey yasa,
dikey tavan, dikey sönümleme, dikey lead, dikey roll telafisi, yatay lead,
kaçış telafisi, dikey bütçe kısıtı. Hepsi hedefe 8.9 m kala, tek karede.

**Asıl bulgu:** terminal bir güdüm gereği değil, **iki tane "uzakta dur"
ayarını iptal eden yama**:

| ayar | değer | anlamı |
|---|---|---|
| `BOYUT_REF` | 25 px | PI dengesi 160/25 = **6.4 m'de PARK** |
| `CY_NISAN` | ≈301 px | ufkun **5° yukarısı** = R·sin(5°) kadar altta dur |

Terminal bu ikisini ezmek için vardı. Ayarları düzeltince anahtara gerek
kalmıyor — kullanıcının dediği tam olarak bu.

## 2 · TEK FAZ NE YAPIYOR

| | terminal mandalı | hız | dikey |
|---|---|---|---|
| **taban** | 8.9 m'de atılır | seyir PI → 6.4 m'de park; terminal 16 m/s sabit | seyir: cy=301'de TUT · terminal: tan tabanlı KESİŞİM |
| **tek faz** | **hiç atılmaz** | denge kutusu = TEMAS kutusu (160 px ≈ 1 m) → hep kapat, `V_TEK`=20'de otur | **yatayın aynı matematiği**: `elev_cmd = TEK_K_ELEV·elev_los`, \|v\| korunur |

Ek: **"kaçıracaksan yavaşla"** (`TEK_YAVASLA`, kullanıcının kendi fikri).
Vektörün eğilebileceği en dik açı `asin(VZ_MAX/V_TEK)` = 23.6°; hedef daha
dikse yatay kısılır ki vektör hedefi gösterebilsin (B106: 30°'de 23.6° yerine
30.0°, 45°'de 41.8°).

**Roll telafisi tek fazda HER KAREDE var** (`los_seviye`) — tabandaki
"seyirde açık / terminalde kapalı" tutarsızlığı ortadan kalkar.

## 3 · MEKANİZMA KAPISI (§5.1) — GEÇTİ

| koşu | durum dağılımı | güdüm logu |
|---|---|---|
| TF01_K | IBVS 1080 · **TERMINAL 163** · **TERM_KOR 581** · KUTU_YOK 496 | **22** |
| TF02_T | **TEK_FAZ 689** · KUTU_YOK 395 · KURTARMA 16 | **4** |

Tek faz kolunun 4 koşusunda **tek bir TERMINAL karesi yok**.

**Yapısal garanti (§5.10), ölçümden güçlü:** birim testi **B98** — tek faz
açıkken 14 terminal ayarı saçma değerlere çekildi (`V_TERMINAL`=99,
`VZ_MAX_TERM`=88, `K_VZ_D`=55, `TERM_*` hepsi açık…) → 288 girdi
kombinasyonunda `komut()` farkı **0.00e+00**. Terminal dalı **ölü kod**.

## 4 · SONUÇ — n=4/kol, permütasyon testi

| ölçüt | KONTROL | TEK FAZ | p |
|---|---|---|---|
| **\|dikey\| ıska (BİRİNCİL)** | 1.77 m | **0.66 m** | 0.114 |
| koşunun en yakını | 1.71 m | **0.71 m** | 0.200 |
| **güdüm logu (faz düşüşü)** | 11.5 | **3.5** | **0.086** |
| **kör hücum karesi** | 376 | **0** | 0.143 |
| **\|pitch\| p90** | 38.4° | **20.8°** | **0.086** |
| **son 3 s \|yatış\| p90** | 32.9° | **6.65°** | **0.057** |
| son 3 s \|vz\| p90 | 2.25 | 1.41 | 1.000 |
| vz işaret / s | 0.53 | **2.01** | 0.057 ✗ |
| **İSABET** | **2/4** | **4/4** | |

Koşu en yakınları: K = [0.62, 2.42, 1.98, 1.44] · T = [**1.09, 1.71, 0.16, 0.32**]

**3 m'nin içindeki yaklaşmalar** (D2'de bozulan yerdi — burada bozulmadı):

| | n | \|dikey\| | \|yanal\| |
|---|---|---|---|
| KONTROL | 11 | 0.58 m | 0.42 m |
| TEK FAZ | 7 | **0.27 m** | **0.12 m** |

## 5 · KARELERDEN — 480'er kare, temas anları tek tek incelendi

### KONTROL (TF03_K) — kullanıcının tarif ettiği hata

| kare | menzil | dikey | görüntü | hız |
|---|---|---|---|---|
| #455 | 5.12 m | +0.26 | hedef kadrajda, kutu net | 17.63 |
| **#456** | **4.33 m** | **+1.23** | **KADRAJ BOŞ** | 16.72 |
| #457 | 3.99 m | +1.81 | boş | 16.15 |
| #458 | 3.80 m | **+1.99** | boş | 15.67 |
| #460 | 3.23 m | +0.93 | hedef alttan tekrar beliriyor | 15.61 |

> Hız **18.93 → 15.5** (fren), dikey **+0.26 → +1.99 m** (üstüne çıkma),
> hedef **4 kare (2 s) boyunca kadraj dışı**. Kullanıcının cümlesi birebir:
> *"tam çarpacakken birden hedef aracın üstüne çıkıp üstünden geçiyor."*

### TEK FAZ (TF08_T) — düz hat kesişimi

| kare | menzil | dikey | hız |
|---|---|---|---|
| #148 | 16.26 m | −0.26 | 20.24 |
| #150 | 12.66 m | −0.30 | 20.12 |
| #152 | 7.46 m | −0.33 | 19.90 |
| #154 | 3.92 m | −0.37 | 19.76 |
| **#155** | **0.32 m** | **−0.27** | 18.64 — **TEMAS** |

> Dikey **16 metreden temasa kadar −0.26…−0.37 arasında sabit**. Hız hiç
> düşmüyor. Hedef **her karede** kadrajda. TF06_T'de de aynı desen
> (dz ±0.4 içinde, 18.95 → 20.4 → 19.41).

## 6 · ⛔ GERİLEME — saklanmıyor (§5.10)

**Tek faz hedefe yakın DAHA AZ zaman geçiriyor:**

| | KONTROL | TEK FAZ |
|---|---|---|
| medyan menzil | 69.7 m | 72.7 m |
| **< 10 m'de geçen süre** | **%17** | **%1** |
| < 25 m'de | %31 | %5 |
| maks menzil | 507 m | **227 m** |

**Yaklaşma başına dönüşüm:**

| | yaklaşma | dip medyanı | 1 m altına inen |
|---|---|---|---|
| KONTROL | **27** | 3.60 m | 1/27 = **%4** |
| TEK FAZ | **13** | **2.71 m** | 2/13 = **%15** |

**Yorum:** kontrol iki katı yaklaşma yapıyor ama dönüştüremiyor — %17'lik
"10 m altı" süresinin çoğu hedefin yanında asılı kalmak (ölçülmüştü: terminal
bacağı 26.7 s). Tek faz vurup geçiyor ve geri dönmesi zaman alıyor.
**Ama maksimum menzil 507 → 227 m**: kontrol kolu bazı koşularda hedefi
büsbütün kaybediyor (815 ve 2067 m'ye çıktı), tek fazda öyle bir kaçış yok.

⚠ TF04_T'de üç yaklaşma 12.9-14.9 m'de takıldı — tek faz her zaman içeri
giremiyor. Bakılması gereken bir davranış.

## 7 · SALINIM (§4 kuralı) — genlik iyi, işaret oranı yüksek

| ölçüt | KONTROL | TEK FAZ | |
|---|---|---|---|
| \|pitch\| p90 | 38.4° | **20.8°** | araç daha sakin |
| son 3 s \|yatış\| p90 | 32.9° | **6.65°** | araç çok daha sakin |
| son 3 s \|vz\| p90 | 2.25 | 1.41 | genlik aynı/daha iyi (p=1.000) |
| vz işaret / s | 0.53 | 2.01 | **daha çok işaret değişimi** |

İşaret oranı yüksek ama **genlik düşük ve aracın gerçek duruşu çok daha
sakin**. Sebep: tek fazda dikey kanal saf takip kazancıyla (1.0) sürekli
çalışıyor; tabanda seyir yasasının kazancı `K_VZ·V_NOM = 6 m/s/rad` olduğu
için komut kıpırdamıyor. Yani "az işaret değişimi" iyi kontrol değil, **az
kontrol**. Kullanıcının 2026-08-10 kuralının aradığı "dengesizce savrulan
araç" tarifi **kontrol koluna** uyuyor (son 3 s'de 32.9° yatış).

## 8 · KARAR KURALI DENETİMİ

| # | kural | sonuç |
|---|---|---|
| 1 | \|dikey ıska\| iyileşir | ✓ 1.77 → **0.66 m** (p=0.114) |
| 2 | en yakın menzil kötüleşmez | ✓ 1.71 → **0.71 m** |
| 3 | görsel temas kötüleşmez | ✓ kadraj dışı %2 → **%0**; kör hücum 376 → **0** kare |
| 4 | salınım genliği artmaz | ✓ pitch p90 38.4 → 20.8°, son 3 s yatış 32.9 → 6.65° |

**Dördü de geçti. İsabet 2/4 → 4/4.**

⚠ n=4/kol. p değerleri 0.057-0.200 arasında — yön tutarlı ama küçük n'de
kesinlik iddia edilmez (§5.4). Gerileme (§6) gerçek ve kullanıcının kararına
sunuluyor.

## 9 · RAPORDAN ÖNCE ÜÇ SORU (§5.8)

1. **Özellik çalıştı mı?** Evet — 4/4 tek faz koşusunda sıfır TERMINAL karesi;
   ayrıca B98 yapısal garantisi terminal ayarlarının okunmadığını kanıtlıyor.
2. **Ölçütüm kötü bir sebeple mi iyileşti?** Geçerlilik eşleri kontrol edildi:
   kadraj dışı %0 ve kör hücum 0 kare olduğu için "hedefi kaybettiği için
   sakin görünme" tuzağı yok. Salınım ölçütü ayrıca adil pencerede
   (son 3 s) ve **aracın gerçek duruşundan** doğrulandı.
3. **n kaç?** 4/kol. Yön tutarlı (isabet 4/4, dikey 4/4 daha iyi), ama
   §5.4 gereği kesin hüküm için 6+/kol gerekir.
