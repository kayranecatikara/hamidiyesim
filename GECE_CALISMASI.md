# Gece çalışması — 2026-08-16/17

Kullanıcı 12 saat için ayrıldı; otonom çalışma. Bu dosya ne yapıldığının
sırayla kaydı. Sonuç bölümü en altta.

---

## SORU

"Dronun hedefi düz uçuşta, manevrada, her türlü uçuşta vurmasını nasıl
sağlarız?"

## YÖNTEM — önce teşhis, sonra deney

Rastgele parametre denemek yerine önce **hangi eksende kaybettiğimizi**
ölçtüm. Üç bağımsız kaynak aynı yeri gösterdi.

### Kanıt 1 — kara kutu, 248 uçuş (12-15 Ağustos)

| geçişin dikeyi | temas oranı |
|---|---|
| aynı seviye (±0.5 m) | **%24.9** |
| drone 2 m'den fazla ALTTA | **%0.4** |

**60 kat fark.** Dikey ıska baskın kayıp ekseni.

### Kanıt 2 — terminal karelerinde dikey hata (n=52 036)

| menzil | `eps_elev` medyanı |
|---|---|
| >8 m | −5.7° |
| 4-8 m | −10.0° |
| 1-2 m | −19.8° |
| en yakın an (medyan 1.51 m) | **−22.9°** |

Negatif = hedef ÜSTÜMÜZDE. Yaklaştıkça kötüleşiyor; temasa **~0.5 m altta**
varıyoruz. İsabet zarfı ise dikeyde yalnız **+0.29 / −0.13 m**.

### Kanıt 3 — manevrada 2.6 kat kötü

| hedef rejimi | `eps_elev` medyanı | `vz_cmd` medyanı |
|---|---|---|
| DÜZ | −3.4° | −0.23 |
| ORTA | −6.2° | −0.18 |
| MANEVRA | **−9.0°** | −0.23 |

⚠ Hata büyürken **komut sabit kalıyor.** Kullanıcının "manevrada da vursun"
isteği doğrudan buraya bağlanıyor.

## SUÇLU KİM DEĞİL (elenenler)

| şüpheli | ölçüm | sonuç |
|---|---|---|
| Araç komutu uygulamıyor | kara kutu `PSCD.DVD` vs `VD`: hata medyan **0.00 m/s**, p90 0.11 | **temiz** |
| Dikey hız tavanı yetmiyor | `VZ_MAX_TERM=5` yalnız **%0.6** doyuyor | **temiz** |
| Araç dikey ivme sınırı | `WPNAV_ACCEL_Z` zaten 250 (2.5 m/s²), takip hatasız | **temiz** |
| Yatay/dönüş kısıtı | `hiz_yonu` aracın gerçek yaw'ından, slew'den geçmiyor | **temiz** |

## SUÇLU — komutun kendisi

```python
v_dikey = clamp(kapanma, KAPANMA_MIN=1.5, v_los)
vz      = −v_dikey · tan(nişan_elev)
```

Dikey düzeltme hızı **kapanma hızıyla** ölçekleniyor. Kapanma durunca
(takılma) `v_dikey` 1.5 tabanına düşüyor ve 9.5° hata için komut **0.26 m/s**
kalıyor. Hız vektörünü gerçekten hedefe doğrultmak 16·tan(9.5°) = **2.7 m/s**
ister. Terminal karelerinin **%58.3'ünde bu taban bağlıyor.**

İkinci, bağımsız kaynak: tutuş fazının nişan noktası

```python
_CY_SEVIYE = geo.CY + geo.FY·tan(20°)   # = 301 px
```

Kamera +25° yukarı sabit → 20°, ufkun **5° ÜSTÜ** demek. Yani tutuş fazı
hedefi bilerek yukarımızda tutuyor, biz altta kalıyoruz. Kodun kendi yorumu:
*"CY_NISAN'da (≈5° yukarıda) tutmaya çalışıyor, yani ALTINDAN geçiyoruz"*.

## İKİ DENEY KURULDU

| ad | ne değişiyor | kontrol → deney | ölçüt ilanı |
|---|---|---|---|
| **D-V** | dikey düzeltme tabanı | `KAPANMA_MIN` 1.5 → 6.0 | `docs/kampanya/DV_OLCUTLER.md` |
| **D-N** | tutuş nişanı | `CY_NISAN` 301 → 318 (seviye) | `docs/kampanya/DN_OLCUTLER.md` |

İkisi de **panelde düğme**, varsayılan KAPALI, env kill-switch'li.
Ölçütler ve karar kuralı **koşmadan önce** yazıldı (§4).

### Kapsam kanıtı — `tests/test_dikey_kanal.py` 10/10

- **DV1**: D-V tutuş fazına 25 girdi kombinasyonunda **bit bit** dokunmuyor
- **DV2**: kapanma > 6 m/s iken terminalde de etkisiz (gerçek kesişimler korunur)
- **DV3**: yavaş kapanmada dikey komut −0.79 → **−3.15 m/s** (mekanizma çalışıyor)
- **DN1**: D-N terminal kesişimini **değiştirmiyor** (12 kombinasyon)
- **DN2**: tutuşta −0.75 → −1.35 m/s (doğru yönde)
- **DN3**: 318 px'in gerçekten "seviye" olduğu geometriden doğrulandı

---

## KAMPANYA

*(sonuçlar aşağıda, kampanya bitince doldurulacak)*

---

## ⚑⚑ GECENİN EN ÖNEMLİ SAYISI — 1978 geçiş üzerinden

Kara kutudaki TÜM geçişler (tüm arşiv, 250+ uçuş) ıska bandına ayrıldı:

| ıska bandı | n | \|yatay\| medyan | \|dikey\| medyan | **dikeyin toplam ıskadaki payı** |
|---|---|---|---|---|
| **TEMAS (≤0.5 m)** | 165 | 0.10 m | 0.27 m | **%93** |
| 0.5-1.0 m | 126 | 0.29 m | 0.54 m | **%91** |
| 1.0-2.0 m | 125 | 0.68 m | 1.12 m | **%89** |
| 2.0-5.0 m | 284 | 3.77 m | 1.21 m | %26 |

**YATAY KANAL ÇÖZÜLMÜŞ.** Yakın geçişlerde yatay hata medyanı 0.10-0.68 m,
isabet zarfı ise yatayda ±0.65 m — yani yatayda zaten içerideyiz.

**KALAN SORUNUN TAMAMI DİKEY.** Yakın ıskaların (0.5-2 m arası, 251 adet)
**135'inde (%54) yatay hata ZATEN ≤0.5 m** — o geçişlerde tek yanlış olan
dikey.

⇒ **Dikey düzelseydi temas 165 → 300 olurdu (×1.8).**

Bu sayı, gecenin bütün çalışmasının gerekçesi: D-V, D-V2 ve D-N'nin üçü de
dikey ekseni hedefliyor ve doğru yere bakıyoruz.

*(2.0-5.0 m bandında dikeyin payı %26'ya düşüyor — orada henüz yatayda da
uzağız, yani o band "yaklaşma" sorunu; yakın bandlar "nişan" sorunu.)*

## DİKEY ISKA: ÖNYARGI mı SAÇILMA mı — ikisi de var, ÖNYARGI baskın

| ıska bandı | n | medyan dikey | ortalama | std | **ALTTAN** | ÜSTTEN |
|---|---|---|---|---|---|---|
| TEMAS (≤0.5 m) | 165 | +0.25 m | +0.22 | 0.21 | **%66** | %7 |
| **0.5-1.0 m** | 130 | **+0.53 m** | +0.39 | **0.40** | **%81** | %11 |
| 1-2 m | 125 | +0.36 m | +0.20 | 1.14 | %54 | %36 |
| yakın (0.5-2 m) | 255 | +0.52 m | +0.30 | 0.85 | **%74** | — |

⚠ **KENDİ HÜKMÜMÜ DÜZELTİYORUM.** Scriptim "ortalama < 0.5×std ⇒ saçılma
baskın" diye otomatik hüküm bastı. O test KABA ve yanıltıcı: geçişlerin
**%74-81'i tek yönde** (alttan) — bu bir ÖNYARGI imzasıdır. Ortalamanın
std'den küçük görünmesi, 1-2 m bandındaki birkaç büyük aykırı değerden
geliyor (std 1.14, oysa 0.5-1 m bandında 0.40).

En temiz bandda (0.5-1.0 m, dar ve kalabalık) tablo net:
**+0.53 m sistematik alt kayma, ±0.40 m saçılma.**

### Bunun anlamı

İsabet zarfı dikeyde **+0.29 / −0.13 m**. Şu an dağılım +0.53 m'de merkezli
→ zarfın tamamen dışında. Önyargı sıfırlansa dağılım 0'da merkezlenir ve
±0.40 m saçılmayla geçişlerin kayda değer bir kısmı zarfın içine girer.

⇒ **Önce ÖNYARGIYI düzelt (D-N), sonra saçılmayı.** Sıralama bu.

---

## ⚑⚑⚑ NEDENSEL ZİNCİR KAPANDI

Tutuş fazı yasası hedefi `CY_NISAN`'da **tutmaya** çalışır. Denge kurulunca
`vz → 0` — yani drone o ofseti **kapatmaz, KORUR**. `CY_NISAN = 301 px`
kamera +25° tiltiyle **ufkun 5° ÜSTÜ** demek.

5°'nin fiziksel dikey karşılığı menzile göre:

| menzil | dikey ofset |
|---|---|
| 4 m | 0.35 m |
| **6 m** | **0.52 m** |
| 8 m | 0.70 m |
| 10 m | 0.87 m |

**ÖLÇÜLEN önyargı (0.5-1.0 m ıska bandı, n=130): +0.53 m**
**6 m menzilde 5°'nin karşılığı: 0.52 m**

⇒ **Bire bir eşleşiyor.** Dikey önyargı bir kontrol kusuru, gürültü ya da
araç yetersizliği DEĞİL — `CY_NISAN`'ın 5°'lik tasarımının doğrudan
geometrik sonucu.

### Zincirin tamamı

1. `CY_NISAN = CY + FY·tan(20°)`, kamera +25° → **denge noktası ufkun 5° üstü**
2. Tutuş fazı bu dengeye oturur ve **orada kalır** (vz→0)
3. Temas menzilinde (~6 m) bu **+0.52 m dikey ofset** demek
4. İsabet zarfı dikeyde **+0.29 / −0.13 m** → dağılım zarfın DIŞINDA merkezli
5. Terminal son 1-2 saniyede kapatmaya çalışıyor ama kapanma-hızı tabanı
   komutu 0.26 m/s'de tutuyor → yetişemiyor
6. Sonuç: geçişlerin **%74-81'i alttan**, yakın ıskaların **%89-93'ü dikey**

### Zamanlama bütçesi D-N için uygun

Görsel faz en yakın andan önce medyan **6.5 s** sürüyor, %81'i tutuş fazı
→ ~**5.3 s**. D-N'nin komutu 1.2-1.6 m/s → o sürede **6-8 m** tırmanma
kapasitesi. Gereken 0.5-1.0 m. **Vakit bol.**

⇒ D-N sorunu KAYNAĞINDA kesiyor; D-V/D-V2 ise sonucu son saniyede telafi
etmeye çalışıyor. Bu yüzden öncelik D-N.

---

## ELENEN ŞÜPHELİLER — hepsi ölçümle, tahminle değil

| şüpheli | ölçüm | sonuç |
|---|---|---|
| Araç komutu uygulamıyor | kara kutu `PSCD.DVD` vs `VD`, hata medyan **0.00 m/s**, p90 0.11 | temiz |
| Dikey hız tavanı yetmiyor | D-V öncesi yalnız **%0.6** doyum | temiz |
| Araç dikey ivme sınırı | `WPNAV_ACCEL_Z` zaten **250** (2.5 m/s²), takip hatasız | temiz |
| Yatay / dönüş kısıtı | `hiz_yonu` aracın **gerçek** yaw'ından, slew'den geçmiyor | temiz |
| Algı gecikmesi | medyan **38 ms** → 1 m/s bağıl hızda **4 cm** | temiz |
| Kutu merkez gürültüsü | 1 px = 0.34° = 6 m'de **3.6 cm** | temiz |
| Kör hücum fazı | son GÖRÜLEBİLEN karede (1.51 m) zaten **0.59 m altta** — ıska kör faza girmeden belirlenmiş | temiz |

⇒ **Geriye tek suçlu kalıyor:** tutuş fazının nişan noktası (`CY_NISAN`) ve
onun ürettiği +0.52 m'lik denge ofseti. Terminalin bunu son 1-2 saniyede
kapatamaması ikincil (kapanma-hızı tabanı).

---

# NEDEN VURAMIYORUZ — TEK SAYFALIK CEVAP

**Soru:** dronun hedefi düz uçuşta, manevrada, her türlü uçuşta vurmasını
nasıl sağlarız?

**Cevap:** yatay kanal zaten çözülmüş. Kalan sorunun tamamı dikey, dikeyin
kaynağı da tek bir tasarım satırı.

## Kanıt zinciri (hepsi ölçüm)

1. **Yakın ıskaların %89-93'ü dikey** (1978 geçiş).
   Yatay hata medyanı 0.10-0.68 m, yatay zarf ±0.65 m → yatayda içerideyiz.

2. **Yakın ıskaların %54'ünde yatay ZATEN ≤0.5 m.** Tek yanlış dikey.
   Dikey düzelse temas **165 → 300 (×1.8)**.

3. **Dikey ıska tek yönlü:** geçişlerin **%74-81'i alttan** (0.5-1 m
   bandında %81 alttan, %11 üstten). Bu bir önyargı imzası.

4. **Önyargının büyüklüğü +0.53 m** (0.5-1 m bandı, n=130).

5. **Kaynağı:** `CY_NISAN = CY + FY·tan(20°)`. Kamera +25° tiltli olduğu
   için bu, hedefi **ufkun 5° ÜSTÜNDE** tutmak demek. Tutuş fazı bu dengeye
   oturur ve orada **kalır** (vz→0). 6 m menzilde 5° = **0.52 m**.
   Ölçülen 0.53 m. **Bire bir.**

6. **İsabet zarfı dikeyde +0.29 / −0.13 m** → dağılım zarfın DIŞINDA
   merkezli. Iskanın tamamı bu.

7. **Terminal telafi edemiyor:** dikey düzeltme kapanma hızıyla ölçekleniyor,
   kapanma durunca taban 1.5'e düşüp komut 0.26 m/s'de kalıyor. Terminal
   karelerinin %58.3'ünde bu taban bağlıyor.

8. **Manevrada kusur 2.6 kat büyüyor** (−3.4° → −9.0°) ama komut sabit
   kalıyor. Kullanıcının "manevrada da vursun" isteği doğrudan buraya bağlı.

## Çözüm sırası

| # | deney | ne yapar | durum |
|---|---|---|---|
| 1 | **D-N** | nişan noktasını seviyeye alır → önyargı kaynağında biter | kampanya |
| 2 | **D-V2** | dikey tavanı açar → D-V'nin doyumu biter | hazır |
| 3 | **D-V** | terminal dikey tabanını yükseltir → telafi hızlanır | ölçüldü, bölünmüş |
| 4 | Ö-T | terminal mandalını süreyle bırakır (takılma) | hazır |

**D-N önce, çünkü tek o kaynağı kesiyor.** Diğer üçü sonucu telafi ediyor.

---

## ⚑ ÖNCEDEN İLAN EDİLEN TAHMİN (kampanyalar bitmeden yazıldı)

Ölçülen dikey dağılım (+0.53 ± 0.40 m) ve isabet zarfı (−0.13 … +0.29 m)
ile normal yaklaşım:

| senaryo | zarfa düşme | kat |
|---|---|---|
| şimdi | %22.5 | — |
| D-N | %39.3 | 1.7× |
| **D-S tek başına** | **%11.5** | **0.5× (kötüleşir)** |
| D-N + D-S | %66.9 | 3.0× |

**Bu tahmin bir tuzak yakaladı:** D-S'yi D-N kapalıyken sınamak onu haksız
yere elerdi. Kampanya scripti artık D-S'yi D-N onaylanmadan koşmuyor
(`logs/ds_ONAY` kapısı).

Tahmin düşülebilir: kampanya tutmazsa bu da bir bulgudur ve öyle raporlanır.

---

# ⚑⚑ SONUÇLAR — kampanyalar bitti

## D-V (terminal dikey tabanı 1.5→6.0) · 22 koşu · BÖLÜNMÜŞ → kullanıcıya

Mekanizma çalıştı (dikey komut 10-40 kat). Dikey eksende **iki kat kazanım**:
|dikey| ıska 1.07 → 0.56 m (koşu-başı), aynı seviye geçiş %51 → %80,
temas 0 → 2.

**Bedeli:** dikey komut tavana dayandı (karelerin %20-68'i), bütçe kısıtı
yatay hızı 16 → 10 m/s kesti, salınım 1.03 → 2.26/s.

⚠ Birincil ölçüt ilanım **belirsizmiş**: "en yakın menzil medyanı" yazmışım
ama neyin medyanı olduğunu belirtmemişim. Tüm-geçiş medyanı kapalı lehine
(4.53 vs 5.14), koşu-başı en iyi medyanı açık lehine (1.19 vs 1.07). Fark
geçiş sayısından (86 vs 47). İkisini de raporladım, ölçütü sonuca bakarak
seçmedim.

## D-N (tutuş nişanı 301→318 px) · 16 koşu · GİRMEZ

Mekanizma çalıştı (`cy` 288 → 303) ama **hiçbir ölçüt değişmedi**:
dikey ıska 0.10 vs 0.12 m, temas 3 vs 3, aynı seviye %75 vs %76.

### ⚠⚠ VE BU, ANA HİPOTEZİMİ ÇÜRÜTTÜ

"Dikey önyargının kaynağı `CY_NISAN`'dır" diyordum. İki kanıt bunu çürüttü:

1. **Önyargı, hedef sabit uçarken de aynı** (+0.33 m / %66 alttan) tırmanırken
   de (+0.34 m / %67). Hedefin davranışıyla ilgisi yok.
2. **Temas geometrisini TUTUŞ değil TERMİNAL belirliyor.** En yakın ana kadarki
   sürenin 4.2 s'si tutuş, **1.0 s'si terminal** — ama terminal yasası
   `CY_NISAN`'ı KULLANMIYOR, mutlak yükseliş hesaplıyor. Tutuş fazı ofseti
   kuruyor, terminal son saniyede üzerine yazıyor.

Geometrik eşleşme (5° ↔ +0.52 m) doğruydu ama **nedensellik değildi**.
Korelasyonu nedensellik sandım.

### Bu çürütme yolu netleştirdi

| deney | neye dokunuyor | sonuç |
|---|---|---|
| **D-V** | **TERMİNAL** dikey kanalı | **işe yaradı** (ıska yarıya) |
| D-N | tutuş nişanı | hiçbir şey |
| D-S | tutuş sönümlemesi | **aynı sebeple şüpheli** — koşulmadı |

⇒ Dikey ıskayı belirleyen yer **terminal fazı**. Çalışma oraya odaklanmalı.

---

# 2026-08-17 · KULLANICI UÇUŞU → YENİ KAMPANYA

Kullanıcı 2 uçuş yapıp sordu: *"aradaki mesafe hep 6 metre civarında, dron ile
hedef hemen hemen aynı irtifada — aynı irtifadayken 6 metreyi kapatması mümkün
olmadığı için mi takılı kalıyor?"*

**Gözlem doğru çıktı.** 261 saniyelik koşu, 5208 karenin tamamı TERMINAL:

| ölçüm | değer |
|---|---|
| menzil | medyan 8.7 m, **min 6.1 m** (261 s'de altına inmedi) |
| kapanma | **0.00 m/s** |
| dikey hata | **+0.7°** — dikeyde sorun YOK |
| irtifa farkı (kara kutu) | **−0.09 m** |

Kara kutu: menzil 6.16 m, **yatay 6.15 m, dikey −0.39 m** — mesafenin tamamı
yatayda.

**Ama sebep "aynı irtifada olmak" değil — hız marjı:**

| | m/s |
|---|---|
| komut (`v_los`) | 16.00 |
| avcının gerçekleşen hızı | 15.35 |
| hedefin hızı | 15.15 |
| **kapanma kapasitesi** | **+0.20** |

Aynı oturumun kendi kontrol grubu: mandal kurulmayan 9 koşuda hız 21-23 m/s
ve en yakın menzil **1.3-1.5 m**; mandal kilitlenen koşuda 16 m/s ve **6.1 m**.

⇒ **Ö-T kampanyası başlatıldı** (8 tur, 150 s ölçüm penceresi) +
**D-V2 zincirlendi** (4 tur, D-V tabanı iki kolda açık).
Ölçüt ilanı: `docs/kampanya/OT_OLCUTLER.md`.
