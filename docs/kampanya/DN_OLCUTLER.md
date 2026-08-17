# D-N · Tutuş nişanı seviyeye alınıyor — ölçüt ilanı

**Tarih:** 2026-08-16 (koşulmadan ÖNCE yazıldı, §4)
**Kol anahtarı:** `AVCI_IBVS_CY` · panel: "D-N · Tutuş nişanı 5° yukarı → SEVİYE"
**Kontrol:** 301 px (mevcut) · **Deney:** 318 px

---

## Neden bu deney — kök neden tasarımda YAZILI

```python
_CY_SEVIYE = geo.CY + geo.FY * math.tan(math.radians(20.0))
CY_NISAN   = _env_f("AVCI_IBVS_CY", round(_CY_SEVIYE, 0))   # = 301 px
```

Kamera gövdeye **+25° YUKARI** sabit (donanım, değişmez). Buradaki **20°**,
kamera ekseninden 20° aşağı = **ufkun 5° ÜSTÜ** demek. Yani tutuş fazı hedefi
kendi seviyemizin **5° YUKARISINDA** tutuyor — biz bilerek ALTINDA uçuyoruz.

Kodun kendi yorumu da bunu söylüyor (satır 184):
> "CY_NISAN'da (≈5° yukarıda) tutmaya çalışıyor, yani **ALTINDAN geçiyoruz**"

GPS fazı da aynı yönde: `ISTASYON_ELEV_DEG = 15°` → istasyon hedefin
2.85 m ALTINDA. **İki faz da bizi alta koyuyor.**

## Bunun bedeli ölçüldü

248 uçuşluk kara kutu taraması (12-15 Ağustos):

| geçişin dikeyi | temas oranı |
|---|---|
| aynı seviye (±0.5 m) | **%24.9** |
| drone 2 m'den fazla ALTTA | **%0.4** |

169 koşunun EN YAKIN anı (kutu tepe noktası, medyan menzil 1.51 m):

| ölçüm | değer |
|---|---|
| `cy` medyanı | **230 px** (CY_NISAN'ın 70 px ÜSTÜNDE) |
| `eps_elev` medyanı | **−22.9°** |

Menzile göre ima edilen fiziksel dikey ofset:

| menzil | `eps_elev` | ~ofset |
|---|---|---|
| >8 m | −5.7° | ~1.0 m altta |
| 4-8 m | −10.0° | ~1.0 m altta |
| 1-2 m | −19.8° | ~0.5 m altta |

Drone açığı kapatıyor ama **yetiştiremiyor**; temasa ~0.5 m altta varıyor.
İsabet zarfı dikeyde yalnız **+0.29 / −0.13 m** (UYGULANACAK §3). Iska tam burada.

## Ne yapar

`CY_NISAN = CY + FY·tan(25°) = 318 px` → tutuş fazı hedefi **tam kendi
seviyemizde** tutar. Dikey açık en baştan oluşmaz, terminalin kapatacak
bir şeyi kalmaz.

⚠ **Yalnız TUTUŞ fazını değiştirir.** Terminal kesişim yasası `cy`'yi mutlak
yükselişe çevirir (`piksel_elev(cy) + iris_pitch`) ve CY_NISAN'ı KULLANMAZ.

---

## ÖLÇÜTLER

### Birincil
**Kara kutudan gerçek en yakın menzil** — deney kolunun medyanı küçük olmalı.

### İkincil
1. **|dikey| ıska medyanı** (kara kutu) — mekanizmanın doğrudan hedefi
2. **Temas sayısı** (≤0.5 m geçiş)
3. **Aynı seviye geçiş oranı** (|dikey| < 0.5 m olan geçişlerin payı)

### Mekanizma kapısı (§5.1)
Deney kolunda tutuş fazı `eps_elev` medyanı kontrolünkinden **mutlak değerce
küçük** olmalı ve `cy` medyanı ~318'e yaklaşmalı. Değilse özellik iş
görmemiştir; o koşu veri değildir.

### Geçerlilik eşi (§5.2)
| ölçüt | kötü sebeple iyileşir mi | zorunlu eş |
|---|---|---|
| \|dikey\| ıska | **evet** — hiç yaklaşmazsan dikey de küçük görünür | geçiş sayısı + en yakın menzil |
| en yakın menzil | evet — savrulup şans eseri | salınım + vuruş sınıfı |
| görsel temas | evet — hedef kadrajın ALTINA kayarsa kutu kaybolur | **kutusuz kare oranı %40'ı geçerse ölçüt güvenilmez** |

⚠ **KENDİ RİSK ANALİZİMİ DÜZELTİYORUM (işaret hatası).**
Önce "nişan 17 px aşağı kayınca hedef kadrajda YÜKSELİR, üstten çıkabilir"
yazmıştım. **Yanlış.** Tutuş yasası hedefi nişan noktasına SÜRÜKLER; nişan
aşağı kayınca hedef de kadrajda **AŞAĞI** iner.

Ölçülen: yakın menzilde `cy` medyanı **230** (hedef kadrajda YUKARIDA, üst
kenara yakın). Yeni denge **318** → hedef 230'dan 318'e, yani **merkeze ve
aşağı** hareket eder. **Üst kenardan çıkma riski AZALIR.** Alt kenar 480,
318 ondan 162 px uzakta — alt risk de yok.

⇒ D-N kadrajda tutmayı **iyileştirmesi** beklenir. Kutusuz kare oranı yine
de zorunlu eş olarak raporlanır (ölçüm, tahmin değil).

### Salınım
`cx` işaret değişimi/s · `|roll|` p90 · roll işaret değişimi/s · yaw komut p90

### Geçerlilik (§4)
Hedef 20-250 m / 6-25 m/s bandında. Dışına çıkan koşu SAYILMAZ.

---

## ETKİ ALANI TABLOSU (§5.10)

| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Tutuş fazı dikey konum | doğrudan hedef | `duz` — ana ölçüm |
| **Kadrajda tutma** | hedef 17 px yükselir, üstten çıkabilir | `duz` — kutusuz kare oranı |
| Terminal kesişim | **etkilenmez** — CY_NISAN'ı kullanmaz | birim testi |
| Manevrada dikey | dönüşte kadraj kayması büyür | `circle` — REGRESYON |
| GPS fazı / istasyon | **etkilenmez** — ayrı modül | — |

**Cevaplanacak:** "hedeflenen yeri iyileştirdi ama başka bir yeri bozdu mu?"

---

## KOŞU PLANI

`duz`, hibrit, **n=4/kol** dönüşümlü + `circle` regresyonu n=2/kol.
Diğer deneyler (D-V, Ö-T) **iki kolda da KAPALI** — tek değişken (§4).
İrtifa tutucu iki kolda da AÇIK.

## KARAR KURALI (önceden ilan)

- Birincil + en az bir ikincil deney lehine → **GİRER**
- Birincil kötüleşir **veya** kutusuz kare oranı %40'ı geçerse → **GİRMEZ**
- Bölünürse kullanıcıya; ölçüt değiştirilmez (§5.6)
- n<4/kol → **ara veri**, hüküm yok (§5.4)

---

## ⚑ D-V KAMPANYASINDAN ÖĞRENİLEN — D-N'nin YAPISAL ÜSTÜNLÜĞÜ

D-V (dikey düzeltme tabanı 1.5→6.0) koşuldu ve **darboğazı tavana taşıdı**:
terminal karelerinin %20-68'inde `|vz|` `VZ_MAX_TERM=5` tavanına dayandı,
bütçe kısıtı yatay hızı 16 → 10 m/s'ye kesti, hedef 15.1 m/s uçarken geride
kalındı.

D-N aynı tuzağa **düşmez**. Tutuş fazı yasası `vz = K_VZ·V_NOM·eps_elev`,
tavan `VZ_MAX=3.0`. Ölçülen `cy` medyanlarıyla hesaplandı:

| menzil bandı | `cy` | `vz` KAPALI (301) | `vz` AÇIK (318) | doyum |
|---|---|---|---|---|
| >8 m | 284 | −0.61 | **−1.21** | yok |
| 4-8 m | 272 | −1.03 | **−1.62** | yok |
| 1-2 m | 241 | −2.07 | **−2.60** | yok (tavan 3.0) |

Komut ~2 kat büyüyor ama **tavana değmiyor** → bütçe kısıtı tetiklenmiyor →
yatay hız kesilmiyor.

**Asıl fark zamanlama:** D-V son 2-3 saniyede bir açığı kapatmaya çalışıyor;
D-N açığın **hiç oluşmamasını** sağlıyor — tutuş fazı onlarca saniye sürüyor
ve orada 1 m'lik ofseti kapatmak için bol vakit var.

⇒ Öncelik sırası: **D-N > D-V2 > D-V**.

---

# SONUÇ — D-N, n=16 koşu (7 açık / 9 kapalı), 6 sim oturumu

## ⚠ MEKANİZMA KAPISINI YİNE YANLIŞ SEÇMİŞİM (dördüncü öz-düzeltme)

İlan ettiğim kapı "deney kolunda `|eps_elev|` küçülmeli" idi. **Bu ölçüt
kendi kendine referanslı ve mekanizmayı ÖLÇEMEZ:**

```python
eps_elev = atan((cy − CY_NISAN) / FY)
```

Kontrolcü `cy`'yi `CY_NISAN`'a sürüklüyor → `eps_elev` **her iki kolda da**
sıfıra gider, `CY_NISAN` ne olursa olsun. Doğru sütun `cy`'nin KENDİSİ.

Doğru kapıyla ölçüldü (tutuş fazı kareleri):

| kol | `cy` medyanı | hedef | fark |
|---|---|---|---|
| kapalı | **288** | 301 | −12 px |
| açık | **303** | 318 | −15 px |

**Mekanizma ÇALIŞTI**: `cy` 288 → 303, yani +15 px kayma (amaçlanan +17 px).
Drone hedefi kadrajda 15 px aşağıda tutuyor = ~5° daha yukarıda uçuyor.

## SONUÇ: ÖLÇÜLEBİLİR ETKİ YOK

| ölçüt | kapalı (n=9) | açık (n=7) |
|---|---|---|
| geçiş | 52 | 50 |
| en yakın medyan | 4.94 m | 4.92 m |
| en iyi | 0.22 m | 0.23 m |
| ≤0.5 m TEMAS | 3 | 3 |
| \|dikey\| ıska | 0.10 m | 0.12 m |
| aynı seviye | %75 | %76 |

Her ölçüt gürültü içinde. Mekanizma çalıştı ama **sonuç değişmedi**.

⚠ Ayrıca kontrol kolu geçerlilik eşiğini geçemedi: görsel temas **%58.4**
(ilan edilen sınır %60). Kendi kuralıma göre o kolun ölçütleri güvenilmez.

## NEDEN ETKİ YOK — kök neden hipotezim ÇÜRÜDÜ

İki bağımsız kanıt:

**1. Arşivde önyargı, hedef sabit uçarken de aynı.**

| hedef | n | dikey medyan | alttan |
|---|---|---|---|
| sabit irtifada | 352 | +0.33 m | %66 |
| tırmanıyor | 90 | +0.34 m | %67 |

**2. Temas geometrisini TUTUŞ değil TERMİNAL belirliyor.**

En yakın ana kadarki süre: tutuş 4.2 s, **terminal 1.0 s**. Ve terminal
kesişim yasası `CY_NISAN`'ı **kullanmaz** — `piksel_elev(cy) + iris_pitch`
ile mutlak yükseliş hesaplar. Yani tutuş fazının nişan noktası nereye
kurulursa kurulsun, son 1 saniyede terminal yasası yeniden nişan alıyor ve
tutuşun bıraktığı ofseti siliyor.

⇒ **"Dikey önyargının kaynağı CY_NISAN'dır" hipotezim YANLIŞTI.**
Geometrik eşleşme (5° ↔ +0.52 m) doğruydu ama NEDENSELLİK değil: tutuş fazı
o ofseti kuruyor, terminal onu eziyor.

## HÜKÜM: GİRMEZ

Mekanizma çalıştı, sonuç değişmedi, kontrol kolu geçerlilik eşini geçemedi.
`CY_NISAN` varsayılanı **301'de kalır**.

## BU BULGUNUN SONUCU — çalışma yönü değişiyor

D-N'nin başarısızlığı, D-V'nin başarısını açıklıyor: **dikey ıskayı belirleyen
TERMİNAL yasasıdır.** D-V terminal dikey kanalına dokunuyor ve dikey ıskayı
0.42 → 0.15 m iyileştirdi. D-N tutuşa dokunuyor ve hiçbir şey değiştirmedi.

⇒ **D-S de aynı sebeple şüpheli** — o da TUTUŞ fazını hedefliyor. Tutuş
fazının temas geometrisini belirlemediği ölçüldüğüne göre, tutuş salınımını
sönümlemek de temasa yansımayabilir. D-S'nin ölçüt ilanı bu bulguyla birlikte
yeniden değerlendirilmeli.

⇒ **Sıradaki iş D-V2** (terminal dikey tavanı 5→8): D-V'nin kazanımı
gerçekti, bedeli tavan doyumuydu. D-V2 tam onu açıyor.
