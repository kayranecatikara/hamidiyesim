# D-S · Tutuş fazına dikey sönümleme — ölçüt ilanı

**Tarih:** 2026-08-16 (koşulmadan ÖNCE yazıldı, §4)
**Kol anahtarı:** `AVCI_IBVS_TUTUS_SONUM` · panel: "D-S · Tutuş fazına dikey sönümleme"
**Kontrol:** 0.0 (kapalı) · **Deney:** 0.6 (terminaldeki `K_VZ_D` ile aynı)

---

## Neden — dikey ıskanın İKİNCİ bileşeni

Dikey ıska iki bileşene ayrıldı (kara kutu, 1978 geçiş):

| bileşen | büyüklük | kaynağı | çaresi |
|---|---|---|---|
| **ÖNYARGI** | +0.53 m (geçişlerin %74-81'i alttan) | `CY_NISAN` 5° tasarımı | **D-N** |
| **SAÇILMA** | ±0.40 m (0.5-1 m bandı std) | *bu belge* | **D-S** |

İsabet zarfı dikeyde **+0.29 / −0.13 m**. Önyargı düzelse bile ±0.40 m
saçılma zarfın dışına taşar; ikisi birden çözülmeli.

## Kök neden

Tutuş fazının dikey yasası **saf P kontrolcü**:

```python
vz = clamp(K_VZ · V_NOM · eps_elev, ±VZ_MAX)
```

Türev terimi **YOK**. Terminalde var (`K_VZ_D = 0.6`), tutuşta yok.
Gecikmeli bir sistemde (aracın dikey ivme rampası) saf P doğal olarak
salınır.

**Ölçüldü (113 koşu, tutuş fazı kareleri):**

| ölçüm | değer |
|---|---|
| `eps_elev` salınımı | 0.45 /s |
| `eps_elev` genliği (p90) | **12.7°** |
| `vz_cmd` işaret değişimi | 0.29 /s |

## Ne yapar

Terminaldeki sönümlemenin **aynısı** tutuşa uygulanır:

```python
vz_istenen = K_VZ · V_NOM · eps_elev
vz = vz_istenen + TUTUS_SONUM · (vz_istenen − vz_gerçek)
```

Araç istenenden hızlı tırmanıyorsa komut geri çekilir → aşım biter.

⚠ **Terminal fazına DOKUNMAZ** — orada zaten `K_VZ_D` çalışıyor.
Birim testi DS2 bunu 9 kombinasyonda sınıyor.
⚠ 0.0 iken tutuş yasası **bit bit** eski — DS1, 25 kombinasyon.

---

## ÖLÇÜTLER

### Birincil
**Kara kutudan gerçek en yakın menzil** (deney kolunun medyanı küçük olmalı).

### İkincil
1. **Dikey ıskanın STANDART SAPMASI** — mekanizmanın doğrudan hedefi
   (medyan değil! D-S saçılmayı hedefliyor, önyargıyı değil)
2. Temas sayısı (≤0.5 m)
3. Aynı seviye geçiş oranı (|dikey| ≤ 0.5 m)

### Mekanizma kapısı (§5.1)
Deney kolunda tutuş fazı `eps_elev` **salınımı** (işaret değişimi/s) ve
**genliği** (p90) kontrolünkinden küçük olmalı. Değilse özellik iş
görmemiştir; o koşu veri değildir.

⚠ Bu kapı `vz_cmd` üzerinden DEĞİL salınım üzerinden okunur — D-V
kampanyasında mekanizma kapısını sonuç ölçütüyle karıştırma hatası yapıldı,
tekrarlanmıyor.

### Geçerlilik eşi (§5.2)
| ölçüt | kötü sebeple iyileşir mi | zorunlu eş |
|---|---|---|
| dikey std | **evet** — hiç yaklaşmazsan saçılma da küçük görünür | geçiş sayısı + en yakın menzil |
| salınım azalması | **evet** — sönümleme aşırıysa araç DONUK kalır, hataya cevap veremez | `eps_elev` MEDYANI büyümemeli |

⚠ **En büyük risk: aşırı sönümleme.** `TUTUS_SONUM` çok büyükse dikey kanal
tembelleşir ve önyargı BÜYÜR. Bu yüzden `eps_elev` medyanı zorunlu eş.

### Geçerlilik (§4)
Hedef 20-250 m / 6-25 m/s. Dışına çıkan koşu SAYILMAZ.

---

## ETKİ ALANI TABLOSU (§5.10)

| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Tutuş fazı dikey salınım | doğrudan hedef | `duz` — ana ölçüm |
| Tutuş fazı dikey ÖNYARGI | aşırı sönümleme tembelleştirir | `duz` — `eps_elev` medyanı |
| Terminal kesişim | **etkilenmez** — ayrı dal | birim testi DS2 |
| Manevrada dikey | dönüşte hata hızlı değişir, sönümleme geciktirebilir | `circle` — REGRESYON |
| Yatay kanal | **etkilenmez** — yalnız `vz` | birim testi DS1 |

---

## KOŞU PLANI

`duz`, hibrit, **n=4/kol** dönüşümlü + `circle` regresyonu n=2/kol.
Diğer dört deney (D-N, D-V, D-V2, Ö-T) **iki kolda da KAPALI** — tek değişken.
İrtifa tutucu iki kolda da AÇIK.

## KARAR KURALI (önceden ilan)

- Birincil + dikey std deney lehine → **GİRER**
- Birincil kötüleşir **veya** `eps_elev` medyanı büyürse (aşırı sönümleme)
  → **GİRMEZ**
- Bölünürse kullanıcıya; ölçüt değiştirilmez (§5.6)
- n<4/kol → **ara veri** (§5.4)

---

## ⚠⚠ D-S TEK BAŞINA SINANMAZ — ÖNCEDEN İLAN EDİLEN TAHMİN

Ölçülen dağılımdan (merkez +0.53 m, std 0.40 m) ve isabet zarfından
(dikeyde −0.13 … +0.29 m) normal yaklaşımla hesaplandı:

| senaryo | merkez | std | zarfa düşme | şimdikine göre |
|---|---|---|---|---|
| ŞİMDİ | +0.53 | 0.40 | %22.5 | — |
| **D-N** (önyargı sıfır) | 0.00 | 0.40 | %39.3 | **1.7×** |
| **D-S TEK BAŞINA** | +0.53 | 0.20 | **%11.5** | **0.5× — KÖTÜLEŞİR** |
| **D-N + D-S** | 0.00 | 0.20 | %66.9 | **3.0×** |

**D-S tek başına ZARARLI.** Önyargı dururken saçılmayı daraltmak, şans eseri
zarfa düşen geçişleri de yok eder — hepsi +0.53'te toplanır, zarfın dışında.

⇒ **D-S, D-N AÇIKKEN sınanır.** Tek değişken kuralı korunur (kollar arasında
yalnız D-S değişir), ama taban doğru olur.

Kampanya scripti bunu zorluyor: `logs/ds_ONAY` dosyası yoksa çıkıyor.
Dosyanın içeriği D-N'nin taban konumunu (`acik`/`kapali`) belirtir ve
ancak D-N kampanyası ANALİZ EDİLDİKTEN sonra oluşturulur.

⚠ Bu bir TAHMİN, ölçüm değil (normal yaklaşım; gerçek dağılım çarpık
olabilir). Kampanya bu tahmini sınayacak — tahmin tutmazsa o da bir bulgudur.
