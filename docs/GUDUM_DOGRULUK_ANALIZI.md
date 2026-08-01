# Güdüm Doğruluk Analizi — 1. Tur

**Veri:** 2026-07-27 19:52, `square` senaryosu, tek chase oturumu, iki görsel faz.
`logs/visual_lead_20260727_195241.csv` (38 kare) ve `..._195254.csv` (29 kare).

**Sonuç: 0/2 vuruş.** Ama başarısızlık biçimi beklenen değil — hedef ıskalanmadı,
**hedef kadrajdan çıktı**. Kör dalışa hiç girilmedi.

---

## Özet tablo

| | Devir 1 | Devir 2 |
|---|---|---|
| Görsel faz süresi | 1.2 s (38 kare) | 0.9 s (29 kare) |
| Devirde menzil | 10.0 m | 11.3 m |
| Devirde aspect | 85.6° | 93.0° |
| Devirde yandanlık (gerçek) | 0.88 | 1.21 |
| Üretilen lead | 20.1° | 14.8° |
| `tespit_yok` kare | 20 (ardışık) | 20 (ardışık) |
| Bitiş | temas kaybı | temas kaybı |
| En yakın menzil | 8.89 m | 11.03 m |

---

## 1. Ana hipotez ÇÜRÜDÜ

Plan şu varsayımla kurulmuştu: *GPS fazı kuyruktan devrediyor → `yandanlik ≈ 0`
→ lead ≈ 0 → görsel faz bilgisiz başlıyor.*

**Ölçüm bunun tersini söylüyor.** Devir anında aspect **85-93°**, yani neredeyse
**tam yandan**. `yandanlik` 0.88-1.21, lead 15-25° üretiliyor. Yönelim sinyali
bol; kör nokta yok.

Sorun ters yönde: **tam yandan + 10 m menzil, LOS açısal hızı için EN KÖTÜ
kombinasyon.**

## 2. Gerçek arıza: hedef kadrajdan çıkıyor

`bbox` kolonunun kare kare yürüyüşü (Devir 1):

```
kare  0: 263|189|277|201   merkeze yakın
kare  6: 216|171|236|192   sola kayıyor
kare 10: 149|147|180|178
kare 14:  59|109|102|154   sol kenara dayandı
kare 17:   0| 88| 21|123   kadrajdan çıkıyor
kare 18+: tespit_yok ×20   → GPS'e dönüş
```

Aynı anda `yaw_hata_deg`: **−32° → −95°**. Kameranın yatay yarı-FOV'u 62.5°
(HFOV 125°). Nişan hatası 62.5°'yi aşınca hedef **fiziksel olarak görüntünün
dışında** kalır.

Devir 2 aynı arıza, farklı yön: bbox sağ-yukarı yürüyor (`336|119` → `401|0`),
`pitch_hata` +58° → +70°, hedef **üstten** çıkıyor.

### Neden: LOS hızı yaw hızını aşıyor

- Devirde menzil ~10 m, aspect ~90° → yanal bağıl hız neredeyse tam kapanma hızı
- `V_KAPANMA = 25 m/s` → LOS açısal hızı ≈ 25/10 = 2.5 rad/s ≈ **143°/s**
- `YAW_HIZ_MAX = 90°/s` → kamera yetişemez, geride kalır, hedef kaçar

Quad hız vektörünü yana sürebiliyor (`vy_cmd = −18.4 m/s` sabit) ama **kamera
gövdeye sabit** — nereye baktığı yaw'a bağlı ve yaw 90°/s ile sınırlı.

## 3. Somut hata: devirde hız rampası SIFIRDAN başlıyor

`CopterAdapter.v_onceki` her `run_visual_lead` çağrısında `(0,0,0)`'dan başlıyor.
`IVME_TAVAN = 4 m/s²` ile 25 m/s'ye ulaşmak **6.25 s** sürer; görsel faz ~1 s.

**Devir 2'de tam olarak bu oldu** — `v_cmd` sütunu:
```
kare 1: 0.08   kare 3: 0.17   kare 5: 0.25   kare 8: 0.36  (m/s)
```
GPS fazı drone'u ~19 m/s'de teslim etti, görsel faz **0.08 m/s komut etti** —
yani fiilen "dur" dedi. Hedef uçup gitti (`menzil_gercek_m` 14 → 30 m).

**Devir 1'de rampa devreye girmedi** çünkü ilk karede `dt is None` ve
`adapter_copter.compute()` o durumda ivme sınırlayıcıyı **atlıyor** → tek karede
0'dan 25 m/s'ye sıçradı. İki devir, aynı kökten iki farklı yanlış davranış:
biri sert fren, diğeri sert sıçrama.

Doğrusu: görsel faz başlarken `v_onceki` aracın **gerçek anlık hızıyla**
tohumlanmalı (GPS fazından devralınan hız). Kod `LOCAL_POSITION_NED`'i zaten
okuyor (`_ArasState`), veri elde var.

## 4. Pose modeli aslında İYİ

Ölçülebilen karelerde (Devir 1, n=18):

| Ölçüm | Sonuç | Yorum |
|---|---|---|
| `yandanlik` hatası | −0.040 ± 0.041 | çok iyi |
| Eksen açı hatası | medyan −0.4°, %83'ü ±5° içinde | çok iyi |
| Keypoint hatası | 2.9 px (5-10 m), 4.6 px (10-15 m) | kabul edilebilir |
| `sin(aspect)` uyumu | 0.877 teorik / 0.875 ölçülen | formül doğrulandı |

**Yani model, güdümün başarısızlığının sebebi değil.** Yönelim kestirimi sağlam.

### Tek istisna: burun/kuyruk takası %11

10-15 m bandında **2/18 kare** takas (%22). `flip_sayaci = 0` — yani
`guidance_core`'un flip koruması **hiç tetiklenmedi**, çünkü o yalnız 0.2 s
içindeki ani takası görüyor, kalıcı yanlış etiketlemeyi göremiyor. Takas lead'i
tam ters çevirir. Kullanılmayan v-tail keypoint'leri burun/kuyruk ayrımını
doğrulamak için değerlendirilmeli.

## 5. `yandanlik_gercek = 1.21` — düzeltme aşırı telafi ediyor

`yandanlik = a / olcek`, `olcek = olcek_ham / duzeltme` ve `duzeltme ≥ 1`.
Yükselti büyükken `olcek < a` olabiliyor ve oran 1'i aşıyor. Aynı şey güdüm
tarafında da olur → `carpim > 0.95` → `cozumsuz` bayrağı ve şişmiş lead.
Sınırlandırılmalı (`min(yandanlik, 1.0)`) ya da düzeltme formülü gözden
geçirilmeli.

## 6. GPS fazı kendi hedefini tutturamıyor

`[GPS-YAKLASMA]` logu: `d_h` 96 → 203 → 69 → 125 m arası salınıyor, hiç
oturmuyor. Komut yaw'ı −29° → 174° → −95° savruluyor — drone hedefin **etrafında
dönüyor**, arkasına yerleşmiyor.

Bu, aspect'in 90° çıkmasının sebebi: **kuyruk istasyonu (`APPROACH_STANDOFF`)
mesafeyi tutturuyor ama YÖNÜ tutturamıyor.** Devir, drone yandayken oluyor.

Ek gözlemler:
- Hedef **209 m irtifaya** tırmandı; drone onu 186 m'ye kadar takip etti.
  `square` senaryosunda Talon sürekli tırmanıyor — senaryo ayrı incelenmeli.
- Hedef telemetrisi çoğu döngüde bayat (`fresh=0`, `hold` 0.1-2.4 s).

---

## Yapılacaklar — öncelik sırası

1. ~~**Devir hız sürekliliği** (Bölüm 3).~~ **YAPILDI (2026-07-27).**
   - `CopterAdapter.tohumla(v_ned)` eklendi; `visual_lead` ilk komuttan önce
     rampayı aracın `LOCAL_POSITION_NED` hızıyla tohumluyor.
   - `dt is None` dalındaki sınırlayıcı atlaması kaldırıldı; dt yoksa nominal
     kare aralığı (`NOMINAL_DT_S = 1/30`) kullanılıyor, tavan asla delinmiyor.
   - Regresyon testleri T31/T32.

2. ~~**LOS hızı ↔ yaw hızı uyumsuzluğu** (Bölüm 2).~~ **YAPILDI (2026-07-27)** —
   kullanıcı kararı: kapanma hızını menzille ölçekle.
   `CopterAdapter.kapanma_hizi(menzil)`: `v ≤ LOS_PAY · radians(YAW_HIZ_MAX) · menzil`,
   `V_KAPANMA` tavanı ve `V_KAPANMA_MIN` tabanı arasında kırpılır.

   | menzil | eski | yeni | gereken yaw |
   |---|---|---|---|
   | 8 m | 25 m/s | 10.1 m/s | 72°/s |
   | 10 m | 25 m/s | 12.6 m/s | 72°/s |
   | 15 m | 25 m/s | 18.8 m/s | 72°/s |
   | 20 m+ | 25 m/s | 25 m/s | ≤72°/s |

   Gereken yaw hızı artık her menzilde 90°/s tavanının altında. Regresyon
   testleri T33-T35. **Menzil kestirimi yalnız `kalite > 0` iken kapıya
   verilir**; ölçek güvenilmezken kapı kapanır (eski davranış).

   *Bilinen sınır:* kapı en kötü durumu (tam yandan geçiş) varsayar. Kuyruktan
   takipte bağıl teğet hız küçük olduğundan kısıt gereğinden muhafazakârdır —
   madde 3 çözüldükten sonra yeniden ayarlanmalı.

   **Tasarım sözleşmesi değişti:** `guidance_core` docstring'i eskiden
   "menzil kestirimi güdüme bağlanmaz, SADECE log" diyordu. Artık bağlanıyor.
   Ground truth hâlâ güdüme GİRMİYOR — kullanılan değer pose'dan türetilen
   kestirim (`menzil_kestirim_m`), gerçek menzil değil.

3. **GPS fazı kuyruk yakınsaması** (Bölüm 6). İstasyon yönü tutturulmuyor;
   yaklaşma açısı kontrolü gerekiyor.

4. ~~**`yandanlik` sınırlaması** (Bölüm 5).~~ **YAPILDI (2026-07-27).**
   `guidance_core` ve `vision/dogruluk` ikisinde de `min(a/olcek, 1.0)`.

5. **Burun/kuyruk takası** (Bölüm 4) — v-tail keypoint'leriyle doğrulama.

6. **`square` senaryosunda hedefin sürekli tırmanması** — ayrı inceleme.

**Not:** 1. tur verisi tek oturum ve iki kısa görsel fazdır; 4. ve 5. maddelerin
istatistiği zayıf (n=18). Madde 1-2 düzeltildikten sonra görsel fazlar uzayacak
ve o zaman anlamlı örneklem toplanabilecek.
