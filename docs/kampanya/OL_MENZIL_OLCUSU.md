# KAMPANYA OL — KUTU → MENZİL ÖLÇÜSÜ (çarpım ↔ köşegen)

**Tek değişken:** `AVCI_IBVS_OLCU` = `carpim` / `kosegen`
**6 uçuş**, n=3/kol (her kolda 2× `circle_xl` + 1× `duz`+`yatay`, §5.9 eşit),
dönüşümlü · `V_HUCUM = 18` iki kolda da sabit · 2026-08-19

> **Kullanıcı önerisi:** *"köşegen sqrt(w²+h²) çıkartıp onunla oranlamak.
> Böyle yaparsak hedef aracı arkadan gördüğümüzde hedef aracın ve bizim
> aracın roll değerleri nasıl olursa olsun sistem bozulmaz değil mi."*

---

## 1 · ÖN ÇALIŞMA — model ölçüleri ve kalibrasyon

**Gerçek boyutlar** (`mini_talon_vtail` collision mesh, STL sınır kutusu):
kanat açıklığı **1.280 m** · gövde boyu **0.814 m** · yükseklik **0.286 m**.

⛔ **Mevcut sabit yanlıştı.** `MENZIL_PX_M = 160.0` kullanılıyordu; 8 uçuşun
5812 karesi gerçek menzille eşlenip ölçülünce doğrusu **185.7 px·m** çıktı.
Menziller **%14 eksik** tahmin ediliyordu. Düzeltildi.

⭐ **Kullanıcının duruş tespiti doğrulandı:** görüş açısının **%91'i 0-15°**
(tam arkadan), medyan **1°**.

**Çevrimdışı ölçü kıyası** (§2: hipotez üretir, karar vermez), 0-15° bandı:

| ölçü | menzil hatası p50 | yatış duyarlılığı 0-90° |
|---|---|---|
| `sqrt(w·h)` (bugünkü) | %22 | %83 |
| **`sqrt(w²+h²)` köşegen** | **%14** | **%19** |
| `w` tek başına | %12 | %359 (kırılgan) |
| `h` tek başına | %35 | — |

**Köşegenin matematiği:** kutu eksen-hizalı olduğu için θ dönmüş ince bir
çubukta `w = L·|cosθ|`, `h = L·|sinθ|` → `sqrt(w²+h²) = L`, **yatıştan tam
bağımsız** (birim testi G2: 0-89° döndürüldü, değişim 2.8e-14).

---

## 2 · MEKANİZMA KAPISI (§5.1) — GEÇTİ

Logdaki `boyut` sütunu `w,h`'den yeniden hesaplanıp hangi formülle
üretildiği doğrulandı:

| koşu | `boyut` formülü | kalibre C |
|---|---|---|
| OL01_C · OL03_C · OL05_C | **çarpım** (605/891/642 kare, köşegen 0) | 166-185 |
| OL02_K · OL04_K · OL06_K | **KÖŞEGEN** (531/231/478 kare, çarpım 0) | 275-335 |

Karışma yok.

---

## 3 · SONUÇ — n=3/kol

| ölçüt | ÇARPIM | KÖŞEGEN | p |
|---|---|---|---|
| **bağıl menzil hatası p50** | **%18** | **%10** | 0.600 |
| bağıl menzil hatası p90 | %47 | %38 | 0.600 |
| **faz düşüşü (log sayısı)** | **4** | **1** | 0.200 |
| ilk temasa süre | 53 s | 48 s | 1.000 |
| koşunun en yakını | 1.06 m | 1.18 m | 1.000 |
| en iyi yaklaşma | 1.00 m | 1.16 m | 1.000 |
| \|vz\| p90 | 4.84 | 3.77 | 0.600 |
| \|yatış\| p90 | 38.4° | 37.5° | 1.000 |
| <10 m'de geçen süre | %6 | %2 | 0.400 |
| **İSABET** | **3/3** | **3/3** | |

⚠ n=3/kol'de permütasyon testinin verebileceği **en küçük p = 0.100**
(20 kombinasyondan 2'si). Yani hiçbir ölçüt istatistiksel olarak ayrışmadı.
§5.4 gereği bunlar **ARA VERİ**.

---

## 4 · ⭐ ASIL BULGU — iyileşmenin uçuşa gidecek yolu YOK

Kampanya sırasında fark edildi: bugünkü ayarla menzil tahmini `R` güdümde
neredeyse hiçbir şeyi sürmüyor.

| `R`'nin tüketicisi | durum |
|---|---|
| lead sönmesi (`LEAD_SONUM`) | ✅ **tek gerçek yol** |
| hız PI hatası | ⚠ hız zaten doygun — **13325 karenin %58'i tam 18.0 m/s** |
| yanal kesişme (`YANAL_K = 0`) | ❌ kapalı |
| yaw menzil tavanı (`YAW_MENZIL_REF = 0`) | ❌ kapalı |
| kaçış telafisi (`KACIS_KD = 0`) | ❌ kapalı |

> **Menzil tahmini %18 → %10 iyileşti ama bu iyileşmenin uçuşa gidebileceği
> tek kanal lead sönmesi — yumuşak, ikincil bir etki.** Sonuç ölçütlerinin
> ayrışmaması bu yüzden BEKLENEN. Ölçü değişikliği bugün "işe yaramadı"
> değil; **işe yarayacağı yer henüz açık değil.**

---

## 5 · KARAR KURALI DENETİMİ

| # | kural | sonuç |
|---|---|---|
| 1 | bağıl menzil hatası düşer | ✓ **%18 → %10** (p=0.600, n=3) |
| 2 | isabet kötüleşmez | ✓ **3/3 → 3/3** |
| 3 | en yakın menzil kötüleşmez | ~ 1.06 → 1.18 m (p=1.000, gürültü) |

**Üçü de sağlandı** ama hiçbiri istatistiksel değil.

**Yan kazanç:** faz düşüşü **4 → 1**. Köşegen kolunda görsel kilit çok daha
sürekli. Anlamlı değil (p=0.200) ama yönü tutarlı (3/3 koşuda daha az).

**Yan gerileme:** <10 m'de geçen süre %6 → %2, yaklaşma sayısı 5 → 3.
İkisi de gürültü içinde (p=0.400).

---

## 6 · AI ÖNERİSİ

**Köşegen GİRSİN** — ama sonuç ölçütü için değil, **altyapı** olduğu için:

1. Menzil tahmini ölçülebilir biçimde daha doğru (%18 → %10) ve kalibre
   hatası (160 → 185.7) da yol boyunca düzeltildi.
2. Yol haritasındaki **her sıradaki adım `R` ve `ṙ`'ye dayanıyor** —
   yavaşlama profili, integral yolu, aykırı değer kapısı, kestirim.
   Menzil yanlışken onların üstüne bina kurmak hatayı taşır.
3. Bugün zarar vermiyor: isabet aynı, salınım aynı, yatış aynı.
4. Yatış bağışıklığı yapısal olarak kanıtlı (test G2), ölçümden bağımsız.

**Bedeli açık:** bugün ölçülebilir bir uçuş kazancı YOK. Kazanç, yol
haritasının 2. adımı (yavaşlama profili) gelince görünecek.

⚠ **Karar kullanıcınındır** — komutlar §7'de.

---

## 7 · KULLANICININ KENDİ SINAMASI

```bash
cd ~/projects/avci_sim
bash scripts/kapat.sh && bash scripts/mkur.sh test
```

Panel → **🎚 AYAR KONSOLU** → **⓪ KUTU → MENZİL ÖLÇÜSÜ** → `BOYUT_OLCU`
düğmesi (`carpim` / `kosegen`). Uçuş sırasında değiştirilebilir.

**Neye bakılacak:** panelin `mesafe` göstergesi ile gerçek yakınlığın
uyumu. Köşegende hedef yatarken menzil daha az zıplamalı. Ve sim
terminalinde `[SUPERVISOR] GPS fazı` satırları daha seyrek olmalı
(faz düşüşü 4 → 1).
