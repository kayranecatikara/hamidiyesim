# KAMPANYA DL — DİKEY ZARF GENİŞLETMESİ × KOMUT ÖLÇEĞİ

**16 uçuş toplam** (DK 8 + DL 8), 2×2 tam çarpanlı tasarım · 2026-08-20

> **Kullanıcı kuralı:** *"aracı kısıtlayan güdüm algoritmasının en iyi hale
> gelmesini engelleyen ne var ne yoksa bunları genişlet… gerçek ortamdaki
> aracımız çok güçlü bir FPV drone, simdeki aracın manevra kabiliyetinin
> çok çok üstünde."*

---

## 1 · KISIT BULUNDU: `PSC_JERK_Z` hiç dokunulmamıştı

| parametre | önceki | yeni | gerekçe |
|---|---|---|---|
| **`PSC_JERK_Z`** | **5.0 (ArduPilot varsayılanı)** | **40** | yatay `PSC_JERK_XY` 40'a çıkarılmıştı — **8 kat asimetri** |
| `WPNAV_ACCEL_Z` | 800 | 2000 | itki/ağırlık 7.08 → 60 m/s² mevcut |
| `WPNAV_SPEED_UP` | 1200 | 2000 | |
| `WPNAV_SPEED_DN` | 1000 | 1500 | |
| `Cfg.VZ_MAX` | 8.0 | 15.0 | güdüm kendi tavanıyla aracı boğmasın |

Jerk 5 m/s³ ile 3 m/s'lik dikey hız değişimi `2·√(3/5) = 1.55 s` sürer.
Ölçülen komut→gerçekleşme gecikmesi **0.8-1.0 s** ile birebir örtüşüyordu.

**Genişletme doğrulandı (§5.1, araç üzerinde mekanizma kapısı):**

| | jerk_z = 5 | jerk_z = 40 |
|---|---|---|
| komut→gerçekleşme gecikmesi | 0.8-1.0 s | **0.6 s** |
| takip oranı | 0.72-0.90 | **0.90** |

---

## 2 · ⭐⭐ ASIL BULGU — İKİ DEĞİŞİKLİK ETKİLEŞİYOR

2×2 tablo (her hücre n=4, aynı senaryo karışımı):

| dikey zarf | komut ölçeği | **İSABET** | en yakın (med) | \|dikey\| temasta |
|---|---|---|---|---|
| eski (jerk 5) | `v_los` | **4/4** | 1.67 m | 0.08 m |
| eski (jerk 5) | `kapanma` | **1/4** | 1.72 m | 0.59 m |
| yeni (jerk 40) | `v_los` | **2/4** | 1.91 m | 0.66 m |
| **yeni (jerk 40)** | **`kapanma`** | **3/4** | **1.02 m** | **0.07 m** |

**Köşegen okuma:** iki değişikliğin **her biri tek başına kötü**, ikisi
birlikte iyi.

**Fizik açıklaması:**
- `v_los` ölçeği **BÜYÜK** komut üretir (ölçüldü: gerekenin 4-18 katı).
  YAVAŞ dikey kanalda araç bunu filtreler; **fazlalık gecikmeyi telafi
  eder** ve sonuç doğru çıkar. HIZLI kanalda komut gerçekten uygulanır →
  **aşım** → ıska.
- `kapanma` ölçeği **KÜÇÜK** komut üretir. Yavaş kanalda hiç yetmez.
  Hızlı kanalda tam yeter.

> **Eski sistem KAZAYLA çalışıyordu:** aşırı ölçeklenmiş bir komut,
> yavaş bir araç tarafından filtreleniyordu. İki hata birbirini
> götürüyordu.

---

## 3 · ⚠ BUNUN GERÇEK DONANIM İÇİN ANLAMI

Kullanıcı: *"gerçek ortamdaki aracımız çok güçlü bir FPV drone, simdeki
aracın manevra kabiliyetinin çok çok üstünde."*

**Eski ayar (jerk 5 + `v_los`) gerçek dronda ÇÖKER.** Çünkü onun 4/4'ü,
aracın komutu uygulayAMAmasına dayanıyordu. Hızlı bir araçta aynı ayar
"yeni zarf + `v_los`" hücresine düşer: **2/4**.

Zarf genişletmesi bu gizli bağımlılığı **görünür yaptı**. Simde kalsaydık
gerçek uçuşta sürprizle karşılaşacaktık.

---

## 4 · GENİŞLETİLMİŞ ZARFTA KOL KIYASI (DL, n=4/kol)

| ölçüt | `v_los` | `kapanma` | p |
|---|---|---|---|
| **İSABET** | **2/4** | **3/4** | |
| koşunun en yakını | 1.91 m | **1.02 m** | 0.486 |
| **\|dikey\| temasta** | 0.66 m | **0.07 m** | 0.543 |
| **\|vz\| p90** | 3.77 | **2.25** | **0.057** |
| faz düşüşü | 16.0 | **5.5** | 0.800 |
| savrulma son 3 m | 0.72 | 0.58 | 0.600 |
| ilk temasa süre | 65.6 s | 60.2 s | 0.829 |

| koşu | senaryo | imha | en yakın | \|dz\| temasta |
|---|---|---|---|---|
| DL01_K | daire | ✗ | 1.89 | 1.65 |
| DL03_K | daire | ✗ | 1.94 | 0.07 |
| DL05_K | düz+yatay | ✓ | 1.93 | 1.21 |
| DL07_K | düz+yok | ✓ | 0.43 | 0.11 |
| **DL02_A** | daire | ✓ | **0.76** | **0.01** |
| DL04_A | daire | ✗ | **4.23** | **3.50** ⚠ |
| DL06_A | düz+yatay | ✓ | 1.28 | 0.09 |
| DL08_A | düz+yok | ✓ | 0.53 | 0.05 |

⚠ **DL04_A aykırı** (4.23 m, dikey 3.50 m) — gizlenmiyor. Tek kötü koşu
ve isabet oranını 4/4'ten 3/4'e indiren o.

---

## 5 · ⚠ HÜKÜM KURULMUYOR

n=4/kol, p değerleri 0.057-0.829. **Hiçbir ölçüt istatistiksel olarak
ayrışmadı** (§5.4). Yön tutarlı ama kesinlik yok.

En güçlü sinyal: **\|vz\| p90 3.77 → 2.25 (p=0.057)** — mekanizma
ölçütü, yani "komut gerçekten küçüldü".

---

## 6 · AI ÖNERİSİ

**İkisi birlikte girsin:**
1. Dikey zarf genişletmesi (`PSC_JERK_Z` 5→40 ve diğerleri) — **kalıcı**
2. `DIKEY_KAPANMA` — **varsayılan AÇIK**

**Gerekçe sonuç ölçütü değil, GEÇERLİLİK:** eski ayar aracın yavaşlığına
bağımlıydı; gerçek FPV dronda o bağımlılık yok. Genişletilmiş zarf gerçek
donanıma daha yakın, ve o zarfta doğru ölçek `kapanma`.

**Bedeli açık:** isabet 4/4 (eski) → 3/4 (yeni), tek uçuş farkı, n=4'te
anlamsız. Ve DL04_A gibi kötü bir aykırı var.

⚠ **Karar kullanıcınındır.** Hiçbir varsayılan onay olmadan
değiştirilmedi — `DIKEY_KAPANMA` hâlâ **KAPALI**. Zarf parametreleri ise
`.parm` dosyasında **değiştirildi** (kullanıcı açıkça "genişlet" dedi).

---

## 7 · KULLANICININ KENDİ SINAMASI

```bash
cd ~/projects/avci_sim
bash scripts/kapat.sh && bash scripts/mkur.sh test
```

Panel → **🎚 AYAR KONSOLU** → **⭐ DİKEY KOMUT ÖLÇEĞİ** → `DIKEY_KAPANMA`.

**Neye bakılacak:** açık/kapalı çevirip **son 3 metreye** bak.
- **Kapalı:** araç dikeyde sert davranıp aşıyor (zarf genişledi, komut
  hâlâ büyük).
- **Açık:** dikey sakin, hedefe hizalı giriyor.

Zarf genişletmesini de görebilirsin: dikey manevralar genel olarak daha
çevik olmalı (jerk 5→40).
