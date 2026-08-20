# KAMPANYA YV — YAVAŞLAMA PROFİLİ

**Tek değişken:** `AVCI_IBVS_YAVASLAMA` 0/1 · 8 uçuş, n=4/kol
(her kolda 2× `circle_xl` + 1× `duz`+`yatay` + 1× `duz`+`yok`, §5.9 eşit),
dönüşümlü · `BOYUT_OLCU=kosegen`, `V_HUCUM=18` iki kolda sabit · 2026-08-20

> **Kullanıcı fikri:** *"hedefin kadrajda büyümesiyle orantılı şekilde hızı
> azaltma… hedef araç ile yakın hızlarda olmak kaçırma riskini minimuma
> indirir, daha dengeli yaklaşmayı mümkün kılar."*

---

## 1 · ⚠ İLK DENEME GEÇERSİZ — mekanizma kapısı hatayı yakaladı

İlk çiftte (`YV01_K`/`YV02_A`) `kapanma_hedefi` sütunu doluydu ama:

| menzil | v_los | kapanma_hedefi | hiz_I |
|---|---|---|---|
| 0-4 m | **18.00** | 1.50 | 21.62 |
| 15-30 m | **18.00** | 5.46 | **24.00 = I_MAX** |

**Hıza sıfır etki.** Kök neden **integral windup**: uzakta profil
V_HUCUM'un izin verdiğinden fazla kapanma istiyor, hata kapanmıyor,
`hiz_I` tavana tırmanıyor; yakında profil daralsa bile toplam yine
doyuma çarpıyor.

**Düzeltme — anti-windup:** çıktı doyumdayken, doyumu derinleştiren yönde
integral dondurulur. İki uçuş **geçersiz** sayıldı, kampanya baştan koşuldu.

> Bu, §5.1 mekanizma kapısının işe yaradığı üçüncü kayıtlı olay. Kapı
> olmasaydı "yavaşlama denendi, fark etmedi" diye yanlış rapor yazılacaktı.

---

## 2 · MEKANİZMA KAPISI (2. deneme) — GEÇTİ

| menzil | KAPALI v_los | AÇIK v_los | AÇIK doyum |
|---|---|---|---|
| 15-30 m | 18.00 | 18.00 | %72 |
| 8-15 m | 18.00 | 17.82 | %35 |
| **4-8 m** | 18.00 | **16.14** | **%3** |
| 0-4 m | 18.00 | 18.00 | %96 |

Doyum oranı kol geneli **%97.5 → %54.3** (p=0.057). Hız gerçekten
tavandan çıkıyor.

---

## 3 · SONUÇ — n=4/kol

| ölçüt | KAPALI | AÇIK | p | |
|---|---|---|---|---|
| **İSABET** | **4/4** | **2/4** | | ✗ |
| **koşunun en yakını** | **0.84 m** | **1.76 m** | **0.057** | ✗ |
| **ilk temasa süre** | **52 s** | **164 s** | 0.171 | ✗ |
| doyum % (mekanizma) | 97.5 | 54.3 | 0.057 | ✓ |
| \|vz\| p90 | **4.04** | **7.23** | **0.057** | ✗ |
| \|dikey\| temasta | 0.54 m | **0.21 m** | 0.571 | ✓ |
| \|yatış\| p90 | 35.5° | 26.1° | 0.829 | ≈ |

Koşu bazında:

| koşu | senaryo | imha | en yakın | ilk temasa |
|---|---|---|---|---|
| YV11_K | daire | ✓ | 0.58 | 30 s |
| YV13_K | daire | ✓ | 0.60 | 39 s |
| YV15_K | düz+yatay | ✓ | 1.09 | 127 s |
| YV17_K | düz+yok | ✓ | 1.38 | 66 s |
| YV12_A | daire | ✓ | 1.47 | 36 s |
| YV14_A | daire | ✓ | 1.50 | 201 s |
| **YV16_A** | **düz+yatay** | **✗** | **4.34** | 195 s |
| **YV18_A** | **düz+yok** | **✗** | **2.03** | 132 s |

**İki ıskanın ikisi de `duz` senaryosunda** — tam da risk olarak
işaretlenen yerde.

---

## 4 · ⛔ BİRİNCİL ÖLÇÜT ÖLÇÜLEMEDİ (§5.8 dürüstlük)

İlan edilen birincil ölçüt **"temas anındaki bağıl hız"**tı. Ölçülemedi:
temas anında hedef sim'de yeniden doğduğu için bağıl hız hesabı
**24.72 · 42.88 · 21.28 m/s** gibi artefaktlar üretiyor. Pencere
daraltıp aykırı değer eleyince bile örnekler kirli kaldı.

**Bu ölçüt bu enstrümantasyonla güvenilir değil.** Karar, geçerlilik
eşlerine (isabet, en yakın menzil, ilk temasa süre) dayandırıldı.

---

## 5 · KARAR KURALI DENETİMİ

| # | kural | sonuç |
|---|---|---|
| 1 | isabet kötüleşmez | ✗ **4/4 → 2/4** |
| 2 | temas anındaki bağıl hız düşer | **ÖLÇÜLEMEDİ** (§4) |
| 3 | ilk temasa süre kötüleşmez | ✗ **52 → 164 s (3 kat)** |
| 4 | en yakın menzil kötüleşmez | ✗ **0.84 → 1.76 m** (p=0.057) |

**Üç kural açıkça düştü.** Özellik **bu ayarlarla GİRMEZ.**

---

## 6 · NE ÖĞRENDİK

**Kullanıcının hipotezi kısmen doğrulandı:** temas anındaki dikey hizalama
**0.54 → 0.21 m** iyileşti. Yani "yakın hızda daha dengeli yaklaşma" fikri
geometri açısından çalışıyor.

**Ama taktik bedeli ağır bastı:** ilk temasa süre 3 katına çıktı ve iki
ıskanın ikisi de kaçamak/düz senaryoda geldi. Yavaş kapanma hedefe kaçma
zamanı veriyor — bu risk kampanyadan ÖNCE ilan edilmişti (ölçülmüş kanıt:
0.9 m/s kapanmada araç 8 saniye 6 metrede asılı kalmıştı) ve
**gerçekleşti**.

**Beklenmedik:** dikey salınım kötüleşti (\|vz\| p90 4.04 → 7.23,
p=0.057). Hız değiştikçe dikey kanalın çalışma noktası da kayıyor.

### Ayar çok agresifti — daha hafif profiller

| ayar | 20 m | 10 m | 6 m | 3 m | son 5 m süresi |
|---|---|---|---|---|---|
| **SU AN** (T_GO 4, taban 1.5) | 5.00 | 2.50 | 1.50 | 1.50 | **3.3 s** |
| ORTA (T_GO 2.5, taban 2.5) | 6.00 | 4.00 | 2.50 | 2.50 | 2.0 s |
| HAFİF (T_GO 2, taban 3.5) | 6.00 | 5.00 | 3.50 | 3.50 | 1.4 s |

Kod ve kill-switch **duruyor**; ayarlar konsoldan değiştirilebilir.
Daha hafif profille yeniden sınamak mümkün — ama bu **kullanıcının
kararıdır**, AI kendiliğinden yeni kampanya açmaz.

---

## 7 · AI ÖNERİSİ

**Bu ayarlarla GİRMESİN.** Varsayılan `YAVASLAMA = KAPALI` kalır
(zaten öyle, değiştirilmedi).

İki yol var:
1. **Bırak** — sistem 4/4 isabetle iyi durumda; yol haritasının bir sonraki
   adımına (aykırı değer kapısı) geç.
2. **Hafif profille tekrar dene** — ORTA ya da HAFİF ayarla 6-8 uçuş.
   Dikey hizalama kazancı (0.54 → 0.21 m) gerçekti; daha az yavaşlayarak
   o kazancı taktik bedel ödemeden almak mümkün olabilir.

Karar kullanıcınındır.

---

## 8 · KULLANICININ KENDİ SINAMASI

```bash
cd ~/projects/avci_sim
bash scripts/kapat.sh && bash scripts/mkur.sh test
```

Panel → **🎚 AYAR KONSOLU** → **⭐ YAVAŞLAMA PROFİLİ** grubu.
`YAVASLAMA` düğmesi + `T_GO`, `KAPANMA_TABAN`, `KAPANMA_TAVAN`, `K_I_KAP`.

**Neye bakılacak:** hedefe 8 metrenin altında yaklaşırken aracın gözle
görülür şekilde yavaşlaması. Ve **hedefin kaçıp kaçmadığı** — asıl bedel
orada.
