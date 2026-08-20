# KAMPANYA DK — DİKEY KOMUT ÖLÇEĞİ

**Tek değişken:** `AVCI_IBVS_DIKEY_KAPANMA` 0/1 · 8 uçuş, n=4/kol
(her kolda 2× `circle_xl` + 1× `duz`+`yatay` + 1× `duz`+`yok`, §5.9 eşit),
dönüşümlü · `YAVASLAMA` **iki kolda da KAPALI** · 2026-08-20

---

## 1 · HİPOTEZ (AI'nın önerisi)

Kullanıcı gözlemi: *"son anlarda dikeyde hizaya gelmeye çalışılırken çok
salınım oluyor, alttan üstten kaçırılıyor."*

AI teşhisi: dikey komut `vz = v_los·sin(ε)` ile hesaplanıyor; bu **durgun
hedef** için doğru. Hedef 15 m/s ile kaçarken gerçek kapanma 1.5-3 m/s;
`d` metrelik ofseti `t_go = R/ṙ` sürede kapatmak için gereken
`vz = ṙ·sin(ε)`. Oran ~11 kat.

Çevrimdışı ölçüm (kullanıcı uçuşu 20260820_124706, 251 kare) bunu
destekliyordu: komut/gereken oranı 0-3 m'de **18.6 kat**.

---

## 2 · MEKANİZMA KAPISI — GEÇTİ

| koşu | `dikey_olcek` medyan | `v_los` medyan | oran |
|---|---|---|---|
| DK01_K (kapalı) | 18.00 | 18.00 | 1.00 |
| DK02_A (açık) | **1.50** | 18.00 | **0.08** |

Dikey komut medyanı **1.31 → 0.73**, p90 **4.32 → 2.15**. Özellik tam
istendiği gibi çalıştı.

---

## 3 · ⛔ SONUÇ — HİPOTEZ ÇÜRÜDÜ

| ölçüt | KAPALI | AÇIK | p | |
|---|---|---|---|---|
| **İSABET** | **4/4** | **1/4** | | ✗ |
| **\|dikey\| temasta** | **0.08 m** | **0.59 m** | **0.086** | ✗ |
| **faz düşüşü** | **3.5** | **14.5** | **0.057** | ✗ |
| savrulma son 3 m (BİRİNCİL) | **0.36** | 1.70 | 0.371 | ✗ |
| ilk temasa süre | 58.7 s | 118.7 s | 0.171 | ✗ |
| \|vz\| medyan | 1.31 | 0.73 | 0.229 | (mekanizma) |
| \|vz\| p90 | 4.32 | 2.15 | 0.171 | (mekanizma) |

| koşu | senaryo | imha | en yakın | savrulma | \|dz\| temasta |
|---|---|---|---|---|---|
| DK01_K | daire | ✓ | 1.55 | 2.44 | 0.03 |
| DK03_K | daire | ✓ | 0.92 | 0.40 | 0.07 |
| DK05_K | düz+yatay | ✓ | 1.92 | 0.31 | 0.09 |
| DK07_K | düz+yok | ✓ | 1.78 | 0.21 | 0.18 |
| DK02_A | daire | ✗ | 2.40 | 1.91 | **0.65** |
| DK04_A | daire | ✗ | 3.44 | 0.00 | **2.00** |
| **DK06_A** | düz+yatay | ✓ | **0.39** | 0.35 | 0.09 |
| DK08_A | düz+yok | ✗ | 1.04 | 1.70 | **0.53** |

**Senaryo içinde (§5.9):**
- `daire`: KAPALI **2/2** [1.55, 0.92] · AÇIK **0/2** [2.40, 3.44]
- `duz`: KAPALI **2/2** [1.92, 1.78] · AÇIK **1/2** [**0.39**, 1.04]

---

## 4 · ⚠ AI HİPOTEZİ NEDEN YANLIŞTI

Birincil ölçüt (savrulma) **ters yöne gitti**: 0.36 → 1.70. Ve asıl hedef
olan temas anındaki dikey hizalama **7 KAT KÖTÜLEŞTİ** (0.08 → 0.59 m).

**Eksik varsayım: aracın komutu ANINDA uyguladığı kabul edildi.** Ölçüldü —
uygulamıyor:

| koşu | komut \|vz\| | gerçekleşen \|vz\| | takip oranı |
|---|---|---|---|
| DK01_K | 2.01 | 1.40 | **0.70** |
| DK03_K | 1.39 | 1.24 | 0.89 |
| DK04_A | 1.15 | 0.98 | 0.85 |

`vz = d/t_go` hesabı "komut anında hız olur" varsayar. ArduPilot dikey hızı
`WPNAV_ACCEL_Z` ile rampalar; araç komutun ancak %70-89'unu gerçekleştiriyor
ve gecikmeyle. **Komuttaki "fazlalık" boşuna değildi — aracın dikey
gecikmesini telafi ediyordu.** 12 kat kısınca dikey kanal ofseti hiç
kapatamaz oldu.

> Bu, `docs/` altındaki eski bir dersin tekrarı: *"araç dikey komutu 4 s'de
> uyguluyor"* (bkz. hafıza: dikey döngü kök nedeni). Analitik olarak doğru
> görünen bir düzeltme, aracın gerçek dinamiğini hesaba katmayınca çöktü.

---

## 5 · KARAR KURALI DENETİMİ

| # | kural | sonuç |
|---|---|---|
| 1 | son 3 m'deki savrulma azalır | ✗ **0.36 → 1.70** |
| 2 | isabet kötüleşmez | ✗ **4/4 → 1/4** |
| 3 | en yakın menzil kötüleşmez | ~ 1.67 → 1.72 (p=0.971, nötr) |

**İki kural açıkça düştü. ÖZELLİK GİRMEZ.**

---

## 6 · YİNE DE BİR ŞEY DURUYOR

**DK06_A** (`duz`+`yatay`) **0.39 m ile vurdu** — kampanyanın en iyi
sonucu. Ve `duz` senaryosunda açık kolun en yakınları [0.39, 1.04],
kapalının [1.92, 1.78]'inden **daha yakın**.

Yani: düz uçan hedefte küçük dikey ölçek işe yarayabiliyor; dairede
(hedef sürekli yatık, dikey geometri hızlı değişiyor) çöküyor.

**Aradaki bir ayar denenmemiş durumda:** ölçek şu an ya `v_los` (18) ya
`kapanma` (1.5) — 12 kat sıçrama. Kazanç çarpanlı bir ara değer
(`N·ṙ`, N≈3-5 — klasik orantısal seyrüseferin yaptığı) sınanmadı.

⚠ Bu bir **öneri değil, gözlem**. Yeni kampanya kullanıcının kararıdır.

---

## 7 · AI ÖNERİSİ

**GİRMESİN.** Varsayılan `DIKEY_KAPANMA = KAPALI` kalır (değiştirilmedi).

Kullanıcının **gözlemi doğruydu** (son metrelerde salınım var, ölçüldü);
**AI'nın çaresi yanlıştı**. Salınımın sebebi "aşırı komut" değil —
komut, aracın dikey gecikmesini telafi etmek için zaten gerekli
büyüklükteydi.

Kod ve kill-switch duruyor; `DIKEY_KAP_TABAN` konsoldan ayarlanabilir.

---

## 8 · KULLANICININ KENDİ SINAMASI

```bash
cd ~/projects/avci_sim
bash scripts/kapat.sh && bash scripts/mkur.sh test
```

Panel → **🎚 AYAR KONSOLU** → **⭐ DİKEY KOMUT ÖLÇEĞİ** grubu.
`DIKEY_KAPANMA` düğmesi + `DIKEY_KAP_TABAN`.

**Neye bakılacak:** açıkken aracın dikeyde **tembelleştiği** — hedef
yukarıda/aşağıdayken tırmanma/alçalma isteksiz. Daire senaryosunda
belirgin.
