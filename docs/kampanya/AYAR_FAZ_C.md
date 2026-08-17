# AYAR KAMPANYASI — FAZ C: KOMUT ZARFI SEVİYESİ

> Faz A ve B'de altı parametre ailesi denendi, hiçbiri sonuç vermedi.
> Geriye tek sınanmamış kaldıraç kalmıştı: **zarfın kendisi.**
> 9 uçuş, `square` (sürekli manevra), n=3/kol, dönüşümlü · 2026-08-17

---

## 1 · TEK DEĞİŞKEN — üç seviye, tutarlı ölçekleme

| kol | `ANGLE_MAX` | `WPNAV_ACCEL` | `PSC_JERK_XY` |
|---|---|---|---|
| **ESKİ** | 45° | 8 m/s² | 12 |
| **ARA** | 55° | 15 m/s² | 20 |
| **YENİ** (bugünkü) | 70° | 26 m/s² | 40 |

Üçü tek fiziksel büyüklüğün üç yerdeki ifadesi olduğu için birlikte
ölçeklendi. Her koşuda araçtan geri okunarak teyit edildi (§5.1).

---

## 2 · SONUÇ — HİÇBİR KOL AYRIŞMADI

| kol | **\|dz\| (dikey ıska)** | **\|yatış\| p90** | en yakın menzil | kutu oranı |
|---|---|---|---|---|
| ESKİ 45°/8/12 | 3.39 m | 37.3° | 19.00 m | %42.4 |
| ARA 55°/15/20 | 3.29 m | 41.2° | 20.32 m | %42.8 |
| YENİ 70°/26/40 | 3.53 m | 38.7° | 20.07 m | %32.9 |

Permütasyon testi (n=3+3):

| ölçüt | yen–ara | yen–esk | ara–esk |
|---|---|---|---|
| \|dz\| | p=1.000 | p=0.600 | p=1.000 |
| \|yatış\| p90 | p=1.000 | p=0.600 | p=0.400 |
| en yakın menzil | p=0.600 | p=1.000 | p=0.600 |

**Dokuz kıyasın dokuzu da gürültü.** Zarf seviyesi ölçülebilir hiçbir fark
üretmiyor. İlan edilen geçerlilik eşinden (en yakın menzil, YENİ'nin
%125'i = 25.09 m) her iki kol da geçti — yani hiçbiri elenmedi de.

---

## 3 · ⭐ ASIL BULGU — DİKEY ISKA YENİ ZARFIN ÜRÜNÜ DEĞİL

Faz C'nin `square` sayıları eski araçla aynı ölçütle (telem.csv 10 Hz,
gerçek 3B mesafe) yan yana konduğunda:

| koşu | araç | en yakın | **\|dz\|** |
|---|---|---|---|
| D02_J10_kare | **ESKİ** | 2.04 m | **4.71 m** |
| D05_J10_kare | **ESKİ** | 4.82 m | **4.15 m** |
| D08_J10_kare | **ESKİ** | 4.05 m | **2.67 m** |
| D11_J10_kare | **ESKİ** | 3.73 m | **3.99 m** |
| SL01_K | **ESKİ** | 3.59 m | **3.42 m** |
| SL05_K | **ESKİ** | 3.87 m | **3.54 m** |
| TD01_esk | yeni itki | 4.61 m | 3.52 m |
| TD04_esk | yeni itki | 5.79 m | 3.39 m |
| TD03_yen | **YENİ tam** | 13.78 m | 3.53 m |
| TD06_yen | **YENİ tam** | 5.06 m | 3.63 m |

> **`square`'de dikey ıska ESKİ araçta da 2.67–4.71 m'ydi.**
> Yeni araçta 3.39–3.63 m — yani **aynı, hatta biraz daha dar.**

En yakın menzil: eski medyan **3.80 m**, yeni **5.42 m** — n=6 vs n=4 ve
saçılma (2.0–13.8 m) farkı yutuyor.

**Kullanıcının "hedefin altından/üstünden geçiyor" dediği davranış sürekli
manevrada ZATEN VARDI ve zarf büyütmesinin ürünü değil.** Üç fazın en
önemli bulgusu bu.

---

## 4 · ⚠ FAZ C'NİN SINIRI — "ESKİ" kolu TAM GERİ ALMA DEĞİLDİ

`kosuD2.sh` yalnız ArduPilot parametrelerini yazıyor. **Rotor itkisi
`iris_cam/model.sdf` içindeki `<area>` değeri** ve MAVLink'ten
değiştirilemez. Yani ESKİ kolunda:

| | geri alındı mı |
|---|---|
| `ANGLE_MAX`, `WPNAV_ACCEL`, `PSC_JERK_XY` | ✓ evet |
| **rotor itkisi (×2.5)** | ⛔ **HAYIR — hâlâ yeni** |
| `V_TERMINAL` 20, `DIKEY_ROLL` açık | ⛔ hayır (bilinçli: ölçülüp girmişlerdi) |

Bu yüzden Faz C, "zarf parametreleri" sorusunu cevaplar; "itki değişikliği"
sorusunu **cevaplamaz.** Tam geri alma isteniyorsa SDF de geri alınmalı ve
bu ayrı bir koşudur.

---

## 5 · ÜÇ FAZIN TOPLAMI — 25 uçuş, 7 parametre ailesi

| # | denenen | sonuç |
|---|---|---|
| 1 | AUTOTUNE | araç devrildi (`AngErr=106`) |
| 2 | `MOT_THST_HOVER` 0.39→0.14 | açı hatası 7.11° → **47.45°** |
| 3 | `ATC_ACCEL_R_MAX` 250k→110k | **83.71°** |
| 4 | `ATC_RAT_*_P` ×2 / ×4 | 23.45° / doyum %61 |
| 5 | itki modeli eşleşik üçlü | en yakın 0.4 → **11.5 m** |
| 6 | dikey bütçe `VZ_MAX` 3→8 | **p = 1.000** |
| 7 | `CY_NISAN` / seyir eğimi | hipotez **çürüdü** |
| 8 | **komut zarfı 3 seviye** | **9 kıyasın 9'u gürültü** |

**Hiçbiri işe yaramadı. En iyi yapılandırma, hiçbir şeyin değiştirilmediği
hâl.** Depo tabanda bırakıldı.

### Ölçülen gerçek durum

| | ESKİ araç | YENİ zarf |
|---|---|---|
| iç döngü açı hatası p90 | 7.68–15.54° | **7.11°** ✓ aynı/daha iyi |
| `duz`+kaçamak en yakın | ~1.5 m | **0.40–0.90 m** ✓ daha iyi |
| `square` \|dz\| | 2.67–4.71 m | 3.39–3.63 m ✓ aynı |
| `square` en yakın | 3.80 m | 5.42 m ≈ aynı (saçılma içinde) |
| \|yatış\| p90 | 20–36° | 37–48° ✗ tek gerçek fark |

**Yeni araç ölçülebilir hiçbir ölçütte "kontrolsüz" değil.** Tek ayrışan
şey yatış genliği — ve Faz C onu zarfla ilişkilendirmeyi de başaramadı
(ESKİ kolu 37.3°, YENİ 38.7°, p=0.600).

---

## 6 · SIRADAKİ ADIM — parametre değil, GÜDÜM YASASI

Dikey ıska hem eski hem yeni araçta 2.7–4.7 m; isabet zarfı **+0.29/−0.13 m**.
Yani dikey hata zarfın **10–30 katı** ve bu **parametre ayarıyla
düzelmiyor** — 25 uçuş bunu gösterdi.

Geriye kalan tek yer güdümün **dikey yasası**:
`vz = −v_dikey · tan(nişan_elev)` + `K_VZ_D` türev sönümlemesi.
Bu yasa hiç sınanmadı; yatay kanal (Ö5, Ö8, Ö9, S1/S3, V_TERMINAL) onlarca
uçuşla ölçülürken dikey kanala yalnız T1b (roll telafisi) dokunuldu.

**Öneri:** Faz D — dikey güdüm yasası. Ama bu bir güdüm değişikliğidir,
§1 gereği kullanıcı onayı gerekir.
