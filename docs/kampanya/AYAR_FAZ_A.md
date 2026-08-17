# AYAR KAMPANYASI — FAZ A: İÇ DÖNGÜ (duruş/hız denetleyicisi)

> Kullanıcı: *"bu aracın yapısını değiştirmemiz aracı kontrolsüz hale
> getirdi, kontrolünü sağlamak için düzgün parametre ayarlarını bulmamız
> lazım."* · 2026-08-17

---

## 1 · ÖLÇÜM ALETİ

**ArduPilot dataflash (BIN), `RATE` + `ATT` mesajları.** Güdüm logu 20 Hz ve
iç döngü salınımı 2-5 Hz mertebesinde — §5.3 gereği 20 Hz ile ölçülemez.
BIN'de komut edilen ve gerçekleşen açısal hız yüksek hızda yan yana.

Araç: `tools/durus_kalitesi.py`.
**Segment:** yalnız irtifa > 25 m olan kesim (kalkış/iniş/çarpma dışarıda).

---

## 2 · AUTOTUNE DENENDİ — ARAÇ DÜŞTÜ

İlk plan ArduPilot'un kendi AUTOTUNE'uydu (`tools/autotune_kos.py`).
GUIDED → arm → 40 m kalkış → LOITER → AUTOTUNE dizisi koşuldu ve araç
**tune sırasında devrildi**:

```
AP: Crash: Disarming: AngErr=106>30, Accel=0.8<3.0
```

Açı hatası 106°'ye çıkmış, ArduPilot'un kendi çarpma dedektörü disarm etmiş.
**AUTOTUNE bu araçta koşulamıyor** → ilan edilen yedeğe (manuel tarama)
geçildi.

---

## 3 · MANUEL TARAMA — 5 uçuş, `square`, 180 s

Her adımda TEK değişken; kazanan bir sonraki adıma taşınır.

| log | `MOT_THST_HOVER` | `ATC_ACCEL_R_MAX` | `ATC_RAT_*_P` | hız hatası p90 | **açı hatası p90** | çıkış p90 | doyum |
|---|---|---|---|---|---|---|---|
| 235 **TABAN** | 0.39 | 250000 | 0.054 | 126.9 °/s | **7.11°** | 0.310 | %0.3 |
| 237 | **0.14** | 250000 | 0.054 | 138.2 | **47.45°** | 0.254 | %0.3 |
| 239 | 0.14 | **110000** | 0.054 | 201.5 | **83.71°** | 0.324 | %1.5 |
| 241 | 0.14 | 110000 | **0.108** | 184.7 | 23.45° | 0.850 | %5.1 |
| 243 | 0.14 | 110000 | **0.216** | 223.6 | 21.04° | 1.926 | **%61.3** |

**DÖRT DEĞİŞİKLİĞİN DÖRDÜ DE TABANDAN KÖTÜ.** En iyi hâl, hiçbir şeyin
değiştirilmediği hâl.

### 3.1 · Hipotezim yanlış çıktı — açıkça

`MOT_THST_HOVER` 0.39 iken gerçek hover gazının **0.141** olduğunu ölçmüş ve
"denetleyicinin iç modeli 2.8 kat yanlış, düzeltirsek toparlanır" demiştim.
Düzeltince açı hatası **7.11° → 47.45°**, yani **altı kat kötüleşti.**
Hipotez yanlıştı ve gizlenmiyor.

### 3.2 · Kazanç yükseltmek de çözmedi

`P` 0.054 → 0.108 açı hatasını 23.45°'ye çekiyor ama **taban 7.11°'in hâlâ
üç katı**. 0.216'da çıkış **%61.3 oranında doyuyor** — denetleyici rayda,
kullanılamaz.

---

## 4 · ⭐ ASIL BULGU — İLAN ETTİĞİM KAPI YANLIŞTI

Faz A'nın kapısını *"hız hatası p90 < 25 °/s"* diye ilan etmiştim. Bunu
ders kitabı beklentisinden yazdım, **eski aracı ölçmeden.** Ölçtüm:

| araç | log | `ATC_RAT_*_P` | hız hatası p90 | **açı hatası p90** | \|yatış\| p90 |
|---|---|---|---|---|---|
| **ESKİ** (ANGLE_MAX 45°) | 151 | 0.135 | 78.8 °/s | **7.68°** | 22.7° |
| **ESKİ** | 153 | 0.135 | 111.3 | **15.54°** | 20.7° |
| **ESKİ** | 154 | 0.135 | 84.4 | **15.13°** | 36.4° |
| **YENİ, taban** | 235 | 0.054 | 126.9 | **7.11°** | 40.8° |

> **Kullanıcının memnun olduğu ESKİ araç da hiçbir zaman 25 °/s'ye yakın
> değildi (79-111 °/s), ve açı hatası 7.7-15.5°'ydi.**
> Yeni aracın tabanı (7.11°) eski aracın EN İYİSİ kadar, ortalamasından
> DAHA İYİ.

**Yani iç döngü, kullanıcının şikâyet ettiği kararsızlığın sebebi değil.**
Kapı ulaşılamaz bir yere konmuştu; hatayı kabul ediyorum.

---

## 5 · O HÂLDE NE DEĞİŞTİ?

Aynı tabloda ayrışan tek sütun **\|yatış\| p90**: eski araç 20-36°, yeni araç
**40.8°** (kullanıcının uçuşunda terminalde medyan 36.9°, p90 46.5°, tepe 68°).

Değişenler iç döngü değil, **dışarıdan verilen komut zarfı**:

| | eski | yeni | kat |
|---|---|---|---|
| `ANGLE_MAX` | 45° | 70° | 1.6× |
| `WPNAV_ACCEL` | 8.0 m/s² | 26 m/s² | 3.3× |
| `PSC_JERK_XY` | 12 | 40 | 3.3× |
| itki/ağırlık | 2.56 | 7.08 | 2.8× |

Araç kendini eskisinden **2-3 kat sert** savuruyor. Kullanıcının gördüğü
"dengesizce hareket" bu; denetleyicinin hedefini ıskalaması değil, hedefin
kendisinin çok agresif olması.

---

## 6 · FAZ A KARARI

- **İç döngüde değiştirilecek bir şey BULUNAMADI.** Taban en iyisi;
  denenen dört ayarın dördü de kötüleştirdi. Depo tabanda bırakıldı
  (`ATC_RAT_*_P/I = 0.054`, `_D = 0.00144`, `ATC_ACCEL_R/P_MAX = 250000`).
- **İlan edilen kapı (< 25 °/s) GEÇİLEMEDİ** — ama kapının kendisi
  hatalıydı: eski araç da geçemiyordu. Bu bir başarısızlık değil, ölçüt
  hatasının düzeltilmesidir.
- **Faz B'nin konusu değişmeli.** Dikey kanalı ayarlamadan önce asıl soru
  şu: komut zarfı (70° / 26 m/s² / jerk 40) fazla mı? Çevikliği tamamen
  kaybetmeden ne kadar geri çekilmeli?

**Faz B için önerilen tek değişken:** `ANGLE_MAX` ve `WPNAV_ACCEL` birlikte,
eski (45°/8) ile yeni (70°/26) arasında ara seviyeler — 55°/15 gibi.
Kullanıcının onayı beklenecek.

---

## 7 · ARTEFAKTLAR

- `tools/durus_kalitesi.py` — BIN'den iç döngü kalitesi (bu kampanyanın aleti)
- `tools/autotune_kos.py` — AUTOTUNE denemesi (araç düştü, kayda geçsin)
- `~/.avci_sim/kosuA.sh` — tek değişkenli iç döngü koşusu, param geri
  okuyarak teyit eder (§5.1)
- Uçuşlar: `logs/kacamak/TA01_taban` … `TA05_P4x`
