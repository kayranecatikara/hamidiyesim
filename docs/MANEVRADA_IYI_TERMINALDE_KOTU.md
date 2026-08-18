# MANEVRADA İYİ, TERMİNALDE KÖTÜ HAL — 2026-08-18

> **Etiket:** `manevrada-iyi-terminalde-kotu` → commit `da1f1a2`
> **Geri dönüş:** `git checkout manevrada-iyi-terminalde-kotu`

Kullanıcının paneldeki **① ② ④ ⑤ açık, ③ kapalı** hâli. Bu, açılış
varsayılanının ta kendisidir — `scripts/mkur.sh` ile kurulan sistem
hiçbir düğmeye dokunmadan bu konumda başlar.

Adı, sistemin ölçülmüş karakterini birebir tarif ediyor: **hedefin
manevralarına verilen tepki iyi, bitiriş kötü.** Kullanıcının kendi
cümlesiyle: *"tüm paslar okey, bitiriş çok kötü."*

---

## 1 · PANEL — açılışta ne açık

| # | düğme | alan | açılış | anlamı |
|---|---|---|---|---|
| ① | DİKEY HİZALAMA KAPISI | `DIKEY_KAPI_M` | **AÇIK** = 2.0 m | irtifa eşitlenmeden terminale geçilmez |
| ② | DİKEY GÜÇ | `VZ_MAX` | **AÇIK** = 8 m/s | dikey hız tavanı (3'ten açıldı) |
| ③ | TERMİNAL HIZI | `V_TERMINAL` | **KAPALI** = 16 m/s | kullanıcı 20'den geri aldı |
| ④ | TERMİNALE GEÇİŞ ANI | `TERMINAL_BOYUT` | **AÇIK** = 18 px ≈ 8.9 m | erken geçiş |
| ⑤ | T1b DİKEY ROLL TELAFİSİ | `DIKEY_ROLL` | **AÇIK** | yatışta cy düzeltmesi |

⚠ ① ve ② **birlikte** çalışır: ② olmadan ①'in kapatacağı irtifa bütçeye
sığmaz. ① ise ⑤'e dayanır (dikey ofset ölçümü ondan gelir).

## 2 · GÜDÜM (`control/guidance/bbox_ibvs.py`)

| alan | değer | | alan | değer |
|---|---|---|---|---|
| `CY_NISAN` | ≈300 px | | `V_TERMINAL` | 16.0 m/s |
| `K_YAW` | 1.0 | | `V_TERM_MIN` | 10.0 m/s |
| `YAW_RATE_MAX_DEG` | 120 °/s | | `V_TOPLAM_MAX` | 24.0 m/s |
| `K_VZ` | 0.5 | | `TERMINAL_BOYUT` | 18.0 px |
| `K_VZ_D` | 0.6 | | `TERMINAL_SURE` | 2.0 s |
| `VZ_MAX` | 8.0 m/s | | `TERM_BIRAK_M` | 20.0 m |
| `VZ_MAX_TERM` | 10.0 m/s | | `LEAD_SURE` | 0.4 s |
| `DIKEY_KAPI_M` | 2.0 m | | `LEAD_SONUM` | açık |
| `BOYUT_REF` | 25.0 px | | `KAPANMA` | açık |
| `K_FWD` | 0.35 | | `KAPANMA_MIN` | 1.5 m/s |
| `K_I` | 0.04 | | `KAPANMA_EMA` | 0.20 |
| `MAX_ACCEL` | 12.0 m/s² | | `CONF_MIN` | 0.35 |

**Kapalı olanlar:** `KACIS_KD`=0, `YANAL_K`=0, `SONUM_T`=0, `DONUS_A`=0,
`YAW_MENZIL_REF`=0, `LEAD_ERKEN`=kapalı.

**Supervisor:** `KILIT_N`=10, `KAYIP_M`=20, `POSE_CONF_MIN`=0.0,
`GATE_KILIT`=kapalı.

## 3 · ARAÇ (`sim/ardupilot_params/avci_copter.parm`)

`ANGLE_MAX` 7000 (70°) · `WPNAV_SPEED` 2500 · `WPNAV_ACCEL` 2600 ·
`PSC_JERK_XY` 40 · `WPNAV_SPEED_UP/DN` 1200/1000 · `WPNAV_ACCEL_Z` 800 ·
`ATC_RAT_RLL_P/I` 0.054 · `_D` 0.00144 · `ATC_ACCEL_R_MAX` 250000 ·
`MOT_THST_HOVER` 0.39.

**Model:** `iris_cam/model.sdf` rotor `LiftDrag <area>` = **0.005**
(0.002'den ×2.5) → itki/ağırlık 2.56 → **7.08**.

**Kamera:** gövdeye **25° yukarı** sabit vidalı. Değişmez (kullanıcı kuralı).

---

## 4 · ⭐ ÇALIŞAN KISIM — "manevrada iyi"

`logs/kayit/ucus_20260818_113552` (172 kare, 171 s, **her kareye tek tek
bakıldı**) + 3525 kare 20 Hz güdüm logu:

**Yanal kesişme ÇÖZÜLMÜŞ.** Iska vektörü hedefin kendi çerçevesinde
ayrıştırıldığında 8 yaklaşmanın **8'inde de** yanal sapma −0.73…+0.39 m —
isabet zarfının (±0.65 m) içinde ya da kıyısında.

**Görsel temas 5-20 m bandında %89.**

**Terminale giriş geometrisi kusursuz:** dikey ofset #64'te +0.03 m,
#138'de −0.01 m, #111'de +0.27 m. Eski (2026-08-02) "terminale +4.65 m
ofsetle giriliyor" sorunu **çözülmüş durumda** — ① + ② + ⑤ birlikte iş
görüyor.

Kullanıcının cümlesi: *"baş kısımlarda hedefin yaptığı manevralara
verdiğimiz reaksiyon aşırı iyi, çok çok iyi, salınım falan da yok."*

---

## 5 · ⛔ BOZUK KISIM — "terminalde kötü"

Yedi sorun ölçüldü; tamamı `docs/kampanya/SORUN_ENVANTERI.md`'de.
Bitirişi öldüren ilk ikisi:

### S1 · Terminal freni → burun kalkıyor → hedef kadrajdan çıkıyor

```
seyir 19.4 m/s → terminal girişi v:=16.0  (ani −3.4 m/s basamak)
  → quad frene basar → BURUN +27° kalkar
  → kamera zaten 25° yukarı vidalı → eksen ~52° yukarı
  → hedef kadrajın ALTINDAN çıkar (cy 305 → 471; alt kenar 480)
  → kör kalınır, donmuş komut TIRMANMA (ölçüldü −10 m/s)
  → hedefin 1.4-2.8 m ÜSTÜNDEN geçilir
```

Aynı uçuş, aynı ayar, tek fark girişte hız basamağı var mı:

| terminale giriş | n | pitch tepe | cy tepe | kadraj dışı |
|---|---|---|---|---|
| seyir **> 18 m/s** (fren) | 8 | **26.1°** | **456** | **5/8 = %62** |
| seyir **= 16 m/s** (fren yok) | 30 | 2.5° | 319 | 1/30 = %3 |

Geometrik sınır: hedef seviyedeyken **pitch +27°'de** kadrajdan çıkıyor.
Ölçülen fren pitch'i 21-29°. Tam sınırın üstünde uçuluyor.

### S2 · Terminalde kapanma hızı 0.9 m/s

`V_TERMINAL` 16.0 − hedef 15.1 = **0.90 m/s**. 6 m kapatmak **6.7 s**.
Ölçüldü: araç 8 saniye boyunca 5.7-6.2 m'de asılı kaldı (kareler
#73-#82), güven 0.81-0.89, hedef hep kadrajda, **hiç yaklaşamadı**.
8/8 yaklaşmada en yakın anda boyuna **−2.6…−7.4 m** — dikey ve yanal
mükemmel olsa bile hedefin arkasında kalınıyor.

### Diğerleri (özet)

| # | sorun | ölçü |
|---|---|---|
| S3 | 0-5 m'de görsel temas | %89 → **%50** |
| S4 | dikey kanal salınımı | `vz` işaret 1.62/s, \|vz\| p90 8.0 m/s |
| S5 | faz zıplaması | 14 kez / 171 s; güdüm logu 9 kez yeniden açıldı |
| S6 | ıska sonrası toparlanma | 12-37 s; **171 s'nin ~110'u** |
| S7 | yatış tepesi | 41° |

---

## 6 · BİLİNEN ÇÖZÜM ADAYI — ölçüldü ama SİSTEMDE YOK

`docs/kampanya/H_TERMINAL_HIZ.md` — 16 uçuş, `duz` senaryosunda n=6/kol.
**D2 (terminalde fren yok)** S1'in zincirini kırdı:

| ölçüt | bu hâl | D2 açık | p |
|---|---|---|---|
| kadraj dışı oranı | %73 | **%0** | **0.039** |
| cy tepesi | 467 | **348** | **0.026** |
| terminal süresi | 26.7 s | **3.0 s** | **0.013** |
| koşunun en yakını | 1.75 m | **1.20 m** | **0.030** |
| \|yatış\| p90 (son 3 s) | 29.4° | **4.2°** | **0.039** |
| isabet | 3/6 | **5/6** | |

**Ama kararı verilmedi** ve §0.2 gereği panelde birikemezdi → koddan
çıkarıldı. Bedeli ölçülmüştü: 3 m'nin içinde \|dikey\| 0.21 → 1.06 m
(daha hızlı gelindiği için dikey kanala oturma süresi kalmıyor).

Kod gerekirse `aeb0ddd` commit'inden alınır.

---

## 7 · BU HÂLE NASIL DÖNÜLÜR

```bash
git checkout manevrada-iyi-terminalde-kotu
bash scripts/kapat.sh
bash scripts/mkur.sh test
```

Panel http://127.0.0.1:8000 · hiçbir düğmeye dokunma; ① ② ④ ⑤ açık,
③ kapalı gelir. Kayıt **0.5 s/kare**.
