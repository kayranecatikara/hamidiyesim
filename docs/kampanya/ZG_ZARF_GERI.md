# KAMPANYA ZG — DİKEY ZARFI ESKİ HALİNE DÖNDÜRME

**5 uçuş, daire (circle_xl), kaçamak yok · 2026-08-21 12:51 → 13:34**
Kullanıcı gözlemiyle başladı: *"dün normalde daire senaryosunda full
vuruyorduk ama bugün dikeyde salınım yaptığını gördüm."*

---

## 1 · KOL — `.parm` dosyasına DOKUNULMADI

Değerler sim ayağa kalktıktan sonra **panel API'sinden** yazıldı
(MAVLink `PARAM_SET`), her uçuşta geri okunarak teyit edildi:

| parametre | bugünkü | **ZG kolu (eski)** |
|---|---|---|
| `PSC_JERK_Z` | 40 | **5** |
| `WPNAV_ACCEL_Z` | 2000 | **800** |
| `WPNAV_SPEED_UP` | 2000 | **1200** |
| `WPNAV_SPEED_DN` | 1500 | **1000** |
| `Cfg.VZ_MAX` | 15 | **8** |

⚠ `WPNAV_ACCEL_Z`'nin yazılabilmesi, aynı gün C adımında kaydırıcı
aralığının [50,1500] → [50,3000] düzeltilmesi sayesinde mümkün oldu.
Eski aralıkla değer sessizce 1500'e kırpılacaktı.

## 2 · MEKANİZMA KAPISI (§5.1) — AÇIK, ETKİ BÜYÜK

| ölçüt | bugünkü zarf (daire, n=10) | **eski zarf (ZG, n=5)** |
|---|---|---|
| aracın dikey **tepe** hızı \|w\|max | **7.47 m/s** | **4.34 m/s** (−%42) |
| \|w\| p90 | 3.19 | 3.14 |
| \|vz\| komut tepesi | 8.77 | 7.31 |
| \|vz\| p90 | 4.61 | 6.30 |

`|vz|max` ZG01'de tam **8.00**'de oturdu — `VZ_MAX=8` tavanı yürürlükte.
**Kullanıcının gördüğü dikey salınım ölçülebilir biçimde yarıya indi.**
Bu bir kare-başı ölçüttür (binlerce örnek), istatistiksel gücü yüksek.

## 3 · İSABET — yön iyi, ama ANLAMLI DEĞİL

| uçuş | isabet | en yakın |
|---|---|---|
| ZG01_E | ✓ | 1.91 m |
| ZG02_E | ✓ | 1.77 m |
| ZG03_E | ✓ | 1.67 m |
| ZG04_E | ✓ | 1.37 m |
| ZG05_E | ✓ | 1.09 m |
| | **5/5** | |

### ⚠ ÖNCE BİR KARIŞTIRICI — kendi ilk hesabımı geri alıyorum

İlk hesapta "eski zarf 10/10 vs yeni 6/10, **p = 0.043, anlamlı**" çıktı.
**Bu yanlıştı.** Tarihsel eski-zarf uçuşları (YV, DK) **faz eşiği
düzeltmesinden ÖNCE** koşulmuştu; DM/GE/ZG ise sonra. İki değişken
karışıyordu.

**Doğru kıyas — ikisi de faz düzeltmeli, daire, kontrol kolu:**

| | isabet | Fisher tek yönlü |
|---|---|---|
| eski zarf (ZG, taze) | **5/5** | |
| yeni zarf (DM+GE) | **6/8** | **p = 0.359 — ANLAMLI DEĞİL** |

n bu kadarken isabet farkı gürültüden ayrılamıyor. **Kanıtlanan tek şey
salınımın azalması.**

### ⚠ "EN YAKIN" BU İŞTE KULLANILAMAZ (§5.3)

Telemetri 10 Hz, temas civarındaki kapanma 7-9 m/s → **örnekler arası
0.85-0.91 m yol**. Ölçütün kendi çözünürlüğü, aradığımız farkla aynı
mertebede. Nitekim kendi içinde çelişiyor:

* yeni zarfta **0.84 m**'lik bir yaklaşma **ISKA** (DM07_K)
* eski zarfta **1.91 m**'lik bir yaklaşma **İSABET** (ZG01_E)

Güvenilir sonuç ölçütü **çarpma sensörü** (`imha`), mesafe değil.

## 4 · VURUŞ KALİTESİ — ZG lehine DEĞİL, dürüstçe

`tools/vurus_kalitesi.py` (§4 sınıflandırması):

| zarf | KONTROLLÜ | ŞANS |
|---|---|---|
| eski (ZG, 5 isabet) | 1 | **4** |
| yeni (6 isabet) | 2 | 4 |

⚠ Ama ZG'nin dört ŞANS'ı da **TEK ölçütten** düştü: *"temas kesintisiz"*
(1-2 kopuk kare). Diğer beş ölçüt rahat geçiyor — hedef merkezden
10-11 px, kutu 2.0-3.0× büyüyor, yatış sakin, sıçrama yok. Yani bunlar
"savrulup şans eseri çarpma" değil; son anda 1-2 kare tespit kopması.
(§5.6: sınıflandırıcıyı kendi lehime yorumlamıyorum, eşiğin katı
olduğunu ayrıca not ediyorum.)

## 5 · ⭐ VİDEO BACAĞI — YENİ BİR SORUN GÖRÜNDÜ

ZG05_E (tek KONTROLLÜ vuruş), kare kare:

| kare | mesafe | görüntü |
|---|---|---|
| 128 | 4.5 m | hedef kadrajda |
| 129 | **3.3 m** | hedef **tam merkezde**, kutu net, conf 0.85 |
| 130 | 2.4 m | |
| 131 | **2.1 m** | **hedef kadrajın ALTINDAN ÇIKMIŞ** — yalnız gri
`ID:13 tahmin(1)` kutusu, yani Kalman kestirimi, alt kenarda |

Son 2 metrede hedef kamera kadrajının altına düşüyor. Kamera 25°
sabit (kullanıcı kuralı, değiştirilmez). Bu, GE09_K'nin ıskasında da
görülen zincirin aynısı — **zarftan bağımsız, ayrı bir sorun.** Dört
ZG uçuşunun ŞANS sınıfı da buradan geliyor.

## 6 · FİZİK DENETİMİ — `WPNAV_ACCEL_Z` için ayrı ve BAĞIMSIZ gerekçe

Kullanıcı itirazı: *"jerkin 40 olması hiç gerçekçi değil."*

| parametre | ArduPilot belgelenmiş aralığı | bizim değer | hüküm |
|---|---|---|---|
| `PSC_JERK_Z` (`AC_PosControl.cpp:321`) | **5–50 m/s³** | 40 | **aralık İÇİNDE** |
| `WPNAV_ACCEL_Z` (`AC_WPNav.cpp:67`) | **50–500 cm/s²** | 2000 | **tavanın 4 KATI** |

Ayrıca `MOT_THST_HOVER = 0.39` → itki/ağırlık = **2.56** → aracın
fiziksel dikey ivme tavanı **(2.56−1)·9.81 = 15.3 m/s²**.
`WPNAV_ACCEL_Z = 2000` = **20 m/s²** → **aracın üretemeyeceği ivme.**
`800` = 8 m/s² → fiziğin içinde.

⛔ **DL belgesindeki gerekçemi GERİ ALIYORUM.** Orada
*"itki/ağırlık 7.08 → 60 m/s² mevcut"* yazmışım; bu sayının kaynağı ne
`.parm`'da ne model dosyasında var. Parametreler 2.56 diyor.
`WPNAV_ACCEL_Z = 2000` yanlış bir sayıyla gerekçelendirilmiş.

**Sonuç:** jerk 40 gerçekçilik açısından savunulabilir (ArduPilot'un
kendi aralığında, gerçek FPV motoru daha da hızlı); **`WPNAV_ACCEL_Z
= 2000` savunulamaz.**

## 7 · AI ÖNERİSİ

**`WPNAV_ACCEL_Z` 2000 → 800: ölçümden BAĞIMSIZ olarak dönmeli.**
Belgelenmiş aralığın 4 katı, aracın itki kapasitesinin üstünde, ve
gerekçesi hatalı bir sayıydı.

**Zarfın geri kalanı (jerk, VZ_MAX, SPEED_UP/DN): dönmesini öneriyorum,
ama kanıt "salınım yarıya indi"dir, "daha çok vuruyor" DEĞİLDİR.**
İsabet farkı (5/5 vs 6/8) n bu kadarken kanıtlanamıyor.

⚠ **Kalıcı çare bu değil.** Eski zarfın iyi çalışması, aşırı büyük dikey
komutun araç tarafından SÜZÜLMESİNE dayanıyor. Gerçek FPV dronda o
süzme olmayacak. Asıl kusur dikey komutun ÖLÇEĞİ: `v_los`(18) fazla
büyük, `kapanma`(N=1) fazla küçük, aradaki `N·ṙ` (N=3-5) hiç
denenmedi.
