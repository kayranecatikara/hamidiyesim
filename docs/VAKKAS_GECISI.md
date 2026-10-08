# VAKKAS GEÇİŞİ — simülasyon aracının yeniden inşası

**Tarih:** 2026-09-06 · **Dal:** `kayramin_super_gudumu`
**Ham veri:** `logs/vakkas/` · **Kareler:** `logs/vakkas/taret/`, `logs/vakkas/taret_son/`

Teknofest süreci bitti; F4 uçuş kartlı, çubuk (yaw/pitch/roll/throttle) sürülen
drone dönemi kapandı. Sıradaki araç **VAKKAS**: baş kısmında ağ fırlatma
mekanizmalı taret taşıyan bir önleyici (interceptor) platformu.

Bu belge, simdeki `iris` aracının VAKKAS'a göre yeniden ayarlanmasını ve
**ölçülen** sonuçları kaydeder. VAKKAS'ın 3B modeli geldiğinde gövde geometrisi
de değişecek; bu adım yalnız **kütle · itki · taret · kamera** kapsıyor.

---

## 1 · VAKKAS'ın beyan edilen özellikleri

| bileşen | değer | simde nasıl temsil edildi |
|---|---|---|
| uçuş kontrol kartı | Orange Cube | ArduCopter SITL (zaten kullanılıyordu) |
| GPS | HERE 4 | SITL'in kendi GPS modeli |
| uçuş bilgisayarı | NVIDIA Jetson Orin D131 | görüntü hattını koşan makine |
| kamera | Sony IMX477, 12.3 MP | taret üstünde, namluyla aynı doğrultuda |
| kütle | ~4 kg | **ölçüldü: 4.0000 kg** |
| maks. itki | 8 kgf | **ölçüldü: 7.96 kgf** |

---

## 2 · ÖLÇÜM — önce/sonra

Yöntem: `tools/asili_gaz_olc.py`. Asılı uçuşta motor PWM'i okunur;
ArduPilotPlugin PWM'i `u = (PWM−1100)/800` ile normalize eder ve rotor devrini
buna orantılı sürer. Pervane itkisi devrin karesiyle arttığı için
**T_max = m·g / u_asılı²**. Bu formül ArduPilot'un iç itki modelinden
(MOT_THST_EXPO / SPIN_MIN / SPIN_MAX) tamamen bağımsızdır.

| | ESKİ sim aracı | VAKKAS | koşu |
|---|---|---|---|
| kütle | 1.75 kg | **4.00 kg** | `tools/sdf_kutle.py` |
| asılı motor PWM | 1358 µs | **1667 µs** | T01 / T03 |
| normalize devir `u` | 0.323 | **0.709** | |
| maks. itki | 16.76 kgf | **7.96 kgf** | |
| **itki/ağırlık** | **9.58** | **1.99** | |

⚠ Parametre dosyasının eski yorumları itki/ağırlık için "6.4" ve "7.08"
diyordu — **ikisi de yanlıştı**, gerçek 9.58'di. Zarf, sanılandan da güçlü
bir araca göre açılmıştı.

### Kütle bütçesi (toplam tam 4.0000 kg)
| link | kg |
|---|---|
| `base_link` (gövde + Orange Cube + HERE4 + Jetson + batarya) | 3.40 |
| `imu_link` | 0.15 |
| 4 × rotor | 0.10 |
| taret (kaide + pan + tilt + namlu) | 0.35 |

Atalet tensörü geometri değişmediği için kütleyle **doğrusal** ölçeklendi
(×2.267). ⚠ Gerçek 4 kg'lık bir 550-sınıfı gövdenin Ixx'i tipik 0.05-0.08
kg·m² olur; buradaki 0.018 muhtemelen düşük kalıyor. VAKKAS'ın 3B modeliyle
yeniden hesaplanmalı.

---

## 3 · TARET

İki eksenli, gövdenin üstünde, merkezde.

| eksen | eklem | sınır | not |
|---|---|---|---|
| PAN (sağa-sola) | `turret_yaw_joint`, Z ekseni | −180° … +180° | 0 = burun yönü |
| TILT (yukarı-aşağı) | `turret_pitch_joint`, −Y ekseni | −30° … +90° | + = yukarı |

Varsayılan duruş **pan 0 / tilt +25°** — eski gövdeye-sabit kameranın baktığı
yönün aynısı. Böylece taret bir **üst küme**dir: mevcut güdüm ve tespit
davranışı değişmez, üstüne dönme yeteneği gelir.

Komut:
```bash
gz topic -t /model/iris_with_ardupilot/joint/turret_yaw_joint/0/cmd_pos \
         -m gz.msgs.Double -p 'data: 0.5'      # pan, radyan
gz topic -t /model/iris_with_ardupilot/joint/turret_pitch_joint/0/cmd_pos \
         -m gz.msgs.Double -p 'data: 0.4363'   # tilt, radyan
```

### Ölçüm (`tools/taret_dene.py`, 9 duruş)
En büyük açı hatası **0.51°**. Kameranın taretle döndüğü kareler
`logs/vakkas/taret_son/` altında; tilt +90'da tam gökyüzü ve altta kendi
pervanesi, ±90 pan'da yandaki pervaneler, +180'de arkadaki güneş görünüyor.

### ⛔ İlk devşirilen kazançlar aracı UÇURTMUYORDU
`vakkas-entegre` dalından (2fefa04) alınan taretin PID kazançları
p=30 / i=25 / d=1.5 / i_max=15 idi. Eklem ataleti 3.3e-4 kg·m² olduğu için
i_max=15 N·m **45 000 rad/s²** demek; eklem 6 rad/s hız sınırında limit
çevrimine giriyordu. Titreşim gövdeye geçip ivmeölçerleri 0.5-0.8 g
salladı, ArduPilot `PreArm: Accels inconsistent` deyip ARM ETMEDİ.

Yeniden boyutlandırıldı (ω_n = 40 rad/s, ζ = 0.9): **Kp 0.5 · Kd 0.004**,
integral yalnız yerçekimi torkunu (0.041 N·m) yenecek kadar.
Titreşim **0.000 g**'ye indi.

---

## 4 · KAMERA — Waveshare IMX477-160 (ürün kodu 22232)

Taretin tilt linkinde, **namluyla aynı doğrultuda**. Boresight matematiksel
olarak doğrulandı: `namlu_ekstrinsik()` ile `kamera_ekstrinsik()` her taret
açısında **birebir aynı** yön vektörünü veriyor (tek fark 3.5 cm dikey ofset
= 100 m'de 0.02° paralaks). Topic **`iris_cam/image` aynı kaldı**.

### 4.1 · 160° neyin açısı — ve niye 128° kullanıyoruz

Bu tip M12 lenslerde ilan edilen değer **köşegen** FOV'dur. Ve 160°'lik bir
lens **zorunlu olarak balıkgözdür**: düz izdüşümde (rectilinear, `r = f·tan θ`)
kenar büyütmesi 180°'de sonsuza gider, öyle bir lens yapılamaz. Balıkgözün
standardı **eş uzaklık** izdüşümüdür: `r = f·θ` — açı yarıçapla DOĞRU ORANTILI.

Bu orantı sayesinde FOV'lar sensör kenar oranlarıyla ölçeklenir:

| eksen | sensör | oran | FOV |
|---|---|---|---|
| köşegen | 9.823 mm | 1.000 | **160.0°** (ilan edilen) |
| yatay | 7.857 mm | 0.800 | **128.0°** ← kullanılan |
| dikey | 5.895 mm | 0.600 | **96.0°** |

⚠ Yanlışlıkla düz izdüşüm varsayılsaydı yatay 155.1° çıkardı — **27° hata**.

Simdeki `<camera>` Gazebo'nun DÜZ İZDÜŞÜMLÜ kamerasıdır: aynı yatay açıyı
kapsar ama balıkgözün kenar bozulmasını üretmez. Bu **kasıtlıdır**: güdüm
zaten düz izdüşüm modeli (FX) varsayıyor ve gerçek boru hattında da balıkgöz
düzeltmesi (undistort) planlanıyor. Yani düz izdüşüm, İŞLENMİŞ görüntünün
doğru modelidir. Gerçek balıkgöz gerekirse Gazebo destekliyor
(`<sensor type="wideanglecamera">` + `<lens><type>equidistant</type>`), ama o
zaman `R = FX·S/p` formülü geçersiz olur.

### 4.2 · Çözünürlük: 1352 × 1014 — İKİ ölçümle seçildi

Sensör 4056×3040; bu onun tam **1/3**'ü (her iki eksende).

**Ölçüm 1 — Gazebo render maliyeti** (`tools/kamera_olcek_dene.py`):

| çözünürlük | MP | kare/s | RTF |
|---|---|---|---|
| 640×480 | 0.31 | 23.4 | 0.878 |
| 1280×960 | 1.2 | 23.7 | 0.851 |
| 2028×1520 | 3.1 | 23.5 | 0.830 |
| 4056×3040 | 12.3 | 15.0 | **0.474** ⛔ |

⛔ Bu ölçümün İLK sürümü geçersizdi: yalnız `gz sim` başlatılıyordu, SITL
başlatılmıyordu. `<lock_step>1</lock_step>` yüzünden Gazebo her fizik adımında
uçuş kontrolcüsünü bekliyor; SITL yokken RTF her çözünürlükte 0.22-0.29
çıkıyordu — render maliyeti değil BOŞTA BEKLEME ölçülmüştü (§5.1).

**Ölçüm 2 — tespit boru hattı** (`gcs_server.process_iris_frame`, yalıtılmış):

| çözünürlük | kare süresi | tavan |
|---|---|---|
| 640×480 | 15.5 ms | 64.6 k/s |
| 1014×760 | 28.6 ms | 35.0 k/s |
| **1352×1014** | **28.6 ms** | **35.0 k/s** |
| 2028×1520 | 32.9 ms | 30.4 k/s |

**NİYE 1352 EN İYİSİ:** `vision/detector.py` YOLO girdi boyutunu ağırlık
dosyasından okuyor: **imgsz = 1280**. Bu yüzden YOLO'nun maliyeti kaynak
çözünürlükten BAĞIMSIZ (ölçüldü: her boyutta 10.5 ms) ve tespit kalitesi
1280 kaynak genişliğinde DOYUYOR:
- 1014 verilirse 1280'e **büyütülür** → olmayan bilgi uydurulur
- **1352 verilirse 1.06× küçültülür → bilgi neredeyse tam**
- 2028 verilirse 1.58× küçültülür → fazlası atılır, ama GPU ve JPEG ödenir

1014 ile 1352 aynı maliyette (28.6 ms) olduğu için 1352 **bedavaya** daha çok
bilgi veriyor.

**Canlı sistemde** (aynı kod, aynı koşullar, gz + 2 SITL + gcs_server):

| çözünürlük | işlenen tespit/s |
|---|---|
| 640×480 | 9.2 |
| 1014×760 | 6.9 |
| 1352×1014 | 6.7 |

Yani çözünürlük artışının bedeli **%27**. Sınır bu makinenin toplam yükü
(Gazebo render + YOLO aynı GPU), çözünürlük değil.

⚠ Bir ara "640×480'de 18-23 tespit/s" diye kıyas yapılmıştı; o sayı BAŞKA bir
oturumun logundan geliyordu ve aynı koşul değildi. Elmayla elma kıyası
yukarıdaki tablodur.

### 4.3 · Önizleme küçültmesi — ölçülerek eklendi

`process_iris_frame` panele giden MJPEG'i TAM çözünürlükte kodluyordu.
`cv2.imencode` maliyeti piksel sayısıyla doğrusal: 0.3 MP ≈ 4.5 ms,
3.1 MP ≈ 45.8 ms. Tespit ve takip tam çözünürlükte kalır (bilgi kaybı YOK);
yalnız EKRANA giden kopya 960 px genişliğe küçültülür. Ölçülen kazanç
2028×1520'de **58.7 → 32.9 ms** (neredeyse yarı). `AVCI_ONIZLEME_EN=0` kapatır.

### 4.4 · FX değişti — piksel sabitleri artık TÜRETİLİYOR

FX (piksel cinsinden odak uzaklığı) 166.58 → **329.84** (×1.98). Güdümdeki
piksel birimli sabitler eskiden elle yazılıydı; kamera değişince sessizce
bozulurlardı. Artık `geo.FX`'ten türetiliyorlar:

```python
_PX_OLCEK = geo.FX / geo.FX_OLCUM_REFERANSI   # 166.58 = ölçüldükleri kamera
MENZIL_PX_M_KOSEGEN = 296.8 * _PX_OLCEK        # px·m  → büyür
K_FWD               = 0.35  / _PX_OLCEK        # (m/s)/px → küçülür
```

Doğrulandı: aynı fiziksel menzil, iki kamerada da **aynı** menzil kestirimini
veriyor. `control/ayar_konsolu.py`'deki kaydırıcı aralıkları da aynı ölçekten
türetiliyor — bunu **bekçi testi yakaladı** (`test_bbox_ibvs` K1: "her
kaydırıcının aralığı yürürlükteki değeri KAPSAR").

⚠ Testlerin bir kısmı kadraj merkezini (320,240) ve kenarlarını (640,480)
SABİT yazıyordu ve kamera değişince YANLIŞ SEBEPLE düştü. `geo`'dan türetilir
hale getirildi — bir daha kamera değişince kendiliğinden uyarlanacaklar.

---

## 4B · TARET JOYSTICK — panelde

`AVCI DRONE` sekmesinde **🔭 TARET (namlu + kamera)** kartı:
- kare joystick (yatay = pan ±180°, dikey = tilt −30…+90°) — sürükle
- **KOMUT / GERÇEK / HATA** yan yana: eklem komuta gerçekten gitti mi
  (§5.1 mekanizma kapısı, panelde canlı)
- hazır açı düğmeleri: İLERİ 25° · DÜZ · TAM ÜST · SOL 90 · SAĞ 90 · ALT −30
- **🕸 AĞ AT** düğmesi + namlu hızı kaydırıcısı, altında atış sonucu

Sunucu tarafı: `POST /api/taret {pan, tilt}` → Gazebo `cmd_pos`;
`GET /api/taret` → komut + gerçek + hata. Komut 20 Hz'e kısılır.

⛔ İki arıza ölçülerek bulundu ve düzeltildi:
1. **Keşif gecikmesi:** gz-transport'ta yayıncı ile abone birbirini bulana
   kadar atılan mesaj DÜŞÜYOR. Sunucu açıldıktan sonraki ilk taret komutu
   hiç ulaşmıyordu. Çözüm: `advertise` sonrası kısa bekleme + **2 Hz
   tekrarlayıcı iplik** (Gazebo yeniden başlarsa da senkron kalır).
2. **Panel–sunucu ayrışması:** panel yenilenince yerel komut sıfırlanıyor ve
   "KOMUT 0/25" yazarken taret başka açıdaydı. Açılışta ve her tazelemede
   sunucudan okunuyor (gerçek kaynak sunucudur).

Ölçülen açı hatası: **0.00-0.93°**.

---

## 4C · AĞ FIRLATICI — **saf Python**, C++ eklenti YOK

Kullanıcı kuralı: *"ağ fırlatıcı taşınsın, C++ falan istemem python olsun."*
`vakkas-entegre` dalındaki C++ gz-sim eklentileri (NetLauncherPlugin /
NetCapturePlugin) **taşınmadı**. Yerine `control/ag_firlatici.py`:

- **balistik** Python'da entegre edilir (yerçekimi + hava direnci + açılma)
- **görselleştirme** `ag_konisi` modeli `set_pose` ile taşınır
- **yakalama** hedefin ağ ağzına girip girmediği geometrik olarak sınanır

⛔ Bedeli açıkça: ağ Gazebo fiziğinde rijit cisim DEĞİL. Sekmez, dolanmaz,
hedefe kuvvet uygulamaz. "Ağ hedefi sardı mı" sorusu için yeterli; "ağ
pervaneye dolandı mı" sorusu bu modelle CEVAPLANAMAZ.

### Balistik model
```
v0 = v_araç + NAMLU_HIZI · n̂          (araç hızı TOPLANIR — unutulursa menzil yarılanır)
a  = g⃗ − ½·ρ·C_d,etkin·A(t)·|v⃗|·v⃗ / m
```

⛔ **İlk modelim ağı DOLU disk saydı ve menzili 4 kat kısa verdi** (20 m/s'de
2.8 m; gerçek ağ tabancaları 10-20 m yapar). Ağ **gözeneklidir**, hava içinden
geçer. Gözenekli perde için:
```
C_d,etkin = C_d,dolu · (1 − β²)      β = açık alan oranı
β=0 (dolu) → C_d,dolu ✔    β=1 (ağ yok) → 0 ✔    β=0.9 → 1.2·0.19 = 0.23
```
Menzil (β=0.9, ağız yarıçapı 0.60 m, 0.30 kg):

| namlu hızı | yatay menzil |
|---|---|
| 10 m/s | 4.2 m |
| 20 m/s | 6.5 m |
| 30 m/s | 8.2 m |

Sürükleme hızın karesiyle arttığı için hızı artırmak az kazandırır; **ağı daha
GEÇ açmak** (`ACILMA_S`) çok daha etkilidir.

### ⛔ Zaman tabanı — İKİ KEZ yanlış yapıldı
1. `t` duvar saatinden, entegrasyon sabit `dt=1/100` ile → sim zamanı geride
   kaldı, "3.84 s uçan" ağ 1.74 m gitti.
2. `dt`yi gerçek geçen süre yapıp **0.05 s'e kırpmak** → iplik 0.3 s aç
   kalınca 0.25 s'lik fizik çöpe gitti, menzil yine kısa.

✔ Doğrusu **alt adımlı yetişme**: fizik sabit küçük adımla (4 ms) ilerler,
her uyanışta duvar saatiyle arasındaki farkı kapatana kadar ardışık adım atar.
Görselleştirme fizikten ayrı ve daha seyrek (50 Hz). Sonuç artık ipliğin
yüküne bağlı değil. Rapora **`zaman_sapmasi`** alanı eklendi — mekanizma
kapısı: sapma büyükse menzil yanlıştır.

Doğrulandı: çıkış 20.0 m/s → son hız 4.3 m/s (hesaplanan son hız 4.3),
menzil 16.5 m (yatay ~7 m + düşüş ~15 m).

---

## 5 · UÇUŞ ZARFI — hesap, ölçüm ve **çürüyen bir hipotez**

### 5.1 Eğim tavanı: 70° → 45°
Araç θ kadar yatarken itkinin yalnız cos θ kadarı yukarı bakar; irtifayı
korumak için gereken itki **W/cos θ**'dır.

| eğim | gereken | asılı devir u(θ) | duruş için kalan pay (tavan 1.00) |
|---|---|---|---|
| 30° | 1.15× | 0.762 | +0.238 |
| 40° | 1.31× | 0.810 | +0.190 |
| **45°** | **1.41×** | **0.843** | **+0.157** ← seçildi |
| 55° | 1.74× | 0.936 | +0.064 |
| 60° | 2.00× | 1.003 | −0.003 ⛔ imkânsız |
| 70° | 2.92× | — | ⛔ imkânsız |

**Bu, 2026-09-02'de gerçek dronun düştüğü mekanizmanın ta kendisidir**
(60°+60° → bileşke 75.5° → dikey itki oranı 0.25 → gereken gaz %124).

Uçuş ölçümü (`tools/egim_zarfi.py`): 45° kolunda 2.6 s'de yalnız −3.7 m,
tepe motor `u` 0.977 (tavan 1.00'in altında). 30° kolunda −0.2 m, tepe 0.818.

### ⛔ Bir kez YANLIŞ HESAPLADIM — motor tavanı
Önce `MOT_SPIN_MAX = 0.95` tavan sanıldı ve zarf 40°'ye kadar indirildi.
Ama `ArduPilotPlugin.cc:1676`:
```cpp
raw_cmd = clamp((pwm - servo_min)/(servo_max - servo_min), 0, 1);
```
`servo_min=1100, servo_max=1900`; ArduPilot'un 0.95'i PWM 1950'ye denk gelir
ve eklenti onu 1900'e **kırpar** → gerçek tavan **u = 1.00**. Kullanılabilir
itki/ağırlık 1.80 değil **1.99**. Zarf 45°'ye geri alındı.

### 5.2 ⛔ ÇÜRÜYEN HİPOTEZ — açısal kazançları ×4.75 büyütmek

**İddia:** "Açısal ivme = tork/atalet. İtki 0.48 kat düştü, atalet 2.27 kat
arttı → yetki 0.21 kat. Kazançlar 1/0.21 = 4.75 ile çarpılmalı."
Buna göre `ATC_RAT_*_P/I` 0.054 → 0.256, `_D` 0.00144 → 0.00684 yapıldı.

**A/B ölçümü (tek değişken, aynı desen, aynı araç):**

| | 12 m/s kare deseni | 18 m/s |
|---|---|---|
| kazanç **0.256** | 2. bacakta **−46.7 m**, `Potential Thrust Loss`, `Crash: AngErr=62>30` | −46.6 m, düştü |
| kazanç **0.135** (fabrika) | dört bacakta da **0.0 m**, doyum %0, hız takibi %101 | sarkma ≤0.5 m, %97 |

**Hipotez çürüdü.** Neden: hesap toplam ataleti değil yalnız `base_link`
ölçeğini kullandı — rotorların m·y² katkısı (0.0048 kg·m²) DEĞİŞMEDİ, gerçek
oran 2.27 değil ~1.96. Ayrıca D kazancını 1.9 katına çıkarmak ArduPilot'un
20 Hz türev süzgeciyle gürültüyü büyütüp motor karıştırıcısını doyuruyor —
`Potential Thrust Loss` tam bunun mesajı.

**Şimdiki değerler ArduCopter fabrika varsayılanlarıdır.** Gerçek VAKKAS
uçtuğunda doğru yol teorik ölçekleme değil **ArduPilot AUTOTUNE**'dur.

### 5.3 Uygulanan parametreler

| parametre | eski | yeni | gerekçe |
|---|---|---|---|
| `ANGLE_MAX` | 7000 (70°) | **4500 (45°)** | pay tablosu + uçuş |
| `WPNAV_ACCEL` | 2600 | **950** | g·tan45° = 9.81 m/s² |
| `WPNAV_SPEED_UP` | 2000 | **1000** | (T/W−1)·g = 9.7 m/s² tavan |
| `WPNAV_SPEED_DN` | 1500 | **800** | iniş asimetrik risk |
| `WPNAV_ACCEL_Z` | 2000 | **500** | fiziksel tavana pay |
| `ATC_RAT_*_P/I` | 0.054 | **0.135** | fabrika (×4.75 çürüdü) |
| `ATC_RAT_*_D` | 0.00144 | **0.0036** | fabrika |
| `ATC_ACCEL_R/P_MAX` | 250000 | **110000** | fabrika |
| `PSC_JERK_XY/Z` | 40 | **10** | zarf küçüldü |
| `MOT_THST_HOVER` | 0.31 | **0.47** | ÖLÇÜLDÜ |
| `MOT_HOVER_LEARN` | — | **0** | deney disiplini |

---

## 6 · ⛔⛔ EEPROM TUZAĞI — eski kampanyaları etkileyen bulgu

`sim/ardupilot_params/avci_copter.parm` dosyası eğim tavanı için **7000**
yazıyordu. Araç **6000** uyguluyordu.

**Kök neden:** `--add-param-file` yolu `AP_Param::set_default_by_name()`
çağırır (`AP_Param.cpp:2723`) — yalnız **varsayılanı** değiştirir. Bir
parametrenin `~/ardupilot/eeprom.bin` içinde **kayıtlı** değeri varsa
(biri bir kez GCS'ten yazdıysa) kayıtlı değer kazanır ve dosya sessizce
yok sayılır. **Hata mesajı yoktur.**

Yani araç 2026-08-17'den beri 70° değil **60°** tavanla uçtu; o dönemin
kampanya raporları yanlış bir varsayım üzerine kuruldu. (Diğer 20
parametrenin hepsi doğru uygulanmıştı; yalnız o biri kayıtlıydı.)

**Çözüm:** `tools/param_dogrula.py` — dosyayı araçla karşılaştırır,
`--yaz` ile uyuşmayanı MAVLink'le yazar. `scripts/start_harmonic.sh`
artık her başlatmada bunu kendisi koşuyor.

---

## 7 · Orange Cube'un bedava getirdiği emniyet

Betaflight ANGLE kipinde yatış ve eğim **ayrı ayrı** sınırlanır; bileşkeyi
sınırlayan bir şey yoktur. 60°+60° = 75.5° bileşke mümkündür ve 2026-09-02
kazasında olan tam budur.

ArduCopter bileşkeyi **dairesel** sınırlar — `AP_Math/control.cpp:617`:
```cpp
thrust.limit_length(thrust_limit);
```
Yatay itki vektörünün UZUNLUĞU kısılır, yönü korunur. Orange Cube'a
geçmekle bu koruma bedava geliyor.

---

## 8 · Bilinen açık maddeler

1. **Atalet tensörü** iris geometrisinden ölçekleniyor; VAKKAS'ın 3B modeliyle
   yeniden hesaplanmalı.
2. **Taret sürücüsü yok** — taret panelden ELLE komut ediliyor; tespitle
   kapalı çevrim (hedefi otomatik nişanda tutma) bağlanmadı. Sıradaki adım bu.
3. **Ağ örgü fiziği yok** — ağ Gazebo'da rijit cisim değil; dolanma/sekme
   modellenmiyor (bkz. §4C).
4. **Ağın gerçek gözenekliliği ölçülmedi** — β = 0.90 varsayıldı ve menzil
   buna çok duyarlı (β 0.85 → 5.4 m, 0.95 → 8.7 m). Gerçek ağ eline geçince
   `AVCI_AG_GOZENEK` ölçülmeli.
5. **Balıkgöz bozulması modellenmiyor** — düz izdüşüm kullanılıyor (§4.1).
   Gerçek boru hattında undistort planlandığı için kasıtlı.
6. **`tests/test_frpn_guidance.py` D2 düşüyor** — `frpn_guidance` ile
   `gps_guidance` durum sözlüklerinin anahtarları uyuşmuyor.
   ⚠ Bu bu oturumdan ÖNCE de düşüyordu (temiz HEAD kopyasında doğrulandı).

---

## 9 · Taretin dışarıdan fotoğrafı nasıl alınır

`logs/vakkas/vakkas_taret_disaridan.jpg` şu yolla üretildi. Dünya dosyasına
GEÇİCİ olarak eklenip sonra geri alındı (sürekli 1280×960 render maliyeti
boşuna olmasın diye):

```xml
<!-- sim/gazebo_harmonic/worlds/avci_harmonic.sdf içine, </world>'den önce -->
<model name="vitrin_kam">
  <static>1</static>
  <pose degrees="true">1.1 -0.55 0.55 0 12 155</pose>
  <link name="link">
    <sensor name="kam" type="camera">
      <always_on>1</always_on><update_rate>10</update_rate>
      <topic>vitrin/image</topic>
      <camera>
        <horizontal_fov>0.7</horizontal_fov>
        <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>100</far></clip>
      </camera>
    </sensor>
  </link>
</model>
```

Sonra `/vitrin/image` konusuna abone olup taret açılarını sırayla komut et
(`tools/taret_dene.py` içindeki `komut()` deseni).
