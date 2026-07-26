# GPS GÜDÜM — Tam Geliştirme Günlüğü ve Teknik Doküman

Bu belge, avcı drone'un GPS güdümünün **sıfırdan bugüne** nasıl geliştiğini anlatır:
denenen her yöntem, karşılaşılan her arıza, o arızayı nasıl **ölçtüğümüz**, hangi
çözümün neden **işe yaramadığı** ve sistemin şu an nasıl çalıştığı.

Buradaki her sayı gerçek uçuş kaydından gelir. Tahmin yok; her iddianın arkasında
20 Hz'lik CSV kaydı veya araçtan okunmuş parametre dökümü var.

---

## İÇİNDEKİLER

1. [Görev nedir](#1-görev-nedir)
2. [Sistem mimarisi ve dosya dosya ne yapar](#2-sistem-mimarisi--dosya-dosya-ne-yapar)
3. [Güdümün bugünkü çalışma mantığı](#3-güdümün-bugünkü-çalışma-mantığı)
4. [Bütün ayar değişkenleri ve neden o değerde](#4-bütün-ayar-değişkenleri-ve-neden-o-değerde)
5. [Geliştirme günlüğü — denenen her yöntem](#5-geliştirme-günlüğü--denenen-her-yöntem)
6. [ASIL KÖK NEDEN: uygulanmayan parametreler](#6-asıl-kök-neden-uygulanmayan-parametreler)
7. [Dikey eksen: tırmanma ve alçalma](#7-dikey-eksen-tırmanma-ve-alçalma)
8. [Güvenlik: yere çarpma ve kontrol kaybı korumaları](#8-güvenlik-yere-çarpma-ve-kontrol-kaybı-korumaları)
9. [Güdümün dışındaki arızalar](#9-güdümün-dışındaki-arızalar)
10. [Ölçüm yöntemi — uçuş kaydı](#10-ölçüm-yöntemi--uçuş-kaydı)
11. [Sonuçlar tablosu](#11-sonuçlar-tablosu)
12. [Nasıl çalıştırılır](#12-nasıl-çalıştırılır)
13. [Bilinen sınırlar ve sıradaki işler](#13-bilinen-sınırlar-ve-sıradaki-işler)

---

## 1. GÖREV NEDİR

Avcı drone (**ArduCopter / iris quad**) hedef uçağı (**ArduPlane / Talon sabit kanat**)
takip eder. İstenen davranış:

- Hedefin **tam arkasında ~10 metre** mesafede kalmak
- Hedefin **6 metre altında** uçmak (kamera 25° yukarı baktığı için hedef kadrajda kalır)
- Burun daima hedefe dönük (kamera hedefi görsün)
- Hedef manevra yapınca **duraklamadan, kopmadan** peşinde kalmak
- Hedefin önüne geçmemek

İki aracın da GPS konumu ve hızı bilinir; görsel tespit ayrı hattadır.

**Zorluk:** avcı bir quadcopter, hedef bir sabit kanat uçak. Quad havada asılı
kalabilir ama yatay hızı gövde eğimiyle sınırlıdır; uçak sürekli 15 m/s ile uçar
ve keskin dönebilir.

---

## 2. SİSTEM MİMARİSİ — DOSYA DOSYA NE YAPAR

### Komut zinciri

```
Arayüz "Chase" butonu
        │
        ▼
gcs/gcs_server.py  ──►  _chase_thread()
        │                 ├─ iris'e bağlan (MAVLink)
        │                 ├─ GUIDED moda al, arm et
        │                 ├─ 30 m'ye kalkış  (havadaysa atlanır)
        │                 └─ güdüm fonksiyonunu çağır
        ▼
guidance/gps_gudum/gps_approach.py  ──►  run_gps_approach()
        │                 20 Hz döngü: hedefi oku → komut hesapla → gönder
        ▼
guidance/ortak/common.py  ──►  send_velocity()
        │                 SET_POSITION_TARGET_LOCAL_NED paketi
        ▼
ArduCopter SITL  ──►  400 Hz iç kontrolcü ──► motorlar ──► Gazebo fiziği
```

### GPS güdümü dosyaları (`guidance/gps_gudum/`)

#### `gps_approach.py` — **ANA GÜDÜM, ŞU AN KULLANILAN** (422 satır)

Bütün geliştirme bu dosyada yapıldı. `run_gps_approach(conn, get_plane, get_iris,
stop_event)` fonksiyonu 20 Hz'lik bir döngü çalıştırır; her turda hedefin
telemetrisini okur, avcının gitmesi gereken noktayı bulur, hız komutunu hesaplar
ve MAVLink ile gönderir.

İçindeki bölümler:
- `Cfg` sınıfı — bütün ayar değişkenleri (bölüm 4)
- Telemetri tazelik kontrolü ve ölü hesap
- Yere çarpma / kontrol kaybı koruması
- Gölge noktası hesabı (`_gecikmeli_nokta`)
- Dikey referans ve dikey feedforward
- Yatay komut: hedefin hızı + gölgeye çekme + bütçe tavanı
- Yaw hesabı
- Uçuş kaydı (CSV)

#### `gps_chase.py` — eski sürüm, **kullanılmıyor** (varsayılanda)

`SPRINT → APPROACH → LOCK → STRIKE` durum makinesi. Kamera geometrisini GPS'ten
simüle ederek duruş mesafesi hesaplardı. Hâlâ çağrılabilir: `AVCI_GPS_LAW=v2`.
Silinmedi çünkü durum makinesi yaklaşımı ileride referans olabilir.

#### `gps_strike.py` — terminal vuruş hattı, **ayrı görev**

Proportional Navigation (LOS oranı → ivme komutu) ile terminal vuruş. Takip
görevi değil, çarpma görevi içindir. `run_strike()` ile ayrı çağrılır.

#### `__init__.py` — boş, paket tanımı

### Ortak altyapı (`guidance/ortak/`)

#### `common.py` (161 satır)

Bütün güdüm hatlarının paylaştığı yardımcılar:

| Fonksiyon | Ne yapar |
|---|---|
| `send_velocity(conn, vx, vy, vz, yaw)` | **Saf hız + yaw** komutu gönderir. `SET_POSITION_TARGET_LOCAL_NED`, type_mask ile pozisyon/ivme yok sayılır. **Bugün kullanılan komut budur.** |
| `send_position_setpoint(...)` | Pozisyon + feedforward hız + yaw gönderir. GPS güdümünde denendi ve **terk edildi** (bölüm 5.6) |
| `clamp(val, lo, hi)` | Değeri aralığa sıkıştırır |
| `normalize_angle(a)` | Açıyı `-π..+π` aralığına indirger (yaw farkı hesapları için) |
| `limit_acceleration(...)` | İvme sınırlayıcı — GPS güdümünde **kullanılmıyor** (manevrada komutu kısıp duraklama yaptığı için kaldırıldı) |
| `timestamp_ms()`, `vec3_len()` | Küçük yardımcılar |

### Güdümü besleyen taraf (`gcs/gcs_server.py`)

GPS güdümü için kritik üç işi yapar:

**1. Telemetri toplama.** İki ayrı MAVLink bağlantısı dinler (iris 14541, plane
14550). Her okumada kuyruğu **tamamen boşaltır** — tek mesaj okusaydı kuyruk
birikir ve veri saniyelerce gecikirdi (bölüm 9.1).

**2. Çerçeve hizalama (`_frame_off`).** İki SITL'in EKF orijini **aynı değildir**;
her araç kendi spawn noktasını sıfır kabul eder. Plane'in konumunu ham haliyle
kullanmak ~12 m sabit hata veriyordu (drone hedefin *yanından* takip ediyordu).
Çözüm: iki aracın `GLOBAL_POSITION_INT` (GPS) verisinden sabit ofset kendinden
kalibre edilir ve plane'in konumu **iris çerçevesine taşınır**. Güdüme gelen
`get_plane()` verisi bu düzeltilmiş haldedir.

**3. Güdüm seçimi.** `_chase_thread()` içinde:

| Ortam değişkeni | Çalışan güdüm |
|---|---|
| *(varsayılan)* | `gps_approach.run_gps_approach` — **saf GPS** |
| `AVCI_HYBRID=on` | `hibrit_gudum/supervisor.run_hybrid` — GPS ↔ görsel geçişli |
| `AVCI_GPS_LAW=v2` | `gps_chase.run_chase` — eski durum makinesi |

Görsel faz **bilerek kapalıdır**: görsel güdüm hedefi kadrajda bulamayınca drone
havada duraklıyordu. Kod silinmedi, `AVCI_HYBRID=on` ile geri açılır.

### Aracın davranışını belirleyen dosya

#### `sim/ardupilot_params/avci_copter.parm`

**Bu dosya güdüm kadar önemlidir.** Güdüm ne kadar doğru komut verirse versin,
araç o komutu uygulayamıyorsa bir şey değişmez. Projedeki en büyük arıza buradaydı
(bölüm 6).

### Hedefi uçuran taraf

#### `demos/run_plane_scenario.py`

Hedef uçağı otonom uçurur (`square`, `circle`, `aggressive`). GPS güdümü test
edilebilsin diye iki ekleme yapıldı:
- **İrtifa P denetimi:** hedef eskiden durmadan yükseliyordu (46 m → 124 → 322 → 413 m),
  bu testleri geçersiz kılıyordu. Artık pitch, hedef irtifayı koruyacak şekilde sürülür.
- **`AVCI_HEDEF_IRTIFA` env:** senaryo istenen irtifada başlatılabilir. Uçak zaten
  havadaysa kalkış atlanır — böylece senaryo yeniden başlatılarak hedefin irtifası
  değiştirilebiliyor (tırmanma testleri için gerekliydi).

---

## 3. GÜDÜMÜN BUGÜNKÜ ÇALIŞMA MANTIĞI

### Tek cümlelik özet

> Avcı, hedefin **0.7 saniye önceki konumuna** kilitlenir; oraya hedefin hızıyla
> gider, üstüne bir düzeltme bineri; 6 m altında uçar, burnu hedefe dönüktür.

Bu yaklaşımın adı **gölge takibi**. Avcı hedefin gölgesidir: hedef nereye gittiyse
avcı da tam oraya gider, sadece biraz sonra.

Mesafe kendiliğinden oluşur:

```
mesafe ≈ hedef_hızı × GECIKME   →   15.2 m/s × 0.7 s ≈ 10.6 m
```

Hedef hızlanınca aralık açılır, yavaşlayınca kapanır. Yani avcı hedefin **yolunda**
kalır; mesafeyi ayrıca kovalamak gerekmez.

### Döngünün adımları (20 Hz)

**Adım 1 — Telemetriyi oku, tazeliği kontrol et.**
Telemetri 4 Hz gelir, döngü 20 Hz döner. Yeni paket geldiyse hedefin
`(zaman, konum, hız)` üçlüsü geçmişe yazılır. Her döngüde yazsaydık aynı konumdan
5 kopya birikir ve "0.7 sn öncesi" araması bozulurdu.

Telemetri kesilirse drone **durdurulmaz**: hedef son bilinen hızıyla ileri taşınır
(ölü hesap). Ancak `DUR_S` (10 sn) boyunca hiç paket gelmezse güvenlik için havada
tutulur.

**Adım 2 — Güvenlik kontrolü.** Yere çarpma veya kontrol kaybı riski varsa takip
bırakılır, kurtarmaya geçilir (bölüm 8).

**Adım 3 — Gölge noktasını bul.** Geçmişte `şimdi − 0.7 s` anına karşılık gelen
konum, iki komşu ölçüm arasında **doğrusal ara değerle** hesaplanır. Ham örnek
seçmek 0.25 saniyelik basamaklar üretir ve hedef noktası zıplardı.

**Adım 4 — Dikey referans.** Hedefin **anlık** irtifasının 6 m altı, güvenlik
tabanı 15 m. (Dikeyde gölge kullanılmaz — bölüm 7.)

**Adım 5 — Yatay komut.**

```
komut = hedefin_hızı  +  gölgeye_çekme
```

- **Hedefin hızı (feedforward):** avcının hedefle aynı hızda gitmesini sağlar.
  Bu terim olmadan avcı sürekli geride kalır.
- **Gölgeye çekme (PD):** konum hatasını kapatır.
  `çekme = KP_KONUM × hata − KD_KONUM × yaklaşma_hızı`
  D terimi olmadan avcı gölgeye dalıp üstünden geçiyordu (bölüm 5.7).
- **Bütçe tavanı:** toplam komut `V_TAVAN`'ı aşamaz. **Ama vektör ölçeklenmez** —
  hedefin hızı dokunulmaz kalır, çekmeye yalnız tavandan **artan pay** verilir.
  `|ff + c·u| = V_TAVAN` denkleminin çözümü kullanılır. (Neden bu şekilde: bölüm 5.8)

**Adım 6 — Dikey komut.**

```
dikey = hedefin_dikey_hızı  −  KP_DIKEY × irtifa_hatası
```
Tırmanma tavanı 8 m/s, alçalma tavanı 3.5 m/s (**ayrı olmak zorunda** — bölüm 7.3).

**Adım 7 — Yaw.** Burun hedefe döner. Hedefe 4 m'den yakınsa yaw **dondurulur**
(o mesafede "hedefe bakan açı" tanımsızlaşır, en ufak gürültü burnu çevirir).

**Adım 8 — Gönder.** `send_velocity()` ile saf hız + yaw komutu.

---

## 4. BÜTÜN AYAR DEĞİŞKENLERİ VE NEDEN O DEĞERDE

`gps_approach.py` içindeki `Cfg` sınıfı:

### Takip geometrisi

| Değişken | Değer | Anlamı ve gerekçesi |
|---|---|---|
| `GECIKME` | 0.7 s | Avcı hedefin bu kadar saniye önceki konumunda durur. **Mesafeyi belirleyen tek ayar.** 15 m/s'de ≈ 10.6 m. `0.5` → ~8 m, `1.0` → ~15 m |
| `ALT_OFFSET` | 6.0 m | Hedefin kaç metre altında uçulacağı. Kamera 25° yukarı baktığı için hedef kadrajda kalır |
| `MIN_ALT` | 15.0 m | Referans irtifanın tabanı. Hedef alçalsa da avcı bunun altına inmez |
| `LOOP_HZ` | 20.0 | Döngü frekansı |

### Gölgeye çekme (manevrada geri kalmayı kapatan terim)

| Değişken | Değer | Gerekçe |
|---|---|---|
| `KP_KONUM` | 0.8 | Konum hatası (m) → ek hız (m/s) |
| `KD_KONUM` | 2.0 | Sönümleme. **Şart:** yalnız P ile denendiğinde avcı gölgeye 25 m/s ile dalıp üstünden geçti, takip limit döngüsüne girdi (12 m → 107 m → 58 m → 134 m) |
| `V_TAVAN` | 20.0 m/s | Komutun **toplam** tavanı. Tavansız denendi: komut 27-29 m/s'e çıktı, quad ulaşamayacağı komutu kovalarken **çakıldı** |
| `KP_DIKEY` | 1.0 | İrtifa hatası (m) → dikey hız (m/s) |

### Hız ve dikey sınırlar

| Değişken | Değer | Gerekçe |
|---|---|---|
| `V_MAX` | 19.0 m/s | Yakalama fazındaki hız komutunun tavanı |
| `VZ_YUKARI` | 8.0 m/s | Tırmanma tavanı. **3.0 idi ve darboğazdı** — araç 8 m/s tırmanabilirken güdüm üçte birinde tutuyordu, 100 m fark 33 saniye sürüyordu |
| `VZ_ASAGI` | 3.5 m/s | Alçalma tavanı. **8 m/s alçalma drone'u çaktırdı** — hızlı alçalışta yaw otoritesi kayboluyor (bölüm 7.3) |
| `YAKALAMA` / `YAKALAMA_CIK` | 60 / 80 m | Uzakta saf kovalama, yakında gölge takibi. Histerezisli — tek eşikte mod sürekli zıplıyordu |
| `GECMIS_S` | 8.0 s | Hedef konum geçmişi bu kadar geriye tutulur |

### Güvenlik korumaları

| Değişken | Değer | Gerekçe |
|---|---|---|
| `KURTARMA_ALT` | 12.0 m | Bu irtifanın altı = koşulsuz kurtarma. Bilerek `MIN_ALT`'ın altında: normal alçak uçuşta tetiklenmemeli |
| `KURTARMA_SURE` | 3.0 s | Çarpmaya bu kadar süre kaldıysa kurtarma. `irtifa ÷ düşüş_hızı` — 60 m'de bile serbest düşüşteyse yakalar |
| `GUVENLI_ALT` | 35.0 m | Kurtarmadan ancak bu irtifada çıkılır (histerezis) |
| `YAW_SAPMA_SINIR` | 75° | Burun komuttan bu kadar saparsa **kontrol kaybı** sayılır |
| `YAW_SAPMA_CIK` | 40° | Kurtarmadan çıkış için sapma bunun altına inmeli |
| `YAW_MIN_MESAFE` | 4.0 m | Hedefe bundan yakınken yaw dondurulur |
| `DUR_S` | 10.0 s | Telemetri bu kadar susarsa havada tutulur |

### Durum bildirimi

| Değişken | Değer | Anlamı |
|---|---|---|
| `KILIT_MESAFE` | 40.0 m | Altında durum "KİLİT" (görsel faz devralabilir) |

---

## 5. GELİŞTİRME GÜNLÜĞÜ — DENENEN HER YÖNTEM

Bu bölüm kronolojik. Her madde: **ne denendi → ne oldu → neden başarısız → ne öğrenildi.**

### 5.1 Nokta hedefleme (ilk sürüm) ❌

**Yöntem:** Hedefin arkasında sabit bir nokta hesapla, oraya git.
`nişan = hedef_konumu − standoff × hedef_yön_vektörü`

**Sonuç:** Hedef 90° dönünce nişan noktası **yana fırlıyordu**. Drone 19 m/s'lik hız
vektörünü bir anda çevirmek zorunda kalıyordu (|Δv| ≈ 27 m/s) ve fizik gereği
yavaşlıyordu.

**Ders:** Sorun kazanç ayarında değil, **dayatılan yönün kendisindeydi**. Kazanç/fren
ayarıyla çözülemez.

### 5.2 Kazanç artırma (Kp = 8) ❌

**Yöntem:** Konum hatası kazancını yükselterek hatayı daha hızlı kapat.

**Sonuç:** 2.4 m hatada bile tam gaz veriyordu; drone nişan noktasını aşıp geri
dönüyor, dönüşlerde **ilmek atıyordu**. (Kullanıcının ekran görüntülerinde sarı
dairelerle işaretlediği savrulmalar.)

**Ders:** Yüksek kazanç + gecikme = salınım. Kp 2.5'e indirildi.

### 5.3 İz takibi / trail following ⚠️ kısmi

**Yöntem:** Hedefin geçtiği noktaları kaydet, drone bu iz üzerinde hedeften
`STANDOFF` metre gerideki noktaya gitsin. "Hedef nasıl döndüyse drone da aynı
dönüşü yapar."

**Sonuç:** Düz uçuşta çok iyi. Ama dönüşte iz noktası **köşeye kadar yürüyordu**;
drone köşeye varana kadar hedef yeni yönde uzaklaşmış oluyordu (dış yay = uzun yol).
Ölçüm: `d_h 7.5 m → 26.6 m → 50 m`, üstelik komut zaten 19/19 tam gazdı.

**Ders:** Yol takibi doğru fikir ama mesafe tabanlı geriye yürüme köşelerde kayıp
verir. (Zaman tabanlı gecikmeye — gölge takibine — geçişin tohumu.)

### 5.4 Köşe kesme + dönüş öngörüsü ⚠️ kısmi

**Yöntem:** Dönüş sertleştikçe izi bırak, hedefin **şu anki** arkasına nişan al
(kiriş kesme). Ayrıca hedefin yönünü kapanma süresi kadar ileri döndürüp iç yaydan
git.

**Sonuç:** Bir dönüşte mesafe 11.5 m'de kaldı (öncesinde 65 m'ye açılıyordu). Ama
tutarsızdı; bazı dönüşlerde hâlâ 40 m'ye açılıyordu.

**Ders:** Doğru yöne gidiyordu ama asıl darboğaz başka yerdeydi (bölüm 6).

### 5.5 GÖLGE TAKİBİ ✅ **temel çözüm**

**Yöntem (kullanıcının fikri):** "Aracımızı direkt GPS olarak bağlasak, ufak bir
gecikmeyle." Yani mesafe ile değil, **zaman ile** geriye git: hedefin 0.7 saniye
önceki konumu.

**Neden daha iyi:** İz takibinde mesafe sabitti; hedef yavaşlayınca/hızlanınca
geometri bozuluyordu. Zaman gecikmesinde nokta her zaman hedefin **gerçek yolunun**
üzerindedir ve hedefle **aynı hızda** hareket eder.

**Sonuç (ilk ölçüm):** Düz uçuşta `d_h ≈ 11 m`, irtifa farkı tam 6.0 m, avcı hızı
hedefe birebir eşleşti (15.1 / 15.2 m/s), gölge noktasına hata **0.3–1.0 m**.

**Ders:** Doğru referans noktası her şeyi basitleştirdi. Bu, bugünkü mimarinin
temelidir.

### 5.6 Konum + hız komutu (`posvel`) ❌ **önemli ders**

**Yöntem:** Gölge noktasına `send_position_setpoint()` ile konum + feedforward hız
gönder. Mantık: "hız profilini ArduCopter'ın 400 Hz'lik kontrolcüsü çıkarsın."

**Sonuç:** Düz uçuşta mükemmeldi ama manevrada **çuvalladı**. Kayıt:

```
d_gölge  8.0 m   KOMUT 20.0 m/s   GERÇEK 14.6 m/s   roll 13°
d_gölge 28.9 m   KOMUT 20.0 m/s   GERÇEK 14.3 m/s   roll  9°
d_gölge 38.4 m   KOMUT 20.0 m/s   GERÇEK 14.3 m/s   roll 13°
```

Güdüm 20 m/s istiyor, araç 14.3'te takılıyor. **Aynı araç**, yakalama fazında saf
hız komutuyla 19 m/s yapıyordu.

**Neden:** ArduCopter `posvel` modunda verilen konumu kendi jerk-limitli profiliyle
"yumuşatıyor" ve hız komutunu boğuyor.

**Ders:** Araya ikinci bir yorumlayıcı katman koymayın. **Saf hız komutuna** geçildi.

### 5.7 Gölgeye çekme terimi — tavansız ❌ **drone çakıldı**

**Yöntem:** Konum hatasını ArduCopter'a bırakmak yerine biz kapatalım:
`komut = hedef_hızı + KP × hata`. Tavan koymadık, "WPNAV_SPEED zaten sınırlar" diye.

**Sonuç:** Komut 27–29 m/s'e çıktı. Quad'ın fiziksel tavanı ~19 m/s; ulaşamayacağı
komutu kovalarken savruldu:

```
roll  24° → -51° → 95° → -115° → 120°
irtifa 59 m → 0 m      "Crash: Disarming: AngErr=180"
```

**Ders:** Araca yapamayacağı komutu vermeyin. Tavan **şart**.

### 5.8 Naif ölçekleme ❌ / Bütçe tavanı ✅

**Naif yöntem:** Komut tavanı aşarsa tüm vektörü ölçekle.

**Sorun:** Bu, **feedforward'ı da kesiyor**. Hedef doğuya 15 m/s giderken drone 20 m
yandaysa komut (15, 19) oluyor, ölçeklenince (11.8, 14.9) → drone doğuya yalnız
11.8 m/s gidiyor, **hedef 15 m/s ile kaçıyor**. Drone geri kalıyor, hata büyüyor,
daha çok yana çekiyor, daha da geri kalıyor.

**Doğru yöntem (bütçe):** Hedefin hızı **dokunulmaz**; çekmeye yalnız tavandan
**artan pay** verilir. `|ff + c·u| = V_TAVAN` ikinci derece denkleminin çözümü:

```python
b    = ffx·ux + ffy·uy
disk = b² − (|ff|² − V_TAVAN²)
bütçe = −b + √disk
```

Doğrulama: feedforward 15.1 m/s iken çekme yönüne göre bütçe —

| Çekme yönü | Bütçe | Toplam komut |
|---|---|---|
| 90° (yanal) | 13.1 m/s | 20.0 m/s |
| 45° | 6.2 m/s | 20.0 m/s |
| 180° (head-on) | 35.1 m/s | 20.0 m/s |

Her yönde toplam tam tavanda, feedforward hiç kesilmiyor.

### 5.9 Sönümleme (D terimi) ✅

**Sorun:** Yalnız P ile çekme, avcıyı gölgeye 25 m/s ile daldırıyordu; gölgede
duramayıp üstünden geçiyor, sonra geri dönüyordu. Takip limit döngüsüne girdi:
`d_h 12 m → 107 m → 58 m → 134 m`.

**Çözüm:** `çekme = KP × hata − KD × yaklaşma_hızı`. Avcı gölgeye yaklaştıkça
kendini frenler.

Doğrulama: 38.5 m hatada durgunken çekme 26.9 m/s; aynı hatada gölgeye 20 m/s ile
dalarken **14.9 m/s** (frenliyor).

### 5.10 Yaw'ı hareket yönüne bağlama ❌ **hipotez çürütüldü**

**Hipotez:** Manevrada burun 134° dönüyor; belki yaw dönüşü yatay hızı kesiyordur.

**A/B testi:**

| Yaw modu | Manevrada geri kalma | Max eğim |
|---|---|---|
| Burun hedefte | 41.7 m | 17° |
| Burun hareket yönünde | 40.8 m | 19° |

**Sonuç:** Fark yok. Yaw suçlu değildi. Kullanıcının istediği "burun hedefte"
davranışı korundu.

**Ders:** Hipotezi ölçmeden değiştirmeyin. Bu değişiklik yapılsaydı kamera kadrajı
bozulur, hiçbir şey kazanılmazdı.

### 5.11 Telemetri kesintisinde durma ❌ → ölü hesap ✅

**Eski davranış:** 3 saniye yeni paket gelmezse `send_velocity(0,0,0)` — yani dur.

**Ölçüm:** Avcı **0.6 m/s**'ye çakıldı, mesafe 78 m → 233 m açıldı.

**Çözüm:** Kısa kesintide durma; hedefi son bilinen hızıyla ileri taşı (ölü hesap).
Yalnız 10 saniyelik gerçek kesintide havada tut.

### 5.12 Head-on kilitlenme ❌ → mesafeye bağlı feedforward ✅

**Sorun:** Karşı karşıya gelişte hedefin hız vektörü drone'a **doğru** bakar;
feedforward ile yaklaşma terimi birbirini götürüyordu. Kayıt: `d_h = 191 m iken
avcı 1.3 m/s`.

**Çözüm:** Feedforward mesafeye bağlandı — 60 m üstünde saf kovalama, 25 m altında
tam eşleme. (Bugünkü sürümde bu, `YAKALAMA` histerezisiyle birlikte çalışır.)

---

## 6. ASIL KÖK NEDEN: UYGULANMAYAN PARAMETRELER

Yukarıdaki bütün güdüm iyileştirmeleri gerçek ve gerekliydi — ama manevradaki
geri kalmanın **asıl sebebi** güdümde değildi.

### Nasıl bulundu

Kayıt bir çelişki gösteriyordu: **komut 20 m/s, araç 14.3 m/s, eğim 17°.** Oysa
`ANGLE_MAX` 55° ayarlanmıştı — araç 55°'ye kadar yatabilmeliydi.

Doğrudan deney yapıldı (güdüm devre dışı, araca elle komut):

```
Test 1: kuzeye 20 m/s  →  8 saniyede ulaştı, eğim ASLA 16°'yi geçmedi
Test 2: ani 90° dönüş  →  hız 19.7 → 8.3 m/s düştü, eğim yine max 17°
```

16° eğim = `9.81 × tan(16°)` = **2.8 m/s²** ivme. Bu, ArduCopter'ın fabrika
`WPNAV_ACCEL` değeri olan 2.5 m/s² ile birebir uyuşuyordu.

Aracın **tüm 1368 parametresi** döktürülüp bizim dosyayla karşılaştırıldı:

```
ANGLE_MAX 5500     → ARAÇTA BÖYLE PARAMETRE YOK
WPNAV_ACCEL 1800   → ARAÇTA BÖYLE PARAMETRE YOK
WPNAV_SPEED 2200   → ARAÇTA BÖYLE PARAMETRE YOK
PSC_VELXY_P 2.0    → ARAÇTA BÖYLE PARAMETRE YOK
ATC_SLEW_YAW 9000  → ARAÇTA BÖYLE PARAMETRE YOK
```

### Sebep

Kullandığımız ArduPilot sürümü parametreleri **yeniden adlandırmış** ve **SI
birimlerine** geçmiş. ArduPilot bilinmeyen parametre adı için **hata vermez,
sessizce yok sayar**. Yani drone günlerce **fabrika ayarlarıyla** uçtu.

| Bizim yazdığımız (eski) | Aracın gerçek adı (yeni) | Fabrika değeri |
|---|---|---|
| `ANGLE_MAX` (centi-derece) | `ATC_ANGLE_MAX` (derece) | 30 |
| `WPNAV_ACCEL` (cm/s²) | `WP_ACC` (m/s²) | **2.5** |
| `WPNAV_SPEED` (cm/s) | `WP_SPD` (m/s) | 10 |
| `WPNAV_SPEED_UP/DN` | `WP_SPD_UP` / `WP_SPD_DN` | 2.5 / 1.5 |
| `LOIT_SPEED` | `LOIT_SPEED_MS` (m/s) | 12.5 |
| `PSC_VELXY_P` | `PSC_NE_VEL_P` | 2.0 |
| `ATC_SLEW_YAW` | *(kaldırılmış)* `ATC_RATE_Y_MAX`=0 zaten sınırsız | — |
| — | `WP_JERK` (m/s³) | **1.0** |

`WP_JERK = 1.0 m/s³` ayrıca kritikti: ivmeyi 0'dan tam eğime çıkarmak **10 saniye**
sürüyordu, manevra ise 8 saniye. Araç tam eğime hiç ulaşamıyordu.

### Düzeltme

`avci_copter.parm` doğru isim ve birimlerle yeniden yazıldı:

| Parametre | Fabrika | Yeni |
|---|---|---|
| `ATC_ANGLE_MAX` | 30° | **45°** |
| `WP_ACC` | 2.5 m/s² | **9.5** |
| `WP_JERK` | 1.0 m/s³ | **15** |
| `WP_SPD` | 10 m/s | **22** |
| `PSC_JERK_NE` | 5 | **15** |
| `PSC_NE_POS_P` | 1.0 | **1.5** |

### Sonuç

| Ölçüm | Önce (fabrika) | Sonra |
|---|---|---|
| Hedefe mesafe (ortalama) | 25.1 m | **10.6 m** |
| Hedefe mesafe (**en kötü**) | 126.3 m | **12.4 m** |
| Gölge noktasına hata | 15.92 m | **1.10 m** |
| Aracın yattığı max açı | 18° | **49°** |
| 10 m'den fazla geri kalma | zamanın %41.6'sı | **%1.0** |
| 25 m'den fazla geri kalma | zamanın %26'sı | **%0.0** |

### ⚠️ BUNDAN SONRA HER PARAMETRE DEĞİŞİKLİĞİNDE

`sim_vehicle` başlarken `Saved N parameters to mav_5_1.parm` yazar.
`~/ardupilot/mav_5_1.parm` **aracın gerçek değerleridir.** Parm dosyasındaki her
satırın orada aynı değerle göründüğü kontrol edilmelidir:

```bash
cd ~/ardupilot
grep -iE "^ATC_ANGLE_MAX|^WP_ACC|^WP_JERK|^WP_SPD" mav_5_1.parm
```

Eski kod yorumlarındaki "ANGLE_MAX 60/65 denendi, crash etti" gibi notlar bu
yüzden **geçersizdi** — o denemelerde parametre zaten hiç uygulanmıyordu, drone
hep 30°'deydi.

---

## 7. DİKEY EKSEN: TIRMANMA VE ALÇALMA

### 7.1 Tırmanma çok yavaştı

**Şikayet:** "Hedef 100 m'den 200 m'ye çıktı, bizim araç çok yavaş irtifa kazanıyor."

**İki sebep bulundu:**

**a) Güdümün kendi tavanı çok düşüktü.** `VZ_MAX = 3.0 m/s` — araç `WP_SPD_UP = 8 m/s`
tırmanabiliyorken güdüm üçte birinde tutuyordu. 100 m'lik fark **33 saniye** sürüyordu.

**b) Dikey feedforward hiç yoktu.** Yatayda hedefin hızını eşliyorduk ama dikeyde
sadece hata düzeltmesi vardı (`hata × Kp`). Bu, hedef sürekli tırmanırken **kalıcı
geri kalma** demek: kararlı durumda hep `hedefin_tırmanma_hızı ÷ Kp` kadar altta
kalınır.

**c) Dikeyde gölge kullanılıyordu.** Yatayda gölge doğru ("aynı yoldan git"), ama
irtifada istediğimiz "hep 6 m altında olmak" — hedefin **anlık** irtifasına geçildi.

### 7.2 Tırmanma testi sonucu

Hedef 64 m'den 208 m'ye çıkarıldı (91 m'lik ani tırmanış):

| t | hedef | avcı | fark |
|---|---|---|---|
| 6 s | 110.4 m | 103.9 m | 6.5 m |
| 18 s | 162.8 m | 148.8 m | **14.0 m** (en kötü) |
| 30 s | 214.3 m | 209.0 m | 5.4 m |
| 54 s | 202.7 m | 198.8 m | 3.9 m |

Tırmanma hızı 8.5 m/s (tavana vurdu). Yatay takip bozulmadı.

### 7.3 Hızlı alçalma drone'u çaktırdı ❌

`VZ_MAX = 8` tırmanma için doğruydu ama **alçalma için tehlikeliydi**.

Hedef 65 m'den daldığında avcı peşinden indi ve kontrolü kaybetti:

```
t=319.4  güdüm 60° komut ediyor   aracın yaw'ı   77°
t=319.9  güdüm 59° komut ediyor   aracın yaw'ı  134°
t=320.4  güdüm 59° komut ediyor   aracın yaw'ı -116°   ← 250° dönmüş
```

**Kritik ayrıntı:** roll/pitch küçük kalmıştı (0–9°). Yani bu bir **savrulma değil**,
**yaw otoritesi kaybı**. Hızlı alçalışta motorlar düşük itkiye iner ve yaw'ı tutacak
diferansiyel itki kalmaz.

**Çözüm:** Tırmanma ve alçalma tavanları **ayrıldı**:
- `VZ_YUKARI = 8.0` (tırmanış hızlı kalsın)
- `VZ_ASAGI = 3.5` (yaw otoritesi korunsun)

> Hedefi kaybetmek çakılmaktan iyidir.

---

## 8. GÜVENLİK: YERE ÇARPMA VE KONTROL KAYBI KORUMALARI

Avcı kontrolü kaybedip düşmeye başlarsa takip **anlamsızdır**; tek iş hayatta
kalmaktır.

### Koruma üç şeye birden bakar

1. **Mutlak taban:** irtifa `KURTARMA_ALT` (12 m) altına indiyse
2. **Çarpma süresi:** `irtifa ÷ düşüş_hızı` < `KURTARMA_SURE` (3 s) — yüksekte bile
   serbest düşüşteyse yakalar
3. **Yaw sapması:** burun komuttan `YAW_SAPMA_SINIR` (75°) fazla saptıysa = kontrol kaybı

### Devreye girince

- **Yatay komut sıfırlanır** (tüm itki dikeye gider)
- **Tam gaz tırmanılır** — bu aynı zamanda throttle'ı yükseltip yaw otoritesini
  geri kazandırır
- Çıkış `GUVENLI_ALT` (35 m) ile **histerezisli** — eşiğin başında açılıp kapanıp
  salınım üretmesin diye

### Doğrulama (masa testi, 5 senaryo)

| Senaryo | Beklenen | Sonuç |
|---|---|---|
| 100 m, sabit | karışma | ✅ normal takip |
| 100 m, 8 m/s alçalıyor (12 sn var) | karışma | ✅ normal takip |
| 20 m, 8 m/s alçalıyor (2.5 sn kaldı) | devreye gir | ✅ koruma |
| 8 m, sabit (taban altı) | devreye gir | ✅ koruma |
| 60 m, 25 m/s serbest düşüş (2.4 sn) | devreye gir | ✅ koruma |

### Canlı test — hedef yere çakıldı

Hedef 57.5 m'den yere daldırıldı:

```
t=63   hedef 57.5 m   avcı 56.2 m
t=67   hedef 12.5 m   avcı 42.4 m
t=69   hedef  0.0 m   avcı 35.8 m
sonra  hedef yerde     avcı 15.3 m'de DURDU
```

**Avcı çakılmadı**, `MIN_ALT` tabanında kaldı, `armed=True`. Kurtarma logu
tetikleyen kriteri de gösterdi: çarpma süresi değil (15 saniye vardı), **yaw
sapmasıydı**.

### Ek koruma: yakın mesafede yaw dondurma

Avcı hedefin tam üstündeyken (ölçümde 0.2 m'ye kadar indi) "hedefe bakan açı"
tanımsızlaşıyor, en ufak gürültü burnu çeviriyordu (sapma 57°'ye çıkmıştı).
`YAW_MIN_MESAFE` (4 m) altında burun son yönde tutulur.

---

## 9. GÜDÜMÜN DIŞINDAKİ ARIZALAR

Bu arızalar güdümde değildi ama **güdümün ölçülmesini imkânsız kılıyordu**.

### 9.1 Arayüz gecikmesi — üç ayrı sebep

| Sebep | Belirti | Çözüm |
|---|---|---|
| MAVLink kuyruğu birikiyordu (tek mesaj okunuyordu) | telemetri saniyelerce gecikmeli | her okumada kuyruğu **tamamen boşalt** |
| Tarayıcı eski `script.js`'i önbellekten alıyordu | düzeltmeler hiç görünmüyordu | sürüm damgası + `no-store` başlığı |
| Kamera kuyruğu doluyordu (30 Hz giriş / 14 Hz çıkış) | görüntü gecikmesi | callback yalnız **en son kareyi** tutar, işlemeyi ayrı thread yapar |

Ayrıca MAVProxy `--streamrate=25` ile paket hızı 3.9 Hz'den yükseltildi.

### 9.2 CUDA çalışmıyordu — YOLO 16 kat yavaş

`nvidia-smi: Driver/library version mismatch`. Reboot çözmedi çünkü **doğru kernel
modülü diskte yoktu**: `dkms status` modülü `added` gösteriyordu (derlenmemiş).
Sebep: kernel `gcc-12` ile derlenmiş, makinede yalnız `gcc-11` vardı; DKMS
`cc: unrecognized command-line option '-ftrivial-auto-var-init=zero'` alıp sessizce
vazgeçmişti.

Çözüm: `gcc-12` kuruldu → modül yeniden derlendi → `update-initramfs` → reboot.

| | Süre | FPS |
|---|---|---|
| CPU | 58.2 ms/kare | 17 |
| **GPU (RTX 4060)** | **3.6 ms/kare** | **277** |

Artık `vision/detector.py` ve `pose_detector.py` seçtikleri cihazı **açıkça basar**;
CPU'ya düşerse uyarı verir — bir daha sessizce yavaşlamasın diye.

### 9.3 Gazebo GPU'yu hiç kullanmıyordu

Makine hibrit grafikli (`prime-select query` = `on-demand`); NVIDIA yalnız açıkça
istenirse devreye girer. Gazebo Intel iGPU'da (hatta yazılım render'da) koşuyordu.

| | RTF | Gazebo CPU | GPU |
|---|---|---|---|
| Intel/yazılım | **0.71** | %404 | boş |
| NVIDIA | **1.00** | %355 | 1054 MiB |

RTF 0.71 demek simülasyon gerçek zamanın %71'i hızında koşuyor demekti — telemetri
ve kamera da o oranda geç geliyordu.

Çözüm (`start_harmonic.sh` ve dokümanlara eklendi):
```bash
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
```

### 9.4 Hedef irtifası kaçıyordu

Hedef uçak durmadan yükseliyordu (46 m → 124 → 322 → 413 m). Bu hem görevi imkânsız
kılıyor hem testleri geçersiz yapıyordu. `run_plane_scenario.py`'ye irtifa P denetimi
eklendi.

### 9.5 Kalkış yatay hareketi engelliyordu

Chase, hedefin irtifasına kalkmaya çalışıyordu; drone dakikalarca tırmanıp yatayda
hiç ilerlemiyordu. Kalkış sabit 30 m'ye çekildi, irtifa farkını güdüm yatayda
ilerlerken kapatıyor. Araç zaten havadaysa kalkış atlanıyor.

---

## 10. ÖLÇÜM YÖNTEMİ — UÇUŞ KAYDI

5 saniyede bir örnek alarak manevrayı anlamak **imkânsızdı** — manevra ~8 saniye
sürüyor, elde 1-2 nokta kalıyordu. Bu yüzden güdüme 20 Hz'lik CSV kaydı eklendi.

### Açma

```bash
export AVCI_GUDUM_LOG=1
export AVCI_GUDUM_LOG_YOL=/tmp/gudum_kayit.csv   # istege bagli
```

Varsayılan **kapalıdır**; normal uçuşta hiçbir maliyeti yoktur.

### Kaydedilen sütunlar

| Sütun | Anlamı |
|---|---|
| `t` | zaman (monotonic) |
| `tx, ty, tz` | hedefin konumu (NED) |
| `tvx, tvy` | hedefin hızı |
| `gx, gy, gz` | gölge noktası |
| `gvx, gvy` | **gönderilen** hız komutu |
| `ix, iy, iz` | avcının konumu |
| `ivx, ivy` | avcının **gerçek** hızı |
| `iroll, ipitch, iyaw` | avcının duruşu |
| `cmd_yaw` | komut edilen burun açısı |
| `d_h` | hedefe yatay mesafe |
| `d_golge` | gölge noktasına mesafe |
| `mod` | `POS` / `VEL` |

**Bu kaydın kritik değeri:** `gvx,gvy` (komut) ile `ivx,ivy` (gerçekleşen) yan yana
durur. Manevradaki asıl arıza ancak bu ikisinin ayrıştığı görülünce bulundu —
"komut 20 m/s, araç 14.3 m/s" tespiti buradan çıktı.

---

## 11. SONUÇLAR TABLOSU

### Yatay takip (230 saniyelik kesintisiz takip, kare senaryo)

| Ölçüm | Sonuç |
|---|---|
| Hedefe mesafe (ortalama) | **10.6 m** |
| Hedefe mesafe (en kötü) | **12.4 m** |
| Gölge noktasına hata | **1.10 m** |
| Düz uçuşta gölge hatası | **0.3 – 0.9 m** |
| İrtifa farkı | **6.0 m** (istenen 6.0) |
| Duraklama (<3 m/s) | **0 kez** |
| 25 m'den fazla geri kalma | **%0.0** |

### Manevra

| Ölçüm | Fabrika ayarlarıyla | Şimdi |
|---|---|---|
| Manevrada geri kalma | 39–48 m | **≤12.4 m** |
| Aracın yattığı max açı | 17–18° | **49°** |
| Manevrada araç hızı | 14.3 m/s'de takılı | komutu takip ediyor |

### Dikey

| Ölçüm | Sonuç |
|---|---|
| 91 m ani tırmanışta en kötü fark | **14.0 m** |
| Tırmanma hızı | **8.5 m/s** |
| Hedef yere daldığında | avcı **15.3 m**'de durdu, çakılmadı |

### Sistem

| Ölçüm | Önce | Sonra |
|---|---|---|
| YOLO çıkarım | 58.2 ms (CPU) | **3.6 ms** (GPU) |
| Gazebo RTF | 0.71 | **1.00** |
| GCS CPU kullanımı | %793 | **%59** |

---

## 12. NASIL ÇALIŞTIRILIR

Ayrıntı: `docs/SIMULASYON_CALISTIRMA.md`. Özet:

```bash
# 1) Gazebo — NVIDIA render satırı ATLANMAMALI
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
gz sim -r -v4 sim/gazebo_harmonic/worlds/avci_harmonic.sdf

# 2) ArduCopter (avcı)
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  -I0 --sysid 5 --no-rebuild \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_copter.parm \
  --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 \
  --mavproxy-args="--streamrate=25"

# 3) ArduPlane (hedef)
python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 \
  -I1 --sysid 2 --no-rebuild \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_plane.parm \
  --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 \
  --mavproxy-args="--streamrate=25"

# 4) GCS
cd ~/projects/avci_sim && source /opt/ros/humble/setup.bash
export AVCI_GZ_CAMERA=1
python3 -m gcs.gcs_server
```

Sonra arayüzden hedef senaryosunu (kare/daire/agresif) ve **Chase**'i başlatın.

### Ortam değişkenleri

| Değişken | Varsayılan | Etkisi |
|---|---|---|
| `AVCI_HYBRID` | `off` | `on` → görsel faz devreye girer |
| `AVCI_GPS_LAW` | `yaklasma` | `v2` → eski chase durum makinesi |
| `AVCI_GUDUM_LOG` | *(kapalı)* | `1` → 20 Hz CSV uçuş kaydı |
| `AVCI_HEDEF_IRTIFA` | `60` | Hedef senaryosunun koruyacağı irtifa (m) |
| `AVCI_GZ_CAMERA` | — | `1` → Gazebo kameraları dinlenir |

---

## 13. BİLİNEN SINIRLAR VE SIRADAKİ İŞLER

### Fiziksel sınır

Avcı bir quadcopter, hedef sabit kanat. `ATC_ANGLE_MAX = 45°` ile aracın yatay ivme
tavanı `9.81 × tan(45°)` = 9.8 m/s². 55°'ye çıkarılabilir (14 m/s²) ama kararlılık
birkaç tur izlenmeden yapılmamalı — 45°'de quad dikey itkisinin %71'ini korur,
55°'de %57'sine düşer.

### Doğrulanmamış

- **Yakın mesafede yaw dondurma** (`YAW_MIN_MESAFE`) masada doğrulandı, canlı uçuşta
  ölçülmedi.
- **ArduPlane parametreleri:** `avci_plane.parm`'daki `TRIM_ARSPD_CM`,
  `ARSPD_FBW_MIN`, `ARSPD_FBW_MAX` de araca uygulanmıyor (aynı isim değişikliği
  sorunu). Hedef şu an 15 m/s uçtuğu için acil değil ama düzeltilmeli.

### Kapalı duran

- **Görsel güdüm** (`AVCI_HYBRID=off`): görsel faz hedefi kadrajda bulamayınca drone
  havada duraklıyordu. Kod silinmedi.
- `gps_chase.py` ve `gps_strike.py` varsayılanda kullanılmıyor.

### Öneriler

1. Manevra performansı yeterli değilse önce `ATC_ANGLE_MAX`'ı 50-55°'ye çıkarıp
   ölçün — güdüm tarafında yapılacak fazla bir şey kalmadı.
2. Mesafeyi değiştirmek için **tek** ayar: `GECIKME`.
3. Herhangi bir "çalışmıyor" durumunda **önce ayarın araca ulaştığını doğrulayın**
   (bölüm 6 sonundaki komut).

---

## EN ÖNEMLİ DERS

Bu projede en çok zaman kaybettiren şey, **arızayı yanlış yerde aramaktı**.

Manevradaki geri kalma haftalarca güdüm algoritmasının hatası sanıldı; kazançlar
değiştirildi, yeni algoritmalar yazıldı, filtreler eklendi. Oysa güdüm doğru komutu
veriyordu — **araç o komutu uygulayamıyordu**, çünkü performans parametreleri hiç
uygulanmamıştı.

Bunu gösteren tek şey **ölçüm** oldu: komut ile gerçekleşen değerin yan yana
kaydedilmesi. O iki sütun ayrışmasaydı kök neden hâlâ bulunmamış olurdu.

> Bir şey çalışmıyorsa: önce ölç, sonra değiştir.
> Ölçemiyorsan, önce ölçme aracını yap.
