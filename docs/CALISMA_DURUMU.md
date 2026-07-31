# ÇALIŞMA DURUMU — Görsel Güdüm (2026-07-31)

Bu belge, avcı drone'un görsel güdüm fazında yaşanan iki problemin teşhisini,
yapılan değişiklikleri ve bekleyen işleri kaydeder. Amaç: başka bir makinede
ya da başka bir kişiyle devam edilebilsin.

**Dal:** `kubra_laptop` — `main` üstüne kurulu, henüz `main`'e birleştirilmedi.

---

## Özet — nerede duruyoruz

İki ayrı problem var ve **birbirinden bağımsız**:

| Problem | Durum | Nerede |
|---|---|---|
| Drone hedefin altından geçiyor (dikey ıska) | Kısmen çalışıldı, sürüyor | Görsel faz + GPS fazı |
| Drone kendi etrafında dönüyor | Kök neden bulundu, uygulanmadı | Firmware parametresi |
| Arayüz telemetrisi donuyor | Teşhis edildi, uygulanmadı | `gcs_server` |

Ölçümlerin tamamı `logs/` altındaki uçuş CSV'lerinden çıkarıldı (loglar
`.gitignore`'da, repoya girmiyor).

---

## Problem 1 — Dikey ıska: drone hedefin altından geçiyor

### Belirti

Görsel güdüm sağlanıyor, drone yaklaşıyor, ama hedefin altından çarpmadan
geçiyor. En yakın anda `pitch_hata` **her uçuşta pozitif**: +22°, +35°, +36°,
+42°, +56°, +76°. Yani hedef her seferinde burnun üstünde kalıyor.

### Kök neden — "sabit metre" ile "sabit açı" farkı

GPS fazı drone'u hedefin **sabit 4.65 metre altında** park ediyor
(`gps_guidance.py`: `RANGE_SET(11) × sin(CENTER_ELEV_DEG(25°))`). Bu, kameranın
+25° yukarı tilt'i yüzünden bilinçli bir tasarım: hedef kadrajın merkezinde ve
gökyüzü arka planında kalsın diye.

**Ama sabit METRE, kapanan menzilde sabit AÇI değildir:**

| menzil | dikey ofset | LOS yükselişi | kadrajda nerede |
|-------:|------------:|--------------:|-----------------|
| 11 m | 4.65 m | 25° | merkez |
| 8 m | 4.65 m | 35° | merkezden 11° yukarıda |
| 6 m | 4.65 m | 51° | merkezden 26° yukarıda |
| 4 m | 4.65 m | >90° | **kadrajın dışında** |

Kamera kadrajının üst sınırı **+80.2°** (`tests/test_visual_lead.py` T12).
Ölçtüğümüz `pitch_hata +76°` tam o sınır — hedef kadrajın tepesinden çıkıyor,
tespit kopuyor, drone altından geçiyor. Kör dalış mekanizması
(`TERMINAL_MENZIL`/`TERMINAL_SURE`) zaten bunu örtmek için var.

**Yani mevcut tasarım, korumak istediği görsel teması yakın menzilde kendisi
bozuyor.** Alttan yaklaşma ilkesi doğru; yanlış olan ofsetin metre cinsinden
sabit tutulması.

### Doğrulanmış ölçümler

- `gps_guidance_20260731_193548.csv`: yatay menzil 38.5 → 6.4 m'ye inerken
  drone hedefin **4.6 → 4.2 m altında** kalıyor. Dikey fark kapanmıyor.
- Aynı log: dikey komut −0.32 m/s, **gerçekleşen −0.39 m/s**. Yani araç komutu
  sadıkla uyguluyor — sorun araçta veya güdüm çıkışında değil, komutun kendisi
  küçük.
- Görsel faz 20 m yerine **6-10 m'de** başlıyor, elinde 0.6-1.9 s kalıyor.
  4.2 m'yi bu sürede kapatmak 5-6.6 m/s dikey hız ister.
- Karelerin yalnız **%12'si** temiz `ok`; %36 `kpt_dusuk`, %28 `tespit_yok`,
  %20 kör dalış.

### Asıl dinamik — faz gidip gelmesi

`19:35` uçuşunda görsel faz **4 KEZ** başlayıp koptu (her biri 1-1.9 s). Her
kopuşta GPS fazı drone'u istasyona (hedefin 4.65 m altına) **geri çekiyor** —
görsel fazın kazandığı irtifa geri alınıyor.

---

## Problem 2 — Drone kendi etrafında dönüyor

### Kök neden: motor doygunluğu (firmware)

`sim/ardupilot_params/avci_copter.parm` içinde **`ANGLE_MAX 7000`** — 70° eğime
izin veriliyor. Bu, aracın fiziksel kapasitesinin üstünde:

| eğim | gereken itki | kullanılan gaz | yaw yetkisi |
|---|---:|---:|---|
| 55° | 1.74x | %68 | sağlam |
| 65° | 2.37x | %92 | sınırda |
| **70°** | **2.92x** | **%114** | **doygun** |

(`MOT_THST_HOVER=0.39` → itki/ağırlık 2.56x.)

ArduCopter motorlar doyduğunda **önce yaw'ı feda eder** — bilinçli tasarım.
Yaw yetkisi sıfırlanınca araç serbest döner ve dönme hızlanır.

### Doğrulanmış ölçümler

- `gps_guidance_20260731_211556.csv`: **36.9 saniyede 7290° gerçek dönüş**
  (20 tam tur), komut edilen yaw ise sadece 24°.
- `gps_guidance_20260731_181940.csv`: yaw komutu sabit 99.2°'de dururken araç
  500-1100 °/s hızlanarak döndü.
- Ayırt edici imza: dönme yalnız **manevra** sırasında oluyor; hız tavana
  oturup düz uçuşa geçince kendiliğinden duruyor.
- `iris_roll_deg` ve `iris_pitch_deg`, yaw ile **çeyrek faz kaymalı** salınıyor
  — bu fiziksel dönmenin imzasıdır, telemetri hatası değil.
- Görsel fazda yaw uslu (kare başına max 3-9°). Dönme **GPS fazında**.

### Durum

`ANGLE_MAX` 5500'e çekildi, sonra kullanıcı isteğiyle `main` hâline geri
döndürüldü. **Bulgu uygulanmadı**, GPS fazı incelemesine bırakıldı.
Düzeltmek isteyen `ANGLE_MAX 5500` yapar (gereken 1.74x, gaz %68, yaw için bol
pay; yatay ivme tavanı 14 m/s² ve guidance `MAX_ACCEL=12` bunun altında kalır).

---

## Problem 3 — Arayüz telemetrisi donuyor

Drone verisi **14541 portundan** geliyor; bu portu aynı anda **tek program**
dinleyebiliyor.

- **GPS fazında** güdüm portu okurken `telemetry_state`'i de besliyor. Ölçüldü:
  737 satırın **0'ı** tekrar — telemetri kesintisiz canlı.
- **Görsel fazda** `run_visual_lead` veriyi kendi `_ArasState`'ine çekiyor ve
  `telemetry_state`'e **hiç yazmıyor** → arayüz son değerde donuyor.

`21:14-21:16` oturumunda görsel faz 6 kez çalıştı, toplam 20.4 s → arayüz
zamanın **%16'sında donuk**, 6 kez donup çözüldü.

Kalkış öncesi drone orijinde durduğu için donan değerler ≈0; ekranda "hepsi
sıfır" gibi görünüyor. Değerler sıfırlanmıyor, **donuyor**.

**Çözüm (uygulanmadı):** iris SITL zaten **14550'ye de** yayın yapıyor
(`scripts/start_harmonic.sh`). `gcs_server.mavlink_listener` o paketleri alıp
atıyor — `sysid_is_plane.get(sys_id, False)` False dönünce hiçbir şey yapmıyor.
Oraya bir `else` dalı ekleyip quadrotor sysid'ini `"iris"` olarak işlemek
yeterli (~3 satır, güdüm koduna dokunmadan).

---

## Bu dalda YAPILANLAR

Hepsi görsel güdüm tarafında; `gps_guidance.py`'a ve firmware parametrelerine
dokunulmadı.

### 1. Yaw/ivme limitinde `dt` tavanı — `guidance_core.DT_TAVAN_S`

`YAW_HIZ_MAX` ve `IVME_TAVAN` birer **hız** limiti; kare başına paya
çevrilirken `dt` ile çarpılıyor. Tespit kesilince `dt` şişiyor (boşluğun tamamı
tek `dt`'ye biniyor) ve biriken hak tek karede harcanıyor. `160249` uçuşunda
`dt=0.825 s` → 74°'lik yaw adımı tek MAVLink mesajında gitti.

Limit hesabı artık `min(dt, 0.1 s)` kullanıyor. Ham `dt` filtre/PN'de aynen
kalıyor (orada türev paydası, harcanacak pay değil).

**Ölçüm:** aynı log yeniden oynatıldığında max yaw adımı **74.2° → 9.0°**,
`vz` kare-arası sıçraması 2.3 → 0.3 m/s.

### 2. Azimut tekilliği kapısı — `AZIMUT_TAM/TEKIL_YUKSELIS_DEG`

`yaw_hata = atan2(u_govde[1], u_govde[0])` gövde azimutudur; nişan dikeye
yaklaşınca yatay bileşen sıfıra iner ve **tanımsızlaşır**. `141017` uçuşunda
ardışık üç kare: yatay 0.446 → 0.027 → 0.156, `yaw_hata` +64.7° → −154.2° →
+130.8°.

Artık her karede `azimut_kalite` (0..1) üretiliyor ve yaw adımı onunla
çarpılıyor. Tepedeki hedefte yaw susuyor.

### 3. İvme tavanı yatay/dikey ayrıldı — `IVME_TAVAN` / `IVME_TAVAN_DIKEY`

İki eksenin fiziği farklı:
- **Yatay** ivme burun eğimi gerektirir → kamera (+25° sabit) aşağı bakar,
  gökyüzü arka planı kaybolur. 4 m/s²'de eğim 22°, kamera +2.8°.
- **Dikey** ivme burun eğimi GEREKTİRMEZ — yalnız itki artar, kamera sabit.
  22° eğimde 10 m/s² → gaz %85 (yaw için pay kalır).

Tek 3B tavan, kameranın **yatay** kısıtını dikeye de dayatıyordu — tam da
kaçırdığımız eksende. Dikey tavan 4 → **10 m/s²**.

**Ölçüm:** dikey PN karelerin %27-78'inde tavanındayken `v_doygun` %93-99'du.
Ayrımdan sonra `vz_cmd` −19.5 m/s'ye çıkabiliyor.

### 4. Kadraj tutma — "metre altta kal" değil "AÇI altta kal" — `KP_KADRAJ`

Nişanın **gövde çerçevesindeki** yükselişi kadraj merkezinden
(`KAMERA_TILT_DEG`) saptıkça orantılı düzeltme uygulanıyor. Ofset böylece
menzille orantılı küçülüyor. Simetrik: hedef merkezin altındaysa nişan aşağı
iniyor.

Gövde çerçevesi önemli — hedefin sensörde nereye düştüğünü bu belirler, dünya
yükselişi değil (aradaki fark aracın kendi pitch'i).

**Kazanç taraması** (kapalı çevrim simülasyon, %31 tespit kopukluğu + araç
gecikmesi):

| KP_KADRAJ | ıska | en kötü | vuruş |
|---|---:|---:|---|
| 0.0 (kapalı) | 0.59 m | 1.39 m | 24/24 |
| **0.5 (seçilen)** | **0.55 m** | **1.15 m** | 24/24 |
| 1.0 | 1.59 m | 3.23 m | 20/24 |
| 1.5 | 4.90 m | 5.45 m | **0/24** |

### 5. Faz kapıları: ardışık → kayan pencere

- **Giriş** (`SupCfg.KILIT_N/KILIT_PENCERE`): 10 ardışık güvenli kare yerine
  son 15 karenin 10'u. Matematiksel olarak asla daha geç tetiklenmez.
  *Eldeki loglarda ölçülebilir fark üretmedi* (giriş GPS fazında olur, orası
  loglanmıyor).
- **Çıkış** (`Cfg.KAYIP_PENCERE/KAYIP_MIN_ISABET`): 20 ardışık `tespit_yok`
  yerine "son 40 karede en az 4 tespit". **Gerçek loglarda 5 erken bırakma → 0.**
  Bu, faz gidip gelmesini doğrudan azaltıyor.

### 6. Menzil makullük kapısı — `_MenzilKapisi`

`menzil_gercek_m` fiziksel olarak imkânsız zıplıyor: `193559` uçuşunda 33 ms'de
22.4 → 6.6 m (= 479 m/s) örneği geldi ve kod bunu **VURULDU** saydı. Doğrulanan
7 gerçek uçuş vuruşunun **1'i sahteydi**.

Sinyal yalnız log değil — kör dalış tetiğini ve terminal co-altitude kilidini
de besliyor.

Kapı: iki örnek arası değişim `MENZIL_HIZ_TAVAN(50 m/s)·dt`'yi aşarsa örnek
reddediliyor, son geçerli değer korunuyor. `MENZIL_RESENK_N(8)` ardışık red
sonrası yeni seviyeye senkronize oluyor (bayat değere kilitlenmesin).

**Ölçüm:** tüm gerçek uçuş loglarına uygulandığında **sahte vuruş elendi,
6 gerçek vuruşun hepsi korundu.**

### 7. Yeni CSV sütunları (gözlem)

`yatay_bilesen`, `azimut_kalite`, `yaw_adim_deg`, `menzil_ham_m`, `menzil_red`,
`los_elev_deg`, `los_elev_rate_dps`, `gama_deg`, `kadraj_hata_deg`,
`kadraj_duz_deg`.

**Başarı ölçütü:** `kadraj_hata_deg` sıfıra yakın kalmalı (hedef kadraj
merkezinde). Şu ana kadar terminalde +50°'ye çıkıyordu.

---

## DENENİP GERİ ALINANLAR — tekrar denemeyin

### Gerçek PN (`γ += N·Δλ`)

Klasik oransal seyrüsefer uygulandı, testleri geçti (γ, λ'dan tam 2.00 kat
hızlı değişiyordu). Kapalı çevrim simülasyonda ıska **0.66 m → 1.5-2.1 m'ye
ÇIKTI**.

**Sebep:** devir anında λ zaten doğal olarak azalıyor (drone hedefe
yakınsıyor). PN bunu "sıfırlanacak LOS hızı" sanıp yakınsamayla savaşıyor ve
hedef yukarıdayken γ eksiye (**dalışa**) gidiyor. İzlendi ve doğrulandı:
γ −22°'ye inerken λ hâlâ +1.3°'deydi.

PN küçük sapmaları düzeltmek içindir, büyük bir başlangıç ofsetini kapatmak
için değil. Gerekçe `adapter_copter._dikey_pn` docstring'inde duruyor.

### Dikey PN'i güçlendirme (tavan 15°→30°, süre 0.4→0.6 s)

PN yeni tavana da %79 oranında çakıldı — doygunluk noktası yukarı kaydı,
kalkmadı. `PN_LEAD_SURE`/`PN_DIKEY_MAX_DEG` şu an 0.6/30 değerlerinde duruyor
ama etkisi sınırlı.

### `KP_KADRAJ ≥ 1.0`

Yüksek kazanç yakınsamayla savaşıp salınım üretiyor — 1.5'te 0/24 vuruş.

---

## BEKLEYEN GÖREVLER

| # | İş | Boyut | Nerede |
|---|---|---|---|
| 1 | **GCS telemetrisi** — 14550'den iris paketlerini de işle | ~3 satır | `gcs_server.mavlink_listener` |
| 2 | **`ANGLE_MAX` 7000 → 5500** — kendi etrafında dönmeyi bitirir | 1 satır | `avci_copter.parm` (Kayra) |
| 3 | **GPS istasyon geometrisi** — `d_below` menzille orantılı küçülsün | orta | `gps_guidance.py` (Kayra) |
| 4 | **Lead'in yumuşak geçişi** — `kpt_dusuk`'ta sert 0'lanıyor, ~15° nişan zıplaması (58 geçiş, ort 10.8°, max 24.9°) | orta | `guidance_core.process` |
| 5 | **Menzil verisi neden zıplıyor** — kapı semptomu kesti, kök neden duruyor. Baş şüpheli `gcs_server._frame_off` dikey kalibrasyonu (`sd = 0.0` varsayımı) | uzun | `gcs_server` |
| 6 | **Hedef açısını ArduPilot'tan al** — pose keypoint yerine doğrudan telemetri | orta | netleştirilecek |

**Görev 6 için açık soru:** hedefin telemetrisini kullanmak bir simülasyon
kolaylığıdır — gerçek harekâtta düşman uçağın telemetrisi olmaz, görsel
güdümün varlık sebebi zaten budur. Güdüm hattında kalıcı mı kullanılacak,
yoksa pose modelinin hatasını ölçmek için referans mı? İkisi çok farklı iş.

---

## Testler

    python3 -m tests.test_visual_lead      # 44/44
    python3 -m tests.test_gps_guidance     # 9/9

Yeni testler: T30-T33 (azimut tekilliği), T34-T35 (limit `dt` tavanı),
T36-T37 (yatay/dikey ivme ayrımı), T38/T38b (menzil kapısı),
T39/b/c/d (kadraj tutma).

---

## Başka bir makinede devam etmek

Bu dalı çekmek koda ve dokümana erişim verir:

    git clone https://github.com/kayranecatikara/hamidiyesim.git
    cd hamidiyesim
    git checkout kubra_laptop

**Ama simülasyonu ÇALIŞTIRMAK için ayrıca şunlar gerekir** (repoda yok):
ArduPilot SITL (`~/ardupilot`), `ardupilot_gazebo` eklentisi, Gazebo Harmonic,
ROS2 Humble, YOLO ağırlıkları. Kurulum ve çalıştırma:
`docs/SIMULASYON_CALISTIRMA.md` (headless anlatılıyor).

**Uçuş logları repoda YOK** — `logs/` `.gitignore`'da. Bu belgedeki tüm
ölçümler o loglardan çıkarıldı; sayılar burada kayıtlı, ham veri değil.
Log gerekiyorsa ayrıca kopyalanmalı.
