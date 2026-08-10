# 12 — `control/guidance/` Hibrit Güdüm Paketi

> ⚠ **2026-08-10 — BU BÖLÜMLERİN BİR KISMI KEYPOINT DÖNEMİNE AİT.**
> Görsel güdüm 2026-08-06'da keypoint zincirinden çıktı; aktif yasa artık
> `control/guidance/bbox_ibvs.py` (yalnız tespit kutusu). Kamera↔güdüm köprüsü
> KARE köprüsüdür (`wait_new_frame` / `kayit["det"]`). Aşağıda keypoint'lerden,
> `pose_detector`'dan ya da `process(pose, ...)` imzasından söz eden her yer
> TARİHSELDİR → [`POSEA_GERI_DONMEK_ISTERSENIZ/`](../POSEA_GERI_DONMEK_ISTERSENIZ/README.md).
> Canlı davranış için kaynak kodu esastır.


> **Projenin kalbi.** Avcı drone'un hedefi *nasıl* vuracağını belirleyen tüm
> matematik ve karar mantığı. Fonksiyon bazında açıklama.

---

## Paket haritası ve bağımlılık yönü

```
                    ┌─────────────────┐
                    │  supervisor.py  │  ← görev döngüsü (gcs_server çağırır)
                    │   run_hybrid()  │
                    └────┬───────┬────┘
              GPS fazı ──┘       └── görsel faz
                    │                 │
       ┌────────────▼──────┐   ┌──────▼─────────────┐
       │  gps_guidance.py  │   │  visual_lead.py    │
       │  20 Hz sabit döngü│   │  kareye kilitli    │
       └────────┬──────────┘   └──────┬─────────────┘
                │                     │
                │              ┌──────▼──────────────┐
                │              │ guidance_core.py    │ ← PLATFORMDAN BAĞIMSIZ
                │◄─────────────┤ LeadPursuitCore     │   saf matematik, IO yok
                │ hedef_kadraj │ + frame dönüşümleri │
                │  _hatasi()   └──────┬──────────────┘
                │                     │
                │              ┌──────▼──────────────┐
                │              │ adapter_copter.py   │ ← PLATFORMA BAĞLI
                │              │ CopterAdapter       │   quad komut üretimi
                │              └──────┬──────────────┘
                └─────────┬───────────┘
                          ▼
                  ┌───────────────┐
                  │   common.py   │  send_velocity → MAVLink GUIDED
                  └───────────────┘
```

**Katman ayrımının anlamı:** `guidance_core` bir **yön** üretir (nereye
nişan alacağız). `adapter_copter` o yönü **quad komutuna** çevirir. Sabit kanat
avcı eklenirse yalnız yeni bir adaptör yazılır, çekirdek aynı kalır.

---
---

# `common.py`

**Yol:** `control/guidance/common.py` · **74 satır**
**Rol:** İki hattın paylaştığı skaler matematik + **tek MAVLink göndericisi**.

---

## `clamp(val, lo, hi)`

```python
return max(lo, min(hi, val))
```
Değeri aralığa sıkıştırır. Paket boyunca yaw adımı, dikey hız, PN açısı gibi her
sınırlamada kullanılır.

---

## `normalize_angle(a)`

```python
while a > math.pi:  a -= 2 * math.pi
while a < -math.pi: a += 2 * math.pi
return a
```

**Dönüş:** `[-π, π]` aralığında açı.

**Neden kritik:** Yaw hatası `hedef − mevcut` olarak hesaplanır. 350° ile 10°
arasındaki fark ham hâliyle `-340°` çıkar — araç uzun yoldan döner. Sarma sonrası
`+20°` olur, kısa yoldan döner.

---

## `vec3_len(x, y, z)`

```python
return math.sqrt(x*x + y*y + z*z)
```

---

## `limit_acceleration(vx_cmd, vy_cmd, vz_cmd, vx_p, vy_p, vz_p, max_a, dt)`

**Dönüş:** sınırlanmış `(vx, vy, vz)`

```python
if dt <= 0:
    return vx_cmd, vy_cmd, vz_cmd

dvx, dvy, dvz = vx_cmd - vx_p, vy_cmd - vy_p, vz_cmd - vz_p
dv = vec3_len(dvx, dvy, dvz)
max_dv = max_a * dt

if dv > max_dv and dv > 0:
    s = max_dv / dv
    return vx_p + dvx * s, vy_p + dvy * s, vz_p + dvz * s
return vx_cmd, vy_cmd, vz_cmd
```

### Neden **vektörel** sınırlama
Bileşenler ayrı ayrı kırpılsaydı hız vektörünün **yönü değişirdi**. Örnek:
`(10, 10, 0)` isteniyor, bileşen sınırı 5 → `(5, 5, 0)` yön korunur, ama
`(10, 2, 0)` → `(5, 2, 0)` yön bozulur.

Buradaki yöntem tüm değişim vektörünü aynı `s` katsayısıyla ölçekler: **yön
korunur, sadece büyüklük yavaşlar.** Güdümde yön her şeydir — nişan yönünün
bozulması ıska demektir.

---

## `send_velocity(conn, vx, vy, vz, yaw)`

Paketin **tek MAVLink çıkışı**. Her iki faz da komutu buradan yollar.

```python
_TYPEMASK_VEL_YAW = (
    (1 << 0) | (1 << 1) | (1 << 2) |   # pozisyon YOKSAY
    (1 << 6) | (1 << 7) | (1 << 8) |   # ivme YOKSAY
    (1 << 9) |                          # force YOKSAY
    (1 << 11)                           # yaw_rate YOKSAY
)
# bit3,4,5 (hız) ve bit10 (yaw) TEMİZ → bu alanlar kullanılır

def send_velocity(conn, vx, vy, vz, yaw):
    conn.mav.set_position_target_local_ned_send(
        timestamp_ms(),
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _TYPEMASK_VEL_YAW,
        0.0, 0.0, 0.0,          # x, y, z — yoksayılır
        vx, vy, vz,             # ← AKTİF (m/s, NED)
        0.0, 0.0, 0.0,          # afx, afy, afz — yoksayılır
        yaw, 0.0                # ← yaw AKTİF (rad), yaw_rate yoksayılır
    )
```

### Neden pozisyon değil hız
Hareketli hedefi kovalarken pozisyon hedefi sürekli "geçmişe" işaret eder —
hedef siz oraya varana kadar başka yerdedir. Hız komutu **anlık kesme
vektörünü** doğrudan ifade eder.

### Neden `yaw_rate` değil `yaw`
Mutlak yaw hedefi verilir, dönüş hızı değil. Slew sınırlaması adaptörde
(`adapter_copter`) uygulanır — kontrolcü değil güdüm karar verir.

---
---

# `guidance_core.py`

**Yol:** `control/guidance/guidance_core.py` · **344 satır**
**Rol:** IBVS lead pursuit çekirdeği. **Platformdan tamamen bağımsız** — IO yok,
MAVLink yok, saf hesap. Birim test edilebilir.

**Temel iddia:** Hedefin hızı ve mesafesi **ölçülmez**. Tek ayar `K_LEAD`.

---

## Fiziksel sabitler

```python
GOVDE_BOYU_M      = 0.81     # Talon gövde uzunluğu (X ekseni)
KANAT_ACIKLIGI_M  = 1.28     # kanat açıklığı (Y ekseni)
GOVDE_KANAT_ORANI = 0.633    # 0.81 / 1.28
```

Gazebo collision mesh'inden ölçülmüş, fabrika X-UAV Mini Talon ile uyumlu.

---

## `class Cfg` — ayar yüzeyi

Kritik olanlar `AVCI_IBVS_*` ortam değişkenleriyle canlı override edilir
(`_env_f` okuyucusu ile).

### Çekirdek ayarları

| Ayar | Değer | Anlamı |
|------|------:|--------|
| `K_LEAD` | 0.5 | ≈ hedef_hızı / bizim_hız. **Tek ayar.** Tarama aralığı 0.0-1.0 |
| `MAX_LEAD_DEG` | 35° | Lead açı tavanı |
| `OLCEK_KAPALI_PX` | 6.0 | `olcek_px = fx·0.81/R = 134.9/R` → 6 px ≈ **22.5 m** (lead yok) |
| `OLCEK_TAM_PX` | 14.0 | → 14 px ≈ **9.6 m** (tam lead) |
| `FILTRE_TAU_S` | 0.12 | 30 Hz'te ~3.6 kare — pencere dar, uzun tutma |
| `MIN_GOVDE_PX` | 2.0 | Altında lead söner (deadband) |
| `FLIP_DT_TAVAN_S` | 0.20 | 30 Hz'te 6 kare — üstünde flip kontrolü atlanır |
| `GECIKME_TAVAN_S` | 0.12 | Bundan bayat kare atlanır (döngü kullanır) |
| `KAMERA_TILT_DEG` | 25.0 | Sabit montaj tilt'i |
| `KPT_CONF_MIN` | 0.5 | Keypoint güven eşiği |
| `YUKSELTI_DUZELT` | True | Adım 3 açık |
| `UNDISTORT_AKTIF` | False | Simde distorsiyon yok; gerçek donanımda True |

### Adaptör ayarları

| Ayar | Değer | Not |
|------|------:|-----|
| `V_KAPANMA` | 25 m/s | Sabit kapanma hızı |
| `KP_YAW` | 1.2 | Yaw oransal kazancı |
| `YAW_HIZ_MAX` | 90 °/s | Yaw slew tavanı — `AVCI_IBVS_YAW_HIZ_MAX` ile override |
| `IVME_TAVAN` | 4 m/s² | Hız rampası — `AVCI_IBVS_IVME_TAVAN` ile override |
| `DT_TAVAN_S` | 0.1 s | Limit hesabındaki dt kırpması — `AVCI_IBVS_DT_TAVAN` ile override |
| `AZIMUT_TAM_YUKSELIS_DEG` | 60° | Altında azimut tam güvenilir (`azimut_kalite`=1) |
| `AZIMUT_TEKIL_YUKSELIS_DEG` | 75° | Üstünde azimut tekil (`azimut_kalite`=0, yaw susar) |

> **Tavanlar geri alındı (2026-07-31).** 2026-07-25'teki "limitsiz test" ayarı
> (`YAW_HIZ_MAX=1080`, `IVME_TAVAN=1000`) slew ve rampayı fiilen kapatıyordu;
> tespit gürültüsü doğrudan gövdeye geçti. `141017` uçuşu: tek karede 136° yaw
> komut sıçraması, 4.1 s'te 637° dönüş, `vz_cmd`'de 8.2 m/s kare-arası sıçrama
> (247 m/s² talep). Tavanlar ayarlı değerlerine döndürüldü, tarama için ikisi de
> env ile override edilebilir.

> **Kadraj tutma — "metre altta kal" değil "AÇI altta kal" (2026-07-31).**
> GPS fazı hedefin **sabit 4.65 m** altında istasyon tutuyor
> (`RANGE_SET·sin(CENTER_ELEV_DEG)`). Sabit METRE, kapanan menzilde sabit AÇI
> değildir — 11 m'de 25° (kadraj merkezi), 6 m'de 51°, 4 m'de kadrajın **dışı**
> (üst sınır +80.2°). Hedef kadrajın tepesinden çıkıyor, tespit kopuyor, drone
> altından geçiyor. GPS fazına dokunmadan görsel faz bunu düzeltir: nişanın
> **gövde çerçevesindeki** yükselişi kadraj merkezinden (`KAMERA_TILT_DEG`)
> saptıkça `KP_KADRAJ` ile orantılı düzeltme uygulanır. Ofset böylece menzille
> orantılı küçülür ve sabit görüş açısı = çarpışma rotası olur.
>
> Bu **PN değildir** ve PN'in başarısız olduğu yerde çalışır: PN açının
> DEĞİŞİMİNİ sıfırlar (hangi açıdaysa onu kilitler), buradaki terim açının
> KENDİSİNİ merkeze geri çeker. Kazanç kapalı çevrim simülasyonda tarandı:
> **0.5 en iyi** (ıska 0.59→0.55 m, en kötü 1.39→1.15 m); 1.0 ve üstü zararlı
> (1.5'te 0/24 vuruş — yüksek kazanç yakınsamayla savaşıp salınım üretiyor).
> CSV: `kadraj_hata_deg` (0 ⇒ hedef tam merkezde), `kadraj_duz_deg`.

> **Görsel fazı erken bırakma düzeltmesi (2026-07-31).** Görsel faz 19:35
> uçuşunda **4 KEZ** başlayıp koptu (her biri 1-1.9 s). Her kopuşta GPS fazı
> drone'u istasyona geri çekiyor — görsel fazın kazandığı irtifa geri alınıyor.
> Dikey ıskanın asıl dinamiği bu. Sebep: `KAYIP_M` **ARDIŞIK** pose'suz kare
> ölçütü; tespit kümelenmiş koptuğu için (karelerin %28'i `tespit_yok`) sık
> sağlanıyordu. Yeni ölçüt kayan pencere: son `KAYIP_PENCERE` (40) karede en az
> `KAYIP_MIN_ISABET` (4) tespit varsa temas sürüyor sayılır.
> **Gerçek loglara uygulandığında 5 erken bırakma → 0.**
> Aynı desen giriş kapısında da uygulandı (`SupCfg.KILIT_PENCERE`): 10 ardışık
> yerine son 15 karenin 10'u — matematiksel olarak asla daha geç tetiklenmez,
> ama eldeki loglarda ölçülebilir fark üretmedi (giriş GPS fazında olur, orası
> loglanmıyor).

> **Menzil makullük kapısı (2026-07-31).** `menzil_gercek_m` fiziksel olarak
> imkânsız zıplıyor: `193559` uçuşunda 33 ms'de 22.4 → 6.6 m (= 479 m/s) örneği
> geldi ve kod bunu **VURULDU** saydı — oysa drone hedefin altından geçip
> uzaklaşıyordu. Doğrulanan 7 gerçek uçuş vuruşunun 1'i bu şekilde sahteydi.
> Sinyal yalnız log değil: kör dalış tetiğini (`_terminal_giris_ok`) ve terminal
> co-altitude kilidini de besliyor. `visual_lead._MenzilKapisi` iki örnek
> arasındaki değişim `MENZIL_HIZ_TAVAN·dt`'yi aşarsa örneği reddeder, son
> geçerli değeri korur; `MENZIL_RESENK_N` ardışık red sonrası yeni seviyeye
> senkronize olur (bayat değere kilitlenmesin). CSV'de `menzil_ham_m` ve
> `menzil_red` sütunlarıyla izlenir — sessiz filtreleme yok.
>
> Tüm gerçek uçuş loglarına uygulandığında: **sahte vuruş elendi, 6 gerçek
> vuruşun hepsi korundu.** Bu kapı SEMPTOMU keser; verinin neden zıpladığı ayrı
> iş (baş şüpheli `gcs_server._frame_off` dikey kalibrasyonu).

> **Gerçek PN denendi ve GERİ ALINDI (2026-07-31).** Dikey ıskayı çözmek için
> klasik oransal seyrüsefer (`γ += N·Δλ`) uygulandı, kapalı çevrim
> simülasyonda ıska **0.66 m → 1.5-2.1 m'ye çıktı**. Sebep: devir anında λ
> zaten doğal olarak azalıyor (drone hedefe yakınsıyor); PN bunu "sıfırlanacak
> LOS hızı" sanıp yakınsamayla savaşıyor ve hedef yukarıdayken γ eksiye
> (dalışa) gidiyor. PN küçük sapmaları düzeltmek içindir, büyük bir başlangıç
> ofsetini kapatmak için değil.
>
> **Dikey ıskanın kökü bu yasa değil.** Ölçüm (`193548`): drone dikey komutu
> sadıkla uyguluyor — komut −0.32 m/s, gerçekleşen −0.39 m/s. Komutun KENDİSİ
> küçük, çünkü GPS fazı hedefin **4.65 m altında** istasyon tutuyor
> (`RANGE_SET·sin(CENTER_ELEV_DEG)`) ve görsel faz devraldığında (6-10 m)
> farkı kapatacak zaman kalmıyor. Sabit METRE ofset, kapanan menzilde sabit
> AÇI değildir: 11 m'de 25°, 6 m'de 51°, 4 m'de kadraj dışı (üst sınır +80.2°).
> Düzeltme GPS fazının istasyon geometrisine ait.

> **Limit dt tavanı (2026-07-31).** `YAW_HIZ_MAX` ve `IVME_TAVAN` birer **hız**
> limitidir; adaptör bunları kare başına paya çevirirken `dt` ile çarpar. `dt`
> ardışık kare zaman damgası farkıdır ve tespit kesildiğinde **şişer** —
> `process()` çağrılmadığı için boşluğun tamamı tek `dt`'ye biner. `160249`
> uçuşu satır 60: 15 kare kör dalıştan sonra `dt=0.825 s` geldi, yaw tavanı
> `90 × 0.825 = 74.2°` oldu ve adımın **tamamı tek MAVLink mesajında** gitti;
> araç bunu rampa değil basamak olarak gördü. Limit hesabı artık
> `min(dt, DT_TAVAN_S)` kullanır. Ham `dt` filtre/PN'de aynen kalır — orada
> `dt` bir türev paydası, harcanacak bir pay değil.
>
> Aynı log yeni kodla tekrar oynatıldığında: max yaw adımı **74.2° → 9.0°**,
> toplam dönme 397° → 318°, `vz` kare-arası sıçraması 2.3 → 0.3 m/s.

> **Azimut tekilliği kapısı (2026-07-31).** `yaw_hata = atan2(u_govde[1],
> u_govde[0])` gövde azimutudur; hedef kadraj tepesine çıkınca nişan vektörü
> dikeye yaklaşır, yatay bileşen (= cos(yükseliş)) sıfıra iner ve `atan2`
> **tanımsızlaşır**. `141017`'de ardışık üç kare: yatay 0.446 → 0.027 → 0.156,
> `yaw_hata` +64.7° → −154.2° → +130.8°. `guidance_core` artık her karede
> `azimut_kalite` (0..1) üretir, `adapter_copter` yaw adımını onunla çarpar.
> Tepedeki hedefte yaw susar — o geometride düzeltmeyi dikey kanal (PN +
> co-altitude) yapar. CSV'de `yatay_bilesen`, `azimut_kalite`, `yaw_adim_deg`
> sütunlarıyla izlenir.

### Terminal ayarları

| Ayar | Değer | Anlamı |
|------|------:|--------|
| `TERMINAL_MENZIL` | 8 m | Altında temas koparsa kör dalış |
| `TERMINAL_SURE` | 0.6 s | Kör dalış süresi — 25 m/s'de ≈ **15 m**, 6 m'lik kör bölgeyi kapatır |
| `VURUS_MENZIL` | 3 m | Altı = VURULDU |

> **3 m neden:** Hedef telemetrisi ~4-5 Hz, drone 25 m/s → menzil örnekleri ~5 m
> aralıklı. Araç açıklıkları ~1.3 m. 3 m merkez-merkez ≈ fiziksel temas.

### Dikey düzeltme ayarları

| Ayar | Değer | Anlamı |
|------|------:|--------|
| `ELEV_EMA` | 0.4 | Dikey aim EMA katsayısı |
| `ELEV_STEP_MAX_DEG` | 10° | Tek-kare slew kırpması |
| `PN_LEAD_SURE` | 0.4 s | Dikey LOS lead anticipasyonu |
| `PN_DIKEY_MAX_DEG` | 15° | Dikey lead tavanı (25'ten indirildi) |
| `PN_RATE_EMA` | 0.3 | Yükseliş oranı EMA |
| `TERMINAL_COALT_DEG` | 10° | Terminalde sabit yukarı yanlılık |
| `TERMINAL_COALT_MENZIL` | 12 m | Altında co-altitude (kilitli) |

---

## `cfg_copy()`

```python
import types
return types.SimpleNamespace(
    **{k: v for k, v in vars(Cfg).items() if not k.startswith("_")})
```

`Cfg`'nin **bağımsız** kopyası. Testlerde ve parametre taramasında kullanılır —
bir testin değiştirdiği ayar diğerini etkilemesin diye.

---

## `kamera_to_govde(u_kamera, tilt_rad)`

**İmza:** `(array(3), float) -> array(3)`
**Dönüş:** gövde FRD birim vektörü `[ileri, sağ, aşağı]`

```python
ham = np.array([u_kamera[2], u_kamera[0], u_kamera[1]], dtype=float)
c, s = math.cos(tilt_rad), math.sin(tilt_rad)
Ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
return Ry @ ham
```

### Satır 1 — eksen yeniden sıralama
| Kamera (OpenCV) | → | Gövde (FRD) |
|---|---|---|
| X = sağ | → | Y (indeks 1) |
| Y = aşağı | → | Z (indeks 2) |
| Z = ileri | → | X (indeks 0) |

Bu bir rotasyon **değil**, indeks permütasyonu. Her iki çerçeve de sağ-el
olduğundan işaret değişmez.

### Satır 2-3 — pitch rotasyonu
`Ry(tilt)` Y ekseni (sağ) etrafında döndürür = pitch. `tilt` **pozitif** =
kamera **yukarı** bakıyor.

### Neden TEK dönüşüm noktası
Docstring der ki: *"gimbal gelirse yalnız burası dinamikleşir."* Bugün tilt sabit
(25°); gimbal eklenirse `tilt_rad` gimbal açısından okunur ve **başka hiçbir yer
değişmez**.

### Doğrulama testi
```
merkez [0, 0, 1] + tilt 25°  →  [0.906, 0, -0.423]
```
Yani kadrajın merkezine bakan bir ışın, gövde çerçevesinde **+25° yukarı**
gösterir. `-0.423 = -sin(25°)` (NED'de negatif Z = yukarı).

> ⚠️ **Tilt atlanırsa:** güdüme sürekli **25° sabit hata** girer — drone hedefin
> hep altını nişanlar ve altından geçer.

---

## `_dcm_govde_dunya(roll, pitch, yaw)`

Gövde FRD → dünya NED yön kosinüs matrisi.

```python
return np.array([
    [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
    [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
    [-sp,    cp*sr,             cp*cr],
])
```

Standart `DCM = Rz(ψ)·Ry(θ)·Rx(φ)` açılımı — ArduPilot'un attitude
konvansiyonuyla uyumlu.

---

## `govde_to_dunya(u_govde, roll, pitch, yaw)`

```python
return _dcm_govde_dunya(roll, pitch, yaw) @ np.asarray(u_govde, dtype=float)
```

---

## `hedef_kadraj_hatasi(hedef_ned, drone_ned, roll, pitch, yaw)`

**Dönüş:** `{menzil, yaw_hata, elev, pitch_hata, u, v, onde}`

Çekirdeğin **ters izdüşümü**: kamera görüntüsü yerine GPS + attitude'dan hedefin
kadraj konumunu hesaplar. **GPS fazının başarı ölçütü budur.**

### Adım adım

```python
los = hedef_ned - drone_ned                    # görüş hattı vektörü
menzil = np.linalg.norm(los)
u_dunya = los / menzil                         # birim vektör

R = _dcm_govde_dunya(roll, pitch, yaw)
u_govde = R.T @ u_dunya                        # dünya → gövde (transpoz = ters)

yaw_hata = math.atan2(u_govde[1], u_govde[0])                     # sağ +
elev = math.atan2(-u_govde[2], math.hypot(u_govde[0], u_govde[1])) # yukarı +
```

`R.T` kullanımı: rotasyon matrisleri ortogonaldir, transpoz = ters. Dünyadan
gövdeye çevirmek için.

### Piksel izdüşümü (yalnız log/UI)

```python
tilt = math.radians(Cfg.KAMERA_TILT_DEG)
c, s = math.cos(tilt), math.sin(tilt)
ham = np.array([c*u_govde[0] - s*u_govde[2],      # Ry(-tilt) @ u_govde
                u_govde[1],
                s*u_govde[0] + c*u_govde[2]])
u_kam = np.array([ham[1], ham[2], ham[0]])         # [sağ, aşağı, ileri]
onde = u_kam[2] > 1e-6
u_px = (geo.CX + geo.FX * u_kam[0] / u_kam[2]) if onde else None
v_px = (geo.CY + geo.FY * u_kam[1] / u_kam[2]) if onde else None
```

`Ry(-tilt)` — `kamera_to_govde`'nin **tersi**. Sonra pinhole projeksiyonu.

### Merkez tanımı

```
Kadraj MERKEZİ  ⇔  yaw_hata = 0  VE  elev = +25°  ⇔  (u, v) = (CX, CY) = (320, 240)
```

`pitch_hata = elev − tilt` → **0 = dikey merkez**.

### Neden bu fonksiyon çekirdekte
Docstring açıklıyor: *"IBVS pose zincirinin AYNI konvansiyonu (NED/FRD, 25° tilt)
— hiç ENU/Gazebo karışmaz."* Yani GPS fazı ile görsel faz **aynı geometri
tanımını** paylaşır; iki farklı "merkez" tanımı olsaydı fazlar arası geçişte
sıçrama olurdu.

---

## `yukselti_duzeltme(eps_rad)`

```python
s = math.sin(eps_rad)
return math.sqrt(1.0 + s * s)
```

**Adım 3 düzeltme katsayısı:** `olcek = olcek_ham / sqrt(1 + sin²ε)`

`ε` = LOS'un yatayla açısı. Hedefin yaklaşık **seviyeli uçtuğu** varsayılır.

**Neden gerekli:** Drone hedefin altından bakarken hedefin gövdesi ve kanatları
görüntüde **kısalır** (perspektif kısalması). Düzeltilmezse ölçek olduğundan
küçük görünür → menzil olduğundan uzak sanılır → lead yanlış hesaplanır.

---

## `class LeadPursuitCore`

**Durum:** `yandanlik_f` (EMA), `d_birim_onceki` (flip koruması),
`t_onceki` (dt hesabı), `flip_sayaci`.

Keypoint indeksleri: `_I_BURUN=0, _I_KUYRUK=1, _I_SOLK=2, _I_SAGK=3`
(`vision.geometry.KEYPOINT_NAMES` sırası).

---

### `_undistort(pts)`

```python
if not self.cfg.UNDISTORT_AKTIF:
    return pts                    # simde işlemsiz
import cv2
K = np.array([[geo.FX, 0, geo.CX], [0, geo.FY, geo.CY], [0, 0, 1]])
dist = np.asarray(self.cfg.DIST_KATSAYILARI or [0,0,0,0,0], float)
und = cv2.undistortPoints(np.asarray(pts).reshape(-1,1,2), K, dist, P=K)
return und.reshape(-1, 2)
```

**Kanca (hook) deseni:** Simülasyonda lens distorsiyonu yok, bu yüzden kapalı.
Gerçek donanıma geçildiğinde `UNDISTORT_AKTIF=True` + katsayılar verilir,
**başka hiçbir yer değişmez**.

---

### `process(pose, stamp, attitude)` — 8 adımlı ana hesap

**Girdiler:**
| Parametre | Tip | Açıklama |
|-----------|-----|----------|
| `pose` | dict | `{cx, cy, conf, kpts: 6×(u,v,conf)}` — tam kare piksel |
| `stamp` | float | Kare `header.stamp` (s) — **dt bundan** |
| `attitude` | tuple/None | `(roll, pitch, yaw)` radyan |

**Dönüş:** 25+ alanlı dict — tüm ara değerler, `u_govde`, hata açıları, `durum`,
`warn` listesi.

#### dt hesabı
```python
dt = (stamp - self.t_onceki) if self.t_onceki is not None else None
self.t_onceki = stamp
```
**Duvar saati ASLA kullanılmaz** — sim saati ile duvar saati farklı hızda akabilir.

#### Adım 1 — ham ölçümler
```python
d = burun - kuyruk                    # 2D gövde ekseni vektörü (px)
a = float(np.hypot(d[0], d[1]))       # gövde projeksiyonu (uzunluk)
b = float(np.hypot(*(solk - sagk)))   # kanat projeksiyonu — SADECE uzunluk
```
Kanat için yön kullanılmaz, sadece uzunluk — kanat ucu keypoint'leri gövde
ekseninden daha gürültülüdür.

#### Adım 2 — ham ölçek
```python
olcek_ham = math.sqrt(a*a + (GOVDE_KANAT_ORANI * b)**2)
```
Gövde ve kanat izdüşümlerinin **ölçek-değişmez** birleşimi. Hedef hangi açıda
olursa olsun (tam karşıdan `a≈0`, tam yandan `b≈0`) ölçek anlamlı kalır.

#### Flip koruması
```python
if a > 1e-9:
    d_birim = d / a
else:
    d_birim = self.d_birim_onceki if self.d_birim_onceki is not None else np.array([1.0, 0.0])

if (self.d_birim_onceki is not None and dt is not None
        and dt < cfg.FLIP_DT_TAVAN_S
        and float(np.dot(d_birim, self.d_birim_onceki)) < -0.5):
    d_birim = self.d_birim_onceki     # yön korunur
    self.flip_sayaci += 1
    warn.append("flip")               # sessizce düzeltme YOK — her seferinde logla
```

**Sorun:** Pose modeli burun ile kuyruğu karıştırabilir. `d` tersine dönerse
lead **ters yöne** kayar — hedefin önüne değil arkasına nişan alınır.

**Tespit:** Ardışık iki karenin yön vektörü skaler çarpımı `< -0.5` ise (yani
>120° dönmüşse) flip var demektir.

**`dt < FLIP_DT_TAVAN_S` koşulu neden:** Düşük kare hızında gerçek aspect
değişimi tek karede olabilir. Zaman aralığı büyükse kontrol atlanır, yanlış
alarm üretilmez.

**Neden sessiz düzeltme yok:** Her flip loglanır. Sessizce düzeltilseydi pose
modelinin bozulduğu gizlenirdi.

#### Adım 6 ön hazırlık — saf takip yönü
```python
u = np.array([(bbox_cx - geo.CX) / geo.FX,
              (bbox_cy - geo.CY) / geo.FY, 1.0])
u = u / np.linalg.norm(u)
```
Ters pinhole: piksel → kamera birim vektörü.

#### Adım 3 — yükselti düzeltmesi
```python
tilt = math.radians(cfg.KAMERA_TILT_DEG)
u_govde_hedef = kamera_to_govde(u, tilt)        # lead'siz yön
if cfg.YUKSELTI_DUZELT:
    if attitude is not None:
        u_dunya_hedef = govde_to_dunya(u_govde_hedef, *attitude)
        eps = math.asin(clamp(-float(u_dunya_hedef[2]), -1, 1))   # NED: -Z yukarı
        duzeltme = yukselti_duzeltme(eps)
    else:
        warn.append("attitude_yok")              # sağlamlık: düzeltmesiz devam
olcek = olcek_ham / duzeltme
```

Attitude yoksa **durmaz** — düzeltmesiz devam eder ve uyarır.

#### Adım 4 — yandanlık, kalite, filtre
```python
yandanlik = (a / olcek) if olcek > 1e-9 else 0.0
kalite = clamp((olcek - cfg.OLCEK_KAPALI_PX) / (cfg.OLCEK_TAM_PX - cfg.OLCEK_KAPALI_PX), 0, 1)

# kanat ucu güveni düşükse b'yi bbox'tan UYDURMA — kaliteyi söndür
if (kpts[_I_SOLK][2] < cfg.KPT_CONF_MIN or kpts[_I_SAGK][2] < cfg.KPT_CONF_MIN):
    kalite = 0.0
    durum = "kanat_dusuk"

kpt_govde_ok = (kpts[_I_BURUN][2] >= cfg.KPT_CONF_MIN
                and kpts[_I_KUYRUK][2] >= cfg.KPT_CONF_MIN)
```

| Kavram | Anlamı |
|--------|--------|
| **yandanlık** | 0 = hedef tam karşıdan (bize doğru geliyor) · 1 = tam yandan (dik geçiyor) |
| **kalite** | Ölçek güvenilirlik ölçüsü — hedef ne kadar büyük görünüyor |

**Kanat güveni düşükse `b` uydurulmaz:** Bbox genişliğinden `b` tahmin etmek
mümkündü ama yapılmıyor — uydurma ölçüm, **güveni yüksek yanlış lead** üretirdi.
Bunun yerine kalite sıfırlanıp lead söndürülür.

#### dt tabanlı EMA
```python
if self.yandanlik_f is None or dt is None:
    self.yandanlik_f = yandanlik
else:
    alpha = 1.0 - math.exp(-dt / cfg.FILTRE_TAU_S)   # dt'den türetilir, SABİT DEĞİL
    self.yandanlik_f = alpha * yandanlik + (1.0 - alpha) * self.yandanlik_f
```

**Kritik:** `alpha` sabit değil, `dt`'den türetilir. Sabit olsaydı kare hızı
değiştiğinde filtrenin **zaman sabiti kayar** — 30 Hz'te doğru ayarlanmış filtre
15 Hz'te iki kat yavaş olurdu.

#### Adım 5 — lead açısı
```python
carpim = cfg.K_LEAD * guven * kalite * self.yandanlik_f
if carpim > 0.95:
    durum = "cozumsuz"       # kesme çözümü olmayabilir
    warn.append("cozumsuz")  # güdüm DURMAZ, sadece işaretlenir
lead = math.atan(carpim)
lead = min(lead, math.radians(cfg.MAX_LEAD_DEG))

if a < cfg.MIN_GOVDE_PX or not kpt_govde_ok:
    lead = 0.0               # deadband: saf takibe düş
    if not kpt_govde_ok:
        durum = "kpt_dusuk"
```

**Çarpımın dört terimi:**
| Terim | Etkisi |
|-------|--------|
| `K_LEAD` | Temel ayar (hız oranı) |
| `guven` | Pose modelinin genel güveni |
| `kalite` | Ölçek güvenilirliği (uzakta 0) |
| `yandanlik_f` | Yandan geçen hedefe çok lead, karşıdan gelene az |

`atan()` neden: Çarpım büyüdükçe lead açısı doyar. `carpim = 1` → 45°.
Fiziksel olarak `tan(lead) = hedef_hızı/bizim_hız` bağıntısının tersidir.

**Deadband:** Gövde projeksiyonu 2 pikselden küçükse yön ölçümü anlamsızdır →
saf takibe düşülür.

#### Adım 6 — yön vektörü uzayında kaydırma
```python
e = np.array([d_birim[0] / geo.FX, d_birim[1] / geo.FY, 0.0])
e = e - float(np.dot(e, u)) * u        # u'ya dik bileşeni al (Gram-Schmidt)
e_n = np.linalg.norm(e)
if e_n > 1e-12:
    e = e / e_n
    u_nisan = math.cos(lead) * u + math.sin(lead) * e
    u_nisan = u_nisan / np.linalg.norm(u_nisan)
else:
    u_nisan = u.copy()   # gövde ekseni bakış yönüyle çakışık — kaydırma tanımsız
```

**Bu adım projenin en özgün kararı.** Master spec'ten:

> *"Kaydırma PİKSEL uzayında DEĞİL, yön vektörü uzayında yapılır (FOV 125°
> geniş: kenarda 1 piksel, merkezdekinin ~çeyreği kadar açı eder)."*

125° FOV'da piksel↔açı ilişkisi **doğrusal değildir**. Piksel uzayında sabit
kaydırma yapılsaydı, lead açısı hedefin kadrajdaki konumuna göre değişirdi —
merkezde doğru, kenarda 4 kat yanlış.

`e` vektörü: gövde ekseninin bakış yönüne **dik** bileşeni. Gram-Schmidt ile
`u`'nun projeksiyonu çıkarılır. Sonra `u` ve `e` düzleminde `lead` kadar
döndürülür.

Dejenere durum: hedef tam bize doğru geliyorsa gövde ekseni bakış yönüyle
çakışır, kaydırma yönü tanımsızdır → saf takip.

#### Adım 7-8 — gövdeye çevir, hata açıları
```python
u_govde = kamera_to_govde(u_nisan, tilt)          # aynı fonksiyon, ikinci çağrı
yaw_hata = math.atan2(u_govde[1], u_govde[0])                          # sağ +
pitch_hata = math.atan2(-u_govde[2], math.hypot(u_govde[0], u_govde[1]))  # yukarı +
```

#### Dönüş sözlüğü — önemli alanlar

| Alan | Anlamı |
|------|--------|
| `u_govde` | **Ana çıktı** — nişan yönü (FRD birim vektör) |
| `yaw_hata`, `pitch_hata` | Türetilmiş hata açıları (rad) |
| `durum` | `ok` / `cozumsuz` / `kanat_dusuk` / `kpt_dusuk` |
| `warn` | Bu karede oluşan uyarılar listesi |
| `kalite` | 0-1, adaptör PN'i buna göre söndürür |
| `lead_deg` | Uygulanan lead açısı |
| `a`, `b`, `olcek`, `yandanlik_f` | Ara değerler (CSV log) |
| `eksen_disi_deg` | `acos(u_nisan[2])` — nişanın boresight'tan sapması |
| `menzil_kestirim_m` | `FX · 0.81 / olcek` — **SADECE LOG**, güdüme girmez |
| `flip_sayaci` | Kümülatif flip sayısı |

---
---

# `adapter_copter.py`

**Yol:** `control/guidance/adapter_copter.py` · **123 satır**
**Rol:** Çekirdeğin **yönünü** multirotor **komutuna** çevirir (Adım 9).

---

## `class CopterAdapter`

**Durum:**
```python
self.v_onceki = (0.0, 0.0, 0.0)   # ivme rampası için
self.elev_f = None                # yumuşatılmış dikey aim yükseliş açısı (rad)
self.elev_onceki = None           # PN türevi için
self.elev_rate_f = 0.0            # EMA'lı yükseliş oranı (rad/s)
```

---

### `_dikey_pn(u_dunya, dt, kalite, terminal)`

**Dönüş:** `(u_dunya_yeni, pn_lead_rad, coalt_rad)`

Aim yönünü dikey düzlemde yukarı döndürür. **Azimut sabit kalır, `|u|` korunur** —
yalnız yükseliş açısı değişir.

```python
elev_ham = math.asin(clamp(-float(u_dunya[2]), -1, 1))   # yukarı +
az = math.atan2(float(u_dunya[1]), float(u_dunya[0]))    # azimut — DOKUNULMAZ
```

#### (1) Dikey aim yumuşatma
```python
if self.elev_f is None:
    self.elev_f = elev_ham
else:
    step = clamp(elev_ham - self.elev_f,
                 -math.radians(cfg.ELEV_STEP_MAX_DEG),    # ±10°
                 math.radians(cfg.ELEV_STEP_MAX_DEG))
    self.elev_f += cfg.ELEV_EMA * step                     # 0.4
elev = self.elev_f
```

**İki aşamalı:** önce tek-kare **slew kırpma** (gürültü sıçramasını sınırla),
sonra **EMA**. Kırpma olmadan tek büyük sıçrama EMA'yı da bozar.

#### (2) Dikey PN
```python
pn_lead = 0.0
if dt is not None and 0.0 < dt <= 0.2 and self.elev_onceki is not None:
    rate = (elev - self.elev_onceki) / dt                  # YUMUŞATILMIŞ türev
    self.elev_rate_f = (cfg.PN_RATE_EMA * rate
                        + (1.0 - cfg.PN_RATE_EMA) * self.elev_rate_f)
    pn_lead = clamp(
        cfg.PN_LEAD_SURE * self.elev_rate_f * clamp(kalite, 0, 1),
        -math.radians(cfg.PN_DIKEY_MAX_DEG),               # ±15°
        math.radians(cfg.PN_DIKEY_MAX_DEG))
self.elev_onceki = elev
```

**Oransal Seyrüsefer (Proportional Navigation) mantığı:** LOS açısı değişiyorsa
çarpışma rotasında değiliz. Lead = `zaman_sabiti × açısal_hız`.

`kalite` çarpanı: pose güvenilmezse PN söner (gürültüden yanlış anticipasyon
üretmesin).

`dt <= 0.2` koşulu: uzun boşluktan sonraki türev anlamsızdır.

#### (3) Terminal co-altitude
```python
coalt = math.radians(cfg.TERMINAL_COALT_DEG) if (terminal and elev > 0.0) else 0.0
```

**`elev > 0` koşulu kritik:** Yukarı yanlılık yalnız hedef **üstteyken**
uygulanır. Hedef aşağıdaysa yukarı nişan almak tamamen yanlış olurdu.

#### Yeni yön vektörü
```python
elev_yeni = elev + pn_lead + coalt
ce = math.cos(elev_yeni)
u_yeni = np.array([ce*math.cos(az), ce*math.sin(az), -math.sin(elev_yeni)])
```
Küresel koordinattan kartezyene — azimut korunmuş, yükseliş güncellenmiş.

---

### `compute(u_govde, yaw_hata, attitude, dt, mevcut_yaw, kalite=1.0, terminal=False)`

**Saf hesap — göndermez, test edilir.**

```python
u_dunya = govde_to_dunya(u_govde, *attitude)
u_dunya = u_dunya / np.linalg.norm(u_dunya)
u_dunya, pn_lead, coalt = self._dikey_pn(u_dunya, dt, kalite, terminal)
v_hedef = cfg.V_KAPANMA * u_dunya                     # 25 m/s × birim yön

if dt is None or dt <= 0:
    v_cmd = tuple(v_hedef)
    v_doygun = False
else:
    v_cmd = limit_acceleration(v_hedef[0], v_hedef[1], v_hedef[2],
                               *self.v_onceki, cfg.IVME_TAVAN, dt)
    v_doygun = (abs(v_cmd[0]-v_hedef[0]) + abs(v_cmd[1]-v_hedef[1])
                + abs(v_cmd[2]-v_hedef[2])) > 1e-9
self.v_onceki = v_cmd

# Yaw: mevcut heading üstüne KP'li adım, YAW_HIZ_MAX ile slew-limitli
adim_ham = cfg.KP_YAW * yaw_hata
tavan = math.radians(cfg.YAW_HIZ_MAX) * (dt if dt else 1.0/30.0)
adim = clamp(adim_ham, -tavan, tavan)
yaw_doygun = abs(adim_ham) > tavan
yaw_cmd = mevcut_yaw + adim
```

**Dönüş:** `{v_cmd, yaw_cmd, u_dunya, v_doygun, yaw_doygun, pn_dikey_deg, coalt_deg}`

`v_doygun` / `yaw_doygun` bayrakları CSV'ye yazılır — komutun kırpılıp
kırpılmadığını uçuş sonrası görebilmek için. Sürekli doygunluk, ayarların
gerçekçi olmadığını gösterir.

---

### `command(conn, ...)`

```python
out = self.compute(u_govde, yaw_hata, attitude, dt, mevcut_yaw,
                   kalite=kalite, terminal=terminal)
send_velocity(conn, out["v_cmd"][0], out["v_cmd"][1], out["v_cmd"][2],
              out["yaw_cmd"])
return out
```

`compute` (saf) ile `command` (yan etkili) ayrımı **test edilebilirlik için**:
testler `compute`'u çağırır, MAVLink bağlantısı gerekmez.

---

## Neden `SET_ATTITUDE_TARGET` kullanılmıyor

Dosya docstring'i:

> *"Multirotorda attitude komutu bu iş için yanlış araç (burun yukarı = tırmanış
> değil geri yavaşlama)."*

Quad'da burun yukarı eğilmek **geriye ivmelenme** demektir, tırmanma değil.
Tırmanma toplam iticiden gelir. Attitude komutuyla güdüm yapmak, dolaylı ve
yanlış bir kontrol yolu olurdu.

## Quad'ın sabit kanada göre avantajı

> *"Nereye uçtuğun (v) ile nereye baktığın (yaw) bağımsız — yaw hedefi kadrajda
> tutarken hız vektörü kesme rotasında kalır."*

Sabit kanatta bu ikisi kilitlidir (uçak baktığı yöne gider). Quad kamerayı
hedefe kilitleyip **yana doğru** kesme rotasında uçabilir.

## İvme tavanının fiziksel gerekçesi

> *"Quad ileri ivmelenmek için burnunu aşağı eğer; kamera gövdeye +25° bağlı
> olduğundan ~5 m/s² üstünde kamera dünyada AŞAĞI bakmaya başlar (gökyüzü arka
> planı kaybolur, yer karmaşası tespit modeline girer)."*

Bu yüzden `visual_lead` CSV'sinde `kamera_dunya_pitch_deg` kolonu var — uçuş
sonrası gökyüzünün ne zaman kaybedildiğini görmek için.

---
---

# `visual_lead.py`

**Yol:** `control/guidance/visual_lead.py` · **319 satır**
**Rol:** Görsel fazın ana döngüsü. **Olay güdümlü, kameraya kilitli.**

---

## `class _ArasState`

Kendi aracımızın MAVLink durumunu **non-blocking** drenajla günceller.

```python
def drenaj(self, conn):
    while True:
        msg = conn.recv_match(
            type=["ATTITUDE", "HEARTBEAT", "LOCAL_POSITION_NED"], blocking=False)
        if msg is None:
            return
        t = msg.get_type()
        if t == "ATTITUDE":
            self.attitude = (msg.roll, msg.pitch, msg.yaw)
        elif t == "HEARTBEAT" and msg.get_srcSystem() != 255:
            self.mode = msg.custom_mode
        elif t == "LOCAL_POSITION_NED":
            self.pos = (msg.x, msg.y, msg.z)
```

**Neden gerekli:** Görsel thread aktifken `gcs_server`'ın iris telemetri worker'ı
**durur** (tek UDP portunu iki dinleyici bind edemez). Bu sınıf aynı `conn`
üzerinden kendi telemetrisini toplar.

`srcSystem != 255` filtresi: kendi GCS heartbeat'imizi mod olarak okumamak için.

**Non-blocking neden:** Döngü kareye kilitli; telemetri beklemek için bloklanırsa
kare kaçar.

---

## `_menzil_hesapla(get_plane_truth, iris_pos)`

```python
p = get_plane_truth()
return math.sqrt((p["x"]-iris_pos[0])**2 + (p["y"]-iris_pos[1])**2
                 + (p["z"]-iris_pos[2])**2)
```

> **NOT (docstring):** *"sim ground-truth; gerçek uçuşta yerini yakınlık/menzil
> sensörü alır."* Bu değer **güdüme girmez** — yalnız log ve vuruş tespiti.

---

## `run_visual_lead(conn, wait_kare, get_plane_truth, stop_event, cfg=Cfg, kayip_kare_esik=None)`

**Dönüş:** `"vuruldu"` | `"kayip"` | `"durduruldu"`

### Durum değişkenleri

| Değişken | İşlev |
|----------|-------|
| `son_seq` | Son işlenen kare numarası |
| `menzil_onceki`, `t_menzil_onceki` | Kapanma hızı hesabı |
| `bayat_sayaci` | Atlanan kare sayısı |
| `kayip_sayaci` | Ardışık pose'suz kare |
| `son_v_cmd` | Son komut — kör dalışta sürdürülür |
| `terminal_latch` | **Kör dalış KİLİDİ** |
| `terminal_min` | Kör dalışta görülen en yakın menzil |
| `coalt_latch` | Co-altitude kilidi |

---

### Ana döngü

```python
while not stop_event.is_set():
    kayit = wait_kare(son_seq, timeout=0.5)    # Condition üzerinde bekle
```

#### Kare gelmediyse
```python
if kayit is None:
    if terminal_latch or _terminal_giris_ok():
        sonuc = _terminal_adim()                # son nişanı sürdür
        if sonuc: return sonuc
        time.sleep(0.02)
        continue
    if kayip_kare_esik is not None and time.time() - son_kayit_wall > 1.0:
        return "kayip"                          # kare akışı kesildi
    continue
```

#### Vuruş kontrolü (ground truth)
```python
d = math.sqrt(...)                              # gerçek menzil
satir["menzil_gercek_m"] = round(d, 3)
kapaniyor = (menzil_onceki is not None and d < menzil_onceki)
if menzil_onceki is not None and stamp > t_menzil_onceki:
    satir["kapanma_hizi_ms"] = round(-(d - menzil_onceki) / (stamp - t_menzil_onceki), 2)
menzil_onceki, t_menzil_onceki = d, stamp
if d < cfg.VURUS_MENZIL:                        # 3 m
    return "vuruldu"
```

#### Pose yoksa
```python
if pose is None:
    if terminal_latch or _terminal_giris_ok():
        satir["durum"] = "kor_dalis"
        ... son_v_cmd'yi logla ...
        sonuc = _terminal_adim()
        if sonuc: return sonuc
        continue                                 # GPS'e DÖNME
    satir["durum"] = "tespit_yok"
    kayip_sayaci += 1
    if kayip_kare_esik is not None and kayip_sayaci >= kayip_kare_esik:
        return "kayip"
    continue
kayip_sayaci = 0                                 # pose var → temas sürüyor
terminal_baslangic = None                        # kör dalış sıfırla
terminal_latch = False
terminal_min = None
```

#### Bayat kare kapısı
```python
gecikme = (time.time() - wall_recv) if wall_recv else 0.0
if gecikme > cfg.GECIKME_TAVAN_S:               # 0.12 s
    bayat_sayaci += 1
    satir["durum"] = "bayat"
    _satir(satir)
    continue                                     # komut GÖNDERME, son komut korunur
```

**Duvar saati burada kullanılır** — karenin gcs'e geliş anı ile şimdi arasındaki
fark. `dt` için sim saati, gecikme için duvar saati: **aynı saat cinsinden
ölçüm** kuralı.

#### GUIDED kontrolü
```python
if aras.mode is not None and aras.mode != mav_common.COPTER_MODE_GUIDED:
    satir["durum"] = "mod_hata"
    continue
```

Kod yorumu açıklıyor:
> *"mod HENÜZ BİLİNMİYORSA (None) komut KESME: HEARTBEAT ~1 Hz, ilk saniye None
> kalıyor; GPS fazından yeni geldik, zaten GUIDED'iz. Yalnız POZİTİF olarak
> GUIDED-dışı görürsek blokla."*

#### Çekirdek + adaptör
```python
res = core.process(pose, stamp, aras.attitude)
for warntip in res["warn"]:
    print(f"[LEAD WARN] {warntip} (kare t={stamp:.3f})")

if aras.attitude is not None:
    mevcut_yaw = aras.attitude[2]
    if menzil_onceki is not None and menzil_onceki < cfg.TERMINAL_COALT_MENZIL:
        coalt_latch = True                       # bir kez inince KİLİTLE
    cmd = adapter.command(conn, res["u_govde"], res["yaw_hata"],
                          aras.attitude, res["dt"], mevcut_yaw,
                          kalite=res["kalite"], terminal=coalt_latch)
    son_v_cmd = (cmd["v_cmd"][0], cmd["v_cmd"][1], cmd["v_cmd"][2], cmd["yaw_cmd"])
```

#### Quad'a özgü izleme
```python
pitch_body = math.degrees(aras.attitude[1])
kam = govde_to_dunya([math.cos(math.radians(cfg.KAMERA_TILT_DEG)), 0.0,
                      -math.sin(math.radians(cfg.KAMERA_TILT_DEG))], *aras.attitude)
satir["kamera_dunya_pitch_deg"] = round(
    math.degrees(math.asin(clamp(-float(kam[2]), -1, 1))), 2)
```

Kameranın boresight vektörünü dünyaya taşıyıp yükseliş açısını yazar. **İvme
tavanı aşılırsa bu değer negatife düşer** — kamera yere bakmaya başladı demektir.

---

### `_terminal_giris_ok()`

```python
return (kayip_kare_esik is not None and son_v_cmd is not None
        and menzil_onceki is not None
        and menzil_onceki < cfg.TERMINAL_MENZIL)
```

Docstring:
> *"Kapanma bayrağı aranmaz — bu kadar yakında temas koparsa hedef önümüzde,
> ileri commit doğru (gürültülü menzil kapanma bayrağını titretiyordu)."*

`kayip_kare_esik is not None` koşulu: kör dalış yalnız **hibrit modda** (supervisor
altında) çalışır. İzole `start_visual` testinde devre dışıdır.

---

### `_terminal_adim()`

**Dönüş:** `"vuruldu"` / `"kayip"` / `None` (sürüyor)

```python
if not terminal_latch:
    terminal_latch = True
    terminal_baslangic = time.time()
    terminal_min = menzil_onceki
    print(f"[LEAD] KÖR DALIŞ (KİLİTLİ) — menzil ~{menzil_onceki:.1f} m, ...")

send_velocity(conn, *son_v_cmd)          # son nişanı SÜRDÜR
aras.drenaj(conn)
m = _menzil_hesapla(get_plane_truth, aras.pos)
if m is not None:
    terminal_min = min(terminal_min, m) if terminal_min is not None else m
    if m < cfg.VURUS_MENZIL:
        return "vuruldu"

if time.time() - terminal_baslangic > cfg.TERMINAL_SURE:
    if terminal_min is not None and terminal_min < cfg.VURUS_MENZIL:
        return "vuruldu"                  # en yakın nokta vuruş menzilindeydi
    return "kayip"
return None
```

**`terminal_min` neden takip ediliyor:** Ground-truth menzil ~4-5 Hz örneklenir,
drone 25 m/s gider. En yakın geçiş anı iki örnek arasına düşebilir. Süre
dolduğunda **görülen en yakın** değere bakılır — anlık değere değil.

---

## Neden sabit Hz'te dönmüyor

Dosya docstring'i:
> *"Sabit Hz'te DÖNMEZ: kamera 30 Hz, kare geldikçe işler (sabit döngü kare
> tekrarı ve bayat veri üretir)."*

Sabit döngü aynı kareyi iki kez işler (`dt=0` → filtre bozulur) veya kare kaçırır.
`wait_new_frame` bir `threading.Condition` üzerinde bekler.

---
---

# `gps_guidance.py`

**Yol:** `control/guidance/gps_guidance.py` · **292 satır**
**Rol:** GPS fazı — 20 Hz sabit döngü. **Amacı vuruş değil, kadraj merkezleme.**

---

## `class Cfg`

| Grup | Ayar | Değer | Anlamı |
|------|------|------:|--------|
| Döngü | `LOOP_HZ` | 20 | Sabit frekans |
| **Kadraj** | `CENTER_ELEV_DEG` | 25° | Merkez için gereken LOS yükselişi = kamera tilt'i |
| | `RANGE_SET` | 11 m | Slant menzil setpoint (pose tatlı noktası) — `AVCI_GPS_RANGE` |
| | `TRACK_MIN_SPD` | 3 m/s | Üstünde hız-yönü gerisi, altında LOS gerisi |
| | `LOOKUP_MIN_ALT` | 8 m | Alçalma tabanı (yere çakılma koruması) |
| **Hız** | `KP_H` / `KD_H` | 0.8 / 0.20 | Yatay PD |
| | `KP_Z` / `VZ_MAX` | 1.0 / 6 m/s | Dikey (eski 3.5 darboğazı açıldı) |
| | `V_MAX` / `MAX_ACCEL` | 20 m/s / 12 m/s² | Tavanlar |
| | `DERIV_EMA` | 0.2 | Türev yumuşatma |
| **Yaw** | `YAW_DEADBAND` | 3° | Altında yaw komutu değişmez |
| | `YAW_RATE_MAX` | 120°/s | Dönüş hızı tavanı |
| **Filtre** | `POS_EMA` / `VEL_EMA` | 0.4 / 0.3 | Hedef telemetri filtresi |
| | `HOLD_S` | 3 s | Bu kadar donuk kalırsa DROPOUT |
| **Devir** | `HANDOFF_RANGE` | 20 m | Altında `durum=KILIT` |

---

## `status` sözlüğü

```python
status = {"durum": "WARMUP", "d_h": None, "menzil": None,
          "kadraj_yaw_deg": None, "kadraj_elev_deg": None, "none_count": 0}
```

**Salt gözlem** — `supervisor.izci` ve `gcs_server` okur, kimse yazmaz (döngü
hariç).

---

## `run_gps_guidance(conn, get_plane, get_iris, stop_event, cfg=Cfg)`

### Kurulum — istasyon geometrisi

```python
center_elev = math.radians(cfg.CENTER_ELEV_DEG)
d_behind = cfg.RANGE_SET * math.cos(center_elev)     # yatay standoff (~9.97 m)
d_below  = cfg.RANGE_SET * math.sin(center_elev)     # dikey alt ofset (~4.65 m)
```

```
                    hedef ✈
                   ╱  │
       11 m slant ╱   │ 4.65 m (d_below)
                 ╱    │
             🚁 ──────┘
              9.97 m (d_behind)
```

Bu noktada durulduğunda hedef **geometrik olarak kadrajın merkezindedir**
(azimut 0, yükseliş +25°). Test **G7** bunu doğrular.

---

### Adım 1 — tazelik + filtre

```python
raw = (plane["x"], plane["y"], plane["z"])
frozen = bool(plane.get("frozen", False))
fresh = (not frozen) and (raw != last_raw)

if fresh:
    last_raw = raw
    none_count = 0
    if est_x is None:
        est_x, est_y, est_z = raw                 # ilk kestirim
    else:
        a = cfg.POS_EMA
        nx = a*raw[0] + (1-a)*est_x               # EMA pozisyon
        ...
        if t_last_fresh is not None:
            fdt = now - t_last_fresh
            if 1e-3 < fdt < 2.0:
                b = cfg.VEL_EMA
                vel_x = b*((nx - est_x)/fdt) + (1-b)*vel_x   # sonlu-fark hız
        est_x, est_y, est_z = nx, ny, nz
    t_last_fresh = now
else:
    none_count += 1
```

**`raw != last_raw` neden:** Hedef telemetrisi ~4-5 Hz gelir ama döngü 20 Hz
döner. Aynı değer tekrar işlenirse sonlu-fark hız **sıfıra düşer**. Yalnız
değişen veri işlenir.

**`frozen` bayrağı:** `gcs_server._apply_gps_noise` GPS karıştırma sırasında bu
bayrağı set eder — jamming simülasyonunun güdüme giriş noktası.

**`1e-3 < fdt < 2.0` koruması:** Çok küçük `fdt` hızı patlatır, çok büyük olan
anlamsızdır.

---

### Adım 2 — WARMUP / DROPOUT

```python
if est_x is None:
    _hover(); status.update(durum="WARMUP"); continue
if none_count * loop_period > cfg.HOLD_S:          # 3 saniye
    _hover()
    vx_prev = vy_prev = vz_prev = 0.0              # ivme rampasını sıfırla
    status.update(durum="DROPOUT"); continue
```

`DROPOUT` durumu supervisor tarafından okunur → **görsel faza kaçış izni**.

---

### Adım 4 — kadraj noktası (istasyon)

```python
tgt_spd_h = math.hypot(vel_x, vel_y)
if tgt_spd_h >= cfg.TRACK_MIN_SPD:                 # 3 m/s
    bx, by = -vel_x/tgt_spd_h, -vel_y/tgt_spd_h    # hız yönünün gerisi (kuyruk)
elif d_h > 1e-6:
    bx, by = -ex/d_h, -ey/d_h                      # LOS gerisi (drone tarafı)
else:
    bx, by = 0.0, 0.0

st_x = est_x + bx * d_behind
st_y = est_y + by * d_behind
st_z = est_z + d_below                              # NED: +z aşağı = altında
if -st_z < cfg.LOOKUP_MIN_ALT:                      # yere çakılma koruması
    st_z = -cfg.LOOKUP_MIN_ALT
```

**İki mod neden:** Hedef yavaşken hız vektörü **gürültüdür** — yön anlamsız
salınır, istasyon zıplar. O durumda drone'un kendi tarafı (LOS gerisi) kararlı
bir referanstır.

---

### Adım 6 — hız komutu

```python
vx = vel_x + cfg.KP_H * ex_cmd + cfg.KD_H * de[0]
vy = vel_y + cfg.KP_H * ey_cmd + cfg.KD_H * de[1]
vmag = math.hypot(vx, vy)
if vmag > cfg.V_MAX:
    s = cfg.V_MAX / vmag
    vx *= s; vy *= s                                # yön korunarak ölçekle
vz = clamp(vel_z + cfg.KP_Z * ez_cmd, -cfg.VZ_MAX, cfg.VZ_MAX)
```

**`vel_x +` terimi = hedef-hızı feedforward.** Bu terim olmadan drone hedefin
**sürekli gerisinde** kalırdı (klasik takip hatası): PD ancak hata büyüyünce
tepki verir, hedef sabit hızla kaçtığı için hata hiç kapanmaz.

Feedforward ile: drone hedefin hızıyla **birlikte gider**, PD yalnız kalan
konum hatasını kapatır → kilitlenince **kararlı hold**.

Yatay hız yön korunarak ölçeklenir; dikey ayrıca kırpılır.

---

### Adım 7 — yaw

```python
bearing = math.atan2(ey, ex)                        # GERÇEK hedefe (istasyona değil)
if cmd_yaw is None:
    cmd_yaw = bearing
yaw_err = normalize_angle(bearing - cmd_yaw)
if abs(yaw_err) > cfg.YAW_DEADBAND:                 # 3°
    step = clamp(yaw_err, -cfg.YAW_RATE_MAX*dt, cfg.YAW_RATE_MAX*dt)
    cmd_yaw = normalize_angle(cmd_yaw + step)
```

**Burun gerçek hedefe döner** — istasyon *nerede duracağımızı* söyler, *nereye
bakacağımızı* değil. Kamera hedefi görmelidir.

Deadband küçük salınımları keser; rate limit ani dönüşü engeller.

---

### Adım 9 — kadraj hatası (başarı ölçütü)

```python
kad = hedef_kadraj_hatasi((est_x, est_y, est_z), (ix, iy, iz), iroll, ipitch, iyaw)
```

**Bu fazın tüm iddiasının ölçümü.** Her kare CSV'ye yazılır:
`kadraj_yaw_deg`, `kadraj_elev_deg`, `kadraj_pitch_hata_deg`, `u_px`, `v_px`.

Mükemmel merkezleme: `yaw=0`, `elev=25°`, `(u,v)=(320,240)`.

> **Kademe notu (docstring):** *"KADEME 1 (bu sürüm): GEOMETRİK kadraj-noktası
> takibi... KADEME 2'de: gerçek attitude'la kadraj hatasını doğrudan kapatma
> eklenecek."* Yani şu an bu hata **ölçülüyor ama geri beslemeye girmiyor**.

---

## `_sleep(t_start, period)`

```python
elapsed = time.monotonic() - t_start
if elapsed < period:
    time.sleep(period - elapsed)
```

**Sabit frekans koruması:** İşlem süresi düşülerek uyunur. Düz `sleep(0.05)`
olsaydı gerçek frekans işlem süresi kadar düşerdi.

`time.monotonic()` kullanımı: sistem saati değişse bile etkilenmez.

---
---

# `supervisor.py`

**Yol:** `control/guidance/supervisor.py` · **114 satır**
**Rol:** GPS ↔ görsel faz geçiş denetleyicisi. **Tek görev döngüsü.**

---

## `class SupCfg`

| Ayar | Değer | Anlamı |
|------|------:|--------|
| `KILIT_N` | 10 | Ardışık güvenli pose karesi → geç (~0.33 s @30 Hz) |
| `KAYIP_M` | 20 | Ardışık pose'suz kare → GPS'e dön (~0.66 s) |
| `KILIT_CONF_MIN` | 0.5 | Pose güven eşiği |
| `GATE_KILIT` | True | Menzil kapısı aktif |
| `GATE_MENZIL` | 20 m | Kapı menzili — `AVCI_HYBRID_GATE_MENZIL` |

### `GATE_MENZIL = 20` gerekçesi (koddaki yorumdan)

> *"GPS handoff bayrağı 40 m'de açılıyor ama orada kutu ~7 px, pose güvenilmez
> (uzakta devralınca hedef hemen kaçtı — 2026-07-24 log). 20 m'de kutu ~7 px
> hâlâ küçük; pose asıl 10-12 m'de sağlam. GPS istasyonu 10 m; kapı 20 → GPS
> yaklaşırken pose kilidini bu banda çeker."*

Gerçek bir uçuş logundan çıkarılmış ayar.

---

## `_kopru(parent_event, child_event)`

```python
def izle():
    while not parent_event.is_set() and not child_event.is_set():
        parent_event.wait(0.5)
    if parent_event.is_set():
        child_event.set()
threading.Thread(target=izle, daemon=True).start()
```

**Neden gerekli:** GPS fazı kendi `faz_stop` event'iyle çalışır (izci onu
kırabilsin diye). Ana `stop_chase` geldiğinde bu alt-event'in de set olması
gerekir, yoksa faz döngüsü durmaz ve `stop_chase` etkisiz kalır.

Köprü **çift yönlü değil** — child set olunca parent etkilenmez (izci fazı
kırdığında görev bitmemeli).

---

## `run_hybrid(conn, get_plane, get_iris, wait_kare, get_plane_truth, stop_event, sup_cfg=SupCfg, lead_cfg=LeadCfg)`

### GPS fazı + izci

```python
while not stop_event.is_set():
    status["faz"] = "GPS"
    faz_stop = threading.Event()
    _kopru(stop_event, faz_stop)
    tetik = {"gorsel": False}

    def izci():
        sayac, son_seq = 0, 0
        while not faz_stop.is_set():
            kayit = wait_kare(son_seq, timeout=0.5)
            if kayit is None:
                continue
            son_seq = kayit["seq"]
            pose = kayit["pose"]
            if pose is not None and pose.get("conf", 0.0) >= sup_cfg.KILIT_CONF_MIN:
                sayac += 1
            else:
                sayac = 0                        # ARDIŞIK olmalı — sıfırla
            status["kilit_sayac"] = sayac
            if sayac >= sup_cfg.KILIT_N:
                d_h = _ga.status.get("d_h")
                yakin = (d_h is not None and d_h < sup_cfg.GATE_MENZIL)
                dropout = _ga.status.get("durum") == "DROPOUT"   # jamming fallback
                kapi = (not sup_cfg.GATE_KILIT) or yakin or dropout
                if kapi:
                    tetik["gorsel"] = True
                    faz_stop.set()               # gps_guidance döngüsünü kır
                    return

    threading.Thread(target=izci, daemon=True).start()
    run_gps_guidance(conn, get_plane, get_iris, faz_stop)     # BLOKLAR
```

**Mimari:** GPS döngüsü ana thread'i bloklar; izci **paralel** çalışıp pose
akışını izler. Koşul oluşunca `faz_stop.set()` ile GPS döngüsünü kırar.

**`sayac = 0` sıfırlama:** Kareler **ardışık** olmalı. Aralıklı tek tük tespit
kilit sayılmaz.

**Kapı üç yoldan açılır:**
1. `not GATE_KILIT` — kapı devre dışıysa
2. `yakin` — `d_h < 20 m`
3. `dropout` — GPS düştüyse (**jamming yedeği**, menzil bilinemez)

### Görsel faz

```python
if stop_event.is_set() or not tetik["gorsel"]:
    break

status["faz"] = "VISUAL"
status["gecis_sayisi"] += 1
sebep = run_visual_lead(conn, wait_kare, get_plane_truth, stop_event,
                        cfg=lead_cfg, kayip_kare_esik=sup_cfg.KAYIP_M)
status["son_sebep"] = sebep

if sebep == "vuruldu":
    status["faz"] = "VURULDU"
    return                          # GÖREV TAMAMLANDI
if sebep == "kayip":
    continue                        # GPS fazına dön
break                               # durduruldu
```

**Üç çıkış yolu:**
| Sebep | Sonuç |
|-------|-------|
| `"vuruldu"` | Görev biter, `faz=VURULDU` |
| `"kayip"` | `continue` → GPS fazı yeniden başlar (döngü sürer) |
| `"durduruldu"` | `break` → temiz kapanış |

Görsel faza `stop_event` (ana event) verilir, `faz_stop` değil — görsel faz ana
durdurmaya doğrudan duyarlıdır.

---

## `status` sözlüğü

```python
status = {"faz": "GPS", "gecis_sayisi": 0, "kilit_sayac": 0, "son_sebep": None}
```

`gcs_server` bunu `/api/chase_status` ile arayüze yayınlar — kullanıcı hangi
fazda olduğunu ve kaç kez geçiş yapıldığını canlı görür.
