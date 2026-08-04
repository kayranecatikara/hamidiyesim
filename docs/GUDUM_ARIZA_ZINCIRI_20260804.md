# Güdüm arıza zinciri — 2026-08-04 (ikinci tur)

Bu belge, 2026-08-01 uçuşlarının loglarından çıkarılan **dört yeni arızayı** ve
her birinin ölçüsünü kaydeder. `docs/GUDUM_ARIZA_ZINCIRI_20260801.md`'nin
devamıdır (o belge ilk sekiz arızayı anlatır).

Tüm ölçümler mevcut loglardan yapıldı — bu tur **hiç uçuş yapılmadı**.

---

## Başlangıç durumu: 19 dakika, sıfır devir

`logs/gps_guidance_20260801_173612.csv` — 22784 kare, **1143 saniye** kesintisiz
chase. Sonuç:

| Ölçüm | Değer |
|---|---|
| menzil (medyan) | **87.4 m** |
| menzil, dakika dakika | 89 → 82 ve **19 dakika boyunca orada** |
| menzil < 20 m olan kare | 9 / 22784 (%0.04) |
| görsel faza devir | **0** |
| hedef hızı | 18.0 m/s |
| avcı hızı (kümülatif yol/süre) | **8.3 m/s** |
| avcı hız komutu | 20.0 m/s (karelerin %99'u tavanda) |
| avcı toplam eğim | medyan **14.6°**, std **1.18°** |
| kadraj yaw hatası | medyan 2.8°, p90 6.7° |

Kadraj mükemmel, dikey kanal mükemmel (|iris_z − st_z| medyanı 1 cm), yasa
doğru yeri gösteriyor (%87'si 30° içinde). **Tek sorun menzilin kapanmaması.**

---

## Arıza 9 — ArduCopter parametrelerinin HİÇBİRİ uygulanmıyordu

**En büyüğü. `avci_copter.parm`'daki her hareket parametresi ölü addı.**

ArduCopter, hareket parametrelerini hem yeniden adlandırdı hem **birim
değiştirdi**. ArduPilot bilinmeyen adı sessizce yok sayar:

| dosyadaki (ölü) | gerçek ad | birim | istenen | **gerçekte uygulanan** |
|---|---|---|---|---|
| `ANGLE_MAX 7000` | `ATC_ANGLE_MAX` | derece | 70° | **30°** |
| `WPNAV_SPEED 3000` | `WP_SPD` | m/s | 30 | **10** |
| `WPNAV_ACCEL 2000` | `WP_ACC` | m/s² | 20 | **2.5** |
| `WPNAV_SPEED_UP 3000` | `WP_SPD_UP` | m/s | 30 | **2.5** |
| `WPNAV_SPEED_DN 3000` | `WP_SPD_DN` | m/s | 30 | **1.5** |
| `LOIT_SPEED 3000` | `LOIT_SPEED_MS` | m/s | 30 | **12.5** |
| `PSC_VELXY_P 2.0` | `PSC_NE_VEL_P` | — | 2.0 | 2.0 (tesadüfen aynı) |

**Kanıt zinciri:**

1. `WP_ACC` varsayılanı 2.5 m/s². Yatay ivme 2.5 m/s² ise denge yatışı
   `atan(2.5/9.81) = 14.3°`.
2. Ölçülen yatış: **medyan 14.6°, std yalnız 1.18°** (22784 kare). Bu kadar dar
   bir dağılım bir kontrol çıktısı değil, bir **tavandır**.
3. GUIDED hız modunun bu iki parametreyi kullandığı kaynakta doğrulandı:
   `ArduCopter/mode_guided.cpp:255` →
   `NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss())`
4. Doğrulama yolu (CLAUDE.md §8'in tarif ettiği kesin yöntem): SITL'in kendi
   yazdığı `~/Masaüstü/ardupilot/mav_5_1.parm` dökümünde `ANGLE_MAX` **yok**,
   `ATC_ANGLE_MAX 30.000000` **var**.

**Düzeltme:** `sim/ardupilot_params/avci_copter.parm` doğru adlarla yeniden
yazıldı — `ATC_ANGLE_MAX 55`, `WP_ACC 12`, `WP_SPD 25`, `WP_SPD_UP/DN 6`,
`WP_ACC_Z 5`, `PSC_JERK_NE 20`.

`ATC_ANGLE_MAX` neden 55, 70 değil: `MOT_THST_HOVER = 0.39` (dökümden). Bir
multirotorun irtifasını koruyabildiği en yüksek yatış `acos(0.39) = 67°`. 70°
o sınırın üstünde — araç yatışı koruyamaz, irtifa sızdırır. 55° 12° pay bırakır
ve beklenen terminal hız `8.33·√(tan55/tan14.6) = 19.5 m/s` eder.

**Nüksü önleme:** `tools/analiz/parm_dogrula.py` — her `.parm` adını SITL
dökümüyle karşılaştırır, ölü adları ve uygulanmamış değerleri raporlar, olası
yeni adı önerir. Bu tuzağa iki kez düşüldü (2026-08-01 ArduPlane hız adları,
2026-08-04 ArduCopter hareket adları); üçüncüsü olmasın.

```bash
python3 -m tools.analiz.parm_dogrula      # salt okuma
```

---

## Arıza 10 — Dönüş tuzağı: saf takip kapalı desende yakınsamaz

GPS fazı istasyon noktasını hedefin **anlık** konumunun kuyruğuna kuruyordu —
saf takip (pure pursuit). Hedef daire/kare uçtuğunda istasyon da yörüngede
döner; avcı sürekli dönen bir noktayı kovalar ve **desenin ortasına
spirallenir**.

Dönen bir hız vektörünü sürdürmenin bedeli yanal ivmedir:

```
v = a_yanal / omega_komut
```

| büyüklük | ölçüm (173612) |
|---|---|
| eğim medyanı | 14.6° → `a_yanal = g·tan(14.6°)` = **2.56 m/s²** |
| komut yönü dönme hızı, medyan | 15.0 °/s = **0.262 rad/s** |
| **öngörülen hız tavanı** | 2.56 / 0.262 = **9.8 m/s** |
| **gerçekleşen hız, medyan** | **9.4 m/s** |

Birebir tutuyor. Ayrıca ayırt edici: aynı hesap, menzili kapatabilen diğer
uçuşlarda tavanı 24-32 m/s buluyor ve "dönüş tuzağı baskın değil" diyor.

**Düzeltme:** `gps_guidance.py` KADEME 1b — istasyon artık **öngörülü**.
Hedefin dönüş hızı (`omega`) hız vektörünün dönme oranından kestirilir, `t_go`
sonraki konumu sabit-dönüş (coordinated turn) modeliyle tahmin edilir ve
istasyon **oraya** kurulur. Kapalı form:

```
psi_t = psi + omega·t            R = V/omega
x(t)  = x + R·( sin(psi_t) − sin(psi) )
y(t)  = y − R·( cos(psi_t) − cos(psi) )
```

`t_go = mesafe/V_MAX`, iki tavanla kırpılır: `T_GO_MAX = 6 s` ve
`|omega·t_go| ≤ 120°` (sabit yatış varsayımı yarım turdan uzun geçerli kalmaz).

**Kilitte özdeşlik garantisi:** istasyondayken `t_go → 0`, tahmin anlık konuma
eşitlenir, davranış eski saf takiple **aynı** olur (test G14). Yani kesme, hold
kararlılığını bozmaz.

`AVCI_GPS_KESME=off` ile kapatılabilir (A/B için).

---

## Arıza 11 — Hedefin gaz slider'ı yakalanamaz bir banda eşleniyordu

FBWB'de gaz slider'ı doğrudan **hız komutudur**, ama hangi formülle olduğu
`FLIGHT_OPTIONS`'a bağlı (`ArduPlane/navigation.cpp:161-189`). Dökümde
`FLIGHT_OPTIONS 0`, yani varsayılan dal işliyor:

```
hedef_hız = AIRSPEED_MIN + (AIRSPEED_MAX − AIRSPEED_MIN) · slider_oranı
```

**`AIRSPEED_CRUISE` bu modda hiç okunmuyor.** Geçmişte onu 14→12→10 çekmenin
neden hiçbir şeyi değiştirmediği böylece açıklandı.

Doğrulama: MIN 12 / MAX 22, slider 500/1000 → talep `(22−12)·50 + 1200 = 1700`
cm/s = **17.0 m/s**; ölçülen yer hızı **18.0 m/s**. Formül birebir tutuyor.

Yani slider'ın **hiçbir konumu** yakalanabilir bir hız vermiyordu.

**Düzeltme:** `AIRSPEED_MAX 22 → 16`. Yeni slider anlamı:

| slider | talep | ≈ gerçek |
|---|---|---|
| %0 | 12 m/s | ~13 |
| %50 | 14 m/s | ~15 |
| %100 | 16 m/s | ~17 |

`AIRSPEED_MIN` 12'de kalıyor: TECS talebi MIN'in altına indiremez, ve daha
düşüğü yatışta stall riskini açar (`AIRSPEED_STALL 8`, senaryolar 29-48° yatış;
45°'de stall hızı 8 → 9.5 m/s'ye çıkar).

---

## Arıza 12 — Devir yandan yapılıyordu, menzil kapısı bunu görmüyordu

Tek gerçek devir: `logs/visual_lead_20260801_173610.csv`. Menzil kapısı
sağlandı (8-10 m) ama **geometri felaketti**:

| kare | bbox x | menzil kestirimi | yandanlık |
|---|---|---|---|
| t=39.63 | 223…242 | 8.26 m | 1.027 |
| t=39.93 | 0…36 | 3.12 m | 0.889 |
| t=39.96 | — | — | **tespit yok (20 kare)** |

**0.30 saniyede** hedef kadrajın ortasından sol kenarına yürüdü ve çıktı.
`yandanlik_f` 0.89-1.03 — yani **tam yandan geçiş**, pose modelinin ve lead
yasasının en kötü çalıştığı geometri. İlk karede nişan hatası zaten −51°
(yarım-HFOV 62.5°).

Sebep geometrik. Bakış hattının dönme hızı:

```
omega_LOS ≈ v · (K_LEAD·yandanlik + sin(lead)) / menzil
          = 25 · (0.5·0.95 + sin 23°) / 8  =  2.71 rad/s  =  155 °/s
```

Aracın izleyebildiği: **~90 °/s** (ölçüm: 173612'de gerçek yaw hızı p90 84,
p99 136 °/s, ve o bantta yaw takip hatası medyanı 2.8°). Fark kadar hedef
kadrajda kayar → 0.3 s'de kenardan çıkar.

**İki düzeltme:**

### (a) LOS kapısı — kapanma hızını geometriyle ölçekle

`guidance_core.Cfg.LOS_KAPISI` + `adapter_copter.kapanma_hizi()`:

```
v ≤ (LOS_YAW_IZLENEBILIR · LOS_PAY) · menzil / (K_LEAD·yandanlik + sin(lead))
```

Payda **geometriye** bağlı olduğu için kapı kuyruk takibinde (yandanlık→0,
lead→0) **kendiliğinden açılır** — tam gaz kapanırız — ve yalnız yandan
geçişte bağlar. (`stash@{0}`'daki ilk sürüm paydayı 1 varsayıyordu, yani
kuyrukta da gereksiz frenliyordu.)

Ölçülen geometride: 155 °/s → **74 °/s** (test T36). Taban
`V_KAPANMA_MIN = 12 m/s`, hedefin en yavaş halini (12 m/s) garanti eder.

### (b) Devir kapısına geometri koşulu

`supervisor.SupCfg` — menzile ek olarak:

* `GATE_KADRAJ_YAW 25°` — hedef kamera merkezine yakın olsun. GPS fazı bunu
  zaten yapabiliyor (|kadraj_yaw| medyanı 2.8°, p90 6.7°), yani eşik fazın
  normal çalışmasını engellemez.
* `GATE_KUYRUK_ACI 60°` — hedefin arkasında olalım (yandanlık ≤ 0.87).
  Ölçülen 0.89-1.03'ü keser.

GPS **DROPOUT** (jamming) tüm geometri koşullarını atlar: telemetri yoksa
bunlar ölçülemez, görsel temasın kendisi tek kapıdır.

Kapı açılmadığında **hangi koşulun** bloklandığı artık her 10 saniyede bir
loglanıyor (`[SUPERVISOR] ... | kapı: kuyruk açısı 88° > 60° (yandan)`) ve
`/api/chase_status` içinde `kapi_engel` alanında yayınlanıyor. 2026-08-01'de
kilit 4'te kaldı ve sebebi hiçbir logdan çıkarılamamıştı.

---

## Yan düzeltme — yandanlık fiziksel sınırı

`yandanlik = a/olcek` fiziksel olarak `|sin(aspect)| ∈ [0,1]`. Ama `olcek`
yükselti düzeltmesine bölündüğü için düzeltme aşırı telafi ettiğinde oran 1'i
aşıyor (ölçüldü: 1.21 ve 1.0269). Sınırsız bırakılırsa lead şişer, sahte
`cozumsuz` bayrağı üretir ve **LOS kapısının paydasını fizik dışı büyütür**.
`min(a/olcek, 1.0)` geri getirildi (`stash@{0}`'dan, test T41).

---

## Yan düzeltme — kalıcı burun/kuyruk takası

Mevcut flip koruması yalnız 0.2 s içindeki **ani** takası yakalar; model bir
süre tutarlı biçimde ters etiketlerse hiç devreye girmez ve **lead işareti ters
kalır** (hedefin önüne değil arkasına nişan alırız).

Bağımsız kanıt: uçak burnu ileri uçar, dolayısıyla hedefin kadrajdaki kayma
yönü gövde ekseniyle aynı işarette olmalıdır. `bbox` merkezinin kareler arası
akışı ile ham eksen vektörünün iç çarpımı sürekli negatifse etiketler takas
olmuştur. Karar tek kareye bırakılmaz — işaret EMA'lanır (`AKIS_EMA 0.3`,
≈3 karede karar).

Kuyruk takibinde hedef kadrajda neredeyse durgundur; `AKIS_MIN_PX_S = 60`
tabanı o durumda **oy verdirmez**, yani kapı kendiliğinden susar (test T40).

Ölçülen takas oranı 2/12 (%17) ve takas karelerinde `kpt_hata_px_ort` 27.0/27.7
— diğer karelerde 0.9-6.0. Yani takas, modelin genel olarak bozulduğu karelerde
oluyor; akış kontrolü tam da o karelerde ham ekseni reddeder.

---

## Yeni ölçüm kolonları

Bir sonraki uçuşun cevaplaması gereken soruları ayırmak için:

`gps_guidance_*.csv`:
`iris_vx`, `iris_vy`, `iris_vz`, `iris_hiz` (**ArduPilot'un kendi/EKF hız
kestirimi**), `iris_egim_deg`, `v_cmd_mag`, `tgt_hiz`, `tgt_omega_deg`,
`t_go_s`, `kuyruk_aci_deg`

`iris_hiz` kritik: konum türeviyle uyuşmuyorsa sorun EKF'te, uyuşuyorsa araç
komutu gerçekten uygulamıyor demektir. `analiz_gps` bunu otomatik ayırıyor (§8).

`visual_lead_*.csv`:
`v_kapanma_izin` (LOS kapısının o karede izin verdiği hız — `V_KAPANMA`'dan
küçükse kapı bağlamış), `takas_sayaci`, `akis_skor`

---

## Sonraki uçuşta bakılacaklar

```bash
python3 -m tools.analiz.parm_dogrula   # ÖNCE: parametreler gerçekten uygulandı mı
python3 -m tools.analiz.analiz_gps     # dönüş tuzağı + hız ölçümü bölümleri
python3 -m tools.analiz.analiz_devir
```

Beklenen değişimler ve **çürütme ölçütleri**:

| beklenti | doğrulama | çürüterse ne demek |
|---|---|---|
| avcı 18-19 m/s yapabiliyor | `analiz_gps` GERÇEKLEŞEN/KOMUT ≥ 0.9 | `parm_dogrula` çalıştır: parametreler hâlâ uygulanmıyor olabilir |
| eğim 14.6°'de takılı değil | `iris_egim_deg` p90 > 30° | `ATC_ANGLE_MAX` uygulanmamış |
| menzil 82 m'de kilitlenmiyor | menzil < 20 m olan kare > 0 | kesme yetmedi; `T_GO_MAX`'ı büyüt ya da `AVCI_GPS_KESME=off` ile A/B yap |
| devir kuyruktan oluyor | `kuyruk_aci_deg` medyanı < 60° | GPS fazı kuyruğa oturamıyor (Adım 6b hâlâ açık) |
| görsel faz 1 s'den uzun | `visual_lead_*.csv` satır sayısı > 30 | `v_kapanma_izin` kolonuna bak: kapı bağladı mı |

Kapı hiç açılmazsa `[SUPERVISOR] ... | kapı: ...` satırı **hangi koşulun**
bloklandığını söyler — eşikler `AVCI_HYBRID_GATE_KADRAJ` /
`AVCI_HYBRID_GATE_KUYRUK` ile gevşetilebilir.

---

## Testler

`python3 -m tests.test_visual_lead` → **42/42**
(yeni: T34-T37 LOS kapısı, T38-T40 hareket tutarlılığı, T41 yandanlık sınırı)

`python3 -m tests.test_gps_guidance` → **18/18**
(yeni: G10-G15 kesme/eğrisel kestirim, G16-G18 devir kapısı)

G15, kapalı formu kendisiyle değil **bağımsız sayısal integrasyonla**
karşılaştırır — totoloji değil.
