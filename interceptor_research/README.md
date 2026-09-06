# Interceptor Research — Ağ Atan Taretli İnterceptor Drone

BismillahirRahmanirRahim.

Hedefi imha etmek yerine **2 eksenli taretten ağ fırlatıp yakalayan** bir
interceptor drone simülasyonu. Fortem DroneHunter F700 / Delft Dynamics
DroneCatcher sınıfı bir sistemin Gazebo Harmonic karşılığı.

**Platform: `bullet_net_interceptor`** — dikey mermi/roket gövde, taret gövdenin
**tepesinde** (z = +0.265; ham gövdenin burun konisi kaldırılıp yerine oturtuldu).
Namlu **+X**'e bakar, ağ **ileri** atılır. Taret limitleri: pan ±100°,
tilt −60°..+30°. Ağ yüklü kütle 2.59 kg, ölçülen menzil 27.8 m (20 m/s, −8°).

> **Bu proje `avci_sim`'den ayrıdır.** `avci_sim` Teknofest Avcı İHA yarışması için
> **kamikaze** müdahale yapıyor. Burada amaç **yakalama**. Aynı yığın kullanılıyor
> (ArduPilot + Gazebo Harmonic) ki olgunlaşan ağ/taret modülü ileride oraya taşınabilsin.

**Yığın:** ArduPilot · Gazebo Harmonic (gz-sim 8.14) · gz-transport 13 · ROS 2 Humble · Python 3.10

📷 **Simülasyon görüntüleri: [`docs/GORUNTULER.md`](docs/GORUNTULER.md)**
🚀 **Çalıştırma: [`docs/BULLET_CALISTIRMA.md`](docs/BULLET_CALISTIRMA.md)**

> **Not — depo sadeleştirildi.** Aday gövde araştırması (8 `cand_*` gövdesi),
> iris tabanlı `avci_net_interceptor` tasarımı, vitrin (`showcase`) ve skycat
> modelleri ile bunlara ait üretim/kıyas script'leri depodan çıkarıldı. Geriye
> yalnızca mermi gövdeli platform ve onu sürmek için gereken araçlar kaldı.
> Bu yüzden model **yeniden üretilemez** (ham gövde `cand_bullet` ve üretici
> `21_build_bullet_interceptor.py` silindi); hazır SDF doğrudan kullanılır.

---

## Durum özeti

| Aşama | Durum | Kanıt |
|---|---|---|
| 3 · 2 eksenli taret | ✅ komut −8.00° → ölçülen −8.03° | `scripts/turret_state.py` |
| 4 · Ağ fırlatma | ✅ 20 m/s → 27.8 m menzil | [`docs/bench_raw/menzil_taramasi.csv`](docs/bench_raw/menzil_taramasi.csv) |
| 5 · Yakalama (hedefin ağa kilitlenmesi) | ✅ tuttu | `scripts/42_capture_test.sh` |
| 6 · Örgü ağ (5×5 düğüm) | ⬜ yapılmadı | — |
| 7 · ArduPilot SITL ile uçan hedef angajmanı | ⬜ yapılmadı | — |

**Ticari referanslarla karşılaştırma:**

| Büyüklük | Hedef (ticari) | `bullet_net_interceptor` |
|---|---|---|
| Ağ menzili | 20 m (DroneCatcher) | **27.8 m** @ 20 m/s, −8° |
| Interceptor kütlesi | ≤ 6 kg (DroneCatcher) | **2.29 kg** kuru (1.94 gövde + 0.35 taret), ağ yüklü **2.59 kg** |
| Yakalama oranı | %85 ilk atış (Fortem F700) | tuttu (kontrollü senaryo) |

Ağ ayrı model (`net_cone`, 0.30 kg).

---

## Hızlı başlangıç

```bash
cd interceptor_research

# 1) Ağ atıcı eklentilerini derle (bir kez)
cd plugins && mkdir -p build && cd build && cmake .. && make -j4 && cd ../..

# 2) Ortam
source scripts/env.sh

# 3) Doğrulama
./scripts/41_range_test.sh          # menzil taraması (balistik)
./scripts/42_capture_test.sh 5 20   # yakalama başarı oranı
```

Görsel olarak izlemek için:
```bash
source scripts/env.sh
gz sim -r worlds/bullet_net_test.sdf     # ayrı terminalde GUI
python3 scripts/turret_aim.py 25 -15     # tareti çevir
python3 scripts/fire_net.py --hiz 20     # ağı at
```

`turret_aim.py`, `fire_net.py`, `turret_state.py`, `net_track.py` ve
`52_action_shots.py` varsayılan olarak `bullet_net_interceptor` /
`bullet_net_test` üzerinde çalışır.

> **GUI dünyayı duraklatılmış açar.** Duraklatılmışken atış komutu birikir ve
> devam edince ağ ters yöne fırlar. Kolaylık için `./GOSTER.sh` bunu (ve kamera
> konumlandırmayı) otomatik yapar.

---

## Mimari

### Modeller

```
models/
├── interceptors/bullet_net_interceptor/  ← dikey mermi gövde + TEPEDE taret
├── net_launchers/net_cone/               ← 1. aşama ağ: rijit koni "yakalama hacmi"
└── targets/target_box/                   ← 0.7 kg dinamik test hedefi
```

### Taret zinciri

```
base_link
  └─ turret_mount_joint (fixed)      → turret_base_link    0.10 kg
       └─ turret_yaw_joint (Z, ±100°) → turret_yaw_link     0.08 kg
            └─ turret_pitch_joint (Y, −60°..+30°) → turret_pitch_link  0.10 kg
                 └─ muzzle_joint (fixed) → muzzle_link      0.07 kg
                                                   toplam:  0.35 kg
```

Her iki eksen `gz-sim-joint-position-controller-system` ile sürülüyor:
```
/model/bullet_net_interceptor/joint/turret_yaw_joint/0/cmd_pos    (gz.msgs.Double, radyan)
/model/bullet_net_interceptor/joint/turret_pitch_joint/0/cmd_pos
```

### Kendi yazdığımız eklentiler

`plugins/` altında iki C++ sistem eklentisi — ikisi de Gazebo Harmonic'te
hazır karşılığı olmadığı için yazıldı:

| Eklenti | Ne yapar | Neden gerekti |
|---|---|---|
| **`NetLauncherPlugin`** | Ağı namluya kilitler; tek komutla **aynı fizik adımında** ayırır ve namlu yönünde çıkış hızı verir | Hazır `detachable-joint` + `apply-link-wrench` ikilisi iki ayrı topic'e dışarıdan iki ayrı mesaj gerektiriyordu; aradaki gecikme kontrol edilemediği için aynı parametrelerle menzil 2 m ↔ 108 m arası oynuyordu |
| **`NetCapturePlugin`** | Ağın temas ettiği hedefi çalışma anında ağa kilitler (runtime `DetachableJoint`) | `gz-sim-detachable-joint-system` yalnızca **koparabiliyor**, yeni bağlantı **kuramıyor**. CTU MRS'in `link_attacher.cpp`'si tam bunu yapıyor ama Gazebo Classic + ROS1 için |

Her ikisi de ArduPilot'un `ParachutePlugin.cc`'sindeki
`_ecm.CreateEntity()` + `components::DetachableJoint` deseninden türetildi.

Arayüz:
```
/bullet_net_interceptor/net/fire   (gz.msgs.Double)     → ateşle, data = çıkış hızı m/s
/net/captured                      (gz.msgs.StringMsg)  → yakalanan modelin adı
```

### Test dünyaları

| Dünya | Amaç |
|---|---|
| `worlds/bullet_net_test.sdf` | Tam sistem: interceptor + taret + ağ + hedef (interceptor yerde) |
| `worlds/net_ballistics.sdf` | Menzil kalibrasyonu: ağ 10 m'de askıda, gövde yok |
| `worlds/net_capture_test.sdf` | Deterministik yakalama testi: hedef ölçülen yörünge üzerinde |

---

## Ölçüm sonuçları

### Menzil taraması (`docs/bench_raw/menzil_taramasi.csv`)

Gövdeden bağımsız balistik ölçüm (`net_ballistics.sdf`, ağ 10 m'de askıda):

| Çıkış hızı | Tilt 0° | Tilt −10° | Tilt −20° |
|---|---|---|---|
| 15 m/s | 19.8 m | 23.1 m | 24.6 m |
| **20 m/s** | **26.6 m** | 32.4 m | 34.5 m |
| 25 m/s | 33.3 m | 41.8 m | 44.1 m |

Tekrarlanabilirlik: 20 m/s / 0° için 3 koşumda 26.66 / 26.60 / 26.65 m (±%0.2).

### Ağ modeli — bilinçli basitleştirme

`net_cone` geometrik olarak **tam açık** bir koni (0.7 m ağız çapı — yakalama
hacmi), ama aerodinamik olarak **toplu paket** gibi modellendi
(sürükleme alanı 0.10 m², ağız alanı 0.385 m² değil).

Sebep ölçüldü: tam açık alanla 18 m/s'de 90 N direnç çıkıyor ve ağ 4.3 m'de
duruyor — DroneCatcher'ın 20 m menzili fiziksel olarak imkânsız olurdu. Gerçek
ağ tabancaları toplu bir paket atar, ağ hedefe yakın açılır. 2. aşamada
(`net_mesh`) açılma gerçekten modellenecek.

### Kütle bütçesi

Taret gövdenin **tepesine** oturunca kütle merkezi belirgin şekilde yükseliyor.
Ölçülen (taretsiz ham gövde → taret + ağ yüklü):

| Büyüklük | Taretsiz | Ağ yüklü | Oran |
|---|---|---|---|
| Kütle | 1.940 kg | 2.590 kg | ×1.34 |
| CG yüksekliği | +0.6 mm | **+76.1 mm** | rotor düzlemi z = −60 mm'de sabit |
| Ixx (roll) | 0.04514 | 0.08943 | ×1.98 |
| Iyy (pitch) | 0.04523 | 0.09344 | ×2.07 |
| Izz (yaw) | 0.01118 | 0.01523 | ×1.36 |
| İtki/ağırlık | 9.0 : 1 | 6.8 : 1 | |

`config/bullet_net_interceptor.param` bu oranlara göre ölçeklendi: 22 parametre
değişti (`MOT_THST_HOVER` 0.14→0.19, rate PID'leri kendi eksenlerinin atalet
oranıyla, harmonik notch 84→98 Hz). Her satırın sonunda gerekçesi yazılı.

Kazançlar **ağ yüklü** hale göre ayarlandı, çünkü kritik faz (nişan + atış) ağ
takılıyken yaşanıyor. Ağ atıldıktan sonra atalet ~yarıya düşer ve aynı kazançlar
yüksek kalır — atış sonrası salınım görülürse `ATC_RAT_*_P/D` kademeli düşürülmeli.

> **Bu değerler ölçüm değil.** Kütle/atalet oranlarından türetilmiş başlangıç
> değerleridir; SITL'de doğrulanmadılar (aşağıya bakın).

---

## Bilinen sınırlar

- **`bullet_net_test.sdf` yakalaması kaotik olabilir.** İnterceptor bu dünyada
  yerde duruyor; ağ alçak irtifadan atılıp zeminde sekebiliyor, hedefe isabet
  tesadüfe kalıyor. Deterministik yakalama ölçümü için `42_capture_test.sh`
  kullanın.
- **Model uçurulmadı.** ArduPilot SITL bağlantısı bir kez kuruldu ama
  arm aşamasında `PreArm: Accels inconsistent` ile takıldı; ilgili model ve
  test scriptleri silindi.
- **Uçuş parametreleri doğrulanmadı.** `bullet_net_interceptor.param` hesapla
  türetildi, SITL'de sınanmadı. Taret ve ağ tarafı simülasyonda doğrulandı
  (komut −8.00° → ölçülen −8.03°, yakalama tuttu) ama uçuş değil.
- **Hedef basit.** `target_box` gerçek bir hedef drone değil, 0.7 kg'lık bir
  kutu; direğe oturtulup ağ çarpınca sökülüyor. Uçan, manevra yapan hedef
  Aşama 7'de gelecek.
- **Tepe-ağır.** CG rotor düzleminin 136 mm üstünde. Taret tepede olduğu sürece
  kaçınılmaz; yalpalama-öteleme kuplajını artırır, tuning'i zorlaştırır.
- **Namlu kamerayı kesiyor.** Tilt tam aşağı (+30°) geldiğinde namlu, burun
  kamerasının görüş alanının üst kısmına giriyor. Kamera → taret otomasyonuna
  geçerken hesaba katılmalı.
- **Kamera hattı bağlanmadı.** Otonom nişanlama (YOLOv8 / HSV → taret açısı)
  henüz yok; taret elle komutlanıyor.
- **Model yeniden üretilemez.** Ham gövde (`cand_bullet`) ve üretici script
  (`21_build_bullet_interceptor.py`) depodan çıkarıldı. `model.sdf` elle
  düzenlenir; gerekirse git geçmişinden geri alınabilir.

## Sıradaki adımlar

1. **Aşama 7:** ArduPilot SITL ile uçur — 2.59 kg'da hover doğrula (param dosyası
   hesapla türetildi, ilk sınanacak yer burası), uçan hedefe angajman yap
2. **Aşama 6:** `net_mesh` — 5×5 düğümlü örgü ağ, `net_cone` ile aynı arayüz
3. Kamera → hedef tespiti → taret nişan açısı otomasyonu
4. Yakalanan hedefi güvenli bölgeye taşıma / kontrollü paraşüt modu

---

## Ticari referanslar

Ticari sistemlerin (Fortem, DroneCatcher, Airspace) modelleri kapalı kaynak.
Ayrıntılı döküm: [`ticari_referanslar/README.md`](ticari_referanslar/README.md).
