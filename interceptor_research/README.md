# Interceptor Research — Ağ Atan Taretli İnterceptor Drone

BismillahirRahmanirRahim.

Hedefi imha etmek yerine **2 eksenli taretten ağ fırlatıp yakalayan** bir
interceptor drone simülasyonu. Fortem DroneHunter F700 / Delft Dynamics
DroneCatcher sınıfı bir sistemin Gazebo Harmonic karşılığı.

**İki ayrı gövde tasarımı var.** Taret zinciri, ağ fırlatıcı ve yakalama
eklentisi ikisinde de aynı; değişen yalnızca gövde ve taretin nereye oturduğu:

| | `avci_net_interceptor` | `bullet_net_interceptor` |
|---|---|---|
| Gövde | yatay iris quadcopter (`cand_iris`) | dikey mermi/roket gövde (`cand_bullet`) |
| Taret nerede | **burunda** (x = +0.16) | **tepede** (z = +0.265, burun konisi kaldırıldı) |
| Kütle (ağ yüklü) | 2.40 kg | 2.59 kg |
| Ölçülen menzil | 26.6 m (20 m/s, 0°) | 27.8 m (20 m/s, −8°) |
| Üretici | `scripts/20_build_interceptor.py` | `scripts/21_build_bullet_interceptor.py` |

Her ikisinde de namlu **+X**'e bakar, ağ **ileri** atılır; taret limitleri aynı
(pan ±100°, tilt −60°..+30°).

> **Bu proje `avci_sim`'den ayrıdır.** `avci_sim` Teknofest Avcı İHA yarışması için
> **kamikaze** müdahale yapıyor. Burada amaç **yakalama**. Aynı yığın kullanılıyor
> (ArduPilot + Gazebo Harmonic) ki olgunlaşan ağ/taret modülü ileride oraya taşınabilsin.

**Yığın:** ArduPilot · Gazebo Harmonic (gz-sim 8.14) · gz-transport 13 · ROS 2 Humble · Python 3.10

📷 **Simülasyon görüntüleri: [`docs/GORUNTULER.md`](docs/GORUNTULER.md)**


---

## Durum özeti

| Aşama | Durum | Kanıt |
|---|---|---|
| 0 · Kaynak repoların klonlanması | ✅ 11 repo, 3.2 GB | `repos/` |
| 1 · Aday gövdelerin Harmonic'e alınması + kıyas | ✅ 8 aday ölçüldü | [`docs/KIYAS_RAPORU.md`](docs/KIYAS_RAPORU.md) |
| 2 · Gövde seçimi ve fork | ✅ `cand_iris` seçildi | [`docs/SECIM_KARARI.md`](docs/SECIM_KARARI.md) |
| 3 · 2 eksenli taret | ✅ komut −10° → ölçülen −9.77° | `scripts/turret_state.py` |
| 4 · Ağ fırlatma | ✅ 20 m/s → 26.6 m menzil, %0.2 tekrarlanabilirlik | [`docs/bench_raw/menzil_taramasi.csv`](docs/bench_raw/menzil_taramasi.csv) |
| 5 · Yakalama (hedefin ağa kilitlenmesi) | ✅ **5/5 başarı** | `scripts/42_capture_test.sh` |
| 5b · İkinci gövde: mermi gövdeli, taret tepede | ✅ menzil 27.8 m, yakalama tuttu | `worlds/bullet_net_test.sdf` |
| 6 · Örgü ağ (5×5 düğüm) | ⬜ yapılmadı | — |
| 7 · ArduPilot SITL ile uçan hedef angajmanı | ⬜ yapılmadı | — |

**Ticari referanslarla karşılaştırma:**

| Büyüklük | Hedef (ticari) | 1. tasarım (iris) | 2. tasarım (mermi) |
|---|---|---|---|
| Ağ menzili | 20 m (DroneCatcher) | **26.6 m** @ 20 m/s, 0° | **27.8 m** @ 20 m/s, −8° |
| Çıkış hızı | — | 18.1 m/s (komut 20, sürükleme kaybı) | aynı fırlatıcı |
| Interceptor kütlesi | ≤ 6 kg (DroneCatcher) | **2.10 kg** kuru (1.75 gövde + 0.35 taret), ağ yüklü **2.40 kg** | **2.29 kg** kuru (1.94 gövde + 0.35 taret), ağ yüklü **2.59 kg** |
| Yakalama oranı | %85 ilk atış (Fortem F700) | **%100** (5/5, kontrollü senaryo) | tuttu (tek koşum) |

Ağ her iki tasarımda da ayrı model (`net_cone`, 0.30 kg).

---

## Hızlı başlangıç

```bash
cd ~/projects/interceptor_research

# 1) Kaynak repoları çek (bir kez)
./scripts/00_clone_all.sh

# 2) Aday gövdeleri hazırla
python3 scripts/10_stage_candidates.py
python3 scripts/11_render_jinja.py

# 3) Kıyas tezgahı (Gazebo headless) + rapor
./scripts/12_bench.sh
python3 scripts/13_report.py        # -> docs/KIYAS_RAPORU.md

# 4) Taretli interceptor'ları üret (iki gövde de)
python3 scripts/20_build_interceptor.py          # 1. tasarım: iris, taret burunda
python3 scripts/21_build_bullet_interceptor.py   # 2. tasarım: mermi, taret tepede

# 5) Ağ atıcı eklentilerini derle
cd plugins && mkdir -p build && cd build && cmake .. && make -j4 && cd ../..

# 6) Doğrulama
./scripts/40_verify.sh 0 -8 20      # taret + atış + yakalama, uçtan uca
./scripts/41_range_test.sh          # menzil taraması (balistik)
./scripts/42_capture_test.sh 5 20   # yakalama başarı oranı

# 7) Görsel: bütün modelleri yan yana göster
python3 scripts/50_showcase.py
gz sim -r worlds/showcase.sdf       # canlı pencere
```

Görsel olarak izlemek için:
```bash
source scripts/env.sh

# 1. tasarım (iris, taret burunda)
gz sim -r worlds/net_test.sdf            # ayrı terminalde GUI
python3 scripts/turret_aim.py 25 -15     # tareti çevir
python3 scripts/fire_net.py --hiz 20     # ağı at

# 2. tasarım (mermi, taret tepede) — aynı araçlar, --model ile
gz sim -r worlds/bullet_net_test.sdf
python3 scripts/turret_aim.py 25 -15 --model bullet_net_interceptor
python3 scripts/fire_net.py  --hiz 20    --model bullet_net_interceptor
```

`turret_aim.py`, `fire_net.py`, `turret_state.py` ve `52_action_shots.py`
`--model` almadığında **1. tasarımı** sürer; eski komutlar aynen çalışır.

---

## Mimari

### Modeller

```
models/
├── interceptors/
│   ├── avci_net_interceptor/   ← 1. tasarım (20_build_interceptor.py)
│   │   └── iris gövdesi + BURUNDA taret + namlu
│   ├── bullet_net_interceptor/ ← 2. tasarım (21_build_bullet_interceptor.py)
│   │   └── dikey mermi gövde + TEPEDE taret; burun konisi kaldırıldı
│   ├── cand_bullet/            ← 2. tasarımın ham gövdesi (taretsiz, elle değiştirilmez)
│   ├── cand_iris/              ← seçilen aday (avci_sim iris_cam kopyası)
│   ├── cand_iq_camera/         ← iq_sim, Classic→Harmonic çevrildi
│   ├── cand_mrs_{x500,t650,m690,f450,naki}/  ← jinja'dan render
│   ├── cand_d2d_x500/          ← PX4 bağımlılığı nedeniyle değerlendirilemedi
│   ├── iris_with_standoffs/    ← cand_iris bağımlılığı
│   └── iris_base/              ← cand_iq_camera bağımlılığı (çevrilmiş)
├── net_launchers/net_cone/     ← 1. aşama ağ: rijit koni "yakalama hacmi"
└── targets/target_box/         ← 0.7 kg dinamik test hedefi
```

### Taret zinciri

```
iris_with_standoffs::base_link
  └─ turret_mount_joint (fixed)      → turret_base_link    0.10 kg
       └─ turret_yaw_joint (Z, ±100°) → turret_yaw_link     0.08 kg
            └─ turret_pitch_joint (Y, −60°..+30°) → turret_pitch_link  0.10 kg
                 └─ muzzle_joint (fixed) → muzzle_link      0.07 kg
                                                   toplam:  0.35 kg
```

Her iki eksen `gz-sim-joint-position-controller-system` ile sürülüyor:
```
/model/avci_net_interceptor/joint/turret_yaw_joint/0/cmd_pos    (gz.msgs.Double, radyan)
/model/avci_net_interceptor/joint/turret_pitch_joint/0/cmd_pos
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
/avci_net_interceptor/net/fire   (gz.msgs.Double)     → ateşle, data = çıkış hızı m/s
/net/captured                    (gz.msgs.StringMsg)  → yakalanan modelin adı
```

### Test dünyaları

| Dünya | Amaç |
|---|---|
| `worlds/bench.sdf.in` | Aday gövde kıyas şablonu (`@CANDIDATE@` değiştirilir) |
| `worlds/net_test.sdf` | Tam sistem: interceptor + taret + ağ + hedef (interceptor yerde) |
| `worlds/net_ballistics.sdf` | Menzil kalibrasyonu: ağ 10 m'de askıda, gövde yok |
| `worlds/net_capture_test.sdf` | Deterministik yakalama testi: hedef ölçülen yörünge üzerinde |
| `worlds/bullet_net_test.sdf` | 2. tasarım (mermi gövde) için atış + yakalama testi |

---

## Ölçüm sonuçları

### Menzil taraması (`docs/bench_raw/menzil_taramasi.csv`)

| Çıkış hızı | Tilt 0° | Tilt −10° | Tilt −20° |
|---|---|---|---|
| 15 m/s | 19.8 m | 23.1 m | 24.6 m |
| **20 m/s** | **26.6 m** | 32.4 m | 34.5 m |
| 25 m/s | 33.3 m | 41.8 m | 44.1 m |

10 m irtifadan atış. Tekrarlanabilirlik: 20 m/s / 0° için 3 koşumda
26.66 / 26.60 / 26.65 m (±%0.2).

### Ağ modeli — bilinçli basitleştirme

`net_cone` geometrik olarak **tam açık** bir koni (0.7 m ağız çapı — yakalama
hacmi), ama aerodinamik olarak **toplu paket** gibi modellendi
(sürükleme alanı 0.10 m², ağız alanı 0.385 m² değil).

Sebep ölçüldü: tam açık alanla 18 m/s'de 90 N direnç çıkıyor ve ağ 4.3 m'de
duruyor — DroneCatcher'ın 20 m menzili fiziksel olarak imkânsız olurdu. Gerçek
ağ tabancaları toplu bir paket atar, ağ hedefe yakın açılır. 2. aşamada
(`net_mesh`) açılma gerçekten modellenecek.

### 2. tasarım (mermi gövde) — kütle bütçesi

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

- **`net_test.sdf` yakalaması kaotik.** İnterceptor bu dünyada yerde duruyor;
  ağ alçak irtifadan atılıp zeminde sekiyor, hedefe isabet tesadüfe kalıyor.
  Deterministik yakalama ölçümü için `42_capture_test.sh` kullanın.
- **Hiçbir model uçurulmadı.** ArduPilot SITL bağlantısı bir kez kuruldu ama
  arm aşamasında `PreArm: Accels inconsistent` ile takıldı; ilgili model ve
  test scriptleri silindi.
- **Hedef basit.** `target_box` gerçek bir hedef drone değil, 0.7 kg'lık bir
  kutu; direğe oturtulup ağ çarpınca sökülüyor. Uçan, manevra yapan hedef
  Aşama 7'de gelecek.
- **MRS adayları uçmuyor.** Geometri/kütle ölçüldü ama CTU'nun
  `MrsGazeboCommonResources_MulticopterMotorModel` kütüphanesi olmadığı için
  motorları dönmüyor — RTF değerleri `cand_iris` ile kıyaslanamaz.
- **Kamera hattı bağlanmadı.** Otonom nişanlama (YOLOv8 / HSV → taret açısı)
  henüz yok; taret elle komutlanıyor.
- **2. tasarımın uçuş parametreleri doğrulanmadı.** `bullet_net_interceptor.param`
  hesapla türetildi, SITL'de sınanmadı. Taret ve ağ tarafı simülasyonda doğrulandı
  (komut −8.00° → ölçülen −8.03°, menzil 27.84 m, yakalama tuttu) ama uçuş değil.
- **2. tasarım tepe-ağır.** CG rotor düzleminin 136 mm üstünde (1. tasarımda
  61 mm). Taret tepede olduğu sürece kaçınılmaz; yalpalama-öteleme kuplajını
  artırır, tuning'i zorlaştırır.
- **Namlu kamerayı kesiyor.** 2. tasarımda tilt tam aşağı (+30°) geldiğinde namlu,
  burun kamerasının görüş alanının üst kısmına giriyor. Kamera → taret
  otomasyonuna geçerken hesaba katılmalı.

## Sıradaki adımlar

1. **Aşama 7:** ArduPilot SITL ile interceptor'ları uçur — 1. tasarımda 2.40 kg,
   2. tasarımda 2.59 kg'da hover doğrula (2. tasarımın param dosyası hesapla
   türetildi, ilk sınanacak yer burası), uçan hedefe (mini Talon veya quad)
   angajman yap
2. **Aşama 6:** `net_mesh` — 5×5 düğümlü örgü ağ, `net_cone` ile aynı arayüz
3. Kamera → hedef tespiti → taret nişan açısı otomasyonu
   (`repos/PX4-ROS2-Gazebo-YOLOv8` ve `repos/d2dtracker_sim` referans)
4. Yakalanan hedefi güvenli bölgeye taşıma / kontrollü paraşüt modu

---

## Kaynak repolar (`repos/`)

Araştırma sonucu: **verilen 8 reponun hiçbirinde ağ atan interceptor SDF'i yok.**
Ticari sistemlerin (Fortem, DroneCatcher, Airspace) modelleri kapalı kaynak,
"CTU MRS Net Launcher Plugin" ise mevcut değil. Ayrıntılı döküm:
[`docs/KIYAS_RAPORU.md`](docs/KIYAS_RAPORU.md) ve
[`ticari_referanslar/README.md`](ticari_referanslar/README.md).

Repolar gövde / sahne / algoritma tedarikçisi olarak kullanılıyor.
