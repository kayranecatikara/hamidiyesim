# Interceptor Research — Ağ Atan Taretli İnterceptor Drone

BismillahirRahmanirRahim.

Hedefi imha etmek yerine **burnundaki 2 eksenli taretten ağ fırlatıp yakalayan**
bir interceptor drone simülasyonu. Fortem DroneHunter F700 / Delft Dynamics
DroneCatcher sınıfı bir sistemin Gazebo Harmonic karşılığı.

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
| 6 · Örgü ağ (5×5 düğüm) | ⬜ yapılmadı | — |
| 7 · ArduPilot SITL ile uçan hedef angajmanı | ⬜ yapılmadı | — |

**Ticari referanslarla karşılaştırma:**

| Büyüklük | Hedef (ticari) | Ölçülen |
|---|---|---|
| Ağ menzili | 20 m (DroneCatcher) | **26.6 m** @ 20 m/s, 0° |
| Çıkış hızı | — | 18.1 m/s (komut 20, sürükleme kaybı) |
| Interceptor kütlesi | ≤ 6 kg (DroneCatcher) | model **2.10 kg** (1.75 gövde + 0.35 taret); ağ yüklüyken **2.40 kg** (ağ ayrı model, 0.30 kg) |
| Yakalama oranı | %85 ilk atış (Fortem F700) | **%100** (5/5, kontrollü senaryo) |

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

# 4) Taretli interceptor'ı üret
python3 scripts/20_build_interceptor.py

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
gz sim -r worlds/net_test.sdf            # ayrı terminalde GUI
python3 scripts/turret_aim.py 25 -15     # tareti çevir
python3 scripts/fire_net.py --hiz 20     # ağı at
```

---

## Mimari

### Modeller

```
models/
├── interceptors/
│   ├── avci_net_interceptor/   ← ilk deneme (20_build_interceptor.py)
│   │   └── iris gövdesi + taret + namlu (düz çerçeve)
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

## Sıradaki adımlar

1. **Aşama 7:** ArduPilot SITL ile interceptor'ı uçur, 2.40 kg'da hover doğrula,
   uçan hedefe (mini Talon veya quad) angajman yap
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
