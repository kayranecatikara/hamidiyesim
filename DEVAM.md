# DEVAM — başka bir makinede kaldığın yerden

Bu dosya "yarın laptop'ta açtım, ne yapacaktım?" sorusunun cevabı.
Asıl iş listesi **[UYGULANACAK.md](UYGULANACAK.md)**; oradaki **DURUM**
bölümü her zaman güncel tutulur. Bu dosya ona nasıl ulaşacağını anlatır.

---

## 1. Dalı senkronla (laptop'ta)

```bash
cd ~/projects/avci_sim
git fetch origin
git checkout kubra_laptop
git merge origin/kubra_masaustu       # masaüstündeki iş buraya gelir
```

Çakışma çıkarsa: masaüstü dalı **her zaman daha yeni**, çakışan dosyada
onu al (`git checkout --theirs <dosya>`) ve `UYGULANACAK.md` ile
`TODO.md`'yi elle gözden geçir.

Laptop'ta iş bitince ters yön:

```bash
git push origin kubra_laptop
# masaüstünde:  git merge origin/kubra_laptop
```

> **Neden iki dal?** Tek dal kullanıp iki makineden push etmek de olurdu,
> ama açık PR (#4) `kubra_masaustu` üzerinde. Laptop'ta çalışıp
> `kubra_masaustu`'ye merge etmek PR'ı otomatik günceller.

## 2. Çalışıyor mu — 30 saniyede doğrula

```bash
python3 -m tests.test_visual_lead      # 53/53 bekleniyor
python3 -m tests.test_gps_guidance     # 12/12 bekleniyor
```

Bu ikisi geçiyorsa güdüm kodu sağlam demektir. Geçmiyorsa **önce bunu
düzelt**, uçma.

## 3. Simülasyonu başlat

```bash
# Terminal A — ~50 s sürer, KOMUT SATIRI GERİ GELENE KADAR BEKLE
GZ_HEADLESS=1 bash scripts/start_harmonic.sh

# Terminal B — A bittikten SONRA, ayrı terminalde
source /opt/ros/humble/setup.bash && export AVCI_GORSEL=on AVCI_GZ_CAMERA=1 AVCI_NO_BROWSER=1
fuser -k 8000/tcp 2>/dev/null; python3 -m control.gcs_server
```

Sonra <http://localhost:8000>.

| komut | ne yapar |
|---|---|
| `bash scripts/start_harmonic.sh stop` | durdurur ve **doğrular** (Ctrl+C işe yaramaz) |
| `bash scripts/start_harmonic.sh durum` | ne çalışıyor — hiçbir şeyi öldürmez |

⚠ **Elle `pkill -9 -f 'gz sim|sim_vehicle|...'` YAZMA.** `pkill -f` deseni
çağıran kabuğun komut satırında da arar; o satırda aynı kelimeler geçtiği
için **kendi terminalini öldürür** ve satırın gerisi hiç çalışmaz. Ölçüldü.

⚠ **Terminal B'yi A'dan ÖNCE başlatma.** Gazebo'dan önce açılan gz kamera
aboneliği geri gelmiyor — `✓ ilk görüntü` satırları hiç çıkmaz, arayüzde
kamera kararır. Ayrıca A açılışta yaptığı temizlikte B'yi de öldürür.

**Sürenin neden 50 s olduğu:** ~4 s Gazebo, ~46 s ArduPilot'un kendi EKF+GPS
kilidi. Kısaltılamaz, kalkış zaten ondan önce mümkün değil. Script her
koşuda faz zamanlamasını `⏱` satırlarıyla basar.

## 4. Ne yapacaktın

**[UYGULANACAK.md](UYGULANACAK.md) → DURUM bölümü.** Orada biten maddeler,
sıradakiler, son ölçülen uçuş sonuçları ve "tekrar denenmeyecekler" listesi
var. Yeni bir Claude oturumu açacaksan tek cümle yeter:

```
UYGULANACAK.md dosyasını oku, başındaki DURUM bölümünden devam edeceğiz.
```

**Çalışma kuralı:** tek seferde tek değişken → testler → uç → ölç →
*Sonuç:* satırına yaz. Bu kural, sekiz değişikliğin bir arada uçurulup
hangisinin ne yaptığının ayırt edilememesi üzerine kondu.

## 5. Uçtuktan sonra: ölçüm

```bash
python3 tools/gecis_analiz.py            # en son uçuş
python3 tools/gecis_analiz.py 126 127    # belirli BIN'ler
python3 tools/gecis_analiz.py --liste    # son 10 uçuş
```

Çıktı: her geçişin gerçek menzili/yatayı/**dikeyi**, seyirdeki heading
titremesi, ve `visual_lead` fazlarının durum dağılımı.

> **Neden bu araç var:** "nereden ıskaladı" sorusunun cevabı CSV'de YOK.
> `menzil_gercek_m` MAVLink telemetrisinden geliyor, EKF çerçeve ofsetinden
> etkileniyor ve en yakın anı geriden gösteriyor — bir vuruşta CSV 3.20 m
> derken kara kutu 0.21 m diyordu. Araç iki aracın `POS` kaydını GPS haftası
> saatiyle hizalayıp gerçek geometriyi çıkarıyor.

Diğer ölçüm komutları:

```bash
curl -s localhost:8000/api/debug/carpisma   | python3 -m json.tool  # temas kaynağı sağlam mı
curl -s localhost:8000/api/telemetry/pnp    | python3 -m json.tool  # faz ve kapılar
python3 tools/parm_denetle.py                                        # parametreler uygulandı mı
```

## 6. Laptop'ta ayrıca gerekenler (depoda YOK)

Bunlar depo dışında, laptop'ta zaten kurulu olmalı:

- `~/ardupilot` — SITL (`sim_vehicle.py`, `build/sitl/bin/arducopter|arduplane`)
- `~/ardupilot_gazebo` — `ArduPilotPlugin` (`build/` derlenmiş olmalı)
- ROS 2 Humble + Gazebo Harmonic + `gz-transport13` / `gz-msgs10` python paketleri
- `pymavlink`, `opencv`, `ultralytics`, `fastapi`, `uvicorn`

Kurulum adımları: `README.md` → "ADIM 9: KURULUM DOĞRULAMASI".

**Depoda olan ve otomatik gelen:** YOLO ağırlıkları
(`vision/models/avci_yolo.pt`, `avci_pose.pt` — `.gitignore`'da özellikle
beyaz listede), Gazebo dünya/model dosyaları, ArduPilot parametreleri.

**Gelmeyen:** `logs/` (gitignore) — uçuş CSV'leri ve `~/ardupilot/logs/*.BIN`
makinede kalır. Masaüstündeki ölçümleri laptop'tan tekrar üretemezsin; bu
yüzden bütün sonuçlar `UYGULANACAK.md`'ye ve commit mesajlarına sayılarıyla
yazılıyor.

## 7. Tekrar denenmeyecekler

Bunların hepsi denendi, **ölçümle çürütüldü**, gerekçesi ilgili dosyada
yorum olarak duruyor. Tekrar denemeden önce o yorumu oku:

| fikir | nerede yazılı | ne oldu |
|---|---|---|
| `ATC_ANG_YAW_P` 4.5 → 3.0 | `sim/ardupilot_params/avci_copter.parm` | yaw takip hatası 1.4° → 8.5-11.9° |
| `supervisor.KILIT_N` 10 → 7 | `control/guidance/supervisor.py` | faz/uçuş 3.4 → 8.0, her ölçüt kötüleşti |
| hedef hızına ivme kapısı | `UYGULANACAK.md` A4 | hız kestiriminin sıfırdan oturmasını da engelledi |
| "araç komutu uygulamıyor" teşhisi | `UYGULANACAK.md` A7 | kara kutu çürüttü; takip hatası 0.1 m/s |
| `pkill` köşeli parantez hilesi | `dokumantasyon/17_KOD_scripts_tools_tests.md` | `pkill -f` için geçersiz, kendi kabuğunu öldürüyor |

**Kural:** "araç komutu uygulamıyor" demeden önce MUTLAKA kara kutuda
`PSCD.DVD` vs `PSCD.VD` (dikey) veya `ATT.DesYaw` vs `ATT.Yaw` (yaw)
karşılaştır.
