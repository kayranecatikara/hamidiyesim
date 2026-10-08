# ÇALIŞTIRMA — tam komut listesi (script yok, hepsi elle)

VAKKAS simülasyonunu ayağa kaldırmak için **5 terminal**. Her komut olduğu
gibi kopyalanır. Sıra önemlidir: Gazebo → SITL'ler → parametre → arayüz.

> ⚠ Neden sıra önemli: `iris_cam/model.sdf`'teki ArduPilotPlugin
> `<lock_step>1</lock_step>` ile çalışır — Gazebo HER fizik adımında uçuş
> kontrolcüsünden cevap bekler. SITL'ler yokken Gazebo boşta bekler.
> Arayüz de en sona kalmalı: Gazebo'dan ÖNCE açılan kamera aboneliği geri
> gelmiyor.

---

## TERMİNAL 1 — Gazebo (dünya + araçlar)

```bash
cd ~/projects/avci_sim
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
gz sim -r -v2 sim/gazebo_harmonic/worlds/avci_harmonic.sdf
```

Görüntüsüz (daha hızlı, taret/kamera yine çalışır):
```bash
gz sim -s -r --headless-rendering -v2 sim/gazebo_harmonic/worlds/avci_harmonic.sdf
```

---

## TERMİNAL 2 — ArduCopter SITL (avcı drone, VAKKAS)

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  -I0 --sysid 5 --no-rebuild \
  --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/copter.parm \
  --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_copter.parm \
  --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
```

> ⚠ `cd ~/ardupilot` ŞART. Başka dizinden çalıştırırsan SITL kendi artıklarını
> (`mav_5_1.parm`, `mav.tlog.raw`, `terrain/`) O dizine yazar.
> ⚠ İlk iki `--add-param-file` de ŞART: `--model JSON` verildiği için SITL
> frame varsayılanlarını bulamıyor, "PreArm: Motors: Check frame class and
> type" der ve kalkış hiç olmaz.

EKF+GPS kilidi ~60 s sürer. `EKF3 IMU0 is using GPS` satırını bekle.

---

## TERMİNAL 3 — ArduPlane SITL (hedef uçak)

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 \
  -I1 --sysid 2 --no-rebuild \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_plane.parm \
  --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
```

---

## TERMİNAL 4 — parametre doğrulama (SITL hazır olunca, BİR KEZ)

```bash
cd ~/projects/avci_sim
python3 tools/param_dogrula.py --yaz
```

> ⛔ NİYE ŞART: `--add-param-file` yalnız VARSAYILANI değiştirir. Bir
> parametrenin `~/ardupilot/eeprom.bin` içinde KAYITLI değeri varsa o kazanır
> ve dosya SESSİZCE yok sayılır. Bu yüzden araç aylarca 70° yerine 60° eğim
> tavanıyla uçtu. Bu komut dosyayla aracı karşılaştırır ve farkı yazar.
> `uyuşmayan 0` görmelisin.

---

## TERMİNAL 5 — arayüz (GCS + kamera + tespit + taret + ağ)

```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export AVCI_GZ_CAMERA=1
python3 -m control.gcs_server
```

Sonra tarayıcı: **http://127.0.0.1:8000**
(sunucu tarayıcıyı kendisi de açar)

Beklenen satırlar:
```
[TARET] eklem durumu dinleniyor: /world/avci/model/iris_with_ardupilot/joint_state
[GCS] gz-transport kamera dinleniyor (/iris_cam/image, Harmonic)
[GCS] ✓ Iris kamerasından ilk görüntü!
```

---

## PANELDE NE YAPILIR

1. Üstte **AVCI DRONE** sekmesine geç → **🔭 TARET (namlu + kamera)** kartı
2. Kare joystick'i sürükle: yatay = pan (±180°), dikey = tilt (−30…+90°)
   - **FPV görüntüsü taretle birlikte döner** (kamera namluyla aynı doğrultuda)
   - **KOMUT / GERÇEK / HATA** satırları eklemin komuta gerçekten gittiğini gösterir
3. Hazır açılar: `İLERİ 25°` `DÜZ` `TAM ÜST` `SOL 90` `SAĞ 90` `ALT −30`
4. **🕸 AĞ AT** → ağ namlunun baktığı yöne gider; altta sonuç yazar
   (YAKALANDI / ISKA / YERE DÜŞTÜ · menzil · en yakın · çıkış hızı)

---

## TERMİNALDEN TARET VE AĞ (panel olmadan)

```bash
cd ~/projects/avci_sim

# taret açısı (derece)
curl -X POST http://127.0.0.1:8000/api/taret \
     -H 'Content-Type: application/json' -d '{"pan":-45,"tilt":30}'
curl http://127.0.0.1:8000/api/taret

# doğrudan Gazebo'ya (radyan) — arayüz kapalıyken de çalışır
gz topic -t /model/iris_with_ardupilot/joint/turret_yaw_joint/0/cmd_pos \
         -m gz.msgs.Double -p 'data: -0.785'
gz topic -t /model/iris_with_ardupilot/joint/turret_pitch_joint/0/cmd_pos \
         -m gz.msgs.Double -p 'data: 0.524'

# ağ at
curl -X POST http://127.0.0.1:8000/api/ag/ates \
     -H 'Content-Type: application/json' -d '{"hiz":22}'
curl http://127.0.0.1:8000/api/ag/durum

# ağ fırlatıcı komut satırından (arayüz gerekmez)
python3 -m control.ag_firlatici --ates --hiz 22
python3 -m control.ag_firlatici --menzil-tablosu     # uçmadan balistik tablo
```

---

## ÖLÇÜM ARAÇLARI

```bash
cd ~/projects/avci_sim
python3 tools/sdf_kutle.py iris_cam                       # toplam kütle (4.0000 kg)
python3 tools/asili_gaz_olc.py --kutle 4.0 --etiket k1    # itki/ağırlık (uçar)
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
  python3 tools/taret_dene.py --dizin /tmp/taret          # taret 9 duruş + kareler
python3 tools/gorev_saglamasi.py --hiz 18                 # kare deseni uçuş
python3 tools/egim_zarfi.py --acilar 30,45                # eğim zarfı
python3 tools/param_dogrula.py                            # yalnız rapor
```

---

## DURDURMA

```bash
# 1) önce NE ayakta, gör:
ps -eo pid,args | grep -E '[g]z sim|[s]im_vehicle|[m]avproxy|[b]in/ardu|[c]ontrol\.gcs_server'

# 2) hepsini indir (kendi kabuğunu ve atalarını DIŞLAR):
KORU="$$ $PPID"
for p in $(ps -eo pid,args \
      | grep -E '[g]z sim|[s]im_vehicle|[m]avproxy|[b]in/ardu|[c]ontrol\.gcs_server' \
      | awk '{print $1}'); do
    case " $KORU " in *" $p "*) continue ;; esac
    kill -9 "$p"
done

# 3) teyit et — çıktı BOŞ olmalı:
ps -eo pid,args | grep -E '[g]z sim|[s]im_vehicle|[m]avproxy|[b]in/ardu|[c]ontrol\.gcs_server'
```

> ⛔ **`pkill -f 'gz sim|...'` KULLANMA.** `pkill -f` ÇAĞIRAN KABUĞUN komut
> satırına da bakar; desen orada da geçtiği için kabuğu öldürür (exit 144) ve
> satırın geri kalanı HİÇ çalışmaz — süreçler ayakta kalır ama sen
> "durdurdum" sanırsın. Bu depoda iki kez yaşandı.
>
> Yukarıdaki komut iki katmanla korunuyor:
> 1. **Köşeli parantez hilesi** (`[g]z sim`): regex "gz sim" metnini arar;
>    komutun kendi satırında yazan "[g]z sim" buna UYMAZ.
>    ⚠ Bu hile yalnız `ps | grep` için geçerlidir, `pkill -f` için DEĞİL.
> 2. **`KORU="$$ $PPID"`**: kendi kabuk PID'ini ve ata sürecini listeden düşer.

## ORTAM DEĞİŞKENLERİ (hepsi isteğe bağlı)

| değişken | varsayılan | ne yapar |
|---|---|---|
| `AVCI_GZ_CAMERA` | — | **1 ŞART** — yoksa ROS2 moduna düşer, görüntü gelmez |
| `AVCI_ONIZLEME_EN` | 960 | panele giden MJPEG genişliği (0 = küçültme yok) |
| `AVCI_TRACKER` | on | `off` → HybridSORT kapanır, tespit hızlanır |
| `AVCI_DETECTOR` | yolo | `off` → YOLO kapanır |
| `AVCI_AG_HIZ` | 20 | namlu hızı (m/s) |
| `AVCI_AG_GOZENEK` | 0.90 | ağın açık alan oranı — menzili BELİRLER |
| `AVCI_AG_YARICAP` | 0.60 | açılmış ağ ağız yarıçapı (m) |
| `AVCI_AG_ACILMA` | 0.20 | ağın açılma süresi (s) — menzile çok etkili |
