---
name: ucus-testi
description: Otonom uçuş testi — simi başlat, uçuşu HTTP API ile yönet, saniyede 1 kare + telemetri kaydet, görüntü+CSV çapraz analiziyle raporla. Güdüm değişikliklerini insan faktörü olmadan doğrulamak için. Kullanıcı "/ucus-testi" yazınca ya da "uçuş testini sen koş" deyince kullan.
---

# Otonom Uçuş Testi

Amaç ve arka plan: `docs/OTONOM_UCUS_TESTI.md`. Aşağıdaki adımları SIRAYLA
uygula; kullanıcıdan hiçbir adım isteme. Argüman verildiyse (ör. senaryo adı
veya süre) planı ona göre uyarla; verilmediyse varsayılan: daire (circle,
150 s) + kare (square, 150 s).

## 1 · Temizlik (kendi kabuğunu öldürme tuzağına dikkat)

```bash
pkill -TERM -f 'gz [s]im|gz-sim-[s]erver|gz-sim-[g]ui|sim_[v]ehicle|mav[p]roxy|ardu[c]opter|ardu[p]lane|model [J]SON|control.gcs_[s]erver|run_plane_[s]cenario'
sleep 3; pkill -KILL -f '<aynı desen>'
```
Köşeli parantezler şart: desen bunlarsız kendi `bash -c` komut satırınla
eşleşir ve kabuğunu öldürür (exit 144).

## 2 · Süreçleri arka planda başlat (4 adet, run_in_background)

- **Gazebo**: `cd ~/projects/avci_sim && source /opt/ros/humble/setup.bash &&
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build &&
  export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds &&
  export DISPLAY=:1 && gz sim -r -v2 sim/gazebo_harmonic/worlds/avci_harmonic.sdf`
- **ArduCopter** ve **ArduPlane**: `docs/SIMULASYON_CALISTIRMA.md`/geçmişteki
  bilinen sim_vehicle komutları, ama MAVProxy TTY'siz hemen öldüğü için
  `script -qfec "python3 Tools/autotest/sim_vehicle.py ..." /dev/null`
  ile sarmalayarak. Copter: `-I0 --sysid 5 -w` + avci_copter.parm;
  Plane: `-I1 --sysid 2` + avci_plane.parm.
- İki SITL çıktısında da `EKF3 IMU0 is using GPS` bekle (arka plan
  until-grep döngüsü).
- **gcs_server**: `export AVCI_GZ_CAMERA=1 && fuser -k 8000/tcp;
  python3 -m control.gcs_server` → `/api/scenario_status` 200 dönene kadar bekle.
- gcs_server açılış banner'larını KONTROL ET (test edilen kod gerçekten
  aktif mi — bu dalda ör. `[GPS] yükseliş her menzilde 15° kalır`,
  `[TAKIP] HybridSORT`; istasyon yükselişi SABİT, kararlı daldaki dinamik
  sürüm burada yok).

## 3 · Kayıt + BEKÇİ + uçuş

```bash
python3 tools/ucus_kaydi.py <scratchpad>/ucusN 420   # arka planda
curl -X POST http://127.0.0.1:8000/api/command/plane/scenario/circle
# hedef hızı > 12 m/s olana kadar bekle (/api/debug/telem)
curl -X POST http://127.0.0.1:8000/api/command/iris/start_chase
# faz 1: 150 s → sonra: .../plane/scenario/square → faz 2: 150 s
curl -X POST .../iris/stop_chase ; curl -X POST .../plane/stop_scenario
```
Düz uçuş ölçümü için senaryo **`duz`** kullan (manuel mod DEĞİL — manuel
nötr elevator'la uçak alçalıp 12 m'ye indi, koşu geçersiz oldu).

**BEKÇİ ZORUNLU**: chase başlar başlamaz `python3 tools/ucus_bekci.py
<uçuş_süresi+60> 30` komutunu **Monitor** olarak çalıştır. "İHLAL:" satırı
düşerse test SAPITMIŞTIR: uçuşu hemen durdur, o koşunun verisini GEÇERSİZ
say, simi komple öldürüp koşuyu baştan kur. (Ders: hedef 12 m'ye alçaldı,
canlı fark edilmedi; test sonrası kontrolsüz bırakılan uçak yerin 1738 m
altına savruldu ve kullanıcı bozuk simle karşılaştı.)

## 3b · GÜVENLİ KAPANIŞ (her testin sonunda, atlanamaz)

Test bitince araçları havada KONTROLSÜZ BIRAKMA — stop_chase/stop_scenario
sonrası uçak kumandasızdır ve çakılır. HER ZAMAN simi komple öldür
(1. adımdaki köşeli parantezli pkill) ve kullanıcıya "sim kapalı, kendin
uçuracaksan baştan başlat" de. Kullanıcı "açık kalsın" dediyse bile araçlar
uçuyorken bırakma: önce senaryoyu yeniden başlat (uçak desende kalır) ya da
kapat.

## 4 · Analiz — dört bacak, dördü de zorunlu

0. **GEÇERLİLİK**: `tools/ucus_analiz.py` çıktısındaki GEÇERLİLİK satırına
   önce bak — ⚠ varsa koşu yinelemeden sonuç bildirme.
1. **CSV**: `python3 tools/ucus_analiz.py logs/gps_guidance_<yeni>.csv` —
   ve kıyas gerekiyorsa AYNI komutu eski uçuşun CSV'sine de uygula. Elle
   not edilmiş eski panel değerleriyle medyan kıyaslama YAPMA.
2. **Görüntü**: meta.csv'den temsilci kareler seç (en uzak an, en yakın an,
   faz ortaları) ve Read ile GÖZLE incele: hedef kadrajın neresinde,
   dedektör kutusu var mı.
3. **Çapraz kontrol**: panel mesafesi (meta.csv) ↔ CSV menzil_ham tutarlı mı;
   görüntüdeki konum ↔ v_px/kadraj_pitch_hata tutarlı mı. Çelişki varsa
   sonuçtan önce ÇELİŞKİYİ raporla.

## 5 · Çıktılar

- `ffmpeg -framerate 5 -i <dir>/frames/f%04d.jpg -c:v libx264 -pix_fmt yuv420p
  logs/ucus_<tarih>_kamera.mp4` — kullanıcı izleyebilsin.
- Rapor: tablo (ölçüt | eski | yeni), kanıt kareleri, taban tabloyla kıyas.
- Sonucu `TODO.md`'deki ilgili maddenin altına işle (bu dalda `UYGULANACAK.md` yok).
- Commit için sor; push için AYRICA sor (kullanıcı kuralı: kafana göre push yok).

## Kurallar

- Her koşuda TEK değişken; A/B gerekiyorsa kill-switch env ile aynı oturumda.
- Süreçleri kapatmadan önce kullanıcıya sor — kendi uçuşunu yapmak isteyebilir.
