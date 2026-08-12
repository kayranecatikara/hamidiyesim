# AGENTS.md

## 1. Projenin amacı

Bu depo, Gazebo Harmonic içinde gerçek ArduPilot SITL kullanan Teknofest Avcı
İHA hava-hava müdahale simülatörüdür. Iris avcı, Mini Talon hedefi izler ve
fiziksel temas kurmaya çalışır. Uçuş/güdüm kodu güvenlik-kritiktir; ölçüm
kanıtı ve kullanıcı onayı olmadan davranış değiştirilmez.

## 2. İlk okunması gereken dosyalar

Her işe başlamadan önce:

1. `CLAUDE.md` dosyasını baştan sona oku; deney, uçuş, ölçüm ve Git
   kurallarının ayrıntılı ve bağlayıcı kaynağıdır.
2. `UYGULANACAK.md` dosyasının en yeni tarihli bölümlerini oku. Dosya ters
   kronoloji içerir; alttaki eski hükümler güncel kararı geçersiz kılamaz.
3. Çalıştırma için `docs/SIMULASYON_CALISTIRMA.md`, mimari için
   `README.md` ve `docs/GUIDANCE.md` dosyalarını kodla çapraz kontrol et.
4. `TODO.md`, `DEVAM.md` ve üretilmiş `dokumantasyon/` çıktılarının bazı
   kısımları eskidir; güncel kod ve yeni tarihli kararlar üstündür.
5. Her zaman `git status --short --branch` ve ilgili diff'i incele. Kullanıcıya
   ait mevcut/commit edilmemiş değişiklikleri koru.

## 3. Repo dizin yapısı

- `control/`: FastAPI GCS, MAVLink, senaryolar ve güdüm yasaları.
- `control/guidance/`: GPS, supervisor, bbox-IBVS, FRPN ve kurtarma mantığı.
- `control/gcs_ui/`: web paneli.
- `vision/`: YOLO tespit/pose, HybridSORT kilidi, geometri ve paylaşılan durum.
- `sim/gazebo_harmonic/`: dünya, Iris/Talon modelleri ve ArduPilot köprüleri.
- `scripts/`: ana simülasyon ve yardımcı başlatıcılar.
- `tools/`: uçuş kaydı, sağlık bekçisi, kaçamak ve ölçüm/analiz araçları.
- `tests/`: sentetik birim/regresyon testleri.
- `logs/`: yerel uçuş kanıtları; Git dışıdır ve korunur.

## 4. Ana mimari

- Ana sim başlatıcı `scripts/start_harmonic.sh`; web/orkestrasyon giriş noktası
  `python3 -m control.gcs_server` komutudur.
- Gazebo, ArduCopter Iris'i FDM UDP 9002/MAVLink 14541; ArduPlane Talon'u
  FDM UDP 9012/MAVLink 14542 üzerinden besler. GCS yayını 14550, Mission
  Planner 14551'dir.
- Görüntü hattı: Gazebo kamera → YOLO → HybridSORT/TargetLock →
  `vision.detection_state` → telemetri/overlay ve isteğe bağlı bbox-IBVS.
- Güncel kod varsayılanı `AVCI_GORSEL=off`, yani saf GPS “istasyon” güdümüdür.
  Dokümanlardaki “hybrid varsayılan” anlatımına güvenme. Hybrid için açıkça
  `AVCI_GORSEL=on` gerekir.
- Hybrid devirde hedef GPS hızı yalnız bir kez dondurulmuş sayı olarak aktarılır.
  Görsel temas sırasında bbox yasasına canlı hedef GPS'i verilemez.
- `AVCI_POSE_KAYNAK` varsayılanı `gercek` simülasyon gerçeğidir; ölçüm
  kolaylığıdır, gerçek sisteme taşınabilir bir algı girdisi değildir.
- T1a roll telafisi varsayılan açıktır. T1b `AVCI_IBVS_DIKEY_ROLL` varsayılan
  kapalıdır ve mekanizma analizinde güncel soruna çözüm olmadığı görülmüştür.

## 5. Projeyi çalıştırma komutları

Kullanıcının 2026-08-12 tarihli açık talimatı: simülasyon
`scripts/start_harmonic.sh` ile başlatılmaz. Gazebo, iki SITL, GCS ve Mission
Planner aşağıdaki beş ayrı terminalde elle çalıştırılır; böylece her bileşenin
çıktısı doğrudan görünür. Bundan sonraki kullanıcı ve yapay zekâ testlerinde
bu komutlar ve bu sıra kullanılır. Eski dokümanlardaki launcher akışı bu
tercihin önüne geçmez.

### Temizlik — ayrı ve boş bir interaktif terminal

Bu blok çalışan bileşenlerin bulunduğu terminalde değil, ayrı boş terminalde
çalıştırılır:

```bash
pkill -TERM -f 'gz sim|gz-sim-server|gz-sim-gui|sim_vehicle|mavproxy|arducopter|arduplane|model JSON|control.gcs_server|run_plane_scenario'
sleep 3
pkill -KILL -f 'gz sim|gz-sim-server|gz-sim-gui|sim_vehicle|mavproxy|arducopter|arduplane|model JSON|control.gcs_server|run_plane_scenario'
sleep 1
pgrep -af 'gz sim|gz-sim-server|sim_vehicle|arducopter|arduplane|control.gcs_server' || echo "TEMİZ ✓"
```

### Terminal 1 — Gazebo Harmonic

İlk açılır; diğer terminallere geçmeden yaklaşık 15 saniye beklenir:

```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export DISPLAY=:1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build"
export GZ_SIM_RESOURCE_PATH="$HOME/projects/avci_sim/sim/gazebo_harmonic/models"
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$HOME/ardupilot_gazebo/models"
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$HOME/ardupilot_gazebo/worlds"
gz sim -r -v2 sim/gazebo_harmonic/worlds/avci_harmonic.sdf
```

### Terminal 2 — ArduCopter (avcı Iris, FDM 9002)

```bash
cd ~/ardupilot
APT=$HOME/ardupilot/Tools/autotest
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  -I0 --sysid 5 --no-rebuild -w \
  --add-param-file=$APT/default_params/copter.parm \
  --add-param-file=$APT/default_params/gazebo-iris.parm \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_copter.parm \
  --out udp:127.0.0.1:14541 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
  --mavproxy-args="--streamrate=25"
```

### Terminal 3 — ArduPlane (hedef Talon, gerçek Gazebo uçuşu, FDM 9012)

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1:9012 \
  -I1 --sysid 2 --no-rebuild \
  --add-param-file=$HOME/projects/avci_sim/sim/ardupilot_params/avci_plane.parm \
  --out udp:127.0.0.1:14542 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
  --mavproxy-args="--streamrate=25"
```

### Terminal 4 — GCS Server (kamera + web + senaryolar)

```bash
cd ~/projects/avci_sim
source /opt/ros/humble/setup.bash
export AVCI_GZ_CAMERA=1
export AVCI_GORSEL=on
fuser -k 8000/tcp
python3 -m control.gcs_server
```

### Terminal 5 — Mission Planner

```bash
cd ~/projects/avci_sim/tools/mission_planner
export LD_LIBRARY_PATH="$HOME/projects/avci_sim/tools/mission_planner/native_libs:$LD_LIBRARY_PATH"
mono MissionPlanner.exe
```

Terminaller yukarıdaki sırayla açılır. Otomatik testte de görünür çıktı
korunur; bileşenler gizli bir launcher içinde başlatılmaz.

## 6. Test ve doğrulama komutları

Bayt kodu/cache yazmadan kısa testler:

- `PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_bbox_ibvs`
- `PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_visual_lead`
- `PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_gps_guidance`
- `PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_frpn`
- `PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_frpn_guidance`
- `PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_hedef_kestirim`
- `PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_kurtarma`
- `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider
  tests/test_sim_truth_temas.py tests/test_tracker.py`

Devir anında `test_frpn_guidance` D2 durum-sözleşmesi kontrolü nedeniyle
14/15 ve çıkış 1 vermektedir: FRPN statüsünde `tgt_vx/tgt_vy/tgt_vz` eksik.
FRPN opsiyonel ve uçuşta doğrulanmamıştır; bu açık hata çözülmeden tam test
paketi yeşil denemez. Tracker bağımlılığı yoksa test skip edebilir; skip,
başarı kanıtı değildir.

Güdüm davranışı için birim test yeterli değildir. `CLAUDE.md` uyarınca taze
A/B uçuş, video, 20 Hz log, kara kutu/gerçek geometri ve görsel inceleme gerekir.

## 7. Bağlayıcı test ve deney protokolü

Aşağıdaki protokol `CLAUDE.md` içindeki test ve deney hükümlerinden
**özetlenmeden ve içerik çıkarılmadan** doğrudan aktarılmıştır. Yalnız başlık
düzeyleri `AGENTS.md` yapısına uyarlanmıştır. Bundan sonraki bütün güdüm,
uçuş, A/B, mekanizma ve kabul testleri bu bölümün tamamına göre yapılır.

### 1 · GELİŞTİRME STRATEJİSİ — döngünün tamamı

Her güdüm özelliği şu beş adımdan geçer. Adım atlanmaz.

**1. YAPAY ZEKÂ ÖNERİR.** Sisteme eklenmesi gereken özelliği, *neden*
gerektiğini ve *hangi ölçümün* onu gösterdiğini söyler. Riskleri ve geri
dönüş yolunu (kill-switch) birlikte sunar. Kullanıcı onaylamadan güdüm
davranışını değiştiren kod yazılmaz.

**2. YAPAY ZEKÂ TEST EDER — EN AZ 4 UÇUŞ.** Bölüm 2'deki test mekanizmasıyla,
bölüm 3'teki senaryo tasarımıyla. Kontrol/deney kolları dönüşümlü koşulur.
Kanıt: taze uçuş + video + log, üçü birden.

**3. YAPAY ZEKÂ RAPOR EDER.** Özellik sistemi iyileştirdi mi, kötüleştirdi mi,
yoksa nötr mü — sayılarla ve kanıt kareleriyle. İyileştirdiyse girer,
kötüleştirdiyse ÇIKAR, nötrse karar kullanıcıya bırakılır. Yapay zekâ kendi
önceki hükmünü çürüten veri bulursa bunu açıkça söyler.

**4. YAPAY ZEKÂ "SEN DE GÖZÜNLE GÖR" DER.** Kullanıcının kendi makinesinde
aynı şeyi uçurabilmesi için **tüm çalıştırma komutlarını sırasıyla** verir
(5 terminal + panel adımları + hangi env değişkeniyle hangi kolun açıldığı).
Neye bakması gerektiğini de söyler: "şunu görürsen benim ölçtüğümle aynı şeyi
görüyorsun demektir."

**5. İNSAN DOĞRULAR.** Kullanıcı uçurur. Yapay zekânın göremediği bir şey
görürse söyler, birlikte yeniden mütalaa edilir.

**Bu yapının amacı:** analiz yükünü insandan alıp yapay zekâya vermek —
insanın analiz kapasitesi limiti geliştirmeyi kısıtlamasın; ama her özelliği
insanın da gözüyle görmesi, yapay zekânın kaçırdığı bir şeye karşı ekstra
güvenlik katmanı olarak kalsın.

---

### 2 · TEST MEKANİZMASI — değişmez sekiz adım

1. **TAZE UÇUŞ.** Testi yapay zekâ koşar. Kullanıcı gözlemci değil,
   doğrulayıcıdır (adım 5).
2. **SANİYEDE 1 KARE + TELEMETRİ.** `python3 tools/ucus_kaydi.py <dizin> <süre>`
   — her kare o anki panel telemetrisiyle eşli olarak `meta.csv`'ye yazılır.
3. **KARELERİ BİRLEŞTİR → VİDEO.**
   `ffmpeg -framerate 5 -i <dizin>/frames/f%04d.jpg -c:v libx264 -pix_fmt yuv420p logs/<ad>.mp4`
4. **VİDEOYU GÖZLE ANALİZ ET.** Kareleri `Read` ile aç ve BAK. İki üç kareye
   bakıp geçmek yeterli DEĞİLDİR: her yaklaşma olayının **giriş → en yakın →
   çıkış sıralı dizisi** incelenir. Sorulacaklar: hedef kadrajın neresinde,
   kutu var mı ve güveni ne, ufuk ne kadar yatık (= araç ne kadar yatıkta),
   hedef büyüyor mu (yaklaşıyoruz) yoksa aynı boyda mı kalıyor (gölge
   ediyoruz), temas anında hedefi nereden görüyoruz.
5. **LOGLARI ANALİZ ET.** `logs/bbox_ibvs_*.csv` (20 Hz) ve
   `logs/gps_guidance_*.csv`. Kıyas her zaman aynı segmentasyonla.
6. **İKİSİNİ ÇAPRAZLA — ZORUNLU.** Görüntü ile log birbirini doğrulamalı.
   Çelişki varsa sonuçtan ÖNCE çelişki raporlanır ve önce o çözülür.
   *(Bu adım gerçekten iş görüyor: panelin "4.8 m" dediği karede hedef
   kadrajda 20 px'ti — 4.8 m'de 45 px olmalıydı. Panel `mesafe` 1 Hz ve
   buluşmadaki kapanma 13-22 m/s olduğu için örtüşme yapıyordu. Video
   olmasaydı bozuk ölçütle karar verilecekti.)*
7. **KARAR.** İyileştirdiyse girer, kötüleştirdiyse çıkar, nötrse kullanıcıya.
8. **RAPOR + ÇALIŞTIRMA KOMUTLARI.** Bölüm 1 adım 4.

#### Kanıt sayılMAYAN şeyler

- ⛔ **Eski uçuş loglarını yeniden oynatmak.** Çevrimdışı replay yalnız
  HİPOTEZ üretir. Kabul kararını sadece taze uçuş + video verir.
- ⛔ **Yalnız CSV istatistiği.** Video bacağı olmadan rapor yazılmaz.
- ⛔ **Elle okunmuş panel değeri.** Medyanla kıyasla.
- ⛔ **Tek koşu, hatta iki koşu.** Koşular arası değişkenlik kol farkını
  yutabiliyor (ölçüldü: aynı kolda tepe kutu boyutu 76.5 px ve 22.7 px).
  **En az 4 uçuş**, tercihen kol başına 3.

---

### 3 · SENARYO TASARIMI — testi ölçtüğün şeye göre kur

#### 3.1 · Ana kural

**Eklediğin özellik neyle ilgiliyse, testi ÖZELLİKLE o kısmı ölçecek şekilde
kur.** Genel amaçlı bir uçuş koşup içinden anlam çıkarmaya çalışma.

#### 3.2 · ⛔ Hedefe sürekli tam daire çizdirme

Varsayılan test senaryosu daire DEĞİLDİR. Neden (yaşandı):

- Drone dairenin içine hiç giremiyor; hedefe ilk anda ters açıdan yaklaşıyor,
  hedef kaçıyor, drone tekrar yaklaşmaya çalışıyor, aynı döngü tekrarlıyor.
- Ölçülen buluşma kapanma hızı medyan 4.9-12.4 / p90 13-22 m/s. Saf kuyruk
  takibinde bu en fazla 3 m/s olurdu (18 − 14.9) — yani buluşmalar **kafa
  kafaya**. İsabet zarfının içinde ~0.05 s kalınıyor.
- Sonuç: her koşu birbirinden çok farklı çıkıyor, hiçbir şey anlaşılmıyor,
  ölçüm gürültüye boğuluyor.

Daire yalnız "sürekli dönen hedefte ne oluyor" sorusunun kendisi
soruluyorsa kullanılır — o zaman da tek başına değil, aşağıdaki testlerin
yanında.

#### 3.3 · ✅ VARSAYILAN: düz uçuş + buluşma anında TETİKLENMİŞ KAÇAMAK

Manevra tepkisi böyle ölçülür — tekrarlanabilir, izole, hızlı:

1. Hedef **`duz`** senaryosunda uçar, hıza/irtifaya oturur.
2. Avcı drone takibe başlar ve temiz bir kuyruk yaklaşması kurar.
3. Bir tetikleyici, iki aracın **anlık GPS'ini** izler (`/api/debug/telem`).
4. Mesafe eşiğe inince (tipik **25 m**, yani "tam vuracakken") hedef
   **manuel RC'ye** devralınır ve belirli bir kaçamak uygulanır.
5. Avcı dronun tepkisi ölçülür: tepki gecikmesi, görsel temas korundu mu,
   ıska mesafesi, isabet.

Araç: `tools/kacamak_testi.py`. Kaçamak çeşitleri (hepsi denenmeli):

| ad | RC | ne sınar |
|---|---|---|
| `yatay` | aileron 2000 | ani yanal kırılma — yatay kanal, roll telafisi |
| `dikey_yukari` | elevator 1150 | tırmanış — dikey bütçe, kamera üst görüşü |
| `dikey_asagi` | elevator 1850 | dalış — kamera −30° alt sınırı |
| `capraz` | aileron 1950 + elevator 1250 | birleşik — kanallar arası etkileşim |
| `hizlan` | throttle 2000 | kapanma hızının kaybı |
| `yok` | (kaçamak yok) | TABAN koşusu — kıyas çizgisi |

**`yok` kolu her kampanyada koşulur.** Kaçamaksız isabet oranını bilmeden
kaçamaklı sonuç yorumlanamaz.

#### 3.4 · Diğer senaryolar

`square` (basamak dönüşler), `aggressive` (rastgele), `circle_xl/l/m/s/xs`
(farklı çaplar) mevcut. Bunlar ana testi tamamlar, yerine geçmez.

---

### 4 · DENEY DİSİPLİNİ

- **TEK DEĞİŞKEN.** Her koşuda bir şey değişir. A/B kill-switch env ile.
- **DÖNÜŞÜMLÜ A/B.** K, M, K, M, K, M — sim kayması iki kolu eşit etkilesin.
  Bir kolun 3'ünü arka arkaya koşup diğerine geçme.
- **ÖLÇÜTLER VE KARAR KURALI KOŞMADAN ÖNCE YAZILIR.** Birincil/ikincil ayrımı
  ve "hangi kol kaç ölçüt kazanırsa ne olur" önceden ilan edilir. Sonuca
  bakıp ölçüt seçmek yasak.

- ⛔ **SALINIM ÖLÇÜLMEDEN "İYİLEŞTİ" DENMEZ** (kullanıcı kuralı 2026-08-10).
  Yalnız "isabet + en yakın menzil"e bakan bir ölçüt, dengesizce savrulup
  şans eseri çarpan bir aracı ÖDÜLLENDİRİR. Salınan araç kötüdür — çok
  yaklaşma üretse bile. Her karşılaştırmada şunlar da raporlanır:
    * `cx` işaret değişimi / s (hedef kadrajda sağa-sola atıyor mu)
    * yatış işaret değişimi / s ve |yatış| p90
    * yaw komutu değişim hızı (°/s) — doyuma gidiyor mu
    * görsel temas kesintisi sayısı ve süresi

- ⛔ **HER VURUŞ "KONTROLLÜ VURUŞ" DEĞİLDİR.** Temas anı ve ÖNCESİNDEKİ
  kareler tek tek incelenir ve vuruş sınıflandırılır:
    * **KONTROLLÜ**: son ~2 s boyunca hedef kadrajda kesintisiz, `cx`
      merkeze yakın ve sakin, kutu boyutu DÜZGÜN büyüyor, yatış sakin.
    * **ŞANS**: hedef kadraj kenarında ya da aralıklı kayboluyor, kutu
      boyutu sıçramalı, araç salınıyor, temas ani ve rastgele geliyor.
  Şans vuruşu isabet sayılır ama **iyileşme kanıtı sayılmaz**. Rapor
  ikisini ayrı verir. Araç `tools/vurus_kalitesi.py`.
- **GEÇERLİLİK.** Hedef 20-250 m irtifa / 6-25 m/s bandı dışına çıktıysa koşu
  SAYILMAZ. Uçuş boyunca `tools/ucus_bekci.py` çalışır.
- **ÖRTÜŞMEYE DİKKAT.** Panel `mesafe` 1 Hz'tir; kapanma hızlıyken gerçek en
  yakın anı ıskalar. Yakınlık ölçütü 20 Hz bbox logundan (kutu boyutundan)
  alınır.
- **ARŞİVLE.** Her koşunun log dosyaları kendi dizinine kopyalanır. Zaman
  damgası tahmin ederek glob yapma.
- **KILL-SWITCH.** Her davranış değişikliğinin `AVCI_*` env anahtarı olur;
  varsayılanını ölçüm belirler.

---

### 5 · ÖLÇÜM HATALARINA KARŞI MEKANİZMALAR — zorunlu kontrol listesi

Bu bölüm 2026-08-10/11 oturumunda YAPILAN GERÇEK HATALARDAN üretildi.
Her madde bir hataya karşılık gelir. Yeni bir özellik ölçülürken bu liste
baştan sona uygulanır; atlanan madde varsa sonuç RAPOR EDİLMEZ.

#### 5.1 · MEKANİZMA KAPISI — "özellik gerçekten çalıştı mı?"

**Kolları kıyaslamadan ÖNCE, özelliğin devreye girdiği KANITLANIR.**
Her özellik, ne kadar iş yaptığını gösteren bir sütun loglar
(`kacis_ek`, `eps_hiz_deg`, `sonum_deg` gibi). Deney kolunda bu sütun
sıfırsa o koşu **veri noktası değil, GEÇERSİZ koşudur**.

*Yaşandı:* Ö6 (ANGLE_MAX 45→55°) "isabet 0/4 → 2/4" diye raporlandı; sonra
loglara bakınca deney kolunun 2 koşusunda araç **38-40°'de kalmış**, yani
45° tavanına bile dayanmamıştı. O koşular fiilen kontrol koşusuydu ve tablo
sahteydi. Ö1 de TERMINAL durumunda inert olduğu için kullanıcının uçuşunda
hiç çalışmamıştı — "fark göremedim" demesinin sebebi buydu.

#### 5.2 · ÖLÇÜT GEÇERLİLİK EŞİ — "bu sayı KÖTÜ bir sebeple de düşer mi?"

Her ölçüt için şu soru sorulur ve cevabı yazılır: **"bu değer, sistem
KÖTÜLEŞTİĞİNDE de iyi görünebilir mi?"** Cevap evetse, ölçüt tek başına
raporlanamaz; yanında **geçerlilik eşi** zorunludur.

| ölçüt | kötü sebeple iyileşir mi | zorunlu eşi |
|---|---|---|
| salınım (cx işaret değişimi) | **EVET** — hedef kadrajdan çıkarsa ölçülemez, 0 görünür | görsel temas oranı (%60 altı → GÜVENİLMEZ) |
| en yakın menzil | evet — savrulup şans eseri yaklaşma | vuruş sınıfı (KONTROLLÜ/ŞANS) |
| isabet | evet — dengesiz araç şans eseri çarpar | salınım + vuruş sınıfı |
| temas süresi | evet — uzakta durup hiç yaklaşmamak | kutu boyutu / yaklaşma |

*Yaşandı:* "Ö8 salınımı 0.073 → 0.000 yaptı" diye rapor edildi. Salınım
YALNIZ kutu olan karelerde sayılıyordu; hedefi daha çok kaybeden koşu daha
sakin görünüyordu. Ölçüt, hedefi kaybetmeyi ödüllendiriyordu. Kullanıcı
"ben fark görmedim" deyince yakalandı.

#### 5.3 · ÖRNEKLEME HIZI KURALI

Bir ölçütün örnekleme hızı, ölçtüğü şeyin değişim hızının **en az 5 katı**
olmalıdır. Değilse o kaynak kullanılmaz; daha hızlı kaynağa geçilir.

*Yaşandı:* "en yakın menzil" panel telemetrisinden (1 Hz) alınıyordu, ama
buluşmadaki kapanma hızı 13-22 m/s'ydi. Panelin "4.8 m" dediği karede hedef
kadrajda 20 px'ti (4.8 m'de 45 px olmalıydı). Ölçüt 15 m'ye kadar
yanılıyordu. 20 Hz bbox logundan (kutu boyutundan) yeniden hesaplandı.

#### 5.4 · EN AZ n=4/KOL — ve altındaki her şey "ARA VERİ"

n < 4 iken **hüküm cümlesi kurulmaz**. Bu aşamadaki sayılar
"ara veri, karar değil" diye sunulur.

*Yaşandı, ÜÇ KEZ:*
- M3: n=2'de "açık 2/2, kapalı 0/2" → n=6'da eridi, nötr çıktı.
- Ö6: n=2'de "0/2 → 2/2" kazanım diye sunuldu → n=4'te mekanizmanın hiç
  çalışmadığı ortaya çıktı.
- Ö8: n=3'te "medyan 1.88 vs 2.40, Ö8 önde" → n=6'da **TERSİNE döndü**
  (2.76 vs 1.96).

⚠ Koşular arası değişkenlik gerçek: aynı kolda tepe kutu boyutu 76.5 px ve
22.7 px ölçüldü (3 kat). Tek koşunun sonucu, kolun kendi dağılımından
çekilmiş rastgele bir sayıdır.

#### 5.5 · ÖLÇÜT KULLANICININ HEDEFİNDEN TÜRETİLİR

Birincil ölçüt seçilirken kullanıcının isteği **birebir alıntılanır** ve
ölçütün o cümleyi ölçtüğü gösterilir. Hesaplaması kolay olan değil, hedefi
temsil eden ölçüt seçilir.

*Yaşandı:* "maks açılan mesafe" birincil ölçüt yapıldı ve ÜÇ testte
üst üste kolları ayıramadı — savrulma kaçamağın geometrisinden geliyordu,
iyileşmeyle ilgisi zayıftı. Dahası kullanıcı açıkça *"biraz mesafe açılsa
ama salınım olmasa okeydir"* demişti; seçtiğim ölçüt tam da kabul ettiği
şeyi cezalandırıyordu.

#### 5.6 · KENDİ LEHİNE YORUM YASAĞI

Sonuç bölünmüş çıkarsa ölçüt DEĞİŞTİRİLMEZ, kullanıcıya götürülür.
Sınıflandırıcı bir sonucu kendi önerdiğin özelliğin lehine çeviriyorsa,
önce sınıflandırıcının doğruluğu sorgulanır.

*Yaşandı (doğru yapıldı, örnek olsun):* kontrol kolundaki tek isabet
`vurus_kalitesi` tarafından ŞANS sayıldı; ama altı ölçütten BEŞİNİ geçmişti
ve tek takıldığı 1 kopuk kareydi. Eşik fazla katıydı — Ö8 lehine sayıya
çevrilmedi, eşik hatası olarak raporlandı.

#### 5.7 · VERİ DAYANIKLILIĞI

Koşu çıktıları **`logs/` altına** yazılır. `/tmp` gecelik temizleniyor.

*Yaşandı, İKİ KEZ:* 12 uçuşluk kampanyanın kare/olay dosyaları ve daha
önce scratchpad'deki tüm scriptler silindi. Özet sayılar `UYGULANACAK.md`'de
olduğu için sonuçlar kurtarıldı, ham veri kurtarılamadı.

#### 5.8 · RAPORDAN ÖNCE ÜÇ SORU

Her rapor öncesi yazılı olarak cevaplanır:

1. **Özellik çalıştı mı?** (5.1 — mekanizma sütunu sıfır değil mi)
2. **Ölçütüm kötü bir sebeple mi iyileşti?** (5.2 — geçerlilik eşi ne diyor)
3. **n kaç, ve bu n'de hüküm kurulur mu?** (5.4)

Üçünden birine cevap verilemiyorsa rapor değil, **eksik listesi** sunulur.

---

### 6 · PANELDE CANLI AÇ/KAPA — her yeni özellik için ZORUNLU

**Sisteme eklenen her davranış anahtarının panelde bir aç/kapa düğmesi olur.**
Kullanıcı kuralı (2026-08-10): "her defasında tüm sistemi baştan farklı
ayarlarla çalıştırmak" kabul edilemez — hem zahmetli, hem de farkı anlık
gözlemeyi imkânsız kılıyor.

**Nasıl çalışıyor:** `bbox_ibvs.Cfg` bir SINIF ve güdüm döngüsü her karede
`cfg.<ALAN>` okuyor. Sunucu sınıf niteliğini değiştirince bir sonraki kareden
itibaren geçerli olur — **uçuş sırasında, yeniden başlatmadan.**

**Yeni özellik eklerken yapılacaklar (atlanmaz):**

1. `Cfg`'ye alanı ve `AVCI_*` env varsayılanını ekle (kill-switch).
2. `control/gcs_server.py` içindeki **`_OZELLIKLER`** sözlüğüne bir satır ekle:
   `"ad": ("ALAN", "bool"|"kazanc"|"param", "Etiket", "Açıklama", "anahtar", açık_değeri)`
   - `"param"` tipi ARAÇ parametresidir (ör. `ANGLE_MAX`); uçuş sırasında
     MAVLink `PARAM_SET` ile yazılır, geri okuma `_param_cache`'ten teyit
     edilir. Kod değişikliği gerektirmeyen deneyler böyle yapılır.
3. Başka hiçbir şey gerekmez — panel listeyi `/api/gudum_ozellikleri`'nden
   çeker, arayüz kendiliğinden büyür (HTML/CSS/JS'e dokunma).
4. Raporda kullanıcıya "panelden şu düğmeyi açıp kapatarak farkı gör" de.

**⚠ PANELDE YALNIZ O ANKİ ADIMIN ÖZELLİĞİ DURUR.** Bir özelliğin kararı
verilince — ister sisteme **GİRSİN** ister **ELENSİN** — düğmesi
`_OZELLIKLER`'den **SİLİNİR**. Yeni adıma geçerken önceki adımın düğmesi
temizlenir; panelde her zaman sadece o an denenen şey görünür.

*Neden:* düğmeler birikince panel çöplüğe döner ve kullanıcı o adımda neyi
sınadığını göremez. Silmek özelliği yok etmez — kill-switch env anahtarı
kodda kalır, karar ve ölçümler `UYGULANACAK.md`'de durur, silinen özellikler
`_OZELLIKLER` üstündeki yorum bloğunda env anahtarlarıyla listelenir.

**Sonuç:** çalıştırma komutları ARTIK DEĞİŞMİYOR. Kol seçimi env ile değil,
panelden yapılır. Env anahtarları yine duruyor (otomatik kampanyalar ve
başlangıç varsayılanı için), ama insan testinde gerekmez.

⚠ Otomatik A/B kampanyalarında hâlâ env + tam restart kullan: koşu boyunca
anahtarın değişmediğinden emin olmak deney disiplininin parçası (§4).

---

### 7 · YAPAY ZEKÂNIN ÇALIŞMA BİÇİMİ

- **ARKA PLANDA GİZLİ SHELL YOK.** Uçuşlar ve analizler sohbette, doğrudan
  çalıştırılır ki kullanıcı ne olup bittiğini görsün. Uzun süren bir işi arka
  plana atıp sessiz kalma; boşa zaman harcama.
- Bir şey takılırsa **sebebini bul ve söyle**, sessizce yeniden deneme.
- Kendi önceki hükmünü çürüten veri çıkarsa **açıkça düzelt**.
- Sonuçlar `UYGULANACAK.md`'ye ilgili maddenin altına işlenir.

---

### 8 · DEPO KURALLARI

- **Kafana göre push YOK.** Commit için sor, push için AYRICA sor.
- Merge ile gelen davranış değişikliklerini işaretle; uçuş-kritik yolda
  önce sor.
- Güdüm davranışını değiştiren düzenleme, uygulanmadan önce **riskleriyle
  birlikte** sunulur.

---

### 9 · SİM ÇALIŞTIRMA TUZAKLARI (hepsi yaşandı)

- `pkill -f` **kendi kabuğunu öldürür** (exit 144). Deseni köşeli parantezle
  kır (`gz [s]im`) ya da komutu ayrı script dosyasına al.
- Sim kuran scripti **boruya bağlama** (`| tail`, `| grep`). Arka plandaki sim
  süreçleri yüzünden boru EOF almaz, script orada asılı kalır ve uçuş komutu
  hiç gönderilmez. Çıktıyı dosyaya yaz, sonra dosyayı oku.
- MAVProxy TTY'siz çıkar → `script -qfec "..." /dev/null` ile sarmala.
- **gcs_server'ı tek başına yeniden başlatma** (Gazebo+SITL ayaktayken):
  chase/scenario bağlantısı kopar. Env değişikliği için TAM restart.
- `GZ_SIM_RESOURCE_PATH` tek satırda uzunsa terminalde kırılıyor; kısa
  parçalar hâlinde ekleyerek ver.
- Test bitince araçları havada KONTROLSÜZ BIRAKMA — simi komple kapat.
- `/tmp` temizlenebilir (gecelik). Kritik veriyi `logs/` altına da arşivle.
- `tools/ucus_bekci.py` uçuştan uzun süre koşarsa, sim kapatılınca
  "API cevapsız" ihlali basar — bu uçuş SONRASI artefakttır, koşuyu geçersiz
  kılmaz. Uçuş içi bandı `meta.csv`'den ayrıca doğrula.

---

### 10 · YARIŞMA KISITI (üstün kural)

Görsel temas varken **GPS güdümü yasak** — yalnız bbox. Temas kesilince GPS
serbest. Görsel döngü hedefe dair veriyi devirde bir kez SAYI olarak alır;
canlı GPS erişimi yapısal olarak yoktur (`tests/test_bbox_ibvs.py` B5 bekçisi
bunu sınar).

---

Ayrıntılı arka plan: `docs/OTONOM_UCUS_TESTI.md`,
`.claude/skills/ucus-testi/SKILL.md`, sonuç kaydı `UYGULANACAK.md`.

## 8. Kodlama ve değişiklik kuralları

- Tek değişkeni değiştir; önce hipotez, birincil/ikincil ölçüt, karar eşiği,
  risk ve geri dönüş yolunu yaz.
- Her davranış özelliğine varsayılanı koruyan `AVCI_*` kill-switch ekle.
- Özelliğin gerçekten devreye girdiğini ayrı bir log sütunuyla kanıtla.
- İnsan testinde yalnız güncel deney panelde görünür. Kararı verilen düğmeyi
  `_OZELLIKLER` listesinden kaldır; env kill-switch kodda kalabilir.
- Otomatik A/B'de kolları dönüşümlü koş; n<4/kol ile hüküm kurma. Kullanıcının
  daha yüksek örnek talebi varsa onu uygula.
- bbox/cx tek başına gerçek salınım ölçütü değildir. Hedef çerçevesindeki gerçek
  yanal hareketi, dikey ıska, temas sürekliliği, yatış/yaw ve kontrollü/şans
  vuruş ayrımını birlikte raporla.
- İlgisiz refactor, formatlama veya dependency değişikliği yapma.

## 9. Kullanıcı onayı gerektiren işlemler

Önceden açık onay al:

- Güdüm, uçuş, arm/disarm, fiziksel temas/hasar veya güvenlik davranışı değişimi.
- SITL/Gazebo uçuş kampanyası ve gerçek donanıma komut.
- Yeni dependency, sistem paketi, model eğitimi/ağırlık değişimi.
- Commit, tag, branch/remote işlemi. Commit izni push izni değildir; push için
  ayrıca sor.
- Merge/rebase/pull, dal değiştirme veya mevcut yerel işi etkileyen işlem.

## 10. Kritik davranış ve güvenlik sınırları

- Görsel temas varken hedef GPS güdümü yasaktır; yalnız bbox kullanılabilir.
- Sim-truth temas/poz sonucu ölçebilir, hedef konumu olarak görsel yasayı
  besleyemez.
- Uçuş boyunca `tools/ucus_bekci.py` ile irtifa/hız/bağlantı geçerliliğini
  izle. Geçersiz koşuyu veri noktası sayma.
- Araçları test sonunda havada veya kontrolsüz bırakma; kontrollü tam kapanış
  yap.
- FRPN uçuşta doğrulanmamıştır ve koruma/supervisor entegrasyonu risklidir.
- Web GCS `0.0.0.0` üzerinde kimlik doğrulamasız kontrol API'si açar; güvenilmeyen
  ağda çalıştırma.

## 11. Git, commit ve push kuralları

- Yerel ağaç uzak repodan daha değerli/güncel olabilir. Önce status, diff,
  untracked ve upstream durumunu incele.
- Kullanıcı onayı olmadan commit, tag veya push yapma; force push yapma.
- `git reset`, `git restore`, `git checkout --`, `git clean` ve `git stash`
  ile kullanıcı işini geri alma/silme.
- Açık onay olmadan pull/merge/rebase, branch değiştirme veya remote düzenleme.
- `CLAUDE_SESSION_EXPORT.txt` ile `.codex-handoff/` yalnız yereldir; asla
  commit/push etme.

## 12. Dokümantasyon kuralları

- Deney sonucu ve kararları tarih/kanıt/testlerle `UYGULANACAK.md` dosyasının
  en yeni bölümüne işle.
- Kod, test ve doküman çelişirse sessizce birini seçme; çelişkiyi raporla ve
  güncel gerçek davranışı kod/test ile doğrula.
- Komut, port, varsayılan veya log şeması değişirse ilgili elle yazılmış
  dokümanı güncelle. Üretilmiş siteyi otorite sayma.
- Başarı, uçuş veya test sonucu uydurma; gerçek çıktı yoksa “doğrulanmadı” de.

## 13. Bir işin tamamlanmış kabul edilme şartları

Bir davranış değişikliği ancak şu şartlarla tamamdır:

1. Kod ve geri dönüş anahtarı uygulanmış; mekanizma logda gerçekten çalışmış.
2. İlgili kısa testler geçmiştir; skip/eksik dependency açıkça belirtilmiştir.
3. Önceden tanımlı ölçütlerle yeterli, dönüşümlü taze A/B uçuş yapılmıştır.
4. 1 fps kareler, video, 20 Hz güdüm logları ve kara kutu/gerçek geometri
   çapraz incelenmiştir.
5. Kontrollü ve şans vuruşları ayrılmış; salınım/güvenlik gerilememiştir.
6. Kullanıcıya komutlar, dosyalar, diff, riskler ve kanıt sunulmuş; insan
   görsel doğrulaması beklenmiştir.
7. Dokümantasyon güncel, sim güvenle kapalı ve Git durumu raporlanmıştır.

## 14. Yasak veya tehlikeli komutlar

- `sudo` ve kullanıcı onaysız dependency/sistem değişikliği.
- Geniş/destrüktif `rm -rf` veya belirsiz süreç öldürme.
- `pkill -f` yalnız Bölüm 5'te kullanıcının verdiği tam temizlik bloğunda,
  ayrı boş interaktif terminalde kullanılabilir; farklı/geniş desen uydurma.
- Simülasyonu `start_harmonic.sh` ile başlatma; Bölüm 5'teki beş görünür
  terminal akışını kullan.
- Gazebo/SITL ayaktayken yalnız GCS'yi restart edip aracı sahipsiz bırakma.
- Kritik kanıtı yalnız `/tmp` altında tutma.
- İlk incelemede ağır eğitim, Gazebo/SITL uçuşu veya donanım komutu.

## 15. Claude'dan devralınan yerel değişikliklerin korunması

Claude'un commit edilmemiş dosyaları, ignored `logs/` verisi, ArduPilot
`.BIN` logları, dışa aktarım ve bağlı worktree kanıt sayılır. Bunları silme,
temizleme, ezme veya uzak sürümle değiştirme. Devir anındaki aktif dal
`kayramin_super_gudumu` ve HEAD `2b90498` idi; dal değiştirmeden önce mutlaka
yeniden doğrula ve kullanıcıdan izin al.
