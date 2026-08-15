# `hit_irtifa_tutucu` dalı — `kayramin_super_gudumu`'ne göre ne değişti

**Tarih:** 2026-08-15
**Dal:** `hit_irtifa_tutucu` (origin'e pushlu)
**Taban:** `kayramin_super_gudumu` @ `0fb8476` ile **eşit** — Kayra'nın son
commit'i (DAİRE TEŞHİSİ) dahil, hepsi birleştirildi.

> ## ⚠ ÖNCE ŞU: GÜDÜM KODUNA HİÇ DOKUNULMADI
> `bbox_ibvs.py`, `gps_guidance.py`, `kurtarma.py`, `frpn.py`,
> `guidance_core.py` — **tek satır değişmedi.** Avcının uçuş yolu bu dalda
> aynen `kayramin_super_gudumu`'ndeki gibi. Değişenler: hedef İHA'nın
> senaryosu, sunucunun kamera/panel/log uçları ve arayüz.

## Değişen dosyalar

| dosya | satır | ne |
|---|---:|---|
| `control/run_plane_scenario.py` | +116 | hedefin irtifa tutucusu |
| `control/senaryo_cfg.py` | +26 (yeni) | tutucunun canlı ayarı |
| `tests/test_irtifa_tutucu.py` | +164 (yeni) | 14 test, hepsi geçiyor |
| `control/gcs_server.py` | +146/−? | kamera onarımı, hibrit varsayılan, /panel, /api/senaryo_ayar |
| `sim/.../iris_with_standoffs/model.sdf` | +28 | dış görüş kamerası |
| `sim/.../mini_talon_vtail/model.sdf` | +28 | dış görüş kamerası |
| `control/gcs_ui/{index.html,script.js,style.css}` | — | arayüz `claude_kubra` sürümü + kaçamak paneli + irtifa düğmesi |

---

## 1 · Hedef İHA'nın irtifa tutucusu (YENİ ÖZELLİK)

### Sorun

Hedef uçak `duz` senaryosunda **durmadan tırmanıyordu.** Ölçüldü:
dakikada **+17.8 m** düz uçuşta, **+58.6 m** dairede. Hiç oturmuyor.

8 koşuluk dönüşümlü bir A/B sırasında hedef **22 m'den 224 m'ye** çıktı ve
CLAUDE.md §4'ün geçerlilik bandına (20-250 m) dayandığı için kampanyanın
ortasında simi sıfırlamak zorunda kaldık.

### Kök neden

`scenario_duz`, FBWA modunda `pitch=0` ve gaz = GCS slider gönderiyordu.

**FBWA bir tutum modudur, irtifa modu değildir.** `pitch=0` = "0° burun
açısı", "0° uçuş yolu açısı" DEĞİL. İtki seviye uçuşun sürüklemesinden
fazlaysa uçak burnu 0°'de dururken bile fazla enerjiyi irtifaya çevirir.
FBWA'da ne TECS ne irtifa kilidi var.

Dairede zaten elle bulunmuş bir `pitch = 150/cos(yatış)` bias'ı vardı (yük
faktörü telafisi) — ama o da açık çevrim ve yetersiz: dairede sürüklenme
düzden 3 kat fazla çıktı.

### Çözüm

Pitch komutuna PD'li kapalı çevrim irtifa düzeltmesi:

```
düzeltme = clamp(30·hata − 60·tırmanma, ±300)      # ±300 ≈ ±13.5°
pitch    = taban + düzeltme
```

- **Gaza DOKUNULMAZ** — slider kullanıcının, "hedefi yavaşlat" isteği çalışır.
- Düzeltme kırpılır, **taban kırpılmaz** → dairenin yatış payı kaybolmaz.
- Bağlandığı yerler: düz faz, bekleme turu, daire. `square`/`aggressive`
  dokunulmadı.
- Kazançlar `main` dalında uçuşla doğrulanmış referanstan; `avci_plane.parm`
  iki dalda birebir aynı.

### Ölçüm — 12 uçuş, n=4/kol, dönüşümlü A/B

**DÜZ senaryo:**

| koşu | kol | irtifa baş→son | sürükleme | vz medyan | hız medyan |
|---|---|---|---|---|---|
| U1 | AÇIK | 25.4 → 27.8 m | **+1.2 m/dk** | +0.000 | 15.21 |
| U2 | kapalı | 43.9 → 75.1 m | +17.6 m/dk | −0.300 | 15.14 |
| U3 | AÇIK | 79.0 → 79.4 m | **+0.2 m/dk** | +0.000 | 15.21 |
| U4 | kapalı | 95.8 → 129.7 m | +19.1 m/dk | −0.320 | 15.13 |
| U5 | AÇIK | 129.3 → 129.7 m | **+0.2 m/dk** | −0.000 | 15.21 |
| U6 | kapalı | 145.6 → 176.3 m | +17.3 m/dk | −0.290 | 15.14 |
| U7 | AÇIK | 173.5 → 173.8 m | **+0.2 m/dk** | +0.000 | 15.21 |
| U8 | kapalı | 189.0 → 221.2 m | +18.1 m/dk | −0.320 | 15.13 |

**AÇIK medyan +0.23 m/dk · KAPALI medyan +17.80 m/dk.**
Kollar **hiç örtüşmüyor**: açığın en kötüsü (+1.2), kapalının en iyisinden
(+17.3) 14 kat düşük.

**Mekanizma kapısı (§5.1):** açık kolda `|vz medyan|` dördünde de 0.000 →
GEÇTİ.

**DAİRE regresyonu (§5.10):** C1 (açık) **+0.7 m/dk** · C2 (kapalı)
**+58.6 m/dk**. Dairede kusur daha ağır, tutucu orada da düzeltiyor ve
yatış trimini bozmuyor (C1'de hedef 15.20 m/s ile normal daire çiziyor).

### Bedeli — dürüstçe

**Hedefin hızı +0.075 m/s (%0.5) artıyor** (15.135 → 15.210). Gaz sabitken
tırmanışa giden fazla enerji artık hıza gidiyor. Kaçınılmaz: irtifayı gazla
değil elevator'le tutuyorsun, enerji bir yere gitmek zorunda.

*(Not: raporda önce "0.2-0.5 m/s" tahmin edilmişti — fazla tahmin edilmiş,
gerçek bunun üçte biri.)*

### Salınım (ölçülmeden "iyileşti" denmez kuralı)

| ölçüt | AÇIK | kapalı |
|---|---|---|
| \|roll\| p90 | 11.0° | 13.6° |
| roll işaret değişimi/s | 2.18 | 2.52 |
| \|yaw_cmd\| p90 | 132.6° | 74.0° |
| en yakın menzil | 6.75 m | 7.40 m |

**Bu tablo kolları AYIRMIYOR ve ayırması da beklenmiyor.** İki kolda hedef
farklı uçuyor (biri sabit, biri tırmanıyor), yani avcı-tarafı ölçütleri
tasarım gereği karıştırılmış. n=4'te bunlarda hüküm kurulmaz (§5.4).
H-İT'in iddiası avcıyı iyileştirmek değil.

### Ne olduğu

**Bu bir güdüm iyileştirmesi değil, ÖLÇÜM ALETİNİN TAMİRİ.** Bundan sonraki
her A/B sabit irtifada uçan bir hedefle koşulabilir.

⚠ **Kabul edilirse:** bu tarihten önceki kampanya sayıları "tırmanan hedefle
ölçülmüş" sayılmalı; yeni sayılarla doğrudan kıyaslanamaz.

### Kapatma

- Panelde **"🛩 Hedef Ayarları → İRTİFA TUTUCU"** düğmesi (uçuş sırasında,
  yeniden başlatmadan)
- Env: `AVCI_SCN_IRTIFA_TUT=0`
- A/B'de iki kolu aynı irtifada uçurmak için: `AVCI_SCN_ALT=120`

Kapalıyken çıktı **bit bit** eski davranış — test İT1 bunu 32 girdi
kombinasyonunda sınıyor.

### Ham veri

`logs/hit_kampanya/` — koşu başına `telem.csv` (5 Hz bağımsız telemetri),
`gps_guidance.csv`, `kayit/frames/` + `meta.csv`, `bekci.log`.
Analiz: `logs/hit_kampanya/analiz.py`. Video: `logs/hit_{U1,U2,C1,C2}_kamera.mp4`.
*(`logs/` gitignore'da — dosyalar Kübra'nın makinesinde.)*

---

## 2 · Dış görüş kameraları onarıldı

Kök neden tek satırdı:

```python
if vehicle not in ["iris", "plane"]: vehicle = "iris"
```

Arayüz **dört** akış istiyordu (`iris`, `plane`, `iris_chase`,
`talon_chase`); sunucu ikisini tanıyıp gerisini **sessizce avcının ön
kamerasına düşürüyordu**. Yani "Avcı Dış Görüş" düğmesi avcının kendi
kamerasını gösteriyordu.

Üç parça birden eksikti, üçü de eklendi:
- SDF sensörleri: iris'e 1.2 m arka / 0.5 m üst, Talon'a 2.5 m / 0.8 m (15 Hz)
- `latest_frames`'e `iris_chase` / `talon_chase` tamponları
- gz abonelikleri (`AVCI_GZ_CHASE_CAM=0` kapatır)

Sabit liste kaldırıldı; artık tamponların kendisine bakılıyor.

**Canlı simde doğrulandı:** `gz topic -l` dört topic'i de yayınlıyor, sunucu
dördünden de "ilk görüntü" basıyor, ve dört akışın JPEG md5'i **farklı**
(eskiden aynı olurdu). `talon_chase` karesi gözle incelendi: Talon'a
arkadan/üstten bakıyor.

⚠ **Sensörler modele YENİ eklendi → Gazebo yeniden başlatılmalı.**

*(Bu düzeltme `claude_kubra` dalında zaten vardı, oradan taşındı.)*

---

## 3 · `AVCI_GORSEL=on` artık gerçekten görsel fazı açıyor

```python
_GORSEL_ACIK = os.environ.get("AVCI_GORSEL","off").lower() in ("on","1")
_GECERLI_MODLAR = ("gps","visual","hybrid") if _GORSEL_ACIK else ("gps",)
_guidance_mode = "gps"          # ← VARSAYILAN HEP GPS'Tİ
```

O bayrak modu yalnız **seçilebilir** yapıyordu, açmıyordu; ayrıca
`POST /api/guidance_mode` gerekiyordu. **Bu yüzden 8 uçuşluk bir kampanya
saf GPS güdümüyle koştu** ve fark ancak loglara bakınca görüldü
(`[CHASE] Güdüm modu: GPS — görsel temas sağlansa da devredilmez`).

Artık `AVCI_GORSEL=on` demek "hibrit başla" demek.
`AVCI_GUDUM_MODU=gps|visual|hybrid` ile başlangıç modu ezilebilir.

---

## 4 · "Uçuş Logları" düğmesi bağlandı

`/panel` → `/loglar/gps_log_panel.html`'e yönlendiriyordu ama **o dosya hiç
üretilmiyordu.** Koddaki yorum *"panel her GPS uçuşu bitince otomatik
tazelenir (`gps_guidance._panel_tazele`)"* diyordu — **öyle bir fonksiyon
yok**; `tools/gps_log_viz.py:panel_uret()` hiçbir yerden çağrılmıyordu.

Artık panel **istendiğinde** üretiliyor (her basışta taze) ve üretilemezse
404 yerine sebebi yazılıyor.

⚠ **GPS güdüm koduna dokunulmadı** — düzeltme yalnız `gcs_server.py`'nin
`/panel` yolunda.

---

## 5 · Arayüz `claude_kubra` sürümüne geçti

`index.html` + `script.js` + `style.css`. Getirdikleri: 4 kamera seçici,
kilitlenme paneli, FPV'yi pencereye alma, dış görüş kameraları.

Uyum doğrulandı: yeni arayüzün çağırdığı **20 uç noktanın hepsi** bu
backend'de var.

**Kaçamak testi paneli bu arayüze taşındı** (arayüz değişiminde kaybolmuştu):
kaçamak türü + tetik seçimi, başlat/durdur, canlı durum, sonuç (isabet +
KONTROLLÜ/ŞANS sınıfı + salınım + yatış p90) ve son 5 koşunun geçmişi.

Sürüm etiketi `?v=61 → ?v=62` (tarayıcı önbelleği).

---

## 6 · İrtifa tutucu KALICI düğme

`_OZELLIKLER`'de **değil**: orası o an DENENEN özelliğin listesi ve karar
verilince satır siliniyor (§6). Tutucu ölçüldü ve sistemin sabit parçası
oldu, deney değil. Panelde kendi **"🛩 Hedef Ayarları"** bölümü var.

Uç: `GET /api/senaryo_ayar` + `POST /api/senaryo_ayar`.

**Neden ayrı bir yol gerekti:** senaryo, gcs_server'ın `subprocess` ile
başlattığı **ayrı bir süreçtir**; güdüm özelliklerindeki "sunucu `Cfg` sınıf
niteliğini değiştirir, döngü bir sonraki karede okur" yolu burada işlemez
(iki süreç bellek paylaşmaz). Senaryo değeri `gcs_throttle` ile aynı desende
0.5 s önbellekle HTTP'den çekiyor — düğme uçuş sırasında etki ediyor.

---

## Kayra'nın teyit etmesi gereken tek yer

Birleştirmede **`_OZELLIKLER` çakıştı** ve **Kayra'nın tarafı alındı**
(Ö-B silinmiş, O-J kampanyası durdurulmuş, liste boş → `_OZELLIKLER = {}`).
H-İT de zaten oradan çıkıp kalıcı düğmeye taşındığı için liste boş kaldı.
Doğru mu?

## Testler

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_irtifa_tutucu.py   # 14/14
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_bbox_ibvs.py       # 76/76
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_gps_guidance.py    # 34/34
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_kurtarma.py        # 13/13
```

⚠ `tests/test_frpn_guidance.py` D2 (`status` anahtarları) **kırık — ama bizden
değil**: `kayramin_super_gudumu`'nda da kırık, dokunmadığımız iki modülü
(`frpn_guidance`, `gps_guidance`) import ediyor.

## Kendi gözünle görmek için

```bash
# Terminal A — ~50 s (SDF sensörleri yeni, Gazebo ŞART yeniden başlasın)
GZ_HEADLESS=1 bash scripts/start_harmonic.sh

# Terminal B
source /opt/ros/humble/setup.bash
export AVCI_GORSEL=on AVCI_GZ_CAMERA=1 AVCI_NO_BROWSER=1
fuser -k 8000/tcp 2>/dev/null; python3 -m control.gcs_server
```

Panelden `duz` senaryosunu başlat. **Neye bakacaksın:**

- İrtifa göstergesi bir değerin etrafında **oturuyor**. Düğmeyi kapat →
  dakikalar içinde tırmanmaya başlıyor (+17.8 m/dk).
- Üst çubuktaki kamera seçicide "Avcı/Talon Dış Görüş" artık **aracı
  dışarıdan** gösteriyor, ön kamerayı değil.
- Açılışta güdüm modu **HİBRİT** (GPS değil).
- "Uçuş Logları" düğmesi paneli açıyor.
