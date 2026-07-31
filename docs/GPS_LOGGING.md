# GPS Güdüm Loglama & Görselleştirme

GPS güdüm döngüsü (`control/guidance/gps_guidance.py`) her karede bir CSV satırı
yazar. Bu doküman **log formatını** (kolon anlamları) ve **logları görselleştirme
aracını** (`tools/gps_log_viz.py`) anlatır. Amaç: isteyen logları çekip tarayıcıda
açılan tek bir HTML panelinde okuyabilsin.

---

## 1. Log dosyaları

- **Konum:** `logs/gps_guidance_<YYYYMMDD>_<HHMMSS>.csv`
- **Frekans:** 20 Hz (döngü `Cfg.LOOP_HZ`), her karede bir satır.
- **Yeni dosya ne zaman açılır?** `run_gps_guidance()` her çağrıldığında. Hibrit
  görevde (`supervisor.run_hybrid`) GPS↔görsel her geçişte GPS fazı yeniden başlar,
  yani **her GPS fazı için ayrı dosya** oluşur.
  > İpucu: Uzun ama **tek** bir GPS logu = araç hiç görsel faza geçememiş demektir
  > (dönen hedefi hiç yakalayamama imzası). Çok sayıda kısa dosya = sık GPS↔görsel
  > gidip gelme.

---

## 2. Kolonlar

Kaynak: `_CSV_ALANLAR`, `gps_guidance.py`. Çerçeve: **NED/FRD**, açılar derece.

| Kolon | Birim | Anlam |
|---|---|---|
| `t` | s | Zaman damgası (monotonik saat). |
| `dt` | s | Bir önceki kareden geçen süre (~0.05). |
| `durum` | — | Güdüm durumu — aşağıdaki tabloya bak. |
| `d_h` | m | Drone→hedef **yatay** mesafe (`hypot(Δx, Δy)`). Ana yaklaşma metriği. |
| `menzil` | m | Drone→hedef **3B slant** menzil (irtifa farkı dahil). |
| `tgt_x` `tgt_y` `tgt_z` | m | Hedef (sabit kanat) EMA-filtreli konumu (NED; z aşağı +). |
| `tgt_vx` `tgt_vy` `tgt_vz` | m/s | Hedef hızı (konumdan sonlu-fark + EMA). |
| `iris_x` `iris_y` `iris_z` | m | Avcı drone konumu (NED). |
| `iris_roll_deg` `iris_pitch_deg` `iris_yaw_deg` | ° | Drone tutumu (attitude). |
| `st_x` `st_y` `st_z` | m | **İstasyon** = kadraj hedef noktası (hedefin ~10 m gerisi + ~4.6 m altı). Drone buraya sürülür. |
| `vx_cmd` `vy_cmd` `vz_cmd` | m/s | Araca gönderilen hız komutu (NED). Büyüklüğü `V_MAX`=20 ile sınırlı. |
| `yaw_cmd_deg` | ° | Komut edilen burun yönü (hedefe kerteriz). |
| `kadraj_yaw_deg` | ° | **Yatay kadraj hatası**: hedefin kameradaki yatay sapması. 0 = tam ortada. |
| `kadraj_elev_deg` | ° | **Dikey kadraj açısı**: hedefin LOS yükselişi. Hedef **25°** (kamera tilt'i) = dikeyde ortada. |
| `kadraj_pitch_hata_deg` | ° | `kadraj_elev_deg − 25`. 0 = dikeyde ortalanmış. |
| `u_px` `v_px` | px | Hedefin tahmini piksel konumu (640×480). Merkez = **(320, 240)**. |

### `durum` değerleri

| Durum | Koşul | Anlam |
|---|---|---|
| `WARMUP` | başlangıç / telemetri henüz geçerli değil | Isınma. |
| `ARAMA` | `d_h ≥ 20 m` (`HANDOFF_RANGE`) | Yaklaşıyor, henüz devir bandı dışında. |
| `KILIT` | `d_h < 20 m` | Görsel devir bandında — supervisor görsel faza geçebilir. |
| `DROPOUT` | hedef telemetrisi `HOLD_S`=3 s dondu | GPS verisi kesik (jamming fallback). |
| `DURDU` | döngü sonlandı | — |

### Kadraj başarı ölçütü

Hedef kamerada **ortalanmış** ⇔ `kadraj_yaw_deg ≈ 0` **ve** `kadraj_elev_deg ≈ 25°`
⇔ `(u_px, v_px) ≈ (320, 240)`. GPS güdümün amacı budur: hedefi görsel güdümün
devralabileceği pozisyona (kadraj merkezi, ~11 m slant) getirmek — vurmak değil.

---

## 3. Görselleştirme paneli — `tools/gps_log_viz.py`

Logları **tek, kendine yeten HTML paneline** çevirir (internet/CDN gerektirmez).

### 3.1 Otomatik güncelleme — elle çalıştırmaya gerek yok

**Panel her GPS uçuşu bitince kendiliğinden tazelenir.** `run_gps_guidance()`
sonlanırken `_panel_tazele()` çağrılır (bkz. `control/guidance/gps_guidance.py`),
en yeni 12 log yeniden işlenir ve panel üzerine yazılır. Panel üretimi hata alsa
bile uçuş etkilenmez (istisna yutulur).

Uçuş sonunda konsola link basılır. Panele üç yoldan erişilir:

| Yol | Adres |
|---|---|
| **GCS arayüzü** | Sağ üstteki **📊 UÇUŞ LOGLARI** düğmesi |
| **Kısayol URL** | <http://localhost:8000/panel> |
| **Doğrudan dosya** | `logs/gps_log_panel.html` (GCS kapalıyken `file://…` ile aç) |

`gcs_server` `logs/` dizinini `/loglar` altında servis eder; kısayol `/panel`
oraya yönlendirir.

### 3.2 Panelde ne var

**Özet kartları** — en yakın menzil · hız farkı (drone−hedef) · komut doygunluğu ·
KILIT oranı · telemetri Hz.

**Grafikler** (hepsi hover'lı: imleci gezdir → o andaki tüm değerler + dönüş
etiketi). Dönüş (manevra) fazları grafiklerde **gölgeli bant**tır.

| Grafik | Ne söyler |
|---|---|
| **Hız** — drone gerçek / hedef / komut | **Ana teşhis.** Drone çizgisi hedefin altındaysa açı kapanmaz. Komut ile gerçek arasındaki fark = aracın uygulayamadığı istek (fizik limiti). Komut düz bir tavana yapışıksa yazılım `V_MAX` sınırlıyordur. |
| Kuşbakışı yörünge | Drone hedeften **büyük** yay çiziyorsa dışarıda orbit atıyordur (dönen hedefi içeriden kesemiyor). |
| Kamera nişangâhı | Hedefin (u,v) izi; merkez (320,240) = kadrajlandı. |
| Menzil d_h | 11 m istasyon çizgisine iniyor mu, yoksa sabit yükseklikte mi asılı. |
| Kadraj açıları | elev→25°, yaw→0° oturuyor mu. |
| Araç eğimi \|roll\|/\|pitch\| | Eğim ≈ yatay ivme (a ≈ g·tan θ). Dönüşte düşük kalıyorsa itki rezervi yok. |
| İrtifa profili | Drone hedefin ~4.6 m altında mı (kamera 25° yukarı baksın diye). |

**Faz kırılımı tablosu** — aynı uçuşun *düz uçuş* ve *dönüş* rejimleri yan yana:
manevrada ne kaybedildiğini sayıyla gösterir.

**Tüm uçuşlar tablosu** — süre, telemetri Hz, d_h min/ort, KILIT %, drone/hedef
hız, Δhız, doygunluk. Satıra tıkla → o uçuş yukarıda açılır.

**Otomatik yorum** — veriden türetilir: hız yetersizliği, manevra kaybı,
yaklaşamama, kadraj sonucu, yavaş telemetri uyarısı.

### 3.3 Türetilmiş metrikler (CSV'de yok, panel hesaplar)

| Metrik | Nasıl |
|---|---|
| **Drone gerçek hızı** | `iris_x/y`'nin türevi (yalnız taze telemetri örnekleri arası). Komut hızıyla karıştırma — araç komutu uygulayamıyor olabilir. EKF sıçramaları (>60 m/s) elenir, 7'lik kayan medyan ile örnekleme artefaktı süzülür. |
| **Telemetri Hz** | `tgt_x` / `iris_x` değerinin kaç saniyede bir *değiştiği*. Log döngüsü 20 Hz olduğu için tavan 20'dir; 4 Hz görüyorsan MAVProxy `--streamrate` düşüktür. |
| **Faz (düz/dönüş)** | Hedefin hız vektörü yönünün değişim hızı: ≥15°/s dönüş, <8°/s düz. |
| **Komut doygunluğu** | Komut hızının, uçuşta gözlenen tavanın %98'ini aştığı kare oranı. Yüksekse `V_MAX` darboğazdır. |

### 3.4 Elle çalıştırma

```bash
python3 tools/gps_log_viz.py                 # en yeni 8 log
python3 tools/gps_log_viz.py --last 20       # en yeni 20 log
python3 tools/gps_log_viz.py logs/a.csv logs/b.csv
python3 tools/gps_log_viz.py --last 4 -o rapor.html --open
```

Büyük loglar panele ~700 noktaya indirilerek gömülür (downsample); **özet
istatistikler her zaman tam veriden** hesaplanır. 20 kareden kısa loglar
(anlık başlat/durdur) panele alınmaz.

### 3.5 Panel nasıl okunur (hızlı)

| Görülen | Anlamı |
|---|---|
| Drone hız çizgisi hedefin **altında** | Yetişemez — araç yavaş ya da `V_MAX` düşük (doygunluğa bak). |
| Komut çizgisi düz **tavan**, gerçek çok altında | Araç komutu uygulayamıyor: itki/fizik limiti. |
| Dönüş bantlarında drone hızı **çöküyor** | Manevra kabiliyeti yetersiz (itki rezervi yok; \|roll\| grafiğine bak). |
| d_h 11 m'e iniyor, nişangâhta noktalar merkezde | GPS güdüm çalışıyor, kadrajladı. |
| d_h 60-90 m'de asılı, 0 KILIT, drone yörüngesi hedeften **büyük** | Dönen hedefi yakalayamıyor (dışarıda orbit). |
| Nişangâh noktaları merkezin **altında** (v_px > 240) | Dikeyde aşağı bias (elev < 25°). |
| Telemetri kartı **4 Hz** | MAVProxy `--streamrate` düşük → faz gecikmesi, yüksek hızda salınım. |
