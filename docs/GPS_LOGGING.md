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

## 3. Görselleştirme aracı — `tools/gps_log_viz.py`

Bir veya daha çok CSV logunu **tek, kendine yeten HTML paneline** çevirir
(internet/CDN gerektirmez). Panelde uçuş başına:

- **Kuşbakışı yörünge** (drone vs hedef, N-E) — "drone hedefin dışında mı orbit atıyor" bunu gösterir.
- **Kamera nişangâhı** — hedefin (u,v) izi, merkeze göre sapma.
- **Menzil d_h** zaman serisi (11 m istasyon çizgisiyle).
- **Kadraj açıları** (elev/yaw, hedef çizgileriyle).
- Veriden türetilen **otomatik yorum** (yaklaşamadı / kadraja girdi / kilitlenemedi).

### Kullanım

```bash
# En yeni 6 log:
python3 tools/gps_log_viz.py

# En yeni 8 log:
python3 tools/gps_log_viz.py --last 8

# Belirli dosyalar:
python3 tools/gps_log_viz.py logs/gps_guidance_20260728_152012.csv \
                             logs/gps_guidance_20260728_152524.csv

# Çıktı yolu + oluşturunca tarayıcıda aç:
python3 tools/gps_log_viz.py --last 4 -o rapor.html --open
```

Varsayılan çıktı: `logs/gps_log_panel.html`. Üstteki düğmelerden uçuşlar arasında
geçilir. Büyük loglar panele ~600 noktaya indirilerek gömülür (downsample).

### Panel nasıl okunur (hızlı)

| Görülen | Anlamı |
|---|---|
| d_h 11 m'e iniyor, nişangâhta noktalar merkezde | GPS güdüm çalışıyor, kadrajladı. |
| d_h 60-90 m'de asılı, 0 KILIT, drone yörüngesi hedeften **büyük** | Dönen hedefi yakalayamıyor (dışarıda orbit). |
| Nişangâh noktaları merkezin **altında** (v_px > 240) | Dikeyde aşağı bias (elev < 25°). |
| yaw sık sık ±90° aşıyor, hedef arkaya düşüyor | Overshoot / yüksek-hız kararsızlığı. |
