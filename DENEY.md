# DENEY — hangi değişiklik bozuyor, teker teker

**Amaç:** 2026-08-04'te üst üste yapılan değişikliklerden hangisinin uçuşu
bozduğunu bulmak. Bugüne kadarki sorun, birden fazla şeyin **aynı anda**
değişmesiydi — hiçbir sonuç tek bir sebebe bağlanamadı.

**Kural:** Bir adımda **tek değişken** değişir. Adımı uç, karneyi al, tabloya
yaz, sonra bir sonrakine geç. Sonuç kötüleşirse **o adımdaki değişken suçlu.**

Gazebo gerçek pozu (GT) tabanda kalır — kararlaştırıldığı gibi.

---

## Nasıl uçulur

```bash
# Terminal A (her deneyden önce bir kez; ~50 s)
cd ~/projects/hamidiyesim
GZ_HEADLESS=1 bash scripts/start_harmonic.sh

# Terminal B — adımı seç
bash scripts/gcs.sh A0        # sonra A1, A2, A3, A4

# Uçuştan sonra
python3 tools/gudum_karne.py                    # son uçuşun karnesi
python3 tools/gudum_karne.py --kiyasla <A> <B>  # iki adımı yan yana
```

Her adımda **en az 5-6 görsel faz** topla. Tek faz gürültüdür; 18:01'deki
uçuşta tek faz vardı ve hiçbir şey söylemiyordu.

Başlangıçta `gcs.sh` hangi bayrakların açık olduğunu tek satırda basar —
ölçüme başlamadan **o satırı oku**, beklediğin adımda olduğunu doğrula.

---

## Adımlar

| adım | GT poz | pose modeli | pose kilidi kapısı | HybridSORT | kilitli-ID |
|---|---|---|---|---|---|
| **A0** taban | açık | **kapalı** | **atlanıyor** | kapalı | kapalı |
| **A1** | açık | **açık** | atlanıyor | kapalı | kapalı |
| **A2** | açık | açık | **açık** | kapalı | kapalı |
| **A3** | açık | açık | açık | **açık** | kapalı |
| **A4** | açık | açık | açık | açık | **açık** |
| **pose** | **kapalı** | açık | açık | kapalı | kapalı |

Kalın olan, bir önceki adıma göre değişen tek şey.

### A0 — taban: sadece GT

Güdüm tamamen Gazebo gerçek pozundan. Pose modeli hiç yüklenmiyor, takip yok,
geçiş kapısı yalnız menzile bakıyor. **Bu, sistemin en sade hâli** — burada da
kötüyse suç yukarıdaki hiçbir eklentide değil, güdüm yasasındadır.

> Not: bu adımın çalışabilmesi için kare akışı pose modelinden ayrıldı
> (`gcs_server.process_iris_frame`). Eskiden `AVCI_POSE=off` yapılınca güdüm
> döngüsü kare bekleyip donuyordu.

### A1 — pose modeli yüklensin

Güdüme **girmiyor** (GT açık), sadece ekran/log için çalışıyor. Test ettiği
şey: pose çıkarımının kendisi kare hızını/gecikmeyi bozuyor mu? YOLO-pose her
karede ek GPU işi demek; `gecikme_s` sütunu ve `bayat` durum sayısı buna bakar.

### A2 — pose kilidi kapısı geri gelsin

GPS→görsel geçişi artık "son 15 karenin 10'unda güvenli pose" şartına da bağlı.
**Bu kapı bir gecikme görevi görüyor:** kapalıyken devir `GATE_MENZIL`=20 m'ye
yapışıyor, açıkken ~6 m'de oluyor.

Ölçülmüş (ama takiple karışık, bu yüzden tekrar ediliyor):

| | kapı açık | kapı kapalı |
|---|---|---|
| görsel faza giriş | 6.6 m | 19.6 m |
| en yakın menzil | 0.68 m | 2.41 m |
| GPS istasyonda oturma | %33.7 | %0.4 |

### A3 — HybridSORT takip açılsın ⚠

**Bu adım en çok şüphelenilen.** `boxmot` paketi bugüne dek kurulu olmadığı
için takipçi **hiç çalışmamıştı**; projedeki bütün eski ölçümler takipçisiz.
2026-08-04'te kurulunca sessizce devreye girdi ve aynı gün kapı değişikliğiyle
karıştı — payı **hiç ayrı ölçülmedi**.

Takipçi tespit kutusunu değiştirebilir: Kalman köprüsü, kimlik karışması,
gecikme. Kötüleşme buradaysa `AVCI_TRACKER=off` kalıcı çözümdür.

### A4 — kilitli-ID politikası

Takipçinin üstünde çalışır: hedefi bir ID'ye kilitler, kilit COAST'a düşünce
nişan komutu üretmez. A3 ile A4 arasındaki fark bu politikanın payıdır.

### pose — gerçek sistem

GT kapalı, güdüm pose modelinden. Teslim edilecek yapılandırma bu.
A0-A4 teşhis; bu satır "gerçekte ne kadar iyiyiz" sorusunun cevabı.

---

## Sonuç tablosu — doldur

Karneden alınacak alanlar. Boş bırakma; "uçmadım" bile bir bilgidir.

| adım | görsel faz | vuruş | en yakın (m) | devir menzili (m) | görsel yaw RMS | GPS oturma % | not |
|---|---|---|---|---|---|---|---|
| A0 | | | | | | | |
| A1 | | | | | | | |
| A2 | | | | | | | |
| A3 | | | | | | | |
| A4 | | | | | | | |
| pose | | | | | | | |

**Referans — bugün ölçülenler** (karışık değişkenli, sadece kıyas için):

| koşu | faz | vuruş | en yakın | devir | not |
|---|---|---|---|---|---|
| 164352 | 7 | 0 | 0.68 m | 6.6 m | GT, takip yok, kapı açık |
| 172103 | 13 | 0 | 2.41 m | 19.6 m | GT, **takip VAR**, **kapı YOK** ← iki değişken |
| 180044 | 1 | 0 | 1.42 m | 6.0 m | GT, takip var, kapı açık (tek faz) |

---

## Yorumlama

- **A0 zaten kötüyse:** suç eklentilerde değil. Sıradaki şüpheli kapanma hızı /
  ivme bütçesi — `TODO.md` §1. Hesap: 25 m/s ve 4 m/s² ile 1 m yanal hatayı
  düzeltmek 0.71 s, yani 17.7 m menzilde bitmiş olmalı; devir ~6 m'de oluyor.
  Denenecek: `AVCI_IBVS_V_KAPANMA=15`.
- **A2'de bozuluyorsa:** kapı gerçekten zararlı → kapalı bırak, ama devir
  menzilini `AVCI_HYBRID_GATE_MENZIL` ile elle ayarla.
- **A3'te bozuluyorsa:** HybridSORT suçlu → `AVCI_TRACKER=off` kalıcı,
  `vision/tracker.py` gözden geçirilir.
- **Hiçbirinde bozulmuyorsa:** bozulma bu bayraklarda değil; ortamda
  (Gazebo RTF, GPU yükü) veya uçuş senaryosunda değişen bir şey var.
  `gz topic -e -t /stats -n 2` ile RTF'yi ölç — 0.98 altındaysa fizik yavaşlamış
  demektir ve tüm zamanlama kayar.

---

## Değişen varsayılanlar (2026-08-04)

Deneyi etkilemesin diye kod varsayılanları da düzeltildi:

- `AVCI_TRACKER` artık **varsayılan kapalı**. Gerekçe: proje tarihindeki bütün
  ölçümler takipçisiz alınmıştı; `boxmot` kurulunca sessizce açılması ölçüm
  tabanını haber vermeden değiştirmek olurdu.
- `AVCI_GT_KILIT_BYPASS` varsayılan kapalı (kapı açık) — A2 ve sonrası bu.
- Kare akışı pose modelinden ayrıldı, böylece A0 mümkün.
