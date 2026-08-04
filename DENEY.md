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
| **A5** | açık | açık | **atlanıyor** | açık | kapalı |
| **pose** | **kapalı** | açık | açık | kapalı | kapalı |

Kalın olan, bir önceki adıma göre değişen tek şey.
**A5 istisna:** A3'ün değil **A1'in** eşi — tek farkı takip. Bkz. aşağıda.

---

## SONUÇLAR (2026-08-04, yapılandırma damgalı, gz menzil)

| adım | faz | vuruş | <1.5 m | en yakın | min medyan | devir | yaw ort | tespit_yok |
|---|---|---|---|---|---|---|---|---|
| A0 | 8 | 0 | 3/8 | 0.24 m | 12.99 m | 21.9 m | 40.8° | %17 |
| A1 | 15 | 2 | 4/15 | 0.29 m | 5.93 m | 20.1 m | 36.2° | %21 |
| **A2** | 8 | **2** | **7/8** | 0.34 m | **0.67 m** | **6.1 m** | 33.7° | **%5** |
| A3 | 3 | 1 | 1/3 | 0.64 m | 1.55 m | 6.5 m | **8.1°** | %0 |
| A4 | 1 | 0 | 0/1 | 11.88 m | 11.88 m | 15.8 m | 47.5° | %11 |

**A2 açık ara en iyi.** A1→A2 arasındaki tek fark pose kilidi kapısı: kapı
açılınca devir 20.1 → 6.1 m'ye iniyor, yakın geçiş oranı 4/15 → 7/8 oluyor.
Bu, kapının bağımsız üçüncü ölçümü (önceki ikisi 164352/172103 ve 18:55 turu).

A3/A4 örneklem çok küçük (3 ve 1 faz) — takip hakkında **hâlâ karar yok**.

---

## A5 — takibin etkisini İZOLE EDER

A3, A2'den hem takiple hem başka gürültüyle ayrılıyordu ve 3 fazlıktı. A5
bunun yerine **A1 ile eşleştirilir**: ikisinde de kapı atlanıyor, ikisinde de
pose açık, tek fark takip.

| | A1 | A5 |
|---|---|---|
| kapı | atlanıyor | atlanıyor |
| takip | **kapalı** | **açık** |

A1'in 15 fazlık verisi zaten elde. A5'i 8-10 faz uçarsan takibin payı doğrudan
okunur — kapı etkisi ikisinde de aynı olduğu için sadeleşir.

```bash
bash scripts/gcs.sh A5
python3 tools/gudum_karne.py
```

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

**Devir için İKİ kapı var, karıştırmayın:**

1. **Pose kilidi kapısı** — "hedefi net görüyor musun?" Son `KILIT_PENCERE`=15
   karenin en az `KILIT_N`=10'unda pose güveni ≥ `POSE_CONF_MIN`=0.5.
   Pratikte ~6 m'de sağlanıyor. `AVCI_GT_KILIT_BYPASS=on` bunu atlar.
2. **Menzil kapısı** — "yeterince yaklaştın mı?" Yatay mesafe <
   `GATE_MENZIL`=20 m (`AVCI_HYBRID_GATE_MENZIL`). Gerekçesi kinematik:
   görsel faz sabit `V_KAPANMA` ile kapanıyor, uzaktan devralırsa yetişemiyor.

İkisi birden sağlanmalı. Kilit kapısı kapatılınca devir **yalnız menzil
kapısına** kalıyor ve tam 20 m'de tetikleniyor — yani kilit kapısı fiilen bir
"daha yaklaş" gecikmesi işlevi görüyor. Ölçümde bu farkın bedeli büyük.

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
