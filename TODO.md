# TODO — GT modu bulgusundan çıkan işler

**Ana belge [DURUM.md](DURUM.md)'dir.** Bu dosya sadece 2026-08-04'te GT
rotasyon modu ölçüldükten sonra ortaya çıkan işleri tutar. Bir madde
bitince DURUM.md §6 "Bitenler"e taşı.

> ⚠ **Önce [DENEY.md](DENEY.md).** 2026-08-04'te çok sayıda değişiklik üst üste
> bindi ve hiçbir sonuç tek sebebe bağlanamadı. A0-A4 adımları tek tek uçulup
> suçlu bulunmadan buradaki maddelere geçme — aksi hâlde yine karışık değişkenli
> ölçüm yaparsın.

---

## Neden bu liste var: algı darboğaz değilmiş

GT modu (`AVCI_GT_ROT=on`) güdümün algı girdisini tamamen Gazebo gerçek
poza çevirdi — yani pose modeli mükemmel olsaydı ne olacağını ölçtük.

| | pose modu | GT modu |
|---|---|---|
| görsel faz | 11 | 9 |
| vuruş | 2 (%18) | 1 (%11) |
| medyan en yakın mesafe | 0.92 m | 0.98 m |

**Fark yok.** Algıyı kusursuz yapmak isabeti değiştirmedi. Darboğaz pose
modeli değil; aşağıdaki maddeler asıl şüpheliyi kovalıyor.

---

## 1. Terminal fazda kontrol yetkisi — EN YÜKSEK ÖNCELİK

**Şüphe:** drone hedefe ~1 m'ye geliyor ve ıskalıyor. Bu bir nişan hatası
değil, **fizik sınırı** olabilir.

Mevcut parametrelerle (`V_KAPANMA=25` m/s, `IVME_TAVAN=4` m/s² yatay):

| yanal hata | düzeltmek için gereken süre | bu menzilde çoktan bitmiş olmalı |
|---|---|---|
| 0.5 m | 0.50 s | 12.5 m |
| 1.0 m | 0.71 s | **17.7 m** |
| 2.0 m | 1.00 s | 25.0 m |

Görsel faza giriş menzili medyanı **~10 m**. Yani 10 m'de elde kalan
1 m'lik yanal hata **düzeltilemez** — 0.4 s'de 4 m/s² ile ancak 0.32 m
yana kayabiliyoruz. Dönüş yarıçapı 25 m/s'te yatayda 156 m, dikeyde 62 m.

**Ölçüm:** `V_KAPANMA`'yı 25 → 15 → 10 düşürüp karne al. Hipotez doğruysa
en yakın mesafe medyanı belirgin düşer. Yavaşlamak hedefe yetişememe riski
getirir — GPS fazı zaten yetiştiriyor, o yüzden denemeye değer.

```bash
AVCI_IBVS_V_KAPANMA=15 bash scripts/gcs.sh
python3 tools/gudum_karne.py --kiyasla <eski> <yeni>
```

**Uyarı:** `IVME_TAVAN=4` keyfi değil — quad ileri ivmelenmek için burnunu
eğiyor, kamera gövdeye +25° bağlı, 5 m/s² üstünde kamera yere bakmaya
başlıyor (bkz. `adapter_copter`). GT modunda bu kısıt yok sayılabilir
(kamera zaten kullanılmıyor) — **GT modunda `IVME_TAVAN`'ı yükseltmek ayrı
bir deney**, ve yasanın tavanını ölçmenin temiz yolu.

---

## 1b. Pose kilidi kapısı — DENENDİ, ÇÜRÜTÜLDÜ, GERİ ALINDI

GT modunda pose kilidi kapısı kaldırılmıştı (mantık: güdüm pose'a bakmıyorsa
geçişi pose tutmasın). **Ölçüm çürüttü**, varsayılan kapalıya alındı
(`AVCI_GT_KILIT_BYPASS=on` ile açılabilir):

| | kilit VAR (164352) | kilit YOK (172103) |
|---|---|---|
| görsel faza giriş medyanı | 6.6 m | **19.6 m** |
| en yakın menzil | 0.68 m | **2.41 m** |
| GPS istasyonda oturma | %33.7 | **%0.4** |
| GPS kadraj yaw RMS | 35.7° | **116.8°** |
| faz sonucu | 3 ıska / 4 kayıp | **13/13 kayıp** |

**Mekanizma:** pose kilidi farkında olmadan bir **gecikme** görevi
görüyormuş — ~6 m'de oturuyor, devir orada oluyordu. Kilit kalkınca devir
`GATE_MENZIL=20` kapısına yapıştı; görsel faz yetişemeyeceği menzilde
devralıp hemen kaybediyor. GPS fazı da 8-12 m istasyon bandına hiç
giremiyor.

Bu §1'i doğruluyor: **sorun kapanma hızı/ivme bütçesi.** 25 m/s ile 20 m'den
devralmanın düzeltme payı zaten yok. Kapıyı ancak `V_KAPANMA` düşürüldükten
sonra tekrar açmayı dene.

---

## 2. Ölçümler yeniden alınmalı — HybridSORT kapalıymış

`boxmot` kurulu değildi, `[GCS] HybridSORT yüklenemedi` yazıyordu. Yani
**bugüne kadarki tüm uçuşlar takipçisiz** yapıldı; kilitli-ID politikası
(`TargetLock`) hiç devreye girmedi.

2026-08-04'te kuruldu (`pip install boxmot==19.0.0`). Sonuç: mevcut karne
sayıları takipçili sistemin performansı **değil**. §1'e geçmeden önce
takipçili bir taban ölçümü al, yoksa neyi neyle kıyasladığın belirsiz olur.

⚠ **Takipçi ile §1b aynı anda değişti — ayrıştırılmadı.** 172103 uçuşundaki
kötüleşmenin baskın sebebi §1b (devir menzili 6.6→19.6 m; takipçi devir
menzilini değiştiremez). Takipçinin kendi etkisi hâlâ ölçülmedi: pose oranı
%60→%62 ile neredeyse aynı kaldı, yani algıyı bozmuyor gibi görünüyor.
Temiz ölçüm için `AVCI_TRACKER=off` ile bir koşu al ve karneleri kıyasla.

---

## 3. `_menzil_olc()` bağlanmamış (ölü kod)

`control/guidance/visual_lead.py:251` — "önce sim-truth, yoksa telemetri"
diye yazılmış ama **hiçbir yerden çağrılmıyor**. Fiilen kullanılan menzil
`get_plane_truth()` → hedefin MAVLink telemetrisi.

Sorun: `sim_truth.py` tam olarak bu yüzden yazılmıştı — telemetri menzili
iki ayrı akışın zaman hizasız farkı, 250 ms'ye kadar bayat, kapanmada 5-6 m
hata veriyor. Zaman hizalı gz menzili duruyor ama kullanılmıyor.

**İş:** ana döngüde `_menzil_hesapla(...)` yerine `_menzil_olc()` çağır,
`menzil_gercek_m` sütununu iki kaynakla kıyasla. Terminal faz kararları
(kör dalış girişi, `coalt_latch`) bu menzile bakıyor — §1'i ölçmeden önce
bu düzeltilmeli, yoksa §1'in sonucu bayat menzille kirlenir.

---

## 4. GT modunda menzil kapısı gevşetilsin mi

Pose kilidi GT modunda kaldırıldı; geçiş artık yalnız `GATE_MENZIL=20 m`
kapısına bakıyor. GT'de "uzakta pose güvenilmez" gerekçesi yok — kapı
tamamen açılırsa görsel faz çok daha erken başlar ve §1'deki düzeltme
bütçesi büyür.

```bash
AVCI_HYBRID_GATE_MENZIL=40 bash scripts/gcs.sh
```

Kapının kinematik gerekçesi duruyor (sabit `V_KAPANMA` ile uzaktan
yetişilemez), o yüzden bu §1 ile **birlikte** denenmeli: yavaş kapanma +
erken devir aynı deneyin iki yarısı.

---

## 5. Pose modunun manevra körlüğü (gerçek algı bulgusu)

`tests/test_visual_lead.py` T53b ile ölçüldü: hedef **bank yaparken** pose
yandanlığı gerçeğin altında kalıyor — 45° yatıkta 1.00 yerine **0.73**.
Sebep: `yandanlik = a/olcek` "hedef seviyeli uçuyor" varsayıyor.

Sonuç: manevra yapan hedefte lead eksik hesaplanıyor, drone yeterince
önüne kesmiyor. GT modunda bu körlük yok — ama GT teslim edilebilir değil.

**İş:** pose'un 6. ve 5. keypoint'i (V-tail uçları) kullanılmıyor. Bank
açısı bu ikisinin kanat ekseni etrafındaki asimetrisinden kestirilebilir.
Bu, GT'ye başvurmadan yandanlık ölçümünü düzeltmenin yolu.

---

## 6. GT modunda karne alma — dikkat

GT modunda `menzil_kestirim_m` sütunu gerçek menzili aynen yansıtıyor,
`pose_*_sapma` sütunları sıfırlanıyor. **Algı sapması ölçülemez.**
Model karnesi almak için `bash scripts/gcs.sh pose` kullan.
