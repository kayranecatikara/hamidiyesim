# Yandan Eskort Tekniği + Gimbal Planı

> Bu doküman, 2026-08-08'de uçuşla doğrulanan "yakın iç eskort" tekniğini
> ve onun bilinçli olarak kabul edilen sınırını kayda geçirir. İleride
> drone'a **gimbal** eklenirse buradaki teknik tam gücüyle geri getirilir.

## Teknik ne?

Dönen hedefe karşı istasyonun "arka" bileşeni dönüş hızıyla eritilir
(`Cfg.ARKA_KISALT`, gps_guidance.py) ve istasyon saf "iç" pozisyona geçer.
Dairesel kovalamacanın temel yasası bunu zorunlu kılar:

```
yarıçap = hız / açısal_hız
```

Hedefin gerisindeki nokta hedefin KENDİ çemberinin üzerindedir — orada durmak
aynı yarıçap, dolayısıyla aynı hız demektir ve mesafe asla kapanmaz (ölçüldü:
eski sistem 29-34 m'de donuyordu). İçeri kesmek küçük yarıçap + düşük hız
demektir; **dönüşte hem yakın hem arkada olmak geometrik olarak imkânsızdır.**

Uçuşla ölçülen sonuç (truth medyanlar, 2026-08-08):

| Senaryo | Eski | Yakın iç eskort |
|---|---|---|
| Düz | 13-14 m | 10.3 m (tam kuyruktan — değişmedi) |
| ⌀96 | 15-16 m | 8.9 m |
| ⌀71 | 15-16 m | 7.0-7.2 m |
| ⌀55 | 15-16 m | 5.7 m |
| ⌀41 | 16-17 m | 9.5-10.4 m |

## Bilinçli kabul edilen sınır

Dönüşte drone hedefi **tam yandan** görür (90° bakış; eski geometri de 66°
ile yandan görüyordu, yenilik yakınlık ve tam yan olması). Sonuçları:

- Tespit (bbox) GÜÇLENİR: yakın + yan profil = büyük kutu, güven ~0.85.
- Pose zinciri KİLİTLENEMEZ: yan profilde "kanat yok" → pose kilidi yok →
  supervisor görsel faza devri açmaz (panel: GEÇİŞİ ENGELLEYEN: POSE KİLİDİ).
- IBVS görsel güdüm kuyruk-takibi varsayımıyla kurulduğu için yandan devir
  alırsa hedefi kadrajdan kaçırma riski yüksektir.

Önemli: hedef DÜZE geçtiği anda arka bileşen kendiliğinden geri büyür
(ω→0, ~2-4 s) ve drone kuyruğa kayar — devir geometrisi otomatik geri gelir.
Sınır yalnız SÜREKLİ dönüş sırasında geçerlidir.

## Gimbal gelirse ne değişir? (gelecek planı)

Kameraya yaw gimbalı eklenirse gövde yönü ile bakış yönü ayrışır:

1. Drone iç eskort rotasını uçarken kamera hedefe YANDAN kilitli kalır —
   **5-7 m mesafe + kesintisiz görsel kilit aynı anda** mümkün olur.
2. Pose/IBVS zinciri yan profil için yeniden eğitilir/formüle edilir
   (görüntü hatası → komut eşlemesi gimbal çerçevesine taşınır).
3. Geri getirme adımları:
   - `AVCI_GPS_ARKA_KISALT=1` (bugünkü varsayılan zaten bu; geri alındıysa)
   - Kamera yönlendirme yasası: gövde yaw yerine gimbal açısı hedef bearing'e
   - Dinamik istasyon yükselişi (`ELEV_DINAMIK`) gimbal pitch'ine taşınır
4. Bu dosyanın yazıldığı andaki ölçüm referansları: log 141740 (⌀55, 5.7 m),
   144907 + 150726 (çap taraması), commit `6b0bfea` / `82fffa3`.

## Gimbalsız dönemde politika

GPS fazının amacı görsel faza EN İYİ pozda devretmektir. Yandan devir
verilmez; devir kapısı kuyruk-benzeri geometri ister (pose kilidi bugünkü
doğal bekçi; açık ω/bakış-açısı kapısı UYGULANACAK'ta önerilmiştir).
Dönüşte GPS yakın eskort yapar (tespit kilidi süre toplamaya devam eder),
devir hedef düzeldiğinde gerçekleşir.
