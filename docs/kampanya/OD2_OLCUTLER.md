# Ö-D2 KAMPANYASI — ölçütler (KOŞMADAN ÖNCE yazıldı, 2026-08-13)

## Özellik
`AVCI_GPS_FOV=25` — GPS fazında burun komutu gövdeden en fazla 25° önde:
`cmd_yaw = iyaw + clamp(cmd_yaw − iyaw, ∓25°)`, slew'den SONRA.

## Kanıt (kullanıcının uçuş kaydı ucus_20260813_175002)
Uçuşun **%24.6'sı** KURTARMA'da geçti (2261 karenin 557'si). İki olay:
15.9 s ve 12.0 s. Tetik anında araç TAMAMEN DÜZDÜ (roll +2°/−3°,
pitch −3°/+1°) — takla değil, emredilen ~175°'lik dönüşü yapıyordu.
Zincir: bearing 175° döner → cmd_yaw aracın çok öncesinde varır →
borç 175° → ArduPilot yaw'ı sertçe sürer (300-530°/s) → bekçi tetikler
→ hız sıfır → 12-16 s donma → hedef kaçar.

## BİRİNCİL ÖLÇÜT
**KURTARMA'da geçen kare oranı (%)** — `gps_guidance_*.csv`'de
`durum == KURTARMA` karelerinin toplama oranı. KÜÇÜK olan kazanır.
Referans: kullanıcının uçuşunda %24.6.
- Geçerlilik eşi (§5.2): uçuşun tamamlanması + isabette gerileme olmaması.
  Araç hiç dönmeyip hedefi hiç kovalamazsa KURTARMA da olmaz — bu sahte
  iyileşmedir; bu yüzden isabet ve en yakın menzil ZORUNLU eşlik eder.

## İKİNCİL ÖLÇÜTLER
1. `fov_kis` mekanizma sütunu (§5.1 kapısı). Deney kolunda sıfırsa GEÇERSİZ.
2. İSABET, en yakın menzil.
3. Görsel faza devir menzili ve devir süresi (kamera gövdeye sabit;
   burun geç dönerse hedefi geç görür — bu düzeltmenin ASIL RİSKİ).
4. Toplam görsel temas süresi.

## ETKİ ALANI TABLOSU (CLAUDE.md §5.10 — kullanıcı kuralı 2026-08-13)
| etkilenebilecek davranış | neden | hangi senaryoda sınanır |
|---|---|---|
| Büyük dönüşler yavaşlar | borç sınırlanınca burun daha az hata ile döner | `duz` + kaçamak |
| Sürekli manevrada takip | dairede burun sürekli geride kalabilir | `circle` REGRESYON |
| Görsel faza devir | kamera gövdeye sabit → burun geç dönerse geç görür | her koşuda devir menzili/süresi |
| Kuyruk takibi / istasyon tutma | borç birkaç derece → sınır bağlamaz | `duz` + `yok` TABAN |
| Görsel/terminal faz | Ö-D2 yalnız gps_guidance'ta | yapısal: dokunulmuyor |

## KARAR KURALI (önceden ilan)
- Ö-D2 **girer** eğer: KURTARMA oranı belirgin düşer VE isabet/en yakın
  menzilde gerileme yok VE devir menzili/süresi kötüleşmez VE `circle`
  regresyonunda en yakın menzil %30'dan fazla kötüleşmez.
- Ö-D2 **girmez** eğer: bunlardan biri sağlanmazsa.
- Bozma varsa GİZLENMEZ, ölçüsüyle raporlanır; kararı kullanıcı verir.

## n ve denge
8 uçuş `duz`+kaçamak (4v4, her kolda 2 yatay + 2 capraz, dönüşümlü)
+ 4 uçuş `circle` regresyonu (2v2) + 2 uçuş `duz`+`yok` tabanı (1v1).

## YAPISAL GARANTİ (G17e)
Hız vektörü `vx = ff_x + KP_H·ex + KD_H·de` ile konum hatasından
hesaplanır ve `cmd_yaw` ONDAN SONRA gelir; `limit_acceleration` girdisinde
cmd_yaw YOK. Kaynak sırası birim testiyle denetleniyor.
G17b: borç < 25° iken komut bit bit aynı → sakin takip ETKİLENMEZ.
