# Ö-D KAMPANYASI — ölçütler (KOŞMADAN ÖNCE yazıldı, 2026-08-13)

## Özellik
`AVCI_IBVS_FOV=25` — burun komutu gövdeden en fazla 25° önde olabilir:
`cmd_yaw = iris_yaw + clamp(cmd_yaw − iris_yaw, ∓25°)`. Slew'den SONRA.

## Neden (Ö12'nin çürütülmesinden doğdu, 16 uçuş)
Olay kayıtları W02 ve U05: hedef 2-3 m'de kadraj kenarından çıkıyor → kutu
kayıp → kör hücum → araç gövdeden 40° önde olan bayat komutu kovalarken
gerçek yaw 550°/s → bekçi kesiyor. Ö12 komut DOYMASINI çözdü (%23→%3) ve
olay yine oldu ⇒ sebep doyma değil, kayıp anındaki BORÇ.

## BİRİNCİL ÖLÇÜT (kullanıcının 2026-08-13 talimatı)
> "daha iyi şekilde takip edebilecek ve daha kısa sürede vurabilecek"

**Tetikten sonra yeniden saldırıya geçme süresi** = kaçamak tetiğinden
sonra kapanma hızının yeniden negatife (kapanıyor) dönmesine kadar geçen
süre. 20 Hz bbox logundan, `kapanma` sütunu. KÜÇÜK olan kazanır.
- Geçerlilik eşi (§5.2): **görsel temas oranı**. Hedefi hiç göremeyen
  koşuda `kapanma` hesaplanamaz → ölçüt sahte iyi görünür. Temas < %30
  olan koşu ölçütten DÜŞÜLÜR.

## İKİNCİL ÖLÇÜTLER
1. `fov_kis` mekanizma sütunu (§5.1 kapısı). Deney kolunda sıfırsa
   o koşu GEÇERSİZ, veri noktası sayılmaz.
2. KURTARMA yaşayan koşu sayısı ve kare sayısı (Ö12'nin birincili).
3. İSABET, en yakın menzil — gerileme kontrolü.
4. Görsel temas kesinti sayısı ve toplam süresi.

## KARAR KURALI (önceden ilan)
- Ö-D **girer** eğer: yeniden saldırı süresi kısalır VE isabet/en yakın
  menzilde gerileme yok.
- Ö-D **girmez** eğer: yeniden saldırı süresi kötüleşir, ya da isabet
  gerilemesi var, ya da mekanizma hiç bağlamaz (`fov_kis` = 0).
- Sonuç bölünmüşse ölçüt DEĞİŞTİRİLMEZ, kullanıcıya götürülür (§5.6).

## n ve denge (§5.4, §5.9)
8 uçuş: 4v4, her kolda 2 yatay + 2 capraz. Dönüşümlü K, Ö, K, Ö...
Sonuç umut vericiyse n=8/kola çıkarılır (Ö12'de olduğu gibi).

## REGRESYON (§5.10)
**YAPISAL GARANTİ var — test B75:** Ö-D döngüde, slew'den SONRA uygulanıyor.
Hız vektörü `komut()` içinde `hiz_yonu`ndan hesaplanıyor ve bu sınırdan
GEÇMİYOR. 162 girdi kombinasyonunda `komut()` çıktısı (vx, vy, vz, yaw
hedefi) bit bit aynı, maks sapma 0.00e+00. Uçuş yolu DEĞİŞEMEZ.
Ayrıca B72: sakin takipte (borç < 25°) komut bit bit aynı → `duz`+`yok`
tabanında ve `circle`'da hiçbir şey değişemez.
⇒ Bu yüzden ayrı circle regresyonu GEREKMİYOR (matematiksel kanıt daha
güçlü — CLAUDE.md §5.10).
