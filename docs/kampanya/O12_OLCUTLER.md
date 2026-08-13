# Ö12 KAMPANYASI — ölçütler (KOŞMADAN ÖNCE yazıldı, 2026-08-13)

## Özellik
`AVCI_IBVS_YAW_MENZIL=15` — yaw slew tavanı menzille kısılır:
`tavan = 120°/s · clamp(R/15, 0.35, 1)` → 20 m'de 120, 8 m'de 64, 3 m'de 42 °/s.

## Kullanıcının hedefi (§5.5 — birebir alıntı)
> "bizim drone arada sırada böyle manevra limitleri zorlandığında yada hedef aracı
> kaçırdığında yada hedef aracı pas geçtiğinde vs. böyle bazen mal mal hareketler
> yapıp kendi etrafında dönmeye başlıyor çok hızlı yaw yapıyor ve olduğu yerde kalıp
> bi 15 saniyede düzeliyor bu süre zarfında hedef araç çok uzaklaşmış oluyor"

## BİRİNCİL ÖLÇÜT
**KURTARMA olayı yaşayan koşu sayısı** ve **toplam KURTARMA karesi.**
Küçük olan kazanır. Kaynak: 20 Hz bbox logu `kurtarma` sütunu.
- Geçerlilik eşi (§5.2): görsel temas oranı. Hedefi hiç göremeyen koşuda
  güdüm komut üretmez → KURTARMA da tetiklenmez, ölçüt sahte iyi görünür.
  Temas < %30 olan koşu ölçütten DÜŞÜLÜR.

## İKİNCİL ÖLÇÜTLER
1. `yaw komut doyma oranı` (mekanizma sütunu — §5.1 kapısı). Deney kolunda
   tavan hiç bağlamadıysa o koşu GEÇERSİZ.
2. **YENİ (kullanıcının 2026-08-13 talimatı): yeniden saldırıya geçme süresi.**
   Tetikten sonra kapanma hızının yeniden negatife dönmesine kadar geçen süre.
   Kullanıcı: *"daha iyi şekilde takip edebilecek ve daha kısa sürede vurabilecek"*
   → küçük olan kazanır. Bu ölçüt Ö-D/Ö-B/Ö-C için de BİRİNCİL olacak.
3. İSABET, en yakın menzil — gerileme kontrolü.
4. SAĞA AŞIM — U kampanyasındaki yan gözlemin (41.3 → 69.4 m, n=2v2) doğrulaması.

## KARAR KURALI (önceden ilan)
- Ö12 **girer** eğer: KURTARMA koşu sayısı azalır VE isabet/en yakın menzilde
  gerileme yok VE yeniden saldırı süresi kötüleşmez.
- Ö12 **girmez** eğer: KURTARMA farkı yok, ya da isabet gerilemesi var,
  ya da SAĞA AŞIM %30'dan fazla kötüleşir.
- Sonuç bölünmüşse ölçüt DEĞİŞTİRİLMEZ, kullanıcıya götürülür (§5.6).

## n ve denge (§5.4, §5.9)
Mevcut: U01-U08 = 4v4 (kol başına 2 yatay + 2 capraz).
Bu kampanya: **8 uçuş daha** (W01-W08), kol başına 2 yatay + 2 capraz.
→ TOPLAM n=8/kol, tür dağılımı eşit (4 yatay + 4 capraz her kolda).
Sıra dönüşümlü: K, Ö, K, Ö, K, Ö, K, Ö.

## YAPISAL GARANTİ (§5.10)
Ö12 yalnız BURNU (yaw) etkiler. Hız vektörü `hiz_yonu`'ndan hesaplanır ve bu
sınırdan GEÇMEZ. Birim test B67: 32 girdi kombinasyonunda `komut()` çıktısı
(vx, vy, vz, yaw) bit bit aynı. Uçuş yolu DEĞİŞEMEZ.
⇒ `circle` regresyonu bu yüzden GEREKMİYOR — matematiksel kanıt daha güçlü.
