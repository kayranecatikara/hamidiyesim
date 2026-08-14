# V_TERMINAL 18 → 16 m/s — ölçütler (KOŞMADAN ÖNCE yazıldı, 2026-08-14)

## Kullanıcının hipotezi (birebir alıntı, §5.5)
> "terminal fazda dronun hızını hedefin hızına yaklaştıralım — ondan sadece
> bir miktar hızlı olsun, üzerine atılmayalım. Böylece kapanma hızı düşer,
> gecikmenin ürettiği hata küçülür, son metrelerde düzeltme yapacak kare
> sayımız artar."

## Değişken
`AVCI_IBVS_VTERM` : 18.0 (KONTROL) ↔ 16.0 (DENEY). Kod değişikliği YOK —
anahtar G10'dan beri var. Terminal mandalı kutu 25 px'te (≈6.4 m) kilitlenir.

## Neden 16, 15 değil (ölçüme dayalı)
Hedefin hız dağılımı, 177.815 örnek: p10 14.9 / p50 15.1 / p90 15.2 m/s.
  V_TERMINAL=18 → hedef bizden hızlı: karelerin %3'ünde
  V_TERMINAL=16 → %6
  V_TERMINAL=15 → %84   ⛔ son 6.4 m hiç kapanmaz
16, hipotezi sınayacak kadar düşük (kapanma 2.9 → 0.9 m/s), sistemi
kilitlemeyecek kadar yüksek.

## BİRİNCİL ÖLÇÜTLER
1. **İSABET** (imha oranı). BÜYÜK kazanır.
2. **EN YAKIN MENZİL** medyanı (m). KÜÇÜK kazanır.

## ZORUNLU EŞ ÖLÇÜT (§5.2 geçerlilik eşi)
**TERMİNAL FAZDA GEÇEN SÜRE (s)** — `durum == TERMINAL` ve kutulu karelerin
toplamı. Kısalırsa "yaklaşamıyoruz" demektir; ıska sayısı iyileşse bile
sonuç GEÇERSİZ sayılır.
Ölçülmüş taban (103 koşu): VURAN 8.15 s · ISKALAYAN 3.98 s.
Yani terminal süresi isabetin en güçlü belirteci — bu ölçüt olmadan
"hız düşürmek işe yaradı" demek yanıltıcı olur.

## İKİNCİL ÖLÇÜTLER
1. Terminal fazda ölçülen kapanma hızı (mekanizma kapısı, §5.1).
   16 kolunda 18 koluna göre DÜŞMELİ; düşmediyse koşu geçersiz.
2. **Dikey ıska işareti** — kullanıcının "altından sıyrılıp uzaklaşıyoruz"
   gözlemi. `iris_alt − plane_alt` en yakın anda. (− = altta.)
3. Görsel temas oranı.

## KARAR KURALI (önceden ilan, DEĞİŞTİRİLEMEZ)
- 16 **GİRER** eğer: isabet artar VEYA en yakın menzil belirgin düşer —
  **VE** terminal süresi kısalmaz.
- 16 **GİRMEZ** eğer: terminal süresi kısalır (yaklaşamama), ya da isabet
  düşer, ya da mekanizma kapısı geçilmez (kapanma düşmediyse).
- Sonuç bölünmüşse ölçüt DEĞİŞTİRİLMEZ, kullanıcıya götürülür (§5.6).

## n ve denge (§5.4, §5.9)
8 uçuş: 4v4, her kolda 2 `yatay` + 2 `capraz`, dönüşümlü K, T, K, T...
Env + TAM RESTART (§4: otomatik kampanyada anahtar koşu boyunca değişmesin).

## ETKİ ALANI TABLOSU (§5.10)
| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Terminale girip yaklaşamama | kapanma 2.9 → 0.9 m/s | terminal süresi (eş ölçüt) |
| Kaçamak sonrası yeniden yakalama | terminal hızı düşünce geri kapatma yavaşlar | `duz`+kaçamak, aynı koşular |
| Dikey kesişim | `vz = −ṙ·tan(yükseliş)` — ṙ değişiyor | dikey ıska ölçütü |
| Seyir fazı | `V_TOPLAM_MAX` AYRI alan (24 m/s), dokunulmuyor | yapısal: değişemez |

## NOT — bu deneyin sınırı
Ölçüm zaten şunu söylüyor: terminal kapanma medyanı +0.98 m/s ve terminal
faz 6.3 s (189 kare). Yani "düzeltme penceresi çok dar" varsayımı mevcut
veriyle desteklenmiyor. Bu kampanya hipotezi ELEMEK ya da DOĞRULAMAK için
koşuluyor; beklentim girmeyeceği yönünde ama ölçüm karar verecek.
