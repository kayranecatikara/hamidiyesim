# Ö-B · KÖŞE DÖNÜŞÜ — ölçütler (KOŞMADAN ÖNCE yazıldı, 2026-08-15)

## Özellik
`AVCI_IBVS_KOSE=9` — |eps_yaw| > 60° iken hız 9 m/s'ye kısılır;
< 25°'ye düşünce VEYA 2.5 s dolunca rampa (8 m/s²) ile açılır.
**Rampa bitmeden yeniden girilmez** (B65'in yakaladığı kusurun kilidi).

## Neden — ölçülmüş geometri
R = V²/(g·tanθ). 18 m/s @45° → R = 33 m. Hedef 15 m/s @60° → R = 13 m.
Drone 2.5 kat geniş yay çiziyor ve dairede içeri giremiyor:
**circle senaryosunda 9 koşuda 0 isabet** (Ö-M kampanyası), terminal
mandalı kurulamıyor (terminal süresi 0.3-4.8 s).
9 m/s'de R = 8.3 m — hedefin çemberinden DAR. Açısal hız: hedef 15/13 =
66°/s, biz 9/8.3 = 62°/s → yakın.

## Ö11'den farkı (Ö11 elendi ve KOMPLE SİLİNDİ, §5.12)
Ö11: `kapanma < −5 ∧ |eps_yaw| > 45°`, DURUMSUZ. Ölçüldü: uçuş başına
0.4-0.6 s ateşledi, daire regresyonunda en yakın menzili %65 kötüleştirdi.
Ö-B: histerezis (60→25°) + süre tavanı (2.5 s) + çıkış rampası + rampa
kilidi. B65: sürekli 70°'de bile görev döngüsü %62, 4 yay çevrimi.

## BİRİNCİL ÖLÇÜT
**`circle` senaryosunda en yakın menzil medyanı (m).** KÜÇÜK kazanır.
Taban (Ö-M kampanyası, A kolu): 2.01 m, isabet 0/3.
İkincil birincil: `circle`'da İSABET (şu an 0/9).

## ZORUNLU EŞ ÖLÇÜTLER (§5.2)
1. `kose` mekanizma sütunu (§5.1) — deney kolunda aktif kare oranı.
   Sıfırsa koşu GEÇERSİZ. Beklenen: dairede %30-60.
2. **Görsel temas oranı** — yavaşlarken hedefi kaybediyor muyuz.
3. Terminal mandalının kurulup kurulmadığı (6.4 m'ye inebiliyor muyuz).
4. **`duz`+kaçamak REGRESYONU** — Ö-B düz uçuşta hiçbir şeyi bozmamalı;
   orada |eps_yaw| nadiren 60°'yi aşar, yani mekanizma neredeyse ölü
   olmalı. Aktif kare oranı düzde %5'i aşarsa TASARIM HATASI demektir.

## KARAR KURALI (önceden ilan, DEĞİŞTİRİLEMEZ)
- Ö-B **GİRER** eğer: `circle`'da en yakın menzil belirgin düşer (veya ilk
  isabet gelir) **VE** `duz`+kaçamakta en yakın menzil/isabet gerilemez.
- Ö-B **GİRMEZ** eğer: dairede kazanım yoksa, ya da düzde gerileme varsa,
  ya da mekanizma kapısı geçilmezse.
- Bölünmüş sonuç → ölçüt DEĞİŞTİRİLMEZ, kullanıcıya (§5.6).

## n ve DAĞILIM (§5.4, §5.9) — 16 uçuş
  circle        : kontrol 4, Ö-B 4   (asıl hedef, dönüşümlü)
  duz + kaçamak : kontrol 2, Ö-B 2   (regresyon; yatay+capraz)
  aggressive    : kontrol 2, Ö-B 2   (ikinci manevra rejimi)
Env + TAM RESTART. Sistem Ö-M ile (mandal 20 m, VTERM 16) — bugünkü hâli.

## ETKİ ALANI TABLOSU (§5.10)
| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Düz takip | 60° eşiği düzde nadir aşılır → ölü olmalı | `duz` regresyon + mekanizma oranı |
| Kaçamak tepkisi | kaçamak anında eps_yaw 60°'yi aşar → devreye girer | `duz`+kaçamak |
| Yetişme (kapanma) | yavaşlarken hedef uzaklaşabilir | görsel temas + terminal kurulma |
| Terminal hücum | terminalde v_los = V_TERMINAL, Ö-B bunu `min` ile kısabilir | terminal süresi + en yakın menzil |
| Kurtarma bekçisi | daha yavaş → daha az savrulma beklenir | KURTARMA olay sayısı |
