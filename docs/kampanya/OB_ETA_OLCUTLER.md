# Ö-B (η tetikli) — ölçütler (KOŞMADAN ÖNCE yazıldı, 2026-08-15)

## Kullanıcı itirazı (§5.5, birebir)
> "burası simülasyon ortamı ve hedef aracın nasıl gideceğini ben
> belirliyorum. Yarışma sırasında belki farklı şekilde gidecek... bizim
> simde en iyi çalışan algoritmayı kurmayalım, her koşulda en iyi çalışan
> algoritmayı kurmaya çalışalım."

HAKLI. `|eps_yaw| > 40°` gibi bir eşik simdeki hedefin hızına/yarıçapına
göre ayarlanır — aşırı uydurma. Bu sürümde tetik BOYUTSUZ:

    η = V · λ̇ / (g · tanθmax)      eşik = 1  (fiziğin sınırı)

η<1 → bu hızda dönebiliriz · η>1 → dönemeyiz, tek çare hızı düşürmek.
Aktifken hedef hız da ayarsız: **V = a_max/λ̇** (η'yı 1'e getiren hız),
taban KOSE_V_MIN = 8 m/s (havada kalmayı önler).

Ayarlanabilir kalan tek sayılar: histerezis bandı (1.0/0.6), süre tavanı
(2.5 s), rampa (8 m/s²). Üçü de DİNAMİK parametre, senaryo parametresi değil.

## Ölçülen η (kontrol koşuları, kutulu kareler)
    duz+kaçamak  medyan 0.17 · η>1 karelerin %11'inde
    circle       medyan 1.76 · η>1 karelerin %75'inde
    aggressive   medyan 0.73 · η>1 karelerin %37'sinde
Ayırma gücü η>1'de 6.8 kat (eps_yaw>30° ile 3.7 kattı).

## KAZANIM NEREDE ÖLÇÜLÜR (§5.13 — tasarım zarfı)
Ö-B "yayda yavaşla, DÜZ KESİMDE hızlan" çevrimidir. Kazanım, düz bacak +
keskin köşe içeren senaryoda ölçülür: **`square`**.
`circle`'da düz kesim YOKTUR → orada kazanım ARANMAZ, yalnız regresyon.

## BİRİNCİL ÖLÇÜTLER (yalnız `square`)
1. **En yakın menzil medyanı (m)** — KÜÇÜK kazanır.
2. **İSABET.**

## ZORUNLU EŞ ÖLÇÜTLER (§5.2 / §5.13)
1. `kose` mekanizma sütunu — `square`'de aktiflik oranı. %5'in altındaysa
   ölçüm özelliği SINAMIYORDUR, koşu geçersiz.
2. `eta` sütunu — dağılımı raporlanır (tetiğin gerçekten fizikten geldiğinin
   kanıtı; kolda η>1 oranı ile aktiflik oranı tutarlı olmalı).
3. **Görsel temas oranı** — yavaşlarken hedefi kaybediyor muyuz.
4. Terminal mandalının kurulup kurulmadığı.

## REGRESYON (§5.10) — kazanım aranmaz, BOZMA aranır
  `duz`+kaçamak : η medyanı 0.17 → mekanizma neredeyse ölü olmalı.
                  Aktiflik %15'i aşarsa TASARIM HATASI.
  `circle`      : zarf dışı. Ö5'in "yavaş kal" kusuru tekrarlıyor mu diye
                  bakılır (aktiflik %60'ı aşarsa uyarı).

## KARAR KURALI (önceden ilan, DEĞİŞTİRİLEMEZ)
- Ö-B **GİRER** eğer: `square`'de en yakın menzil belirgin düşer veya isabet
  artar **VE** `duz`+kaçamakta gerileme yok **VE** `circle`'da Ö5'inkine
  benzer bir çöküş (en yakın menzil %30+ kötüleşme) yok.
- Ö-B **GİRMEZ** eğer: `square`'de kazanım yoksa, ya da regresyon varsa,
  ya da mekanizma kapısı geçilmezse.
- Bölünmüş sonuç → ölçüt DEĞİŞTİRİLMEZ, kullanıcıya (§5.6).

## n ve DAĞILIM — 16 uçuş
  square        : kontrol 4, Ö-B 4   (KAZANIM — dönüşümlü)
  duz + kaçamak : kontrol 2, Ö-B 2   (regresyon; yatay + capraz)
  circle        : kontrol 2, Ö-B 2   (regresyon; zarf dışı)
Env + TAM RESTART. Sistem bugünkü hâliyle (K-V2 açık, Ö-M açık, VTERM 16).

## ETKİ ALANI TABLOSU (§5.10)
| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Düz takip | η düzde 0.17 → ölü olmalı | `duz` regresyon + aktiflik oranı |
| Kaçamak tepkisi | kaçamakta λ̇ fırlar → devreye girer (İSTENEN) | `duz`+kaçamak |
| Sürekli dönüş | η>1 %75 → Ö5 gibi yavaş kalma riski | `circle` regresyon + B67 |
| Terminal hücum | terminalde v_los kısılabilir | terminal süresi + en yakın menzil |
| Yetişme | yavaşlarken hedef uzaklaşır | görsel temas + terminal kurulma |
