# Ö-J · ARAÇ ZARFI (ivme / jerk / yatış) — ölçütler (KOŞMADAN ÖNCE, 2026-08-15)

## Neden — ölçüm zinciri
1. Kapalı çevrim kinematik model, aynı sınırlarla istasyona 3.4 m'ye
   oturuyor; gerçekte 75 m. ⇒ GEOMETRİ darboğaz DEĞİL.
2. Güdümün istediği yanal ivme 2.5 m/s², aracın yaptığı 3.2 m/s²,
   gereken 6.8 m/s². ⇒ Araç istenenden fazlasını veriyor; komut da
   düzgün (0.5°/adım, salınım yok).
3. Gerçekleşen yanal ivme p90 = **8.5 m/s² ≈ WPNAV_ACCEL 8.0** —
   ARAÇ TAVANINA DAYANIYOR. Güdümün MAX_ACCEL'i 12, yani hiç bağlamıyor.
4. Jerk 4 m/s³ ile 6.8 m/s²'ye ulaşmak **1.7 s**; dairede gereken ivmenin
   yönü 21.5°/s dönüyor → 1.7 s'de 37° kayıyor. Araç dönüşü kurmayı
   bitiremeden hedef yön değişiyor. Yer hızı 18 → 9.8 m/s'ye çöküyor.

⇒ Dört kampanyadır HIZ kolunu çevirdik (Ö5, Ö11, Ö-B×2), dördü de kaybetti.
Darboğaz hız değil, komutun UYGULANMA HIZI.

## KOLLAR (araç parametreleri; kod değişikliği YOK)
  A · TABAN : WPNAV_ACCEL 800 · WPNAV_JERK 4  · ANGLE_MAX 4500
              → etkin tavan 8.0 m/s², kurulması 1.7 s
  B · Ö-J   : WPNAV_ACCEL 1500 · WPNAV_JERK 15 · ANGLE_MAX 4500
              → sınır ARACIN değil FİZİĞİN: yatış 45° = 9.81 m/s², 0.45 s
  C · Ö-J+  : B + ANGLE_MAX 5500
              → tavan 14.0 m/s². Ö6 bunu TEK BAŞINA denemiş ve mekanizma
                kapısından geçememişti (araç 38-40°'de kalıyordu); şimdi
                sebebini biliyoruz — jerk bağlıyordu.

## BİRİNCİL ÖLÇÜTLER (`circle` — hedeflenen rejim)
1. **En yakın menzil medyanı (m)** — KÜÇÜK kazanır. Taban ~2.7-5.1 m.
2. **İSABET** — şu an TÜM kollarda 0/9. Tek isabet bile büyük sinyaldir.

## MEKANİZMA KAPISI (§5.1) — ZORUNLU
**Gerçekleşen yanal ivme medyanı** (hız vektörünün dönme hızı × hız,
gps logu konumlarından). Taban 3.2 m/s². B/C kolunda YÜKSELMEZSE
parametre yazılmamış demektir → koşu GEÇERSİZ.
İkinci kapı: **yer hızı** (taban dairede 9.8 m/s) yükselmeli.

## ZORUNLU EŞ ÖLÇÜTLER (§5.2)
1. **KURTARMA olay sayısı** — daha agresif ivme savrulmayı artırabilir.
   Artarsa kazanım sahte olabilir.
2. Görsel temas oranı.
3. İrtifa kaybı / MOT doyumu — 55°'de itki maliyeti 1.74×.

## REGRESYON (§5.10)
`duz`+kaçamak — sistemin İYİ olduğu rejim. İsabet 2/2 ve en yakın menzil
~0.4-1.4 m; bozulmamalı. Daha yüksek jerk terminal yaklaşmayı sertleştirip
dikey ıskayı artırabilir → dikey ıska işareti de raporlanır.

## KARAR KURALI (önceden ilan, DEĞİŞTİRİLEMEZ)
- Ö-J **GİRER** eğer: `circle`'da en yakın menzil belirgin düşer (veya ilk
  isabet gelir) **VE** `duz`'da isabet/en yakın menzilde gerileme yok
  **VE** KURTARMA olay sayısı artmaz.
- **EMNİYET (üstün):** bir koşuda bile araç düşerse o kol ANINDA elenir.
- B ve C ayrı değerlendirilir: C, B'den iyi değilse ANGLE_MAX açılmaz.
- Bölünmüş sonuç → ölçüt DEĞİŞTİRİLMEZ, kullanıcıya (§5.6).

## n ve DAĞILIM — 18 uçuş
  circle        : A 4, B 4, C 4  = 12   (BİRİNCİL)
  duz + kaçamak : A 2, B 2, C 2  =  6   (REGRESYON; yatay+capraz)
Dönüşümlü A,B,C,A,B,C... Her koşuda TAM RESTART + PARAM_SET.

## ETKİ ALANI TABLOSU (§5.10)
| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Dairede kapanma | asıl hedef | `circle` birincil |
| Düz takip / terminal | daha sert ivme, dikey kanal | `duz` regresyon + dikey ıska |
| Kurtarma bekçisi | agresif ivme → savrulma | KURTARMA olay sayısı |
| İtki bütçesi (yalnız C) | 55°'de 1.74× itki | irtifa kaybı |
| Dikey kanal | WPNAV_ACCEL_Z'ye DOKUNULMADI | yapısal: değişmez |
