# Ö-M · TERMİNAL MANDALINI MENZİLLE BIRAK — ölçütler (KOŞMADAN ÖNCE, 2026-08-15)

## Özellik
`AVCI_IBVS_TERM_BIRAK=20` — terminal mandalı, menzil 20 m'yi aşınca AÇILIR
ve seyir yasası (PI + V_TOPLAM_MAX 24 m/s) geri gelir.
Kilitlenme eşiği DEĞİŞMEDİ: kutu ≥ 25 px (≈6.4 m). Histerezis 6.4 ↔ 20 m.

## Neden — ölçülmüş ikilem
Mandal bir kez kilitlenince görsel faz boyunca açılmıyordu; V_TERMINAL
"son vuruş hızı" değil, 6.4 m'ye ilk inişten sonraki HER ŞEYİN hızıydı.
    duz + kaçamak : V_TERMINAL 16 DAHA İYİ (isabet 3/4→4/4, 71→52 s)
    circle/aggr   : V_TERMINAL 16 DAHA KÖTÜ (2.87→6.18 m, örtüşmeyen)
Tek sabit hız iki rejimi memnun edemiyor. Ö-M rejimleri AYIRIR.

## KOLLAR (3 kol — tek değişkenli değil, ama kasıtlı: ikilemi çözüyoruz)
  A · TABAN     : BIRAK=0,  VTERM=18   (bugünkü sistem)
  B · Ö-M       : BIRAK=20, VTERM=16   (aday: yakında yavaş, uzakta hızlı)
  C · yalnız 16 : BIRAK=0,  VTERM=16   (Ö-M'siz 16 — kıyas çizgisi,
                  dairede kötüleştiği ZATEN ölçüldü; burada teyit)
Kol C, Ö-M'nin katkısının 16'dan mı mandaldan mı geldiğini ayırır.

## BİRİNCİL ÖLÇÜTLER
1. **En yakın menzil medyanı (m)** — senaryo İÇİNDE kıyaslanır (§5.9).
2. **İSABET.**

## ZORUNLU EŞ ÖLÇÜTLER (§5.2)
1. `mandal_birakma` sayısı (§5.1 mekanizma kapısı) — B kolunda sıfırsa
   o koşu GEÇERSİZ. Logdan "[IBVS] ⚑ terminal mandalı BIRAKILDI".
2. Görsel temas oranı.
3. Terminal fazda geçen süre — Ö-M'nin bunu KISALTMASI beklenir
   (uzaktayken artık terminal sayılmıyoruz); kısalması BAŞARIDIR,
   ama en yakın menzil bozulmadan.
4. 10 m'nin altına dönebilme (kapanma yeteneği).

## KARAR KURALI (önceden ilan, DEĞİŞTİRİLEMEZ)
- Ö-M **GİRER** eğer: `duz`+kaçamakta 16'nın kazanımı KORUNUR (isabet ve/veya
  en yakın menzil taban A'dan iyi) **VE** `circle`/`aggressive`'de en yakın
  menzil taban A'dan %30'dan fazla kötüleşmez.
- Ö-M **GİRMEZ** eğer: manevra senaryolarında C koluna benzer gerileme
  görülürse (yani mandal bırakma sorunu çözmüyorsa) ya da düz uçuşta
  kazanım kaybolursa.
- Bölünmüş sonuç → ölçüt DEĞİŞTİRİLMEZ, kullanıcıya (§5.6).

## n ve DAĞILIM (§5.4, §5.9) — toplam 21 uçuş
  duz + kaçamak (yatay/capraz dönüşümlü) : A 3, B 3, C 3  =  9
  circle                                  : A 3, B 3, C 3  =  9
  aggressive                              : A 1, B 1, C 1  =  3
Kollar senaryo İÇİNDE kıyaslanır. Env + TAM RESTART her koşuda.

## ETKİ ALANI TABLOSU (§5.10)
| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Düz uçuşta 16'nın kazanımı | mandal erken bırakılırsa son yaklaşma 24 m/s'ye döner | `duz`+kaçamak, A/B/C |
| Manevrada kapanma | mandal bırakınca tam hız → daire düzelmeli | `circle`, A/B/C |
| Kör hücum | mandal açılınca pencere kapanır | birim test B75 + uçuşta ıska sınıfı |
| Mod titremesi | 6.4↔20 m bandı dar olursa | `mandal_birakma` sayısı: koşu başına 1-3 beklenir, 10+ ise titreme |
| Faz geçişi (GPS↔görsel) | mandal `supervisor`'a görünmez | yapısal: dokunulmadı |
