# Ö-K · KÖR DEVAM — ölçütler (KOŞMADAN ÖNCE yazıldı, 2026-08-15)

## Kullanıcı kararı (§5.5)
> "eğer simde uçurmadan bazı veriler üzerinde test yaparsak ve bizi
> yanıltırsa belki çok işe yarayacak bir özelliği hiç denemeden çöpe atmış
> oluruz"
Bu, CLAUDE.md §2'nin kendi kuralı: çevrimdışı replay yalnız HİPOTEZ üretir.
Bu yüzden dropout süre dağılımını ölçüp elemek yerine DOĞRUDAN UÇULUYOR.

## Özellik
`AVCI_IBVS_KOR=1` — kutu kaybolunca komutu dondurmak yerine son ölçülen
LOS dönüş hızıyla (λ̇) nişanı döndürmeye devam et. Hız vektörü + burun
birlikte döner; BÜYÜKLÜK değişmez. Toplam döndürme 40° ile sınırlı.

## Teşhis zinciri (docs/kampanya/DAIRE_TESHIS.md)
1. Dairede kuyruğa hiç yerleşemiyoruz — 60 m içinde 0-30° kuyrukta SIFIR kare
2. 50-100 m'de takılıyoruz; orada tespit %11-16 (0-25 m'de %62-98)
   ⚠ aspect açısının tespite etkisi YOK (ölçüldü) → dedektör suçlu değil
3. Ara sıra 10 ardışık tespit denk gelip görsel faza geçiliyor
4. Kutu hemen kayboluyor; komut 1 s DONUYOR → nişan 22° kayıyor, 16 m
   yanlış yöne uçuluyor. Toplam 67 s bayat komut (uçuşun 1/3'ü)
5. Hedefin ÖNÜNE düşüyoruz (kuyruk 130-170°), mesafe 85-120 m'ye açılıyor
6. GPS yeniden yaklaşıyor ama 6 s sonra döngü baştan
   (dairede GPS fazı 50 parça / medyan 6 s · duzde 7 parça / medyan 12 s, max 41 s)

## BİRİNCİL ÖLÇÜTLER (`circle`)
1. **Faz geçişi sayısı** (bbox log dosyası adedi / uçuş). Taban 19-30.
   KÜÇÜK kazanır — zincirin kırıldığının doğrudan göstergesi.
2. **En yakın menzil medyanı.** Taban 2.7-5.1 m.

## MEKANİZMA KAPISI (§5.1)
`kor_don_deg` sütunu — kör karelerde uygulanan döndürme. Deney kolunda
sıfırsa koşu GEÇERSİZ. Beklenen: kör karelerin çoğunda 5-40°.

## ZORUNLU EŞ ÖLÇÜTLER (§5.2)
1. **Görsel fazda kutu oranı** (taban circle %34-39). Yükselmeli.
2. **Kuyruk açısı** — 60 m içinde 0-30° kuyrukta kare sayısı (taban SIFIR).
3. İSABET (taban circle 0/9).
4. KURTARMA olay sayısı — döndürme aracı savurabilir.

## REGRESYON (§5.10)
`duz`+kaçamak: λ̇≈0 olduğu için mekanizma ÖLÜ olmalı (birim test B63).
`kor_don_deg` düzde ~0 olmalı; değilse tasarım hatası. İsabet 2/2 ve en
yakın menzil bozulmamalı.

## KARAR KURALI (önceden ilan, DEĞİŞTİRİLEMEZ)
- Ö-K **GİRER** eğer: `circle`'da faz geçişi belirgin azalır VE (en yakın
  menzil iyileşir VEYA kutu oranı yükselir) VE `duz`'da gerileme yok.
- Ö-K **GİRMEZ** eğer: faz geçişi azalmazsa, ya da düzde gerileme varsa,
  ya da mekanizma kapısı geçilmezse.
- Bölünmüş sonuç → ölçüt DEĞİŞTİRİLMEZ, kullanıcıya (§5.6).

## n ve DAĞILIM — 12 uçuş
  circle        : kontrol 4, Ö-K 4  (BİRİNCİL, dönüşümlü)
  duz + kaçamak : kontrol 2, Ö-K 2  (REGRESYON; yatay + capraz)
Env + TAM RESTART.

## ETKİ ALANI TABLOSU (§5.10)
| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Daire/manevra takibi | asıl hedef | `circle` birincil |
| Düz takip | λ̇≈0 → ölü olmalı | `duz` regresyon + kor_don_deg |
| Kaçamak tepkisi | kaçamakta λ̇ fırlar, kutu da kaybolur → devreye girer | `duz`+kaçamak |
| Terminal kör hücum | AYRI dal (TERM_KOR), Ö-K oraya DOKUNMUYOR | yapısal: değişmez |
| Kurtarma bekçisi | döndürme savrulma üretebilir | KURTARMA olay sayısı |
