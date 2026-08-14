# V_TERMINAL 16 — MANEVRA REGRESYONU (ölçütler, KOŞMADAN ÖNCE, 2026-08-14)

## Kullanıcının endişesi (birebir, §5.5)
> "bu hızı 16 yapmamız hedef aracın manevra yaptığı senaryolarda takibi
> çok kötüleştirebilir ona dikkat etmeliyiz"

## Neden ciddi — KODDAN ÇIKAN MEKANİZMA
`bbox_ibvs.py:1054` — `terminal_mandal` bir kez True olunca görsel faz
boyunca AÇILMIYOR. Yani `V_TERMINAL` yalnız son 6.4 m değil, **ilk kez
6.4 m'ye indikten sonraki TÜM görsel faz** için geçerli.
⇒ Iskadan sonra 16 m/s ile kovalıyoruz; hedef p50 15.1 m/s → kapanma
0.9 m/s. Hedef hızlanırsa (arşiv p99 = 27.3 m/s) yetişmek imkânsız.
Dairede köşe kesmek daha yüksek hız ister; tavan bağlar.

## BİRİNCİL ÖLÇÜT
**En yakın menzil medyanı (m)** — manevralı hedefte yaklaşabiliyor muyuz.
KÜÇÜK kazanır. 16 kolunda BÜYÜRSE gerileme var demektir.

## ZORUNLU EŞ ÖLÇÜTLER (§5.2)
1. **KAPANMA YETENEĞİ** — terminal kilidi kurulduktan sonra açılan en
   büyük menzil ve 10 m'nin altına geri dönebilme. Asıl risk bu.
2. **Görsel temas oranı** — hedefi tutabiliyor muyuz.
3. İSABET.

## KARAR KURALI (önceden ilan)
- 16 **manevrada GÜVENLİ** sayılır: en yakın menzil %30'dan fazla
  kötüleşmez VE temas gerilemez VE 10 m altına geri dönüş oranı düşmez.
- 16 **manevrada RİSKLİ**: bunlardan biri bozulursa. O durumda seçenek
  "16'yı yalnız düz uçuşta uygula" (menzil/rejim kapılı) olur — ama bu
  YENİ bir özellik demektir, ayrı ölçülür.
- Bölünmüş sonuç → kullanıcıya (§5.6).

## n ve senaryo (§5.4, §5.9)
6 uçuş `circle` (3v3, dönüşümlü) — sürekli manevra, en sert sınav.
2 uçuş `aggressive` (1v1) — nokta kontrolü.
Kaçamak `yok` (senaryonun kendisi zaten manevra).
Env + TAM RESTART.

## ETKİ ALANI (§5.10) — bu kampanya neyi kapsıyor
| durum | kapsandı mı |
|---|---|
| düz uçuş + kaçamak | ✓ önceki kampanya (isabet 3/4→4/4) |
| sürekli manevra (circle) | ✓ BU KAMPANYA |
| rastgele agresif manevra | ✓ BU KAMPANYA |
| ıska sonrası yeniden yakalama | ✓ kapanma yeteneği ölçütü |
