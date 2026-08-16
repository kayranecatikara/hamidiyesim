# Ö-N · GÖRSELİ BIRAKMA EŞİĞİ (`KAYIP_M`) — ölçütler KOŞMADAN ÖNCE

**Tarih:** 2026-08-15 · **Durum:** ilan edildi, uçuş bekliyor
**Kill-switch:** `AVCI_HYBRID_KAYIP_M` (varsayılan 20 = bugünkü davranış)
**Panel:** `Ö-N · Görseli bırakma eşiği`

---

## 0 · KULLANICININ HEDEFİ (§5.5 — birebir alıntı)

> *"Daireyi bırakıp duz/square'e odaklanalım"* (2026-08-15)

ve daha önce:

> *"daha kısa sürede vurabilecek"*

Kare senaryosu bugün **0/8 isabet**, en yakın menzil medyanı **12.70 m** —
daireden (4.05 m) bile kötü. Yani "kareye odaklan" demek, sistemin **en kötü
olduğu yeri** düzeltmek demek. Birincil ölçüt bu cümleyi ölçmeli: karede
hedefe fiilen ne kadar yaklaşıyoruz.

---

## 1 · ÖZELLİK NE YAPIYOR

`KAYIP_M`, görsel fazın vazgeçme eşiği: bu kadar **ardışık kutusuz kare**
görülünce faz `'kayip'` döner ve GPS'e dönülür.

| kol | değer | kör kalınan süre |
|---|---|---|
| **K** (kontrol) | 20 kare | ~0.7-1.0 s |
| **N** (deney) | 40 kare | ~1.3-2.0 s |

Tek değişken. Başka hiçbir şeye dokunulmuyor.

---

## 2 · MEKANİZMA KAPISI (§5.1) — ölçmeden önce kanıtlanacak

Deney kolunda **faz sonu `kayip_sayac` değeri 19'u aşmalı**. Aşmıyorsa o
koşu veri noktası değil, GEÇERSİZ koşudur.

**Kapı zaten mevcut veriyle açık:** kare senaryosunun 163 görsel fazından
**156'sı (%96)** tam bu eşikte ölüyor. Eşik karede sistemin en çok bağlayan
kapısı — değiştirmek kesinlikle bir şeyi değiştirir.

Raporlanacak aktiflik oranı: her kolda faz sonu sayacının dağılımı.

---

## 3 · NEDEN 40 — hipotezin kaynağı

⚠ Aşağıdakiler **mevcut loglardan** üretildi. §2 gereği bunlar **hipotezdir,
kanıt değildir**; kabul kararını yalnız taze uçuş verir.

Kare senaryosunda görsel faz kesildikten sonra yeniden kurulma süresi:

| süre | oran |
|---|---|
| ≤ 1 s | %22 |
| ≤ 2 s | %27 |
| ≤ 3 s | %30 |
| medyan | 6.8 s |

Yani **her dört kesintiden birinde** kutu bir saniye içinde geri geliyor —
o kesinti gereksizdi. (Yeniden kurulma zaten `KILIT_N=10` ardışık tespit
istiyor, dolayısıyla 1 s'lik dönüşlerde gerçek tespit boşluğu eşiğin ancak
10-20 kare ötesindeydi. 40 bunu kapsar.)

Fazı **bitirmeyen** iç boşluklar: medyan 2, p90 10, max 36 kare. Normal
tespit boşlukları eşiğin çok altında — eşiği büyütmek onları etkilemez.

**Karşı taraf (niye zarar verebilir):** 20 kare fazladan kör uçuş, 18 m/s'de
~18 m bayat komut. Hedef gerçekten kaybolduysa GPS'e geç dönülür ve GPS
hedefin tam konumunu bilir. Karede fazların %73'ünde kutu 1 s içinde geri
GELMİYOR — onlar bu bedeli öder.

İki taraf da gerçek. Bu yüzden ölçülüyor.

---

## 4 · ÖLÇÜTLER

### BİRİNCİL

**En yakın menzil — kare senaryosu, koşu medyanı** (`olay.json → en_yakin`).

*§5.2 geçerlilik eşi:* "en yakın menzil" savrulup şans eseri yaklaşmayla da
iyileşebilir. Eşi normalde vuruş sınıfı (KONTROLLÜ/ŞANS) ama karede taban
0/8 — vuruş yok, sınıflandırılacak bir şey yok. Yerine **iki eş** konuyor:
görsel fazda kutu oranı (İK-1) ve salınım (İK-2). Birincil iyileşir ama
ikisinden biri belirgin gerilerse sonuç BÖLÜNMÜŞ sayılır.

### İKİNCİL

| # | ölçüt | kaynak | niye |
|---|---|---|---|
| İK-1 | **görsel fazda kutu oranı** (kutulu kare / toplam görsel kare) | bbox log 20 Hz | ⭐ birincilin geçerlilik eşi. Taban kare: %29. Eşiği büyütmek yalnız kör kare eklerse BU DÜŞER — o zaman kazanım sahtedir. |
| İK-2 | **salınım**: `cx` işaret değişimi/s (yalnız kutulu kare) | bbox log | §4 zorunlu. Geçerlilik eşi: görsel temas oranı %60 altındaysa GÜVENİLMEZ damgası. |
| İK-3 | **yatış** işaret değişimi/s ve \|yatış\| p90 | bbox log | §4 zorunlu |
| İK-4 | **kuyruk konisi**: 60 m içinde VE 0-30° kuyrukta kare sayısı | birlesik | kök neden ölçütü — asıl sorun buraya yerleşememek |
| İK-5 | isabet | olay.json | bilgi amaçlı. Taban 0/8; n=4/kol'da ayırt etmesi BEKLENMİYOR, tek başına karar vermez. |
| İK-6 | görsel faz sayısı ve süresi | bbox dosya adedi | ⚠ **MEKANİZMA SÜTUNU, BAŞARI ÖLÇÜTÜ DEĞİL.** Eşiği büyütünce faz sayısı zaten matematiksel olarak düşer — bunu "iyileşme" diye raporlamak totolojidir. |

⚠ İK-6'nın altı çizili: bu tuzağa §5.2'de daha önce düşüldü. Faz sayısının
düşmesi kazanım DEĞİL, mekanizmanın çalıştığının kanıtıdır.

---

## 5 · ETKİ ALANI TABLOSU (§5.10) — kodu yazmadan önce

| etkilenebilecek davranış | neden etkilenebilir | hangi senaryoda sınanır |
|---|---|---|
| **düz uçuş isabeti** (%65'lik çekirdeğimiz) | `KAYIP_M` her senaryoda geçerli; 2 s bayat komut son yaklaşmayı bozabilir | `duz` + `yatay` ×2, `duz` + `capraz` ×2 — **4 regresyon uçuşu** |
| **terminal kör hücum** | terminal fazda da kutu kayboluyor | ⭐ **YAPISAL GARANTİ** — TERM_KOR dalı eşik kontrolünden ÖNCE `continue` eder, kendi `TERMINAL_SURE` bütçesini kullanır. Birim testi **B64** sıralamayı sabitler. Uçuş gerekmez. |
| **KURTARMA'dan çıkış** (K-V2 ile etkileşim) | `kayip_sayac` KURTARMA sırasında da artıyor; eşiği büyütmek "kurtarma uzarsa GPS'e kaç" kapısını 20 kare geciktirir | `duz` regresyon koşularında KURTARMA süresi ayrıca raporlanır (K-V2 tabanı: 4.9 s) |
| **daire** | aynı tetik; taban zaten 0/45 | kullanıcı daireyi bıraktı → **kazanım orada aranmıyor** (§5.13). Ama regresyon olarak en yakın menzil kaydı raporlanır: mevcut kolun tabanı 4.05 m. Bu kampanyada daire UÇULMUYOR; karar kuralına girmez, açık borç olarak yazılır. |

**"Hedeflenen yeri iyileştirdi ama başka bir yeri bozdu mu?"** → düz
regresyon koşularının sonucuyla açıkça cevaplanacak.

---

## 6 · TASARIM ZARFI (§5.13)

**Bu özelliğin devreye girip tamamlanması için senaryoda ne bulunmalı?**
Görsel fazın kutu kaybederek ölmesi, ve kutunun kısa süre sonra geri
gelebilecek olması.

- **KAZANIM nerede ölçülür:** `kare` — fazların %96'sı eşikte ölüyor,
  kesintilerin %22'si 1 s içinde toparlanıyor. Zarf tam burası.
- **REGRESYON nerede ölçülür:** `duz` — orada fazlar uzun (medyan 236 kare)
  ve terminal ağırlıklı; eşiğin bozacak bir şeyi varsa orada görünür.

---

## 7 · KOŞU PLANI (§4 dönüşümlü, §5.9 tür-eşli)

**Kare (kazanım) — 8 uçuş, dönüşümlü:**

```
N01_K_kare (20)  N02_N_kare (40)  N03_K_kare  N04_N_kare
N05_K_kare       N06_N_kare       N07_K_kare  N08_N_kare
```

**Düz (regresyon) — 4 uçuş, dönüşümlü, kaçamak türü kollar arasında EŞİT:**

```
N09_K_duz_yatay (20)   N10_N_duz_yatay (40)
N11_K_duz_capraz (20)  N12_N_duz_capraz (40)
```

Tür dağılımı: her kolda 4 kare + 1 yatay + 1 capraz. **Eşit.**

n = 4/kol (kare) ve 2/kol (duz). §5.4: kare için hüküm kurulabilir; duz
regresyonu n=2/kol olduğu için **tek başına hüküm kurmaz** — yalnız "isabet
4/4 korundu mu" biçiminde EVET/HAYIR kapısı olarak kullanılır.

---

## 8 · KARAR KURALI — sonuca bakmadan ilan edildi

**GİRER** (varsayılan 40 olur), şu ÜÇÜ birden sağlanırsa:
1. Birincil (kare en yakın menzil medyanı) **iyileşir**, ve
2. İK-1 (kutu oranı) **5 puandan fazla düşmez**, ve
3. Düz regresyonda isabet **4/4 korunur**.

**ÇIKAR** (silinir, §5.12), şunlardan biri olursa:
- Birincil geriler, **veya**
- Düz regresyonda isabet 4/4'ün altına düşer.

**KULLANICIYA** (nötr/bölünmüş): yukarıdaki iki kümenin dışındaki her hâl —
özellikle birincil iyileşip İK-1'in 5 puandan fazla düştüğü durum.

---

## 9 · RAPORDAN ÖNCE ÜÇ SORU (§5.8)

1. **Özellik çalıştı mı?** EVET, tartışmasız. Kontrol kolunda 85 fazın 76'sı
   tam **19**'da, deney kolunda 50 fazın 43'ü tam **39**'da bitti. Mekanizma
   kapısı sonuna kadar açık.
2. **Ölçütüm kötü bir sebeple mi iyileşti?** EVET — ve bu sonucu belirledi.
   Birincil "iyileşti" göründü ama tam permütasyon testinde **p = 0.83**
   (gürültü). Geçerlilik eşi İK-1 ise 8.8 puan düştü ve bu **seyrelme değil
   gerçek kayıp**: mutlak kutulu kare 678 → 566 (−%17).
3. **n kaç, hüküm kurulur mu?** Kare n=4/kol → hüküm kurulur. Düz n=2/kol →
   yalnız EVET/HAYIR kapısı, hüküm kurulmaz (§5.4).

---

## 10 · SONUÇ (12 uçuş, 2026-08-15)

### Kare — kazanım senaryosu (n=4/kol)

| ölçüt | K20 (kontrol) | N40 (deney) | yorum |
|---|---|---|---|
| **BİRİNCİL** en yakın menzil, medyan | 13.82 m | 11.21 m | **p = 0.83 → GÜRÜLTÜ** |
| isabet | 0/4 | 0/4 | düz |
| **İK-1 kutu oranı** | %31.2 | %22.4 | −8.8 puan (ilan edilen sınır: 5) |
| İK-1b mutlak kutulu kare | 678 | 566 | **−%17 → seyrelme değil, gerçek kayıp** |
| **medyan mesafe** | **68.6 m** | **98.7 m** | **+%44 KÖTÜ** |
| **60 m içinde geçen süre** | **94 s** | **57 s** | **−%39** |
| İK-2 salınım (cx dgş/s) | 0.259 | 0.184 | ⚠ İK-1 düştüğü için GÜVENİLMEZ |
| İK-3 yatış p90 | 23.4° | 21.8° | fark yok denecek kadar az |
| İK-4 kuyruk konisi (oran) | ~%40 | ~%30 | kötüleşti |
| İK-6 faz sayısı *(mekanizma)* | 85 | 50 | totoloji, kazanım DEĞİL |

Ham değerler — en yakın menzil (m):
K20 `3.80, 12.95, 14.70, 15.93` · N40 `13.00, 4.06, 9.42, 14.91`.
Dağılımlar neredeyse tamamen örtüşüyor.

### Düz — regresyon (n=2/kol, hüküm kurulmaz)

| koşu | kol | isabet | en yakın | vuruş anı | medyan mesafe |
|---|---|---|---|---|---|
| N09 yatay | K20 | ıska | 0.83 m | — | 106.4 m |
| N11 capraz | K20 | **ISABET** | 0.28 m | **120.5 s** | 45.5 m |
| N10 yatay | N40 | **ISABET** | 1.73 m | 173.7 s | 62.9 m |
| N12 capraz | N40 | **ISABET** | 1.55 m | 196.4 s | 121.2 m |

Deney kolu 2/2, kontrol 1/2 — **ama** kullanıcının hedefi *"daha kısa sürede
vurabilecek"*: N40'ın vuruşları 174 ve 196 s'de, K20'ninki 120 s'de. n=2,
hüküm yok; yine de yön diğer tüm ölçütlerle aynı tarafta.

### Video bacağı (§2 adım 4)

`logs/on_N03_K_kare.mp4`, `logs/on_N06_N_kare.mp4`. Yaklaşma olaylarının
giriş → en yakın → çıkış dizileri incelendi:

- **N06 (N40) kare 58-61**, en yakın 10.1 m: 58'de ufuk ~40° yatık, hedef
  kadrajda YOK. 60'ta (en yakın an) kutu kadrajın **sağ üst köşesinde** —
  yani hedef yanımızdan geçiyor, önümüzden değil. 61'de tamamen kayboldu.
- **N03 (K20) kare 70-90**, 21 s'lik en uzun olay: 70'te hedef güven **0.30**
  (dedektör eşiği 0.35'e kıl payı), kutu ~12 px, ufuk ~25° yatık. 85'te güven
  0.36, hâlâ ~10 px, ufuk ~40°. 90'da hedef yok, ufuk ~45°.

**Çapraz doğrulama (§2 adım 6): çelişki YOK.** Loglar "kutu oranı %22-31,
medyan mesafe 69-99 m" diyor; kareler bunu doğruluyor — her iki kolda da
hedef, sert yatıştayken görülen sınırda güvenli minik bir leke. Ne K20 ne
N40 kuyruk takibine oturabiliyor.

---

## 11 · KARAR — ilan edilen kurala göre

- **GİRER değil:** koşul (2) çiğnendi (İK-1 8.8 puan düştü, sınır 5'ti).
- **ÇIKAR değil:** birincil gerilemedi (gürültü).
- → **KULLANICIYA.** Bu, §8'de adı konarak öngörülen hâlin ta kendisi:
  *"birincil iyileşip İK-1'in 5 puandan fazla düştüğü durum"*.

**Yapay zekânın önerisi: ÇIKAR (varsayılan 20'de kalsın, özellik silinsin).**
Gerekçe: birincil gürültü; sürekliliği ölçen HER ölçüt (medyan mesafe, 60 m
içi süre, mutlak kutulu kare, kuyruk konisi oranı) deney kolu aleyhine.

**Mekanizma yorumu (hipotez, ölçülmedi):** karede mesafeyi kapatan şey GPS
fazı. Görselde 2 s bayat komutla beklemek, kutu geri gelmediğinde (%73) o
GPS yaklaşmasını geciktiriyor; kazanılan %22'lik "az kalsın geri gelecekti"
vakası bu bedeli karşılamıyor.

---

## 12 · BU KAMPANYANIN ASIL BULGUSU — eşik değil, ölçüt

`kayip_kare_esik` sorunun kaldıracı değilmiş. Ama kampanya daha değerli bir
şey gösterdi: **"en yakın menzil" isabet üretmeyen bir senaryoda kötü bir
birincil ölçüt.** 240 s'lik bir uçuşun tek şanslı anını ölçüyor; K20'nin
`3.80` değeri ile N40'ın `4.06` değeri aynı uçuşların medyan mesafesi 69 m
ve 101 m iken üretildi.

**Öneri (§5.5 uyarınca kullanıcının hedefinden türetilerek):** kare/manevra
senaryolarında birincil ölçüt bundan sonra **60 m içinde geçen süre** ya da
**medyan mesafe** olsun — "daha kısa sürede vurabilecek" cümlesini bunlar
temsil ediyor, tek bir en-yakın-an değil.
