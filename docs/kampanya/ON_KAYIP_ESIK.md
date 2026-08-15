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

## 9 · RAPORDAN ÖNCE ÜÇ SORU (§5.8) — koşu sonrası doldurulacak

1. Özellik çalıştı mı? (faz sonu sayacı 19'u aştı mı) → …
2. Ölçütüm kötü bir sebeple mi iyileşti? (İK-1 ne diyor) → …
3. n kaç, bu n'de hüküm kurulur mu? → …
