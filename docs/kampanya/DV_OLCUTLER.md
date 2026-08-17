# D-V · Dikey düzeltme tabanı — ölçüt ilanı

**Tarih:** 2026-08-16 (kampanya koşulmadan ÖNCE yazıldı, §4)
**Kol anahtarı:** `AVCI_IBVS_KAPANMA_MIN` · panel: "D-V · Dikey düzeltme tabanı"
**Kontrol:** 1.5 (mevcut) · **Deney:** 6.0

---

## Neden bu deney

248 uçuşluk kara kutu taraması (12-15 Ağustos) tek bir ekseni işaret etti:

| geçişin dikeyi | temas oranı |
|---|---|
| aynı seviye (±0.5 m) | **%24.9** |
| drone 2 m'den fazla ALTTA | **%0.4** |

60 kat fark. Dikey ıska baskın kayıp ekseni.

## Kök neden (ölçüldü, tahmin değil)

Terminal karelerinde (n=52 036, 15-16 Ağustos):

| menzil | `eps_elev` medyanı | yorum |
|---|---|---|
| 4-8 m | **−10.4°** | hedef 10° YUKARIDA |
| 2-4 m | −2.4° | |
| <2 m | **−20.3°** | temas anında 20° yukarıda |

Yani son yaklaşmada hedef sistematik olarak ÜSTÜMÜZDE ve kapanmıyor.

**Araç suçlu değil:** kara kutuda `PSCD.DVD` (komut) vs `PSCD.VD` (gerçekleşen)
takip hatası medyan **0.00 m/s**, p90 0.11. Komut neyse o uygulanıyor
(CLAUDE.md'nin şart koştuğu kontrol).

**Dikey tavan da suçlu değil:** `VZ_MAX_TERM=5` yalnız karelerin **%0.6**'sında
doyuyor.

**Suçlu, komutun KENDİSİ.** Terminal dikey yasası:

```
v_dikey = clamp(kapanma, KAPANMA_MIN=1.5, v_los)
vz      = −v_dikey · tan(nişan_elev)
```

Dikey düzeltme hızı KAPANMA HIZIYLA ölçekleniyor. Geometrik olarak doğru bir
kesişim yasası — ama kapanma durduğunda (takılma) `v_dikey` **1.5 tabanına**
düşüyor ve 9.5°'lik hata için komut **0.26 m/s** oluyor. Oysa hız vektörünü
gerçekten hedefe doğrultmak 16 m/s × tan(9.5°) = **2.7 m/s** ister.

Ölçüldü: terminal karelerinin **%58.3'ünde taban bağlıyor.**
Taban 6.0 olsaydı o karelerde komut 0.26 → **1.06 m/s** olurdu (4 kat).

⚠ Taban YALNIZ kapanma yavaşken bağlar. Hızlı kapanmada `v_dikey = kapanma`
aynen kalır — yani bu değişiklik gerçek kesişimlere DOKUNMAZ, sadece
takılma durumundaki ölü dikey kanalı canlandırır.

---

## ÖLÇÜTLER

### Birincil
**Kara kutudan gerçek en yakın menzil** (`tools/gecis_analiz.py`).
CSV menzili KULLANILMAZ — kestirimdir, 38.9 m derken araç parçalanmıştı.

Karar: deney kolunun medyanı kontrolünkinden KÜÇÜK olmalı.

### İkincil
1. **Temas sayısı** — kara kutuda ≤0.5 m geçiş
2. **|dikey| ıska medyanı** — mekanizmanın hedefi tam bu
3. **eps_elev medyanı** (terminal, menzil <8 m) — mekanizma kapısı

### Mekanizma kapısı (§5.1)
Deney kolunda terminal `eps_elev` medyanı kontrolünkinden **mutlak değerce
küçük** olmalı. Değilse özellik iş görmemiştir ve o koşu veri değildir.

### Geçerlilik eşi (§5.2)
| ölçüt | kötü sebeple iyileşir mi | zorunlu eş |
|---|---|---|
| en yakın menzil | evet — savrulup şans eseri yaklaşma | salınım + görsel temas oranı |
| \|dikey\| ıska | evet — hiç yaklaşmazsan dikey de küçük görünür | geçiş sayısı |
| temas | evet — dengesiz araç şans eseri çarpar | salınım |

### Salınım (kullanıcı kuralı — ölçülmeden "iyileşti" denmez)
- `cx` işaret değişimi / s
- `|roll|` p90 ve roll işaret değişimi / s
- yaw komut hızı p90
- görsel temas oranı (kutusuz kare %) — **%60 altına inerse ölçüt güvenilmez**

### Geçerlilik (§4)
Hedef 20-250 m irtifa, 6-25 m/s bandında olmalı. Dışına çıkan koşu SAYILMAZ.

---

## ETKİ ALANI TABLOSU (§5.10 — zorunlu)

| etkilenebilecek davranış | neden | hangi senaryoda sınanır |
|---|---|---|
| Terminal dikey kanal | doğrudan hedef | `duz` — ana ölçüm |
| Terminal YATAY hız | dikey bütçe kısıtı yatayı kısabilir (`v_los = VZ_MAX_TERM/tan`) | `duz` — v_los medyanı raporlanır |
| Manevrada dikey | dönüşte elev hatası büyür, taban daha çok bağlar | `circle` — REGRESYON |
| Seyir (tutuş) fazı | **etkilenmez** — taban yalnız `terminal` dalında | birim testi |
| Hızlı kesişim | **etkilenmez** — kapanma > 1.5 iken taban bağlamaz | ölçülür (taban bağlama oranı) |

**Cevaplanacak soru:** "hedeflenen yeri iyileştirdi ama başka bir yeri bozdu mu?"

---

## KOŞU PLANI

- `duz` senaryo, hibrit güdüm, **n=4/kol**, dönüşümlü (K,D,K,D,…)
- `circle` regresyonu **n=2/kol**
- Ö-T her iki kolda da **KAPALI** (tek değişken kuralı, §4)
- İrtifa tutucu her iki kolda da **AÇIK** (hedef sabit irtifada uçsun)
- Her koşu: angajman (menzil<40 m) sonrası 120 s ölçüm
- Kare kaydı + video her koşuda

## KARAR KURALI (önceden ilan)

- Birincil ölçüt + en az bir ikincil deney kolu lehineyse **GİRER**
- Birincil kötüleşirse **GİRMEZ**
- Bölünürse **kullanıcıya götürülür** — ölçüt değiştirilmez (§5.6)
- n<4/kol kalırsa **ara veri** olarak sunulur, hüküm kurulmaz (§5.4)

---

# EK BULGULAR (kampanya sürerken, mevcut loglardan)

## 1 · Dikey kusur MANEVRADA 2.6 KAT KÖTÜLEŞİYOR

Hedefin LOS dönüş hızına göre ayrıştırıldı (menzil <20 m, tüm arşiv):

| hedef rejimi | n | `eps_elev` medyanı | p10 | `vz_cmd` medyanı |
|---|---|---|---|---|
| DÜZ (\|λ̇\|<0.1) | 24 946 | −3.4° | −12.7° | −0.23 |
| ORTA (0.1-0.4) | 37 467 | −6.2° | −13.3° | −0.18 |
| **MANEVRA (>0.4)** | 18 824 | **−9.0°** | **−20.0°** | −0.23 |

⚠ Hata büyürken **komut SABİT kalıyor** (−0.18…−0.23 m/s, üç rejimde de aynı).
Bu tam olarak D-V'nin tarif ettiği tıkanma: dikey komut kapanma hızıyla
ölçekleniyor, hata ile değil. Manevrada hata büyüyor ama komut cevap vermiyor.

⇒ D-V'nin manevrada **daha çok** kazandırması beklenir, daha az değil.

## 2 · Yatay kanal (dönüş yarıçapı) kısıtlı DEĞİL

`hiz_yonu = iris_yaw + K_YAW·eps_hiz − sönüm + lead_az` — aracın **gerçek**
yaw'ından hesaplanıyor, yaw slew tavanından GEÇMİYOR (kod satır 802).
Yani Kayra'nın ölçtüğü dönüş yarıçapı açığı (bizim 121.8 m, hedef 33.1 m)
bir kısıttan değil, **saf takip yasasının kendisinden** geliyor.
Ayrı bir iş; bu kampanyanın konusu değil.

## 3 · Nişan noktası denetimi → D-N deneyi doğdu

169 koşunun en yakın anında `cy` medyanı 230 px, `CY_NISAN` 301 px.
70 piksellik sistematik kayma. Ayrıntı: `docs/kampanya/DN_OLCUTLER.md`.

---

# ⚠ KAMPANYA SIRASINDA FARK EDİLEN ÖLÇÜT KUSURU (n=7'de)

**Mekanizma kapısını YANLIŞ tanımlamışım.** İlan ettiğim kapı
"deney kolunda `|eps_elev|` küçülmeli" idi. Ama `eps_elev` bir SONUÇ
ölçütü, mekanizma ölçütü değil — özelliğin doğrudan değiştirdiği şey
dikey KOMUT (`vz_cmd`), hatanın kendisi değil.

Ölçülen (n=7):

| kol | `vz_cmd` medyanı | `eps_elev` medyanı |
|---|---|---|
| kapalı | +0.12 · −0.10 · −0.04 · −4.39 | +0.5° |
| açık | **−5.00 · −4.40 · −1.11** | −15.6° |

Komut açık kolda 10-40 kat büyük → **mekanizma KESİNLİKLE çalışıyor.**
Ama `eps_elev` büyüdü. İki okuma mümkün:

1. Araç daha sert tırmanıyor ama hedef yine de üstte kalıyor — yani
   düzeltme hâlâ yetmiyor, sadece daha çok deniyor.
2. Sert tırmanış dikey salınım üretiyor; anlık `eps_elev` genliği büyüyor.

**Ayrıca beklenen yan etki gerçekleşti:** `v_los` bir koşuda 16.0 → **10.0**
(yani `V_TERM_MIN` tabanına) düştü. Dikey bütçe kısıtı yatay hızı kesiyor —
etki alanı tablosunda öngörülmüştü, şimdi ölçüldü.

⚠ **BİRİNCİL ÖLÇÜT DEĞİŞTİRİLMEDİ** (§5.6). Kara kutudan en yakın menzil
neyse odur. Yalnız mekanizma kapısı, `vz_cmd` üzerinden okunacak şekilde
DÜZELTİLDİ; bu kapı koşuyu geçerli/geçersiz saymak içindir, kolu kazandırmak
için değil.

**Sonuç n=7'de BÖLÜNMÜŞ:**
- menzil / temas → açık lehine
- dikey ıska / aynı seviye oranı → kapalı lehine

Bölünmüş sonuç ölçüt değiştirilerek çözülmez; kullanıcıya götürülür (§5.6).
Kampanya n≥4/kol'a tamamlanıp tekrar bakılacak.

## ⚑ BAĞLAYICI KISIT YER DEĞİŞTİRDİ — dikey TAVAN doydu

Terminal karelerinde `|vz_cmd| ≥ 4.9` (yani `VZ_MAX_TERM=5` tavanında):

| koşu | kol | doyum oranı | `v_los` medyanı |
|---|---|---|---|
| t00/K01 | kapalı | %0.6 | 16.0 |
| t01/K01 | kapalı | %0.0 | 16.0 |
| t02/K01 | kapalı | %0.0 | 16.0 |
| **t00/K02** | **açık** | **%68.0** | **10.0** ← taban |
| **t01/K02** | **açık** | **%45.2** | 16.0 |
| **t02/K02** | **açık** | **%20.0** | 16.0 |

**Zincir:**
1. D-V dikey talebi 4 kat büyütüyor (taban 1.5 → 6.0)
2. Talep `VZ_MAX_TERM = 5` tavanını aşıp DOYUYOR
3. Kodun "dikey bütçe kısıtı" devreye giriyor:
   `v_los = max(V_TERM_MIN, VZ_MAX_TERM/tan(elev))`
4. Yatay hız 16 → **10 m/s** (V_TERM_MIN tabanı)
5. Hedef 15.1 m/s uçuyor → **geride kalıyoruz**

⇒ D-V dikey kanalın ölü olmasını çözdü ama **darboğazı tavana taşıdı.**
Dikey ıskanın kapalı kolda daha iyi çıkmasının makul açıklaması bu:
açık kolda araç dikeyde doymuş ve yatayda yavaşlamış durumda.

### Sıradaki deney önerisi (D-V2)
Taban 6.0 **ile birlikte** `VZ_MAX_TERM` 5 → 8. O zaman:
- dikey talep karşılanır (doyum biter)
- bütçe kısıtı tetiklenmez, yatay hız 16'da kalır

⚠ Bu, kampanya sonucuna bakarak ölçüt/ayar değiştirmek DEĞİL — mevcut
kampanyanın birincil ölçütü olduğu gibi raporlanacak. D-V2 AYRI bir deney
olarak, kendi ölçüt ilanıyla koşulur.

---

# SONUÇ — D-V, n=20 koşu (9 açık / 11 kapalı), 6 sim oturumu

## Mekanizma kapısı: GEÇTİ
`vz_cmd` açık kolda 10-40 kat büyük. Özellik kesinlikle çalıştı.

## Birincil ölçüt: İKİ OKUMA ÇELİŞİYOR — kendi ilanım belirsizmiş

"En yakın menzil medyanı" yazmışım ama **neyin medyanı** olduğunu
belirtmemişim. İki okuma:

| okuma | kapalı | açık | kim önde |
|---|---|---|---|
| tüm geçişlerin medyanı | **4.53 m** | 5.14 m | kapalı |
| **koşu-başı en iyinin medyanı** | 1.19 m | **1.07 m** | açık |

Fark, kol başına geçiş sayısından geliyor (açık 86, kapalı 47). Tüm-geçiş
medyanı, çok geçiş üreten kolu cezalandırıyor — geçiş sayısı arttıkça uzak
geçişler de medyana giriyor. **Koşu-başı okuma bu karıştırıcıdan bağımsız
ve daha savunulabilir.**

⚠ Ölçütü SONUCA BAKARAK seçmiyorum (§5.6). İkisini de raporluyorum ve
belirsizliğin benim ilanımdan kaynaklandığını kayda geçiriyorum.

## İkincil ölçütler: HEPSİ AÇIK KOL LEHİNE

| ölçüt | kapalı | açık |
|---|---|---|
| ≤0.5 m TEMAS | **0** | **2** |
| en iyi geçiş | 0.60 m | **0.20 m** |
| \|dikey\| ıska (tüm geçiş) | 0.42 m | **0.15 m** |
| \|dikey\| ıska (koşu-başı) | 1.07 m | **0.56 m** |
| aynı seviye geçiş (\|dikey\|≤0.5) | %51 | **%80** |

**Mekanizmanın doğrudan hedefi olan dikey eksende açık kol iki kat iyi**,
her iki okumada da tutarlı.

## Bedeli — ölçüldü, gizlenmiyor

| ölçüt | kapalı | açık |
|---|---|---|
| `vz` tavan doyumu | %0-0.6 | **%20-68** |
| `v_los` (bir koşuda) | 16.0 | **10.0** (V_TERM_MIN tabanı) |
| `cx` işaret değişimi | 1.03/s | **2.26/s** |
| görsel temas | %73.4 | %71.7 (ikisi de >%60 ✓) |

Dikey talep tavanı doyurunca bütçe kısıtı yatay hızı kesiyor. Salınım da
iki katına çıkıyor.

## HÜKÜM: BÖLÜNMÜŞ → KULLANICIYA (§5.6)

- Mekanizma çalıştı, dikey eksen **belirgin iyileşti**, temas 0 → 2
- Ama birincil ölçüt okumaya göre değişiyor ve salınım arttı
- Yan etki gerçek: dikey doyum + yatay hız kaybı

**Ölçüt değiştirilerek çözülmez.** Karar kullanıcının.

⇒ **Öneri:** D-V'yi tek başına almak yerine **D-V2 ile birlikte** (tavan
5→8) sınamak. O zaman doyum biter, yatay hız kesilmez ve dikey kazanım
bedelsiz kalabilir. D-V2 hazır, kendi ölçüt ilanıyla koşulur.

---

## ⚑ SONRADAN FARK EDİLEN ETKİLEŞİM — D-V takılmayı ARTIRIYOR olabilir

Kullanıcının 2026-08-17 uçuşunda ortaya çıkan "6 m'de takılma" sorunu
ışığında D-V verisi yeniden okundu:

| kol | takılma (6-12 m bandında geçen terminal karesi) |
|---|---|
| kapalı (n=11) | **%26** — koşular: 15,26,39,15,24,3,31,74,85,4,64 |
| açık (n=10) | **%49** — koşular: 0,3,32,65,6,80,69,95,84,18 |

Medyan iki katı, ama **dağılımlar tamamen örtüşüyor** → eğilim, hüküm değil.

**Mekanizma açıklaması tutarlı:**
D-V dikey talebi büyütür → `VZ_MAX_TERM` tavanı doyar (%20-68 ölçüldü) →
dikey bütçe kısıtı yatay hızı keser (bir koşuda 16 → 10 m/s) → hedef
15.15 m/s uçarken kapanma NEGATİFE döner → araç 6-12 m bandında sıkışır.

⇒ **D-V ve Ö-T zıt yönde çalışıyor:** D-V yatay hızı kesiyor, Ö-T mandalı
bırakıp hızı 24 m/s'ye salıyor.

⇒ **D-V2'nin önemi artıyor:** tavan 8'e çıkarsa bütçe kısıtı tetiklenmez,
yatay hız 16'da kalır ve D-V'nin dikey kazanımı bu bedelsiz gelir.
D-V2 kampanyası Ö-T'nin arkasına zincirlendi.

⚠ Bu okuma kampanya BİTTİKTEN sonra, başka bir sorunun ışığında yapıldı.
Birincil ölçüt DEĞİŞTİRİLMEDİ; bu ek bir gözlem olarak kaydediliyor (§5.6).
