# KAMPANYA H — TERMİNAL HIZ KORUMA (D2)

**Tek değişken:** `AVCI_IBVS_TERM_HIZ_KORU` (panel: `d2_hiz_koru`) · 0 / 1
**Hedeflenen sorunlar:** `SORUN_ENVANTERI.md` **S1** (terminal freni →
kadraj kaybı) ve **S2** (kapanma 0.9 m/s). İkisi de aynı anahtara bağlı.

---

## 1 · ÖZELLİK NE YAPIYOR

Taban davranış: terminale girince `v_los := V_TERMINAL = 16.0` — seyirdeki
19.4 m/s'den **ani −3.4 m/s basamak**.

D2 açıkken: terminale girerken seyir PI yasasının ulaştığı hız **korunur**
(`v_term_kilit`, `[V_TERM_MIN, V_TOPLAM_MAX]` arasına kırpılır). Basamak
yok → fren yok → burun kalkmaz → kamera hedefi kaybetmez.

---

## 2 · MEKANİZMA KAPISI (§5.1)

**DENEY kolunda terminal `v_los` medyanı 16.00 ise o koşu GEÇERSİZDİR.**
Kontrol kolunda 16.00 olmalı. Her koşuda 20 Hz logdan yazdırılır.

---

## 3 · BİRİNCİL ÖLÇÜT (§5.5)

Kullanıcının cümlesi: *"hedef aracı alttan kaçırdık"*, *"dikey hizalama
çok kötü"*, *"üstünden geçip gidiyoruz"*.

→ **Her yaklaşmanın en yakın anındaki |dikey ıska|** (m), gerçek konumdan
(10 Hz `telem.csv`), yaklaşma başına bir değer, kol içinde medyan.

### Geçerlilik eşleri (§5.2)

| ölçüt | kötü sebeple iyileşir mi | zorunlu eş |
|---|---|---|
| \|dikey ıska\| | **EVET** — hiç yaklaşmazsak dikey hata da küçük kalır | **en yakın menzil** |
| \|dikey ıska\| | **EVET** — kör uçarsak ölçemeyiz | **0-5 m görsel temas oranı** |
| en yakın menzil | evet — savrulup şans eseri | **vz salınımı** |

---

## 4 · İKİNCİL ÖLÇÜTLER — koşmadan önce ilan edildi

1. terminal girişinden sonra **pitch tepesi** (°)
2. terminal girişinden sonra **cy tepesi** ve **kadraj dışı oranı** (cy>440)
3. terminalde **kapanma hızı** (m/s)
4. en yakın anda **boyuna mesafe** (m)
5. **vz_cmd işaret değişimi / s** ve |vz| p90  (§4 salınım kuralı)
6. **|yatış| p90** ve yatış işaret değişimi / s
7. görsel temas kesinti sayısı ve süresi
8. isabet (varsa) ve vuruş sınıfı (KONTROLLÜ / ŞANS)

---

## 5 · KARAR KURALI — koşmadan önce ilan edildi

**D2 GİRER**, şu DÖRDÜ birden sağlanırsa:
1. \|dikey ıska\| medyanı iyileşir, **VE**
2. en yakın menzil medyanı kötüleşmez (D2 ≤ kontrol × 1.15), **VE**
3. 0-5 m görsel temas oranı kötüleşmez, **VE**
4. vz işaret değişimi / s kötüleşmez.

Bölünmüş çıkarsa **ölçüt değiştirilmez, kullanıcıya götürülür** (§5.6).

---

## 6 · ETKİ ALANI TABLOSU (§5.10) — her satır koşulur

| etkilenebilecek davranış | neden etkilenebilir | hangi senaryoda sınanır |
|---|---|---|
| sakin kuyruk takibi | terminale 19-20 m/s girilirse hedefi geçip gidebiliriz | `duz` + `yok` (blok B) |
| kaçamağa tepki | hız yüksek → dönüş yarıçapı R=V²/(g·tanθ) büyür → geç tepki | `duz` + `yatay` (blok A) |
| sürekli dönen hedef | hız hiç kısılmazsa dairede kalıcı aşım | `circle_l` (blok C) |

Cevaplanacak cümle: **"hedeflenen yeri iyileştirdi ama başka bir yeri
bozdu mu?"**

---

## 7 · KOŞU LİSTESİ — 16 uçuş, ≈1 sa 40 dk

| # | ad | kol | senaryo / kaçamak |
|---|---|---|---|
| 1 | H01_K_yatay | KONTROL | duz / yatay |
| 2 | H02_D_yatay | DENEY | duz / yatay |
| | ⏸ **ARA RAPOR — mekanizma kapısı** | | |
| 3 | H03_K_yatay | KONTROL | duz / yatay |
| 4 | H04_D_yatay | DENEY | duz / yatay |
| 5 | H05_K_yatay | KONTROL | duz / yatay |
| 6 | H06_D_yatay | DENEY | duz / yatay |
| 7 | H07_K_yatay | KONTROL | duz / yatay |
| 8 | H08_D_yatay | DENEY | duz / yatay |
| | ⏸ **ARA RAPOR — ana blok, n=4/kol** | | |
| 9 | H09_K_yok | KONTROL | duz / **yok** (TABAN) |
| 10 | H10_D_yok | DENEY | duz / yok |
| 11 | H11_K_yok | KONTROL | duz / yok |
| 12 | H12_D_yok | DENEY | duz / yok |
| | ⏸ **ARA RAPOR — taban regresyonu** | | |
| 13 | H13_K_dai | KONTROL | circle_l |
| 14 | H14_D_dai | DENEY | circle_l |
| 15 | H15_K_dai | KONTROL | circle_l |
| 16 | H16_D_dai | DENEY | circle_l |

Kollar **dönüşümlü** (K, D, K, D…) — sim kayması iki kolu eşit etkilesin (§4).
Senaryo karışımı **eşit** (§5.9): her kolda 4 yatay + 2 yok + 2 daire.

Koşu: `bash ~/.avci_sim/kosuHK.sh <ad> <0|1> <kaçamak> 25 240 <senaryo>`

---

## 8 · SONUÇLAR — 16 uçuş (H12 bir kez tekrarlandı: hedef havalanamadı)

### 8.1 · MEKANİZMA KAPISI (§5.1) — GEÇTİ

Terminal `v_los` medyanı: **KONTROL 16.00** (8/8 koşu) · **DENEY 18.5-22.6**
(7/8 koşu). Tek istisna **H14_D_dai**: terminal karesi **0** → özellik hiç
çalışmadı → §5.1 gereği **GEÇERSİZ koşu**, veri noktası değil.

### 8.2 · BLOK A+B — `duz` senaryosu, n=6/kol (4 yatay + 2 yok, §5.9 eşit)

| ölçüt | KONTROL | DENEY | p |
|---|---|---|---|
| **\|dikey\| ıska (BİRİNCİL)** | 1.62 | 1.14 | 0.567 |
| **koşunun en yakını (m)** | 1.75 | **1.20** | **0.030** |
| pitch tepe (°) | 24.57 | **11.05** | 0.093 |
| cy tepe (px) | 467.5 | **348.3** | **0.026** |
| **kadraj dışı oranı** | **%73** | **%0** | **0.039** |
| terminal süre (kare) | 533.5 | **60** | **0.013** |
| vz işaret / s (ham) | 0.59 | 1.17 | 0.271 |
| \|roll\| p90 (°) | 32.45 | **20.80** | 0.100 |
| **isabet** | **3/6** | **5/6** | |

Koşu en yakınları: K = [3.17, 1.30, 1.87, 1.64, 1.57, 2.10] ·
D = [**0.81, 0.55, 1.09, 1.40, 1.70, 1.31**]

### 8.3 · ⚠ BİRİNCİL ÖLÇÜT n İLE YÖN DEĞİŞTİRDİ (§5.4 tekrar doğrulandı)

| n | KONTROL | DENEY | yön |
|---|---|---|---|
| n=4/kol | 1.30 | 1.39 | D2 **KÖTÜ** |
| n=6/kol | 1.62 | 1.14 | D2 **İYİ** |

Aynı Ö8 örüntüsü. **Birincil ölçüt bu n'de çözmüyor** — hüküm kurulmaz.
Sebebi de belli: ölçüt 13 m'de biten yaklaşmayı 1 m'de biteniyle aynı
kefeye koyuyor; kollar farklı menzil dağılımına sahip olunca ölçüt
"kaç uzak yaklaşma oldu"yu ölçüyor.

### 8.4 · SALINIM — ölçüt geçerliliği İKİ KEZ sınandı (§5.2)

**1. sınama:** "kontrol kolu hedefi daha çok kaybettiği için mi sakin
görünüyor?" → **HAYIR**, iki kolda da terminal karelerinin **%100**'ünde
kutu var, donmuş kare yok.

**2. sınama:** ham ölçüt kontrolün **26.7 s**'lik terminalini deneyin
**3.0 s**'lik hücumuyla kıyaslıyor — aynı faz değil. **Adil pencerede**
(en yakın andan geriye 3 s) yeniden ölçüldü:

| ölçüt | KONTROL | DENEY | p |
|---|---|---|---|
| \|vz\| p90 | 1.77 | 1.75 | **1.000** (aynı) |
| vz işaret / s | 0.17 | 0.96 | 0.152 |
| **\|roll\| p90 (°)** | **29.40** | **4.20** | **0.039** |
| roll işaret / s | 1.36 | 2.95 | 0.190 |

**Genlikte D2 eşit ya da çok daha iyi** (roll 29.4° → 4.2°). İşaret
değişimi oranı yüksek ama genlik düşük = kontrolcü çalışıyor, savrulmuyor.
Kullanıcının 2026-08-10 kuralının aradığı "dengesizce savrulan araç"
tarifi **kontrol koluna** uyuyor, deney koluna değil.

### 8.5 · ⛔ ALEYHTE BULGU — 3 m'nin içinde dikey hassasiyet DÜŞTÜ

| | 3 m'ye giren yaklaşma | \|dikey\| medyan | \|yanal\| medyan |
|---|---|---|---|
| KONTROL | 14 | **0.21 m** | 0.29 m |
| DENEY | 16 | **1.06 m** | **0.10 m** |

Kontrol 3 m'ye daha az giriyor ama girdiğinde dikeyde **isabet zarfının
içinde**. Deney daha sık giriyor, dikeyde daha dağınık, yanalda daha iyi.

**Sebep ölçüldü:** kontrolün terminal bacağı 533 kare (26.7 s), deneyin
60 kare (3.0 s). Deney kolunun dikey kanalı aynı düzeltmeyi **dokuzda bir
sürede** yapmak zorunda. Bu, D2'nin bir kusuru değil, bir **ödünleşme**.

### 8.6 · BLOK C (daire) — ÖLÇÜM GEÇERSİZ, YAPISAL GARANTİ İLE KAPATILDI

| koşu | en yakın | terminal kare | D2 aktiflik |
|---|---|---|---|
| H13_K_dai | 4.72 m | 26 | — |
| H15_K_dai | 2.78 m | 86 | — |
| H14_D_dai | 10.36 m | **0** | **%0** |
| H16_D_dai | 12.92 m | 23 | **%0.5** |

Deney kolu kâğıt üstünde çok daha kötü. **Ama §5.13 madde 4:** mekanizma
aktiflik oranı %0 ve %0.5. `circle`'da araç terminale neredeyse hiç
girmiyor → **D2 çalışmadı** → bu fark D2'den gelemez.

**Ölçüm yerine YAPISAL GARANTİ (§5.10):** `TERM_HIZ_KORU` yalnız
`bbox_ibvs.py:979`'da, `if terminal:` dalının içinde okunur. Test
**B97**: 162 girdi kombinasyonunda (cx / cy / boyut / roll / kilit)
`terminal=False` iken `komut()` çıktısı **bit bit aynı** (fark 0.00e+00).
**Terminale girilmeyen senaryoda uçuş yolu DEĞİŞEMEZ.**

Dolayısıyla daire farkı **koşu değişkenliğidir** (aynı senaryoda 3 kat
saçılma daha önce ölçülmüştü).

### 8.7 · KARAR KURALI DENETİMİ

| # | kural | sonuç |
|---|---|---|
| 1 | \|dikey ıska\| iyileşir | **BELİRSİZ** — n=4'te ters, n=6'da doğru, p=0.567 |
| 2 | en yakın menzil kötüleşmez | ✓ **İYİLEŞTİ** 1.75 → 1.20 m (p=0.030) |
| 3 | görsel temas kötüleşmez | ✓ **İYİLEŞTİ** kadraj dışı %73 → %0 (p=0.039) |
| 4 | salınım kötüleşmez | **KARIŞIK** — genlik eşit/daha iyi, işaret oranı yüksek |

**1 ve 4 net değil → §5.6 gereği ölçüt değiştirilmez, KARAR KULLANICIYA.**

### 8.8 · RAPORDAN ÖNCE ÜÇ SORU (§5.8)

1. **Özellik çalıştı mı?** Evet — `v_ter` 16.0 → 18.5-22.6, 7/8 deney
   koşusunda. H14 geçersiz ilan edildi.
2. **Ölçütüm kötü bir sebeple mi iyileşti?** İki ayrı geçerlilik sınaması
   yapıldı (§8.4). Biri temiz çıktı, biri gerçek bir kusur buldu ve ölçüm
   adil pencerede tekrarlandı.
3. **n kaç, hüküm kurulur mu?** n=6/kol (`duz`). Anlamlı çıkan ölçütlerde
   (en yakın, cy, kadraj dışı, terminal süre, roll p90) hüküm kurulur.
   **Birincil ölçütte kurulmaz** — n ile yön değiştirdi.

---

## 9 · SIRADAKİ TASARIM ÖNERİSİ — PITCH SINIRLI FREN

Ölçüm iki şeyi birden gösterdi:
* fren **kadrajı öldürüyor** (kontrol: pitch 24.6°, kadraj dışı %73)
* frenin **yokluğu** dikey kanala oturma süresi bırakmıyor (deney:
  3 m içinde \|dikey\| 0.21 → 1.06 m)

İkisinin arasında bir yer var: **frenle, ama kameranın izin verdiği kadar.**

Ölçülmüş sınır: hedef seviyedeyken **pitch +27°'de** kadrajdan çıkıyor.
Güvenli tavan ~**15°** → yavaşlama tavanı `a = g·tan(15°) ≈ 2.6 m/s²`.
19.4 → 16.0 m/s geçişi 1.3 saniyeye yayılır; burun 15°'yi geçmez, hedef
kadrajda kalır, dikey kanal oturma süresini geri alır.

Bu **yeni koddur** — §1 gereği kullanıcı onayı olmadan yazılmaz.
