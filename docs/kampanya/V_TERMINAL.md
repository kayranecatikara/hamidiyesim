# V_TERMINAL TARAMASI — hücum hızı 16 / 20 / 24 m/s

> **Ölçütler ve karar kuralı KOŞMADAN ÖNCE yazıldı (CLAUDE.md §4).**
> Yazım tarihi 2026-08-17, ilk uçuştan önce.

---

## 0 · NİYE — üç kampanyanın ortak kalıntısı

Ö5 (20 uçuş), S (40 uçuş) ve ZARF (11 uçuş) hep aynı imzayı üretti:
**takip düzeliyor, bitiriş bozuluyor.** Ö5 raporunun kalıcı bulgusu tek
satırdı ve hiç sınanmadı:

> `V_TERMINAL = 16 m/s`, hedef **15.1 m/s** → kalan kapanma **0.9 m/s**.

Terminal faz `TERMINAL_BOYUT = 25 px` ile başlıyor, yani **son 6.4 metre**.
0.9 m/s kapanmayla bu 6.4 m **7 saniye** sürüyor — hedefin kaçması için
bolca zaman. 24 m/s'de aynı mesafe **0.7 saniye**.

**Bu kilidi ZARF kampanyası açtı.** Eskiden hızı artırmak dönüşü
imkânsızlaştırıyordu (R = V²/(g·tanθ)); artık değil:

| | V | yatış tavanı | dönüş yarıçapı |
|---|---|---|---|
| eski araç | 16 m/s | 45° | 26.1 m |
| yeni zarf | 16 m/s | 70° | 9.5 m |
| **yeni zarf** | **24 m/s** | 70° | **21.4 m** — eski aracın 16 m/s'sinden DAR |

Yeni araç 24 m/s'de bile eskisinin 16 m/s'deki dönüşünden dar dönüyor.

---

## 1 · TEK DEĞİŞKEN

```
K  : V_TERMINAL = 16   (bugünkü varsayılan)
A  : V_TERMINAL = 20
B  : V_TERMINAL = 24
```
Diğer her şey ZARF kampanyasının bıraktığı yerde: `ANGLE_MAX 70°`,
`WPNAV_ACCEL 2600`, `PSC_JERK_XY 40`, `Cfg.MAX_ACCEL 12`, `SONUM_T 0`.

⚠ 24 tavan değil seçim: `WPNAV_SPEED 2500` (25 m/s) ve `V_TOPLAM_MAX 24`
zaten orada. 24'ün üstü ayrı bir adımdır.

---

## 2 · MEKANİZMA KAPISI (§5.1)

Terminal karelerde loglanan `v_los`, kolun değerine eşit olmalı:

| kol | beklenen `v_los` medyanı (TERMINAL karelerde) |
|---|---|
| K | 16.0 ± 0.5 |
| A | 20.0 ± 0.5 |
| B | 24.0 ± 0.5 |

⚠ Dikey bütçe bloğu `v_los`'u `V_TERM_MIN = 10`'a kadar kısabiliyor
(`if v_dikey·tan(nişan_elev) > VZ_MAX_TERM`). Bu yüzden **medyan** bakılır,
ve kısılma oranı (v_los < kolun değeri olan kare yüzdesi) raporlanır.
Bir kolda medyan hedefin ±0.5'i dışındaysa o koşu GEÇERSİZDİR.

---

## 3 · ÖLÇÜTLER

### BİRİNCİL · **en yakın menzil** (koşu başına, `olay.json`)

Kaynak `telem.csv` 10 Hz gerçek 3B mesafe. **Düşük = iyi.**

*§5.3 örnekleme:* kapanma 0.9-9 m/s → 10 Hz'te örnek başına 0.09-0.89 m.
İsabet zarfı ±0.65 m. **B kolunda (24 m/s) örnekleme çözünürlüğü zarfla
aynı mertebeye geliyor** — bu ölçütün B kolunda 0.9 m'ye kadar yanılabileceği
peşinen ilan edilir. Bu yüzden isabet EŞ BİRİNCİL'dir (aşağı).

### EŞ BİRİNCİL · **isabet** (tür-eşli)

Kullanıcının hedefi birebir bu. n=4/kol'da düşük güçlü olduğu biliniyor
(taban 4/8 = %50); tek başına hüküm kurmaz, birincille BİRLİKTE okunur.

*§5.2 geçerlilik eşi (ikisi için de):* **temasın son 2 saniyesinde nişan
sapması** `|cx − 320|` (20 Hz bbox). En yakın menzil, savrulup şans eseri
yakından geçmekle de düşebilir; kontrollü bir yaklaşmada nişan sapması
küçük kalır. Sapma büyürken menzil düşüyorsa "şans" damgası vurulur (§4).

### İKİNCİL

| # | ölçüt | kaynak | niye |
|---|---|---|---|
| İK-1 | terminal fazda geçen süre | bbox 20 Hz | mekanizmanın doğrudan sonucu (7 s → 0.7 s bekleniyor) |
| İK-2 | görsel fazda kutu oranı | bbox 20 Hz | hızlanma teması bozdu mu |
| İK-3 | ψ̇ işaret değişimi/s, <30 m | telem 10 Hz | salınım (kutudan bağımsız) |
| İK-4 | yanal ivme p90 | bbox 20 Hz | çeviklik kısılmadı mı |
| İK-5 | `v_los` kısılma oranı | bbox 20 Hz | dikey bütçe bloğu ne kadar müdahale etti |
| İK-6 | KURTARMA sayısı | bbox `durum` | görsel temas kesintisi |

---

## 4 · ETKİ ALANI TABLOSU (§5.10)

| etkilenebilecek davranış | neden | hangi senaryoda sınanır |
|---|---|---|
| **dikey kanal** | `v_dikey` ve dikey bütçe bloğu `v_los`'tan besleniyor: `vz = −v_dikey·tan(nişan_elev)` ve `v_los` bu bütçeyle kısılıyor. V_TERMINAL büyüyünce dikey komut da büyür. **YAPISAL DEĞİL, GERÇEK BİR BAĞ.** | düz+kaçamak (İK-5) + `dikey_yukari`/`dikey_asagi` kaçamağı **regresyonda** |
| **dönüş yarıçapı / manevra takibi** | R = V²/(g·tanθ); 24 m/s'de 21.4 m (16'da 9.5 m) — 2.3 kat geniş | **`square` regresyonu, n=2/kol** |
| **görsel temas** | daha hızlı kapanma = kadrajda daha hızlı büyüyen hedef; tespit kaçırabilir | İK-2 |
| **isabet zarfında kalma süresi** | 8.9 m/s kapanmada ±0.65 m zarfın içinde ~0.15 s kalınır (0.9 m/s'de 1.4 s) | BİRİNCİL + eş birincil doğrudan ölçüyor |

**Bu, özelliğin ilan edilmiş asıl RİSKİDİR:** hızlanmak zarfa girme
olasılığını artırıp zarfta KALMA süresini kısaltıyor. İki etki ters yönde;
kampanya hangisinin baskın olduğunu ölçüyor.

---

## 5 · KOŞU PLANI (§4 dönüşümlü, §5.9 tür-eşli)

**Düz + kaçamak (kazanım) — 12 uçuş, n=4/kol (2 yatay + 2 çapraz):**
```
VT01_K_yatay   VT02_A_yatay   VT03_B_yatay
VT04_K_capraz  VT05_A_capraz  VT06_B_capraz
VT07_K_yatay   VT08_A_yatay   VT09_B_yatay
VT10_K_capraz  VT11_A_capraz  VT12_B_capraz
```

**Regresyon — 6 uçuş (kazanan kol vs kontrol):**
```
kare  : VT13_K  VT14_W  VT15_K  VT16_W        (n=2/kol)
dikey : VT17_K_dikey_asagi   VT18_W_dikey_asagi   (n=1/kol, etki alanı)
```

---

## 6 · KARAR KURALI — sonuca bakmadan ilan edildi

**Adım 0 — KAPILAR.** Mekanizma kapısından (§2) geçmeyen koşu atılır.

**Adım 1 — KAZANIR** bir kol, İKİSİ birden olursa:
1. BİRİNCİL (en yakın menzil medyanı) kontrolden **düşük**, **ve**
2. Eş birincil (isabet) kontrolden **az değil**.

**Adım 2 — ELENİR** bir kol, şunlardan biri olursa:
- BİRİNCİL **artarsa**, **veya**
- İsabet kontrolün altına düşerse, **veya**
- Geçerlilik eşi düşerse: en yakın menzil düşerken nişan sapması
  **iki katına** çıkarsa (yani yakınlık şansa dönmüşse).

**Adım 3 — İKİ KOL DA KAZANIRSA:** en yakın menzili düşük olan seçilir;
fark %15'in altındaysa **düşük hız tercih edilir** (dönüş yarıçapı ve dikey
bağ riski daha az).

**Adım 4 — REGRESYON (§5.10):** kazanan kol `square`'da medyan mesafeyi
kontrolün %115'inin üstüne çıkarırsa ya da dikey kaçamakta isabeti
kaybederse, bu bir GERİLEMEDİR; ölçüsüyle raporlanır, **kararı kullanıcı
verir.**

**Hiçbir kol kazanmazsa** bu açıkça yazılır ve `V_TERMINAL = 16` kalır.

---

## 7 · SONUÇLAR — 18 uçuş, 2026-08-17

### 7.0 · Mekanizma kapısı — KUSURSUZ

Terminal karelerde `v_los` medyanı: **16.00 / 20.00 / 24.00** (hedefe tam
eşit). Dikey bütçe kısılması %0 / %0 / %3 — ihmal edilebilir.
Ve öngörülen doğrudan sonuç gerçekleşti: **terminal fazda geçen süre
10.3 s → 1.7 s → 3.3 s.**

### 7.1 · KAZANIM (düz + kaçamak, n=4/kol, tür-eşli)

| ölçüt | K · 16 | **A · 20** | B · 24 |
|---|---|---|---|
| **BİRİNCİL en yakın menzil** | 1.83 m | **1.14 m** | **1.14 m** |
| **EŞ BİRİNCİL isabet** | 1/4 | **2/4** | 1/4 |
| **geçerlilik eşi: nişan sapması** | 19 px | **4 px** | **3 px** |
| terminal süre | 10.3 s | 1.7 s | 3.3 s |
| kutu oranı | %60.2 | %55.2 | %60.7 |
| yanal ivme p90 | 5.85 | 8.32 | 6.91 m/s² |
| ψ̇ (<30 m) | 1.392 | 2.031 | 1.575 |

Permütasyon: en yakın menzil A p=0.543, B p=0.143 — **ikisi de gürültü
sınırında.** Hüküm bu yüzden tek ölçüte değil, örüntüye dayanıyor.

**⭐ EN GÜÇLÜ SİNYAL GEÇERLİLİK EŞİNDEN GELDİ.** Temasın son 2 saniyesinde
nişan sapması **19 → 4 → 3 px**. İlan ederken şunu yazmıştım: *"en yakın
menzil, savrulup şans eseri yakından geçmekle de düşebilir; kontrollü bir
yaklaşmada nişan sapması küçük kalır."* Tam tersi oldu — menzil düşerken
sapma da **beşte bire indi**, yani yakınlaşma kontrollü. Kıyas için:
jerk 15'in bozuk terminali bu ölçütte **30 px**'ti.

⚠ İki koşu (VT04_K_capraz, VT05_A_capraz) terminale HİÇ girmedi
(en yakın 17.33 / 17.49 m, terminal süre 0.0 s) ve medyanları yukarı
çekiyor. **Post-hoc dışlanmadılar** (§5.6). Yalnız bilgi olarak: onlarsız
medyanlar K 1.71 / A 0.89 / B 1.14 m.

### 7.2 · REGRESYON (§5.10) — GERİLEME YOK

| | K · 16 | A · 20 | ilan edilen eşik |
|---|---|---|---|
| **kare** medyan mesafe | 56.1 m | 56.9 m (**%101**) | %115 altı ✓ |
| kare 60 m içinde süre | 137 s | 130 s | — |
| **`dikey_asagi`** kaçamak | İSABET, 1.22 m | **İSABET, 0.91 m** | isabet kaybı yok ✓ |

Dönüş yarıçapı riski (24 m/s'de 21.4 m) 20 m/s'de gerçekleşmedi. Dikey bağ
(`vz = −v_dikey·tan(nişan_elev)`) da zarar vermedi — aksine daha yakın geçti.

### 7.3 · KARAR — İLAN EDİLEN KURALIN UYGULANMASI

- **Adım 1 (KAZANIR):** A ve B'nin ikisi de en yakın menzili düşürdü
  (1.83 → 1.14) **ve** isabeti düşürmedi (1/4 → 2/4 ve 1/4). **İkisi de kazandı.**
- **Adım 3 (iki kol da kazanırsa):** en yakın menzil **berabere** (1.14 =
  1.14), fark %0 < %15 → kural **düşük hızı** seçtiriyor (dönüş yarıçapı ve
  dikey bağ riski daha az).
- **Adım 4 (regresyon):** iki kapı da temiz.

> **`V_TERMINAL = 20` GİRDİ.** Varsayılan `Cfg.V_TERMINAL` 16 → 20.

### 7.4 · RAPORDAN ÖNCE ÜÇ SORU (§5.8)

1. **Özellik çalıştı mı?** Evet — `v_los` medyanları tam 16/20/24, terminal
   süresi 10.3 → 1.7 s.
2. **Ölçütüm kötü bir sebeple mi iyileşti?** Hayır, ve bu sefer geçerlilik
   eşi bunu ETKİN biçimde gösterdi: nişan sapması 19 → 4 px ile birlikte
   düştü. Kutu oranı da korundu (%60 → %55, eşik yok ama belirgin düşüş de
   yok).
3. **n kaç?** 4/kol (kazanım), 2/kol (kare regresyonu), 1/kol (dikey).
   §5.4'ün sınırında; p değerleri 0.14-0.54, yani **tek ölçüt anlamlı
   değil.** Hüküm, dört göstergenin (menzil, isabet, nişan sapması, terminal
   süresi) aynı yöne bakmasına ve iki regresyon kapısının temiz geçilmesine
   dayanıyor. Daha güçlü kanıt isteniyorsa n artırılabilir.

### 7.5 · BUNDAN SONRASI

Bu, **oturumun ilk gerçek "bitiriş" kazanımı.** Ö5/S/ZARF'ın hepsi takibi
düzeltip terminali bozmuştu; V_TERMINAL ilk kez terminali düzeltti ve
takibi bozmadı.

Açık kalanlar: `TERMINAL_BOYUT = 25 px` (terminalin 6.4 m'de başlaması) hiç
sınanmadı — daha erken başlatmak 20 m/s'lik hamleye daha uzun bir koşu
mesafesi verir. Ve `V_TOPLAM_MAX = 24` ile seyir hızı hâlâ eski araca göre.
