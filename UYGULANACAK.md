# UYGULANACAK — teker teker, ölçerek

Aşağıdakiler **tek tek** uygulanacak; her maddeden sonra uçulup ölçülecek,
sonuç maddenin altına yazılacak. Bir madde bitmeden diğerine geçilmeyecek.

**Neden böyle:** bir keresinde 8 grup değişiklik bir arada uçuruldu. Bazıları
ölçümle işe yaradı, biri ölçülebilir zarar verdi (görsel kilit: kör uçuş %64,
drone hedefin üstüne çıkıp zemine çakıldı). Hangisinin ne yaptığı ayırt
edilemedi.

**Kural:** her adımda `python3 -m tests.test_visual_lead` ve
`python3 -m tests.test_gps_guidance` çalıştırılır.

---

> Başka bir makinede/dalda devam edeceksen önce **[DEVAM.md](DEVAM.md)**:
> dal senkronu, sistem başlatma, ölçüm araçları, laptop'ta ayrıca gerekenler.

## S — YATAY SALINIM · 40 UÇUŞ · 3 DALGA · 5 ADAY · **ÇÖZÜLMEDİ**

**Tam rapor:** `docs/kampanya/S_SALINIM.md` (ölçütler her dalgada koşmadan
önce ilan edildi) · kullanıcı isteği: *"bu ivme şeysini bozmadan salınımı
azaltmamız lazım... şu salınımı kes ve aracın ivme şeysini kısma."*

Kullanıcının 2. şartı bir **ELEME KAPISI** olarak kuruldu (gerçekleşen yanal
ivme p90 < kontrolün %85'i → kol elenir), sonuca bakmadan.

**TEŞHİS (D kampanyasının logundan, çevrimdışı):** hız vektörü komutunun
istediği yanal ivme medyan **8.43** / p90 **26.3 m/s²**, aracın fiziksel
tavanı **9.81**. Komut, dönüş tavanını **karelerin %43'ünde** aşıyor — ve
bu oran **jerk 5/10/15'te birebir aynı.** Salınımın kaynağı ivme bütçesi
değil, komutun kendisi.

| dalga | aday | birincil | KISIT ivme | sonuç |
|---|---|---|---|---|
| 1 | **S1** komut dönüş tavanı (ω_max) | −%8 (p=0.484) | **%81** (p=0.024) | ⛔ **ELENDİ** |
| 1 | **S3** anti-windup geri besleme | **+%24 kötü** | **%81** | ⛔ **ELENDİ** |
| 1 | S2 D terimi `SONUM_T=0.30` | −%9 (p=0.524) | %104 ✓ | kaldı |
| 2 | **W1** `PSC_VELXY_D` 0.5→1.2 | +%5 (p=0.563) | %109 | fayda YOK |
| 2 | **W2** `PSC_VELXY_P` 2.0→1.2 | −%11 (p=0.304) | %97 | gürültüde |
| 3 | **S2/A** doğru rejimde, n=8/kol | **−%19** (p=0.308) | **%102** ✓ | **KULLANICIYA** |

**Dalga 3 (duz+kaçamak, birincil <30 m, n=8/kol):** ψ̇ 1.450 → **1.174**,
kutu oranı %60.6 → **%69.9**, **isabet 4/8 → 7/8** (Fisher p=0.282),
ivme p90 **%102** (çevikliği kısmıyor).

**⛔ AMA:** hiçbiri istatistiksel olarak ayrışmıyor, **ilan edilen doz-tepki
şartı DÜŞTÜ** (0.60, 0.30'dan daha az sönümledi) ve **n artırma kararını
isabeti gördükten sonra, özelliğin lehine aldım** — bu kanıtı zayıflatır ve
gizlenmiyor. `SONUM_T` varsayılan YAPILMADI, kapalı bırakıldı.

**⚠ ÖLÇÜM TASARIMI HATAM (kendi hatam, kayda geçiyor):** dalga 1-2'nin
tamamı p>0.30'da kaldı çünkü birincili "60 m içi" tanımlamıştım. Salınım
menzile göre 3 kat değişiyor ve kullanıcının şikâyet ettiği bant (0-15 m)
`square` koşusunun yalnız **8 saniyesi** — 200 saniyelik uzak menzil içinde
eridi. İkinci kusur: `|roll| p90` ile `ivme p90` **aynı büyüklüktür**
(`a = g·tan roll`); salınım genlik değil **işaret değiştirme** demektir.

**⭐ EN ÖNEMLİ BULGU — BAĞLAYICI KISIT `PSC_JERK_XY`'DİR:**
Araç 45°'ye HİÇ dayanmıyor (≥44.5° kare oranı **%0.0-0.8**) → `ANGLE_MAX`
bağlayıcı değil, uçmadan elendi (Ö6 tuzağının tekrarı olurdu).
`WPNAV_ACCEL` 800 → 1000 denendi, araç fazladan boşluğu **hiç kullanmadı**
(ivme p95 7.18 → 6.97) → 4 uçuşta elendi, kapı kapalı.
Ölçülen jerk ise parametreyle **neredeyse orantılı**: p90 7.29 / 11.37 /
14.61 ← 5 / 10 / 15, karelerin %16-33'ünde tavana yapışıyor.
**Yanal çevikliği bağlayan TEK şey jerk'tir**; kullanıcının 5→15
değişikliğinin neden gözle görülür tek iyileşme olduğunun açıklaması bu.
Sert gövde tavanı 19.41 m/s³ — bugünkü 10'un üstünde hâlâ pay var.

**⚠ KALICI BULGU:** komutun dönüş tavanını %43 aşması **zararlı bir artık
değil** — doymuş bir sistemin en hızlı tepkisidir. İleri beslemeyle kesmek
(S1) de geri beslemeyle telafi etmek (S3) de aracı YAVAŞLATTI. Salınımı komut tarafından çözme yolu kapandı.

S1 ve S3 §5.12 uyarınca koddan **tamamen çıkarıldı** — 960 kombinasyonda
bit bit denklik doğrulandı (fark 0.00e+00), grep sıfır, bekçi testleri
B65-B68 eklendi.

---

## Ö5 KAREDE (`DONUS_A` 0 → 9.81) · 20 UÇUŞ · **KARAR KULLANICIDA**

**Tam rapor:** `docs/kampanya/O5_KARE.md` · 8 kare + 8 düz + 4 daire,
n=4/kol (daire 2/kol), dönüşümlü, tür-eşli. Varsayılan **KAPALI bırakıldı.**

**Niye elenmiş özellik yeniden ölçüldü (§5.13):** Ö5 tetiği düzde karelerin
%16'sında sağlanıyor, karede **%57**'sinde. 2026-08-11'de düzde elenmişti —
yani tasarım zarfının dışında sınanmıştı.

| ölçüt | KONTROL | Ö5 | |
|---|---|---|---|
| **KARE 60 m içinde süre (birincil)** | 114.0 s | **142.5 s** | +%25 |
| KARE medyan mesafe (geçerlilik eşi) | 62.3 m | **55.6 m** | ✓ aynı yönde |
| **DÜZ isabet** | **4/4** | **2/4** | yatay 2/2→**0/2**, çapraz 2/2→2/2 |
| DÜZ medyan mesafe / kutu oranı | 35.5 m / %65 | **24.2 m / %85** | daha yakın, daha iyi temas |
| DAİRE 60 m süre / yarıçap | 107.5 s / 50.7 m | **173.5 s / 38.1 m** | |
| mekanizma: tavan bağlar / `v_los` | %0 / 20.25 | **%94-99 / 15.63** | kapı sonuna kadar açık |

**Karar kuralı → KULLANICIYA** (GİRER'in 3. şartı düştü: düz isabeti azaldı;
ÇIKAR şartlarının hiçbiri oluşmadı). İlan ederken yazdığım "karede kazanıp
düzde kaybeden bölünmüş sonuç" hâli aynen çıktı.

**⚠ ÜÇ SENARYODA TEK İMZA — kalıcı bulgu:** Ö5 **takibi belirgin
iyileştiriyor, bitirişi bozuyor.** Düzde hedefi kontrol kolundan DAHA İYİ
görüyor (%85 vs %65 kutu) ve yine de vuramıyor — yani "göremedi" açıklaması
elenir. Sebep tek satırda:

> `V_TERMINAL = 16 m/s`, hedef 15.1 m/s → kalan kapanma **0.9 m/s**.
> Ö5 hızı 12-16 m/s'ye kısınca bu marj sıfırlanıyor; araç hedefi mükemmel
> izler ama asla yetişemez.

Bu, **"hızı kısmak" ailesindeki HER özelliğin** (Ö5, Ö11, Ö-B) neden aynı
duvara çarptığını açıklar. Sıradaki adım bu ailenin devamı olmamalı.

**⚠ Kendi öngörümü düzelttim:** dairede "kalıcı yavaşlık → geride kalır"
demiştim. Yavaşlık gerçekleşti (`v_los` 19.27 → **11.92 m/s**, hedef 15.1)
ama araç geride KALMADI, daha yakın durdu — dairede kaybedilen şey hız
değil geometri; yavaşlayınca yarıçap 50.7 → 38.1 m'ye düşüp içeriden
kestirme yapıyor.

---

## ⚑ ŞU AN SINANAN: `PSC_JERK_XY = 12` — kullanıcı uçuruyor (2026-08-17)

Varsayılan **10 → 12** yapıldı (kullanıcı kararı). Gerekçe: S/X kampanyası
jerk'in **tek bağlayıcı kısıt** olduğunu kanıtladı ve salınımın jerk'ten
GELMEDİĞİNİ gösterdi (komutun istediği yanal ivme jerk 5/10/15'te birebir
aynı: 8.36 / 8.43 / 8.38 m/s²). Dolayısıyla 15'in çeviklik kazanımına
yaklaşıp 15'in terminal bedelinden (nişan sapması 30 px, düz isabet 2/4)
kaçınmak mümkün olabilir.

**⚠ 12 HENÜZ ÖLÇÜLMEDİ.** 5/10/15 ölçüldü; 12 ara değerdir ve kullanıcı
kendi uçuşuyla sınıyor. Sonuç geldikçe bu maddenin altına yazılacak.

Panel düğmesi **12 ↔ 15** — uçuş sırasında etki ETMEZ, `stop_chase →
start_chase` gerekir.

---

## D — YATAY JERK TARAMASI (`PSC_JERK_XY` 5 / 10 / 15) · 24 UÇUŞ · varsayılan 10 → **12'ye çekildi**

**Tam rapor:** `docs/kampanya/D_JERK_TARAMA.md` (ölçütler koşmadan önce ilan
edildi) · 12 kare + 12 düz, n=4/seviye, dönüşümlü, tür-eşli.

**Sebep:** kullanıcı jerk 15'i kendi uçurdu, "çevik, geride kalmıyor — ama
kontrolsüz ve salınımlı" dedi, **"en iyi değerine çekelim, 10 olabilir mi"**
diye sordu.

| | jerk 5 | **jerk 10** | jerk 15 |
|---|---|---|---|
| KARE medyan mesafe (birincil) | 66.0 m | **59.5 m** | 54.5 m |
| KARE 60 m içinde süre | 96 s | **122 s** | 134 s |
| KARE kutu oranı | %30.2 | **%38.6** | %41.5 |
| DÜZ isabet | 4/4 | **3/4** | 2/4 |
| DÜZ terminal nişan sapması (son 2 s) | 7 px | **8 px** | **30 px** |
| DÜZ terminal yanal ivme | 0.44 | **1.19** | 4.49 m/s² |
| mekanizma: jerk p90 | 6.73 | **10.49** | 13.33 |

**JERK 5 ELENDİ** — karede %17 daha uzak, kutu oranı en düşük.
**10 ile 15 arası n=4'te AYRILMADI → §5.6 gereği karar kullanıcıda.**
Varsayılan ölçümün önerdiği yere (10) çekildi; panel düğmesi **10 ↔ 15**.

**⚠ İKİ ÖNEMLİ BULGU:**

1. **"Salınım" ölçütü geçersiz çıktı, ilan edilen eşik yakaladı.** `cx`
   işaret değişimi yalnız kutu olan karelerde sayılabiliyor; jerk 5, 10-20 m
   kovasında 430 kare verirken 15 → 806 kare veriyor. Kutu oranı üç seviyede
   de %60'ın altında → ölçüt karardan ÇIKARILDI (§5.2). Yerine
   **`telem.csv` 10 Hz, koşulsuz** kaynaktan rota açısı salınımı ölçüldü:
   **hiçbir seviye ayrılmıyor** (p=0.29-0.57). Jerk arttıkça araç daha SIK
   dönüş değiştirmiyor, her dönüşü daha SERT yapıyor (|ψ̇| p90 15.7 → 24.3
   °/s) ve hedefi kadrajda merkeze daha YAKIN tutuyor (|cx| p90 81 → 62 px).

2. **Karedeki salınımın kökü jerk DEĞİL, dönüş hızı doygunluğu (§5.11).**
   ω_max = g·tan(45°)/16 m/s = **35 °/s**; ölçülen LOS dönüş hızı karede
   **medyan 42-57 °/s, kutulu karelerin %65-80'inde ω_max'ın ÜSTÜNDE**.
   Düzde bu oran %10-12 — düzün çalışıp karenin çalışmamasının sebebi bu.
   Hiçbir jerk değeri bunu değiştiremez (jerk formülde yok). Kaldıraçlar
   `V` ve `ANGLE_MAX`. Bkz. `docs/kampanya/D_JERK_TARAMA.md` §1 ve §8.

---

## ADAY C — YATAY JERK (`PSC_JERK_XY` 5→15) · ÖLÇÜLDÜ · **D TARAMASI YERİNE GEÇTİ**

> ⬆ Bu maddenin sorusu D kampanyasında üç seviyeli olarak yeniden soruldu ve
> cevaplandı. C'nin düz regresyon bulgusu (isabet 4/4 → 0/4) D'de bağımsız
> olarak DOĞRULANDI (4/4 → 2/4, aynı yön), mekanizması da aynı çıktı.

**Tam rapor:** `docs/kampanya/C_JERK.md` · 18 uçuş (8 kare + 8 düz + 2 daire).

⛔ Önce bir düzeltme: raporda (`docs/donus_acigi.html`) bu adayı
`WPNAV_JERK` diye önermiştim — **yanlıştı, uçsaydık ölü koşu olurdu**.
GUIDED hız modu `wp_nav`'ın yol üreticisini kullanmıyor; bağlayan parametre
`PSC_JERK_XY` (AC_PosControl.cpp:439). Sert gövde tavanı 19.41 m/s³.

**KAZANIM (kare, n=4/kol, tam ayrışma, p=0.057):**

| ölçüt | K5 | C15 |
|---|---|---|
| **60 m içinde süre (birincil)** | 68.5 s | **124.0 s** (+%81) |
| medyan mesafe (geçerlilik eşi) | 74.6 m | **59.3 m** ✓ aynı yönde |
| dönüş yarıçapı | 118.7 m | **89.9 m** |
| en yakın menzil | 19.07 m | **5.31 m** |
| mekanizma: jerk p90 | 7.26 | **14.57** |

**REGRESYON (düz, n=4/kol, tam ayrışma):** **isabet 4/4 → 0/4**,
en yakın 1.20 → 2.23 m. *(n, ÇIKAR kapısını güçlendirmek için 2'den 4'e
çıkarıldı — özelliğin aleyhine, §5.6.)*

**Regresyonun mekanizması ölçüldü** — temastan önceki 2 s:
yanal ivme **0.57 → 3.88 m/s²** (6.8×), hedefin merkezden sapması
8.5 → 21.5 px. Kovalamada işe yarayan çeviklik, terminalde piksel
gürültüsünü kovalamaya dönüşüyor.

**Daire (yapısal öngörü, n=1/kol):** fark yok (3.90 → 3.30 m) — ilan edilen
öngörü buydu, sürekli dönüşte jerk alakasız. **Model çürütülmedi.**

**Karar:** ilan edilen kural **ÇIKAR** diyor (düz isabet kapısı). §5.10
gereği gerileme varsa kararı kullanıcı verir. **Önerim:** global hâlini
çıkar, **C2 = faza göre jerk** (kovalamada 15, terminalde 5) ölçülsün.
⚠ C2'nin uygulanabilirliği açık soru: parametre uçuş sırasında okunmuyor;
alternatifi jerk'i araç parametresi yerine güdümün kendi komut
yumuşatmasında kurmak.

---

## Ö-N — GÖRSELİ BIRAKMA EŞİĞİ (`KAYIP_M` 20→40) · ÖLÇÜLDÜ, KARAR KULLANICIDA

**Tam rapor:** `docs/kampanya/ON_KAYIP_ESIK.md` (ölçütler koşmadan önce ilan
edildi, sonuç + video bacağı orada).

12 uçuş: 8 kare (kazanım zarfı, n=4/kol) + 4 düz (regresyon, n=2/kol).
Mekanizma kapısı sonuna kadar açık: K20'de 76/85 faz tam 19'da, N40'ta 43/50
faz tam 39'da bitti.

| ölçüt (kare) | K20 | N40 | |
|---|---|---|---|
| BİRİNCİL en yakın menzil | 13.82 m | 11.21 m | **p=0.83 → GÜRÜLTÜ** |
| kutu oranı | %31.2 | %22.4 | −8.8 puan (sınır 5'ti) |
| mutlak kutulu kare | 678 | 566 | −%17 |
| medyan mesafe | 68.6 m | 98.7 m | **+%44 kötü** |
| 60 m içinde süre | 94 s | 57 s | **−%39** |
| isabet | 0/4 | 0/4 | düz |

Düz regresyon: N40 2/2, K20 1/2 — ama N40'ın vuruşları 174/196 s'de,
K20'ninki 120 s'de. n=2, hüküm kurulmaz.

**Karar (ilan edilen kurala göre): KULLANICIYA** — GİRER koşulu (2) çiğnendi,
ÇIKAR koşulu (birincil geriler) sağlanmadı. **Yapay zekânın önerisi: ÇIKAR.**

**⭐ Kampanyanın asıl bulgusu:** `kayip_kare_esik` kaldıraç değilmiş; ama
**"en yakın menzil", isabet üretmeyen senaryolarda kötü bir birincil ölçüt.**
240 s'lik uçuşun tek şanslı anını ölçüyor. Kare/manevra senaryolarında
birincil ölçüt bundan böyle *60 m içinde geçen süre* ya da *medyan mesafe*
olmalı.

---

## Ö-K — KÖR DEVAM · ÖLÇÜLDÜ, ELENDİ, SİLİNDİ (2026-08-15)

**Fikir:** kutu kaybolunca güdüm son komutu 1 s (20 kare) boyunca DONDURUYOR.
Dairede kerteriz 21.5°/s döndüğü için o komut anında bayatlıyor. Ö-K,
dondurmak yerine son ölçülen LOS azimut hızıyla nişanı döndürmeye devam
ediyordu (40° tavanla). D0 uyumluydu — hedefin GPS'ini kullanmıyordu.

**Ölçüm (K01-K12, 12 uçuş, dönüşümlü A/B, daire + duz):**

| ölçüt | kapalı (K) | **açık (O)** | karar kuralı |
|---|---|---|---|
| en yakın menzil, daire (medyan) | **3.24 m** | 4.28 m | birincil — GERİLEDİ |
| isabet, daire | 0/5 | 0/5 | düz |
| isabet, duz | 2/2 | 1/2 | ara veri (n<4) |

**Karar: GİRMEZ.** Birincil ölçüt düz kaldı, en yakın menzil kötüleşti.
Mekanizma kapısı (§5.1) geçilmişti — `kor_don_deg` deney kolunda gerçekten
dönüyordu — yani özellik çalıştı ama işe yaramadı.

**Neden işe yaramadı (hipotez, ölçülmedi):** bayat bir λ̇ kestirimiyle
ekstrapolasyon, kutu 50-100 m'de zaten %11-16 tespitle geldiği için
gürültülü bir λ̇ üzerine kuruluyordu. Yanlış yöne döndürmek, dondurmaktan
daha kötü.

**Silme:** §5.12 uyarınca `Cfg.KOR_DEVAM`/`KOR_MAX_DEG`, `AVCI_IBVS_KOR`,
döngü bloğu, `kor_don_deg` sütunu, panel düğmesi, B62-B67 testleri ve
`~/.avci_sim/kosuK.sh` çıkarıldı. Doğrulama: grep sıfır + `komut()` 7776
girdi kombinasyonunda **bit bit aynı** (fark 0).

---

## DURUM — 2026-08-09 · MANEVRA (en güncel)

Kullanıcı gözlemi: "düz uçuşta ıskalamıyor, hedef **manevra** yapınca görsel
güdüm sapıtıyor, yatayda çok salınım oluyor."

Altı uçuş koşuldu (daire senaryosu, 210 s, aynı profil, koşu başına TEK
değişken). Hepsi geçerli: hedef 20-250 m bandında, hız 14.8-15.1 m/s.
Videolar `logs/manevra_*.mp4`, ölçüm aracı `manevra.py` + `kayip.py`.

### M1 — Yatay roll/pitch telafisi (T1a) · UÇUŞTA DOĞRULANDI ✓ · varsayılan AÇIK

`AVCI_IBVS_ROLL=0` → eski yol. Kök neden ae2c600'de.

| ölçüt | A1 telafisiz | A2 telafisiz | B1 **telafili** | B2 **telafili** |
|---|---|---|---|---|
| yatay hata medyan | 66.5 px | 53.0 px | **50.2** | **44.5** |
| salınım (işaret değişimi/s) | 0.104 | 0.143 | **0.000** | **0.057** |
| görsel temas oranı | %53.6 | %36.7 | **%64.4** | **%56.2** |
| İMHA | ✗ | ✗ | **✓** | ✗ |

Üç ölçüt de her iki çiftte telafi lehine (6/6). **Ama manevrayı ÇÖZMÜYOR:**
210 s'de hâlâ ~12 temas kopuşu, 2 koşuda 1 vuruş.

### M2 — Tespit eşiği 0.35 → 0.15 · ÖLÇÜLDÜ, HENÜZ VARSAYILAN DEĞİL

Kopuşların **%100'ünde hedef hâlâ kadrajın İÇİNDE**; kopuştan önceki 5 karede
güven medyanı 0.39, min 0.35 = `CONF_MIN`. Yani dedektör görüyor, güdüm eşikte
atıyor. `AVCI_POSE_CONF=0.15 AVCI_IBVS_CONF=0.15` ile:

| ölçüt | B1/B2 (0.35) | C1/C2 (**0.15**) |
|---|---|---|
| yatay hata medyan | 50.2 / 44.5 px | **17.0 / 15.5** |
| yatay hata p90 | 197 / 154 px | **102.5 / 54.5** |
| toplam temas süresi | 37 / 53 s | **88 / 111 s** |
| İMHA | 1/2 | 0/2 |

Takip 3× iyi, temas 2× uzun — **ama vuruş yok.** Varsayılan yapılmadı: (a) düz
uçuş gerilemesi ölçülmedi, (b) düz eşik yerine histerezis olmalı (yakala 0.35,
tut 0.20), (c) vuruşu engelleyen darboğaz M3.

### M3 — Lead kapısı kaldırıldı · 3'e 3 UÇULDU → NÖTR (varsayılan KAPALI)

`AVCI_IBVS_LEAD_ERKEN=1` → açılır. Kod, kill-switch ve testler (B33-B37) duruyor.

Yatay lead `if terminal:` kapısının arkasındaydı; mandal 6.4 m'de kapandığı
için lead ancak son 6 metrede çalışıyordu. `lead_olcek` o noktaya kadar zaten
1.0 — **sönüm kusurlu değildi, KAPI kusurluydu.** Kapı kaldırıldı.

⚠ **İLK HÜKÜM (n=2) YANLIŞTI.** "Yaklaşmayı bozdu" demiştim; o kıyasta kontrol
koluna şanslı bir isabet denk gelmişti. Kullanıcı itiraz etti, 3'e 3 DÖNÜŞÜMLÜ
(K,M,K,M,K,M) kampanya koşuldu — altısı da geçerli, her biri 210 kare + video.

**Kapanma ölçütü DÜZELTİLDİ (video log'u yakaladı):** panel `mesafe` 1 Hz, ama
buluşmadaki kapanma hızı medyan 4.9-12.4 / p90 13-22 m/s. 1 Hz örnekleme
gerçek en yakın anı 15 m'ye kadar ıskalıyor. Panelin "4.8 m" dediği karede
hedef kadrajda ~20 px'ti (4.8 m'de ~45 px olmalı). Yakınlık artık 20 Hz bbox
logundan, kutu boyutundan ölçülüyor (`yakinlik.py`).

| ölçüt (20 Hz, örtüşmesiz) | kontrol n=3 | M3 n=3 |
|---|---|---|
| İMHA | 0/3 | 0/3 |
| tepe kutu boyutu (medyan) | 27.9 px | 26.9 px |
| ≥20 px (≈≤8 m) kare | 13 | 12 |
| ≥30 px (≈≤5 m) kare | 2 | 1 |
| yatay hata p90 | 110 px | **99** |
| toplam temas süresi | 56.4 s | **59.9 s** |

**Koşular arası değişkenlik kol farkını YUTUYOR**: K1 tepe 76.5 px, K2 22.7 px
— aynı kolda 3×. n=2 ile karar vermenin neden yanıltıcı olduğu tam olarak bu.

**Mekanizma hükmü:** lead tasarlandığı gibi çalışıyor (20-35 m'de 8.7°,
13-20 m'de 19.3°, 8-13 m'de 25° doymuş) **ama λ̇ DÜŞMÜYOR** (13-20 m'de
0.72 → 0.84 rad/s) ve doyma oranı iyileşmiyor (%71 → %79). Burnu öne almak,
aracın sahip olmadığı yanal ivmeyi yaratmıyor.

**KARAR: nötr → varsayılan KAPALI.** Zarar verdiği için değil; ölçülebilir
hiçbir şey değiştirmeden %82 doyan bir terim eklediği için.

### M5 — KAÇAMAK TESTİYLE ÖLÇÜLEN ASIL DARBOĞAZ: manevra sonrası HIZ ÇÖKÜŞÜ

Kaçamak testi mimarisiyle (bkz. CLAUDE.md §3.3) 16 uçuş. Kullanıcının kendi
uçuşu (`logs/kayit/ucus_20260810_103525`) da aynısını gösteriyor.

**İSTİSNASIZ HER KOŞUDA**, kaçamaktan sonraki 15 s içinde:

    drone hızı  7.7-13.9 m/s'ye düşüyor      hedef 15.4-16.3 m/s
    açılan mesafe 48-147 m
    ⇒ hedeften YAVAŞKEN mesafe matematiksel olarak kapanmaz

Vuruş ancak hedef DÜZ uçmaya dönünce oluyor. Kullanıcının tarifi birebir:
"manevra sırasında mesafe kapatılamıyor, hedef çok uzağa gidiyor."

**KÖK NEDEN — hız yasası saf bir MENZİL düzenleyicisi, hız farkını hiç
görmüyor:**

    hata  = BOYUT_REF − boyut = 25 − boyut        K_I = 0.04 (m/s)/(px·s)
    hiz_I = clamp(hiz_I + K_I·hata·dt, 0, 24)
    v_los = hiz_I + K_FWD·hata                    (IBVS/seyir durumunda)

Yakın geçişte kutu 88-102 px'e çıkıyor → hata = −63…−77 → integral
**saniyede 3.1 m/s düşüyor**. Normal hata ≈ +15'te ise **saniyede 0.6 m/s**
toparlanıyor — **5:1 asimetri**. Kullanıcının uçuş logunda birebir:
hiz_I 15.1 → 12.0 (2 s) → geri çıkması ~5 s.
Ve seyirde v_los = 11 + 0.35·11 ≈ **14.9 m/s** — hedefin 15.1'inin ALTINDA.

**ÖNERİLEN SIRA (her biri ayrı test, tek değişken):**

1. **Ö1 · Kapanma hızı geri beslemesi.** ṙ zaten hesaplanıyor (KAPANMA,
   dikey kanal için). Hıza da ekle: `v_los = hiz_I + K_FWD·hata − K_D·ṙ`.
   Hedef kaçmaya başladığı ANDA hız artar; integralin 5 saniyesini beklemez.
2. **Ö2 · İntegral tabanı = hedefin seyir hızı.** `I_MIN=0` şu an; görsel
   temas varken komut hedefin hızının altına düşmemeli. Taşıyıcı (`ff_hiz`)
   yalnız başlangıç değerini veriyor, tabanı tutmuyor.
3. **Ö3 · Asimetrik integral** — hızlanma yönü yavaşlama yönünden hızlı olsun.
4. **Ö4 · T1b dikey roll telafisi** — ölçülen 33° işaret hatası (aşağıda).
5. **Ö5 · Yatış-farkında hız bütçesi** — komut 18 m/s, yatıktayken ulaşılan
   10.8-14.3 m/s.

## ⚑ Ö12 · YAKIN MENZİLDE YAW TAVANI — "kendi ekseninde dönme" (8 uçuş)

`AVCI_IBVS_YAW_MENZIL` (varsayılan 0 = kapalı; denenen 15 m).
`tavan = YAW_RATE_MAX · clamp(R/15, 0.35, 1)` → 20 m'de 120°/s (değişmez),
8 m'de 64°/s, 3 m'de 42°/s.

**KÖK NEDEN (ölçüm):** 30 koşunun 10'unda kurtarma bekçisi tetiklenmiş.
T09'da tetikten hemen önce: cx 208→222→262→280 (pas geçiş, hedef kadrajı
tarıyor), yaw komut hızı 122/118/122 °/s — **YAW_RATE_MAX tavanında sürekli
kaçıyor**. Aracın gerçek yaw hızı 300°/s'yi aşınca bekçi güdümü kesiyor →
araç olduğu yerde dönüyor. Menzil küçüldükçe hedefin açısal hızı 1/R ile
patlıyor (8 m'de ~100°/s, 2 m'de ~400°/s) — araç zaten TAKİP EDEMEZ.

**⚠ YAPISAL GARANTİ (kullanıcının şartı):** yaw slew sınırı YALNIZ BURNU
etkiler. Hız vektörü `hiz_yonu`ndan hesaplanır ve bu sınırdan GEÇMEZ.
Birim testi **B67**: 32 girdi kombinasyonunda `komut()` çıktısı (vx,vy,vz,yaw)
BİT BİT AYNI. Yani uçuş yolu/kesişim geometrisi değişemez.

| ölçüt | KONTROL n=4 | Ö12 n=4 |
|---|---|---|
| KURTARMA yaşayan koşu (birincil) | 1/4 (19 kare) | **0/4 (0 kare)** |
| yaw komut doyma medyanı | 17% | **5%** |
| İSABET | 3/4 | 3/4 — gerileme yok |
| en yakın menzil | 0.70 m | 0.64 m — gerileme yok |

**MEKANİZMA ÇALIŞIYOR** (doyma %17 → %5). Ama KURTARMA olayı seyrek
(taban ~%33); 1/4 → 0/4 farkı **n=4'te gürültüden ayrılamaz** (§5.4).
⚠ Yan gözlem: `yatay` SAĞA AŞIM 41.3 → 69.4 m (n=2v2) — gerileme olabilir,
DOĞRULANMALI.

⇒ Karar için n=8+/kol gerekiyor (seyrek olay). Varsayılan KAPALI.

### ⚑ Ö11 DAİRE REGRESYONU KOŞULDU (§5.10) — KARAR KURALI TETİKLENDİ

Kullanıcının öngördüğü risk sınandı: `circle` senaryosu, 4 uçuş (2v2),
kaçamak `yok` (sürekli manevranın kendisi zaten zorlayıcı).

| ölçüt | KONTROL n=2 | Ö11 n=2 | karar kuralı |
|---|---|---|---|
| **en yakın menzil** (birincil 1) | 5.15 m | **8.49 m (+65%)** | ⚠ **%30 eşiği AŞILDI** |
| Ö11 tetik oranı (birincil 2) | — | 3% | eşik %30 — aşılmadı |
| isabet | 0/2 | 0/2 | — |
| görsel temas | 41% | 46% | — |

**KARAR: Ö11 VARSAYILAN YAPILAMAZ.** Önceden ilan edilen kural tetiklendi.

⚠ İki nüans, dürüstlük için:
1. **Korktuğum mekanizma DEĞİL.** "Sürekli 9 m/s'de kilitlenir" diye
   düşünmüştüm; tetik oranı yalnız %3. Yani araç sürekli yavaş kalmıyor —
   ama yine de dairede daha uzakta kalıyor. Sebep başka; anlaşılmadı.
2. **n=2 ve ham değerler ÖRTÜŞÜYOR:** kontrol [2.45, 7.85], Ö11 [13.42,
   3.57]. Medyan farkı %65 ama dağılımlar iç içe. §5.4 gereği bu bir
   "hüküm" değil, **kuralın tetiklediği bir DURDURMA**.

⇒ Ö11 kapalı kalır. Girmesi için: daire kolu n=4+ ile doğrulanmalı VE
gerileme sürerse tetik daraltılmalı (ör. yalnız kaçamak sonrası ilk N
saniye, ya da `boyut` yakın geçişi doğrulayan ek şart).

Kullanıcı haklı olarak sordu: "hedef sürekli sert manevra yaparsa, daire
çizerse Ö11 takibi kötüleştirmez mi?" Ö11 yalnız `kapanma < −5 m/s` VE
`|eps_yaw| > 45°` iken tetikleniyor; sürekli dönen hedefte bu koşul sık
sağlanabilir ve araç sürekli 9 m/s'de kalabilir → hedefi hiç yakalayamaz.
**Bu senaryoda HİÇ test edilmedi.** Ö11 varsayılan yapılmadan önce
`circle` senaryosunda kontrol/Ö11 kıyası ZORUNLU.

## ⚑ Ö11 · ISKA SONRASI DÖNÜŞ YAVAŞLAMASI — 12 UÇUŞ (2026-08-12)

`AVCI_IBVS_DONUS_YAVAS` (varsayılan 0 = kapalı; denenen 9.0 m/s).
Tetik yalnız kutudan: `kapanma < −5 m/s` (hedefi geçtik) **ve**
`|eps_yaw| > 45°` (dönmemiz gerek). Durum tutmaz — dönüş ilerledikçe
eps_yaw küçülür ve kendiliğinden serbest bırakır. CANLI GPS YOK, D0 temiz.

**Gerekçe (ölçüm):** 66 m'lik aşım bir salınım değil, 18 m/s'deki minimum
dönüş çemberi (2R = 66 m). R ∝ V² olduğu için 9 m/s'de 2R = 17 m.

**MEKANİZMA (§5.1):** deney kolunun 3 koşusunda yavaşlama 8-18 karede aktif,
hız 9.0 m/s'ye indi. **T12 koşusunda HİÇ tetiklenmedi → GEÇERSİZ sayıldı.**

**YATAY KIRILMA — kullanıcının şikâyet ettiği manevra (n=4 vs 3):**

| ölçüt | KONTROL | Ö11 | değişim |
|---|---|---|---|
| **SAĞA AŞIM** (birincil) | 61.8 m | **24.2 m** | **−61%** |
| İSABET (birincil) | 2/4 | 2/3 | ~ |
| en yakın menzil | 0.44 m | **0.36 m** | −19% |
| yanal medyan | 3.05 m | **1.80 m** | −41% |
| yandan yana geçiş | 3.5 | 4.0 | +14% |

Ham aşım: kontrol [22.8, 56.8, 66.8, 69.6] → Ö11 [22.1, 24.2, 28.9].
Kontrolün 4'ünden 3'ü Ö11'in TAMAMININ üstünde; bir kontrol koşusu (22.8)
Ö11 bandına giriyor — tam ayrışma YOK ama yön çok net.
Ölçülen etki, fizikten ÖNCEDEN hesaplanan değerle uyuşuyor (2R: 66→17 m
tahmin, 62→24 m ölçüm).

**ÇAPRAZ kaçamak (n=2v2, AZ VERİ):** aşım 36.1 → 42.1 m (kötüleşti),
en yakın 1.3 → 0.6 m (iyileşti). Ham [36.3, 36.0] → [48.9, 35.4] — bir
koşu kötü, örtüşüyor. **Hüküm için yetersiz (§5.4).**

⇒ **DURUM:** yatayda güçlü ve fizikle tutarlı kazanım; çaprazda belirsiz.
Varsayılan KAPALI, karar kullanıcıda. Çapraz kolu n=4'e çıkarılmalı.

## ⚑⚑ GECE ÇALIŞMASI SONUCU (2026-08-11 gecesi, 22 uçuş) — ARIZA YERİ DEĞİŞTİ

### 0 · ⚠⚠ İŞARET HATASI — GECE RAPORUNUN BİR HÜKMÜ YANLIŞTI (2026-08-12)

`tools/salinim.py`'de gidiş yönünün SAĞI `(hy, −hx)` yazılmıştı; NED'de
(x=kuzey, y=doğu, z=aşağı) doğrusu `ẑ×ĥ = (−hy, hx)`. Yazdığım ifade SOLU
veriyordu → "SAĞA AŞIM" sütunu TERS TARAFI ölçüyordu.
Etkilenmeyenler: en yakın menzil, isabet, temas %, |yanal| medyanı, yandan
yana geçiş sayısı (işaretten bağımsız). Etkilenen: `asim_sag`.
Düzeltildi + `tests/test_salinim.py` bekçisi (5/5). Hata GPT Codex tarafından
bulundu, matematiği bağımsız doğrulandı.

**DÜZELTİLMİŞ RAKAMLAR (aynı 22 uçuş, yeniden hesap):**

| ölçüt | KONTROL | Ö9 | KONTROL | Ö5 |
|---|---|---|---|---|
| **SAĞA AŞIM** | 39.00 m | **25.35 m** ✅ | 37.40 m | 43.20 m ❌ |
| yandan yana geçiş | 1 | 1 | 2 | **1** |
| yanal medyan | **5.90 m** | 6.70 m | **6.20 m** | 8.50 m |
| en yakın menzil | **1.82 m** | 2.33 m | **1.89 m** | 3.23 m |
| isabet | 0/6 | 0/6 | 0/5 | 0/5 |

### ⚑ TAZE A/B + EŞLENMİŞ ANALİZ (2026-08-12, 10 uçuş) — "%35" DE ŞİŞİKMİŞ

Taze kampanya (S01-S10) koşuldu. İki şey çıktı:

**1) Kol dengesizliği (benim kurulum hatam):** kontrolde 3 yatay + 2 capraz,
Ö9'da 2 yatay + 3 capraz. `yatay` aşımı ~66 m, `capraz` ~31 m olduğu için
kaba medyan kıyası (65.7 → 34.8) KARIŞIM ORANINI ölçüyor, kolları değil.
→ CLAUDE.md §5.9 olarak kural yazıldı.

**2) Tür-içi eşlenmiş gerçek etki (22 uçuş birleşik):**

| manevra | KONTROL | Ö9 | fark |
|---|---|---|---|
| yatay (n=6v5) | 66.8 m | 56.0 m | **−16%** |
| capraz (n=5v6) | 31.5 m | 29.9 m | **−5%** |

Ö9'un etkisi GERÇEK ama MÜTEVAZI (%5-16), gece iddia ettiğim %35 değil.
Koşu-içi değişkenlik çok yüksek (Ö9 yatay ham: 5.0 / 23.1 / 56.0 / 60.7 /
66.8 m). İsabet ve en yakın menzilde tür-içi fark YOK.

### ⚑⚑ ASIL BULGU: 66 m AŞIM BİR SALINIM DEĞİL, DÖNÜŞ ÇEMBERİ

Aşımın NE ZAMAN olduğu ölçüldü — beş koşuda da **tetikten tam +7 s sonra**
ve **66-69 m**:

    R = V²/(g·tan45°) = 18²/9.81 = 33 m   →   U-dönüşü 2R = 66 m

Yani drone hedefi geçiyor ve geri dönmek için minimum çemberini çiziyor.
En yakın an ise çok SONRA (+68…+145 s) geliyor.
⇒ Bu bir kazanç/nişan sorunu DEĞİL. Ö5/Ö8/Ö9'un üçü de yanlış katmandaydı.
⇒ Çare: dönüş anında HIZI KISMAK. R ∝ V²: 18 m/s'de 2R=66 m, 9 m/s'de
2R=17 m — dört kat dar.

⇒ **Ö9 hakkındaki "fayda yok" hükmüm YANLIŞTI.** Ö9 kullanıcının şikâyet
ettiği şeyi — kaçamak yönünde aşım — **%35 azaltıyor**. Bedeli: en yakın
menzil 1.82 → 2.33 m. Ö5 ise doğru işaretle daha da kötü çıktı.
⇒ Ö9 TAZE A/B ile yeniden sınanacak; birincil ölçüt SAĞA AŞIM.

⚠ **GÜDÜM DAVRANIŞI ETKİLENMEDİ:** işaret hatası yalnız ÖLÇÜM aracındaydı.
Gece öncesi etiket ↔ bugün, 1080 girdi kombinasyonunda güdüm çıktısı BİT BİT
AYNI (fark 0, sapma 0.00e+00). Gece eklenen her şey varsayılan-KAPALI
seçenek + ölçüm aracı + belgeydi.

### 1 · Doğru salınım ölçütü kuruldu: `tools/salinim.py`

Kullanıcı eski ölçütü çürüttü: "salınımı bbox'ın oynamasından ölçme; bizim
dronun hedefe göre hareketine bak — hedefin solundayken sağına geçiyor mu,
hedef sağa kırınca biz onun DAHA DA sağına mı taşıyoruz?"

Yeni araç iki aracın GPS'inden hedefin çerçevesini kurar:
    ĥ = hedefin gidiş yönü ; n̂ = ĥ'nin 90° sağı
    yanal  = (drone − hedef)·n̂   (+ = drone hedefin SAĞINDA)
    boyuna = (drone − hedef)·ĥ   (− = drone ARKADA)
Kutuya HİÇ bakmaz → görsel temas kopsa bile ölçer (eski ölçütün kusuru buydu:
hedefi kaybeden koşu "sakin" görünüyordu).
Geometri doğrulandı: |yanal²+boyuna²| − mesafe = 0.0000 m.

### 2 · ⚠⚠ İKİ KEZ DÜZELTİLDİ — "geri dönemiyor" hükmü YANLIŞTI

İlk hüküm: "22 koşunun 21'inde drone geri dönemedi, asıl arıza bu."
**BU YANLIŞTI ve kendi §5.2 kuralım yakaladı.** Kaçırmadan sonraki gözlem
süresi MEDYAN 6 s; 22 koşunun 13'ünde 10 s'den az. 76 m'yi ~2.5 m/s
kapanmayla kapatmak ~30 s ister. Geri dönebilen tek koşu (P07), gözlem
süresi en uzun olan koşuydu (40 s) — yani "dönebilen" değil "zamanı olan".

KAYIT 230 s'ye uzatılıp GERÇEK ölçüm yapıldı (R01, R02):
    R02: t=88 tetik → t=115'te 283 m'ye açıldı → t=211'de 0.11 m'ye DÖNDÜ
    R01: t=94 tetik → t=145'te 0.33 m ve İSABET
**Yeniden temas ÇALIŞIYOR**, ~50-100 s sürüyor. 100 s'lik kayıtlar bunu
yapısal olarak göremiyordu. Kaçamak testinin varsayılan süresi ARTIRILMALI.

### 3 · ⚑ GERÇEK DURUM: kesişim ÇÖZÜLDÜ, kalan iş DİKEY SANTİMETRELER

Uzun kayıtlı iki koşunun temas anı, bileşenlerine ayrılmış:

| koşu | yatay | dikey | sonuç |
|---|---|---|---|
| R01 | 0.33 m | **+0.05 m** | **İSABET** |
| R02 | 0.12 m | **−0.11 m** | ıska (zarf sınırında) |

Zarf: yatay ±0.65 m, dikey **+0.29 / −0.13 m** — dikey eksen **5 kat dar**.
Güdüm kesişimi 10-40 cm'ye kadar çözüyor; isabetle ıska arasındaki fark
artık SANTİMETRE ve DİKEY eksende.

### 3b · T1b KODLANDI ama UÇULMADI — mekanizma kapısı (§5.1) uçmadan eledi

`AVCI_IBVS_DIKEY_ROLL` (varsayılan 0). Dikey hataya roll telafisi FARK
olarak eklenir; pitch iki terimde de aynı bırakılır (nişan noktası CY_NISAN
seyir pitch'iyle birlikte ayarlanmıştı — onu kaydırmak ayrı bir değişken).

⚠ **"Dikeyde 33° okuma hatası" İDDİAM GEÇERSİZDİ.** İki FARKLI büyüklüğü
kıyaslıyordum (seviye yükselişi ↔ piksel farkı hatası). Birim testi B58 bunu
yakaladı: ilk sürüm roll=pitch=0'da bile komutu 0.51 m/s değiştiriyordu.

GERÇEK roll kaynaklı dikey sapma, hedefin kadraj merkezinden uzaklığıyla
büyür (cy=240 için):

| yatış | cx=320 | cx=360 | cx=420 | cx=500 |
|---|---|---|---|---|
| 30° | 3.5° | 10.4° | 18.0° | 23.5° |
| 45° | 7.6° | 17.1° | 27.4° | 35.1° |

Ama TERMİNAL yaklaşmada hedef merkeze yakın (|cx−320| medyan 14-18 px) ve
araç neredeyse düz (yatış medyan 4°). Gerçek uçuş kayıtlarında (R01/R02)
terminal fazdaki düzeltme **MEDYAN −0.06°** — pratikte sıfır.

⇒ **T1b, santimetrelik dikey ıskanın çaresi DEĞİL.** §5.1 mekanizma kapısı
bunu UÇMADAN önce gösterdi; 10 uçuşluk kampanya boşa gitmedi.
Kod, kill-switch ve testler (B58-B61) duruyor — dikey sapmanın büyüdüğü
başka bir rejimde (hedef kadraj kenarında + sert yatış) işe yarayabilir.

⇒ Sıradaki iş AÇIK DEĞİL. Dikey ıska bir OKUMA hatası değil; nişan noktası
(CY_NISAN) / dikey kontrol yasası sorusu. Kullanıcıyla konuşulmalı.

### 4 · (geçersiz kılınan ilk hüküm — kayıt için)

22 koşu (Ö9 kampanyası 12 + Ö5 kampanyası 10), tetik 8 m:

    en yakın menzil                       0.26 - 3.63 m  ← YAKLAŞMA ÇALIŞIYOR
    kaçırma sonrası açılan maks mesafe    MEDYAN 76 m
    20 m'ye geri dönebilen koşu           1 / 22          ← ASIL ARIZA

Drone hedefe 0.3-3.6 metreye kadar giriyor, 1-3 m ile ıskalıyor ve
**bir daha asla dönemiyor**. Kayıt penceresi (100 s) boyunca 21 koşuda
mesafe bir daha 20 m'nin altına inmedi.

Mekanizma (D09, kaçamak yatay, saniye saniye):
    t+1.5s   mesafe 2.3 m   drone 18.8 m/s   ← neredeyse temas
    t+2.5s   mesafe 15.0 m
    t+4.5s   mesafe 60.2 m  ← 2 saniyede 45 m açıldı = 22.5 m/s ayrılma
    t+5.5s   mesafe 84.9 m  drone 8.3 m/s
22.5 m/s ayrılma, |v_drone|=18.2 ve |v_hedef|=15.4 iken ancak hız
vektörleri **~84° ayrışmışsa** olur. Yani drone kaçamağı takip ederken
vektörünü sertçe çeviriyor, yanından geçiyor ve hedefe göre neredeyse DİK
uçar hâle geliyor. Sonra 18 m/s'den dönmeye çalışırken 76 m açılıyor.

**SONUÇ: bütün gece nişan/salınım/dönüş ayarları denendi (7 özellik), oysa
arıza orada değil.** Sıradaki iki yön:
  A) TERMİNAL İSABET — son 1-3 metreyi kapatmak (zarf yatayda ±0.65 m)
  B) YENİDEN TEMAS — ıskadan sonra hedefe geri dönebilmek. 22 koşuda 1 kez
     olabildi. Bunu çözen sistem her kaçırmada yeni şans kazanır; çözmeyen
     tek şansla kalır. **Muhtemelen daha değerli olan bu.**

### 3 · Ö5 — DÖNÜŞ-FARKINDA HIZ TAVANI · 10 UÇUŞ · GİRMEDİ

`v ≤ DONUS_A/λ̇` (yalnız kısar). `AVCI_IBVS_DONUS` (varsayılan 0).
Mekanizma doğrulandı: tavan karelerin %6-36'sında BAĞLADI, bağlıyken hız
10 m/s tabanına indi (yarıçap 33 m → 10 m; birim testi B53).

| ölçüt | KONTROL n=5 | Ö5 n=5 |
|---|---|---|
| SAĞA AŞIM (birincil 1) | 14.6 m | 14.2 m — berabere |
| yandan yana geçiş (birincil 2) | 2 | **1** |
| en yakın menzil | **1.89 m** | 3.23 m — **kötüleşti** |
| isabet | 0/5 | 0/5 |

Dönüş sıkışıyor ama kapanma bitiyor (1.7 kat uzakta kalıyor). Varsayılan KAPALI.

### 4 · Ö9 yeniden değerlendirildi — DOĞRU ölçütle de fayda yok

Doğru pencereyle (tetik → kayıt sonu) SAĞA AŞIM: kontrol 14.75 m, Ö9 16.45 m.
Ö9'un "sakinleştirme" bulgusu gerçek salınımı azaltmıyor.

### Ö9 — YATAY SÖNÜMLEME (D terimi) · 12 UÇUŞ · SAKİNLEŞTİRDİ, YAKLAŞTIRMADI

`yaw_cmd = iris_yaw + K_YAW·eps_yaw − SONUM_T·ω` (ω = aracın KENDİ yaw hızı).
`AVCI_IBVS_SONUM` (varsayılan 0 = kapalı; denenen 0.30 s).

**GEREKÇE — sistemin gecikmesi ÖLÇÜLDÜ:**

    kamera → komut              73 ms   (gecikme_s sütunu, 96 örnek)
    komut → gerçekleşen yaw    300 ms   (çapraz korelasyon, 490 kare)
    hız vektörünü 30° döndürmek 780 ms   (Δv 9.3 m/s ÷ MAX_ACCEL 12)

8 m'de LOS ~100°/s süpürüyor; 300 ms'de geometri 30° değişiyor. Yani
denetleyici hep BAYAT duruma göre karar veriyor. Yatay kanal saf-P
(türev terimi yok) → gecikmeli sistemde salınım YAPISAL olarak kaçınılmaz.

| ölçüt | KONTROL n=6 | Ö9 n=6 | kazanan |
|---|---|---|---|
| **yatış p90** | 21° | **14°** | **Ö9** |
| yaw komut hızı p90 | 120°/s | **115°/s** | Ö9 |
| en yakın menzil | **1.82 m** | 2.33 m | KONTROL |
| görsel temas | **%56** | %54 | KONTROL |
| isabet | 0/6 | 0/6 | — |

**Mekanizma doğrulandı (§5.1):** deney kolunun 6 koşusunda da sönüm aktif
(kare oranı %44-66, medyan 0.4-0.9°, tepe 11.9-22.0°); kontrol kolunun
6 koşusunda da tam 0.00° — kollar temiz ayrışmış.

⚠ **BİRİNCİL ÖLÇÜT DEĞERLENDİRİLEMEDİ (§5.2 kapısı devreye girdi):** salınım
yalnız temas ≥%60 olan koşularda geçerli. KONTROL'de 3 koşu geçerli
(medyan 0.092), Ö9'da **yalnız 1** — kıyas yapılamaz. Bu bir Ö9 bulgusu
değil, TEST KURULUMU bulgusu: 8 m tetikte temas oranı %42-81 arasında
sallanıyor, yarısı eşiğin altında. Salınım sorusu bu senaryoda ölçülemiyor;
15 m tetikte temas daha iyiydi, salınım oradan ölçülmeli.

**SONUÇ:** Ö9 aracı ölçülebilir biçimde SAKİNLEŞTİRİYOR (yatış p90 %33
düşüş, 6'ya 6 tutarlı) ama yaklaştırmıyor. Birincil ölçüt ölçülemediği için
karar kullanıcıya. Varsayılan KAPALI.

### Ö8 — YANAL KOMUT KAÇIRMA MESAFESİYLE · 18 UÇUŞ · SALINIMI ÇÖZDÜ, MENZİLİ ÇÖZMEDİ

Hız vektörünün yönü açıya değil, kalan sürede yanal kaçırmayı kapatacak
hıza göre belirlenir; BURUN tam hedefte kalır. `AVCI_IBVS_YANAL` (varsayılan
0 = kapalı), menzil kapısı `AVCI_IBVS_YANAL_M=12` m ile yumuşak harman.

**KÖK NEDEN (ölçüldü, O7A son 0.4 s):** 1.5 m'de `eps_yaw` 58°'ye fırlıyor ve
yaw komutu 120°/s'de doyuyor. Ama 1.5 m'de 58° = yalnız **1.3 m** yanal
kaçırma. Güdüm AÇIYA tepki veriyor, oysa önemli olan MESAFE. 18 m/s'lik
vektörü 58° döndürmek 17.5 m/s'lik değişim ister; MAX_ACCEL ile 1.45 s
sürer, geometri 0.08 s bırakır → doyma → savrulma → geri savrulma.
⚠ Ö7'nin neden boşa gittiği de buradan: yaw sınırı yalnız BURNU yavaşlatıyor,
hız vektörü zaten anında savruluyordu. Salınım HIZ kanalındaydı.

**n=3 SONUÇ n=6'DA TERSİNE DÖNDÜ** — düşük n'in neden yanılttığının kesin
kanıtı:

| | n=3 (ilk tur) | **n=6 (taze kampanya)** | havuz n=9 |
|---|---|---|---|
| en yakın medyan KONTROL | 2.40 m | **1.96 m** | 2.23 m |
| en yakın medyan Ö8 | 1.88 m | **2.76 m** | 2.31 m |
| kim önde | Ö8 | **KONTROL** | berabere |

**n=6 kampanyasının ölçütleri:**

| ölçüt | KONTROL n=6 | Ö8 n=6 |
|---|---|---|
| **cx salınımı /s** (birincil 2) | 0.073 | **0.000** |
| yatış p90 medyanı | 22° | **17°** |
| en yakın menzil medyanı | **1.96 m** | 2.76 m |
| ≤2.5 m'ye giren | **4/6** | 3/6 |
| isabet | 1/6 | 0/6 |
| komut kısma (mekanizma) | %0 | %17 |
| 25 m'de gerileme | — | **yok** (13.21 → 12.14 m) |

**SONUÇ — bölünmüş:** Ö8 **salınımı tamamen ortadan kaldırıyor** (0.073 → 0.000,
yatış 22° → 17°) ama menzili kapatmıyor (1.96 → 2.76 m). Yani saldırganlığı
sakinliğe çeviriyor.

⚠ **Kullanıcının kendi kabul ölçütü tam da bunu istiyordu:** "direkt mesafeyi
yaklaştıramasak manevra kısmında biraz mesafe açılsa ama SALINIM OLMASA, çok
açılmasa ve sonra hemen kapatılabilse okeydir." Önceden ilan ettiğim birincil
ölçüt (en yakın menzil) bunu cezalandırıyor — karar kullanıcıya bırakıldı.

⚠ **Sınıflandırıcı özeleştirisi:** kontrol kolundaki tek isabet `vurus_kalitesi`
tarafından ŞANS sayıldı ama altı ölçütten BEŞİNİ geçmişti (hedef medyan 11 px
merkezde, kutu 3.55× düzgün büyümüş, yatış sakin); tek takıldığı 1 kopuk
kareydi. "Sıfır kopuk kare" eşiği fazla katı — o vuruş fiilen kontrollüydü,
Ö8 lehine sayıya çevrilmedi.

### D0 KURAL DÜZELTMESİ — devir ölçütü sadeleşti · İŞE YARADI

Kullanıcı tespiti: devir ölçütünde fazladan şart vardı. Eski hâli:
kayan pencere (son 15'in 10'u) **+ conf ≥ 0.5** (dedektörün kendi eşiği 0.35).
İkinci şart KATIYDI — model "gördüm" derken güdüm GPS'te kalıyordu; D0
ihlali riski buydu. Yeni hâli: **ARDIŞIK 10 tespit, ekstra güven eşiği YOK.**
Geri dönüş: `AVCI_HYBRID_ARDISIK=0`, `AVCI_HYBRID_CONF=0.5`.

⚠ Riski vardı: "10 ardışık" gürültülü tespitte geç sağlanıyor diye 2026-07-31'de
kayan pencereye geçilmişti (devir 6-10 m'de oluyordu). ÖLÇÜLDÜ — risk
gerçekleşmedi, tersi oldu:

    ilk devir menzili: 16.8 / 17.8 / 17.8 / 18.8 m  (tarihsel 6-10 m)

Devir artık iki kat uzakta oluyor: hem kural uyumu daha iyi hem görsel faza
daha çok süre kalıyor.

### M3 — ERKEN LEAD, 15 m tetikte · 4 UÇUŞ · YİNE SİNYAL YOK

| koşu | kol | en yakın | cx salınım/s | yatış p90 |
|---|---|---|---|---|
| M3A yatay | KONTROL | 5.93 m | 0.000 | 28° |
| M3B yatay | **M3** | 5.98 m | 0.000 | 38° |
| M3C capraz | KONTROL | 7.13 m | 0.000 | 30° |
| M3D capraz | **M3** | 7.87 m | 0.000 | 34° |

İsabet 0/4. Birincil 1 (kontrollü vuruş) 0-0; birincil 2 (salınım) her iki
kolda da 0.000. **M3 girmiyor** — üçüncü kez ölçülüp üçüncü kez nötr.

Yan bulgu: 15 m tetikte SALINIM YOK (cx değişimi 0.000). Kullanıcının gördüğü
salınım 7-8 m'lik yakın kırılmaya özgü.

### Ö7 — YAW HIZ TAVANI (120 → 200°/s) · 4 UÇUŞ · SİNYAL YOK

Panelden canlı (`AVCI_IBVS_YAWRATE`). Gerekçe: kaçamak sonrası karelerin
%23-47'sinde yaw komutu 120°/s tavanına yapışıyordu; doymuş sınırlayıcı faz
gecikmesi katar ve salınım üretir (kullanıcı gözlemi).

| koşu | kol | en yakın | cx dğş/s | yatış p90 | yaw doyma |
|---|---|---|---|---|---|
| O7A yatay | KONTROL | 2.19 m | 0.000 | 6° | %8 |
| O7B yatay | **Ö7** | 2.94 m | 0.103 | 20° | %31 |
| O7C capraz | KONTROL | 3.29 m | 0.161 | 26° | %40 |
| O7D capraz | **Ö7** | 3.99 m | 0.086 | 31° | %6 |

**Birincil 1 (kontrollü vuruş): 0-0 — hiçbir koşuda vuruş yok.**
**Birincil 2 (cx salınımı): kontrol 0.081, Ö7 0.095 — kontrol hafif önde.**
Kural gereği Ö7 GİRMEZ.

⚠ **Mekanizma da tutmadı:** tavanı 200'e çıkarınca doyma azalmadı — O7B
%31 doydu (güdüm 200°/s'den de fazlasını istiyor), O7D %6. Kontrol kolunda
%8 ve %40. Aynı kolda 5 kat fark: doyma **kararlı bir mekanizma değil**,
koşudan koşuya değişen bir sonuç.

⚠ **ASIL BULGU — 8 m kırılma şu an ÇÖZÜLMEMİŞ:** bu 4 koşu + Ö6'nın 4
kontrol koşusu = 8 koşuda **isabet yok** (Ö6'nın deney kolundaki 2 isabet
hariç, o da yatışın gerçekten kullanıldığı tek koşuya dayanıyordu). Tetik
25 m'de sistem 4/6 vuruyor; 8 m'de neredeyse hiç. Aradaki fark, üzerinde
çalışılması gereken şey.

### Ö6 — YÜKSEK YATIŞ (ANGLE_MAX 45° → 55°) · 8 UÇUŞ · ELENDİ

Panelden canlı aç/kapa (araç parametresi, MAVLink PARAM_SET; geri okuma
`_param_cache`'ten teyit ediliyor). Kod değişikliği yok.

**GEREKÇE — dönüş yeteneği kıyası** (ω = g·tan(yatış)/V, R = V²/a):

| araç | hız | yatış | yarıçap | dönüş hızı |
|---|---|---|---|---|
| AVCI drone (45°) | 18 m/s | 45° | **33.0 m** | **31°/s** |
| AVCI drone (55°) | 18 m/s | 55° | 25.0 m | 44°/s |
| HEDEF Talon | 15 m/s | 60° | 13.2 m | 65°/s |

Hedef 2.1× hızlı, 2.5× dar dönüyor — drone fiziksel olarak takip edemiyor.
İtki maliyeti 1.74×; araçta 2.56× pay var (`MOT_THST_HOVER=0.39`).

**TEST — tetik 8 m** (kullanıcının gördüğü yakın dövüş; 25 m'lik tetikte
sistem zaten 4/6 vuruyordu, ayırt etmiyordu). `yatay`+`capraz`, dönüşümlü:

| ölçüt | KONTROL n=4 | Ö6 n=4 |
|---|---|---|
| **İSABET** | **0/4** | **2/4** |
| en yakın menzil medyanı | 3.47 m | **1.41 m** |
| ≤2 m'ye gelen koşu | 2/4 | **3/4** |
| görsel temas oranı (medyan) | %67 | **%78** |
| maks açılan mesafe (birincil) | 52.2 m | 52.6 m — berabere |

**Mekanizma doğrulandı:** Ö6 koşusunda yatış **51°**'ye çıktı; kontrol kolu
38°'de kaldı (45° tavanı). İtki payı yetti — irtifa kaybı/çakılma olmadı.

⚠ **"Maks açılan mesafe" ölçütü İKİNCİ KEZ ayırt edemedi** (Ö1'de de öyle
olmuştu). Savrulma, kaçamağın geometrisinden geliyor ve iyileşmeyle ilgisi
zayıf. Kural gereği tek taraflı değiştirmiyorum ama ÖNERİM: birincil ölçüt
bundan sonra **isabet + en yakın menzil** olsun, savrulma ikinciye insin.

**8 m tetik gerçekten zor koşul:** kontrol kolu orada 0/4. 25 m tetikte aynı
sistem 4/6 vuruyordu. Ö6, bu zor koşulda skor yapan İLK özellik.

### Ö1 — KAÇIŞ TELAFİSİ · 8 UÇUŞ · ÖLÇÜT ÇELİŞKİSİ → KULLANICIYA

`v_los = hiz_I + K_FWD·hata + KACIS_KD·max(0, −ṙ)` — yalnız SEYİR fazında,
yalnız hızlandırma yönünde. `AVCI_IBVS_KD=0` varsayılan (kapalı).
Mekanizma doğrulandı: karelerin %34'ünde terim aktif, 10 m/s tavanına dayandı.

4'e 4 dönüşümlü (`yatay`, `capraz`), tetik 25 m:

| ölçüt | KONTROL n=4 | Ö1 n=4 | kazanan |
|---|---|---|---|
| **maks açılan mesafe** (birincil) | **107.7 m** | 116.9 m | kontrol |
| **drone min hız** (birincil) | **12.8 m/s** | 11.2 m/s | kontrol |
| en yakın menzil medyanı | 0.68 m | **0.51 m** | Ö1 |
| isabet | 2/4 | **3/4** | Ö1 |
| ≤1 m'ye gelen koşu | 3/4 | **4/4** | Ö1 |

**Önceden ilan edilen kural Ö1'i ELER** (iki birinciliyi de kaybetti).
**Ama tüm ikincil ölçütler ve VİDEO Ö1 lehine.**

Video (en yakın geçiş karesi): KONTROL koşusunda hedef kadrajda YOK — altta
`ID:7 tahmin(3)`, yani izleyici görmüyor, tahmin ediyor; drone hedefe KÖR
gidiyor. Ö1 koşusunda hedef kadrajın ortasında, kutu kilitli (conf 0.93).

⚠ **İTİRAF: birincil ölçütü yanlış seçmişim.** "Maks açılan mesafe" dönüş
manevrasındaki savrulmayı ölçüyor, kesişimin kalitesini değil. Ö1 daha sert
atak yapıp daha çok savruluyor ama daha iyi kesişim üretiyor. Sonuca bakıp
ölçüt değiştirmek CLAUDE.md §4'e aykırı olduğu için kararı TEK BAŞIMA
DEĞİŞTİRMİYORUM — kullanıcıya götürüyorum.

Yan bulgu (Ö5'i destekler): Ö1 daha yüksek hız KOMUT ediyor ama ULAŞILAN
min hız daha düşük (11.2 < 12.8). Komut arttıkça araç daha çok yatıyor ve
ileri hız düşüyor — yatış-farkında hız bütçesi gerçek bir kısıt.

### M4 — M3 yeniden testi (kullanıcı itirazı üzerine) → HÂLÂ AYIRT EDİLEMİYOR

Kaçamak testiyle 10 uçuş (`yatay` ve `capraz`, dönüşümlü):

| | n | isabet | en yakın medyan | ≤1.5 m |
|---|---|---|---|---|
| M3 KAPALI | 6 | 4/6 | 1.24 m | 4/6 |
| M3 AÇIK | 4 | 2/4 | 0.93 m | 3/4 |

İlk 4 uçuşta "AÇIK 2/2, KAPALI 0/2" çıkmıştı — n arttıkça eridi. M3 zarar
VERMİYOR; en yakın menzil medyanında hafif önde. Varsayılan kapalı kalıyor,
karar kullanıcıda. Değişkenliğin kaynağı M3 değil, yukarıdaki hız çöküşü.

### ⚠ Daire senaryosunda görülen ayrı darboğaz — buluşma GEOMETRİSİ

Nişan yasası değil, karşılaşma geometrisi. Ölçüldü:

    kapanma hızı        medyan 4.9-12.4 m/s, p90 13-22 m/s
    saf kuyruk takibi   en fazla 18 − 14.9 = 3 m/s verirdi
    ⇒ buluşmalar YÜKSEK AÇILI / KAFA KAFAYA (~30 m/s bağıl)

Neden: daire senaryosunda araç dönmek için yatmak zorunda, yatınca ileri hızı
düşüyor (dönüşte 9-14 m/s ölçüldü), hedefin gerisine düşüyor, sonra kirişi
kesip hedefle KARŞIDAN buluşuyor. O geometride isabet zarfının (yatay ±0.65 m)
içinde ~0.05 s kalıyor.

Bütçe tablosu (gereken yanal ivme = V·λ̇, tavan = g·tan45° = 9.81 m/s²):

| menzil | gereken a | tavanı aşan kare |
|---|---|---|
| 20-35 m | 8.8 m/s² | %38 |
| 13-20 m | 14.4 | %71 |
| 8-13 m | 23.1 | %96 |
| 5-8 m | 25.8 | %100 |

**M4 adayları (nişan yasası DEĞİL, geometri/enerji):**
1. **Dönüş-farkında hız tavanı** — gereken a = V·λ̇. λ̇=0.8 rad/s'de V=20 → 16 m/s²
   (bütçe dışı), V=12 → 9.6 m/s² (bütçe içi). Hızı dönüşte kısmak düzeltmeyi
   uygulanabilir kılıyor.
2. **Dairenin İÇİNDEN kesme** — kafa kafaya buluşmayı önlemek için hedefin
   dönüş merkezine yakın yay izlemek.

### M3 teşhis verisi — fiziksel tavan (kalıcı, sonraki işler için)

C koşularında ≤10 m'deki 91 kare:

    cx medyan 453 (merkez 320)   yatay açı medyan 39.6°, p90 63.3°
    LOS oranı medyan 1.47 rad/s (84°/s), p90 4.81 rad/s
    tutmak için gereken: 15 m/s ÷ 8 m = 1.87 rad/s (107°/s)
    lead açısı medyan 0.00°   ← ÇALIŞMIYOR
    durum: 91 karenin yalnız 21'i TERMINAL

İki yapısal sebep, ikisi de kodda:
1. `lead_az` YALNIZ `terminal` iken uygulanıyor → yakın karelerin %77'sinde yok.
2. `lead_olcek = clamp(BOYUT_REF/boyut, 0, 1)` → hedef büyüdükçe (yaklaştıkça)
   lead SÖNÜYOR. Düz takipte doğru (LOS oranı ≈ 0), **dönüşte tam tersi**.

Yani yakın menzilde saf takip hedefin BULUNDUĞU yeri gösteriyor, hedef 40-63°
yanda ve LOS 84-276°/s süpürüyor. Saf takip bu geometriyi kapatamaz.
Öneri: lead'i menzille söndürmek yerine **LOS oranıyla ölçekle** (gerçek PN) ve
TERMINAL kapısından çıkar. ⚠ Kullanıcı onayı alınmadan uygulanmayacak.

---

## DURUM — 2026-08-06 (GPS fazı; aşağıdaki 08-02 bölümü görsel faz dönemine ait)

**Kararlı hal:** `KARARLI_HAL.md` + `gps_kararli_hal` dalı + `kararli-gps-gudumu`
etiketi. Ölçülen: düz 13-14 m, daireler 15-17 m, kare kenar 14 / köşe 21 m.

### C1 — İstasyon ofseti artık TABAN (sıradaki bakılacak yer)

2026-08-06 tespiti: kalan mesafenin çoğu artık takip hatası değil,
**istasyonun kendi tasarım ofseti**:

- Dönüşte istasyon hedefe **17.8 m slant** duruyor (10.63 m arka + 14 m iç
  kayma + 2.85 m alt) — dairelerdeki ölçüm 15-16 m, yani drone tasarlanan
  noktanın üzerinde/az önünde.
- Düz uçuşta istasyon 11 m'de, ölçüm 13-14 m → takip payı yalnız 2-3 m.

**Sonuç:** her senaryoda <10 m hedefi güdümü iyileştirmekten değil, bu
geometriyi küçültmekten geçiyor. En büyük aday: dönüşte 10.63 m'lik **arka
bileşen** — iç kaymayla vektörel toplanıp menzili 17.6 m'ye şişiriyor
(açı: kuyruk hattından 52.8° içeri). Denenecek: dönüşte arka bileşeni
daraltmak (ör. ω ölçeğiyle) — her testte TEK değişken kuralıyla.
⚠ RANGE_SET artık maskeli değil: 13-17 m bandında komut doygun değil,
istasyon yerinin her milimetresi davranışa yansıyor.

*Sonuç (2026-08-08, üç otonom uçuş):*
- **Hamle 1 — dönüş ileri beslemesi (v_ist = v_hedef + ω×r): ELENDİ.**
  Formül doğru (G14b) ama daire 15.1 → 23.0 m'ye AÇILDI (log 131037).
  Mekanizma: "doğru" FF komut hızını düşürüp aracın dönen çerçevedeki
  takip gecikmesini telafisiz bırakıyor; eski v_hedef fazlalığı kazara
  faydalı lead'miş. Varsayılan KAPALI, ders koda gömüldü.
- **Hamle 2 — RANGE_SET 11 → 8: KABUL, varsayılan yapıldı.**
  Daire: 15.1 → **13.3 m** (log 131611). Düz: 13-14 → **10.3 m**
  [p10 9.8, p90 10.8] (log 134512, `duz` senaryosu, bekçi temiz).
- Dönüşte kadraj −9.4 → −15.2'ye geriledi (dikey ofset RANGE_SET'e göre,
  tutuş menzili değişmiyor — r_eff tavanı). Sıradaki hamlelerden biri:
  d_below'u gerçek menzille ölçekle; diğeri: dönüşte arka bileşeni daralt.
- **Hamle 3 — arka kısaltma (dönüşte arka bileşen ω ölçeğiyle erir): KABUL.**
  Daire truth MESAFE 13.3 → **5.7 m** (med; bant 5.3-6.8, min 4.8, temas yok;
  log 141740). Beklenti 12 idi — fazlası geldi çünkü drone'un "istasyonun
  gerisinde sürüklenme" payı, istasyon yana geçince hedefe DOĞRU katlanıyor.
  Kadraj dönüşte −9.6°'ya toparladı (menzil < RANGE_SET → sabit-açı rejimi),
  tespit güveni 0.85'e çıktı (~6 m'de kutu büyük). Kare köşeleri p90 20.7 ≈
  eski seviye, düz uçuş etkilenmedi (ω=0 → kısaltma kapalı).
  ⚠ Yalnız ⌀55'te ölçüldü; kilitlemeden önce ⌀96/71/41 doğrulaması gerek.
- **Çap doğrulaması (2026-08-08, log 144907 + 150726 teyit): GEÇTİ.**
  Truth MESAFE, oturmuş medyanlar: ⌀96 **8.9** · ⌀71 **7.0/7.2** (iki
  bağımsız uçuş) · ⌀55 **5.7** · ⌀41 **9.5/10.4** · düz **10.3**.
  Temas yok (en yakın 4.6 m). C1'in hedefi (<10 m her senaryoda) düz ve
  tüm dairelerde tutturuldu; kare köşe geçişleri (~20 m tepe) ayrı konu.
  ⚠ Bekçi her iki uçuşta da irtifa bandı ihlali bildirdi: daire trimleri
  uçağı ~0.9 m/s tırmandırıyor, uzun koşuda 250 m tavanı aşılıyor. Sayılar
  irtifadan bağımsız çıktı (⌀41: 9.5 @300-388 m vs 10.4 @175-263 m) ama
  KÖK NEDEN backlog'da: senaryo pitch trimi irtifa tutacak şekilde ayarlanmalı.

### E — bbox-IBVS görsel faz İNŞA (2026-08-08, devam ediyor)

- `control/guidance/bbox_ibvs.py` yazıldı: saf görüntü, GPS'siz (D0 uyumlu).
  komut(cx,cy,w,h,iris_yaw): yaw←yatay px, vz←dikey px, ileri←kutu boyutu.
  9 birim test (test_bbox_ibvs). supervisor AVCI_VISUAL=bbox varsayılan.
- Kademeli uçuş testi (Claude koşacak):
  1. DÜZ uçuşta devir — en kolay giriş, kuyruktan yaklaşma. [SIRADA]
  2. DÖNÜŞte devir — kritik: 66° kuyruk girişinden pure-pursuit kuyruğa
     süzülebiliyor mu?
  3. Kayıp → GPS → yeniden devir döngüsü.
- ✅ **ÇALIŞTI (2026-08-08, log 184748 / video ucus_20260808_gorsel_faz.mp4):**
  düz uçuşta TEK devir, **160 s kesintisiz görsel faz**, kutu kaybı %0.4.
  Truth MESAFE med **7.2 m** (p10 5.3, min 4.8, temas yok). Kutu 14 px,
  conf 0.86, cy 300 ≈ nişan 301 (dikey kanal oturmuş).
- Üç düzeltme birlikte çalıştı:
  1. **Dikey nişan** 210 → ≈300 (25° tilt geometrisi; 210 "8 m alta dal"dı).
  2. **DONDURULMUŞ TAŞIYICI** — devir anındaki son GPS hız kestirimi sayı
     olarak görsel faza geçilir, faz boyunca güncellenmez. Kutu boyutu
     MENZİL vekilidir HIZ vekili değil; taşıyıcısız yasa 8 m/s üretip
     15 m/s hedefin gerisinde kalıyordu. Ölçüldü: ff=(10.0,-10.5,-0.3),
     kapanma med +3.8 m/s → toplam ~18 m/s.
  3. **İvme sınırlayıcı drone'un gerçek hızından başlatıldı** (0'dan değil) —
     devirde 1.25 s'lik sahte fren kalktı.
- ⚠ **MENZİL KAPISI KALDIRILDI (kural düzeltmesi):** kapı, görsel temas
  varken GPS güdümünü sürdürerek D0'ı ihlal ediyordu; 20→12 m çekmek ihlali
  BÜYÜTÜYORDU (kullanıcı yakaladı). Artık tek şart tespit sürekliliği.
  Devir 34 m'de gerçekleşti ve görsel faz oradan 5 m'ye kadar taşıdı.
- Sıradaki: (2) dönüşte devir, (3) kayıp→GPS→yeniden devir döngüsü.
  Açık kalibrasyon: BOYUT_REF=25px (≈6 m denge), K_FWD, V_KAPANMA_MAX.

### D — Görsel faza devir: BAĞLAYICI tasarım kararları (2026-08-08)

**D0 — YARIŞMA KURALI (her şeyin üstündeki kısıt, kullanıcı aktarımı):**
Görsel temas sağlandığı anda (detection hedefi tespit edince) GPS verisiyle
güdüm YASAK — yalnız bbox'a dayalı görsel güdüm. Temas kesilirse GPS yeniden
serbest. Temas tanımı tek kare değil, ~10 kare süreklilik gibisinden
(⚠ kesin sayı şartnameden doğrulanacak). SONUÇ: faz geçişi bizim seçimimiz
değil, kuralın sonucu; aşağıdaki 3-4 buna göre REVİZE edildi.

Kullanıcı kararları (yeni görsel faz inşasında uyulacak):

1. **Pose devir denkleminden ÇIKTI.** Yeni görsel güdüm yalnız bbox
   verisiyle IBVS. `supervisor.py`'deki pose-kare sayacı (KILIT_N) yeni
   fazla birlikte bbox-kararlılık sayacına dönüşecek; pose şartı hiçbir
   geçiş koşulunda kullanılmayacak.
2. **Yandan devir YASAK (gimbalsız dönem).** Korkulan mod birebir doğru:
   yandan devirde IBVS "hedef merkezde" deyip İLERİ verir, hedef yana
   kaydığı için kadrajdan çıkar. Sayısal: 6 m'de yan geçiş hızı 14.5 m/s →
   LOS dönüşü 2.4 rad/s = 139°/s — yaw tavanının (120°/s) ve her türlü
   görsel takibin üstünde. Bkz. docs/YANDAN_ESKORT_VE_GIMBAL.md.
3. ~~Devir kapısı geometrik olacak~~ **D0 ile REVİZE:** geçişi geometri
   kapısı değil, KURAL belirler (tespit sürekliliği → görsel; kayıp → GPS).
   Geometri kapısının yerine geçen ilke: **GPS, tespit sürekliliğinin
   başlayacağı anda geometriyi görsel-yaşanabilir tutmalı** — kuyruk-benzeri
   yaklaşma, aşırı yakın-yandan eskort YOK (o `gimball_gudum` arşivinde).
   Eski supervisor iskeleti (KILIT_N ardışık kare → devir, KAYIP_M kayıp →
   GPS'e dön) kurala yapı olarak birebir uyar; pose penceresi yerine
   bbox penceresi sayılır.
4. ~~IBVS'e GPS hız ileri beslemesi (hibrit)~~ **İPTAL — D0 kural ihlali:**
   görsel temas varken GPS verisi güdümde kullanılamaz. Görsel faz SAF bbox
   olacak; hız sürekliliği için yalnız devir ANINDAKİ son komut yumuşak
   başlangıç değeri olarak taşınabilir (o andan sonra GPS verisi akmaz).

### C2 — Dinamik istasyon yükselişi (2026-08-06'da kodlandı, uçuş bekliyor)

Kullanıcı fikri: kamera gövdeye vidalı → gövde duruşu değişince sabit açılı
istasyon hedefi kadrajda sabit tutamaz. Loglarla doğrulandı ve `elev =
kamera_tilt + gövde_pitch(EMA)` olarak uygulandı (`AVCI_GPS_ELEV_DIN=0` ile
eski yol). Ayrıntı: `gps_guidance.py` Cfg.ELEV_DINAMIK bloğu.
*Sonuç (2026-08-08 uçuşu, log 121248 — Claude'un otonom koşusu, kare+CSV):*
- Kadraj dikey sapma: dönüşte **−23° → −9.4°**, düzde **−10° → −3.0°**
  (v_px 310-330 → 268 dönüş, 270 → 248 düz). Karelerde gözle doğrulandı:
  hedef köşe sonrası en uzak anda bile merkezde.
- Menzil DEĞİŞMEDİ (beklendiği gibi): daire 15.0-15.1 m (taban 15-16);
  kare, eski kare uçuşuyla (152954) aynı segmentasyonda düz 21.7 vs 22.5,
  dönüş 16.2 vs 16.4. ("kenar 14" panel okuması en iyi anmış; medyan hep ~22.)
- EMA sağlıklı: ist_elev tick adımı med 0.10°, max 0.93°; dikey salınım yok.
- Dönüşteki −9.4° kalıntının nedeni C1 ile aynı: drone 15 m'de tutunurken
  dikey ofset RANGE_SET=11'e göre hesaplanıyor (r_eff tavanlı). d_below'u
  gerçek menzille ölçekleme C1 kapsamında değerlendirilecek.

---

## DURUM — 2026-08-02 22:30 (yeni oturum buradan devam etsin)

**Depo:** `kubra_masaustu`, temiz, `origin` ile eşit. Son commit `f5737ca`.
Açık PR: **#4** (`gh pr view 4`). Testler: **53/53** ve **12/12**.

**Bitenler:** A1 ✓ · A5 ✓ · dikey ıska 2. tur (istasyon 25° → 15°) ✓ ·
başlatma/durdurma script hataları ✓
**Sıradakiler:** A2, A3, A4, A6, A7 hâlâ **kodda YOK** · B1, B2, B3, B4, B5,
B6 hiç uygulanmadı · A8 (görsel kilit) B1+B2 olmadan uygulanmayacak

**Ölçülen son hal** (A5 + 15° istasyon, 17 geçiş):
en yakın menzil medyanı **1.73 m**, vuruş **3/17**, faz/uçuş **3.4**.
Vuruşu belirleyen tek güçlü değişken: vuran 4 geçişin dördünde de
`kor_dalis` ≤ **%3**, ıskalayanlarda medyan %19-27.

**Bir sonraki iş — üçünden biri:**
- **B7 (açık soru)** — istasyon açısını kamera tilt'inden ayırmak doğru muydu?
  Ölçümler olumlu ama karışık; merkez dışı kadrajlamanın kendi bedeli izole
  edilmedi ve asıl alternatif (`WP_ACC_Z` yükseltmek) hiç denenmedi.
  B6'dan önce karara bağlanmalı — B6 algıyla uğraşacak, algının geometriden
  ne kadar etkilendiği belirsizken çalışmak boşa gider.
- **B6 (terminal algı sürekliliği)** — asıl kaldıraç. Hedefin son 1-2 s'de
  kadrajda kalması. `kpt_dusuk` terminalde %30-60.
- **B5 (fly-past)** — her ıskadan sonra drone 5-7 m yukarı fırlıyor,
  toparlaması 10-20 s. Vuruş oranını değil görev süresini etkiliyor. A5
  sonrası artık HER ıskada yaşanıyor. Ek olarak ölçüldü: faz biterken yaw
  **hız** komutu iptal edilmiyor (bkz. B5 altındaki ⚑ notu).

⚠ **Tekrar denenmeyecekler** (ölçümle çürütüldü, gerekçeleri ilgili yerlerde):
`ATC_ANG_YAW_P` düşürmek · `supervisor.KILIT_N` düşürmek · hedef hızına
ivme kapısı · "araç dikey/yaw komutunu uygulamıyor" teşhisi

---

## Yeni oturuma başlıyorsan

**Sırayla git, atlama.** Her madde: uygula → testler → **uç** → ölç →
*Sonuç:* satırına yaz → tikle. Bir madde bitmeden diğerine geçme. Bu kural
var çünkü hepsi bir arada uygulanınca hangisinin ne yaptığı ayırt edilemedi.

**Ölçüm yöntemi — CSV'ye tek başına güvenme.** Geometri sorularının dürüst
kaynağı iki aracın kara kutusu: her iki `.BIN`'den `POS` (Lat/Lng/Alt) alınıp
`GPS.GWk`+`GPS.GMS` ile ortak saate hizalanır, sonra aradaki yatay/dikey
mesafe hesaplanır. CSV'deki `menzil_gercek_m` EKF çerçeve ofsetinden
etkileniyor ve en yakın anı geriden gösteriyor.

**Sistemi başlatma** (iki terminal, ayrıntı `docs/SIMULASYON_CALISTIRMA.md`):

```bash
# Terminal A
GZ_HEADLESS=1 bash scripts/start_harmonic.sh    # eski surecleri kendi temizler
# durdurmak : bash scripts/start_harmonic.sh stop    (Ctrl+C ISE YARAMAZ)
# kontrol    : bash scripts/start_harmonic.sh durum  (hicbir seyi oldurmez)
# ELLE pkill -9 -f 'gz sim|sim_vehicle|...' KULLANMA — kendi kabugunu oldurur.

# Terminal B
source /opt/ros/humble/setup.bash && export AVCI_GZ_CAMERA=1 AVCI_NO_BROWSER=1
fuser -k 8000/tcp 2>/dev/null; python3 -m control.gcs_server
```

### Sözlük

| terim | anlamı |
|---|---|
| **kara kutu** | ArduPilot'un kendi uçuş kaydı (`~/ardupilot/logs/*.BIN`). "Araç komutu uyguladı mı" sorusunun tek dürüst kaynağı. |
| **istasyon** | GPS fazının "şurada dur" dediği hayali nokta. Sabit metre DEĞİL, sabit AÇI: hedeften `RANGE_SET`(11 m) uzakta, LOS yükselişi `ISTASYON_ELEV_DEG`. 2026-08-02'de bu açı kamera tilt'inden (25°) ayrılıp **15°**'ye indirildi → 10.63 m geride + **2.85 m** altta (eskiden 9.97 m + 4.65 m). Sebep: terminalin kapatacağı dikey mesafe aracın 1 m/s²'lik dikey ivme bütçesine sığmıyordu. Drone hedefi değil bu noktayı takip eder. B5'teki "istasyona dön" = ıskaladıktan sonra kontrollü şekilde bu bekleme noktasına dönüp yeni hücuma hazırlanmak. |
| **`ok` oranı** | `visual_lead` her kareye `durum` yazar. `ok` = pose temiz, keypoint'ler güvenilir. Diğerleri `kpt_dusuk` / `tespit_yok` / `kor_dalis`. "%51 ok" = karelerin yarısında güdüm sağlam veriyle çalışıyor. |
| **faz** | GPS fazı (yaklaşma) ↔ görsel faz (terminal hücum). Geçişi `supervisor` yönetir. |
| **fly-past** | Drone hedefe temas etmeden yanından geçmesi. Sonrasında "hedefe uç" komutu yukarı-geriyi gösterir → kontrolsüz tırmanma. Bkz. B5. |

### Bu oturumda öğrenilen — tekrarlamayın

- **"Araç komutu uygulamıyor" demeden önce kara kutuya bakın.** Bu teşhis bir
  kez kondu ve çürütüldü: alçalma emredilen anlarda `PSCD.DVD +6.36` iken
  `VD +6.43`, takip hatası 0.1 m/s. Araç kusursuz uyguluyordu.
- **Tek seferde tek değişken.** Üç kez birden fazla şey değiştirildi ve
  hangisinin ne yaptığı ayırt edilemedi.
- **Ölçmeden değer değiştirmeyin.** `ATC_ANG_YAW_P 4.5 → 3.0` iyi niyetle
  yapıldı, yaw takip hatasını 1.36° → 4.94°'ye çıkardı.
- **Bozuk veriyle ölçüm yapmayın.** Hedefin hızı bir süre `tgt_vx` sütunundan
  17.5 m/s sanıldı; o sütun zaten bozuk olduğu kanıtlanan kestirimdi. Gerçek
  değer ArduPlane kara kutusundan **14.0 m/s** çıktı.

---

## Önerilen sıra

```
A1 → A2 → A3 → A4 → A6    kanıtlı / düşük risk (birlikte uçulabilir)
A7                         belge, uçuş gerektirmez
B1                         güvenlik (irtifa tabanı) — kendi uçuşu
A5 + B5                    gerçek temas + fly-past davranışı — BİRLİKTE
A8 + B2                    görsel kilit + dikey sönümleme; B1 olmadan ASLA
B6                         terminal algı kalitesi (asıl darboğaz, uzun iş)
B3 / B4                    yalnız gerekirse
```

**A5 ve B5 neden birlikte:** A5 erken durmayı kaldırıyor, B5 ondan sonra ne
olacağını tanımlıyor. A5 tek başına uygulanırsa drone hedefi geçtikten sonra
kontrolsüz tırmanır — kilit olmasa bile.

**B1 neden A5'ten önce:** görsel fazda yere çakılma koruması yok. Fly-past
denemeleri sırasında drone alçalabilir; taban olmadan zemine girer.

---

## KARAR: (b) — gerçek temas ölçütü

2026-08-02, push haliyle 18 görsel faz ölçüldü. İki seçenek vardı:

**(a)** 1.5 m yakınlığı vuruş saymak — ekranda güzel görünür, görev "başarılı"
biter, ama gerçekte ıskaladığımızı bilmeyiz.
**(b)** Gerçek fiziksel temas — dürüst, ama şu an çoğu denemede vuramıyoruz.

**(b) seçildi.** Gerekçe: push halinin "başarısı" kısmen erken durmadan
geliyor. Drone 1.5 m'ye gelince "VURULDU" deyip güdüm DURUYOR; hedefin
yanından geçtikten sonra ne olacağıyla hiç yüzleşmiyor. Gerçek temas ölçütü
bu sorunu **yaratmadı, görünür kıldı**.

### Ölçülen gerçek: terminal isabet tespit kalitesine bağlı

18 görsel fazın en yakın yaklaşmaları (push hali, hiç değişiklik yok):

| en yakın menzil | `ok` oranı | vuruldu |
|---:|---:|:---:|
| **0.99 m** | %51 | ✓ |
| **1.12 m** | %68 | ✓ |
| 2.42 – 3.13 m | %6 – 22 | — |
| 4.23 – 6.80 m | %5 – 50 | — |
| 10.24 – 12.66 m | %0 – 27 | — |

`ok` oranı %50'nin üstündeyken 1 m altına iniliyor; %30'un altındayken 2-12 m
ıskalanıyor. ~~**Asıl darboğaz terminal algı kalitesi.**~~

### ⚑ DÜZELTME (2026-08-02, A5 sonrası 3 uçuş): darboğaz algı DEĞİL, DİKEY BÜTÇE

Yukarıdaki "asıl darboğaz algı" çıkarımı **korelasyonu nedenle karıştırmış**.
A5'ten sonra üç uçuş kara kutuyla (iki aracın `POS` mesajları, GPS haftası
saatiyle hizalanmış) ölçüldü. Üçü de terminale **aynı** geometriyle giriyor:
yatay ~12.5 m, dikey **+4.65 m** (istasyon ofseti), hedef düz uçuyor.

**Kök neden: ArduPilot dikey hız komutunu 1.0 m/s² ile rampalıyor.**
Ölçüldü — `PSCD.DVD`'nin pozitif eğim medyanı üç uçuşta da tam **1.00 m/s²**
(`WP_ACC_Z = 1.0`). Güdüm 8-22 m/s tırmanma istiyor, tavan (`WP_SPD_UP = 5.0`)
hiç görülmüyor (en yüksek DVD 2.13-2.76) — yani **hız değil, İVME sınırlıyor.**
Araç kusursuz uyguluyor: DVD↔VD takip hatası 0.1 m/s, gaz hiç %95'i aşmıyor,
RCOU doygun değil. Komutun büyüklüğü tamamen alakasız.

Sıfırdan 4.65 m kapatmak 1 m/s²'de **3.05 s** sürer. Elde olan süre:

| | **A (vurdu)** | B (ıska) | C (ıska) |
|---|---:|---:|---:|
| görsel faza giriş menzili (3B) | **10.32 m** | 7.65 m | 9.16 m |
| faz başından en yakın ana | **2.64 s** | 2.38 s | 2.77 s |
| yatay 9 m'de DVD (tırmanma) | **+0.46** | +0.40 → 0.37 düştü | +0.19 → 0.09 düştü |
| kapatılan dikey | **4.25 m** | 2.70 m | 2.42 m |
| gereken dikey | 4.28 m | 4.22 m | 4.48 m |
| **en yakın anda kalan dikey** | **+0.03 m** | **+1.52 m** | **+2.06 m** |
| sonuç | **GERÇEK TEMAS** | alttan geçti | alttan geçti |

Üçünün de süresi 3.05 s'nin altında. A yalnızca **rampayı erken başlattığı**
için yetişti: görsel faza 10.3 m'de girdi ve 9 m'ye geldiğinde zaten
0.46 m/s tırmanıyordu (≈1.2 m'lik avans). B 7.65 m'de, C 9.16 m'de girdi ve
rampaları başta duraksadı.

**Algı çöküşü SONUÇ, sebep değil.** Drone altta kaldıkça hedef kadrajın
üstünden çıkıyor — 4-2 m bandında ölçülen:

| 4-2 m bandı | A | B | C |
|---|---:|---:|---:|
| `gercek_kadraj_ici` | **%69** | %21 | %15 |
| `gercek_v_px` (görüntü yüksekliği 480) | **336** | 20 | −1 |
| son 1.5 s'de `kor_dalis` kare | **1/46** | 30/46 | 29/46 |
| `pn_dikey_deg` | −1.7° | +21.9° | **+30.0° (tavanda)** |

Kısır döngü: dikey geride kalır → hedef kadrajın tepesinden çıkar → tespit
ölür → `kor_dalis` komutu dondurur → düzeltme büsbütün biter. Ama halkanın
başı dikey bütçe.

**Bunun üç maddeye etkisi:**
- **B6** yeniden çerçevelenmeli: algıyı düzeltmek dikey bütçeyi düzeltmez.
  Önce dikey, sonra algı — sıra bu.
- ⚠ **DENENDİ VE GERİ ALINDI: `supervisor.KILIT_N` 10 → 7.** Devir menzili ile
  vuruş arasında güçlü bir bağıntı vardı (vuranlar 11.11 m'de, ıskalayanlar
  9.05 m'de devraldı), kapıyı gevşetip devri uzaklaştırmak denendi. Her
  ölçütte kötüleşti: faz/uçuş 3.4 → 8.0, giriş menzili medyanı 10.00 → 9.62 m
  (**düştü**), en yakın menzil medyanı 1.73 → 2.08 m, `kor_dalis` medyanı
  %19 → %27, 1.5 s'den kısa kopan faz 2/17 → 4/8, vuruş 3/17 → 1/8.
  Mekanizma: kapı cılız tespitte de açılıyor, erken devir gerçekten oluyor
  (14.73 ve 10.47 m) ama 0.9-1.3 s'de ölüyor, GPS'e dönülüyor, drone bu arada
  yaklaşıyor, sonraki devir DAHA YAKINDA oluyor.
  **Ders:** devir menzili ↔ vuruş bağıntısı nedensel değil; ikisi de "tespit
  o an gerçekten sağlam mı"ya bağlı. Kapı sağlamlık üretmiyor.
- Yeni aday: `WP_ACC_Z` (1.0 m/s²) terminalde geçici olarak yükseltilebilir
  mi, ya da istasyonun 4.65 m'lik dikey ofseti küçültülebilir mi? İkisi de
  **ölçülmeden değiştirilmeyecek** — bkz. `ATC_ANG_YAW_P` dersi.
- Devir menzili (`supervisor.GATE_MENZIL`/kilit koşulu) doğrudan dikey süreyi
  belirliyor: A 10.3 m'de devraldı ve vurdu, B 7.65 m'de devraldı ve 1.5 m
  alttan geçti. Erken devir = daha çok tırmanma süresi.

Not: 0.61 m'de bile Gazebo temas sensörü tetiklenmedi (ölçüldü). Yani gerçek
temas için ~0.3 m daha kapatmak gerekiyor.

---

## A) Push sonrası yapılanlar — geri alındı, tekrar uygulanacak

- [ ] **A1 — `ATC_ANG_YAW_P 3.0` satırını kaldır**
      `sim/ardupilot_params/avci_copter.parm`
      *Neden:* push'ta bu satır vardı ve zararlıydı. Ölçüm — 4.5'te yaw takip
      hatası std **1.36°**, seyirde dönme ~0 °/s; 3.0'da std **4.94°** ve
      **11.96 tur** dönme. Kazancı düşürmek aracın komut edilen başlığı
      yakalamasını yavaşlatıyor; düzeltmeye çalıştığımız şeyi 4 katına çıkardı.
      Satırı silmek varsayılan 4.5'e döndürür.
      *Ölçüt:* kara kutuda `ATT.Yaw − ATT.DesYaw` std'si ~1.4°; toplam yaw
      dönüşü 1 turun altı.
      *Sonuç:* **UYGULANDI — 1. ölçüt geçti, 2. ölçüt yanlışmış.**
      Kara kutuda `ATC_ANG_YAW_P = 4.5` doğrulandı (log 105/107/108).
      Aynı oturumda temiz A/B (4 kopter kaydı) — sabit-başlık (DesYaw ±5°,
      ≥8 s) dilimlerinde takip hatası std:

      | log | P | dilim | süre | std | \|max\| |
      |---|---|---:|---:|---:|---:|
      | 00000100 | 3.0 | 3 | 45 s | 11.88° | 47.5° |
      | 00000103 | 3.0 | 2 | 54 s | 8.53° | 45.0° |
      | 00000105 | 4.5 | 3 | 46 s | **1.32°** | 3.8° |
      | 00000107 | 4.5 | 5 | 133 s | 4.64° | 44.6° |
      | 00000108 | 4.5 | 1 | 22 s | **1.43°** | 2.5° |

      Bozulmanın **karakteri** de değişti: 3.0'da kalıcı sapma (log 100,
      36-45 s: ortalama hata −15.3°, karelerin %32'si >20° — araç başlığı
      yakalayamıyor), 4.5'te yalnız anlık sıçrama (log 107, 136-222 s:
      ortalama +0.60°, %2'si >20°). Motor doygunluğu yok (RCOU max 1866).

      ⚠ **2. ölçüt (toplam dönüş < 1 tur) kullanılamaz — P ile ilgisi yok.**
      Net dönme: 3.0 → 0.102 ve 0.150 tur/s; 4.5 → 0.176, 0.045, 0.238 tur/s.
      Korelasyon yok. Belgedeki "3.0 → 11.96 tur" bir P-kazancı etkisi değilmiş:
      dönme sabit-başlık dilimlerinin DIŞINDA, `DesYaw`'ın kendisi dönerken
      oluyor → güdümün komut ettiği dönme. Bkz. aşağıdaki B5 notu.

- [ ] **A2 — MAVLink kuyruk boşaltma**
      `control/gcs_server.py` → `mavlink_listener`
      *Neden:* döngü her 5 ms'de **tek** mesaj okuyordu → tavan 200 msg/s.
      İki araç × 4 mesaj tipi × 25 Hz ≈ 200/s, tam sınırda. Kuyruk birikip
      mesajlar TOPLU teslim ediliyordu (varış aralığı medyan 0.050 s ama
      **max 0.30 s**). Tur başına kuyruk boşaltılacak (üst sınır 400).
      *Ölçüt:* `/api/debug/hedef_telem` → `varis_araligi_s.duzensizlik_orani`
      **5.9 → 1.2**.
      *Sonuç:*

- [ ] **A3 — Hedef hızı aracın KENDİ saatinden**
      `control/gcs_server.py` + `control/guidance/gps_guidance.py`
      *Neden:* hız `Δkonum / Δvarış` ile hesaplanıyordu. Mesajlar toplu
      gelince 0.25 s'de biriken hareket 0.05 s'lik aralığa bölünüp **~100 m/s
      sahte hız** üretiyordu (ölçülen max 106.8; Talon'un gerçek hızı
      **medyan 14.0 m/s**, ArduPlane kara kutusu GPS.Spd ile doğrulandı).
      Bu sahte hız güdüme FEEDFORWARD olarak giriyor
      (`vx = vel_x + KP_H·hata`) → araç şiddetle pitch'liyor.
      *Nasıl:* `LOCAL_POSITION_NED.time_boot_ms` →
      `telemetry_state["plane"]["t_boot_ms"]` → `_noisy_plane_telem` →
      `gps_guidance` hız paydası. Damga yoksa duvar saatine düşen yedek yol.
      *Ölçüt:* `ham_konum_hizi.max` **106.8 → ~17 m/s**, `imkansiz_40ustu` 0,
      medyan ~14 (gerçek hızla uyuşmalı).
      *Sonuç:*

- [ ] **A4 — Hedef sıçrama kapısı (emniyet ağı)**
      `control/guidance/gps_guidance.py` → `_HedefKapisi` + testler G11-G13
      *Neden:* A2+A3 kök nedeni çözüyor; bu, ölçüm zincirinde başka bir yerde
      bozulma olursa güdüm korumasız kalmasın diye. Desen
      `visual_lead._MenzilKapisi` ile aynı (kanıtlı, T38/T38b ile testli):
      imkânsız sıçrama reddedilir, son geçerli değer korunur, ısrarlı redde
      yeniden senkronize olunur.
      `HEDEF_HIZ_TAVAN = 35 m/s` (gerçek tepe 21.8'in belirgin üstü),
      `HEDEF_RESENK_N = 8`, CSV'ye `hedef_red` sütunu.
      ⚠ Ayrıca bir **ivme kapısı** denendi ve KALDIRILDI — hız kestiriminin
      sıfırdan oturmasını da engelliyordu (G9 yakaladı). Tekrar eklemeyin.
      *Ölçüt:* `hedef_red` ~0 kalmalı. 0 değilse kök neden geri gelmiş demektir.
      *Test:* 11/11 → 14/14
      *Sonuç:*

- [ ] **A5 — Gerçek çarpışma tespiti**
      `sim/gazebo_harmonic/worlds/avci_harmonic.sdf` (contact-system eklentisi)
      `sim/gazebo_harmonic/models/mini_talon_vtail/model.sdf` (`carpisma_sensoru`)
      `control/carpisma_state.py` (YENİ) · `control/gcs_server.py` (hasar modülü)
      `control/guidance/visual_lead.py` (`_vurus_oldu`) · testler T46-T48
      *Neden:* eski hâli **1.5 m yakınlığı** vuruş sayıyordu. Ölçüldü — bir
      koşuda 0.61 / 0.69 / 0.75 / 1.06 / 1.16 / 1.20 m yaklaşma vardı, yani
      **6 sahte vuruş** raporlanırdı. Yakınlık çarpışma değildir. Dahası sahte
      vuruş güdümü DURDURUYOR; drone tam hızla giderken komutsuz kalıp
      savruluyordu.
      *Ayrıntı:* sensör gövde+kanat+kuyrukta; **tekerlek DAHİL DEĞİL** (pistte
      sürekli yere değiyor, her kalkışta sahte çarpışma üretirdi). Karşı taraf
      iris değilse imha sayılmaz (zemine çarpma elenir). `VURUS_MENZIL` (1.5 m)
      artık yalnız temas kaynağı yoksa devreye giren **yedek** ölçüt.
      *Ölçüt:* ıskalayıp yanından geçince "VURULDU" **yazmamalı** ve hedef
      düşmemeli; gerçekten çarpınca `VURULDU — GERÇEK TEMAS` yazmalı ve hedef
      düşmeli. Açılışta `[HASAR] GERÇEK çarpışma dinleniyor` satırı görünmeli.
      *Test:* +3 (T46-T48)
      *Sonuç:* **UYGULANDI (kullanıcı isteğiyle sıradan önce çekildi).**
      Testler 50/50 → **53/53**, GPS 11/11 bozulmadı.

      Yerinde doğrulandı (uçuş değil, sistem ayaktayken):
      - `gz topic -l` → `/world/avci/model/mini_talon/link/base_link/sensor/`
        `carpisma_sensoru/contact` **var** (varsayılan `_HASAR_TOPIC` ile birebir).
      - Topic dinlendi: uçak pistte dururken
        `mini_talon::base_link::fuselage_collision ↔ grass_field::link::collision`
        akıyor. Karşı tarafta `iris` geçmediği için süzgeç doğru şekilde
        **imha saymıyor**.
      - `gcs_server` açılışı: `[HASAR] GERÇEK çarpışma dinleniyor: ...` +
        `vuruş ölçütü = fiziksel temas`.
      - `GET /api/debug/carpisma` → `{"temas":false, ..., "kaynak_hazir":true}`.

      Ek olarak (belgede yoktu, gerekliydi): `start_chase` ve `start_visual`
      artık temas mandalını sıfırlıyor. Mandal latch'li olduğu için önceki
      denemede gelen temas, yeni görsel fazın İLK karesinde "vuruldu"
      dedirtiyordu.
      ⚠ **Uçuşta beklenen:** 18 fazın yalnız 2'si 1.5 m altına iniyordu ve
      0.61 m'de bile temas sensörü tetiklenmemişti. Yani bu maddeden sonra
      "VURULDU" oranının **düşmesi** normal — hata değil, dürüstlük. Asıl iş
      B6 (terminal algı) ve B5 (geçiş sonrası davranış).

- [ ] **A6 — Tanılama endpoint'i + `AVCI_IRIS_14550` bayrağı**
      `control/gcs_server.py` → `/api/debug/hedef_telem`
      *Neden:* A2/A3'ün kök nedenini bu ayırt etti (varış düzensizliği mi,
      ham veri mi, `_frame_off` mu, GPS gürültü slider'ı mı). Salt gözlem,
      güdüme dokunmaz. İleride tekrar lazım olacak.
      *Sonuç:*

- [ ] **A7 — TODO.md güncellemeleri**
      Özellikle **"⚠ YANLIŞ TEŞHİS"** bölümü: *"araç dikey komutu
      uygulamıyor"* iddiası kara kutu `PSCD` ile çürütüldü — alçalma emredilen
      anlarda DVD +6.36 iken VD +6.43, takip hatası ~0.1 m/s, ve "aşağı
      emredildi ama yukarı gidiyor" örneği **0/3648**. Hata: 5 saniyelik bir
      CSV penceresine bakılmış, araç o sırada mevcut bir tırmanışı tersine
      çeviriyormuş, aracın kendi kaydına bakılmamış.
      **Kural:** "araç komutu uygulamıyor" demeden önce MUTLAKA kara kutuda
      `PSCD` (dikey) veya `ATT.DesYaw` (yaw) ile istenen–gerçekleşen
      karşılaştırılacak.
      *Uçuş gerektirmez.*
      *Sonuç:*

- [ ] **A8 — Görsel kilit** ⚠ **B1 ve B2 OLMADAN UYGULAMAYIN**
      `control/guidance/supervisor.py` + `control/guidance/visual_lead.py`
      + testler T49-T50
      *Ne yapar:* kısa tespit kopmalarında GPS'e dönülmez, son nişan komutu
      sürdürülür (`kilit_kor` durumu); 10 s hiç tespit gelmezse pes edilir.
      *Kanıtlanan FAYDA:* faz girişi **23 → 9**, ortalama süre **3.5 s → 8.9 s**
      (en uzun 27.8 s).
      *Ölçülen ZARAR:* karelerin **%64'ü `kilit_kor`** (kör uçuş) ve dondurulan
      komut TIRMANIŞ:

      | durum | kare | medyan dikey komut |
      |---|---:|---:|
      | `ok` (tespit var) | 509 | **+10.14** = aşağı |
      | `kilit_kor` (tespit yok) | 1539 | **−12.43** = yukarı |

      Tespit tam da drone hedefe doğru tırmanırken kopuyor (hedef kadrajdan
      çıkıyor), yani kilit **kaybın sebebi olan komutu** 10 s sürdürüyor.
      Sonuç: drone hedefin üstüne çıkıyor, toparlayamıyor, zemine çakılıyor.
      *Ölçüt:* faz girişi < 5; `kilit_kor` karelerinde dikey komut medyanı
      0'a yakın (B2 ile); zemine çarpma 0 (B1 ile).
      *Test:* +2 (T49-T50)
      *Sonuç:*

---

## B) Öneriler — henüz hiç uygulanmadı

- [ ] **B1 — Görsel faza irtifa tabanı** · öncelik **YÜKSEK**, A8'den ÖNCE
      `control/guidance/visual_lead.py` (veya `adapter_copter`)
      *Neden:* GPS fazında `LOOKUP_MIN_ALT = 8 m` yere çakılma koruması var
      (`gps_guidance.py:50`); **görsel fazda hiç yok**. Son üç uçuşun üçünde de
      takla = zemine çarpma; kara kutu: irtifa **8.0 → 0.2 m**, 4 m/s alçalışla,
      ardından `|roll| > 90°`. Kilit olsun olmasın bu bağımsız bir eksiklik.
      *Nasıl:* dikey komut, drone tabana yaklaştıkça **yumuşak** kırpılacak.
      Sert kesme terminal dalışı bozar; taban yaklaşımında oransal sönümleme.
      *Ölçüt:* zemine çarpma **3/3 → 0**; en düşük irtifa tabanın altına
      inmemeli.
      *Sonuç:*

- [ ] **B2 — `kilit_kor` sırasında dikey komutu sönümle** · A8 ile birlikte
      `control/guidance/visual_lead.py`
      *Neden:* kör uçarken kaybın SEBEBİ olan tırmanışı sürdürmek yanlış.
      *Nasıl:* dondurulan komutun **dikey** bileşeni zamanla sıfıra çekilir;
      yatay ve yaw korunur (nişan yönü bilgisi hâlâ değerli).
      *Ölçüt:* `kilit_kor` karelerinde dikey komut medyanı **−12.4 → 0'a yakın**.
      *Sonuç:*

- [ ] **B3 — Kilit süresini kısalt** · B2'ye alternatif, daha kaba
      `supervisor.SupCfg.GORSEL_KILIT_SURE` 10 s → 1-2 s.
      Mevcut kör dalış (`_terminal_adim` / `TERMINAL_SURE`) zaten kısa
      tutulmuş; faz seviyesindeki kilidi 10 s yapmak o dersi görmezden
      gelmekti. B2 çalışırsa gerekmez.
      *Sonuç:*

- [ ] **B5 — FLY-PAST DAVRANIŞI** · öncelik **YÜKSEK**, A5 ile birlikte
      `control/guidance/visual_lead.py` (+ muhtemelen `supervisor.py`)
      *Neden:* push halinde drone 1.5 m'ye gelince "VURULDU" deyip **duruyor**;
      ıskalayıp yanından geçtikten sonrası hiç yaşanmıyor. A5 (gerçek temas)
      o erken durmayı kaldırınca ortaya çıkan davranış şu: drone hedefi geçer,
      "hedefe uç" komutu artık **yukarı-geriyi** gösterir, drone tırmanır,
      hedef kadrajdan çıkar, tespit kopar. Kilit varsa kör tırmanışa dönüşür;
      kilit yoksa bile kontrolsüz bir yukarı hamle olur.
      **A5 tek başına uygulanırsa bu sorun kilit olmasa da gelir.**
      *Ne lazım:* "geçtim" durumunun tespiti ve ondan sonrası için ayrı bir
      davranış. Kaba taslak — uygulamadan önce ölçülecek:
      - Tespit: menzil çok küçüldü (< ~3 m) VE artık **büyüyor**, ya da hedef
        gövde çerçevesinde arkaya düştü (`u_govde[0] < 0`).
      - Davranış: terminal hamleyi bırak, tırmanmayı kes, kontrollü şekilde
        istasyon geometrisine dön. Kör tırmanışı **sürdürme**.
      - Gerekirse "yeniden hücum" sayacı: kaç kez denendi, ne zaman vazgeç.
      *Ölçüt:* geçiş sonrası irtifa aşımı ve zemine çarpma 0; drone kontrollü
      şekilde yeni bir yaklaşmaya geçebilmeli.

      ⚑ **2026-08-02'de ÖLÇÜLDÜ — "vuruldu"dan sonrası çok daha kötü.**
      Log `00000108` (temiz koşu, 1.07 m'de `vuruldu`). Vuruş anından itibaren
      kara kutu:

      | t (uçuş) | yaw | irtifa |
      |---:|---|---:|
      | 39.5 s | sabit 350° | 21.5 m |
      | 40–55 s | **kesintisiz dönüş, net 14.57 tur, ~350 °/s** | 21.8 → 2.0 m |
      | 55–70 s | dönüş sönüyor | 2.0 m sabit |

      `DesYaw` de aynı rampayı ~45° gerisinden izliyor — yani araç kusursuz
      uyguluyor, **komut kesilmemiş**. `ATT.DesYaw`'ın sürekli rampa çizmesi
      bir yaw **HIZ** komutunun iptal edilmemiş olduğunu gösteriyor: güdüm
      "VURULDU" deyip CSV'yi kapatıyor (o andan sonra ne gps_guidance ne
      visual_lead kaydı var), ama son yaw-hızı komutu araçta yaşamaya devam
      ediyor. Araç 15 s boyunca ~350 °/s dönerek 21.8 m'den 2.0 m'ye düşüyor.
      Mod hep GUIDED (4), mod değişimi yok.
      **Bu A1'in yan etkisi değil:** dönme 3.0'lı koşularda da vardı
      (log 100/103: 0.10-0.15 tur/s).
      → B5 çözümüne "faz biterken yaw-hızı komutunu SIFIRLA" maddesi eklenmeli;
      A5 (gerçek temas) bu davranışı daha da uzun süre görünür kılacak.
      *Sonuç:*

- [ ] **B6 — Terminal algı kalitesi** · asıl darboğaz
      *Neden:* ölçüm net — `ok` oranı %50 üstündeyken en yakın menzil 0.99-1.12 m
      (vuruş), %30 altındayken 2.4-12.7 m (ıska). 18 fazın yalnız 2'si 1.5 m
      altına indi. Gerçek temas için ~0.3 m daha kapatmak gerekiyor ve bunu
      sağlayacak şey daha iyi/kararlı pose.
      *Nereye bakılacak:* `kpt_dusuk` oranı terminalde %27-46 — keypoint güveni
      düşüyor. Yeni eklenen cevap anahtarı sütunları (`pose_elev_sapma_deg`,
      `pose_yaw_sapma_deg`, `gercek_kadraj_ici`) algı hatasını doğrudan ölçüyor;
      bunlar A5 sonrası loglarda incelenip hatanın açı mı, ölçek mi, yoksa
      kadraj mı olduğu ayrıştırılmalı.
      *Ölçüt:* terminal `ok` oranı %50 üstüne çıkmalı; en yakın menzil
      dağılımının medyanı 1 m altına inmeli.
      *Sonuç:*

- [ ] **B7 — AÇIK SORU: istasyon açısını kamera tilt'inden ayırmak doğru muydu?**
      · öncelik **ORTA**, B6'dan önce karara bağlanmalı
      `control/guidance/gps_guidance.py` → `ISTASYON_ELEV_DEG`

      *Şüphe:* 25° tesadüf değildi — kamera tilt'i o. İstasyon 25°'de
      kurulunca hedef kadrajın TAM MERKEZİNDE oluyordu. 15°'ye indirince
      hedef merkezin ~10° altına düştü. Pose modelinin merkez dışı
      performansı, lens distorsiyonu, ve hedefin gökyüzü yerine zemin
      önünde görünmeye başlaması (daha yatık bakış) bedel olabilir.
      Değişiklik ölçüme dayanıyordu ama **bedeli izole ölçülmedi**.

      *Elde olan (2026-08-02, 10 faz @25° vs 17 faz @15°):*

      | | 25° | 15° |
      |---|---:|---:|
      | `ok` oranı (tüm faz) | %24.0 | **%32.0** |
      | `ok` oranı (menzil < 8 m) | %8.7 | **%18.2** |
      | hedef kadraj içi | %59.8 | **%67.0** |
      | pose kalite medyanı | 1.00 | 1.00 |
      | `pose_elev_sapma` medyanı | 3.10° | 3.51° |
      | en yakın menzil medyanı | 5.25 m | **1.73 m** |
      | vuruş | 1/10 | 3/17 |

      ⚠ **Bu tablo şüpheyi ÇÜRÜTMÜYOR, çünkü karışık.** Algının iyileşmesi
      büyük ölçüde geometrinin SONUCU: drone hedefin seviyesine yakın
      kalınca hedef kadrajdan geç çıkıyor, tespit doğal olarak uzuyor.
      Yani ölçülen şey "merkez dışı kadrajlama zararsız" değil, "net etki
      olumlu". Merkez dışı olmanın kendi bedeli **hâlâ bilinmiyor**.

      *Bilinmeyenler:*
      1. 15° taranarak seçilmedi — dikey ivme bütçesi hesabından çıktı.
         18° ve 20° hiç denenmedi. Bütçeye sığan **en büyük** açı hangisi?
      2. **Asıl alternatif hiç denenmedi:** istasyonu 25°'de bırakıp
         `WP_ACC_Z`'yi (1.0 → 2.5-3.0) yükseltmek. İşe yararsa hem merkez
         kadrajlama hem dikey kapanma birlikte elde edilir ve bu ayrım
         gereksiz hale gelir.

      *Nasıl karara bağlanır — sırayla, tek değişken:*
      - **Adım 1:** `ISTASYON_ELEV_DEG=25` geri + `avci_copter.parm`'a
        `WP_ACC_Z 2.5`. Tek uçuş. ⚠ `WP_ACC_Z` global bir kopter
        parametresi — kalkışı ve istasyon tutmayı da etkiler; irtifa
        aşımı/salınım için `PSCD.DVD` vs `VD` bakılacak.
      - **Adım 2:** Adım 1 tutmazsa açıyı tara: 20°, 18°. Bütçeye sığan
        en büyük açı seçilir (test G11 sınırı zaten kontrol ediyor).

      *Ölçüt — 15°'nin şu anki haline göre bozulmamalı:* en yakın menzil
      medyanı ≤ 1.73 m · terminal `ok` oranı ≥ %18 · kadraj içi ≥ %67 ·
      dikey artık medyanı |·| ≤ 0.9 m. Bunları tutturan **en yüksek**
      istasyon açısı kazanır (merkez kadrajlamaya en yakın olan).
      *Ölçüm aracı:* `python3 tools/gecis_analiz.py`
      *Sonuç:*

- [ ] **B4 — `coalt` kapsamını daralt** · düşük öncelik
      `guidance_core.TERMINAL_COALT_DEG = 10°` yukarı yanlılık **1064 karede**
      aktifti; tırmanışı büyüten etkenlerden biri. `coalt_latch` menzil eşiğine
      bir kez girince kilitleniyor. B1+B2 sonrası hâlâ sorun varsa bakılır.
      *Sonuç:*

---

## Ölçüm komutları

```bash
# Uçuş SONRASI — geçişlerin gerçek geometrisi (iki aracın kara kutusundan).
# CSV'deki menzil EKF ofsetinden etkileniyor; bu araç dürüst kaynağa bakar.
python3 tools/gecis_analiz.py            # en son uçuş
python3 tools/gecis_analiz.py 126 127    # belirli BIN'ler
python3 tools/gecis_analiz.py --liste    # son 10 uçuş

# Uçuş sırasında — gerçek temas kaynağı sağlam mı (A5)
curl -s localhost:8000/api/debug/carpisma | python3 -m json.tool

# Uçuş sırasında — hedef telemetrisi sağlığı (A6 uygulanınca)
curl -s localhost:8000/api/debug/hedef_telem | python3 -m json.tool

# Uçuş sırasında — faz ve kapılar
curl -s localhost:8000/api/telemetry/pnp | python3 -m json.tool

# Uçuş sonrası — parametreler gerçekten uygulandı mı
python3 tools/parm_denetle.py
```

Kara kutu ölçümleri (`~/ardupilot/logs/*.BIN`): yaw için `ATT.Yaw` vs
`ATT.DesYaw`; dikey için `PSCD.DVD` (istenen) vs `PSCD.VD` (gerçekleşen);
motor doygunluğu için `RCOU`.
