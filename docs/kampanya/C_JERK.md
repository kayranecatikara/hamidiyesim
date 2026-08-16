# ADAY C · YATAY JERK TAVANI (`PSC_JERK_XY` 5 → 15) — ölçütler KOŞMADAN ÖNCE

**Tarih:** 2026-08-16 · **Durum:** ilan edildi, uçuş bekliyor
**Kill-switch:** `PSC_JERK_XY = 5.0` (firmware varsayılanı = bugünkü davranış)
**Panel:** `Aday C · Yatay jerk tavanı` (tip: `param`)

---

## 0 · ⛔ ÖNCE BİR DÜZELTME — YANLIŞ PARAMETREYİ ÖNERMİŞTİM

`docs/donus_acigi.html` raporunda Aday C'yi **`WPNAV_JERK` 4 → 12** diye
önerdim. **Bu yanlış ve uçsaydık ölü koşu olurdu.** Kaynaktan doğruladım:

```
ArduCopter/mode_guided.cpp:229   pva_control_start()
    pos_control->set_max_speed_accel_NE_cm(hız, ivme)   ← jerk PARAMETRESİ YOK

AC_PosControl.cpp:439 (set_max_speed_accel_NE_cm içinde)
    _jerk_max_ne_cmsss = _shaping_jerk_ne_msss * 100.0;   ← PSC_JERK_XY
```

GUIDED hız modu `wp_nav`'ın **yol üreticisini hiç kullanmıyor**; yalnız
hız/ivme *getter*'larını çağırıyor. `WPNAV_JERK` bu yolda **hiç okunmuyor**.
Bağlayan parametre **`PSC_JERK_XY`** ve firmware varsayılanı **5.0 m/s³**.

Bu tam olarak `avci_copter.parm`'ın baştaki uyarısının hatası:
> *"grep ile `mav_5_1.parm`'da doğrula; çıktı boşsa parametre SESSİZCE yok
> sayılmıştır. Bu tuzağa iki kez düşüldü."*

Üçüncü kez düşmedik — ama düşmemizin sebebi kural, dikkat değil.

**Düzelen sayılar:**

| | rapordaki (yanlış) | gerçek |
|---|---|---|
| bağlayan parametre | `WPNAV_JERK` = 4 | **`PSC_JERK_XY` = 5** |
| 6.25 m/s²'ye ulaşma | 1.6 s → 28 m | **1.25 s → 22.5 m** |

Sonuç değişmedi (jerk sınamaya değer), parametre değişti.

---

## 1 · ÜST SINIR — nereye kadar açılabilir

`AC_PosControl.cpp:441-449` jerk'e iki ek tavan koyuyor:

```
tavan-1 (açısal HIZ):  ATC_RATE_R_MAX = 0  → is_positive() FALSE → UYGULANMAZ
tavan-2 (açısal İVME): 0.5·√(a_max · snap_max)
                     = 0.5·√(8.0 · 188.3) = 19.41 m/s³
```

Yani **19.41 m/s³** gövdeden gelen sert tavan. Deney kolu **15** seçildi:
3× artış, tavanın güvenli altında, mekanizmanın gerçekten devreye girmesi
garanti.

| PSC_JERK_XY | etkin | 6.25 m/s²'ye | 18 m/s'de yol |
|---|---|---|---|
| **5** (kontrol) | 5.00 | 1.25 s | 22.5 m |
| **15** (deney) | 15.00 | 0.42 s | 7.5 m |
| 20 | 19.41 (tavan) | 0.32 s | 5.8 m |

---

## 2 · MEKANİZMA KAPISI (§5.1)

⚠ **Bu parametre uçuş sırasında değiştirilemez.** `velaccel_control_start()`
yalnız alt mod DEĞİŞİMİNDE çağrılıyor (`mode_guided.cpp:564`); hız komutu
akarken `PSC_JERK_XY` değişse bile okunmaz. Bu yüzden:

- Kampanya **env değil, takip BAŞLAMADAN ÖNCE param yazımı + tam restart**
  ile koşulur. Sim ayağa kalkar → param yazılır → `start_chase` ilk hız
  komutunu gönderir → `velaccel_control_start()` **taze değeri okur**.
- Panel düğmesi var ama açıklamasında bu sınır BÜYÜK HARFLE yazılı.

**Mekanizma sütunu:** 20 Hz `iris_roll_deg`'den türetilen yanal ivme
`a = g·tan(roll)` ve onun türevi. Deney kolunda **|ȧ| p90** belirgin
yükselmeli. Yükselmiyorsa o koşu **GEÇERSİZ**, veri noktası değil.

⚠ Örnekleme notu (§5.3): `iris_roll_deg` yalnız kutu olan karelerde
yazılıyor (karelerin ~%29'u). Kapsama oranı her koşuda raporlanır. 1 Hz
`meta.csv` jerk için KULLANILMAZ — jerk 0.2-1 s ölçeğinde iş görüyor.

---

## 3 · TASARIM ZARFI (§5.13)

> **"Bu özelliğin devreye girip TAMAMLANMASI için senaryoda ne bulunmalı?"**

Jerk, ivmenin **değişim hızını** sınırlar. Yalnız **geçici (transient)**
dönüşlerde bağlar. Sürekli dönüşte ilk ~1.25 s'den sonra ivme oturur ve
jerk **alakasız** hâle gelir.

| senaryo | jerk bağlar mı | rolü |
|---|---|---|
| **kare** — düz bacak + basamak köşe | **EVET**, her köşede | ⭐ **KAZANIM BURADA ÖLÇÜLÜR** |
| `duz` + kaçamak — ani basamak | evet, tepki anında | regresyon + ikincil kazanım |
| `circle` — sürekli dönüş | **HAYIR** (oturmuş ivme) | ⭐ **YAPISAL ÖNGÖRÜ TESTİ** |
| `duz` + `yok` — sakin takip | hayır (dönüş yok) | taban, bozulmamalı |

`circle` özellikle ilginç: modelim doğruysa orada **fark ÇIKMAMALI**.
Belirgin fark çıkarsa mekanizma anlayışım yanlış demektir. n=1/kol olduğu
için karar vermez, ama **modeli çürütebilir**.

---

## 4 · ETKİ ALANI TABLOSU (§5.10) — kodu yazmadan önce

| etkilenebilecek davranış | neden etkilenebilir | hangi senaryoda sınanır |
|---|---|---|
| **düz uçuş isabeti** (%65 çekirdek) | `PSC_JERK_XY` global bir araç parametresi — GPS fazı, görsel faz, terminal, hepsi aynı pos_control'ü kullanıyor | `duz`+`yatay` ×2, `duz`+`capraz` ×2 |
| **terminal son yaklaşma** | son 1-2 s'de ani ivme değişimi hedefi kadrajdan atabilir | duz koşularında en yakın menzil + vuruş sınıfı |
| **KURTARMA bekçisi** (K-V2) | daha hızlı jerk = daha sert tutum değişimi = takla eşiğine yaklaşma | duz koşularında KURTARMA sayısı ve süresi raporlanır |
| **dikey kanal** | ⭐ **YAPISAL: ETKİLENEMEZ.** `PSC_JERK_XY` yalnız NE (yatay) düzlemi kurar (`_jerk_max_ne_cmsss`); dikey `PSC_JERK_Z` ayrı parametre ve DEĞİŞMİYOR | uçuş gerekmez — kaynak ayrımı |
| **salınım / yalpalama** | `avci_copter.parm` notu: *"Agresif bir değişiklik; yalpalama görülürse ÖNCE bunu 2.0'a çekin"* (o not `WPNAV_JERK` için ama uyarı geçerli) | her koşuda yatış işaret değişimi/s + \|yatış\| p90 |

**"Hedeflenen yeri iyileştirdi ama başka bir yeri bozdu mu?"** → düz
regresyon koşularının sonucuyla açıkça cevaplanacak.

---

## 5 · ÖLÇÜTLER

### BİRİNCİL

**60 m içinde geçen süre — kare senaryosu, koşu medyanı** (`meta.csv`, 1 Hz).

*§5.5 gerekçe:* Ö-N kampanyasının asıl bulgusu, isabet üretmeyen bir
senaryoda **"en yakın menzil"in kötü bir birincil ölçüt** olduğuydu — 240
saniyenin tek şanslı anını ölçüyor. Kullanıcının hedefi
*"daha kısa sürede vurabilecek"*; onu temsil eden şey hedefle **ne kadar
süre temas mesafesinde kaldığımız**.

*§5.3 örnekleme:* bu bir **integral** (yaklaşık 60-100 örnek üzerinden
süre), tepe değer değil. 1 Hz yeterli — Ö-N'de yanan "en yakın an" tuzağı
burada yok.

*§5.2 geçerlilik eşi:* **"bu değer kötü bir sebeple de artar mı?"**
Evet — hedefe yaklaşamayıp sürekli 55-59 m'de sürünsek de artar. Eşi
**medyan mesafe** (İK-1): ikisi aynı yöne gitmezse sonuç **BÖLÜNMÜŞ**.

### İKİNCİL

| # | ölçüt | kaynak | niye |
|---|---|---|---|
| İK-1 | **medyan mesafe** | meta 1 Hz | ⭐ birincilin geçerlilik eşi |
| İK-2 | dönüş yarıçapı medyanı (biz) | meta, yol eğriliği | mekanizmaya en yakın sonuç ölçütü. Taban: **121.8 m** (hedef 33.1) |
| İK-3 | salınım: `cx` işaret değişimi/s | bbox 20 Hz | §4 zorunlu. Eşi: görsel temas %60 altı → GÜVENİLMEZ |
| İK-4 | yatış işaret değişimi/s ve \|yatış\| p90 | bbox 20 Hz | §4 zorunlu — yalpalama uyarısı |
| İK-5 | görsel fazda kutu oranı | bbox 20 Hz | temas bozuldu mu |
| İK-6 | kuyruk konisi oranı (60 m içi, &lt;30°) | bbox+meta | kök neden ölçütü. Kare tabanı: ~%45 |
| İK-7 | isabet | olay.json | bilgi. Kare tabanı 0/8; n=4/kol'da ayırt etmesi BEKLENMİYOR |
| İK-8 | KURTARMA sayısı/süresi | bbox `durum` | §5.10 etki alanı maddesi |

---

## 6 · KOŞU PLANI (§4 dönüşümlü, §5.9 tür-eşli)

**Kare (kazanım) — 8 uçuş:**
```
C01_K_kare (5)   C02_C_kare (15)   C03_K_kare   C04_C_kare
C05_K_kare       C06_C_kare        C07_K_kare   C08_C_kare
```

**Düz (regresyon) — 4 uçuş, kaçamak türü kollar arasında EŞİT:**
```
C09_K_duz_yatay (5)    C10_C_duz_yatay (15)
C11_K_duz_capraz (5)   C12_C_duz_capraz (15)
```

**Daire (yapısal öngörü) — 2 uçuş, KARAR VERMEZ:**
```
C13_K_daire (5)   C14_C_daire (15)
```

Tür dağılımı her kolda: 4 kare + 1 yatay + 1 capraz + 1 daire. **Eşit.**

n = 4/kol (kare → hüküm kurulur), 2/kol (duz → yalnız kapı),
1/kol (daire → yalnız model çürütme).

---

## 7 · KARAR KURALI — sonuca bakmadan ilan edildi

**GİRER** (varsayılan `PSC_JERK_XY = 15` olur), ÜÇÜ birden sağlanırsa:
1. Birincil (kare, 60 m içinde süre) **artar**, ve
2. İK-1 (medyan mesafe) **artmaz** (yani yakınlaşma gerçek), ve
3. Düz regresyonda deney kolu isabeti **kontrol kolundan az değil**.

**ÇIKAR**, şunlardan biri olursa:
- Birincil **azalır**, **veya**
- Düz regresyonda deney kolu **0/2** alırken kontrol ≥1/2, **veya**
- İK-4 yalpalama ölçütü belirgin kötüleşir (yatış işaret değişimi/s
  kontrol kolunun **1.5 katını** aşarsa).

**KULLANICIYA**: geri kalan her hâl — özellikle birincil artıp medyan
mesafenin de arttığı (yani "uzakta sürünme") durum.

---

## 8 · RAPORDAN ÖNCE ÜÇ SORU (§5.8)

1. **Özellik çalıştı mı?** EVET, tartışmasız. Jerk p90 **7.26 → 14.57 m/s³**
   (kare), yanal ivme medyanı **1.59 → 3.77 m/s²**. Her koşuda param araçtan
   geri okunarak teyit edildi.
2. **Ölçütüm kötü bir sebeple mi iyileşti?** HAYIR. Geçerlilik eşi İK-1
   (medyan mesafe) **aynı yöne** gitti: 74.6 → 59.3 m. Kutu oranı da arttı
   (%26.3 → %37.1). "Uzakta sürünme" senaryosu gerçekleşmedi.
3. **n kaç, hüküm kurulur mu?** Kare n=4/kol ve **tam ayrışma** (p=0.057, bu
   n'de ulaşılabilir en düşük değer). Düz n=4/kol'a **çıkarıldı** (aşağıda
   gerekçesi) ve orada da tam ayrışma. İkisinde de hüküm kurulur.

---

## 9 · SONUÇ (18 uçuş, 2026-08-16)

### Kare — kazanım senaryosu (n=4/kol)

| ölçüt | K5 (kontrol) | C15 (deney) | |
|---|---|---|---|
| **BİRİNCİL** 60 m içinde süre | 68.5 s | **124.0 s** | **+%81 · p=0.057** |
| İK-1 medyan mesafe (geçerlilik eşi) | 74.6 m | **59.3 m** | aynı yönde ✓ |
| İK-2 dönüş yarıçapı (biz) | 118.7 m | **89.9 m** | hedef: 33.1 m |
| en yakın menzil | 19.07 m | **5.31 m** | |
| İK-5 kutu oranı | %26.3 | **%37.1** | |
| İK-6 kuyruk konisi | %44.7 | %50.2 | |
| İK-3 salınım (cx dgş/s) | 0.213 | 0.594 | ⚠ 2.8× kötü |
| İK-4 yatış dgş/s · \|yatış\| p90 | 0.352 · 19.4° | 0.358 · 36.1° | yalpalama eşiği (1.5×) **aşılmadı** |
| İK-7 isabet | 0/4 | 0/4 | düz |
| **mekanizma** jerk p90 | 7.26 | **14.57** | kapı açık ✓ |

Ham değerler — 60 m içinde süre (s):
K `76, 59, 69, 68` · C `124, 132, 110, 124`. **Hiç örtüşme yok.**

### Düz — regresyon (n=4/kol)

| ölçüt | K5 | C15 | |
|---|---|---|---|
| **İSABET** | **4/4** | **0/4** | ⛔ |
| en yakın menzil | 1.20 m | 2.23 m | ⛔ |
| 60 m içinde süre | 110 s | 128 s | daha iyi |
| medyan mesafe | 71.9 m | 50.5 m | daha iyi |
| kutu oranı | %77.2 | %88.0 | daha iyi |
| salınım (cx dgş/s) | 2.782 | 1.840 | daha iyi |
| KURTARMA kare | 19 | 37 | ~2× |

Ham en yakın (m): K `0.58, 1.70, 1.55, 0.84` · C `2.13, 2.51, 1.85, 2.33`.
**Burada da hiç örtüşme yok** — ama ters yönde.

⚠ **n neden 4'e çıkarıldı:** ilan edilen plan düz için n=2/kol'du ve orada
ÇIKAR kapısı tetiklendi (0/2 ↔ 2/2). Karar tamamen o kapıya bakacağı ve
§5.4 n&lt;4'te hüküm kurulmasını yasakladığı için 4 uçuş **eklendi**. Eklenen
uçuşlar özelliğin **aleyhine** olan kapıyı güçlendirdi, lehine değil; ölçüt
değişmedi (§5.6).

### Düz regresyonun MEKANİZMASI — ölçüldü

Deney kolu düzde **her ara ölçütte daha iyi** ama hiç vuramıyor. Temastan
önceki 2 saniyeye bakınca sebep görünüyor:

| terminal faz, son 2 s | K5 | C15 |
|---|---|---|
| \|yanal ivme\| medyan | **0.57 m/s²** | **3.88 m/s²** (6.8×) |
| \|jerk\| p90 | 8.91 | 19.31 |
| hedefin merkezden sapması | 8.5 px | 21.5 px |

Kontrol kolu son iki saniyede neredeyse **düz uçuyor** ve hedefi kadrajın
ortasında tutuyor. Deney kolu yanal olarak savruluyor. Kovalarken işe
yarayan çeviklik, son anda **piksel gürültüsünü kovalamaya** dönüşüyor:
menzil küçüldükçe küçük konum hatalarının açısal karşılığı büyüyor, jerk
serbestken araç o sıçramaları fiilen takip ediyor.

### Daire — yapısal öngörü testi (n=1/kol, karar vermez)

En yakın 3.90 → 3.30 m, dönüş yarıçapı 72.2 → 69.8 m. **Fark yok denecek
kadar az** — ilan edilen öngörü buydu: sürekli dönüşte ivme oturur, jerk
alakasız hâle gelir. **Model çürütülmedi.**

### Video bacağı (§2 adım 4)

`logs/c_C01_K_kare.mp4`, `c_C02_C_kare.mp4`, `c_C09_K_duz_yatay.mp4`,
`c_C10_C_duz_yatay.mp4`.

- **Kazanım:** C01 (kontrol) en yakın anında (19.7 m) hedef ~14 px'lik leke,
  güven 0.62. C02 (deney) en yakın anında (4.5 m) hedef net çözünmüş uçak,
  güven 0.93, araç ~50° yatıkta — mekanizma çalışırken böyle görünüyor.
- **Kayıp:** C09 (kontrol, İSABET) temastan 1 kare önce hedef **tam merkezde**,
  ufuk **düz**, kutu 90×40 px, güven 0.92 — ders kitabı KONTROLLÜ vuruş.
  C10 (deney, ıska) aynı anda hedef merkezin üstünde-solunda ve araç
  **~25° yatıkta**.
- **Çapraz doğrulama (§2 adım 6): çelişki YOK.** Terminal ivme sayıları
  (0.57 ↔ 3.88 m/s²) karelere birebir yansımış.

---

## 10 · KARAR — ilan edilen kurala göre

İlan edilen **ÇIKAR** koşullarından biri tetiklendi:
> *"Düz regresyonda deney kolu 0/2 alırken kontrol ≥1/2"* — n=4'te **0/4 ↔ 4/4**.

GİRER koşulu (3) sağlanamadı. Yalpalama eşiği (1.5×) **aşılmadı**, yani
ÇIKAR yalnız isabet kaybından geliyor.

**→ Kural ÇIKAR diyor.** Ama §5.10 açık: *"gerileme VARSA ölçülüp raporlanır
ve kararı kullanıcı verir; sessizce geçilmez."* Karar kullanıcıda.

**Yapay zekânın önerisi:** Aday C'yi **bu hâliyle** (global jerk 15) ÇIKAR,
ve yerine aşağıdaki C2'yi ölç.

---

## 11 · ADAY C2 — FAZA GÖRE JERK (kanıta dayalı, henüz ölçülmedi)

Veri tek bir şeyi söylüyor: **yüksek jerk kovalamada kazandırıyor, terminalde
kaybettiriyor.** İkisi farklı fazlar ve zaten ayrı kod yollarımız var.

| | kovalama | terminal |
|---|---|---|
| ölçülen etki | 60 m içinde süre +%81 | isabet 4/4 → 0/4 |
| önerilen jerk | 15 | 5 (bugünkü) |

**Tetik:** terminal mandalı (`terminal_mandal`) — zaten var, `TERM_BIRAK_M`
ile bırakılıyor. Mandal girince `PSC_JERK_XY` 5'e, çıkınca 15'e yazılır.

⚠ **Önemli engel:** parametre uçuş sırasında **okunmuyor** (§2). Faz geçişinde
bir GUIDED alt mod yenilemesi gerekir. Uygulanabilirliği koddan
doğrulanmadan C2 önerilmez — bu satır bir **açık soru**, çözüm değil.

**Alternatif (kod yolu):** jerk'i araç parametresi olarak değil, güdümün
kendi komut yumuşatması olarak kurmak — `hiz_yonu`'ndaki değişim hızını
terminalde sınırlamak. Bu, ArduPilot'un mod yenileme sorununu tamamen
atlar ve bizim kontrolümüzde kalır.
