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

## 8 · RAPORDAN ÖNCE ÜÇ SORU (§5.8) — koşu sonrası doldurulacak

1. Özellik çalıştı mı? (|ȧ| p90 deney kolunda yükseldi mi) → …
2. Ölçütüm kötü bir sebeple mi iyileşti? (İK-1 ne diyor) → …
3. n kaç, bu n'de hüküm kurulur mu? → …
