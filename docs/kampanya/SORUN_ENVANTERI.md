# SORUN ENVANTERİ — 2026-08-18

> Kullanıcı: *"tek sorun dikeydeki hizalama da değil, başka sorunlar da var;
> sıra sıra hepsini çözmemiz lazım. İlk önce bi tüm sorunların neler olduğunu
> çıkartalım, sonra bu sorunları çözmek için farklı yöntemler geliştirelim."*

**Kaynak:** `logs/kayit/ucus_20260818_113552` — 172 kare (1 Hz), 171 s,
**172 karenin tamamı tek tek incelendi** + 9 adet 20 Hz güdüm logu
(3525 kare) + `kayit.csv`.

⚠ **n = 1 uçuş.** Aşağıdaki *mekanizma* tespitleri tek uçuş içinde 7/7 ve
5/8 gibi ayrık oranlarla doğrulandı — mekanizma kimliği sağlam (§5.1).
Ama **hiçbir çözümün kazanımı ölçülmedi**; §5.4 gereği kazanım iddiası için
kol başına n≥4 gerekir.

---

## 0 · ÖNCE İYİ HABER — bu uçuşta ÇALIŞAN şeyler

Iska vektörü hedefin kendi çerçevesinde ayrıştırıldığında (dikey / yanal
kesişme / boyuna) 8 yaklaşmanın tamamı:

| # | t | en yakın | **DİKEY** | **YANAL** | **BOYUNA** |
|---|---|---|---|---|---|
| 1 | 15 s | 2.85 m | **+2.53** | +0.27 | −2.66 |
| 2 | 51 s | 3.74 m | **+1.37** | +0.12 | −5.17 |
| 3 | 67 s | 3.38 m | **+2.83** | −0.73 | −2.61 |
| 4 | 74 s | 5.74 m | −1.11 | +0.01 | −7.23 |
| 5 | 78 s | 5.88 m | −0.53 | +0.21 | −7.43 |
| 6 | 110 s | **1.25 m** | −0.27 | −0.22 | −2.82 |
| 7 | 122 s | 3.13 m | **+2.41** | +0.39 | −3.41 |
| 8 | 138 s | 4.81 m | **+1.68** | −0.10 | −6.07 |

İsabet zarfı: yanal **±0.65 m**, dikey **+0.29 / −0.13 m**.

> ⭐ **YANAL KESİŞME ÇÖZÜLMÜŞ.** 8/8 yaklaşmada yanal sapma
> −0.73…+0.39 m — zarfın içinde ya da kıyısında. Kullanıcının
> *"yataydaki hizalama iyi"* gözlemi ölçümle birebir doğrulanıyor.
>
> ⛔ Daha önce "yanal ıska" sanılan 2.5-2.8 m'lik `dxy`, aslında
> **BOYUNA** bileşenmiş — yani hedefin arkasında kalmak. Ayrı sorun (S2).

Ayrıca: **5-20 m bandında görsel temas %89.** Takip fazı sağlam.

---

## S1 · ⭐⭐⭐ TERMİNAL FRENİ → BURUN KALKIYOR → HEDEF KADRAJIN ALTINDAN ÇIKIYOR

**En büyük sorun. "Üstünden geçiyoruz"un tek ve yeterli açıklaması.**

### Zincir

```
seyir v_los ≈ 19.4 m/s
   │  terminale giriş: v_los := V_TERMINAL = 16.0     ← ANİ −3.4 m/s BASAMAK
   ▼
quad frene basar (WPNAV_ACCEL 26 m/s² serbest)
   │
   ▼
BURUN YUKARI KALKAR  → ölçülen pitch +21…+29° (maks 28.8°)
   │
   ▼
kamera gövdeye 25° YUKARI vidalı → kamera ekseni 25+27 = ~52° yukarı
   │
   ▼
hedef kadrajın ALTINDAN çıkar → cy 305 → 471 (alt kenar 480)
   │
   ▼
kör kalırız; donmuş komut TIRMANMA'dır (ölçüldü: −10 m/s'ye kadar)
   │
   ▼
HEDEFİN 1.4-2.8 m ÜSTÜNDEN GEÇERİZ
```

### Kanıt 1 — kontenjans tablosu (aynı uçuş, aynı ayar)

| terminale giriş | n | pitch tepe (medyan) | cy tepe (medyan) | **cy > 440 = kadraj dışı** |
|---|---|---|---|---|
| seyir hızı **> 18 m/s** (fren var) | 8 | **26.1°** | **456** | **5/8 = %62** |
| seyir hızı **= 16 m/s** (fren yok) | 30 | **2.5°** | **319** | **1/30 = %3** |

**20 kat fark.** Tek değişken: girişte hız basamağı olup olmaması.

### Kanıt 2 — geometrik sınır (kamera 25° sabit)

Hedef seviyedeyken, gövde pitch'i arttıkça hedefin kadraj konumu:

| pitch | cy | |
|---|---|---|
| 0° | 318 | tamam |
| +15° | 380 | tamam |
| +20° | 407 | tamam |
| **+27°** | **453** | **kenarda** |
| +30° | 478 | kadraj dışı |

Hedef 1 m altımızdayken 4 m menzilde, **pitch +20° yeter** → cy 523 →
kadraj dışı. Ölçülen fren pitch'i 21-29°. **Tam sınırın üstünde uçuyoruz.**

### Kanıt 3 — kareler (172 karenin hepsine bakıldı)

| kare | menzil | dikey | görüntü |
|---|---|---|---|
| #15 | 8.26 m | 0.90 m üstte | hedef kadrajda, **tam nişanda**, kutu net |
| **#16** | **2.85 m** | **2.53 m üstte** | **kadraj TAMAMEN BOŞ** |
| #65 | 8.39 m | 0.24 m | hedef kadrajda, temiz |
| #66 | 4.42 m | 0.46 m | hedef **kadrajın alt kenarında**, iri, tespit ölmüş |
| #138 | 6.08 m | **0.01 m** | mükemmel hizalı, hedef alt-ortada |
| #139 | 4.81 m | 1.68 m üstte | **yok** |

Her seferinde aynı: **6-8 m'de kusursuz → 1 saniye sonra kör ve üstte.**

### Kill-switch durumu
`D2 / TERM_HIZ_KORU` **zaten kodlanmış ve KAPALI** — terminale girerken
seyir hızını koruyor. Bu bulgu tam olarak onu hedefliyor.

---

## S2 · ⭐⭐⭐ TERMİNALDE KAPANMA HIZI 0.9 m/s — hedefi yakalayamıyoruz

`V_TERMINAL = 16.0`, hedef hızı **15.1 m/s** → net kapanma **0.90 m/s**.

- 6 m kapatmak **6.7 saniye** sürer.
- Ölçüldü: yaklaşma 4-5'te araç **8 saniye boyunca 5.7-6.2 m'de asılı kaldı**
  (kareler #73-#82), tespit güveni 0.81-0.89, hedef sürekli kadrajda,
  **ve hiç yaklaşamadı.**
- 8 yaklaşmanın 8'inde de en yakın anda **boyuna −2.6…−7.4 m** — yani
  dikey/yanal mükemmel olsa bile **hedefin arkasındayız**.
- Yaklaşma 6: dikey −0.27, yanal −0.22 (ikisi de zarfa çok yakın) ama
  boyuna **−2.82 m**. Sırf bu yüzden isabet yok.

⚠ Bu S1 ile aynı kökten: `V_TERMINAL` hem freni yaratıyor hem kapanmayı
öldürüyor.

---

## S3 · ⭐⭐ SON 5 METREDE GÖRSEL TEMAS %50'YE DÜŞÜYOR

| menzil bandı | kutu var | oran |
|---|---|---|
| 10-20 m | 59/66 | **%89** |
| 5-10 m | 42/47 | **%89** |
| **0-5 m** | **11/22** | **%50** |

Tam karar anında yarı yarıya körüz. Sebeplerin bir kısmı S1 (pitch), bir
kısmı bağımsız: 3 m'de hedef kadrajın büyük kısmını kaplıyor, tespit
kutusu kararsızlaşıyor.

---

## S4 · ⭐⭐ DİKEY KANAL SALINIYOR

Terminal karelerinde (n=1749):

| ölçüt | değer |
|---|---|
| `vz_cmd` işaret değişimi | **1.62 / s** (≈0.8 Hz salınım) |
| \|vz_cmd\| p90 | **8.01 m/s** |
| ±10 m/s tavanına dayanan kare | %4 |
| \|pitch\| p90 / maks | 15.2° / 28.8° |

9.3 saniyelik bir terminal bacağında `vz_cmd` şu diziyi izledi:
`+0.8, −1.6, −6.7, −3.3, +5.4, +7.0, +0.1, −6.6, −8.9, +4.6, +10.0`.
Kutu boyutu 15.5 ↔ 29.2 arası gidip geldi ve **hiç büyümedi**.

Kısmen S1'in sonucu (pitch ↔ cy geri beslemesi), kısmen kendi başına.

---

## S5 · ⭐⭐ FAZ ZIPLAMASI — 14 kez / 171 s

- **14 faz değişimi** = 4.9/dk. 20 Hz güdüm logu **9 kez yeniden açılmış**.
- Kök neden bulundu (**E1**): iki katman farklı eşik kullanıyor —
  `supervisor.POSE_CONF_MIN = 0.0` (eşik yok) girişe karar veriyor,
  `bbox_ibvs.CONF_MIN = 0.35` kutuyu kullanıyor.
- Bu uçuşta conf **0.268-0.334** olan kareler var (#37-#42, #128-#132):
  supervisor "görsel" diyor, güdüm kutuyu reddediyor.
- Bedeli: t=53-57 s arasında 4 faz değişimi, hız **18.6 → 7.6 m/s**,
  menzil **4.65 → 24.7 m** açıldı.

Kill-switch `E1 / e1_faz_tutarli` **zaten kodlanmış ve KAPALI**.

---

## S6 · ⭐⭐ ISKA SONRASI TOPARLANMA 12-37 SANİYE

| yaklaşma | dip | tepe | maks dikey sapma | geri dönüş |
|---|---|---|---|---|
| 1 | 2.85 m | 20.2 m (+5 s) | **13.74 m** | **36 s** |
| 3 | 3.38 m | 11.8 m (+3 s) | 3.66 m | **37 s** |
| 4 | 5.74 m | 15.5 m (+15 s) | 4.21 m | 27 s |
| 8 | 4.81 m | 16.1 m (+14 s) | 4.94 m | 29 s |

Sebep ölçüldü: **kutu terminalde kaybolunca donan komut TIRMANMA oluyor**
(`vz_cmd` −5.3 → −10.0 dizisi loglandı; negatif = tırman). Kör hücum bunu
sürdürüyor, araç hedefin 13 m üstüne çıkıyor.

171 saniyenin ~**110 saniyesi** toparlanmayla geçti.

---

## S7 · ⭐ YATIŞ TEPESİ 41°

\|roll\| medyan 2.5°, p90 11.8°, **maks 41.1°**. Terminal p90 10.7°.
Takip fazında sorun görünmüyor; kayıp sonrası dönüşlerde tepe yapıyor.
Şimdilik **düşük öncelik** — kullanıcı takip fazından memnun.

---

# ÖNCELİK SIRASI ve ÇÖZÜM ADAYLARI

| # | sorun | etki | çözüm adayı | durum |
|---|---|---|---|---|
| **S1** | terminal freni → kadraj kaybı | %62 kadraj dışı | hız basamağını kaldır | **D2 kodlu, kapalı** |
| **S2** | kapanma 0.9 m/s | 8/8 boyuna ıska | terminal hızını seyirden al | **D2 aynı anahtar** |
| **S5** | faz zıplaması | 14/171 s | iki eşiği eşitle | **E1 kodlu, kapalı** |
| **S6** | kayıpta tırmanma | 12-37 s kayıp | kayıpta irtifayı KORU | yeni |
| **S4** | dikey salınım | 1.62 işaret/s | vz eğim sınırı | yeni |
| **S3** | 0-5 m'de %50 temas | karar anı kör | S1 çözülünce ölç | bekle |
| **S7** | yatış 41° | düşük | — | bekle |

**S1 ve S2 aynı anahtarla (D2) düşüyor.** İlk hamle orası.

---

## ⛔ KULLANICININ SORUSUNA DOĞRUDAN CEVAP

> *"eskiden de bu dikey kaçırma sorunuyla cebelleşmiştik ve çözmüştük,
> aynı çözüm yöntemini denesek olmuyor mu?"*

**O yöntem zaten sistemde ve İŞE YARADI.**

Eski sorun (2026-08-02, `UYGULANACAK.md` §"DÜZELTME"): terminale
**+4.65 m dikey ofsetle** giriliyordu ve `WP_ACC_Z = 1.0 m/s²` bunu
3.05 s'de kapatabiliyordu — elde 2.4-2.8 s vardı. Çözüm: **ofseti
terminalden ÖNCE küçültmek** (istasyon yükselişi 25° → 15°, ofset
4.65 → 2.85 m) + dikey ivme bütçesini açmak.

Bugün ölçülen: `WPNAV_ACCEL_Z = 800` (8 m/s², 8 kat) ve **terminale giriş
anındaki dikey ofset ≈ 0**:

| kare | menzil | dikey ofset |
|---|---|---|
| #64 | 12.44 m | **+0.03 m** |
| #138 | 6.08 m | **−0.01 m** |
| #111 | 1.25 m | **+0.27 m** |

**Yani eski çözüm bugün de çalışıyor** — 6-12 m'ye kadar dikey hizalama
kusursuz. Bugünkü dikey ıska **başka bir yerden** geliyor: terminal fazının
KENDİSİ, son 1 saniyede yeni bir dikey hata üretiyor (S1).

Eski yöntemi tekrar uygulamak fayda etmez, çünkü **kaldıracak bir ön-ofset
kalmadı.**
