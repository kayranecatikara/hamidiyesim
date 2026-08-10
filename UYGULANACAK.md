# UYGULANACAK — teker teker, ölçerek

Aşağıdakiler **tek tek** uygulanacak; her maddeden sonra uçulup ölçülecek,
sonuç maddenin altına yazılacak. Bir madde bitmeden diğerine geçilmeyecek.

**Neden böyle:** bir keresinde 8 grup değişiklik bir arada uçuruldu. Bazıları
ölçümle işe yaradı, biri ölçülebilir zarar verdi (görsel kilit: kör uçuş %64,
drone hedefin üstüne çıkıp zemine çakıldı). Hangisinin ne yaptığı ayırt
edilemedi.

**Kural:** her adımda testler çalıştırılır. ⚠ 2026-08-10: aktif görsel yasa
`bbox_ibvs` olduğu için asgari küme **`test_bbox_ibvs` + `test_supervisor` +
`test_gps_guidance`**; `test_visual_lead` artık alternatif yasayı sınıyor.
⚠ **`pytest` bu dosyaların asıl kontrollerini KOŞMAZ** (hepsi `main()` ile
çalışır) — hepsini birden koşmak için:

```bash
for t in tests/test_*.py; do echo "── $t"; PYTHONPATH=. python3 "$t" | tail -2; done
```

---

> Başka bir makinede/dalda devam edeceksen önce **[DEVAM.md](DEVAM.md)**:
> dal senkronu, sistem başlatma, ölçüm araçları, laptop'ta ayrıca gerekenler.

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
atıyor. `AVCI_IBVS_CONF=0.15` ile:

⚠ **Düzeltme (2026-08-10):** ölçüm `AVCI_POSE_CONF=0.15 AVCI_IBVS_CONF=0.15` ile
koşulmuştu ama **`AVCI_POSE_CONF` kodda YOK** (tarandı) — o değişken hiçbir şey
yapmadı, işi tek başına `AVCI_IBVS_CONF` gördü. Sonuç geçerli; tekrar ederken
yalnız `AVCI_IBVS_CONF` yeter. Dedektörün kendi eşiği ayrı: `AVCI_YOLO_CONF`.

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

### Ö5 — ANİ KAÇIŞ: TESPİT + DÖNÜŞ-FARKINDA HIZ TAVANI · SONUÇSUZ (2026-08-10)

Kullanıcı fikri: "hedef manevra yapınca kadrajda hızla kayıp uzaklaşıyor;
drone bunu tespit edip normalden farklı tepki versin — hedefin yöneldiği yöne
roll ile birlikte dönsün."

**Fikrin iki noktası ölçümle düzeltildi:**

1. **Tespit piksel hızıyla yapılamaz.** 1.5 m'de hedef HİÇ manevra yapmasa
   bile saf geçiş geometrisi ~1666 px/s üretir; o ölçütle kurulan dedektör 75
   kare yakaladı ve **75'i de TERMINAL** (normal çarpma anı) çıktı. Menzille
   çarpınca 1/R patlaması gider: `v_yanal = |λ̇|·R` [m/s]. Ölçüldü (2540 kare):
   normal takipte medyan **1.2 m/s** (yakında 0.5), p90 15.7. 8 m/s eşiği
   karelerin %3.2'sini yakalıyor.
2. **Tepki "daha çok roll" olamaz.** Yarıçap = v²/(g·tanθ):

   | | hız | yatış | yarıçap | dönüş hızı |
   |---|---|---|---|---|
   | avcı | 18 m/s | 45° | 33.0 m | 31 °/s |
   | avcı | 18 m/s | 55° | 23.1 m | 45 °/s |
   | **hedef** | 15 m/s | 60° | **13.2 m** | **65 °/s** |
   | avcı | 11 m/s | 45° | 12.3 m | 51 °/s |

   18 m/s'de hiçbir yatış hedefi yakalamıyor; bağlayıcı değişken **HIZ**.
   Uygulanan: `v ≤ g·tan(MANEVRA_ACI)/|λ̇|` (λ̇=1 rad/s → tavan 9.8 m/s).
   Bu, "fren yok" kararının **tespit penceresiyle sınırlı** istisnasıdır.

**Kod:** `bbox_ibvs.Cfg.MANEVRA` (varsayılan KAPALI) + panelde 🎛 düğmesi +
CSV'ye `manevra`/`v_yanal` + testler B43-B48.

**2×2 kampanya (4 uçuş, hepsi taze restart, kollar panelden doğrulandı):**

| kol | Ö5 | ANGLE_MAX | tetik | en yakın | imha |
|---|---|---|---|---|---|
| A2 taban | kapalı | 45° | 24.8 m | 0.20 m | ✓ |
| B | kapalı | **55°** | 24.5 m | 0.19 m | ✓ |
| C | **açık** | 45° | 22.1 m | **0.13 m** | ✓ |
| D | **açık** | **55°** | 23.6 m | 0.26 m | ✓ |

**HÜKÜM: SONUÇSUZ — varsayılan KAPALI kalıyor.** Dördü de vurdu, en yakın
menziller 0.13-0.26 m bandında (gürültü içinde), kol farkı ölçülemedi.

⚠ **SEBEP — TEST AYIRT ETMİYOR (CLAUDE.md §3.1):** `kacamak_testi`'nin
`bekle_hedef_hazir()` fonksiyonu hedefi **tırmanış geçişinde** yakalıyor
(22 m/s okuyor; kararlı seyir **15.2 m/s**). Chase hedef yavaşken başlıyor,
kaçamak t≈19 s'de tetikleniyor ve kesişim her kolda kolaylaşıyor.
Hedefin oturması beklenen TEK koşuda (drone 194 m geriden, kapanma 2.8 m/s,
tetik t=79.6 s) **taban kolu ıskaladı** (en yakın 1.25 m, imha yok).
Ayırt edici geometri odur.

**Özellik mekanik olarak çalışıyor:** C kolunda 8, D kolunda 10 karede
tetiklendi, `v_yanal` 21.0-22.7 m/s'ye çıktı — dedektör tasarlandığı gibi ve
nadir çalışıyor. Ama müdahale ~0.5 s sürdüğü için sonuca yansımadı.

**SIRADAKİ:** önce testi düzelt (hedef kararlı seyre oturana kadar bekle),
sonra kol başına 3 koşuyla tekrarla. Düzeltilmemiş testle alınacak sonuç
kabul edilemez.

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
  ✅ **ÇÖZÜLDÜ (2026-08-09):** açık çevrim pitch trimi yerine kapalı çevrim
  irtifa tutucu (`run_plane_scenario._irtifa_pitch`, PD; yük faktörü payı
  taban, üstüne düzeltme biner, gaza dokunulmaz). `AVCI_SCN_ALT=<m>` ile
  hedef irtifası zorlanabilir → A/B'nin iki kolu AYNI irtifada uçar. Bu, tek
  bir senaryo kusuru değil 08-08'deki dört A/B'yi birden geçersiz kılan
  karıştırıcının kendisiydi (kollar 134-175 m farklı irtifada uçmuştu).

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
| **`ok` oranı** | Görsel yasa her kareye `durum` yazar. `ok` = tespit kutusu temiz ve güvenilir. Diğerleri `kutu_kucuk` / `tespit_yok` / `kor_dalis`. "%51 ok" = karelerin yarısında güdüm sağlam veriyle çalışıyor. |
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
