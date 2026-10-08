# KAMPANYA GE — GÖRME EŞİĞİ (`CONF_MIN` 0.35 → 0.25)

**Tek değişken:** `AVCI_IBVS_CONF` · **n=6/kol, 12 uçuş** · dönüşümlü ·
2026-08-20 · ⚠ Ölçütler ve karar kuralı KOŞMADAN ÖNCE yazıldı (§4).

---

## 1 · TEŞHİS — neden bu?

20 DM uçuşunun kontrol kolundan (yürürlükteki ayar) ölçüldü:

| menzil bandı | kutu var | oran |
|---|---|---|
| 0-5 m | 941/1083 | %86.9 |
| 5-10 m | 900/998 | %90.2 |
| 10-20 m | 2566/3235 | %79.3 |
| **20-40 m** | **1940/3340** | **%58.1** |

Körlük artık YAKINDA değil, **UZAKTA**. (Terminal fazı silinince yakın
menzil düzeldi: eski ölçüm 0-5 m'de %50'ydi.)

**629 görsel kayıp başlangıcının durumu:**

| büyüklük | değer |
|---|---|
| kutu boyutu (medyan) | 14.2 px |
| → menzil (medyan) | 20.9 m |
| **conf (medyan)** | **0.41** |
| **conf (minimum)** | **0.35 = tam eşik** |
| kadraj kenarında | %3 |

Kayıplar kadraj dışına çıkmaktan DEĞİL. Kabul edilen kutuların güven
dağılımı eşikte **kesilmiş**: p05 = 0.371, p10 = 0.390, %12.1'i
[0.35, 0.40) aralığında. Eşiğin hemen altında kütle var demektir.

**Zincir (gcs_server.py:1804-1890):**

```
detect_all(conf ≥ 0.25) → HybridSORT → TargetLock → pose(=det kutusu)
                                                       ↓
                                        bbox_ibvs.CONF_MIN = 0.35  ← KESEN KAPI
```

Tespit modeli **0.25** ile çalışıyor; güdüm **0.35** ile eliyor. Aradaki
tespitler üretiliyor, takip ediliyor, güdüme ulaşıyor ve **atılıyor**.

**Bağımsız doğrulama:** `vision/detector.py:21` — 2026-07-30'da TESPİT
eşiği için yapılmış ölçüm, 40 m'de sabit takipte:

| eşik | yakalama |
|---|---|
| ≥0.45 | %6.7 |
| **≥0.35** | **%63.6** |
| ≥0.25 | %92.9 |

O gün tespit eşiği 0.45→0.25 indirildi. **Güdümün kendi eşiği 0.35'te
bırakıldı** — düzeltme güdüm yoluna hiç yansımadı. Bugün ölçtüğümüz
%58.1 (20-40 m), o tablonun %63.6'sıyla neredeyse birebir.

---

## 2 · KOD DEĞİŞİKLİĞİ (koşulardan önce, davranışa etkisiz)

1. `supervisor.SupCfg.POSE_CONF_MIN` artık **sabit değil**:
   `os.environ.get("AVCI_HYBRID_CONF", IbvsCfg.CONF_MIN)`.
   İki katman yapısal olarak ayrışamaz. Test C1d kaynağa bakar.
2. `KUTU_YOK` satırına iki **teşhis** sütunu: `red_conf`, `kilit_kaynak`.
   Kör karenin sebebini üçe ayırır:
   * `red_conf` dolu → tespit VAR, `CONF_MIN` reddetti ← bu kampanyanın hedefi
   * `kilit_kaynak = tahmin` → kilit COAST'ta, gcs `det=None` veriyor
   * ikisi de boş → tespit hiç yok (conf < 0.25)
3. Ayar konsolu metinleri güncellendi.

**Doğrulama:** 64/64 birim + 17/17 pytest. `komut()` çıktısı **104 976
kombinasyonda bit bit aynı** (§5.12).

---

## 3 · KOLLAR

| kol | `AVCI_IBVS_CONF` | supervisor eşiği |
|---|---|---|
| **K** (kontrol) | 0.35 | 0.35 (bağdan) |
| **D** (deney) | **0.25** | **0.25** (bağdan) |

Tek env değişkeni iki katmanı birden taşır → **tek değişken** korunur.

---

## 4 · MEKANİZMA KAPISI (§5.1)

D kolunda **kabul edilen kutuların ≥ %2'si** `conf ∈ [0.25, 0.35)`
olmalı. Altındaysa o koşu **veri noktası değil, GEÇERSİZ koşudur** —
özellik fiilen devreye girmemiştir.

Ek olarak `red_conf` sütunu K kolunda da dolar; K'daki
"`red_conf ∈ [0.25,0.35)` olan kare sayısı" = D kolunun kazanabileceği
üst sınır. İki kol birbirini denetler.

---

## 5 · ÖLÇÜTLER — koşmadan önce ilan edildi

### Birincil
| # | ölçüt | yön | geçerlilik eşi (§5.2) |
|---|---|---|---|
| **B1** | **İSABET** | düşmemeli | — (kullanıcının hedefi bu) |
| **B2** | **KUTU_YOK % (20-40 m)** | düşmeli | B3 salınım — çöp kutu kabul edip "kör değilim" demek |
| **B3** | **cx işaret değişimi / s** | %20'den fazla artmamalı | görsel temas oranı |

### İkincil
faz düşüşü · koşunun en yakını · |dikey| temasta · |vz| p90 ·
|roll| p90 · devir menzili (görsel faza giriş) · kilit yeniden kurulma
sayısı (`[LOCK]` satırları — yanlış kilit göstergesi)

### KARAR KURALI
* **GİRER** ⟺ B1 düşmez **VE** B2 iyileşir **VE** B3 %20'den fazla artmaz.
* **ÇIKAR** ⟸ B1 düşerse (tek başına yeter).
* **BÖLÜNMÜŞSE** → ölçüt değiştirilmez, kullanıcıya götürülür (§5.6).

---

## 6 · ETKİ ALANI TESTİ (§5.10) — zorunlu tablo

| etkilenebilecek davranış | neden etkilenebilir | hangi senaryoda sınanır |
|---|---|---|
| yanlış kilit (pist/bulut FP) | zayıf tespit kabul edilir; `capture_runway_negatives.py`'de pist başlangıcı conf 0.57 ile FP vermişti | `duz` (yere yakın, pist kadrajda) |
| yaw salınımı | zayıf kutunun merkezi gürültülü → `cx` titrer | `circle_xl` + `duz` |
| hız gürültüsü (`v_los`) | kutu boyu gürültülü → `boyut_hata` gürültülü → integral titrer | hepsi |
| **faz erken girme** | supervisor eşiği de düşüyor → daha UZAKTA görsele geçilir; devir menzili büyürse hücuma daha çok yol kalır (iyi) ama zayıf kilitle girilir (kötü) | hepsi — devir menzili ölçülür |
| yakın menzil (0-5 m) | orada conf zaten yüksek → **etkisiz beklenir**; bozulursa gerileme | `duz` + `yok` |

Her satır için koşu var. **"Hedeflenen yeri iyileştirdi ama başka bir yeri
bozdu mu?"** sorusu rapora açıkça cevaplanacak.

---

## 7 · KOŞU TABLOSU

| # | ad | kol | CONF | senaryo / kaçamak |
|---|---|---|---|---|
| 1 | GE01_K | kontrol | 0.35 | circle_xl / yok |
| 2 | GE02_D | deney | 0.25 | circle_xl / yok |
| 3 | GE03_K | kontrol | 0.35 | duz / yatay |
| 4 | GE04_D | deney | 0.25 | duz / yatay |
| 5 | GE05_K | kontrol | 0.35 | circle_xl / yok |
| 6 | GE06_D | deney | 0.25 | circle_xl / yok |
| 7 | GE07_K | kontrol | 0.35 | duz / yok |
| 8 | GE08_D | deney | 0.25 | duz / yok |
| 9 | GE09_K | kontrol | 0.35 | circle_xl / yok |
| 10 | GE10_D | deney | 0.25 | circle_xl / yok |
| 11 | GE11_K | kontrol | 0.35 | duz / yatay |
| 12 | GE12_D | deney | 0.25 | duz / yatay |

Kol başına: 3 circle_xl + 2 duz+yatay + 1 duz+yok — **senaryo karışımı
eşit** (§5.9).

**ARA RAPOR:** GE01/GE02 bitince mekanizma kapısı raporlanır. Kapı
kapalıysa kampanya orada durur.

---

# SONUÇLAR — 12 uçuş tamamlandı (2026-08-20, 21:53 → 23:06)

## 8 · MEKANİZMA KAPISI (§5.1) — AÇIK

| uçuş | kol | kabul edilen kutuların `conf∈[0.25,0.35)` oranı | K'da eşiğin reddettiği kare |
|---|---|---|---|
| GE01_K | K | — | **531** |
| GE02_D | D | **%36.2** | 0 |
| GE03_K | K | — | 53 |
| GE04_D | D | **%18.1** | 0 |
| GE05_K | K | — | 185 |
| GE06_D | D | **%26.1** | 0 |
| GE07_K | K | — | 8 |
| GE08_D | D | **%23.9** | 0 |
| GE09_K | K | — | 300 |
| GE10_D | D | **%14.9** | 0 |
| GE11_K | K | — | 96 |
| GE12_D | D | **%25.7** | 0 |

Kapı eşiği %2'ydi; en düşük D koşusu %14.9. **Altı D koşusunun altısı da
geçerli.** Kontrol kolunda kör karelerin **%62'si** eşik reddiydi.

## 9 · BİRİNCİL ÖLÇÜTLER (n=6/kol, medyan)

| # | ölçüt | KONTROL 0.35 | DENEY 0.25 | kural | sonuç |
|---|---|---|---|---|---|
| **B1** | İSABET | **5/6** | **6/6** | düşmemeli | ✅ geçti |
| **B2** | kör @20-40 m | **%37.0** | **%27.2** | düşmeli | ✅ geçti |
| **B3** | cx işaret değişimi/s | 1.42 | **1.28** | %20 artmamalı | ✅ geçti (azaldı) |

**B2 senaryo-içi eşli kıyas (§5.9):**

| senaryo | KONTROL | DENEY | |
|---|---|---|---|
| daire | %33.0 | **%26.4** | D |
| daire | %51.8 | **%27.9** | D |
| daire | %52.5 | **%36.7** | D |
| duz+yatay | %34.1 | **%25.4** | D |
| duz+yatay | %39.9 | **%35.0** | D |
| duz+yok | %17.647 | %17.606 | ~berabere |

**Beş net galibiyet, bir berabere, sıfır mağlubiyet.** Eşlenmemiş
permütasyon testi p = 0.093 (n=6'da 0.05'e ulaşmıyor); eşli işaret testi
5/5 tek yönlü p = 0.031.

## 10 · İKİNCİL ÖLÇÜTLER

| ölçüt | KONTROL | DENEY | yön |
|---|---|---|---|
| KUTU_YOK % (genel) | 25.3 | **20.9** | → DENEY |
| görsel temas % | 72.3 | **77.6** | → DENEY |
| faz düşüşü (adet) | 8.0 | **4.5** | → DENEY |
| \|vz\| p90 | 5.66 | **4.81** | → DENEY |
| \|roll\| p90 | 35.1 | 34.8 | denk |
| kör @0-5 m % | 21.0 | **19.8** | → DENEY |
| en yakın (m) | **0.88** | 0.99 | → kontrol (p=0.94) |
| devir menzili (m) | **22.4** | 21.5 | → kontrol |

## 11 · ⭐ YANLIŞ ALARM DENETİMİ — risk TERSİNE döndü

Beklenen risk: *"cılız kutu buluta/piste oturur."* Ölçüldü — kutudan
çıkan kerteriz ile GPS'ten hesaplanan gerçek kerteriz kıyaslandı
(GPS burada **yalnız çevrimdışı analizde**; güdüm yolu §10 gereği
GPS görmüyor):

| | KONTROL 0.35 | DENEY 0.25 |
|---|---|---|
| medyan açısal hata | 4.19° | **2.64°** |
| p90 | 20.80° | **12.98°** |
| p99 | 48.86° | **33.76°** |
| >20° oranı | %10.68 | **%4.51** |
| **>45° (kaba hata)** | **%1.34** | **%0.45** |

**Zayıf kutuların kendisi ayrıca incelendi (yalnız DENEY kolu):**

| güven grubu | n | medyan hata | >45° |
|---|---|---|---|
| **zayıf [0.25, 0.35)** | 1116 | **2.49°** | **%0.09** |
| orta [0.35, 0.60) | 1788 | 2.10° | %0.39 |
| güçlü ≥0.60 | 2015 | 4.08° | %0.69 |

Attığımız kutular yanlış alarm DEĞİLDİ — **en isabetli gruptu.** Sebebi
basit: zayıf tespitler UZAK menzilde oluyor, orada hedef kadrajda yavaş
hareket ediyor ve açısal hata küçük. Güçlü kutuların hatası daha büyük
çünkü onlar yakın menzilde, hedefin kadrajda hızlı süpürdüğü yerde.

## 12 · §5.10 — "başka bir yeri bozdu mu?"

| etkilenebilecek davranış | ölçüm | sonuç |
|---|---|---|
| yanlış kilit (pist/bulut FP) | açısal hata >45°: %1.34 → %0.45 | **bozulmadı, düzeldi** |
| yaw salınımı | cx işaret değişimi 1.42 → 1.28 | **bozulmadı** |
| hız gürültüsü | \|vz\| p90 5.66 → 4.81 | **bozulmadı** |
| faz erken girme | devir menzili 22.4 → 21.5 m | **~1 m daha yakın** (küçük, ters yön) |
| yakın menzil (0-5 m) | kör oran %21.0 → %19.8 | **bozulmadı** |
| KURTARMA bekçisi | K medyan %2.45 · D %0.5 | sistematik fark yok |

**Cevap: hedeflenen yeri iyileştirdi, ölçülen hiçbir yeri bozmadı.**
Ters yönde iki küçük işaret var: devir menzili ~1 m daha yakın, ve
"en yakın" medyanı 0.88 → 0.99 m. İkincisi karışık: KONTROL kümesindeki
1.55 m değeri bir **ıskadır** (GE09_K), yani o kol lehine görünen sayı
aslında başarısız bir uçuştan geliyor.

**Tek aykırı gözlem:** GE04_D'de kilit **118 kez** yeniden kuruldu
(diğerlerinde 12-35). Aynı uçuş kampanyanın en yakın temasını yaptı
(0.24 m) ve isabet etti. Sebebi araştırılmadı; kayıt altına alındı.

## 13 · VİDEO BACAĞI (§2 adım 4-6) — ÇAPRAZLAMA

**GE09_K'nin ıskası incelendi (tek ıska).** Karelerde 7.3 → 6.0 → 4.0 →
2.8 → **1.7 m** dizisi; 1.7 m karesindeki overlay "HEDEF 0.34" yazıyor.
İlk okumada bunu "eşik ıskayı yaptı" sandım — **log bunu çürüttü**:
o anda güdümün gördüğü güven 0.87 → 0.65 idi. Gerçek zincir:

```
t−0.64 s  kutu 154 → 64.8 px'e ÇÖKTÜ (hedef çok yakın, kadrajı taşıyor)
t−0.59 s  kilit COAST'a düştü → det=None → KUTU_YOK
t−0.58 s  KURTARMA bekçisi devreye girdi, komut 0.6 s KESİLDİ
t+1.25 s  temas yeniden kuruldu, hedef 18 px (uzaklaşmış)
```

Iska **`CONF_MIN`'den değil, yakın menzilde kutu çöküşü + KURTARMA
bekçisinden**. Bu ayrı bir sorundur ve bu kampanyanın değişkeni değildir.
⚠ Çapraz doğrulama olmasaydı yanlış bir zafer yazılacaktı (§2 adım 6).

**Zayıf kutuların gözle doğrulanması denendi ve BAŞARISIZ:** 60-70 m'de
kutu 12-13 px; overlay çizgisi hedefin üstünü kapatıyor, kayıt karesinden
gözle teyit mümkün değil. Bu yüzden §11'deki açısal denetim yapıldı —
gözün yapamadığını geometri yaptı.

## 14 · §5.8 — RAPORDAN ÖNCE ÜÇ SORU

1. **Özellik çalıştı mı?** Evet — D kolunda kabul edilen kutuların
   %14.9-36.2'si eşiğin altındaydı; K kolunda kör karelerin %62'si eşik
   reddiydi. Kapı altı koşuda da açık.
2. **Ölçütüm kötü bir sebeple mi iyileşti?** Hayır. Üç geçerlilik eşi de
   aynı yönü gösterdi: salınım azaldı, isabet arttı, nişan hatası küçüldü.
   "Çöp kutu kabul edip kör değilim demek" senaryosu §11'de elendi.
3. **n kaç, hüküm kurulur mu?** n=6/kol (§5.4 asgarisi 4). Eşlenmemiş p
   değerleri 0.05'in üstünde; hüküm **eşli senaryo-içi kıyasa** dayanıyor
   (5 galibiyet, 1 berabere, 0 mağlubiyet).

## 15 · KARAR

Karar kuralı §5: **GİRER** ⟺ B1 düşmez **VE** B2 iyileşir **VE** B3 %20'den
fazla artmaz. **Üçü de sağlandı.**

**AI ÖNERİSİ: `CONF_MIN` 0.35 → 0.25 GİRSİN.**
⚠ Varsayılan DEĞİŞTİRİLMEDİ — kullanıcı kendi uçuşunda doğrulayıp
onaylayana kadar 0.35'te duruyor (`feedback-onay-almadan-ekleme-yok`).
