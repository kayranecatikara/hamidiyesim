# CLAUDE.md — bu depoda güdüm geliştirmenin kuralları

Bu dosya depoya dahildir. Bu branch'i çeken herkes bunu da çeker; Claude
(veya başka bir yapay zekâ) bu depoda çalışırken **bu kurallara uyar**.
Geliştirme yolu herkes için aynıdır.

---

# 1 · GELİŞTİRME STRATEJİSİ — döngünün tamamı

Her güdüm özelliği şu beş adımdan geçer. Adım atlanmaz.

**1. YAPAY ZEKÂ ÖNERİR.** Sisteme eklenmesi gereken özelliği, *neden*
gerektiğini ve *hangi ölçümün* onu gösterdiğini söyler. Riskleri ve geri
dönüş yolunu (kill-switch) birlikte sunar. Kullanıcı onaylamadan güdüm
davranışını değiştiren kod yazılmaz.

**2. YAPAY ZEKÂ TEST EDER — EN AZ 4 UÇUŞ.** Bölüm 2'deki test mekanizmasıyla,
bölüm 3'teki senaryo tasarımıyla. Kontrol/deney kolları dönüşümlü koşulur.
Kanıt: taze uçuş + video + log, üçü birden.

**3. YAPAY ZEKÂ RAPOR EDER.** Özellik sistemi iyileştirdi mi, kötüleştirdi mi,
yoksa nötr mü — sayılarla ve kanıt kareleriyle. İyileştirdiyse girer,
kötüleştirdiyse ÇIKAR, nötrse karar kullanıcıya bırakılır. Yapay zekâ kendi
önceki hükmünü çürüten veri bulursa bunu açıkça söyler.

**4. YAPAY ZEKÂ "SEN DE GÖZÜNLE GÖR" DER.** Kullanıcının kendi makinesinde
aynı şeyi uçurabilmesi için **tüm çalıştırma komutlarını sırasıyla** verir
(5 terminal + panel adımları + hangi env değişkeniyle hangi kolun açıldığı).
Neye bakması gerektiğini de söyler: "şunu görürsen benim ölçtüğümle aynı şeyi
görüyorsun demektir."

**5. İNSAN DOĞRULAR.** Kullanıcı uçurur. Yapay zekânın göremediği bir şey
görürse söyler, birlikte yeniden mütalaa edilir.

**Bu yapının amacı:** analiz yükünü insandan alıp yapay zekâya vermek —
insanın analiz kapasitesi limiti geliştirmeyi kısıtlamasın; ama her özelliği
insanın da gözüyle görmesi, yapay zekânın kaçırdığı bir şeye karşı ekstra
güvenlik katmanı olarak kalsın.

---

# 2 · TEST MEKANİZMASI — değişmez sekiz adım

1. **TAZE UÇUŞ.** Testi yapay zekâ koşar. Kullanıcı gözlemci değil,
   doğrulayıcıdır (adım 5).
2. **SANİYEDE 1 KARE + TELEMETRİ.** `python3 tools/ucus_kaydi.py <dizin> <süre>`
   — her kare o anki panel telemetrisiyle eşli olarak `meta.csv`'ye yazılır.
3. **KARELERİ BİRLEŞTİR → VİDEO.**
   `ffmpeg -framerate 5 -i <dizin>/frames/f%04d.jpg -c:v libx264 -pix_fmt yuv420p logs/<ad>.mp4`
4. **VİDEOYU GÖZLE ANALİZ ET.** Kareleri `Read` ile aç ve BAK. İki üç kareye
   bakıp geçmek yeterli DEĞİLDİR: her yaklaşma olayının **giriş → en yakın →
   çıkış sıralı dizisi** incelenir. Sorulacaklar: hedef kadrajın neresinde,
   kutu var mı ve güveni ne, ufuk ne kadar yatık (= araç ne kadar yatıkta),
   hedef büyüyor mu (yaklaşıyoruz) yoksa aynı boyda mı kalıyor (gölge
   ediyoruz), temas anında hedefi nereden görüyoruz.
5. **LOGLARI ANALİZ ET.** `logs/bbox_ibvs_*.csv` (20 Hz) ve
   `logs/gps_guidance_*.csv`. Kıyas her zaman aynı segmentasyonla.
6. **İKİSİNİ ÇAPRAZLA — ZORUNLU.** Görüntü ile log birbirini doğrulamalı.
   Çelişki varsa sonuçtan ÖNCE çelişki raporlanır ve önce o çözülür.
   *(Bu adım gerçekten iş görüyor: panelin "4.8 m" dediği karede hedef
   kadrajda 20 px'ti — 4.8 m'de 45 px olmalıydı. Panel `mesafe` 1 Hz ve
   buluşmadaki kapanma 13-22 m/s olduğu için örtüşme yapıyordu. Video
   olmasaydı bozuk ölçütle karar verilecekti.)*
7. **KARAR.** İyileştirdiyse girer, kötüleştirdiyse çıkar, nötrse kullanıcıya.
8. **RAPOR + ÇALIŞTIRMA KOMUTLARI.** Bölüm 1 adım 4.

## Kanıt sayılMAYAN şeyler

- ⛔ **Eski uçuş loglarını yeniden oynatmak.** Çevrimdışı replay yalnız
  HİPOTEZ üretir. Kabul kararını sadece taze uçuş + video verir.
- ⛔ **Yalnız CSV istatistiği.** Video bacağı olmadan rapor yazılmaz.
- ⛔ **Elle okunmuş panel değeri.** Medyanla kıyasla.
- ⛔ **Tek koşu, hatta iki koşu.** Koşular arası değişkenlik kol farkını
  yutabiliyor (ölçüldü: aynı kolda tepe kutu boyutu 76.5 px ve 22.7 px).
  **En az 4 uçuş**, tercihen kol başına 3.

---

# 3 · SENARYO TASARIMI — testi ölçtüğün şeye göre kur

## 3.1 · Ana kural

**Eklediğin özellik neyle ilgiliyse, testi ÖZELLİKLE o kısmı ölçecek şekilde
kur.** Genel amaçlı bir uçuş koşup içinden anlam çıkarmaya çalışma.

## 3.2 · ⛔ Hedefe sürekli tam daire çizdirme

Varsayılan test senaryosu daire DEĞİLDİR. Neden (yaşandı):

- Drone dairenin içine hiç giremiyor; hedefe ilk anda ters açıdan yaklaşıyor,
  hedef kaçıyor, drone tekrar yaklaşmaya çalışıyor, aynı döngü tekrarlıyor.
- Ölçülen buluşma kapanma hızı medyan 4.9-12.4 / p90 13-22 m/s. Saf kuyruk
  takibinde bu en fazla 3 m/s olurdu (18 − 14.9) — yani buluşmalar **kafa
  kafaya**. İsabet zarfının içinde ~0.05 s kalınıyor.
- Sonuç: her koşu birbirinden çok farklı çıkıyor, hiçbir şey anlaşılmıyor,
  ölçüm gürültüye boğuluyor.

Daire yalnız "sürekli dönen hedefte ne oluyor" sorusunun kendisi
soruluyorsa kullanılır — o zaman da tek başına değil, aşağıdaki testlerin
yanında.

## 3.3 · ✅ VARSAYILAN: düz uçuş + buluşma anında TETİKLENMİŞ KAÇAMAK

Manevra tepkisi böyle ölçülür — tekrarlanabilir, izole, hızlı:

1. Hedef **`duz`** senaryosunda uçar, hıza/irtifaya oturur.
2. Avcı drone takibe başlar ve temiz bir kuyruk yaklaşması kurar.
3. Bir tetikleyici, iki aracın **anlık GPS'ini** izler (`/api/debug/telem`).
4. Mesafe eşiğe inince (tipik **25 m**, yani "tam vuracakken") hedef
   **manuel RC'ye** devralınır ve belirli bir kaçamak uygulanır.
5. Avcı dronun tepkisi ölçülür: tepki gecikmesi, görsel temas korundu mu,
   ıska mesafesi, isabet.

Araç: `tools/kacamak_testi.py`. Kaçamak çeşitleri (hepsi denenmeli):

| ad | RC | ne sınar |
|---|---|---|
| `yatay` | aileron 2000 | ani yanal kırılma — yatay kanal, roll telafisi |
| `dikey_yukari` | elevator 1150 | tırmanış — dikey bütçe, kamera üst görüşü |
| `dikey_asagi` | elevator 1850 | dalış — kamera −30° alt sınırı |
| `capraz` | aileron 1950 + elevator 1250 | birleşik — kanallar arası etkileşim |
| `hizlan` | throttle 2000 | kapanma hızının kaybı |
| `yok` | (kaçamak yok) | TABAN koşusu — kıyas çizgisi |

**`yok` kolu her kampanyada koşulur.** Kaçamaksız isabet oranını bilmeden
kaçamaklı sonuç yorumlanamaz.

## 3.4 · Diğer senaryolar

`square` (basamak dönüşler), `aggressive` (rastgele), `circle_xl/l/m/s/xs`
(farklı çaplar) mevcut. Bunlar ana testi tamamlar, yerine geçmez.

---

# 4 · DENEY DİSİPLİNİ

- **TEK DEĞİŞKEN.** Her koşuda bir şey değişir. A/B kill-switch env ile.
- **DÖNÜŞÜMLÜ A/B.** K, M, K, M, K, M — sim kayması iki kolu eşit etkilesin.
  Bir kolun 3'ünü arka arkaya koşup diğerine geçme.
- **ÖLÇÜTLER VE KARAR KURALI KOŞMADAN ÖNCE YAZILIR.** Birincil/ikincil ayrımı
  ve "hangi kol kaç ölçüt kazanırsa ne olur" önceden ilan edilir. Sonuca
  bakıp ölçüt seçmek yasak.

- ⛔ **SALINIM ÖLÇÜLMEDEN "İYİLEŞTİ" DENMEZ** (kullanıcı kuralı 2026-08-10).
  Yalnız "isabet + en yakın menzil"e bakan bir ölçüt, dengesizce savrulup
  şans eseri çarpan bir aracı ÖDÜLLENDİRİR. Salınan araç kötüdür — çok
  yaklaşma üretse bile. Her karşılaştırmada şunlar da raporlanır:
    * `cx` işaret değişimi / s (hedef kadrajda sağa-sola atıyor mu)
    * yatış işaret değişimi / s ve |yatış| p90
    * yaw komutu değişim hızı (°/s) — doyuma gidiyor mu
    * görsel temas kesintisi sayısı ve süresi

- ⛔ **HER VURUŞ "KONTROLLÜ VURUŞ" DEĞİLDİR.** Temas anı ve ÖNCESİNDEKİ
  kareler tek tek incelenir ve vuruş sınıflandırılır:
    * **KONTROLLÜ**: son ~2 s boyunca hedef kadrajda kesintisiz, `cx`
      merkeze yakın ve sakin, kutu boyutu DÜZGÜN büyüyor, yatış sakin.
    * **ŞANS**: hedef kadraj kenarında ya da aralıklı kayboluyor, kutu
      boyutu sıçramalı, araç salınıyor, temas ani ve rastgele geliyor.
  Şans vuruşu isabet sayılır ama **iyileşme kanıtı sayılmaz**. Rapor
  ikisini ayrı verir. Araç `tools/vurus_kalitesi.py`.
- **GEÇERLİLİK.** Hedef 20-250 m irtifa / 6-25 m/s bandı dışına çıktıysa koşu
  SAYILMAZ. Uçuş boyunca `tools/ucus_bekci.py` çalışır.
- **ÖRTÜŞMEYE DİKKAT.** Panel `mesafe` 1 Hz'tir; kapanma hızlıyken gerçek en
  yakın anı ıskalar. Yakınlık ölçütü 20 Hz bbox logundan (kutu boyutundan)
  alınır.
- **ARŞİVLE.** Her koşunun log dosyaları kendi dizinine kopyalanır. Zaman
  damgası tahmin ederek glob yapma.
- **KILL-SWITCH.** Her davranış değişikliğinin `AVCI_*` env anahtarı olur;
  varsayılanını ölçüm belirler.

---

# 5 · ÖLÇÜM HATALARINA KARŞI MEKANİZMALAR — zorunlu kontrol listesi

Bu bölüm 2026-08-10/11 oturumunda YAPILAN GERÇEK HATALARDAN üretildi.
Her madde bir hataya karşılık gelir. Yeni bir özellik ölçülürken bu liste
baştan sona uygulanır; atlanan madde varsa sonuç RAPOR EDİLMEZ.

## 5.1 · MEKANİZMA KAPISI — "özellik gerçekten çalıştı mı?"

**Kolları kıyaslamadan ÖNCE, özelliğin devreye girdiği KANITLANIR.**
Her özellik, ne kadar iş yaptığını gösteren bir sütun loglar
(`kacis_ek`, `eps_hiz_deg`, `sonum_deg` gibi). Deney kolunda bu sütun
sıfırsa o koşu **veri noktası değil, GEÇERSİZ koşudur**.

*Yaşandı:* Ö6 (ANGLE_MAX 45→55°) "isabet 0/4 → 2/4" diye raporlandı; sonra
loglara bakınca deney kolunun 2 koşusunda araç **38-40°'de kalmış**, yani
45° tavanına bile dayanmamıştı. O koşular fiilen kontrol koşusuydu ve tablo
sahteydi. Ö1 de TERMINAL durumunda inert olduğu için kullanıcının uçuşunda
hiç çalışmamıştı — "fark göremedim" demesinin sebebi buydu.

## 5.2 · ÖLÇÜT GEÇERLİLİK EŞİ — "bu sayı KÖTÜ bir sebeple de düşer mi?"

Her ölçüt için şu soru sorulur ve cevabı yazılır: **"bu değer, sistem
KÖTÜLEŞTİĞİNDE de iyi görünebilir mi?"** Cevap evetse, ölçüt tek başına
raporlanamaz; yanında **geçerlilik eşi** zorunludur.

| ölçüt | kötü sebeple iyileşir mi | zorunlu eşi |
|---|---|---|
| salınım (cx işaret değişimi) | **EVET** — hedef kadrajdan çıkarsa ölçülemez, 0 görünür | görsel temas oranı (%60 altı → GÜVENİLMEZ) |
| en yakın menzil | evet — savrulup şans eseri yaklaşma | vuruş sınıfı (KONTROLLÜ/ŞANS) |
| isabet | evet — dengesiz araç şans eseri çarpar | salınım + vuruş sınıfı |
| temas süresi | evet — uzakta durup hiç yaklaşmamak | kutu boyutu / yaklaşma |

*Yaşandı:* "Ö8 salınımı 0.073 → 0.000 yaptı" diye rapor edildi. Salınım
YALNIZ kutu olan karelerde sayılıyordu; hedefi daha çok kaybeden koşu daha
sakin görünüyordu. Ölçüt, hedefi kaybetmeyi ödüllendiriyordu. Kullanıcı
"ben fark görmedim" deyince yakalandı.

## 5.3 · ÖRNEKLEME HIZI KURALI

Bir ölçütün örnekleme hızı, ölçtüğü şeyin değişim hızının **en az 5 katı**
olmalıdır. Değilse o kaynak kullanılmaz; daha hızlı kaynağa geçilir.

*Yaşandı:* "en yakın menzil" panel telemetrisinden (1 Hz) alınıyordu, ama
buluşmadaki kapanma hızı 13-22 m/s'ydi. Panelin "4.8 m" dediği karede hedef
kadrajda 20 px'ti (4.8 m'de 45 px olmalıydı). Ölçüt 15 m'ye kadar
yanılıyordu. 20 Hz bbox logundan (kutu boyutundan) yeniden hesaplandı.

## 5.4 · EN AZ n=4/KOL — ve altındaki her şey "ARA VERİ"

n < 4 iken **hüküm cümlesi kurulmaz**. Bu aşamadaki sayılar
"ara veri, karar değil" diye sunulur.

*Yaşandı, ÜÇ KEZ:*
- M3: n=2'de "açık 2/2, kapalı 0/2" → n=6'da eridi, nötr çıktı.
- Ö6: n=2'de "0/2 → 2/2" kazanım diye sunuldu → n=4'te mekanizmanın hiç
  çalışmadığı ortaya çıktı.
- Ö8: n=3'te "medyan 1.88 vs 2.40, Ö8 önde" → n=6'da **TERSİNE döndü**
  (2.76 vs 1.96).

⚠ Koşular arası değişkenlik gerçek: aynı kolda tepe kutu boyutu 76.5 px ve
22.7 px ölçüldü (3 kat). Tek koşunun sonucu, kolun kendi dağılımından
çekilmiş rastgele bir sayıdır.

## 5.5 · ÖLÇÜT KULLANICININ HEDEFİNDEN TÜRETİLİR

Birincil ölçüt seçilirken kullanıcının isteği **birebir alıntılanır** ve
ölçütün o cümleyi ölçtüğü gösterilir. Hesaplaması kolay olan değil, hedefi
temsil eden ölçüt seçilir.

*Yaşandı:* "maks açılan mesafe" birincil ölçüt yapıldı ve ÜÇ testte
üst üste kolları ayıramadı — savrulma kaçamağın geometrisinden geliyordu,
iyileşmeyle ilgisi zayıftı. Dahası kullanıcı açıkça *"biraz mesafe açılsa
ama salınım olmasa okeydir"* demişti; seçtiğim ölçüt tam da kabul ettiği
şeyi cezalandırıyordu.

## 5.6 · KENDİ LEHİNE YORUM YASAĞI

Sonuç bölünmüş çıkarsa ölçüt DEĞİŞTİRİLMEZ, kullanıcıya götürülür.
Sınıflandırıcı bir sonucu kendi önerdiğin özelliğin lehine çeviriyorsa,
önce sınıflandırıcının doğruluğu sorgulanır.

*Yaşandı (doğru yapıldı, örnek olsun):* kontrol kolundaki tek isabet
`vurus_kalitesi` tarafından ŞANS sayıldı; ama altı ölçütten BEŞİNİ geçmişti
ve tek takıldığı 1 kopuk kareydi. Eşik fazla katıydı — Ö8 lehine sayıya
çevrilmedi, eşik hatası olarak raporlandı.

## 5.7 · VERİ DAYANIKLILIĞI

Koşu çıktıları **`logs/` altına** yazılır. `/tmp` gecelik temizleniyor.

*Yaşandı, İKİ KEZ:* 12 uçuşluk kampanyanın kare/olay dosyaları ve daha
önce scratchpad'deki tüm scriptler silindi. Özet sayılar `UYGULANACAK.md`'de
olduğu için sonuçlar kurtarıldı, ham veri kurtarılamadı.

## 5.8 · RAPORDAN ÖNCE ÜÇ SORU

Her rapor öncesi yazılı olarak cevaplanır:

1. **Özellik çalıştı mı?** (5.1 — mekanizma sütunu sıfır değil mi)
2. **Ölçütüm kötü bir sebeple mi iyileşti?** (5.2 — geçerlilik eşi ne diyor)
3. **n kaç, ve bu n'de hüküm kurulur mu?** (5.4)

Üçünden birine cevap verilemiyorsa rapor değil, **eksik listesi** sunulur.

## 5.9 · KOLLAR SENARYO KARIŞIMINA GÖRE EŞLENİR

A/B kollarında **her senaryo/kaçamak türünden EŞİT sayıda koşu** olmalı; ve
kıyas **senaryo türü içinde** yapılır. Taban değeri türe göre çok
değişiyorsa, kaba medyan kıyası kolları değil KARIŞIM ORANINI ölçer.

*Yaşandı (2026-08-12):* `yatay` kaçamağında aşım ~66 m, `capraz`ta ~31 m.
Kontrol koluna 3 yatay + 2 capraz, deney koluna 2 yatay + 3 capraz denk
geldi (10. uçuşu yanlış etiketledim). Kaba medyan "65.7 → 34.8 m, %47
iyileşme" dedi. Türe göre eşlenince gerçek etki: yatay −9%, capraz +2%.
Aynı hata daha büyük veride de vardı: "%35 iyileşme" iddiası, eşlenince
yatay −16% / capraz −5%'e indi.

**Kontrol listesi:** kolların tür dağılımını RAPORDAN ÖNCE yazdır; eşit
değilse ya dengele ya da yalnız tür-içi kıyas raporla.

## 5.10 · ⛔ BİR DURUMU DÜZELTİRKEN BAŞKASINI BOZMA — REGRESYON ZORUNLU

**Kullanıcı kuralı (2026-08-12):** "Sistemi bir durumda iyileştirmeye
çalışırken başka durumlardaki halini bozmayalım."

Bir özellik ÖLÇÜLDÜĞÜ senaryoda kazandı diye giremez. Önce şu soru yazılı
cevaplanır: **"bu değişikliğin ETKİLEYEBİLECEĞİ başka hangi durumlar var?"**
Cevaptaki HER durum için regresyon testi koşulur.

**Etki alanı nasıl çıkarılır — tetik koşuluna bak:**
Özellik hangi koşulda devreye giriyorsa, o koşulun sağlanabileceği TÜM
senaryolar etki alanındadır.
*Örnek:* Ö11 `kapanma < −5 m/s` ve `|eps_yaw| > 45°` iken hızı 9 m/s'ye
kısıyor. Bu koşul yalnız "ıska sonrası dönüş"te değil, **sürekli dönen bir
hedefte de sürekli** sağlanabilir → araç kalıcı olarak yavaş kalır ve 15 m/s
hedefi hiç yakalayamaz. Bu yüzden `duz`+kaçamakta kazanması YETMEZ;
`circle` senaryosunda da sınanmalıdır.

**Zorunlu regresyon listesi** (özellik güdüm davranışını değiştiriyorsa):
1. `duz` + kaçamak (özelliğin hedeflendiği durum)
2. `circle` — sürekli manevra; hız/dönüş kısıtları burada kalıcı olur
3. `duz` + `yok` kaçamağı — TABAN; sakin takipte hiçbir şey bozulmamalı


**⛔ ETKİ ALANI TESTİ ZORUNLU** (kullanıcı kuralı 2026-08-13): "bir özellik
eklerken veya bir sorunu çözerken BAŞKA BOZULMA İHTİMALİ OLAN bir şey varsa
onu da uygun bir senaryoda TEST EDELİM."

Kodu yazmadan önce şu tablo doldurulur ve rapora KONULUR:

| etkilenebilecek davranış | neden etkilenebilir | hangi senaryoda sınanır |
|---|---|---|
| ... | ... | ... |

Tabloda yazan HER satır için koşu yapılır. Koşulmayan satır varsa rapor
"eksik listesi"dir, sonuç değildir. Ve şu cümle açıkça cevaplanır:
**"hedeflenen yeri iyileştirdi ama başka bir yeri bozdu mu?"** — bozduysa
gizlenmez, ölçüsüyle raporlanır ve kararı kullanıcı verir.

**EN İYİSİ: YAPISAL GARANTİ.** Regresyon testinden daha güçlü olan,
değişikliğin diğer kanalı MATEMATİKSEL OLARAK etkileyememesidir. Böyle bir
tasarım bulunursa birim testiyle KANITLA.
*Örnek (doğru yapıldı):* Ö12 yaw slew tavanını kısıyor ama hız vektörü
`hiz_yonu`ndan hesaplanıp bu sınırdan GEÇMİYOR. Test B67 — 32 girdi
kombinasyonunda `komut()` çıktısı bit bit aynı. Uçuş yolu DEĞİŞEMEZ.

**Kabul edilebilir gerileme yoktur diye bir kural yok** — ama gerileme
VARSA ölçülüp raporlanır ve kararı kullanıcı verir; sessizce geçilmez.

## 5.11 · "SALINIM" SANDIĞIN ŞEY FİZİK OLABİLİR

Bir sapmayı kontrol kusuru saymadan önce **aracın geometrik sınırıyla**
kıyasla. Dönüş yarıçapı R = V²/(g·tan θ_max); bir U-dönüşü 2R yanal
süpürür.

*Yaşandı:* 66 m'lik "sağa aşım" kazanç ayarıyla çözülmeye çalışıldı
(Ö5/Ö8/Ö9). Sonra ölçüldü: aşım HER koşuda tetikten tam +7 s sonra
oluyor ve 66-69 m — yani 18 m/s'de 2R = 66 m, aracın MİNİMUM dönüş
çemberi. Kazanç değil GEOMETRİ. Çare hızı kısmak (R ∝ V²).


## 5.12 · ⛔ SİLİNEN ÖZELLİK TAMAMEN SİLİNİR — ARTIK BIRAKMA

**Kullanıcı kuralı (2026-08-14):** *"bazı şeyleri ekliyoruz sonra siliyoruz
ama her şeyini silmiyoruz; o sildiğimiz şeylerden bir şeyler sistemde kalıp
sistemi bozuyor."*

Bir özellik ELENDİĞİNDE koddan **eksiksiz** çıkarılır. Kill-switch bırakmak
da YASAKTIR — ölü env anahtarı, okunmayan `Cfg` alanı, boş kalan log sütunu
ve "0 ise atla" kapısı birikince güdüm yolu okunamaz hale gelir ve bir
sonraki değişiklik bunlardan birine çarpar.

**Silme kontrol listesi — hepsi işaretlenmeden "silindi" denmez:**

1. `Cfg` alanı ve `AVCI_*` env varsayılanı
2. Güdüm döngüsündeki/`komut()` içindeki kod bloğu
3. CSV log sütunu (`_CSV_ALANLAR` / `_SUTUNLAR` listesi **ve** `writerow`)
4. `tani` sözlüğüne / `status`'a eklenen anahtarlar
5. `gcs_server._OZELLIKLER` düğmesi
6. Birim testleri
7. Analiz araçlarındaki (`tools/*.py`) o sütuna bakan kod
8. Koşu scriptlerindeki (`~/.avci_sim/*.sh`) env/panel satırı

**Silme sonrası ZORUNLU doğrulama — ikisi birden:**

- `grep -rn "<ALAN>\|<AVCI_ANAHTAR>\|<sutun_adi>" control/ tools/ tests/`
  → **sıfır sonuç** (yalnız tarihsel yorum bloğunda adı geçebilir).
- **Bit bit denklik:** silmeden önceki HEAD ile silinmiş hâl aynı girdilerde
  karşılaştırılır; güdüm çıktısı (vx, vy, vz, yaw) **birebir aynı** olmalı.
  Fark çıkarsa silme sırasında davranış değişmiş demektir — geri al.

Karar ve ölçümler kaybolmaz: `UYGULANACAK.md`'de ve `docs/ibvs_sicili.html`
sicilinde durur. Kod deposu tarihçesi de duruyor — geri getirmek gerekirse
commit'ten alınır. **Ölü kod tutmak arşiv değildir, borçtur.**

---

# 6 · PANELDE CANLI AÇ/KAPA — her yeni özellik için ZORUNLU

**Sisteme eklenen her davranış anahtarının panelde bir aç/kapa düğmesi olur.**
Kullanıcı kuralı (2026-08-10): "her defasında tüm sistemi baştan farklı
ayarlarla çalıştırmak" kabul edilemez — hem zahmetli, hem de farkı anlık
gözlemeyi imkânsız kılıyor.

**Nasıl çalışıyor:** `bbox_ibvs.Cfg` bir SINIF ve güdüm döngüsü her karede
`cfg.<ALAN>` okuyor. Sunucu sınıf niteliğini değiştirince bir sonraki kareden
itibaren geçerli olur — **uçuş sırasında, yeniden başlatmadan.**

**Yeni özellik eklerken yapılacaklar (atlanmaz):**

1. `Cfg`'ye alanı ve `AVCI_*` env varsayılanını ekle (kill-switch).
2. `control/gcs_server.py` içindeki **`_OZELLIKLER`** sözlüğüne bir satır ekle:
   `"ad": ("ALAN", "bool"|"kazanc"|"param", "Etiket", "Açıklama", "anahtar", açık_değeri)`
   - `"param"` tipi ARAÇ parametresidir (ör. `ANGLE_MAX`); uçuş sırasında
     MAVLink `PARAM_SET` ile yazılır, geri okuma `_param_cache`'ten teyit
     edilir. Kod değişikliği gerektirmeyen deneyler böyle yapılır.
3. Başka hiçbir şey gerekmez — panel listeyi `/api/gudum_ozellikleri`'nden
   çeker, arayüz kendiliğinden büyür (HTML/CSS/JS'e dokunma).
4. Raporda kullanıcıya "panelden şu düğmeyi açıp kapatarak farkı gör" de.

**⚠ PANELDE YALNIZ O ANKİ ADIMIN ÖZELLİĞİ DURUR.** Bir özelliğin kararı
verilince — ister sisteme **GİRSİN** ister **ELENSİN** — düğmesi
`_OZELLIKLER`'den **SİLİNİR**. Yeni adıma geçerken önceki adımın düğmesi
temizlenir; panelde her zaman sadece o an denenen şey görünür.

*Neden:* düğmeler birikince panel çöplüğe döner ve kullanıcı o adımda neyi
sınadığını göremez.

⚠ **Düğmeyi silmek YETMEZ.** Özellik ELENDİYSE kodu da tamamen çıkarılır —
bkz. §5.12 (silme kontrol listesi + bit bit denklik doğrulaması). Karar ve
ölçümler `UYGULANACAK.md` ile `docs/ibvs_sicili.html`'de yaşamaya devam eder.

**Sonuç:** çalıştırma komutları ARTIK DEĞİŞMİYOR. Kol seçimi env ile değil,
panelden yapılır. Env anahtarları yine duruyor (otomatik kampanyalar ve
başlangıç varsayılanı için), ama insan testinde gerekmez.

⚠ Otomatik A/B kampanyalarında hâlâ env + tam restart kullan: koşu boyunca
anahtarın değişmediğinden emin olmak deney disiplininin parçası (§4).

---

# 7 · YAPAY ZEKÂNIN ÇALIŞMA BİÇİMİ

- **ARKA PLANDA GİZLİ SHELL YOK.** Uçuşlar ve analizler sohbette, doğrudan
  çalıştırılır ki kullanıcı ne olup bittiğini görsün. Uzun süren bir işi arka
  plana atıp sessiz kalma; boşa zaman harcama.
- Bir şey takılırsa **sebebini bul ve söyle**, sessizce yeniden deneme.
- Kendi önceki hükmünü çürüten veri çıkarsa **açıkça düzelt**.
- Sonuçlar `UYGULANACAK.md`'ye ilgili maddenin altına işlenir.

---

# 8 · DEPO KURALLARI

- **Kafana göre push YOK.** Commit için sor, push için AYRICA sor.
- Merge ile gelen davranış değişikliklerini işaretle; uçuş-kritik yolda
  önce sor.
- Güdüm davranışını değiştiren düzenleme, uygulanmadan önce **riskleriyle
  birlikte** sunulur.

---

# 9 · SİM ÇALIŞTIRMA TUZAKLARI (hepsi yaşandı)

- `pkill -f` **kendi kabuğunu öldürür** (exit 144). Deseni köşeli parantezle
  kır (`gz [s]im`) ya da komutu ayrı script dosyasına al.
- Sim kuran scripti **boruya bağlama** (`| tail`, `| grep`). Arka plandaki sim
  süreçleri yüzünden boru EOF almaz, script orada asılı kalır ve uçuş komutu
  hiç gönderilmez. Çıktıyı dosyaya yaz, sonra dosyayı oku.
- MAVProxy TTY'siz çıkar → `script -qfec "..." /dev/null` ile sarmala.
- **gcs_server'ı tek başına yeniden başlatma** (Gazebo+SITL ayaktayken):
  chase/scenario bağlantısı kopar. Env değişikliği için TAM restart.
- `GZ_SIM_RESOURCE_PATH` tek satırda uzunsa terminalde kırılıyor; kısa
  parçalar hâlinde ekleyerek ver.
- Test bitince araçları havada KONTROLSÜZ BIRAKMA — simi komple kapat.
- `/tmp` temizlenebilir (gecelik). Kritik veriyi `logs/` altına da arşivle.
- `tools/ucus_bekci.py` uçuştan uzun süre koşarsa, sim kapatılınca
  "API cevapsız" ihlali basar — bu uçuş SONRASI artefakttır, koşuyu geçersiz
  kılmaz. Uçuş içi bandı `meta.csv`'den ayrıca doğrula.

---

# 10 · YARIŞMA KISITI (üstün kural)

Görsel temas varken **GPS güdümü yasak** — yalnız bbox. Temas kesilince GPS
serbest. Görsel döngü hedefe dair veriyi devirde bir kez SAYI olarak alır;
canlı GPS erişimi yapısal olarak yoktur (`tests/test_bbox_ibvs.py` B5 bekçisi
bunu sınar).

---

Ayrıntılı arka plan: `docs/OTONOM_UCUS_TESTI.md`,
`.claude/skills/ucus-testi/SKILL.md`, sonuç kaydı `UYGULANACAK.md`.
