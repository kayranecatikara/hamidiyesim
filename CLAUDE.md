# CLAUDE.md — bu depoda güdüm geliştirmenin kuralları

Bu dosya depoya dahildir. Bu branch'i çeken herkes bunu da çeker; Claude
(veya başka bir yapay zekâ) bu depoda çalışırken **bu kurallara uyar**.
Geliştirme yolu herkes için aynıdır.

---

# 0 · BU BRANCH'İN VİZYONU — GÖRSEL GÜDÜM (kuzey yıldızı)

Avcı drone'un güdümü **iki fazlıdır**: GPS fazı + görsel faz. Bu branch'te
odak **yalnızca görsel güdümdür**. Amaç net:

> **GPS güdümü hedefi görsele devrettikten SONRAKİ aşamayı devralıp
> drone'u hedefe ÇARPTIRMAYA kadar götürmek.** Bunun için mümkün olan en
> iyi görsel güdüm algoritmasını kurmak.

## GPS fazı — DONMUŞ REFERANS (takımdaki arkadaşın işi, birebir alınır)
- GPS fazı takımdaki bir arkadaşın (`kayramin_super_gudumu` dalı) yaptığı
  şekilde **en iyi hâlinde** ve **test edilip doğruluğu kanıtlanmıştır**.
- Bu branch'te GPS güdümü **geliştirilmez, denenmez, "iyileştirilmez"**.
  `kayramin_super_gudumu`'ndaki hâli **birebir** kullanılır: `gps_guidance.py`,
  `kurtarma.py`, `common.py` ve `guidance_core.py`'deki `hedef_kadraj_hatasi`
  o daldan byte-aynı çekilir. GPS'e dair "salınım/kurtarma/burun borcu" gibi
  kendi eklemelerim GEÇERSİZDİR — çıkarılır.
- **Parametreler değerce birebir kayramin.** Tek istisna isim şeması: bu
  makinenin ArduCopter firmware'i SI adlarını ister (`ATC_ANGLE_MAX`,
  `WP_SPD`, `PSC_NE_VEL_P`); kayramin'in eski adları (`ANGLE_MAX`,
  `WPNAV_SPEED`, `PSC_VELXY_P`) bu firmware'de **sessizce reddedilir**
  (canlı SITL dökümüyle ölçüldü). Bu yüzden **değerler kayramin'inki, adlar
  bu firmware'in karşılıklarıdır** — kayramin GPS davranışı bu makinede ancak
  böyle birebir çalışır. Eski-ad dosyasını olduğu gibi basmak = firmware
  varsayılanları (10 m/s, 30°) = kayramin davranışının KAYBI. Değişiklik
  önce **canlı `param_request_read` ile** doğrulanır.

## Görsel faz — AKTİF ÇALIŞMA ALANI (asıl iş burada)
- Görsel güdüm, standart IBVS'ten **farklı** kurduğumuz bir algoritmadır.
  Şu an bir görsel güdüm algoritması üzerinde çalışıyoruz; **onun üzerinden
  devam edilir**, sıfırdan başka mimari kurulmaz (aksi açıkça istenmedikçe).
- Bu fazın görevi: GPS→görsel devir noktasından hedefi devralmak, kadrajda
  tutmak, şartname kilidini biriktirmek (§7/§8) ve terminal dalışla **fiziksel
  çarpışmaya** kadar götürmek. Başarı ölçütü = temas/imha, sadece kilit değil.
- İyileştirme daima §1–§4 döngüsüyle yapılır: uçuş → görüntü+log analizi →
  emin değişiklik önerisi → kullanıcı onayı → test komutu → A/B doğrulama.

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

# 5 · YAPAY ZEKÂNIN ÇALIŞMA BİÇİMİ

- **ARKA PLANDA GİZLİ SHELL YOK.** Uçuşlar ve analizler sohbette, doğrudan
  çalıştırılır ki kullanıcı ne olup bittiğini görsün. Uzun süren bir işi arka
  plana atıp sessiz kalma; boşa zaman harcama.
- Bir şey takılırsa **sebebini bul ve söyle**, sessizce yeniden deneme.
- Kendi önceki hükmünü çürüten veri çıkarsa **açıkça düzelt**.
- Sonuçlar `UYGULANACAK.md`'ye ilgili maddenin altına işlenir.

---

# 6 · DEPO KURALLARI

- **Kafana göre push YOK.** Commit için sor, push için AYRICA sor.
- Merge ile gelen davranış değişikliklerini işaretle; uçuş-kritik yolda
  önce sor.
- Güdüm davranışını değiştiren düzenleme, uygulanmadan önce **riskleriyle
  birlikte** sunulur.

---

# 7 · SİM ÇALIŞTIRMA TUZAKLARI (hepsi yaşandı)

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

# 8 · YARIŞMA KISITI (üstün kural)

Görsel temas varken **GPS güdümü yasak** — yalnız bbox. Temas kesilince GPS
serbest. Görsel döngü hedefe dair veriyi devirde bir kez SAYI olarak alır;
canlı GPS erişimi yapısal olarak yoktur (`tests/test_bbox_ibvs.py` B5 bekçisi
bunu sınar).

---

Ayrıntılı arka plan: `docs/OTONOM_UCUS_TESTI.md`,
`.claude/skills/ucus-testi/SKILL.md`, sonuç kaydı `UYGULANACAK.md`.

---

# 7 · ŞARTNAME 6.1 KURALLARI (yerel dala özgü — kilitlenme görevi)

> Bu bölüm gorsel_ibvs_entegre/pn_pred dalının önceki CLAUDE.md'sinden
> korunmuştur (şartname 6.1 ile doğrulandı). Yukarıdaki süreç kuralları
> BUNLARLA birlikte geçerlidir.

# Kilitlenme Görevi — Kalıcı Kurallar (şartname 6.1 ile doğrulandı)

## Geometri (Şekil 2 + 6.1.4)
- AK = kamera karesi (W×H). AV (hedef vuruş alanı, SARI) = yatay [0.25W, 0.75W],
  dikey [0.10H, 0.90H]. AH (kilitlenme dörtgeni, KIRMIZI) = hedef bbox. HH = hedef.
- KİLİT KOŞULU (ikisi birden):
  a) hedefin MERKEZİ AV içinde (bbox'ın tamamı değil),
  b) AH_en >= ESIK·W  VEYA  AH_boy >= ESIK·H (eksenlerden EN AZ BİRİ; VE değil).
- Resmi şart %5; İÇ KARAR/BİLDİRİM EŞİĞİ 0.06 (şartname tavsiyesi: tam %5'te
  paket göndermek hakem incelemesinde hatalı sayılabilir).
- marj = max(en/(ESIK·W), boy/(ESIK·H)).

## Süre kuralları (6.1.4)
- 10.0 sn değerlendirme penceresinde kümülatif kilit >= 5.0 sn; kesik kesik olabilir.
- KARE TOLERANSI: kilit segmenti içindeki kısa boşluklar, bildirilen sürenin
  %5'i bütçesiyle (5 sn için 200 ms) köprülenir; tolerans segmentin BAŞINDA ve
  SONUNDA geçersiz — segment gerçek kilitli kareyle başlar ve biter.
- Değerlendirme penceresi organizasyon tanımlı olabilir (ör. bildirim anı
  referanslı) — bildirim tarafında pencere referansı parametrik.

## Bildirim (6.1.4)
- Kilitlenme, yarışma sunucusuna paketle bildirilir (kilitlenme bitiş zamanı;
  format haberleşme dokümanıyla gelecek — modül soyut arayüz olarak yazılır).
- Pakette beyan edilen sürenin TAMAMI şartı sağlamalı.

## Tespit doğrulama (6.1.1)
- Tek karelik tespit yeterli değil: süreli/çok kareli doğrulama kapısı; tespit
  çıktısı telemetri/görev kaydından doğrulanabilir olmalı.

## Angajman (6.1.3)
- Çarpışma öncesi SON 3.0 sn: aktif takip + güdüm komutları hedefe yönelik
  (telemetriyle ispat) + çarpışma vektörü hedef doğrultusunda + mesafe
  SİSTEMATİK azalıyor.
- ANGAJMAN fazında merkez/%5 şartı ARANMAZ (yakın mesafede bbox ekranı kaplar);
  kriter aktif takiptir. Log şeması ispat üretir: t, faz, bbox, marj,
  komut vektörü, LOS, mesafe.
- Kilit/takip fazında mesafe korunur; angajmanda kapanır.

## Çözünürlük kuralı
- W,H hiçbir dosyada sabit yazılmaz; çalışma anında kaynak kareden okunur.
- Eşikler ve AV sınırları, bbox ile AYNI koordinat çerçevesinde hesaplanır
  (küçültülmüş YOLO karesi kullanılıyorsa geri ölçekleme doğrulanır).

## Dokunulmazlar
- gps_guidance.py'ye tek karakter dokunulmaz. send_velocity / DoW köprü hattı
  değiştirilmez; gerekirse aynı imzalı sarmalayıcı yazılır.

## Çalışma disiplini
- Her istekte SADECE istenen adım yapılır, bitince kısa özet verilir ve DURULUR.
