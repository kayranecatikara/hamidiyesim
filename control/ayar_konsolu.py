#!/usr/bin/env python3
"""AYAR KONSOLU — sistemdeki tüm ayarlanabilir büyüklüklerin tek kaydı.

Kullanıcı isteği (2026-08-18): *"arayüze bir buton koy, bu butona basınca bir
panel açılsın ve bu panelden sistemdeki tüm tune edilmesi gereken şeylerin
parametrelerin katsayılarını slidebarlardan ayarlayabileyim... her şeyin de ne
işe yaradığı, neyi kontrol ettiği, neyi artırıp neyi azalttığı bilinsin."*

⚠ BU, DENEY PANELİ DEĞİLDİR. İkisi ayrı yüzeydir:

  • `gcs_server._OZELLIKLER` (🎛 GÜDÜM ÖZELLİKLERİ) → o an KARARI BEKLEYEN
    özellik(ler). CLAUDE.md §0.2 gereği aynı anda EN FAZLA BİR yeni özellik.
  • BU DOSYA (🎚 AYAR KONSOLU) → sistemin TAMAMI. Keşif/tarama içindir,
    karar defteri değildir. Buradan bir şey değiştirmek onu "sisteme girdi"
    yapmaz; girmesi için §1'deki beş adım (öner → ölç → raporla → göster →
    doğrula) gerekir.

⚠ HİÇBİR VARSAYILAN DEĞİŞTİRİLMEDİ. Konsol yalnız var olan alanları OKUR ve
YAZAR. Açılış hâli `manevrada-iyi-terminalde-kotu` etiketiyle birebir aynıdır
(bit bit doğrulandı: 972 girdi kombinasyonunda fark 0.000e+00).

CANLI: `bbox_ibvs.Cfg` bir SINIF ve güdüm döngüsü her karede `cfg.<ALAN>`
okuyor → sınıf niteliği değişince BİR SONRAKİ KAREDEN itibaren geçerli.
Uçuş sırasında, yeniden başlatmadan.

Alan adı "modul:ALAN" biçiminde verilebilir (bkz. gcs_server._hedef_cfg).
"""

# (ad, alan, etiket, grup, tip, birim, min, max, adim,
#  ne_yapar, artarsa, azalirsa)
#
# tip: "sayi"  → kayan çubuk + sayı kutusu (Cfg alanı)
#      "bool"  → aç/kapa (Cfg alanı)
#      "param" → ARAÇ parametresi (MAVLink PARAM_SET, uçuşta yazılır)
#      "secim" → metin seçeneği; min alanına seçenek listesi konur

AYARLAR = [

    # ═════════════════════════════════════════════════════════════════════
    ("G0", None, "① HÜCUM — mesafeyi kapat ve çarp (TEK GÖRSEL YASA)", None,
     "grup", None, None, None, None,
     "⭐ GÖRSEL GÜDÜM TEK PARÇADIR. Ayrı bir 'terminal' fazı YOKTUR — "
     "2026-08-19'da koddan TAMAMEN silindi (kullanıcı fikri: 'görsel faz "
     "ikiye falan bölünmesin, tek parça kalsın'). Eskiden mandal atıldığı an "
     "dokuz şey birden değişiyordu (hız yasası, dikey yasa, dikey tavan, "
     "sönümleme, lead, roll telafisi…) ve ölçülen bütün bitiriş sorunları "
     "oradan çıkıyordu. Kaldırınca 8 uçuşta isabet 2/4 → 4/4, dikey ıska "
     "1.77 → 0.66 m, kör hücum 376 → 0 kare oldu.",
     None, None),

    ("V_HUCUM", "V_HUCUM", "Hücum hız tavanı", "G0", "sayi", "m/s",
     8.0, 30.0, 0.5,
     "Hız burada oturur (PI hep 'kapat' dediği için tavana "
     "dayanır). Hedef 15.1 m/s uçuyor → kapanma = V_TEK − 15.1.",
     "Kapanma hızlanır, temas daha erken olur. ⚠ Dikey kanala oturma "
     "süresi azalır: D2 ölçümünde hızlanınca 3 m içinde |dikey| "
     "0.21 → 1.06 m olmuştu.",
     "Kapanma yavaşlar; dikey daha iyi oturur ama hedefle daha uzun süre "
     "yan yana uçulur. 15.1'in altına inersen hiç yaklaşamazsın."),

    ("K_ELEV", "K_ELEV", "Dikey saf takip kazancı (yatayla aynı)", "G0", "sayi", "",
     0.0, 2.5, 0.05,
     "Dikeyin yatayla AYNI matematiği: hız vektörünün YÖNÜ hedefe döner, "
     "BÜYÜKLÜĞÜ korunur. 1.0 = saf takip (yataydaki K_YAW da 1.0).",
     "Dikey hataya daha sert tepki; 1'in üstü aşım ve salınım üretir.",
     "Dikeyde tembelleşir; 0 = dikey komut hiç verilmez."),

    ("K_VZ_D", "K_VZ_D", "Dikey sönümleme", "G0", "sayi",
     "", 0.0, 3.0, 0.05,
     "Türev sönümlemesi: aracın KENDİ dikey hızı nişanın ötesine geçtiyse "
     "komut geri çekilir. Terminalin K_VZ_D'sini ödünç ALMAZ — kendi "
     "kazancı (§5.12: iki özellik aynı alanı paylaşmasın).",
     "Aşım azalır, hedefin üstünden geçme biter. Çok artarsa gürültüyü "
     "büyütür ve vz tavana çarpar.",
     "0 = sönümleme yok; dikey salınabilir."),

    ("YAVASLA", "YAVASLA", "Kaçıracaksan YAVAŞLA", "G0", "bool",
     None, None, None, None,
     "⚠ SAF TAKİBİN SESSİZ DELİĞİNİ KAPATIR. Vektörün eğilebileceği en dik "
     "açı asin(VZ_MAX/V_TEK) — 8 ve 20 ile yalnız 23.6°. Hedef daha "
     "dikteyse kesişim imkânsız, komut kırpılır, altından geçilir. Bu, "
     "tavan yetmediğinde YATAYI kısar.",
     "AÇIK — kullanıcının kendi fikri ('kaçıracak gibiysek hızı azaltıp "
     "dengeli yaklaşsak'). Ölçüldü (B106): hedef 30°'de vektör 23.6° yerine "
     "30.0°'ye (v=16.0), 45°'de 41.8°'ye (v=12.0) ulaşıyor. Hızı ASLA "
     "artırmaz (B107), 23.6° altında hiç kısmaz (B108).",
     "KAPALI = dik hedefte komut kırpılır ve altından/üstünden geçilir."),

    ("V_HUCUM_MIN", "V_HUCUM_MIN", "Yavaşlama hız tabanı", "G0", "sayi", "m/s",
     6.0, 20.0, 0.5,
     "Yukarıdaki yavaşlamanın inebileceği en düşük hız.",
     "Dik hedefte az yavaşlar; geometri düzelmez ama hedefi kaçırmazsın.",
     "Daha çok yavaşlayıp daha dik girebilir. ⚠ 15.1'in altında hedef "
     "senden uzaklaşır — yalnız geometriyi düzeltmelik kısa süre için."),

    # ═════════════════════════════════════════════════════════════════════
    ("GM", None, "⓪ KUTU → MENZİL ÖLÇÜSÜ", None,
     "grup", None, None, None, None,
     "Menzil iğne deliği kamera bağıntısından çıkar: R = (FX·S)/p. "
     "FX = odak uzaklığı (166.58 px), S = hedefin görünen boyu, p = kutunun "
     "kadrajdaki boyu. ⚠ S'nin SABİT olduğu varsayılır — ama bir uçağın "
     "görünen boyu yatışına ve bakış açısına göre değişir. `p` olarak neyi "
     "aldığın bu hatayı belirler. MODEL: kanat 1.280 m, gövde 0.814 m, "
     "yükseklik 0.286 m (collision mesh'ten doğrulandı).",
     None, None),

    ("BOYUT_OLCU", "BOYUT_OLCU", "⭐ Menzil ölçüsü", "GM", "secim",
     None, ["carpim", "kosegen"], None, None,
     "carpim = sqrt(w·h) (2026-08-19 öncesi tek yol) · "
     "kosegen = sqrt(w²+h²), eksen-hizalı kutunun köşegeni. "
     "KÖŞEGENİN MANTIĞI: kadrajda θ dönmüş İNCE bir çubuk için "
     "w=L·|cosθ|, h=L·|sinθ| olur ve sqrt(w²+h²)=L kalır — YATIŞTAN "
     "BAĞIMSIZ. Talon arkadan bakınca büyük ölçüde ince çubuktur.",
     "kosegen — ÖLÇÜLDÜ (8 uçuş, 5812 kare): görüş açısının %91'i 0-15° "
     "(tam arkadan). O bantta bağıl menzil hatası %22 → %14 (%36 daha iyi). "
     "Teorik yatış duyarlılığı 0-90°: köşegen %19, çarpım %83, w %359.",
     "carpim — bugünkü. Yatışta menzil %83'e varan hata veriyor."),

    ("MENZIL_PX_M_CARPIM", "MENZIL_PX_M_CARPIM", "Kalibre C — çarpım", "GM",
     "sayi", "px·m", 80.0, 400.0, 1.0,
     "R = C / sqrt(w·h). ⚠ 2026-08-19'a kadar 160.0 kullanılıyordu; gerçek "
     "loglardan ölçülen 185.7 (medyan p·R, 0-15° bandı, n=5274). Yani "
     "menziller %14 EKSİK tahmin ediliyordu — kendimizi olduğumuzdan yakın "
     "sanıyorduk.",
     "Menzil tahmini büyür (kendimizi daha uzak sanırız).",
     "Menzil tahmini küçülür."),

    ("MENZIL_PX_M_KOSEGEN", "MENZIL_PX_M_KOSEGEN", "Kalibre C — köşegen",
     "GM", "sayi", "px·m", 120.0, 600.0, 1.0,
     "R = C / sqrt(w²+h²). Ölçülen 296.8 (aynı yöntem). İma ettiği görünen "
     "boy 1.78 m — model köşegeni 1.31 m'den büyük, çünkü YOLO kutusu "
     "görsel modeli gevşek sarıyor. Ampirik sabit doğrusudur.",
     "Menzil tahmini büyür.", "Menzil tahmini küçülür."),

    ("HUCUM_MENZIL_M", "HUCUM_MENZIL_M", "PI'nın sıfır noktası", "GM",
     "sayi", "m", 0.3, 8.0, 0.1,
     "Hız PI'sının dengeye oturduğu MENZİL. Burada hata sıfırlanır. "
     "⚠ 1.0 m = TEMAS, yani 'hiç durma'. Metre cinsinden tutulduğu için "
     "ölçü değiştirince anlamı değişmez.",
     "Daha uzakta dengeye gelir → orada PARK eder (eski hatanın kendisi: "
     "6.4 m'de park ediyordu ve terminal fazı onu ezmek için vardı).",
     "Temasa daha yakın; hep kapatma davranışı korunur."),

    ("LEAD_MENZIL_M", "LEAD_MENZIL_M", "Lead sönme menzili", "GM", "sayi",
     "m", 1.0, 30.0, 0.5,
     "Lead (hedefi öne alma) bu menzilin altında kademeli söner: "
     "lead_ölçek = R / LEAD_MENZIL_M, [0,1] arasına kırpılır.",
     "Lead daha geç söner — yakında da kestirme yapılır.",
     "Lead daha erken söner; temas anı sakinleşir."),

    # ═════════════════════════════════════════════════════════════════════
    ("GY", None, "⭐ YAVAŞLAMA PROFİLİ — menzille azalan kapanma", None,
     "grup", None, None, None, None,
     "KULLANICI FİKRİ: 'hedef araca çarparken hedef araç ile yakın hızlarda "
     "olmak kaçırma riskini minimuma indirir.' Hız şu an SABİT (karelerin "
     "%58'i tam V_HUCUM'da). Bu grup onun yerine menzille azalan bir kapanma "
     "hedefi koyuyor: kapanma = R/T_GO, taban ve tavanla sınırlı. "
     "⛔ KUTU BOYUTUYLA ORANTILI YAPILMADI — kutu 1/R gittiği için o, temas "
     "anında ANİ FREN demek olurdu (sildiğimiz terminal fazının kök nedeni).",
     None, None),

    ("YAVASLAMA", "YAVASLAMA", "⭐ YAVAŞLAMA AÇIK", "GY", "bool",
     None, None, None, None,
     "Açıkken hız yasası değişir: v_los = hedef_hızı_kestirimi + kapanma "
     "profili. Kestirim, kapanma hatasıyla sürülen bir integraldir — "
     "gürültülü kapanma sinyalinin DOĞRU yeri orasıdır (integratör alçak "
     "geçiren filtredir; oransal yola konsa komut titrerdi).",
     "AÇIK = menzille yavaşlayan yaklaşma. Hedefe yakın hızda temas.",
     "KAPALI = bugünkü sabit hız (V_HUCUM'da doygun)."),

    ("T_GO", "T_GO", "Profil zaman sabiti", "GY", "sayi", "s",
     1.0, 12.0, 0.25,
     "kapanma = R / T_GO. 4 s → 20 m'de 5.0 m/s · 10 m'de 2.5 · 5 m'de 1.25. "
     "Adı 'çarpışmaya kalan süre'den gelir: profil, sabit t_go hedefler.",
     "Daha yavaş yaklaşma — dikey/yanal kanala oturma süresi artar. ⚠ Hedefe "
     "kaçma zamanı da verir: ölçüldü, 0.9 m/s kapanmada araç 8 SANİYE 6 "
     "metrede asılı kaldı ve hiç yaklaşamadı.",
     "Daha hızlı yaklaşma; profil sertleşir, tavan daha erken bağlar."),

    ("KAPANMA_TABAN", "KAPANMA_TABAN", "⭐ Kapanma tabanı (Zenon kalkanı)",
     "GY", "sayi", "m/s", 0.3, 6.0, 0.1,
     "⚠ TEMASI GARANTİ EDER. Taban olmasaydı R→0 iken kapanma→0 olurdu ve "
     "araç hedefe MATEMATİKSEL OLARAK asla değmezdi (Zenon paradoksu).",
     "Son metrelerde daha hızlı temas; oturma süresi azalır.",
     "Daha nazik temas. ⚠ Çok düşürürsen hedefe yaklaşma sonsuza uzar."),

    ("KAPANMA_TAVAN", "KAPANMA_TAVAN", "Kapanma tavanı", "GY", "sayi", "m/s",
     1.0, 15.0, 0.5,
     "Uzaktayken imkânsız hız istenmesin diye profilin üst sınırı. Zaten "
     "V_HUCUM da ayrıca bağlar.",
     "Uzaktan daha agresif kapanma.",
     "Uzakta da sakin; hedefe varış gecikir."),

    ("K_I_KAP", "K_I_KAP", "Kestirim öğrenme hızı", "GY", "sayi", "",
     0.0, 3.0, 0.05,
     "Hedefin hızını öğrenen integralin kazancı. Kapanma hatasıyla sürülür: "
     "ölçülen kapanma hedeften büyükse kestirim düşer, küçükse yükselir. "
     "⚠ Kutu kaybolunca DONAR (bayat ölçümle sürüklenmesin).",
     "Hedef hız değişimine daha çabuk uyum. Çok yükseğe çekilirse gürültüyü "
     "içeri alır ve hız salınır.",
     "Sakin ama geç öğrenir; 0 = hiç öğrenmez, kestirim başlangıçta kalır."),

    # ═════════════════════════════════════════════════════════════════════
    ("G1", None, "② YATAY KANAL — hedefi kadrajda ortalama (yaw)", None,
     "grup", None, None, None, None,
     "Kutunun yatay piksel hatası (cx − 320) yaw komutuna çevrilir. "
     "Ölçüldü: yanal kesişme 8/8 yaklaşmada isabet zarfının (±0.65 m) "
     "içinde — bu kanal ÇALIŞIYOR. Dokunurken dikkatli ol.",
     None, None),

    ("K_YAW", "K_YAW", "Yaw kazancı", "G1", "sayi", "", 0.0, 3.0, 0.05,
     "Yatay piksel hatasını yön komutuna çeviren P kazancı. "
     "hiz_yonu = iris_yaw + K_YAW·eps_yaw − sönüm + lead.",
     "Hedefe daha sert dönülür; kadrajda daha çabuk ortalanır. ÇOK "
     "artarsa sağa-sola salınır (aşım) ve yaw doyuma gider.",
     "Yumuşak ama tembel dönüş; manevra yapan hedefin arkasında kalınır, "
     "hedef kadraj kenarına kaçar."),

    ("YAW_RATE_MAX_DEG", "YAW_RATE_MAX_DEG", "Yaw hız tavanı", "G1",
     "sayi", "°/s", 20.0, 300.0, 5.0,
     "Yaw komutunun saniyede kaç derece değişebileceği (slew sınırı).",
     "Ani kırılmalara daha hızlı yetişilir.",
     "Komut yumuşar, salınım söner; ama keskin kaçamakta geç kalınır. "
     "⚠ Ölçüldü: görsel temas kesilince gerçek yaw hızı 550°/s'ye "
     "fırlıyor — bu tavan KOMUTU sınırlar, aracı değil."),

    ("ROLL_TELAFI", "ROLL_TELAFI", "Yatay roll telafisi", "G1", "bool",
     None, None, None, None,
     "Araç yattığında piksel azimutu gerçek azimutu göstermez. Bu, "
     "duruşu kullanarak SEVİYE çerçevesine çevirir (los_seviye).",
     "AÇIK = doğru geometri. 30-40° yatışta 11-14° sapma ölçüldü.",
     "KAPALI = ham piksel okuması; yatışta hedefi yanlış yerde sanır."),

    ("SONUM_T", "SONUM_T", "Yaw sönümleme süresi", "G1", "sayi", "s",
     0.0, 1.0, 0.02,
     "Hedefin kadrajdaki KAYMA HIZINI yaw komutundan çıkarır (D terimi). "
     "0 = kapalı.",
     "Aşım azalır, salınım söner. ÇOK artarsa dönüş tembelleşir ve "
     "hedefi takip edemez.",
     "0'da hiç sönümleme yok — saf P kontrol."),

    ("YAW_MENZIL_REF", "YAW_MENZIL_REF", "Yaw tavanını menzille kıs", "G1",
     "sayi", "m", 0.0, 40.0, 1.0,
     "Bu menzilin ALTINDA yaw hız tavanı kademeli kısılır (0 = kapalı). "
     "Yakında sakin, uzakta çevik olsun diye.",
     "Daha uzaktan kısmaya başlar — yakın hücumda araç sakinleşir.",
     "0 = kapalı; tavan her menzilde aynı."),

    ("YAW_MIN_KAT", "YAW_MIN_KAT", "Yaw tavanı alt sınırı", "G1", "sayi",
     "×", 0.05, 1.0, 0.05,
     "Yukarıdaki kısma en fazla ne kadar indirebilir (tavanın çarpanı).",
     "Kısma zayıflar, araç yakında da çevik kalır.",
     "Yakında yaw neredeyse donar — çok küçükse hedefi takip edemez."),

    ("DONUS_A", "DONUS_A", "Dönüş-farkında hız tavanı", "G1", "sayi",
     "m/s²", 0.0, 20.0, 0.5,
     "Yaw hızı yüksekken ileri hızı kısar (dönüş yarıçapı R = V²/(g·tanθ) "
     "büyümesin diye). 0 = kapalı, tipik açık değeri 9.81.",
     "Dönerken daha çok yavaşlar → daha dar döner.",
     "0 = kapalı; dönüşte hız düşmez, yarıçap büyür."),

    ("DONUS_V_MIN", "DONUS_V_MIN", "Dönüşte hız tabanı", "G1", "sayi",
     "m/s", 4.0, 20.0, 0.5,
     "Yukarıdaki kısmanın inebileceği en düşük hız.",
     "Dönüşte fazla yavaşlamaz; hedefi kaybetmez.",
     "Daha çok yavaşlayabilir; çok düşükse hedef kaçar."),

    ("YANAL_K", "YANAL_K", "Yanal kesişme (PN) kazancı", "G1", "sayi", "",
     0.0, 6.0, 0.25,
     "Orantısal seyrüsefer benzeri yanal ivme terimi. 0 = kapalı, "
     "açık ~3.0. Yalnız YANAL_MENZIL içinde çalışır.",
     "Kesişme noktasına daha agresif kestirme yapar.",
     "0 = kapalı; saf takip (hedefin bulunduğu yere gidilir)."),

    ("YANAL_MENZIL", "YANAL_MENZIL", "PN devreye girme menzili", "G1",
     "sayi", "m", 0.0, 40.0, 1.0,
     "Yukarıdaki terimin hangi menzilin altında çalışacağı.",
     "Daha uzaktan kestirmeye başlar.",
     "Yalnız çok yakında çalışır."),

    # ═════════════════════════════════════════════════════════════════════
    ("GD", None, "⭐ DİKEY KOMUT ÖLÇEĞİ — salınımın kök nedeni", None,
     "grup", None, None, None, None,
     "KULLANICI GÖZLEMİ: 'son anlarda dikeyde hizaya gelmeye çalışılırken "
     "çok salınım oluyor, alttan üstten kaçırılıyor.' Sebep bulundu: dikey "
     "komut vz = v_los·sin(ε) ile hesaplanıyor — bu DURGUN hedef için "
     "doğru. Hedef 15 m/s ile kaçarken gerçek kapanma 1.5-3 m/s; d metrelik "
     "ofseti t_go = R/ṙ sürede kapatmak için gereken vz = ṙ·sin(ε). "
     "Oran v_los/ṙ ≈ 11 kat.",
     None, None),

    ("DIKEY_KAPANMA", "DIKEY_KAPANMA", "⭐ Dikeyi KAPANMA hızıyla ölçekle",
     "GD", "bool", None, None, None, None,
     "Açıkken dikey komut ölçeği v_los yerine kapanma hızı olur: "
     "vz = clamp(ṙ, taban, v_los)·sin(ε).",
     "AÇIK — ÖLÇÜLDÜ (kullanıcı uçuşu 20260820_124706, 251 kare): komut "
     "edilen dikey hız gerekenin 4-18 KATI. 0-3 m'de 3.08 m/s komut, "
     "gereken 0.17. Karelerde: dikey ofset 11.7→2.2 m arasında kusursuz "
     "(−0.26…−0.02), sonra son 2 metrede −0.86'ya dalıp +0.63'e savruluyor.",
     "KAPALI = bugünkü saf takip (durgun hedef varsayımı)."),

    ("DIKEY_KAP_TABAN", "DIKEY_KAP_TABAN", "Dikey ölçek tabanı", "GD",
     "sayi", "m/s", 0.2, 8.0, 0.1,
     "Kapanma ~0 iken (yan yana uçuş) dikey komut da 0 olurdu ve ofset hiç "
     "düzelmezdi. Taban asgari yetki bırakır.",
     "Yavaş kapanırken dikey daha canlı; ama aşırı komut geri gelir.",
     "Daha sakin; çok düşükse yan yana uçarken hizalanma durur."),

    # ═════════════════════════════════════════════════════════════════════
    ("G2", None, "③ DİKEY KANAL — tavan ve bütçe", None,
     "grup", None, None, None, None,
     "Dikey YASA artık ① gruptadır (yatayın aynı matematiği). Burada yalnız "
     "BÜTÇE kalıyor: komutun tavanı. ⚠ Bu tavan aynı zamanda vektörün "
     "eğilebileceği en dik açıyı belirler: asin(VZ_MAX/V_HUCUM). Hedef daha "
     "dikse ① gruptaki YAVASLA devreye girer.",
     None, None),

    ("VZ_MAX", "VZ_MAX", "Dikey hız tavanı (seyir)", "G2", "sayi", "m/s",
     1.0, 15.0, 0.5,
     "Seyirde tırmanma/alçalma hız tavanı. Panel düğmesi ② bunu 3 ↔ 8 "
     "arasında değiştirir.",
     "İrtifa daha hızlı eşitlenir. Araç tarafında WPNAV_SPEED_UP/DN de "
     "yetmeli, yoksa bu tavan boşa gider.",
     "Dikey bütçe daralır; ① DİKEY KAPI irtifayı kapatamaz ve terminale "
     "hiç girilemez."),

    ("K_VZ_D", "K_VZ_D", "Dikey sönümleme", "G2", "sayi", "", 0.0, 3.0, 0.05,
     "Türev sönümleme: vz = vz_nişan + K_VZ_D·(vz_nişan − iris_vz). "
     "Aracın MEVCUT dikey hızını hesaba katar.",
     "Dikey komut daha çabuk oturur; aşım azalır. ÇOK artarsa gürültüyü "
     "büyütür ve vz tavana çarpar.",
     "0 = sönümleme yok; dikey kanal salınır (ölçüldü: vz işaret "
     "değişimi 1.62/s, |vz| p90 8.0 m/s)."),

    # ═════════════════════════════════════════════════════════════════════
    ("G3", None, "④ HIZ — kutu boyutundan PI", None,
     "grup", None, None, None, None,
     "İleri hız, kutu boyutu hatasından PI ile üretilir. Denge kutusu "
     "① gruptaki HUCUM_BOYUT_REF'tir (temas kutusu) — yani hata hep "
     "pozitif kalır ve hız V_HUCUM tavanında oturur. Buradaki kazançlar "
     "o döngünün dinamiğini belirler.",
     None, None),

    ("K_FWD", "K_FWD", "Hız P kazancı", "G3", "sayi", "(m/s)/px",
     0.0, 2.0, 0.05,
     "Kutu boyutu hatasının ANLIK hıza katkısı.",
     "Menzil hatasına daha hızlı tepki; ama hız salınabilir.",
     "Hız yavaş toparlanır; integralin oturması beklenir."),

    ("K_I", "K_I", "Hız İ kazancı", "G3", "sayi", "(m/s)/(px·s)",
     0.0, 0.3, 0.005,
     "Hedefin KENDİ HIZINI öğrenen integral. Kalıcı hatayı siler.",
     "Hedef hızını daha çabuk öğrenir. ÇOK artarsa windup ve aşım.",
     "Öğrenme yavaşlar; hedef hızlanınca geride kalınır."),

    ("MAX_ACCEL", "MAX_ACCEL", "Komut değişim sınırı", "G3", "sayi",
     "m/s²", 1.0, 40.0, 1.0,
     "Hız KOMUTUNUN kare başına ne kadar değişebileceği (slew). Aracın "
     "kendi ivmesi değil, komutun düzgünlüğü.",
     "Komut daha çabuk değişir; ani manevraya hızlı tepki. ⚠ 26 "
     "denendi ve ÖLÇÜLDÜ: en yakın menzil 1.79 → 2.92 m, KÖTÜLEŞTİ.",
     "Komut yumuşar; ani kaçamakta geç kalınır."),


    # ÖA — 18.0'da ölçüldü ve elendi (A kampanyası). Ayar KALIR: kullanıcı
    # kuralı 2026-08-21 "ayar konsolundan hiçbir şey silme". Tarama için durur.
    ("MAX_ACCEL_YATAY", "MAX_ACCEL_YATAY", "ÖA · yatay ivme bütçesi (AYRI)",
     "G3", "sayi", "m/s²", 0.0, 30.0, 1.0,
     "0 = KAPALI (tek 3B tavan = MAX_ACCEL, bugünkü hâl). >0 ise yatay ivme "
     "bu tavana, dikey MAX_ACCEL'e ayrı bağlanır.",
     "⚠ 18.0 ÖLÇÜLDÜ: hız 15.78→15.00, görsel temas %86.8→%64.4, en yakın "
     "menzil 4.40→5.40 m — ÜÇÜ DE KÖTÜLEŞTİ. Komut slew'ini açmak eğim "
     "yetkisini ileri hızdan alıp yön değiştirmeye harcatıyor.",
     "0 = bugünkü davranış, birebir. 12'nin ALTI hiç denenmedi."),

    # ⭐⭐ ÖC — 46 uçuşun (T/A/B/C/D/E) çıktısı. KARARI BEKLEYEN ÖZELLİK.
    ("IC_YERLESME", "IC_YERLESME", "⭐ ÖC · iç yerleşme (yayın içine gir)",
     "G3", "bool", "", None, None, None,
     "Hedef sürekli tek yöne dönerken (yay) hız vektörünü yayın İÇİNE kaydırır "
     "ve derin kestikçe YATAY hızı kısar. NEDEN: ölçüldü — circle_s'te avcı "
     "hedefin çemberinin 1 m DIŞINDA kalıyor, tur başına 8.2 m fazla yol "
     "koşuyor ve kapanma payı 0.07 m/s'ye iniyor. circle_xl'de İÇERİDE ve "
     "6/6 vuruyor. Tetik ṙ değil, λ̇ işaretinin yavaş EMA'sı (duz'da %1).",
     "⛔ ÖLÇÜLDÜ VE ELENDİ (F kampanyası, 8 uçuş, 2026-08-22). MEKANİZMA "
     "KUSURSUZ ÇALIŞTI: kesme 42-58°, avcı çemberin 9-11.5 m İÇİNE girdi. "
     "AMA REGRESYON KAPISI ÇÖKTÜ: circle_xl 7/7 → 0/1, en yakın 1.30 → "
     "10.70 m, temas %84.5 → %50.7. KÖK NEDEN: 'ṙ düşükse daha çok kes' "
     "döngüsü, kesince YAVAŞLADIĞI için ṙ'yi daha da düşürüyor — ters yönde "
     "pozitif geri besleme. Tam kesmede kilitlenip içeride oturuyor, hiç "
     "vurmuyor. Eksik olan bir VURUŞ FAZI (dışa atılma tetiği).",
     "Bugünkü davranış (kuyruk kovalama). Düz uçuşta zaten fark yok."),

    ("IC_KESME_MAX_DEG", "IC_KESME_MAX_DEG", "⭐ ÖC · kesme tavanı",
     "G3", "sayi", "°", 0.0, 80.0, 5.0,
     "Hız vektörünün LOS'tan en fazla ne kadar içeri kayabileceği.",
     "Daha derin girer. ⚠ Fazlası avcıyı hedefin ÖNÜNE düşürür "
     "(DAIRE_TESHIS'in ölçtüğü 130-170° arızası).",
     "Sığ kesme; dar dairede yetmez."),

    ("IC_HIZ_K", "IC_HIZ_K", "⭐ ÖC · derin kesme → yavaşlama",
     "G3", "sayi", "", 0.0, 1.0, 0.05,
     "Kesme derinleştikçe YATAY hızın ne kadar kısılacağı. 0.6'da tam "
     "kesmede 18 → ~7.2 m/s (içeride eş dönmek için gereken hız). "
     "⚠ Dikey kanala DOKUNMAZ — hedef ~1 m/s tırmanıyor, o takip sürmeli.",
     "İçeride daha iyi tutunur; fazlası hedeften kopartır.",
     "0 = hız kısma yok, sadece kesme (ÖB gibi davranır)."),

    ("IC_KAPANMA_HEDEF", "IC_KAPANMA_HEDEF", "⭐ ÖC · istenen kapanma hızı",
     "G3", "sayi", "m/s", 0.0, 5.0, 0.5,
     "ṙ bu değerin altındaysa kesme büyür, üstündeyse kendiliğinden bırakır. "
     "2.0 seçildi: zarf haritasında pay ≥ 2 m/s olan her koşu vurdu (9/9).",
     "Daha ısrarcı keser.", "Erken bırakır."),

    # ⭐ ÖB — T kampanyası + Aşama 1a'nın çıktısı. KARARI BEKLEYEN TEK ÖZELLİK.
    ("KAPANMA_PAYI", "KAPANMA_PAYI", "⭐ ÖB · garantili kapanma payı",
     "G3", "sayi", "m/s", 0.0, 4.0, 0.1,
     "0 = KAPALI. >0 ise: kapanma hızı bu değerin altına düşünce hız vektörü "
     "yayın İÇİNE kaydırılır (kiriş, yaydan kısadır). NEDEN: dar dairede avcı "
     "hedefin çemberinin 1 m DIŞINDA kalıyor, hızının tamamı daha uzun yayı "
     "çevirmeye gidiyor ve kapatmaya 0.07 m/s kalıyor. ⚠ Burun kanalına "
     "DOKUNMAZ — kamera hedefte kalır (ÖA'yı deviren şey temas kaybıydı).",
     "Daha erken/sert keser; kapanma payı artar. ⚠ Fazlası avcıyı hedefin "
     "ÖNÜNE düşürür (DAIRE_TESHIS'in ölçtüğü 130-170° arızası).",
     "Bugünkü davranış (0 = birebir eski yol). Düz uçuşta zaten etkisiz."),

    ("PAYI_K", "PAYI_K", "⭐ ÖB · kesme kazancı", "G3", "sayi", "°/(m/s)",
     0.0, 40.0, 1.0,
     "Eksik kapanmanın kaç dereceye çevrileceği. β = PAYI_K × (hedef − ṙ).",
     "Aynı eksikte daha sert keser; hızlı tepki ama ṙ gürültüsü komuta sızar.",
     "Yumuşak; kesme geç kurulur."),

    ("PAYI_MAX_DEG", "PAYI_MAX_DEG", "⭐ ÖB · kesme tavanı", "G3", "sayi", "°",
     0.0, 60.0, 5.0,
     "Hız vektörünün LOS'tan en fazla ne kadar sapabileceği.",
     "Daha agresif kesme; ÖNE DÜŞME riski artar.",
     "Güvenli ama dar dairede yetmeyebilir."),

    ("KACIS_KD", "KACIS_KD", "Ö1 · kaçış telafisi", "G3", "sayi", "",
     0.0, 5.0, 0.1,
     "Hedef UZAKLAŞIYORSA (kapanma < 0) hızı ANINDA artırır — "
     "integralin 5 saniyesini beklemez. 0 = kapalı. Yalnız hızlandırma "
     "yönünde çalışır.",
     "Kaçan hedefe tepki hızlanır.",
     "0 = kapalı; hız yalnız PI ile toparlanır."),

    ("KACIS_MAX", "KACIS_MAX", "Kaçış telafisi tavanı", "G3", "sayi",
     "m/s", 0.0, 20.0, 0.5,
     "Yukarıdaki terimin ekleyebileceği en fazla hız.",
     "Daha büyük ani hız takviyesi.",
     "Takviye sınırlanır."),

    # ═════════════════════════════════════════════════════════════════════
    ("G6", None, "⑤ LEAD — hedefi öne alma", None,
     "grup", None, None, None, None,
     "Hedefin kadrajdaki KAYMA HIZINDAN nereye gideceği kestirilir ve "
     "nişan oraya alınır. Kestirim yalnız GÖRÜNTÜDEN — hedefin GPS'ine "
     "bakılmaz (yarışma kuralı §10).",
     None, None),

    ("LEAD_SURE", "LEAD_SURE", "Lead süresi", "G6", "sayi", "s",
     0.0, 2.0, 0.05,
     "Hedefin kaç saniye sonraki yerine nişan alınır.",
     "Daha çok öne alınır; dönen hedefte kestirme yapılır. ÇOK artarsa "
     "hedefin önüne düşülür ve kadrajdan kaçırılır.",
     "0 = saf takip (hedefin ŞU ANKİ yerine gidilir); dönen hedefin "
     "sürekli arkasında kalınır."),

    ("LEAD_SONUM", "LEAD_SONUM", "Lead'i menzille sönümle", "G6", "bool",
     None, None, None, None,
     "Yakınlaştıkça lead'i kademeli sıfırlar (yakında öne almanın "
     "anlamı yok, hata büyütür).",
     "AÇIK = yakında lead söner, temas anı sakinleşir.",
     "KAPALI = sabit lead her menzilde."),

    ("LEAD_MAX_DEG", "LEAD_MAX_DEG", "Lead açı tavanı", "G6", "sayi", "°",
     0.0, 60.0, 1.0,
     "Lead teriminin ekleyebileceği en büyük açı.",
     "Daha büyük kestirme açısı.",
     "Kestirme sınırlanır; ani manevrada güvenli."),

    ("LEAD_EMA", "LEAD_EMA", "LOS hızı yumuşatma", "G6", "sayi", "",
     0.02, 1.0, 0.02,
     "Hedefin kadrajdaki kayma hızı ölçümünün EMA katsayısı.",
     "Daha çevik ama gürültülü kestirim.",
     "Daha sakin ama gecikmeli."),

    # ═════════════════════════════════════════════════════════════════════
    ("G7", None, "⑥ ALGI ve FAZ GEÇİŞİ", None,
     "grup", None, None, None, None,
     "Hangi kutu güvenilir sayılır ve GPS ↔ GÖRSEL geçişi ne zaman olur. "
     "⛔ Ölçüldü: 171 saniyede 14 faz değişimi, güdüm logu 9 kez "
     "yeniden açıldı.",
     None, None),

    ("CONF_MIN", "CONF_MIN", "Kutu güven eşiği (güdüm)", "G7", "sayi", "",
     0.05, 0.95, 0.01,
     "Bu güvenin ALTINDAKİ tespit kutusu YOK SAYILIR (bbox güdümü).",
     "Yalnız sağlam kutular kullanılır; ama zayıf tespitte kör kalınır.",
     "Cılız tespitler de kullanılır; gürültülü kutu güdümü bozar."),

    ("BOYUT_MIN", "BOYUT_MIN", "En küçük geçerli kutu", "G7", "sayi", "px",
     2.0, 20.0, 0.5,
     "Bundan küçük kutu gürültü sayılır (menzil hesabı patlamasın).",
     "Uzaktaki küçük hedef de sayılır; menzil tahmini gürültülenir.",
     "Yalnız yeterince büyük kutular; çok uzakta görsel kullanılmaz."),

    ("sup:POSE_CONF_MIN", "sup:POSE_CONF_MIN",
     "E1 · faz geçişi güven eşiği", "G7", "sayi", "", 0.0, 0.95, 0.01,
     "⚠ FAZ ZIPLAMASININ KÖK NEDENİ. Supervisor'ın GÖRSEL faza GİRME "
     "eşiği. Varsayılan 0.0 = eşik YOK; ama güdüm CONF_MIN=0.35 altındaki "
     "kutuyu REDDEDİYOR. Arada kalan tespitlerde supervisor 'görsel' der, "
     "güdüm kutuyu kullanamaz, 20 kare sonra GPS'e dönülür.",
     "0.35'e (CONF_MIN ile aynı) çekmek iki katmanı EŞİTLER. Ölçüldü: "
     "conf 0.268-0.334 olan kareler zıplamayı üretiyordu; 20 saniyede "
     "8 faz değişimi, hız 18.1→8.5 m/s, menzil 28.9→51.0 m açıldı.",
     "0.0 = eşik yok (bugünkü); en cılız tespitte bile görsel faza "
     "girilir."),

    ("sup:KILIT_N", "sup:KILIT_N", "Görsel faza giriş kare sayısı", "G7",
     "sayi", "kare", 1.0, 40.0, 1.0,
     "Görsel faza geçmek için kaç ardışık sağlam tespit gerekir.",
     "Geçiş zorlaşır, yanlış kilit azalır; ama devir GEÇ olur ve hücuma "
     "az mesafe kalır. ⚠ 10→7 denendi, HER ölçütte kötüleşti.",
     "Çabuk devredilir; cılız tespitte de görsel faza girilir."),

    ("sup:KAYIP_M", "sup:KAYIP_M", "GPS'e dönüş kare sayısı", "G7", "sayi",
     "kare", 3.0, 60.0, 1.0,
     "Kaç ardışık kutusuz kareden sonra GPS'e dönülür (~30 Hz).",
     "Görselde daha uzun kalınır; kısa kopmalar faz değiştirmez. ⚠ 20→40 "
     "ÖLÇÜLDÜ ve ELENDİ: kare medyan mesafe 68.6→98.7 m.",
     "Çabuk GPS'e dönülür; terminalde hedef kadrajdan çıkınca hücum "
     "iptal olur."),

    ("sup:GATE_MENZIL", "sup:GATE_MENZIL", "Devir menzil kapısı", "G7",
     "sayi", "m", 5.0, 80.0, 1.0,
     "GATE_KILIT açıksa, görsel faza yalnız bu menzilin altında girilir.",
     "Daha uzaktan devredilir.",
     "Yalnız yakında devredilir."),

    ("sup:GATE_KILIT", "sup:GATE_KILIT", "Devir menzil kapısı AÇIK", "G7",
     "bool", None, None, None, None,
     "Yukarıdaki menzil kapısını devreye sokar.",
     "AÇIK = menzil şartı da aranır.",
     "KAPALI = yalnız tespit sürekliliği aranır (bugünkü)."),

    # ═════════════════════════════════════════════════════════════════════
    ("G8", None, "⑦ ARAÇ — ArduPilot parametreleri (CANLI YAZILIR)", None,
     "grup", None, None, None, None,
     "⚠ Bunlar GÜDÜM değil ARAÇ ayarı; MAVLink PARAM_SET ile uçuş "
     "sırasında yazılır ve geri okunarak teyit edilir. ⚠ ArduPilot "
     "GUIDED kipinde WPNAV_ACCEL / PSC_JERK_XY'yi YALNIZ alt-kip "
     "değişiminde okur — etkisi bir sonraki hız komutu tipinde görünür.",
     None, None),

    ("ANGLE_MAX", "ANGLE_MAX", "Azami yatış/pitch açısı", "G8", "param",
     "santi-derece", 2000.0, 8000.0, 250.0,
     "Aracın eğilebileceği en büyük açı (7000 = 70°). Dönüş yarıçapını "
     "belirler: R = V²/(g·tanθ).",
     "Daha dar dönüş, daha çevik. ⚠ Faz C'de 45/55/70° denendi, "
     "9 kıyasın 9'u gürültü çıktı.",
     "Yumuşak ama geniş dönüş."),

    ("WPNAV_ACCEL", "WPNAV_ACCEL", "Yatay ivme tavanı", "G8", "param",
     "cm/s²", 200.0, 3000.0, 100.0,
     "Aracın yatay hız komutunu ne kadar hızlı takip edeceği (2600 = "
     "26 m/s²).",
     "Hız komutuna daha sert tepki. ⚠ Terminalde fren de sertleşir → "
     "burun daha çok kalkar → hedef kadrajdan çıkar (S1).",
     "Yumuşak hızlanma/yavaşlama; ani kaçamağa geç tepki."),

    ("PSC_JERK_XY", "PSC_JERK_XY", "Yatay jerk sınırı", "G8", "param",
     "m/s³", 2.0, 60.0, 1.0,
     "İvmenin değişim hızı. ⭐ ÖLÇÜLDÜ: yatay çeviklikte BAĞLAYICI olan "
     "TEK kısıt bu — ANGLE_MAX ve WPNAV_ACCEL değişimlerinin sıfır "
     "etkisi vardı, jerk p90 ise parametreyi birebir izliyor.",
     "Daha ani ivme değişimi = daha çevik. Ölçülen salınım da artar.",
     "Yumuşak; 12'ye indirmek kullanıcının gördüğü salınımı azaltmıştı."),

    ("WPNAV_SPEED", "WPNAV_SPEED", "Yatay hız tavanı (araç)", "G8",
     "param", "cm/s", 500.0, 3500.0, 100.0,
     "Aracın kabul edeceği en yüksek yatay hız. Güdümün V_TOPLAM_MAX'ı "
     "bunun üstüne çıkamaz.",
     "Güdüm daha hızlı uçabilir.",
     "Güdüm ne isterse istesin araç bu tavanda kalır."),

    ("WPNAV_SPEED_UP", "WPNAV_SPEED_UP", "Tırmanma hız tavanı", "G8",
     "param", "cm/s", 100.0, 2000.0, 50.0,
     "Aracın kabul edeceği en yüksek TIRMANMA hızı. VZ_MAX bunun "
     "üstüne çıkamaz.",
     "Dikey hizalama hızlanır (① DİKEY KAPI için gerekli).",
     "Dikey bütçe daralır; irtifa kapanmaz."),

    ("WPNAV_SPEED_DN", "WPNAV_SPEED_DN", "Alçalma hız tavanı", "G8",
     "param", "cm/s", 100.0, 2000.0, 50.0,
     "En yüksek ALÇALMA hızı.",
     "Yukarıdan gelen düzeltme hızlanır.",
     "Alçalma yavaş; hedefin üstünde kalınır."),

    ("WPNAV_ACCEL_Z", "WPNAV_ACCEL_Z", "Dikey ivme tavanı", "G8", "param",
     "cm/s²", 50.0, 1500.0, 50.0,
     "Dikey hız komutunun rampa hızı. ⭐ 2026-08-02'de bu 100 (1 m/s²) "
     "iken dikey ıskanın KÖK NEDENİYDİ: 4.65 m kapatmak 3.05 s "
     "sürüyordu, elde 2.4-2.8 s vardı.",
     "Dikey komut daha çabuk uygulanır.",
     "Dikey komut rampalanır; güdüm ne isterse istesin araç yetişemez."),
]


def _grup_mu(k):
    return k[4] == "grup"


def gruplar():
    """[(kod, baslik, aciklama)] — sırayla."""
    return [(a[0], a[2], a[9]) for a in AYARLAR if _grup_mu(a)]


def ayarlar():
    """Grup satırları hariç, gerçek ayarlar."""
    return [a for a in AYARLAR if not _grup_mu(a)]


def bul(ad):
    for a in ayarlar():
        if a[0] == ad:
            return a
    return None


def sozluk(a):
    (ad, alan, etiket, grup, tip, birim, mn, mx, adim,
     ne_yapar, artarsa, azalirsa) = a
    return {"ad": ad, "alan": alan, "etiket": etiket, "grup": grup,
            "tip": tip, "birim": birim, "min": mn, "maks": mx, "adim": adim,
            "ne_yapar": ne_yapar, "artarsa": artarsa, "azalirsa": azalirsa}
