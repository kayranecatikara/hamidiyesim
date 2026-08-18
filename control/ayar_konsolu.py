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

AYARLAR = [

    # ═════════════════════════════════════════════════════════════════════
    ("G1", None, "① YATAY KANAL — hedefi kadrajda ortalama (yaw)", None,
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
    ("G2", None, "② DİKEY KANAL — irtifa (BOZUK OLAN YER)", None,
     "grup", None, None, None, None,
     "Kutunun dikey pikseli (cy) irtifa komutuna çevrilir. ⛔ ÖLÇÜLDÜ: "
     "temas anındaki dikey ıska 1.4-2.8 m, isabet zarfı dikeyde "
     "+0.29/−0.13 m — hata zarfın 5-20 katı. Sistemin en zayıf yeri.",
     None, None),

    ("CY_NISAN", "CY_NISAN", "Nişan noktası (dikey piksel)", "G2", "sayi",
     "px", 240.0, 400.0, 1.0,
     "Hedefin kadrajda TUTULACAĞI dikey piksel. Kamera 25° yukarı sabit "
     "vidalı; cy=318 seviye hedefe, cy≈300 ise ~5° YUKARI karşılık gelir "
     "(yani hedefin biraz altından yaklaşılır). Kadraj yüksekliği 480.",
     "BÜYÜTMEK hedefi kadrajda AŞAĞI çeker = daha ALTTAN yaklaşılır "
     "(hedefin altına düşme riski).",
     "KÜÇÜLTMEK hedefi YUKARI çeker = daha ÜSTTEN yaklaşılır. ⚠ Ölçüldü: "
     "araç zaten üstten geçiyor (5/8 yaklaşma)."),

    ("K_VZ", "K_VZ", "Dikey kazanç (seyir)", "G2", "sayi", "", 0.0, 2.0, 0.05,
     "Seyir fazında dikey piksel hatasını tırmanma/alçalma hızına çeviren "
     "P kazancı. vz = K_VZ · V_NOM · eps_elev.",
     "İrtifa hatası daha hızlı kapatılır. ÇOK artarsa dikey salınır.",
     "Dikeyde tembelleşir; irtifa farkı kapanmadan terminale girilir."),

    ("VZ_MAX", "VZ_MAX", "Dikey hız tavanı (seyir)", "G2", "sayi", "m/s",
     1.0, 15.0, 0.5,
     "Seyirde tırmanma/alçalma hız tavanı. Panel düğmesi ② bunu 3 ↔ 8 "
     "arasında değiştirir.",
     "İrtifa daha hızlı eşitlenir. Araç tarafında WPNAV_SPEED_UP/DN de "
     "yetmeli, yoksa bu tavan boşa gider.",
     "Dikey bütçe daralır; ① DİKEY KAPI irtifayı kapatamaz ve terminale "
     "hiç girilemez."),

    ("VZ_MAX_TERM", "VZ_MAX_TERM", "Dikey hız tavanı (terminal)", "G2",
     "sayi", "m/s", 1.0, 20.0, 0.5,
     "Terminal hücumunda dikey tavan. ⚠ KRİTİK GEOMETRİ: hız vektörünün "
     "gösterebileceği en dik açı = asin(VZ_MAX_TERM / v_los). 10 ve 16 "
     "ile bu 38.7° — hedef daha yukarıdaysa kesişim İMKÂNSIZ.",
     "Daha dik hücum edilebilir; yukarıdaki hedefe yetişilir.",
     "Dik hücum kapanır; hedef yukarıdaysa altından geçilir."),

    ("K_VZ_D", "K_VZ_D", "Dikey sönümleme", "G2", "sayi", "", 0.0, 3.0, 0.05,
     "Türev sönümleme: vz = vz_nişan + K_VZ_D·(vz_nişan − iris_vz). "
     "Aracın MEVCUT dikey hızını hesaba katar.",
     "Dikey komut daha çabuk oturur; aşım azalır. ÇOK artarsa gürültüyü "
     "büyütür ve vz tavana çarpar.",
     "0 = sönümleme yok; dikey kanal salınır (ölçüldü: vz işaret "
     "değişimi 1.62/s, |vz| p90 8.0 m/s)."),

    ("DIKEY_KAPI_M", "DIKEY_KAPI_M", "① Dikey hizalama kapısı", "G2",
     "sayi", "m", 0.0, 10.0, 0.5,
     "Hedefle aramızdaki DİKEY OFSET bundan büyükse terminale GEÇİLMEZ; "
     "araç seyir yasasında kalıp önce irtifayı eşitler. 0 = kapalı. "
     "Ofset yalnız bbox'tan kurulur (R=160/kutu, ofset=R·sin(el)) — "
     "yarışma kuralına uygun (§10).",
     "Kapı gevşer; daha büyük dikey hatayla terminale girilir.",
     "Kapı sıkılaşır; irtifa iyi eşitlenmeden hücum başlamaz. ÇOK "
     "küçükse hiç terminale girilemez ve araç sürekli beklerde kalır."),

    ("DIKEY_ROLL", "DIKEY_ROLL", "⑤ T1b · dikey roll telafisi", "G2",
     "bool", None, None, None, None,
     "Yatışta cy pikselinin gerçek yükselişi göstermemesini düzeltir "
     "(SEYİR fazında). ⚠ ① DİKEY KAPI buna DAYANIR.",
     "AÇIK = doğru dikey ölçüm. 80 terminal karesinde telafisiz fark "
     "ortalama 11.8°, maks 26.5° — saniyede 5 m sahte tırmanma.",
     "KAPALI = ham okuma; ① de bozulur. Düz uçuşta etkisiz."),

    # ═════════════════════════════════════════════════════════════════════
    ("G3", None, "③ HIZ — kutu boyutundan PI", None,
     "grup", None, None, None, None,
     "İleri hız, kutu boyutu hatasından (BOYUT_REF − boyut) PI ile "
     "üretilir. Kutu küçükse uzaktayız → hızlan. Terminalde bu döngü "
     "DEVRE DIŞI kalır ve V_TERMINAL sabiti kullanılır.",
     None, None),

    ("BOYUT_REF", "BOYUT_REF", "Denge kutu boyutu", "G3", "sayi", "px",
     8.0, 60.0, 1.0,
     "PI'nın hedeflediği kutu boyutu (sqrt(w·h)). Menzil karşılığı "
     "R = 160/boyut → 25 px ≈ 6.4 m, 18 px ≈ 8.9 m.",
     "Daha BÜYÜK kutu istenir = daha YAKIN durulmak istenir → araç "
     "sürekli yaklaşmaya çalışır, hız artar.",
     "Daha küçük kutu yeter = uzakta durulur; gölge edilir, "
     "yaklaşılmaz."),

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

    ("V_TOPLAM_MAX", "V_TOPLAM_MAX", "Yatay hız tavanı", "G3", "sayi",
     "m/s", 5.0, 35.0, 0.5,
     "Güdümün isteyebileceği en yüksek yatay hız. Araç tarafında "
     "WPNAV_SPEED de yetmeli.",
     "Kaçan hedefe yetişme şansı artar; dönüş yarıçapı R=V²/(g·tanθ) "
     "büyür.",
     "Araç yavaş kalır; 15.1 m/s uçan hedefi hiç yakalayamaz."),

    ("MAX_ACCEL", "MAX_ACCEL", "Komut değişim sınırı", "G3", "sayi",
     "m/s²", 1.0, 40.0, 1.0,
     "Hız KOMUTUNUN kare başına ne kadar değişebileceği (slew). Aracın "
     "kendi ivmesi değil, komutun düzgünlüğü.",
     "Komut daha çabuk değişir; ani manevraya hızlı tepki. ⚠ 26 "
     "denendi ve ÖLÇÜLDÜ: en yakın menzil 1.79 → 2.92 m, KÖTÜLEŞTİ.",
     "Komut yumuşar; ani kaçamakta geç kalınır."),

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
    ("G4", None, "④ TERMİNAL — hücum (BİTİRİŞ)", None,
     "grup", None, None, None, None,
     "Kutu TERMINAL_BOYUT'u aşınca (ve ① kapısı açıksa) hücum mandalı "
     "atılır: PI devre dışı, hız sabit, hedefe KESİŞİM vektörü. "
     "⛔ Ölçüldü: bitiriş burada bozuluyor.",
     None, None),

    ("TERMINAL_BOYUT", "TERMINAL_BOYUT", "④ Terminale geçiş eşiği", "G4",
     "sayi", "px", 8.0, 50.0, 1.0,
     "Bu kutu boyutu aşılınca hücum başlar. Menzil karşılığı R=160/boyut: "
     "18 px ≈ 8.9 m, 25 px ≈ 6.4 m.",
     "BÜYÜTMEK = DAHA GEÇ geçiş (daha yakında). Hücuma az mesafe kalır.",
     "KÜÇÜLTMEK = DAHA ERKEN geçiş (daha uzakta). Hedef hâlâ manevra "
     "yaparken taahhüt edilir; ıska sonrası dönüş maliyeti artar."),

    ("TERMINAL_SURE", "TERMINAL_SURE", "Terminal kör hücum süresi", "G4",
     "sayi", "s", 0.5, 8.0, 0.25,
     "Terminalde kutu kaybolunca son komutla kaç saniye devam edilir. "
     "Süre dolunca ıska sayılır ve GPS'e dönülür.",
     "Kör hücum uzar; hedef kadrajdan çıksa bile devam edilir. ⚠ "
     "Ölçüldü: donmuş komut TIRMANMA olabiliyor (−10 m/s) → araç "
     "hedefin 13 m üstüne çıkıyor.",
     "Kutu kaybolunca çabuk vazgeçilir; ama son metrelerde temas zaten "
     "%50'ye düşüyor (0-5 m bandı)."),

    ("V_TERMINAL", "V_TERMINAL", "③ Hücum hızı", "G4", "sayi", "m/s",
     8.0, 30.0, 0.5,
     "Terminalde SABİT tutulan hız (PI devre dışı). ⚠ Hedef 15.1 m/s "
     "uçuyor → kalan kapanma = V_TERMINAL − 15.1.",
     "Kapanma hızlanır, terminal kısalır. ⚠ AMA seyir hızından çok "
     "FARKLIYSA giriş anında fren/gaz basamağı oluşur → burun oynar → "
     "hedef kadrajdan çıkar (S1).",
     "16 → kapanma 0.9 m/s → 6 metre 6.7 saniye sürer; ölçüldü, araç "
     "8 saniye 6 m'de asılı kaldı ve hiç yaklaşamadı (S2)."),

    ("V_TERM_MIN", "V_TERM_MIN", "Hücum hız tabanı", "G4", "sayi", "m/s",
     4.0, 20.0, 0.5,
     "Dikey bütçe kısıtı yatayı kısarken inilebilecek en düşük hız.",
     "Dik hücumda daha az yavaşlar.",
     "Dik hücum için daha çok yavaşlayabilir; hedefi kaçırma riski."),

    ("TERM_BIRAK_M", "TERM_BIRAK_M", "Terminali bırakma menzili", "G4",
     "sayi", "m", 0.0, 60.0, 1.0,
     "Menzil bunun üstüne çıkarsa hücum iptal edilir (ıska kabul). "
     "0 = kapalı.",
     "Daha geç vazgeçilir; ıskadan sonra kovalamaya devam.",
     "Çabuk vazgeçilip yeni yaklaşma kurulur."),

    ("KAPANMA", "KAPANMA", "Dikeyi kapanma hızıyla ölçekle", "G4", "bool",
     None, None, None, None,
     "Terminalde dikey hızı v_los yerine KAPANMA hızıyla ölçekler "
     "(hedefe göreli yaklaşma hızı).",
     "AÇIK = dikey komut gerçek kapanmaya göre; yavaş kapanırken "
     "aşırı tırmanma olmaz.",
     "KAPALI = tam hızla ölçekler (A1 ile aynı etki)."),

    ("KAPANMA_MIN", "KAPANMA_MIN", "Kapanma ölçeği tabanı", "G4", "sayi",
     "m/s", 0.0, 10.0, 0.25,
     "Yukarıdaki ölçeğin inebileceği en düşük değer.",
     "Yavaş kapanırken bile dikey komut canlı kalır.",
     "Kapanma yavaşsa dikey komut da sönükleşir."),

    ("KAPANMA_EMA", "KAPANMA_EMA", "Kapanma yumuşatma", "G4", "sayi", "",
     0.02, 1.0, 0.02,
     "Kapanma hızı ölçümünün kare başına yumuşatma katsayısı (EMA).",
     "Daha çevik ama gürültülü kapanma tahmini.",
     "Daha sakin ama gecikmeli tahmin."),

    # ═════════════════════════════════════════════════════════════════════
    ("G5", None, "⑤ TERMİNAL ADAYLARI — ölçüldü, kararı VERİLMEDİ", None,
     "grup", None, None, None, None,
     "⚠ Bunlar sistemin PARÇASI DEĞİL — hepsi varsayılan KAPALI ve kapalı "
     "hâlde davranış `manevrada-iyi-terminalde-kotu` ile BİT BİT aynı "
     "(972 kombinasyonda fark 0.000e+00). Keşif için buradalar. Birini "
     "kalıcı yapmak §1'in beş adımını gerektirir.",
     None, None),

    ("TERM_HIZ_KORU", "TERM_HIZ_KORU", "D2 · terminalde FREN YOK", "G5",
     "bool", None, None, None, None,
     "Terminale girerken V_TERMINAL'e SIÇRAMAK yerine seyirdeki hız "
     "KİLİTLENİR. S1'in doğrudan çaresi: hız basamağı yoksa fren yok, "
     "fren yoksa burun kalkmıyor, burun kalkmayınca hedef kadrajda "
     "kalıyor.",
     "AÇIK — 16 uçuşta ölçüldü (n=6/kol, `duz`): kadraj dışı %73→%0 "
     "(p=0.039), cy tepesi 467→348 (p=0.026), terminal süresi 26.7→3.0 s "
     "(p=0.013), en yakın 1.75→1.20 m (p=0.030), |yatış| p90 son 3 s'de "
     "29.4°→4.2° (p=0.039), isabet 3/6→5/6. BEDELİ: 3 m içinde |dikey| "
     "0.21→1.06 m (daha hızlı gelindiği için dikeye oturma süresi yok).",
     "KAPALI = bugünkü davranış (fren var, kadraj kaybı var)."),

    ("TERM_SAF3B", "TERM_SAF3B", "D1 · saf takip 3B", "G5", "bool",
     None, None, None, None,
     "Yatay kanalın matematiğini dikeye de uygular: hız vektörü hedefe "
     "DÖNDÜRÜLÜR, büyüklüğü korunur. vz = −v_los·sin(elev), yatay = "
     "v_los·cos(elev). Eski yasa ayrı bir dikey ölçek ve tan() "
     "kullanıyordu.",
     "AÇIK = |v| sabit, yön hedefe. Kullanıcının 'yataydaki matematiğin "
     "aynısını dikey için de kullanamaz mıyız' sorusunun karşılığı. "
     "⚠ Dikey tavan 38.7°'de bağlar; üstünde D3 gerekir.",
     "KAPALI = eski ayrık dikey yasa."),

    ("TERM_YAVASLA", "TERM_YAVASLA", "D3 · kaçıracaksan YAVAŞLA", "G5",
     "bool", None, None, None, None,
     "⚠ YALNIZ D1 AÇIKKEN anlamlı. Gereken dikey hız tavanı aşıyorsa "
     "YATAY hızı kısar ki vektör hedefe bakabilsin: "
     "v_los = VZ_MAX_TERM / sin(|elev|), V_TERM_MIN tabanıyla.",
     "AÇIK — kullanıcının kendi fikri ('kaçıracak gibiysek hızı azaltıp "
     "dengeli yaklaşsak'). Doğrulandı: 45°'de D1 tek başına 38.7°'de "
     "takılıyor (ıska), D3 ile 45.0° (v=14.1). 55°'de 55.0° (v=12.2). "
     "Hızı ASLA artırmaz (B94), 38.7° altında hiç kısmaz (B95).",
     "KAPALI = dikey tavan aşılınca hedefin altından geçilir."),

    ("TERM_TAM_HIZ", "TERM_TAM_HIZ", "A1 · dikeyde tam hız ölçeği", "G5",
     "bool", None, None, None, None,
     "Terminalde dikey komutu KAPANMA hızı yerine v_los (tam hız) ile "
     "ölçekler. KAPANMA'yı devre dışı bırakmanın terminal karşılığı.",
     "AÇIK = dikey komut daha güçlü; yavaş kapanırken bile tırmanır.",
     "KAPALI = kapanma ölçeği kullanılır (bugünkü)."),

    ("TERM_ROLL", "TERM_ROLL", "T1c · terminalde roll telafisi", "G5",
     "bool", None, None, None, None,
     "⑤ DIKEY_ROLL seyir fazında roll'u telafi ediyor ama TERMİNALDE "
     "etmiyordu — tutarsızlık. Bu, terminalde de seviye çerçevesindeki "
     "gerçek yükselişi kullanır.",
     "AÇIK = tutarlı geometri. 4234 terminal karesinde telafili/telafisiz "
     "fark medyan 0.64°, p90 8.15°, maks 42.2°. 4 uçuşta ölçüldü ama "
     "kollar AYIRT EDİLEMEDİ (ara veri).",
     "KAPALI = bugünkü (terminalde telafisiz). roll=0'da zaten bit bit "
     "aynı (B76)."),

    # ═════════════════════════════════════════════════════════════════════
    ("G0", None, "⭐ TEK FAZ — terminal fazını tamamen kaldır", None,
     "grup", None, None, None, None,
     "KULLANICI FİKRİ (2026-08-18): 'terminal fazı diye bir şey neden var ki? "
     "Sistem iki fazdan oluşsa: GPS ve görsel. Görsel güdümün amacı her "
     "saniye mesafeyi kapatıp hedefi kadrajda ortalamak olmalı. Kapata "
     "kapata en sonunda çarpar zaten.' — Terminal mandalı atıldığı anda "
     "DOKUZ şey birden değişiyor ve ölçülen bütün bitiriş sorunları oradan "
     "çıkıyor. AÇIKKEN terminal dalı ÖLÜ KOD olur (birim testi B98: 14 "
     "terminal ayarı saçma değerlere çekildi, komut farkı 0.00e+00).",
     None, None),

    ("TEK_FAZ", "TEK_FAZ", "⭐ TEK FAZ AÇIK", "G0", "bool",
     None, None, None, None,
     "Terminal mandalı hiç atılmaz; görsel güdüm baştan sona TEK yasadır: "
     "yatayda hedefi ortala, dikeyde hedefi ortala (aynı matematik), "
     "mesafeyi kapat. Dikey kapı da anlamsızlaştığı için devre dışı kalır.",
     "AÇIK = tek yasa. Dokuz süreksizlik sıfıra iner; roll telafisi her "
     "karede olur (seyir açık / terminal kapalı tutarsızlığı biter); "
     "'6.4 m'de park et' ve 'ufkun 5° yukarısında dur' ofsetleri kalkar.",
     "KAPALI = bugünkü iki parçalı görsel faz (seyir + terminal)."),

    ("V_TEK", "V_TEK", "Tek faz hız tavanı", "G0", "sayi", "m/s",
     8.0, 30.0, 0.5,
     "Tek fazda hız burada oturur (PI hep 'kapat' dediği için tavana "
     "dayanır). Hedef 15.1 m/s uçuyor → kapanma = V_TEK − 15.1.",
     "Kapanma hızlanır, temas daha erken olur. ⚠ Dikey kanala oturma "
     "süresi azalır: D2 ölçümünde hızlanınca 3 m içinde |dikey| "
     "0.21 → 1.06 m olmuştu.",
     "Kapanma yavaşlar; dikey daha iyi oturur ama hedefle daha uzun süre "
     "yan yana uçulur. 15.1'in altına inersen hiç yaklaşamazsın."),

    ("TEK_BOYUT_REF", "TEK_BOYUT_REF", "Tek faz denge kutusu", "G0", "sayi",
     "px", 30.0, 400.0, 5.0,
     "PI'nın 'ulaşınca dur' kutusu. R = 160/boyut → 160 px ≈ 1.0 m, yani "
     "TEMAS. Taban sistemde bu 25 px (6.4 m) olduğu için araç orada PARK "
     "ediyordu; terminal fazı tam da bunu ezmek için vardı.",
     "Daha büyük = daha yakında dur = hep kapat (istenen).",
     "Küçültürsen park davranışı geri gelir — 160/değer metrede durur."),

    ("TEK_K_ELEV", "TEK_K_ELEV", "Dikey saf takip kazancı", "G0", "sayi", "",
     0.0, 2.5, 0.05,
     "Dikeyin yatayla AYNI matematiği: hız vektörünün YÖNÜ hedefe döner, "
     "BÜYÜKLÜĞÜ korunur. 1.0 = saf takip (yataydaki K_YAW da 1.0).",
     "Dikey hataya daha sert tepki; 1'in üstü aşım ve salınım üretir.",
     "Dikeyde tembelleşir; 0 = dikey komut hiç verilmez."),

    ("TEK_K_VZ_D", "TEK_K_VZ_D", "Dikey sönümleme (tek faz)", "G0", "sayi",
     "", 0.0, 3.0, 0.05,
     "Türev sönümlemesi: aracın KENDİ dikey hızı nişanın ötesine geçtiyse "
     "komut geri çekilir. Terminalin K_VZ_D'sini ödünç ALMAZ — kendi "
     "kazancı (§5.12: iki özellik aynı alanı paylaşmasın).",
     "Aşım azalır, hedefin üstünden geçme biter. Çok artarsa gürültüyü "
     "büyütür ve vz tavana çarpar.",
     "0 = sönümleme yok; dikey salınabilir."),

    ("TEK_YAVASLA", "TEK_YAVASLA", "Kaçıracaksan YAVAŞLA", "G0", "bool",
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

    ("TEK_V_MIN", "TEK_V_MIN", "Yavaşlama hız tabanı", "G0", "sayi", "m/s",
     6.0, 20.0, 0.5,
     "Yukarıdaki yavaşlamanın inebileceği en düşük hız.",
     "Dik hedefte az yavaşlar; geometri düzelmez ama hedefi kaçırmazsın.",
     "Daha çok yavaşlayıp daha dik girebilir. ⚠ 15.1'in altında hedef "
     "senden uzaklaşır — yalnız geometriyi düzeltmelik kısa süre için."),

    # ═════════════════════════════════════════════════════════════════════
    ("G6", None, "⑥ LEAD — hedefi öne alma", None,
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

    ("LEAD_ERKEN", "LEAD_ERKEN", "Lead'i erken uygula", "G6", "bool",
     None, None, None, None,
     "Lead terimini güdüm zincirinde daha erken devreye sokar.",
     "AÇIK = kestirim daha erken etkiler.",
     "KAPALI = bugünkü sıralama."),

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
    ("G7", None, "⑦ ALGI ve FAZ GEÇİŞİ", None,
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
    ("G8", None, "⑧ ARAÇ — ArduPilot parametreleri (CANLI YAZILIR)", None,
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
