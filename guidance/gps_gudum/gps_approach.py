"""
=============================================================
  GPS GÜDÜM — GÖLGE TAKİBİ (doğrudan konum bağlantısı)
=============================================================
MANTIK TEK CÜMLE:
    Avcı, hedefin GECIKME saniye ÖNCEKİ konumuna kilitlenir;
    ALT_OFFSET metre altında uçar, burnu hedefe dönüktür.

Yani avcı hedefin GÖLGESİDİR. Hedef nereye gittiyse avcı da tam oraya
gider — sadece biraz sonra. Aradaki mesafe kendiliğinden oluşur:

    mesafe ≈ hedef_hızı × GECIKME        (15 m/s × 0.7 s ≈ 10 m)

NEDEN BÖYLE (önceki sürümlerin hepsi buradan patlıyordu):
    Eskiden güdüm HIZ komutu üretiyordu: nişan noktasına bak, hata hesapla,
    kazançla çarp, tavana sığdır, feedforward ekle... Her manevrada bu
    zincirin bir halkası komutu kısıyordu ve drone duraklıyordu. Ölçümle
    bulunan beş ayrı arıza (tavan feedforward'ı kesiyor, head-on'da terimler
    birbirini götürüyor, dönüşte fren, köşeyi dolaşma, telemetri boşluğunda
    durma) hep AYNI kök nedenin belirtileriydi: hız komutunu biz üretiyorduk.

    Artık üretmiyoruz. Drone'a "şu NOKTAYA git, hedef de şu hızla gidiyor"
    diyoruz; hız profilini ArduCopter'ın 400 Hz'lik konum kontrolcüsü
    çıkarıyor. Bizim 20 Hz'lik döngümüzün frenleme/kazanç/tavan hesabı
    tamamen devre dışı — kısacak bir halka kalmadı.

Uçuş profili:
    Yatay : hedefin GECIKME saniye önceki (x, y) noktası
    Dikey : o noktanın irtifasının ALT_OFFSET altı (kamera 25° yukarı
            baktığı için hedef kadrajda kalır), güvenlik tabanı MIN_ALT
    Burun : daima hedefe dönük

Arayüz (değişmedi):
    run_gps_approach(conn, get_plane, get_iris, stop_event)
      get_plane() -> {x,y,z,vx,vy,vz,...}  (m, NED — iris çerçevesine
                     taşınmış hâlde; bkz. gcs_server _frame_off)
      get_iris()  -> {x,y,z,...}           (m, NED)
=============================================================
"""

import math
import os
import time

from guidance.ortak.common import clamp, normalize_angle, send_velocity


class Cfg:
    LOOP_HZ = 20.0

    # --- GÖLGE: TEK AYAR ---
    # Avcı, hedefin bu kadar saniye önceki konumunda durur.
    # Mesafe = hedef_hızı × GECIKME olduğundan hedef hızlanınca aralık
    # kendiliğinden açılır, yavaşlayınca kapanır — yani avcı hedefin
    # YOLUNDA kalır, mesafeyi ayrıca kovalamak gerekmez.
    GECIKME    = 0.7      # s; 15 m/s'de ≈ 10 m arkada
    ALT_OFFSET = 6.0      # m; hedefin kaç metre ALTINDA uçacak (kadraj için)
    MIN_ALT    = 15.0     # m; asgari güvenli irtifa (referans irtifa tabanı)

    # --- YERE ÇARPMA KORUMASI ---
    # KURTARMA_ALT bilerek MIN_ALT'ın ALTINDA: normal alçak uçuşta
    # tetiklenmemeli, yalnız gerçekten aşağı kaçarken devreye girmeli.
    KURTARMA_ALT  = 12.0  # m; bu irtifanın altı = koşulsuz kurtarma
    KURTARMA_SURE = 3.0   # s; çarpmaya bu kadar süre kaldıysa kurtarma
    GUVENLI_ALT   = 35.0  # m; kurtarmadan ancak bu irtifada çıkılır
    YAW_SAPMA_SINIR = 75.0  # °; burun komuttan bu kadar saparsa kontrol kaybı
    YAW_SAPMA_CIK   = 40.0  # °; kurtarmadan çıkış için sapma bunun altına inmeli
    YAW_MIN_MESAFE  = 4.0   # m; hedefe bundan yakınken yaw dondurulur

    # --- GÖLGEYE ÇEKME (manevrada geri kalmayı kapatan terim) ---
    KP_KONUM  = 0.8       # konum hatası (m) → ek hız (m/s)
    KP_DIKEY  = 1.0       # irtifa hatası (m) → dikey hız (m/s)
    KD_KONUM  = 2.0       # sönümleme: yaklaşma hızını keser (overshoot önler)
    V_TAVAN   = 20.0      # m/s; KOMUTUN TOPLAM tavanı. Quad ANGLE_MAX=55°
                          # ile ~19 m/s yapabiliyor; bunun çok üstünde komut
                          # vermek drone'u savurup ÇAKTIRDI (ölçüldü).

    # --- SINIRLAR (yalnız emniyet; normal uçuşta devreye girmez) ---
    V_MAX   = 19.0        # m/s; yakalama fazındaki hız komutunun tavanı
    # ★ TIRMANMA ve ALÇALMA tavanları AYRI olmak zorunda.
    # VZ_YUKARI: 3.0 idi ve darboğazdı — araç WP_SPD_UP=8 m/s tırmanabilirken
    #   güdüm üçte birinde tutuyordu (100 m fark 33 saniye sürüyordu).
    # VZ_ASAGI : 8 m/s ALÇALMA drone'u ÇAKTIRDI. Hedef 65 m'den 29 m'ye
    #   daldığında avcı peşinden indi; hızlı alçalışta motorlar düşük itkiye
    #   iner ve yaw'ı tutacak diferansiyel itki kalmaz. Kayıt: güdüm sabit
    #   59° yaw komut ederken aracın yaw'ı 77° → 134° → -116° savruldu
    #   (roll/pitch küçük kaldı, yani savrulma değil YAW OTORİTESİ KAYBI),
    #   ardından yere çakıldı. Alçalma yaw otoritesini koruyacak kadar
    #   yavaş tutulur; hedefi kaybetmek çakılmaktan iyidir.
    VZ_YUKARI = 8.0       # m/s; tırmanma tavanı
    VZ_ASAGI  = 3.5       # m/s; alçalma tavanı (yaw otoritesi için)
    VZ_MAX  = 8.0         # m/s; kurtarma tırmanışında kullanılır
    YAKALAMA     = 60.0   # m; konum moduna DÖNÜŞ eşiği
    YAKALAMA_CIK = 80.0   # m; hız moduna ÇIKIŞ eşiği (histerezis)
    GECMIS_S = 8.0        # s; konum geçmişi bu kadar geriye tutulur

    # --- DURUM ---
    KILIT_MESAFE = 40.0   # m; altında "KİLİT" (görsel faz devralabilir)
    DUR_S = 10.0          # s; telemetri bu kadar susarsa havada tut


# Arayüz/supervisor okur (salt gözlem)
status = {"durum": "BEKLE", "d_h": None, "handoff": False, "vcap": None}


def run_gps_approach(conn, get_plane, get_iris, stop_event):
    periyot = 1.0 / Cfg.LOOP_HZ

    # UÇUŞ KAYDI (AVCI_GUDUM_LOG=1): her döngüde (20 Hz) tam durum CSV'ye
    # yazılır. 5 saniyede bir örnek alarak manevrayı anlamak imkânsızdı —
    # manevra ~6 sn sürüyor, elde 1-2 nokta kalıyordu. Kök nedeni ancak
    # kare kare veri gösteriyor.
    kayit = None
    if os.environ.get("AVCI_GUDUM_LOG") == "1":
        yol = os.environ.get("AVCI_GUDUM_LOG_YOL", "/tmp/gudum_kayit.csv")
        kayit = open(yol, "w", buffering=1)
        kayit.write("t,tx,ty,tz,tvx,tvy,gx,gy,gz,gvx,gvy,"
                    "ix,iy,iz,ivx,ivy,iroll,ipitch,iyaw,"
                    "cmd_yaw,d_h,d_golge,mod\n")
        print(f"[GPS-GUDUM] ucus kaydi: {yol}")

    cmd_yaw = None
    gecmis = []                 # [(t, x, y, z, vx, vy, vz)] hedefin izi
    son_taze = None             # son TAZE telemetri anı
    son_konum = None
    vel_modu = True             # başlangıçta uzak: hız komutu
    kurtarma = False            # yere çarpma koruması etkin mi
    sayac = 0

    print("=" * 58)
    print("[GPS-GUDUM] GÖLGE TAKİBİ aktif (doğrudan konum bağlantısı)")
    print(f"[GPS-GUDUM] avcı = hedefin {Cfg.GECIKME:.1f} sn önceki konumu, "
          f"{Cfg.ALT_OFFSET:.0f} m altı (taban {Cfg.MIN_ALT:.0f} m)")
    print("=" * 58)

    while not stop_event.is_set():
        simdi = time.monotonic()

        iris = get_iris()
        plane = get_plane()

        ix, iy, iz = iris["x"], iris["y"], iris["z"]
        tx, ty, tz = plane["x"], plane["y"], plane["z"]
        tvx = float(plane.get("vx", 0.0) or 0.0)
        tvy = float(plane.get("vy", 0.0) or 0.0)
        tvz = float(plane.get("vz", 0.0) or 0.0)
        hedef_hiz = math.hypot(tvx, tvy)

        # ── 1) TAZE PAKET Mİ? ──
        # Geçmişe yalnız GERÇEK ölçüm yazılır. Telemetri 4 Hz, döngü 20 Hz:
        # her döngüde yazsaydık aynı konumdan 5 kopya birikir ve "0.7 sn
        # öncesi" araması bozulurdu.
        konum = (tx, ty, tz)
        taze = konum != son_konum
        if taze:
            son_konum = konum
            son_taze = simdi
            gecmis.append((simdi, tx, ty, tz, tvx, tvy, tvz))
            # Geçmişi buda (bellek sabit kalsın)
            sinir = simdi - Cfg.GECMIS_S
            while len(gecmis) > 2 and gecmis[0][0] < sinir:
                gecmis.pop(0)

        yas = (simdi - son_taze) if son_taze is not None else 999.0
        if yas > Cfg.DUR_S:
            # Telemetri gerçekten kesildi: havada tut (hız 0), konum komutu
            # gönderme — eski bir noktaya kilitlenip oraya gitmesin.
            send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)
            status.update(durum="TELEMETRI YOK", d_h=None)
            _uyu(simdi, periyot)
            continue

        # ══ YERE ÇARPMA KORUMASI (her şeyin önünde) ══
        # Avcı kontrolü kaybedip düşmeye başlarsa takip ANLAMSIZDIR; tek iş
        # hayatta kalmaktır. Ölçümde görüldü: hedef hızla alçalırken avcı da
        # peşinden daldı, aynı anda 20 m/s yatay komut sürüyordu, quad
        # aşırı yüklenip savruldu (roll -49° → pitch +43° → -54° → yere).
        #
        # Koruma iki şeye birden bakar:
        #   1) mutlak taban  — irtifa KURTARMA_ALT'ın altına indiyse
        #   2) çarpma süresi — irtifa / alçalma hızı; yüksekte bile hızlı
        #      düşüyorsa (ör. 8 m/s ile 24 m'de 3 sn kalmıştır) devreye girer
        # Devreye girince YATAY KOMUT SIFIRLANIR (tüm itki dikeye gider) ve
        # tam gaz tırmanılır. Çıkış GUVENLI_ALT ile histerezislidir, yoksa
        # eşiğin başında açılıp kapanıp salınım üretirdi.
        irtifa = -iz
        dusus = float(iris.get("vz", 0.0) or 0.0)          # NED: + aşağı
        carpma_s = (irtifa / dusus) if dusus > 0.5 else 999.0
        # Aracın burnu komut edilen yönden çok saptıysa kontrol kaybı var
        # demektir (ölçüldü: komut 59° sabitken araç 134°/-116° savruldu).
        # Bu durumda da kurtarmaya geçilir; tam gaz tırmanış throttle'ı
        # yükseltir ve yaw otoritesini geri kazandırır.
        yaw_sapma = 0.0
        if cmd_yaw is not None:
            yaw_sapma = abs(normalize_angle(
                math.radians(float(iris.get("yaw", 0.0) or 0.0)) - cmd_yaw))
        if kurtarma:
            kurtarma = (irtifa < Cfg.GUVENLI_ALT
                        or yaw_sapma > math.radians(Cfg.YAW_SAPMA_CIK))
        else:
            kurtarma = (irtifa < Cfg.KURTARMA_ALT
                        or carpma_s < Cfg.KURTARMA_SURE
                        or yaw_sapma > math.radians(Cfg.YAW_SAPMA_SINIR))
        if kurtarma:
            send_velocity(conn, 0.0, 0.0, -Cfg.VZ_MAX, cmd_yaw or 0.0)
            status.update(durum="KURTARMA",
                          d_h=round(math.hypot(tx - ix, ty - iy), 1),
                          handoff=False, vcap=round(Cfg.V_TAVAN, 1))
            if sayac % int(Cfg.LOOP_HZ) == 0:
                sebep = ("irtifa" if irtifa < Cfg.KURTARMA_ALT else
                         "carpma_suresi" if carpma_s < Cfg.KURTARMA_SURE else
                         "YAW_KONTROL_KAYBI")
                print(f"[GPS-GUDUM] !!! KURTARMA ({sebep}) !!! "
                      f"irtifa={irtifa:.1f}m dusus={dusus:+.1f}m/s "
                      f"carpmaya={carpma_s:.1f}s yaw_sapma={math.degrees(yaw_sapma):.0f}° "
                      f"-> yatay kesildi, tam gaz tirmanis")
            sayac += 1
            _uyu(simdi, periyot)
            continue

        if len(gecmis) < 2:
            # Henüz iz yok: hedefin kendisini hedefle, iz birikince gölgeye oturur
            gx, gy, gz = tx, ty, tz
            gvx, gvy, gvz = tvx, tvy, tvz
        else:
            gx, gy, gz, gvx, gvy, gvz = _gecikmeli_nokta(
                gecmis, simdi - Cfg.GECIKME)

        # ── 2) DİKEY: hedefin ANLIK irtifasının ALT_OFFSET altı ──
        # NED'de z aşağı pozitiftir → "altında olmak" = z BÜYÜK.
        # ★ Yatayda gölge (0.7 sn gerisi) doğru davranış — "aynı yoldan git"
        # demek. Ama DİKEYDE gölge yanlış: hedef tırmanmaya başladığında
        # gölgenin irtifası 0.7 sn boyunca eski değerde kalıyor, avcı
        # tırmanmaya geç başlıyordu. İrtifada istediğimiz "aynı yoldan
        # gitmek" değil, "hep 6 m altında olmak" — o yüzden hedefin ANLIK
        # irtifası kullanılır.
        hedef_irtifa = -tz
        ref_irtifa = max(hedef_irtifa - Cfg.ALT_OFFSET, Cfg.MIN_ALT)

        # ── 3) KOMUT HIZI = hedefin hızı + gölgeye çekme ──
        #
        # ★★ MANEVRADAKİ GERİ KALMANIN KÖK NEDENİ BURASIYDI ★★
        # Eskiden yalnız hedefin hızı (feedforward) gönderiliyor, konum
        # hatasını kapatma işi ArduCopter'a bırakılıyordu. 20 Hz uçuş kaydı
        # (5 manevra, hepsi birebir aynı) bunun ÇALIŞMADIĞINI gösterdi:
        #
        #   t=100.5  gölgeye 2.6 m   avcı 15.1 m/s  roll 13°  yön farkı 80°
        #   t=105.0  gölgeye 24.4 m  avcı 15.0 m/s  roll  6°  yön farkı 78°
        #   t=109.0  gölgeye 39.3 m  avcı 15.0 m/s  roll 10°  yön farkı 52°
        #
        # Avcı gölgeye 80° YAN açıyla, gölgeyle TAM AYNI hızda uçuyordu:
        # yani paralel gidiyor, aradaki 39 m'yi kapatmaya hiç çalışmıyordu.
        # Roll 18°'yi hiç geçmedi (ANGLE_MAX 55°) — 18° tam olarak dairede
        # dönmeye yeten ivmedir, hatayı kapatmak için fazlası yok. Dönüş
        # biter bitmez 22.8 m/s'ye çıkıp kapatıyordu; demek ki araçta güç
        # vardı, komutta düzeltme yoktu.
        #
        # Çare: konum hatasını BİZ kapatıyoruz. Hedefin hızının üstüne
        # gölgeye doğru bir çekme bileşeni bindirilir; ArduCopter bunu
        # "istenen hız" olarak alır ve gereken kadar yatar.
        # ★ SÖNÜMLEME ŞART (ölçüldü): yalnız P terimiyle (çekme = Kp·hata)
        # denendiğinde avcı gölgeye 25 m/s ile dalıp ÜSTÜNDEN GEÇTİ, mesafe
        # 12 m → 107 m açıldı, sonra geri döndü — takip limit döngüsüne girdi
        # (kayıt: d_h 12 → 107 → 58 → 134 m). Fren terimi olmadan hedefte
        # duramıyor. Bu yüzden D terimi eklendi: avcının gölgeye YAKLAŞMA
        # hızı çekmeden düşülür, yani hedefe yaklaştıkça kendini frenler.
        hx = gx - ix
        hy = gy - iy
        hata = math.hypot(hx, hy)
        if hata > 1e-6:
            ux, uy = hx / hata, hy / hata
            # Gölgeye göre bağıl yaklaşma hızı (+ = kapanıyor).
            # Feedforward (gvx,gvy) hâlâ ham hâlde — çekme henüz eklenmedi.
            yaklasma = ((iris.get("vx", 0.0) - gvx) * ux
                        + (iris.get("vy", 0.0) - gvy) * uy)

            # ★★ TAVAN ŞART — TAVANSIZ SÜRÜM DRONE'U ÇAKTI ★★
            # Tavansız denendi: komut 27-29 m/s'e çıktı. Quad'ın ANGLE_MAX=55°
            # ile fiziksel tavanı ~19 m/s; ulaşamayacağı komutu kovalarken
            # roll kontrolden çıktı ve takla attı (kayıt: roll 24° → -51° →
            # 95° → -115° → 120°, irtifa 59 m → 0 m, "Crash: AngErr=180").
            #
            # Ama düz ölçekleme de YANLIŞ: tüm vektörü küçültmek feedforward'ı
            # da keser, drone hedeften yavaş kalır (bu, daha önce ölçülen
            # "manevrada geri kalma"nın ta kendisiydi).
            #
            # Doğrusu BÜTÇE: hedefin hızı dokunulmaz, çekmeye yalnız tavandan
            # ARTAN pay verilir. |ff + c·u| = V_TAVAN denkleminin çözümü.
            # 15.1 m/s feedforward + 20 m/s tavan → dik yönde 13.1 m/s çekme
            # kalır; 40 m yanal hatayı ~3 sn'de kapatmaya yeter.
            ff = math.hypot(gvx, gvy)
            b = gvx * ux + gvy * uy
            disk = b * b - (ff * ff - Cfg.V_TAVAN * Cfg.V_TAVAN)
            butce = (-b + math.sqrt(disk)) if disk > 0.0 else 0.0
            # Üst sınır YALNIZ bütçe: sabit bir CEKME_MAX, hedef üzerimize
            # gelirken (feedforward geri iterken) gereken büyük çekmeyi
            # kesip komutu ters yöne çeviriyordu. Bütçe zaten toplam
            # komutu V_TAVAN'da tutuyor.
            cekme = clamp(Cfg.KP_KONUM * hata - Cfg.KD_KONUM * yaklasma,
                          0.0, max(butce, 0.0))
            gvx += ux * cekme
            gvy += uy * cekme

        d_h_ham = math.hypot(tx - ix, ty - iy)

        # ── 4) YAW: burun HEDEFE (gölgeye değil) ──
        # Kamera hedefi görsün diye burun daima gerçek hedefe bakar.
        # Burun DAİMA hedefte (kamera hedefi kadrajda tutsun).
        # A/B ölçüldü: burnu gidiş yönüne bağlamak manevrada geri kalmayı
        # değiştirmedi (41.7 m vs 40.8 m) — yaw suçlu değildi, o yüzden
        # kullanıcının istediği "burun hedefte" davranışı korunuyor.
        #
        # ★ ÇOK YAKINDA YAW DONDURULUR: hedefin tam üstündeyken (ölçümde
        # d_h 0.2 m'ye kadar indi) "hedefe bakan açı" tanımsızlaşır, en
        # ufak konum gürültüsü burnu 180° çevirir. Kayıtta yaw sapması
        # 57°'ye fırladı. O bölgede burun son bilinen yönde tutulur.
        if d_h_ham > Cfg.YAW_MIN_MESAFE or cmd_yaw is None:
            cmd_yaw = math.atan2(ty - iy, tx - ix)

        # ── 5) GÖNDER ──
        # İKİ FAZ — hangi komutun hangi mesafede iyi olduğu ÖLÇÜLDÜ:
        #
        # YAKIN (gölge ≤ YAKALAMA): konum + feedforward hız. Ölçümde
        #   d_h ≈ 11 m'de kilitlendi, irtifa farkı tam 6.0 m, avcı hızı
        #   hedefe birebir eşleşti (15.1 / 15.2 m/s), dönüşte 21 m/s'ye
        #   hızlanıp toparladı. Takip için doğru komut bu.
        #
        # UZAK (gölge > YAKALAMA): saf hız, tam gaz. Sebebi ölçüm: konum
        #   komutuyla 200 m mesafede avcı yalnız 6.2 m/s gidiyordu —
        #   ArduCopter uzak bir konum hedefine yumuşak profil uygular,
        #   yakalama dakikalarca sürerdi. Bu fazda amaç zaten tek şey:
        #   aradaki mesafeyi en hızlı şekilde kapatmak.
        # ★★ KONUM KOMUTU KALDIRILDI — ÖLÇÜMLE KANITLANDI ★★
        # Konum+hız (posvel) komutu gönderildiğinde ArduCopter, verdiğimiz
        # konumu kendi jerk-limitli profiliyle "yumuşatıyor" ve hız komutunu
        # boğuyordu. 20 Hz kayıt, manevranın ortasında şunu gösterdi:
        #
        #   d_gölge  8.0 m   KOMUT 20.0 m/s   GERÇEK 14.6 m/s   roll 13°
        #   d_gölge 28.9 m   KOMUT 20.0 m/s   GERÇEK 14.3 m/s   roll  9°
        #   d_gölge 38.4 m   KOMUT 20.0 m/s   GERÇEK 14.3 m/s   roll 13°
        #
        # Yani güdüm 20 m/s istiyor, araç 14.3 m/s'de takılıyor ve 13°'den
        # fazla yatmıyor. AYNI araç, yakalama fazında (saf hız komutu)
        # 19 m/s yapıyordu — güçsüzlük değil, komut tipi sorunuydu.
        # Artık her mesafede SAF HIZ komutu gidiyor: hızı zaten biz doğru
        # hesaplıyoruz (hedefin hızı + gölgeye çekme + bütçe tavanı), araya
        # ikinci bir yorumlayıcı katman girmiyor.
        d_golge = math.hypot(gx - ix, gy - iy)
        # ★ DİKEY FEEDFORWARD (eksikti): hedefin kendi tırmanma hızı komuta
        # eklenir. Eskiden yalnız P terimi vardı (hata × Kp); bu, hedef
        # sürekli tırmanırken KALICI hataya yol açar — kararlı durumda
        # hata = hedefin tırmanma hızı / Kp kadar geride kalınır. Yatayda
        # hedefin hızını eşliyorduk, dikeyde eşlemiyorduk.
        # tvz NED'dir (aşağı +); hedef tırmanıyorsa negatiftir.
        vz_ned = clamp(tvz - Cfg.KP_DIKEY * (ref_irtifa - (-iz)),
                       -Cfg.VZ_YUKARI, Cfg.VZ_ASAGI)
        send_velocity(conn, gvx, gvy, vz_ned, cmd_yaw)

        # ── Durum ──
        d_h = math.hypot(tx - ix, ty - iy)

        if kayit is not None:
            kayit.write(
                f"{simdi:.3f},{tx:.2f},{ty:.2f},{tz:.2f},{tvx:.2f},{tvy:.2f},"
                f"{gx:.2f},{gy:.2f},{gz:.2f},{gvx:.2f},{gvy:.2f},"
                f"{ix:.2f},{iy:.2f},{iz:.2f},"
                f"{iris.get('vx',0.0):.2f},{iris.get('vy',0.0):.2f},"
                f"{iris.get('roll',0.0):.1f},{iris.get('pitch',0.0):.1f},"
                f"{iris.get('yaw',0.0):.1f},"
                f"{math.degrees(cmd_yaw):.1f},{d_h:.2f},{d_golge:.2f},"
                f"{'VEL' if d_golge > Cfg.YAKALAMA else 'POS'}\n")
        kilit = d_h < Cfg.KILIT_MESAFE
        status.update(durum="KILIT" if kilit else "TAKIP",
                      d_h=round(d_h, 1), handoff=kilit,
                      vcap=round(Cfg.V_MAX, 1))

        sayac += 1
        if sayac % int(Cfg.LOOP_HZ * 3) == 0:
            iris_irtifa = -iz
            print(f"[GPS-GUDUM] {'KILIT' if kilit else 'TAKIP'} "
                  f"d_h={d_h:.1f}m d_golge={d_golge:.1f}m "
                  f"irtifa={iris_irtifa:.1f}/{ref_irtifa:.1f}m "
                  f"ff=({gvx:+.1f},{gvy:+.1f}) hedef_v={hedef_hiz:.1f} "
                  f"iz={len(gecmis)}")

        _uyu(simdi, periyot)

    send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)
    if kayit is not None:
        kayit.close()
    status.update(durum="DURDU")
    print("[GPS-GUDUM] durduruldu.")


def _gecikmeli_nokta(gecmis, istenen_t):
    """Hedefin `istenen_t` anındaki konum+hızını geçmişten çıkarır.

    İki komşu ölçüm arasında DOĞRUSAL ARA DEĞER yapılır: telemetri 4 Hz
    geldiği için ham örnek seçmek 0.25 sn'lik basamaklar üretir ve konum
    hedefi zıplar. Ara değerle gölge noktası pürüzsüz ilerler.
    """
    if istenen_t <= gecmis[0][0]:
        return gecmis[0][1:]                    # o kadar geriye iz yok
    if istenen_t >= gecmis[-1][0]:
        return gecmis[-1][1:]                   # henüz gecikme dolmadı

    for k in range(len(gecmis) - 1, 0, -1):
        t1 = gecmis[k][0]
        t0 = gecmis[k - 1][0]
        if t0 <= istenen_t <= t1:
            aralik = t1 - t0
            o = (istenen_t - t0) / aralik if aralik > 1e-9 else 0.0
            a = gecmis[k - 1]
            b = gecmis[k]
            return tuple(a[i] + (b[i] - a[i]) * o for i in range(1, 7))
    return gecmis[-1][1:]


def _uyu(baslangic, periyot):
    gecen = time.monotonic() - baslangic
    if gecen < periyot:
        time.sleep(periyot - gecen)
