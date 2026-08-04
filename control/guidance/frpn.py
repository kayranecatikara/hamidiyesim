"""
frpn.py — FRPN güdüm yasası, HIZ FORMU. Saf matematik; MAVLink/telemetri yok.

Kaynak: Pliska, Vrba, Báča, Saska — "Towards Safe Mid-Air Drone Interception",
IEEE RA-L 9(10):8810-8817, 2024 (arXiv 2405.13542). Makalenin denklem (19)'u
İVME komutu üretir:

    a_cmd = G((1-W)·(Δp + Δv·t_go)/t_go² + W·Δp)

BU MODÜL O DENKLEMİ AYNEN UYGULAMAZ — bilerek. İki ölçülmüş sebep:

1) KATSAYI ÖLÇEĞİ. Makalenin katsayıları (G=19.7, W=5.1e-2) onların sahasına
   (Δp ~ 10-30 m) göre grid-search ile bulunmuş. Bizim loglarımızda (Δp ~ 100-1000 m)
   aynı yasa medyan 163-219 m/s² komut ediyor; aracın bütçesi ~5-10 m/s².
   Ölçüldü (2026-08-04, logs/gps_guidance_20260804_{122519,145230}.csv):
       takip terimi  G·W·Δp        : medyan 150.7 / 205.2 m/s²
       kesme terimi  G(1-W)ZEM/t_go²: medyan   8.3 /   3.2 m/s²
       karelerin %100'ü 4 m/s² sınırını aşıyor
   Yani doygunluk altında geriye yalnız "hedefe doğru tam gaz" kalır — bu, tam
   olarak kaçındığımız saf takibin kendisi. Katsayıyı kopyalamak hiçbir şey
   değiştirmezdi.

2) ARAYÜZ. Makalenin aracı ivme komutuyla sürülüyor; bizimki hız komutuyla
   (send_velocity → SET_POSITION_TARGET_LOCAL_NED). İvmeyi sayısal integralle
   hıza çevirmek gecikme + sürüklenme getirir.

TÜRETME (hız formu). Hız komutlu bir araçta ZEM'i sıfırlamak ANLIKTIR: kendi
hızımızı δ kadar değiştirirsek Δv → Δv − δ ve ZEM → ZEM − δ·t_go. ZEM_yeni = 0
için δ = ZEM/t_go. Bunun kapalı çözümü çarpışma üçgenidir:

    v_cmd = v_hedef + V_c·û          →  ZEM ≡ 0  (manevrasız hedef için ÖZDEŞ)

    kanıt: Δv = −V_c·û,  t_go = ‖Δp‖/V_c,  ZEM = ‖Δp‖û − V_c·û·‖Δp‖/V_c = 0

Hedef manevra yapınca üçgen bozulur ve artık bir ZEM doğar; FRPN'in harman
yapısını orada koruyoruz. Uygulanan yasa:

    v_cmd = v_hedef + V_c·û + K_ZEM·ZEM/t_go
            └─ 1 ─┘  └─ 2 ─┘  └────── 3 ──────┘

    1) hedef hızı ileri beslemesi — istasyonda kararlı asılı kalmayı sağlar
    2) kapanma — uzakta tavanda, yaklaşınca menzille orantılı azalıp yumuşak
       duruş verir. Makalenin doyup kırpılan W·Δp teriminin DOĞAL SINIRLI hali.
    3) manevra düzeltmesi — asıl PN katkısı. Manevrasız hedefte 0'a gider.

Neden bu form bizim için üstün:
  • Ölçekten bağımsız: V_c zaten m/s ve tavanlı; 200 m/s² gibi anlamsız komut yok.
  • Aracın arayüzüne birebir uyuyor; integral yok.
  • t_go → ∞ dejenerasyonu zararsız: istasyonda Δv→0 iken 3. terim söner,
    geriye saf konum tutma kalır — istenen davranış.

Kullanım (hepsi saf fonksiyon, yan etkisiz):
    sanal_hedef(p_hedef, v_hedef, p_drone, cfg) -> (p_sanal, tanı)
    komut(dp, dv, v_hedef, cfg)                 -> {v_cmd, t_go, zem, ...}
"""

import math

# NED: +z aşağı. Tüm 3'lüler (kuzey, doğu, aşağı).


class Cfg:
    # ── KAPANMA (terim 2) ──
    # V_c = clamp(K_C · ‖Δp‖, V_C_MIN, V_C_MAX)
    # K_C menzille orantılı yaklaşma kazancı: 100 m'de K_C=0.25 → 25 m/s ister
    # (tavana dayanır), 20 m'de 5 m/s, 4 m'de 1 m/s → kendiliğinden yumuşak duruş.
    # Katsayılar F3'te (tools/frpn_replay.py) grid-search ile sabitlenecek;
    # buradakiler MAKUL BAŞLANGIÇ, ayarlanmış değer DEĞİL.
    # ── AYARLANDI (F3 grid-search, tools/frpn_replay.py --tara) ──
    # Amaç fonksiyonu: 4 yörünge × {20 Hz, 1 Hz telemetri} koşullarında
    # 15 m altında geçirilen TOPLAM süre (görsel fazın devralabileceği pencere).
    # 144 kombinasyon tarandı; kazanan K_C=0.40, V_C_MAX=20.0, K_ZEM=0.60
    # (265.3 s), başlangıç değerlerine (0.25/22/1.0) göre belirgin üstün.
    K_C = 0.40
    V_C_MIN = 1.0            # m/s; istasyona oturunca bile hafif baskı kalsın
    V_C_MAX = 20.0           # m/s; ölçülen araç zarfı: düz 18, dairede 14.4 m/s

    # ── MANEVRA DÜZELTMESİ (terim 3) — "lead" terimi ──
    # ⚠ 1.0 (tam düzeltme) EN İYİ DEĞİL, ölçüldü. Denge analizi bunu öngörüyordu:
    # v_cmd = v_hedef + c·û + K·Δv yasasında, komut tam izlenirse
    #     Δv_yeni⊥ = −K·Δv_eski⊥
    # yani K=1 dik bileşeni SÖNDÜRMEZ, işaretini çevirip salındırır; K<1 söndürür.
    # Gerçek araçta gecikme ve ivme sınırı bunu yumuşatıyor ama eğilim aynı.
    # Mevcut gps_guidance'ın KD_H=0.20'si ise fazla sönümlü — kesme rotasına
    # geç oturuyor. Ayarlanan 0.60 ikisinin arasında.
    K_ZEM = 0.60
    V_ZEM_MAX = 12.0         # m/s; düzeltmenin kendi tavanı (ani ZEM sıçraması
                             # tüm komutu ele geçirmesin)

    # ── SAYISAL KORUMA ──
    DV_MIN = 0.3             # m/s; ‖Δv‖ bunun altındaysa t_go = T_GO_MAX
    T_GO_MIN = 0.5           # s; son metrelerde ZEM/t_go patlamasın
    T_GO_MAX = 60.0          # s
    DP_MIN = 0.1             # m; û tanımsız olmasın

    # ── SANAL HEDEF (kadraj istasyonu) ──
    # Sabit METRE değil sabit AÇI: menzil RANGE_SET altına inince ofset onunla
    # birlikte küçülür, böylece LOS yükselişi her menzilde ISTASYON_ELEV_DEG
    # kalır ve hedef kadrajın tepesinden taşmaz. (Bu, eski modülden korunan tek
    # fikir; 2026-08-01 dikey ıska düzeltmesinin gerekçesi orada.)
    RANGE_SET = 11.0         # m; slant menzil setpoint
    ISTASYON_ELEV_DEG = 15.0
    TRACK_MIN_SPD = 3.0      # m/s; altında hız yönü güvenilmez → LOS gerisi kullan
    LOOKUP_MIN_ALT = 8.0     # m; istasyon bu irtifanın altına inmesin (yer koruması)

    # ── İSTASYONA MENZİLE BAĞLI GEÇİŞ (2026-08-04, mimari revizyon) ──
    # ESKİ DAVRANIŞ: istasyon 1000 m'den itibaren hedefleniyordu. Yani drone,
    # daha 1 km uzaktayken "hedefin 10.6 m gerisindeki nokta"ya gidiyordu →
    # tüm yaklaşma boyunca kuyruk kovalama geometrisine mahkûmdu. 63 dakikalık
    # uçuş (132820) bunun manevra yapan hedefte hiç yakınsamadığını gösterdi.
    #
    # YENİ: istasyon geometrisi yalnız DEVİR ANINDA gerekli, yaklaşma boyunca
    # değil. Uzakta doğrudan hedefe nişan al (saf kesme, en hızlı kapanma),
    # yaklaştıkça nişan noktası yumuşakça istasyona kaysın.
    #
    #     menzil ≥ GECIS_BASLA  → w = 0  : ofset yok, saf kesme
    #     menzil ≤ GECIS_TAM    → w = 1  : tam istasyon (devir geometrisi)
    #     arası                 → doğrusal
    #
    # İKİ BİLEŞEN BİRLİKTE ÖLÇEKLENİR: oran, dolayısıyla hedefin kadrajdaki
    # yüksekliği (atan(d_alt/d_arka)) menzilden bağımsız sabit kalır. Böylece
    # geçiş boyunca hedefin kadraj içindeki yeri kaymaz.
    #
    # ⚠ KADRAJ KISITI SANILDIĞI KADAR DAR DEĞİL (2026-08-04 düzeltmesi):
    # Kamera YUKARI bakıyor, aşağı değil. Kanıt — guidance_core.kamera_to_govde
    # ile boresight gövde FRD'de [0.906, 0, -0.423]; z NEGATİF = YUKARI, yani
    # yükseliş +25°. Görülebilir bant: ufkun 30.25° ALTINDAN 80.25° ÜSTÜNE
    # (dikey yarı-açı 55.25°). İstasyonun 15°'lik yükselişi bandın ortasında,
    # eksenin yalnız 10° altında.
    # Bunun sonucu: kuyruk ofsetini kısıp yükselişi büyütmek (örn. 37.5°) kadraj
    # açısından SORUN DEĞİL — hâlâ bandın rahat içinde. Daha önce bu tersine
    # yazılmıştı (kamera aşağı bakıyor sanılmıştı); o gerekçe geçersiz.
    # Yani arka/alt oranı F3'te SERBEST bir ayar değişkeni: kadraj değil,
    # terminal fazın devraldığı geometri ve gökyüzü arka planı belirleyecek.
    #
    # Alt ofsetin kendi gerekçesi ayrıca güçlü ve ölçülmüş: drone hedefin
    # altında kalınca hedef GÖKYÜZÜ arka planında görünür. Tespit modelimiz
    # zemin/pist karelerinin %55'inde hayalet kutu üretiyor (2026-08-04 ölçümü,
    # ortalama güven 0.75 — eşikle elenemez). Gökyüzü arka planı tercih değil,
    # dedektörün ölçülmüş zaafına karşı zorunluluk.
    #
    # Değerler F3'te grid-search ile ayarlanacak; bunlar makul başlangıç.
    GECIS_BASLA = 120.0      # m; bu menzilin üstünde ofset YOK (saf kesme)
    GECIS_TAM = 30.0         # m; bu menzilin altında ofset TAM (devir geometrisi)


# ─────────────────────────── vektör yardımcıları ───────────────────────────
# (numpy'a bağımlılık yok: bu modül birim testte ve gömülü döngüde aynı hızda
#  çalışsın, içe aktarma maliyeti olmasın.)

def _ekle(a, b):      return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def _cikar(a, b):     return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def _olcek(a, s):     return (a[0] * s, a[1] * s, a[2] * s)
def _norm(a):         return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _birim(a, esik=1e-9):
    n = _norm(a)
    if n < esik:
        return (0.0, 0.0, 0.0), 0.0
    return (a[0] / n, a[1] / n, a[2] / n), n


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ─────────────────────────────── sanal hedef ───────────────────────────────

def sanal_hedef(p_hedef, v_hedef, p_drone, cfg=Cfg):
    """Kadraj istasyonu: hedefin hız yönünün gerisinde + altında bir nokta.

    FRPN çarpışmaya güder (Δp → 0). Bizim GPS fazımızın işi çarpışmak değil,
    görsel fazın devralabileceği bir konuma gelmek. Bu yüzden yasa gerçek
    hedefe değil, hedefle birlikte hareket eden bu SANAL hedefe uygulanır.

    Dönüş: (p_sanal, tani) — tani içinde arka/alt ofset ve kullanılan yön.
    """
    ist = math.radians(cfg.ISTASYON_ELEV_DEG)
    fark = _cikar(p_hedef, p_drone)
    menzil = _norm(fark)

    # Etkin standoff: yakınlaşınca ofset menzille birlikte küçülür → yükseliş
    # açısı sabit kalır. Uzakta (menzil ≥ RANGE_SET) davranış tam ofsetli.
    # Menzile bağlı geçiş ağırlığı: uzakta 0 (saf kesme), yakında 1 (tam
    # istasyon). İki ofset de AYNI w ile ölçeklenir → oran, dolayısıyla hedefin
    # kadrajdaki yüksekliği korunur. Bkz. Cfg.GECIS_BASLA / GECIS_TAM.
    if menzil >= cfg.GECIS_BASLA:
        w = 0.0
    elif menzil <= cfg.GECIS_TAM:
        w = 1.0
    else:
        w = (cfg.GECIS_BASLA - menzil) / (cfg.GECIS_BASLA - cfg.GECIS_TAM)

    r_eff = min(menzil, cfg.RANGE_SET)
    d_arka = r_eff * math.cos(ist) * w
    d_alt = r_eff * math.sin(ist) * w

    # Kuyruk yönü: hedef yeterince hızlıysa HIZ yönünün gerisi (gerçek kuyruk),
    # değilse LOS'un gerisi (drone tarafı) — yavaş/duran hedefte hız yönü gürültü.
    v_yatay = math.hypot(v_hedef[0], v_hedef[1])
    if v_yatay >= cfg.TRACK_MIN_SPD:
        b = (-v_hedef[0] / v_yatay, -v_hedef[1] / v_yatay, 0.0)
        yon_kaynak = "hiz"
    elif menzil > cfg.DP_MIN:
        d_yatay = math.hypot(fark[0], fark[1])
        if d_yatay > 1e-6:
            b = (-fark[0] / d_yatay, -fark[1] / d_yatay, 0.0)
        else:
            b = (0.0, 0.0, 0.0)
        yon_kaynak = "los"
    else:
        b = (0.0, 0.0, 0.0)
        yon_kaynak = "yok"

    p_sanal = (p_hedef[0] + b[0] * d_arka,
               p_hedef[1] + b[1] * d_arka,
               p_hedef[2] + d_alt)              # NED: +z aşağı → altında

    # Yere çakılma koruması: istasyon taban irtifanın altına inmesin.
    if -p_sanal[2] < cfg.LOOKUP_MIN_ALT:
        p_sanal = (p_sanal[0], p_sanal[1], -cfg.LOOKUP_MIN_ALT)
        yer_kirpma = True
    else:
        yer_kirpma = False

    return p_sanal, {"d_arka": d_arka, "d_alt": d_alt, "menzil_gercek": menzil,
                     "gecis_w": w, "yon_kaynak": yon_kaynak,
                     "yer_kirpma": yer_kirpma}


# ──────────────────────────────── yasa ────────────────────────────────────

def t_go_hesapla(dp, dv, cfg=Cfg):
    """t_go = ‖Δp‖/‖Δv‖, sayısal korumalarla kırpılmış.

    ‖Δv‖ → 0 (hedefle hız eşleşmiş, istasyonda asılıyız) durumunda bölme yok:
    t_go tavana oturur ve ZEM/t_go terimi kendiliğinden söner.
    """
    ndp = _norm(dp)
    ndv = _norm(dv)
    if ndv < cfg.DV_MIN:
        return cfg.T_GO_MAX, ndp, ndv
    return _clamp(ndp / ndv, cfg.T_GO_MIN, cfg.T_GO_MAX), ndp, ndv


def komut(dp, dv, v_hedef, cfg=Cfg):
    """FRPN hız formu.

    dp      : p_sanal − p_drone   (m, NED)  — sanal hedefe göreli konum
    dv      : v_sanal − v_drone   (m/s)     — göreli hız (v_sanal ≈ v_hedef)
    v_hedef : hedefin KESTİRİLMİŞ hızı (m/s) — ham sonlu fark DEĞİL

    Dönüş (hepsi loglanır):
      v_cmd      : komut hızı (m/s, NED) — henüz tavan/ivme sınırı UYGULANMAMIŞ
      t_go, zem  : ara büyüklükler
      terim_ff / terim_kapanma / terim_zem : üç terimin ayrı katkısı
      v_c        : seçilen kapanma hızı
      zem_kirpildi : ZEM terimi kendi tavanına dayandı mı
    """
    t_go, ndp, ndv = t_go_hesapla(dp, dv, cfg)
    u, _ = _birim(dp, cfg.DP_MIN)

    # ── terim 2: kapanma ──
    v_c = _clamp(cfg.K_C * ndp, cfg.V_C_MIN, cfg.V_C_MAX)
    terim_kapanma = _olcek(u, v_c)

    # ── terim 3: manevra düzeltmesi ──
    zem = _ekle(dp, _olcek(dv, t_go))
    duzeltme = _olcek(zem, cfg.K_ZEM / t_go)
    n_duz = _norm(duzeltme)
    zem_kirpildi = n_duz > cfg.V_ZEM_MAX
    if zem_kirpildi and n_duz > 1e-9:
        duzeltme = _olcek(duzeltme, cfg.V_ZEM_MAX / n_duz)

    # ── terim 1 + toplam ──
    v_cmd = _ekle(_ekle(v_hedef, terim_kapanma), duzeltme)

    return {
        "v_cmd": v_cmd,
        "t_go": t_go,
        "zem": zem,
        "zem_norm": _norm(zem),
        "v_c": v_c,
        "menzil": ndp,
        "kapanma_ham": ndv,
        "terim_ff": tuple(v_hedef),
        "terim_kapanma": terim_kapanma,
        "terim_zem": duzeltme,
        "zem_kirpildi": zem_kirpildi,
    }
