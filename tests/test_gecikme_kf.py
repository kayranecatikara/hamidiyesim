"""
tests/test_gecikme_kf.py — Ö-KF (görüntü gecikmesi telafisi) kabul bekçileri.

Gazebo'suz, saf mantık.
Kullanım: PYTHONPATH=. python3 tests/test_gecikme_kf.py

Kapsam:
  K1-K2   ⚠ YAPISAL: piksel ↔ açı dönüşümü TAM TERSİNİR (yaklaşım yok)
  K3      ⛔ BİT BİT DENKLİK: KF kapalıyken güdüm ölçümleri DEĞİŞMEZ (§5.10)
  K4      ⛔ YARIŞMA KURALI §10: hedef GPS'ine yapısal erişim YOK
  K5-K7   süzgeç doğruluğu: yakınsama, GECİKME TELAFİSİ, hedef hızı kestirimi
  K8      ⭐ MEKANİZMA KAPISI (§5.1): özellik fiilen iş yapıyor mu
  K9      emniyet kilidi: kayma tavanı aşılamaz
  K10     hayalet hedef kalkanı: ufuk tavanından sonra kestirim GEÇERSİZ
  K11     dayanıklılık: bozuk girdi güdümü düşürmez
  K12     aykırı ölçüm kapısı kalıcı kilitlenme yapmaz
  K13     CSV mekanizma sütunları yerinde
"""

import math

from vision import geometry as geo
from control.guidance import bbox_ibvs as ib
from control.guidance import gecikme_kf as gkf
from control.guidance.guidance_core import Cfg as GeoCfg

_sonuclar = []
_TILT = math.radians(GeoCfg.KAMERA_TILT_DEG)


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'✓' if kosul else '✗'} {ad}" + (f"  — {detay}" if detay else ""))
    return bool(kosul)


# ═════════════════════════════════════════════════════════════════════════
def k1_izdusum_tersinir():
    """los_seviye(seviye_to_piksel(az, el)) == (az, el) — TAM tersinirlik.

    Neden kritik: süzgeç kestirimi piksele geri izdüşürülüp `komut()`e
    veriliyor. Bu dönüşüm yaklaşık olsaydı, süzgeç doğru bilse bile güdüm
    yanlış nişan alırdı — ve hata YATIŞLA büyürdü (tam da telafi etmeye
    çalıştığımız durumda).
    """
    en_kotu = 0.0
    n = 0
    for az_d in (-40, -15, -3, 0, 3, 15, 40):
        for el_d in (-25, -8, 0, 8, 25, 45):
            for roll_d in (-50, -20, 0, 20, 50):
                for pitch_d in (-25, 0, 25):
                    az, el = math.radians(az_d), math.radians(el_d)
                    roll, pitch = math.radians(roll_d), math.radians(pitch_d)
                    pk = gkf.seviye_to_piksel(az, el, roll, pitch, geo.FX,
                                              geo.FY, geo.CX, geo.CY, _TILT)
                    if pk is None:
                        continue
                    az2, el2 = ib.los_seviye(pk[0], pk[1], roll, pitch)
                    en_kotu = max(en_kotu, abs(az2 - az), abs(el2 - el))
                    n += 1
    kontrol("K1 piksel↔açı dönüşümü tam tersinir",
            n > 300 and en_kotu < 1e-9,
            f"{n} kombinasyon, en büyük sapma {math.degrees(en_kotu):.2e}°")


def k2_elev_tersinir():
    """govde_elev_to_cy, piksel_elev'in tam tersi (ROLL_TELAFI kapalı yol)."""
    en_kotu = 0.0
    for cy in range(0, 481, 20):
        e = ib.piksel_elev(cy)
        cy2 = gkf.govde_elev_to_cy(e, geo.FY, geo.CY, _TILT)
        en_kotu = max(en_kotu, abs(cy2 - cy))
    kontrol("K2 yükseliş↔cy dönüşümü tam tersinir", en_kotu < 1e-9,
            f"en büyük sapma {en_kotu:.2e} px")


# ═════════════════════════════════════════════════════════════════════════
def k3_bit_bit_denklik():
    """⛔ §5.10 — KF KAPALIYKEN güdüme giden ölçümler BİT BİT AYNI.

    `kf_tazele` güdüm ile kutu arasındaki TEK yeni halkadır. Kapalıyken
    girdileri aynen döndürüyorsa, uçuş yolu değişemez — bu, regresyon
    testinden güçlü bir YAPISAL GARANTİdir (§5.10).
    """
    eski = ib.Cfg.KF_ACIK
    try:
        ib.Cfg.KF_ACIK = False
        suzgec = gkf.GecikmeKF()
        tampon = gkf.TelemetriTamponu()
        farkli = 0
        n = 0
        for cx in (60.0, 320.0, 601.3):
            for cy in (40.0, 240.0, 447.7):
                for bw, bh in ((8.0, 3.0), (44.0, 11.5), (120.0, 30.0)):
                    for roll in (-0.7, 0.0, 0.55):
                        for tau in (0.0, 0.095, 0.21):
                            lh = (0.31, -0.12)
                            kap = 7.25
                            out = ib.kf_tazele(
                                cx, cy, bw, bh, ib.kutu_olcusu(bw, bh),
                                roll, 0.14, 1.02, 12.0, -3.0, 0.4,
                                100.0 + n * 0.06, tau, lh, kap,
                                ib.Cfg, suzgec, tampon)
                            n += 1
                            if (out[0] != cx or out[1] != cy or out[2] != bw
                                    or out[3] != bh or out[4] is not lh
                                    or out[5] != kap):
                                farkli += 1
        kontrol("K3 ⛔ KF kapalı → ölçümler bit bit aynı",
                n >= 200 and farkli == 0,
                f"{n} girdi kombinasyonu, {farkli} sapma; los_hiz nesnesi bile aynı")
        kontrol("K3b KF kapalı → süzgeç hiç çalıştırılmadı",
                suzgec.olcum_sayisi == 0 and not suzgec.hazir(),
                f"olcum_sayisi={suzgec.olcum_sayisi}")
    finally:
        ib.Cfg.KF_ACIK = eski


def k4_yarisma_kurali():
    """⛔ §10 — süzgeç hedefin GPS'ine YAPISAL olarak erişemez.

    B5 bekçisiyle aynı ruh: kaynak metninde hedef telemetrisine giden bir
    yol olmamalı. Girdi yalnız kutu pikselleri + DRONE'UN KENDİ duruş/hızı.
    """
    import inspect
    yasak = ("get_plane", "plane_telem", "hedef_gps", "get_target",
             "scenario", "requests", "urllib", "api/debug")
    kaynak = (inspect.getsource(gkf) + inspect.getsource(ib.kf_tazele)).lower()
    bulunan = [k for k in yasak if k.lower() in kaynak]
    kontrol("K4 ⛔ §10 hedef GPS'ine erişim yok", not bulunan,
            f"yasak ad bulunamadı ({len(yasak)} desen tarandı)"
            if not bulunan else f"BULUNDU: {bulunan}")
    imza = list(inspect.signature(ib.kf_tazele).parameters)
    kontrol("K4b girdiler yalnız kutu + KENDİ telemetrisi",
            all(a.startswith(("cx", "cy", "bw", "bh", "boyut", "i", "now",
                              "gecikme", "los_hiz", "kapanma", "cfg",
                              "suzgec", "tampon")) for a in imza),
            ", ".join(imza))


# ═════════════════════════════════════════════════════════════════════════
def _senaryo(tau, adim=0.0625, sure=3.0, v_hedef=(-15.0, 8.0, 0.0),
             p0=(60.0, 0.0, -12.0), u_own=(0.0, 0.0, 0.0), q=15.0):
    """Bilinen gerçekle sentetik uçuş; GECİKMELİ ölçüm besler.

    Dönüş: (kf_hatasi_listesi, ham_hatasi_listesi) — her ikisi de o ANDAKİ
    gerçek göreli konuma göre. `ham`, güdümün bugün gördüğü şeydir: τ eski
    ölçümün kendisi.
    """
    kf = gkf.GecikmeKF(hedef_ivme=q, ufuk_s=0.30)
    import numpy as np

    def gercek(t):
        return np.array([p0[0] + (v_hedef[0] - u_own[0]) * t,
                         p0[1] + (v_hedef[1] - u_own[1]) * t,
                         p0[2] + (v_hedef[2] - u_own[2]) * t])

    kf_hata, ham_hata = [], []
    t = 0.0
    while t <= sure:
        y = gercek(max(0.0, t - tau))          # ⚠ GECİKMELİ ölçüm
        Rn = np.eye(3) * 0.25
        kf.ilerlet(t, u_own)
        kf.olcum(max(0.0, t - tau), y, Rn, u_own)
        est = kf.kestirim(t, u_own)
        g = gercek(t)
        if est["gecerli"]:
            R, az, el = est["R"], est["az"], est["el"]
            p_kf = np.array([R * math.cos(el) * math.cos(az),
                             R * math.cos(el) * math.sin(az),
                             -R * math.sin(el)])
            kf_hata.append(float(np.linalg.norm(p_kf - g)))
            ham_hata.append(float(np.linalg.norm(y - g)))
        t += adim
    return kf_hata, ham_hata


def k5_yakinsama():
    """Gecikmesiz, temiz ölçümde süzgeç gerçeğe oturur."""
    kf_h, _ = _senaryo(tau=0.0)
    son = kf_h[-20:]
    kontrol("K5 gecikmesiz → süzgeç gerçeğe yakınsıyor",
            len(son) == 20 and max(son) < 0.5,
            f"son 20 karede en büyük hata {max(son):.3f} m")


def k6_gecikme_telafisi():
    """⭐ ASIL İDDİA: τ eski ölçümle bugünkü gerçeğe nişan alabiliyor muyuz?

    `ham` = güdümün BUGÜN gördüğü (τ eski ölçüm) → gecikme hatası.
    `kf`  = süzgecin komut anına taşınmış kestirimi.
    """
    kf_h, ham_h = _senaryo(tau=0.095)          # ölçülen medyan gecikmemiz
    n = len(kf_h) // 2
    kf_son = sorted(kf_h[n:])
    ham_son = sorted(ham_h[n:])
    kf_med = kf_son[len(kf_son) // 2]
    ham_med = ham_son[len(ham_son) // 2]
    kontrol("K6 ⭐ 95 ms gecikme telafi ediliyor",
            kf_med < ham_med * 0.25,
            f"ham (telafisiz) {ham_med:.2f} m → süzgeç {kf_med:.3f} m "
            f"= %{100 * (1 - kf_med / ham_med):.0f} azalma")
    kf_h2, ham_h2 = _senaryo(tau=0.212)        # ölçülen EN KÖTÜ gecikmemiz
    n2 = len(kf_h2) // 2
    kf2 = sorted(kf_h2[n2:])[len(kf_h2[n2:]) // 2]
    ham2 = sorted(ham_h2[n2:])[len(ham_h2[n2:]) // 2]
    kontrol("K6b en kötü gecikmede (212 ms) de telafi ediyor",
            kf2 < ham2 * 0.25,
            f"ham {ham2:.2f} m → süzgeç {kf2:.3f} m")


def k6c_gercekci_kosul():
    """⚠ §5.2 — K5/K6 GÜRÜLTÜSÜZ ve SABİT HIZLI hedefte koşuyor; orada model
    gerçekle birebir örtüştüğü için hata 0.000 çıkıyor. Bu, süzgeci değil
    tautolojiyi sınar. ASIL SORU: gürültü VAR ve hedef MANEVRA YAPARKEN?

    Burada hedef 2 g'lik sürekli dönüş yapıyor (sabit hız modeli KASITLI
    olarak yanlış) ve ölçüme 1σ = 0.5 m gürültü biniyor. Beklenen: süzgeç
    hâlâ ham gecikmeli ölçümden İYİ, ama artık mükemmel DEĞİL.
    """
    import numpy as np
    rng = np.random.default_rng(12345)
    kf = gkf.GecikmeKF(hedef_ivme=15.0, ufuk_s=0.30)
    tau, dt = 0.095, 0.0625
    g = 9.81
    hiz = 15.0
    omega = 2.0 * g / hiz                      # 2 g dönüş → açısal hız (rad/s)
    u = np.array([12.0, 2.0, 0.0])             # BİZ de uçuyoruz

    def gercek(t):
        # hedef sabit hızla DÖNÜYOR (yay), biz düz gidiyoruz
        px = 60.0 + (hiz / omega) * math.sin(omega * t) - u[0] * t
        py = (hiz / omega) * (1.0 - math.cos(omega * t)) - u[1] * t
        return np.array([px, py, -12.0])

    kf_h, ham_h = [], []
    t = 0.0
    while t <= 4.0:
        tm = max(0.0, t - tau)
        y = gercek(tm) + rng.normal(0.0, 0.5, 3)     # ⚠ GÜRÜLTÜLÜ
        kf.ilerlet(t, u)
        kf.olcum(tm, y, np.eye(3) * 0.25, u)
        est = kf.kestirim(t, u)
        if est["gecerli"]:
            R, az, el = est["R"], est["az"], est["el"]
            p_kf = np.array([R * math.cos(el) * math.cos(az),
                             R * math.cos(el) * math.sin(az),
                             -R * math.sin(el)])
            kf_h.append(float(np.linalg.norm(p_kf - gercek(t))))
            ham_h.append(float(np.linalg.norm(y - gercek(t))))
        t += dt
    n = len(kf_h) // 2
    kfm = sorted(kf_h[n:])[len(kf_h[n:]) // 2]
    hamm = sorted(ham_h[n:])[len(ham_h[n:]) // 2]
    kontrol("K6c ⭐ GERÇEKÇİ: 2 g manevra + 0.5 m gürültüde de önde",
            kfm < hamm * 0.6,
            f"ham {hamm:.2f} m → süzgeç {kfm:.2f} m "
            f"= %{100 * (1 - kfm / hamm):.0f} azalma (mükemmel DEĞİL — olması gereken bu)")


def k7_hedef_hizi():
    """Süzgeç hedefin hızını (gizli durum) doğru öğreniyor mu?"""
    import numpy as np
    kf = gkf.GecikmeKF(hedef_ivme=15.0)
    v_h = np.array([-15.0, 8.0, 0.0])
    u = np.array([3.0, 1.0, 0.0])              # BİZ de hareket ediyoruz
    p0 = np.array([60.0, 0.0, -12.0])
    tau, t = 0.095, 0.0
    while t <= 4.0:
        tm = max(0.0, t - tau)
        y = p0 + (v_h - u) * tm
        kf.ilerlet(t, u)
        kf.olcum(tm, y, np.eye(3) * 0.25, u)
        t += 0.0625
    hata = float(np.linalg.norm(kf.x[3:6] - v_h))
    kontrol("K7 hedef hızı kestirimi doğru (bizim hızımız ayrıştırıldı)",
            hata < 1.0,
            f"gerçek {v_h.tolist()} → kestirim "
            f"[{kf.x[3]:.2f}, {kf.x[4]:.2f}, {kf.x[5]:.2f}], hata {hata:.3f} m/s")


# ═════════════════════════════════════════════════════════════════════════
def _tam_yol(kare_sayisi=40, tau=0.095, acik=True, lam=0.6):
    """`kf_tazele` üzerinden UÇTAN UCA — piksel girip piksel çıkıyor."""
    eski = ib.Cfg.KF_ACIK
    ib.Cfg.KF_ACIK = acik
    try:
        suzgec = gkf.GecikmeKF(hedef_ivme=ib.Cfg.KF_HEDEF_IVME,
                               ufuk_s=ib.Cfg.KF_UFUK_S)
        tampon = gkf.TelemetriTamponu()
        C = ib.menzil_sabiti(ib.Cfg)
        cikti = []
        t = 100.0
        for i in range(kare_sayisi):
            # hedef kadrajda sabit hızla kayıyor + yaklaşıyor
            R = 45.0 - 0.55 * i
            az = lam * (i * 0.0625)                     # rad; LOS süpürmesi
            cx = geo.CX + geo.FX * math.tan(az * 0.5)
            cy = geo.CY + 6.0
            boyut = C / R
            bw = boyut / 2.0
            bh = boyut / 2.0
            o = ib.kf_tazele(cx, cy, bw, bh, ib.kutu_olcusu(bw, bh),
                             0.0, 0.0, 0.0, 14.0, 0.0, 0.0,
                             t, tau, (0.0, 0.0), 5.0,
                             ib.Cfg, suzgec, tampon)
            cikti.append((cx, cy, o))
            t += 0.0625
        return cikti
    finally:
        ib.Cfg.KF_ACIK = eski


def k8_mekanizma_kapisi():
    """⭐ §5.1 — özellik FİİLEN iş yapıyor mu? `kf_dcx` sıfırsa koşu geçersiz."""
    cikti = _tam_yol()
    aktif = [o for _, _, o in cikti if o[6]["durum"] == "AKTIF"]
    dcx = [abs(o[6]["dcx"]) for o in aktif if o[6]["dcx"] != ""]
    oran = 100.0 * len(aktif) / len(cikti)
    kontrol("K8 ⭐ mekanizma kapısı: süzgeç fiilen nişanı kaydırıyor",
            len(dcx) > 20 and max(dcx) > 1.0,
            f"AKTIF %{oran:.0f} ({len(aktif)}/{len(cikti)} kare), "
            f"kayma en büyük {max(dcx) if dcx else 0:.1f} px, "
            f"medyan {sorted(dcx)[len(dcx)//2]:.1f} px")
    kapali = _tam_yol(acik=False)
    hic = all(o[6]["dcx"] == "" for _, _, o in kapali)
    kontrol("K8b kapalı kolda mekanizma sütunu BOŞ (kol ayrımı net)", hic)


def k9_emniyet_kilidi():
    """Sapıtmış süzgeç bile nişanı tavandan fazla savuramaz."""
    cikti = _tam_yol(lam=6.0, kare_sayisi=60)   # absürt hızlı LOS süpürmesi
    en = 0.0
    for cx, cy, o in cikti:
        en = max(en, abs(o[0] - cx), abs(o[1] - cy))
    kontrol("K9 emniyet kilidi: kayma tavanı aşılamıyor",
            en <= ib.Cfg.KF_MAX_KAYMA_PX + 1e-9,
            f"en büyük kayma {en:.1f} px ≤ tavan {ib.Cfg.KF_MAX_KAYMA_PX:.0f} px")


def k10_hayalet_hedef_kalkani():
    """Ölçüm kesilince kestirim UFUK_S sonrası GEÇERSİZ olmalı."""
    import numpy as np
    kf = gkf.GecikmeKF(hedef_ivme=15.0, ufuk_s=0.30)
    t = 0.0
    for _ in range(20):                        # önce sağlıklı besle
        y = np.array([50.0 - 10 * t, 2.0 * t, -12.0])
        kf.ilerlet(t, (0.0, 0.0, 0.0))
        kf.olcum(t, y, np.eye(3) * 0.25, (0.0, 0.0, 0.0))
        t += 0.0625
    once = kf.kestirim(t, (0.0, 0.0, 0.0))["gecerli"]
    # ölçümü kes, yalnız tahminle git
    gecerlilik = []
    for _ in range(10):
        t += 0.0625
        kf.ilerlet(t, (0.0, 0.0, 0.0))
        gecerlilik.append(kf.kestirim(t, (0.0, 0.0, 0.0))["gecerli"])
    kontrol("K10 hayalet hedef kalkanı: ufuktan sonra GEÇERSİZ",
            once and gecerlilik[0] and not gecerlilik[-1],
            f"ölçüm kesildi → {sum(gecerlilik)}/10 karede hâlâ geçerli, "
            f"sonra kesildi (tavan {kf.ufuk_s * 1000:.0f} ms)")


def k11_dayaniklilik():
    """Bozuk/uç girdi süzgeci de güdümü de düşürmemeli."""
    eski = ib.Cfg.KF_ACIK
    ib.Cfg.KF_ACIK = True
    try:
        suzgec = gkf.GecikmeKF()
        tampon = gkf.TelemetriTamponu()
        kotu = [
            (320.0, 240.0, 0.0, 0.0, 0.0),          # sıfır kutu
            (0.0, 0.0, 1.0, 1.0, 1.0),              # köşede minik kutu
            (639.0, 479.0, 400.0, 400.0, 400.0),    # kadrajı dolduran kutu
            (320.0, 240.0, 20.0, 5.0, 10.0),        # normal
        ]
        n_ok = 0
        t = 50.0
        for tau in (0.0, 0.5, 5.0, -1.0):
            for cx, cy, bw, bh, boyut in kotu:
                try:
                    o = ib.kf_tazele(cx, cy, bw, bh, boyut, 1.2, -0.6, 2.9,
                                     40.0, -40.0, 9.0, t, tau,
                                     (0.0, 0.0), 0.0, ib.Cfg, suzgec, tampon)
                    assert all(math.isfinite(v) for v in o[:4])
                    n_ok += 1
                except Exception as e:
                    print(f"      ⚠ çöktü: tau={tau} kutu={bw}x{bh} → {e}")
                t += 0.0625
        kontrol("K11 bozuk girdiye dayanıklı (çökme yok, çıktı sonlu)",
                n_ok == 16, f"{n_ok}/16 girdi sorunsuz")
    finally:
        ib.Cfg.KF_ACIK = eski


def k12_aykiri_olcum_kilitlenmez():
    """Aykırı ölçüm kapısı süzgeci KALICI olarak kilitleyemez."""
    import numpy as np
    kf = gkf.GecikmeKF(hedef_ivme=15.0)
    t = 0.0
    for _ in range(12):                        # sağlıklı kilit
        kf.ilerlet(t, (0.0, 0.0, 0.0))
        kf.olcum(t, np.array([50.0, 0.0, -12.0]), np.eye(3) * 0.25,
                 (0.0, 0.0, 0.0))
        t += 0.0625
    # hedef aniden BAŞKA yere ışınlansın (tespit sıçraması gibi)
    for _ in range(10):
        t += 0.0625
        kf.ilerlet(t, (0.0, 0.0, 0.0))
        kf.olcum(t, np.array([-30.0, 40.0, -5.0]), np.eye(3) * 0.25,
                 (0.0, 0.0, 0.0))
    est = kf.kestirim(t, (0.0, 0.0, 0.0))
    p = np.array([est["R"] * math.cos(est["el"]) * math.cos(est["az"]),
                  est["R"] * math.cos(est["el"]) * math.sin(est["az"]),
                  -est["R"] * math.sin(est["el"])])
    hata = float(np.linalg.norm(p - np.array([-30.0, 40.0, -5.0])))
    kontrol("K12 aykırı ölçüm kapısı kilitlenmiyor (sıfırlanıp topluyor)",
            hata < 5.0, f"ışınlanma sonrası hata {hata:.2f} m")


def k13_csv_sutunlari():
    """Mekanizma sütunları log şemasında var mı (§5.1 analiz edilebilirlik)."""
    gerekli = ["kf_durum", "kf_tau_ms", "kf_ileri_ms", "kf_dcx", "kf_dcy",
               "kf_R", "kf_kapanma", "kf_lam_az", "kf_P_iz", "kf_yenilik"]
    eksik = [a for a in gerekli if a not in ib._CSV_ALANLAR]
    kontrol("K13 CSV mekanizma sütunları yerinde", not eksik,
            f"{len(gerekli)} sütun, toplam {len(ib._CSV_ALANLAR)} alan"
            if not eksik else f"EKSİK: {eksik}")


def k14_dongu_duman():
    """⭐ DÖNGÜ DUMAN TESTİ — GERÇEK `run_bbox_ibvs` sahte kutu akışıyla.

    K1-K13 `kf_tazele`'yi sınar; bu test DÖNGÜ KABLOLAMASINI sınar: değişken
    kapsamı, CSV anahtarları, hata yolu. (Depoda daha önce döngüyü koşturan
    hiçbir test yoktu; bu boşluk Ö-KF eklenirken kapatıldı.)

    Saat SAHTEDİR: gerçek döngü `time.monotonic()` + `time.sleep()` kullanır,
    `dt` koşudan koşuya birkaç ms oynar ve komutlar ister istemez farklı
    çıkar. Bit bit denklik ancak saat sabitlenince sınanabilir.
    """
    import glob
    import os
    import shutil
    import tempfile
    import threading
    import time as _gercek_zaman
    from control.guidance import common

    class _SahteSaat:
        def __init__(self):
            self.t = 5000.0
        def monotonic(self):
            return self.t
        def time(self):
            return self.t
        def sleep(self, s):
            self.t += max(0.0, s)
        def strftime(self, *a, **k):
            return _gercek_zaman.strftime(*a, **k)

    saat = _SahteSaat()
    gonderilen = []
    eski_send_ib = ib.send_velocity
    eski_time = ib.time
    eski_logdir = ib._LOG_DIR
    eski_kf = ib.Cfg.KF_ACIK
    gecici = tempfile.mkdtemp(prefix="okf_duman_")
    try:
        ib.time = saat
        ib._LOG_DIR = gecici
        ib.send_velocity = lambda conn, vx, vy, vz, yaw: gonderilen.append(
            (round(vx, 6), round(vy, 6), round(vz, 6), round(yaw, 6)))

        def kosu(kf_acik, n_kare=45):
            del gonderilen[:]
            saat.t = 5000.0
            ib.Cfg.KF_ACIK = kf_acik
            d = {"seq": 0}

            def wait_pose(son_seq, timeout=0.5):
                if d["seq"] >= n_kare:
                    return None
                d["seq"] += 1
                i = d["seq"]
                R = 30.0 - 0.5 * i                # 30 m → 8 m
                boyut = ib.menzil_sabiti(ib.Cfg) / max(R, 4.0)
                kenar = boyut / math.sqrt(2.0)    # ölçü 'kosegen'
                saat.t += 0.0625
                return {"seq": i, "wall_recv": saat.t - 0.095,
                        "pose": {"cx": 320.0 + 90.0 * math.sin(i * 0.19),
                                 "cy": 240.0 + 25.0 * math.sin(i * 0.11),
                                 "w": kenar, "h": kenar, "conf": 0.9},
                        "stamp": saat.t, "lock": True}

            def get_iris():
                i = d["seq"]
                return {"yaw": 0.15 * math.sin(i * 0.17),
                        "roll": 0.6 * math.sin(i * 0.23),
                        "pitch": 0.12 * math.sin(i * 0.13),
                        "vx": 17.0, "vy": 1.5, "vz": -0.3}

            sonuc = ib.run_bbox_ibvs(None, get_iris, wait_pose,
                                     threading.Event(), ib.Cfg,
                                     kayip_kare_esik=3, ff_hiz=(15.0, 0.0, 0.0))
            log = sorted(glob.glob(os.path.join(gecici, "*.csv")))[-1]
            with open(log) as fh:
                import csv as _csv
                sat = [r for r in _csv.DictReader(fh) if r["durum"] == "GORSEL"]
            return sonuc, list(gonderilen), sat

        sA, cmdA, satA = kosu(False)
        sA2, cmdA2, _ = kosu(False)
        sB, cmdB, satB = kosu(True)

        kontrol("K14 döngü koşuyor (KF açık ve kapalı, çökme yok)",
                sA == sA2 == sB == "kayip" and len(satA) == len(satB) == 45,
                f"kapalı {len(satA)} kare / açık {len(satB)} kare, sonuç={sB}")
        kontrol("K14b ⛔ KF kapalı → aynı girdide komutlar BİT BİT aynı",
                cmdA == cmdA2 and len(cmdA) == 45,
                f"{len(cmdA)} komut, fark 0")
        aktif = [r for r in satB if r["kf_durum"] == "AKTIF"]
        dcx = [abs(float(r["kf_dcx"])) for r in aktif if r["kf_dcx"]]
        kontrol("K14c ⭐ döngüde MEKANİZMA KAPISI açık (§5.1)",
                len(aktif) >= 35 and dcx and max(dcx) > 2.0,
                f"AKTIF %{100 * len(aktif) / len(satB):.0f}, |kf_dcx| medyan "
                f"{sorted(dcx)[len(dcx) // 2]:.1f} px, maks {max(dcx):.1f} px, "
                f"τ medyan {sorted(float(r['kf_tau_ms']) for r in aktif)[len(aktif) // 2]:.0f} ms")
        n = min(len(cmdA), len(cmdB))
        fark = max((max(abs(a - b) for a, b in zip(cmdA[i], cmdB[i]))
                    for i in range(n)), default=0.0)
        kontrol("K14d açık kol GERÇEKTEN farklı uçuyor (nötr değil)",
                cmdA != cmdB and fark > 0.05,
                f"en büyük komut farkı {fark:.3f}")
    finally:
        ib.send_velocity = eski_send_ib
        ib.time = eski_time
        ib._LOG_DIR = eski_logdir
        ib.Cfg.KF_ACIK = eski_kf
        shutil.rmtree(gecici, ignore_errors=True)


def main():
    print("\n=== Ö-KF · GÖRÜNTÜ GECİKMESİ TELAFİSİ — kabul bekçileri ===\n")
    print(" [yapısal — dönüşüm]")
    k1_izdusum_tersinir(); k2_elev_tersinir()
    print("\n [yapısal — bozmama garantisi]")
    k3_bit_bit_denklik(); k4_yarisma_kurali()
    print("\n [süzgeç doğruluğu]")
    k5_yakinsama(); k6_gecikme_telafisi(); k6c_gercekci_kosul()
    k7_hedef_hizi()
    print("\n [uçtan uca + emniyet]")
    k8_mekanizma_kapisi(); k9_emniyet_kilidi(); k10_hayalet_hedef_kalkani()
    k11_dayaniklilik(); k12_aykiri_olcum_kilitlenmez(); k13_csv_sutunlari()
    print("\n [döngü — uçtan uca kablolama]")
    k14_dongu_duman()
    gecen = sum(1 for _, k, _ in _sonuclar if k)
    print(f"\n=== {gecen}/{len(_sonuclar)} geçti ===")
    return 0 if gecen == len(_sonuclar) else 1


def test_gecikme_kf():
    assert main() == 0


if __name__ == "__main__":
    raise SystemExit(main())
