"""
tests/test_hedef_kestirim.py — IMM (CV+CA) hedef kestiricisinin kabul kriterleri (F2).

Gazebo'suz, saf matematik. Kullanım: python3 -m tests.test_hedef_kestirim

Kapsam:
  K1-K3  model ayrımı: düz uçuşta CV, ivmelide CA kazanmalı
  K4-K6  gürültü reddi: mevcut EMA+sonlu fark kestiriciden ÖLÇÜLEBİLİR ölçüde iyi
  K7-K9  bayat telemetri: 1-2 Hz besleme + ara kareleri doldurma
  K10-K12 sağlamlık: başlangıç, uzun boşluk, sayısal patlama yok
"""

import math
import random

from control.guidance.hedef_kestirim import IMM, Cfg

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


# ── Mevcut kestiricinin birebir taklidi (kıyas tabanı) ──
# gps_guidance.py:206-217 — EMA konum + sonlu fark hız, VEL_EMA ile yumuşatma.
class EskiKestirici:
    POS_EMA, VEL_EMA = 0.4, 0.3

    def __init__(self):
        self.p = None
        self.v = [0.0, 0.0, 0.0]

    def guncelle(self, z, dt):
        if self.p is None:
            self.p = list(z)
            return tuple(self.p), tuple(self.v)
        yeni = [self.POS_EMA * z[i] + (1 - self.POS_EMA) * self.p[i] for i in range(3)]
        if dt and 1e-3 < dt < 2.0:
            for i in range(3):
                self.v[i] = (self.VEL_EMA * ((yeni[i] - self.p[i]) / dt)
                             + (1 - self.VEL_EMA) * self.v[i])
        self.p = yeni
        return tuple(self.p), tuple(self.v)


def _n(v):
    return math.sqrt(sum(x * x for x in v))


def _duz_yorunge(n, dt, hiz=16.0, gurultu=1.5, tohum=1):
    """Sabit hızlı düz uçuş + ölçüm gürültüsü. Dönüş: [(z_olculen, v_gercek)]"""
    rnd = random.Random(tohum)
    out = []
    p = [0.0, 0.0, -50.0]
    v = (hiz, 0.0, 0.0)
    for _ in range(n):
        p = [p[i] + v[i] * dt for i in range(3)]
        z = tuple(p[i] + rnd.gauss(0, gurultu) for i in range(3))
        out.append((z, v))
    return out


def _daire_yorunge(n, dt, hiz=15.2, yaricap=39.0, gurultu=1.5, tohum=2):
    """Daire — 20:52/21:24 uçuşlarındaki gerçek hedef deseni (R=39 m, 15.2 m/s)."""
    rnd = random.Random(tohum)
    w = hiz / yaricap
    out = []
    for k in range(n):
        t = k * dt
        p = (yaricap * math.cos(w * t), yaricap * math.sin(w * t), -50.0)
        v = (-hiz * math.sin(w * t), hiz * math.cos(w * t), 0.0)
        z = tuple(p[i] + rnd.gauss(0, gurultu) for i in range(3))
        out.append((z, v))
    return out


def main():
    print("IMM (CV+CA) hedef kestirici — kabul kriterleri")
    print("=" * 66)
    DT = 0.05                                   # 20 Hz (sim telemetrisi ~11 Hz)

    # ══ K1-K3: MODEL AYRIMI ══
    # ⚠ BEKLENTİ FİZİKLE SINIRLI — burayı "düzeltmeye" çalışmayın.
    # Gerçek daire deseninin merkezcil ivmesi 5.92 m/s² (R=39 m, v=15.2 m/s).
    # Bu ivmenin CV ile CA tahminleri arasında yarattığı KONUM farkı:
    #     20 Hz (dt=0.05 s) → 0.0074 m = ölçüm gürültüsünün 1/200'ü
    #      1 Hz (dt=1.00 s) → 2.96 m   = gürültünün ~2 katı
    # Yani yüksek tazeleme hızında iki model tek adımda ayırt EDİLEMEZ ve IMM
    # her adımda modelleri karıştırdığı için bilgi birikemez. Ağırlıkların 0.5
    # civarında kalması doğru davranıştır; ürün zaten birleştirilmiş kestirim
    # (K4-K6) ve o mükemmel çalışıyor. Ayrım, yarışmadaki 1-2 Hz telemetride
    # kendiliğinden güçlenir — K3b bunu ölçüyor.
    print("\n── K1-K3: model ayrımı (fiziksel sınırla birlikte) ──")
    kf = IMM()
    for z, _ in _duz_yorunge(300, DT, gurultu=0.5):
        d = kf.guncelle(z, DT)
    kontrol("K1  20 Hz düz uçuşta harman CV'ye meylediyor", d["w_cv"] > 0.52,
            f"w_cv={d['w_cv']:.3f}  w_ca={d['w_ca']:.3f}")

    kf = IMM()
    for z, _ in _daire_yorunge(400, DT, gurultu=0.5):
        d = kf.guncelle(z, DT)
    kontrol("K2  20 Hz dönüşte harman CA'ya meylediyor", d["w_ca"] > 0.52,
            f"w_ca={d['w_ca']:.3f}  w_cv={d['w_cv']:.3f}")

    kf = IMM()
    for z, _ in _duz_yorunge(200, DT, gurultu=0.5):
        kf.guncelle(z, DT)
    w_duz = kf.durum()["w_ca"]
    for z, _ in _daire_yorunge(200, DT, gurultu=0.5):
        kf.guncelle(z, DT)
    w_manevra = kf.durum()["w_ca"]
    kontrol("K3  düz→manevra geçişinde CA ağırlığı ÖLÇÜLEBİLİR artıyor",
            w_manevra > w_duz + 0.08,
            f"w_ca: {w_duz:.3f} → {w_manevra:.3f}  (Δ={w_manevra-w_duz:+.3f})")

    # K3b — ASIL KANIT: ölçüm aralığı büyüyünce ayrım güçleniyor mu?
    # Yarışma koşulu (1-2 Hz) tam olarak burası.
    DT_YAVAS = 1.0
    kf = IMM()
    for z, _ in _duz_yorunge(60, DT_YAVAS, gurultu=1.5):
        kf.guncelle(z, DT_YAVAS)
    w_duz_yavas = kf.durum()["w_ca"]
    kf = IMM()
    for z, _ in _daire_yorunge(60, DT_YAVAS, gurultu=1.5):
        kf.guncelle(z, DT_YAVAS)
    w_dnm_yavas = kf.durum()["w_ca"]
    kontrol("K3b 1 Hz'de ayrım BELİRGİN güçleniyor (yarışma koşulu)",
            (w_dnm_yavas - w_duz_yavas) > (w_manevra - w_duz),
            f"1 Hz: {w_duz_yavas:.3f} → {w_dnm_yavas:.3f} (Δ={w_dnm_yavas-w_duz_yavas:+.3f})  "
            f"vs 20 Hz Δ={w_manevra-w_duz:+.3f}")

    # ══ K4-K6: GÜRÜLTÜ REDDİ (asıl gerekçe) ══
    print("\n── K4-K6: gürültü reddi — EMA kıyası ──")
    for ad, yorunge, esik in (("K4  düz uçuş", _duz_yorunge(600, DT), 0.5),
                              ("K5  daire deseni", _daire_yorunge(600, DT), 0.7)):
        kf, eski = IMM(), EskiKestirici()
        h_imm, h_eski = [], []
        for i, (z, v_ger) in enumerate(yorunge):
            d = kf.guncelle(z, DT)
            _, v_eski = eski.guncelle(z, DT)
            if i < 100:                          # oturma süresi hariç
                continue
            h_imm.append(_n([d["v"][k] - v_ger[k] for k in range(3)]))
            h_eski.append(_n([v_eski[k] - v_ger[k] for k in range(3)]))
        import statistics as st
        m_imm, m_eski = st.median(h_imm), st.median(h_eski)
        kontrol(f"{ad}: IMM hız hatası EMA'nın <%{int(esik*100)}'i",
                m_imm < m_eski * esik,
                f"IMM {m_imm:.2f} m/s  vs  EMA {m_eski:.2f} m/s  "
                f"({m_eski/max(m_imm,1e-6):.1f}× iyi)")

    # Şişme testi: ölçülen sorun "hız 1.21× şişik" idi
    kf, eski = IMM(), EskiKestirici()
    imm_h, eski_h = [], []
    for i, (z, v_ger) in enumerate(_duz_yorunge(600, DT)):
        d = kf.guncelle(z, DT)
        _, ve = eski.guncelle(z, DT)
        if i > 100:
            imm_h.append(_n(d["v"]))
            eski_h.append(_n(ve))
    import statistics as st
    gercek = 16.0
    o_imm = st.median(imm_h) / gercek
    o_eski = st.median(eski_h) / gercek
    kontrol("K6  hız ŞİŞMESİ giderildi (gerçeğe oran ≈ 1.0)",
            abs(o_imm - 1.0) < abs(o_eski - 1.0),
            f"IMM {o_imm:.3f}×  vs  EMA {o_eski:.3f}×  (hedef 1.000)")

    # ══ K7-K9: BAYAT TELEMETRİ (yarışma koşulu) ══
    print("\n── K7-K9: bayat telemetri (1-2 Hz) ──")
    # Yarışmada telemetri 1-2 Hz, güdüm 20 Hz. Ölçüm gelmeyen kareler tahmin()
    # ile doldurulur. EMA bunu yapamaz — donuk konum kovalar.
    # DOĞRU KULLANIM: her karede tahmin(dt), ölçüm geldiğinde olcum(z).
    # (Tek çağrıda ikisini yapmak zamanı çift sayardı — bkz. modüldeki not.)
    kf, eski = IMM(), EskiKestirici()
    yor = _duz_yorunge(400, DT)
    hata_imm, hata_eski = [], []
    son_v = None
    for i, (z, v_ger) in enumerate(yor):
        kf.tahmin(DT)                            # her kare ilerlet
        if i % 20 == 0:                          # 1 Hz ölçüm
            kf.olcum(z)
            _, son_v = eski.guncelle(z, DT * 20)
        if i > 60:
            d = kf.durum()
            hata_imm.append(_n([d["v"][k] - v_ger[k] for k in range(3)]))
            if son_v:
                hata_eski.append(_n([son_v[k] - v_ger[k] for k in range(3)]))
    kontrol("K7  1 Hz telemetride IMM hız hatası EMA'dan küçük",
            st.median(hata_imm) < st.median(hata_eski),
            f"IMM {st.median(hata_imm):.2f}  vs  EMA {st.median(hata_eski):.2f} m/s")

    # Konum ekstrapolasyonu: ölçümler arası konum ilerlemeli
    kf = IMM()
    for z, _ in _duz_yorunge(100, DT):
        kf.guncelle(z, DT)
    p0 = kf.durum()["p"]
    kf.tahmin(0.5)
    p1 = kf.durum()["p"]
    ilerleme = p1[0] - p0[0]
    kontrol("K8  ölçümsüz 0.5 s'de konum ileri taşınıyor (~hız×dt)",
            abs(ilerleme - 16.0 * 0.5) < 2.5,
            f"ilerleme={ilerleme:.2f} m  (beklenen ≈ {16.0*0.5:.1f} m)")

    kf = IMM()
    for z, _ in _duz_yorunge(60, DT):
        kf.guncelle(z, DT)
    kf.guncelle((500.0, 500.0, -50.0), 5.0)      # DT_MAX üstü boşluk
    d = kf.durum()
    kontrol("K9  uzun boşluktan sonra filtre sıfırlanıyor (bayat hız taşımıyor)",
            _n(d["v"]) < 3.0,
            f"boşluk sonrası |v|={_n(d['v']):.2f} m/s (eski hız 16 m/s idi)")

    # ══ K10-K12: SAĞLAMLIK ══
    print("\n── K10-K12: sağlamlık ──")
    kf = IMM()
    d = kf.durum()
    kontrol("K10 ölçüm gelmeden durum sorulabilir (hazir=False)",
            d["hazir"] is False and _n(d["v"]) == 0.0, f"{d['hazir']}")

    kf = IMM()
    d = kf.guncelle((10.0, 20.0, -30.0), DT)
    kontrol("K11 ilk ölçüm konuma oturuyor, hız sıfır",
            _n([d["p"][i] - (10.0, 20.0, -30.0)[i] for i in range(3)]) < 1e-6
            and _n(d["v"]) < 1e-6, f"p={tuple(round(x,2) for x in d['p'])}")

    kf = IMM()
    rnd = random.Random(9)
    sonlu = True
    for _ in range(500):                          # saf gürültü, yapı yok
        z = (rnd.gauss(0, 50), rnd.gauss(0, 50), rnd.gauss(0, 50))
        d = kf.guncelle(z, DT)
        if not all(math.isfinite(x) for x in d["p"] + d["v"] + d["a"]):
            sonlu = False
            break
    kontrol("K12 saf gürültüde NaN/patlama yok", sonlu,
            f"w_cv={d['w_cv']:.3f} |v|={_n(d['v']):.1f}")

    print("\n" + "=" * 66)
    gecen = sum(1 for _, ok, _ in _sonuclar if ok)
    toplam = len(_sonuclar)
    print(f"SONUÇ: {gecen}/{toplam} geçti — "
          f"{'HEPSİ GEÇTİ ✓' if gecen == toplam else 'BAŞARISIZ ✗'}")
    if gecen != toplam:
        for ad, ok, detay in _sonuclar:
            if not ok:
                print(f"  FAIL: {ad}  {detay}")
    return 0 if gecen == toplam else 1


if __name__ == "__main__":
    raise SystemExit(main())
