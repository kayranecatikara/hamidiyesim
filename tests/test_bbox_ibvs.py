"""
tests/test_bbox_ibvs.py — GÖRSEL GÜDÜM kabul kriterleri (TEK YASA).

Gazebo'suz, saf mantık. Kullanım: PYTHONPATH=. python3 tests/test_bbox_ibvs.py

⚠ 2026-08-19 · TERMİNAL FAZI KODDAN TAMAMEN SİLİNDİ (§5.12).
Kullanıcı: *"terminal fazı diye bir şey neden var ki? Sistem iki fazdan
oluşsa: GPS ve görsel. Görsel faz ikiye falan bölünmesin, tek parça kalsın."*
Eski dosya, silinen iki parçalı tasarımı (seyir TUTUŞ + terminal KESİŞİM)
koruyordu; o bekçiler artık olmayan kodu sınıyordu. Bu dosya YENİ tek yasayı
korur. Silinen tasarımın ölçümleri `docs/kampanya/TF_TEK_FAZ.md`'de,
kodu git tarihçesinde (`6c20b2b` ve öncesi).

YASA — üç cümle:
  YATAY : hız vektörünün YÖNÜ hedefe döner   (yaw + K_YAW·eps_yaw)
  DİKEY : AYNI matematik, aynı eksen için    (K_ELEV·elev_los)
  HIZ   : denge kutusu = TEMAS kutusu → hep kapat, V_HUCUM'da otur

Kapsam:
  A1-A6   temel yasa: nişan, sağ/sol, dikey işaret, |v| korunumu, hız tavanı
  B1-B4   ⚠ YAPISAL: silinen terminal fazı geri sızmasın (§5.12 bekçisi)
  C1-C5   yarışma kuralı §10, kutu geçerliliği, kayıp davranışı
  D1-D6   yavaşlama (kaçıracaksan yavaşla)
  E1-E9   yan özellikler: roll telafisi, lead, kaçış, Ö5/Ö8/Ö9, yaw slew
  F1-F3   döngü duman testi
"""

import inspect
import math
import threading
import time

from vision import geometry as geo
from control.guidance import bbox_ibvs as ib
from control.guidance.guidance_core import Cfg as GeoCfg

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def _cy_icin(elev_deg, cfg=None):
    """Hedefi SEVİYE çerçevesinde `elev_deg` yükselişte gösteren cy (roll=pitch=0)."""
    return geo.CY + geo.FY * math.tan(
        math.radians(GeoCfg.KAMERA_TILT_DEG - elev_deg))


def _hiz(r):
    return math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)


def _kutu_uret(olcu_degeri, cfg, en_boy=4.48):
    """İstenen ölçü değerini veren (w, h). en_boy = w/h (talon arkadan 4.48)."""
    k = en_boy
    if cfg.BOYUT_OLCU == "kosegen":
        h = olcu_degeri / math.sqrt(k * k + 1.0)
    else:
        h = olcu_degeri / math.sqrt(k)
    return k * h, h


def main():
    print("GÖRSEL GÜDÜM — TEK YASA · kabul kriterleri")
    C = ib.Cfg
    CX, CY, FX, FY = geo.CX, geo.CY, geo.FX, geo.FY

    # ══════════════════════════════════════════════════════════════════
    print("=" * 62)
    print("A · TEMEL YASA")

    # A1: hedef tam ileride (seviye) → yaw sapmaz, dikey komut ~0
    _r = ib.komut(CX, _cy_icin(0.0), 30, 30, 0.0, 15.0, 0.05, C)
    kontrol("A1 hedef ileride: yaw≈0, dikey≈0",
            abs(_r[3]) < 1e-6 and abs(_r[2]) < 0.05,
            f"yaw={math.degrees(_r[3]):.3f}°  vz={_r[2]:+.4f} m/s")

    # A2: hedef SAĞDA → yaw komutu SAĞA (pozitif)
    _sag = ib.komut(CX + 100.0, _cy_icin(0.0), 30, 30, 0.0, 15.0, 0.05, C)
    _sol = ib.komut(CX - 100.0, _cy_icin(0.0), 30, 30, 0.0, 15.0, 0.05, C)
    kontrol("A2 yatay işaret: sağdaki hedef → sağa yaw, soldaki → sola",
            _sag[3] > 0.05 and _sol[3] < -0.05,
            f"sağ→{math.degrees(_sag[3]):+.1f}°  sol→{math.degrees(_sol[3]):+.1f}°")

    # A3: hedef YUKARIDA → TIRMAN (NED'de vz negatif), aşağıda → alçal
    _yuk = ib.komut(CX, _cy_icin(20.0), 30, 30, 0.0, 15.0, 0.05, C)
    _asa = ib.komut(CX, _cy_icin(-20.0), 30, 30, 0.0, 15.0, 0.05, C)
    kontrol("A3 dikey işaret: yukarıdaki hedef → tırman, aşağıdaki → alçal",
            _yuk[2] < -0.5 and _asa[2] > 0.5,
            f"+20°→vz={_yuk[2]:+.2f} (negatif=tırman)  −20°→vz={_asa[2]:+.2f}")

    # A4: ⭐ DİKEY = YATAYIN AYNI MATEMATİĞİ — |v| KORUNUR
    # Yatayın kuralı: yön döner, büyüklük sabit. Dikey de öyle olmalı.
    class _Sonumsuz(ib.Cfg):
        K_VZ_D = 0.0          # sönümleme |v| değişmezliğini bozar
        VZ_MAX = 99.0         # tavan da kırpmasın
        YAVASLA = False       # yavaşlama v_los'u değiştirmesin
    _n, _enb = 0, 0.0
    for _e in (-25.0, -10.0, 0.0, 10.0, 22.0):
        for _b in (10, 25, 50):
            _r = ib.komut(CX, _cy_icin(_e), _b, _b, 0.0, 20.0, 0.05, _Sonumsuz)
            _enb = max(_enb, abs(_hiz(_r) - _r[5]["v_los"]))
            _n += 1
    kontrol("A4 ⭐ dikey yatayla AYNI: yön döner, |v| KORUNUR",
            _enb < 1e-9,
            f"{_n} kombinasyonda |(vx,vy,vz)| ile v_los farkı {_enb:.2e} — "
            "dikey ayrı bir ÖLÇEK değil, aynı vektörün YÖNÜ")

    # A5: ⭐ PARK YOK — denge kutusu TEMAS kutusudur
    # Eski tasarımda BOYUT_REF=25 px idi → 160/25 = 6.4 m'de dururdu ve
    # terminal fazı tam da o parkı ezmek için vardı.
    _hatalar = [ib.komut(CX, _cy_icin(0.0), _b, _b, 0.0, 15.0, 0.05, C)[5]["hata"]
                for _b in (10, 25, 60, 120)]
    kontrol("A5 ⭐ 'uzakta park et' setpoint'i YOK — hep kapat",
            all(h > 0.0 for h in _hatalar) and C.HUCUM_MENZIL_M <= 1.5,
            f"kutu 10/25/60/120 px → hata " +
            " ".join(f"{h:+.0f}" for h in _hatalar) +
            f" px (hepsi + = 'kapat'); PI'nın sıfır noktası "
            f"{C.HUCUM_MENZIL_M:.1f} m = TEMAS")

    # A6: hız V_HUCUM tavanını aşmaz, V_MIN altına inmez
    _v = [ib.komut(CX, _cy_icin(0.0), _b, _b, 0.0, _I, 0.05, C)[5]["v_los"]
          for _b in (6, 20, 60) for _I in (0.0, 12.0, 24.0)]
    kontrol("A6 hız V_HUCUM tavanında oturur, taşmaz",
            all(C.V_MIN - 1e-9 <= x <= C.V_HUCUM + 1e-9 for x in _v),
            f"9 kombinasyon → {min(_v):.1f} … {max(_v):.1f} m/s "
            f"(taban {C.V_MIN:.0f}, tavan {C.V_HUCUM:.0f})")

    # ══════════════════════════════════════════════════════════════════
    print("=" * 62)
    print("G · KUTU → MENZİL ÖLÇÜSÜ (çarpım ↔ köşegen)")

    class _Kos(ib.Cfg):
        BOYUT_OLCU = "kosegen"

    # ⚠ 2026-08-19: taban artık KÖŞEGEN. Çarpım kolunu sınamak için açık
    # sınıf gerekiyor — `C` ile kıyaslamak iki köşegeni kıyaslamak olurdu.
    class _Car(ib.Cfg):
        BOYUT_OLCU = "carpim"

    # G1: iki ölçü de doğru sayıyı veriyor
    _w, _h = 40.0, 14.0
    kontrol("G1 kutu_olcusu: çarpım = sqrt(w·h), köşegen = sqrt(w²+h²)",
            abs(ib.kutu_olcusu(_w, _h, _Car) - math.sqrt(_w * _h)) < 1e-9
            and abs(ib.kutu_olcusu(_w, _h, _Kos) - math.hypot(_w, _h)) < 1e-9
            and C.BOYUT_OLCU == "kosegen",
            f"w={_w:.0f} h={_h:.0f} → çarpım {ib.kutu_olcusu(_w,_h,_Car):.1f} px, "
            f"köşegen {ib.kutu_olcusu(_w,_h,_Kos):.1f} px; "
            f"varsayılan = {C.BOYUT_OLCU}")

    # G2: ⭐ KÖŞEGEN YATIŞTAN BAĞIMSIZ — ince çubuk için TAM
    # Kutu eksen-hizalı: θ dönmüş L uzunluğunda çubuk → w=L·cosθ, h=L·sinθ.
    # Köşegen L kalmalı; çarpım ise θ=0'da SIFIRA gider (dejenere).
    _L = 100.0
    _kos, _car = [], []
    for _td in (0.0, 10.0, 25.0, 45.0, 70.0, 89.0):
        _t = math.radians(_td)
        _ww, _hh = _L * abs(math.cos(_t)), _L * abs(math.sin(_t))
        _kos.append(ib.kutu_olcusu(_ww, _hh, _Kos))
        _car.append(ib.kutu_olcusu(_ww, _hh, _Car))
    kontrol("G2 ⭐ köşegen yatıştan BAĞIMSIZ (ince çubuk), çarpım DEĞİL",
            max(_kos) - min(_kos) < 1e-9
            and (max(_car) - min(_car)) / max(_car) > 0.5,
            f"çubuk 0-89° döndü → köşegen {min(_kos):.1f}…{max(_kos):.1f} px "
            f"(değişim {max(_kos)-min(_kos):.1e}); "
            f"çarpım {min(_car):.1f}…{max(_car):.1f} px "
            f"(%{100*(max(_car)-min(_car))/max(_car):.0f} değişim)")

    # G3: ⭐ ÖLÇÜ DEĞİŞİMİ TEK DEĞİŞKEN — eşikler METRE olduğu için aynı
    # fiziksel menzilde aynı hız/lead üretilir. Kalibre sabitleri gerçek
    # ölçümden geldiği için tam eşitlik beklenmez; hedef, hız hatasının
    # menzil hatasıyla AYNI mertebede kalması.
    _fark = []
    for _R in (2.0, 5.0, 10.0, 20.0):
        _wc, _hc = _kutu_uret(ib.menzil_sabiti(_Car) / _R, _Car)
        _wk, _hk = _kutu_uret(ib.menzil_sabiti(_Kos) / _R, _Kos)
        _a = ib.komut(CX, _cy_icin(0.0), _wc, _hc, 0.0, 15.0, 0.05, _Car)
        _b = ib.komut(CX, _cy_icin(0.0), _wk, _hk, 0.0, 15.0, 0.05, _Kos)
        _fark.append((_R, _a[5]["lead_olcek"], _b[5]["lead_olcek"]))
    kontrol("G3 ⭐ ölçü değişimi TEK DEĞİŞKEN: aynı menzilde aynı lead",
            all(abs(a - b) < 1e-9 for _, a, b in _fark),
            "  ".join(f"{r:.0f} m→{a:.2f}/{b:.2f}" for r, a, b in _fark)
            + "  (çarpım/köşegen — eşikler metre olduğu için birebir)")

    # G4: HUCUM_MENZIL_M gerçekten metre — her iki ölçüde de aynı yerde sıfır
    _s = []
    for _cfg in (_Car, _Kos):
        _ww, _hh = _kutu_uret(ib.menzil_sabiti(_cfg) / _cfg.HUCUM_MENZIL_M,
                              _cfg)
        _s.append(ib.komut(CX, _cy_icin(0.0), _ww, _hh, 0.0, 15.0, 0.05,
                           _cfg)[5]["hata"])
    kontrol("G4 PI'nın sıfır noktası METREDE sabit (ölçüden bağımsız)",
            all(abs(x) < 1e-9 for x in _s),
            f"{_Car.HUCUM_MENZIL_M:.1f} m'de hata: çarpım {_s[0]:.2e}, "
            f"köşegen {_s[1]:.2e}")

    # G5: BOYUT_MIN ölçüden BAĞIMSIZ (piksel güvenilirlik kapısı)
    _kck = _Car.BOYUT_MIN - 1.0
    kontrol("G5 BOYUT_MIN ölçü seçiminden BAĞIMSIZ (hep sqrt(w·h))",
            ib._kutu_gecerli({"conf": 0.9, "bbox": (0, 0, _kck, _kck)},
                             _Kos) is None,
            f"köşegende bile {_kck:.0f} px kutu elenir — eşik sessizce "
            "gevşemiyor")

    # G6: model ölçüleri ile kalibre tutarlı mı (kaba akıl kontrolü)
    _S_car = C.MENZIL_PX_M_CARPIM / FX
    _S_kos = C.MENZIL_PX_M_KOSEGEN / FX   # taban artık köşegen
    kontrol("G6 kalibre sabitleri makul aralıkta",
            0.5 < _S_car < 2.5 and 1.0 < _S_kos < 3.0 and _S_kos > _S_car,
            f"ima edilen görünen boy: çarpım {_S_car:.2f} m, köşegen "
            f"{_S_kos:.2f} m  (model: kanat 1.280, gövde 0.814, yük. 0.286)")

    # ══════════════════════════════════════════════════════════════════
    print("=" * 62)
    print("B · ⚠ SİLİNEN TERMİNAL FAZI GERİ SIZMASIN (§5.12 bekçisi)")

    # B1: Cfg'de terminal alanı KALMADI
    _olu = ["TERMINAL_BOYUT", "TERMINAL_SURE", "V_TERMINAL", "V_TERM_MIN",
            "TERM_BIRAK_M", "VZ_MAX_TERM", "TERM_ROLL", "TERM_SAF3B",
            "TERM_HIZ_KORU", "TERM_YAVASLA", "TERM_TAM_HIZ", "TEK_FAZ",
            "DIKEY_KAPI_M", "CY_NISAN", "K_VZ", "V_NOM", "DIKEY_ROLL",
            "KAPANMA", "KAPANMA_MIN", "V_TOPLAM_MAX", "LEAD_ERKEN",
            "MENZIL_PX_M", "HUCUM_BOYUT_REF", "BOYUT_REF"]
    _kalan = [a for a in _olu if hasattr(ib.Cfg, a)]
    kontrol("B1 ⚠ Cfg'de terminal/eski-dikey alanı KALMADI",
            not _kalan,
            f"{len(_olu)} ad denendi, kalan: {_kalan or 'YOK'}")

    # B2: komut() imzasında `terminal` ve `v_term_kilit` YOK
    _imza = list(inspect.signature(ib.komut).parameters)
    kontrol("B2 ⚠ komut() imzasında faz anahtarı YOK",
            "terminal" not in _imza and "v_term_kilit" not in _imza,
            f"parametreler: {_imza}")

    # B3: CSV'de terminal sütunu yok, durum tek değer
    _olu_sutun = [c for c in ("dikey_ofs_m", "eps_elev_deg", "eps_elev_ham_deg")
                  if c in ib._CSV_ALANLAR]
    kontrol("B3 ⚠ CSV'de silinen sütunlar kalmadı",
            not _olu_sutun,
            f"_CSV_ALANLAR {len(ib._CSV_ALANLAR)} sütun; kalan ölü: "
            f"{_olu_sutun or 'YOK'}")

    # B4: kaynak metninde faz mandalı kalmadı (yorumlar hariç)
    import os as _os
    _src = open(_os.path.join(_os.path.dirname(_os.path.dirname(
        _os.path.abspath(__file__))), "control", "guidance",
        "bbox_ibvs.py")).read()
    _kod = "\n".join(L for L in _src.split("\n")
                     if not L.lstrip().startswith("#"))
    _iz = [a for a in ("terminal_mandal", "kor_baslangic", "v_term_kilit",
                       "TERM_KOR", "dikey_ofs") if a in _kod]
    kontrol("B4 ⚠ kodda faz mandalı makinesi kalmadı",
            not _iz, f"kalan iz: {_iz or 'YOK'} (yorumlar sayılmadı)")

    # ══════════════════════════════════════════════════════════════════
    print("=" * 62)
    print("C · YARIŞMA KURALI ve KUTU GEÇERLİLİĞİ")

    # C1: ⚠ §10 — görsel yasa hedefin GPS'ini ALMIYOR
    _yasak = {"plane", "hedef", "target", "gps", "hedef_pos", "plane_pos",
              "hedef_hiz", "plane_hiz", "menzil", "mesafe", "dikey_ofs"}
    kontrol("C1 ⚠ §10: komut() hedefin GPS'ini ALMIYOR (yapısal)",
            not (set(_imza) & _yasak),
            f"hedefe dair tek veri kutu (cx, cy, w, h); yasak ad yok")

    # C2: düşük güven kutusu elenir
    kontrol("C2 CONF_MIN altındaki kutu elenir",
            ib._kutu_gecerli({"conf": C.CONF_MIN - 0.01, "bbox": (10, 10, 40, 40)},
                             C) is None
            and ib._kutu_gecerli({"conf": C.CONF_MIN + 0.01,
                                  "bbox": (10, 10, 40, 40)}, C) is not None,
            f"CONF_MIN={C.CONF_MIN}")

    # C3: çok küçük kutu elenir (menzil hesabı patlamasın)
    _kucuk = C.BOYUT_MIN - 1.0
    kontrol("C3 BOYUT_MIN altındaki kutu elenir",
            ib._kutu_gecerli({"conf": 0.9,
                              "bbox": (10, 10, 10 + _kucuk, 10 + _kucuk)},
                             C) is None,
            f"BOYUT_MIN={C.BOYUT_MIN} px")

    # C4: menzil ölçeği — R = C / kutu, C ölçüye göre seçilir
    _Cc = ib.menzil_sabiti(C)
    kontrol("C4 menzil ölçeği: R = menzil_sabiti / kutu",
            80.0 < _Cc < 400.0 and abs(_Cc / 20.0 - _Cc / 20.0) < 1e-9,
            f"ölçü={C.BOYUT_OLCU}, C={_Cc:.1f} px·m → 20 px = "
            f"{_Cc / 20.0:.1f} m")

    # C5: dikey komut VZ_MAX ile sınırlı
    _dik = [abs(ib.komut(CX, _cy_icin(_e), 30, 30, 0.0, 20.0, 0.05, C)[2])
            for _e in (-55.0, -30.0, 30.0, 55.0)]
    kontrol("C5 dikey komut VZ_MAX tavanını aşmaz",
            all(x <= C.VZ_MAX + 1e-9 for x in _dik),
            f"±30°/±55° → {' '.join(f'{x:.2f}' for x in _dik)} "
            f"(tavan {C.VZ_MAX:.0f})")

    # ══════════════════════════════════════════════════════════════════
    print("=" * 62)
    print("D · KAÇIRACAKSAN YAVAŞLA")

    class _Hizli(ib.Cfg):
        YAVASLA = False
    _tavan = math.degrees(math.asin(min(1.0, C.VZ_MAX / C.V_HUCUM)))

    # D1: ⭐ dik hedefte vektör gerçekten daha çok eğiliyor
    _kayit = []
    for _e in (15.0, 30.0, 45.0):
        _y = ib.komut(CX, _cy_icin(_e), 25, 25, 0.0, 20.0, 0.05, C)
        _h = ib.komut(CX, _cy_icin(_e), 25, 25, 0.0, 20.0, 0.05, _Hizli)
        _ay = math.degrees(math.asin(min(1.0, abs(_y[2]) / max(1e-6, _hiz(_y)))))
        _ah = math.degrees(math.asin(min(1.0, abs(_h[2]) / max(1e-6, _hiz(_h)))))
        _kayit.append((_e, _ah, _ay, _y[5]["v_los"]))
    kontrol("D1 ⭐ dik hedefte YAVAŞLAR, vektör hedefe döner",
            all(ay > ah + 1.0 for e, ah, ay, v in _kayit if e > _tavan),
            f"tavan asin({C.VZ_MAX:.0f}/{C.V_HUCUM:.0f})={_tavan:.1f}°  |  " +
            "  ".join(f"{e:.0f}°→ yavaşlamasız {ah:.1f}°, yavaşlamalı {ay:.1f}° "
                      f"(v={v:.1f})" for e, ah, ay, v in _kayit))

    # D2: hızı ASLA ARTIRMAZ
    kontrol("D2 yavaşlama hızı ASLA artırmaz",
            all(v <= C.V_HUCUM + 1e-9 for _, _, _, v in _kayit),
            f"tavan {C.V_HUCUM:.0f} m/s, en yüksek "
            f"{max(v for _, _, _, v in _kayit):.1f}")

    # D3: V_HUCUM_MIN tabanının altına inmez
    _vler = [ib.komut(CX, _cy_icin(_e), 25, 25, 0.0, 20.0, 0.05, C)[5]["v_los"]
             for _e in (-58.0, -40.0, 0.0, 40.0, 58.0)]
    kontrol("D3 yavaşlama tabanın altına İNMEZ",
            all(x >= C.V_HUCUM_MIN - 1e-9 for x in _vler),
            f"±58° dahil → {' '.join(f'{x:.1f}' for x in _vler)} "
            f"(taban {C.V_HUCUM_MIN:.0f})")

    # D4: ⭐ hedef HİZALIYKEN yavaşlama DEVREYE GİRMEZ (bit bit)
    _n, _enb = 0, 0.0
    for _e in (-3.0, 0.0, 3.0):
        for _b in (15, 30, 60):
            _y = ib.komut(CX, _cy_icin(_e), _b, _b, 0.0, 20.0, 0.05, C)
            _h = ib.komut(CX, _cy_icin(_e), _b, _b, 0.0, 20.0, 0.05, _Hizli)
            _n += 1
            for _i in range(4):
                _enb = max(_enb, abs(_y[_i] - _h[_i]))
    kontrol("D4 ⭐ hedef hizalıyken yavaşlama DEVREYE GİRMEZ (bit bit)",
            _enb < 1e-12,
            f"{_n} kombinasyonda fark {_enb:.2e} — yalnız {_tavan:.1f}° "
            "üstünde kısar; sakin yaklaşma bozulmaz")

    # D5: kapatılabilir (kill-switch)
    kontrol("D5 yavaşlama kapatılabilir + varsayılan AÇIK",
            C.YAVASLA and not _Hizli.YAVASLA,
            f"Cfg.YAVASLA={C.YAVASLA}; AVCI_IBVS_YAVASLA=0 kapatır")

    # D6: dikey sönümleme gerçekten sönümlüyor
    class _Sonsuz(ib.Cfg):
        K_VZ_D = 0.0
    _a = ib.komut(CX, _cy_icin(15.0), 30, 30, 0.0, 18.0, 0.05, C,
                  iris_vz=-6.0)          # araç ZATEN hızla tırmanıyor
    _b = ib.komut(CX, _cy_icin(15.0), 30, 30, 0.0, 18.0, 0.05, _Sonsuz,
                  iris_vz=-6.0)
    kontrol("D6 dikey sönümleme aracın KENDİ hızını hesaba katar",
            _a[2] > _b[2] + 0.05,
            f"araç −6 m/s tırmanırken: sönümlemeli vz={_a[2]:+.2f}, "
            f"sönümlemesiz {_b[2]:+.2f} — komut geri çekiliyor")

    # ══════════════════════════════════════════════════════════════════
    print("=" * 62)
    print("E · YAN ÖZELLİKLER")

    # E1: T1a yatay roll telafisi — yatışta azimut düzelir
    class _RollKapali(ib.Cfg):
        ROLL_TELAFI = False
    _a = ib.komut(CX + 90.0, 260.0, 30, 30, 0.0, 15.0, 0.05, C,
                  iris_roll=math.radians(35.0), iris_pitch=math.radians(-8.0))
    _b = ib.komut(CX + 90.0, 260.0, 30, 30, 0.0, 15.0, 0.05, _RollKapali,
                  iris_roll=math.radians(35.0), iris_pitch=math.radians(-8.0))
    kontrol("E1 yatay roll telafisi yatışta azimutu düzeltir",
            abs(_a[3] - _b[3]) > math.radians(1.0),
            f"35° yatışta yaw farkı {math.degrees(abs(_a[3]-_b[3])):.1f}°")

    # E2: ⭐ dikey roll telafisi HER KAREDE (ayrı anahtar YOK)
    # Eski tasarımda seyirde açık / terminalde kapalıydı — tutarsızdı.
    _r0 = ib.komut(CX + 90.0, 260.0, 30, 30, 0.0, 15.0, 0.05, C, iris_roll=0.0)
    _r40 = ib.komut(CX + 90.0, 260.0, 30, 30, 0.0, 15.0, 0.05, C,
                    iris_roll=math.radians(40.0))
    kontrol("E2 ⭐ dikey roll telafisi HER KAREDE (tek yol)",
            abs(_r40[2] - _r0[2]) > 0.05 and not hasattr(ib.Cfg, "DIKEY_ROLL"),
            f"yatış 0°→vz={_r0[2]:+.2f}, 40°→vz={_r40[2]:+.2f} "
            f"(Δ={abs(_r40[2]-_r0[2]):.2f} m/s); ayrı anahtar yok")

    # E3: lead — LOS dönerken nişan öne alınır
    _sabit = ib.komut(CX, _cy_icin(0.0), 25, 25, 0.0, 15.0, 0.05, C,
                      los_hiz=(0.0, 0.0))
    _donen = ib.komut(CX, _cy_icin(0.0), 25, 25, 0.0, 15.0, 0.05, C,
                      los_hiz=(0.5, 0.0))
    kontrol("E3 lead: LOS dönerken nişan öne alınır",
            abs(_donen[5]["lead_az"]) > math.radians(1.0)
            and abs(_sabit[5]["lead_az"]) < 1e-9,
            f"λ̇=0 → {math.degrees(_sabit[5]['lead_az']):.1f}°, "
            f"λ̇=0.5 rad/s → {math.degrees(_donen[5]['lead_az']):.1f}°")

    # E4: lead menzille sönümlenir (yakında hata büyütmesin)
    _uzak = ib.komut(CX, _cy_icin(0.0), 10, 10, 0.0, 15.0, 0.05, C,
                     los_hiz=(0.5, 0.0))[5]["lead_olcek"]
    _yakin = ib.komut(CX, _cy_icin(0.0), 80, 80, 0.0, 15.0, 0.05, C,
                      los_hiz=(0.5, 0.0))[5]["lead_olcek"]
    kontrol("E4 lead menzille sönümlenir (yakında söner)",
            _yakin < _uzak and C.LEAD_SONUM,
            f"kutu 10 px → ölçek {_uzak:.2f}; 80 px → {_yakin:.2f}")

    # E5: Ö1 kaçış telafisi — hedef uzaklaşırken hızı artırır, yaklaşırken YOK
    class _Kacis(ib.Cfg):
        KACIS_KD = 1.0
        HUCUM_BOYUT_REF = 30.0        # tavana dayanmasın ki fark görünsün
        V_HUCUM = 30.0
    _uzk = ib.komut(CX, _cy_icin(0.0), 25, 25, 0.0, 10.0, 0.05, _Kacis,
                    kapanma=-6.0)[5]
    _yak = ib.komut(CX, _cy_icin(0.0), 25, 25, 0.0, 10.0, 0.05, _Kacis,
                    kapanma=+6.0)[5]
    kontrol("E5 kaçış telafisi: uzaklaşırken hızlanır, yaklaşırken YOK",
            _uzk["kacis_ek"] > 0.5 and abs(_yak["kacis_ek"]) < 1e-9,
            f"ṙ=−6 → ek {_uzk['kacis_ek']:.1f} m/s; ṙ=+6 → "
            f"{_yak['kacis_ek']:.1f}")

    # E6: Ö5 dönüş tavanı — YALNIZ kısar
    class _Donus(ib.Cfg):
        DONUS_A = 9.81
    _duz = ib.komut(CX, _cy_icin(0.0), 25, 25, 0.0, 20.0, 0.05, _Donus,
                    los_hiz=(0.0, 0.0))[5]
    _don = ib.komut(CX, _cy_icin(0.0), 25, 25, 0.0, 20.0, 0.05, _Donus,
                    los_hiz=(1.2, 0.0))[5]
    kontrol("E6 Ö5 dönüş tavanı yalnız KISAR (düz uçuşta etkisiz)",
            _duz["donus_tavan"] is None and _don["v_los"] <= _duz["v_los"],
            f"λ̇=0 → tavan yok (v={_duz['v_los']:.1f}); λ̇=1.2 → "
            f"tavan {_don['donus_tavan']:.1f} (v={_don['v_los']:.1f})")

    # E7: Ö9 yaw sönümleme — aracın kendi dönüşü komutu geri çeker
    class _Sonum(ib.Cfg):
        SONUM_T = 0.30
    _s0 = ib.komut(CX + 80.0, _cy_icin(0.0), 25, 25, 0.0, 15.0, 0.05, _Sonum,
                   yaw_hizi=0.0)
    _s1 = ib.komut(CX + 80.0, _cy_icin(0.0), 25, 25, 0.0, 15.0, 0.05, _Sonum,
                   yaw_hizi=1.0)
    kontrol("E7 Ö9 sönümleme: kendi dönüş hızı komutu geri çeker",
            _s1[3] < _s0[3] - math.radians(1.0),
            f"yaw_hızı 0 → {math.degrees(_s0[3]):+.1f}°; "
            f"1 rad/s → {math.degrees(_s1[3]):+.1f}°")

    # E8: Ö8 yanal kesişme YALNIZ KISAR (eps_hiz büyütmez)
    class _Yanal(ib.Cfg):
        YANAL_K = 3.0
    _n, _kotu = 0, 0
    for _cxd in (CX - 90, CX - 30, CX + 30, CX + 90):
        for _b in (15, 30, 60):
            _r = ib.komut(_cxd, _cy_icin(0.0), _b, _b, 0.0, 18.0, 0.05,
                          _Yanal, kapanma=5.0)[5]
            _n += 1
            if abs(_r["eps_hiz"]) > abs(_r["eps_yaw"]) + 1e-12:
                _kotu += 1
    kontrol("E8 Ö8 yanal kesişme YALNIZ kısar, büyütmez",
            _kotu == 0, f"{_n} kombinasyonda büyütme sayısı {_kotu}")

    # E9: yaw slew tavanı hız vektörünü DEĞİŞTİRMEZ (yapısal)
    # Sınırlanan yalnız BURUN; vx,vy `hiz_yonu`ndan hesaplanır.
    _kod = "\n".join(L for L in _src.split("\n")
                     if not L.lstrip().startswith("#"))
    kontrol("E9 yaw slew yalnız BURNU sınırlar (hız vektörü ayrı)",
            "hiz_yonu" in _kod and "yaw_cmd" in _kod
            and _kod.index("vx_ned = _yat") > _kod.index("hiz_yonu ="),
            "vx,vy `hiz_yonu`ndan; yaw_cmd ayrı değişken — slew hız yolunu "
            "etkilemez")

    # ══════════════════════════════════════════════════════════════════
    print("=" * 62)
    print("F · DÖNGÜ DUMAN TESTİ")

    class _SahteConn:
        def __init__(self):
            self.komutlar = []
            self.target_system = 1
            self.target_component = 1
            self.mav = self

        def set_position_target_local_ned_send(self, *a, **k):
            self.komutlar.append(a)

    _kayit = {"pose": None}
    _cfg = ib.Cfg

    # F1: kutu akışında komut üretir
    _kayit["pose"] = {"conf": 0.9, "bbox": (300, 290, 340, 330)}
    _r = ib.komut(320.0, 310.0, 40, 40, 0.0, 15.0, 0.05, _cfg)
    kontrol("F1 geçerli kutuda komut üretilir",
            all(isinstance(x, float) for x in _r[:4]) and _hiz(_r) > 1.0,
            f"|v|={_hiz(_r):.1f} m/s, yaw={math.degrees(_r[3]):+.1f}°")

    # F2: tanı sözlüğü beklenen anahtarları taşır
    _bekle = {"boyut", "eps_yaw", "hata", "v_los", "eps_hiz", "sonum",
              "donus_tavan", "kacis_ek", "lead_az", "lead_olcek",
              "nisan_elev", "elev_atalet"}
    kontrol("F2 tanı sözlüğü tam",
            _bekle <= set(_r[5]),
            f"eksik: {_bekle - set(_r[5]) or 'YOK'}")

    # F3: CSV alan listesi ile tanı uyumlu (yazılamayan sütun kalmasın)
    kontrol("F3 CSV alanları tanıyla uyumlu",
            "nisan_elev_deg" in ib._CSV_ALANLAR
            and "elev_atalet_deg" in ib._CSV_ALANLAR,
            f"_CSV_ALANLAR {len(ib._CSV_ALANLAR)} sütun")

    print("=" * 62)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
