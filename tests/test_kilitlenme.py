"""
DEPRECATED — güncel testler tests/test_kilit_kriteri.py ve tests/test_kilit_sure.py.
Bu dosya eski %5 eşiği ve pencere-sıfırlama davranışını kodlar; kilit kriteri artık
config.ESIK_BILDIRIM (0.06) + vision/kilit_kriteri, süre ise control/kilit_sure ile
yürür (RESET_POLICY=kumulatif_korunur). K2a/K7 bu yeni kurallarla bilinçli olarak
uyumsuzdur; onarılmaya çalışılmaz.

tests/test_kilitlenme.py — Şartname 6.1.4 kilitlenme mantığı kabul kriterleri.

Gazebo'suz, saf mantık. Kullanım: python3 -m tests.test_kilitlenme

Kapsam:
  K1  AV (sarı kutu) geometrisi = Şekil 2 boşlukları (yatay %25, dikey %10)
  K2  boyut şartı: yatay VEYA dikeyde ≥ %5 (biri yeterli)
  K3  merkez AV dışındaysa anlık kilit YOK (boyut sağlansa bile)
  K4  faz: tespit yok → BEKLE; küçük hedef → TAKIBE_GECIS; %5 sonrası → TAKIP
  K5  10 sn pencerede 5 sn kümülatif kilit → kilit isteri OK + TERMINAL fazı
  K6  kümülatif kesintili olabilir (kayan pencere) — toplam süre sayılır
  K7  uzun temas kaybı → sıfırlanır (fazlar ve pencere temizlenir)
"""

from control.kilitlenme import KilitTakip, KilitCfg

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def _merkez_bbox(w, h, cx=320, cy=240):
    """Verilen genişlik/yükseklikte, merkezi (cx,cy) olan bbox."""
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def main():
    print("Şartname 6.1.4 kilitlenme mantığı")
    print("=" * 60)

    # ── K1: AV geometrisi (640×480) ──
    kt = KilitTakip(640, 480)
    kontrol("K1 AV kutusu = (160,48,480,432)", kt.av == (160, 48, 480, 432),
            f"av={kt.av}")

    # ── K2: boyut şartı — yatayda %5 (32 px) yeterli, dikey küçük olsa da ──
    kt = KilitTakip(640, 480)
    d = kt.guncelle(_merkez_bbox(32, 4), now=1.0)   # %5 yatay, %0.8 dikey
    kontrol("K2a yatay %5 → boyut_ok", d["boyut_ok"], f"kap_x={d['kaplama_x']:.3f}")
    d = kt.guncelle(_merkez_bbox(20, 4), now=1.05)  # %3.1 yatay, %0.8 dikey
    kontrol("K2b ikisi de <%5 → boyut YOK", not d["boyut_ok"],
            f"kap_x={d['kaplama_x']:.3f} kap_y={d['kaplama_y']:.3f}")

    # ── K3: merkez AV dışında → anlık kilit yok ──
    kt = KilitTakip(640, 480)
    # Yeterli boyut ama merkez sol kenarda (AV dışı: cx=100 < 160)
    d = kt.guncelle(_merkez_bbox(60, 60, cx=100, cy=240), now=1.0)
    kontrol("K3 merkez AV dışında → anlık kilit YOK",
            d["boyut_ok"] and not d["merkez_av_icinde"] and not d["anlik_kilit"],
            f"merkez_ic={d['merkez_av_icinde']}")

    # ── K4: faz akışı ──
    kt = KilitTakip(640, 480)
    d0 = kt.guncelle(None, now=0.0)
    kontrol("K4a tespit yok → BEKLE", d0["faz"] == "BEKLE", d0["faz"])
    d1 = kt.guncelle(_merkez_bbox(10, 8), now=0.1)   # çok küçük, <%5
    kontrol("K4b küçük hedef → TAKIBE_GECIS", d1["faz"] == "TAKIBE_GECIS", d1["faz"])
    d2 = kt.guncelle(_merkez_bbox(40, 40), now=0.2)  # ≥%5, merkezde
    kontrol("K4c boyut şartı sağlandı → TAKIP", d2["faz"] == "TAKIP", d2["faz"])

    # ── K5: 5 sn kümülatif kilit → kilit isteri + TERMINAL ──
    kt = KilitTakip(640, 480)
    t = 0.0
    bbox = _merkez_bbox(40, 40)          # merkezde, ≥%5 → anlık kilit
    son = None
    for _ in range(140):                 # ~140 kare × 0.05 s ≈ 7 s
        t += 0.05
        son = kt.guncelle(bbox, now=t)
    kontrol("K5a kümülatif ≥ 5 s", son["kumulatif_s"] >= 5.0,
            f"kumulatif={son['kumulatif_s']:.2f}s")
    kontrol("K5b kilit isteri OK + TERMINAL",
            son["kilit_isteri_ok"] and son["faz"] == "TERMINAL", son["faz"])

    # ── K6: kesintili kilit — kayan pencere toplamı ──
    # 3 s kilit + 4 s boşluk (kilit penceresi dışına düşer) + 3 s kilit → son
    # 10 s pencerede yaklaşık 3+3=6 s? Hayır: 4 s boşluk sonrası ilk 3 s hâlâ
    # pencere içinde olabilir. Bu testte SADECE kesintili birikimin çalıştığını
    # (tek parça şart olmadığını) doğruluyoruz: 2 s + 2 s = 4 s < 5 s, sonra +2 s.
    kt = KilitTakip(640, 480)
    t = 0.0
    def kilitle(sure):
        nonlocal t
        d = None
        n = int(sure / 0.05)
        for _ in range(n):
            t += 0.05
            d = kt.guncelle(bbox, now=t)
        return d
    def bosta(sure):
        nonlocal t
        d = None
        n = int(sure / 0.05)
        for _ in range(n):
            t += 0.05
            d = kt.guncelle(_merkez_bbox(10, 8), now=t)  # tespit var ama <%5 → kilit yok
        return d
    kilitle(2.0); bosta(1.0); d = kilitle(2.0)
    kontrol("K6a kesik kesik 2+2 s ≈ 4 s < 5 s", 3.5 <= d["kumulatif_s"] <= 4.5,
            f"kumulatif={d['kumulatif_s']:.2f}s")
    d = kilitle(2.0)
    kontrol("K6b +2 s → toplam ≥ 5 s, isteri OK", d["kilit_isteri_ok"],
            f"kumulatif={d['kumulatif_s']:.2f}s")

    # ── K7: uzun temas kaybı → sıfırlama ──
    kt = KilitTakip(640, 480)
    t = 0.0
    kilitli = _merkez_bbox(40, 40)
    d = None
    for _ in range(40):
        t += 0.05
        d = kt.guncelle(kilitli, now=t)
    onceki_faz = d["faz"]
    # SIFIRLA_S (varsayılan 3 s) üzeri tespitsiz zaman geçir
    t += KilitCfg.SIFIRLA_S + 1.0
    d = kt.guncelle(None, now=t)
    kontrol("K7 uzun kayıp → BEKLE + kümülatif 0",
            d["faz"] == "BEKLE" and d["kumulatif_s"] == 0.0,
            f"önce={onceki_faz} sonra={d['faz']} kum={d['kumulatif_s']:.2f}")

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
