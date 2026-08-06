"""tools/kilit_log_analiz.py — Kilit hakem CSV'sini tek sayfada özetler (ADIM 9B).

Sim'siz, SALT-OKUR analiz aracı. CSV yolunu argümandan alır, özeti terminale
basar (dosya OLUŞTURMAZ). Yalnız stdlib: csv, argparse, statistics. Config
import'u YOK — eşikler bağımsız olsun diye burada sabit (aşağıda). Test edilecek
karar mantığı yok; bu bir okuyucu/rapor aracıdır.

Kullanım:
    PYTHONPATH=. python tools/kilit_log_analiz.py logs/kilit_YYYYmmdd_HHMMSS.csv \
        [--bildirim logs/bildirim_YYYYmmdd_HHMMSS.csv]

Kilit CSV kolonları (control/kilit_log.py ile aynı):
    t_sim, faz, gorev_faz, x1,y1,x2,y2, marj, kumulatif_s, kesintisiz_s,
    kilit, kilit_isteri_ok, tespit_dogrulandi, menzil, menzil_ref,
    vx,vy,vz,yaw, los_x,los_y,los_z
gorev_faz değerleri: GPS / VISUAL / ANGAJMAN / VURULDU / DURDU
"""

import argparse
import csv
import statistics

# ── Bağımsız eşikler (config'ten KOPYA, import değil — araç tek başına dursun) ──
KUMULATIF_HEDEF = 5.0    # şartname 6.1.4 kümülatif kilit hedefi (sn)
PENCERE_SN = 10.0        # değerlendirme penceresi (sn)
ANGAJMAN_SN = 3.0        # çarpışma öncesi angajman kesiti (sn)
MARJ_ALT = 1.45         # mesafe tutucu alt bandı (MARJ_REF)
MARJ_UST = 1.75         # mesafe tutucu üst bandı (MARJ_UST)

BEKLENEN_SIRA = ["GPS", "VISUAL", "ANGAJMAN", "VURULDU"]

_CIZGI = "─" * 72


# ───────────────────────────── yardımcılar ──────────────────────────────

def _fnum(row, key):
    """Hücreyi float'a çevir; boş/bozuksa None."""
    v = (row.get(key) or "").strip()
    try:
        return float(v)
    except (ValueError, TypeError):
        return None


def _inum(row, key):
    v = (row.get(key) or "").strip()
    try:
        return int(float(v))
    except (ValueError, TypeError):
        return None


def _yukle(yol):
    with open(yol, newline="") as f:
        return list(csv.DictReader(f))


def _baslik(no, ad):
    print("\n" + _CIZGI)
    print(f"{no}) {ad}")
    print(_CIZGI)


# ─────────────────────────── 1) FAZ GEÇİŞLERİ ───────────────────────────

def faz_gecisleri(rows):
    _baslik(1, "FAZ GEÇİŞLERİ (gorev_faz)")
    gecisler = []
    onceki = None
    sira = []
    for r in rows:
        gf = (r.get("gorev_faz") or "").strip()
        if not gf:
            continue
        if gf != onceki:
            t = _fnum(r, "t_sim")
            if onceki is not None:
                gecisler.append((t, onceki, gf))
            sira.append(gf)
            onceki = gf

    if not sira:
        print("  (gorev_faz kolonu boş — faz bilgisi yok)")
        return
    if not gecisler:
        print(f"  Tek faz görüldü: {sira[0]} (geçiş yok)")
    for t, a, b in gecisler:
        ts = f"{t:8.2f}" if t is not None else "     ?  "
        print(f"  t={ts}   {a:>9}  →  {b}")

    goruldu = " → ".join(sira)
    print(f"\n  Görülen sıra : {goruldu}")
    # Beklenen sırayı alt-dizi olarak içeriyor mu?
    it = iter(sira)
    tam = all(f in it for f in BEKLENEN_SIRA)
    print(f"  Beklenen     : {' → '.join(BEKLENEN_SIRA)}")
    print(f"  Değerlendirme: {'✓ beklenen sıra korunuyor' if tam else '⚠ beklenen sıra eksik/bozuk'}")


# ─────────────────────────── 2) KİLİT SEGMENTLERİ ───────────────────────

def kilit_segmentleri(rows):
    _baslik(2, "KİLİT SEGMENTLERİ (kilit=1 ardışık bloklar + köprülenen boşluklar)")
    segmentler = []      # (t_bas, t_bit, kare_sayisi)
    acik = None          # [t_bas, t_son, n]
    for r in rows:
        k = _inum(r, "kilit")
        t = _fnum(r, "t_sim")
        if k == 1 and t is not None:
            if acik is None:
                acik = [t, t, 1]
            else:
                acik[1] = t
                acik[2] += 1
        else:
            if acik is not None:
                segmentler.append(tuple(acik))
                acik = None
    if acik is not None:
        segmentler.append(tuple(acik))

    if not segmentler:
        print("  (kilit=1 satır yok — hiç kilit oluşmamış)")
        return
    toplam = 0.0
    for i, (a, b, n) in enumerate(segmentler, 1):
        sure = b - a
        toplam += sure
        print(f"  #{i:<2} [{a:8.2f} → {b:8.2f}]  süre {sure:6.2f} s  ({n} kare)")
    print(f"  → {len(segmentler)} segment, toplam kilitli süre {toplam:.2f} s")

    # Köprülenen boşluklar: kilit=0 AMA kesintisiz_s>0 (süreklilik span'i hâlâ diri)
    kopru = []           # [t_bas, t_son, n]
    acik = None
    for r in rows:
        k = _inum(r, "kilit")
        kes = _fnum(r, "kesintisiz_s")
        t = _fnum(r, "t_sim")
        koprulu = (k == 0 and kes is not None and kes > 0.0 and t is not None)
        if koprulu:
            if acik is None:
                acik = [t, t, 1]
            else:
                acik[1] = t
                acik[2] += 1
        else:
            if acik is not None:
                kopru.append(tuple(acik))
                acik = None
    if acik is not None:
        kopru.append(tuple(acik))

    if kopru:
        print("\n  Köprülenen boşluklar (kilit=0 ama süreklilik korunmuş):")
        for a, b, n in kopru:
            print(f"    [{a:8.2f} → {b:8.2f}]  {b - a:5.2f} s  ({n} kare)  ← kare toleransı köprüledi")
    else:
        print("\n  Köprülenen boşluk yok (segmentler kesintisiz).")


# ─────────────────────────── 3) BEYAN ARALIĞI ───────────────────────────

def beyan_araligi(rows, bildirim_yol=None):
    _baslik(3, f"BEYAN ARALIĞI (kümülatif ≥ {KUMULATIF_HEDEF:.1f} s)")
    hedef_row = None
    en_yakin = None
    en_yakin_kum = -1.0
    for r in rows:
        kum = _fnum(r, "kumulatif_s")
        if kum is None:
            continue
        if kum > en_yakin_kum:
            en_yakin_kum, en_yakin = kum, r
        if hedef_row is None and kum >= KUMULATIF_HEDEF:
            hedef_row = r

    hedef = hedef_row or en_yakin
    if hedef is None:
        print("  (kumulatif_s kolonu boş — beyan hesaplanamıyor)")
        return None
    bitis_t = _fnum(hedef, "t_sim")
    kum_val = _fnum(hedef, "kumulatif_s")

    # baslangic'ı pencereden geri kur: bitis - 10 s içindeki ilk kilitli kare.
    pencere_bas = bitis_t - PENCERE_SN
    baslangic_t = None
    for r in rows:
        t = _fnum(r, "t_sim")
        if t is None or t < pencere_bas or t > bitis_t:
            continue
        if _inum(r, "kilit") == 1:
            baslangic_t = t
            break

    ulasti = hedef_row is not None
    print(f"  {'✓ hedefe ULAŞILDI' if ulasti else '⚠ hedefe ULAŞILAMADI (en yakın satır)'}")
    print(f"  bitis_t      : {bitis_t:8.2f}   (kümülatifin {'5.0 gördüğü' if ulasti else 'zirve yaptığı'} an)")
    if baslangic_t is not None:
        print(f"  baslangic_t  : {baslangic_t:8.2f}   (pencere içi ilk kilitli kare; {PENCERE_SN:.0f} s geri kurulmuş)")
    else:
        print(f"  baslangic_t  :      ?     (pencere içinde kilitli kare bulunamadı)")
    print(f"  kümülatif    : {kum_val:8.2f} s")

    if bildirim_yol:
        _bildirim_capraz(bildirim_yol, bitis_t)
    return bitis_t


def _bildirim_capraz(bildirim_yol, kilit_bitis_t):
    print("\n  Bildirim CSV çapraz doğrulama:")
    try:
        brows = _yukle(bildirim_yol)
    except OSError as e:
        print(f"    ⚠ bildirim CSV okunamadı: {e}")
        return
    saglanan = [r for r in brows if (r.get("sart_saglandi") or "").strip() == "1"]
    if not saglanan:
        print("    ⚠ sart_saglandi=1 satır YOK — sunucuya geçerli beyan gitmemiş.")
        return
    for r in saglanan:
        bt = _fnum(r, "bitis_t")
        bas = _fnum(r, "baslangic_t")
        kum = _fnum(r, "kumulatif_sn")
        yakin = (bt is not None and kilit_bitis_t is not None
                 and abs(bt - kilit_bitis_t) <= 0.5)
        isaret = "✓ kilit CSV ile örtüşüyor" if yakin else "· (bitiş kilit CSV'den farklı)"
        print(f"    beyan [{bas:8.2f} → {bt:8.2f}]  kümülatif {kum:.2f} s  sart_saglandi=1  {isaret}")


# ────────────────────── 4) ANGAJMAN İSPAT KESİTİ ────────────────────────

def angajman_ispat(rows):
    _baslik(4, f"ANGAJMAN İSPAT KESİTİ (gorev_faz=ANGAJMAN, son {ANGAJMAN_SN:.0f} s)")
    ang = [r for r in rows if (r.get("gorev_faz") or "").strip() == "ANGAJMAN"]
    if not ang:
        print("  (ANGAJMAN fazı hiç görülmemiş)")
        return
    t_son = max((_fnum(r, "t_sim") or 0.0) for r in ang)
    kesit = [r for r in ang if (_fnum(r, "t_sim") or 0.0) >= t_son - ANGAJMAN_SN]
    print(f"  ANGAJMAN toplam {len(ang)} kare; son {ANGAJMAN_SN:.0f} s kesiti {len(kesit)} kare "
          f"[{(_fnum(kesit[0],'t_sim') or 0):.2f} → {t_son:.2f}]")

    # (a) Komut hedefe yönelik mi: dot(v, los) > 0
    dots = []
    for r in kesit:
        vx, vy, vz = _fnum(r, "vx"), _fnum(r, "vy"), _fnum(r, "vz")
        lx, ly, lz = _fnum(r, "los_x"), _fnum(r, "los_y"), _fnum(r, "los_z")
        if None in (vx, vy, vz, lx, ly, lz):
            continue
        dots.append(vx * lx + vy * ly + vz * lz)
    print("\n  (a) Komut yönü — iç çarpım <v, los> (>0 = hedefe doğru):")
    if dots:
        pozitif = sum(1 for d in dots if d > 0)
        oran = 100.0 * pozitif / len(dots)
        print(f"      örnek {len(dots)}  |  ortalama {statistics.mean(dots):+.3f}  |  "
              f"pozitif %{oran:.0f}")
        print(f"      → {'✓ komutlar hedefe yönelik' if oran >= 70 else '⚠ komut yönü tutarsız'}")
    else:
        print("      (vx/vy/vz veya los_* boş — iç çarpım hesaplanamadı)")

    # (b) Mesafe (menzil) sistematik azalıyor mu
    ts, ms = [], []
    for r in kesit:
        t, m = _fnum(r, "t_sim"), _fnum(r, "menzil")
        if t is not None and m is not None:
            ts.append(t)
            ms.append(m)
    print("\n  (b) Mesafe eğilimi (menzil):")
    if len(ms) >= 2:
        mt = statistics.mean(ts)
        var = sum((t - mt) ** 2 for t in ts)
        egim = (sum((t - mt) * (m - statistics.mean(ms)) for t, m in zip(ts, ms)) / var
                if var > 0 else 0.0)
        dusus = sum(1 for i in range(1, len(ms)) if ms[i] < ms[i - 1])
        mon_oran = 100.0 * dusus / (len(ms) - 1)
        print(f"      {ms[0]:.2f} m → {ms[-1]:.2f} m  |  lineer eğim {egim:+.3f} m/s  |  "
              f"monoton azalma %{mon_oran:.0f}")
        sistematik = egim < 0 and mon_oran >= 70
        print(f"      → {'✓ mesafe sistematik azalıyor' if sistematik else '⚠ mesafe düzenli azalmıyor'}")
    else:
        print("      (menzil kolonu yetersiz — eğilim hesaplanamadı)")


# ─────────────────────── 5) MARJ-ZAMAN ÖZETİ ────────────────────────────

def marj_ozet(rows):
    _baslik(5, f"MARJ-ZAMAN ÖZETİ (VISUAL fazı, band {MARJ_ALT}–{MARJ_UST})")
    marjlar = []
    for r in rows:
        if (r.get("gorev_faz") or "").strip() != "VISUAL":
            continue
        m = _fnum(r, "marj")
        if m is not None and m > 0.0:       # marj=0 → tespitsiz kare, hariç
            marjlar.append(m)
    if not marjlar:
        print("  (VISUAL fazında tespitli (marj>0) kare yok)")
        return
    bandda = sum(1 for m in marjlar if MARJ_ALT <= m <= MARJ_UST)
    oran = 100.0 * bandda / len(marjlar)
    print(f"  örnek {len(marjlar)} kare")
    print(f"  min {min(marjlar):.2f}  |  ort {statistics.mean(marjlar):.2f}  |  max {max(marjlar):.2f}")
    print(f"  band içi ({MARJ_ALT}–{MARJ_UST}): %{oran:.0f}  ({bandda}/{len(marjlar)} kare)")
    print(f"  → {'✓ mesafe tutucu bandı iyi koruyor' if oran >= 50 else '⚠ marj bandın dışında baskın'}")


# ──────────────────────────────── ana ───────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Kilit hakem CSV'si tek sayfa özet")
    ap.add_argument("kilit_csv", help="logs/kilit_YYYYmmdd_HHMMSS.csv")
    ap.add_argument("--bildirim", help="logs/bildirim_YYYYmmdd_HHMMSS.csv (opsiyonel)")
    a = ap.parse_args()

    rows = _yukle(a.kilit_csv)
    print(_CIZGI)
    print(f"KİLİT LOG ANALİZİ — {a.kilit_csv}")
    print(f"  {len(rows)} veri satırı")
    if rows:
        t0 = _fnum(rows[0], "t_sim")
        t1 = _fnum(rows[-1], "t_sim")
        if t0 is not None and t1 is not None:
            print(f"  zaman aralığı: {t0:.2f} → {t1:.2f} s  ({t1 - t0:.1f} s)")

    faz_gecisleri(rows)
    kilit_segmentleri(rows)
    beyan_araligi(rows, a.bildirim)
    angajman_ispat(rows)
    marj_ozet(rows)
    print("\n" + _CIZGI)


if __name__ == "__main__":
    main()
