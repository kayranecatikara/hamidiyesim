"""
tools/analiz/ortak.py — analiz scriptlerinin paylaştığı CSV yükleme + istatistik.

Tasarım notu: pandas KULLANILMAZ (depoda bağımlılık değil). csv + istatistik
elle yapılır; veri boyutu bir uçuşta birkaç bin satır, buna fazlasıyla yeter.
"""

import csv
import glob
import math
import os
import sys

PROJE_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LOG_DIZIN = os.path.join(PROJE_KOK, "logs")

# Yalnız istatistik kurulamayacak kadar kısa dosyaları eler. DÜŞÜK tutulmalı:
# gerçek bir görsel faz 1 saniyeden kısa sürebiliyor (30 Hz'te ~30 kare) ve o
# KISALIK zaten bulgunun kendisidir — eleyip gizlememeli. Test koşuları artık
# AVCI_LEAD_LOG_DIR ile geçici dizine yazdığından logs/ kirlenmez.
MIN_SATIR = 5


def _sayi(s):
    """CSV hücresi → float veya None (boş hücre, '', 'None' hepsi None)."""
    if s is None or s == "" or s == "None":
        return None
    try:
        return float(s)
    except ValueError:
        return None


def yukle(yol):
    """Bir CSV → satır sözlükleri listesi. Sayısal alanlar float'a çevrilir,
    `durum`/`bbox`/`mod` metin kalır."""
    metin_alanlar = {"durum", "bbox", "mod"}
    satirlar = []
    with open(yol, newline="") as f:
        for ham in csv.DictReader(f):
            s = {}
            for k, v in ham.items():
                s[k] = v if k in metin_alanlar else _sayi(v)
            satirlar.append(s)
    return satirlar


def dosyalari_bul(argv):
    """Komut satırı argümanları → CSV yolları. Argüman yoksa logs/ içindeki EN
    YENİ dosyayı seçer."""
    yollar = [a for a in argv if a.endswith(".csv")]
    if not yollar:
        hepsi = sorted(glob.glob(os.path.join(LOG_DIZIN, "visual_lead_*.csv")))
        if not hepsi:
            print(f"HATA: {LOG_DIZIN} altında visual_lead_*.csv yok.\n"
                  f"Önce bir test uçuşu yapılmalı (bkz. CLAUDE.md §3).")
            sys.exit(1)
        yollar = [hepsi[-1]]
    return yollar


def yukle_hepsi(argv):
    """dosyalari_bul + yukle; kısa/boş dosyaları uyarıyla eler.
    Dönüş: [(yol, satirlar), ...]"""
    cikti = []
    for y in dosyalari_bul(argv):
        s = yukle(y)
        if len(s) < MIN_SATIR:
            print(f"  ATLANDI  {os.path.basename(y)} — yalnız {len(s)} satır "
                  f"(test çıktısı olabilir, eşik {MIN_SATIR})")
            continue
        cikti.append((y, s))
    if not cikti:
        print("HATA: analiz edilebilir uçuş logu bulunamadı.")
        sys.exit(1)
    return cikti


# ── İstatistik ──

def ist(degerler):
    """None'ları eleyip temel istatistik döner (yoksa None)."""
    v = sorted(x for x in degerler if x is not None)
    if not v:
        return None
    n = len(v)
    ort = sum(v) / n
    std = math.sqrt(sum((x - ort) ** 2 for x in v) / n) if n > 1 else 0.0
    return {"n": n, "ort": ort, "std": std, "min": v[0], "max": v[-1],
            "med": v[n // 2], "p05": v[int(0.05 * n)], "p95": v[int(0.95 * n)]}


def ist_yaz(ad, s, birim="", genislik=30):
    """ist() çıktısını tek satırda yazar."""
    if s is None:
        print(f"  {ad:<{genislik}} veri yok")
        return
    print(f"  {ad:<{genislik}} n={s['n']:<5d} ort={s['ort']:+8.3f}{birim} "
          f"med={s['med']:+8.3f} std={s['std']:7.3f} "
          f"[{s['min']:+.2f} … {s['max']:+.2f}]")


def kolon(satirlar, ad, filtre=None):
    """Bir kolonun değerleri (None'lar dahil). filtre(satir) verilirse süzer."""
    return [s.get(ad) for s in satirlar if filtre is None or filtre(s)]


def bantla(satirlar, bant_kolonu, kenarlar):
    """Satırları bir kolona göre bantlara ayırır.
    Dönüş: [(etiket, [satır...]), ...]"""
    gruplar = []
    for i in range(len(kenarlar) - 1):
        alt, ust = kenarlar[i], kenarlar[i + 1]
        grup = [s for s in satirlar
                if s.get(bant_kolonu) is not None and alt <= s[bant_kolonu] < ust]
        gruplar.append((f"{alt:g}-{ust:g}", grup))
    return gruplar


def baslik(metin):
    print()
    print("=" * 78)
    print(f"  {metin}")
    print("=" * 78)


def altbaslik(metin):
    print()
    print(f"── {metin} " + "─" * max(0, 74 - len(metin)))


def dosya_ozeti(yol, satirlar):
    """Dosyanın kimliği + durum kodu dağılımı."""
    ad = os.path.basename(yol)
    durumlar = {}
    for s in satirlar:
        d = s.get("durum") or "(boş)"
        durumlar[d] = durumlar.get(d, 0) + 1
    sure = None
    ts = [s["t_ros"] for s in satirlar if s.get("t_ros") is not None]
    if len(ts) >= 2:
        sure = max(ts) - min(ts)
    print(f"\n▸ {ad}  —  {len(satirlar)} kare"
          + (f", {sure:.1f} s sim süresi" if sure else ""))
    sirali = sorted(durumlar.items(), key=lambda kv: -kv[1])
    print("   durum: " + "  ".join(f"{k}={v}" for k, v in sirali))
    # Doğruluk kolonları dolu mu — yoksa analizin yarısı çalışmaz
    truth_var = sum(1 for s in satirlar if s.get("menzil_gercek_gz_m") is not None)
    if truth_var == 0:
        print("   ⚠ UYARI: ground truth kolonları BOŞ. gz_truth çalışmamış "
              "(AVCI_TRUTH=off mu? Gazebo world adı 'avci' mi?)")
    elif truth_var < len(satirlar) * 0.5:
        print(f"   ⚠ UYARI: ground truth yalnız {truth_var}/{len(satirlar)} karede var")
    return truth_var > 0
