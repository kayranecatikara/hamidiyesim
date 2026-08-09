#!/usr/bin/env python3
"""GÖREV ÖLÇÜTÜ — bir uçuşu GÖREV gibi değerlendirir, olay gibi değil.

⚠ NEDEN VAR (2026-08-09, kullanıcı uçuşta yakaladı):
Bir yapılandırmayı "8/9 vuruş, dikey kaçırma 0.16 m" diye en iyi ilan ettim.
Ölçütlerim vuruş/ıska ve en yakın andaki dikey sapmaydı. İkisi de doğruydu —
ve ikisi de GÖREVİ görmüyordu. Kullanıcı izleyerek gördü: araç hedefin üstüne
çıkıp FREN YAPIYOR, görsel temas kopuyor, geri dönüp tekrar deniyor, birkaç
denemeden sonra bir şekilde temas oluyor. Ölçütüm bunu "başarı" sayıyordu.

Sonradan sayıya döküldü: kutu >40 px iken komut edilen hız 9 koşunun 5'inde
0.0 m/s'ye iniyordu ve ilk vuruş medyanı 61 s idi (eski yapılandırma ~46 s).
Yani vuruş SAYISI arttı, GÖREV kötüleşti.

DERS: bir ölçüt neyi ölçmediğini sana söylemez. Bu araç, kullanıcının
saydığı soruları doğrudan yanıtlar:

  1. Takip başladıktan kaç saniye sonra vurdu?          (t_vurus)
  2. Vurana kadar kaç başarısız geçiş yaptı?            (basarisiz_gecis)
  3. Kaç kez görsel temas koptu?                        (temas_kopmasi)
  4. Hedefe yaklaşırken FREN yaptı mı?                  (fren_olayi)
  5. Ne kadar süre hedefe yaklaşmadan oyalandı?         (bosa_sure)

Kullanım:
    python3 tools/gorev_olcut.py <kosu_dizini> [...]
    python3 tools/gorev_olcut.py --kampanya <kampanya_dizini>
"""
import csv
import glob
import json
import math
import os
import statistics as st
import sys

# Bir "geçiş" (pass): mesafe bu eşiğin altına inip sonra tekrar üstüne çıkması.
YAKIN_ESIK = 8.0      # m — bu mesafenin altı "hücum denemesi" sayılır
UZAK_ESIK = 20.0      # m — buraya geri açılırsa deneme BAŞARISIZ bitmiş demektir
FREN_HIZ = 6.0        # m/s — hedefe 10 m'den yakınken bunun altı FREN sayılır
FREN_MESAFE = 10.0    # m


def sayi(x):
    try:
        v = float(x)
        return v if v == v else None
    except (TypeError, ValueError):
        return None


def gorev_olc(dizin, logs_dizini=None):
    """Bir koşuyu görev ölçütleriyle değerlendirir."""
    logs_dizini = logs_dizini or os.path.expanduser("~/projects/avci_sim/logs")
    ks = os.path.join(dizin, "kosu1.csv")
    if not os.path.exists(ks):
        return None
    try:
        rows = list(csv.DictReader(open(ks)))
    except OSError:
        return None
    p = []
    for r in rows:
        t = sayi(r.get("t"))
        v = [sayi(r.get(k)) for k in
             ("plane_x", "plane_y", "plane_z", "iris_x", "iris_y", "iris_z")]
        if t is None or any(x is None for x in v):
            continue
        d = math.dist((v[0], v[1], v[2]), (v[3], v[4], v[5]))
        p.append((t, d, v[3], v[4], int(r.get("imha") or 0)))
    if len(p) < 10:
        return None
    t0 = p[0][0]
    o = {"dizin": os.path.basename(dizin)}

    # 1) İLK VURUŞ SÜRESİ — takip başlangıcından
    vurus = next((x for x in p if x[4]), None)
    o["vuruldu"] = vurus is not None
    o["t_vurus"] = round(vurus[0] - t0, 1) if vurus else None
    o["sure"] = round(p[-1][0] - t0, 1)

    # 2) BAŞARISIZ GEÇİŞ — yakına indi, vuramadı, geri açıldı
    #    (vuruşla biten son yaklaşma sayılmaz)
    gecis, icerde, basarisiz = 0, False, 0
    for t, d, *_ in p:
        if not icerde and d < YAKIN_ESIK:
            icerde = True
            gecis += 1
        elif icerde and d > UZAK_ESIK:
            icerde = False
            basarisiz += 1
    o["yaklasma"] = gecis
    o["basarisiz_gecis"] = basarisiz

    # 3) FREN — KOMUT EDİLEN hıza bakılır, yer hızına DEĞİL.
    # ⚠ Bunu önce yer hızıyla ölçtüm ve "fren yok" çıktı; yanlıştı. Araç
    # 18 m/s'den frene bastığında momentum yer hızını bir süre yüksek tutuyor,
    # olay ölçüme yakalanmıyor. Güdümün NE İSTEDİĞİ tek dürüst sinyal.
    # Kaynak: bbox_ibvs logundaki v_los (yalnız IBVS/TERMINAL satırları;
    # KURTARMA/KUTU_YOK satırlarında hız zaten anlamsız).
    bas, bit = p[0][0] - 30, p[-1][0] + 15
    loglar = [y for y in glob.glob(os.path.join(logs_dizini, "bbox_ibvs_*.csv"))
              if bas <= os.path.getmtime(y) <= bit]
    fren, fren_kare, onceki = 0, 0, False
    for y in sorted(loglar, key=os.path.getmtime):
        try:
            gr = list(csv.DictReader(open(y)))
        except OSError:
            continue
        for x in gr:
            if x.get("durum") not in ("IBVS", "TERMINAL"):
                onceki = False
                continue
            b, v = sayi(x.get("boyut")), sayi(x.get("v_los"))
            if b is None or v is None:
                continue
            # kutu > 40 px ≈ 4 m'den yakın; orada 6 m/s altı komut = FREN
            simdi = (b > 40.0 and v < FREN_HIZ)
            if simdi:
                fren_kare += 1
                if not onceki:
                    fren += 1
            onceki = simdi
    o["fren_olayi"] = fren
    o["fren_sure"] = round(fren_kare * 0.05, 1)      # güdüm döngüsü 20 Hz

    # 4) GÖRSEL TEMAS KOPMASI — koşu penceresindeki güdüm logu sayısı
    o["gorsel_faz"] = len(loglar)
    o["temas_kopmasi"] = max(0, len(loglar) - 1)

    # 5) EN YAKIN GEÇİŞ
    o["en_yakin"] = round(min(x[1] for x in p), 2)
    return o


def yazdir(kayitlar):
    print(f"{'koşu':14} {'VURUŞ':>6} {'süre':>6} {'başarısız':>10} "
          f"{'fren':>6} {'fren s':>7} {'temas kopması':>14}")
    print("─" * 76)
    for o in kayitlar:
        sure_yazi = ("%.0f s" % o["t_vurus"]) if o["t_vurus"] else "—"
        vur_yazi = "✓" if o["vuruldu"] else "✗"
        print(f"{o['dizin']:14} {vur_yazi:>6} {sure_yazi:>6} "
              f"{o['basarisiz_gecis']:10d} "
              f"{o['fren_olayi']:6d} {o['fren_sure']:6.1f}s "
              f"{o['temas_kopmasi']:14d}")
    vur = [o for o in kayitlar if o["vuruldu"]]
    print("─" * 76)
    print(f"  vuruş {len(vur)}/{len(kayitlar)}"
          + (f"   ilk vuruş medyanı {st.median([o['t_vurus'] for o in vur]):.0f} s"
             if vur else "")
          + f"   başarısız geçiş medyanı "
            f"{st.median([o['basarisiz_gecis'] for o in kayitlar]):.0f}"
          + f"   fren süresi medyanı "
            f"{st.median([o['fren_sure'] for o in kayitlar]):.1f} s")
    print("\n  ⚠ İYİ bir görev: vuruş ✓, süre KISA, başarısız geçiş 0,")
    print("    fren 0, temas kopması 0. Yalnız 'vuruş ✓' bakmak yanıltır.")


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--kampanya":
        dizinler = sorted(glob.glob(os.path.join(sys.argv[2], "*_k*")))
    else:
        dizinler = sys.argv[1:]
    if not dizinler:
        print(__doc__)
        raise SystemExit(1)
    kayitlar = [o for o in (gorev_olc(d) for d in dizinler) if o]
    if not kayitlar:
        print("değerlendirilecek koşu bulunamadı")
        raise SystemExit(1)
    yazdir(kayitlar)


if __name__ == "__main__":
    main()
