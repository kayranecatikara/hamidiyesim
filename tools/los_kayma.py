#!/usr/bin/env python3
"""los_kayma.py — Görsel fazın TEK objektif ölçütü: LOS açısı sabit kaldı mı?

NEDEN BU ÖLÇÜT: Çarpışma rotasının tanımı, iki cisim arasındaki görüş hattı
açısının DEĞİŞMEMESİDİR (menzil kısalır, açı durur). Açı dönüyorsa çarpma
olmaz — ne kadar yaklaştığın önemli değil.

2026-08-07 uçuşlarında ölçüldü:
    vuran faz  : LOS yükselişi 9.9 m'den 0.5 m'ye kadar −14.9° ± 0.3°  (KİLİTLİ)
    ıskalayan  : +8.3° → −19.0°  (27° kaydı)
"En yakın menzil" ikisinde de 0.3-0.9 m; o sayı ayırt ETMİYOR, bu ayırt ediyor.

KULLANIM
    python3 tools/los_kayma.py                    # bugünün tüm fazları
    python3 tools/los_kayma.py 20260807_1754      # tek oturum (ön ek)
    python3 tools/los_kayma.py --son 20           # son N faz

ÖLÇÜM BANDI: menzil 12 m → 2 m arası. Üstü devir gürültüsü, altı zaten
geometrik olarak patlar (λ̇ ∝ 1/R) ve düzeltilemez.
"""

import csv
import glob
import math
import os
import statistics as st
import sys

_LOGS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")

BANT_UST = 12.0    # m; ölçüm bandının üst sınırı
BANT_ALT = 2.0     # m; alt sınır (altında λ̇ zorunlu olarak patlar)
IYI_ESIK = 5.0     # °; bu kaymanın altı "kilitli" sayılır


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def faz_olc(yol):
    """Bir görsel faz logunu ölç. Dönüş: dict ya da None (veri yetersiz)."""
    with open(yol, newline="") as fh:
        satirlar = list(csv.DictReader(fh))
    if len(satirlar) < 10:
        return None

    durumlar = [s.get("durum") for s in satirlar]
    sonuc = ("VURDU" if "vuruldu" in durumlar
             else "kayıp" if durumlar.count("tespit_yok") >= 15
             else "ıska")

    bant = []          # ölçüm bandındaki (menzil, los_elev)
    menziller = []
    for s in satirlar:
        m = _f(s.get("menzil_gercek_m"))
        if m is not None:
            menziller.append(m)
        e = _f(s.get("los_elev_deg"))
        if m is not None and e is not None and BANT_ALT <= m <= BANT_UST:
            bant.append((m, e))

    if len(bant) < 5:
        return {"ad": os.path.basename(yol)[-10:-4], "sonuc": sonuc,
                "n": len(bant), "kayma": None, "std": None,
                "en_yakin": min(menziller) if menziller else None,
                "devir": menziller[0] if menziller else None}

    # Menzile göre sırala (uzaktan yakına) — zaman sırası değil, çünkü
    # menzil monoton kapanmayabilir (fly-past sonrası geri açılır).
    bant.sort(key=lambda t: -t[0])
    aci = [e for _, e in bant]
    kayma = max(aci) - min(aci)          # bandın tamamındaki toplam salınım
    egim = aci[-1] - aci[0]              # uzaktan yakına net kayma (işaretli)

    return {"ad": os.path.basename(yol)[-10:-4], "sonuc": sonuc,
            "n": len(bant), "kayma": kayma, "egim": egim,
            "std": st.pstdev(aci) if len(aci) > 1 else 0.0,
            "en_yakin": min(menziller) if menziller else None,
            "devir": menziller[0] if menziller else None}


def main():
    argv = sys.argv[1:]
    son_n = None
    onek = ""
    i = 0
    while i < len(argv):
        if argv[i] == "--son" and i + 1 < len(argv):
            son_n = int(argv[i + 1]); i += 2
        else:
            onek = argv[i]; i += 1

    desen = os.path.join(_LOGS, f"visual_lead_{onek}*.csv" if onek
                         else "visual_lead_*.csv")
    dosyalar = sorted(glob.glob(desen))
    if son_n:
        dosyalar = dosyalar[-son_n:]
    if not dosyalar:
        print(f"Log bulunamadı: {desen}")
        return 1

    print(f"\nLOS KAYMASI — menzil {BANT_UST:.0f} m → {BANT_ALT:.0f} m bandı")
    print("Çarpışma rotası = açı SABİT. Kayma küçükse geometri doğru.\n")
    print(f"{'faz':>8} {'sonuç':>6} {'devir':>7} {'enyakın':>8} "
          f"{'kayma°':>8} {'net°':>7} {'std°':>6}  değerlendirme")
    print("-" * 78)

    olculen = []
    for y in dosyalar:
        r = faz_olc(y)
        if r is None:
            continue
        if r["kayma"] is None:
            print(f"{r['ad']:>8} {r['sonuc']:>6} "
                  f"{r['devir'] or 0:7.1f} {r['en_yakin'] or 0:8.2f} "
                  f"{'--':>8} {'--':>7} {'--':>6}  bantta veri yok (n={r['n']})")
            continue
        olculen.append(r)
        etiket = ("KİLİTLİ ✓" if r["kayma"] < IYI_ESIK
                  else "kayıyor" if r["kayma"] < 15 else "ÇOK KAYIYOR ✗")
        print(f"{r['ad']:>8} {r['sonuc']:>6} {r['devir']:7.1f} {r['en_yakin']:8.2f} "
              f"{r['kayma']:8.1f} {r['egim']:+7.1f} {r['std']:6.1f}  {etiket}")

    if not olculen:
        print("\nÖlçülebilir faz yok.")
        return 1

    print("-" * 78)
    vuran = [r for r in olculen if r["sonuc"] == "VURDU"]
    iska = [r for r in olculen if r["sonuc"] == "ıska"]
    kilitli = [r for r in olculen if r["kayma"] < IYI_ESIK]

    print(f"\nÖZET — {len(olculen)} ölçülebilir faz")
    print(f"  vuruş            : {len(vuran)}/{len(olculen)} "
          f"(%{100*len(vuran)/len(olculen):.0f})")
    print(f"  kilitli (<{IYI_ESIK:.0f}°)   : {len(kilitli)}/{len(olculen)} "
          f"(%{100*len(kilitli)/len(olculen):.0f})   ← ASIL TAKİP EDİLECEK SAYI")
    print(f"  kayma medyanı    : {st.median([r['kayma'] for r in olculen]):.1f}°")
    if vuran:
        print(f"  vuranların kayması : {st.median([r['kayma'] for r in vuran]):.1f}°")
    if iska:
        print(f"  ıskaların kayması  : {st.median([r['kayma'] for r in iska]):.1f}°")

    print(f"\n  En yakın menzil medyanı: "
          f"{st.median([r['en_yakin'] for r in olculen]):.2f} m")
    print("  ⚠ Bu sayıya BAKMA — vuran ve ıskalayan fazlarda aynı çıkıyor.")
    print(f"  Kararı 'kilitli' oranına göre ver: yükseliyorsa ayar doğru yönde.\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
