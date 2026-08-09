#!/usr/bin/env python3
"""ab_gecerli_mi.py — iki A/B kolunun KIYASLANABİLİR olup olmadığını denetler.

NEDEN VAR
---------
2026-08-08'de dört A/B ("PN_KAPI, PN_MAX, V_KAPANMA, IVME") yapıldı, hepsi
"fark yok" çıktı ve öyle raporlandı. Sonradan ölçüldü ki hedef uçak DAİRE
senaryosunda irtifa tutmuyor (+35…+92 m/dk, hiç oturmuyor); iki kol 134-175 m
farklı irtifada uçmuş. Ölçtüğümüz şey değişikliğin etkisi değildi; dördü de
çöpe gitti. Bu araç o kontrolü gözden alıp koda veriyor: kıyastan ÖNCE
çalıştır, KIRMIZI derse o A/B'yi rapor etme.

⚠ 2026-08-09 DÜZELTMESİ — bir kol = bir UÇUŞ, bir DOSYA değil.
İlk sürüm son iki CSV DOSYASINI iki kol sanıyordu. Oysa supervisor her faz
geçişinde yeni dosya açıyor (`supervisor.py:180,190,199`): 08-09 uçuşu tek
başına 45 dosya / 23 GPS + 22 görsel faz. Araç bu yüzden "21 s / 14 s sürmüş"
diyordu — onlar tek tek FAZLARDI, kollar değil. Artık gruplama
`gudum_karne.ucuslar()`'a devredildi (120 s boşluk kuralı, tek kaynak).

KULLANIM
--------
    PYTHONPATH=. python3 tools/ab_gecerli_mi.py            # son iki uçuş
    PYTHONPATH=. python3 tools/ab_gecerli_mi.py 1421 1435  # damgayla

Çıkış kodu: kollar kıyaslanabilirse 0, değilse 1.
"""
from __future__ import annotations

import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.gudum_karne import ucuslar as _ucuslar    # noqa: E402  (yol ayarı üstte)

# Eşikler — gerekçeleri:
#   irtifa 25 m : ölçülen tırmanma 35-92 m/dk; 25 m'lik fark ~20-40 saniyelik
#                 kayma demek, menzil/geometri sonucunu görünür şekilde kaydırır.
#   tırmanma ±15 m/dk : düz uçuşta plato ölçüldü (65 m ve 134 m'de tam 0.0);
#                 dairede 35-92 m/dk. 15 ikisini ayırır. ASIL SEBEBİ yakalar:
#                 kollar aynı irtifada başlasa bile hedef tırmanmaya devam
#                 ediyorsa fark koşu boyunca büyür.
#   süre  %40   : bir kol yarı sürede biterse örneklem sayısı kıyaslanamaz.
#   asgari 40 s : altında irtifa platosuna oturmamış olabilir.
IRTIFA_ESIK_M = 25.0
TIRMANMA_ESIK_MDK = 15.0
SURE_ORAN_ESIK = 0.40
ASGARI_SURE_S = 40.0


def _yasa(rows):
    """GPS kolunun hangi yasayla uçtuğunu CSV SÜTUNLARINDAN tanı.

    `frpn_guidance` çıktısını `gps_guidance` ile AYNI dosya adıyla yazıyor
    (`frpn_guidance.py:128`) ve GPS tarafında yapılandırma damgası yok — yani
    dosya adından hangi yasanın uçtuğu anlaşılmıyor. Sütun kümeleri ayrık
    olduğu için imza oradan okunur.
    """
    if not rows:
        return None
    a = rows[0].keys()
    if "t_go" in a:            # frpn_guidance.py:83-93
        return "frpn"
    if "ist_elev_deg" in a:    # gps_guidance._CSV_ALANLAR
        return "istasyon"
    return "bilinmiyor"


def _ozet(etiket, dosyalar):
    """Bir UÇUŞUN (tüm fazları birlikte) A/B geçerliliği açısından özeti."""
    ts, alt = [], []
    yasalar = set()
    for tip, _p, rows in dosyalar:
        if tip != "gps":
            continue
        y = _yasa(rows)
        if y:
            yasalar.add(y)
        for s in rows:
            try:
                ts.append(float(s["t"]))
                # tgt_z NED'dir (aşağı POZİTİF). İrtifa = -tgt_z; işareti
                # burada çevriliyor ki tabloda "43 m" yazsın, "-43" değil.
                alt.append(-float(s["tgt_z"]))
            except (KeyError, ValueError, TypeError):
                continue
    if len(ts) < 20:
        return None
    # Fazlar ayrı dosyalarda ama 't' aynı saat ekseninde → uçuş süresi uçtan uca
    sure = max(ts) - min(ts)
    sirali = sorted(alt)
    ortanca = sirali[len(sirali) // 2]
    # Tırmanma (m/dk): zamanla sıralanıp ilk %20 ile son %20 karşılaştırılır
    ikili = sorted(zip(ts, alt))
    n = len(ikili)
    d = max(1, n // 5)
    bas = st.mean(a for _, a in ikili[:d])
    son = st.mean(a for _, a in ikili[-d:])
    tirmanma = (son - bas) / sure * 60.0 if sure > 0 else 0.0
    return {
        "etiket": etiket, "sure": sure, "n": n,
        "irtifa": ortanca, "tirmanma": tirmanma,
        "yasa": "+".join(sorted(yasalar)) if yasalar else "?",
        "faz": sum(1 for t, _, _ in dosyalar if t == "gps"),
    }


def main(argv):
    gruplar = _ucuslar()
    if len(argv) > 2:
        secili = []
        for anahtar in argv[1:3]:
            adaylar = [g for g in gruplar if anahtar in g[0]]
            if not adaylar:
                print(f"⚠ '{anahtar}' damgalı uçuş yok "
                      f"(PYTHONPATH=. python3 tools/gudum_karne.py --liste)")
                return 1
            secili.append(adaylar[-1])
    else:
        secili = gruplar[-2:]

    if len(secili) < 2:
        print("⚠ kıyaslanacak İKİ UÇUŞ yok "
              f"(logs/ içinde {len(gruplar)} uçuş bulundu).")
        print("  Bir A/B iki AYRI uçuş ister; tek uçuşun fazları kol değildir.")
        return 1

    ozetler = [_ozet(e, d) for e, d in secili]
    if any(o is None for o in ozetler):
        print("⚠ uçuşlardan biri boş/kısa — kıyas yok")
        return 1
    a, b = ozetler

    print(f"{'kol':<8}{'uçuş':<18}{'süre s':>9}{'GPS faz':>9}"
          f"{'hedef irtifa m':>16}{'tırmanma m/dk':>15}{'GPS yasası':>13}")
    for ad, o in (("A", a), ("B", b)):
        print(f"{ad:<8}{o['etiket']:<18}{o['sure']:>9.1f}{o['faz']:>9}"
              f"{o['irtifa']:>16.1f}{o['tirmanma']:>15.1f}{o['yasa']:>13}")

    sorunlar = []
    d_irt = abs(a["irtifa"] - b["irtifa"])
    if d_irt > IRTIFA_ESIK_M:
        sorunlar.append(f"hedef irtifası {d_irt:.1f} m farklı "
                        f"(eşik {IRTIFA_ESIK_M:.0f} m) — kollar aynı "
                        f"geometride uçmamış")
    for o in (a, b):
        if o["sure"] < ASGARI_SURE_S:
            sorunlar.append(f"{o['etiket']} yalnız {o['sure']:.0f} s sürmüş "
                            f"(asgari {ASGARI_SURE_S:.0f} s)")
        if abs(o["tirmanma"]) > TIRMANMA_ESIK_MDK:
            sorunlar.append(
                f"{o['etiket']}: hedef {o['tirmanma']:+.0f} m/dk ile irtifa "
                f"değiştiriyor (eşik ±{TIRMANMA_ESIK_MDK:.0f}) — geometri koşu "
                f"boyunca kayıyor, plato beklenmemiş")
    kisa, uzun = sorted((a["sure"], b["sure"]))
    if uzun > 0 and kisa / uzun < SURE_ORAN_ESIK:
        sorunlar.append(f"süreler çok farklı ({kisa:.0f} s ↔ {uzun:.0f} s)")

    print()
    if a["yasa"] != b["yasa"]:
        print(f"ℹ GPS yasası kollarda FARKLI: A={a['yasa']}  B={b['yasa']}")
        print("  (FRPN A/B'sinde beklenen budur; başka bir A/B'de KARIŞTIRICIDIR)")
        print()
    if sorunlar:
        print("KIRMIZI — bu A/B KIYASLANAMAZ:")
        for s in sorunlar:
            print(f"  ✗ {s}")
        print("\nNe yapmalı: her kol için simi baştan kur")
        print("  bash scripts/start_harmonic.sh yeniden")
        print("ve DÜZ senaryo kullan (dairede hedef sürekli tırmanıyor).")
        return 1
    print("YEŞİL — kollar kıyaslanabilir. (Bu yalnız GEOMETRİ denetimidir;")
    print("değişikliğin etkisi ayrıca ölçülmeli: tools/gudum_karne.py --kiyasla)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
