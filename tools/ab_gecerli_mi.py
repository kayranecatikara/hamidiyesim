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


def _damga(dosyalar):
    """Görsel fazın yapılandırma damgasını {alan: değer} olarak çöz.

    Damga yalnız CSV'nin İLK veri satırında var (`visual_lead.py:344-382`).
    A/B'de iki kolun YALNIZ sınanan değişkende ayrılması gerekir; başka bir
    alan da farklıysa deney iki değişkenlidir ve sonuç yorumlanamaz.
    """
    for tip, _p, rows in dosyalar:
        if tip != "vis" or not rows:
            continue
        ham = rows[0].get("yapilandirma")
        if not ham:
            continue
        d = {}
        for parca in ham.split(","):
            if "=" in parca:
                k, v = parca.split("=", 1)
                d[k.strip()] = v.strip()
        if d:
            return d
    return {}


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

    damgalar = [_damga(d) for _e, d in secili]
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

    # (sorun metni, o soruna ÖZGÜ tavsiye) — tavsiye teşhise göre değişmeli.
    # ⚠ 2026-08-09: ilk sürüm her sorunda aynı sabit metni basıyordu ("DÜZ
    # senaryo kullan"). Kullanıcı düz uçtuğu hâlde bunu gördü ve haklı olarak
    # "saçma çıktı" dedi. Tavsiye teşhisten türemezse araca güven kalmıyor.
    sorunlar = []
    d_irt = abs(a["irtifa"] - b["irtifa"])
    if d_irt > IRTIFA_ESIK_M:
        sorunlar.append((
            f"hedef irtifası {d_irt:.1f} m farklı "
            f"({a['irtifa']:.0f} m ↔ {b['irtifa']:.0f} m, eşik "
            f"{IRTIFA_ESIK_M:.0f} m) — kollar aynı geometride uçmamış",
            "Hedefin irtifasını ARAYÜZDEKİ GAZ SLIDER'I belirliyor: "
            "scenario_duz pitch=0 (seviye) uçuyor, fazla itki tırmanışa "
            "gidiyor (run_plane_scenario.py:207-215, hold(...) → "
            "gcs_throttle()). İki kolda da SLIDER'I AYNI DEĞERE koy; "
            "ölçülen tavan 133.8 m."))
    for o in (a, b):
        if o["sure"] < ASGARI_SURE_S:
            sorunlar.append((
                f"{o['etiket']} yalnız {o['sure']:.0f} s sürmüş "
                f"(asgari {ASGARI_SURE_S:.0f} s)",
                "Kolu daha uzun uçur; kısa koşuda faz sayısı örneklem "
                "oluşturmuyor."))
        if abs(o["tirmanma"]) > TIRMANMA_ESIK_MDK:
            sorunlar.append((
                f"{o['etiket']}: hedef {o['tirmanma']:+.0f} m/dk ile irtifa "
                f"değiştiriyor (eşik ±{TIRMANMA_ESIK_MDK:.0f}) — geometri "
                f"koşu boyunca kayıyor",
                "DAİRE senaryosunda hedef hiç oturmuyor (+35…+92 m/dk); "
                "A/B için DÜZ senaryo kullan. Düzde de tırmanıyorsa gaz "
                "slider'ını düşür ya da hedefi tavana (133.8 m) oturt."))
    kisa, uzun = sorted((a["sure"], b["sure"]))
    if uzun > 0 and kisa / uzun < SURE_ORAN_ESIK:
        sorunlar.append((
            f"süreler çok farklı ({kisa:.0f} s ↔ {uzun:.0f} s)",
            "İki kolu yaklaşık aynı süre uçur."))

    # ── Yapılandırma farkı: A/B YALNIZ tek değişkende ayrılmalı ──
    da, db = damgalar
    if da and db:
        farklar = sorted(k for k in set(da) | set(db)
                         if da.get(k) != db.get(k))
        print()
        if not farklar:
            # ⚠ 2026-08-09: bu dalda `sorunlar_damga` HİÇ ATANMIYORDU ve
            # birkaç satır aşağıdaki `if sorunlar_damga:` UnboundLocalError
            # veriyordu — yani "iki kol aynı" durumu, aracın çökmesi demekti.
            # Tam da en sık karşılaşılan hâl (damga sınanan değişkeni
            # taşımıyorsa kollar hep "aynı" görünür).
            sorunlar_damga = False
            print("yapılandırma damgası: iki kol AYNI "
                  "(sınanan değişken damgada değilse elle doğrula)")
        else:
            print(f"yapılandırma farkı ({len(farklar)} alan):")
            for k in farklar:
                print(f"    {k:<12} A={da.get(k, '—'):<10} B={db.get(k, '—')}")
            if len(farklar) > 1:
                sorunlar_damga = True
            else:
                sorunlar_damga = False
    else:
        sorunlar_damga = False
        print("\n⚠ görsel damga okunamadı — kolların ayarı doğrulanamıyor")
    if sorunlar_damga:
        sorunlar.append((
            f"iki kol {len(farklar)} ayarda birden farklı "
            f"({', '.join(farklar)}) — A/B tek değişkenli DEĞİL",
            "Kolları yalnız SINANAN değişkende ayır. Presetlere dikkat: "
            "`scripts/gcs.sh bbox` TRACKER/LOCK'u off'a, `takip` on'a ZORLAR "
            "(gcs.sh:28-30); iki kolda da aynı preset'i kullan."))

    print()
    if a["yasa"] != b["yasa"]:
        print(f"ℹ GPS yasası kollarda FARKLI: A={a['yasa']}  B={b['yasa']}")
        print("  (FRPN A/B'sinde beklenen budur; başka bir A/B'de KARIŞTIRICIDIR)")
        print()
    if sorunlar:
        print("KIRMIZI — bu A/B KIYASLANAMAZ:")
        for s, _ in sorunlar:
            print(f"  ✗ {s}")
        print("\nNe yapmalı:")
        gorulen = set()
        for _, tavsiye in sorunlar:
            if tavsiye in gorulen:
                continue
            gorulen.add(tavsiye)
            print(f"  → {tavsiye}")
        print("  → Her kol için simi baştan kur: "
              "bash scripts/start_harmonic.sh yeniden")
        return 1
    print("YEŞİL — kollar kıyaslanabilir. (Bu yalnız GEOMETRİ denetimidir;")
    print("değişikliğin etkisi ayrıca ölçülmeli: tools/gudum_karne.py --kiyasla)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
