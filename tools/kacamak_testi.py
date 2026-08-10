#!/usr/bin/env python3
"""Kaçamak testi — hedef TAM VURULACAKKEN manevra yapar, tepki ölçülür.

NEDEN VAR (bkz. CLAUDE.md §3): hedefe sürekli tam daire çizdirmek verimsiz
bir test. Drone dairenin içine giremiyor, buluşmalar kafa kafaya oluyor
(kapanma p90 13-22 m/s), her koşu birbirinden çok farklı çıkıyor ve manevra
tepkisi hakkında hiçbir şey anlaşılmıyor.

BUNUN YERİNE: hedef DÜZ uçar, drone temiz bir kuyruk yaklaşması kurar, ve
mesafe eşiğe inince — yani tam vuracakken — hedef BELİRLİ bir kaçamak yapar.
Böylece her koşu aynı başlangıç geometrisinden başlar ve ölçülen tek şey
"bu kaçamağa güdüm nasıl tepki verdi" olur.

Kullanım:
    python3 tools/kacamak_testi.py <cikti_dizini> <kacamak> [tetik_m] [kayit_s]

Kaçamaklar:  yok | yatay | dikey_yukari | dikey_asagi | capraz | hizlan
    yok = TABAN koşusu (kaçamak yapılmaz) — kıyas çizgisi, her kampanyada koş.

Çıktı:
    <dir>/frames/*.jpg + meta.csv   (ucus_kaydi.py — video bacağı)
    <dir>/kacamak.csv               (10 Hz: t, mesafe, faz, rc)
    <dir>/olay.json                 (tetik anı, ıska mesafesi, isabet)
"""
import csv
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8000"

# aileron / elevator / throttle (PWM; 1500 nötr, gaz cruise 1600)
KACAMAKLAR = {
    "yok":          None,
    "yatay":        (2000, 1500, 1600),   # sert sağa kırılma
    "dikey_yukari": (1500, 1150, 1800),   # tırmanış (gaz artar, stall olmasın)
    "dikey_asagi":  (1500, 1850, 1500),   # dalış
    "capraz":       (1950, 1250, 1700),   # yatay + tırmanış birlikte
    "hizlan":       (1500, 1500, 2000),   # düz ama tam gaz
}

KACAMAK_SURE = 6.0        # s; kaçamak RC'si bu kadar sürer
NOTR = (1500, 1500, 1600)


def istek(yol, veri=None, zaman_asimi=3.0):
    url = BASE + yol
    if veri is None:
        r = urllib.request.Request(url, method="POST")
    else:
        r = urllib.request.Request(
            url, data=json.dumps(veri).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(r, timeout=zaman_asimi) as f:
            return json.loads(f.read().decode() or "{}")
    except Exception:
        return None


def oku(yol, zaman_asimi=3.0):
    try:
        with urllib.request.urlopen(BASE + yol, timeout=zaman_asimi) as f:
            return json.loads(f.read().decode())
    except Exception:
        return None


def durum():
    """(mesafe_m, iris, plane) — anlık GPS'ten. Yoksa (None, {}, {})."""
    d = oku("/api/debug/telem")
    if not d:
        return None, {}, {}
    t = d.get("telemetry_state", {})
    i, p = t.get("iris") or {}, t.get("plane") or {}
    try:
        mes = ((i["x"] - p["x"]) ** 2 + (i["y"] - p["y"]) ** 2
               + (i["z"] - p["z"]) ** 2) ** 0.5
    except (KeyError, TypeError):
        mes = None
    return mes, i, p


def rc(a, e, t):
    istek("/api/command/plane/manual",
          {"aileron": a, "elevator": e, "throttle": t})


def bekle_hedef_hazir(sn=240, oturma_s=12.0, hiz_bant=0.8, irt_bant=3.0):
    """Hedef KARARLI SEYRE oturana kadar bekler.

    ⚠ 2026-08-10 DÜZELTMESİ — ESKİ HÂLİ TÜM A/B'LERİ GEÇERSİZ KILIYORDU.
    Eski ölçüt "hız > 12 m/s ve irtifa 20-300 m" idi ve hedefi TIRMANIŞ
    GEÇİŞİNDE yakalıyordu: kalkışta hız 22 m/s'ye fırlıyor, sonra kararlı
    seyir 15.2 m/s'ye oturuyor. Chase o yüzden hedef henüz oturmamışken
    başlıyordu; kaçamak t≈19 s'de tetikleniyor ve kesişim HER KOLDA
    kolaylaşıyordu. Ölçüldü (Ö5 2×2 kampanyası, 4 uçuş): dört kol da vurdu,
    en yakın menziller 0.13-0.26 m — kol farkı ölçülemedi, kampanya çöp oldu.
    Buna karşılık hedefin oturması BEKLENEN tek koşuda taban kolu ıskaladı
    (194 m geriden, kapanma 2.8 m/s, tetik t=79.6 s, en yakın 1.25 m).

    Yeni ölçüt: hız VE irtifa `oturma_s` boyunca dar bir bantta KALMALI.
    Böylece her koşu aynı geometriden başlar (CLAUDE.md §3.3'ün amacı budur).
    """
    t0 = time.time()
    pencere = []                       # (t, hiz, irtifa) — son oturma_s saniye
    while time.time() - t0 < sn:
        _, _, p = durum()
        simdi = time.time()
        hiz = p.get("speed") or 0.0
        irt = -(p.get("z") or 0.0)
        if hiz > 6.0 and 20 < irt < 300:
            pencere.append((simdi, hiz, irt))
        else:
            pencere.clear()            # bant dışı → oturma sayacı sıfırlanır
        # ⚠ Pencere oturma_s'DEN GENİŞ tutulur; yoksa kırpma sonrası
        # "span >= oturma_s" koşulu matematiksel olarak sağlanamaz (off-by-one).
        pencere[:] = [x for x in pencere if simdi - x[0] <= oturma_s * 2.0]
        if pencere and simdi - pencere[0][0] >= oturma_s:
            son = [x for x in pencere if simdi - x[0] <= oturma_s]
            hizlar = [x[1] for x in son]
            irtler = [x[2] for x in son]
            if (max(hizlar) - min(hizlar) <= hiz_bant
                    and max(irtler) - min(irtler) <= irt_bant):
                print(f"  hedef KARARLI SEYİRDE: {sum(hizlar)/len(hizlar):.1f} m/s "
                      f"(bant {max(hizlar)-min(hizlar):.2f}), "
                      f"{sum(irtler)/len(irtler):.0f} m "
                      f"(bant {max(irtler)-min(irtler):.1f}) — {simdi-t0:.0f} s'de oturdu")
                return True
        time.sleep(1.0)
    print(f"  ⚠ hedef {sn:.0f} s'de kararlı seyre OTURMADI — koşu GEÇERSİZ")
    return False


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        raise SystemExit(1)
    dizin = sys.argv[1]
    kacamak = sys.argv[2]
    tetik_m = float(sys.argv[3]) if len(sys.argv) > 3 else 25.0
    kayit_s = float(sys.argv[4]) if len(sys.argv) > 4 else 120.0
    if kacamak not in KACAMAKLAR:
        print(f"bilinmeyen kaçamak: {kacamak}  ({'|'.join(KACAMAKLAR)})")
        raise SystemExit(1)
    os.makedirs(dizin, exist_ok=True)

    print(f"[KAÇAMAK TESTİ] {kacamak}  tetik={tetik_m:.0f} m  kayıt={kayit_s:.0f} s")

    # ── temiz başlangıç: düz uçuş ──
    istek("/api/command/iris/stop_chase")
    istek("/api/command/plane/stop_manual")
    istek("/api/command/plane/stop_scenario")
    time.sleep(3.0)
    istek("/api/hasar/sifirla")
    istek("/api/command/plane/scenario/duz")
    print("  hedef kalkıyor (duz)...")
    if not bekle_hedef_hazir():
        raise SystemExit(2)
    time.sleep(8.0)                      # düz uçuş otursun

    # ── kare kaydı (video bacağı — CLAUDE.md §2.2) ──
    kok = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    kayit = subprocess.Popen(
        ["python3", os.path.join(kok, "tools", "ucus_kaydi.py"),
         dizin, str(int(kayit_s))],
        cwd=kok, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    istek("/api/command/iris/start_chase")
    t0 = time.time()
    print(f"  takip başladı {time.strftime('%H:%M:%S')}")

    # ── izleme döngüsü: 10 Hz, mesafe eşiğe inince kaçamağı TETİKLE ──
    satirlar = []
    tetiklendi = tetik_t = None
    en_yakin, en_yakin_t = 1e9, None
    en_yakin_sonra = 1e9
    faz = "yaklasma"
    while time.time() - t0 < kayit_s:
        mes, i, p = durum()
        t = time.time() - t0
        if mes is not None:
            if mes < en_yakin:
                en_yakin, en_yakin_t = mes, t
            if tetiklendi and mes < en_yakin_sonra:
                en_yakin_sonra = mes
            # TETİK: eşiğe indi, kaçamak henüz yapılmadı
            if (not tetiklendi and mes <= tetik_m and t > 5.0
                    and KACAMAKLAR[kacamak] is not None):
                istek("/api/command/plane/start_manual")
                time.sleep(0.2)
                a, e, g = KACAMAKLAR[kacamak]
                rc(a, e, g)
                tetiklendi, tetik_t, faz = True, t, "kacamak"
                print(f"  ⚡ KAÇAMAK '{kacamak}' TETİKLENDİ  t={t:.1f}s  "
                      f"mesafe={mes:.1f} m  rc=({a},{e},{g})")
            elif not tetiklendi and mes <= tetik_m and t > 5.0:
                tetiklendi, tetik_t, faz = True, t, "taban"
                print(f"  ⚑ TABAN işareti (kaçamak yok) t={t:.1f}s "
                      f"mesafe={mes:.1f} m")
        if tetiklendi and KACAMAKLAR[kacamak] is not None:
            gecen = t - tetik_t
            if gecen < KACAMAK_SURE:
                rc(*KACAMAKLAR[kacamak])       # RC'yi canlı tut (override)
            elif faz == "kacamak":
                rc(*NOTR)
                faz = "notr"
                print(f"  kaçamak bitti, nötre alındı  t={t:.1f}s")
            else:
                rc(*NOTR)
        satirlar.append({
            "t": round(t, 2), "mesafe": None if mes is None else round(mes, 2),
            "faz": faz,
            "iris_spd": round(i.get("speed") or 0.0, 2),
            "plane_spd": round(p.get("speed") or 0.0, 2),
            "iris_alt": round(-(i.get("z") or 0.0), 1),
            "plane_alt": round(-(p.get("z") or 0.0), 1),
        })
        time.sleep(0.1)

    kayit.wait(timeout=30)
    istek("/api/command/iris/stop_chase")
    istek("/api/command/plane/stop_manual")
    istek("/api/command/plane/stop_scenario")

    hasar = oku("/api/hasar") or {}
    with open(os.path.join(dizin, "kacamak.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(satirlar[0]))
        w.writeheader()
        w.writerows(satirlar)
    olay = {
        "kacamak": kacamak, "tetik_m": tetik_m,
        "tetiklendi": bool(tetiklendi),
        "tetik_t": None if tetik_t is None else round(tetik_t, 2),
        "en_yakin": round(en_yakin, 2) if en_yakin < 1e9 else None,
        "en_yakin_t": None if en_yakin_t is None else round(en_yakin_t, 2),
        "en_yakin_tetikten_sonra": (round(en_yakin_sonra, 2)
                                    if en_yakin_sonra < 1e9 else None),
        "imha": bool(hasar.get("imha")),
        "temas": hasar.get("temas"),
    }
    with open(os.path.join(dizin, "olay.json"), "w") as f:
        json.dump(olay, f, ensure_ascii=False, indent=2)
    print(f"  SONUÇ: imha={olay['imha']}  en_yakın={olay['en_yakin']} m  "
          f"tetik sonrası en yakın={olay['en_yakin_tetikten_sonra']} m")
    print(f"  uçuş bitti {time.strftime('%H:%M:%S')}")


if __name__ == "__main__":
    main()
