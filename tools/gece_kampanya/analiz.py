#!/usr/bin/env python3
"""Gece kampanya karşılaştırma belgesi üreteci (idempotent, salt-okur).

logs/gece_kampanya/*/ altındaki her uçuşu (meta.json + olay.json + kacamak.csv
+ arşivlenmiş kilit_*.csv) okur, (vmax, caprazlead, kacamak) gruplar, metrik
çıkarır ve RAPOR_KARSILASTIRMA.md yazar. Her uçuştan sonra yeniden çağrılabilir.
"""
import csv, json, glob, os, statistics, sys

OUT = "/home/aysenur/projects/hamidiyesim/logs/gece_kampanya"

def _f(x, d=None):
    try: return float(x)
    except (TypeError, ValueError): return d

def _truthy(x):
    return str(x).strip().lower() in ("1", "true", "yes", "var", "evet")

def kacamak_metrik(path):
    """kacamak.csv'den kapanma hızı, savrulma, plane_med, iris_max."""
    try:
        rows = list(csv.DictReader(open(path)))
    except OSError:
        return {}
    if not rows: return {}
    t0 = _f(rows[0].get("t"), 0.0)
    ts = [_f(r.get("t"), 0.0) - t0 for r in rows]
    ms = [_f(r.get("mesafe")) for r in rows]
    iris = [_f(r.get("iris_spd")) for r in rows if _f(r.get("iris_spd")) is not None]
    pl = [_f(r.get("plane_spd")) for r in rows if _f(r.get("plane_spd")) is not None]
    ms_v = [m for m in ms if m is not None]
    # kapanma 100->30 m
    rate = None
    i100 = next((i for i, m in enumerate(ms) if m is not None and m <= 100), None)
    i30 = next((i for i, m in enumerate(ms) if m is not None and m <= 30), None)
    if i100 is not None and i30 is not None and ts[i30] > ts[i100]:
        rate = round((ms[i100] - ms[i30]) / (ts[i30] - ts[i100]), 2)
    # savrulma: 25 m altına indikten sonra >40 m'ye geri fırlama sayısı
    savrul, below, up = 0, False, False
    for m in ms:
        if m is None: continue
        if m < 25: below, up = True, False
        if below and m > 40 and not up: savrul += 1; up = True
        if m < 25: up = False
    return dict(rate=rate, savrul=savrul,
                plane_med=round(statistics.median(pl), 1) if pl else None,
                iris_max=round(max(iris), 1) if iris else None,
                dmin=round(min(ms_v), 2) if ms_v else None)

def tespit_menzil(kilit_path):
    """kilit_*.csv'den menzil-binli tespit oranı + ilk tespit menzili."""
    try:
        rows = list(csv.DictReader(open(kilit_path)))
    except OSError:
        return None
    if not rows or "menzil" not in rows[0] or "tespit_dogrulandi" not in rows[0]:
        return None
    bins = [(0,25),(25,50),(50,100),(100,200),(200,9999)]
    say = {b: [0,0] for b in bins}   # [tespit, toplam]
    ilk = None
    for r in rows:
        mz = _f(r.get("menzil"));  td = _truthy(r.get("tespit_dogrulandi"))
        if mz is None: continue
        for b in bins:
            if b[0] <= mz < b[1]:
                say[b][1] += 1
                if td: say[b][0] += 1
                break
        if td and (ilk is None or mz > ilk): ilk = mz
    oran = {b: (round(100*say[b][0]/say[b][1]) if say[b][1] else None) for b in bins}
    return dict(ilk_tespit_menzil=round(ilk,1) if ilk else None, bin_oran=oran)

def ucus_yukle(d):
    meta = {}
    mp = os.path.join(d, "meta.json")
    if os.path.exists(mp):
        try: meta = json.load(open(mp))
        except (OSError, ValueError): meta = {}
    ol = {}
    op = os.path.join(d, "olay.json")
    if os.path.exists(op):
        try: ol = json.load(open(op))
        except (OSError, ValueError): ol = {}
    km = kacamak_metrik(os.path.join(d, "kacamak.csv"))
    kilit = sorted(glob.glob(os.path.join(d, "kilit_*.csv")))
    tesp = tespit_menzil(kilit[-1]) if kilit else None
    thr = thrash_metrik(d)
    return dict(dir=os.path.basename(d), meta=meta, ol=ol, km=km, tesp=tesp, thr=thr)

def thrash_metrik(d):
    """gcs.log'dan GPS↔görsel titremesi: görsel yürütücü girişi ve kayip sayısı."""
    p = os.path.join(d, "gcs.log")
    if not os.path.exists(p): return None
    try: txt = open(p, errors="ignore").read()
    except OSError: return None
    return dict(giris=txt.count("GÖRSEL YÜRÜTÜCÜ"), kayip=txt.count("sebep=kayip"))

def ort_sd(xs):
    xs = [x for x in xs if x is not None]
    if not xs: return (None, None)
    m = round(statistics.mean(xs), 2)
    s = round(statistics.pstdev(xs), 2) if len(xs) > 1 else 0.0
    return (m, s)

def grup_satiri(us):
    """Bir config grubunun uçuşlarından özet."""
    gecerli = [u for u in us if u["meta"].get("durum") == "OK"]
    n_top = len(us); n_gec = len(gecerli)
    imha = [1 if u["ol"].get("imha") else 0 for u in gecerli]
    hit = round(100*sum(imha)/len(imha)) if imha else None
    eny = [u["ol"].get("en_yakin") for u in gecerli if u["ol"].get("en_yakin") is not None]
    tet = [u["ol"].get("tetik_t") for u in gecerli if u["ol"].get("tetik_t") is not None]
    rate = [u["km"].get("rate") for u in gecerli]
    sav = [u["km"].get("savrul") for u in gecerli if u["km"].get("savrul") is not None]
    thr = [u["thr"].get("giris") for u in gecerli if u.get("thr") and u["thr"].get("giris") is not None]
    rate_m, _ = ort_sd(rate)
    return dict(n_gec=n_gec, n_top=n_top, hit=hit,
                eny_med=round(statistics.median(eny),2) if eny else None,
                eny_min=round(min(eny),2) if eny else None,
                tet_m=round(statistics.mean(tet)) if tet else None,
                rate_m=rate_m,
                sav_m=round(statistics.mean(sav),1) if sav else None,
                thr_m=round(statistics.mean(thr),1) if thr else None)

def main():
    dirs = sorted(d for d in glob.glob(os.path.join(OUT, "*")) if os.path.isdir(d)
                  and os.path.exists(os.path.join(d, "meta.json")))
    ucuslar = [ucus_yukle(d) for d in dirs]
    L = []
    L.append("# GECE KAMPANYA — KARŞILAŞTIRMALI BELGE\n")
    L.append(f"Toplam uçuş: **{len(ucuslar)}** · "
             f"geçerli: {sum(1 for u in ucuslar if u['meta'].get('durum')=='OK')} · "
             f"geçersiz: {sum(1 for u in ucuslar if u['meta'].get('durum')=='GECERSIZ')} · "
             f"başarısız: {sum(1 for u in ucuslar if u['meta'].get('durum')=='BASARISIZ')}\n")
    L.append("> Otonom koşu = hipotez + sayı. Kabul kararı sabah insanla ortak "
             "görsel doğrulamadan sonra (CLAUDE.md §2).\n")

    def grupla(anahtar):
        g = {}
        for u in ucuslar:
            k = anahtar(u["meta"])
            g.setdefault(k, []).append(u)
        return g

    # § A — Kapanma (yok, CAPRAZLEAD off): VMAX tatlı nokta
    L.append("\n## §1 Kapanma tatlı noktası (izole, kaçamak=yok)\n")
    L.append("| VMAX | N(geç/top) | kapanma m/s | tetik@ s | en_yakın med | en_yakın min | temas % |")
    L.append("|---|---|---|---|---|---|---|")
    A = grupla(lambda m: (m.get("kacamak"), int(m.get("vmax",0))))
    for (kac, vm) in sorted(A):
        if kac != "yok": continue
        s = grup_satiri(A[(kac, vm)])
        L.append(f"| {vm} | {s['n_gec']}/{s['n_top']} | {s['rate_m']} | {s['tet_m']} | "
                 f"{s['eny_med']} m | {s['eny_min']} m | {s['hit']}% |")

    # § B/C/D/E — kaçamak × CAPRAZLEAD (off vs on)
    L.append("\n## §2-3 Kaçamak × CAPRAZLEAD (temas oranı + en_yakın + savrulma)\n")
    L.append("| VMAX | kaçamak | CAP | ek-env | N(geç/top) | temas % | en_yakın med | savrulma | titreme | kapanma m/s |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    BC = grupla(lambda m: (int(m.get("vmax",0)), m.get("kacamak"), m.get("caprazlead"), m.get("ekenv","-")))
    for key in sorted(BC, key=lambda x:(str(x[1]), x[0], str(x[2]), str(x[3]))):
        vm, kac, cap, ek = key
        if kac == "yok": continue
        s = grup_satiri(BC[key])
        ek_s = ek if ek and ek != "-" else "—"
        L.append(f"| {vm} | {kac} | {cap} | {ek_s} | {s['n_gec']}/{s['n_top']} | {s['hit']}% | "
                 f"{s['eny_med']} m | {s['sav_m']} | {s['thr_m']} | {s['rate_m']} |")

    # § 4 — Tespit vs menzil (tüm uçuşlardan birleşik)
    L.append("\n## §4 Uzun-menzil tespit boşluğu (kilit logu, tespit_dogrulandi)\n")
    binkeys = [(0,25),(25,50),(50,100),(100,200),(200,9999)]
    topla = {b: [0,0] for b in binkeys}; ilkler = []
    for u in ucuslar:
        t = u["tesp"]
        if not t: continue
        if t.get("ilk_tespit_menzil"): ilkler.append(t["ilk_tespit_menzil"])
        for b in binkeys:
            o = t["bin_oran"].get(b)
            # bin_oran anahtarları tuple; JSON'dan gelmediği için doğrudan
    # tespit oranını ham say'dan tekrar hesapla (kilit dosyalarını yeniden gez)
    for u in ucuslar:
        kilit = sorted(glob.glob(os.path.join(OUT, u["dir"], "kilit_*.csv")))
        if not kilit: continue
        try: rows = list(csv.DictReader(open(kilit[-1])))
        except OSError: continue
        if not rows or "menzil" not in rows[0]: continue
        for r in rows:
            mz = _f(r.get("menzil")); td = _truthy(r.get("tespit_dogrulandi"))
            if mz is None: continue
            for b in binkeys:
                if b[0] <= mz < b[1]:
                    topla[b][1]+=1; topla[b][0]+= (1 if td else 0); break
    L.append("| menzil bandı | tespit oranı % | örnek |")
    L.append("|---|---|---|")
    for b in binkeys:
        ad = f"{b[0]}-{b[1] if b[1]<9999 else '∞'} m"
        o = round(100*topla[b][0]/topla[b][1]) if topla[b][1] else None
        L.append(f"| {ad} | {o}% | {topla[b][1]} |")
    if ilkler:
        L.append(f"\nİlk-tespit menzili (uzaktan yaklaşırken tespit oturduğu mesafe): "
                 f"medyan **{round(statistics.median(ilkler),1)} m**, "
                 f"maks {round(max(ilkler),1)} m ({len(ilkler)} uçuş).")

    # § 5 — durum listesi
    L.append("\n## §5 Koşu durumları\n")
    for st in ("GECERSIZ", "BASARISIZ"):
        bad = [u for u in ucuslar if u["meta"].get("durum") == st]
        if bad:
            L.append(f"**{st}** ({len(bad)}): " +
                     ", ".join(f"{u['dir']}({u['meta'].get('sebep','?')})" for u in bad))
    L.append(f"\n_son güncelleme: uçuş sayısı {len(ucuslar)}_")

    open(os.path.join(OUT, "RAPOR_KARSILASTIRMA.md"), "w").write("\n".join(L) + "\n")
    print(f"[ANALIZ] {len(ucuslar)} uçuş işlendi → RAPOR_KARSILASTIRMA.md")

    # ── GENEL RAPOR (yalnız 'genel' argümanıyla; sonda çağrılır) ──
    if len(sys.argv) > 1 and sys.argv[1] == "genel":
        MAN = ["yatay", "dikey_yukari", "dikey_asagi", "capraz", "hizlan"]
        gr = {}
        for u in ucuslar:
            m = u["meta"]
            gr.setdefault((int(m.get("vmax", 0)), m.get("kacamak"),
                           m.get("caprazlead"), m.get("ekenv", "-")), []).append(u)
        def _hr(us):
            g = [u for u in us if u["meta"].get("durum") == "OK"]
            if not g: return None, 0
            return round(100 * sum(1 for u in g if u["ol"].get("imha")) / len(g)), len(g)
        def _base(cap, ek): return cap == "off" and (not ek or ek == "-")
        def _aday(cap, ek): return cap == "on" and bool(ek) and "ERKEN_GORSEL=on" in ek
        G = ["# GENEL RAPOR — Manevralı Uçuşta Vuruş Kampanyası\n",
             f"Toplam uçuş: {len(ucuslar)} · "
             f"geçerli {sum(1 for u in ucuslar if u['meta'].get('durum')=='OK')}\n",
             "## Manevra temas oranı — baseline vs aday (VMAX=24)\n",
             "| kaçamak | baseline temas% (N) | aday temas% (N) |", "|---|---|---|"]
        for man in MAN:
            b = [u for k, us in gr.items() if k[1] == man and k[0] == 24 and _base(k[2], k[3]) for u in us]
            a = [u for k, us in gr.items() if k[1] == man and k[0] == 24 and _aday(k[2], k[3]) for u in us]
            hb, nb = _hr(b); ha, na = _hr(a)
            G.append(f"| {man} | {hb if hb is not None else '—'}% ({nb}) | {ha if ha is not None else '—'}% ({na}) |")
        # yatay'da en iyi config sıralaması
        G += ["\n## En iyi config (yatay, temas oranına göre)\n",
              "| sıra | temas% | N | VMAX | CAP | ek-env |", "|---|---|---|---|---|---|"]
        cand = []
        for k, us in gr.items():
            if k[1] != "yatay": continue
            h, n = _hr(us)
            if h is not None: cand.append((h, n, k))
        for i, (h, n, k) in enumerate(sorted(cand, reverse=True)[:6], 1):
            G.append(f"| {i} | {h}% | {n} | {k[0]} | {k[2]} | {k[3] if k[3] != '-' else '—'} |")
        # düz regresyon kontrolü
        yb = [u for k, us in gr.items() if k[1] == "yok" for u in us]
        hy, ny = _hr(yb)
        G.append(f"\n**Düz (yok) regresyon:** temas {hy if hy is not None else '—'}% ({ny}) — "
                 f"kampanya boyunca düşmemeli.")
        G += ["\n> Otonom = hipotez + sayı. **Kabul kararı sabah insanla ortak görsel "
              "doğrulamadan sonra** (CLAUDE.md §2).",
              "> Temas <0.5 m stokastik; oran N tekrar üzerinden. Titreme sütunu (§2-3 canlı "
              "raporda) ERKEN_GORSEL sağlığını gösterir (yüksek=görsel tutunamıyor)."]
        open(os.path.join(OUT, "GENEL_RAPOR.md"), "w").write("\n".join(G) + "\n")
        print("[ANALIZ] GENEL_RAPOR.md yazıldı")

if __name__ == "__main__":
    main()
