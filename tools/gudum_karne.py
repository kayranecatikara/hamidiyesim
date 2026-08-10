"""
tools/gudum_karne.py — Uçuş karnesi: güdüm loglarından otomatik metrik raporu.

Güdüm toparlama yol haritasının (2026-08-02) ölçüm aracı: "iyi/kötü" tartışması
hissiyatla değil bu karneyle yapılır. Bir UÇUŞ = zaman içinde ardışık gps/visual
CSV'leri (dosya adı damgaları arasında >120 s boşluk yeni uçuş sayılır).

Kullanım:
  python3 tools/gudum_karne.py                 # en son uçuşun karnesi (--son)
  python3 tools/gudum_karne.py --liste         # tespit edilen uçuşlar
  python3 tools/gudum_karne.py --ucus 1308     # damgası 1308* olan uçuş
  python3 tools/gudum_karne.py --kiyasla 1308 1316   # iki uçuş yan yana

Not: 2026-08-02 öncesi logs/ test artefaktlarıyla karışıktı (testler artık
tmp'ye yazıyor). Eski sentetik dosyalar içerik imzasıyla elenir: görsel
t_ros=1/30'dan başlayan tam aritmetik seri; gps ilk hedef (50,20,-40) G9 imzası.
"""

import argparse
import csv
import glob
import math
import os
import re
import statistics as st
from collections import Counter

_LOGS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")

# İstasyonun LOS yükselişi — güdümden okunur, sabit yazılmaz (2026-08-06'da
# 25° → 15° değişti ve buradaki sabit yanlış referans gösteriyordu).
try:
    from control.guidance.gps_guidance import Cfg as _GpsCfg
    _ISTASYON_ELEV = float(_GpsCfg.ISTASYON_ELEV_DEG)
except Exception:
    _ISTASYON_ELEV = 15.0
_AD = re.compile(r"(gps_guidance|visual_lead)_(\d{8})_(\d{6})\.csv$")


def _oku(p):
    with open(p) as f:
        return list(csv.DictReader(f))


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _sentetik(p, rows):
    """Eski test artefaktı imzaları (2026-08-02 öncesi kirlilik)."""
    if not rows:
        return True
    if "visual" in p:
        t0 = _f(rows[0].get("t_ros"))
        return t0 is not None and abs(t0 - 1 / 30) < 1e-9
    return (rows[0].get("tgt_x"), rows[0].get("tgt_y"),
            rows[0].get("tgt_z")) == ("50.0", "20.0", "-40.0")


def ucuslar():
    """logs/ içindeki dosyaları uçuşlara grupla: [(etiket, [(tip, yol, rows)])]."""
    kayitlar = []
    for p in sorted(glob.glob(os.path.join(_LOGS, "*.csv"))):
        m = _AD.search(os.path.basename(p))
        if not m:
            continue
        tip = "gps" if m.group(1).startswith("gps") else "vis"
        tarih, saat = m.group(2), m.group(3)
        sn = (int(saat[:2]) * 3600 + int(saat[2:4]) * 60 + int(saat[4:6])
              + int(tarih) * 86400)          # gün sınırında kaba ama yeterli
        rows = _oku(p)
        if _sentetik(p, rows):
            continue
        kayitlar.append((sn, tip, p, rows, f"{tarih}_{saat}"))
    kayitlar.sort()
    gruplar = []
    for k in kayitlar:
        if not gruplar or k[0] - gruplar[-1][-1][0] > 120:
            gruplar.append([])
        gruplar[-1].append(k)
    return [(g[0][4], [(tip, p, rows) for _, tip, p, rows, _ in g]) for g in gruplar]


# ── metrikler ──────────────────────────────────────────────────────────────

def _gps_metrik(dosyalar):
    dh, kapanma, kadyaw, kadelev = [], {}, [], []
    dikey_acik, dikey_pri, n = [], 0, 0
    for _, rows in dosyalar:
        onceki = None
        for r in rows:
            d = _f(r.get("d_h"))
            t = _f(r.get("t"))
            if d is None or t is None:
                continue
            n += 1
            dh.append(d)
            if onceki is not None and t > onceki[1]:
                band = ("50+" if onceki[0] > 50 else "20-50" if onceki[0] > 20
                        else "10-20" if onceki[0] > 10 else "<10")
                kapanma.setdefault(band, []).append(-(d - onceki[0]) / (t - onceki[1]))
            onceki = (d, t)
            if r.get("durum") == "KILIT":
                ky, ke = _f(r.get("kadraj_yaw_deg")), _f(r.get("kadraj_elev_deg"))
                if ky is not None:
                    kadyaw.append(ky)
                if ke is not None:
                    kadelev.append(ke)
            iz, tz = _f(r.get("iris_z")), _f(r.get("tgt_z"))
            if iz is not None and tz is not None:
                dikey_acik.append(iz - tz)      # + = hedef yukarıda
            if r.get("dikey_oncelik") == "1":
                dikey_pri += 1
    if not dh:
        return None
    # Oturma: İLK kez istasyon bandına (d_h<15) girdikten SONRAKİ karelerde ölç
    # (uzun ilk yaklaşma yüzdesi sulandırmasın).
    try:
        ilk15 = next(i for i, d in enumerate(dh) if d < 15.0)
        oturma = 100.0 * sum(1 for d in dh[ilk15:] if 8.0 <= d <= 12.0) / len(dh[ilk15:])
    except StopIteration:
        oturma = 0.0

    def p90(v):
        s = sorted(v)
        return s[min(len(s) - 1, int(0.9 * len(s)))]

    return {
        "kare": n,
        "oturma_%": oturma,
        "min_d_h": min(dh), "son_d_h": dh[-1],
        # p90: "sıcak yaklaşma" sinyali (medyan, kaçış fazlarıyla sulanıyor)
        "kapanma": {b: p90(v) for b, v in kapanma.items() if len(v) > 3},
        "kadraj_yaw_rms": math.sqrt(st.mean(v * v for v in kadyaw)) if kadyaw else None,
        "kadraj_elev_ort": st.mean(kadelev) if kadelev else None,
        "dikey_acik_max": max(dikey_acik) if dikey_acik else None,
        "dikey_acik_son": dikey_acik[-1] if dikey_acik else None,
        "dikey_oncelik_%": 100.0 * dikey_pri / n,
    }


def _vis_metrik(dosyalar):
    fazlar, yawlar, leadler = [], [], []
    kalite_k, temas_v, toplam = 0, False, 0
    for p, rows in dosyalar:
        mg = [_f(r["menzil_gercek_m"]) for r in rows if _f(r.get("menzil_gercek_m")) is not None]
        durumlar = Counter(r.get("durum") for r in rows)
        t0, t1 = _f(rows[0].get("t_ros")), _f(rows[-1].get("t_ros"))
        sure = (t1 - t0) if (t0 is not None and t1 is not None) else None
        sonuc = ("VURULDU" if durumlar.get("vuruldu") else
                 "kayıp" if durumlar.get("tespit_yok", 0) >= 15 else "ıska/koptu")
        if durumlar.get("vuruldu"):
            temas_v = True
        fazlar.append({
            "ad": os.path.basename(p)[-10:-4], "kare": len(rows), "sure": sure,
            "devir_menzil": mg[0] if mg else None,
            "min_menzil": min(mg) if mg else None,
            "son_menzil": mg[-1] if mg else None,
            "sonuc": sonuc, "durumlar": dict(durumlar),
        })
        toplam += len(rows)
        # 2026-08-06: keypoint'ler kalktı; "kalite" artık kutu ölçeğinden gelen
        # algı kalitesi (0..1). Metrik adı da ona göre: kalite oranı.
        kalite_k += sum(1 for r in rows if r.get("kalite"))
        yawlar += [_f(r["yaw_hata_deg"]) for r in rows if _f(r.get("yaw_hata_deg")) is not None]
        # 2026-08-06: lead artık şekilden değil azimut oranından geliyor
        # (adapter_copter._yatay_pn). Sütun adı lead_deg → yatay_lead_deg.
        leadler += [_f(r["yatay_lead_deg"]) for r in rows
                    if _f(r.get("yatay_lead_deg")) is not None]
    if not fazlar:
        return None
    return {
        "faz_sayisi": len(fazlar), "fazlar": fazlar,
        "kalite_orani_%": 100.0 * kalite_k / toplam if toplam else 0.0,
        "yaw_rms": math.sqrt(st.mean(v * v for v in yawlar)) if yawlar else None,
        "lead_ort": st.mean(leadler) if leadler else None,
        "en_yakin": min(f["min_menzil"] for f in fazlar if f["min_menzil"] is not None)
                    if any(f["min_menzil"] is not None for f in fazlar) else None,
        "vurus": temas_v,
    }


_HIZLANMA_ESIK = 15.0     # m/s — "faz hızına ulaştı" sayılan yatay hız
_KISA_FAZ_S = 3.0         # s — bunun altındaki görsel faz "sahte" sayılır
_GERCEK_DEVIR_M = 15.0    # m — devir kapısı 20 m; altı yeniden-giriş demek


def _gecis_metrik(dosyalar):
    """FAZ GEÇİŞİ SAĞLIĞI — 2026-08-09'da eklendi.

    NEDEN VAR: kullanıcı "gps kopup duruyo, sürekli gps-görsel geçişi oluyor"
    dedi ve hiçbir araç bunu raporlamıyordu. Karne faz SAYISINI basıyordu ama
    bir uçuşta 22 faz olmasının anormal olduğunu söyleyen bir ölçüt yoktu.

    Ölçülen (08-09, 23 GPS + 22 görsel faz / 7,5 dk — beklenen 1):
        GPS fazı başlangıç hızı medyan 0.16 m/s   → araç her geçişte DURUYOR
        15 m/s'e hiç ulaşamayan GPS fazı 9/23
        görsel fazların %45'i 3 s'den kısa
        görsel fazların %77'si fly-past ('gecildi') ile bitiyor
        devir menzili İKİLİ: yarısı ~19.6 m (gerçek), yarısı ~9.6 m (yeniden-giriş)

    Sağlıklı bir uçuşta beklenen: az sayıda faz, kısa faz oranı düşük,
    GPS fazları faz hızına ULAŞIYOR.
    """
    gps_ilk_hiz, gps_ulasamadi, gps_n = [], 0, 0
    for _, rows in dosyalar.get("gps", []):
        v = []
        for r in rows:
            vx, vy, t = _f(r.get("vx_cmd")), _f(r.get("vy_cmd")), _f(r.get("t"))
            if vx is None or vy is None or t is None:
                continue
            v.append((t, math.hypot(vx, vy)))
        if len(v) < 5:
            continue
        gps_n += 1
        gps_ilk_hiz.append(v[0][1])
        if not any(h >= _HIZLANMA_ESIK for _, h in v):
            gps_ulasamadi += 1

    sureler, girisler, gecildi, vis_n = [], [], 0, 0
    for _, rows in dosyalar.get("vis", []):
        t = [_f(r.get("t_ros")) for r in rows]
        t = [x for x in t if x is not None]
        if len(t) < 2:
            continue
        vis_n += 1
        sureler.append(t[-1] - t[0])
        m = [_f(r.get("menzil_gercek_m")) for r in rows]
        m = [x for x in m if x is not None]
        if m:
            girisler.append(m[0])
        if any(r.get("durum") == "gecildi" for r in rows):
            gecildi += 1

    if not gps_n and not vis_n:
        return None
    return {
        "gps_faz": gps_n, "vis_faz": vis_n,
        "gps_ilk_hiz_med": st.median(gps_ilk_hiz) if gps_ilk_hiz else None,
        "gps_ulasamadi": gps_ulasamadi,
        "gps_ulasamadi_%": 100.0 * gps_ulasamadi / gps_n if gps_n else None,
        "vis_sure_med": st.median(sureler) if sureler else None,
        "vis_kisa_%": (100.0 * sum(1 for s in sureler if s < _KISA_FAZ_S)
                       / len(sureler)) if sureler else None,
        "gecildi_%": 100.0 * gecildi / vis_n if vis_n else None,
        # Yeniden-giriş: devir kapısı 20 m; 15 m'nin altında başlayan faz
        # önceki fazın hemen ardından alınmış demektir.
        "yeniden_giris_%": (100.0 * sum(1 for m in girisler if m < _GERCEK_DEVIR_M)
                            / len(girisler)) if girisler else None,
        "giris_med": st.median(girisler) if girisler else None,
    }


def karne(etiket, dosyalar):
    gps_d = [(p, r) for t, p, r in dosyalar if t == "gps"]
    vis_d = [(p, r) for t, p, r in dosyalar if t == "vis"]
    gps = _gps_metrik(gps_d)
    vis = _vis_metrik(vis_d)
    gecis = _gecis_metrik({"gps": gps_d, "vis": vis_d})
    return {"etiket": etiket, "gps": gps, "vis": vis, "gecis": gecis}


# ── yazdırma ───────────────────────────────────────────────────────────────

def _fmt(v, birim="", nd=1):
    if v is None:
        return "—"
    return f"{v:.{nd}f}{birim}"


def yazdir(k):
    print(f"\n════════ UÇUŞ {k['etiket']} ════════")
    g = k["gps"]
    if g:
        print(f"[GPS] {g['kare']} kare")
        print(f"  istasyonda oturma (d_h 8-12 m): {_fmt(g['oturma_%'], '%')}"
              f"   min d_h: {_fmt(g['min_d_h'], ' m')} (overshoot <7 kötü)")
        prof = "  ".join(f"{b}:{_fmt(v, '', 1)}" for b, v in sorted(g["kapanma"].items()))
        print(f"  kapanma m/s (band p90 — sıcak yaklaşma): {prof}")
        print(f"  kadraj @KILIT: yaw RMS {_fmt(g['kadraj_yaw_rms'], '°')}"
              f"  elev ort {_fmt(g['kadraj_elev_ort'], '°')}"
              f" (istasyon {_ISTASYON_ELEV:.0f}°)")
        print(f"  dikey açık: max {_fmt(g['dikey_acik_max'], ' m')}"
              f"  son {_fmt(g['dikey_acik_son'], ' m')}"
              f"  | dikey öncelik %{_fmt(g['dikey_oncelik_%'], '', 0)}")
    else:
        print("[GPS] veri yok")
    v = k["vis"]
    if v:
        print(f"[GÖRSEL] {v['faz_sayisi']} faz | kalite oranı {_fmt(v['kalite_orani_%'], '%', 0)}"
              f" | yaw RMS {_fmt(v['yaw_rms'], '°')} | lead ort {_fmt(v['lead_ort'], '°')}"
              f" | en yakın {_fmt(v['en_yakin'], ' m', 2)}"
              f" | {'VURULDU ✓' if v['vurus'] else 'vuruş yok'}")
        for f in v["fazlar"]:
            print(f"    {f['ad']}: {f['kare']:4d} kare {_fmt(f['sure'], ' s')}"
                  f"  menzil {_fmt(f['devir_menzil'], '', 1)}→min {_fmt(f['min_menzil'], '', 2)}"
                  f"→{_fmt(f['son_menzil'], '', 1)}  → {f['sonuc']}")
    else:
        print("[GÖRSEL] faz yok (hiç devir olmadı)")
    gc = k.get("gecis")
    if gc:
        print(f"[GEÇİŞ] {gc['gps_faz']} GPS + {gc['vis_faz']} görsel faz"
              f"   (sağlıklı uçuşta bir avuç olmalı)")
        print(f"  GPS fazı başlangıç hızı: {_fmt(gc['gps_ilk_hiz_med'], ' m/s', 2)}"
              f"   {_HIZLANMA_ESIK:.0f} m/s'e ULAŞAMAYAN: "
              f"{gc['gps_ulasamadi']}/{gc['gps_faz']}"
              f" ({_fmt(gc['gps_ulasamadi_%'], '%', 0)})")
        print(f"  görsel faz süresi: medyan {_fmt(gc['vis_sure_med'], ' s')}"
              f"   <{_KISA_FAZ_S:.0f} s olan: {_fmt(gc['vis_kisa_%'], '%', 0)}"
              f"   fly-past ile biten: {_fmt(gc['gecildi_%'], '%', 0)}")
        print(f"  devir menzili medyan {_fmt(gc['giris_med'], ' m')}"
              f"   <{_GERCEK_DEVIR_M:.0f} m'de başlayan (yeniden-giriş): "
              f"{_fmt(gc['yeniden_giris_%'], '%', 0)}")
        _uyari = []
        if gc["gps_ilk_hiz_med"] is not None and gc["gps_ilk_hiz_med"] < 2.0:
            _uyari.append("araç her geçişte DURUYOR")
        if gc["gps_ulasamadi_%"] is not None and gc["gps_ulasamadi_%"] > 20:
            _uyari.append("GPS fazlarının çoğu faz hızına ulaşamıyor")
        if gc["yeniden_giris_%"] is not None and gc["yeniden_giris_%"] > 30:
            _uyari.append("faz SALINIMI (yeniden-giriş oranı yüksek)")
        if _uyari:
            print("  ⚠ " + "  ·  ".join(_uyari))


def kiyasla(k1, k2):
    print(f"\n════════ KIYAS: {k1['etiket']}  vs  {k2['etiket']} ════════")

    def satir(ad, f1, f2, birim="", nd=1):
        print(f"  {ad:<32} {_fmt(f1, birim, nd):>12} {_fmt(f2, birim, nd):>12}")

    g1, g2 = k1["gps"], k2["gps"]
    print(f"  {'':<32} {k1['etiket'][-6:]:>12} {k2['etiket'][-6:]:>12}")
    if g1 and g2:
        satir("GPS oturma %", g1["oturma_%"], g2["oturma_%"], "%")
        satir("GPS min d_h (m)", g1["min_d_h"], g2["min_d_h"])
        satir("GPS kadraj yaw RMS (°)", g1["kadraj_yaw_rms"], g2["kadraj_yaw_rms"])
        satir("GPS dikey açık max (m)", g1["dikey_acik_max"], g2["dikey_acik_max"])
        for b in ("50+", "20-50", "10-20", "<10"):
            satir(f"GPS kapanma {b} (m/s)", g1["kapanma"].get(b), g2["kapanma"].get(b))
    v1, v2 = k1["vis"], k2["vis"]
    if v1 and v2:
        satir("GÖRSEL faz sayısı", v1["faz_sayisi"], v2["faz_sayisi"], "", 0)
        satir("GÖRSEL kalite oranı %", v1["kalite_orani_%"], v2["kalite_orani_%"], "%", 0)
        satir("GÖRSEL yaw RMS (°)", v1["yaw_rms"], v2["yaw_rms"])
        satir("GÖRSEL en yakın (m)", v1["en_yakin"], v2["en_yakin"], "", 2)
        satir("VURUŞ", 1.0 if v1["vurus"] else 0.0, 1.0 if v2["vurus"] else 0.0, "", 0)
    c1, c2 = k1.get("gecis"), k2.get("gecis")
    if c1 and c2:
        satir("GEÇİŞ GPS faz sayısı", c1["gps_faz"], c2["gps_faz"], "", 0)
        satir("GEÇİŞ GPS ilk hız (m/s)", c1["gps_ilk_hiz_med"], c2["gps_ilk_hiz_med"], "", 2)
        satir(f"GEÇİŞ {_HIZLANMA_ESIK:.0f} m/s'e ulaşamayan %",
              c1["gps_ulasamadi_%"], c2["gps_ulasamadi_%"], "%", 0)
        satir("GEÇİŞ görsel faz süresi (s)", c1["vis_sure_med"], c2["vis_sure_med"])
        satir("GEÇİŞ kısa faz %", c1["vis_kisa_%"], c2["vis_kisa_%"], "%", 0)
        satir("GEÇİŞ yeniden-giriş %", c1["yeniden_giris_%"], c2["yeniden_giris_%"], "%", 0)


def _bul(gruplar, anahtar):
    adaylar = [g for g in gruplar if anahtar in g[0]]
    if not adaylar:
        raise SystemExit(f"'{anahtar}' damgalı uçuş yok — --liste ile bak")
    return adaylar[-1]


def main():
    ap = argparse.ArgumentParser(description="Uçuş karnesi (güdüm metrikleri)")
    ap.add_argument("--liste", action="store_true")
    ap.add_argument("--ucus", help="damga (örn 1308 veya 20260802_1308)")
    ap.add_argument("--kiyasla", nargs=2, metavar=("A", "B"))
    args = ap.parse_args()

    gruplar = ucuslar()
    if not gruplar:
        raise SystemExit("logs/ içinde uçuş bulunamadı")
    if args.liste:
        for etiket, dosyalar in gruplar:
            tipler = Counter(t for t, _, _ in dosyalar)
            print(f"  {etiket}: {tipler.get('gps', 0)} gps + {tipler.get('vis', 0)} görsel faz")
        return
    if args.kiyasla:
        a = _bul(gruplar, args.kiyasla[0])
        b = _bul(gruplar, args.kiyasla[1])
        ka, kb = karne(*a), karne(*b)
        yazdir(ka)
        yazdir(kb)
        kiyasla(ka, kb)
        return
    secim = _bul(gruplar, args.ucus) if args.ucus else gruplar[-1]
    yazdir(karne(*secim))


if __name__ == "__main__":
    main()
