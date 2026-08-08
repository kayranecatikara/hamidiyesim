#!/usr/bin/env python3
"""Vuruş oranı ölçümü: her TERMİNAL HÜCUM bir deneme, kaçı isabet?

⚠ NEDEN VAR (2026-08-08): "vurduk" tek bir uçuşun tek olayıdır — tekrarlanabilir
mi bilinmez. Kullanıcı hedefi: DÜZ UÇUŞTA HER DENEME İSABET (ıska yok). Bu araç
o oranı ölçer ve ıskaların dik mesafesini (yanal + dikey) çıkarır ki düzeltme
doğru eksene yapılsın.

Kullanım (sim + gcs_server çalışırken, mod hybrid):
    python3 tools/vurus_orani.py <cikti_dizini> <kosu_sayisi> [kosu_suresi_s]

⚠ AYNI SİMDE ARDIŞIK KOŞU YAPILAMAZ (2026-08-08'de ölçümle görüldü):
Vuruştan sonra hedef uçak İMHA olur ve düşer; sonraki koşuda kalkamaz.
Ölçümde koşu-3 tam böyle geçersiz çıktı (hedef irtifası 94 → −233 m,
yani enkaz yerin altına savrulmuştu) ve "ıska" gibi göründü. Bu araç artık
hedefin GERÇEKTEN uçtuğunu doğrular; uçmuyorsa koşuyu GEÇERSİZ işaretler,
ıska saymaz. Her geçerli koşu için sim BAŞTAN kurulmalıdır.
"""
import csv
import json
import math
import os
import subprocess
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8000"


def getir(yol):
    try:
        with urllib.request.urlopen(BASE + yol, timeout=3) as r:
            return json.loads(r.read())
    except Exception:
        return {}


def gonder(yol):
    try:
        req = urllib.request.Request(BASE + yol, method="POST", data=b"")
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())
    except Exception as e:
        return {"status": "error", "message": str(e)}


def main():
    cikti = sys.argv[1]
    kosu_sayisi = int(sys.argv[2])
    kosu_suresi = float(sys.argv[3]) if len(sys.argv) > 3 else 240.0
    os.makedirs(cikti, exist_ok=True)

    ozet = []
    for k in range(1, kosu_sayisi + 1):
        print(f"\n═══ KOŞU {k}/{kosu_sayisi} ═══", flush=True)
        gonder("/api/command/iris/stop_chase")
        gonder("/api/command/plane/stop_scenario")
        time.sleep(3)
        gonder("/api/command/plane/scenario/duz")

        # hedef havalanana kadar bekle — UÇMUYORSA KOŞU GEÇERSİZ
        t0 = time.time()
        spd = 0.0
        alt = 0.0
        while time.time() - t0 < 90:
            ts = getir("/api/debug/telem").get("telemetry_state", {})
            p = ts.get("plane", {}) or {}
            spd = p.get("speed") or 0
            alt = -(p.get("z") or 0)
            if spd > 12 and 20 < alt < 300:
                break
            time.sleep(2)
        if not (spd > 12 and 20 < alt < 300):
            print(f"  ⚠ GEÇERSİZ KOŞU — hedef uçmuyor (hız {spd:.1f} m/s, "
                  f"irtifa {alt:.0f} m). Önceki vuruştan sonra enkaz kalmış "
                  f"olabilir; sim baştan kurulmalı.", flush=True)
            ozet.append({"kosu": k, "vuruldu": False, "gecersiz": True,
                         "sure": 0.0})
            continue
        gonder("/api/command/iris/start_chase")
        print(f"  chase başladı (hedef {spd:.1f} m/s, {alt:.0f} m)", flush=True)

        # 10 Hz izleme
        yol = os.path.join(cikti, f"kosu{k}.csv")
        vuruldu = False
        with open(yol, "w", newline="") as g:
            w = csv.DictWriter(g, fieldnames=[
                "t", "mesafe", "faz", "imha",
                "plane_x", "plane_y", "plane_z", "iris_x", "iris_y", "iris_z"])
            w.writeheader()
            t0 = time.time()
            while time.time() - t0 < kosu_suresi:
                t = time.time()
                ch = getir("/api/chase_status")
                hs = getir("/api/hasar")
                tel = getir("/api/debug/telem").get("telemetry_state", {})
                p = tel.get("plane", {}) or {}
                i = tel.get("iris", {}) or {}
                imha = bool(hs.get("imha"))
                w.writerow({"t": round(t, 3), "mesafe": ch.get("distance"),
                            "faz": (ch.get("supervisor") or {}).get("faz"),
                            "imha": int(imha),
                            "plane_x": p.get("x"), "plane_y": p.get("y"),
                            "plane_z": p.get("z"), "iris_x": i.get("x"),
                            "iris_y": i.get("y"), "iris_z": i.get("z")})
                g.flush()
                if imha:
                    vuruldu = True
                    print(f"  ✓ VURULDU ({time.time()-t0:.0f} s) — "
                          f"menzil {hs.get('menzil')}", flush=True)
                    break
                kalan = 0.1 - (time.time() - t)
                if kalan > 0:
                    time.sleep(kalan)
        ozet.append({"kosu": k, "vuruldu": vuruldu, "gecersiz": False,
                     "sure": round(time.time() - t0, 1)})
        if not vuruldu:
            print(f"  ✗ süre doldu ({kosu_suresi:.0f} s), vuruş yok", flush=True)

    gonder("/api/command/iris/stop_chase")
    gonder("/api/command/plane/stop_scenario")
    gecerli = [o for o in ozet if not o.get("gecersiz")]
    n_v = sum(1 for o in gecerli if o["vuruldu"])
    print(f"\n═══ SONUÇ: {n_v}/{len(gecerli)} GEÇERLİ koşuda vuruş "
          f"({len(ozet) - len(gecerli)} geçersiz) ═══", flush=True)
    for o in ozet:
        durum = ("GEÇERSİZ" if o.get("gecersiz")
                 else ("VURULDU" if o["vuruldu"] else "ıska"))
        print(f"  koşu {o['kosu']}: {durum} ({o['sure']:.0f} s)", flush=True)
    with open(os.path.join(cikti, "ozet.json"), "w") as g:
        json.dump(ozet, g, indent=2)


if __name__ == "__main__":
    main()
