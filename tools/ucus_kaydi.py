#!/usr/bin/env python3
"""Uçuş gözlem kaydı: saniyede 1 kamera karesi + o anki panel telemetrisi.

Otonom uçuş testinin kayıt bacağı (bkz. docs/OTONOM_UCUS_TESTI.md).
gcs_server'ın MJPEG akışından (/api/video_feed/iris) en son kareyi tutar,
her saniye diske yazar ve aynı ana ait panel verisini (mesafe, senaryo,
hızlar, konumlar) meta.csv'ye ekler. Böylece her kare "o anda panel ne
diyordu" bilgisiyle eşli olarak saklanır — görüntü ile log çelişirse
yakalanır (panel +8 m hatasının dersi).

Kullanım:
    python3 tools/ucus_kaydi.py <cikti_dizini> <sure_s>

Çıktı:
    <dir>/frames/f0001.jpg, f0002.jpg, ...   (kare aralığı 0.5 s)
    <dir>/meta.csv                            (kare ↔ telemetri eşleşmesi)

Video üretmek için (0.5 s kare → 5 fps ≈ 2.5× hızlandırılmış):
    ffmpeg -framerate 5 -i <dir>/frames/f%04d.jpg -c:v libx264 \
           -pix_fmt yuv420p ucus.mp4
"""
import csv
import json
import os
import sys
import threading
import time
import urllib.request

BASE = "http://127.0.0.1:8000"
BOUNDARY = b"--frame"

son_kare = {"veri": None, "t": 0.0}
kilit = threading.Lock()


def akis_okuyucu():
    """MJPEG akışını sürekli oku, en son JPEG'i bellekte tut (latest-wins)."""
    while True:
        try:
            r = urllib.request.urlopen(BASE + "/api/video_feed/iris", timeout=10)
            tampon = b""
            while True:
                parca = r.read(16384)
                if not parca:
                    break
                tampon += parca
                while True:
                    b1 = tampon.find(BOUNDARY)
                    if b1 < 0:
                        break
                    b2 = tampon.find(BOUNDARY, b1 + len(BOUNDARY))
                    if b2 < 0:
                        # tamponu şişirme: tek kare 200 KB'ı aşmaz
                        if len(tampon) > 2_000_000:
                            tampon = tampon[-len(BOUNDARY):]
                        break
                    blok = tampon[b1:b2]
                    tampon = tampon[b2:]
                    j = blok.find(b"\r\n\r\n")
                    if j >= 0:
                        jpeg = blok[j + 4:].rstrip(b"\r\n")
                        if jpeg.startswith(b"\xff\xd8"):
                            with kilit:
                                son_kare["veri"] = jpeg
                                son_kare["t"] = time.time()
        except Exception as e:
            print(f"[AKIS] koptu ({e}), 1 s sonra tekrar", flush=True)
            time.sleep(1.0)


def getir(yol):
    try:
        with urllib.request.urlopen(BASE + yol, timeout=2) as r:
            return json.loads(r.read())
    except Exception:
        return {}


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        raise SystemExit(1)
    cikti = sys.argv[1]
    sure = float(sys.argv[2])
    # Kullanıcı kuralı (2026-08-18): kare aralığı 1.0 → 0.5 s.
    aralik = float(os.environ.get("AVCI_KAYIT_ARALIK", "0.5"))
    os.makedirs(os.path.join(cikti, "frames"), exist_ok=True)

    threading.Thread(target=akis_okuyucu, daemon=True).start()

    # ── HIZLI TELEMETRİ (10 Hz) — ayrı dosya, kare kaydından bağımsız ──
    # ⚠ NEDEN: 1 Hz kare örneklemesi terminal hücumu ÖLÇEMEZ. 24 m/s kapanma
    # hızında iki örnek arası 24 m; "en yakın 0.8 m" gibi sayılar gerçek en
    # yakın an DEĞİL, sadece rastgele bir örnek. Isabet/ıska payını görmek
    # için 10 Hz şart (2.4 m çözünürlük).
    dur_bayrak = {"dur": False}

    def hizli_telem():
        yol = os.path.join(cikti, "telem.csv")
        with open(yol, "w", newline="") as g:
            tw = csv.DictWriter(g, fieldnames=[
                "wall_t", "mesafe", "faz", "plane_x", "plane_y", "plane_z",
                "iris_x", "iris_y", "iris_z"])
            tw.writeheader()
            while not dur_bayrak["dur"]:
                t = time.time()
                ch = getir("/api/chase_status")
                tel = getir("/api/debug/telem").get("telemetry_state", {})
                p = tel.get("plane", {}) or {}
                i = tel.get("iris", {}) or {}
                if ch:
                    tw.writerow({
                        "wall_t": round(t, 3), "mesafe": ch.get("distance"),
                        "faz": (ch.get("supervisor") or {}).get("faz"),
                        "plane_x": p.get("x"), "plane_y": p.get("y"),
                        "plane_z": p.get("z"), "iris_x": i.get("x"),
                        "iris_y": i.get("y"), "iris_z": i.get("z")})
                    g.flush()
                kalan = 0.1 - (time.time() - t)
                if kalan > 0:
                    time.sleep(kalan)

    threading.Thread(target=hizli_telem, daemon=True).start()

    meta_yol = os.path.join(cikti, "meta.csv")
    alanlar = ["kare", "wall_t", "kare_yasi_s", "mesafe", "chase_aktif",
               "senaryo", "plane_spd", "iris_spd",
               "plane_x", "plane_y", "plane_z", "iris_x", "iris_y", "iris_z"]
    f = open(meta_yol, "w", newline="")
    w = csv.DictWriter(f, fieldnames=alanlar)
    w.writeheader()

    t0 = time.time()
    n = 0
    while time.time() - t0 < sure:
        hedef_t = t0 + n * aralik
        bekle = hedef_t - time.time()
        if bekle > 0:
            time.sleep(bekle)
        n += 1

        with kilit:
            jpeg = son_kare["veri"]
            yas = time.time() - son_kare["t"] if jpeg else -1.0
        if jpeg:
            with open(os.path.join(cikti, "frames", f"f{n:04d}.jpg"), "wb") as g:
                g.write(jpeg)

        chase = getir("/api/chase_status")
        sen = getir("/api/scenario_status")
        tel = getir("/api/debug/telem").get("telemetry_state", {})
        p = tel.get("plane", {}) if isinstance(tel.get("plane"), dict) else {}
        i = tel.get("iris", {}) if isinstance(tel.get("iris"), dict) else {}
        w.writerow({
            "kare": n, "wall_t": round(time.time(), 2),
            "kare_yasi_s": round(yas, 2),
            "mesafe": chase.get("distance"), "chase_aktif": chase.get("active"),
            "senaryo": sen.get("name"),
            "plane_spd": p.get("speed"), "iris_spd": i.get("speed"),
            "plane_x": p.get("x"), "plane_y": p.get("y"), "plane_z": p.get("z"),
            "iris_x": i.get("x"), "iris_y": i.get("y"), "iris_z": i.get("z"),
        })
        f.flush()
        if n % 30 == 0:
            print(f"[KAYIT] {n} kare, mesafe={chase.get('distance')}", flush=True)

    dur_bayrak["dur"] = True
    time.sleep(0.2)
    f.close()
    print(f"[KAYIT] bitti: {n} kare + 10 Hz telem.csv → {cikti}", flush=True)


if __name__ == "__main__":
    main()
