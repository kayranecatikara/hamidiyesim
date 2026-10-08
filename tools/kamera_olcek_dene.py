#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
KAMERA ÇÖZÜNÜRLÜK ÖLÇÜMÜ — "bu makine hangi çözünürlüğü gerçek zamanda render eder?"
================================================================================
NİYE VAR:
  Gerçek kamera Waveshare IMX477-160: sensör 4056×3040 (12.3 MP). Simde bunu
  30 Hz render etmek gerçek-zaman çarpanını (RTF) yere serebilir. "Sensörün
  çözünürlüğü budur, o hâlde simde de bu olsun" demek ÖLÇÜM DEĞİLDİR.

  İki şey birden ölçülür:
    1. GERÇEK-ZAMAN ÇARPANI (RTF, real-time factor): simülasyonun sim-saniyesi
       başına kaç duvar-saati saniyesi harcandığı. RTF = 1.0 → sim gerçek
       zamanda akıyor. RTF = 0.5 → sim gerçeğin YARISI hızda; ArduPilot
       lock-step'te olduğu için uçuş da yavaşlar ve ölçümler bozulur.
    2. KAMERA KARE HIZI: /iris_cam/image konusuna saniyede kaç kare düşüyor.
       İstenen 30; altına düşüyorsa render darboğaz demektir.

⛔ İLK SÜRÜM GEÇERSİZ ÖLÇÜM ÜRETTİ (2026-09-06):
  Yalnız `gz sim` başlatılıyordu, ArduPilot SITL başlatılmıyordu. Ama
  iris_cam/model.sdf'teki ArduPilotPlugin `<lock_step>1</lock_step>` ile
  çalışır: Gazebo HER FİZİK ADIMINDA uçuş kontrolcüsünden cevap bekler.
  SITL yokken Gazebo aç kalıyor ve RTF her çözünürlükte 0.22-0.29 çıkıyordu —
  640×480'de bile. Yani render maliyeti DEĞİL, boşta bekleme ölçülmüştü.
  (CLAUDE.md §5.1: ölçüm koşulu sağlanmadıysa o koşu veri noktası değildir.)

  Doğrusu TAM YIĞINI başlatmaktır: gz sim + ArduCopter SITL + ArduPlane SITL.

KULLANIM (hiçbir sim ayakta olmamalı — betik kendi başlatır):
    python3 tools/kamera_olcek_dene.py
================================================================================
"""
import os
import re
import subprocess
import sys
import threading
import time

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDF = os.path.join(KOK, "sim/gazebo_harmonic/models/iris_cam/model.sdf")
DUNYA = os.path.join(KOK, "sim/gazebo_harmonic/worlds/avci_harmonic.sdf")
BOYUTLAR = [(640, 480), (1280, 960), (2028, 1520), (4056, 3040)]
OLCUM_S = 12.0


def cozunurluk_yaz(w, h):
    s = open(SDF, encoding="utf-8").read()
    s = re.sub(r"(<sensor name=\"camera\".*?<image>\s*<width>)\d+(</width>\s*<height>)\d+(</height>)",
               r"\g<1>%d\g<2>%d\g<3>" % (w, h), s, flags=re.S)
    open(SDF, "w", encoding="utf-8").write(s)
    yeni = re.search(r"<sensor name=\"camera\".*?<width>(\d+)</width>\s*<height>(\d+)</height>",
                     open(SDF, encoding="utf-8").read(), re.S)
    return (int(yeni.group(1)), int(yeni.group(2))) if yeni else (None, None)


def yigin_durdur():
    """gz sim + iki SITL + mavproxy — hepsini indir."""
    desen = ("gz sim", "gz-sim-server", "sim_vehicle", "mavproxy",
             "bin/arducopter", "bin/arduplane")
    for p in subprocess.run(["ps", "-eo", "pid,args"], capture_output=True,
                            text=True).stdout.splitlines():
        if any(d in p for d in desen) and "grep" not in p and "kamera_olcek" not in p:
            subprocess.run(["kill", "-9", p.split()[0]], capture_output=True)
    time.sleep(3)


def _ortam():
    o = dict(os.environ)
    o["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.path.expanduser("~/ardupilot_gazebo/build")
    o["GZ_SIM_RESOURCE_PATH"] = ":".join([
        os.path.join(KOK, "sim/gazebo_harmonic/models"),
        os.path.expanduser("~/ardupilot_gazebo/models"),
        os.path.expanduser("~/ardupilot_gazebo/worlds")])
    o["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    o.pop("DISPLAY", None)
    return o


def yigin_baslat(ortam):
    """gz sim (headless) + ArduCopter SITL + ArduPlane SITL.
    lock_step yüzünden ÜÇÜ DE gerekir; yoksa Gazebo boşta bekler."""
    AP = os.path.expanduser("~/ardupilot")
    APT = os.path.join(AP, "Tools/autotest")
    log = open("/tmp/gz_olcek.log", "w")
    sur = [subprocess.Popen(
        ["gz", "sim", "-s", "-r", "--headless-rendering", "-v1", DUNYA],
        env=ortam, stdout=log, stderr=log, stdin=subprocess.DEVNULL,
        start_new_session=True)]
    time.sleep(4)
    for arac, kanat, model, inst, sysid, ek in (
            ("ArduCopter", "gazebo-iris", "JSON", 0, 5,
             ["--add-param-file=%s/default_params/copter.parm" % APT,
              "--add-param-file=%s/default_params/gazebo-iris.parm" % APT,
              "--add-param-file=%s/sim/ardupilot_params/avci_copter.parm" % KOK]),
            ("ArduPlane", "plane", "JSON:127.0.0.1:9012", 1, 2,
             ["--add-param-file=%s/sim/ardupilot_params/avci_plane.parm" % KOK])):
        sur.append(subprocess.Popen(
            ["python3", os.path.join(APT, "sim_vehicle.py"), "-v", arac,
             "-f", kanat, "--model", model, "-I%d" % inst, "--sysid", str(sysid),
             "--no-rebuild", "--mavproxy-args=--daemon"] + ek,
            cwd=AP, env=ortam, stdout=log, stderr=log,
            stdin=subprocess.DEVNULL, start_new_session=True))
    return sur


def olc(w, h):
    yigin_durdur()
    ortam = _ortam()
    sur = yigin_baslat(ortam)
    from gz.transport13 import Node as GzNode
    from gz.msgs10.image_pb2 import Image as GzImage
    sayac = {"n": 0, "t0": None, "w": 0, "h": 0}

    def cb(m):
        if sayac["t0"] is None:
            sayac["t0"] = time.time()
            sayac["w"], sayac["h"] = m.width, m.height
        sayac["n"] += 1

    node = GzNode()
    node.subscribe(GzImage, "/iris_cam/image", cb)

    # ── MEKANİZMA KAPISI: RTF gerçekten 1'e yaklaşana kadar bekle.
    #   SITL'lerin EKF kilidi ~60 s sürüyor; o bitmeden ölçüm başlarsa
    #   yine boşta bekleme ölçülür.
    t0 = time.time()
    hazir = False
    while time.time() - t0 < 150:
        r = subprocess.run(["gz", "topic", "-e", "-t", "/world/avci/stats", "-n", "1"],
                           capture_output=True, text=True, timeout=8, env=ortam).stdout
        m = re.search(r"real_time_factor:\s*([0-9.eE+-]+)", r)
        if m and float(m.group(1)) > 0.8:
            hazir = True
            break
        time.sleep(2)
    if not hazir or sayac["n"] == 0:
        for p in sur:
            p.kill()
        return None
    sayac["n"], sayac["t0"] = 0, time.time()

    rtf = []
    son = time.time() + OLCUM_S
    while time.time() < son:
        r = subprocess.run(["gz", "topic", "-e", "-t", "/world/avci/stats", "-n", "1"],
                           capture_output=True, text=True, timeout=8, env=ortam).stdout
        rs = re.search(r"real_time_factor:\s*([0-9.eE+-]+)", r)
        if rs:
            rtf.append(float(rs.group(1)))
        time.sleep(0.4)
    fps = sayac["n"] / (time.time() - sayac["t0"])
    for p in sur:
        p.kill()
    yigin_durdur()
    return {"w": sayac["w"], "h": sayac["h"], "fps": fps,
            "rtf": (sum(rtf) / len(rtf)) if rtf else float("nan"),
            "rtf_min": min(rtf) if rtf else float("nan")}


def main():
    print("=" * 76)
    print("  KAMERA ÇÖZÜNÜRLÜK ÖLÇÜMÜ  (TAM YIĞIN: gz + 2 SITL, %.0f s/boyut)" % OLCUM_S)
    print("=" * 76)
    print("  %-12s %10s %12s %10s %s" % ("çözünürlük", "MP", "kare/s", "RTF ort", "RTF en düşük"))
    sonuc = []
    for w, h in BOYUTLAR:
        gw, gh = cozunurluk_yaz(w, h)
        r = olc(w, h)
        if r is None:
            print("  %-12s  ⛔ kare gelmedi" % ("%dx%d" % (w, h)))
            continue
        sonuc.append((w, h, r))
        print("  %-12s %9.1f %11.1f %10.3f %12.3f%s"
              % ("%dx%d" % (r["w"], r["h"]), w * h / 1e6, r["fps"], r["rtf"],
                 r["rtf_min"], "   ⛔ GERÇEK ZAMANI BOZUYOR" if r["rtf"] < 0.95 else ""))
    yigin_durdur()
    print()
    uygun = [s for s in sonuc if s[2]["rtf"] >= 0.95 and s[2]["fps"] >= 25]
    if uygun:
        w, h, r = uygun[-1]
        print("  ✔ GERÇEK ZAMANI BOZMAYAN EN YÜKSEK: %dx%d  (%.1f kare/s, RTF %.3f)"
              % (w, h, r["fps"], r["rtf"]))
    else:
        print("  ⛔ hiçbir boyut ölçütü geçmedi")
    return 0


if __name__ == "__main__":
    sys.exit(main())
