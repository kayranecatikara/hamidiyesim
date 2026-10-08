#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
BORU HATTI ÇÖZÜNÜRLÜK ÖLÇÜMÜ — "hangi çözünürlükte TESPİT yetişiyor?"
================================================================================
tools/kamera_olcek_dene.py yalnız RENDER maliyetini ölçer ve 2028×1520'ye
kadar "bedava" der — doğrudur, Gazebo tarafı gerçekten bedavadır.

AMA ölçülmemiş bir şey vardı: gcs_server'ın GÖRÜNTÜ BORU HATTI (YOLO tespiti
+ takipçi + MJPEG kodlama) o kareye yetişiyor mu? Ölçüldü (2026-09-06):
    640 × 480   → 18-23 kare/s geldi, HEPSİ işlendi        (%0 düştü)
    2028 × 1520 →   18 kare/s geldi, 3.7 işlendi           (%80 DÜŞTÜ)
Yani çözünürlük 5 kat tespit hızı yiyor. Görsel güdüm (IBVS) 20 Hz döner;
3.7 tespit/s ile o döngü aç kalır.

Bu betik aday çözünürlükleri TAM BORU HATTIYLA (gz + 2 SITL + gcs_server)
koşar ve gcs_server'ın kendi [GZ-CAM] sağlık satırlarını okur.

KULLANIM:  python3 tools/boru_hatti_olc.py
================================================================================
"""
import math
import os
import re
import subprocess
import sys
import time

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDF = os.path.join(KOK, "sim/gazebo_harmonic/models/iris_cam/model.sdf")
GEO = os.path.join(KOK, "vision/geometry.py")
DUNYA = os.path.join(KOK, "sim/gazebo_harmonic/worlds/avci_harmonic.sdf")
AP = os.path.expanduser("~/ardupilot")
APT = os.path.join(AP, "Tools/autotest")

# IMX477'nin GERÇEK modları + ara adımlar (hepsi 4:3, sensörle aynı en-boy)
ADAYLAR = [(640, 480), (1014, 760), (1352, 1014)]
OLCUM_S = 45.0


def yaz_cozunurluk(w, h):
    s = open(SDF, encoding="utf-8").read()
    s = re.sub(r"(<sensor name=\"camera\".*?<image>\s*<width>)\d+(</width>\s*<height>)\d+(</height>)",
               r"\g<1>%d\g<2>%d\g<3>" % (w, h), s, flags=re.S)
    open(SDF, "w", encoding="utf-8").write(s)
    g = open(GEO, encoding="utf-8").read()
    g = re.sub(r"^IMG_W, IMG_H = \d+, \d+$", "IMG_W, IMG_H = %d, %d" % (w, h),
               g, flags=re.M)
    open(GEO, "w", encoding="utf-8").write(g)


def durdur():
    desen = ("gz sim", "gz-sim-server", "sim_vehicle", "mavproxy",
             "bin/arducopter", "bin/arduplane", "control.gcs_server")
    for p in subprocess.run(["ps", "-eo", "pid,args"], capture_output=True,
                            text=True).stdout.splitlines():
        if any(d in p for d in desen) and "grep" not in p and "boru_hatti" not in p:
            subprocess.run(["kill", "-9", p.split()[0]], capture_output=True)
    time.sleep(3)


def ortam():
    o = dict(os.environ)
    o["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.path.expanduser("~/ardupilot_gazebo/build")
    o["GZ_SIM_RESOURCE_PATH"] = ":".join([
        os.path.join(KOK, "sim/gazebo_harmonic/models"),
        os.path.expanduser("~/ardupilot_gazebo/models"),
        os.path.expanduser("~/ardupilot_gazebo/worlds")])
    o["AVCI_GZ_CAMERA"] = "1"
    o["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    o.pop("DISPLAY", None)         # tarayıcı açılmasın
    return o


def kos(w, h):
    durdur()
    yaz_cozunurluk(w, h)
    o = ortam()
    log = open("/tmp/boru_gz.log", "w")
    subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", "-v1", DUNYA],
                     env=o, stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                     start_new_session=True)
    time.sleep(5)
    for arac, kanat, model, inst, sysid, ek in (
            ("ArduCopter", "gazebo-iris", "JSON", 0, 5,
             ["--add-param-file=%s/default_params/copter.parm" % APT,
              "--add-param-file=%s/default_params/gazebo-iris.parm" % APT,
              "--add-param-file=%s/sim/ardupilot_params/avci_copter.parm" % KOK]),
            ("ArduPlane", "plane", "JSON:127.0.0.1:9012", 1, 2,
             ["--add-param-file=%s/sim/ardupilot_params/avci_plane.parm" % KOK])):
        subprocess.Popen(
            ["python3", os.path.join(APT, "sim_vehicle.py"), "-v", arac,
             "-f", kanat, "--model", model, "-I%d" % inst, "--sysid", str(sysid),
             "--no-rebuild", "--mavproxy-args=--daemon"] + ek,
            cwd=AP, env=o, stdout=log, stderr=log, stdin=subprocess.DEVNULL,
            start_new_session=True)
    time.sleep(60)
    gl = "/tmp/boru_gcs_%dx%d.log" % (w, h)
    gf = open(gl, "w")
    subprocess.Popen([sys.executable, "-m", "control.gcs_server"], cwd=KOK,
                     env=o, stdout=gf, stderr=gf, stdin=subprocess.DEVNULL,
                     start_new_session=True)
    time.sleep(OLCUM_S)
    gf.flush()
    metin = open(gl, encoding="utf-8", errors="ignore").read()
    kayit = re.findall(r"\[GZ-CAM\] ([\d.]+) kare/s geldi, ([\d.]+) işlendi, "
                       r"(\d+) düştü \(%(\d+)\)", metin)
    durdur()
    if not kayit:
        return None
    kayit = kayit[1:] or kayit          # ilk pencere ısınma
    gelen = sum(float(k[0]) for k in kayit) / len(kayit)
    islenen = sum(float(k[1]) for k in kayit) / len(kayit)
    dusen = sum(int(k[3]) for k in kayit) / len(kayit)
    return {"gelen": gelen, "islenen": islenen, "dusen": dusen}


def main():
    print("=" * 76)
    print("  BORU HATTI ÇÖZÜNÜRLÜK ÖLÇÜMÜ (gz + 2 SITL + gcs_server, %.0f s)"
          % OLCUM_S)
    print("=" * 76)
    print("  %-12s %8s %11s %11s %9s  %s"
          % ("çözünürlük", "MP", "gelen k/s", "işlenen k/s", "düşen %", "hüküm"))
    sonuc = []
    for w, h in ADAYLAR:
        r = kos(w, h)
        if r is None:
            print("  %-12s  ⛔ ölçüm alınamadı" % ("%dx%d" % (w, h)))
            continue
        sonuc.append((w, h, r))
        yeter = r["islenen"] >= 15.0
        print("  %-12s %7.2f %10.1f %11.1f %8.0f%%  %s"
              % ("%dx%d" % (w, h), w * h / 1e6, r["gelen"], r["islenen"],
                 r["dusen"], "✔ yetişiyor" if yeter else "⛔ tespit hızı düşük"))
    uygun = [s for s in sonuc if s[2]["islenen"] >= 15.0]
    print()
    if uygun:
        w, h, r = uygun[-1]
        print("  ✔ TESPİT HIZI KORUNARAK EN YÜKSEK: %dx%d (%.1f tespit/s)"
              % (w, h, r["islenen"]))
        print("     açısal çözünürlük %.4f °/px  (FX = %.1f px)"
              % (128.0 / w, (w / 2.0) / math.tan(math.radians(64.0))))
    else:
        print("  ⛔ hiçbir aday 15 tespit/s eşiğini geçmedi")
    return 0


if __name__ == "__main__":
    sys.exit(main())
