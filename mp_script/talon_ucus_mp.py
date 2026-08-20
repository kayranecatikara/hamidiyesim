# -*- coding: utf-8 -*-
#
# Mission Planner (IronPython 3) — Talon ELLE FIRLATMA -> rota -> basa don -> in
# =============================================================================
# Flight Data -> Scripts sekmesinden "Run script" ile calisir. Komutu BU script
# baslatir; ucusun KENDISINI depodaki KANITLANMIS modul yapar:
#     python3 -m control.talon_firlat_rota_inis
# Boylece ince ayarli inis denetleyicisi (run_plane_scenario.inis) BIREBIR
# calisir; MP tarafinda yeniden yazilmaz (bozulma riski yok).
#
# MP 14555'ten (sysid 2) baglidir; modul 14542'den baglanir — AYNI MAVProxy'nin
# AYRI cikislari, cakisma YOK. Ikisi ayni anda ucaga baglanabilir.
#
# NE UCAR: modul AUTO'ya gecince UCAGA YUKLU olan gorevi ucar. Yani MP'de Plan
# ekranindan kendi noktalarini cizip "Write" ile ucaga yazarsan ONLAR ucur.
# Gorevin SONU RTL (cmd 20) olmali — modul rota bitisini mod=RTL'den yakalar,
# sonra baslangica donup iner. (Yuklu talon_dongu.waypoints zaten RTL ile biter.)
#
# Cikti: repo/logs/talon_mp_ucus.log dosyasina yazilir; asagida MP konsoluna
# saniyede bir telemetri (irtifa/mod/hiz) basilir.

REPO = "/home/aysenur/projects/hamidiyesim"
MODUL = "control.talon_firlat_rota_inis"
LOG = REPO + "/logs/talon_mp_ucus.log"

import clr
import time
from System.Diagnostics import Process, ProcessStartInfo


def yaz(m):
    try:
        print("[TALON-MP] " + m)
    except:
        pass


def on_kontrol():
    # Telemetri geliyor mu / GPS 0,0 mi?
    try:
        lat = float(cs.lat)
        lon = float(cs.lng)
    except:
        yaz("HATA: telemetri okunamadi — MP uca bagli mi (14555)?  BASLAMADI")
        return False
    if abs(lat) < 0.0000001 and abs(lon) < 0.0000001:
        yaz("HATA: GPS 0,0 — sim/uydu hazir degil.  BASLAMADI")
        return False
    yaz("On kontrol OK — konum %.6f, %.6f  mod=%s  armed=%s" %
        (lat, lon, str(cs.mode), str(cs.armed)))
    return True


def ucusu_baslat():
    # bash -c ile modulu repo kokunden calistir, ciktiyi log dosyasina yaz.
    # ONEMLI: MP (mono) bash'i BOS PATH ile baslatiyor -> "python3: komut yok"
    # (cikis 127). Ayrica modul firlatma icin 'gz' de cagiriyor; ikisi de
    # /usr/bin'de. Bu yuzden bash'e TAM PATH export ediyoruz.
    yol = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    komut = ("export PATH=%s && cd '%s' && python3 -m %s > '%s' 2>&1"
             % (yol, REPO, MODUL, LOG))
    psi = ProcessStartInfo()
    psi.FileName = "/bin/bash"
    psi.Arguments = "-c \"" + komut + "\""
    psi.UseShellExecute = False
    psi.WorkingDirectory = REPO
    yaz("Ucus modulu baslatiliyor:  python3 -m " + MODUL)
    yaz("Cikti/log:  " + LOG)
    return Process.Start(psi)


def main():
    yaz("=== Talon ELLE FIRLATMA -> rota -> basa don -> INIS (MP baslatici) ===")
    if not on_kontrol():
        return
    proc = ucusu_baslat()
    yaz("Baslatildi. Ucusu Gazebo'da izle; telemetri asagida.")
    yaz("(Rota = uca yuklu gorev; sonu RTL olmali. Iptal icin script'i durdur")
    yaz(" + gerekirse terminalden:  pkill -f " + MODUL + ")")

    # Modul bitene kadar MP konsoluna telemetri bas.
    while proc is not None and not proc.HasExited:
        try:
            yaz("irtifa %.1f m | mod %s | hiz %.1f m/s | armed %s" %
                (float(cs.alt), str(cs.mode), float(cs.groundspeed),
                 str(cs.armed)))
        except:
            pass
        Script.Sleep(1000)

    kod = -1
    try:
        kod = int(proc.ExitCode)
    except:
        pass
    yaz("Ucus modulu bitti (cikis kodu %d). Ayrinti icin: %s" % (kod, LOG))


main()
