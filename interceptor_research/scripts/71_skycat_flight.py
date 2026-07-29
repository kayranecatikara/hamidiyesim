#!/usr/bin/env python3
"""skycat_tvbs'i ArduPilot SITL ile ucurur ve uctugunun kanitini PNG olarak alir.

Sekans (tailsitter quadplane):
    QLOITER -> arm -> VTOL kalkis (hedef irtifa) -> havada bekle
    -> (istege bagli) FBWA'ya gecis, ileri ucus -> QLAND

Onkosul: gz sim worlds/skycat_runway.sdf ve SITL ayri ayri calisiyor olmali:
    source scripts/env.sh
    gz sim -r worlds/skycat_runway.sdf
    ./scripts/72_skycat_sitl.sh

Kullanim:
    ./71_skycat_flight.py                  # 20 m'ye kalk, hover, PNG al, in
    ./71_skycat_flight.py --irtifa 30 --ileri-ucus
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent
CAPTURE = ROOT / "scripts" / "51_capture_shots.py"


def kare_al(topic: str, dosya: str) -> None:
    """51_capture_shots.py ile tek kare yakalar; hata uçuşu durdurmaz."""
    r = subprocess.run([sys.executable, str(CAPTURE), "--topic", topic, "--cikti", dosya],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(f"  [UYARI] {dosya} alinamadi")
    else:
        print(f"  [PNG] {dosya}")


def irtifa(m: mavutil.mavfile) -> float:
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    return msg.relative_alt / 1000.0 if msg else float("nan")


def mod_bekle(m: mavutil.mavfile, mod: str, timeout: float = 20) -> bool:
    m.set_mode(mod)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if msg and mavutil.mode_string_v10(msg) == mod:
            print(f"  mod = {mod}")
            return True
        m.set_mode(mod)
    print(f"  [HATA] {mod} moduna gecilemedi")
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--baglanti", default="tcp:127.0.0.1:5770")
    ap.add_argument("--irtifa", type=float, default=20.0, help="VTOL kalkis irtifasi [m]")
    ap.add_argument("--ileri-ucus", action="store_true",
                    help="hover'dan sonra FBWA'ya gecip ileri ucus yap")
    a = ap.parse_args()

    print(f"SITL'e baglaniliyor: {a.baglanti}")
    m = mavutil.mavlink_connection(a.baglanti)
    m.wait_heartbeat()
    print(f"  heartbeat: sys={m.target_system} comp={m.target_component}")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)

    print("EKF/GPS hazir bekleniyor...")
    t0 = time.time()
    while time.time() - t0 < 120:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
        if msg and msg.fix_type >= 3:
            break
    time.sleep(8)  # EKF'nin oturmasi
    print("  GPS 3D fix")

    kare_al("/skycat/yakin", "skycat_pist.png")

    if not mod_bekle(m, "QLOITER"):
        return 1

    print("Arm ediliyor...")
    # motors_armed_wait() zaman asimsiz bloke ediyor; prearm reddinde asili
    # kalmamak icin kendi dongumuz + STATUSTEXT raporu.
    t0, armed = time.time(), False
    while time.time() - t0 < 60:
        m.arducopter_arm()
        for _ in range(10):
            msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=2)
            if not msg:
                continue
            if msg.get_type() == "STATUSTEXT":
                print(f"    SITL: {msg.text}")
            elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                armed = True
                break
        if armed:
            break
    if not armed:
        print("  [HATA] arm olmadi")
        return 1
    print("  ARMED")

    print(f"VTOL kalkis: {a.irtifa} m")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, a.irtifa)

    t0, cekildi = time.time(), False
    while time.time() - t0 < 90:
        h = irtifa(m)
        print(f"  irtifa = {h:5.1f} m")
        if h > 2.0 and not cekildi:
            kare_al("/skycat/yakin", "skycat_kalkis_yakin.png")
            cekildi = True
        if h >= a.irtifa * 0.9:
            break
        time.sleep(1)

    print(f"Hover: {irtifa(m):.1f} m")
    time.sleep(3)
    kare_al("/skycat/kalkis", "skycat_hover.png")

    if a.ileri_ucus:
        print("FBWA'ya gecis (ileri ucus)")
        mod_bekle(m, "FBWA")
        # ileri itki: throttle kanali
        for _ in range(12):
            m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                            0, 0, 1800, 0, 0, 0, 0, 0)
            time.sleep(1)
        kare_al("/skycat/kalkis", "skycat_ileri_ucus.png")
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        0, 0, 0, 0, 0, 0, 0, 0)

    print("QLAND")
    mod_bekle(m, "QLAND")
    t0 = time.time()
    while time.time() - t0 < 120:
        h = irtifa(m)
        if h < 1.0:
            break
        time.sleep(2)
    kare_al("/skycat/yakin", "skycat_inis.png")
    print(f"Bitti. Son irtifa = {irtifa(m):.1f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
