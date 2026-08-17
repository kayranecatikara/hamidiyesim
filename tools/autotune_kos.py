#!/usr/bin/env python3
"""AVCI DRONE AUTOTUNE — ArduPilot'un kendi kazanç ayarlayıcısı.

Zarf büyütmesinden (rotor itkisi ×2.5, ANGLE_MAX 45°→70°) sonra iç döngü
kazançları elle ölçeklendi ve ÖLÇÜLDÜ: hız döngüsü yatış hatası ort 51.8 /
p90 88.9 °/s, PID çıkışı p90 yalnız 0.49, doyum %1.2 → kazançlar ÇOK DÜŞÜK.
AUTOTUNE bunu firmware'in kendisine buldurur.

Kullanım: python3 tools/autotune_kos.py [süre_sn]
"""
import sys, time, math
sys.path.insert(0, __file__.rsplit("/", 2)[0])
from pymavlink import mavutil

COPTER_MODE_GUIDED = 4
COPTER_MODE_LOITER = 5
COPTER_MODE_AUTOTUNE = 21
GAINS = ["ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D",
         "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D"]


def oku(conn, ad, timeout=4.0):
    conn.mav.param_request_read_send(conn.target_system, conn.target_component,
                                     ad.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if m and m.param_id.strip("\x00") == ad:
            return m.param_value
    return None


def yaz(conn, ad, v):
    conn.mav.param_set_send(conn.target_system, conn.target_component,
                            ad.encode(), float(v),
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)


def mod(conn, m):
    conn.mav.set_mode_send(conn.target_system,
                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, m)


def main():
    sure = float(sys.argv[1]) if len(sys.argv) > 1 else 900.0
    conn = mavutil.mavlink_connection("udp:127.0.0.1:14541")
    conn.wait_heartbeat()
    print(f"[AT] baglandi sysid={conn.target_system}")

    print("[AT] ONCEKI KAZANCLAR:")
    onceki = {g: oku(conn, g) for g in GAINS}
    for g, v in onceki.items():
        print(f"      {g:16} {v}")

    # AUTOTUNE tum eksenler: 1=roll 2=pitch 4=yaw -> 7
    yaz(conn, "AUTOTUNE_AXES", 7)
    print(f"[AT] AUTOTUNE_AXES = {oku(conn,'AUTOTUNE_AXES')}")

    print("[AT] GUIDED + arm + kalkis 40 m")
    mod(conn, COPTER_MODE_GUIDED); time.sleep(1)
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                               0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(2)
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                               0, 0, 0, 0, 0, 0, 0, 40)
    t0 = time.time()
    while time.time() - t0 < 45:
        m = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if m and m.relative_alt / 1000.0 > 35:
            break
    print("[AT] irtifada, LOITER'a gecis")
    mod(conn, COPTER_MODE_LOITER); time.sleep(3)
    print("[AT] AUTOTUNE BASLIYOR — arac kendi kendine sarsacak")
    mod(conn, COPTER_MODE_AUTOTUNE)

    t0, son = time.time(), 0.0
    bitti = False
    while time.time() - t0 < sure and not bitti:
        m = conn.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=2)
        if m is None:
            continue
        if m.get_type() == "STATUSTEXT":
            s = m.text.strip()
            if any(k in s for k in ("AutoTune", "Autotune", "AUTOTUNE")):
                print(f"[AT] {time.time()-t0:6.0f}s  {s}")
                if "Success" in s or "success" in s:
                    bitti = True
        elif m.get_type() == "HEARTBEAT" and time.time() - son > 30:
            son = time.time()
            print(f"[AT] {time.time()-t0:6.0f}s  ... mod={m.custom_mode}")

    print(f"[AT] {'TAMAMLANDI' if bitti else 'SURE DOLDU / bitmedi'}")
    print("[AT] SONRAKI KAZANCLAR:")
    for g in GAINS:
        v = oku(conn, g)
        o = onceki.get(g)
        d = f"  ({o} -> {v})" if o is not None and v is not None and abs(v-o) > 1e-9 else "  (degismedi)"
        print(f"      {g:16} {v}{d}")
    print("\n[AT] NOT: AUTOTUNE kazanclari yalniz DISARM sonrasi kalici olur.")


main()
