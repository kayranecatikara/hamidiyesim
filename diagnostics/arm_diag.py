#!/usr/bin/env python3
"""
arm_diag.py — Plane ARM reddini teşhis eder.

1. sys=3 plane'e bağlanır
2. Parametreleri gönderir ve GERİ OKUR (uygulandı mı?)
3. STATUSTEXT dinler → ARM reddinin tam sebebini gösterir
4. SYS_STATUS'tan unhealthy bitleri okur
"""
import time
import sys
from pymavlink import mavutil

PLANE_PORT   = 14550
PLANE_SYS_ID = 3
MY_SYS       = 251

# ---------------------------------------------------------------------------
def connect():
    print(f"[DIAG] Bağlanılıyor udpin:127.0.0.1:{PLANE_PORT} ...")
    conn = mavutil.mavlink_connection(
        f"udpin:127.0.0.1:{PLANE_PORT}",
        source_system=MY_SYS,
    )
    deadline = time.time() + 15
    while time.time() < deadline:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.get_srcSystem() == PLANE_SYS_ID:
            conn.target_system    = PLANE_SYS_ID
            conn.target_component = msg.get_srcComponent()
            print(f"[DIAG] ✓ Heartbeat: sys={PLANE_SYS_ID} comp={conn.target_component}")
            return conn
    raise TimeoutError("Plane bulunamadı (sys=3)")

# ---------------------------------------------------------------------------
def set_param_int(conn, name: str, value: int):
    bname = name.encode()[:16].ljust(16, b'\x00')[:16]
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        bname, float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    # ACK bekle
    t = time.time()
    while time.time() - t < 2.0:
        m = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m and m.param_id.strip('\x00') == name:
            print(f"  [SET] {name} = {int(m.param_value)}  (hedef={value})")
            return int(m.param_value)
    print(f"  [SET] {name} → ACK yok!")
    return None

def get_param(conn, name: str):
    bname = name.encode()[:16].ljust(16, b'\x00')[:16]
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component,
        bname, -1,
    )
    t = time.time()
    while time.time() - t < 3.0:
        m = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m and m.param_id.strip('\x00') == name:
            return m.param_value
    return None

# ---------------------------------------------------------------------------
def send_gcs_hb(conn):
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )

# ---------------------------------------------------------------------------
def main():
    conn = connect()

    # GCS heartbeat
    print("\n[DIAG] GCS heartbeat gönderiliyor (3s)...")
    for _ in range(15):
        send_gcs_hb(conn)
        time.sleep(0.2)

    # Mevcut CBRK_FLIGHTCHK değerini oku
    print("\n[DIAG] --- Mevcut CBRK Parametreleri ---")
    for pname in ["CBRK_FLIGHTCHK", "CBRK_SUPPLY_CHK", "CBRK_USB_CHK",
                  "COM_ARM_WO_GPS", "SYS_HAS_GPS", "EKF2_GPS_CTRL",
                  "COM_RC_IN_MODE"]:
        v = get_param(conn, pname)
        print(f"  {pname:25s} = {v}")

    # Parametreleri ayarla
    print("\n[DIAG] --- Parametreler Ayarlanıyor ---")
    params_to_set = {
        "COM_RC_IN_MODE":   4,
        "NAV_DLL_ACT":      0,
        "NAV_RCL_ACT":      0,
        "COM_ARM_WO_GPS":   1,
        "SYS_HAS_GPS":      0,
        "EKF2_GPS_CTRL":    0,
        "COM_POWER_COUNT":  0,
        "COM_ARM_SDCARD":   0,
        "COM_ARM_CHK_ESCS": 0,
        "COM_ARM_MAG_STR":  0,
        "CBRK_SUPPLY_CHK":  894281,
        "CBRK_USB_CHK":     197848,
        "CBRK_FLIGHTCHK":   197848,   # <-- ANA KALKAN
    }
    for name, val in params_to_set.items():
        set_param_int(conn, name, val)
        send_gcs_hb(conn)
        time.sleep(0.1)

    # CBRK_FLIGHTCHK geri oku — uygulandı mı?
    print("\n[DIAG] --- CBRK_FLIGHTCHK Doğrulama ---")
    v = get_param(conn, "CBRK_FLIGHTCHK")
    print(f"  CBRK_FLIGHTCHK = {v}  (beklenen: 197848.0)")
    if v != 197848.0:
        print("  !! CBRK_FLIGHTCHK uygulanamadı — parametre yok veya reboot gerekli!")
    
    # SYS_STATUS oku — unhealthy componentler
    print("\n[DIAG] --- SYS_STATUS (Unhealthy Sistemler) ---")
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1
    )
    deadline = time.time() + 3
    sys_status = None
    while time.time() < deadline:
        m = conn.recv_match(type='SYS_STATUS', blocking=True, timeout=0.5)
        if m:
            sys_status = m
            break
    if sys_status:
        present  = sys_status.onboard_control_sensors_present
        enabled  = sys_status.onboard_control_sensors_enabled
        health   = sys_status.onboard_control_sensors_health
        unhealthy = present & enabled & ~health
        print(f"  Present : 0x{present:08X}")
        print(f"  Enabled : 0x{enabled:08X}")
        print(f"  Health  : 0x{health:08X}")
        print(f"  UNHEALTHY bits: 0x{unhealthy:08X}")
        # Bilinen bit maskeleri
        bits = {
            0x00000001: "3D Gyro",
            0x00000002: "3D Accel",
            0x00000004: "3D Mag",
            0x00000008: "Absolute Pressure",
            0x00000010: "Differential Pressure",
            0x00000020: "GPS",
            0x00000040: "Optical Flow",
            0x00000080: "Computer Vision Position",
            0x00000100: "Laser Position",
            0x00000200: "External Ground Truth",
            0x00000400: "Angular Rate Control",
            0x00000800: "Attitude Stabilization",
            0x00001000: "Yaw Position",
            0x00002000: "Z/Altitude Control",
            0x00004000: "X/Y Position Control",
            0x00008000: "Motor Outputs / ESCs",
            0x00010000: "RC Receiver",
            0x00020000: "3D Gyro 2",
            0x00040000: "3D Accel 2",
            0x00080000: "3D Mag 2",
            0x00100000: "GCS",
            0x00200000: "Battery",
        }
        for mask, label in bits.items():
            if unhealthy & mask:
                print(f"  ❌ UNHEALTHY: {label}")
    else:
        print("  SYS_STATUS alınamadı")

    # STATUSTEXT dinle — ARM reddinin tam sebebi
    print("\n[DIAG] --- MANUAL moda geç + ARM dene + STATUSTEXT oku ---")
    # MANUAL mod
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        1, 1, 0, 0, 0, 0, 0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MANUAL=1
    )
    time.sleep(1.0)

    # ARM komutu gönder — STATUSTEXT'i dinle
    print("[DIAG] ARM gönderiliyor (param2=21196 force)...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0,    # arm
        21196.0,  # force
        0, 0, 0, 0, 0,
    )

    print("[DIAG] STATUSTEXT mesajları (5 saniye):")
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = conn.recv_match(
            type=['STATUSTEXT', 'COMMAND_ACK'],
            blocking=True, timeout=0.3
        )
        if msg:
            if msg.get_type() == 'COMMAND_ACK':
                print(f"  ACK cmd={msg.command} result={msg.result} "
                      f"({'✓ ARM OK' if msg.result == 0 else '✗ ARM RED'})")
            elif msg.get_type() == 'STATUSTEXT':
                print(f"  STATUSTEXT [{msg.severity}]: {msg.text.strip()}")

    print("\n[DIAG] Teşhis tamamlandı.")

if __name__ == "__main__":
    main()
