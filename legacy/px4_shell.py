#!/usr/bin/env python3
"""
px4_shell.py — MAVLink SERIAL_CONTROL üzerinden PX4 SITL shell'e erişir.
commander status ve commander check çalıştırarak ARM red sebebini bulur.
"""
import time
import sys
from pymavlink import mavutil

PLANE_PORT = 14550
PLANE_SYS_ID = 3
MY_SYS = 251

def connect():
    conn = mavutil.mavlink_connection(
        f"udpin:127.0.0.1:{PLANE_PORT}",
        source_system=MY_SYS,
    )
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.get_srcSystem() == PLANE_SYS_ID:
            conn.target_system = PLANE_SYS_ID
            conn.target_component = msg.get_srcComponent()
            print(f"[SHELL] Bağlandı: sys={PLANE_SYS_ID}")
            return conn
    raise TimeoutError("Plane bulunamadı")

def shell_cmd(conn, cmd: str, wait: float = 2.0):
    """PX4 shell'e komut gönderir ve yanıtı okur."""
    data = (cmd + "\n").encode('utf-8')
    # SERIAL_CONTROL ile shell komutu gönder
    conn.mav.serial_control_send(
        mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,  # device = shell
        mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE,
        0,       # timeout ms
        0,       # baudrate
        len(data),
        list(data) + [0] * (70 - len(data))
    )
    # Yanıt topla
    output = []
    deadline = time.time() + wait
    while time.time() < deadline:
        msg = conn.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.3)
        if msg and msg.count > 0:
            chunk = bytes(msg.data[:msg.count]).decode('utf-8', errors='replace')
            output.append(chunk)
    return "".join(output)

def read_estimator_status(conn):
    """ESTIMATOR_STATUS'u okur — EKF2 sağlığı."""
    msg = conn.recv_match(type='ESTIMATOR_STATUS', blocking=True, timeout=3.0)
    if not msg:
        return None
    flags = msg.flags
    # EKF2 flag bitleri (PX4 spesifik)
    flag_names = {
        0x0001: "attitude",
        0x0002: "velocity horiz",
        0x0004: "velocity vert",
        0x0008: "pos horiz rel",
        0x0010: "pos horiz abs",
        0x0020: "pos vert abs",
        0x0040: "pos vert agl",
        0x0080: "const pos mode",
        0x0100: "pred pos horiz rel",
        0x0200: "pred pos horiz abs",
        0x0400: "GPS glitch",
        0x0800: "accel error",
    }
    print(f"  ESTIMATOR_STATUS flags: 0x{flags:04X}")
    for bit, name in flag_names.items():
        ok = "✓" if (flags & bit) else "✗"
        print(f"    {ok} {name}")
    return flags

def main():
    conn = connect()

    # 1. GCS heartbeat (2 sn)
    print("\n[SHELL] GCS heartbeat...")
    for _ in range(10):
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
        time.sleep(0.2)

    # 2. ESTIMATOR_STATUS oku
    print("\n[SHELL] --- ESTIMATOR_STATUS ---")
    flags = read_estimator_status(conn)
    if flags is None:
        print("  ESTIMATOR_STATUS alınamadı (stream kapalı olabilir)")

    # 3. PX4 shell: commander status
    print("\n[SHELL] --- commander status ---")
    result = shell_cmd(conn, "commander status", wait=3.0)
    if result.strip():
        print(result)
    else:
        print("  (yanıt yok — SERIAL_CONTROL desteklenmiyor olabilir)")

    # 4. PX4 shell: commander check
    print("\n[SHELL] --- commander check ---")
    result2 = shell_cmd(conn, "commander check", wait=3.0)
    if result2.strip():
        print(result2)
    else:
        print("  (yanıt yok)")

    # 5. STATUSTEXT'leri ARM'dan ÖNCE başla dinlemeye — arka planda
    print("\n[SHELL] --- STATUSTEXT + ARM denemesi ---")
    print("Tüm STATUSTEXT'ler (10 sn boyunca):")
    
    # MANUAL mod
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        1, 1, 0, 0, 0, 0, 0,
    )
    time.sleep(0.5)

    # ARM gönder
    print("[SHELL] ARM (force=21196) gönderiliyor...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, 21196.0, 0, 0, 0, 0, 0,
    )

    # 10 saniye tüm mesajları dinle
    deadline = time.time() + 10
    seen_types = set()
    while time.time() < deadline:
        msg = conn.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'STATUSTEXT':
            print(f"  📢 STATUSTEXT [{msg.severity}]: {msg.text.strip()}")
        elif t == 'COMMAND_ACK':
            r = "✓ OK" if msg.result == 0 else "✗ RED"
            print(f"  ACK cmd={msg.command} result={msg.result} ({r})")
        elif t not in seen_types:
            seen_types.add(t)
            # print(f"  MSG_TYPE: {t}")  # debug

    print("\n[SHELL] Tamamlandı.")

if __name__ == "__main__":
    main()
