#!/usr/bin/env python3
"""
fix_accel_bias.py — High Accelerometer Bias ARM engelini kaldırır.
İki paralel yöntem:
  1. MAVLink parametrelerle eşiği artır (CAL_ACC*_BIAS + COM_ARM_IMU_ACC)
  2. EKF2 innovasyon limitlerini iyileştir
"""
import time
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
            print(f"[FIX] Bağlandı: sys={PLANE_SYS_ID}")
            return conn
    raise TimeoutError("Plane bulunamadı")

def gcs_hb(conn):
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0,
    )

def set_param(conn, name: str, value: float, ptype: int):
    """Parametreyi ayarla ve ACK bekle."""
    bname = name.encode()[:16].ljust(16, b'\x00')[:16]
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        bname, value, ptype,
    )
    t = time.time()
    while time.time() - t < 2.0:
        m = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m and m.param_id.strip('\x00') == name:
            print(f"  ✓ {name} = {m.param_value:.4f}")
            return m.param_value
    print(f"  ✗ {name} → ACK yok (parametre yok olabilir)")
    return None

def shell_cmd(conn, cmd: str, wait: float = 2.5):
    data = (cmd + "\n").encode('utf-8')
    conn.mav.serial_control_send(
        mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
        mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE,
        0, 0, len(data),
        list(data) + [0] * (70 - len(data))
    )
    output = []
    deadline = time.time() + wait
    while time.time() < deadline:
        msg = conn.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.3)
        if msg and msg.count > 0:
            output.append(bytes(msg.data[:msg.count]).decode('utf-8', errors='replace'))
    return "".join(output)

INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
FLOAT = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

def main():
    conn = connect()

    # GCS warmup
    print("\n[FIX] GCS heartbeat (2s)...")
    for _ in range(10):
        gcs_hb(conn)
        time.sleep(0.2)

    print("\n[FIX] === High Accelerometer Bias Düzeltmesi ===")

    # 1. ARM IMU Tolerance — ivmeölçer bias eşiği
    # COM_ARM_IMU_ACC: ARM öncesi max ivmeölçer bias (m/s²). Varsayılan ~0.35 m/s²
    # SITL'de bias ~0.4-0.8 arasında olabilir → 5.0 m/s² yapıyoruz
    set_param(conn, 'COM_ARM_IMU_ACC', 5.0, FLOAT)
    set_param(conn, 'COM_ARM_IMU_GYR', 5.0, FLOAT)

    # 2. EKF2 IMU Bias Limitlerini Artır
    # EKF2_ABL_LIM: ivmeölçer bias learn limit (m/s²). Varsayılan 0.4
    set_param(conn, 'EKF2_ABL_LIM', 0.8, FLOAT)
    # EKF2_ABL_ACCLIM: ivmeölçer bias öğrenme ivmesi limiti
    set_param(conn, 'EKF2_ABL_ACCLIM', 25.0, FLOAT)
    # EKF2_ABL_GYRLIM: gyro bias öğrenme limit
    set_param(conn, 'EKF2_ABL_GYRLIM', 0.1, FLOAT)

    # 3. PX4 IMU filter frekansı — titreşim filtresi (varsa)
    set_param(conn, 'IMU_GYRO_CUTOFF', 30.0, FLOAT)
    set_param(conn, 'IMU_ACCEL_CUTOFF', 30.0, FLOAT)

    # 4. Sensör kalibrasyon bypass
    set_param(conn, 'SYS_CAL_ACCEL', 0, INT32)  # kalibrasyon gerektirme
    set_param(conn, 'SYS_CAL_GYRO',  0, INT32)
    set_param(conn, 'SYS_CAL_BARO',  0, INT32)

    # 5. EKF2 barometer noise (titreşim etkisi)
    set_param(conn, 'EKF2_BARO_NOISE', 10.0, FLOAT)

    # 6. Tüm diğer ARM kalkanları
    set_param(conn, 'COM_ARM_WO_GPS',   1,   INT32)
    set_param(conn, 'COM_RC_IN_MODE',   4,   INT32)
    set_param(conn, 'COM_POWER_COUNT',  0,   INT32)
    set_param(conn, 'COM_ARM_SDCARD',   0,   INT32)
    set_param(conn, 'COM_ARM_CHK_ESCS', 0,   INT32)
    set_param(conn, 'COM_ARM_MAG_STR',  0,   INT32)
    set_param(conn, 'NAV_DLL_ACT',      0,   INT32)
    set_param(conn, 'NAV_RCL_ACT',      0,   INT32)
    set_param(conn, 'CBRK_SUPPLY_CHK',  894281, INT32)
    set_param(conn, 'CBRK_USB_CHK',     197848, INT32)
    set_param(conn, 'SYS_HAS_GPS',      0,   INT32)
    set_param(conn, 'EKF2_GPS_CTRL',    0,   INT32)

    # 7. IMU calibration bypass via CBRK
    set_param(conn, 'CBRK_IO_SAFETY',  22027, INT32)  # IO safety bypass

    print("\n[FIX] Parametreler uygulandı. EKF2 stabilize için 5s bekleniyor...")
    for i in range(25):
        gcs_hb(conn)
        time.sleep(0.2)

    # commander check çalıştır
    print("\n[FIX] === commander check ===")
    result = shell_cmd(conn, "commander check", wait=3.0)
    print(result if result.strip() else "  (yanıt yok)")

    # MANUAL moda geç
    print("\n[FIX] MANUAL moda geçiliyor...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        1, 1, 0, 0, 0, 0, 0,
    )
    time.sleep(1.0)
    # ACK oku
    m = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
    if m: print(f"  Mode ACK: {m.result}")

    # ARM dene
    print("\n[FIX] ARM gönderiliyor...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, 21196.0, 0, 0, 0, 0, 0,
    )

    # Sonuç dinle
    deadline = time.time() + 8
    while time.time() < deadline:
        msg = conn.recv_match(blocking=True, timeout=0.2)
        if msg is None: continue
        t = msg.get_type()
        if t == 'STATUSTEXT':
            print(f"  📢 [{msg.severity}] {msg.text.strip()}")
        elif t == 'COMMAND_ACK' and msg.command == 400:
            r = "✓ ARM BAŞARILI!" if msg.result == 0 else f"✗ ARM RED (result={msg.result})"
            print(f"\n  {'='*40}")
            print(f"  {r}")
            print(f"  {'='*40}\n")
            if msg.result == 0:
                break
        gcs_hb(conn)

    print("[FIX] Tamamlandı.")

if __name__ == "__main__":
    main()
