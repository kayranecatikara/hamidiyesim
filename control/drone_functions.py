"""
drone_functions.py — Iris multicopter drone kontrol fonksiyonları.

ARDUPILOT (ArduCopter) SÜRÜMÜ.
PX4'ten farklar:
- OFFBOARD yerine GUIDED modu kullanılır
- Kalkış NAV_TAKEOFF komutu ile yapılır (PX4'teki setpoint-prestream gereksiz)
- GUIDED modda SET_POSITION_TARGET_LOCAL_NED aynı şekilde desteklenir
- yaw parametresi verilirse typemask'te gerçekten kullanılır (PX4 kodunda
  yaw bit'i yanlışlıkla ignore ediliyordu)
"""

import time
import math
from pymavlink import mavutil

from control.mav_common import (
    connect_mavlink,
    arm,
    disarm,
    set_mode,
    wait_ack,
    get_local_position,
    get_attitude,
    drain_messages,
    timestamp_ms,
    GCS_SOURCE_SYSTEM,
    COPTER_MODE_GUIDED,
    COPTER_MODE_LAND,
)

# ---------------------------------------------------------------------------
# Sabitler
# ---------------------------------------------------------------------------

DRONE_PORT = 14541
DRONE_SYS_ID = GCS_SOURCE_SYSTEM   # 255 — SYSID_MYGCS ile eşleşmeli
SETPOINT_RATE = 0.1          # saniye — setpoint gönderim aralığı
DEFAULT_TAKEOFF_Z = -2.0     # NED: yukarı = negatif z

# Position setpoint type_mask değerleri
# bit0-2: pozisyon, bit3-5: hız, bit6-8: ivme, bit9: force,
# bit10: yaw, bit11: yaw_rate  (set edilen bit = IGNORE)
TYPEMASK_POSITION = 0b0000_1111_1111_1000   # sadece pozisyon (yaw serbest)
TYPEMASK_POS_YAW  = 0b0000_1011_1111_1000   # pozisyon + yaw


# ---------------------------------------------------------------------------
# Bağlantı
# ---------------------------------------------------------------------------

_conn = None  # modül düzeyinde bağlantı referansı


def connect_drone(port: int = DRONE_PORT):
    """
    Drone'a bağlanıp modül referansını döndürür.
    """
    global _conn
    _conn = connect_mavlink(port, source_system=DRONE_SYS_ID)
    print(f"[DRONE] Bağlantı kuruldu (port={port})")
    return _conn


def get_conn():
    """Aktif bağlantıyı döndürür; yoksa hata verir."""
    if _conn is None:
        raise RuntimeError("[DRONE] Önce connect_drone() çağrılmalı")
    return _conn


# ---------------------------------------------------------------------------
# Setpoint Gönderimi
# ---------------------------------------------------------------------------

def _send_position_setpoint(conn, x: float, y: float, z: float,
                            yaw: float = None):
    """
    Tek bir LOCAL_NED position setpoint gönderir.

    yaw=None ise heading serbest bırakılır (mevcut yaw korunur),
    değer verilirse drone o yaw açısına döner (radyan, NED).
    """
    if yaw is None:
        type_mask = TYPEMASK_POSITION
        yaw_val = 0.0
    else:
        type_mask = TYPEMASK_POS_YAW
        yaw_val = yaw
    conn.mav.set_position_target_local_ned_send(
        timestamp_ms(),
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        0.0, 0.0, 0.0,   # vx, vy, vz
        0.0, 0.0, 0.0,   # afx, afy, afz
        yaw_val, 0.0,     # yaw, yaw_rate
    )


# ---------------------------------------------------------------------------
# Arm / Disarm / Mode
# ---------------------------------------------------------------------------

def arm_drone(force_sim: bool = True):
    """
    Drone'u arm eder. force_sim=True ArduPilot force-arm magic'i (2989) gönderir.
    EKF henüz oturmadıysa prearm reddedilebilir; bu yüzden birkaç kez denenir.
    """
    conn = get_conn()
    return arm(conn, force=force_sim, retries=10, retry_interval=2.0)


def disarm_drone(force: bool = True):
    """Drone'u disarm eder."""
    conn = get_conn()
    return disarm(conn, force=force)


def set_guided_mode():
    """Drone'u GUIDED moduna geçirir (PX4 OFFBOARD karşılığı)."""
    conn = get_conn()
    return set_mode(conn, COPTER_MODE_GUIDED)


# Geriye dönük uyumluluk — eski isimle çağıran kod kırılmasın
set_offboard_mode = set_guided_mode


# ---------------------------------------------------------------------------
# Takeoff
# ---------------------------------------------------------------------------

def _wait_gps_ready(conn, timeout: float = 60.0):
    """Arm öncesi 3D GPS fix bekler (EKF'nin oturması için)."""
    print("[DRONE] GPS 3D fix bekleniyor...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
        if msg and msg.fix_type >= 3:
            print(f"[DRONE] GPS hazır (fix={msg.fix_type})")
            return True
    print("[DRONE] GPS fix beklenirken timeout — yine de devam ediliyor")
    return False


def takeoff_to_z(target_z: float = DEFAULT_TAKEOFF_Z,
                 timeout: float = 30.0):
    """
    GUIDED modda drone'u hedef irtifaya kaldırır.

    Sıra (ArduPilot):
    1. GPS/EKF hazır olana kadar bekle
    2. GUIDED mode
    3. ARM (force, retry'li)
    4. MAV_CMD_NAV_TAKEOFF ile |target_z| metreye kalk
    5. Hedef z'ye ulaşana kadar telemetri izle

    Returns:
        True ise kalkış başarılı
    """
    conn = get_conn()
    takeoff_alt = abs(target_z)

    # 0) Zaten havada mı? (önceki bir görevden kalmış olabilir)
    #    ArduCopter uçan aracı NAV_TAKEOFF ile kaldırmaz (reddeder), bu yüzden
    #    havadaysak takeoff'u atlayıp doğrudan hedef irtifaya setpoint yollarız.
    drain_messages(conn)
    cur = get_local_position(conn, timeout=1.0)
    already_airborne = cur is not None and cur["z"] < -1.5

    if already_airborne:
        print(f"[DRONE] Zaten havada (z={cur['z']:.1f}) — takeoff atlanıyor, "
              f"hedef irtifaya çıkılıyor")
        set_mode(conn, COPTER_MODE_GUIDED)
        cx, cy = cur["x"], cur["y"]
        t0 = time.time()
        while time.time() - t0 < timeout:
            _send_position_setpoint(conn, cx, cy, target_z)
            pos = get_local_position(conn, timeout=0.2)
            if pos and abs(pos["z"] - target_z) < 0.3:
                print(f"[DRONE] Hedefe ulaşıldı: z={pos['z']:.2f}")
                return True
            time.sleep(SETPOINT_RATE)
        print("[DRONE] İrtifa hedefine ulaşılamadı (havada)")
        return False

    # 1) GPS/EKF hazırlığı
    _wait_gps_ready(conn)

    # 2) GUIDED
    result = set_mode(conn, COPTER_MODE_GUIDED)
    if not (result and result[1] == 0):
        print("[DRONE] GUIDED moda geçilemedi!")
        return False

    # 3) ARM
    result = arm_drone(force_sim=True)
    if not (result and result[1] == 0):
        print("[DRONE] Arm başarısız!")
        return False
    time.sleep(0.5)

    # 4) NAV_TAKEOFF
    print(f"[DRONE] NAV_TAKEOFF gönderiliyor (alt={takeoff_alt:.1f}m)")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, takeoff_alt,
    )
    wait_ack(conn, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    # 5) Hedefe ulaşana kadar izle
    print(f"[DRONE] Kalkış hedefi z={target_z}")
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = get_local_position(conn, timeout=0.5)
        if pos and abs(pos["z"] - target_z) < 0.3:
            print(f"[DRONE] Hedefe ulaşıldı: z={pos['z']:.2f}")
            return True

    print("[DRONE] Kalkış timeout — hedefe ulaşılamadı")
    return False


# ---------------------------------------------------------------------------
# Hold Position
# ---------------------------------------------------------------------------

def hold_position(x: float, y: float, z: float,
                  duration: float = 5.0, yaw: float = None):
    """
    Belirtilen konumda sürekli setpoint göndererek hover yapar.

    Args:
        duration: Kaç saniye tutulacak
        yaw: None ise heading değişmez; radyan verilirse o yöne döner
    """
    conn = get_conn()
    print(f"[DRONE] Hold: x={x:.2f} y={y:.2f} z={z:.2f} ({duration}s)")
    t0 = time.time()
    while time.time() - t0 < duration:
        _send_position_setpoint(conn, x, y, z, yaw=yaw)
        time.sleep(SETPOINT_RATE)


def hover(duration: float = 5.0, target_z: float = DEFAULT_TAKEOFF_Z):
    """Mevcut pozisyonda hover yapar (basitleştirilmiş)."""
    conn = get_conn()
    # Mevcut pozisyonu oku
    pos = get_local_position(conn, timeout=2.0)
    if pos:
        hold_position(pos["x"], pos["y"], target_z, duration=duration)
    else:
        hold_position(0.0, 0.0, target_z, duration=duration)


# ---------------------------------------------------------------------------
# Hareket Fonksiyonları (NED frame)
# ---------------------------------------------------------------------------
# NED: x=kuzey(ileri), y=doğu(sağ), z=aşağı
# Yaw=0 → kuzey'e bakar

def _get_current_pos(conn):
    """Mevcut pozisyonu okur, None ise (0,0,DEFAULT_TAKEOFF_Z) döndürür."""
    drain_messages(conn)   # bayat kuyruk verisi yerine güncel değeri oku
    pos = get_local_position(conn, timeout=1.0)
    if pos:
        return pos["x"], pos["y"], pos["z"]
    return 0.0, 0.0, DEFAULT_TAKEOFF_Z


def _move_to(x: float, y: float, z: float,
             speed: float = 1.0, yaw: float = None):
    """
    Hedefe doğru sürekli setpoint göndererek hareket eder.
    Hedefe 0.3m yaklaşınca başarılı sayar.
    """
    conn = get_conn()
    print(f"[DRONE] MoveTo: x={x:.2f} y={y:.2f} z={z:.2f}")
    t0 = time.time()
    timeout = 30.0
    while time.time() - t0 < timeout:
        _send_position_setpoint(conn, x, y, z, yaw=yaw)
        pos = get_local_position(conn, timeout=0.2)
        if pos:
            dist = math.sqrt((pos["x"] - x)**2 + (pos["y"] - y)**2 + (pos["z"] - z)**2)
            if dist < 0.3:
                print(f"[DRONE] Hedefe ulaşıldı ({dist:.2f}m)")
                return True
        time.sleep(SETPOINT_RATE)
    print("[DRONE] MoveTo timeout")
    return False


def move_forward(distance: float = 2.0, speed: float = 1.0):
    """NED frame'de ileri hareket (x+ yönü)."""
    conn = get_conn()
    cx, cy, cz = _get_current_pos(conn)
    return _move_to(cx + distance, cy, cz, speed)


def move_backward(distance: float = 2.0, speed: float = 1.0):
    """NED frame'de geri hareket (x- yönü)."""
    conn = get_conn()
    cx, cy, cz = _get_current_pos(conn)
    return _move_to(cx - distance, cy, cz, speed)


def move_right(distance: float = 2.0, speed: float = 1.0):
    """NED frame'de sağa hareket (y+ yönü)."""
    conn = get_conn()
    cx, cy, cz = _get_current_pos(conn)
    return _move_to(cx, cy + distance, cz, speed)


def move_left(distance: float = 2.0, speed: float = 1.0):
    """NED frame'de sola hareket (y- yönü)."""
    conn = get_conn()
    cx, cy, cz = _get_current_pos(conn)
    return _move_to(cx, cy - distance, cz, speed)


def move_up(distance: float = 1.0, speed: float = 1.0):
    """Yukarı hareket (z daha negatif)."""
    conn = get_conn()
    cx, cy, cz = _get_current_pos(conn)
    return _move_to(cx, cy, cz - distance, speed)


def move_down(distance: float = 1.0, speed: float = 1.0):
    """Aşağı hareket (z daha pozitif)."""
    conn = get_conn()
    cx, cy, cz = _get_current_pos(conn)
    return _move_to(cx, cy, cz + distance, speed)


# ---------------------------------------------------------------------------
# Yaw Kontrol
# ---------------------------------------------------------------------------

def _get_current_yaw(conn):
    """Mevcut yaw açısını okur (radyan)."""
    drain_messages(conn)   # bayat kuyruk verisi yerine güncel değeri oku
    att = get_attitude(conn, timeout=1.0)
    if att:
        return att["yaw"]
    return 0.0


def yaw_right(degrees: float = 90.0, hold_time: float = 2.0):
    """Saat yönünde yaw yapar."""
    conn = get_conn()
    current_yaw = _get_current_yaw(conn)
    target_yaw = current_yaw + math.radians(degrees)
    cx, cy, cz = _get_current_pos(conn)
    print(f"[DRONE] Yaw right {degrees}° (current={math.degrees(current_yaw):.1f}°)")
    hold_position(cx, cy, cz, duration=hold_time, yaw=target_yaw)


def yaw_left(degrees: float = 90.0, hold_time: float = 2.0):
    """Saat yönünün tersine yaw yapar."""
    conn = get_conn()
    current_yaw = _get_current_yaw(conn)
    target_yaw = current_yaw - math.radians(degrees)
    cx, cy, cz = _get_current_pos(conn)
    print(f"[DRONE] Yaw left {degrees}° (current={math.degrees(current_yaw):.1f}°)")
    hold_position(cx, cy, cz, duration=hold_time, yaw=target_yaw)


# ---------------------------------------------------------------------------
# İniş
# ---------------------------------------------------------------------------

def land_drone():
    """Drone'u LAND moduna geçirir."""
    conn = get_conn()
    print("[DRONE] İniş başlatılıyor (LAND)")
    return set_mode(conn, COPTER_MODE_LAND)


# ---------------------------------------------------------------------------
# Telemetri
# ---------------------------------------------------------------------------

def get_drone_position():
    """Drone pozisyonunu okur (güncel değer — kuyruk önce boşaltılır)."""
    conn = get_conn()
    drain_messages(conn)
    return get_local_position(conn)


def get_drone_attitude():
    """Drone attitude'unu okur (güncel değer — kuyruk önce boşaltılır)."""
    conn = get_conn()
    drain_messages(conn)
    return get_attitude(conn)


def print_status():
    """Drone durumunu yazdırır."""
    conn = get_conn()
    pos = get_local_position(conn, timeout=1.0)
    att = get_attitude(conn, timeout=1.0)
    if pos:
        print(f"  POS: x={pos['x']:.2f} y={pos['y']:.2f} z={pos['z']:.2f}")
    if att:
        print(f"  ATT: roll={math.degrees(att['roll']):.1f}° "
              f"pitch={math.degrees(att['pitch']):.1f}° "
              f"yaw={math.degrees(att['yaw']):.1f}°")
