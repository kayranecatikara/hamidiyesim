"""
plane_functions.py — Fixed-wing plane kontrol fonksiyonları.

ARDUPILOT (ArduPlane) SÜRÜMÜ.
PX4'ten farklar:
- MANUAL(0) ve STABILIZE(2) modları kullanılır (PX4: MANUAL=1, STABILIZED=7)
- RC override yalnızca SYSID_MYGCS (=255) kaynaklı paketlerden kabul edilir;
  bu yüzden bağlantı source_system=255 ile kurulur
- Force arm magic 2989'dur (mav_common.arm halleder)
- RC override 3 sn içinde yenilenmezse düşer (RC_OVERRIDE_TIME) — kontrol
  döngüleri zaten 10 Hz gönderdiği için sorun olmaz

Fixed-wing mantığı multicopter'dan farklıdır:
- Sürekli hareket halinde olmalı (stall riski)
- RC override ile throttle/pitch/roll/yaw kontrol edilir
"""

import time
import math
import threading
from pymavlink import mavutil

from control.mav_common import (
    connect_mavlink,
    arm,
    disarm,
    set_mode,
    wait_ack,
    get_local_position,
    get_attitude,
    send_gcs_heartbeat,
    GCSKeepalive,
    drain_messages,
    timestamp_ms,
    GCS_SOURCE_SYSTEM,
    PLANE_MODE_MANUAL,
    PLANE_MODE_STABILIZE,
    PLANE_MODE_FBWA,
    PLANE_MODE_LOITER,
    PLANE_MODE_TAKEOFF,
)


# ---------------------------------------------------------------------------
# Sabitler
# ---------------------------------------------------------------------------

PLANE_PORT = 14542
PLANE_SYS_ID = GCS_SOURCE_SYSTEM   # 255 — RC override için SYSID_MYGCS ile eşleşmeli
CONTROL_RATE = 0.1      # saniye — kontrol loop hızı

# manual_control sınırları: -1000..+1000 (pitch/roll/yaw), 0..1000 (throttle)
THROTTLE_IDLE = 0
THROTTLE_CRUISE = 600
THROTTLE_FULL = 900


# ---------------------------------------------------------------------------
# Bağlantı
# ---------------------------------------------------------------------------

_conn = None
_keepalive = None


def connect_plane(port: int = PLANE_PORT):
    """
    Plane'e bağlanıp modül referansını döndürür.
    """
    global _conn
    _conn = connect_mavlink(port, source_system=PLANE_SYS_ID)
    print(f"[PLANE] Bağlantı kuruldu (port={port})")
    return _conn


def get_conn():
    """Aktif bağlantıyı döndürür."""
    if _conn is None:
        raise RuntimeError("[PLANE] Önce connect_plane() çağrılmalı")
    return _conn


# ---------------------------------------------------------------------------
# GCS Keepalive
# ---------------------------------------------------------------------------

def start_gcs_keepalive():
    """
    GCS keepalive thread'ini başlatır.
    ArduPilot'ta GCS failsafe'i sakin tutar ve RC override'ın kaynağı olan
    GCS'in canlı görünmesini sağlar.
    """
    global _keepalive
    conn = get_conn()
    if _keepalive is not None:
        _keepalive.stop()
    _keepalive = GCSKeepalive(conn, interval=0.1)
    _keepalive.start()
    return _keepalive


def stop_gcs_keepalive():
    """GCS keepalive thread'ini durdurur."""
    global _keepalive
    if _keepalive:
        _keepalive.stop()
        _keepalive = None


# ---------------------------------------------------------------------------
# Arm / Disarm
# ---------------------------------------------------------------------------

def _wait_gps_ready(conn, timeout: float = 60.0):
    """Arm öncesi 3D GPS fix bekler (force arm EKF oturmadan arm etmesin)."""
    print("[PLANE] GPS 3D fix bekleniyor...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
        if msg and msg.fix_type >= 3:
            print(f"[PLANE] GPS hazır (fix={msg.fix_type})")
            return True
    print("[PLANE] GPS fix beklenirken timeout — yine de devam ediliyor")
    return False


def arm_plane(warmup_duration: float = 3.0):
    """
    Plane'i arm eder.

    Sıra:
    1. GCS keepalive başlat (zaten başlamamışsa)
    2. Kısa warmup (EKF/telemetri otursun)
    3. GPS fix bekle (force arm EKF hazır olmadan arm etmesin)
    4. Force ARM gönder (retry'li)
    5. MANUAL moda geç (RC override doğrudan servolara işlesin)

    Returns:
        ACK sonucu
    """
    conn = get_conn()

    # Keepalive yoksa başlat
    if _keepalive is None or not _keepalive._running:
        start_gcs_keepalive()

    print(f"[PLANE] GCS warmup ({warmup_duration}s)...")
    time.sleep(warmup_duration)

    _wait_gps_ready(conn)

    # Force ARM (ArduPilot magic 2989, mav_common halleder)
    result = arm(conn, force=True, retries=10, retry_interval=2.0)
    if result and result[1] == 0:
        print("[PLANE] ARM başarılı")
        # RC override'ın motoru doğrudan sürebilmesi için MANUAL moda al.
        set_mode(conn, PLANE_MODE_MANUAL)
        print("[PLANE] Mod MANUAL (0) olarak ayarlandı — RC override aktif")
    else:
        print(f"[PLANE] ARM sonucu: {result}")
    return result


def disarm_plane():
    """Plane'i disarm eder."""
    conn = get_conn()
    return disarm(conn, force=True)


# ---------------------------------------------------------------------------
# Manuel Kontrol
# ---------------------------------------------------------------------------

def send_manual_control(pitch: int = 0, roll: int = 0,
                        throttle: int = THROTTLE_IDLE, yaw: int = 0):
    """
    RC Channels Override ile kontrol komutu gönderir.
    ArduPlane varsayılan kanal düzeni: CH1=Aileron CH2=Elevator CH3=Throttle CH4=Rudder.

    Args:
        pitch: -1000..+1000 (pozitif = burun yukarı)
        roll: -1000..+1000 (pozitif = sağa yatış)
        throttle: 0..1000
        yaw: -1000..+1000 (pozitif = sağa dön)
    """
    conn = get_conn()

    # Alım tamponunu boşalt: komut döngüleri hiç okuma yapmazsa soket tamponu
    # dolar, telemetri bayatlayıp kayıplı hale gelir (SITL'de ölçüldü).
    drain_messages(conn)

    # Standart RC PWM (1000-2000 aralığı) formülü
    rc_roll     = int(1500 + (roll / 2))
    rc_pitch    = int(1500 - (pitch / 2))  # RC2 düşük PWM = elevator yukarı = burun yukarı
    rc_throttle = int(1000 + throttle)     # 0..1000 aralığı 1000..2000'e çevrildi
    rc_yaw      = int(1500 + (yaw / 2))

    conn.mav.rc_channels_override_send(
        conn.target_system,
        conn.target_component,
        rc_roll,      # CH1: Aileron (roll)
        rc_pitch,     # CH2: Elevator (pitch)
        rc_throttle,  # CH3: Throttle
        rc_yaw,       # CH4: Rudder (yaw)
        0, 0, 0, 0    # kullanılmayan kanallar (0 = override yok)
    )


# ---------------------------------------------------------------------------
# Throttle / Heading / Pitch / Roll Helpers
# ---------------------------------------------------------------------------

def set_throttle(throttle: int, duration: float = 1.0):
    """
    Belirli süre boyunca sabit throttle gönderir.

    Args:
        throttle: 0..1000
        duration: saniye
    """
    print(f"[PLANE] Throttle={throttle} ({duration}s)")
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(throttle=throttle)
        time.sleep(CONTROL_RATE)


def set_heading(yaw: int, throttle: int = THROTTLE_CRUISE,
                duration: float = 2.0):
    """
    Belirli süre boyunca yaw + throttle gönderir.

    Args:
        yaw: -1000..+1000
        throttle: 0..1000
        duration: saniye
    """
    print(f"[PLANE] Heading yaw={yaw} throttle={throttle} ({duration}s)")
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(yaw=yaw, throttle=throttle)
        time.sleep(CONTROL_RATE)


def set_pitch(pitch: int, throttle: int = THROTTLE_CRUISE,
              duration: float = 2.0):
    """
    Belirli süre boyunca pitch + throttle gönderir.

    Args:
        pitch: -1000..+1000 (negatif = burun aşağı)
    """
    print(f"[PLANE] Pitch={pitch} throttle={throttle} ({duration}s)")
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(pitch=pitch, throttle=throttle)
        time.sleep(CONTROL_RATE)


def set_roll(roll: int, throttle: int = THROTTLE_CRUISE,
             duration: float = 2.0):
    """
    Belirli süre boyunca roll + throttle gönderir.

    Args:
        roll: -1000..+1000 (negatif = sol kanat aşağı)
    """
    print(f"[PLANE] Roll={roll} throttle={throttle} ({duration}s)")
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(roll=roll, throttle=throttle)
        time.sleep(CONTROL_RATE)


# ---------------------------------------------------------------------------
# Yüksek Seviye Hareket
# ---------------------------------------------------------------------------

def fly_forward(throttle: int = THROTTLE_CRUISE, duration: float = 3.0):
    """Düz ileri uçuş."""
    print(f"[PLANE] İleri uçuş ({duration}s)")
    set_throttle(throttle, duration)


def turn_left(intensity: int = 300, throttle: int = THROTTLE_CRUISE,
              duration: float = 2.0):
    """
    Sola dönüş.

    Args:
        intensity: Dönüş şiddeti (0-1000)
    """
    print(f"[PLANE] Sola dönüş intensity={intensity} ({duration}s)")
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(yaw=-intensity, roll=-intensity, throttle=throttle)
        time.sleep(CONTROL_RATE)


def turn_right(intensity: int = 300, throttle: int = THROTTLE_CRUISE,
               duration: float = 2.0):
    """
    Sağa dönüş.

    Args:
        intensity: Dönüş şiddeti (0-1000)
    """
    print(f"[PLANE] Sağa dönüş intensity={intensity} ({duration}s)")
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(yaw=intensity, roll=intensity, throttle=throttle)
        time.sleep(CONTROL_RATE)


def climb(pitch_intensity: int = 300, throttle: int = THROTTLE_FULL,
          duration: float = 2.0):
    """Tırmanış — burun yukarı + yüksek gaz."""
    print(f"[PLANE] Tırmanış pitch={pitch_intensity} ({duration}s)")
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(pitch=pitch_intensity, throttle=throttle)
        time.sleep(CONTROL_RATE)


def descend(pitch_intensity: int = -300, throttle: int = THROTTLE_IDLE,
            duration: float = 2.0):
    """Alçalma — burun aşağı + düşük gaz."""
    print(f"[PLANE] Alçalma pitch={pitch_intensity} ({duration}s)")
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(pitch=pitch_intensity, throttle=throttle)
        time.sleep(CONTROL_RATE)


def loiter(duration: float = 5.0):
    """
    Basit loiter — cruise throttle ile düz uçuş devam eder.
    (Gerçek LOITER modu için mav_common.set_mode(conn, PLANE_MODE_LOITER)
    kullanılabilir; o modda RC override akışı kesilmelidir.)
    """
    print(f"[PLANE] Loiter ({duration}s)")
    set_throttle(THROTTLE_CRUISE, duration)


# ---------------------------------------------------------------------------
# Telemetri
# ---------------------------------------------------------------------------

def get_plane_position():
    """Plane pozisyonunu okur (güncel değer — kuyruk önce boşaltılır)."""
    conn = get_conn()
    drain_messages(conn)
    return get_local_position(conn)


def get_plane_attitude():
    """Plane attitude'unu okur (güncel değer — kuyruk önce boşaltılır)."""
    conn = get_conn()
    drain_messages(conn)
    return get_attitude(conn)


def print_status():
    """Plane durumunu yazdırır."""
    conn = get_conn()
    drain_messages(conn)
    pos = get_local_position(conn, timeout=1.0)
    att = get_attitude(conn, timeout=1.0)
    if pos:
        print(f"  POS: x={pos['x']:.2f} y={pos['y']:.2f} z={pos['z']:.2f}")
    if att:
        print(f"  ATT: roll={math.degrees(att['roll']):.1f}° "
              f"pitch={math.degrees(att['pitch']):.1f}° "
              f"yaw={math.degrees(att['yaw']):.1f}°")
