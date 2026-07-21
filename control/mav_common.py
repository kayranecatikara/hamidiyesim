"""
mav_common.py — Drone ve Plane arasında paylaşılan düşük seviye MAVLink altyapısı.

ARDUPILOT SÜRÜMÜ (ArduCopter + ArduPlane).
PX4'ten farklar:
- Custom mode tek parametredir (PX4'teki main_mode/sub_mode ayrımı yok)
- Force ARM magic = 2989, force DISARM magic = 21196
- Telemetri akışı REQUEST_DATA_STREAM ile istenmelidir (PX4 kendiliğinden yollar)
- RC override yalnızca SYSID_MYGCS (varsayılan 255) kaynaklı paketlerden kabul
  edilir; bu yüzden script'ler source_system=255 ile bağlanır.

Tüm fonksiyonlar açık bir mavutil.mavlink_connection nesnesi alır.
Bu sayede aynı process içinde birden fazla araca bağlanılabilir.
"""

import time
import threading
from pymavlink import mavutil


# ---------------------------------------------------------------------------
# ArduPilot custom mode sabitleri
# ---------------------------------------------------------------------------

# ArduCopter modları
COPTER_MODE_STABILIZE = 0
COPTER_MODE_ALT_HOLD  = 2
COPTER_MODE_GUIDED    = 4
COPTER_MODE_LOITER    = 5
COPTER_MODE_RTL       = 6
COPTER_MODE_LAND      = 9

# ArduPlane modları
PLANE_MODE_MANUAL     = 0
PLANE_MODE_CIRCLE     = 1
PLANE_MODE_STABILIZE  = 2
PLANE_MODE_FBWA       = 5
PLANE_MODE_FBWB       = 6
PLANE_MODE_CRUISE     = 7
PLANE_MODE_AUTO       = 10
PLANE_MODE_RTL        = 11
PLANE_MODE_LOITER     = 12
PLANE_MODE_TAKEOFF    = 13
PLANE_MODE_GUIDED     = 15

# ArduPilot arm/disarm zorlama sihirli değerleri (param2)
FORCE_ARM_MAGIC    = 2989
FORCE_DISARM_MAGIC = 21196

# Varsayılan GCS kaynak sistemi — SYSID_MYGCS ile eşleşmeli (RC override için)
GCS_SOURCE_SYSTEM = 255

# Bağlantıda istenen telemetri frekansı
DEFAULT_STREAM_RATE_HZ = 10


# ---------------------------------------------------------------------------
# Bağlantı
# ---------------------------------------------------------------------------

def connect_mavlink(port: int,
                    source_system: int = GCS_SOURCE_SYSTEM,
                    source_component: int = 0,
                    protocol: str = "udpin",
                    ip: str = "127.0.0.1",
                    timeout: int = 15,
                    stream_rate_hz: int = DEFAULT_STREAM_RATE_HZ):
    """
    Verilen port üzerinden MAVLink bağlantısı kurar ve OTOPİLOT heartbeat'i
    bekler (diğer GCS'lerin heartbeat'leri yok sayılır). Ardından telemetri
    stream'lerini ister.

    Returns:
        mavutil.mavlink_connection nesnesi
    """
    conn_str = f"{protocol}:{ip}:{port}"
    print(f"[MAV] Bağlanılıyor: {conn_str} (sys={source_system})")
    conn = mavutil.mavlink_connection(
        conn_str,
        source_system=source_system,
        source_component=source_component,
    )

    hb = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        # Yalnızca otopilot bileşeninden gelen heartbeat'i kabul et
        if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            continue
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            continue
        hb = msg
        conn.target_system = msg.get_srcSystem()
        conn.target_component = msg.get_srcComponent()
        break

    if hb is None:
        raise TimeoutError(f"[MAV] Heartbeat gelmedi: {conn_str}")
    print(f"[MAV] Heartbeat alındı — target sys={conn.target_system} "
          f"comp={conn.target_component} type={hb.type}")

    # ArduPilot telemetriyi istek üzerine yollar — stream'leri aç
    if stream_rate_hz > 0:
        request_streams(conn, stream_rate_hz)

    return conn


def request_streams(conn, rate_hz: int = DEFAULT_STREAM_RATE_HZ):
    """Tüm telemetri stream'lerini belirtilen frekansta ister."""
    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1,
    )


def set_message_interval(conn, message_id: int, hz: float):
    """
    Belirli bir mesajın yayın frekansını ayarlar (MAV_CMD_SET_MESSAGE_INTERVAL).
    Örn: LOCAL_POSITION_NED'i 30 Hz istemek için:
        set_message_interval(conn, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30)
    """
    interval_us = int(1e6 / hz) if hz > 0 else -1
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id, interval_us, 0, 0, 0, 0, 0,
    )


# ---------------------------------------------------------------------------
# Heartbeat / GCS Keepalive
# ---------------------------------------------------------------------------

def send_gcs_heartbeat(conn):
    """Bir kez GCS heartbeat paketi gönderir."""
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


class GCSKeepalive:
    """
    Arka planda sürekli GCS heartbeat gönderen thread yöneticisi.
    ArduPilot'ta GCS failsafe ve RC override zaman aşımı için faydalıdır.
    """

    def __init__(self, conn, interval: float = 0.1):
        self.conn = conn
        self.interval = interval
        self._running = False
        self._thread = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[MAV] GCS keepalive başlatıldı")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        print("[MAV] GCS keepalive durduruldu")

    def _loop(self):
        while self._running:
            send_gcs_heartbeat(self.conn)
            time.sleep(self.interval)


# ---------------------------------------------------------------------------
# ACK Bekleme
# ---------------------------------------------------------------------------

def wait_ack(conn, command_id: int = None, timeout: float = 5.0):
    """
    COMMAND_ACK mesajı bekler.

    Args:
        conn: mavutil.mavlink_connection
        command_id: Beklenen komut ID'si (None ise herhangi bir ACK yeterli)
        timeout: Maksimum bekleme süresi

    Returns:
        (command, result) tuple veya None
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if command_id is None or msg.command == command_id:
            print(f"[MAV] ACK cmd={msg.command} result={msg.result}")
            return (msg.command, msg.result)
    print(f"[MAV] ACK timeout (cmd={command_id})")
    return None


# ---------------------------------------------------------------------------
# Arming
# ---------------------------------------------------------------------------

def is_armed(conn) -> bool:
    """Son heartbeat'e göre armed durumunu döndürür."""
    msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
    if msg is None:
        return False
    return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def arm(conn, force: bool = False, retries: int = 1, retry_interval: float = 2.0):
    """
    Aracı arm eder.

    Args:
        force: True ise p2=2989 ile force arm gönderir (ArduPilot sihirli değeri;
               prearm kontrollerini atlar — sim için güvenli)
        retries: ACK reddedilirse kaç kez denenecek (EKF otururken prearm
                 kontrolleri geçici olarak reddedebilir)
    """
    p2 = FORCE_ARM_MAGIC if force else 0
    for attempt in range(1, max(1, retries) + 1):
        print(f"[MAV] ARM gönderiliyor (force={force}, deneme={attempt})")
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, p2, 0, 0, 0, 0, 0,
        )
        result = wait_ack(conn, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if result and result[1] == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return result
        if attempt < retries:
            time.sleep(retry_interval)
    return result


def disarm(conn, force: bool = False):
    """
    Aracı disarm eder.

    Args:
        force: True ise p2=21196 ile force disarm gönderir (uçuşta bile disarm)
    """
    p2 = FORCE_DISARM_MAGIC if force else 0
    print(f"[MAV] DISARM gönderiliyor (force={force})")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, p2, 0, 0, 0, 0, 0,
    )
    return wait_ack(conn, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)


# ---------------------------------------------------------------------------
# Mod Ayarlama
# ---------------------------------------------------------------------------

def set_mode(conn, custom_mode: int, confirm_timeout: float = 3.0):
    """
    ArduPilot custom mode ayarlar.

    Args:
        custom_mode: COPTER_MODE_* veya PLANE_MODE_* sabitlerinden biri
        confirm_timeout: >0 ise heartbeat üzerinden mod teyidi beklenir

    Returns:
        wait_ack() sonucu: (command, result) veya None
    """
    print(f"[MAV] Mode ayarlanıyor: custom_mode={custom_mode}")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,
        0, 0, 0, 0, 0,
    )
    result = wait_ack(conn, mavutil.mavlink.MAV_CMD_DO_SET_MODE)

    if confirm_timeout > 0:
        t0 = time.time()
        while time.time() - t0 < confirm_timeout:
            hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if (hb is not None
                    and hb.get_srcSystem() == conn.target_system
                    and hb.get_srcComponent() == conn.target_component
                    and hb.custom_mode == custom_mode):
                print(f"[MAV] Mod teyit edildi (heartbeat custom_mode={custom_mode})")
                return result
        print(f"[MAV] Mod heartbeat teyidi gelmedi (custom_mode={custom_mode})")
    return result


# ---------------------------------------------------------------------------
# Telemetri Okuma
# ---------------------------------------------------------------------------

def get_local_position(conn, timeout: float = 2.0):
    """
    LOCAL_POSITION_NED mesajını okur.

    Returns:
        dict: {x, y, z, vx, vy, vz} veya None
    """
    msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
    if msg is None:
        return None
    return {
        "x": msg.x, "y": msg.y, "z": msg.z,
        "vx": msg.vx, "vy": msg.vy, "vz": msg.vz,
    }


def get_attitude(conn, timeout: float = 2.0):
    """
    ATTITUDE mesajını okur.

    Returns:
        dict: {roll, pitch, yaw, rollspeed, pitchspeed, yawspeed} veya None
    """
    msg = conn.recv_match(type="ATTITUDE", blocking=True, timeout=timeout)
    if msg is None:
        return None
    return {
        "roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw,
        "rollspeed": msg.rollspeed, "pitchspeed": msg.pitchspeed,
        "yawspeed": msg.yawspeed,
    }


# ---------------------------------------------------------------------------
# Mesaj Drain (bekleyen mesajları temizle)
# ---------------------------------------------------------------------------

def drain_messages(conn, timeout: float = 0.5):
    """Kuyruktaki tüm mesajları okuyup temizler."""
    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            break


# ---------------------------------------------------------------------------
# Yardımcılar
# ---------------------------------------------------------------------------

def timestamp_ms() -> int:
    """Milisaniye cinsinden zaman damgası (32-bit maskelenmiş)."""
    return int(time.time() * 1000) & 0xFFFFFFFF


def log_telemetry(conn, duration: float = 3.0, interval: float = 0.5):
    """Belirli süre boyunca pozisyon telemetrisi yazdırır."""
    t0 = time.time()
    while time.time() - t0 < duration:
        pos = get_local_position(conn, timeout=interval)
        if pos:
            print(f"  pos: x={pos['x']:.2f} y={pos['y']:.2f} z={pos['z']:.2f}")
        time.sleep(interval)
