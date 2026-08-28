"""
guidance/common.py — GPS ve görsel güdüm hatlarının paylaştığı yardımcılar.

Skaler matematik, ivme sınırlama ve MAVLink hız-setpoint göndericisi tek yerde.
Hem GPS fazı (gps_guidance) hem görsel IBVS (adapter_copter / visual_lead)
bu tabanı kullanır.

Setpoint sözleşmesi:
  send_velocity : sadece hız + yaw (GUIDED) — hem GPS fazı hem IBVS hattı
"""

import math
import time

from pymavlink import mavutil


# ══════════════════════════════════════════════════════════
#  SKALER YARDIMCILAR
# ══════════════════════════════════════════════════════════

def timestamp_ms():
    return int(time.time() * 1e6) & 0xFFFFFFFF


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def normalize_angle(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def vec3_len(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def limit_acceleration(vx_cmd, vy_cmd, vz_cmd, vx_p, vy_p, vz_p, max_a, dt):
    """Komut hız vektörünü, önceki komuta göre max_a*dt'lik değişimle sınırlar.
    TEK 3B tavan — GPS fazı bunu kullanır (orada kamera kısıtı yok)."""
    if dt <= 0:
        return vx_cmd, vy_cmd, vz_cmd
    dvx, dvy, dvz = vx_cmd - vx_p, vy_cmd - vy_p, vz_cmd - vz_p
    dv = vec3_len(dvx, dvy, dvz)
    max_dv = max_a * dt
    if dv > max_dv and dv > 0:
        s = max_dv / dv
        return vx_p + dvx * s, vy_p + dvy * s, vz_p + dvz * s
    return vx_cmd, vy_cmd, vz_cmd


def limit_acceleration_split(vx_cmd, vy_cmd, vz_cmd, vx_p, vy_p, vz_p,
                             max_a_h, max_a_v, dt):
    """Yatay ve dikey ivmeyi AYRI tavanlarla sınırlar (görsel faz).

    Neden ayrı — iki eksenin FİZİĞİ farklı:
      YATAY: quad ileri ivmelenmek için burnunu aşağı eğer (eğim = atan(a/g)).
             Kamera gövdeye +25° sabit bağlı, dünyaya bakışı 25°−eğim olur;
             ~5 m/s² üstünde gökyüzü arka planı kaybolur, yer karmaşası tespit
             modeline girer. Tavan bu yüzden düşük tutulur.
      DİKEY: yukarı ivmelenmek burun eğimi GEREKTİRMEZ — yalnız itki artar,
             kamera açısı hiç değişmez. Sınır itki bütçesidir, kamera değil.

    Tek 3B tavan kullanılırken kameranın YATAY kısıtı dikeye de dayatılıyordu:
    dikey PN karelerin %27-78'inde 15°'lik tavanında "yukarı çık" derken
    v_doygun %93-99 çıkıyordu — komut rampada yok oluyordu ve drone hedefin
    ~2 m altından geçiyordu (2026-07-31 uçuşları).
    """
    if dt <= 0:
        return vx_cmd, vy_cmd, vz_cmd
    dvx, dvy, dvz = vx_cmd - vx_p, vy_cmd - vy_p, vz_cmd - vz_p
    dh = math.hypot(dvx, dvy)
    max_dh = max_a_h * dt
    if dh > max_dh and dh > 0:
        s = max_dh / dh
        dvx, dvy = dvx * s, dvy * s
    max_dv = max_a_v * dt
    if abs(dvz) > max_dv:
        dvz = math.copysign(max_dv, dvz)
    return vx_p + dvx, vy_p + dvy, vz_p + dvz


# ══════════════════════════════════════════════════════════
#  MAVLINK SETPOINT GÖNDERİCİ (SET_POSITION_TARGET_LOCAL_NED)
# ══════════════════════════════════════════════════════════

# Sadece hız + yaw aktif; pozisyon/ivme/force/yaw_rate ignore.
_TYPEMASK_VEL_YAW = (
    (1 << 0) | (1 << 1) | (1 << 2) |   # pozisyon ignore
    (1 << 6) | (1 << 7) | (1 << 8) |   # ivme ignore
    (1 << 9) |                          # force ignore
    (1 << 11)                           # yaw_rate ignore
)


def send_velocity(conn, vx, vy, vz, yaw):
    """Saf hız + yaw hedefi (GUIDED) — GPS yaklaşma ve IBVS hattı."""
    conn.mav.set_position_target_local_ned_send(
        timestamp_ms(),
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _TYPEMASK_VEL_YAW,
        0.0, 0.0, 0.0, vx, vy, vz, 0.0, 0.0, 0.0, yaw, 0.0
    )
