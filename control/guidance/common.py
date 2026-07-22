"""
guidance/common.py — GPS ve görsel güdüm hatlarının paylaştığı yardımcılar.

Faz 0 refactor: chase_algorithm.py ve strike_algorithm.py'de BİREBİR kopyalı olan
matematik yardımcıları, EMA filtreleri, PID kontrolcü ve MAVLink setpoint
göndericileri tek yerde toplandı. Mantık DEĞİŞMEDİ — sadece tek kaynağa taşındı.

Setpoint sözleşmeleri:
  send_position_setpoint : pozisyon + feedforward hız + yaw  (chase/GPS takip)
  send_velocity          : sadece hız + yaw                  (strike/terminal, IBVS)
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
    """Komut hız vektörünü, önceki komuta göre max_a*dt'lik değişimle sınırlar."""
    if dt <= 0:
        return vx_cmd, vy_cmd, vz_cmd
    dvx, dvy, dvz = vx_cmd - vx_p, vy_cmd - vy_p, vz_cmd - vz_p
    dv = vec3_len(dvx, dvy, dvz)
    max_dv = max_a * dt
    if dv > max_dv and dv > 0:
        s = max_dv / dv
        return vx_p + dvx * s, vy_p + dvy * s, vz_p + dvz * s
    return vx_cmd, vy_cmd, vz_cmd


# ══════════════════════════════════════════════════════════
#  EMA FİLTRELER
# ══════════════════════════════════════════════════════════

class EMAScalar:
    """Üstel hareketli ortalama (tek skaler)."""
    __slots__ = ('alpha', '_v')

    def __init__(self, alpha):
        self.alpha = alpha
        self._v = None

    def update(self, raw):
        if self._v is None:
            self._v = raw
        else:
            self._v = self.alpha * raw + (1.0 - self.alpha) * self._v
        return self._v

    @property
    def value(self):
        return self._v


class EMAVec3:
    """Üç eksenli EMA (x, y, z)."""
    __slots__ = ('fx', 'fy', 'fz')

    def __init__(self, alpha):
        self.fx = EMAScalar(alpha)
        self.fy = EMAScalar(alpha)
        self.fz = EMAScalar(alpha)

    def update(self, x, y, z):
        return (self.fx.update(x), self.fy.update(y), self.fz.update(z))


# ══════════════════════════════════════════════════════════
#  PID KONTROLCÜ (3 eksen, konum hatası → hız düzeltmesi)
# ══════════════════════════════════════════════════════════

class PIDController:
    def __init__(self, kp, ki, kd, i_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_max = i_max
        self._integral = [0.0, 0.0, 0.0]
        self._prev_err = [None, None, None]

    def compute(self, errors, dt):
        out = [0.0, 0.0, 0.0]
        for i in range(3):
            e = errors[i]
            self._integral[i] += e * dt
            self._integral[i] = max(-self.i_max, min(self.i_max, self._integral[i]))
            d_term = 0.0
            if self._prev_err[i] is not None and dt > 0:
                d_term = self.kd * (e - self._prev_err[i]) / dt
            self._prev_err[i] = e
            out[i] = self.kp * e + self.ki * self._integral[i] + d_term
        return tuple(out)

    def reset(self):
        self._integral = [0.0, 0.0, 0.0]
        self._prev_err = [None, None, None]


# ══════════════════════════════════════════════════════════
#  MAVLINK SETPOINT GÖNDERİCİLER (SET_POSITION_TARGET_LOCAL_NED)
# ══════════════════════════════════════════════════════════

# Pozisyon + hız + yaw aktif; ivme/force/yaw_rate ignore.
_TYPEMASK_POS_VEL_YAW = (
    (1 << 6) | (1 << 7) | (1 << 8) |   # afxyz ignore
    (1 << 9) |                          # force ignore
    (1 << 11)                           # yaw_rate ignore
)

# Sadece hız + yaw aktif; pozisyon/ivme/force/yaw_rate ignore.
_TYPEMASK_VEL_YAW = (
    (1 << 0) | (1 << 1) | (1 << 2) |   # pozisyon ignore
    (1 << 6) | (1 << 7) | (1 << 8) |   # ivme ignore
    (1 << 9) |                          # force ignore
    (1 << 11)                           # yaw_rate ignore
)


def send_position_setpoint(conn, x, y, z, vx, vy, vz, yaw):
    """GUIDED pozisyon + feedforward hız + yaw hedefi (chase hattı)."""
    conn.mav.set_position_target_local_ned_send(
        timestamp_ms(),
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _TYPEMASK_POS_VEL_YAW,
        x, y, z, vx, vy, vz, 0.0, 0.0, 0.0, yaw, 0.0
    )


def send_velocity(conn, vx, vy, vz, yaw):
    """Saf hız + yaw hedefi (strike/terminal ve IBVS hattı)."""
    conn.mav.set_position_target_local_ned_send(
        timestamp_ms(),
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _TYPEMASK_VEL_YAW,
        0.0, 0.0, 0.0, vx, vy, vz, 0.0, 0.0, 0.0, yaw, 0.0
    )
