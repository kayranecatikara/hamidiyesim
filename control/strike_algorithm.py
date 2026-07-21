"""
=============================================================
  AVCI İHA VURMA ALGORİTMASI — GELİŞTİRİLMİŞ VERSİYON
=============================================================
Geliştirmeler:
  1. Proportional Navigation (PN) güdümü
  2. İki fazlı: APPROACH (PN) → TERMINAL (pure pursuit)
  3. EMA filtre + filtrelenmiş hız tahmini
  4. Closing velocity (Vc) hesabı
  5. İvme sınırlama (approach fazında)
  6. İrtifa güvenliği
=============================================================
"""

import math
import time
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil

# ══════════════════════════════════════════════════════════
#  PARAMETRELER
# ══════════════════════════════════════════════════════════

MAX_SPEED           = 15.0
MAX_ACCEL           = 8.0
MIN_SPEED           = 3.0

PN_GAIN             = 4.0

TERMINAL_RADIUS     = 3.0
APPROACH_P_GAIN     = 3.0
TERMINAL_P_GAIN     = 6.0

EMA_ALPHA_POS       = 0.5
EMA_ALPHA_VEL       = 0.3

ALT_FLOOR           = -0.5

LOOP_HZ             = 50


# ══════════════════════════════════════════════════════════
#  YARDIMCI
# ══════════════════════════════════════════════════════════

class EMAScalar:
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

class EMAVec3:
    __slots__ = ('fx', 'fy', 'fz')
    def __init__(self, alpha):
        self.fx = EMAScalar(alpha)
        self.fy = EMAScalar(alpha)
        self.fz = EMAScalar(alpha)
    def update(self, x, y, z):
        return (self.fx.update(x), self.fy.update(y), self.fz.update(z))

def _timestamp_ms() -> int:
    return int(time.time() * 1e6) & 0xFFFFFFFF

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _vec3_len(x, y, z):
    return math.sqrt(x*x + y*y + z*z)

def _normalize_angle(a):
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def _limit_accel(vx, vy, vz, pvx, pvy, pvz, max_a, dt):
    if dt <= 0:
        return vx, vy, vz
    dvx, dvy, dvz = vx - pvx, vy - pvy, vz - pvz
    dv = _vec3_len(dvx, dvy, dvz)
    max_dv = max_a * dt
    if dv > max_dv and dv > 0:
        s = max_dv / dv
        vx = pvx + dvx * s
        vy = pvy + dvy * s
        vz = pvz + dvz * s
    return vx, vy, vz

_TYPEMASK_VEL_YAW = (
    (1 << 0) | (1 << 1) | (1 << 2) |
    (1 << 6) | (1 << 7) | (1 << 8) |
    (1 << 9) |
    (1 << 11)
)

def _send_velocity(conn, vx, vy, vz, yaw):
    conn.mav.set_position_target_local_ned_send(
        _timestamp_ms(),
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _TYPEMASK_VEL_YAW,
        0.0, 0.0, 0.0,
        vx,  vy,  vz,
        0.0, 0.0, 0.0,
        yaw, 0.0
    )


# ══════════════════════════════════════════════════════════
#  ANA VURMA DÖNGÜSÜ
# ══════════════════════════════════════════════════════════

def run_strike(conn, get_plane, get_iris, stop_event):
    loop_period = 1.0 / LOOP_HZ
    loop_count  = 0

    pos_filter = EMAVec3(EMA_ALPHA_POS)
    vel_filter = EMAVec3(EMA_ALPHA_VEL)

    prev_time = None
    prev_px = prev_py = prev_pz = None
    prev_los_az = prev_los_el = None
    cmd_vx_prev = cmd_vy_prev = cmd_vz_prev = 0.0
    min_dist_seen = float('inf')
    phase = "APPROACH"

    print("=" * 55)
    print("[STRIKE] VURMA MODU AKTİF")
    print(f"[STRIKE] MAX_SPEED={MAX_SPEED}m/s  PN_GAIN={PN_GAIN}")
    print(f"[STRIKE] TERMINAL_RADIUS={TERMINAL_RADIUS}m")
    print("=" * 55)

    while not stop_event.is_set():
        now = time.monotonic()
        dt  = (now - prev_time) if prev_time is not None else loop_period
        dt  = _clamp(dt, 0.001, 0.1)
        prev_time = now

        # 1) VERİ OKU & FİLTRELE
        plane_raw = get_plane()
        iris_raw  = get_iris()
        px, py, pz = pos_filter.update(plane_raw["x"], plane_raw["y"], plane_raw["z"])
        ix, iy, iz = iris_raw["x"], iris_raw["y"], iris_raw["z"]

        # 2) HEDEF HIZ TAHMİNİ
        if prev_px is not None and dt > 0.001:
            raw_tvx = (px - prev_px) / dt
            raw_tvy = (py - prev_py) / dt
            raw_tvz = (pz - prev_pz) / dt
            tvx, tvy, tvz = vel_filter.update(raw_tvx, raw_tvy, raw_tvz)
        else:
            tvx = tvy = tvz = 0.0
        prev_px, prev_py, prev_pz = px, py, pz
        target_speed = _vec3_len(tvx, tvy, tvz)

        # 3) GEOMETRİ
        rx, ry, rz = px - ix, py - iy, pz - iz
        dist = _vec3_len(rx, ry, rz)

        if dist < 0.05:
            _send_velocity(conn, 0, 0, 0, math.atan2(ry, rx))
            print("[STRIKE] HEDEFE ULASILDI! dist < 0.05m")
            time.sleep(loop_period)
            continue

        min_dist_seen = min(min_dist_seen, dist)

        los_az = math.atan2(ry, rx)
        los_el = math.atan2(-rz, math.sqrt(rx*rx + ry*ry))

        los_unit_x, los_unit_y, los_unit_z = rx/dist, ry/dist, rz/dist

        rel_vx = tvx - cmd_vx_prev
        rel_vy = tvy - cmd_vy_prev
        rel_vz = tvz - cmd_vz_prev
        closing_vel = -(rel_vx*los_unit_x + rel_vy*los_unit_y + rel_vz*los_unit_z)

        # 4) FAZ
        phase = "TERMINAL" if dist <= TERMINAL_RADIUS else "APPROACH"

        # 5) GÜDÜM
        if phase == "APPROACH":
            if prev_los_az is not None and dt > 0.001:
                los_rate_az = _normalize_angle(los_az - prev_los_az) / dt
                los_rate_el = (los_el - prev_los_el) / dt
            else:
                los_rate_az = los_rate_el = 0.0

            vc_effective = max(closing_vel, MIN_SPEED)
            a_cmd_az = PN_GAIN * vc_effective * los_rate_az
            a_cmd_el = PN_GAIN * vc_effective * los_rate_el

            base_speed = _clamp(APPROACH_P_GAIN * dist, MIN_SPEED, MAX_SPEED)
            cmd_vx = los_unit_x * base_speed
            cmd_vy = los_unit_y * base_speed
            cmd_vz = los_unit_z * base_speed

            perp_az_x = -math.sin(los_az)
            perp_az_y =  math.cos(los_az)
            cmd_vx += perp_az_x * a_cmd_az * dt
            cmd_vy += perp_az_y * a_cmd_az * dt
            cmd_vz += -a_cmd_el * dt

            cmd_vx += tvx
            cmd_vy += tvy
            cmd_vz += tvz
        else:
            raw_rx = plane_raw["x"] - ix
            raw_ry = plane_raw["y"] - iy
            raw_rz = plane_raw["z"] - iz
            raw_dist = _vec3_len(raw_rx, raw_ry, raw_rz)
            if raw_dist > 0.05:
                speed = _clamp(TERMINAL_P_GAIN * raw_dist, MIN_SPEED, MAX_SPEED)
                cmd_vx = (raw_rx/raw_dist)*speed + tvx
                cmd_vy = (raw_ry/raw_dist)*speed + tvy
                cmd_vz = (raw_rz/raw_dist)*speed + tvz
            else:
                cmd_vx, cmd_vy, cmd_vz = tvx, tvy, tvz

        prev_los_az, prev_los_el = los_az, los_el

        # 6) HIZ SINIRLA
        cmd_speed = _vec3_len(cmd_vx, cmd_vy, cmd_vz)
        if cmd_speed > MAX_SPEED:
            s = MAX_SPEED / cmd_speed
            cmd_vx *= s; cmd_vy *= s; cmd_vz *= s

        # 7) İVME SINIRLA (sadece approach)
        if phase == "APPROACH":
            cmd_vx, cmd_vy, cmd_vz = _limit_accel(
                cmd_vx, cmd_vy, cmd_vz,
                cmd_vx_prev, cmd_vy_prev, cmd_vz_prev,
                MAX_ACCEL, dt
            )
        cmd_vx_prev, cmd_vy_prev, cmd_vz_prev = cmd_vx, cmd_vy, cmd_vz

        # 8) İRTİFA GÜVENLİĞİ
        if iz + cmd_vz * dt > ALT_FLOOR:
            cmd_vz = min(cmd_vz, 0.0)

        # 9) YAW
        if phase == "APPROACH":
            toi = min(dist / max(cmd_speed, 1.0), 2.0)
            look_x = px + tvx*toi*0.5 - ix
            look_y = py + tvy*toi*0.5 - iy
        else:
            look_x = plane_raw["x"] - ix
            look_y = plane_raw["y"] - iy
        look_yaw = math.atan2(look_y, look_x)

        # 10) KOMUT GÖNDER
        _send_velocity(conn, cmd_vx, cmd_vy, cmd_vz, look_yaw)

        # 11) LOG
        loop_count += 1
        if loop_count % LOOP_HZ == 0:
            actual_cmd_speed = _vec3_len(cmd_vx, cmd_vy, cmd_vz)
            print(f"[STRIKE|{phase:8s}] dist={dist:.2f}m Vc={closing_vel:+.1f}m/s "
                  f"cmd={actual_cmd_speed:.1f}m/s tgt_spd={target_speed:.1f}m/s min_d={min_dist_seen:.2f}m")

        elapsed = time.monotonic() - now
        sleep_time = loop_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    print(f"[STRIKE] Stop sinyali alındı. Minimum mesafe: {min_dist_seen:.2f}m")
