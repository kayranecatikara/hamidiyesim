"""
=============================================================
  AVCI İHA TAKİP ALGORİTMASI — TEKNOFEST 2026 v2 (ArduPilot)
=============================================================
Yenilikler (v2):
  1. Mesafe-Tabanlı Proaktif Kilitlenme (Pinhole + Aspect Angle)
  2. Kamera tilt (25° yukarı) telafisi — drone hedeften yukarıda uçar
  3. State machine: SPRINT → APPROACH → LOCK → STRIKE
  4. Adaptif lock distance (aspect angle'a göre)
  5. TRACK modunda offset doğru hesaplanıyor (predict yok, anlık pozisyon)

Autopilot: ArduCopter GUIDED. Setpoint'ler SET_POSITION_TARGET_LOCAL_NED
(pozisyon + feedforward hız + yaw) ile gönderilir; GUIDED bu mesajı destekler.
NOT: Yüksek hız için ArduCopter'da ANGLE_MAX yükseltilmeli (avci_copter.parm)
— aksi halde drone ~10 m/s'de takılır ve hedefi yakalayamaz.
=============================================================
"""

import math
import time
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil

# ══════════════════════════════════════════════════════════
#  HEDEF VE KAMERA SABİTLERİ (Cessna hedef uçağı + Avcı Drone)
# ══════════════════════════════════════════════════════════
# ArduPilot mini_talon_vtail mesh'i (scale 1.0) yaklaşık boyutları:
PLANE_WINGSPAN_M    = 1.20     # mini Talon kanat açıklığı
PLANE_FUSELAGE_M    = 0.70     # mini Talon gövde uzunluğu

CAM_FOV_DEG         = 125.0    # Avcı drone kamerasi yatay FOV
CAM_FOV_RAD         = math.radians(CAM_FOV_DEG)
TAN_HALF_FOV        = math.tan(CAM_FOV_RAD / 2.0)   # ≈ 1.921 (125° için)

# Kamera 25° YUKARI tilt — drone öne eğilince ufuk merkezde kalır.
# Drone yatay uçarken kamera ufkun 25° üstünü merkeze alır.
# Bu yüzden Talon'u kameranın merkezinde tutmak için drone Talon'dan YUKARIDA olmalı.
CAM_TILT_DEG        = 25.0
CAM_TILT_RAD        = math.radians(CAM_TILT_DEG)

# Şartname kuralı: Talon ekranda yatayda en az %5 kaplamalı
LOCK_PIXEL_RATIO_MIN = 0.05
# Bizim güvenli hedef oran: %7.5 (eşiğin %50 üstü, parazite karşı tampon)
LOCK_PIXEL_RATIO_TARGET = 0.075

# ══════════════════════════════════════════════════════════
#  STATE MACHINE EŞİKLERİ
# ══════════════════════════════════════════════════════════

# Mesafe eşikleri
APPROACH_DIST       = 30.0     # >30m: SPRINT (hızla yetiş)
# 10-30m arası: APPROACH (azalan hızla yaklaş)
# Lock target ± 30%: LOCK (lock topla, drone kararlı)
LOCK_DIST_MIN_MULT  = 0.7      # lock_target * 0.7
LOCK_DIST_MAX_MULT  = 1.4      # lock_target * 1.4
STRIKE_DIST         = 5.0      # <5m: STRIKE (kamikaze)

# Hız limitleri (ArduCopter ANGLE_MAX=55° ile ~18.6 m/s ölçüldü)
MAX_SPEED           = 19.5
MAX_SPEED_SPRINT    = 19.9
MAX_SPEED_LOCK      = 17.0     # lock'ta plane'le eşit hıza yakın
SPEED_MARGIN_LOCK   = 1.0      # plane_speed + 1 m/s
STRIKE_SPEED        = 19.9

# PID kazançları (mod-bazlı)
KP_SPRINT           = 7.0      # uzaktan agresif yetişme
KP_APPROACH         = 5.0      # orta mesafe
KP_LOCK             = 2.0      # lock'ta yumuşak
KP_STRIKE           = 7.0      # son darbe agresif
KI                  = 0.1
KD                  = 0.4
INTEGRAL_MAX        = 4.0

# Diğer
MAX_ACCEL           = 15.0
MAX_YAW_RATE        = math.radians(360)
EMA_ALPHA_POS       = 0.4
EMA_ALPHA_YAW       = 0.6
PREDICT_HORIZON_FAR = 0.6      # SPRINT/APPROACH'de
PREDICT_HORIZON_LOCK = 0.2     # LOCK'ta düşük (overshoot olmasın)
PREDICT_HORIZON_STRIKE = 0.8   # STRIKE'da yüksek (intercept point)

DEADBAND            = 0.5
ALT_MIN             = -100.0
ALT_MAX             = -2.0
LOOP_HZ             = 20


# ══════════════════════════════════════════════════════════
#  YARDIMCI SINIFLAR
# ══════════════════════════════════════════════════════════

class EMAFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self._val = None
    def update(self, raw):
        if self._val is None:
            self._val = raw
        else:
            self._val = self.alpha * raw + (1.0 - self.alpha) * self._val
        return self._val
    @property
    def value(self):
        return self._val


class Vec3EMAFilter:
    def __init__(self, alpha):
        self.fx = EMAFilter(alpha)
        self.fy = EMAFilter(alpha)
        self.fz = EMAFilter(alpha)
    def update(self, x, y, z):
        return (self.fx.update(x), self.fy.update(y), self.fz.update(z))


class PIDController:
    def __init__(self, kp, ki, kd, i_max):
        self.kp = kp; self.ki = ki; self.kd = kd; self.i_max = i_max
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
#  YARDIMCI FONKSİYONLAR
# ══════════════════════════════════════════════════════════

def _timestamp_ms():
    return int(time.time() * 1e6) & 0xFFFFFFFF

def _clamp(val, lo, hi):
    return max(lo, min(hi, val))

def _normalize_angle(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def _vec3_len(x, y, z):
    return math.sqrt(x*x + y*y + z*z)

def _limit_acceleration(vx_cmd, vy_cmd, vz_cmd, vx_p, vy_p, vz_p, max_a, dt):
    if dt <= 0:
        return vx_cmd, vy_cmd, vz_cmd
    dvx, dvy, dvz = vx_cmd - vx_p, vy_cmd - vy_p, vz_cmd - vz_p
    dv = _vec3_len(dvx, dvy, dvz)
    max_dv = max_a * dt
    if dv > max_dv and dv > 0:
        s = max_dv / dv
        return vx_p + dvx * s, vy_p + dvy * s, vz_p + dvz * s
    return vx_cmd, vy_cmd, vz_cmd


def compute_visible_width(plane_yaw_rad, drone_to_plane_bearing_rad):
    """
    Talon'un drone bakış açısına göre görünen yatay genişliği.
    aspect_angle = plane'e drone'dan bakış ile plane'in heading'i arasındaki açı.

    aspect = 0   → tam arkadan (kanat görünür: 1.718m)
    aspect = π/2 → tam yandan (sadece gövde: 1.20m)
    aspect = π   → tam önden (kanat görünür: 1.718m)
    """
    aspect = _normalize_angle(drone_to_plane_bearing_rad - plane_yaw_rad)
    visible = (PLANE_WINGSPAN_M * abs(math.cos(aspect)) +
               PLANE_FUSELAGE_M * abs(math.sin(aspect)))
    return visible, aspect


def compute_optimal_lock_distance(visible_width):
    """
    Pinhole kamera modeli — Talon'un ekranda LOCK_PIXEL_RATIO_TARGET
    oranını kapladığı mesafeyi döndür.

    pinhole: piksel_oran = w_görünür / (2 * d * tan(FOV/2))
    çözülür: d = w_görünür / (2 * oran * tan(FOV/2))
    """
    return visible_width / (2.0 * LOCK_PIXEL_RATIO_TARGET * TAN_HALF_FOV)


def compute_vertical_offset(horizontal_distance):
    """
    Kamera 25° yukarı tilt'li → drone Talon'a horizontal_distance'tan
    bakıyorsa, Talon'u kameranın merkezinde tutmak için drone Talon'dan
    şu kadar YUKARIDA olmalı:
        h = horizontal_distance * tan(CAM_TILT)

    NED frame: drone z = plane_z - h (plane'den daha negatif z = yukarıda)
    """
    return horizontal_distance * math.tan(CAM_TILT_RAD)


_TYPEMASK_POS_VEL_YAW = (
    (1 << 6) | (1 << 7) | (1 << 8) |  # afxyz ignore
    (1 << 9)  |                        # force ignore
    (1 << 11)                          # yaw_rate ignore
)

def _send_setpoint(conn, x, y, z, vx, vy, vz, yaw):
    conn.mav.set_position_target_local_ned_send(
        _timestamp_ms(),
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _TYPEMASK_POS_VEL_YAW,
        x, y, z, vx, vy, vz, 0.0, 0.0, 0.0, yaw, 0.0
    )


# ══════════════════════════════════════════════════════════
#  ANA TAKİP DÖNGÜSÜ
# ══════════════════════════════════════════════════════════

def run_chase(conn, get_plane, get_iris, stop_event):
    loop_period = 1.0 / LOOP_HZ

    pos_filter = Vec3EMAFilter(EMA_ALPHA_POS)
    yaw_filter = EMAFilter(EMA_ALPHA_YAW)
    pid = PIDController(KP_SPRINT, KI, KD, INTEGRAL_MAX)

    prev_time = None
    prev_px = prev_py = prev_pz = None
    prev_yaw_filt = None
    plane_vx = plane_vy = plane_vz = 0.0
    yaw_rate = 0.0
    vx_prev = vy_prev = vz_prev = 0.0
    current_yaw = 0.0
    loop_count = 0

    print("=" * 60)
    print("[CHASE ALG v2] TEKNOFEST 2026 — Mesafe-Tabanlı Proaktif Lock")
    print(f"[CHASE ALG v2] FOV={CAM_FOV_DEG}°  Tilt={CAM_TILT_DEG}°↑  "
          f"Talon w={PLANE_WINGSPAN_M}m")
    print(f"[CHASE ALG v2] Lock target ratio={LOCK_PIXEL_RATIO_TARGET*100:.1f}%")
    print("=" * 60)

    while not stop_event.is_set():
        now = time.monotonic()
        dt = (now - prev_time) if prev_time is not None else loop_period
        dt = _clamp(dt, 0.001, 0.2)
        prev_time = now

        # ──────────────────────────────────────────────────
        # 1) VERİ OKU & FİLTRELE
        # ──────────────────────────────────────────────────
        plane_raw = get_plane()
        iris_raw  = get_iris()

        px, py, pz = pos_filter.update(plane_raw["x"], plane_raw["y"], plane_raw["z"])
        # Plane yaw: ENU (deg) → NED (rad)
        enu_yaw = math.radians(plane_raw.get("yaw", 0.0))
        ned_yaw = math.pi / 2.0 - enu_yaw
        ned_yaw = _normalize_angle(ned_yaw)
        plane_yaw = yaw_filter.update(ned_yaw)

        ix, iy, iz = iris_raw["x"], iris_raw["y"], iris_raw["z"]

        # ──────────────────────────────────────────────────
        # 2) PLANE HIZ & GERÇEK MESAFE
        # ──────────────────────────────────────────────────
        if prev_px is not None and dt > 0:
            plane_vx = (px - prev_px) / dt
            plane_vy = (py - prev_py) / dt
            plane_vz = (pz - prev_pz) / dt
            if prev_yaw_filt is not None:
                yaw_rate = _normalize_angle(plane_yaw - prev_yaw_filt) / dt
        prev_px, prev_py, prev_pz = px, py, pz
        prev_yaw_filt = plane_yaw
        plane_speed = _vec3_len(plane_vx, plane_vy, plane_vz)

        # Drone-Plane gerçek (anlık) yatay/3D mesafesi
        dx_pt = px - ix
        dy_pt = py - iy
        dz_pt = pz - iz
        plane_dist_3d   = _vec3_len(dx_pt, dy_pt, dz_pt)
        plane_dist_horz = math.sqrt(dx_pt*dx_pt + dy_pt*dy_pt)

        # Drone'dan plane'e yatay bearing (NED frame)
        bearing_to_plane = math.atan2(dy_pt, dx_pt)

        # ──────────────────────────────────────────────────
        # 3) ASPECT ANGLE & DİNAMİK LOCK MESAFESİ
        # ──────────────────────────────────────────────────
        visible_w, aspect = compute_visible_width(plane_yaw, bearing_to_plane)
        d_lock_optimal    = compute_optimal_lock_distance(visible_w)
        # Aspect'e göre 5-15m arası bir lock mesafesi çıkacak

        # Mod tespiti
        if plane_dist_3d < STRIKE_DIST:
            current_mode = "STRIKE"
            predict_horizon = PREDICT_HORIZON_STRIKE
        elif plane_dist_3d < d_lock_optimal * LOCK_DIST_MAX_MULT:
            # Lock zonu (örn. d_lock=8m → 5.6-11.2m arası lock)
            current_mode = "LOCK"
            predict_horizon = PREDICT_HORIZON_LOCK
        elif plane_dist_3d > APPROACH_DIST:
            current_mode = "SPRINT"
            predict_horizon = PREDICT_HORIZON_FAR
        else:
            current_mode = "APPROACH"
            predict_horizon = PREDICT_HORIZON_FAR

        # ──────────────────────────────────────────────────
        # 4) PREDİKTİF HEDEF NOKTASI (mod-bazlı predict horizon)
        # ──────────────────────────────────────────────────
        pred_x = px + plane_vx * predict_horizon
        pred_y = py + plane_vy * predict_horizon
        pred_z = pz + plane_vz * predict_horizon

        # Vertical offset (kamera tilt telafisi)
        # Drone Talon'dan yukarıda olmalı → NED'de daha negatif z
        vert_offset = compute_vertical_offset(min(plane_dist_horz, 30.0))

        # ──────────────────────────────────────────────────
        # 5) HEDEF NOKTAYI (chase point) HESAPLA — moda göre
        # ──────────────────────────────────────────────────
        if current_mode == "STRIKE":
            # Kamikaze: doğrudan plane'e (predict ile intercept)
            chase_x = pred_x
            chase_y = pred_y
            chase_z = _clamp(pred_z, ALT_MIN, ALT_MAX)

        elif current_mode == "SPRINT":
            # Uzaktan yetiş: doğrudan plane'in pozisyonu (offset yok)
            chase_x = pred_x
            chase_y = pred_y
            # SPRINT'te de yukarıdan yaklaş ki kamera Talon'u görsün
            chase_z = _clamp(pred_z - vert_offset, ALT_MIN, ALT_MAX)

        elif current_mode == "APPROACH":
            # Orta mesafe: lock_optimal'e yaklaş ama plane'in arkasından
            # NOT: predict YAPMA (TRACK modunda overshoot oluyordu).
            # Plane'in ANLIK pozisyonuna offset uygulanıyor.
            chase_x = px - math.cos(plane_yaw) * d_lock_optimal
            chase_y = py - math.sin(plane_yaw) * d_lock_optimal
            chase_z = _clamp(pz - vert_offset, ALT_MIN, ALT_MAX)

        else:  # LOCK
            # Lock zonu: plane'in arkasında d_lock_optimal mesafede stabilize
            # vert_offset ile yukarıda
            chase_x = px - math.cos(plane_yaw) * d_lock_optimal
            chase_y = py - math.sin(plane_yaw) * d_lock_optimal
            chase_z = _clamp(pz - vert_offset, ALT_MIN, ALT_MAX)

        # ──────────────────────────────────────────────────
        # 6) PID + FEEDFORWARD
        # ──────────────────────────────────────────────────
        err_x = chase_x - ix
        err_y = chase_y - iy
        err_z = chase_z - iz
        err   = _vec3_len(err_x, err_y, err_z)

        # PID kazancı moda göre
        if current_mode == "STRIKE":
            pid.kp = KP_STRIKE; speed_limit = STRIKE_SPEED
        elif current_mode == "SPRINT":
            pid.kp = KP_SPRINT; speed_limit = MAX_SPEED_SPRINT
        elif current_mode == "APPROACH":
            pid.kp = KP_APPROACH; speed_limit = MAX_SPEED
        else:  # LOCK
            pid.kp = KP_LOCK
            speed_limit = min(MAX_SPEED_LOCK, plane_speed + SPEED_MARGIN_LOCK)

        if err > DEADBAND:
            pid_vx, pid_vy, pid_vz = pid.compute((err_x, err_y, err_z), dt)
            cmd_vx = plane_vx + pid_vx
            cmd_vy = plane_vy + pid_vy
            cmd_vz = plane_vz + pid_vz
            total = _vec3_len(cmd_vx, cmd_vy, cmd_vz)
            if total > speed_limit:
                s = speed_limit / total
                cmd_vx *= s; cmd_vy *= s; cmd_vz *= s
        else:
            cmd_vx = cmd_vy = cmd_vz = 0.0
            pid.reset()

        # ──────────────────────────────────────────────────
        # 7) İVME SINIRLA
        # ──────────────────────────────────────────────────
        cmd_vx, cmd_vy, cmd_vz = _limit_acceleration(
            cmd_vx, cmd_vy, cmd_vz, vx_prev, vy_prev, vz_prev, MAX_ACCEL, dt
        )
        vx_prev, vy_prev, vz_prev = cmd_vx, cmd_vy, cmd_vz

        # ──────────────────────────────────────────────────
        # 8) YAW — drone burnu Talon'a kilitlensin
        # ──────────────────────────────────────────────────
        # Mesafede pred kullan (Talon'un olacağı yere bak), yakında anlık kullan
        if current_mode in ("LOCK", "STRIKE"):
            target_yaw = math.atan2(py - iy, px - ix)
        else:
            target_yaw = math.atan2(pred_y - iy, pred_x - ix)
        yaw_err = _normalize_angle(target_yaw - current_yaw)
        max_step = MAX_YAW_RATE * dt
        current_yaw = _normalize_angle(current_yaw + _clamp(yaw_err, -max_step, max_step))

        # ──────────────────────────────────────────────────
        # 9) SETPOINT GÖNDER
        # ──────────────────────────────────────────────────
        _send_setpoint(conn, chase_x, chase_y, chase_z, cmd_vx, cmd_vy, cmd_vz, current_yaw)

        # ──────────────────────────────────────────────────
        # 10) LOG
        # ──────────────────────────────────────────────────
        loop_count += 1

        # Mod geçişi log'u
        if loop_count == 1 or current_mode != getattr(run_chase, '_last_mode', None):
            aspect_deg = math.degrees(aspect)
            print(f"[CHASE ALG v2] >>> {current_mode} <<< "
                  f"plane_dist={plane_dist_3d:.1f}m  "
                  f"d_lock_opt={d_lock_optimal:.1f}m  "
                  f"visible_w={visible_w:.2f}m  aspect={aspect_deg:.0f}°  "
                  f"vert_off={vert_offset:.1f}m")
        run_chase._last_mode = current_mode

        # Periyodik log (her 3 saniyede)
        if loop_count % (LOOP_HZ * 3) == 0:
            actual_speed = _vec3_len(cmd_vx, cmd_vy, cmd_vz)
            aspect_deg = math.degrees(aspect)
            print(f"[CHASE ALG v2] {current_mode} "
                  f"plane=({px:.1f},{py:.1f},{pz:.1f}) "
                  f"iris=({ix:.1f},{iy:.1f},{iz:.1f}) "
                  f"dist={plane_dist_3d:.1f}m d_opt={d_lock_optimal:.1f}m "
                  f"asp={aspect_deg:.0f}° err={err:.1f}m "
                  f"v={actual_speed:.1f} lim={speed_limit:.1f} "
                  f"plane_v={plane_speed:.1f} "
                  f"voff={vert_offset:.1f}m yaw={math.degrees(current_yaw):.0f}°")

        elapsed = time.monotonic() - now
        sleep_time = loop_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    print("[CHASE ALG v2] Stop sinyali alındı, döngü sonlandı.")