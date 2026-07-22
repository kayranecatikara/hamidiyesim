"""
=============================================================
  GPS TAKİP HATTI — chase (TEKNOFEST 2026, ArduPilot)
=============================================================
SAF GPS/telemetri güdümü. İki aracın da tam konum+hız+rotasyonu bilinir.
State machine: SPRINT → APPROACH → LOCK → STRIKE.

NOT: Buradaki "kamera" matematiği (FOV, pixel oranı, aspect, tilt) GERÇEK kamera
ölçümü DEĞİLDİR — GPS geometrisinden türetilen simüle bir pinhole modelidir;
hedefi kadrajda ideal oranda tutacak duruş mesafesini/irtifasını hesaplar.
Gerçek görsel güdüm ayrı hattadır (visual_guidance.py, Faz 3).

Autopilot: ArduCopter GUIDED. Setpoint: SET_POSITION_TARGET_LOCAL_NED
(pozisyon + feedforward hız + yaw). Yüksek hız için ANGLE_MAX yükseltilmeli
(avci_copter.parm) — aksi halde drone ~10 m/s'de takılır.
=============================================================
"""

import math
import time

from control.guidance.common import (
    EMAScalar, EMAVec3, PIDController,
    clamp, normalize_angle, vec3_len, limit_acceleration,
    send_position_setpoint,
)

# ══════════════════════════════════════════════════════════
#  HEDEF VE KAMERA SABİTLERİ (mini Talon hedef + Avcı Drone)
# ══════════════════════════════════════════════════════════
PLANE_WINGSPAN_M    = 1.20     # mini Talon kanat açıklığı
PLANE_FUSELAGE_M    = 0.70     # mini Talon gövde uzunluğu

CAM_FOV_DEG         = 125.0    # Avcı drone kamerası yatay FOV
CAM_FOV_RAD         = math.radians(CAM_FOV_DEG)
TAN_HALF_FOV        = math.tan(CAM_FOV_RAD / 2.0)   # ≈ 1.921 (125° için)

# Kamera 25° YUKARI tilt — drone Talon'u kadraj merkezinde tutmak için ondan
# YUKARIDA uçmalı.
CAM_TILT_DEG        = 25.0
CAM_TILT_RAD        = math.radians(CAM_TILT_DEG)

# Şartname: Talon ekranda yatayda en az %5 kaplamalı; güvenli hedef %7.5.
LOCK_PIXEL_RATIO_MIN    = 0.05
LOCK_PIXEL_RATIO_TARGET = 0.075

# ══════════════════════════════════════════════════════════
#  STATE MACHINE EŞİKLERİ
# ══════════════════════════════════════════════════════════
APPROACH_DIST       = 30.0     # >30m: SPRINT
LOCK_DIST_MIN_MULT  = 0.7      # lock_target * 0.7  (NOT: mod koşulunda kullanılmıyor)
LOCK_DIST_MAX_MULT  = 1.4      # lock_target * 1.4
STRIKE_DIST         = 5.0      # <5m: STRIKE

# Hız limitleri (ArduCopter ANGLE_MAX=55° ile ~18.6 m/s ölçüldü)
MAX_SPEED           = 19.5
MAX_SPEED_SPRINT    = 19.9
MAX_SPEED_LOCK      = 17.0
SPEED_MARGIN_LOCK   = 1.0
STRIKE_SPEED        = 19.9

# PID kazançları (mod-bazlı)
KP_SPRINT           = 7.0
KP_APPROACH         = 5.0
KP_LOCK             = 2.0
KP_STRIKE           = 7.0
KI                  = 0.1
KD                  = 0.4
INTEGRAL_MAX        = 4.0

# Diğer
MAX_ACCEL           = 15.0
MAX_YAW_RATE        = math.radians(360)
EMA_ALPHA_POS       = 0.4
EMA_ALPHA_YAW       = 0.6
PREDICT_HORIZON_FAR = 0.6
PREDICT_HORIZON_LOCK = 0.2
PREDICT_HORIZON_STRIKE = 0.8

DEADBAND            = 0.5
ALT_MIN             = -100.0
ALT_MAX             = -2.0
LOOP_HZ             = 20


# ══════════════════════════════════════════════════════════
#  PINHOLE (simüle) YARDIMCI FONKSİYONLAR
# ══════════════════════════════════════════════════════════

def compute_visible_width(plane_yaw_rad, drone_to_plane_bearing_rad):
    """
    Talon'un drone bakış açısına göre görünen yatay genişliği (aspect angle'a göre).
    aspect=0 tam arkadan (kanat), π/2 tam yandan (gövde), π tam önden (kanat).
    """
    aspect = normalize_angle(drone_to_plane_bearing_rad - plane_yaw_rad)
    visible = (PLANE_WINGSPAN_M * abs(math.cos(aspect)) +
               PLANE_FUSELAGE_M * abs(math.sin(aspect)))
    return visible, aspect


def compute_optimal_lock_distance(visible_width):
    """
    Pinhole: Talon'un ekranda LOCK_PIXEL_RATIO_TARGET oranını kapladığı mesafe.
        d = w_görünür / (2 * oran * tan(FOV/2))
    """
    return visible_width / (2.0 * LOCK_PIXEL_RATIO_TARGET * TAN_HALF_FOV)


def compute_vertical_offset(horizontal_distance):
    """
    Kamera 25° yukarı tilt'li → Talon'u merkezde tutmak için drone ondan
    h = horizontal_distance * tan(CAM_TILT) kadar YUKARIDA olmalı (NED'de daha negatif z).
    """
    return horizontal_distance * math.tan(CAM_TILT_RAD)


# ══════════════════════════════════════════════════════════
#  ANA TAKİP DÖNGÜSÜ
# ══════════════════════════════════════════════════════════

def run_chase(conn, get_plane, get_iris, stop_event):
    loop_period = 1.0 / LOOP_HZ

    pos_filter = EMAVec3(EMA_ALPHA_POS)
    yaw_filter = EMAScalar(EMA_ALPHA_YAW)
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
        dt = clamp(dt, 0.001, 0.2)
        prev_time = now

        # 1) VERİ OKU & FİLTRELE
        plane_raw = get_plane()
        iris_raw  = get_iris()

        px, py, pz = pos_filter.update(plane_raw["x"], plane_raw["y"], plane_raw["z"])
        # Plane yaw: ENU (deg) → NED (rad)
        enu_yaw = math.radians(plane_raw.get("yaw", 0.0))
        ned_yaw = math.pi / 2.0 - enu_yaw
        ned_yaw = normalize_angle(ned_yaw)
        plane_yaw = yaw_filter.update(ned_yaw)

        ix, iy, iz = iris_raw["x"], iris_raw["y"], iris_raw["z"]

        # 2) PLANE HIZ & GERÇEK MESAFE
        if prev_px is not None and dt > 0:
            plane_vx = (px - prev_px) / dt
            plane_vy = (py - prev_py) / dt
            plane_vz = (pz - prev_pz) / dt
            if prev_yaw_filt is not None:
                yaw_rate = normalize_angle(plane_yaw - prev_yaw_filt) / dt
        prev_px, prev_py, prev_pz = px, py, pz
        prev_yaw_filt = plane_yaw
        plane_speed = vec3_len(plane_vx, plane_vy, plane_vz)

        dx_pt = px - ix
        dy_pt = py - iy
        dz_pt = pz - iz
        plane_dist_3d   = vec3_len(dx_pt, dy_pt, dz_pt)
        plane_dist_horz = math.sqrt(dx_pt*dx_pt + dy_pt*dy_pt)

        bearing_to_plane = math.atan2(dy_pt, dx_pt)

        # 3) ASPECT ANGLE & DİNAMİK LOCK MESAFESİ
        visible_w, aspect = compute_visible_width(plane_yaw, bearing_to_plane)
        d_lock_optimal    = compute_optimal_lock_distance(visible_w)

        if plane_dist_3d < STRIKE_DIST:
            current_mode = "STRIKE"
            predict_horizon = PREDICT_HORIZON_STRIKE
        elif plane_dist_3d < d_lock_optimal * LOCK_DIST_MAX_MULT:
            current_mode = "LOCK"
            predict_horizon = PREDICT_HORIZON_LOCK
        elif plane_dist_3d > APPROACH_DIST:
            current_mode = "SPRINT"
            predict_horizon = PREDICT_HORIZON_FAR
        else:
            current_mode = "APPROACH"
            predict_horizon = PREDICT_HORIZON_FAR

        # 4) PREDİKTİF HEDEF NOKTASI
        pred_x = px + plane_vx * predict_horizon
        pred_y = py + plane_vy * predict_horizon
        pred_z = pz + plane_vz * predict_horizon

        vert_offset = compute_vertical_offset(min(plane_dist_horz, 30.0))

        # 5) HEDEF NOKTAYI (chase point) HESAPLA — moda göre
        if current_mode == "STRIKE":
            chase_x = pred_x
            chase_y = pred_y
            chase_z = clamp(pred_z, ALT_MIN, ALT_MAX)

        elif current_mode == "SPRINT":
            chase_x = pred_x
            chase_y = pred_y
            chase_z = clamp(pred_z - vert_offset, ALT_MIN, ALT_MAX)

        elif current_mode == "APPROACH":
            chase_x = px - math.cos(plane_yaw) * d_lock_optimal
            chase_y = py - math.sin(plane_yaw) * d_lock_optimal
            chase_z = clamp(pz - vert_offset, ALT_MIN, ALT_MAX)

        else:  # LOCK
            chase_x = px - math.cos(plane_yaw) * d_lock_optimal
            chase_y = py - math.sin(plane_yaw) * d_lock_optimal
            chase_z = clamp(pz - vert_offset, ALT_MIN, ALT_MAX)

        # 6) PID + FEEDFORWARD
        err_x = chase_x - ix
        err_y = chase_y - iy
        err_z = chase_z - iz
        err   = vec3_len(err_x, err_y, err_z)

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
            total = vec3_len(cmd_vx, cmd_vy, cmd_vz)
            if total > speed_limit:
                s = speed_limit / total
                cmd_vx *= s; cmd_vy *= s; cmd_vz *= s
        else:
            cmd_vx = cmd_vy = cmd_vz = 0.0
            pid.reset()

        # 7) İVME SINIRLA
        cmd_vx, cmd_vy, cmd_vz = limit_acceleration(
            cmd_vx, cmd_vy, cmd_vz, vx_prev, vy_prev, vz_prev, MAX_ACCEL, dt
        )
        vx_prev, vy_prev, vz_prev = cmd_vx, cmd_vy, cmd_vz

        # 8) YAW — drone burnu Talon'a kilitlensin
        if current_mode in ("LOCK", "STRIKE"):
            target_yaw = math.atan2(py - iy, px - ix)
        else:
            target_yaw = math.atan2(pred_y - iy, pred_x - ix)
        yaw_err = normalize_angle(target_yaw - current_yaw)
        max_step = MAX_YAW_RATE * dt
        current_yaw = normalize_angle(current_yaw + clamp(yaw_err, -max_step, max_step))

        # 9) SETPOINT GÖNDER
        send_position_setpoint(conn, chase_x, chase_y, chase_z, cmd_vx, cmd_vy, cmd_vz, current_yaw)

        # 10) LOG
        loop_count += 1
        if loop_count == 1 or current_mode != getattr(run_chase, '_last_mode', None):
            aspect_deg = math.degrees(aspect)
            print(f"[CHASE ALG v2] >>> {current_mode} <<< "
                  f"plane_dist={plane_dist_3d:.1f}m  "
                  f"d_lock_opt={d_lock_optimal:.1f}m  "
                  f"visible_w={visible_w:.2f}m  aspect={aspect_deg:.0f}°  "
                  f"vert_off={vert_offset:.1f}m")
        run_chase._last_mode = current_mode

        if loop_count % (LOOP_HZ * 3) == 0:
            actual_speed = vec3_len(cmd_vx, cmd_vy, cmd_vz)
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
