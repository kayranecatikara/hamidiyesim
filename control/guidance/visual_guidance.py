"""
=============================================================
  GÖRSEL GÜDÜM HATTI — IBVS (Image-Based Visual Servoing)
=============================================================
GPS YOK. Sadece kameradan YOLO tespiti (bbox) kullanılır. GPS güdümünden farkı:
hedefin dünya konumu bilinmez; yalnız görüntüdeki yeri (cx,cy) ve boyutu (w,h) bilinir.

Kontrol (basitleştirilmiş IBVS):
  yatay hata ex=(cx-CX)/CX  → drone YAW (hedefi kadraj ortasına döndür)
  dikey hata  ey=(cy-CY)/CY  → dikey hız vz (hedefi ortada tut; kamera 25° tilt telafili)
  bbox genişliği w           → yaklaşma: w<hedef → ileri hızlan, hedefe gelince dur
Drone burnunun (yaw) baktığı yönde ilerler → hedefe yaklaşır.

Girdi callback'leri:
  get_detection() -> dict|None   YOLO çıktısı {cx,cy,w,h,conf,bbox} (vision/detection_state)
  get_iris()      -> dict        {x,y,z,yaw}  (yaw derece; drone gövde yönü)

Autopilot: ArduCopter GUIDED. Setpoint: sadece hız + yaw (strike ile aynı arayüz).
GÜVENLİK: tespit yoksa dur (hover); hız/ivme/irtifa sınırlı; kazançlar konservatif.
"""

import math
import time

from control.guidance.common import (
    clamp, normalize_angle, vec3_len, limit_acceleration, send_velocity,
)
from vision import geometry as geo   # CX, CY, FX (kamera intrinsics)

# ── Kontrol parametreleri (konservatif — canlıda ayarlanır) ──
LOOP_HZ = 30

KP_YAW = 1.4                 # yatay piksel hatası → yaw rate (rad/s per birim ex)
MAX_YAW_RATE = math.radians(90)

KP_VZ = 2.5                  # dikey piksel hatası → dikey hız (m/s per birim ey)
MAX_VZ = 2.5

# Yaklaşma: hedef bbox genişliği (piksel). Bundan küçükse ilerle, büyükse dur/geri.
# ~%6 ekran (LOCK bölgesi) ≈ 0.06*640 ≈ 38 px.
W_TARGET_PX = 40.0
KP_FWD = 0.18               # (W_TARGET - w) → ileri hız
MAX_FWD = 6.0               # görsel güdümde konservatif (GPS chase 19 m/s idi)
MIN_FWD = 0.0               # geri gitme yok (sadece ilerle/dur)

MAX_ACCEL = 6.0
ALT_MIN = -100.0           # NED: en yüksek
ALT_FLOOR = -2.0           # yerden en az 2 m (irtifa güvenliği)

LOST_HOLD_S = 1.0          # tespit kaybında bu kadar süre son yaw'da beklet, sonra dur


def run_visual_guidance(conn, get_detection, get_iris, stop_event):
    """IBVS döngüsü — YOLO bbox'ından drone hız+yaw setpoint üretir."""
    loop_period = 1.0 / LOOP_HZ
    CX, CY, FX = geo.CX, geo.CY, geo.FX

    prev_time = None
    vx_prev = vy_prev = vz_prev = 0.0
    current_yaw = None
    last_seen = 0.0
    loop_count = 0

    print("=" * 55)
    print("[VISUAL IBVS] Görsel güdüm aktif (YOLO bbox → hız)")
    print(f"[VISUAL IBVS] hedef genişlik={W_TARGET_PX:.0f}px  max_fwd={MAX_FWD}m/s")
    print("=" * 55)

    while not stop_event.is_set():
        now = time.monotonic()
        dt = (now - prev_time) if prev_time is not None else loop_period
        dt = clamp(dt, 0.001, 0.2)
        prev_time = now

        iris = get_iris()
        iris_yaw = math.radians(iris.get("yaw", 0.0))   # derece → rad
        if current_yaw is None:
            current_yaw = iris_yaw

        det = get_detection()

        # ── Tespit yok: güvenli davranış (yavaşla/dur) ──
        if det is None or det.get("conf", 0.0) < 0.25:
            if now - last_seen > LOST_HOLD_S:
                send_velocity(conn, 0.0, 0.0, 0.0, current_yaw)   # dur (hover)
                vx_prev = vy_prev = vz_prev = 0.0
            # kısa kayıpta son komutu koru (aşağıdaki setpoint tekrar gönderilmez)
            if loop_count % LOOP_HZ == 0:
                print("[VISUAL IBVS] hedef görünmüyor — bekleniyor")
            loop_count += 1
            _sleep(now, loop_period)
            continue

        last_seen = now
        cx, cy, w = det["cx"], det["cy"], det["w"]

        # ── Görüntü hataları (normalize) ──
        ex = (cx - CX) / CX          # +sağ
        ey = (cy - CY) / CY          # +aşağı

        # ── YAW: hedefi yatayda ortala ──
        yaw_rate = clamp(KP_YAW * ex, -MAX_YAW_RATE, MAX_YAW_RATE)
        current_yaw = normalize_angle(current_yaw + yaw_rate * dt)

        # ── Dikey hız: hedefi dikeyde ortala (aşağıdaysa alçal) ──
        vz = clamp(KP_VZ * ey, -MAX_VZ, MAX_VZ)   # ey>0 (hedef aşağı) → vz>0 (NED aşağı)

        # ── İleri hız: hedef boyutuna göre yaklaş ──
        v_fwd = clamp(KP_FWD * (W_TARGET_PX - w), MIN_FWD, MAX_FWD)
        # hedef kadrajın kenarındaysa önce ortala (ileri hızı kıs)
        v_fwd *= clamp(1.0 - abs(ex), 0.2, 1.0)

        # ── NED hız (drone burnu = yaw yönünde ilerle) ──
        vx = v_fwd * math.cos(current_yaw)
        vy = v_fwd * math.sin(current_yaw)

        # ── İvme sınırla ──
        vx, vy, vz = limit_acceleration(vx, vy, vz, vx_prev, vy_prev, vz_prev, MAX_ACCEL, dt)
        vx_prev, vy_prev, vz_prev = vx, vy, vz

        # ── İrtifa güvenliği (yere yaklaşırken aşağı hızı kes) ──
        if iris.get("z", -10.0) + vz * dt > ALT_FLOOR:
            vz = min(vz, 0.0)

        send_velocity(conn, vx, vy, vz, current_yaw)

        loop_count += 1
        if loop_count % (LOOP_HZ) == 0:
            print(f"[VISUAL IBVS] bbox=({cx},{cy},w={w})  ex={ex:+.2f} ey={ey:+.2f}  "
                  f"v_fwd={v_fwd:.1f} vz={vz:+.1f} yaw={math.degrees(current_yaw):.0f}° conf={det['conf']:.2f}")

        _sleep(now, loop_period)

    send_velocity(conn, 0.0, 0.0, 0.0, current_yaw or 0.0)
    print("[VISUAL IBVS] Stop sinyali — döngü sonlandı.")


def _sleep(t_start, period):
    elapsed = time.monotonic() - t_start
    if elapsed < period:
        time.sleep(period - elapsed)
