"""
plane_patterns.py — Plane için scripted manevra paternleri.

Tüm fonksiyonlar plane_functions.py üzerine inşa edilmiştir.
connect_plane() ve start_gcs_keepalive() önceden çağrılmış olmalıdır.

Parametreler:
- Mesafe/süre parametreleri özelleştirilebilir
- Throttle değerleri 0..1000 aralığında
- Yaw/pitch/roll değerleri -1000..+1000 aralığında
"""

import time
import math
from pymavlink import mavutil

from control.plane_functions import (
    get_conn,
    send_manual_control,
    fly_forward,
    turn_left,
    turn_right,
    climb,
    descend,
    set_throttle,
    set_heading,
    set_pitch,
    set_roll,
    loiter,
    get_plane_position,
    get_plane_attitude,
    print_status,
    THROTTLE_CRUISE,
    THROTTLE_FULL,
    THROTTLE_IDLE,
    CONTROL_RATE,
)
from control.mav_common import (
    set_mode,
    PLANE_MODE_TAKEOFF,
    PLANE_MODE_FBWA,
)


# ---------------------------------------------------------------------------
# Takeoff + Stabilize
# ---------------------------------------------------------------------------

def takeoff_then_stabilize(throttle: int = THROTTLE_FULL,
                           climb_duration: float = 8.0,
                           stabilize_duration: float = 3.0):
    """
    ArduPlane otonom kalkış: TAKEOFF (13) modu motoru açar, uçağı kaldırır ve
    TKOFF_ALT irtifasına tırmandırır. Ardından FBWA (5) moda geçilip RC
    override ile uçuşa devam edilir. (FBWA: roll stick = yatış açısı hedefi;
    STABILIZE'a göre dönüşler çok daha etkilidir, stall'a karşı açı limitli.)

    Not: arm_plane() önceden çağrılmış olmalı.
    """
    print("[PATTERN] Otonom Takeoff (TAKEOFF Mode) başlıyor")
    conn = get_conn()

    # ArduPlane TAKEOFF modu (13) — tek parametreli custom_mode
    set_mode(conn, PLANE_MODE_TAKEOFF)

    print(f"[PATTERN] Uçak havalanıyor... Lütfen {climb_duration} saniye bekleyin.")
    time.sleep(climb_duration)

    # FBWA (5) — düz uçuş, RC override ile kontrol bizde
    print(f"[PATTERN] Kalkış bitti, FBWA moda geçiliyor")
    set_mode(conn, PLANE_MODE_FBWA)
    fly_forward(throttle=THROTTLE_CRUISE, duration=stabilize_duration)

    print_status()
    print("[PATTERN] Takeoff + Stabilize tamamlandı")


# ---------------------------------------------------------------------------
# Geometrik Paternler
# ---------------------------------------------------------------------------

def draw_square(side_duration: float = 3.0,
                turn_duration: float = 2.0,
                throttle: int = THROTTLE_CRUISE,
                turn_intensity: int = 500):
    """
    Kare deseni çizer.

    Fixed-wing GPS'siz ortamda kesin mesafe kontrolü zor olduğundan
    zaman bazlı kenarlar kullanılır.

    Args:
        side_duration: Her kenarın uçuş süresi (saniye)
        turn_duration: Her dönüşün süresi (saniye)
        throttle: Uçuş throttle değeri
        turn_intensity: Dönüş şiddeti (yaw)
    """
    print(f"[PATTERN] Kare: side={side_duration}s turn={turn_duration}s")

    for i in range(4):
        print(f"[PATTERN] Kenar {i+1}/4")
        fly_forward(throttle=throttle, duration=side_duration)
        print(f"[PATTERN] Dönüş {i+1}/4")
        turn_right(intensity=turn_intensity, throttle=throttle,
                   duration=turn_duration)

    print("[PATTERN] Kare tamamlandı")


def draw_rectangle(long_side: float = 4.0,
                   short_side: float = 2.0,
                   turn_duration: float = 2.0,
                   throttle: int = THROTTLE_CRUISE,
                   turn_intensity: int = 500):
    """
    Dikdörtgen deseni çizer.

    Args:
        long_side: Uzun kenar süresi (saniye)
        short_side: Kısa kenar süresi (saniye)
    """
    print(f"[PATTERN] Dikdörtgen: long={long_side}s short={short_side}s")

    for i in range(2):
        print(f"[PATTERN] Uzun kenar {i*2+1}")
        fly_forward(throttle=throttle, duration=long_side)
        turn_right(intensity=turn_intensity, throttle=throttle,
                   duration=turn_duration)

        print(f"[PATTERN] Kısa kenar {i*2+2}")
        fly_forward(throttle=throttle, duration=short_side)
        turn_right(intensity=turn_intensity, throttle=throttle,
                   duration=turn_duration)

    print("[PATTERN] Dikdörtgen tamamlandı")


def circle(duration: float = 15.0,
           turn_intensity: int = 300,
           throttle: int = THROTTLE_CRUISE):
    """
    Çember deseni — sürekli sabit yaw ile daire çizer.

    Args:
        duration: Toplam daire süresi (saniye)
        turn_intensity: Dönüş yoğunluğu
    """
    print(f"[PATTERN] Çember: duration={duration}s intensity={turn_intensity}")
    t0 = time.time()
    while time.time() - t0 < duration:
        send_manual_control(yaw=turn_intensity, throttle=throttle)
        time.sleep(CONTROL_RATE)
    print("[PATTERN] Çember tamamlandı")


def zigzag(segments: int = 4,
           segment_duration: float = 2.0,
           turn_duration: float = 1.5,
           throttle: int = THROTTLE_CRUISE,
           turn_intensity: int = 400):
    """
    Zigzag deseni — sola-sağa zigzag.

    Args:
        segments: Toplam segment sayısı
        segment_duration: Her düz uçuş süresi
        turn_duration: Her dönüş süresi
    """
    print(f"[PATTERN] Zigzag: {segments} segments")

    for i in range(segments):
        # Düz uçuş
        fly_forward(throttle=throttle, duration=segment_duration)

        # Sola veya sağa dön
        if i % 2 == 0:
            print(f"[PATTERN] Zigzag segment {i+1}: sola")
            turn_left(intensity=turn_intensity, throttle=throttle,
                      duration=turn_duration)
        else:
            print(f"[PATTERN] Zigzag segment {i+1}: sağa")
            turn_right(intensity=turn_intensity, throttle=throttle,
                       duration=turn_duration)

    print("[PATTERN] Zigzag tamamlandı")


# ---------------------------------------------------------------------------
# Agresif Manevralar
# ---------------------------------------------------------------------------

def aggressive_maneuver_1():
    """
    Agresif Manevra 1: Hızlı tırmanış + dik dalış + toparlanma.
    """
    print("[PATTERN] Agresif Manevra 1: Tırmanış-Dalış-Toparlanma")

    # Hızlı tırmanış
    climb(pitch_intensity=700, throttle=THROTTLE_FULL, duration=3.0)

    # Dik dalış
    descend(pitch_intensity=-600, throttle=THROTTLE_IDLE, duration=2.0)

    # Toparlanma
    climb(pitch_intensity=400, throttle=THROTTLE_FULL, duration=2.0)

    # Stabilize
    fly_forward(throttle=THROTTLE_CRUISE, duration=2.0)

    print("[PATTERN] Agresif Manevra 1 tamamlandı")
    print_status()


def aggressive_maneuver_2():
    """
    Agresif Manevra 2: Keskin S-dönüşü.
    """
    print("[PATTERN] Agresif Manevra 2: Keskin S-dönüşü")

    # Sert sola
    t0 = time.time()
    while time.time() - t0 < 2.0:
        send_manual_control(roll=-700, yaw=-600, throttle=THROTTLE_FULL)
        time.sleep(CONTROL_RATE)

    # Düzelt
    fly_forward(throttle=THROTTLE_CRUISE, duration=1.0)

    # Sert sağa
    t0 = time.time()
    while time.time() - t0 < 2.0:
        send_manual_control(roll=700, yaw=600, throttle=THROTTLE_FULL)
        time.sleep(CONTROL_RATE)

    # Stabilize
    fly_forward(throttle=THROTTLE_CRUISE, duration=2.0)

    print("[PATTERN] Agresif Manevra 2 tamamlandı")
    print_status()


def aggressive_maneuver_3():
    """
    Agresif Manevra 3: Spiral tırmanış.
    """
    print("[PATTERN] Agresif Manevra 3: Spiral tırmanış")

    # Sürekli sağa dönüş + tırmanış
    t0 = time.time()
    while time.time() - t0 < 8.0:
        send_manual_control(
            pitch=400,
            roll=400,
            yaw=400,
            throttle=THROTTLE_FULL,
        )
        time.sleep(CONTROL_RATE)

    # Stabilize
    fly_forward(throttle=THROTTLE_CRUISE, duration=3.0)

    print("[PATTERN] Agresif Manevra 3 tamamlandı")
    print_status()


# ---------------------------------------------------------------------------
# Scripted Demo Paternler
# ---------------------------------------------------------------------------

def demo_basic():
    """
    Temel demo: takeoff → kare → loiter.
    """
    print("=" * 50)
    print("[DEMO] Basic Demo başlıyor")
    print("=" * 50)

    takeoff_then_stabilize()
    draw_square()
    loiter(duration=3.0)

    print("[DEMO] Basic Demo tamamlandı")


def demo_aggressive():
    """
    Agresif demo: takeoff → tüm agresif manevralar.
    """
    print("=" * 50)
    print("[DEMO] Aggressive Demo başlıyor")
    print("=" * 50)

    takeoff_then_stabilize()
    aggressive_maneuver_1()
    aggressive_maneuver_2()
    aggressive_maneuver_3()
    loiter(duration=3.0)

    print("[DEMO] Aggressive Demo tamamlandı")


def demo_mixed():
    """
    Karma demo: takeoff → zigzag → çember → kare → agresif.
    """
    print("=" * 50)
    print("[DEMO] Mixed Demo başlıyor")
    print("=" * 50)

    takeoff_then_stabilize()
    zigzag(segments=4)
    circle(duration=10.0)
    draw_square()
    aggressive_maneuver_1()
    loiter(duration=3.0)

    print("[DEMO] Mixed Demo tamamlandı")
