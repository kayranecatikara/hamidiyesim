#!/usr/bin/env python3
"""
run_plane_scenario.py — Hedef İHA (sabit kanat) uçuş senaryoları.

Kullanım:
    python -m control.run_plane_scenario square      # kare çiz
    python -m control.run_plane_scenario circle      # daire çiz
    python -m control.run_plane_scenario aggressive  # rastgele agresif manevralar

Akış: bağlan → force ARM → TAKEOFF modu ile otonom kalkış → FBWA + RC
override ile seçilen desen. Desen, GCS süreci öldürene (manuel moda geçiş
veya durdur butonu) kadar süresiz döner.

Kare dönüşleri PUSULA (ATTITUDE yaw) tabanlıdır: FBWA'da roll komutu verilir,
heading 90° değişince kenara geçilir. Eski run_plane_square zaman bazlı
rudder(yaw) dönüşü kullanıyordu — FBWA'da rudder tek başına dönüş üretmediği
için kare bozuktu.

Throttle GCS'teki slider'dan okunur (http://127.0.0.1:8000/api/plane_throttle);
agresif manevralar kendi throttle'ını kullanır.
"""

import json
import math
import os
import random
import signal
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil

from control.plane_functions import (
    connect_plane,
    arm_plane,
    get_conn,
    start_gcs_keepalive,
    stop_gcs_keepalive,
    THROTTLE_CRUISE,
    THROTTLE_FULL,
)
from control.mav_common import (
    set_mode,
    PLANE_MODE_TAKEOFF,
    PLANE_MODE_FBWA,
)

# Havada devralma eşiği: bu irtifanın üstünde armlıysak kalkış ATLANIR.
AIRBORNE_ALT_M = 15.0

CONTROL_RATE = 0.05   # 20 Hz komut döngüsü

_abort = False

# _pump ile güncellenen son telemetri
_att = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "ok": False}
_pos = {"z": 0.0}


def _sig_handler(_sig, _frame):
    global _abort
    _abort = True


def _pump(conn):
    """Bekleyen MAVLink mesajlarını tüket; ATTITUDE ve LOCAL_POSITION_NED sakla.

    plane_functions.send_manual_control her çağrıda drain_messages ile HER ŞEYİ
    çöpe atıyordu — heading tabanlı dönüş için attitude'u burada yakalıyoruz.
    Tamponu boşaltmak ayrıca telemetrinin bayatlamasını da önler.
    """
    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            return
        t = msg.get_type()
        if t == "ATTITUDE":
            _att.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, ok=True)
        elif t == "LOCAL_POSITION_NED":
            _pos["z"] = msg.z


def _rc(conn, roll=0, pitch=0, throttle=0, yaw=0):
    """RC override gönder — plane_functions.send_manual_control ile aynı eşleme.

    roll/pitch/yaw: -1000..+1000 (pozitif = sağa yatış / burun yukarı / sağa),
    throttle: 0..1000.
    """
    conn.mav.rc_channels_override_send(
        conn.target_system,
        conn.target_component,
        int(1500 + roll / 2),       # CH1: Aileron
        int(1500 + pitch / 2),      # CH2: Elevator (YÜKSEK PWM = burun yukarı,
                                    #      canlı SITL'de doğrulandı)
        int(1000 + throttle),       # CH3: Throttle
        int(1500 + yaw / 2),        # CH4: Rudder
        0, 0, 0, 0,
    )


_thr_cache = {"val": THROTTLE_CRUISE, "t": 0.0}


def gcs_throttle():
    """GCS slider'ından throttle oku (0.5s önbellekli; GCS yoksa cruise)."""
    now = time.time()
    if now - _thr_cache["t"] > 0.5:
        _thr_cache["t"] = now
        try:
            req = urllib.request.urlopen(
                "http://127.0.0.1:8000/api/plane_throttle", timeout=0.2)
            _thr_cache["val"] = json.loads(req.read().decode()).get(
                "throttle", THROTTLE_CRUISE)
        except Exception:
            pass
    return _thr_cache["val"]


def hold(conn, duration, roll=0, pitch=0, throttle=None, yaw=0):
    """duration boyunca sabit komut uygula (throttle=None → GCS slider)."""
    t0 = time.time()
    while not _abort and time.time() - t0 < duration:
        _pump(conn)
        thr = gcs_throttle() if throttle is None else throttle
        _rc(conn, roll=roll, pitch=pitch, throttle=thr, yaw=yaw)
        time.sleep(CONTROL_RATE)


def _angdiff(a, b):
    """a-b farkını [-pi, pi] aralığına sar."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def turn_by(conn, deg, bank=650, timeout=20.0):
    """Heading tabanlı dönüş: hedef yaw'a ulaşana dek FBWA roll komutu.

    Dönüşte hafif up-elevator irtifa kaybını azaltır. 10° toleransta bırakılır
    (FBWA kanatları düzeltirken kalan momentum farkı kapatır).
    """
    _pump(conn)
    if not _att["ok"]:
        hold(conn, 1.0)
        _pump(conn)
    target = _att["yaw"] + math.radians(deg)
    roll_cmd = bank if deg > 0 else -bank
    t0 = time.time()
    while not _abort and time.time() - t0 < timeout:
        _pump(conn)
        if _att["ok"] and abs(_angdiff(target, _att["yaw"])) < math.radians(10):
            break
        _rc(conn, roll=roll_cmd, pitch=180, throttle=gcs_throttle())
        time.sleep(CONTROL_RATE)


def _read_vehicle_state(conn, wait=1.5):
    """Kısa süre telemetri toplayıp (armed, irtifa_m) döndürür.

    Senaryo geçişinde kritik: önceki senaryo öldürülüp yenisi başlarken araç
    HAVADA. Eski akış havadaki uçağa yerden kalkış prosedürü uyguluyordu
    (warmup + GPS bekleme sırasında RC failsafe → arm_plane'in MANUAL moda
    alması → gaz trim'e düşüp dalış → havada TAKEOFF) ve araç yere çakılıyordu.
    """
    armed = False
    t0 = time.time()
    while time.time() - t0 < wait:
        msg = conn.recv_match(
            type=["HEARTBEAT", "LOCAL_POSITION_NED", "ATTITUDE"],
            blocking=True, timeout=0.3)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "HEARTBEAT" and msg.get_srcSystem() == conn.target_system:
            armed = bool(msg.base_mode
                         & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t == "LOCAL_POSITION_NED":
            _pos["z"] = msg.z
        elif t == "ATTITUDE":
            _att.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, ok=True)
    return armed, -_pos["z"]


def takeoff(conn, climb_time=8.0):
    """Otonom kalkış: TAKEOFF modu motoru açıp TKOFF_ALT'a tırmandırır,
    ardından FBWA'ya geçilip kısa düz uçuşla stabilize edilir."""
    print("[SCN] Otonom kalkış (TAKEOFF modu)...")
    set_mode(conn, PLANE_MODE_TAKEOFF)
    t0 = time.time()
    while not _abort and time.time() - t0 < climb_time:
        _pump(conn)
        time.sleep(0.2)
    print(f"[SCN] Kalkış bitti (irtifa ~{-_pos['z']:.0f}m) → FBWA")
    set_mode(conn, PLANE_MODE_FBWA)
    hold(conn, 2.0)


# ---------------------------------------------------------------------------
# Senaryolar — hepsi süresiz döner, GCS süreci öldürünce biter
# ---------------------------------------------------------------------------

def scenario_square(conn):
    side = 5.0
    print(f"[SCN] KARE — kenar {side}s, 90° pusula dönüşleri")
    i = 0
    while not _abort:
        print(f"[SCN] Kenar {i % 4 + 1}/4")
        hold(conn, side)
        if _abort:
            break
        print(f"[SCN] Dönüş {i % 4 + 1}/4 (heading +90°)")
        turn_by(conn, 90)
        i += 1


def scenario_circle(conn):
    # roll=500 → ~22° yatış: ~18 m/s'de ~80m yarıçaplı daire
    print("[SCN] DAİRE — sabit yatışla süresiz tur")
    while not _abort:
        hold(conn, 0.5, roll=500, pitch=150)


def scenario_aggressive(conn):
    print("[SCN] AGRESİF — rastgele manevralar")
    maneuvers = ["climb", "dive", "bank_l", "bank_r", "s_turn", "spiral"]
    while not _abort:
        m = random.choice(maneuvers)
        if m == "climb":
            print("[SCN] Sert tırmanış")
            hold(conn, random.uniform(1.5, 3.0),
                 pitch=random.randint(500, 800), throttle=THROTTLE_FULL)
        elif m == "dive":
            # irtifa emniyeti: 40m altındaysa dalma, yerine tırman
            if -_pos["z"] > 40.0:
                print("[SCN] Dalış")
                hold(conn, random.uniform(1.0, 2.0),
                     pitch=-random.randint(350, 600), throttle=200)
            else:
                print("[SCN] İrtifa düşük — dalış yerine tırmanış")
                hold(conn, 2.0, pitch=500, throttle=THROTTLE_FULL)
        elif m in ("bank_l", "bank_r"):
            s = -1 if m == "bank_l" else 1
            print("[SCN] Sert yatışlı dönüş" + (" (sol)" if s < 0 else " (sağ)"))
            hold(conn, random.uniform(1.5, 3.0),
                 roll=s * random.randint(600, 900), pitch=200,
                 throttle=THROTTLE_FULL)
        elif m == "s_turn":
            print("[SCN] Keskin S-dönüşü")
            hold(conn, 1.5, roll=-750, pitch=200, throttle=THROTTLE_FULL)
            hold(conn, 1.5, roll=750, pitch=200, throttle=THROTTLE_FULL)
        elif m == "spiral":
            print("[SCN] Spiral tırmanış")
            hold(conn, random.uniform(3.0, 5.0),
                 roll=450, pitch=450, throttle=THROTTLE_FULL)
        # toparlanma: kısa düz uçuş
        hold(conn, random.uniform(1.0, 2.0))


SCENARIOS = {
    "square": scenario_square,
    "circle": scenario_circle,
    "aggressive": scenario_aggressive,
}


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "square"
    if name not in SCENARIOS:
        print(f"[SCN] Bilinmeyen senaryo: {name} — seçenekler: {list(SCENARIOS)}")
        sys.exit(2)

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    print("=" * 50)
    print(f"[SCN] Uçuş senaryosu: {name.upper()}")
    print("=" * 50)

    connect_plane()
    conn = get_conn()
    start_gcs_keepalive()

    armed, alt = _read_vehicle_state(conn)
    if armed and alt > AIRBORNE_ALT_M:
        # HAVADA DEVRALMA — önceki senaryodan/manuelden geçiş. Kalkış YOK;
        # önceki RC override 3 sn içinde düşmeden FBWA + desen devralır.
        print(f"[SCN] Araç zaten havada (irtifa {alt:.0f}m, armlı) — "
              "kalkış atlanıyor, doğrudan FBWA + desen")
        _rc(conn, throttle=gcs_throttle())        # override akışı hemen başlasın
        set_mode(conn, PLANE_MODE_FBWA, confirm_timeout=0)
        hold(conn, 1.0)                           # düz uçuşla kısa stabilizasyon
    elif armed:
        print(f"[SCN] Armlı ama yerde (irtifa {alt:.0f}m) — doğrudan kalkış")
        takeoff(conn)
    else:
        result = arm_plane(warmup_duration=3.0)
        if result is None or result[1] != 0:
            print("[SCN] ARM başarısız!")
            return
        takeoff(conn)

    SCENARIOS[name](conn)

    # Durduruldu → nötr yüzey + cruise gazla bırak (manuel mod hemen devralır)
    _rc(conn, throttle=THROTTLE_CRUISE)
    stop_gcs_keepalive()
    print("[SCN] Senaryo sonlandı.")


if __name__ == "__main__":
    main()
