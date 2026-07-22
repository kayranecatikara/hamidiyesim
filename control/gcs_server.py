import sys
import os
import random
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import asyncio
import math
import signal
import subprocess
import threading
import time
import cv2
import webbrowser
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse, StreamingResponse
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pymavlink import mavutil
import uvicorn

# Cessna renk tabanlı tespit
from vision.detection_state import set_detection, get_detection
# YOLO detector (vision/detector.py) opsiyonel — startup'ta yüklenir (_yolo_detector).

# Kare scriptinin de kullandığı, kanıtlanmış çalışan modüller (ArduPilot)
from control.mav_common import (
    connect_mavlink,
    GCSKeepalive,
    set_mode,
    wait_ack,
    PLANE_MODE_MANUAL,
    PLANE_MODE_FBWA,
)

app = FastAPI(title="Avcı GCS")

ui_path = os.path.join(os.path.dirname(__file__), "gcs_ui")
if not os.path.exists(ui_path):
    os.makedirs(ui_path)
app.mount("/ui", StaticFiles(directory=ui_path, html=True), name="ui")

@app.get("/")
def read_root():
    return RedirectResponse(url="/ui/index.html")

# -----------------------------------------------------------------------
# GLOBAL STATE
# -----------------------------------------------------------------------
telemetry_state = {
    "iris":  {"x": 0, "y": 0, "z": 0, "vx": 0, "vy": 0, "vz": 0, "speed": 0, "roll": 0, "pitch": 0, "yaw": 0, "armed": False,
              "lat": 0.0, "lon": 0.0, "alt_amsl": 0.0},
    "plane": {"x": 0, "y": 0, "z": 0, "vx": 0, "vy": 0, "vz": 0, "speed": 0, "roll": 0, "pitch": 0, "yaw": 0, "armed": False,
              "lat": 0.0, "lon": 0.0, "alt_amsl": 0.0},
}

# ── NED ÇERÇEVE OFSETİ (iris ↔ plane) ──────────────────────────────────
# İki SITL'in LOCAL_POSITION_NED orijinleri AYNI DEĞİL: ArduPilotPlugin dünya-
# çerçeveli pozisyon gönderir, her aracın EKF orijini KENDİ spawn noktasında
# kurulur (iris 0,0 — talon 12,0 → world dosyası). Plane local'ini olduğu gibi
# iris local'iyle karşılaştırmak ~12m sabit hata veriyordu (drone hedefin
# YANINDAN takip ediyordu, kamera hedefi bulamıyordu). Düzeltme: iki aracın
# GLOBAL_POSITION_INT (GPS) verisinden sabit ofset kendinden-kalibre edilir,
# plane local'i iris çerçevesine taşınır.
_frame_off = {"n": 0.0, "e": 0.0, "d": 0.0, "samples": 0, "ok": False}
_plane_local_raw = {"x": 0.0, "y": 0.0, "z": 0.0}
_M_PER_DEG = 111319.4907          # metre / derece (enlem)


def _frame_off_update():
    """Plane GLOBAL geldiğinde çağrılır: GPS'ten plane'in iris-çerçevesindeki
    konumu kurulur, plane LOCAL ham değeriyle farkı (EKF orijin ofseti) EMA'lanır.
    Ofset sabittir (orijinler hareket etmez); EMA yalnız GPS gürültüsünü süzer."""
    ip = telemetry_state["iris"]
    pp = telemetry_state["plane"]
    if ip["lat"] == 0.0 or pp["lat"] == 0.0:
        return                                    # iki GPS de gelmeden kalibre etme
    rel_n = (pp["lat"] - ip["lat"]) * _M_PER_DEG
    rel_e = (pp["lon"] - ip["lon"]) * _M_PER_DEG * math.cos(math.radians(ip["lat"]))
    rel_d = -(pp["alt_amsl"] - ip["alt_amsl"])
    sn = (ip["x"] + rel_n) - _plane_local_raw["x"]
    se = (ip["y"] + rel_e) - _plane_local_raw["y"]
    sd = (ip["z"] + rel_d) - _plane_local_raw["z"]
    if _frame_off["samples"] == 0:
        _frame_off.update(n=sn, e=se, d=sd)
    else:
        a = 0.1
        _frame_off["n"] = (1 - a) * _frame_off["n"] + a * sn
        _frame_off["e"] = (1 - a) * _frame_off["e"] + a * se
        _frame_off["d"] = (1 - a) * _frame_off["d"] + a * sd
    _frame_off["samples"] += 1
    if not _frame_off["ok"] and _frame_off["samples"] >= 20:
        _frame_off["ok"] = True
        print(f"[FRAME] Plane→iris NED çerçeve ofseti kalibre edildi: "
              f"N={_frame_off['n']:+.1f}m E={_frame_off['e']:+.1f}m "
              f"D={_frame_off['d']:+.1f}m (EKF orijinleri spawn farkı)")

# GPS karıştırma simülasyonu — chase thread BU veriyi okur
_gps_noise_level = 0.0   # 0.0 = temiz, 1.0 = tamamen bozuk
_noisy_plane_telem = {"x": 0, "y": 0, "z": 0, "yaw": 0, "frozen": False}
_last_clean_plane = {"x": 0, "y": 0, "z": 0, "yaw": 0}  # freeze için son temiz veri

# Telemetri bağlantısı (sadece okuma)
_mav_conn = None
_plane_sysid = None
_plane_compid = 0

# Uçak görev thread'leri
_square_active = False
_square_stop_event = threading.Event()
_square_thread_obj = None

# Uçak throttle seviyesi — slider ile ayarlanır (0-1000 aralığı, MANUAL_CONTROL)
_plane_throttle = 600   # default = THROTTLE_CRUISE

# Video parazit simülasyonu — iris kamera akışına uygulanır
_video_noise_level = 0.0   # 0.0 = temiz, 1.0 = tamamen parazitli

# Manuel mod durumu
_manual_active = False
_manual_aileron  = 1500
_manual_elevator = 1500
_manual_throttle = 1000

def id_to_name(sysid):
    # ArduPilot SITL: iris/copter sysid=5, plane sysid=2
    if sysid in (1, 5):      # iris (ArduCopter)
        return "iris"
    elif sysid in (2, 3):    # plane (ArduPlane)
        return "plane"
    return None

@app.post("/api/command/plane/square")
def command_plane_square():
    global _square_active, _square_thread_obj
    try:
        # Eğer manuel kontrol aktifse durdur
        global _manual_active
        if _manual_active:
            _manual_active = False
            time.sleep(0.3)

        if _square_active:
            _square_stop_event.set()
            time.sleep(0.5)

        _square_stop_event.clear()
        _square_active = True
        _square_thread_obj = threading.Thread(target=_square_thread, daemon=True)
        _square_thread_obj.start()
        return {"status": "success", "message": "Kare çizme başlatıldı."}
    except Exception as e:
        return {"status": "error", "message": str(e)}

def _square_thread():
    global _square_active
    print("[SQUARE] Thread başlıyor...")
    try:
        import subprocess as _sp
        import os as _os
        project_root = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
        proc = _sp.Popen(
            ["python3", "-m", "control.run_plane_square"],
            cwd=project_root,
            start_new_session=True
        )
        # Stop event gelene kadar bekle, ya da process bitene kadar
        while not _square_stop_event.is_set():
            ret = proc.poll()
            if ret is not None:
                print(f"[SQUARE] Script tamamlandı (exit={ret})")
                break
            import time as _t
            _t.sleep(0.5)
        # Durdurma sinyali geldiyse process'i öldür
        if proc.poll() is None:
            try:
                _os.killpg(_os.getpgid(proc.pid), __import__('signal').SIGKILL)
            except Exception:
                proc.kill()
            proc.wait()
            print("[SQUARE] Script durduruldu.")
    except Exception as e:
        import traceback
        print(f"[SQUARE] HATA: {e}")
        traceback.print_exc()
    finally:
        _square_active = False

@app.post("/api/command/plane/circle")
def command_plane_circle():
    return {"status": "error", "message": "Daire scripti henüz hazır değil!"}

# -----------------------------------------------------------------------
# MANUEL KONTROL
# -----------------------------------------------------------------------
class ManualCmd(BaseModel):
    aileron:  int
    elevator: int
    throttle: int

@app.post("/api/command/plane/start_manual")
def start_manual_mode():
    global _manual_active, _square_active
    global _manual_aileron, _manual_elevator, _manual_throttle

    # =========================================================
    # ADIM 1: Kare scriptini durdur (Eğer çalışıyorsa)
    # =========================================================
    if _square_active:
        print("[GCS] Kare uçuşu durduruluyor...")
        _square_stop_event.set()
        time.sleep(0.5)

    subprocess.run(['pkill', '-9', '-f', 'run_plane_square'], capture_output=True)

    # =========================================================
    # ADIM 2: Manuel kontrol thread'ini başlat
    # =========================================================
    _manual_active = True
    _manual_aileron = 1500
    _manual_elevator = 1500
    _manual_throttle = 1000

    t = threading.Thread(target=_manual_control_thread, daemon=True)
    t.start()
    print("[GCS] Manuel kontrol thread'i başlatıldı.")
    return {"status": "success", "message": "Manuel mod aktif"}


def _manual_control_thread():
    """
    AYNI ALT YAPIYI kullanarak plane'e bağlanır:
    - connect_mavlink (source_system=251, aynı kare script gibi)
    - GCSKeepalive (10 Hz heartbeat)
    - set_mode (COMMAND_LONG ile MAV_CMD_DO_SET_MODE)
    - rc_channels_override_send
    """
    global _manual_active

    print("[MANUAL] Thread başlıyor...")
    try:
        # =====================================================
        # BAĞLANTI — Artık global _mav_conn (14551) kullanıyoruz
        # =====================================================
        if _mav_conn is None:
            raise RuntimeError("Global MAVLink bağlantısı yok!")
            
        conn = _mav_conn
        if _plane_sysid is not None:
            conn.target_system = _plane_sysid
            
        print(f"[MANUAL] Bağlantı kullanılıyor: target_sys={conn.target_system}")

        # =====================================================
        # KEEPALIVE — arming korunması için şart
        # =====================================================
        keepalive = GCSKeepalive(conn, interval=0.1)
        keepalive.start()
        time.sleep(0.5)

        # =====================================================
        # MOD DEĞİŞTİR — ArduPlane MANUAL (0): RC override doğrudan servolara
        # işler. Kare scripti (plane_functions.arm_plane) de aynı modu kullanır.
        # =====================================================
        print("[MANUAL] MANUAL moda geçiliyor...")
        result = set_mode(conn, PLANE_MODE_MANUAL)
        if result and result[1] == 0:
            print("[MANUAL] ✓ ArduPlane MANUAL modu kabul etti (ACK result=0)")
        else:
            print(f"[MANUAL] ⚠ Mode ACK: {result} — yine de devam ediliyor")
            # İkinci deneme
            time.sleep(0.3)
            result2 = set_mode(conn, PLANE_MODE_MANUAL)
            print(f"[MANUAL] İkinci deneme ACK: {result2}")

        # =====================================================
        # RC OVERRIDE DÖNGÜSÜ — 10 Hz
        # =====================================================
        print("[MANUAL] RC Override döngüsü başlıyor (10 Hz)...")
        while _manual_active:
            conn.mav.rc_channels_override_send(
                conn.target_system,
                conn.target_component,
                _manual_aileron,    # CH1: Roll/Aileron
                _manual_elevator,   # CH2: Pitch/Elevator
                _manual_throttle,   # CH3: Throttle
                1500,               # CH4: Yaw nötr
                0, 0, 0, 0
            )
            time.sleep(0.1)

        # Kapanış — throttle sıfırla
        conn.mav.rc_channels_override_send(
            conn.target_system, conn.target_component,
            1500, 1500, 1000, 1500, 0, 0, 0, 0
        )
        keepalive.stop()
        print("[MANUAL] Kapatıldı.")

    except Exception as e:
        import traceback
        print(f"[MANUAL] HATA: {e}")
        traceback.print_exc()
        _manual_active = False


@app.post("/api/command/plane/manual")
def command_plane_manual(cmd: ManualCmd):
    """Joystick değerlerini thread'e iletir."""
    global _manual_aileron, _manual_elevator, _manual_throttle
    if not _manual_active:
        return {"status": "skip"}
    _manual_aileron  = cmd.aileron
    _manual_elevator = cmd.elevator
    _manual_throttle = cmd.throttle
    return {"status": "success"}


@app.post("/api/command/plane/stop_manual")
def stop_manual_mode():
    global _manual_active
    _manual_active = False
    print("[GCS] Manuel mod kapatıldı.")
    return {"status": "success"}

# -----------------------------------------------------------------------
# ARKA TAKİP MODU (CHASE MODE) — Iris → Plane takibi
# -----------------------------------------------------------------------
# drone_functions modülü: ArduCopter GUIDED modda pozisyon setpoint gönderir
from control.drone_functions import (
    connect_drone as df_connect_drone,
    takeoff_to_z as df_takeoff,
    set_guided_mode as df_guided,
    hover as df_hover,
    _send_position_setpoint,
    get_conn as df_get_conn,
    SETPOINT_RATE,
)
from control.mav_common import (
    COPTER_MODE_GUIDED,
    arm as mav_arm,
    set_mode as mav_set_mode,
    timestamp_ms,
)

_chase_active = False
FOLLOW_DIST = 5.0      # metre — hedefin arkasından takip mesafesi
CHASE_ALT_OFFSET = 0.0 # metre — hedefle aynı irtifa (NED, negatif=yukarı)

# -----------------------------------------------------------------------
# GPS KARIŞTIRMA SİMÜLASYONU
# -----------------------------------------------------------------------
class GpsNoiseCmd(BaseModel):
    level: float  # 0.0 — 1.0

@app.post("/api/gps_noise")
def set_gps_noise(cmd: GpsNoiseCmd):
    """GPS karıştırma seviyesini ayarlar (0.0=temiz, 1.0=tam karıştırma)."""
    global _gps_noise_level
    _gps_noise_level = max(0.0, min(1.0, cmd.level))
    pct = int(_gps_noise_level * 100)
    print(f"[GPS-JAM] Karıştırma seviyesi: %{pct}")
    return {"status": "success", "level": _gps_noise_level}

@app.get("/api/gps_noise")
def get_gps_noise():
    return {"level": _gps_noise_level}

def _apply_gps_noise(clean_x, clean_y, clean_z, clean_yaw):
    """
    GPS karıştırma modeli:
    - 0-30%:   Hafif gürültü (±2m), veri gelir
    - 30-70%:  Orta gürültü (±10m) + %30 freeze olasılığı
    - 70-99%:  Şiddetli gürültü (±20m) + %70 freeze + büyük atlamalar
    - 100%:    Veri tamamen donmuş (son bilinen konum)
    """
    global _noisy_plane_telem, _last_clean_plane
    lvl = _gps_noise_level

    # Son temiz veriyi sakla (freeze için)
    _last_clean_plane = {"x": clean_x, "y": clean_y, "z": clean_z, "yaw": clean_yaw}

    if lvl <= 0.001:
        # Karıştırma yok
        _noisy_plane_telem = {"x": clean_x, "y": clean_y, "z": clean_z, "yaw": clean_yaw, "frozen": False}
        return

    if lvl >= 0.999:
        # %100 → veri donmuş, güncellenmez
        _noisy_plane_telem["frozen"] = True
        return

    # Freeze olasılığı (quadratic: %50 seviyede %25 freeze)
    freeze_prob = lvl * lvl
    if random.random() < freeze_prob:
        # Bu tick'te veri donuk — güncellenmez
        _noisy_plane_telem["frozen"] = True
        return

    # Gürültü standart sapması: seviye × 20 metre
    noise_std = lvl * 20.0
    nx = clean_x + random.gauss(0, noise_std)
    ny = clean_y + random.gauss(0, noise_std)
    nz = clean_z + random.gauss(0, noise_std * 0.3)  # irtifada daha az gürültü
    nyaw = clean_yaw + random.gauss(0, lvl * 30)      # yaw'da da gürültü (derece)

    # %70+ seviyede büyük atlamalar (spoofing)
    if lvl > 0.7 and random.random() < 0.15:
        jump = 30.0 * lvl
        nx += random.uniform(-jump, jump)
        ny += random.uniform(-jump, jump)

    _noisy_plane_telem = {"x": round(nx,2), "y": round(ny,2), "z": round(nz,2), "yaw": round(nyaw,1), "frozen": False}

@app.post("/api/video_noise")
def set_video_noise(cmd: GpsNoiseCmd):  # GpsNoiseCmd: level:float, aynı model kullan
    """Iris kamera parazit seviyesini ayarlar (0.0=temiz, 1.0=tam parazit)."""
    global _video_noise_level
    _video_noise_level = max(0.0, min(1.0, cmd.level))
    print(f"[VIDEO-NOISE] Parazit seviyesi: %{int(_video_noise_level*100)}")
    return {"status": "success", "level": _video_noise_level}

@app.get("/api/video_noise")
def get_video_noise():
    return {"level": _video_noise_level}

# -----------------------------------------------------------------------
# UÇAK THROTTLE AYARI
# -----------------------------------------------------------------------
class ThrottleCmd(BaseModel):
    throttle: int  # 0-1000

@app.post("/api/plane_throttle")
def set_plane_throttle(cmd: ThrottleCmd):
    global _plane_throttle
    _plane_throttle = max(0, min(1000, cmd.throttle))
    print(f"[GCS] Uçak throttle: {_plane_throttle}")
    return {"status": "success", "throttle": _plane_throttle}

@app.get("/api/plane_throttle")
def get_plane_throttle():
    return {"throttle": _plane_throttle}

# -----------------------------------------------------------------------
# DEBUG ENDPOINT — telemetri sorunlarını teşhis için
# -----------------------------------------------------------------------
_mavlink_stats = {"total": 0, "by_sysid": {}, "by_type": {}}

@app.get("/api/debug/telem")
def debug_telem():
    """Anlık telemetry state + MAVLink istatistikleri."""
    return {
        "telemetry_state": telemetry_state,
        "mavlink_stats": _mavlink_stats,
        "plane_sysid": _plane_sysid,
    }

@app.post("/api/command/iris/start_chase")
def start_chase():
    """
    Iris drone'u kaldırır ve plane'in arkasından takip etmeye başlar.
    Tüm kontrol drone_functions (OFFBOARD + position setpoint) üzerinden.
    """
    global _chase_active
    if _chase_active:
        return {"status": "error", "message": "Chase zaten aktif!"}

    _chase_active = True
    t = threading.Thread(target=_chase_thread, daemon=True)
    t.start()
    return {"status": "success", "message": "Chase modu başlatılıyor..."}


@app.post("/api/command/iris/stop_chase")
def stop_chase():
    global _chase_active
    _chase_active = False
    print("[CHASE] Durduruldu.")
    return {"status": "success"}


@app.get("/api/chase_status")
def chase_status():
    """Frontend için chase durumu."""
    if not _chase_active:
        return {"active": False, "distance": 0}
    plane = telemetry_state["plane"]
    iris  = telemetry_state["iris"]
    dist = math.sqrt(
        (plane["x"] - iris["x"])**2 +
        (plane["y"] - iris["y"])**2 +
        (plane["z"] - iris["z"])**2
    )
    resp = {"active": True, "distance": round(dist, 1)}
    if _GPS_LAW != "v2":
        # GPS-YAKLASMA yasasının canlı durumu (ARAMA/KILIT/DROPOUT + handoff)
        resp["guidance"] = dict(_gps_approach_mod.status)
    return resp


# -----------------------------------------------------------------------
# PnP POSE TAHMİNİ — Gerçek veriden türetilmiş simüle PnP çıkışı
# -----------------------------------------------------------------------
_pnp_prev_speed = 0.0
_pnp_prev_time  = 0.0

@app.get("/api/telemetry/pnp")
def pnp_telemetry():
    """
    Pose modeli henüz eğitilmedi.
    Gerçek telemetri verilerine gerçekçi gürültü ekleyerek
    PnP çıkışını simüle eder — rapor fotoğrafları için.
    """
    global _pnp_prev_speed, _pnp_prev_time

    plane = telemetry_state["plane"]
    iris  = telemetry_state["iris"]

    # Plane verisi yoksa (henüz telemetri gelmediyse)
    if plane["x"] == 0 and plane["y"] == 0 and plane["z"] == 0:
        return {"active": False}

    # Gerçek mesafe
    dx = plane["x"] - iris["x"]
    dy = plane["y"] - iris["y"]
    dz = plane["z"] - iris["z"]
    real_dist = math.sqrt(dx*dx + dy*dy + dz*dz)

    # Gerçek hız (plane)
    real_speed = math.sqrt(
        plane.get("vx", 0)**2 +
        plane.get("vy", 0)**2 +
        plane.get("vz", 0)**2
    )

    # Gerçek yaw
    real_yaw = plane.get("yaw", 0.0)

    # İvme tahmini (hız farkından türet)
    now = time.monotonic()
    dt_pnp = now - _pnp_prev_time if _pnp_prev_time > 0 else 1.0
    dt_pnp = max(dt_pnp, 0.1)
    real_accel = abs(real_speed - _pnp_prev_speed) / dt_pnp
    _pnp_prev_speed = real_speed
    _pnp_prev_time  = now

    # ── GERÇEKÇİ GÜRÜLTÜ EKLE (PnP sensor noise) ──
    dist_noise  = random.gauss(0, 0.15 + real_dist * 0.01)
    speed_noise = random.gauss(0, 0.2)
    pos_noise_x = random.gauss(0, 0.3)
    pos_noise_y = random.gauss(0, 0.3)
    pos_noise_z = random.gauss(0, 0.15)
    accel_noise = random.gauss(0, 0.15)
    yaw_noise   = random.gauss(0, 2.0)

    return {
        "active":   True,
        "distance": round(max(0, real_dist + 8.0 + dist_noise + random.uniform(-0.5, 0.5)), 1),
        "speed":    round(max(0, real_speed + speed_noise), 1),
        "x":        round(plane["x"] + pos_noise_x, 1),
        "y":        round(plane["y"] + pos_noise_y, 1),
        "z":        round(plane["z"] + pos_noise_z, 1),
        "accel":    round(max(0, real_accel + accel_noise), 2),
        "yaw":      round(real_yaw + yaw_noise, 1)
    }


from control.guidance.gps_chase import run_chase as _run_chase_algorithm
from control.guidance import gps_approach as _gps_approach_mod
from control.guidance.gps_approach import run_gps_approach as _run_gps_approach
from control.guidance.gps_strike import run_strike as _run_strike_algorithm
from control.guidance.visual_guidance import run_visual_guidance as _run_visual_guidance

# GPS güdüm yasası seçimi: varsayılan eski sistemin portu (gps_approach);
# AVCI_GPS_LAW=v2 → önceki chase v2 (SPRINT/APPROACH/LOCK state machine).
_GPS_LAW = os.environ.get("AVCI_GPS_LAW", "yaklasma").lower()


# ══════════════════════════════════════════════════════════
#  GÖRSEL GÜDÜM (IBVS) — izole hat: YOLO bbox → drone hız.
#  GPS chase'i BOZMAZ; ayrı endpoint. (Faz 4'te supervisor birleştirir.)
# ══════════════════════════════════════════════════════════
_visual_active = False
_visual_stop_event = threading.Event()


def _visual_thread():
    """Görsel güdüm altyapısı: kalkış + IBVS döngüsü (get_detection → hız)."""
    global _visual_active
    print("=" * 50)
    print("[VISUAL] Görsel Güdüm (IBVS) başlıyor")
    print("=" * 50)
    try:
        stop_iris_telem()
        time.sleep(0.3)
        conn = df_connect_drone(port=14541)
        print(f"[VISUAL] Iris bağlantısı: target_sys={conn.target_system}")

        success = df_takeoff(target_z=-5.0)          # drone havada olmalı
        if not success:
            print("[VISUAL] Kalkış başarısız!")
            _visual_active = False
            return
        print("[VISUAL] ✓ Kalkış tamam — IBVS başlatılıyor")

        def get_iris():
            _read_iris_telem_from_conn(conn)          # x,y,z,yaw günceller
            t = telemetry_state["iris"]
            return {"x": t["x"], "y": t["y"], "z": t["z"], "yaw": t["yaw"]}

        _visual_stop_event.clear()
        _run_visual_guidance(conn, get_detection, get_iris, _visual_stop_event)

    except Exception as e:
        import traceback
        print(f"[VISUAL] HATA: {e}")
        traceback.print_exc()
    finally:
        _visual_active = False
        start_iris_telem()


@app.post("/api/command/iris/start_visual")
def start_visual():
    global _visual_active, _chase_active
    if _visual_active:
        return {"status": "error", "message": "Görsel güdüm zaten aktif."}
    _chase_active = False       # aynı porta erişen GPS chase'i durdur
    time.sleep(0.3)
    _visual_active = True
    threading.Thread(target=_visual_thread, daemon=True).start()
    return {"status": "success", "message": "Görsel güdüm (IBVS) başlatıldı."}


@app.post("/api/command/iris/stop_visual")
def stop_visual():
    global _visual_active
    _visual_active = False
    _visual_stop_event.set()
    return {"status": "success", "message": "Görsel güdüm durduruldu."}

_strike_active = False
_strike_stop_event = threading.Event()

@app.post("/api/command/iris/start_strike")
def start_strike():
    """Chase'i durdur, vurma moduna geç — tam gaz hedefe çarp."""
    global _chase_active, _strike_active
    if _strike_active:
        return {"status": "error", "message": "Strike zaten aktif!"}

    # Önce chase'i durdur
    if _chase_active:
        _chase_active = False
        time.sleep(0.3)  # chase thread'in durmasını bekle
        print("[STRIKE] Chase durduruldu — strike'a geçiliyor")

    _strike_active = True
    _strike_stop_event.clear()
    t = threading.Thread(target=_strike_thread, daemon=True)
    t.start()
    return {"status": "success", "message": "⚠️ VURMA MODU AKTİF!"}


@app.post("/api/command/iris/stop_strike")
def stop_strike():
    global _strike_active
    _strike_stop_event.set()
    _strike_active = False
    print("[STRIKE] Durduruldu.")
    return {"status": "success"}


@app.get("/api/strike_status")
def strike_status():
    if not _strike_active:
        return {"active": False}
    plane = telemetry_state["plane"]
    iris  = telemetry_state["iris"]
    dist = math.sqrt(
        (plane["x"] - iris["x"])**2 +
        (plane["y"] - iris["y"])**2 +
        (plane["z"] - iris["z"])**2
    )
    return {"active": True, "distance": round(dist, 1)}


def _read_iris_telem_from_conn(conn):
    """
    Chase/Strike conn bağlantısı üzerinden iris telemetrisini oku
    ve telemetry_state['iris']'e yaz. Non-blocking, birden fazla
    mesaj okuyabilir (kuyrukta birikenler).
    """
    for _ in range(10):  # en fazla 10 mesaj oku (kuyruk temizliği)
        msg = conn.recv_match(
            type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT'],
            blocking=False
        )
        if not msg:
            break
        _process_mavlink_msg(msg, "iris")


def _strike_thread():
    """Strike altyapı thread'i: bağlantı + algoritma çağrısı."""
    global _strike_active
    try:
        stop_iris_telem()  # port çatışmasını önle
        time.sleep(0.3)
        conn = df_connect_drone(port=14541)
        print(f"[STRIKE] Iris bağlantısı: target_sys={conn.target_system}")

        # Hedef İHA verisi (GPS-bozulmuş)
        def get_plane():
            return dict(_noisy_plane_telem)

        def get_iris():
            # conn üzerinden iris pozisyonunu oku
            _read_iris_telem_from_conn(conn)
            t = telemetry_state["iris"]
            return {"x": t["x"], "y": t["y"], "z": t["z"]}

        _run_strike_algorithm(conn, get_plane, get_iris, _strike_stop_event)

    except Exception as e:
        import traceback
        print(f"[STRIKE] HATA: {e}")
        traceback.print_exc()
    finally:
        _strike_active = False
        start_iris_telem()  # port'u serbest bırakıp tekrar dinle


def _chase_thread():
    """Chase altyapı thread'i: kalkış + chase_algorithm çağrısı."""
    global _chase_active

    print("=" * 50)
    print("[CHASE] Chase Modu Başlıyor (chase_algorithm.run_chase)")
    print("=" * 50)

    try:
        # ---- PORT SERBEST BIRAK ----
        stop_iris_telem()
        time.sleep(0.3)

        # ---- BAĞLANTI ----
        conn = df_connect_drone(port=14541)
        print(f"[CHASE] Iris bağlantısı kuruldu: target_sys={conn.target_system}")

        # ---- KALKIŞ ----
        plane_z = telemetry_state["plane"]["z"]
        target_z = plane_z if plane_z < -1.0 else -5.0
        print(f"[CHASE] Kalkış irtifası: z={target_z:.1f}m (NED)")

        success = df_takeoff(target_z=target_z)
        if not success:
            print("[CHASE] Kalkış başarısız!")
            _chase_active = False
            return
        print("[CHASE] ✓ Kalkış tamamlandı — algoritma başlatılıyor")

        # ---- CALLBACK'LER ----
        def get_plane():
            noisy = dict(_noisy_plane_telem)
            noisy["yaw"] = _noisy_plane_telem.get("yaw", 0.0)
            return noisy

        def get_iris():
            _read_iris_telem_from_conn(conn)
            t = telemetry_state["iris"]
            return {"x": t["x"], "y": t["y"], "z": t["z"]}

        # ---- _chase_active'i stop_event'e bağla ----
        chase_stop = threading.Event()

        def watch_active():
            while _chase_active:
                time.sleep(0.1)
            chase_stop.set()

        watcher = threading.Thread(target=watch_active, daemon=True)
        watcher.start()

        # ---- ALGORİTMAYI ÇAĞIR (AVCI_GPS_LAW: yaklasma=eski sistem portu | v2) ----
        if _GPS_LAW == "v2":
            print("[CHASE] Güdüm yasası: chase v2 (AVCI_GPS_LAW=v2)")
            _run_chase_algorithm(conn, get_plane, get_iris, chase_stop)
        else:
            print("[CHASE] Güdüm yasası: GPS-YAKLASMA (eski sistem portu, gps_approach)")
            _run_gps_approach(conn, get_plane, get_iris, chase_stop)

        # ---- DURDURMA → HOVER ----
        print("[CHASE] Algoritma sonlandı → hover'a geçiliyor...")
        df_hover(duration=3.0)
        print("[CHASE] Chase modu tamamen sonlandı.")

    except Exception as e:
        import traceback
        print(f"[CHASE] HATA: {e}")
        traceback.print_exc()
        _chase_active = False
    finally:
        start_iris_telem()

# -----------------------------------------------------------------------
# ROS 2 KAMERA
# -----------------------------------------------------------------------
latest_frames = {
    "iris":  {"data": None, "id": 0},
    "plane": {"data": None, "id": 0}
}

_yolo_detector = None   # startup'ta yüklenir (AVCI_DETECTOR=yolo, varsayılan açık)

def process_iris_frame(img):
    """Iris kamera karesini işle: Cessna/hedef tespiti + overlay + video parazit
    simülasyonu + MJPEG kodlama. Hem ROS2 (Gazebo Classic) hem gz-transport
    (Gazebo Harmonic) kamera kaynakları bu fonksiyonu çağırır."""
    # ---- HEDEF TESPİT (YOLO) + OVERLAY ----
    if _yolo_detector is not None:
        try:
            det = _yolo_detector.detect_talon(img)
            set_detection(det)
            img = _yolo_detector.draw_overlay(img, det)
        except Exception as e:
            print(f"[GCS] YOLO tespit hatası: {e}")

    # ---- VIDEO PARAZİT SİMÜLASYONU ----
    lvl = _video_noise_level
    if lvl >= 0.999:
        img = np.zeros_like(img)
    elif lvl > 0.001:
        noise_std = lvl * 90.0
        noise = np.random.randn(*img.shape) * noise_std
        img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        if lvl > 0.3:
            num_lines = int(lvl * 25)
            for _ in range(num_lines):
                y = np.random.randint(0, img.shape[0])
                h = np.random.randint(1, max(2, int(lvl * 4)))
                color = np.random.randint(0, 256, 3).tolist()
                img[y:y+h, :] = color
        if lvl > 0.55:
            num_blocks = int(lvl * 8)
            h_, w_ = img.shape[:2]
            for _ in range(num_blocks):
                bx = np.random.randint(0, max(1, w_ - 60))
                by = np.random.randint(0, max(1, h_ - 30))
                bw = np.random.randint(20, 80)
                bh = np.random.randint(5, 25)
                color = np.random.randint(0, 256, 3).tolist()
                img[by:by+bh, bx:bx+bw] = color
        if lvl > 0.7:
            darken = 1.0 - (lvl - 0.7) * 2.5
            img = np.clip(img.astype(np.float32) * darken, 0, 255).astype(np.uint8)
        jpeg_q = max(5, int(90 - lvl * 85))
        _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])
        if latest_frames["iris"]["data"] is None:
            print("[GCS] ✓ Iris kamerasından ilk görüntü!")
        latest_frames["iris"]["data"] = buf.tobytes()
        latest_frames["iris"]["id"] += 1
        return
    # ---- PARAZİTSİZ ----
    _, buf = cv2.imencode('.jpg', img)
    if latest_frames["iris"]["data"] is None:
        print("[GCS] ✓ Iris kamerasından ilk görüntü!")
    latest_frames["iris"]["data"] = buf.tobytes()
    latest_frames["iris"]["id"] += 1


def gz_iris_camera_thread():
    """Gazebo Harmonic: iris kamerasını gz-transport'tan oku (ros_gz köprüsü
    yerine doğrudan). AVCI_GZ_CAMERA=1 ise startup'ta bu thread başlatılır."""
    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.image_pb2 import Image as GzImage
    except Exception as e:
        print(f"[GCS] gz-transport Python yok, Harmonic kamera atlandı: {e}")
        return

    def cb(msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            process_iris_frame(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
        except Exception as e:
            print(f"[GCS GZ-CAM] hata: {e}")

    topic = os.environ.get("AVCI_GZ_CAMERA_TOPIC", "/iris_cam/image")
    node = GzNode()
    node.subscribe(GzImage, topic, cb)
    print(f"[GCS] gz-transport kamera dinleniyor ({topic}, Harmonic)")
    while True:
        time.sleep(1)


def process_plane_frame(img):
    """Hedef İHA (Talon) burun kamerası: ham görüntü → MJPEG. Iris'ten farkı:
    tespit/overlay YOK (bu hedefin kendi görüşü, avcının değil)."""
    _, buf = cv2.imencode('.jpg', img)
    if latest_frames["plane"]["data"] is None:
        print("[GCS] ✓ Talon (hedef İHA) kamerasından ilk görüntü!")
    latest_frames["plane"]["data"] = buf.tobytes()
    latest_frames["plane"]["id"] += 1


def gz_talon_camera_thread():
    """Gazebo Harmonic: Talon (hedef İHA) burun kamerasını gz-transport'tan oku.
    AVCI_GZ_CAMERA=1 ise startup'ta iris ile birlikte başlatılır."""
    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.image_pb2 import Image as GzImage
    except Exception as e:
        print(f"[GCS] gz-transport Python yok, Talon kamera atlandı: {e}")
        return

    def cb(msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            process_plane_frame(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
        except Exception as e:
            print(f"[GCS GZ-CAM] Talon hata: {e}")

    topic = os.environ.get("AVCI_GZ_TALON_TOPIC", "/talon_cam/image")
    node = GzNode()
    node.subscribe(GzImage, topic, cb)
    print(f"[GCS] gz-transport Talon kamera dinleniyor ({topic}, Harmonic)")
    while True:
        time.sleep(1)


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('gcs_camera_listener')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/iris_cam/front_camera/image_raw', self.cb_iris, 1)
        self.create_subscription(Image, '/plane_cam/front_camera/image_raw', self.cb_plane, 1)
        print("[GCS] ROS 2 Kameraları dinleniyor (/iris_cam & /plane_cam)...")

    def cb_iris(self, data):
        try:
            process_iris_frame(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        except Exception as e:
            print(f"[GCS CAM] Iris hata: {e}")

    def cb_plane(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            _, buf = cv2.imencode('.jpg', img)
            if latest_frames["plane"]["data"] is None:
                print("[GCS] ✓ Plane kamerasından ilk görüntü!")
            latest_frames["plane"]["data"] = buf.tobytes()
            latest_frames["plane"]["id"] += 1
        except Exception as e:
            print(f"[GCS CAM] Plane hata: {e}")

def ros2_spin_thread():
    rclpy.init(args=None)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

async def generate_mjpeg(vehicle: str):
    last_id = -1
    try:
        while True:
            entry = latest_frames.get(vehicle)
            if entry and entry["data"] and entry["id"] != last_id:
                last_id = entry["id"]
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + entry["data"] + b'\r\n')
            await asyncio.sleep(0.067)
    except asyncio.CancelledError:
        pass

@app.get("/api/video_feed/{vehicle}")
def video_feed(vehicle: str):
    if vehicle not in ["iris", "plane"]:
        vehicle = "iris"
    return StreamingResponse(generate_mjpeg(vehicle),
                             media_type="multipart/x-mixed-replace; boundary=frame")

# -----------------------------------------------------------------------
# MAVLINK TELEMETRİ (14551=plane, 14540=iris)
# -----------------------------------------------------------------------
def _process_mavlink_msg(msg, vehicle_name):
    """Gelen MAVLink mesajını işle ve telemetry_state'e yaz."""
    msg_type = msg.get_type()
    sys_id = msg.get_srcSystem()

    # İstatistik güncelle
    _mavlink_stats["total"] += 1
    sid_key = str(sys_id)
    _mavlink_stats["by_sysid"][sid_key] = _mavlink_stats["by_sysid"].get(sid_key, 0) + 1
    _mavlink_stats["by_type"][msg_type] = _mavlink_stats["by_type"].get(msg_type, 0) + 1

    if msg_type == 'LOCAL_POSITION_NED':
        spd = round(math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2), 2)

        # ArduPilot SITL'de araç base_link'i doğrudan gövde merkezidir; PX4
        # Talon mesh'indeki görsel offset kaldırıldı. Cessna görsel mesh'i
        # ADIM 10'da eklendiğinde gerekirse buraya offset geri konulur.
        px, py, pz = round(msg.x, 2), round(msg.y, 2), round(msg.z, 2)

        # Plane local'i İRİS ÇERÇEVESİNE taşı (EKF orijinleri farklı; bkz.
        # _frame_off). Ham değer kalibrasyon için ayrıca saklanır.
        if vehicle_name == 'plane':
            _plane_local_raw.update(x=px, y=py, z=pz)
            if _frame_off["ok"]:
                px = round(px + _frame_off["n"], 2)
                py = round(py + _frame_off["e"], 2)
                pz = round(pz + _frame_off["d"], 2)

        telemetry_state[vehicle_name].update(
            x=px, y=py, z=pz,
            vx=round(msg.vx, 2), vy=round(msg.vy, 2), vz=round(msg.vz, 2),
            speed=spd)
        # Plane verisine GPS gürültüsü uygula
        if vehicle_name == 'plane':
            _apply_gps_noise(px, py, pz,
                             telemetry_state['plane']['yaw'])
    elif msg_type == 'GLOBAL_POSITION_INT':
        telemetry_state[vehicle_name].update(
            lat=msg.lat / 1e7, lon=msg.lon / 1e7, alt_amsl=msg.alt / 1000.0)
        if vehicle_name == 'plane':
            _frame_off_update()                   # çerçeve ofsetini kalibre et
    elif msg_type == 'ATTITUDE':
        telemetry_state[vehicle_name].update(
            roll=round(math.degrees(msg.roll), 1),
            pitch=round(math.degrees(msg.pitch), 1),
            yaw=round(math.degrees(msg.yaw), 1))
    elif msg_type == 'HEARTBEAT' and sys_id != 255:
        telemetry_state[vehicle_name]["armed"] = (
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0)


async def mavlink_listener():
    """Plane telemetrisi — port 14550 (ana GCS broadcast)."""
    global _mav_conn, _plane_sysid, _plane_compid
    print("[GCS] MAVLink PLANE dinleniyor (udpin:0.0.0.0:14550)...")
    _mav_conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    # sysid -> is_plane (bool) eşleşmesi
    sysid_is_plane = {}

    while True:
        msg = _mav_conn.recv_match(
            type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT'],
            blocking=False
        )
        if msg:
            sys_id = msg.get_srcSystem()
            if msg.get_type() == 'HEARTBEAT' and sys_id != 255:
                # msg.type: 1=FixedWing, 2=Quadrotor, vs.
                # Eğer daha önce tespit edilmediyse kontrol et
                if sys_id not in sysid_is_plane:
                    if msg.type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
                        sysid_is_plane[sys_id] = True
                        if _plane_sysid is None:
                            print(f"[GCS] Plane sys_id={sys_id} comp_id={msg.get_srcComponent()} tespit edildi.")
                            _plane_sysid  = sys_id
                            _plane_compid = msg.get_srcComponent()
                    else:
                        sysid_is_plane[sys_id] = False

            # Sadece UÇAĞA (FixedWing) ait MAVLink paketlerini "plane" olarak işle
            if sysid_is_plane.get(sys_id, False):
                _process_mavlink_msg(msg, "plane")
            
        await asyncio.sleep(0.005)


# İris telemetri okuyucu — ayrı thread (chase/strike pasifken)
_iris_telem_thread = None
_iris_telem_stop = threading.Event()
_iris_telem_conn = None       # threading conn (paylaşılmaz)

def _iris_telem_worker():
    """İris SITL'den (14541) telemetri oku → telemetry_state['iris'] güncelle."""
    global _iris_telem_conn
    print("[GCS] İris telemetri thread başladı (udpin:0.0.0.0:14541)")
    try:
        _iris_telem_conn = mavutil.mavlink_connection('udpin:0.0.0.0:14541')
    except Exception as e:
        print(f"[GCS] İris 14541 bağlantı hatası: {e}")
        return

    while not _iris_telem_stop.is_set():
        try:
            msg = _iris_telem_conn.recv_match(
                type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT'],
                blocking=False
            )
            if msg:
                _process_mavlink_msg(msg, "iris")
        except Exception:
            pass
        time.sleep(0.005)

    # Bağlantıyı kapa
    try:
        _iris_telem_conn.close()
    except Exception:
        pass
    _iris_telem_conn = None
    print("[GCS] İris telemetri thread durdu")

def start_iris_telem():
    """İris telemetri okumasını başlat (chase/strike pasifken)."""
    global _iris_telem_thread
    if _iris_telem_thread and _iris_telem_thread.is_alive():
        return  # zaten çalışıyor
    _iris_telem_stop.clear()
    _iris_telem_thread = threading.Thread(target=_iris_telem_worker, daemon=True)
    _iris_telem_thread.start()

def stop_iris_telem():
    """İris telemetri okumasını durdur (chase/strike başlamadan önce port serbest kalsın)."""
    global _iris_telem_thread
    _iris_telem_stop.set()
    if _iris_telem_thread:
        _iris_telem_thread.join(timeout=2.0)
    _iris_telem_thread = None

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(mavlink_listener())          # plane — 14550
    start_iris_telem()                                # iris  — 14541 (background thread)
    # YOLO detector'ı yükle (opsiyonel; AVCI_DETECTOR=off ile kapatılır)
    if os.environ.get("AVCI_DETECTOR", "yolo").lower() == "yolo":
        global _yolo_detector
        try:
            from vision import detector as _det
            _det.load()                          # ağırlık + CUDA warmup
            _yolo_detector = _det
            print("[GCS] YOLO detector hazır (avci_yolo.pt)")
        except Exception as e:
            print(f"[GCS] YOLO detector yüklenemedi ({e}) — tespit kapalı")

    # Kamera kaynağı: Harmonic (gz-transport) veya Classic (ROS2 cv_bridge)
    if os.environ.get("AVCI_GZ_CAMERA", "0") == "1":
        threading.Thread(target=gz_iris_camera_thread, daemon=True).start()   # avcı iris
        threading.Thread(target=gz_talon_camera_thread, daemon=True).start()  # hedef Talon
    else:
        threading.Thread(target=ros2_spin_thread, daemon=True).start()
    if os.environ.get("AVCI_NO_BROWSER", "0") != "1":
        threading.Timer(2.0, lambda: webbrowser.open("http://localhost:8000")).start()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # GPS noise seviyesini de frontend'e gönder
            payload = dict(telemetry_state)
            payload["gps_noise"] = _gps_noise_level
            payload["gps_frozen"] = _noisy_plane_telem.get("frozen", False)
            payload["plane_throttle"] = _plane_throttle
            await websocket.send_json(payload)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass

if __name__ == "__main__":
    print("==================================================")
    print(" AVCI GCS SERVER BAŞLATILIYOR (Port: 8000)")
    print("==================================================")
    uvicorn.run(app, host="0.0.0.0", port=8000)