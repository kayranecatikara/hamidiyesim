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
from vision.detection_state import (set_detection, set_pose_detection,
                                    wait_new_pose, get_detection,
                                    get_pose_detection)
from control import carpisma_state       # A5 — gerçek temas, güdümle ortak durum
# YOLO detector (vision/detector.py) opsiyonel — startup'ta yüklenir (_yolo_detector).

# Kare scriptinin de kullandığı, kanıtlanmış çalışan modüller (ArduPilot)
from control.mav_common import (
    connect_mavlink,
    GCSKeepalive,
    set_mode,
    wait_ack,
    arm as mav_arm,
    disarm as mav_disarm,
    PLANE_MODE_MANUAL,
    PLANE_MODE_FBWA,
    PLANE_MODE_FBWB,
    PLANE_MODE_TAKEOFF,
)

app = FastAPI(title="Avcı GCS")

ui_path = os.path.join(os.path.dirname(__file__), "gcs_ui")
if not os.path.exists(ui_path):
    os.makedirs(ui_path)
app.mount("/ui", StaticFiles(directory=ui_path, html=True), name="ui")

# Uçuş log paneli — gps_log_viz.py'nin ürettiği HTML tarayıcıdan açılsın diye
# logs/ dizini servis edilir. Panel her GPS uçuşu bitince otomatik tazelenir
# (bkz. control/guidance/gps_guidance.py:_panel_tazele). Kısayol: /panel
_logs_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
os.makedirs(_logs_path, exist_ok=True)
app.mount("/loglar", StaticFiles(directory=_logs_path), name="loglar")

@app.get("/")
def read_root():
    return RedirectResponse(url="/ui/index.html")

@app.get("/panel")
def log_paneli():
    """Uçuş log paneli kısayolu — en son uçuşları görselleştirir."""
    return RedirectResponse(url="/loglar/gps_log_panel.html")

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
    sn = (ip["x"] + rel_n) - _plane_local_raw["x"]
    se = (ip["y"] + rel_e) - _plane_local_raw["y"]
    # ── DİKEY: AMSL KULLANMA (2026-07-25 kök-neden düzeltmesi) ──
    # İki SITL'in EKF orijin İRTİFALARI farklı (araç-tipi varsayılan home alt'ları
    # ~12.7m ayrık; start_harmonic.sh --home vermiyor). GPS lat/lon gerçek yatay
    # konumu doğru verirken alt_amsl bu 12.7m sahte ofseti taşıyordu: kamera+ham
    # yerel-z "hedef ALTTA" derken AMSL "ÜSTTE" diyordu → güdüm drone'u hedefin
    # üstüne çıkarıp görsel teması ENGELLİYORDU. İki araç da aynı düz zemine
    # spawn olduğu için dikey orijin farkı = 0; plane ham yerel-z'si doğrudan
    # iris ile kıyaslanabilir → d = 0. (Yatay N/E GPS kalibrasyonu korunuyor.)
    sd = 0.0
    if _frame_off["samples"] == 0:
        _frame_off.update(n=sn, e=se, d=sd)
    else:
        a = 0.1
        _frame_off["n"] = (1 - a) * _frame_off["n"] + a * sn
        _frame_off["e"] = (1 - a) * _frame_off["e"] + a * se
        _frame_off["d"] = 0.0
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

# Uçak senaryo süreci (kare/daire/agresif) — aynı anda en fazla biri çalışır
_scenario_proc = None
_scenario_name = None

# Uçak throttle seviyesi — slider ile ayarlanır (0-1000 aralığı, MANUAL_CONTROL)
_plane_throttle = 600   # default = THROTTLE_CRUISE

# Video parazit simülasyonu — iris kamera akışına uygulanır
_video_noise_level = 0.0   # 0.0 = temiz, 1.0 = tamamen parazitli

# Manuel mod durumu
_manual_active = False
_manual_aileron  = 1500
_manual_elevator = 1500
_manual_throttle = 1000

# Manuel mod: FBWA→FBWB geçiş irtifası ve otonom kalkış için beklenecek süre.
# FBWB irtifa TUTAR, o yüzden yerdeyken kullanılamaz (kalkamaz); FBWA ise açı
# tutar ama irtifa tutmaz (stick ortada uçak alçalır). Bu yüzden yerde FBWA,
# eşiği geçince FBWB.
_MANUAL_FBWB_ALT = float(os.environ.get("AVCI_MANUAL_FBWB_ALT", "15.0"))
_MANUAL_TAKEOFF_SURE = float(os.environ.get("AVCI_MANUAL_TAKEOFF_SURE", "20.0"))


def _havada_mi(esik=None):
    """Uçak FBWB'ye geçebilecek kadar yüksekte mi? NED: -z = yükseklik.
    Telemetri yoksa (z=0) YERDE sayılır — yanlış tarafa hata yapmayalım."""
    return -telemetry_state["plane"].get("z", 0.0) >= (
        _MANUAL_FBWB_ALT if esik is None else esik)

def id_to_name(sysid):
    # ArduPilot SITL: iris/copter sysid=5, plane sysid=2
    if sysid in (1, 5):      # iris (ArduCopter)
        return "iris"
    elif sysid in (2, 3):    # plane (ArduPlane)
        return "plane"
    return None

# -----------------------------------------------------------------------
# UÇUŞ SENARYOLARI (kare / daire / agresif) — run_plane_scenario.py süreci
# -----------------------------------------------------------------------
_SCENARIO_NAMES = ("square", "circle", "aggressive")


def _stop_scenario_proc():
    """Çalışan senaryo sürecini (varsa) öldürür + eski süreç artıklarını süpürür."""
    global _scenario_proc, _scenario_name
    if _scenario_proc is not None and _scenario_proc.poll() is None:
        try:
            os.killpg(os.getpgid(_scenario_proc.pid), signal.SIGKILL)
        except Exception:
            try:
                _scenario_proc.kill()
            except Exception:
                pass
        _scenario_proc.wait()
        print(f"[SCENARIO] '{_scenario_name}' durduruldu.")
    _scenario_proc = None
    _scenario_name = None
    # Emniyet: GCS yeniden başlatıldıysa elde referansı olmayan süreç kalmış olabilir
    subprocess.run(['pkill', '-9', '-f', 'run_plane_scenario'], capture_output=True)


@app.post("/api/command/plane/scenario/{name}")
def start_plane_scenario(name: str):
    """Senaryo başlat: araç takeoff yapıp deseni süresiz uçar.
    square=kare, circle=daire, aggressive=rastgele agresif manevralar."""
    global _scenario_proc, _scenario_name, _manual_active
    if name not in _SCENARIO_NAMES:
        return {"status": "error", "message": f"Bilinmeyen senaryo: {name}"}
    try:
        if _manual_active:                 # manuel kontrol açıksa kapat
            _manual_active = False
            time.sleep(0.3)
        _stop_scenario_proc()              # önceki senaryo (varsa) dursun
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        _scenario_proc = subprocess.Popen(
            ["python3", "-m", "control.run_plane_scenario", name],
            cwd=project_root,
            start_new_session=True,
        )
        _scenario_name = name
        print(f"[SCENARIO] '{name}' başlatıldı (pid={_scenario_proc.pid})")
        return {"status": "success", "message": f"Senaryo başlatıldı: {name}"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.post("/api/command/plane/stop_scenario")
def stop_plane_scenario():
    _stop_scenario_proc()
    return {"status": "success", "message": "Senaryo durduruldu."}


@app.get("/api/scenario_status")
def scenario_status():
    """Frontend buton senkronu: süreç yaşıyorsa aktif senaryo adı."""
    if _scenario_proc is not None and _scenario_proc.poll() is None:
        return {"active": True, "name": _scenario_name}
    return {"active": False, "name": None}

# -----------------------------------------------------------------------
# MANUEL KONTROL
# -----------------------------------------------------------------------
class ManualCmd(BaseModel):
    aileron:  int
    elevator: int
    throttle: int

@app.post("/api/command/plane/start_manual")
def start_manual_mode():
    """Hangi senaryo uçuyorsa durdurur, uçağı klavye kontrolüne devralır.
    Yerdeyken FBWA (tırmanabilsin), AVCI_MANUAL_FBWB_ALT eşiğini geçince
    FBWB (irtifa tutsun, burun aşağı düşmesin). W/S = pitch/tırmanış,
    A/D = yatış açı hedefi. Bkz. _manual_control_thread."""
    global _manual_active
    global _manual_aileron, _manual_elevator, _manual_throttle

    # ADIM 1: Aktif senaryoyu durdur — RC override boşluğu doğmadan
    # manuel thread devralacak
    _stop_scenario_proc()

    # ADIM 2: Manuel kontrol thread'ini başlat.
    # Gaz cruise'dan (1600) başlar — eski kod 1000 (rölanti) veriyordu,
    # havada devralınca uçak stall'a giriyordu.
    _manual_aileron  = 1500
    _manual_elevator = 1500
    _manual_throttle = 1600
    _manual_active = True

    t = threading.Thread(target=_manual_control_thread, daemon=True)
    t.start()
    print("[GCS] Manuel kontrol thread'i başlatıldı.")
    return {"status": "success", "message": "Manuel mod aktif (yerde FBWA → havada FBWB)"}


def _manual_control_thread():
    """
    Uçağı klavye/joystick kontrolüne devralır (10 Hz RC override).
    Mod irtifaya göre seçilir: yerde FBWA (kalkabilsin), havada FBWB (irtifa tutsun).

    ÖNEMLİ: Bu thread paylaşılan _mav_conn üzerinde ASLA blocking recv yapmaz.
    Eski kod set_mode ile ACK/heartbeat okuyordu; aynı bağlantıyı async
    mavlink_listener da okuduğu için mesaj yarışı oluyor, devralma saniyelerce
    gecikiyor ve bu boşlukta araç düşebiliyordu. Artık:
    - Mod komutu ACK beklemeden gönderilir (gerekirse 0.5s'de bir tekrar),
    - Teyit telemetry_state['plane']['mode'] üzerinden (listener HEARTBEAT'ten
      custom_mode yazar),
    - RC override İLK saniyeden itibaren akar → kontrol boşluğu yok.
    """
    global _manual_active

    print("[MANUAL] Thread başlıyor...")
    try:
        # BAĞLANTI — global _mav_conn (14550, sadece gönderim için kullanılır)
        if _mav_conn is None:
            raise RuntimeError("Global MAVLink bağlantısı yok!")

        conn = _mav_conn
        if _plane_sysid is not None:
            conn.target_system = _plane_sysid

        print(f"[MANUAL] Bağlantı kullanılıyor: target_sys={conn.target_system}")

        # KEEPALIVE — arming korunması için şart
        keepalive = GCSKeepalive(conn, interval=0.1)
        keepalive.start()

        # ── YERDEN KALKIŞ: ARM + TAKEOFF ──
        # 2026-08-01: manuel mod yerdeki uçağı HİÇ kaldıramıyordu. Sebep mod
        # seçimi değil, mod DEĞİŞTİRMENİN TEK BAŞINA YETMEMESİ: uçak disarm
        # halde ve motoru kapalı; RC override göndermek bir şey yapmıyor.
        # run_plane_scenario bunu doğru yapıyor ("bağlan → force ARM → TAKEOFF
        # modu ile otonom kalkış → FBWA + RC"); manuel modda o adımlar hiç yoktu.
        # Manuel mod baştan "havadaki uçağı devral" için yazılmış; yerden
        # başlatınca sessizce hiçbir şey olmuyordu.
        # TAKEOFF sırasında RC override GÖNDERİLMEZ — senaryo da göndermiyor,
        # otopilotun kalkış profilini bozar.
        if not telemetry_state["plane"].get("armed", False) or not _havada_mi():
            print("[MANUAL] Uçak yerde/disarm → ARM + TAKEOFF")
            try:
                mav_arm(conn, force=True)
            except Exception as e:
                print(f"[MANUAL] ARM hatası: {e}")
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                PLANE_MODE_TAKEOFF, 0, 0, 0, 0, 0)
            t0 = time.time()
            while (_manual_active and time.time() - t0 < _MANUAL_TAKEOFF_SURE
                   and not _havada_mi()):
                time.sleep(0.2)
            print(f"[MANUAL] Kalkış bitti (irtifa "
                  f"~{-telemetry_state['plane'].get('z', 0.0):.0f} m)")

        # ── İRTİFAYA GÖRE MOD: yerde FBWA, havada FBWB ──
        # Ham MANUAL (0) havada elle uçulamıyordu. FBWA (5) açıyı tutar ama
        # İRTİFAYI TUTMAZ: stick ortada 0° pitch demek ve 0° pitch'te uçak
        # alçalır — burun-aşağı şikâyetinin sebebi buydu. FBWB (6) irtifa tutar,
        # ama TAM DA BU YÜZDEN yerdeyken kalkamaz: "mevcut irtifayı koru" =
        # yerde kal. (Önce koşulsuz FBWB yapılmıştı, Talon hiç kalkmadı.)
        # Çözüm: yerdeyken FBWA ile tırman, eşiği geçince FBWB'ye geç.
        def _istenen_mod():
            return PLANE_MODE_FBWB if _havada_mi() else PLANE_MODE_FBWA

        fbwb_alt = _MANUAL_FBWB_ALT

        def _send_manual_mode(hedef):
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                hedef, 0, 0, 0, 0, 0)

        hedef_mod = _istenen_mod()
        print(f"[MANUAL] {'FBWB' if hedef_mod == PLANE_MODE_FBWB else 'FBWA'} "
              f"komutu gönderildi (FBWB eşiği {fbwb_alt:.0f} m), "
              f"RC override döngüsü başlıyor (10 Hz)...")
        _send_manual_mode(hedef_mod)
        mode_ok = False
        tick = 0
        while _manual_active:
            # İrtifa eşiği geçilince FBWA → FBWB (tırmanış bitti, irtifa tutulsun)
            yeni_hedef = _istenen_mod()
            if yeni_hedef != hedef_mod and yeni_hedef == PLANE_MODE_FBWB:
                hedef_mod = yeni_hedef
                mode_ok = False
                print(f"[MANUAL] irtifa {-telemetry_state['plane'].get('z', 0.0):.0f} m "
                      f"→ {'FBWB (irtifa tut)' if hedef_mod == PLANE_MODE_FBWB else 'FBWA'}")
                _send_manual_mode(hedef_mod)
            if not mode_ok:
                if telemetry_state["plane"].get("mode") == hedef_mod:
                    mode_ok = True
                    print(f"[MANUAL] ✓ {'FBWB' if hedef_mod == PLANE_MODE_FBWB else 'FBWA'}"
                          f" teyit edildi (heartbeat)")
                elif tick > 0 and tick % 5 == 0:      # 0.5s'de bir tekrar dene
                    _send_manual_mode(hedef_mod)
            conn.mav.rc_channels_override_send(
                conn.target_system,
                conn.target_component,
                _manual_aileron,    # CH1: Roll/Aileron
                _manual_elevator,   # CH2: Pitch/Elevator
                _manual_throttle,   # CH3: Throttle
                1500,               # CH4: Yaw nötr
                0, 0, 0, 0
            )
            tick += 1
            time.sleep(0.1)

        # Kapanış — yüzeyler nötr, gaz CRUISE bırakılır (1000=rölanti stall
        # ettiriyordu). Override 3 sn içinde kendiliğinden düşer.
        conn.mav.rc_channels_override_send(
            conn.target_system, conn.target_component,
            1500, 1500, 1600, 1500, 0, 0, 0, 0
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
    """Klavye kontrol değerlerini (PWM) manuel thread'e iletir."""
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
    """Anlık telemetry state + MAVLink istatistikleri.

    DİKEY TEŞHİS: kamera "hedef altta" derken telemetri "hedef üstte" diyorsa
    dikey hiza (frame_off.d) yanlış demektir. Aşağıdaki alanlar kök nedeni açar:
      - iris/plane alt_amsl: iki SITL'in AMSL'i tutarlı mı? (yerde eşit olmalı)
      - plane_local_raw.z vs telemetry_state.plane.z: çerçeve ofseti ne kattı?
      - frame_off.d: kilitli dikey ofset (başlangıçta 1 kez hesaplanır)."""
    ip, pp = telemetry_state["iris"], telemetry_state["plane"]
    return {
        "telemetry_state": telemetry_state,
        "mavlink_stats": _mavlink_stats,
        "plane_sysid": _plane_sysid,
        "frame_off": dict(_frame_off),
        "plane_local_raw": dict(_plane_local_raw),
        "dikey_teshis": {
            "iris_alt_amsl": ip["alt_amsl"],
            "plane_alt_amsl": pp["alt_amsl"],
            "amsl_fark_m": round(pp["alt_amsl"] - ip["alt_amsl"], 2),
            "iris_irtifa_local_m": round(-ip["z"], 2),
            "plane_irtifa_local_m": round(-pp["z"], 2),
            "plane_local_raw_irtifa_m": round(-_plane_local_raw["z"], 2),
            "telemetri_dikey_fark_m": round(-pp["z"] - (-ip["z"]), 2),
        },
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

    # A5: temas mandalını yeni koşu için temizle. Yoksa önceki denemede gelen
    # temas hâlâ latch'li kalır ve görsel faz daha ilk karede "vuruldu" der.
    hasar_durumu.update(imha=False, menzil=None, t=None, temas=None)
    carpisma_state.sifirla()

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
    # GPS-YAKLASMA yasasının canlı durumu (ARAMA/KILIT/DROPOUT + handoff)
    resp["guidance"] = dict(_gps_guidance_mod.status)
    resp["supervisor"] = dict(_supervisor_mod.status)
    return resp


# -----------------------------------------------------------------------
# GÖRÜŞ & FAZ PANELİ — /api/telemetry/pnp
# -----------------------------------------------------------------------
# 2026-08-01: Bu endpoint ESKİDEN SAHTEYDİ. Ground-truth telemetriye yapay
# gauss gürültüsü + sabit 8 m ofset ekleyip "PnP çıkışı" diye sunuyordu
# ("rapor fotoğrafları için" notuyla). Pose modeline hiç dokunmuyordu, o
# yüzden panele bakarak görsel fazın ne zaman devreye gireceği anlaşılamıyordu
# ve saatlerce açık kalan GCS'te 6000 m gibi değerler görünüyordu.
#
# Yerine GERÇEK veri: kameranın ürettiği kestirim ile ground-truth YAN YANA,
# ikisi ayrı ayrı etiketli ("cevap anahtarı" ilkesi: kameranın kestirimi ile
# ground-truth yan yana, fark = algı hatası).
# Ayrıca GPS→görsel geçişinin İKİ kapısı da canlı gösteriliyor, çünkü asıl
# sorulan soru bu: "neden hâlâ görsel faza geçmedi?"
#   • pose kilidi : son KILIT_PENCERE karenin kaçında conf ≥ POSE_CONF_MIN
#   • menzil kapısı: yatay mesafe d_h < GATE_MENZIL mi
# Hangisinin bağlayıcı olduğu panelden tek bakışta görünür.

def _gorus_menzil_kestirimi(pose, iris_att):
    """Pose keypoint'lerinden ölçek tabanlı menzil (m) — guidance_core ile AYNI
    formül (Adım 2-3). Kamera dışında hiçbir bilgi kullanmaz.

    iris_att verilirse yükselti düzeltmesi de uygulanır (hedef seviyeli uçuyor
    varsayımı); yoksa ham ölçek kullanılır ve değer bir miktar iyimser çıkar.
    Dönüş: (menzil_m, kanat_gorunur_mu) veya (None, False).
    """
    try:
        kpts = pose["kpts"]
        burun, kuyruk = kpts[0], kpts[1]
        solk, sagk = kpts[2], kpts[3]
        a = math.hypot(burun[0] - kuyruk[0], burun[1] - kuyruk[1])
        b = math.hypot(solk[0] - sagk[0], solk[1] - sagk[1])
        olcek = math.sqrt(a * a + (_GOVDE_KANAT_ORANI * b) ** 2)
        if olcek <= 1e-9:
            return None, False
        if iris_att is not None:
            # Kamera ışını → gövde → dünya: LOS yükselişi (eps) ile ölçek düzeltilir
            u = np.array([(pose["cx"] - _geo.CX) / _geo.FX,
                          (pose["cy"] - _geo.CY) / _geo.FY, 1.0])
            u = u / np.linalg.norm(u)
            u_g = _kamera_to_govde(u, math.radians(_LeadCfg.KAMERA_TILT_DEG))
            u_d = _govde_to_dunya(u_g, *iris_att)
            eps = math.asin(max(-1.0, min(1.0, -float(u_d[2]))))
            olcek = olcek / _yukselti_duzeltme(eps)
        kanat_ok = (solk[2] >= _LeadCfg.KPT_CONF_MIN
                    and sagk[2] >= _LeadCfg.KPT_CONF_MIN)
        return _geo.FX * _GOVDE_BOYU_M / olcek, kanat_ok
    except Exception:
        return None, False


@app.get("/api/telemetry/pnp")
def pnp_telemetry():
    """Görüş hattının GERÇEK çıktısı + faz kapıları + ground-truth kıyası.

    Buradaki hiçbir değer güdüme girmez; panel yalnız gözlem içindir.
    """
    det = get_detection()
    pose = get_pose_detection()
    iris = telemetry_state["iris"]
    plane = telemetry_state["plane"]

    iris_att = None
    if any(iris.get(k) for k in ("roll", "pitch", "yaw")):
        iris_att = (iris.get("roll", 0.0), iris.get("pitch", 0.0),
                    iris.get("yaw", 0.0))

    # ── Kameranın kendi menzil kestirimi (ölçek tabanlı) ──
    gorus_menzil, kanat_ok = (None, False)
    if pose is not None:
        gorus_menzil, kanat_ok = _gorus_menzil_kestirimi(pose, iris_att)

    # ── Ground-truth (SİM KOLAYLIĞI — gerçek harekâtta yok, etiketli göster) ──
    gercek_menzil = None
    telem_var = not (plane["x"] == 0 and plane["y"] == 0 and plane["z"] == 0)
    if telem_var:
        gercek_menzil = math.sqrt((plane["x"] - iris["x"]) ** 2
                                  + (plane["y"] - iris["y"]) ** 2
                                  + (plane["z"] - iris["z"]) ** 2)

    # ── Faz kapıları: "neden hâlâ geçmedi" sorusunun cevabı ──
    sup = _supervisor_mod.status
    sup_cfg = _supervisor_mod.SupCfg
    gps_st = _gps_guidance_mod.status
    d_h = gps_st.get("d_h")

    kilit_sayac = sup.get("kilit_sayac", 0)
    poz_kapi_ok = kilit_sayac >= sup_cfg.KILIT_N
    menzil_kapi_ok = (not sup_cfg.GATE_KILIT) or (
        d_h is not None and d_h < sup_cfg.GATE_MENZIL) or (
        gps_st.get("durum") == "DROPOUT")

    if sup["faz"] != "GPS":
        engel = "—"
    elif not poz_kapi_ok and not menzil_kapi_ok:
        engel = "POSE + MENZİL"
    elif not poz_kapi_ok:
        engel = "POSE KİLİDİ"
    elif not menzil_kapi_ok:
        engel = "MENZİL KAPISI"
    else:
        engel = "—"

    return {
        "active": det is not None or pose is not None or telem_var,
        "faz": sup.get("faz", "GPS"),
        "gecis_sayisi": sup.get("gecis_sayisi", 0),
        # görüş hattı
        "tespit_var": det is not None,
        "tespit_conf": round(float(det["conf"]), 2) if det else None,
        "pose_var": pose is not None,
        "pose_conf": round(float(pose.get("conf", 0.0)), 2) if pose else None,
        "kanat_gorunur": bool(kanat_ok),
        "gorus_menzil": round(gorus_menzil, 1) if gorus_menzil else None,
        # ground-truth (etiketli)
        "gercek_menzil": round(gercek_menzil, 1) if gercek_menzil else None,
        "menzil_hata": (round(gorus_menzil - gercek_menzil, 1)
                        if (gorus_menzil and gercek_menzil) else None),
        # faz kapıları
        "kilit_sayac": kilit_sayac,
        "kilit_n": sup_cfg.KILIT_N,
        "kilit_pencere": sup_cfg.KILIT_PENCERE,
        "d_h": round(d_h, 1) if d_h is not None else None,
        "gate_menzil": sup_cfg.GATE_MENZIL,
        "poz_kapi_ok": poz_kapi_ok,
        "menzil_kapi_ok": bool(menzil_kapi_ok),
        "engel": engel,
    }


from control.guidance import gps_guidance as _gps_guidance_mod
from control.guidance.gps_guidance import run_gps_guidance as _run_gps_guidance
from control.guidance.visual_lead import run_visual_lead as _run_visual_lead
from control.guidance import supervisor as _supervisor_mod
from control.guidance.supervisor import run_hybrid as _run_hybrid

# Görüş paneli (/api/telemetry/pnp) menzil kestirimini guidance_core'un AYNI
# formülüyle üretir — panelin gösterdiği sayı güdümün gördüğü sayı olsun diye
# ayrı bir kopya YAZILMADI.
from control.guidance.guidance_core import (
    Cfg as _LeadCfg, GOVDE_BOYU_M as _GOVDE_BOYU_M,
    GOVDE_KANAT_ORANI as _GOVDE_KANAT_ORANI,
    kamera_to_govde as _kamera_to_govde, govde_to_dunya as _govde_to_dunya,
    yukselti_duzeltme as _yukselti_duzeltme)
from vision import geometry as _geo


# ══════════════════════════════════════════════════════════
#  HASAR MODÜLÜ — çarpışmada hedef imha olsun
# ══════════════════════════════════════════════════════════
# Gazebo ÇARPIŞMA fiziğini zaten uyguluyor: iki gövde temas edince itilir, hafif
# avcı savrulup düşer (kara kutuda t=148s'de |roll|>90°, irtifa 595→533 m).
# Eksik olan HASAR: hedef vurulduğunu "bilmiyor", darbeden sonra dengesini
# toplayıp uçmaya devam ediyordu — görev başarılı olsa bile ekranda öyle
# görünmüyordu.
#
# ⚠ 2026-08-02: VARSAYILAN KAPALI. İlk sürüm yakınlık eşiği (<2 m) kullanıyordu
# ve ıskalayıp yanından geçtiğinde de hedefi düşürüyordu — yanlıştı. Contact
# sensörlü sürüm için gereken SDF değişiklikleri (dünya eklentisi + talon
# sensörü) GERİ ALINDI, o yüzden bu modül şu an tetiklenemez. AVCI_HASAR=1 ile
# açılırsa yalnız temas topic'ini dinler; topic yoksa hiçbir şey yapmaz.
#
# TETİK: GAZEBO CONTACT SENSÖRÜ — yani GERÇEK fiziksel temas.
# İlk sürüm ground-truth menzil < 2 m'yi "çarpışma" sayıyordu; bu YANLIŞTI ve
# uçuşta ıskalayıp yakınından geçtiği halde hedefi düşürüyordu. Yakınlık
# çarpışma değildir. Artık tek kaynak, mini_talon'un gövde/kanat/kuyruk
# yüzeylerine bağlı `carpisma_sensoru` (bkz. models/mini_talon_vtail/model.sdf
# ve worlds/avci_harmonic.sdf içindeki gz-sim-contact-system).
#
# SÜZGEÇ: hedef sürekli ZEMİNE de değiyor (kalkış, iniş, çakılma). Temasın
# karşı tarafı iris değilse İMHA SAYILMAZ.
#
# NEDEN AYRI MODÜL: vuruş tespiti yalnız visual_lead içinde var, GPS fazında
# yok — çarpışma GPS fazında olursa kimse fark etmiyordu. Bu modül hangi faz
# çalışırsa çalışsın (hatta hiçbiri çalışmasa da) temasi dinler.
# A5: VARSAYILAN AÇIK. Eskiden "0" idi ve modül hiç çalışmıyordu; vuruş kararı
# tamamen visual_lead'deki 1.5 m yakınlık ölçütüne kalmıştı. Kapatmak için
# AVCI_HASAR=0 — o zaman güdüm yakınlık yedeğine düşer (bkz. carpisma_state).
_HASAR_AKTIF = os.environ.get("AVCI_HASAR", "1") == "1"
_HASAR_TOPIC = os.environ.get(
    "AVCI_HASAR_TOPIC",
    "/world/avci/model/mini_talon/link/base_link/sensor/carpisma_sensoru/contact")
# Temasın karşı tarafında bu geçiyorsa avcı ile çarpışmışız demektir.
_HASAR_AVCI_ADI = os.environ.get("AVCI_HASAR_AVCI_ADI", "iris")
hasar_durumu = {"imha": False, "menzil": None, "t": None,
                "temas": None, "kaynak": "gazebo_contact"}


def _hasar_uygula(detay):
    """Hedefi imha et: force-disarm → motor kesilir, yüzeyler ölür, düşer."""
    if hasar_durumu["imha"]:
        return
    ip, pp = telemetry_state["iris"], telemetry_state["plane"]
    d = math.sqrt((pp["x"] - ip["x"]) ** 2 + (pp["y"] - ip["y"]) ** 2
                  + (pp["z"] - ip["z"]) ** 2)
    hasar_durumu.update(imha=True, t=time.time(), menzil=round(d, 2), temas=detay)
    carpisma_state.temas_bildir(detay)   # güdüm buradan okur (A5)
    print("\n" + "=" * 50)
    print(f"[HASAR] \u2738 GERÇEK ÇARPIŞMA — {detay}")
    print(f"[HASAR] temas anındaki menzil {d:.2f} m")
    print("[HASAR] Talon disarm ediliyor (motor + yüzeyler ölü)")
    print("=" * 50)
    if _mav_conn is not None and _plane_sysid is not None:
        onceki = _mav_conn.target_system
        try:
            _mav_conn.target_system = _plane_sysid
            mav_disarm(_mav_conn, force=True)
        except Exception as e:
            print(f"[HASAR] disarm hatası: {e}")
        finally:
            _mav_conn.target_system = onceki


def _hasar_izleyici():
    """gz-transport'tan temas mesajlarını dinler; karşı taraf avcıysa imha eder."""
    if not _HASAR_AKTIF:
        print("[HASAR] Modül KAPALI (AVCI_HASAR=0) — güdüm yakınlık yedeğine düşecek")
        carpisma_state.kaynak_bildir(False)
        return
    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.contacts_pb2 import Contacts
    except Exception as e:
        print(f"[HASAR] gz-transport yok ({e}) — çarpışma tespiti DEVRE DIŞI. "
              f"Hedef vurulsa da düşmeyecek; güdüm yakınlık yedeğine düşecek.")
        carpisma_state.kaynak_bildir(False)
        return

    def cb(msg):
        try:
            if hasar_durumu["imha"]:
                return
            for c in msg.contact:
                # Temasın iki tarafı: biri talon (sensörün kendisi), diğeri ne?
                ad1 = getattr(c.collision1, "name", "") or ""
                ad2 = getattr(c.collision2, "name", "") or ""
                karsi = ad2 if _HASAR_AVCI_ADI in ad2 else (
                    ad1 if _HASAR_AVCI_ADI in ad1 else None)
                if karsi:
                    _hasar_uygula(f"temas: {ad1} ↔ {ad2}")
                    return
        except Exception as e:
            print(f"[HASAR] temas mesajı işlenemedi: {e}")

    node = GzNode()
    if not node.subscribe(Contacts, _HASAR_TOPIC, cb):
        print(f"[HASAR] temas topic'ine abone olunamadı: {_HASAR_TOPIC}")
        print("[HASAR] güdüm yakınlık yedeğine düşecek "
              "(dünyada gz-sim-contact-system ve modelde carpisma_sensoru var mı?)")
        carpisma_state.kaynak_bildir(False)
        return
    carpisma_state.kaynak_bildir(True)
    print(f"[HASAR] GERÇEK çarpışma dinleniyor: {_HASAR_TOPIC}")
    print("[HASAR] vuruş ölçütü = fiziksel temas (yakınlık TEK BAŞINA vuruş değil)")
    while True:
        # menzili yalnız GÖZLEM için güncelle — tetikleyici DEĞİL
        try:
            ip, pp = telemetry_state["iris"], telemetry_state["plane"]
            if not (pp["x"] == 0 and pp["y"] == 0 and pp["z"] == 0):
                hasar_durumu["menzil"] = round(
                    math.sqrt((pp["x"] - ip["x"]) ** 2 + (pp["y"] - ip["y"]) ** 2
                              + (pp["z"] - ip["z"]) ** 2), 2)
        except Exception:
            pass
        time.sleep(0.2)


@app.get("/api/hasar")
def hasar_get():
    """Hasar durumu — arayüz/otomasyon için."""
    return {**hasar_durumu, "aktif": _HASAR_AKTIF, "topic": _HASAR_TOPIC}


@app.post("/api/hasar/sifirla")
def hasar_sifirla():
    """Yeni denemeden önce imha bayrağını temizler."""
    hasar_durumu.update(imha=False, menzil=None, t=None, temas=None)
    carpisma_state.sifirla()      # güdüm tarafı da temizlensin, yoksa yeni
                                  # görsel faz daha ilk karede "vuruldu" der
    return {"status": "success", "message": "Hasar durumu sıfırlandı."}


@app.get("/api/debug/carpisma")
def carpisma_debug():
    """A5 tanılama: temas kaynağı çalışıyor mu, temas geldi mi.
    kaynak_hazir=False ise güdüm hâlâ 1.5 m yakınlık yedeğindedir."""
    return carpisma_state.durum()


# ══════════════════════════════════════════════════════════
#  GÖRSEL GÜDÜM (IBVS lead pursuit v2) — izole hat:
#  pose keypoint'leri → menzil bağımsız lead → hız + yaw komutu.
#  GPS chase'i BOZMAZ; ayrı endpoint. Döngü kameraya kilitli (olay güdümlü).
# ══════════════════════════════════════════════════════════
_visual_active = False
_visual_stop_event = threading.Event()


def _visual_thread():
    """Görsel güdüm altyapısı: kalkış + IBVS lead pursuit döngüsü."""
    global _visual_active
    print("=" * 50)
    print("[VISUAL] Görsel Güdüm (IBVS lead pursuit v2) başlıyor")
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
        print("[VISUAL] ✓ Kalkış tamam — lead pursuit başlatılıyor")

        def get_plane_truth():
            """Hedefin GERÇEK pozu (çerçeve-ofset düzeltmeli NED) — SADECE
            menzil_gercek logu için, güdüme girmez."""
            t = telemetry_state["plane"]
            return {"x": t["x"], "y": t["y"], "z": t["z"]}

        _visual_stop_event.clear()
        _run_visual_lead(conn, wait_new_pose, get_plane_truth, _visual_stop_event)

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
    # A5: temas mandalını yeni koşu için temizle (bkz. start_chase).
    hasar_durumu.update(imha=False, menzil=None, t=None, temas=None)
    carpisma_state.sifirla()
    _visual_active = True
    threading.Thread(target=_visual_thread, daemon=True).start()
    return {"status": "success", "message": "Görsel güdüm (lead pursuit) başlatıldı."}


@app.post("/api/command/iris/stop_visual")
def stop_visual():
    global _visual_active
    _visual_active = False
    _visual_stop_event.set()
    return {"status": "success", "message": "Görsel güdüm durduruldu."}


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


def _chase_thread():
    """Chase altyapı thread'i: kalkış + hibrit güdüm (supervisor.run_hybrid)."""
    global _chase_active

    print("=" * 50)
    print("[CHASE] Chase Modu Başlıyor (hibrit: GPS yaklaşma ↔ görsel lead)")
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
            # x,y,z (m, NED) + attitude (rad; telemetri derece → radyan) + hız (m/s).
            # Kadraj güdümü attitude'u hedefin kameradaki izdüşümü için kullanır.
            return {"x": t["x"], "y": t["y"], "z": t["z"],
                    "roll": math.radians(t["roll"]),
                    "pitch": math.radians(t["pitch"]),
                    "yaw": math.radians(t["yaw"]),
                    "vx": t["vx"], "vy": t["vy"], "vz": t["vz"]}

        # ---- _chase_active'i stop_event'e bağla ----
        chase_stop = threading.Event()

        def watch_active():
            while _chase_active:
                time.sleep(0.1)
            chase_stop.set()

        watcher = threading.Thread(target=watch_active, daemon=True)
        watcher.start()

        # ---- ALGORİTMAYI ÇAĞIR ----
        # Varsayılan: HİBRİT (GPS yaklaşma ↔ görsel lead pursuit, supervisor
        # geçişli). AVCI_HYBRID=off → saf GPS yaklaşma (tek başına test için).
        if os.environ.get("AVCI_HYBRID", "on").lower() in ("off", "0"):
            print("[CHASE] Güdüm yasası: GPS-YAKLASMA (saf GPS, AVCI_HYBRID=off)")
            _run_gps_guidance(conn, get_plane, get_iris, chase_stop)
        else:
            print("[CHASE] Güdüm yasası: HİBRİT — GPS yaklaşma ↔ görsel lead pursuit")

            def get_plane_truth():
                t = telemetry_state["plane"]
                return {"x": t["x"], "y": t["y"], "z": t["z"]}

            _run_hybrid(conn, get_plane, get_iris, wait_new_pose,
                        get_plane_truth, chase_stop)

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
_pose_detector = None   # startup'ta yüklenir (AVCI_POSE=on, varsayılan açık)

def process_iris_frame(img, stamp=None, wall_recv=None):
    """Iris kamera karesini işle: Cessna/hedef tespiti + overlay + video parazit
    simülasyonu + MJPEG kodlama. Hem ROS2 (Gazebo Classic) hem gz-transport
    (Gazebo Harmonic) kamera kaynakları bu fonksiyonu çağırır.
    stamp: kare header.stamp (s, sim saati) — IBVS dt hesabı bunu kullanır;
    wall_recv: karenin geliş duvar anı (time.time) — bayat kare ölçümü."""
    # ---- HEDEF TESPİT (YOLO) + OVERLAY ----
    # İki model de TEMİZ kare üzerinde çıkarım yapar; overlay'ler sonra çizilir
    # (detection kutusu çizilmiş kare pose'a girerse çıkarımı bozar).
    det = pose = None
    if _yolo_detector is not None:
        try:
            det = _yolo_detector.detect_talon(img)
            set_detection(det)
        except Exception as e:
            print(f"[GCS] YOLO tespit hatası: {e}")
    # Pose YALNIZ detection kutu bulduğunda, kutunun çevresindeki kropta çalışır
    # (boşuna yük binmez, sahnenin kalanına yanlış nokta atamaz).
    if _pose_detector is not None:
        try:
            pose = (_pose_detector.detect_pose_in_bbox(img, det["bbox"])
                    if det is not None else None)
            set_pose_detection(pose, stamp=stamp, wall_recv=wall_recv)
        except Exception as e:
            print(f"[GCS] YOLO poz hatası: {e}")
    if _yolo_detector is not None:
        img = _yolo_detector.draw_overlay(img, det)
    if _pose_detector is not None:
        img = _pose_detector.draw_overlay(img, pose)

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
            wall_recv = time.time()
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            process_iris_frame(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR),
                               stamp=stamp, wall_recv=wall_recv)
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
            stamp = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9
            process_iris_frame(self.bridge.imgmsg_to_cv2(data, "bgr8"),
                               stamp=stamp, wall_recv=time.time())
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
# MAVLINK TELEMETRİ (14550=plane/ana GCS broadcast, 14541=iris)
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
        telemetry_state[vehicle_name]["mode"] = msg.custom_mode


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
            elif sysid_is_plane.get(sys_id) is False:
                # QUADROTOR (avcı drone) — 14550'den de yayın var (start_harmonic.sh
                # her iki SITL'i buraya da --out ediyor).
                #
                # NEDEN: 14541'i aynı anda TEK program dinleyebiliyor. Güdüm
                # senaryosu çalışırken iris telem worker'ı susuyor (start/stop_iris_telem)
                # ve portu güdüm alıyor; run_visual_lead veriyi kendi _ArasState'ine
                # çekip telemetry_state'e HİÇ yazmıyor. Sonuç: arayüz son değerde
                # DONUYOR — kalkıştan önceki spawn irtifası (~0.19 m) ekranda kalıyor,
                # drone 50 m'ye çıksa bile. Değerler sıfırlanmıyor, donuyor.
                #
                # Bu dal İKİNCİ ve BAĞIMSIZ bir kaynak: güdüm 14541'i tutsa da
                # arayüz 14550'den canlı kalır. Güdüm koduna dokunmaz.
                #
                # `is False` şart — `.get()` None dönerse sysid henüz HEARTBEAT ile
                # tanınmamıştır; tanınmayan paketi iris sanıp yazmayalım. (sysid 255
                # = GCS/mavproxy zaten sözlüğe hiç girmiyor.)
                _process_mavlink_msg(msg, "iris")


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
    # Hasar modülü: çarpışma menzilini izler, temasta hedefi imha eder.
    # Güdümden bağımsız — hangi faz çalışırsa çalışsın (veya hiçbiri) izler.
    threading.Thread(target=_hasar_izleyici, daemon=True).start()
    print(f"[GCS] Hasar modülü {'aktif' if _HASAR_AKTIF else 'KAPALI'} "
          f"— GERÇEK Gazebo teması (yakınlık eşiği YOK; AVCI_HASAR=0 kapatır)")
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
    # YOLO-pose modelini yükle (detection'ın YANINDA çalışır; AVCI_POSE=off kapatır)
    if os.environ.get("AVCI_POSE", "on").lower() not in ("off", "0"):
        global _pose_detector
        try:
            from vision import pose_detector as _pdet
            _pdet.load()                         # ağırlık + CUDA warmup
            _pose_detector = _pdet
            print("[GCS] YOLO pose hazır (avci_pose.pt)")
        except Exception as e:
            print(f"[GCS] YOLO pose yüklenemedi ({e}) — pose kapalı")

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