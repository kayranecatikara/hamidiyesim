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
try:
    import rclpy
    from rclpy.node import Node as RosNode
    from sensor_msgs.msg import Image as RosImage
    from cv_bridge import CvBridge
    _ROS2_VAR = True
except ImportError:
    _ROS2_VAR = False
    RosNode = object
    RosImage = None
    CvBridge = None
from pymavlink import mavutil
import uvicorn

# Cessna renk tabanlı tespit
from vision.detection_state import (set_detection, set_frame_detection,
                                    set_tracks, wait_new_frame, get_detection,
                                    get_frame_detection)
from control import carpisma_state       # A5 — gerçek temas, güdümle ortak durum
# YOLO detector (vision/detector.py) opsiyonel — startup'ta yüklenir (_yolo_detector).

# Sim gerçek-poz köprüsü (gz-transport; AVCI_TRUTH=off kapatır). Güdüm eski
# haline döndüğü için karar mekanizmalarına BAĞLI DEĞİL — yalnız gözlem:
# [TRUTH] menzil logu + FİZİKSEL TEMAS kanıt satırı terminale düşer.
from control import sim_truth

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
# ── SALT-OKUNUR TELEMETRİ ALANLARI (2026-08-13) ────────────────────────
# batt_*, gps_fix, gps_sat, gps_eph ve mode_ms alanları SYS_STATUS /
# GPS_RAW_INT mesajlarından doldurulur. Bu mesajlar zaten UDP soketine
# geliyordu ama recv_match filtresinde eleniyordu; yani veri vardı, okunmuyordu.
#
# NEDEN EKLENDİ: operatör arayüzü batarya voltajını, uydu sayısını ve GPS fix
# tipini gösteremediği için bu üçünü SİMÜLE etiketiyle uydurmak zorunda
# kalıyordu. Artık gerçek değer yayınlanıyor.
#
# RİSK: yok. Yalnızca OKUMA yapılır; güdüm, komut akışı ve MAVLink gönderimi
# bu değişiklikten etkilenmez. Mesaj gelmezse alanlar None kalır ve arayüz
# "VERİ YOK" gösterir — sıfır göstermez.
_TELEM_BOS = {
    "x": 0, "y": 0, "z": 0, "vx": 0, "vy": 0, "vz": 0, "speed": 0,
    "roll": 0, "pitch": 0, "yaw": 0, "armed": False,
    "lat": 0.0, "lon": 0.0, "alt_amsl": 0.0,
    # SYS_STATUS — mesaj gelmeden None
    "batt_v": None,        # volt
    "batt_a": None,        # amper
    "batt_pct": None,      # % (ArduPilot kapasite tanımlıysa; değilse -1 → None)
    # GPS_RAW_INT — mesaj gelmeden None
    "gps_fix": None,       # 0 yok · 1 no-fix · 2 2B · 3 3B · 4 DGPS · 5/6 RTK
    "gps_sat": None,       # görünür uydu
    "gps_eph": None,       # yatay seyreltme (HDOP)
}

telemetry_state = {
    "iris":  dict(_TELEM_BOS),
    "plane": dict(_TELEM_BOS),
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
# circle_xl/l/s: daire ÇAPI varyantları (bkz. run_plane_scenario.DAIRE_CAPLARI) —
# iç daire nişanını farklı yarıçaplarda sınamak için. Bu dalda arayüzde
# "Daire Çapı" butonları VAR (index.html #cap); kayramin_super_gudumu'nda yok.
# duz: süresiz düz uçuş — düz referans ölçümü için, butonu yok:
#   curl -X POST localhost:8000/api/command/plane/scenario/duz
_SCENARIO_NAMES = ("duz", "square", "circle", "aggressive",
                   "circle_xl", "circle_l", "circle_s")


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
    # Geçiş sayacı GÖREV başına sıfırlanır (run_hybrid'de değil — mod seçici
    # döngüsü onu defalarca çağırıyor, orada sıfırlamak sayacı hep 0 tutuyordu).
    _supervisor_mod.status.update(gecis_sayisi=0)

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


def _gercek_menzil():
    """Araçlar arası GERÇEK mesafe — (metre|None, kaynak).

    Önce sim_truth (iki araç TEK gz mesajından, zaman hizalı), yoksa telemetri
    farkı. visual_lead._menzil_olc ile AYNI öncelik: arayüzdeki sayı ile logdaki
    sayı aynı kaynaktan gelsin. Telemetri farkı iki ayrı akışın zaman hizasız
    farkıdır ve loglarda karelerin %37'sinde donuk çıkıyordu.
    """
    m = sim_truth.menzil()
    if m is not None:
        return m, "gz"
    plane, iris = telemetry_state["plane"], telemetry_state["iris"]
    if plane["x"] == 0 and plane["y"] == 0 and plane["z"] == 0:
        return None, None
    return math.sqrt((plane["x"] - iris["x"]) ** 2
                     + (plane["y"] - iris["y"]) ** 2
                     + (plane["z"] - iris["z"]) ** 2), "telem"


@app.get("/api/chase_status")
def chase_status():
    """Frontend için chase durumu."""
    if not _chase_active:
        return {"active": False, "distance": 0, "mode": _guidance_mode}
    # Mesafe telemetriden DEĞİL _gercek_menzil()'den: zaman hizalı gz önce,
    # telemetri yalnız yedek (telemetri farkı loglarda %37 donuk çıkıyordu).
    dist, kaynak = _gercek_menzil()
    resp = {"active": True, "distance": round(dist, 1) if dist is not None else None,
            "menzil_kaynak": kaynak, "mode": _guidance_mode}
    # GPS-YAKLASMA yasasının canlı durumu (ARAMA/KILIT/DROPOUT + handoff)
    resp["guidance"] = dict(_gps_guidance_mod.status)
    resp["supervisor"] = dict(_supervisor_mod.status)
    return resp


# ── GÜDÜM MODU (UI seçimi): gps | visual | hybrid ──
# gps    : görsel temas sağlansa bile devredilmez, hep GPS güdümü.
# visual : yalnız görsel güdüm; temas koparsa GPS'e DÖNÜLMEZ, araç hover'da
#          durur ve yeni görsel kilit bekler (kullanıcı kararı, 2026-08-03).
# hybrid : mevcut supervisor davranışı (GPS ↔ görsel geçişli) — varsayılan.
# Görev sırasında da değiştirilebilir: chase thread'i aktif fazı durdurup
# seçilen modu kurar. AVCI_HYBRID=off eski bayrağı "gps" varsayılanına eşlenir.
_GECERLI_MODLAR = ("gps", "visual", "hybrid")
_guidance_mode = ("gps" if os.environ.get("AVCI_HYBRID", "on").lower()
                  in ("off", "0") else "hybrid")


class GuidanceModeCmd(BaseModel):
    mode: str


@app.post("/api/guidance_mode")
def set_guidance_mode(cmd: GuidanceModeCmd):
    global _guidance_mode
    if cmd.mode not in _GECERLI_MODLAR:
        return {"status": "error", "message": f"Geçersiz mod: {cmd.mode}"}
    if cmd.mode != _guidance_mode:
        _guidance_mode = cmd.mode
        print(f"[CHASE] Güdüm modu seçildi: {_guidance_mode.upper()}")
    return {"status": "success", "mode": _guidance_mode}


@app.get("/api/guidance_mode")
def get_guidance_mode():
    return {"mode": _guidance_mode}


# ═══════════════════════════════════════════════════════════════════════
# GÜDÜM ÖZELLİKLERİ — CANLI AÇ/KAPA (kullanıcı kuralı, 2026-08-10)
# ═══════════════════════════════════════════════════════════════════════
# NEDEN VAR: her özelliği env değişkeniyle denemek 5 terminali baştan
# kurmayı gerektiriyordu ve kullanıcı farkı ANLIK göremiyordu. Artık her
# davranış anahtarı panelden uçuş SIRASINDA açılıp kapanabiliyor.
#
# ⚠ NASIL ÇALIŞIYOR: bbox_ibvs.Cfg bir SINIF; güdüm döngüsü her karede
# cfg.<ALAN> okuyor. Sınıf niteliğini burada değiştirmek bir sonraki
# kareden itibaren geçerli olur — yeniden başlatma gerekmez.
#
# ⚠ YENİ ÖZELLİK EKLEYEN BUNU DA GÜNCELLER (CLAUDE.md §5): eklenen her
# AVCI_* davranış anahtarının buraya bir satırı olacak, yoksa panelde
# görünmez ve kullanıcı deneyemez.
from control.guidance import bbox_ibvs as _ibvs_mod           # noqa: E402

# ad → (Cfg alanı, tip, etiket, açıklama, env anahtarı, açık değeri)
_OZELLIKLER = {
    "t1a_yatay_telafi": (
        "ROLL_TELAFI", "bool", "T1a · Yatay roll telafisi",
        "Kamera azimutunu araç duruşuyla seviye çerçevesine döndürür. "
        "Manevrada yatay salınımın kök nedeni.", "AVCI_IBVS_ROLL", True),
    "kapanma_olcek": (
        "KAPANMA", "bool", "Dikey komut kapanma hızıyla ölçeklenir",
        "vz = −ṙ·tan(elev). Kapalıyken drone hızıyla ölçeklenir ve son anda "
        "hedefin üstünden geçer.", "AVCI_IBVS_KAPANMA", True),
    "m3_erken_lead": (
        "LEAD_ERKEN", "bool", "M3 · Erken lead (terminal kapısı kalkar)",
        "Lead'i yalnız son 6 m yerine kutu olan her karede uygular. "
        "10 uçuşta nötr çıktı.", "AVCI_IBVS_LEAD_ERKEN", True),
    "o1_kacis_telafisi": (
        "KACIS_KD", "kazanc", "Ö1 · Kaçış telafisi",
        "Hedef uzaklaşırken (ṙ<0) seyir hızını anında artırır. Yalnız "
        "hızlandırma yönü — fren yapmaz.", "AVCI_IBVS_KD", 1.0),
}


class OzellikCmd(BaseModel):
    ad: str
    acik: bool


def _ozellik_durumu():
    d = []
    for ad, (alan, tip, etiket, aciklama, env, acik_deger) in _OZELLIKLER.items():
        v = getattr(_ibvs_mod.Cfg, alan)
        acik = bool(v) if tip == "bool" else (float(v) > 0.0)
        d.append({"ad": ad, "etiket": etiket, "aciklama": aciklama,
                  "env": env, "acik": acik,
                  "deger": (v if tip != "bool" else None)})
    return d


@app.get("/api/gudum_ozellikleri")
def get_gudum_ozellikleri():
    return {"ozellikler": _ozellik_durumu()}


@app.post("/api/gudum_ozellikleri")
def set_gudum_ozellik(cmd: OzellikCmd):
    if cmd.ad not in _OZELLIKLER:
        return {"status": "error", "message": f"Bilinmeyen özellik: {cmd.ad}"}
    alan, tip, etiket, _a, _e, acik_deger = _OZELLIKLER[cmd.ad]
    yeni = (cmd.acik if tip == "bool"
            else (float(acik_deger) if cmd.acik else 0.0))
    setattr(_ibvs_mod.Cfg, alan, yeni)
    print(f"[ÖZELLİK] {etiket}: {'AÇIK' if cmd.acik else 'kapalı'} "
          f"(Cfg.{alan} = {yeni})")
    return {"status": "success", "ozellikler": _ozellik_durumu()}


# -----------------------------------------------------------------------
# GÖRÜŞ & FAZ PANELİ — /api/telemetry/pnp
# -----------------------------------------------------------------------
# 2026-08-01: Bu endpoint ESKİDEN SAHTEYDİ. Ground-truth telemetriye yapay
# gauss gürültüsü + sabit 8 m ofset ekleyip "PnP çıkışı" diye sunuyordu
# ("rapor fotoğrafları için" notuyla). Görüş hattına hiç dokunmuyordu, o
# yüzden panele bakarak görsel fazın ne zaman devreye gireceği anlaşılamıyordu
# ve saatlerce açık kalan GCS'te 6000 m gibi değerler görünüyordu.
#
# Yerine GERÇEK veri: kameranın ürettiği kestirim ile ground-truth YAN YANA,
# ikisi ayrı ayrı etiketli ("cevap anahtarı" ilkesi: kameranın kestirimi ile
# ground-truth yan yana, fark = algı hatası).
# Ayrıca GPS→görsel geçişinin İKİ kapısı da canlı gösteriliyor, çünkü asıl
# sorulan soru bu: "neden hâlâ görsel faza geçmedi?"
#   • görsel kilit: son KILIT_PENCERE karenin kaçında tespit conf ≥ KILIT_CONF_MIN
#   • menzil kapısı: yatay mesafe d_h < GATE_MENZIL mi
# Hangisinin bağlayıcı olduğu panelden tek bakışta görünür.

def _gorus_menzil_kestirimi(det, iris_att):
    """Detection kutusundan ölçek tabanlı menzil (m) — guidance_core ile AYNI
    formül. Kamera dışında hiçbir bilgi kullanmaz.

    2026-08-06: eski keypoint ölçeğinin (gövde+kanat projeksiyonu) yerini
    kutunun GENİŞLİĞİ aldı; kalibrasyon sabiti Cfg.BBOX_L_ETKIN_M.

    iris_att verilirse yükselti düzeltmesi de uygulanır (hedef seviyeli uçuyor
    varsayımı); yoksa ham ölçek kullanılır ve değer bir miktar iyimser çıkar.
    Dönüş: (menzil_m, olcek_guvenilir_mi) veya (None, False).
    """
    try:
        olcek = float(det["w"])
        if olcek < _LeadCfg.MIN_BBOX_PX:
            return None, False
        if iris_att is not None:
            # Kamera ışını → gövde → dünya: LOS yükselişi (eps) ile ölçek düzeltilir
            u = np.array([(det["cx"] - _geo.CX) / _geo.FX,
                          (det["cy"] - _geo.CY) / _geo.FY, 1.0])
            u = u / np.linalg.norm(u)
            u_g = _kamera_to_govde(u, math.radians(_LeadCfg.KAMERA_TILT_DEG))
            u_d = _govde_to_dunya(u_g, *iris_att)
            eps = math.asin(max(-1.0, min(1.0, -float(u_d[2]))))
            olcek = olcek / _yukselti_duzeltme(eps)
        # Kalite rampasının üstündeysek ölçek "güvenilir" sayılır (eski
        # kanat_gorunur bayrağının halefi — panelde aynı yeri doldurur).
        guvenilir = olcek >= _LeadCfg.OLCEK_KAPALI_PX
        return _geo.FX * _LeadCfg.BBOX_L_ETKIN_M / olcek, guvenilir
    except Exception:
        return None, False


@app.get("/api/telemetry/pnp")
def pnp_telemetry():
    """Görüş hattının GERÇEK çıktısı + faz kapıları + ground-truth kıyası.

    Buradaki hiçbir değer güdüme girmez; panel yalnız gözlem içindir.
    """
    det = get_detection()
    iris = telemetry_state["iris"]
    plane = telemetry_state["plane"]

    iris_att = None
    if any(iris.get(k) for k in ("roll", "pitch", "yaw")):
        iris_att = (iris.get("roll", 0.0), iris.get("pitch", 0.0),
                    iris.get("yaw", 0.0))

    # ── Kameranın kendi menzil kestirimi (bbox ölçeği tabanlı) ──
    gorus_menzil, olcek_ok = (None, False)
    if det is not None:
        gorus_menzil, olcek_ok = _gorus_menzil_kestirimi(det, iris_att)

    # ── Ground-truth (SİM KOLAYLIĞI — gerçek harekâtta yok, etiketli göster) ──
    # Kaynak _gercek_menzil ile ortak: zaman hizalı gz önce, telemetri yedek.
    telem_var = not (plane["x"] == 0 and plane["y"] == 0 and plane["z"] == 0)
    gercek_menzil, gercek_menzil_kaynak = _gercek_menzil()

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
        engel = "KİLİT + MENZİL"
    elif not poz_kapi_ok:
        engel = "GÖRSEL KİLİT"
    elif not menzil_kapi_ok:
        engel = "MENZİL KAPISI"
    else:
        engel = "—"

    return {
        "active": det is not None or telem_var,
        "faz": sup.get("faz", "GPS"),
        "gecis_sayisi": sup.get("gecis_sayisi", 0),
        # görüş hattı
        "tespit_var": det is not None,
        "tespit_conf": round(float(det["conf"]), 2) if det else None,
        "kanat_gorunur": bool(olcek_ok),
        "gorus_menzil": round(gorus_menzil, 1) if gorus_menzil else None,
        # ground-truth (etiketli)
        "gercek_menzil": round(gercek_menzil, 1) if gercek_menzil else None,
        "gercek_menzil_kaynak": gercek_menzil_kaynak,
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
from control.guidance.gps_guidance import run_gps_guidance as _run_gps_guidance_eski

# ── GPS GÜDÜM YASASI SEÇİMİ (gps_kararli_hal dalından, 2026-08-04) ──
# İki yasa YAN YANA durur; eskisi silinmez ki tek değişkenle A/B yapılabilsin.
#   AVCI_GPS_GUDUM=istasyon  (VARSAYILAN) → uçuşta doğrulanmış mevcut yasa
#   AVCI_GPS_GUDUM=frpn                   → FRPN + IMM (tezgâhta üstün)
# Varsayılanın "istasyon" olması ÖLÇÜLMÜŞ bir karardır: aynı senaryoda
# oturmuş menzil FRPN 31.1 m, istasyon yasası (KD_H=0.60) 29.4 m.
_GPS_GUDUM = os.environ.get("AVCI_GPS_GUDUM", "istasyon").lower()
if _GPS_GUDUM == "frpn":
    from control.guidance import frpn_guidance as _frpn_mod
    from control.guidance.frpn_guidance import run_frpn_guidance as _run_gps_guidance
    _gps_guidance_mod = _frpn_mod          # status'u supervisor/panel buradan okur
    print("[GCS] GPS GÜDÜMÜ: FRPN (IMM kestirici + menzile bağlı istasyon) "
          "— uçuşta eski yasadan İYİ ÇIKMADI; dönmek için AVCI_GPS_GUDUM=istasyon")
else:
    _run_gps_guidance = _run_gps_guidance_eski
from control.guidance.bbox_ibvs import run_bbox_ibvs as _run_bbox_ibvs, Cfg as _IbvsCfg
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


def _kayit_dongusu():
    """Saniyede 1: kamera karesi + tam durum satırı. Kayıt durana dek sürer."""
    import csv as _csv
    d = _kayit["dizin"]
    kare_dizin = os.path.join(d, "frames")
    os.makedirs(kare_dizin, exist_ok=True)
    f = open(os.path.join(d, "kayit.csv"), "w", newline="")
    w = _csv.DictWriter(f, fieldnames=[
        "kare", "t", "gecen_s", "gudum_modu", "faz", "mesafe_telem",
        "mesafe_gercek", "tespit_conf", "manuel_aktif", "manuel_aileron",
        "manuel_elevator", "manuel_throttle", "hedef_gaz", "senaryo",
        "plane_x", "plane_y", "plane_z", "plane_hiz",
        "iris_x", "iris_y", "iris_z", "iris_hiz", "imha"])
    w.writeheader()
    _kayit["csv"], _kayit["dosya"] = w, f
    n = 0
    t0 = _kayit["t0"]
    while _kayit["aktif"]:
        dongu_bas = time.time()
        n += 1
        kare = latest_frames["iris"]["data"]
        if kare:
            with open(os.path.join(kare_dizin, f"f{n:04d}.jpg"), "wb") as g:
                g.write(kare)
        det = get_detection()
        p, i = telemetry_state["plane"], telemetry_state["iris"]
        try:
            m_ger = sim_truth.menzil()
        except Exception:
            m_ger = None
        w.writerow({
            "kare": n, "t": round(dongu_bas, 2),
            "gecen_s": round(dongu_bas - t0, 1),
            "gudum_modu": _guidance_mode,
            "faz": _supervisor_mod.status.get("faz"),
            "mesafe_telem": round(math.sqrt(
                (p["x"] - i["x"]) ** 2 + (p["y"] - i["y"]) ** 2
                + (p["z"] - i["z"]) ** 2), 2),
            "mesafe_gercek": round(m_ger, 2) if m_ger else None,
            "tespit_conf": round(det["conf"], 3) if det else None,
            # ⚠ MANUEL KUMANDA KONUMLARI: kullanıcı manuel uçarken videoyu
            # yorumlayabilmek için şart ("o an ne yapıyordun" sorusu).
            "manuel_aktif": int(_manual_active),
            "manuel_aileron": _manual_aileron,
            "manuel_elevator": _manual_elevator,
            "manuel_throttle": _manual_throttle,
            "hedef_gaz": _plane_throttle,
            "senaryo": _scenario_name,
            "plane_x": p["x"], "plane_y": p["y"], "plane_z": p["z"],
            "plane_hiz": p.get("speed"),
            "iris_x": i["x"], "iris_y": i["y"], "iris_z": i["z"],
            "iris_hiz": i.get("speed"),
            "imha": int(bool(hasar_durumu.get("imha"))),
        })
        f.flush()
        _kayit["kare"] = n
        kalan = 1.0 - (time.time() - dongu_bas)
        if kalan > 0:
            time.sleep(kalan)
    f.close()
    print(f"[KAYIT] durdu — {n} kare + kayit.csv → {d}")


@app.post("/api/kayit/basla")
def kayit_basla():
    with _kayit_kilit:
        if _kayit["aktif"]:
            return {"status": "error", "message": "Kayıt zaten sürüyor",
                    "dizin": _kayit["dizin"]}
        d = os.path.join(_KAYIT_KOK, time.strftime("ucus_%Y%m%d_%H%M%S"))
        os.makedirs(d, exist_ok=True)
        _kayit.update(aktif=True, dizin=d, kare=0, t0=time.time())
        t = threading.Thread(target=_kayit_dongusu, daemon=True)
        _kayit["thread"] = t
        t.start()
    print(f"[KAYIT] başladı → {d}")
    return {"status": "success", "dizin": d}


@app.post("/api/kayit/dur")
def kayit_dur():
    with _kayit_kilit:
        if not _kayit["aktif"]:
            return {"status": "error", "message": "Kayıt zaten kapalı"}
        _kayit["aktif"] = False
        d, n = _kayit["dizin"], _kayit["kare"]
    return {"status": "success", "dizin": d, "kare": n}


@app.get("/api/kayit/durum")
def kayit_durum():
    return {"aktif": _kayit["aktif"], "dizin": _kayit["dizin"],
            "kare": _kayit["kare"],
            "gecen_s": round(time.time() - _kayit["t0"], 1)
            if _kayit["aktif"] and _kayit["t0"] else 0}


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
#  GÖRSEL GÜDÜM (BBOX IBVS) — izole hat:
#  detection kutusu → saf takip + azimut-oranı lead → hız + yaw komutu.
#  GPS chase'i BOZMAZ; ayrı endpoint. Döngü kameraya kilitli (olay güdümlü).
# ══════════════════════════════════════════════════════════
_visual_active = False
_visual_stop_event = threading.Event()


def _gt_bbox_girdi():
    """GT MODU (AVCI_GT_ROT=on) algı girdisi — YOLO detection'ın YERİNE.

    İki aracın Gazebo world (ENU) pozu TEK mesajdan (sim_truth.pozlar, zaman
    hizalı) → geometry.bbox_gt_goruntu ile nişan noktası + gerçek kutu boyutu +
    menzil. geometry.py ile aynı çerçeve/RPY sözleşmesini kullanır, dönüşüm
    gerekmez. GT akışı bayat/kapalıysa None (güdüm 'tespit yok' gibi davranır).

    ⚠ Simülasyona özgü: gerçek harekâtta hedefin pozu bilinmez.
    """
    p = sim_truth.pozlar()
    if p is None:
        return None
    ir, hd = p["iris"], p["plane"]
    h_rpy = _geo.quat_to_rpy(*hd["quat"])
    i_rpy = _geo.quat_to_rpy(*ir["quat"])
    r = _geo.bbox_gt_goruntu(hd["pos"], h_rpy, ir["pos"], i_rpy)
    if r is not None:
        # Yalnız ÖLÇÜM/teşhis için taşınır; güdüm bu alanları okumaz.
        r["hedef_rpy"] = h_rpy
        r["iris_rpy"] = i_rpy
        r["hedef_pos"] = hd["pos"]
        r["iris_pos"] = ir["pos"]
    return r


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

        def get_iris():
            t = telemetry_state["iris"]
            return {"x": t["x"], "y": t["y"], "z": t["z"],
                    "roll": math.radians(t.get("roll", 0.0)),
                    "pitch": math.radians(t.get("pitch", 0.0)),
                    "yaw": math.radians(t.get("yaw", 0.0)),
                    "vx": t.get("vx", 0.0), "vy": t.get("vy", 0.0),
                    "vz": t.get("vz", 0.0)}

        _visual_stop_event.clear()
        sim_truth.temas_sifirla()
        # ⚠ 2026-08-10: eski `run_visual_lead` ARŞİVLENDİ; bu uç da aktif
        # yasaya (bbox IBVS) bağlandı. GPS fazı koşmadığı için taşıyıcı sıfır.
        _run_bbox_ibvs(conn, get_iris, wait_new_frame, _visual_stop_event,
                       cfg=_IbvsCfg, get_temas=sim_truth.temas)

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
    _supervisor_mod.status.update(gecis_sayisi=0)   # görev başı (bkz. start_chase)
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
    # Kuyruğun TAMAMI boşaltılır (eski sınır 10 mesajdı). Neden: görsel faz
    # sırasında bu bağlantıdan kimse okumaz, mesajlar OS kuyruğunda birikir;
    # GPS'e dönüşte 10'arlı okuma birikimi ~50 s boyunca "geçmişi oynatıyordu"
    # — iris konumu 9 s bayat kalıp d_h'ye 150+ m HAYALET mesafe yazdırdı
    # (2026-08-03 12:16 uçuşu, gerçek menzil 6-9 m iken d_h=158.9). Tavan
    # yalnız güvenlik içindir; tek çağrıda tüm birikim tüketilip en taze
    # mesajda durulur.
    # SYS_STATUS / GPS_RAW_INT bu listeye 2026-08-13'te eklendi. Güdüm
    # fazındayken iris telemetri thread'i DURDURULUR ve iris'i okuyan tek yer
    # burasıdır; listeye eklenmeselerdi batarya ve uydu sayısı tam da en
    # kritik anda donardı. İkisi de ~1-2 Hz'lik salt-okunur mesajdır, boşaltma
    # döngüsünün davranışını değiştirmez.
    for _ in range(2000):
        msg = conn.recv_match(
            type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT',
                  'SYS_STATUS', 'GPS_RAW_INT'],
            blocking=False
        )
        if not msg:
            break
        _process_mavlink_msg(msg, "iris")


def _gorsel_tek_faz(conn, get_iris, stop_event):
    """GÖRSEL mod (UI seçimi): supervisor'suz tek-faz görsel güdüm.

    Kilit oturana dek (KILIT_N ardışık güvenli tespit — hibritteki sayaçla aynı)
    araç hover'da bekler; sonra bbox IBVS koşar. Temas koparsa GPS'e DÖNÜLMEZ
    (kullanıcı kararı): araç hover'da durur, yeni görsel kilit bekler.

    ⚠ 2026-08-10 DÜZELTME: burası `run_visual_lead` çağırıyordu — yani panelde
    GÖRSEL'e basan kullanıcı, hibritte UÇMAYAN eski yasayı uçuruyordu. İki mod
    artık AYNI görsel yasayı (bbox IBVS) kullanır; aralarındaki tek fark temas
    kopunca GPS'e dönülüp dönülmediğidir.
    Dönüş: vuruldu mu (bool)."""
    from control.guidance.common import send_velocity
    from control.guidance.supervisor import SupCfg
    while not stop_event.is_set():
        # ── kilit bekleme: hover'da, tespit akışını sayarak ──
        _supervisor_mod.status.update(faz="VISUAL", son_sebep="kilit-bekleniyor")
        sayac, son_seq = 0, 0
        while not stop_event.is_set():
            kayit = wait_new_frame(son_seq, timeout=0.5)
            _read_iris_telem_from_conn(conn)          # telemetri/UI taze kalsın
            send_velocity(conn, 0.0, 0.0, 0.0,
                          math.radians(telemetry_state["iris"]["yaw"]))
            if kayit is None:
                continue
            son_seq = kayit["seq"]
            kdet = kayit["det"]
            if kdet is not None and kdet.get("conf", 0.0) >= SupCfg.KILIT_CONF_MIN:
                sayac += 1
            else:
                sayac = 0
            _supervisor_mod.status["kilit_sayac"] = sayac
            if sayac >= SupCfg.KILIT_N:
                break
        if stop_event.is_set():
            return False
        # ── görsel faz ──
        _supervisor_mod.status["gecis_sayisi"] += 1
        print(f"[CHASE] ✓ GÖRSEL KİLİT — bbox IBVS başlıyor "
              f"(geçiş #{_supervisor_mod.status['gecis_sayisi']}, GÖRSEL mod)")
        # DONDURULMUŞ TAŞIYICI: hibritteki ile aynı — hedefin son GPS hız
        # kestirimi SAYI olarak geçilir (D0: görsel döngü canlı GPS görmez).
        # GÖRSEL modda GPS fazı hiç koşmadıysa bu sıfır olur; o zaman yasa
        # hız integralini sıfırdan kurar (kabul edilen davranış).
        _ff = (_gps_guidance_mod.status.get("tgt_vx") or 0.0,
               _gps_guidance_mod.status.get("tgt_vy") or 0.0,
               _gps_guidance_mod.status.get("tgt_vz") or 0.0)
        sebep = _run_bbox_ibvs(conn, get_iris, wait_new_frame, stop_event,
                               cfg=_IbvsCfg, kayip_kare_esik=SupCfg.KAYIP_M,
                               ff_hiz=_ff, get_temas=sim_truth.temas)
        _supervisor_mod.status["son_sebep"] = sebep
        if sebep == "vuruldu":
            _supervisor_mod.status["faz"] = "VURULDU"
            print("[CHASE] ✓✓ HEDEF VURULDU (GÖRSEL mod).")
            return True
        if sebep == "kayip":
            print("[CHASE] Görsel temas koptu — GPS'e DÖNÜLMÜYOR (GÖRSEL mod); "
                  "araç hover'da, yeni kilit bekleniyor.")
            continue
        return False                                  # durduruldu / mod değişti
    return False


def _chase_thread():
    """Chase altyapı thread'i: kalkış + seçilen güdüm modu (gps/visual/hybrid)."""
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

        # ---- ALGORİTMAYI ÇAĞIR (UI'dan seçilen güdüm modu) ----
        # gps: hep GPS. visual: yalnız görsel, kopunca DUR. hybrid: supervisor.
        # Mod görev sırasında değişirse mod_izci aktif fazı kırar, döngü
        # yeni modu kurar. Vuruş her modda görevi bitirir.
        def get_plane_truth():
            t = telemetry_state["plane"]
            return {"x": t["x"], "y": t["y"], "z": t["z"]}

        sim_truth.temas_sifirla()          # temas latch'i görev başına taze
        vuruldu = False
        while not chase_stop.is_set() and not vuruldu:
            mod = _guidance_mode
            mod_stop = threading.Event()

            sim_truth.temas_sifirla()      # temas latch'i her faz başına taze

            def mod_izci(m=mod, ev=mod_stop):
                # mod değişince ya da chase durunca aktif fazı kır
                while not ev.is_set():
                    if chase_stop.is_set() or _guidance_mode != m:
                        ev.set()
                        return
                    time.sleep(0.2)
            threading.Thread(target=mod_izci, daemon=True).start()

            if mod == "gps":
                print("[CHASE] Güdüm modu: GPS — görsel temas sağlansa da devredilmez")
                _supervisor_mod.status.update(faz="GPS", son_sebep=None)
                _run_gps_guidance(conn, get_plane, get_iris, mod_stop)
            elif mod == "visual":
                print("[CHASE] Güdüm modu: GÖRSEL — kilit bekleniyor; temas "
                      "koparsa GPS'e dönülmez, araç durur")
                vuruldu = _gorsel_tek_faz(conn, get_iris, mod_stop)
            else:
                print("[CHASE] Güdüm modu: HİBRİT — GPS yaklaşma ↔ görsel lead pursuit")
                _run_hybrid(conn, get_plane, get_iris, wait_new_frame,
                            get_plane_truth, mod_stop,
                            get_temas=sim_truth.temas,
                            get_menzil=sim_truth.menzil,
                            get_gt=_gt_bbox_girdi)   # GT modu (AVCI_GT_ROT=on)
                vuruldu = (_supervisor_mod.status.get("faz") == "VURULDU")
            mod_stop.set()                 # izci thread'i sonlandır
            # TEŞHİS (2026-08-10): görev döngüsünün neden dönüp durduğu ya da
            # neden çıktığı hiçbir yere yazılmıyordu — bkz. TODO §0.
            print(f"[CHASE] mod fazı bitti (mod={mod}) — "
                  f"chase_stop={chase_stop.is_set()} vuruldu={vuruldu} "
                  f"supervisor.faz={_supervisor_mod.status.get('faz')} "
                  f"gps_bitis={_supervisor_mod.status.get('gps_faz_bitis')} "
                  f"gorsel_bitis={_supervisor_mod.status.get('gorsel_faz_bitis')}")

        # ---- DURDURMA → HOVER ----
        print(f"[CHASE] Görev döngüsü ÇIKTI — chase_stop={chase_stop.is_set()} "
              f"vuruldu={vuruldu}  → bundan sonra araca güdüm komutu GİTMEZ")
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
    "plane": {"data": None, "id": 0},
    # DIŞ GÖRÜŞ (chase) kameraları — SDF'teki iris_chase/talon_chase topic'leri.
    # Aracı DIŞARIDAN gösterirler; tespit/kilit hattına GİRMEZLER, ham geçerler.
    # Yalnız gz-transport (Harmonic) yolunda beslenir: ROS 2 (Classic) köprüsü
    # bu topic'leri yayınlamaz, o kurulumda akış boş kalır (arayüz "kaynak yok").
    "iris_chase":  {"data": None, "id": 0},
    "talon_chase": {"data": None, "id": 0},
}

# ══════════════════════════════════════════════════════════════════════
#  UÇUŞ KAYDI (arayüzdeki "KAYIT" butonu) — 2026-08-09, kullanıcı isteği
# ══════════════════════════════════════════════════════════════════════
# Kullanıcının MANUEL uçuşlarını da inceleyebilmek için: saniyede 1 kamera
# karesi + o anki tam durum (faz, mesafe, telemetri, GÜDÜM MODU ve MANUEL
# KUMANDA konumları) tek dizine yazılır. Otomatik testlerde bunu araç
# (tools/ucus_kaydi.py) yapıyordu; manuel uçuşta panelden tetiklenir.
# Kumanda konumları kayda giriyor çünkü "o anda ne yapıyordun" sorusunun
# cevabı olmadan manuel uçuş videosu yorumlanamıyor.
_kayit = {"aktif": False, "dizin": None, "kare": 0, "t0": None,
          "thread": None, "csv": None, "dosya": None}
_kayit_kilit = threading.Lock()
_KAYIT_KOK = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs", "kayit")

_yolo_detector = None   # startup'ta yüklenir (AVCI_DETECTOR=yolo, varsayılan açık)
_talon_tracker = None   # startup'ta yüklenir (AVCI_TRACKER=off kapatır; HybridSORT)
_target_lock   = None   # kilitli-ID politikası (AVCI_LOCK=off kapatır; tracker açıksa)
_tracker_mod   = None   # vision.tracker modülü (draw_tracks için)
_tracker_err   = False  # tekrarlayan tracker hatasında log seli önlenir
_lock_prev_id  = None   # kilit olay logu için önceki kilit ID'si
# Parazit modellere de uygulansın mı? 1 = tespit/tracker PARAZİTLİ kareyi
# görür (görüntü hattının gerçek dayanıklılık testi); 0 (varsayılan) = parazit
# yalnız operatör yayınına biner, modeller temiz kareyi görür (eski davranış).
_NOISE_PRE_DETECT = os.environ.get("AVCI_NOISE_PRE_DETECT", "0") == "1"
# (GT modu AVCI_GT_ROT bayrağı ardında; varsayılan KAPALI. Kapalıyken güdüm
# algısı YOLO detection kutusudur; açıkken Gazebo'nun gerçek pozundan üretilen
# kutu kullanılır — bkz. _gt_bbox_girdi ve guidance_core.Cfg.GT_ROT.)


def _apply_video_noise(img, lvl):
    """Video parazit simülasyonunun piksel kısmı (gauss + satır + blok + karartma;
    lvl>=0.999 tam karartma). JPEG kalite düşürme yayın kodlamasında kalır."""
    if lvl >= 0.999:
        return np.zeros_like(img)
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
    return img

def process_iris_frame(img, stamp=None, wall_recv=None):
    """Iris kamera karesini işle: Cessna/hedef tespiti + overlay + video parazit
    simülasyonu + MJPEG kodlama. Hem ROS2 (Gazebo Classic) hem gz-transport
    (Gazebo Harmonic) kamera kaynakları bu fonksiyonu çağırır.
    stamp: kare header.stamp (s, sim saati) — IBVS dt hesabı bunu kullanır;
    wall_recv: karenin geliş duvar anı (time.time) — bayat kare ölçümü."""
    # ---- PARAZİT (opsiyonel: TESPİTTEN ÖNCE) ----
    # AVCI_NOISE_PRE_DETECT=1 → modeller de parazitli kareyi görür (dayanıklılık
    # testi). Varsayılanda parazit yalnız yayın kodlamasında uygulanır (aşağıda).
    lvl = _video_noise_level
    if _NOISE_PRE_DETECT and lvl > 0.001:
        img = _apply_video_noise(img, lvl)
    # ---- HEDEF TESPİT (YOLO) + OVERLAY ----
    # Çıkarım overlay'siz kare üzerinde yapılır; overlay'ler sonra çizilir.
    det = tracks = dets_all = det_raw = lock = None
    if _yolo_detector is not None:
        try:
            if _talon_tracker is not None:
                # Tek YOLO çıkarımı: düşük eşikli TÜM tespitler tracker'a, en
                # güvenlisi (conf>=eşik) det_raw. NMS'te düşük skorlu kutu
                # yükseği asla bastıramaz → det_raw, detect_talon ile birebir
                # aynı; AVCI_TRACKER=off'ta eski yol aynen korunur.
                dets_all = _yolo_detector.detect_all(img)
                det_raw = _yolo_detector.best_det(dets_all)
            else:
                det = _yolo_detector.detect_talon(img)
        except Exception as e:
            print(f"[GCS] YOLO tespit hatası: {e}")
    # HybridSORT: kareler arası kimlik (ID) takibi. Tespitsiz karede de çağrılır
    # ki track'ler yaşlansın/Kalman köprü kursun (max_age).
    global _tracker_err
    if _talon_tracker is not None and dets_all is not None:
        try:
            tracks = _talon_tracker.update(dets_all, img)
            if _target_lock is not None:
                lock = _target_lock.step(tracks, det_raw)
                # Kilit olay logu: yalnız kilit KURULUNCA/DÜŞÜNCE tek satır
                global _lock_prev_id
                lid = None if lock is None else lock["id"]
                if lid != _lock_prev_id:
                    if lid is None:
                        print("[LOCK] kilit düştü (coast aşıldı / sıçrama / çelişki)")
                    else:
                        print(f"[LOCK] ID:{lid} kilitlendi "
                              f"(conf {lock['conf']:.2f}, "
                              f"{_target_lock.relock_sayisi}. kilitlenme)")
                    _lock_prev_id = lid
            _tracker_err = False
        except Exception as e:
            if not _tracker_err:
                print(f"[GCS] HybridSORT hatası (takip atlanıyor): {e}")
                _tracker_err = True
            tracks = lock = None
        set_tracks(tracks)
    # set_detection'a giden kutu (kilit politikası — GT'li deneyle doğrulandı):
    #  - kilit bu karede EŞLEŞMİŞSE (taze/BYTE) onun kutusu → anlık FP hedefi çalamaz
    #  - kilit COAST'taysa det=None → Kalman tahmini NİŞAN olarak kullanılmaz
    #    (deneyde uzun coast yalnız yanlış kutu ekledi)
    #  - kilit yoksa det_raw → hedef ediniminde eski davranış (gecikmesiz)
    if _yolo_detector is not None:
        if _talon_tracker is not None:
            if _target_lock is None:
                det = det_raw                          # AVCI_LOCK=off: eski seçim
            elif lock is not None and lock["kaynak"] == "eslesme":
                x1, y1, x2, y2 = (int(v) for v in lock["bbox"])
                det = {"cx": (x1 + x2) // 2, "cy": (y1 + y2) // 2,
                       "w": x2 - x1, "h": y2 - y1, "conf": lock["conf"],
                       "bbox": (x1, y1, x2, y2), "track_id": lock["id"]}
            elif lock is None:
                det = det_raw
            else:
                det = None                             # coast: nişan komutu yok
        set_detection(det)
    # KARE KÖPRÜSÜ — güdüm döngüsünün saati. HER karede çağrılır (det None
    # olsa bile): visual_lead kare bekler, çağrı atlanırsa döngü donar ve GT
    # modu da çalışmaz (2026-08-04'te tam bu olmuştu).
    set_frame_detection(det, stamp=stamp, wall_recv=wall_recv, lock=lock)
    if _yolo_detector is not None:
        img = _yolo_detector.draw_overlay(img, det)
    if tracks is not None and len(tracks) and _tracker_mod is not None:
        img = _tracker_mod.draw_tracks(img, tracks, line=1)
    if lock is not None and lock["kaynak"] == "tahmin":
        # Coast köprüsü görünür olsun: gri kesikli his — tahmin, nişan değil
        x1, y1, x2, y2 = (int(v) for v in lock["bbox"])
        cv2.rectangle(img, (x1, y1), (x2, y2), (160, 160, 160), 1)
        cv2.putText(img, f"ID:{lock['id']} tahmin({lock['coast']})",
                    (x1, max(y1 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (160, 160, 160), 1, cv2.LINE_AA)

    # ---- VIDEO PARAZİT (yayın) + MJPEG KODLAMA ----
    if not _NOISE_PRE_DETECT and lvl > 0.001:
        img = _apply_video_noise(img, lvl)         # eski davranış: yalnız yayına
    if 0.001 < lvl < 0.999:
        jpeg_q = max(5, int(90 - lvl * 85))
        _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])
    else:
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

    # ═══════════════════════════════════════════════════════════════════
    #  EN SON KARE KAZANIR (2026-08-05, gps_kararli_hal) — gecikme birikmesi
    # ═══════════════════════════════════════════════════════════════════
    # ESKİ HÂLİ: callback tüm işi SENKRON yapıyordu (YOLO + tracker + overlay
    # + JPEG). Ölçüldü: process_iris_frame medyan 21.8 ms, tepe 32.1 ms —
    # kamera 30 Hz yayın yapıyor, yani bütçe 33.3 ms. BOŞ makinede bile pay
    # neredeyse sıfır; gerçek uçuşta Gazebo render + Talon kamerası + güdüm
    # döngüsü + iki SITL aynı CPU'yu paylaşınca bütçe AŞILIYOR.
    # Bütçe aşılınca gz-transport kareleri KUYRUĞA alıyor ve kuyruk hiç
    # boşalmıyor → arayüz Gazebo'nun giderek gerisine düşüyor.
    # Belirti: "uçuş uzayınca gecikme birikiyor, hedefin hareketleri sonradan
    # geliyor". Yavaş tüketici + sınırsız kuyruk.
    # Desen izole edilip ölçüldü (30 Hz üretici, 40 ms işleme):
    #     eski desen : ilk çeyrek 80 ms → son çeyrek 579 ms (sürekli büyüyor)
    #     yeni desen : ilk çeyrek 19 ms → son çeyrek  19 ms (sabit)
    #
    # YENİ HÂLİ: callback yalnız EN SON kareyi saklayıp döner (~0.1 ms).
    # İşçi thread her turda en yeni kareyi alır; o sırada gelen ara kareler
    # DÜŞÜRÜLÜR. Gecikme en fazla BİR işleme çevrimi kadar olur, birikmez.
    # Görüntü akıcılığı düşebilir ama TAZE kalır — güdüm için doğru takas.
    #
    # ⚠ wall_recv artık karenin GAZEBO'DAN GELDİĞİ an (callback girişi), işleme
    # başlangıcı değil. Eskiden işleme anı yazılıyordu ve kuyrukta geçen süre
    # ÖLÇÜLEMİYORDU — loglar "29 ms gecikme" derken gerçek gecikme görünmüyordu.
    # Artık visual_lead'in bayat-kare kapısı gerçek gecikmeyi görür.
    kare_kutu = {"veri": None, "en": 0, "boy": 0, "stamp": 0.0, "wall": 0.0}
    kare_kilit = threading.Lock()
    kare_olay = threading.Event()
    sayac = {"gelen": 0, "islenen": 0, "dusen": 0, "t_log": time.time()}

    def cb(msg):
        # SADECE sakla ve dön. msg.data bytes'tır; referansı tutmak yeterli
        # (np.frombuffer işçi tarafında sıfır-kopya çalışır).
        try:
            with kare_kilit:
                if kare_kutu["veri"] is not None:
                    sayac["dusen"] += 1          # önceki kare hiç işlenmedi
                kare_kutu["veri"] = msg.data
                kare_kutu["en"] = msg.width
                kare_kutu["boy"] = msg.height
                kare_kutu["stamp"] = (msg.header.stamp.sec
                                      + msg.header.stamp.nsec * 1e-9)
                kare_kutu["wall"] = time.time()
                sayac["gelen"] += 1
            kare_olay.set()
        except Exception as e:
            print(f"[GCS GZ-CAM] cb hata: {e}")

    def isci():
        while True:
            kare_olay.wait(1.0)
            kare_olay.clear()
            with kare_kilit:
                veri = kare_kutu["veri"]
                en, boy = kare_kutu["en"], kare_kutu["boy"]
                stamp, wall = kare_kutu["stamp"], kare_kutu["wall"]
                kare_kutu["veri"] = None         # tüketildi
            if veri is None:
                continue
            try:
                arr = np.frombuffer(veri, dtype=np.uint8).reshape((boy, en, 3))
                process_iris_frame(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR),
                                   stamp=stamp, wall_recv=wall)
                sayac["islenen"] += 1
            except Exception as e:
                print(f"[GCS GZ-CAM] işçi hata: {e}")
            # 10 s'de bir sağlık raporu: düşme oranı yüksekse işleme hattı
            # kameraya yetişemiyor demektir (o zaman YOLO'yu seyreltmek gerekir).
            simdi = time.time()
            if simdi - sayac["t_log"] >= 10.0:
                gecen = simdi - sayac["t_log"]
                g, i, d = sayac["gelen"], sayac["islenen"], sayac["dusen"]
                print(f"[GZ-CAM] {g/gecen:.1f} kare/s geldi, {i/gecen:.1f} işlendi, "
                      f"{d} düştü (%{100*d/max(g,1):.0f}) — gecikme birikmiyor")
                sayac.update(gelen=0, islenen=0, dusen=0, t_log=simdi)

    topic = os.environ.get("AVCI_GZ_CAMERA_TOPIC", "/iris_cam/image")
    node = GzNode()
    threading.Thread(target=isci, daemon=True, name="gz-cam-isci").start()
    node.subscribe(GzImage, topic, cb)
    print(f"[GCS] gz-transport kamera dinleniyor ({topic}, Harmonic) "
          f"— en-son-kare-kazanır")
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

    # İris kamerasıyla AYNI desen (en-son-kare-kazanır). Talon hattı daha ucuz
    # (YOLO yok, ~7 ms) ama 30 Hz'de aynı CPU'yu paylaşıyor; kuyruğa girerse
    # hem kendi görüntüsü gecikir hem iris hattından zaman çalar.
    t_kutu = {"veri": None, "en": 0, "boy": 0}
    t_kilit = threading.Lock()
    t_olay = threading.Event()

    def cb(msg):
        try:
            with t_kilit:
                t_kutu["veri"] = msg.data
                t_kutu["en"], t_kutu["boy"] = msg.width, msg.height
            t_olay.set()
        except Exception as e:
            print(f"[GCS GZ-CAM] Talon cb hata: {e}")

    def t_isci():
        while True:
            t_olay.wait(1.0)
            t_olay.clear()
            with t_kilit:
                veri, en, boy = t_kutu["veri"], t_kutu["en"], t_kutu["boy"]
                t_kutu["veri"] = None
            if veri is None:
                continue
            try:
                arr = np.frombuffer(veri, dtype=np.uint8).reshape((boy, en, 3))
                process_plane_frame(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
            except Exception as e:
                print(f"[GCS GZ-CAM] Talon işçi hata: {e}")

    topic = os.environ.get("AVCI_GZ_TALON_TOPIC", "/talon_cam/image")
    node = GzNode()
    threading.Thread(target=t_isci, daemon=True, name="gz-talon-isci").start()
    node.subscribe(GzImage, topic, cb)
    print(f"[GCS] gz-transport Talon kamera dinleniyor ({topic}, Harmonic)")
    while True:
        time.sleep(1)


def gz_chase_camera_thread(key, topic, etiket):
    """Gazebo Harmonic dış görüş (chase) kamerası → HAM MJPEG.

    iris/plane akışlarından farkı: HİÇBİR işleme yok — YOLO, takip, kilit
    overlay'i, parazit simülasyonu, hakem logu, hiçbiri. Kare doğrudan JPEG'e
    kodlanıp latest_frames[key]'e yazılır. Bu görüntü hakem ispatı DEĞİLDİR,
    yalnız operatör izlemesi içindir; tespit hattına karışmaması bilinçlidir.
    """
    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.image_pb2 import Image as GzImage
    except Exception as e:
        print(f"[GCS] gz-transport Python yok, {etiket} dış görüş atlandı: {e}")
        return

    def cb(msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            ok, buf = cv2.imencode('.jpg', cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
            if not ok:
                return
            if latest_frames[key]["data"] is None:
                print(f"[GCS] ✓ {etiket} dış görüş kamerasından ilk görüntü!")
            latest_frames[key]["data"] = buf.tobytes()
            latest_frames[key]["id"] += 1
        except Exception as e:
            print(f"[GCS GZ-CAM] {etiket} dış görüş hata: {e}")

    node = GzNode()
    node.subscribe(GzImage, topic, cb)
    print(f"[GCS] gz-transport {etiket} dış görüş dinleniyor ({topic}, Harmonic)")
    while True:
        time.sleep(1)


class CameraSubscriber(RosNode):
    def __init__(self):
        if not _ROS2_VAR:
            raise RuntimeError("ROS 2 (rclpy/cv_bridge) bulunamadı.")
        super().__init__('gcs_camera_listener')
        self.bridge = CvBridge()
        self.create_subscription(RosImage, '/iris_cam/front_camera/image_raw', self.cb_iris, 1)
        self.create_subscription(RosImage, '/plane_cam/front_camera/image_raw', self.cb_plane, 1)
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
    if not _ROS2_VAR:
        print("[GCS] ROS 2 bulunamadı (source /opt/ros/humble/setup.bash gerekli) — ROS kamera dinlenemiyor.")
        return
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
    # ⚠ Eskiden burada sabit ["iris", "plane"] listesi vardı: dış görüş
    # kameraları eklenince istekler SESSİZCE iris'e düşüyor, panelde
    # "Avcı Dış Görüş" düğmesi avcının KENDİ ön kamerasını gösteriyordu.
    # Doğru kaynak tamponların kendisi.
    if vehicle not in latest_frames:
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
    elif msg_type == 'SYS_STATUS':
        # Batarya. ArduPilot voltajı mV, akımı cA (10 mA) birimiyle yollar.
        # Kapasite tanımlı değilse battery_remaining -1 gelir → None yaz ki
        # arayüz "%-1" yerine "VERİ YOK" göstersin.
        v = getattr(msg, 'voltage_battery', 0)
        a = getattr(msg, 'current_battery', -1)
        p = getattr(msg, 'battery_remaining', -1)
        telemetry_state[vehicle_name].update(
            batt_v=round(v / 1000.0, 2) if v not in (0, 65535) else None,
            batt_a=round(a / 100.0, 2) if a >= 0 else None,
            batt_pct=int(p) if 0 <= p <= 100 else None)
    elif msg_type == 'GPS_RAW_INT':
        # Uydu sayısı ve fix tipi. 255 = "bilinmiyor" demek, 0 uydu demek değil.
        sat = getattr(msg, 'satellites_visible', 255)
        eph = getattr(msg, 'eph', 65535)
        telemetry_state[vehicle_name].update(
            gps_fix=int(getattr(msg, 'fix_type', 0)),
            gps_sat=int(sat) if sat != 255 else None,
            gps_eph=round(eph / 100.0, 2) if eph not in (0, 65535) else None)
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

    # ── SOKET BOŞALTMA (2026-08-05, gps_kararli_hal) — kamerayla AYNI hata ──
    # ESKİ HÂLİ: her turda TEK mesaj okunup 5 ms uyunuyordu → tavan 200 msg/s.
    # Ama 14550'ye İKİ araç birden yayın yapıyor (--streamrate=25 ×2, tipik
    # 300-500 msg/s). Fark UDP tamponunda birikir; tampon dolana kadar
    # telemetri giderek geriden gelir, dolduktan sonra büyük ve sabit bir
    # gecikmeye oturur. Bu KAMERADAN AYRI bir hattır: harita konumları,
    # irtifa/hız göstergeleri ve GPS güdümünün gördüğü hedef konumu/hızı
    # bundan etkilenir (bayat telemetri → yanlış hedef hız kestirimi).
    # YENİ HÂLİ: her turda soket bitene kadar boşaltılır (BATCH tavanıyla —
    # tek tur asyncio döngüsünü aç bırakmasın).
    BATCH = 500
    while True:
        okunan = 0
        while okunan < BATCH:
            msg = _mav_conn.recv_match(
                type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT',
                      'ATTITUDE', 'HEARTBEAT', 'SYS_STATUS', 'GPS_RAW_INT'],
                blocking=False
            )
            if msg is None:
                break
            okunan += 1
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

    # Soket boşaltma — mavlink_listener ile aynı gerekçe (bkz. oradaki not).
    # Tek mesaj/5 ms tavanı 200 msg/s'ti; iris SITL'i streamrate=25 ile bunun
    # üstünde yayın yapabiliyor ve fark UDP tamponunda birikiyordu.
    BATCH = 500
    while not _iris_telem_stop.is_set():
        try:
            okunan = 0
            while okunan < BATCH:
                msg = _iris_telem_conn.recv_match(
                    type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT',
                          'ATTITUDE', 'HEARTBEAT', 'SYS_STATUS', 'GPS_RAW_INT'],
                    blocking=False
                )
                if msg is None:
                    break
                okunan += 1
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
    # HybridSORT takipçi — detection üstüne kareler arası ID sürdürür
    # VARSAYILAN AÇIK (2026-08-07, kullanıcı kararı: "takip kararlı branch'teki
    # gibi olsun"). Kararlı dal ve ortak ata da açıktı; 08-04'te bu dalda
    # kapatılmıştı çünkü boxmot kurulu değildi ve takipçi hiç çalışmıyordu —
    # sessizce devreye girip ölçüm tabanını değiştirmesin diye. O gerekçe artık
    # geçersiz: boxmot 18.0.0 KURULU ve TalonTracker sorunsuz kuruluyor.
    # ⚠ Bu yüzden takipçi bu daldaki ölçümlerde İLK KEZ etkin olacak —
    # merge sonrası taban ölçümü (TODO §0) artık takipçili taban demektir.
    # Kapatmak: AVCI_TRACKER=off.
    if os.environ.get("AVCI_TRACKER", "on").lower() not in ("off", "0"):
        global _talon_tracker, _tracker_mod, _target_lock
        if _yolo_detector is None:
            print("[GCS] HybridSORT atlandı (YOLO detector kapalı)")
        else:
            try:
                from vision import tracker as _trk
                _talon_tracker = _trk.TalonTracker()
                _tracker_mod = _trk
                print("[GCS] HybridSORT tracker hazır (boxmot HybridSort)")
                # Kilitli-ID politikası: set_detection'ı kilitli kimliğin kutusu
                # besler (GT'li deney: +%25 doğru kare, FP hedefi tek karede
                # çalamaz). AVCI_LOCK=off → eski "en yüksek conf" seçimi.
                if os.environ.get("AVCI_LOCK", "on").lower() not in ("off", "0"):
                    _target_lock = _trk.TargetLock(_talon_tracker)
                    print("[GCS] Kilitli-ID hedef politikası aktif (AVCI_LOCK=off kapatır)")
            except Exception as e:
                print(f"[GCS] HybridSORT yüklenemedi ({e}) — takip kapalı")

    # Vuruş menzili için sim gerçek-poz aboneliği (başarısızsa telemetri fallback)
    try:
        sim_truth.start()
    except Exception as e:
        print(f"[TRUTH] başlatılamadı ({e}) — menzil telemetriden hesaplanacak")

    # Kamera kaynağı: Harmonic (gz-transport) veya Classic (ROS2 cv_bridge)
    if os.environ.get("AVCI_GZ_CAMERA", "0") == "1":
        threading.Thread(target=gz_iris_camera_thread, daemon=True).start()   # avcı iris
        threading.Thread(target=gz_talon_camera_thread, daemon=True).start()  # hedef Talon
        # Dış görüş (chase) kameraları — AVCI_GZ_CHASE_CAM=0 ile kapatılabilir
        # (iki ek 640x480@15Hz render; zayıf GPU'da kapatmak isteyebilirsiniz).
        if os.environ.get("AVCI_GZ_CHASE_CAM", "1") == "1":
            for _key, _env, _vars, _et in (
                ("iris_chase",  "AVCI_GZ_IRIS_CHASE_TOPIC",  "/iris_chase/image",  "Avcı"),
                ("talon_chase", "AVCI_GZ_TALON_CHASE_TOPIC", "/talon_chase/image", "Talon"),
            ):
                threading.Thread(target=gz_chase_camera_thread, daemon=True,
                                 args=(_key, os.environ.get(_env, _vars), _et)).start()
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