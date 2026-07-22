import sys
import os
import random
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import asyncio
import base64
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
from pymavlink import mavutil
import uvicorn

# NOT: rclpy / cv_bridge / sensor_msgs YALNIZCA Gazebo Classic (ROS2) kamera
# yolunda gerekir. Harmonic (gz-transport, AVCI_GZ_CAMERA=1) modunda ROS 2
# kurulu OLMASA BİLE sunucu açılabilsin diye bu importlar ros2_spin_thread()
# içine ERTELENDİ (lazy). Böylece `import rclpy` başarısızsa gz modu etkilenmez.

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
# Manuel moddan çıkışta uçağı disarm et Mİ?
#   True  → "tam duruş" (yere in): override bırakılır + güvenli disarm
#   False → başka moda GEÇİŞ (AUTO/kare): DİSARM ETME → uçak havada düşmesin,
#           armed + throttle korunarak AUTO devralana dek uçmaya devam etsin.
_plane_disarm_on_manual_exit = True
# Manuel kontrol thread nesnesi — moda geçişte "tam durmasını bekle" (join) için.
# Aksi halde manuel thread hâlâ init'teyken (FBWA/arm) kare thread'i LOITER
# kurup ikisi ÇAKIŞIR (race) → uçak yanlış modda kalıp düşer.
_manual_thread_obj = None

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
        # Rastgele/kaçış uçuşu aktifse durdur (aynı uçak/port 14542)
        global _plane_random_active
        if _plane_random_active:
            _plane_random_stop.set()
            _plane_random_active = False
            if _plane_random_thread_obj is not None:
                _plane_random_thread_obj.join(timeout=8.0)
            time.sleep(0.3)
        # Eğer manuel kontrol aktifse durdur — ANCAK bu bir MODA GEÇİŞ olduğundan
        # manuel thread'in uçağı disarm ETMEMESİ gerekir (aksi halde havada düşer).
        global _manual_active, _plane_disarm_on_manual_exit
        if _manual_active:
            _plane_disarm_on_manual_exit = False   # geçiş: armed kal, düşme
            _manual_active = False
            # RACE ÖNLEME: manuel thread'in TAMAMEN durmasını bekle (join). Aksi
            # halde manuel thread hâlâ FBWA/arm init'indeyken LOITER kurulur, ikisi
            # çakışır ve uçak yanlış modda kalıp düşer.
            if _manual_thread_obj is not None:
                _manual_thread_obj.join(timeout=8.0)
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

PLANE_MODE_AUTO = 10     # ArduPlane AUTO custom mode
PLANE_MODE_LOITER = 12   # ArduPlane LOITER custom mode (stabil daire)


def _upload_mission(conn, items):
    """Standart MAVLink mission protokolü ile görev öğelerini yükler.
    items: [{seq, frame, cmd, current, autocontinue, p1..p4, x(lat*1e7), y(lon*1e7), z(alt_m)}]"""
    sysid, comp = conn.target_system, conn.target_component
    conn.mav.mission_count_send(sysid, comp, len(items))
    for _ in range(len(items)):
        req = conn.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                              blocking=True, timeout=5)
        if req is None:
            raise RuntimeError("MISSION_REQUEST zaman aşımı")
        it = items[req.seq]
        conn.mav.mission_item_int_send(
            sysid, comp, it['seq'], it['frame'], it['cmd'],
            int(it.get('current', 0)), int(it.get('autocontinue', 1)),
            float(it.get('p1', 0)), float(it.get('p2', 0)),
            float(it.get('p3', 0)), float(it.get('p4', 0)),
            int(it['x']), int(it['y']), float(it['z']),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    ack = conn.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    return ack


def _square_thread():
    """Hedef İHA (ArduPlane) için 4 noktalı OTONOM kare görevi:
    MAV_CMD_NAV_WAYPOINT'lerden oluşan bir görev yükler, AUTO moda alır ve arm
    ederek otopilotun kareyi tek başına (arka planda) uçmasını sağlar. Avcı iris
    manuel modu bundan HİÇ etkilenmez (ayrı araç / ayrı port 14542)."""
    global _square_active
    print("[SQUARE] Otonom LOITER (stabil daire) handoff başlıyor...")
    conn = None
    try:
        # connect_mavlink OTOPİLOT heartbeat'ini bekler (mavproxy'nin sys=0/GCS
        # heartbeat'lerini yok sayar) → target_system doğru (plane sysid) alınır.
        from control.mav_common import connect_mavlink
        conn = connect_mavlink(14542, source_system=255)
        sysid, comp = conn.target_system, conn.target_component
        print(f"[SQUARE] Plane bağlandı sys={sysid}")

        # Home/mevcut GPS konumunu al
        home = None
        t0 = time.time()
        while time.time() - t0 < 15 and not _square_stop_event.is_set():
            msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg and msg.lat != 0:
                home = (msg.lat, msg.lon)
                break
        if home is None:
            print("[SQUARE] GPS home alınamadı — iptal")
            return
        lat0, lon0 = home
        LOIT = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
        TO   = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        WP   = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        FR   = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

        # UÇAK HAVADA MI? (manuel→AUTO geçişi) — mevcut irtifayı telemetriden al.
        cur_alt_m = -telemetry_state["plane"]["z"]
        airborne = cur_alt_m > 15.0

        if airborne:
            # ===== STABİL LOITER HANDOFF (manuel→otomatik) =====
            # Kare/AUTO waypoint navigasyonu bu Gazebo uçağında düşük hızda sert
            # virajda STALL → çakılmaya yol açıyordu. Bunun yerine LOITER moduna
            # alıyoruz: uçak MEVCUT konum ve irtifada yumuşak, OTOMATİK-GAZLI bir
            # DAİRE çizer ve her banklı/yavaş durumdan kendini toplar → çakılma
            # imkânsız, geçiş %100 sorunsuz. (Mission gerekmez; LOITER bulunduğu
            # noktada döner.) Uçak zaten armed olduğundan itki hiç kesilmez.
            print(f"[SQUARE] Uçak HAVADA (~{cur_alt_m:.0f}m) → STABİL LOITER (daire) handoff")
            conn.mav.rc_channels_override_send(sysid, comp, 0, 0, 0, 0, 0, 0, 0, 0)
            time.sleep(0.2)
            conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                       PLANE_MODE_LOITER, 0, 0, 0, 0, 0)
            time.sleep(0.3)
            # Garanti force-arm (havada zaten armed → no-op)
            conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 2989, 0, 0, 0, 0, 0)
            print("[SQUARE] ✓ LOITER aktif — uçak mevcut irtifada stabil daire çiziyor")
        else:
            # ===== YERDEN KALKIŞ → LOITER =====
            # NAV_TAKEOFF ile kalk, sonra NAV_LOITER_UNLIM ile stabil daire çiz.
            # Sert köşeli kare navigasyonu YOK → stall/çakılma yok.
            # İrtifa 30m (düşük): yüksek/dik tırmanışta hız bitip tepede stall
            # oluyordu; 30m'ye sığ tırmanışla çıkınca hız korunur.
            alt = 30.0                                   # metre (home'a göreli)
            print(f"[SQUARE] Uçak YERDE → NAV_TAKEOFF + LOITER ({alt:.0f}m'de stabil daire)")
            items = [
                dict(seq=0, frame=FR, cmd=WP,   x=lat0, y=lon0, z=alt, current=1),   # home
                dict(seq=1, frame=FR, cmd=TO,   x=lat0, y=lon0, z=alt, p1=15),        # takeoff
                dict(seq=2, frame=FR, cmd=LOIT, x=lat0, y=lon0, z=alt),               # sınırsız daire
            ]
            ack = _upload_mission(conn, items)
            print(f"[SQUARE] Görev yüklendi (ACK type={getattr(ack, 'type', '?')}, {len(items)} nokta)")
            conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                       PLANE_MODE_AUTO, 0, 0, 0, 0, 0)
            time.sleep(0.6)
            conn.mav.rc_channels_override_send(sysid, comp, 0, 0, 0, 0, 0, 0, 0, 0)
            conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 2989, 0, 0, 0, 0, 0)
            time.sleep(0.5)
            conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_MISSION_START,
                                       0, 0, 0, 0, 0, 0, 0, 0)
            print("[SQUARE] AUTO + ARM → kalkış sonrası stabil LOITER (daire)")

        # İzleme döngüsü (otopilot kendi uçar; sadece durdurma sinyalini bekle)
        last_wp = -1
        while not _square_stop_event.is_set():
            m = conn.recv_match(type='MISSION_CURRENT', blocking=False)
            if m and m.seq != last_wp:
                last_wp = m.seq
                print(f"[SQUARE] Aktif waypoint: {last_wp}")
            time.sleep(0.5)
        # Durdurma: RTL/otoland YAPMA. Kare yalnızca BAŞKA bir moda geçmek için
        # durdurulur (manuel vb.) ve o mod hemen devralır. RTL burada otomatik
        # inişe (RTL_AUTOLAND) ve DÜŞÜŞE yol açıyordu; uçağı AUTO'da bırakıp
        # çıkıyoruz → devralan mod (FBWA) kesintisiz alır.
        print("[SQUARE] Durduruldu (moda geçiş — RTL yok, devralan mod uçağı alır).")
    except Exception as e:
        import traceback
        print(f"[SQUARE] HATA: {e}")
        traceback.print_exc()
    finally:
        _square_active = False
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

@app.post("/api/command/plane/circle")
def command_plane_circle():
    return {"status": "error", "message": "Daire scripti henüz hazır değil!"}


# =======================================================================
# HEDEF İHA RASTGELE / KAÇIŞ UÇUŞU — GUIDED + periyodik rastgele hedef
# =======================================================================
# Hedef uçak (Talon) gelişigüzel gezinir (Avcı drone kovalasın diye): kalkar,
# GUIDED moduna geçer ve periyodik olarak ~home çevresinde RASTGELE noktalara
# reposition eder → yumuşak dönüşlerle "yuvarlaklar + sağ-sol" gelişigüzel uçuş.
# ArduPlane VARSAYILAN ayarlarıyla stabil olduğundan (bkz. avci_plane.parm)
# bu manevralar stall/çakılma yapmaz.
PLANE_MODE_GUIDED = 15
_plane_random_active = False
_plane_random_stop = threading.Event()
_plane_random_thread_obj = None
PLANE_RANDOM_ALT     = 55.0     # kalkış/başlangıç irtifası (m) — yüksek başla, marj
PLANE_R_MIN          = 80.0     # leg uzunluğu — dönüşler arası hız toparlar (stall önle)
PLANE_R_MAX          = 160.0    # uzun leg üst sınırı (m)
PLANE_ALT_MIN        = 50.0     # işletme irtifası tabanı — YÜKSEK → havada kalır, stall payı
PLANE_ALT_MAX        = 75.0     # irtifa üst sınırı (geniş 3B, havada gezinir)
PLANE_Z_MIN          = 40.0     # SERT alt limit — bunun altına ASLA inmez (yere yakın uçmaz)
PLANE_ALT_RECOVER    = 35.0     # bu irtifanın altına düşerse → ACİL TOPARLA (düz ileri + tırman)
PLANE_WP_ACCEPT      = 45.0     # varış kabul yarıçapı
PLANE_WP_MAXTIME     = 8.0      # bir waypoint'te maks süre


PLANE_ALT_STEP = 12.0        # bir leg'de maks irtifa değişimi (m) — kademeli


PLANE_OP_RADIUS = 260.0      # home operasyon alanı yarıçapı (m) — dışına taşmasın
PLANE_THREAT_R  = 140.0      # Avcı bu mesafeden yakınsa → kaçış manevrası (ondan uzağa)


def _generate_random_3d_waypoint(cur_x, cur_y, cur_yaw_deg, lat0, lon0, coslat,
                                 prev_alt, avci_x=None, avci_y=None, jdir=1):
    """KAÇAMAK / KAOTİK 3B waypoint — TEK DÜZEN DEĞİL. Her leg farklı manevra:
      • hafif dokuma (weave)   • SERT jink (yön alternasyonlu, S çizer)
      • tamamen gelişigüzel yön
    Ayrıca AVCI yakınsa (PLANE_THREAT_R) → ONDAN UZAĞA sert kaçış legi.
    Amaç: yakalanmamak. Stall koruması: SERT dönüşlerde irtifa KORUNUR (seviye
    uçuş); irtifa oyunu sadece düz/hafif leg'lerde yapılır. Home alanına
    (PLANE_OP_RADIUS) sıkıştırılır, 15 m sert taban korunur.
    jdir: sert jink yön alternasyonu (±1) — sürekli aynı yöne dönüp daire olmasın."""
    base = math.radians(cur_yaw_deg)               # NED yaw: 0=Kuzey(+x), 90=Doğu(+y)
    # --- TEHDİT: Avcı yakın mı? Yakınsa kaçış moduna gir ---
    threat = False
    dxp = dyp = 0.0
    if avci_x is not None:
        dxp, dyp = cur_x - avci_x, cur_y - avci_y  # Avcı'DAN uzağa vektör
        if math.hypot(dxp, dyp) < PLANE_THREAT_R:
            threat = True
    if threat:
        away = math.atan2(dyp, dxp)                # Avcı'nın tam tersi yön
        # DÜZ kaçma = kolay yakalanır → kaçarken de JINK at (alternasyonlu S), YUMUŞAK
        ang  = away + jdir * random.uniform(0.35, 0.8) + random.uniform(-0.2, 0.2)
        r    = random.uniform(PLANE_R_MIN, PLANE_R_MAX)
    else:
        m = random.random()
        if m < 0.45:                               # hafif dokuma
            ang = base + random.uniform(-0.5, 0.5)
        elif m < 0.82:                             # ORTA jink (alternasyonlu → S, kafa karıştırır)
            ang = base + jdir * random.uniform(0.6, 1.1)
        else:                                      # geniş ama SINIRLI yön değişimi (U-dönüşü yok)
            ang = base + (1 if random.random() < 0.5 else -1) * random.uniform(1.0, 1.5)
        r = random.uniform(PLANE_R_MIN, PLANE_R_MAX)
    nx = cur_x + r * math.cos(ang)                 # yeni Kuzey (m)
    ny = cur_y + r * math.sin(ang)                 # yeni Doğu (m)
    d = math.hypot(nx, ny)
    if d > PLANE_OP_RADIUS:                         # alan dışına taşarsa içeri çevir
        nx *= PLANE_OP_RADIUS / d
        ny *= PLANE_OP_RADIUS / d
    tlat = lat0 + int((nx / 111320.0) * 1e7)
    tlon = lon0 + int((ny / (111320.0 * coslat)) * 1e7)
    # --- İRTİFA: sadece DÜZ/HAFİF leg'te oynat; SERT dönüşte KORU (stall önle) ---
    turn = abs(((ang - base + math.pi) % (2 * math.pi)) - math.pi)   # net dönüş açısı
    if turn < 0.7:                                  # düz/hafif → irtifa jink yap
        alt_target = random.uniform(PLANE_ALT_MIN, PLANE_ALT_MAX)
        step = max(-PLANE_ALT_STEP, min(PLANE_ALT_STEP, alt_target - prev_alt))
        talt = prev_alt + step
    else:
        talt = prev_alt                            # sert dönüş → seviye uçuş
    talt = min(max(talt, PLANE_ALT_MIN), PLANE_ALT_MAX)
    talt = max(talt, PLANE_Z_MIN)                  # SERT 15 m tabanı
    return tlat, tlon, talt, nx, ny


def _plane_random_thread():
    global _plane_random_active
    print("[PLANE-RANDOM] Rastgele/kaçış uçuşu başlıyor...")
    conn = None
    try:
        from control.mav_common import connect_mavlink
        conn = connect_mavlink(14542, source_system=255)
        sysid, comp = conn.target_system, conn.target_component
        print(f"[PLANE-RANDOM] Plane bağlandı sys={sysid}")

        # home GPS
        home = None
        t0 = time.time()
        while time.time() - t0 < 15 and not _plane_random_stop.is_set():
            msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg and msg.lat != 0:
                home = (msg.lat, msg.lon)
                break
        if home is None:
            print("[PLANE-RANDOM] GPS home alınamadı — iptal")
            return
        lat0, lon0 = home
        FR = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        cur_alt_m = -telemetry_state["plane"]["z"]
        airborne = cur_alt_m > 15.0
        # Gezinme irtifası: yerden kalkışta PLANE_RANDOM_ALT; havadaysa mevcut
        # irtifayı 40-80m aralığına KISITLA (çok yüksek/düşük başlangıçta bile
        # sürdürülebilir bir irtifada gezinsin, sürekli alçalıp durmasın).
        alt = PLANE_RANDOM_ALT if not airborne else max(50.0, min(round(cur_alt_m), 75.0))

        if not airborne:
            # Yerden kalkış: NAV_TAKEOFF + LOITER (stabil), sonra GUIDED devralır
            WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            TO = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            LOIT = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
            items = [
                dict(seq=0, frame=FR, cmd=WP,   x=lat0, y=lon0, z=alt, current=1),
                dict(seq=1, frame=FR, cmd=TO,   x=lat0, y=lon0, z=alt, p1=15),
                dict(seq=2, frame=FR, cmd=LOIT, x=lat0, y=lon0, z=alt),
            ]
            _upload_mission(conn, items)
            print(f"[PLANE-RANDOM] Yerden kalkış → {alt:.0f}m")
            conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                       PLANE_MODE_AUTO, 0, 0, 0, 0, 0)
            time.sleep(0.6)
            conn.mav.rc_channels_override_send(sysid, comp, 0, 0, 0, 0, 0, 0, 0, 0)
            conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 2989, 0, 0, 0, 0, 0)
            time.sleep(0.5)
            conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_MISSION_START,
                                       0, 0, 0, 0, 0, 0, 0, 0)
            # kalkış tamamlanana kadar bekle
            t0 = time.time()
            while time.time() - t0 < 45 and not _plane_random_stop.is_set():
                if -telemetry_state["plane"]["z"] > alt * 0.7:
                    break
                time.sleep(1)
        else:
            conn.mav.rc_channels_override_send(sysid, comp, 0, 0, 0, 0, 0, 0, 0, 0)

        # Manevralarda stall marjı: airspeed'i yükselt + minimum gazı garantile
        # (kademeli 3B in/çık + dönüşlerde hız çökmesin, süzülürken stall olmasın).
        try:
            # Kaçamak/hızlı dönüş için HIZ + ROLL marjı: yüksek airspeed = aynı
            # yatışta daha SIKI/HIZLI dönüş + stall payı. Roll limitini gevşet
            # (sert bank). TECS hızı korusun (dönüşte hız çökmesin → stall önlenir).
            conn.mav.param_set_send(sysid, comp, b'TRIM_ARSPD_CM', 2300.0,
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            conn.mav.param_set_send(sysid, comp, b'THR_MIN', 45.0,
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            conn.mav.param_set_send(sysid, comp, b'THR_MAX', 100.0,
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            conn.mav.param_set_send(sysid, comp, b'TECS_SPDWEIGHT', 1.5,
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            conn.mav.param_set_send(sysid, comp, b'LIM_ROLL_CD', 4500.0,  # 45° — yumuşak, stall önle
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        except Exception:
            pass

        # GUIDED moduna al → rastgele reposition ile gezin
        conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                   mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                   PLANE_MODE_GUIDED, 0, 0, 0, 0, 0)
        time.sleep(0.6)
        print("[PLANE-RANDOM] GUIDED — kademeli 3B gelişigüzel gezinme başladı")

        coslat = math.cos(math.radians(lat0 / 1e7))
        prev_alt = max(float(alt), PLANE_ALT_MIN)   # kademeli irtifa için başlangıç
        _tick = 0
        jdir = 1                                    # jink yön alternasyonu (S çizsin)
        while not _plane_random_stop.is_set():
            cur  = telemetry_state["plane"]
            avci = telemetry_state["iris"]
            alt_now = -cur["z"]                      # NED: irtifa = -z (m)
            # ==== ACİL TOPARLAMA: yere yaklaşırsa manevrayı BIRAK, DÜZ İLERİ + TIRMAN ====
            # Yerinde tırmanış stall yapar → mevcut heading'de ileri uçarak hız toplayıp
            # güvenli irtifaya çıkar. Böylece yere çakılma / yerde sürünme önlenir.
            if alt_now < PLANE_ALT_RECOVER:
                hd = math.radians(cur["yaw"])
                rx = cur["x"] + 150.0 * math.cos(hd)
                ry = cur["y"] + 150.0 * math.sin(hd)
                dd = math.hypot(rx, ry)
                if dd > PLANE_OP_RADIUS:
                    rx *= PLANE_OP_RADIUS / dd; ry *= PLANE_OP_RADIUS / dd
                rlat = lat0 + int((rx / 111320.0) * 1e7)
                rlon = lon0 + int((ry / (111320.0 * coslat)) * 1e7)
                conn.mav.set_position_target_global_int_send(
                    0, sysid, comp, FR, 0b0000111111111000,
                    rlat, rlon, PLANE_ALT_MAX,       # güvenli yüksek irtifaya tırman
                    0, 0, 0, 0, 0, 0, 0, 0)
                print(f"[PLANE-RANDOM] ⚠ TOPARLAMA irtifa={alt_now:.0f}m → düz ileri + {PLANE_ALT_MAX:.0f}m")
                t0 = time.time()
                while time.time() - t0 < 7 and not _plane_random_stop.is_set():
                    if -telemetry_state["plane"]["z"] > PLANE_ALT_MIN:
                        break
                    time.sleep(0.3)
                prev_alt = max(-telemetry_state["plane"]["z"], PLANE_ALT_MIN)
                continue
            # ---- KAÇAMAK/KAOTİK 3B WAYPOINT (konum+heading+Avcı konumuna göre) ----
            tlat, tlon, talt, dn, de = _generate_random_3d_waypoint(
                cur["x"], cur["y"], cur["yaw"], lat0, lon0, coslat, prev_alt,
                avci.get("x"), avci.get("y"), jdir)
            jdir = -jdir                            # her leg yönü çevir → daire olmasın
            prev_alt = talt
            conn.mav.set_position_target_global_int_send(
                0, sysid, comp, FR,
                0b0000111111111000,                      # sadece pozisyon kullan
                tlat, tlon, talt,
                0, 0, 0, 0, 0, 0, 0, 0)
            _tick += 1
            print(f"[PLANE-RANDOM] 3B hedef #{_tick}: {dn:+.0f}K {de:+.0f}D irtifa={talt:.0f}m")
            # ---- VARIŞ tespiti: hedefe yaklaşınca (veya maks süre) yeni 3B nokta ----
            t0 = time.time()
            while time.time() - t0 < PLANE_WP_MAXTIME and not _plane_random_stop.is_set():
                msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg and msg.lat != 0:
                    # hedefe yatay mesafe (m)
                    dlat_m = (tlat - msg.lat) / 1e7 * 111320.0
                    dlon_m = (tlon - msg.lon) / 1e7 * 111320.0 * coslat
                    if math.hypot(dlat_m, dlon_m) < PLANE_WP_ACCEPT:
                        break                            # vardı → yeni gelişigüzel nokta
                    # İRTİFA KORUMASI: recover eşiğinin altına inerse bu WP'yi BIRAK →
                    # dış döngü acil toparlamayı (düz ileri + tırman) devralır.
                    if msg.relative_alt / 1000.0 < PLANE_ALT_RECOVER:
                        break
                time.sleep(0.3)

        # durdurma → LOITER (stabil daire, çakılmadan bekler)
        conn.mav.command_long_send(sysid, comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                   mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                   PLANE_MODE_LOITER, 0, 0, 0, 0, 0)
        print("[PLANE-RANDOM] Durduruldu → LOITER (stabil).")
    except Exception as e:
        import traceback
        print(f"[PLANE-RANDOM] HATA: {e}")
        traceback.print_exc()
    finally:
        _plane_random_active = False
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass


@app.post("/api/command/plane/start_random")
def plane_start_random():
    """Hedef İHA'yı kaldırıp rastgele/kaçış gezinme uçuşu başlatır."""
    global _plane_random_active, _plane_random_thread_obj, _manual_active
    global _plane_disarm_on_manual_exit, _square_active
    if _plane_random_active:
        return {"status": "success", "message": "Zaten rastgele uçuşta"}
    # Kare/otonom veya manuel aktifse durdur (aynı uçak/araç)
    if _square_active:
        _square_stop_event.set()
        time.sleep(0.5)
    if _manual_active:
        _plane_disarm_on_manual_exit = False   # havada kal
        _manual_active = False
        if _manual_thread_obj is not None:
            _manual_thread_obj.join(timeout=8.0)
        _plane_disarm_on_manual_exit = True
        time.sleep(0.3)
    _plane_random_active = True
    _plane_random_stop.clear()
    t = threading.Thread(target=_plane_random_thread, daemon=True)
    _plane_random_thread_obj = t
    t.start()
    print("[PLANE-RANDOM] Rastgele uçuş başlatıldı.")
    return {"status": "success", "message": "Hedef İHA rastgele/kaçış uçuşu"}


@app.post("/api/command/plane/stop_random")
def plane_stop_random():
    global _plane_random_active
    _plane_random_stop.set()
    _plane_random_active = False
    print("[PLANE-RANDOM] Rastgele uçuş durduruldu.")
    return {"status": "success"}

# -----------------------------------------------------------------------
# MANUEL KONTROL
# -----------------------------------------------------------------------
class ManualCmd(BaseModel):
    aileron:  int
    elevator: int
    throttle: int

@app.post("/api/command/plane/start_manual")
def start_manual_mode():
    global _manual_active, _square_active, _plane_disarm_on_manual_exit
    global _manual_aileron, _manual_elevator, _manual_throttle
    global _plane_random_active

    # Bu manuel oturumu normal biterse (STOP MANUAL) uçak yere insin → disarm.
    # (Kare'ye geçişte command_plane_square bunu tekrar False yapar.)
    _plane_disarm_on_manual_exit = True

    # =========================================================
    # ADIM 1: Otonom/rastgele uçuşları durdur (aynı uçak/port)
    # =========================================================
    if _plane_random_active:
        _plane_random_stop.set()
        _plane_random_active = False
        if _plane_random_thread_obj is not None:
            _plane_random_thread_obj.join(timeout=8.0)
        time.sleep(0.3)
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
    # SORUNSUZ AUTO→MANUEL: uçak HAVADAYSA manuele CRUISE throttle ile gir.
    # Aksi halde manuel throttle 1000'e (rölanti) düşer → uçak süzülüp DÜŞER.
    # (Frontend de start_manual'de throttle'ı ~%60'a çekiyor; ikisi uyumlu.)
    plane_alt = -telemetry_state["plane"]["z"]   # metre (yukarı +)
    airborne = plane_alt > 8.0
    _manual_throttle = 1650 if airborne else 1000
    print(f"[GCS] Manuel başlıyor — plane alt={plane_alt:.0f}m "
          f"{'(havada → cruise throttle)' if airborne else '(yerde → rölanti)'}")

    global _manual_thread_obj
    t = threading.Thread(target=_manual_control_thread, daemon=True)
    _manual_thread_obj = t
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
        # MOD DEĞİŞTİR — FBWA (ANGLE / attitude kontrol).
        # MANUAL (0) modunda aileron doğrudan servoya gider → aileron = roll
        # HIZI; tam kırımda uçak hızla takla atar ("roll çok hızlı"). FBWA (5)
        # modunda aileron çubuğu ROLL AÇISINI komutlar (LIM_ROLL_CD ile sınırlı,
        # otopilot stabilize eder, çubuk merkezde kanatlar otomatik seviyelenir)
        # → yumuşak, hafif, kendi kendine düzelen roll. Maks yatış 45° ile sınırlı.
        # =====================================================
        # Açı LİMİTLERİ — otopilot tarafında uygulanır (giriş gecikmesi YOK).
        #  * LIM_ROLL_CD=6000  → maks yatış 60° (talep). FBWA bunu AŞAMAZ →
        #    uçak ASLA ters dönemez / takla atamaz (yalnızca 60°'ye kadar yatar).
        #  * LIM_PITCH_MAX/MIN → burun yukarı/aşağı açısını sınırla → elevator ile
        #    LOOP (ters takla) yapılamaz.
        def _set_param(name, val):
            try:
                conn.mav.param_set_send(
                    conn.target_system, conn.target_component,
                    name, float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            except Exception as e:
                print(f"[MANUAL] {name.decode()} ayarlanamadı: {e}")

        def _read_param(name, default):
            """Mevcut param değerini oku (çıkışta geri yüklemek için)."""
            try:
                conn.mav.param_request_read_send(
                    conn.target_system, conn.target_component, name, -1)
                t0 = time.time()
                while time.time() - t0 < 0.6:
                    pv = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                    if pv is not None and pv.param_id.strip('\x00') == name.decode():
                        return float(pv.param_value)
            except Exception:
                pass
            return default

        # ÖNCE orijinal limitleri kaydet — manuel bitince geri yüklenecek ki
        # AUTO/kare gibi diğer modlar manuelin agresif 60° limitiyle bozulmasın
        # (düşük hızda 60° yatış → kanat stall → kalkışta ters dönme/çakılma).
        _orig_lims = {
            b'LIM_ROLL_CD':   _read_param(b'LIM_ROLL_CD', 4500.0),
            b'LIM_PITCH_MAX': _read_param(b'LIM_PITCH_MAX', 2000.0),
            b'LIM_PITCH_MIN': _read_param(b'LIM_PITCH_MIN', -2500.0),
        }
        _set_param(b'LIM_ROLL_CD', 6000.0)     # 60° maks yatış (manuel için)
        _set_param(b'LIM_PITCH_MAX', 2000.0)   # +20° maks burun yukarı
        _set_param(b'LIM_PITCH_MIN', -2000.0)  # -20° maks burun aşağı
        print(f"[MANUAL] Açı limitleri: roll ±60°, pitch ±20° "
              f"(orijinaller kaydedildi: {[int(v) for v in _orig_lims.values()]})")

        # FBWA'yı fire-and-forget dayatan yardımcı (bloklamaz) — döngüde de kullanılır.
        def _assert_fbwa():
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                PLANE_MODE_FBWA, 0, 0, 0, 0, 0)

        print("[MANUAL] FBWA (ANGLE) moduna geçiliyor...")
        result = set_mode(conn, PLANE_MODE_FBWA, confirm_timeout=1.0)
        if result and result[1] == 0:
            print("[MANUAL] ✓ ArduPlane FBWA modu kabul etti (ACK result=0)")
        else:
            print(f"[MANUAL] ⚠ Mode ACK: {result} — yine de devam ediliyor")
            # İkinci deneme
            time.sleep(0.2)
            result2 = set_mode(conn, PLANE_MODE_FBWA, confirm_timeout=1.0)
            print(f"[MANUAL] İkinci deneme ACK: {result2}")

        # =====================================================
        # ARM — throttle etkili olsun diye ŞART
        # ArduPlane MANUAL modda motor yalnızca ARMED iken döner; disarm
        # durumunda gaz komutu (CH3) SERVO çıkışına yansımaz → İHA yerinde
        # kalır. Force-arm (magic 2989) prearm kontrollerini atlar (SITL için
        # güvenli). Yerde rölanti gazda ArduPlane ~10 sn sonra otomatik disarm
        # ettiğinden döngü içinde periyodik olarak yeniden arm ediyoruz.
        # =====================================================
        def _force_arm():
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 2989, 0, 0, 0, 0, 0)   # p1=1 arm, p2=2989 force

        print("[MANUAL] Plane ARM ediliyor (force)...")
        _force_arm()
        _last_arm = time.time()

        # =====================================================
        # RC OVERRIDE DÖNGÜSÜ — 10 Hz
        # =====================================================
        # GECİKMESİZ ROLL: aileron komutu ANINDA gönderilir (slew/rampa/ölçekleme
        # YOK → giriş gecikmesi kalkar, çubuk anında karşılık bulur). Eğilme
        # yumuşaklığı ve SINIRI otopilot tarafında sağlanır: FBWA çubuğu roll
        # AÇISINA çevirir ve LIM_ROLL_CD (=60°) ile sınırlar → tam çubuk = 60°.
        print("[MANUAL] RC Override döngüsü başlıyor (10 Hz, gecikmesiz roll)...")
        _log_tick = 0
        _last_mode = time.time()
        while _manual_active:
            conn.mav.rc_channels_override_send(
                conn.target_system,
                conn.target_component,
                _manual_aileron,    # CH1: Roll/Aileron — doğrudan (gecikmesiz)
                _manual_elevator,   # CH2: Pitch/Elevator
                _manual_throttle,   # CH3: Throttle
                1500,               # CH4: Yaw nötr
                0, 0, 0, 0
            )
            # FBWA'yı ~1 sn'de bir YENİDEN DAYAT — mod herhangi bir sebeple
            # MANUAL'e düşerse (RC mod anahtarı, failsafe vb.) uçak rate-roll ile
            # ters takla atabilir; bu koruma onu tekrar açı-limitli FBWA'ya çeker.
            if time.time() - _last_mode > 1.0:
                _assert_fbwa()
                _last_mode = time.time()
            # Yerde otomatik disarm'a karşı ~3 sn'de bir yeniden arm et
            if time.time() - _last_arm > 3.0:
                _force_arm()
                _last_arm = time.time()
            # Takip kolaylığı için saniyede ~1 kez komut logla
            _log_tick += 1
            if _log_tick % 10 == 0:
                print(f"[MANUAL] Komut → ail={_manual_aileron} "
                      f"elv={_manual_elevator} thr={_manual_throttle}")
            time.sleep(0.1)

        # ===================== KAPANIŞ — moda göre =====================
        # ÖNCE açı limitlerini orijinallerine geri yükle → sonraki AUTO/kare
        # güvenli varsayılan limitlerle uçsun (60° manuel limiti kalkışta çakılmaya
        # yol açıyordu).
        for _pname, _pval in _orig_lims.items():
            _set_param(_pname, _pval)
        print(f"[MANUAL] Açı limitleri geri yüklendi: {[int(v) for v in _orig_lims.values()]}")

        if _plane_disarm_on_manual_exit:
            # TAM DURUŞ (STOP MANUAL): override'ları bırak (0=serbest) + güvenli
            # disarm. Yerde/duruş senaryosu için doğru davranış.
            for _ in range(3):
                conn.mav.rc_channels_override_send(
                    conn.target_system, conn.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0)   # tüm kanalları serbest bırak
                time.sleep(0.05)
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0)   # p1=0 disarm, p2=21196 force
            keepalive.stop()
            print("[MANUAL] Kapatıldı (tam duruş → disarm gönderildi).")
        else:
            # MODA GEÇİŞ (AUTO/kare): DİSARM ETME → uçak havada DÜŞMESİN.
            # Roll/pitch/yaw override'larını serbest bırak (AUTO otopiloti kontrol
            # etsin) ama THROTTLE'ı cruise seviyede tut ki AUTO devralana kadar
            # uçak güçte kalıp seviyeli uçsun (AUTO'da bu throttle override'ı TECS
            # tarafından zaten yok sayılır). Böylece geçiş SORUNSUZ, düşme yok.
            cruise_thr = max(1650, int(_manual_throttle))
            for _ in range(3):
                conn.mav.rc_channels_override_send(
                    conn.target_system, conn.target_component,
                    0, 0, cruise_thr, 0, 0, 0, 0, 0)   # yalnız CH3 throttle override
                time.sleep(0.05)
            keepalive.stop()
            print(f"[MANUAL] Moda geçiş → DİSARM YOK, armed + throttle {cruise_thr} "
                  f"korunuyor (AUTO devralacak).")

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
    _send_velocity_setpoint,
    land_drone as df_land,
    get_conn as df_get_conn,
    SETPOINT_RATE,
)
from control.mav_common import (
    COPTER_MODE_GUIDED,
    COPTER_MODE_LOITER,
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


# =======================================================================
# İRİS (AVCI) MANUEL UÇUŞ — GUIDED + velocity setpoint ile klavye kontrolü
# =======================================================================
# ArduCopter yön/hız komutlarını YALNIZCA GUIDED modda ve ARMED iken uygular;
# ayrıca araç HAVADA olmalıdır (yerdeyken hız setpoint'i onu kaldırmaz). Bu
# yüzden manuel moda geçişte otomatik olarak: GUIDED → ARM → TAKEOFF(~3 m) →
# 10 Hz hız (velocity) döngüsü çalışır. Setpoint akışı hiç kesilmez (tuş
# basılı olmasa bile 0-hız gider) → ArduPilot watchdog'u tetiklenmez, hover.
_iris_manual_active = False
_iris_manual_vx = 0.0        # ileri(+)/geri(-)   m/s (body frame)
_iris_manual_vy = 0.0        # sağ(+)/sol(-)      m/s
_iris_manual_vz = 0.0        # aşağı(+)/yukarı(-) m/s (NED: yukarı = negatif)
_iris_manual_yawrate = 0.0   # sağa(+)/sola(-)    rad/s
IRIS_MANUAL_ALT = 3.0        # otomatik kalkış irtifası (m)
_iris_manual_thread_obj = None   # manuel→dans geçişinde "tam dur"u bekle (join)
# Manuel çıkışta İNİLSİN Mİ? True=LAND (tam duruş), False=LOITER'da hover kal
# (dansa kesintisiz geçiş için — inip tekrar kalkmasın).
_iris_land_on_manual_exit = True

# --- Manevra yumuşatma + maks yatış (roll) limiti (ANGLE mantığı) -----------
# ArduCopter ANGLE_MAX parametresi maksimum yatış (roll/pitch) açısını sınırlar;
# 45° = 4500 santi-derece. Bu, aracın herhangi bir manevrada 45°'den fazla
# yatmasını engeller (talep edilen ±45° clamp). Ek olarak komutları kademeli
# (slew rate-limit) uygulayarak sağa/sola dönüşteki "aşırı sert" hareketi
# yumuşatıyoruz — hız hedefe adım adım yaklaşır, ani sıçrama olmaz.
IRIS_MAX_BANK_DEG = 45.0     # ANGLE_MAX → maksimum yatış açısı (roll/pitch)
IRIS_VEL_SLEW = 4.0          # m/s / s  — yatay/dikey hız değişim tavanı
IRIS_YAW_SLEW = 1.2          # rad/s / s — yaw hızı değişim tavanı


def clamp_roll(roll, limit=IRIS_MAX_BANK_DEG):
    """İstenen yatış (roll) açısını ±limit ile sınırla.
    Talep edilen fonksiyon: roll = max(min(roll, 45), -45)."""
    return max(min(roll, limit), -limit)


def _slew(cur, target, max_delta):
    """cur değerini target'a doğru en çok max_delta kadar yaklaştırır (yumuşatma)."""
    if target > cur + max_delta:
        return cur + max_delta
    if target < cur - max_delta:
        return cur - max_delta
    return target


class IrisManualCmd(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


def _iris_manual_thread():
    """Iris'i GUIDED'a alır, arm eder, ~3 m'ye kaldırır ve 10 Hz velocity
    setpoint döngüsüyle klavye komutlarını uygular."""
    global _iris_manual_active
    try:
        stop_iris_telem()          # 14541 portunu serbest bırak
        time.sleep(0.3)
        conn = df_connect_drone(port=14541)   # heartbeat ile target_system set edilir
        print(f"[IRIS-MANUAL] Bağlantı: target_sys={conn.target_system}")

        # GCS KEEPALIVE — 10 Hz heartbeat. Bağlantı kopmalarını (heartbeat
        # düşmesi) ve olası GCS failsafe'i önler; kamera/telemetri akışı stabil kalır.
        keepalive = GCSKeepalive(conn, interval=0.1)
        keepalive.start()

        # GUIDED + ARM + TAKEOFF (takeoff_to_z içeride GPS bekler, force-arm
        # eder, NAV_TAKEOFF yollar ve hedef irtifaya ulaşana dek bekler)
        print(f"[IRIS-MANUAL] GUIDED → ARM → TAKEOFF({IRIS_MANUAL_ALT} m)...")
        ok = df_takeoff(target_z=-abs(IRIS_MANUAL_ALT))
        print(f"[IRIS-MANUAL] Kalkış sonucu: {ok}")
        if not ok:
            print("[IRIS-MANUAL] ⚠ Kalkış doğrulanamadı — yine de hız döngüsüne geçiliyor")

        # ANGLE limiti: maksimum yatış açısını 45° (=4500 cd) ile sınırla →
        # ArduCopter herhangi bir manevrada bu açıdan fazla YATMAZ (roll clamp).
        # Proje parm'ı ANGLE_MAX'ı 55° (5500) yaptığından manevra sert; manuelde
        # 45°'ye çekiyoruz. Orijinal değeri okuyup çıkışta geri yüklüyoruz ki
        # chase/strike gibi diğer modlar etkilenmesin.
        _orig_angle_max = 5500.0
        try:
            conn.mav.param_request_read_send(
                conn.target_system, conn.target_component, b'ANGLE_MAX', -1)
            pv = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.5)
            if pv is not None and pv.param_id.strip('\x00') == 'ANGLE_MAX':
                _orig_angle_max = float(pv.param_value)
        except Exception:
            pass
        try:
            conn.mav.param_set_send(
                conn.target_system, conn.target_component,
                b'ANGLE_MAX', float(IRIS_MAX_BANK_DEG * 100.0),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            print(f"[IRIS-MANUAL] ANGLE_MAX {int(_orig_angle_max)}→{int(IRIS_MAX_BANK_DEG*100)} cd "
                  f"({IRIS_MAX_BANK_DEG:.0f}°) — maks yatış sınırlandı")
        except Exception as e:
            print(f"[IRIS-MANUAL] ANGLE_MAX ayarlanamadı: {e}")

        print("[IRIS-MANUAL] Hız (velocity) komut döngüsü başlıyor (10 Hz, yumuşatmalı)...")
        _tick = 0
        dt = 0.1
        # Yumuşatılmış (slew-limited) anlık komut değerleri — hedefe kademeli yaklaşır
        cvx = cvy = cvz = cyaw = 0.0
        while _iris_manual_active:
            # Sert manevrayı engelle: her tick'te hedefe en çok SLEW*dt kadar yaklaş
            cvx = _slew(cvx, _iris_manual_vx, IRIS_VEL_SLEW * dt)
            cvy = _slew(cvy, _iris_manual_vy, IRIS_VEL_SLEW * dt)
            cvz = _slew(cvz, _iris_manual_vz, IRIS_VEL_SLEW * dt)
            cyaw = _slew(cyaw, _iris_manual_yawrate, IRIS_YAW_SLEW * dt)
            _send_velocity_setpoint(
                conn, cvx, cvy, cvz, cyaw, body_frame=True,
            )
            _read_iris_telem_from_conn(conn)   # telemetriyi güncel tut
            _tick += 1
            if _tick % 10 == 0:                # ~saniyede bir
                print(f"[IRIS-MANUAL] cmd(hedef→yumuşak) vx={_iris_manual_vx:.1f}→{cvx:.1f} "
                      f"vy={_iris_manual_vy:.1f}→{cvy:.1f} vz={_iris_manual_vz:.1f}→{cvz:.1f} "
                      f"yaw={_iris_manual_yawrate:.2f}→{cyaw:.2f}")
            time.sleep(dt)

        # ANGLE_MAX'ı orijinaline geri yükle (diğer modlar sert manevra yapabilsin)
        try:
            conn.mav.param_set_send(
                conn.target_system, conn.target_component,
                b'ANGLE_MAX', float(_orig_angle_max),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            print(f"[IRIS-MANUAL] ANGLE_MAX geri yüklendi → {int(_orig_angle_max)} cd")
        except Exception:
            pass

        # Çıkış → önce sıfır hız (fren). GUIDED'da setpoint akışı kesilirse
        # ArduCopter failsafe'e girer; bu yüzden ya LAND (iniş) ya da LOITER
        # (yerinde hover) moduna alıyoruz.
        for _ in range(5):
            _send_velocity_setpoint(conn, 0, 0, 0, 0)
            time.sleep(0.05)
        if _iris_land_on_manual_exit:
            try:
                df_land()   # tam duruş → otonom iniş + disarm
                print("[IRIS-MANUAL] Döngü durdu → LAND (iniş).")
            except Exception as le:
                print(f"[IRIS-MANUAL] LAND hatası: {le}")
        else:
            # DANSA GEÇİŞ: inme! LOITER'a al → yerinde hover kalır, dans thread'i
            # devralır (LOITER GPS ile konum+irtifa tutar, setpoint gerekmez).
            try:
                mav_set_mode(conn, COPTER_MODE_LOITER)
                print("[IRIS-MANUAL] Dansa geçiş → LOITER (hover, iniş yok).")
            except Exception as le:
                print(f"[IRIS-MANUAL] LOITER hatası: {le}")
    except Exception as e:
        import traceback
        print(f"[IRIS-MANUAL] HATA: {e}")
        traceback.print_exc()
    finally:
        _iris_manual_active = False
        try:
            keepalive.stop()
        except Exception:
            pass
        start_iris_telem()         # pasif telemetriyi geri aç


@app.post("/api/command/iris/start_manual")
def iris_start_manual():
    global _iris_manual_active, _iris_manual_vx, _iris_manual_vy
    global _iris_manual_vz, _iris_manual_yawrate
    global _iris_manual_thread_obj, _iris_land_on_manual_exit, _iris_random_active
    if _chase_active or _strike_active:
        return {"status": "error", "message": "Önce Takip/Strike modunu kapatın"}
    if _iris_manual_active:
        return {"status": "success", "message": "Zaten manuel modda"}
    # Rastgele dans aktifse durdur (LOITER'da hover kalır) ve TAM durmasını bekle
    # (join) → 14541 port çakışması olmadan manuel devralsın.
    if _iris_random_active:
        _iris_random_active = False
        if _iris_random_thread_obj is not None:
            _iris_random_thread_obj.join(timeout=8.0)
        time.sleep(0.3)
    _iris_land_on_manual_exit = True   # normal manuel: durunca insin
    _iris_manual_vx = _iris_manual_vy = _iris_manual_vz = 0.0
    _iris_manual_yawrate = 0.0
    _iris_manual_active = True
    t = threading.Thread(target=_iris_manual_thread, daemon=True)
    _iris_manual_thread_obj = t
    t.start()
    print("[IRIS-MANUAL] Manuel uçuş thread'i başlatıldı.")
    return {"status": "success", "message": "Iris manuel uçuş: kalkış yapılıyor"}


@app.post("/api/command/iris/manual")
def iris_manual_cmd(cmd: IrisManualCmd):
    """Klavye → hız hedeflerini thread'e iletir (10 Hz akış)."""
    global _iris_manual_vx, _iris_manual_vy, _iris_manual_vz, _iris_manual_yawrate
    if not _iris_manual_active:
        return {"status": "skip"}
    _iris_manual_vx = cmd.vx
    _iris_manual_vy = cmd.vy
    _iris_manual_vz = cmd.vz
    _iris_manual_yawrate = cmd.yaw_rate
    return {"status": "success"}


@app.post("/api/command/iris/stop_manual")
def iris_stop_manual():
    global _iris_manual_active
    _iris_manual_active = False
    print("[IRIS-MANUAL] Manuel uçuş kapatıldı (hover).")
    return {"status": "success"}


# =======================================================================
# AVCI (İRİS) RASTGELE DANS — otonom daireler + sağ-sol gelişigüzel uçuş
# =======================================================================
# Bir tuşa (klavyede R) basınca Avcı drone kalkar ve GUIDED velocity setpoint'
# lerle DAİRELER çizip SAĞA-SOLA gelişigüzel hareket eder. İleri hız + yaw-rate
# = daire; rastgele yanal (vy) atışlar = sağ-sol; parametreler periyodik olarak
# rastgele değişir → "gelişi güzel". Slew ile yumuşatılır, akıcı görünür.
_iris_random_active = False
_iris_random_thread_obj = None   # dans→manuel geçişinde "tam dur"u bekle (join)
IRIS_RANDOM_ALT = 5.0        # dans irtifası (m) — daireler görünür olsun


def _iris_random_thread():
    """Avcı iris için OTONOM rastgele dans thread'i (GUIDED + velocity)."""
    global _iris_random_active
    keepalive = None
    try:
        stop_iris_telem()          # 14541 portunu serbest bırak
        time.sleep(0.3)
        conn = df_connect_drone(port=14541)
        print(f"[IRIS-RANDOM] Bağlantı: target_sys={conn.target_system}")
        keepalive = GCSKeepalive(conn, interval=0.1)
        keepalive.start()

        print(f"[IRIS-RANDOM] GUIDED → ARM → TAKEOFF({IRIS_RANDOM_ALT} m)...")
        ok = df_takeoff(target_z=-abs(IRIS_RANDOM_ALT))
        print(f"[IRIS-RANDOM] Kalkış: {ok}")
        if not ok:
            print("[IRIS-RANDOM] ⚠ Kalkış doğrulanamadı — yine de dansa geçiliyor")

        print("[IRIS-RANDOM] Rastgele dans döngüsü başlıyor (10 Hz)...")
        dt = 0.1
        t = 0.0
        next_change = 0.0
        # hedef manevra bileşenleri (periyodik rastgele seçilir)
        tgt_fwd = 1.5      # ileri hız (m/s) — daire yarıçapını yaw ile belirler
        tgt_yaw = 0.5      # yaw-rate (rad/s) — daire yönü/sıkılığı
        tgt_lat = 0.0      # yanal hız (m/s) — sağ(+)/sol(-) atış
        tgt_vz  = 0.0      # dikey hız (m/s) — hafif iniş/çıkış
        # yumuşatılmış (slew) anlık komutlar → akıcı hareket
        cfwd = cyaw = clat = cvz = 0.0
        _tick = 0
        while _iris_random_active:
            if t >= next_change:
                # yeni rastgele manevra parçası seç (daire + sağ-sol dart)
                tgt_fwd = random.uniform(0.8, 2.2)
                tgt_yaw = random.choice([-1, 1]) * random.uniform(0.35, 1.1)
                tgt_lat = random.uniform(-1.8, 1.8)
                tgt_vz  = random.uniform(-0.5, 0.5)
                next_change = t + random.uniform(1.5, 3.5)
            # yumuşat: ani sıçrama olmasın, dans akıcı görünsün
            cfwd = _slew(cfwd, tgt_fwd, IRIS_VEL_SLEW * dt)
            clat = _slew(clat, tgt_lat, IRIS_VEL_SLEW * dt)
            cvz  = _slew(cvz,  tgt_vz,  IRIS_VEL_SLEW * dt)
            cyaw = _slew(cyaw, tgt_yaw, IRIS_YAW_SLEW * dt)
            _send_velocity_setpoint(conn, cfwd, clat, cvz, cyaw, body_frame=True)
            _read_iris_telem_from_conn(conn)
            t += dt
            _tick += 1
            if _tick % 10 == 0:
                print(f"[IRIS-RANDOM] daire: ileri={cfwd:.1f} yanal={clat:.1f} "
                      f"vz={cvz:.1f} yaw={cyaw:.2f}")
            time.sleep(dt)

        # çıkış: dur (fren), sonra LOITER → YERİNDE HOVER kalır (inmez).
        # (GUIDED'da setpoint akışı kesilince failsafe olur; LOITER GPS ile
        #  konum+irtifayı setpoint olmadan tutar → drone asılı bekler.)
        for _ in range(5):
            _send_velocity_setpoint(conn, 0, 0, 0, 0)
            time.sleep(0.05)
        try:
            mav_set_mode(conn, COPTER_MODE_LOITER)
            print("[IRIS-RANDOM] Dans durdu → LOITER (yerinde hover, iniş yok).")
        except Exception as le:
            print(f"[IRIS-RANDOM] LOITER hatası: {le}")
    except Exception as e:
        import traceback
        print(f"[IRIS-RANDOM] HATA: {e}")
        traceback.print_exc()
    finally:
        _iris_random_active = False
        try:
            if keepalive is not None:
                keepalive.stop()
        except Exception:
            pass
        start_iris_telem()


@app.post("/api/command/iris/start_random")
def iris_start_random():
    """Avcı drone'u kaldırıp rastgele dans (daireler + sağ-sol) başlatır.
    Manuel uçuş aktifse KESİNTİSİZ geçer: manueli indirmeden LOITER'a alıp
    (thread'i join ederek) dans devralır — havada iken de sorunsuz."""
    global _iris_random_active, _iris_random_thread_obj
    global _iris_manual_active, _iris_land_on_manual_exit
    if _chase_active or _strike_active:
        return {"status": "error", "message": "Önce Takip/Strike modunu kapatın"}
    if _iris_random_active:
        return {"status": "success", "message": "Zaten rastgele dans modunda"}
    # Manuel uçuş aktifse → KESİNTİSİZ handoff: inme (LOITER'da bekle) + tam dur
    if _iris_manual_active:
        _iris_land_on_manual_exit = False       # inme, LOITER'da hover kal
        _iris_manual_active = False
        if _iris_manual_thread_obj is not None:
            _iris_manual_thread_obj.join(timeout=8.0)   # 14541 serbest kalsın
        _iris_land_on_manual_exit = True        # sonraki normal duruş için geri al
        time.sleep(0.3)
    _iris_random_active = True
    t = threading.Thread(target=_iris_random_thread, daemon=True)
    _iris_random_thread_obj = t
    t.start()
    print("[IRIS-RANDOM] Rastgele dans başlatıldı.")
    return {"status": "success", "message": "Rastgele dans: daireler + sağ-sol"}


@app.post("/api/command/iris/stop_random")
def iris_stop_random():
    global _iris_random_active
    _iris_random_active = False
    print("[IRIS-RANDOM] Rastgele dans kapatıldı (LOITER hover).")
    return {"status": "success"}


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


# Kamera watchdog: her kaynağın son kare zamanı.
#
# ÖNEMLİ (kök neden düzeltmesi): Gazebo Harmonic kamera sensörleri
# <always_on>1</always_on> ile SÜREKLİ yayınlar; asıl kırılganlık gz-transport
# ABONE tarafındadır. Eski kod her 3 sn'de bir YENİ `GzNode()` yaratıp yeniden
# abone oluyordu → "subscriber churn". Harmonic lazy-render kameraları bu churn
# yüzünden KALICI olarak donuyordu (loglarda sonsuz "yeniden abone" döngüsü,
# görüntü hiç geri gelmiyordu). Ayrıca resubscribe sonrası _gz_cam_last sıfırlanıp
# gerçek başarısızlık maskeleniyordu.
#
# Yeni tasarım:
#  - TEK, KALICI node (global sözlükte tutulur → asla GC edilmez, churn yok).
#  - Tek abonelik; akış sağlıklıysa ona DOKUNULMAZ (memory: "don't churn").
#  - Gerçek stall'da (uzun süre kare yok) SON ÇARE olarak AYNI node üzerinde tek
#    bir unsubscribe+subscribe denenir, seyrek (throttle) ve _gz_cam_last'ı sahte
#    sıfırlamadan; böylece kareler dönerse cb() saati günceller, dönmezse spam yok.
_gz_cam_last = {"iris": 0.0, "plane": 0.0}
_gz_cam_stalled = {"iris": False, "plane": False}
_gz_nodes = {}                 # name -> persistent GzNode (GC koruması)
_GZ_CAM_STALL_SEC = 5.0        # bu süre kare gelmezse "donmuş" say
_GZ_CAM_RESUB_SEC = 8.0        # kurtarma denemeleri arası minimum süre (churn'ü sınırla)


def _gz_camera_reader(name, topic_env, default_topic, process_fn):
    """Tek bir gz-transport kamera topic'ini dinler. Kalıcı node + churn'süz
    watchdog: akış donarsa seyrek/tek bir yeniden abonelik dener."""
    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.image_pb2 import Image as GzImage
    except Exception as e:
        print(f"[GCS] gz-transport Python yok, {name} kamera atlandı: {e}")
        return

    topic = os.environ.get(topic_env, default_topic)

    def cb(msg):
        try:
            _gz_cam_last[name] = time.time()
            if _gz_cam_stalled.get(name):
                _gz_cam_stalled[name] = False
                print(f"[GCS] ✓ {name} kamera akışı geri geldi")
            # Kanal sayısını veriden türet (RGB8=3, MONO8=1) — reshape hatasına karşı
            npix = msg.width * msg.height
            ch = (len(msg.data) // npix) if npix else 3
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if ch >= 3:
                img = arr[:npix * 3].reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = cv2.cvtColor(arr[:npix].reshape((msg.height, msg.width)),
                                   cv2.COLOR_GRAY2BGR)
            process_fn(img)
        except Exception as e:
            print(f"[GCS GZ-CAM] {name} hata: {e}")

    # TEK kalıcı node — global'de tutulur ki GC etmesin (churn'ü önler)
    node = GzNode()
    _gz_nodes[name] = node
    node.subscribe(GzImage, topic, cb)
    _gz_cam_last[name] = time.time()
    print(f"[GCS] gz-transport {name} kamera dinleniyor ({topic}, Harmonic)")

    # WATCHDOG — akışa DOKUNMA; yalnızca gerçek stall'da seyrek kurtarma dene
    last_resub = 0.0
    while True:
        time.sleep(1.0)
        idle = time.time() - _gz_cam_last.get(name, 0.0)
        if idle <= _GZ_CAM_STALL_SEC:
            continue
        if not _gz_cam_stalled.get(name):
            _gz_cam_stalled[name] = True
            print(f"[GCS] ⚠ {name} kamera {idle:.0f}s kare göndermedi (stall)")
        # Kurtarmayı throttle et: AYNI node üzerinde tek unsubscribe+subscribe.
        if time.time() - last_resub < _GZ_CAM_RESUB_SEC:
            continue
        last_resub = time.time()
        print(f"[GCS] … {name} kamera tek yeniden abonelik deneniyor ({topic})")
        try:
            try:
                node.unsubscribe(topic)
            except Exception:
                pass
            node.subscribe(GzImage, topic, cb)
        except Exception as e:
            print(f"[GCS] {name} kamera yeniden abone hatası: {e}")


def gz_iris_camera_thread():
    """Gazebo Harmonic: iris kamerasını gz-transport'tan oku (watchdog'lu)."""
    _gz_camera_reader("iris", "AVCI_GZ_CAMERA_TOPIC", "/iris_cam/image", process_iris_frame)


def process_plane_frame(img):
    """Hedef İHA (Talon) burun kamerası: ham görüntü → MJPEG. Iris'ten farkı:
    tespit/overlay YOK (bu hedefin kendi görüşü, avcının değil)."""
    _, buf = cv2.imencode('.jpg', img)
    if latest_frames["plane"]["data"] is None:
        print("[GCS] ✓ Talon (hedef İHA) kamerasından ilk görüntü!")
    latest_frames["plane"]["data"] = buf.tobytes()
    latest_frames["plane"]["id"] += 1


def gz_talon_camera_thread():
    """Gazebo Harmonic: Talon (hedef İHA) burun kamerasını oku (watchdog'lu)."""
    _gz_camera_reader("plane", "AVCI_GZ_TALON_TOPIC", "/talon_cam/image", process_plane_frame)


def ros2_spin_thread():
    """Gazebo Classic yolu: ROS 2 kameralarını cv_bridge ile dinler.
    rclpy/cv_bridge importları BURADA yapılır (lazy) — Harmonic modunda ROS 2
    kurulu değilse sunucunun geri kalanı etkilenmesin."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except Exception as e:
        print(f"[GCS] ROS 2 (rclpy/cv_bridge) yüklenemedi, ROS2 kamera atlandı: {e}")
        return

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
                process_plane_frame(self.bridge.imgmsg_to_cv2(data, "bgr8"))
            except Exception as e:
                print(f"[GCS CAM] Plane hata: {e}")

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

async def _mavlink_heartbeat_loop():
    """GCS heartbeat'ini kesintisiz 1 Hz gönder — bağlantı canlılığı için.
    (Manuel modlar kendi 10 Hz GCSKeepalive'ını çalıştırdığından, plane manuel
    aktifken çakışmayı önlemek için burada atlanır.)"""
    while True:
        try:
            if _mav_conn is not None and not _manual_active:
                _mav_conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            pass
        await asyncio.sleep(1.0)


@app.on_event("startup")
async def startup_event():
    asyncio.create_task(mavlink_listener())          # plane — 14550
    asyncio.create_task(_mavlink_heartbeat_loop())   # 1 Hz GCS heartbeat
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
    _t = 0
    try:
        while True:
            # GPS noise seviyesini de frontend'e gönder
            payload = dict(telemetry_state)
            payload["gps_noise"] = _gps_noise_level
            payload["gps_frozen"] = _noisy_plane_telem.get("frozen", False)
            payload["plane_throttle"] = _plane_throttle
            payload["ts"] = time.time()          # ping/pong: canlılık damgası
            try:
                await websocket.send_json(payload)
            except Exception:
                break                             # bağlantı koptu → çık, frontend reconnect eder
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


@app.websocket("/ws/video/{vehicle}")
async def video_ws(websocket: WebSocket, vehicle: str):
    """Kamera karelerini WebSocket üzerinden base64 olarak canlı yayınlar.
    MJPEG <img> tarayıcıda takılabildiği için asıl video hattı budur:
    HER ZAMAN EN SON kareyi gönderir (buffer'da eski kare birikmez),
    frontend img.src = data:image/jpeg;base64 ile anında çizer."""
    if vehicle not in ("iris", "plane"):
        vehicle = "iris"
    await websocket.accept()
    last_id = -1
    try:
        while True:
            entry = latest_frames.get(vehicle)
            if entry and entry["data"] is not None and entry["id"] != last_id:
                last_id = entry["id"]                       # yalnızca en son kare
                b64 = base64.b64encode(entry["data"]).decode("ascii")
                try:
                    await websocket.send_text(b64)
                except Exception:
                    break
            await asyncio.sleep(0.045)                       # ~22 FPS tavan
    except WebSocketDisconnect:
        pass
    except Exception:
        pass

if __name__ == "__main__":
    print("==================================================")
    print(" AVCI GCS SERVER BAŞLATILIYOR (Port: 8000)")
    print("==================================================")
    uvicorn.run(app, host="0.0.0.0", port=8000)