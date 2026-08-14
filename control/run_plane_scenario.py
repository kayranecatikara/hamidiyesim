#!/usr/bin/env python3
"""
run_plane_scenario.py — Hedef İHA (sabit kanat) uçuş senaryoları.

Kullanım:
    python -m control.run_plane_scenario square      # kare çiz
    python -m control.run_plane_scenario circle      # daire çiz
    python -m control.run_plane_scenario aggressive  # rastgele agresif manevralar
    python -m control.run_plane_scenario kare_gorev  # kalkış → N kare → iniş (biter)

Akış: bağlan → force ARM → TAKEOFF modu ile otonom kalkış → FBWA + RC
override ile seçilen desen. Desen, GCS süreci öldürene (manuel moda geçiş
veya durdur butonu) kadar süresiz döner.

Kare dönüşleri PUSULA (ATTITUDE yaw) tabanlıdır: FBWA'da roll komutu verilir,
heading 90° değişince kenara geçilir. (Kaldırılan eski run_plane_square zaman bazlı
rudder(yaw) dönüşü kullanıyordu — FBWA'da rudder tek başına dönüş üretmediği
için kare bozuktu.)

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
)
from control.mav_common import (
    set_mode,
    disarm,
    PLANE_MODE_TAKEOFF,
    PLANE_MODE_FBWA,
)

# Havada devralma eşiği: bu irtifanın üstünde armlıysak kalkış ATLANIR.
AIRBORNE_ALT_M = 15.0

CONTROL_RATE = 0.05   # 20 Hz komut döngüsü

_abort = False

# _pump ile güncellenen son telemetri
_att = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "ok": False}
_pos = {"x": 0.0, "y": 0.0, "z": 0.0,
        "vx": 0.0, "vy": 0.0, "vz": 0.0,
        "t": 0.0}                          # x=kuzey, y=doğu (NED, m); t=son mesaj
# İniş raporu için arm durumu (temas sonrası force disarm doğrulanır).
_hb = {"armed": False, "mode": None, "ok": False}

# KALKIŞ / FIRLATMA NOKTASI (yerel NED x=kuzey, y=doğu). Kaynağı için
# _ev_noktasi()'ye bak — varsayılan otopilotun HOME'udur.
_kalkis_xy = None

# Küresel konum ve HOME — HOME'u yerel çerçeveye taşımak için ikisi de gerekir.
_gpos = {"lat": 0.0, "lon": 0.0, "t": 0.0}
_home = {"lat": 0.0, "lon": 0.0, "t": 0.0}
_M_PER_DEG = 111319.4907          # metre / enlem derecesi

# Fırlatma noktası kaynağı: "home" (otopilotun HOME'u) | "yerel" (eski davranış:
# script başlarkenki LOCAL_POSITION_NED). Kıyas/geri dönüş için env ile seçilir.
_EV_KAYNAK  = os.environ.get("AVCI_EV_KAYNAK", "home")
_EV_TIMEOUT = float(os.environ.get("AVCI_EV_TIMEOUT", "8.0"))


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
            _pos["x"] = msg.x            # kuzey (m) — kenar MESAFESİ buradan ölçülür
            _pos["y"] = msg.y            # doğu  (m)
            _pos["z"] = msg.z
            _pos["vx"] = msg.vx          # yer hızı → iniş burun denetimi
            _pos["vy"] = msg.vy
            _pos["vz"] = msg.vz          # NED: pozitif = AŞAĞI (iniş döngüsü)
            _pos["t"] = time.time()      # tazelik: bayat/hiç-gelmemiş ayrımı
        elif t == "GLOBAL_POSITION_INT":
            _gpos.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, t=time.time())
        elif t == "HOME_POSITION":
            _home.update(lat=msg.latitude / 1e7, lon=msg.longitude / 1e7,
                         t=time.time())
        elif t == "HEARTBEAT" and msg.get_srcSystem() == conn.target_system:
            _hb.update(ok=True, mode=msg.custom_mode, armed=bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))


def _ev_noktasi(conn, timeout=None):
    """FIRLATMA/KALKIŞ noktasını belirle ve yerel NED çerçevesine taşı.

    ── A: KAYNAK OTOPİLOTUN HOME'U ────────────────────────────────────────
    Eski sürüm, script başlarkenki LOCAL_POSITION_NED'i "ev" sayıyordu. Bu
    iki nedenle fırlatma noktası DEĞİLDİR:
      1) LOCAL_POSITION_NED, HOME'a değil EKF ORİJİNİNE görelidir
         (ArduPilot: GCS_MAVLINK::send_local_position →
          ahrs.get_relative_position_NED_origin_float). EKF orijini aracın
         AÇILDIĞI yerde kurulur ve bir daha oynamaz.
      2) ArduPlane HOME'u, araç DİSARM iken 5 saniyede bir GPS konumuna taşır
         (ArduPlane/Plane.cpp: !arming.is_armed() && ... update_home()).
      Yani "boot yerinde açtım, 200 m öteye yürüyüp fırlattım" akışında HOME
      fırlatma yerine oturur, EKF orijini boot yerinde kalır — ikisi ayrışır.

    Bu yüzden ev, otopilottan HOME_POSITION olarak istenir ve o an eşleşen
    (GLOBAL_POSITION_INT, LOCAL_POSITION_NED) çiftiyle yerel çerçeveye
    taşınır. Düz-dünya dönüşümü birkaç km'de yeterlidir.

    ── C: EKSİK VERİYLE TAHMİN ÜRETİLMEZ ──────────────────────────────────
    Eski sürümün en sinsi tarafı, veri yokken sessizce (0,0)'a düşmesiydi:
    LOCAL_POSITION_NED hiç gelmezse (EKF konumu yoksa firmware bu mesajı
    GÖNDERMEZ) ev "kuzey +0.0 doğu +0.0" olarak basılıyordu ve bu, meşru bir
    yakalamadan ayırt edilemiyordu. Artık üç veri de TAZE değilse None döner;
    çağıran görevi başlatmaz.

    Dönüş: (x, y) yerel NED  |  None (kaynak güvenilir değil)
    """
    timeout = _EV_TIMEOUT if timeout is None else timeout

    if _EV_KAYNAK == "yerel":
        # ESKİ DAVRANIŞ — yalnız kıyas/geri dönüş için.
        _pump(conn)
        if _pos["t"] == 0.0:
            print("[EV] ⚠ LOCAL_POSITION_NED hiç gelmedi — ev belirlenemiyor")
            return None
        print(f"[EV] (yerel kaynak) ev = script başındaki konum: "
              f"kuzey {_pos['x']:+.1f} doğu {_pos['y']:+.1f}")
        return (_pos["x"], _pos["y"])

    t0 = time.time()
    istek = 0.0
    while time.time() - t0 < timeout:
        _pump(conn)
        if time.time() - istek > 1.0:          # HOME'u iste (1 Hz tekrar)
            istek = time.time()
            # Birincil yol: REQUEST_MESSAGE (HOME_POSITION). ArduPilot bunu
            # MSG_HOME'a eşler; GET_HOME_POSITION eski/alternatif yoldur.
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0, 0, 0)
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
        simdi = time.time()
        taze = (simdi - _home["t"] < 5.0 and _home["t"] > 0
                and simdi - _gpos["t"] < 3.0 and _gpos["t"] > 0
                and simdi - _pos["t"] < 3.0 and _pos["t"] > 0)
        if taze:
            # HOME lat/lon → yerel NED (o anki küresel/yerel çifti üzerinden)
            kuzey = (_home["lat"] - _gpos["lat"]) * _M_PER_DEG
            dogu = ((_home["lon"] - _gpos["lon"]) * _M_PER_DEG
                    * math.cos(math.radians(_gpos["lat"])))
            ev_x, ev_y = _pos["x"] + kuzey, _pos["y"] + dogu
            fark = math.hypot(ev_x - _pos["x"], ev_y - _pos["y"])
            if fark > 5000.0:
                print(f"[EV] ⚠ HOME araca {fark:.0f} m uzakta — akıl dışı, "
                      f"ev belirlenemiyor")
                return None
            print(f"[EV] HOME (otopilot) = {_home['lat']:.7f}, {_home['lon']:.7f}")
            print(f"[EV] Araç şu an     = {_gpos['lat']:.7f}, {_gpos['lon']:.7f} "
                  f"| yerel kuzey {_pos['x']:+.1f} doğu {_pos['y']:+.1f}")
            print(f"[EV] EV (yerel NED) = kuzey {ev_x:+.1f} doğu {ev_y:+.1f} "
                  f"(araçtan {fark:.0f} m)")
            # Eski yöntem ne derdi — farkı görünür kıl (kıyas için)
            eski = math.hypot(ev_x - _pos["x"], ev_y - _pos["y"])
            print(f"[EV] Eski yöntem (script başındaki konum) ile arasındaki "
                  f"fark: {eski:.0f} m")
            return (ev_x, ev_y)
        time.sleep(0.2)

    eksik = []
    if _home["t"] == 0: eksik.append("HOME_POSITION")
    if _gpos["t"] == 0: eksik.append("GLOBAL_POSITION_INT")
    if _pos["t"] == 0: eksik.append("LOCAL_POSITION_NED")
    print(f"[EV] ⚠ Fırlatma noktası belirlenemedi ({timeout:.0f} s). "
          f"Gelmeyen/bayat: {', '.join(eksik) if eksik else 'tazelik penceresi'}")
    return None


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


def duz_git(conn, mesafe_m, zaman_asimi=None):
    """Yer üzerinde mesafe_m kadar DÜZ uç ve dur. Ölçüt SÜRE DEĞİL MESAFEDİR.

    NEDEN (2026-08-13, kullanıcı itirazı): kenarlar `hold(conn, 5.0)` ile
    SÜREYE bağlıydı; kenar uzunluğu o anki yer hızına göre değişiyordu ve
    "kare" istenen ölçüde çıkmıyordu. Kenar artık metre cinsinden verilir,
    kat edilen yol LOCAL_POSITION_NED'den ölçülür.

    Dönüş: gerçekten kat edilen mesafe (m) — çağıran raporlayabilsin.
    """
    _pump(conn)
    x0, y0 = _pos["x"], _pos["y"]
    if zaman_asimi is None:                # 5 m/s'lik en kötü hâlde bile yeter
        zaman_asimi = max(10.0, mesafe_m / 5.0)
    t0 = time.time()
    kat = 0.0
    while not _abort and time.time() - t0 < zaman_asimi:
        _pump(conn)
        kat = math.hypot(_pos["x"] - x0, _pos["y"] - y0)
        if kat >= mesafe_m:
            break
        _rc(conn, roll=0, pitch=0, throttle=gcs_throttle())
        time.sleep(CONTROL_RATE)
    return kat


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


def _hedefe(hx, hy):
    """(mesafe_m, kerteriz_rad) — NED: x kuzey, y doğu; kerteriz atan2(doğu, kuzey)."""
    dx, dy = hx - _pos["x"], hy - _pos["y"]
    return math.hypot(dx, dy), math.atan2(dy, dx)


def git_noktaya(conn, hx, hy, varis_m=60.0, bank=650, timeout=180.0,
                hedef_irtifa=None):
    """Verilen yerel NED noktasına uç; yatış kerteriz hatasıyla orantılı.

    hedef_irtifa verilirse yol boyunca O İRTİFAYA TIRMANIR (kullanıcı isteği
    2026-08-13: "kare bittikten sonra irtifayı yükseltsin"). Tırmanış dönüş
    bacağıyla BİRLEŞTİRİLİR — ayrı bir tırmanma turu atılmaz.

    Dönüş: varışta kalan mesafe (m).
    """
    t0 = time.time()
    mesafe = _hedefe(hx, hy)[0]
    while not _abort and time.time() - t0 < timeout:
        _pump(conn)
        mesafe, kerteriz = _hedefe(hx, hy)
        irtifa = -_pos["z"]
        if mesafe <= varis_m and (hedef_irtifa is None
                                  or irtifa >= hedef_irtifa - 2.0):
            break
        hata = _angdiff(kerteriz, _att["yaw"])
        oran = _kelepce(hata / math.radians(45.0), -1.0, 1.0)
        if hedef_irtifa is not None and irtifa < hedef_irtifa - 2.0:
            burun, gaz = 250, 850                 # tırmanış
        else:
            burun, gaz = (100 if abs(oran) > 0.3 else 0), gcs_throttle()
        _rc(conn, roll=int(bank * oran), pitch=burun, throttle=gaz)
        time.sleep(CONTROL_RATE)
    return mesafe


def donus_yap(conn, deg, bank=800, tolerans_deg=1.0, timeout=25.0):
    """Kapalı çevrim dönüş: yatış, KALAN heading hatasıyla orantılı verilir.

    NEDEN turn_by'dan AYRI (2026-08-13, ölçüldü): turn_by 10° toleransla
    döngüden çıkıp yatış komutunu kesiyor; uçak o an 36° yatıkta ve ~27°/s
    dönüyor, kanatları düzeltmesi ~1 s sürüyor → dönüş 90° değil +113°
    oluyordu. Dört köşede 452°: şekil kareye değil ÜÇGENE kapanıyordu
    (iz_150m yer izi ölçümü). Yatış hatayla orantılı verilince uçak dönüşün
    sonuna doğru kendiliğinden düzleşir, savurma kalmaz.

    turn_by'a DOKUNULMADI: `square`/`aggressive` ve kampanya senaryoları onu
    kullanıyor, ölçüm geçmişiyle kıyaslanabilir kalsın.

    TOLERANS 1.0°: oransal yatışla hataya asimptotik yaklaşıldığı için dönüş
    TAM toleransın sınırında kesiliyor — 3.0° ile dört köşenin dördü de 87°
    ölçüldü (tur başına 12° kayma; tek turda görünmez, üç turda şekil döner).

    Dönüş: gerçekleşen heading değişimi (derece) — çağıran doğrulayabilsin.
    """
    _pump(conn)
    if not _att["ok"]:
        hold(conn, 1.0)
        _pump(conn)
    bas_yaw = _att["yaw"]
    hedef = bas_yaw + math.radians(deg)
    t0 = time.time()
    while not _abort and time.time() - t0 < timeout:
        _pump(conn)
        hata = _angdiff(hedef, _att["yaw"])
        if abs(hata) < math.radians(tolerans_deg):
            break
        # 45°+ hatada tam yatış, altında orantılı → çıkışta kanatlar düz
        oran = _kelepce(hata / math.radians(45.0), -1.0, 1.0)
        _rc(conn, roll=int(bank * oran), pitch=180, throttle=gcs_throttle())
        time.sleep(CONTROL_RATE)
    _pump(conn)
    return math.degrees(_angdiff(_att["yaw"], bas_yaw))


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
            _pos["x"] = msg.x            # kuzey (m) — kenar MESAFESİ buradan ölçülür
            _pos["y"] = msg.y            # doğu  (m)
            _pos["z"] = msg.z
            _pos["vx"] = msg.vx          # yer hızı → iniş burun denetimi
            _pos["vy"] = msg.vy
            _pos["vz"] = msg.vz          # NED: pozitif = AŞAĞI (iniş döngüsü)
            _pos["t"] = time.time()      # tazelik damgası — _pump ile aynı
        elif t == "ATTITUDE":
            _att.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, ok=True)
    return armed, -_pos["z"]


def takeoff(conn, climb_time=8.0, hedef_alt=None):
    """Otonom kalkış: TAKEOFF modu motoru açıp TKOFF_ALT'a tırmandırır,
    ardından FBWA'ya geçilip kısa düz uçuşla stabilize edilir.

    ⚠ İRTİFA KAPISI (2026-08-11, TEST ALTYAPISI): sabit climb_time=8 s ÖLÇÜLDÜ —
    uçak yerde ~4 s hızlanma yiyor, TAKEOFF penceresinde yalnız ~4.5 m'ye
    çıkabiliyor, FBWA + bekleme dairesi o irtifada banklayınca takla atıyordu
    (3/3 kaza, roll→55°, canlı trace ile doğrulandı). Artık güvenli irtifaya
    (AVCI_TKOFF_HEDEF, vars. 30 m) çıkana YA DA üst-zaman aşımına (climb_time'ın
    en az 30 s'i) kadar TAKEOFF'ta kalınır. Güdüme dokunmaz (hedefin kalkışı)."""
    if hedef_alt is None:
        hedef_alt = float(os.environ.get("AVCI_TKOFF_HEDEF", 30.0))
    ust_sure = max(climb_time, 30.0)
    print(f"[SCN] Otonom kalkış (TAKEOFF modu, hedef ~{hedef_alt:.0f} m)...")
    set_mode(conn, PLANE_MODE_TAKEOFF)
    t0 = time.time()
    while not _abort and time.time() - t0 < ust_sure:
        _pump(conn)
        if -_pos["z"] >= hedef_alt:
            break
        time.sleep(0.2)
    print(f"[SCN] Kalkış bitti (irtifa ~{-_pos['z']:.0f}m) → FBWA")
    set_mode(conn, PLANE_MODE_FBWA)
    hold(conn, 2.0)


# ---------------------------------------------------------------------------
# OTONOM İNİŞ — motorlu alçalma + flare + disarm
# ---------------------------------------------------------------------------
# ⚠ NEDEN ArduPlane'in KENDİ İNİŞİ (AUTO + NAV_LAND) KULLANILMIYOR
# (2026-08-13, canlı SITL'de ölçüldü — BIN 00000461):
#   İlk sürüm uçağa yaklaşma noktası + NAV_LAND'li geçici bir AUTO görevi
#   yüklüyordu. Görev kabul edildi, uçak AUTO'ya geçti ve YERE ÇAKILDI:
#   TECS hava hızını hedefte tuttu (sp≈spdem≈15.5 m/s, yani stall YOK) ama
#   alçalma için gazı sıfırladı (th=0.0) ve uçak 100 m irtifayı 16 saniyede,
#   -68.8° burun aşağı (ATT), 15.6 m/s ile çarparak kaybetti.
#   ÖLÇÜLEN KÖK NEDEN: Gazebo mini_talon'un motorsuz SÜZÜLME ORANI ≈ 1.4:1
#   (100 m irtifa / 139 m yatay yol; gerçek Talon ~10:1). Bu modelde gaz kesik
#   alçalma "süzülüş" değil DÜŞÜŞTÜR — ArduPlane'in iniş dizisi ise son
#   yaklaşmada gazı keser. Yani hata güdümde değil, modelin süzülme
#   performansındadır ve otopilotun iniş profili bu araca uymaz.
#
# ÇÖZÜM: iniş, motoru AÇIK tutan kendi alçalma döngümüzle yapılır — dosyanın
# geri kalanıyla aynı makine (FBWA + RC override).
#
# ⚠ GERÇEK TALON İÇİN YUMUŞATILDI (2026-08-13, kullanıcı isteği): ilk sürüm
# 2.5 m/s alçalıyor ve burnu SABİT AŞAĞI (-40) tutuyordu; ölçülen izde uçak
# temastan hemen önce -26° burun aşağıydı — gerçek uçakta bu pervane/burun
# çakması demek. Yeni profil, gerçek bir gövde inişinin (belly landing)
# kademelerini uygular:
#
#   Kademe        irtifa        hedef alçalma   burun
#   ─────────────────────────────────────────────────────────────────
#   YAKLAŞMA      > SON_ALT     SINK (1.2 m/s)  hıza göre (denetimli)
#   SON YAKLAŞMA  SON_ALT..FLARE SON_SINK (0.5) hıza göre + hafif yukarı
#   FLARE         < FLARE_ALT   FLARE_SINK(0.2) YUKARI (gövde önce değsin)
#
# İki ayrı döngü, uçak inişinin standart iş bölümü:
#   • GAZ  → alçalma hızını tutar (bu model motorsuz süzülemediği için gaz
#     inişin sonuna kadar açık kalır; sadece flare'de kademeli kısılır).
#   • BURUN → HIZI tutar (yavaşsa burun aşağı, hızlıysa yukarı). Sabit burun
#     aşağı komutu yoktur; flare'de üstüne yukarı biası binerek uçak burnu
#     yukarı, gövdesi üstüne oturur.
#
# ⚠ GERÇEK UÇUŞA TAŞIRKEN: buradaki "hız" LOCAL_POSITION'dan gelen YER HIZIDIR.
# Simde rüzgâr yok, o yüzden hava hızına eşit. Gerçek Talon'da rüzgâr varken
# yaklaşma hava hızıyla (ARSPD / VFR_HUD.airspeed) sürülmelidir; yer hızıyla
# uçulursa rüzgâra karşı inişte uçak stall'a yaklaşır.
# Uçak BULUNDUĞU yere iner; kare deseni evin ~100 m çevresinde uçtuğu için
# iniş noktası da sahanın içinde kalır (piste hizalı iniş HEDEFLENMEZ).
_INIS_SINK       = float(os.environ.get("AVCI_INIS_SINK", 0.8))       # yaklaşma alçalma m/s
_INIS_SON_SINK   = float(os.environ.get("AVCI_INIS_SON_SINK", 0.4))   # son yaklaşma
_INIS_FLARE_SINK = float(os.environ.get("AVCI_INIS_FLARE_SINK", 0.15)) # flare
_INIS_SON_ALT    = float(os.environ.get("AVCI_INIS_SON_ALT", 15.0))
_INIS_FLARE_ALT  = float(os.environ.get("AVCI_INIS_FLARE_ALT", 5.0))
# ⚠ YAKLAŞMA HIZI (ölçüldü 2026-08-13): 15.0 ile uçak flare'e 11.7 m/s ile
# girdi — AIRSPEED_MIN 12'nin ALTI. Burun yukarı komutu orada stall'a çevirdi:
# burun 1 saniyede -26.8°'ye çöktü ve model yere çakılıp araziden geçti.
# Gerçek uçakta yaklaşma hızı ~1.3 × stall alınır: 12 × 1.35 ≈ 16.5.
_INIS_HIZ        = float(os.environ.get("AVCI_INIS_HIZ", 16.5))       # yaklaşma hızı m/s
_INIS_STALL_HIZ  = float(os.environ.get("AVCI_INIS_STALL_HIZ", 12.5)) # altında flare kısılır
# ⚠ GAZ TABANI (ölçüldü 2026-08-13): taban 350 ile alçalma hedefi 1.2 m/s iken
# gerçekleşen 4.1 m/s oldu. Sebep bu modelin süzülme oranı (1.4:1): gaz düşünce
# hız denetimi burnu aşağı alıyor ve yaklaşma dalışa dönüyor. Düz uçuş ~%60 gaz
# istiyor; yumuşak (~1 m/s) alçalma için gaz onun BİRAZ altında tutulmalı, bu
# yüzden yaklaşmada gaz tabanı 450'nin altına indirilmez.
_INIS_GAZ_TABAN  = int(os.environ.get("AVCI_INIS_GAZ", 500))
_INIS_GAZ_ALT_SINIR = int(os.environ.get("AVCI_INIS_GAZ_MIN", 450))
# ⚠ FLARE BURUN KOMUTU (ölçüldü 2026-08-13): 260 yetmedi — temas -11.6° burun
# AŞAĞI oldu. Sebep FBWA ölçeklemesi: komut 1000 = tam stick = LIM_PITCH_MAX
# (~20°), yani 260 yalnız ~5°'lik bir HEDEF demek ve alçalan uçak onu bile
# tutamıyor. Gerçek flare tam stick'e yakın çekilir; uçak son metrede hızı
# irtifaya çevirip gövdesi üstüne oturur.
# ⚠ 520 → 300 (2026-08-13, tanılama uçuşu). Ölçüm: motor açıkken FBWA burun
# komutunu İZLEMİYOR — komut 433→497 tırmanırken gerçek açı -1.6°'de sabit
# kaldı (düşük hızda otopilot burun-yukarı talebini kısıyor). Gaz 0.4 m'de
# kesilince biriken talep boşalıp burnu +24.5°'ye fırlattı, hız 12.6→9.5 m/s.
# Uçağın izleyebileceği bir komut + kademeli gaz kesme ile bu sıçrama kalkar.
_INIS_FLARE_PITCH = int(os.environ.get("AVCI_INIS_FLARE_PITCH", 450))
# 200 → 260: temas -2.9° (hafif burun aşağı) ölçülüyordu; rotasyonun daha ERKEN
# başlaması gövdenin önce değmesini sağlıyor. Tepe 520'de tutuluyor — 700
# denenmişti ve uçağı stall'a sokup burnu -26.8°'ye düşürmüştü.
_INIS_FLARE_PITCH_BAS = int(os.environ.get("AVCI_INIS_FLARE_PITCH_BAS", 200))
_INIS_ZAMAN_ASIMI = float(os.environ.get("AVCI_INIS_ZAMAN_ASIMI", 240.0))
# Yaklaşma giriş mesafesi: 30 m irtifayı 13 m/s'de indirmek için gereken yol
# ~1:13 eğimde 400 m. Daha kısa alınırsa süzülüş dikleşir, daha uzun alınırsa
# uçak gereksiz yol yapar.
# ⚠ GİRİŞ MESAFESİ SABİT DEĞİL, İRTİFADAN HESAPLANIR (2026-08-13, ölçüm):
# sabit 400 m ile uçak kare bitiminde zaten evden 160-227 m uzaktayken İLERİ
# doğru ~200 m daha uçup 180° dönüyordu — saf kayıp. Gereken mesafe basitçe
# irtifa / tan(süzülüş açısı); uçak o mesafenin dışındaysa dışarı çıkmaya gerek
# yoktur, doğrudan süzülüşe geçilir.
_INIS_EGIM_DEG   = float(os.environ.get("AVCI_INIS_EGIM_DEG", 8.0))
_INIS_GIRIS_MIN  = float(os.environ.get("AVCI_INIS_GIRIS_MIN", 150.0))
_INIS_GIRIS_MAX  = float(os.environ.get("AVCI_INIS_GIRIS_MAX", 600.0))
# Kare bitince tırmanılacak dönüş irtifası (0 = tırmanma).
# ⚠ 60 → 30 m (2026-08-13, kullanıcı itirazı "iniş için bu kadar yükseğe
# çıkmaya gerek yok" + ölçüm). Belirleyici olan İRTİFA DEĞİL, flare'e hedefe
# ~90 m kala girmek:
#   dönüş 60 m → flare hedefe 87 m kala  → temas 1-4 m
#   dönüş 30 m → flare hedefe 94 m kala  → temas 3 m      ← seçildi
#   dönüş YOK  → flare hedefe 199 m kala → temas 29 m (irtifa erken tükeniyor,
#                son 200 m sürünerek geliniyor ve uçak kısa kalıyor)
# 30 m ile yaklaşma yolu 467 → 253 m'ye, en uzak nokta 462 → 233 m'ye iniyor.
_DONUS_IRTIFA    = float(os.environ.get("AVCI_DONUS_IRTIFA", 30.0))
# kare_gorev'e ÖZEL desen irtifası. Kampanya senaryoları (duz/square/circle/
# aggressive) AVCI_TKOFF_HEDEF=30 ile uçmaya devam eder — ölçüm geçmişi bozulmasın.
# 20 m: kullanıcı isteği ("iniş için bu kadar yükseğe çıkmaya gerek yok");
# dönüşteki 30 m'lik tırmanışla birlikte temas 3 m ölçüldü.
_KARE_IRTIFA     = float(os.environ.get("AVCI_KARE_IRTIFA", 20.0))
# ⚠ NİŞAN NOKTASI HEDEFTEN GERİDEDİR (ölçüldü 2026-08-13): süzülüş doğrudan
# kalkış noktasına nişanlanınca uçak oraya 2.3 m irtifayla geldi ve flare'de
# süzülüp 103 m İLERİDE temas etti (flare, gaz açık ve burun yukarı olduğu için
# alçalmayı neredeyse durduruyor). Gerçek pilotajda da nişan noktası ile temas
# noktası ayrıdır; süzülüş, hedefin bu kadar gerisine nişanlanır.
# ⚠ NİŞAN KAYDIRMASI SIFIR (2026-08-13, ölçüldü). Tarihçe: flare'de sabit
# alçalmaya geçilen sürümde uçak hedefi süzülerek geçiyordu ve kaydırma telafi
# içindi (0→103 m ileri, 100→50, 180→89 — tekdüze bile değildi). Süzülüş hattı
# flare içinde de sürdürülünce uçak NİŞAN NOKTASINA iniyor; kaydırma o anda saf
# sapmaya dönüştü: 40 m kaydırma → temas 40 m ötede, 0 m → temas 1 m ötede.
# ⚠ NİŞAN = HEDEFİN 20 m GERİSİ — YER KAYMASI TELAFİSİ (2026-08-13, ölçüldü).
# Uçak artık uçuş hızında (13.1 m/s) düz temas ediyor (salınımsız iniş bunu
# gerektirdi) ve temastan sonra 5.4 saniyede 20.6 m KAYIYOR. Ölçüm:
#   temas fırlatma noktasına 3.2 m — ama DURUŞ 17.3 m (avcıya 25.5 m).
# Ölçüt "uçağın durduğu yer" olduğu için nişan, kayma kadar geriye alınır.
# (Tarihçe: nişan kaydırması bir dönem 40 m'ydi ve o zaman FLARE SÜZÜLMESİNİ
# telafi ediyordu; süzülüş hattı flare'e taşınınca gereksizleşip 0'a çekilmişti.
# Şimdi bambaşka bir sebeple, YER KAYMASI için geri geliyor.)
_INIS_NISAN_KAC  = float(os.environ.get("AVCI_INIS_NISAN_KAC", 0.0))


def _kelepce(x, alt, ust):
    return max(alt, min(ust, x))


def _yer_hizi():
    return math.hypot(_pos["vx"], _pos["vy"])


def inis(conn, hedef=None, rapor_hedef=None, kayma_m=0.0):
    """Uçağı kademeli yumuşak gövde inişiyle indirir ve disarm eder.

    hedef: (x, y) yerel NED — verilirse uçak oraya YÖNELİR ve alçalma hızı
    kalan mesafeye göre hesaplanır (süzülüş hattı), böylece TEMAS o noktada
    olur. None ise bulunduğu yere iner (eski davranış).

    ⚠ SALINIM DERSİ (2026-08-13, kullanıcı itirazı + iz ölçümü): önceki sürüm
    burnu HIZA göre sürüyordu (yavaşsa aşağı, hızlıysa yukarı) ve aynı anda gaz
    da ALÇALMA HIZINA göre sürülüyordu. İki gecikmeli P-döngüsü birbirini
    kovaladı: burun 1-2 saniyede ±10-20° salındı (ölçüldü: -8.9 / +0.3 / -5.0 /
    -8.8 / +0.3 / -8.3), hız 12-16 m/s arası gitti geldi ve uçak flare'e her
    seferinde farklı enerjiyle girdi.

    YENİ TASARIM — tek serbestlik derecesi:
      • BURUN: kademeye göre SABİT bir hedef açı (hız döngüsü YOK). Salınımın
        kaynağı buydu; sabit açı salınmaz. Değişim hız sınırlıdır (adım başına
        en fazla ±15), böylece kademe geçişlerinde de sarsıntı olmaz.
      • GAZ: alçalma hızını tutar; ölçüm SÜZÜLÜR ve kazanç düşüktür (150 → 80),
        yani yavaş ve sakin tepki.
      • HIZ: artık denetlenen değil, SONUÇ. Daha yavaş inmek için burun hedefi
        yukarı, gaz bandı aşağı alındı — ölçülen yaklaşma ~13-14 m/s.
      • STALL TABANI: hız _INIS_STALL_HIZ altına inerse gaz tabanı yükselir ve
        burun-yukarı komutu kısılır (12 m/s AIRSPEED_MIN'in altına düşülmesin).
    """
    print(f"[SCN] İNİŞ — sabit burun açısı + süzülmüş gaz denetimi; "
          f"{_INIS_SON_ALT:.0f} m son yaklaşma, {_INIS_FLARE_ALT:.0f} m flare")
    set_mode(conn, PLANE_MODE_FBWA, confirm_timeout=0)

    t0 = time.time()
    faz = ""
    temas_t0 = None
    hiz_f = None          # süzülmüş yer hızı (yalnız stall tabanı için)
    vz_f = None           # süzülmüş alçalma hızı (gaz döngüsü)
    burun = 0             # hız sınırlı burun komutu (salınım olmasın)
    _tani = {"t": 0.0}    # flare tanılama basım zamanlayıcısı
    while not _abort and time.time() - t0 < _INIS_ZAMAN_ASIMI:
        _pump(conn)
        irtifa, vz, hiz = -_pos["z"], _pos["vz"], _yer_hizi()
        hiz_f = hiz if hiz_f is None else (0.9 * hiz_f + 0.1 * hiz)
        vz_f = vz if vz_f is None else (0.85 * vz_f + 0.15 * vz)

        # ── HEDEFE YÖNELME + SÜZÜLÜŞ HATTI ──
        # Hedef verildiyse alçalma hızı SABİT değil, kalan mesafeden gelir:
        #   gereken_sink = irtifa / (mesafe / hız)
        # Böylece uçak yere tam hedefte varır. Sabit sink ile 13 m/s'de 30 m'yi
        # 0.8 m/s ile inmek ~490 m yol demek — uçak evi çok geçerdi.
        yatis = 0
        hedefli_sink = None
        if hedef is not None:
            # NİŞAN = hedefin, UÇAĞIN O ANKİ YÖNÜNDE kayma_m kadar GERİSİ.
            # Sabit bir nişan noktası işe yaramadı (2026-08-13 ölçümü): nişanı
            # kare bitişindeki yöne göre geri almıştım, oysa yer kayması TEMAS
            # ANINDAKİ uçuş yönünde oluyor — temas hedefin 20 m kuzeyine düştü,
            # kayma güneydoğuya gitti, uçak 10 m yanda durdu. Yöne bağlı nişan,
            # "kayma bitince hedefin üstündeyim" koşulunu doğrudan kurar.
            if kayma_m > 0.0 and _att["ok"]:
                nis = (hedef[0] - kayma_m * math.cos(_att["yaw"]),
                       hedef[1] - kayma_m * math.sin(_att["yaw"]))
            else:
                nis = hedef
            mesafe, kerteriz = _hedefe(*nis)
            # Yatış: yalnız yeterince yüksekte ve uzaktayken düzeltilir; son
            # metrelerde kanatlar DÜZ kalır (temas yatık kanatla olmasın).
            if mesafe > 40.0 and irtifa > 2.0:
                hata = _angdiff(kerteriz, _att["yaw"])
                yatis = int(500 * _kelepce(hata / math.radians(45.0), -1.0, 1.0))
            # ⚠ SÜZÜLÜŞ HATTI FLARE'DE DE SÜRER (ölçüldü 2026-08-13): flare'de
            # sabit 0.15 m/s'ye geçilince uçak hedefin üstünden süzülerek geçip
            # 50-103 m ileride iniyordu ve "nişanı geriye al" telafisi tekdüze
            # sonuç vermedi (0→103 m, 100→50 m, 180→89 m). Hat flare'de de
            # korunur, yalnız üst sınır yumuşatılır: hedefe yaklaşınca gereken
            # alçalma kendiliğinden artar, uçak orada yere oturur.
            gereken = irtifa * max(hiz_f, 5.0) / max(mesafe, 15.0)
            if irtifa > _INIS_FLARE_ALT:
                # üst sınır seçilen süzülüş eğimine göre (8° ≈ 1.9 m/s @13.4 m/s)
                sink_ust = math.tan(math.radians(_INIS_EGIM_DEG)) * max(hiz_f, 8.0) * 1.15
                hedefli_sink = _kelepce(gereken, 0.3, sink_ust)
            else:
                # ⚠ TAVAN İKİ KADEMELİ (2026-08-13, ölçümle bulundu):
                # Tavanı TÜM flare boyunca 0.35'e indirmek temas dikeyini
                # 0.46 → 0.32 m/s yaptı ama iniş noktasını 1 m'den 28 m'ye
                # kaydırdı — 5 m'lik flare boyunca yayılma yer kaymasına
                # Tavan 0.8 — kullanıcı kararı (2026-08-13): yavaş temaslı
                # iniş bu değerle ölçülmüştü (run_nisan0: temas 2.8 m/s,
                # dikey +0.10, burun +3.1°, hedefe 1 m).
                hedefli_sink = _kelepce(gereken, 0.15, 0.8)

        # ── kademe: (hedef alçalma, burun hedefi, gaz bandı) ──
        if irtifa > _INIS_SON_ALT:
            yeni_faz, hedef_sink, burun_hedef = "YAKLAŞMA", _INIS_SINK, 60
            gaz_alt, gaz_ust = 420, 650
        elif irtifa > _INIS_FLARE_ALT:
            yeni_faz, hedef_sink, burun_hedef = "SON YAKLAŞMA", _INIS_SON_SINK, 100
            gaz_alt, gaz_ust = 400, 620
        else:
            yeni_faz, hedef_sink = "FLARE", _INIS_FLARE_SINK
            oran = _kelepce(irtifa / _INIS_FLARE_ALT, 0.0, 1.0)      # 1 → 0
            burun_hedef = int(_INIS_FLARE_PITCH_BAS
                              + (_INIS_FLARE_PITCH - _INIS_FLARE_PITCH_BAS)
                              * (1.0 - oran))
            # Gaz flare boyunca AÇIK kalır (model motorsuz süzülemiyor);
            # kesme yalnız tekerlek yüksekliğinde.
            # GAZ 2.0 m'den 0.3 m'ye KADEMELİ kesilir → temas uçuş hızında
            # değil, yavaşlamış hâlde olur (kullanıcı isteği 2026-08-13:
            # motor açık temas 13.3 m/s ile oluyor ve 21 m kayma yapıyordu).
            # ⚠ Kesme sırasında BURUN KOMUTU SABİT tutulur (aşağıda): önceki
            # denemede komut 244→296 tırmanırken gaz düşüyordu ve uçak
            # -15.1°/+18.7° savruluyordu. Sabit komut + azalan gaz = düzgün
            # yavaşlama.
            # GAZ 1 m ALTINDA KESİLİR — kullanıcı kararı (2026-08-13).
            # Yavaş temas bunu gerektiriyor: motor açık kalınca uçak 13 m/s ile
            # değip 21 m kayıyordu. BEDELİ ÖLÇÜLDÜ ve kabul edildi: gaz
            # kesilince burun düşüyor, gövde çarpıyor ve kısa bir sıçrama
            # olabiliyor (25 Hz kayıt: +17.9°, 0.5 m zıplama).
            # Tavan 550: alçalma denetleyicisi düz hat için 505-528 istiyor,
            # 500'de tavana takılıp hattı tutamıyor ve 2.2 m/s'ye dalıyordu
            # (ölçüldü 2026-08-13). Kesme yine 1 m altında.
            gaz_alt = 0 if irtifa < 1.0 else 350
            gaz_ust = 0 if irtifa < 1.0 else 550
        if hedefli_sink is not None:
            hedef_sink = hedefli_sink
        if yeni_faz != faz:
            faz = yeni_faz
            ek = ""
            if hedef is not None:
                ek = f", eve {_hedefe(*hedef)[0]:.0f} m"
            print(f"[SCN]   {faz} — irtifa {irtifa:.1f} m, alçalma {vz_f:+.1f} m/s, "
                  f"hız {hiz_f:.1f} m/s{ek}")

        # ── STALL TABANI — YALNIZ YAKLAŞMADA, FLARE'DE DEĞİL ──
        # Yaklaşmada hız düşerse gaz eklenir ve burun-yukarı komutu kısılır.
        # FLARE'DE KISILMAZ (2026-08-13 ölçümü): flare'in tanımı hızı irtifaya
        # çevirmektir, hız zaten düşer. Koruma orada da çalışınca burun hedefi
        # 216'dan 69'a düştü ve uçak son metrede -18.1° burun aşağı geldi.
        if hiz_f < _INIS_STALL_HIZ and gaz_ust > 0 and yeni_faz != "FLARE":
            gaz_alt = min(gaz_ust, gaz_alt + 120)
            burun_hedef = int(burun_hedef * _kelepce(
                (hiz_f - (_INIS_STALL_HIZ - 1.5)) / 1.5, 0.0, 1.0))

        # ── GAZ: süzülmüş alçalma hızı denetimi, düşük kazanç ──
        gaz = int(_kelepce(_INIS_GAZ_TABAN + 80 * (vz_f - hedef_sink),
                           gaz_alt, gaz_ust))
        # ── BURUN: hedefe HIZ SINIRLI yaklaş (adım başına ±15) ──
        burun += int(_kelepce(burun_hedef - burun, -15, 15))
        _rc(conn, roll=yatis, pitch=int(_kelepce(burun, -60, 800)), throttle=gaz)
        # FLARE TANILAMA (2 Hz): komut ile GERÇEKLEŞEN açıyı yan yana bas.
        # Burun komutu büyükken uçak düz kalıyorsa sınır otopilottadır (FBWA
        # düşük hızda burun-yukarı talebini kısar), kodda değil.
        if faz == "FLARE" and time.time() - _tani["t"] > 0.5:
            _tani["t"] = time.time()
            print(f"[FLARE] irtifa {irtifa:4.1f} m | burun komut {burun:4d} "
                  f"(hedef {burun_hedef:4d}) → gerçek "
                  f"{math.degrees(_att['pitch']):+5.1f}° | gaz {gaz:3d} | "
                  f"hız {hiz_f:4.1f} | alçalma {vz_f:+4.2f}")

        # ── temas: yere oturdu ve dikey hız söndü ──
        if irtifa < 0.5 and abs(vz) < 0.8:
            temas_t0 = temas_t0 or time.time()
            if time.time() - temas_t0 > 1.0:
                break
        else:
            temas_t0 = None
        time.sleep(CONTROL_RATE)

    if _abort:
        return
    _rh = rapor_hedef or hedef
    _sapma = f", kalkış noktasına {_hedefe(*_rh)[0]:.0f} m" if _rh else ""
    print(f"[SCN] TEMAS — irtifa {-_pos['z']:.2f} m, dikey hız {_pos['vz']:+.2f} m/s, "
          f"yer hızı {_yer_hizi():.1f} m/s, "
          f"burun {math.degrees(_att['pitch']):+.1f}°{_sapma}")
    t1 = time.time()
    while not _abort and time.time() - t1 < 8.0:
        _pump(conn)
        _rc(conn, roll=0, pitch=_INIS_FLARE_PITCH, throttle=0)
        if _yer_hizi() < 1.0:
            break
        time.sleep(CONTROL_RATE)

    _rc(conn, roll=0, pitch=0, throttle=0)
    time.sleep(0.5)
    try:
        disarm(conn, force=True)
    except Exception as e:
        print(f"[SCN] disarm hatası: {e}")
    t2 = time.time()
    while time.time() - t2 < 3.0:
        _pump(conn)
        if not _hb["armed"]:
            break
        time.sleep(0.2)

    # RAPOR DÜRÜSTLÜĞÜ: iniş ancak araç GERÇEKTEN durduysa başarıdır.
    irtifa, vz = -_pos["z"], _pos["vz"]
    durdu = irtifa < 2.0 and abs(vz) < 1.0 and not _hb["armed"]
    if durdu:
        print(f"[SCN] ✓ İNİŞ TAMAM — irtifa {irtifa:.2f} m, "
              f"dikey hız {vz:+.1f} m/s, armed={_hb['armed']} "
              f"({time.time() - t0:.0f} s)")
    else:
        print(f"[SCN] ⚠ İNİŞ DOĞRULANAMADI — irtifa {irtifa:.2f} m, "
              f"dikey hız {vz:+.1f} m/s, armed={_hb['armed']} "
              f"({time.time() - t0:.0f} s)")
        if irtifa < -5.0:
            print("[SCN]   irtifa zeminin ALTINDA: Gazebo modeli araziden "
                  "geçmiş olabilir (sim artefaktı) — simi yeniden kur.")


# ---------------------------------------------------------------------------
# Senaryolar — hepsi süresiz döner, GCS süreci öldürünce biter
# (istisna: kare_gorev — kendi biter)
# ---------------------------------------------------------------------------

def scenario_duz(conn):
    """Kalkış + BEKLEME DAİRESİ + süresiz düz uçuş.

    NEDEN VAR (2026-08-08): "düz uçuş" ölçümü manuel modla yapılınca elevator
    nötrde uçak yavaşça alçalıp güdümün 8 m irtifa tabanına dayandı ve koşu
    geçersiz oldu. Desen makinesi (FBWA + RC override) irtifayı kabaca
    koruyor; düz referans ölçümü artık buradan alınır.

    ⚠ BAŞLANGIÇ DAİRESİ (2026-08-09, kullanıcı isteği): kalkıştan hemen sonra
    düze geçince hedef, avcı drone daha kalkarken ufka doğru uzaklaşıyor ve
    her testin ilk ~60 saniyesi sırf mesafe kapatmakla geçiyordu. Uçak artık
    BEKLEME_TUR_S boyunca daire çizerek bölgede kalır; drone yaklaşınca düze
    geçer. Ölçüm daha kısa sürer ve karşılaşma geometrisi de tekrarlanabilir
    olur (her koşuda benzer menzilden başlanır).
    Kapatmak için: AVCI_DUZ_BEKLEME=0
    """
    # 45 → 15 s (2026-08-09, kullanıcı kararı): bölgede kalmak için 15 s yeter,
    # fazlası testi uzatıyordu.
    bekleme = float(os.environ.get("AVCI_DUZ_BEKLEME", 15.0))
    if bekleme > 0 and not _abort:
        print(f"[SCN] DÜZ — önce {bekleme:.0f} s bekleme dairesi "
              f"(drone yaklaşsın), sonra düz uçuş")
        _daire_sureli(conn, DAIRE_CAPLARI["circle"][0], bekleme)
    print("[SCN] DÜZ — süresiz düz uçuş (gaz: GCS slider)")
    while not _abort:
        hold(conn, 0.5)


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


def scenario_kare_gorev(conn):
    """TEK TUŞLUK GÖSTERİ: (kalkış main'de) → N tam kare → otonom iniş.

    `square`den farkı SONLU olması: kare sayısı dolunca uçak eve inip süreç
    biter. Arayüzdeki "Kalkış → Kare → İniş" butonu bunu çağırır; başka bir
    şey (durdurma, manuel devralma) gerekmez.

    KENAR MESAFEDİR, SÜRE DEĞİL (2026-08-13): "kenar N metre düz git, sonra
    sağa 90°" — kare tanımı budur. Eski sürüm kenarı 5 saniye uçuyordu, yer
    hızına göre uzunluk kayıyordu. `scenario_square` (süresiz "Kare Çiz"
    butonu) eski süre tabanlı hâlinde BIRAKILDI: kampanya senaryosu odur,
    ölçüm geçmişiyle kıyaslanabilir kalsın.

    KENAR 30 m (2026-08-13, kullanıcı isteği). ⚠ ŞEKİL UYARISI: bu uçağın 90°
    dönüş yarıçapı 36° yatışta ~32-36 m (R = v²/(g·tanθ), v≈15-16 m/s). Kenar
    yarıçapın altına indiği için desen KARE ÇIKMAZ, yuvarlak bir halka olur —
    ölçüldü: 40 m'de ~90 m'lik halka, 60 m'de yamuk, 100 m'de net kare.
    Daha küçük yarıçap daha düşük hız ister (10 m yarıçap ≈ 9.9 m/s) ve o hız
    AIRSPEED_MIN=12'nin altındadır, yani fiziken mümkün değil.
    İNİŞ bundan ETKİLENMEZ: dönüş bacağı ve süzülüş hattı desenden bağımsız
    olarak fırlatma noktasını hedefler (ölçüm aşağıdaki koşuda).

    Ayarlar: AVCI_KARE_TUR (tur sayısı), AVCI_KARE_KENAR_M (kenar, metre),
             AVCI_KARE_BANK (dönüş yatış komutu, 0-1000).
    """
    tur = int(os.environ.get("AVCI_KARE_TUR", "1"))
    kenar = float(os.environ.get("AVCI_KARE_KENAR_M", "30"))
    bank = int(os.environ.get("AVCI_KARE_BANK", "800"))
    # SON KENARI ATLA (kullanıcı önerisi 2026-08-13): kareyi kapatan 4. kenar
    # uçağı desenin başına geri getiriyor, ardından iniş için yeniden dışarı
    # çıkılıyor. 4. kenar çizilmezse desen açık kalır ("U") ama görev kısalır.
    # AVCI_KARE_SON_KENAR_ATLA=1 ile açılır.
    atla = os.environ.get("AVCI_KARE_SON_KENAR_ATLA", "0") == "1"
    print(f"[SCN] KARE GÖREVİ — {tur} tur × 4 kenar ({kenar:.0f} m), "
          f"sağa 90° dönüşler (yatış {bank}), sonra iniş")
    for t in range(tur):
        son_tur = (t == tur - 1)
        kenar_sayisi = 3 if (son_tur and atla) else 4
        if son_tur and atla:
            print("[SCN] Son tur: 4. kenar ATLANIYOR, 3. kenardan sonra inişe geçilecek")
        for i in range(kenar_sayisi):
            if _abort:
                return
            kat = duz_git(conn, kenar)
            print(f"[SCN] Tur {t + 1}/{tur} · kenar {i + 1}/{kenar_sayisi} bitti "
                  f"({kat:.0f} m düz)")
            if _abort:
                return
            if i == kenar_sayisi - 1 and son_tur and atla:
                break                      # son kenardan sonra dönüş yok
            donuldu = donus_yap(conn, 90, bank=bank)
            print(f"[SCN] Tur {t + 1}/{tur} · dönüş {i + 1} bitti "
                  f"(sağa {donuldu:.0f}°)")
    if _abort:
        return

    # ── KALKIŞ NOKTASINA DÖNÜŞ ──
    # Kullanıcı isteği (2026-08-13): uçak KALKIŞA BAŞLADIĞI yere dönmeli —
    # kare deseninin başladığı yere değil (desen, tırmanış bittiğinde evden
    # birkaç yüz metre uzakta başlıyor).
    # Önce evin YAKLASMA_M kadar dışında bir giriş noktasına gidilir; son
    # yaklaşma oradan eve DÜZ yapılır ve alçalma hattı mesafeye göre sürülür.
    if _kalkis_xy is None:
        print("[SCN] ⚠ Kalkış noktası bilinmiyor — bulunduğu yere inilecek")
        inis(conn)
        return
    ev_x, ev_y = _kalkis_xy
    _pump(conn)
    mes, _k = _hedefe(ev_x, ev_y)
    irtifa_su_an = -_pos["z"]
    # Dönüş irtifası: kare bittikten sonra bu irtifaya tırmanılır. Giriş
    # mesafesi de BU irtifadan hesaplanır, yoksa uçak yükseldikçe hat dikleşir.
    hedef_irtifa = _DONUS_IRTIFA if _DONUS_IRTIFA > 0 else irtifa_su_an
    gerekli = hedef_irtifa / math.tan(math.radians(_INIS_EGIM_DEG))
    giris_m = _kelepce(gerekli + 40.0, _INIS_GIRIS_MIN, _INIS_GIRIS_MAX)
    print(f"[SCN] EVE DÖNÜŞ — fırlatma noktasına {mes:.0f} m, irtifa "
          f"{irtifa_su_an:.0f} → {hedef_irtifa:.0f} m, {_INIS_EGIM_DEG:.0f}° eğim "
          f"için {giris_m:.0f} m gerek")
    dx, dy = _pos["x"] - ev_x, _pos["y"] - ev_y
    d = math.hypot(dx, dy)
    if d < 1.0:
        dx, dy, d = 1.0, 0.0, 1.0
    g_x = ev_x + dx / d * giris_m
    g_y = ev_y + dy / d * giris_m
    if mes >= giris_m and irtifa_su_an >= hedef_irtifa - 2.0:
        print(f"[SCN] Yaklaşma girişine ÇIKMAYA GEREK YOK ({mes:.0f} ≥ "
              f"{giris_m:.0f} m, irtifa yeterli) — süzülüş buradan başlıyor")
    else:
        git_noktaya(conn, g_x, g_y, varis_m=60.0, hedef_irtifa=hedef_irtifa)
    if _abort:
        return
    print(f"[SCN] Yaklaşma girişindeyiz (eve {_hedefe(ev_x, ev_y)[0]:.0f} m) — "
          f"süzülüş başlıyor; nişan, uçuş yönünde {_INIS_NISAN_KAC:.0f} m geride "
          f"(yer kayması telafisi)")
    inis(conn, hedef=(ev_x, ev_y), kayma_m=_INIS_NISAN_KAC)


# ── DAİRE ÇAPLARI (2026-08-05) ──
# Yarıçap yatış açısıyla belirlenir: R = v²/(g·tanθ). Roll komutu FBWA'da
# yatış hedefine ölçeklenir (roll=1000 ≈ 45°). v≈15 m/s için:
#     roll   yatış   yarıçap   yük faktörü   stall hızı×
#      300     14°      96 m       1.03         1.01
#      400     18°      71 m       1.05         1.03
#      500     22°      55 m       1.08         1.04   ← eski tek daire
#      650     29°      41 m       1.15         1.07
#      800     36°      32 m       1.24         1.11
# 40°+ eklenmedi: AIRSPEED_MIN=12 / CRUISE=15 ile stall payı daralıyor.
#
# NEDEN VAR: iç daire nişanının yarıçap-oranlı sürümünü sınamak için hedefin
# FARKLI yarıçaplarda dönmesi gerekiyor. Sabit-metre sürüm (14 m) yalnız
# ~52 m yarıçapta ölçüldü; oranlı sürümün asıl kazancı dar ve geniş dairede
# ortaya çıkar (24 m'de 6.5 m, 80 m'de 21.6 m kayma üretir).
#
# Pitch, yatışla birlikte artar: yatışta düşey kaldırma bileşeni azalır,
# irtifayı korumak için burun biraz yukarı gerekir (kabaca 1/cosθ ile).
DAIRE_CAPLARI = {
    "circle_xl": (300, "çok geniş (~96 m)"),
    "circle_l":  (400, "geniş (~71 m)"),
    "circle":    (500, "orta (~55 m) — referans"),
    "circle_s":  (650, "dar (~41 m)"),
    # ⌀32 (roll 800) KALDIRILDI (2026-08-06, kullanıcı kararı): o kadar sert ve
    # SÜREKLİ bir manevra gerçekçi bir hedef davranışı değil; ayrıca avcı drone
    # orada ivme tavanına dayanıp kontrolü kaybediyordu (v_sürdürülebilir =
    # a_max/ω = 8/0.564 = 14.2 m/s, hedefin hızına eşit → sıfır pay).
    # Geri eklemek gerekirse: "circle_xs": (800, "çok dar (~32 m)")
}


def _daire_sureli(conn, roll_cmd, sure_s):
    """Belirli süre daire çiz (bekleme turu). _daire ile aynı trim mantığı."""
    import math as _m
    import time as _t
    yatis_deg = roll_cmd / 1000.0 * 45.0
    pitch_cmd = int(150 * (1.0 / _m.cos(_m.radians(yatis_deg))))
    t0 = _t.time()
    while not _abort and (_t.time() - t0) < sure_s:
        hold(conn, 0.5, roll=roll_cmd, pitch=pitch_cmd)


def _daire(conn, roll_cmd, etiket):
    """Sabit yatışla süresiz tur. Pitch yatışa göre ölçeklenir (irtifa korunsun)."""
    import math as _m
    yatis_deg = roll_cmd / 1000.0 * 45.0
    pitch_cmd = int(150 * (1.0 / _m.cos(_m.radians(yatis_deg))))
    print(f"[SCN] DAİRE {etiket} — roll={roll_cmd} (~{yatis_deg:.0f}° yatış), "
          f"pitch={pitch_cmd}")
    while not _abort:
        hold(conn, 0.5, roll=roll_cmd, pitch=pitch_cmd)


def scenario_circle(conn):
    _daire(conn, *DAIRE_CAPLARI["circle"])


TIRMANIS_MIN_THR = 600      # tırmanış/spiral için taban gaz (= THROTTLE_CRUISE)


def tirmanis_throttle():
    """Tırmanış manevralarında gaz: GCS slider'ını dinler ama TABAN uygular.

    Neden taban var: burun yukarıdayken gaz düşük olursa uçak hız kaybedip
    stall eder ve düşer — senaryo biter. Slider daha yükseği isterse ona uyar,
    daha düşüğü isterse tırmanış boyunca tabanda kalır (düz/dönüş kısımlarında
    slider aynen geçerli). Yani "hedefi yavaşlat" isteği çalışır, uçak düşmez.
    """
    return max(gcs_throttle(), TIRMANIS_MIN_THR)


def scenario_aggressive(conn):
    print("[SCN] AGRESİF — rastgele manevralar (gaz: GCS slider'ı)")
    maneuvers = ["climb", "dive", "bank_l", "bank_r", "s_turn", "spiral"]
    while not _abort:
        m = random.choice(maneuvers)
        if m == "climb":
            print("[SCN] Sert tırmanış")
            hold(conn, random.uniform(1.5, 3.0),
                 pitch=random.randint(500, 800), throttle=tirmanis_throttle())
        elif m == "dive":
            # irtifa emniyeti: 40m altındaysa dalma, yerine tırman
            if -_pos["z"] > 40.0:
                print("[SCN] Dalış")
                # dalışta gaz KESİLİR (slider'dan bağımsız) — burun aşağıyken
                # gaz vermek uçağı hedefin yakalanamayacağı hıza fırlatır
                hold(conn, random.uniform(1.0, 2.0),
                     pitch=-random.randint(350, 600), throttle=200)
            else:
                print("[SCN] İrtifa düşük — dalış yerine tırmanış")
                hold(conn, 2.0, pitch=500, throttle=tirmanis_throttle())
        elif m in ("bank_l", "bank_r"):
            s = -1 if m == "bank_l" else 1
            print("[SCN] Sert yatışlı dönüş" + (" (sol)" if s < 0 else " (sağ)"))
            hold(conn, random.uniform(1.5, 3.0),
                 roll=s * random.randint(600, 900), pitch=200)   # throttle=None → slider
        elif m == "s_turn":
            print("[SCN] Keskin S-dönüşü")
            hold(conn, 1.5, roll=-750, pitch=200)                # throttle=None → slider
            hold(conn, 1.5, roll=750, pitch=200)
        elif m == "spiral":
            print("[SCN] Spiral tırmanış")
            hold(conn, random.uniform(3.0, 5.0),
                 roll=450, pitch=450, throttle=tirmanis_throttle())
        # toparlanma: kısa düz uçuş (gaz: slider)
        hold(conn, random.uniform(1.0, 2.0))


# Dönüş/iniş bacağı olan görevler — fırlatma noktası bunlar için ZORUNLUDUR.
_DONUS_GEREKTIREN = ("kare_gorev",)

SCENARIOS = {
    "duz": scenario_duz,
    "square": scenario_square,
    "kare_gorev": scenario_kare_gorev,
    "circle": scenario_circle,
    "aggressive": scenario_aggressive,
}

# Beş daire çapı tek tek kaydedilir (functools.partial yerine varsayılan
# argümanlı lambda: döngü değişkeni geç-bağlanma tuzağına düşmesin).
for _ad, (_roll, _etiket) in DAIRE_CAPLARI.items():
    if _ad != "circle":
        SCENARIOS[_ad] = (lambda c, r=_roll, e=_etiket: _daire(c, r, e))


def _kalkis_hedefi(name):
    """kare_gorev alçak desende uçar; diğer senaryolar AVCI_TKOFF_HEDEF'te kalır."""
    return _KARE_IRTIFA if name == "kare_gorev" else None


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

    # FIRLATMA NOKTASI — kaynağı _ev_noktasi() (varsayılan: otopilotun HOME'u).
    # C KAPISI: belirlenemezse dönüş bacağı olan görev BAŞLATILMAZ. Eski sürüm
    # burada sessizce (0,0)'a düşüyordu ve uçak boot noktasına dönüyordu.
    global _kalkis_xy
    _kalkis_xy = _ev_noktasi(conn)
    if _kalkis_xy is None and name in _DONUS_GEREKTIREN:
        print(f"[SCN] ⛔ '{name}' görevi fırlatma noktası olmadan BAŞLATILMAZ "
              f"(dönüş bacağı nereye gideceğini bilemez). "
              f"AVCI_EV_KAYNAK=yerel ile eski davranışa dönülebilir.")
        stop_gcs_keepalive()
        return

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
        takeoff(conn, hedef_alt=_kalkis_hedefi(name))
    else:
        result = arm_plane(warmup_duration=3.0)
        if result is None or result[1] != 0:
            print("[SCN] ARM başarısız!")
            return
        takeoff(conn, hedef_alt=_kalkis_hedefi(name))

    SCENARIOS[name](conn)

    # Durduruldu → nötr yüzey + cruise gazla bırak (manuel mod hemen devralır).
    # İniş sonrası (araç yerde/disarm) cruise gaz VERİLMEZ: yeniden armlanırsa
    # uçak kendi kendine yerde koşmaya başlar.
    _pump(conn)
    _rc(conn, throttle=0 if (_hb["ok"] and not _hb["armed"]) else THROTTLE_CRUISE)
    stop_gcs_keepalive()
    print("[SCN] Senaryo sonlandı.")


if __name__ == "__main__":
    main()
