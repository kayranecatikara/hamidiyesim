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
    PLANE_MODE_TAKEOFF,
    PLANE_MODE_FBWA,
    PLANE_MODE_FBWB,
)

# ══════════════════════════════════════════════════════════════════════
# SEYİR MODU — 2026-08-23, kullanıcı kararı: FBWA → FBWB
# ══════════════════════════════════════════════════════════════════════
# SORUN: FBWA'da nötr elevator "YUNUSLAMA 0° tut" demektir, "irtifa tut"
# değil. Kanadın sıfır-kaldırma açısı a0 = 7.45° olduğu için yunuslama
# 0 iken uçak 7.45° hücum açısıyla uçuyor ve ağırlığından %60 fazla
# kaldırma üretiyordu → SÜREKLİ TIRMANIYORDU.
# ÖLÇÜLDÜ (00000308.BIN): 249 s'de +307 m, tam gazda +2.98 m/s.
#
# NEDEN FBWB "UYDURMA" DEĞİL: gerçek Talon'u kim uçurursa uçursun —
# pilot çubukla ya da otopilot AUTO/CRUISE/FBWB ile — irtifayı TUTAR.
# Hiçbir gerçek uçuşta uçak 4 dakika kontrolsüz tırmanmaz. Yani
# gerçekçi OLMAYAN şey tırmanmaydı. FBWB, ArduPlane'in KENDİ modu;
# dışarıdan hiçbir kontrolcü eklemiyoruz.
#
# FBWB'DE KUMANDA ANLAMLARI DEĞİŞİR:
#   CH2 (elevator) nötr → İRTİFAYI TUTAR. Tam çubuk = ±FBWB_CLIMB_RATE
#                         (uçuştan okundu: 2.0 m/s)
#   CH3 (gaz)           → HAM GAZ DEĞİL, HEDEF HAVA HIZI.
#                         V = AIRSPEED_MIN + (gaz/1000)·(MAX − MIN)
#                         Uçuştan okundu: MIN 12, MAX 22 m/s, ARSPD_USE=1
#                         → gaz 300 = 15 m/s · gaz 500 = 17 m/s
#   CH1 (aileron)       → FBWA ile AYNI (ROLL_LIMIT_DEG 65°)
#                         ⇒ dönüş yarıçapları DEĞİŞMEZ
#
# GERİ DÖNÜŞ: AVCI_PLANE_MODE=FBWA  → eski davranış birebir.
_MOD_ADI = os.environ.get("AVCI_PLANE_MODE", "FBWB").strip().upper()
SEYIR_MODU = PLANE_MODE_FBWB if _MOD_ADI == "FBWB" else PLANE_MODE_FBWA
FBWB_AKTIF = (SEYIR_MODU == PLANE_MODE_FBWB)


def _irtifa_pitch(fbwa_degeri):
    """FBWB irtifayı KENDİ tutar → elevator NÖTR olmalı. FBWA'da ise
    eski 'irtifa kaybını azaltan hafif up-elevator' telafisi geçerli.
    FBWB'de bu telafi bırakılırsa tırmanma KOMUTU olur."""
    return 0 if FBWB_AKTIF else fbwa_degeri

# Havada devralma eşiği: bu irtifanın üstünde armlıysak kalkış ATLANIR.
AIRBORNE_ALT_M = 15.0

CONTROL_RATE = 0.05   # 20 Hz komut döngüsü

_abort = False

# _pump ile güncellenen son telemetri
_att = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "ok": False}
# ⭐ x/y/vx/vy/vz ELİPS senaryosu için eklendi (2026-08-17, kalkis_kare_inis
# dalından taşındı). Mevcut senaryoların hiçbiri bunları okumuyor — ekleme
# tamamen katmerli, eski davranış birebir aynı.
_pos = {"x": 0.0, "y": 0.0, "z": 0.0,      # x=kuzey, y=doğu (NED, m)
        "vx": 0.0, "vy": 0.0, "vz": 0.0}   # vz: pozitif = AŞAĞI


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
            _pos["x"] = msg.x            # kuzey (m) — elips izdüşümü buradan
            _pos["y"] = msg.y            # doğu  (m)
            _pos["z"] = msg.z
            _pos["vx"] = msg.vx          # yer hızı → elips gaz/yatış beslemesi
            _pos["vy"] = msg.vy
            _pos["vz"] = msg.vz          # NED: pozitif = AŞAĞI


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


_hiz_cache = {"val": 15.0, "t": 0.0}


def gcs_hiz():
    """GCS'ten HEDEF HAVA HIZI oku (m/s). Gaz kaydırıcısından AYRI kumanda."""
    now = time.time()
    if now - _hiz_cache["t"] > 0.5:
        _hiz_cache["t"] = now
        try:
            req = urllib.request.urlopen(
                "http://127.0.0.1:8000/api/plane_airspeed", timeout=0.2)
            _hiz_cache["val"] = json.loads(req.read().decode()).get("airspeed", 15.0)
        except Exception:
            pass
    return _hiz_cache["val"]


# ArduPlane FBWB: gaz çubuğu hedef hava hızını verir (AIRSPEED_MIN..MAX arası).
AS_MIN, AS_MAX = 12.0, 22.0


def ch3():
    """CH3'e YAZILACAK değer — moda göre anlamı değişir.

    FBWA / manuel : HAM GAZ           → gaz kaydırıcısı (/api/plane_throttle)
    FBWB          : HEDEF HAVA HIZI   → hız kaydırıcısı (/api/plane_airspeed)

    İki kaydırıcı AYRI durur; hangisinin geçerli olduğunu MOD belirler.
    Böylece mod değişince kullanıcının gaz ayarı bozulmaz.
    """
    if not FBWB_AKTIF:
        return gcs_throttle()
    V = max(AS_MIN, min(AS_MAX, gcs_hiz()))
    return int(round((V - AS_MIN) / (AS_MAX - AS_MIN) * 1000))


def hold(conn, duration, roll=0, pitch=0, throttle=None, yaw=0):
    """duration boyunca sabit komut uygula (throttle=None → GCS slider)."""
    t0 = time.time()
    while not _abort and time.time() - t0 < duration:
        _pump(conn)
        thr = ch3() if throttle is None else throttle
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


def _kelepce(x, alt, ust):
    return max(alt, min(ust, x))


def _yer_hizi():
    return math.hypot(_pos["vx"], _pos["vy"])


def _hedefe(hx, hy):
    """(mesafe_m, kerteriz_rad) — NED: x kuzey, y doğu; kerteriz atan2(doğu, kuzey)."""
    dx, dy = hx - _pos["x"], hy - _pos["y"]
    return math.hypot(dx, dy), math.atan2(dy, dx)


def _param_yaz(conn, ad, deger, timeout=3.0):
    """Canlı PARAM_SET + teyit. Dönüş: ESKİ değer (geri almak için)."""
    conn.mav.param_request_read_send(conn.target_system, conn.target_component,
                                     ad.encode(), -1)
    eski, t0 = None, time.time()
    while time.time() - t0 < timeout:
        m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
        if m and m.param_id.strip("\x00") == ad:
            eski = m.param_value
            break
    conn.mav.param_set_send(conn.target_system, conn.target_component,
                            ad.encode(), float(deger),
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
        if m and m.param_id.strip("\x00") == ad:
            print(f"[ELIPS] {ad}: {eski} -> {m.param_value}")
            return eski
    print(f"[ELIPS] UYARI {ad} teyit gelmedi")
    return eski


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
        _rc(conn, roll=roll_cmd, pitch=_irtifa_pitch(180), throttle=ch3())
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
            _pos["x"] = msg.x            # kuzey (m) — elips izdüşümü buradan
            _pos["y"] = msg.y            # doğu  (m)
            _pos["z"] = msg.z
            _pos["vx"] = msg.vx          # yer hızı → elips gaz/yatış beslemesi
            _pos["vy"] = msg.vy
            _pos["vz"] = msg.vz          # NED: pozitif = AŞAĞI
        elif t == "ATTITUDE":
            _att.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, ok=True)
    return armed, -_pos["z"]


# Seyre geçmeden önce ulaşılması gereken irtifa.
# ⛔ NEDEN VAR (2026-08-23, ÖLÇÜLDÜ 00000319.BIN): eskiden kalkış SABİT 8 s
# sürüyordu ve o an hangi irtifadaysa oradan seyre geçiliyordu. FBWA'da
# önemsizdi (uçak zaten tırmanmaya devam ediyordu), ama FBWB O İRTİFAYI
# KİLİTLER. Sonuç: hedef 400 saniye boyunca 14.6 m'de uçtu — bekçinin
# 20 m alt sınırının bile ALTINDA. Artık süre değil İRTİFA beklenir.
SEYIR_IRTIFA_M = float(os.environ.get("AVCI_SEYIR_IRTIFA", 45.0))


def takeoff(conn, climb_time=40.0):
    """Otonom kalkış: TAKEOFF modu ile SEYIR_IRTIFA_M'ye tırman, sonra seyre geç.

    climb_time artık ZAMAN AŞIMI (emniyet), hedef değil — irtifaya ulaşınca
    beklemeden çıkar."""
    print(f"[SCN] Otonom kalkış (TAKEOFF modu) → hedef {SEYIR_IRTIFA_M:.0f} m")
    set_mode(conn, PLANE_MODE_TAKEOFF)
    t0 = time.time()
    while not _abort and time.time() - t0 < climb_time:
        _pump(conn)
        if -_pos["z"] >= SEYIR_IRTIFA_M:
            print(f"[SCN] {SEYIR_IRTIFA_M:.0f} m'ye ulaşıldı "
                  f"({time.time()-t0:.0f} s)")
            break
        time.sleep(0.2)
    else:
        print(f"[SCN] ⚠ zaman aşımı: {climb_time:.0f} s'de yalnız "
              f"{-_pos['z']:.0f} m — yine de seyre geçiliyor")
    print(f"[SCN] Kalkış bitti (irtifa ~{-_pos['z']:.0f}m) → {_MOD_ADI}")
    set_mode(conn, SEYIR_MODU)
    hold(conn, 2.0)


# ---------------------------------------------------------------------------
# Senaryolar — hepsi süresiz döner, GCS süreci öldürünce biter
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


# ── DAİRE YARIÇAPLARI ──
# ⛔⛔ 2026-08-21 · BU TABLO 2026-08-05'TEN BERİ YANLIŞTI — DÜZELTİLDİ.
#
# Eski tablo "roll=1000 ≈ 45°" varsayıyordu. SITL param dökümü (mav_2_1.parm,
# ArduPlane) bunu ÇÜRÜTÜYOR:
#     ROLL_LIMIT_DEG   65.000000
# Yani roll komutu 65°'ye ölçekleniyor, 45°'ye değil — 1.44 kat fark. Yarıçap
# tanθ ile ters orantılı olduğu için hata yarıçaplarda ~2 KAT büyüyordu.
#
# ÖLÇÜLDÜ (T kampanyası, 2026-08-21, 8 uçuş): iki aracın GPS konumundan hedefin
# yörüngesine en küçük kareler çemberi uyduruldu (koşunun ikinci yarısı):
#     circle_xl → 66.8 / 65.0 / 64.2 m   (tablo "96 m" diyordu)
#     circle    → 35.4 m                  (tablo "55 m" diyordu)
#     circle_s  → 24.9 / 24.9 / 24.9 m    (tablo "41 m" diyordu)
#   circle_s saçılması p10-p90 = 24.7-25.0 m — uyum kusursuz.
#
# DOĞRU TABLO — yatış = roll/1000 × ROLL_LIMIT_DEG(65°), v ≈ 14.9 m/s:
#     roll   yatış    R = v²/(g·tanθ)    ÖLÇÜLEN     senaryo
#      300   19.5°        64.1 m         64-67 m     circle_xl
#      400   26.0°        46.5 m           —         circle_l
#      500   32.5°        35.6 m         35.4 m      circle
#      650   42.3°        25.0 m         24.9 m      circle_s
#      800   52.0°        17.7 m           —         circle_xs
#
# ⚠ ADLAR YARIÇAPTIR, ÇAP DEĞİL. "circle_xl" = yarıçap ~64 m, çap ~128 m.
#
# ⚠ NEYİ BOZDU: Ö5/Ö11/Ö-B/Ö-J kampanyalarının hepsi bu tabloya bakarak
# "hedef 41 m yarıçapta dönüyor" sanıyordu; gerçek 25 m'ydi. Yani avcının
# zorlandığı geometri, sanılandan %60 daha sert. Bu tablo yanlış olduğu için
# o kampanyaların "neden yakınsayamıyoruz" teşhisleri de yanlış tabana oturdu.
# Yalnız YORUM düzeltildi; roll komutları ve davranış DEĞİŞMEDİ.
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
    # ⌀32 (roll 800) — 2026-08-06'da KALDIRILMIŞTI, 2026-08-21'de GERİ AÇILDI.
    #
    # KALDIRMA GEREKÇESİ (2026-08-06): "avcı drone orada ivme tavanına dayanıp
    # kontrolü kaybediyordu (v_sürdürülebilir = a_max/ω = 8/0.564 = 14.2 m/s,
    # hedefin hızına eşit → sıfır pay)."
    #
    # NEDEN GERİ AÇILDI: o hesap ESKİ zarfa aitti (a_max = 8 m/s², WPNAV_ACCEL
    # 800). Bugün araç tarafında a_max = 26-27 m/s² ve güdüm tarafında
    # MAX_ACCEL = 12 m/s². Yeni sayı: v_sürdürülebilir = 12/0.469 = 25.6 m/s —
    # hedefin 15 m/s'sine göre %70 pay var. Yani "sıfır pay" savı artık geçerli
    # değil.
    #
    # AMA HÂLÂ ZARFIN SINIRI: yayı takip etmenin ivme vergisi
    # a_hold = V_p·V_t/R_t = 18·15/32 = 8.44 m/s² — MAX_ACCEL'in %70'i.
    # Bu senaryo bir "isabet beklentisi" değil, ZARF SINIRI ÖLÇÜMÜ içindir.
    # Kuyruk açısı 31 m menzilde asin(31/32) = 75.6°, yani hedef neredeyse
    # tam YANDAN görünür — dedektör 0-15° görüş açısında kalibreli.
    "circle_xs": (800, "çok dar (~32 m) — ZARF SINIRI"),
}


def _daire_sureli(conn, roll_cmd, sure_s):
    """Belirli süre daire çiz (bekleme turu). _daire ile aynı trim mantığı."""
    import math as _m
    import time as _t
    yatis_deg = roll_cmd / 1000.0 * 45.0
    pitch_cmd = int(150 * (1.0 / _m.cos(_m.radians(yatis_deg))))
    t0 = _t.time()
    while not _abort and (_t.time() - t0) < sure_s:
        hold(conn, 0.5, roll=roll_cmd, pitch=_irtifa_pitch(pitch_cmd))


def _daire(conn, roll_cmd, etiket):
    """Sabit yatışla süresiz tur. Pitch yatışa göre ölçeklenir (irtifa korunsun)."""
    import math as _m
    yatis_deg = roll_cmd / 1000.0 * 45.0
    pitch_cmd = int(150 * (1.0 / _m.cos(_m.radians(yatis_deg))))
    print(f"[SCN] DAİRE {etiket} — roll={roll_cmd} (~{yatis_deg:.0f}° yatış), "
          f"pitch={pitch_cmd}")
    while not _abort:
        hold(conn, 0.5, roll=roll_cmd, pitch=_irtifa_pitch(pitch_cmd))


def scenario_circle(conn):
    _daire(conn, *DAIRE_CAPLARI["circle"])


AGRESIF_GAZ = 668           # FBWA HAM GAZ — ölçüldü: 668 → 15.0 m/s
                            # (00000308.BIN, gövde sürtünmesi eklendikten sonra)
TIRMANIS_MIN_THR = 600      # tırmanış/spiral için taban gaz


def tirmanis_throttle():
    """Tırmanış manevralarında gaz: GCS slider'ını dinler ama TABAN uygular.

    Neden taban var: burun yukarıdayken gaz düşük olursa uçak hız kaybedip
    stall eder ve düşer — senaryo biter. Slider daha yükseği isterse ona uyar,
    daha düşüğü isterse tırmanış boyunca tabanda kalır (düz/dönüş kısımlarında
    slider aynen geçerli). Yani "hedefi yavaşlat" isteği çalışır, uçak düşmez.
    """
    return max(gcs_throttle(), TIRMANIS_MIN_THR)


def scenario_aggressive(conn):
    # ⚠ BU SENARYO FBWA'DA KALIR. Sebebi: manevraları YUNUSLAMA AÇISI
    # komutlarıyla kuruyor (pitch 500-800 = 12-20° burun yukarı). FBWB'de
    # aynı komut TIRMANMA HIZI demek olurdu ve FBWB_CLIMB_RATE=2.0 m/s
    # tavanına takılıp manevra etkisiz kalırdı.
    # ⚠ Gaz da slider'dan ALINMAZ: FBWB'de slider hava hızı demek (300 =
    # 15 m/s); FBWA'da ham gaz olarak 300 verilirse uçak perdövitese girer.
    # Ölçülmüş FBWA seyir gazı kullanılır.
    if FBWB_AKTIF:
        print("[SCN] AGRESİF — yunuslama komutları için FBWA'ya geçiliyor")
        set_mode(conn, PLANE_MODE_FBWA)
        hold(conn, 1.0, throttle=AGRESIF_GAZ)
    print("[SCN] AGRESİF — rastgele manevralar")
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
            hold(conn, random.uniform(1.5, 3.0), throttle=AGRESIF_GAZ,
                 roll=s * random.randint(600, 900), pitch=200)
        elif m == "s_turn":
            print("[SCN] Keskin S-dönüşü")
            hold(conn, 1.5, roll=-750, pitch=200, throttle=AGRESIF_GAZ)
            hold(conn, 1.5, roll=750, pitch=200, throttle=AGRESIF_GAZ)
        elif m == "spiral":
            print("[SCN] Spiral tırmanış")
            hold(conn, random.uniform(3.0, 5.0),
                 roll=450, pitch=450, throttle=tirmanis_throttle())
        # toparlanma: kısa düz uçuş
        hold(conn, random.uniform(1.0, 2.0), throttle=AGRESIF_GAZ)


# ═══════════════════════════════════════════════════════════════════
#  ELİPS SENARYOSU  (kalkis_kare_inis dalından taşındı, 2026-08-17)
# ═══════════════════════════════════════════════════════════════════
# ⚠ TAŞIMA NOTU: o daldan YALNIZ bu senaryo alındı (kullanıcı kararı).
# `kare_gorev`, iniş bacağı, sayaç-bazlı faz geçişi ve oradaki görsel güdüm
# değişikliklerinin HİÇBİRİ getirilmedi.
#
# SONSUZ desendir: iniş yok, tur sayısı yok — "Durdur" denene kadar döner,
# avcı görsel güdümü istediği kadar üstüne salabilsin diye.
#
# Ölçümle belirlenmiş tasarım kararları (hepsi canlı SITL uçuşlarında,
# orijinal dalda):
#   • HAT TAKİBİ İZDÜŞÜMLE — parametreyi hızla entegre etmek gecikme
#     biriktiriyordu (sapma 35 m'de sabitlendi, tur 71 s / olması gereken 37 s).
#     Konum her devirde elipse izdüşürülünce sapma 1.4 m'ye indi.
#   • YATIŞ ÖN BESLEMESİ, İLERİDEN — yalnız hata-orantılı sürüş kavisli hattı
#     yapısal olarak gecikmeyle takip ediyordu (sapma 7 m, yatış +56/−27
#     salınımı). Eğrilik uçlarda 256 m'den 21 m'ye çöktüğü için ön besleme
#     HAVUÇ noktasından hesaplanır, bulunulan noktadan değil.
#   • HAVUÇ MESAFESİ HIZA BAĞLI — sabit 25 m, 18 m/s'de yalnız 1.4 s ileri
#     bakmak demekti; 1.6 × hız yapıldı.
#   • UZUN EKSEN KUZEY-GÜNEY'E ÇİVİLİ — eksen kalkış yaw'ına oturtulunca desen
#     her uçuşta başka yöne dönüyordu (kuzeyden 158° sapma ölçüldü).
#   • ÖLÇÜ 344 × 150 m — referans 224 × 98 m şekli 18 m/s ile UÇULAMIYOR:
#     uçlarda R_min = b²/a = 21.4 m, bu 57° yatış ister ve düzeltme payı
#     kalmıyor (üç uçuşta da şekil yumurtaya döndü). Aynı 2.3:1 oranı
#     korunarak 1.5 kat büyütülünce uçlar 32.7 m / 45° oldu.
#
# ⚠ BİLİNEN SINIR (orijinal dalda ölçüldü): bu desende düz uçuş ~%70 gaz
# ister. Panel sürgüsü bunun altındayken uçak alçalmak ZORUNDADIR — burun
# irtifayı tutar ama olmayan enerjiyi üretemez. İRTİFA TABANI YOKTUR.
# ⚠ Bu senaryo araç HAVADAYKEN yeniden BAŞLATILMAMALI (orijinal dalda iki
# kez araç düştü): devirde RC override akışı kesiliyor.
#
# ⚠ İRTİFA: varsayılan 30 m, orijinal dalın kendi kalkışına göre ayarlanmış.
# BİZDEKİ diğer senaryolar ~68 m'de uçuyor. Hedefi yükseğe almak için:
#     AVCI_ELIPS_IRTIFA=70 bash ~/.avci_sim/mkur.sh <etiket>
# (Geçerlilik bandı 20-250 m — 30 m bandın içinde, ama diğer senaryolarla
#  kıyas yapacaksan irtifayı eşitle.)
_ELIPS_A    = float(os.environ.get("AVCI_ELIPS_A", 172.0))
_ELIPS_B    = float(os.environ.get("AVCI_ELIPS_B", 75.0))
_ELIPS_TUR  = float(os.environ.get("AVCI_ELIPS_TUR", 0))   # 0 = SONSUZ
_ELIPS_HIZ  = float(os.environ.get("AVCI_ELIPS_HIZ", 18.0))
_ELIPS_BANK = int(os.environ.get("AVCI_ELIPS_BANK", 1000))
_ELIPS_IRT  = float(os.environ.get("AVCI_ELIPS_IRTIFA", 30.0))
_ELIPS_ROLL_LIM = os.environ.get("AVCI_ELIPS_ROLL_LIMIT", "55")
_ELIPS_ILERI = float(os.environ.get("AVCI_ELIPS_ILERI", 25.0))
# GAZ KAYNAĞI: varsayılan PANEL SÜRGÜSÜ — hız sabit tutulmaz, kullanıcı
# sürgüyle ayarlar (diğer senaryolar zaten böyle; elips tek istisnaydı).
# 0 yapılırsa gaz PI döngüsü _ELIPS_HIZ'i kilitler.
_ELIPS_GAZ_SLIDER = os.environ.get("AVCI_ELIPS_GAZ_SLIDER", "1") == "1"


def _elips_nokta(C, u, w, a, b, t):
    return (C[0] + a * math.sin(t) * u[0] - b * math.cos(t) * w[0],
            C[1] + a * math.sin(t) * u[1] - b * math.cos(t) * w[1])


def elips_ciz(conn, a, b, tur, hiz_hedef, bank, irtifa_hedef, ileri,
              roll_limit_deg=45.0):
    """Havuç takibi + gaz ile HIZ, burun ile İRTİFA denetimi.

    Gaz ve burun AYRI organları sürer (iki döngü aynı organı çekmesin).
    """
    _pump(conn)
    yaw = _att["yaw"]
    u = (1.0, 0.0)                                   # kuzey
    w = (0.0, 1.0)                                   # doğu (u'nun sağı)
    P0 = (_pos["x"], _pos["y"])
    # GİRİŞ NOKTASI: teğeti uçağın burnuna en yakın parametre seçilir ki
    # desene sarsıntısız girsin.
    t_bas, _fark = 0.0, 9e9
    for _k in range(720):
        _tt = math.radians(_k * 0.5)
        _tx = a * math.cos(_tt) * u[0] + b * math.sin(_tt) * w[0]
        _ty = a * math.cos(_tt) * u[1] + b * math.sin(_tt) * w[1]
        _f = abs(_angdiff(math.atan2(_ty, _tx), yaw))
        if _f < _fark:
            _fark, t_bas = _f, _tt
    C = (P0[0] - (a * math.sin(t_bas) * u[0] - b * math.cos(t_bas) * w[0]),
         P0[1] - (a * math.sin(t_bas) * u[1] - b * math.cos(t_bas) * w[1]))
    R_min = b * b / a
    _gaz_et = ("gaz PANEL SÜRGÜSÜNDEN (hız sabit tutulmaz)" if _ELIPS_GAZ_SLIDER
               else f"hedef hız {hiz_hedef:.1f} m/s (PI)")
    print(f"[ELIPS] {2*a:.0f} x {2*b:.0f} m, {_gaz_et}, "
          f"irtifa {irtifa_hedef:.0f} m, yatış komutu {bank}")
    print(f"[ELIPS] en dar viraj yarıçapı {R_min:.1f} m -> "
          f"{math.degrees(math.atan(hiz_hedef**2/(9.81*R_min))):.0f}° yatış ister")
    sonsuz = tur <= 0
    t = t_bas
    t_son = float("inf") if sonsuz else t_bas + 2 * math.pi * tur
    cevre = math.pi * (3*(a+b) - math.sqrt((3*a+b)*(a+3*b)))
    zaman_asimi = float("inf") if sonsuz else tur * cevre / max(hiz_hedef, 5.0) * 2.5
    if sonsuz:
        print("[ELIPS] SONSUZ desen — durdurulana kadar çizilir, İNİŞ YOK")
    t0, son_bilgi = time.time(), 0.0
    sapmalar, hizlar, irtifalar = [], [], []
    gaz_i = 0.0                      # hız integrali (kalıcı hatayı kapatır)
    while not _abort and t < t_son and time.time() - t0 < zaman_asimi:
        _pump(conn)
        hiz, irtifa = _yer_hizi(), -_pos["z"]
        # UÇAĞIN KONUMUNU ELİPSE İZDÜŞÜR (yerel arama, geri gitmez)
        en_iyi, en_uz = t, 1e18
        for _k in range(-3, 41):
            _tt = t + _k * 0.02
            _px, _py = _elips_nokta(C, u, w, a, b, _tt)
            _d = math.hypot(_pos["x"] - _px, _pos["y"] - _py)
            if _d < en_uz:
                en_uz, en_iyi = _d, _tt
        t, uz = en_iyi, en_uz
        tur_hiz = max(math.hypot(
            a*math.cos(t)*u[0] + b*math.sin(t)*w[0],
            a*math.cos(t)*u[1] + b*math.sin(t)*w[1]), 1.0)
        # ── HAVUÇ: hıza göre ileri bak ──
        ileri_m = max(ileri, hiz * 1.6)
        hx, hy = _elips_nokta(C, u, w, a, b, t + ileri_m / tur_hiz)
        hata = _angdiff(_hedefe(hx, hy)[1], _att["yaw"])
        oran = _kelepce(hata / math.radians(35.0), -1.0, 1.0)
        # ── YATIŞ ÖN BESLEME: yerel eğrilik yarıçapının GEREKTİRDİĞİ yatış ──
        t_ff = t + ileri_m / tur_hiz
        R_yerel = ((b*b*math.sin(t_ff)**2 + a*a*math.cos(t_ff)**2) ** 1.5) / (a * b)
        bank_ff = math.degrees(math.atan(hiz*hiz / (9.81 * max(R_yerel, 5.0))))
        ff_cmd = _kelepce(bank_ff / max(roll_limit_deg, 1.0) * 1000.0, 0.0, bank)
        roll_cmd = int(_kelepce(ff_cmd + bank * oran * 0.6, -bank, bank))
        # ── BURUN: irtifa + YÜK FAKTÖRÜ ön beslemesi (yatışta taşıma kaybı) ──
        yuk = 1.0 / max(math.cos(_att["roll"]), 0.25)
        burun = int(_kelepce((irtifa_hedef - irtifa) * 55 + _pos["vz"] * 70
                             + (yuk - 1.0) * 220, -250, 450))
        # ── GAZ ──
        if _ELIPS_GAZ_SLIDER:
            gaz = int(gcs_throttle())
        else:
            h_hata = hiz_hedef - hiz
            gaz_i = _kelepce(gaz_i + h_hata * 8.0 * CONTROL_RATE, -250.0, 250.0)
            gaz = int(_kelepce(650 + h_hata * 150 + gaz_i, 200, 1000))
        _rc(conn, roll=roll_cmd, pitch=burun, throttle=gaz)
        if t > 0.5:
            sapmalar.append(uz); hizlar.append(hiz); irtifalar.append(irtifa)
        if time.time() - son_bilgi > 6.0:
            son_bilgi = time.time()
            print(f"[ELIPS] tur {(t-t_bas)/(2*math.pi):5.2f} | hız {hiz:5.2f} | "
                  f"irtifa {irtifa:5.1f} | yatış {math.degrees(_att['roll']):+5.1f} | "
                  f"sapma {uz:4.1f} m")
        time.sleep(CONTROL_RATE)
    n = len(sapmalar) or 1
    print(f"[ELIPS] DESEN BİTTİ — {(t-t_bas)/(2*math.pi):.2f} tur, {time.time()-t0:.0f} s")
    print(f"[ELIPS] ÖLÇÜM: sapma ort {sum(sapmalar)/n:.1f} m / tepe "
          f"{max(sapmalar or [0]):.1f} m | hız ort {sum(hizlar)/n:.2f} m/s "
          f"(yayılım {max(hizlar or [0])-min(hizlar or [0]):.2f}) | irtifa "
          f"{min(irtifalar or [0]):.1f}-{max(irtifalar or [0]):.1f} m")


def scenario_elips_gorev(conn):
    """Sonsuz elips deseni — İNİŞ YOK, durdurulana kadar döner."""
    eski_roll = None
    if _ELIPS_ROLL_LIM:
        eski_roll = _param_yaz(conn, "ROLL_LIMIT_DEG", float(_ELIPS_ROLL_LIM))
    try:
        elips_ciz(conn, _ELIPS_A, _ELIPS_B, _ELIPS_TUR, _ELIPS_HIZ,
                  _ELIPS_BANK, _ELIPS_IRT, _ELIPS_ILERI,
                  float(_ELIPS_ROLL_LIM) if _ELIPS_ROLL_LIM else 45.0)
    finally:
        # ROLL_LIMIT_DEG mutlaka geri alınır — başka senaryolar etkilenmesin
        if eski_roll is not None:
            _param_yaz(conn, "ROLL_LIMIT_DEG", eski_roll)


SCENARIOS = {
    "duz": scenario_duz,
    "square": scenario_square,
    "circle": scenario_circle,
    "aggressive": scenario_aggressive,
    "elips_gorev": scenario_elips_gorev,
}

# Beş daire çapı tek tek kaydedilir (functools.partial yerine varsayılan
# argümanlı lambda: döngü değişkeni geç-bağlanma tuzağına düşmesin).
for _ad, (_roll, _etiket) in DAIRE_CAPLARI.items():
    if _ad != "circle":
        SCENARIOS[_ad] = (lambda c, r=_roll, e=_etiket: _daire(c, r, e))


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
              f"kalkış atlanıyor, doğrudan {_MOD_ADI} + desen")
        _rc(conn, throttle=ch3())                 # override akışı hemen başlasın
        set_mode(conn, SEYIR_MODU, confirm_timeout=0)
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
