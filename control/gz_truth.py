"""
gz_truth.py — Gazebo'dan araçların GERÇEK pozu (ground truth), analiz referansı.

NEDEN: doğruluk analizinin ölçüm zemini MAVLink telemetrisi OLAMAZ. Hedefin
`telemetry_state['plane']` pozu iki katman hata taşır: (1) ArduPlane EKF kestirimi,
(2) `gcs_server._frame_off` EMA çerçeve kalibrasyonu (iki SITL'in EKF orijinleri
farklı). Pose modelinin kestirimini bu referansa göre ölçmek, iki hatayı birbirine
karıştırmak olur. Gazebo ise fiziğin kendisidir — hata payı sıfır.

Kaynak: `/world/<world>/dynamic_pose/info` (gz-transport, Pose_V). Yalnız HAREKET
EDEN varlıkları yayınlar; ~58 Hz, mesaj başına ~16 kayıt (ölçüldü 2026-07-27).
Kamera 30 Hz olduğundan referans her kare için taze.

ÇERÇEVE: Gazebo dünya çerçevesi (ENU), RPY radyan. ArduPilot NED'ine ÇEVRİLMEZ —
`vision/geometry.py` (target_keypoints / target_bbox / camera_world_pose) tam da bu
çerçeveyi bekler, yani bu modülün çıktısı oraya doğrudan girer. Mesafe zaten
çerçeveden bağımsızdır.

KURAL (bkz. CLAUDE.md §8): buradan gelen veri YALNIZ log/analiz içindir. Güdüm
hesabına sızarsa simülasyon kendi kendini kandırır — gerçek donanımda bu veri yok.

Kullanım:
    from control import gz_truth
    gz_truth.baslat()                  # bir kez; idempotent
    t = gz_truth.get_hedef()           # {x,y,z,roll,pitch,yaw,stamp} veya None
"""

import math
import os
import threading
import time

# Gazebo world adı ve izlenecek ÜST DÜZEY model adları (avci_harmonic.sdf ile
# birebir; nested modeller — iris_with_standoffs, base_link, rotor_* — atlanır,
# onların pozu ebeveyne GÖREDİR, dünya çerçevesinde değil).
WORLD = os.environ.get("AVCI_GZ_WORLD", "avci")
IRIS_MODEL = os.environ.get("AVCI_GZ_IRIS_MODEL", "iris_with_ardupilot")
HEDEF_MODEL = os.environ.get("AVCI_GZ_HEDEF_MODEL", "mini_talon")

_lock = threading.Lock()
_iris = None            # {x,y,z,roll,pitch,yaw,stamp} veya None
_hedef = None
_thread = None
_aktif = False
_mesaj_sayaci = 0


def _quat_to_rpy(x, y, z, w):
    """Kuaterniyon → (roll, pitch, yaw) radyan.

    ZYX sırası: R = Rz(yaw)·Ry(pitch)·Rx(roll) — `vision/geometry.rot_rpy` ile
    AYNI konvansiyon. Farklı bir sıra kullanılırsa projekte edilen keypoint'ler
    sessizce kayar, o yüzden burası geometry ile kilitli tutulmalıdır."""
    # roll (X ekseni)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (Y ekseni) — gimbal kilidinde asin taşmasın diye kırpılır
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    # yaw (Z ekseni)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _cb(msg):
    """Pose_V geldiğinde: yalnız iki üst düzey modeli ayıkla, sözlüğe yaz."""
    global _iris, _hedef, _mesaj_sayaci
    stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    yeni_iris = yeni_hedef = None
    for p in msg.pose:
        if p.name == IRIS_MODEL:
            hedefi = "iris"
        elif p.name == HEDEF_MODEL:
            hedefi = "hedef"
        else:
            continue
        o = p.orientation
        roll, pitch, yaw = _quat_to_rpy(o.x, o.y, o.z, o.w)
        kayit = {"x": p.position.x, "y": p.position.y, "z": p.position.z,
                 "roll": roll, "pitch": pitch, "yaw": yaw, "stamp": stamp}
        if hedefi == "iris":
            yeni_iris = kayit
        else:
            yeni_hedef = kayit
    with _lock:
        # Kısmi mesaj (yalnız biri hareket etmişse) diğerini SİLMEZ
        if yeni_iris is not None:
            _iris = yeni_iris
        if yeni_hedef is not None:
            _hedef = yeni_hedef
        _mesaj_sayaci += 1


def _dinle():
    """Abone thread'i. gz-transport aboneliği callback'i kendi thread'inde
    çalıştırır; burada yalnız node'u canlı tutmak için bekleriz."""
    global _aktif
    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
    except Exception as e:
        print(f"[TRUTH] gz-transport Python yok, ground truth kapalı: {e}")
        return
    topic = f"/world/{WORLD}/dynamic_pose/info"
    node = GzNode()
    if not node.subscribe(Pose_V, topic, _cb):
        print(f"[TRUTH] HATA: {topic} aboneliği başarısız — world adı doğru mu? "
              f"(AVCI_GZ_WORLD ile değiştirilebilir)")
        return
    _aktif = True
    print(f"[TRUTH] Gazebo ground truth dinleniyor ({topic}; "
          f"iris='{IRIS_MODEL}', hedef='{HEDEF_MODEL}')")
    while True:
        time.sleep(1)


def baslat():
    """Abone thread'ini başlat (idempotent). gcs_server startup'ında çağrılır."""
    global _thread
    if _thread is not None:
        return
    _thread = threading.Thread(target=_dinle, daemon=True)
    _thread.start()


def get_iris():
    """Avcının GERÇEK Gazebo pozu — {x,y,z,roll,pitch,yaw,stamp} veya None."""
    with _lock:
        return dict(_iris) if _iris is not None else None


def get_hedef():
    """Hedefin GERÇEK Gazebo pozu — {x,y,z,roll,pitch,yaw,stamp} veya None."""
    with _lock:
        return dict(_hedef) if _hedef is not None else None


def get_ikisi():
    """(iris, hedef) tek kilitte — analizde ikisi AYNI ana ait olmalı."""
    with _lock:
        return (dict(_iris) if _iris is not None else None,
                dict(_hedef) if _hedef is not None else None)


def menzil():
    """iris ↔ hedef gerçek merkez-merkez mesafesi (m) veya None."""
    i, h = get_ikisi()
    if i is None or h is None:
        return None
    return math.sqrt((h["x"] - i["x"]) ** 2 + (h["y"] - i["y"]) ** 2
                     + (h["z"] - i["z"]) ** 2)


def durum():
    """Teşhis: abone canlı mı, kaç mesaj geldi, iki poz da var mı."""
    with _lock:
        return {"aktif": _aktif, "mesaj": _mesaj_sayaci,
                "iris_var": _iris is not None, "hedef_var": _hedef is not None}
