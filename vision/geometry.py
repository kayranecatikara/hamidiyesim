"""
vision/geometry.py — Kamera projeksiyon + Talon 3D kutu (otomatik etiketleme için).

set_pose ile bilinen pozlardan Talon'un 2D bounding box'ını hesaplar:
  1. Talon collision mesh'lerinden base_link çerçevesinde 3D AABB → 8 köşe
  2. Köşeleri hedefin world pozu/yaw'ıyla dünyaya taşı
  3. Kamera intrinsics + extrinsics ile 2D piksele projekte et
  4. Görünür köşelerin min/max'ı = 2D bbox

Kamera (2026-09-06'dan beri TARETTE — iris_cam/model.sdf turret_pitch_link):
FOV 125°, 640×480. Artık gövdeye sabit DEĞİL; pan/tilt taretinin tilt linkinde
duruyor ve namluyla aynı doğrultuya bakıyor. Gazebo/SDF ile birebir tutarlı.
"""

import glob
import math
import os

import numpy as np

# ══ Kamera parametreleri — GERÇEK DONANIMLA EŞLEŞİR ══════════════════════
# Waveshare IMX477-160 (ürün kodu 22232):
#   sensör  Sony IMX477, 4056×3040 (12.3 MP), 7.857 × 5.895 mm (1/2.3")
#   lens    balıkgöz, ilan edilen 160° KÖŞEGEN
#
# 160° köşegen → yatay: eş uzaklık (equidistant, r = f·θ) izdüşümünde açı
# yarıçapla doğru orantılıdır, dolayısıyla FOV'lar sensör kenar oranlarıyla
# ölçeklenir:  yatay = 160 × 7.857/9.823 = 128.0° ,  dikey = 96.0°
# (Düz izdüşüm varsayılsaydı 155.1° çıkardı — yanlış model, 27° hata.)
#
# Çözünürlük 1352×1014 = sensörün tam 1/3'ü. İKİ ölçümle seçildi:
#   render maliyeti  (tools/kamera_olcek_dene.py) → 3 MP'ye kadar bedava
#   tespit maliyeti  (gcs_server.process_iris_frame) → 1014 ve 1352 AYNI
#                     (28.6 ms), 2028 daha pahalı (32.9 ms)
# Ve tespit modeli imgsz=1280 kullanıyor: 1352 o doyum noktasının hemen
# üstü, yani ne bilgi uyduruluyor ne de boşa piksel ödeniyor.
IMG_W, IMG_H = 1352, 1014
HFOV_RAD = 2.23371                                # 128.0° yatay
FX = FY = (IMG_W / 2.0) / math.tan(HFOV_RAD / 2.0)  # ≈ 329.7 px
CX, CY = IMG_W / 2.0, IMG_H / 2.0                 # 676, 507

# Güdümdeki piksel birimli sabitler bu referansa göre ölçeklenir. Değer,
# sabitlerin ÖLÇÜLDÜĞÜ eski kameranın FX'idir (640×480 @ 125°).
FX_OLCUM_REFERANSI = 166.58

# ══ TARET GEOMETRİSİ (iris_cam/model.sdf ile BİREBİR tutulmalı) ══
#
# Kamera 2026-09-06'da gövdeden TARETE taşındı. Konumu artık taretin pan/tilt
# açılarına bağlı. SDF'teki değerler:
#   turret_pitch_link  <pose> 0 0 0.095      → pan ve tilt eksenlerinin kesişimi
#   <sensor camera>    <pose> 0.12 0 0.035   → kamera, tilt linkine göre
#   turret_pitch_joint <axis> 0 -1 0         → POZİTİF açı = namlu YUKARI
#   turret_yaw_joint   <axis> 0 0 1          → POZİTİF açı = burun SOLA
#
# Pan ekseni Z'dir ve tilt linki tam Z ekseni üzerindedir (x=0,y=0); bu yüzden
# pan dönmesi (0,0,0.095) noktasını YERİNDEN OYNATMAZ.
TARET_EKSEN_MERKEZI = np.array([0.0, 0.0, 0.095])   # base_link'e göre
KAMERA_TARET_OFSETI = np.array([0.12, 0.0, 0.035])  # tilt linkine göre

# Taretin VARSAYILAN duruşu (SDF'teki <initial_position> ile aynı olmalı).
# tilt = +25°: eski gövdeye-sabit kameranın baktığı yönün AYNISI — böylece
# taret eklenmesi mevcut güdümü/tespiti değiştirmez.
TARET_PAN_VARSAYILAN = 0.0        # rad
TARET_TILT_VARSAYILAN = 0.4363    # rad = +25° yukarı


# ⚠ kamera_ekstrinsik() ve ondan türeyen CAM_OFFSET_POS / CAM_TILT_RAD
#   rotasyon matrislerine ihtiyaç duyduğu için AŞAĞIDA, rot_rpy'den
#   hemen sonra tanımlıdır.


_MESH_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "sim", "gazebo_harmonic", "models", "mini_talon_vtail", "meshes",
)


# ── Rotasyon matrisleri (world/link, sağ-el) ──
def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_rpy(roll, pitch, yaw):
    """Gazebo pose RPY (roll,pitch,yaw) → rotasyon matrisi: Rz(yaw)·Ry(pitch)·Rx(roll)."""
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def kamera_ekstrinsik(pan_rad=None, tilt_rad=None):
    """Taret açılarından kamera (konum, rotasyon) — base_link çerçevesinde.

    Taret hareket ettiğinde projeksiyonun doğru kalması için VARDIR. Taret
    sürücüsü yazıldığında canlı açılar buraya verilir; verilmezse SDF'teki
    varsayılan duruş kullanılır.

    Dönüş:
      pos (3,)  : kamera merkezinin base_link'e göre konumu, metre
      R   (3,3) : kamera çerçevesinin base_link'e göre rotasyonu
                  (kamera +X'i ileri bakar — namluyla aynı doğrultu)
    """
    pan = TARET_PAN_VARSAYILAN if pan_rad is None else float(pan_rad)
    tilt = TARET_TILT_VARSAYILAN if tilt_rad is None else float(tilt_rad)
    # tilt ekseni SDF'te (0,-1,0): +θ kadar −Y etrafında dönme
    #   = −θ kadar +Y etrafında dönme  → _rot_y(-tilt)
    R = _rot_z(pan) @ _rot_y(-tilt)
    return TARET_EKSEN_MERKEZI + R @ KAMERA_TARET_OFSETI, R


# ── NAMLU (ağ fırlatıcı çıkışı) ────────────────────────────────────────
# Namlu da taretin tilt linkindedir ve KAMERAYLA AYNI DOĞRULTUYA bakar
# (boresight). SDF'teki değerler (iris_cam/model.sdf):
#   muzzle_link  <pose> 0.06 0 0.095   → model çerçevesinde namlunun MERKEZİ
#   silindir uzunluğu 0.10 m           → uç, merkezden 0.05 m ileride
# Tilt linkinin orijini (0,0,0.095) olduğu için namlu merkezi o çerçevede
# (0.06, 0, 0), ucu (0.11, 0, 0).
NAMLU_UC_OFSETI = np.array([0.11, 0.0, 0.0])      # tilt linkine göre, metre


def namlu_ekstrinsik(pan_rad=None, tilt_rad=None):
    """Namlu UCUNUN konumu ve ATIŞ DOĞRULTUSU — base_link çerçevesinde.

    Kamerayla aynı rotasyonu paylaşır (boresight); yalnız orijin farklıdır.
    Dönüş:
      uc  (3,) : namlu ucunun base_link'e göre konumu, metre
      yon (3,) : birim atış doğrultusu (taretin +X'i)
    """
    pan = TARET_PAN_VARSAYILAN if pan_rad is None else float(pan_rad)
    tilt = TARET_TILT_VARSAYILAN if tilt_rad is None else float(tilt_rad)
    R = _rot_z(pan) @ _rot_y(-tilt)
    return TARET_EKSEN_MERKEZI + R @ NAMLU_UC_OFSETI, R @ np.array([1.0, 0.0, 0.0])


# Geriye dönük uyumluluk: modül düzeyindeki sabitler, taret VARSAYILAN
# duruşundaki değerlerdir. (Eski: konum (0.10, 0, 0.05) — gövdeye sabitken.
#  Yeni: (0.094, 0, 0.177) — taret 12.7 cm daha yukarıda. Bakış YÖNÜ aynı.)
CAM_OFFSET_POS = TARET_EKSEN_MERKEZI + (
    _rot_y(-TARET_TILT_VARSAYILAN) @ KAMERA_TARET_OFSETI)
CAM_TILT_RAD = -TARET_TILT_VARSAYILAN             # kamera pitch (negatif = yukarı)


# ── Talon 3D kutu (collision STL'lerden) ──
def _stl_vertices(path):
    """Binary STL → (N,3) vertex dizisi."""
    with open(path, "rb") as f:
        f.seek(80)
        n = int(np.frombuffer(f.read(4), np.uint32)[0])
        d = np.frombuffer(f.read(n * 50), np.uint8).reshape(n, 50)
    # her üçgen: bytes 12..48 = 9 float32 (3 vertex)
    return d[:, 12:48].copy().view(np.float32).reshape(-1, 3)


def talon_aabb():
    """Tüm collision mesh'lerin birleşik AABB'si (base_link frame): (min, max)."""
    mn = np.array([1e9] * 3)
    mx = np.array([-1e9] * 3)
    files = glob.glob(os.path.join(_MESH_DIR, "*collision*.stl"))
    if not files:
        raise FileNotFoundError(f"collision STL bulunamadı: {_MESH_DIR}")
    for p in files:
        v = _stl_vertices(p)
        mn = np.minimum(mn, v.min(0))
        mx = np.maximum(mx, v.max(0))
    return mn, mx


def talon_box_corners():
    """AABB'nin 8 köşesi (base_link frame, (8,3)). (Not: açılı araçta gevşek bbox
    verir — target_bbox artık talon_vertices() kullanır.)"""
    mn, mx = talon_aabb()
    return np.array([[x, y, z]
                     for x in (mn[0], mx[0])
                     for y in (mn[1], mx[1])
                     for z in (mn[2], mx[2])])


_TALON_VERTS = None


def talon_vertices():
    """Tüm collision mesh vertex'leri (base_link frame, (N,3)), cache'li.
    8-köşe AABB yerine bunları projekte etmek SIKI bbox verir: araç hangi açıdan
    görünürse gerçek silueti neyse bbox ona oturur (AABB kanat+gövdeyi birden
    sarıp gevşek kalıyordu)."""
    global _TALON_VERTS
    if _TALON_VERTS is None:
        parts = [_stl_vertices(p)
                 for p in glob.glob(os.path.join(_MESH_DIR, "*collision*.stl"))]
        _TALON_VERTS = np.vstack(parts)
    return _TALON_VERTS


# ── Talon keypoint'leri (pose modeli için) ──
KEYPOINT_NAMES = ["burun", "kuyruk", "sol_kanat", "sag_kanat", "sol_vtail", "sag_vtail"]
KEYPOINT_FLIP_IDX = [0, 1, 3, 2, 5, 4]   # yatay flip augmentasyonunda sol↔sağ eşleşmesi

_TALON_KPTS = None


def talon_keypoints():
    """Talon keypoint'leri (base_link frame, (6,3), KEYPOINT_NAMES sırasında).
    Parça collision STL'lerinin uç noktalarından türetilir (mesh değişirse
    kendiliğinden günceller), cache'li."""
    global _TALON_KPTS
    if _TALON_KPTS is None:
        def part(name):
            return _stl_vertices(
                os.path.join(_MESH_DIR, f"mini_talon_{name}_collision.stl"))
        fus = part("fuselage")
        lw, rw = part("left_wing"), part("right_wing")
        lt, rt = part("left_tail"), part("right_tail")
        _TALON_KPTS = np.array([
            fus[fus[:, 0].argmax()],                     # burun
            fus[fus[:, 0].argmin()],                     # kuyruk (gövde arkası)
            lw[lw[:, 1].argmax()],                       # sol kanat ucu
            rw[rw[:, 1].argmin()],                       # sağ kanat ucu
            lt[(np.abs(lt[:, 1]) + lt[:, 2]).argmax()],  # sol V-tail ucu
            rt[(np.abs(rt[:, 1]) + rt[:, 2]).argmax()],  # sağ V-tail ucu
        ])
    return _TALON_KPTS


_TALON_TRIS = None


def talon_triangles():
    """Tüm collision mesh üçgenleri (base_link frame, (M,3,3)), cache'li.
    STL vertex dizilimi üçgen-sıralı olduğundan reshape yeterli."""
    global _TALON_TRIS
    if _TALON_TRIS is None:
        _TALON_TRIS = talon_vertices().reshape(-1, 3, 3).astype(float)
    return _TALON_TRIS


# Işın kpt'ye varmadan bu mesafeden fazla önce mesh'e çarparsa örtülü say;
# kpt'nin kendi yüzey üçgenlerinin sahte "çarpma" vermemesi için tampon.
_OCCL_EPS = 0.02


def occluded_mask(cam_pos, target_pos, R_t):
    """Her keypoint için öz-örtülme testi (Möller–Trumbore, body frame'de).
    Kameradan keypoint'e ışın, aracın kendi mesh'ine kpt'den _OCCL_EPS'ten
    daha önce çarpıyorsa o keypoint ÖRTÜLÜdür. Dönüş: (K,) bool."""
    cam_b = R_t.T @ (np.asarray(cam_pos, float) - np.asarray(target_pos, float))
    kpts = talon_keypoints()
    tris = talon_triangles()
    v0 = tris[:, 0]
    e1 = tris[:, 1] - v0
    e2 = tris[:, 2] - v0
    s = cam_b - v0
    q = np.cross(s, e1)
    out = np.zeros(len(kpts), bool)
    for k, kp in enumerate(kpts):
        d = kp - cam_b
        t_max = float(np.linalg.norm(d))
        if t_max < 1e-9:
            continue
        d = d / t_max
        p = np.cross(d, e2)
        det = (e1 * p).sum(1)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        u = (s * p).sum(1) * inv
        v = (d * q).sum(1) * inv
        t = (e2 * q).sum(1) * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6) & (t < t_max - _OCCL_EPS)
        out[k] = bool(hit.any())
    return out


def target_keypoints(target_pos, target_rpy, iris_pos, iris_rpy):
    """Keypoint'leri piksele projekte eder → (6,3) [u, v, vis].
    vis=2: kadraj içinde, kameranın önünde VE örtülü değil; vis=0: değil
    (u=v=0, YOLO-pose kuralı). Öz-örtülme (gövde/kanat arkasında kalan uç)
    occluded_mask ile ışın-mesh testinden hesaplanır — örtülü nokta etikete
    KONMAZ (eğitimi bozmasın)."""
    R_t = rot_rpy(*target_rpy)
    world = np.asarray(target_pos, float) + talon_keypoints() @ R_t.T
    cam_pos, R_cam = camera_world_pose(iris_pos, iris_rpy)
    u, v, front = project_points(world, cam_pos, R_cam)
    occl = occluded_mask(cam_pos, target_pos, R_t)
    out = np.zeros((len(u), 3))
    vis = front & ~occl & (u >= 0) & (u < IMG_W) & (v >= 0) & (v < IMG_H)
    out[vis, 0] = u[vis]
    out[vis, 1] = v[vis]
    out[vis, 2] = 2
    return out


# ── Kamera pozu + projeksiyon ──
def quat_to_rpy(qw, qx, qy, qz):
    """Gazebo quaternion (w,x,y,z) → (roll, pitch, yaw) radyan (ZYX)."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def rot_gt_goruntu(hedef_pos, hedef_rpy, iris_pos, iris_rpy):
    """Hedefin GERÇEK yöneliminden görsel güdümün rotasyon girdileri
    (pose modeli DEVRE DIŞI modu, 2026-08-02 — keypoint işaretlemesiz):
      d_birim   : hedef gövde ekseninin görüntü düzlemindeki birim yönü (px
                  uzayı; burun yönü) — lead kaydırmasının yönü. Eksen bakış
                  doğrultusuyla çakışıksa None.
      yandanlik : gövde ekseninin LOS'a dik bileşeni |sin(θ)| (0=önden/arkadan,
                  1=tam yandan) — lead büyüklüğü.
      menzil    : kamera-hedef gerçek mesafe (m) — kalite kapısı için.
    Hedef kamera arkasındaysa None döner."""
    cam_pos, R_cam = camera_world_pose(iris_pos, iris_rpy)
    hedef = np.asarray(hedef_pos, dtype=float)
    f_w = rot_rpy(*hedef_rpy) @ np.array([1.0, 0.0, 0.0])   # hedef ileri (world)
    P = np.vstack([hedef, hedef + f_w])                     # merkez + 1 m burun
    u, v, valid = project_points(P, cam_pos, R_cam)
    if not bool(valid.all()):
        return None
    los = hedef - cam_pos
    menzil = float(np.linalg.norm(los))
    if menzil < 1e-6:
        return None
    los_b = los / menzil
    yandanlik = float(np.linalg.norm(f_w - float(np.dot(f_w, los_b)) * los_b))
    d = np.array([u[1] - u[0], v[1] - v[0]])
    n = float(np.linalg.norm(d))
    d_birim = (float(d[0] / n), float(d[1] / n)) if n > 1e-9 else None
    return {"d_birim": d_birim, "yandanlik": min(1.0, yandanlik), "menzil": menzil}


def camera_world_pose(iris_pos, iris_rpy, taret_pan=None, taret_tilt=None):
    """iris (drone) world poz + rpy'den kamera world (konum, rotasyon matrisi).

    iris_rpy   = (roll, pitch, yaw) radyan — drone GÖVDE oryantasyonu
    taret_pan  = taret pan açısı (rad, sola +); None ise varsayılan duruş
    taret_tilt = taret tilt açısı (rad, yukarı +); None ise varsayılan (25°)

    Kamera taretin tilt linkinde olduğu için toplam rotasyon iki katmandır:
        R_kamera_dünya = R_gövde · R_taret
    """
    iris_pos = np.asarray(iris_pos, dtype=float)
    roll, pitch, yaw = iris_rpy
    R_iris = rot_rpy(roll, pitch, yaw)
    ofs, R_taret = kamera_ekstrinsik(taret_pan, taret_tilt)
    cam_pos = iris_pos + R_iris @ ofs
    R_cam = R_iris @ R_taret
    return cam_pos, R_cam


def project_points(P_world, cam_pos, R_cam):
    """World noktaları (N,3) → piksel (u, v) + önde-mi maskesi."""
    P_link = (P_world - cam_pos) @ R_cam       # = R_cam^T @ (P - t)
    # optik frame: X_opt=-Y_link (sağ), Y_opt=-Z_link (aşağı), Z_opt=X_link (ileri)
    Xo, Yo, Zo = -P_link[:, 1], -P_link[:, 2], P_link[:, 0]
    valid = Zo > 0.01
    Zs = np.where(valid, Zo, 1.0)
    u = CX + FX * Xo / Zs
    v = CY + FY * Yo / Zs
    return u, v, valid


def target_bbox(target_pos, target_rpy, iris_pos, iris_rpy,
                margin_px=0):
    """
    Hedefin 2D bbox'ı: (x1, y1, x2, y2) piksel, veya görünmüyorsa None.
    target_rpy = (roll, pitch, yaw) radyan — hedefin TAM 3D oryantasyonu.
    Kadraj dışına taşan kısımlar kırpılır; en az 4 köşe önde ve bbox kadrajla
    kesişiyorsa döndürülür.
    """
    roll, pitch, yaw = target_rpy
    corners = talon_vertices()                          # tüm mesh vertex → SIKI bbox
    R_t = rot_rpy(roll, pitch, yaw)
    world_corners = np.asarray(target_pos, float) + corners @ R_t.T
    cam_pos, R_cam = camera_world_pose(iris_pos, iris_rpy)
    u, v, valid = project_points(world_corners, cam_pos, R_cam)
    if valid.sum() < 4:
        return None
    uu, vv = u[valid], v[valid]
    x1, y1, x2, y2 = uu.min(), vv.min(), uu.max(), vv.max()
    # tamamen kadraj dışında mı
    if x2 < 0 or x1 > IMG_W or y2 < 0 or y1 > IMG_H:
        return None
    x1 = max(0.0, x1 - margin_px); y1 = max(0.0, y1 - margin_px)
    x2 = min(float(IMG_W), x2 + margin_px); y2 = min(float(IMG_H), y2 + margin_px)
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None
    return (x1, y1, x2, y2)


def bbox_to_yolo(bbox):
    """(x1,y1,x2,y2) piksel → YOLO normalize (cx, cy, w, h) [0..1]."""
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2.0 / IMG_W
    cy = (y1 + y2) / 2.0 / IMG_H
    w = (x2 - x1) / IMG_W
    h = (y2 - y1) / IMG_H
    return cx, cy, w, h


if __name__ == "__main__":
    mn, mx = talon_aabb()
    size = (mx - mn)
    print("Talon 3D kutu (collision STL, base_link frame):")
    print(f"  min={mn.round(3)}  max={mx.round(3)}")
    print(f"  boyut  uzunluk(X)={size[0]:.2f}m  genişlik(Y)={size[1]:.2f}m  yükseklik(Z)={size[2]:.2f}m")

    # Sentetik doğrulama: hedefi kameranın optik ekseni üzerine 20 m koy → merkez ~(320,240)
    iris_pos = np.array([0.0, 0.0, 5.0])
    iris_rpy = (0.0, 0.0, 0.0)
    cam_pos, R_cam = camera_world_pose(iris_pos, iris_rpy)
    optical_fwd = R_cam @ np.array([1.0, 0.0, 0.0])     # 25° yukarı ileri
    tgt = cam_pos + 20.0 * optical_fwd
    bb = target_bbox(tgt, (0.0, 0.0, math.pi / 2), iris_pos, iris_rpy)
    print("\nSentetik test — hedef optik eksende 20 m (yandan görünüm):")
    if bb is None:
        print("  BBOX YOK (projeksiyon hatası!)")
    else:
        cxb, cyb = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
        print(f"  bbox=({bb[0]:.0f},{bb[1]:.0f},{bb[2]:.0f},{bb[3]:.0f})")
        print(f"  merkez=({cxb:.0f},{cyb:.0f})  beklenen≈(320,240)  "
              f"{'✓ DOĞRU' if abs(cxb-320)<40 and abs(cyb-240)<40 else '✗ SAPMA VAR'}")
