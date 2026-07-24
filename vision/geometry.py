"""
vision/geometry.py — Kamera projeksiyon + Talon 3D kutu (otomatik etiketleme için).

set_pose ile bilinen pozlardan Talon'un 2D bounding box'ını hesaplar:
  1. Talon collision mesh'lerinden base_link çerçevesinde 3D AABB → 8 köşe
  2. Köşeleri hedefin world pozu/yaw'ıyla dünyaya taşı
  3. Kamera intrinsics + extrinsics ile 2D piksele projekte et
  4. Görünür köşelerin min/max'ı = 2D bbox

Kamera (iris_with_standoffs/model.sdf): FOV 125°, 640×480; base_link'e göre pose
(0.10, 0, 0.05), pitch −0.4363 rad (25° YUKARI). Gazebo/SDF ile birebir tutarlı.
"""

import glob
import math
import os

import numpy as np

# ══ Kamera parametreleri (SDF ile tutarlı) ══
IMG_W, IMG_H = 640, 480
HFOV_RAD = 2.18166                                # 125°
FX = FY = (IMG_W / 2.0) / math.tan(HFOV_RAD / 2.0)  # ≈ 166.6
CX, CY = IMG_W / 2.0, IMG_H / 2.0                 # 320, 240

CAM_OFFSET_POS = np.array([0.10, 0.0, 0.05])      # base_link'e göre kamera konumu
CAM_TILT_RAD = -0.4363                            # kamera pitch (negatif = yukarı)

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
def camera_world_pose(iris_pos, iris_rpy):
    """iris (drone) world poz + rpy'den kamera world (konum, rotasyon matrisi).
    iris_rpy = (roll, pitch, yaw) radyan — drone gövde oryantasyonu.
    Kamera 25° yukarı tilt'i drone gövde çerçevesinde uygulanır."""
    iris_pos = np.asarray(iris_pos, dtype=float)
    roll, pitch, yaw = iris_rpy
    R_iris = rot_rpy(roll, pitch, yaw)
    cam_pos = iris_pos + R_iris @ CAM_OFFSET_POS
    R_cam = R_iris @ _rot_y(CAM_TILT_RAD)     # kamera tilt drone frame'de
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
