"""
geometry_keypoints.py — Talon KEYPOINT geometrisi (POSE MODELİ DÖNEMİ).

⛔ CANLI SİSTEMDE KULLANILMIYOR. 2026-08-10'da `vision/geometry.py`'den buraya
alındı: pose modeli 2026-08-06'da kaldırıldıktan sonra bu blok orada yaşamaya
devam ediyordu ama DIŞARIDAN HİÇBİR YERDEN çağrılmıyordu (yalnız kendi
içinden). `vision/geometry.py`'de kalan her şey canlı: kamera iç parametreleri
(FX/FY/CX/CY), 3B kutu ve projeksiyon — `bbox_ibvs` onları kullanıyor.

Geri getirmek istersen: bu dosyanın içeriğini `vision/geometry.py`'nin sonuna
yapıştır, `from vision.geometry import ...` satırını sil. Pose modelinin
tamamı için bkz. bu klasörün README.md'si.

İçerik:
  KEYPOINT_NAMES / KEYPOINT_FLIP_IDX  — 6 keypoint'in adı ve flip eşleşmesi
  talon_keypoints()   — keypoint'lerin gövde çerçevesindeki 3B konumu
  talon_triangles()   — collision mesh üçgenleri (örtülme testi için)
  occluded_mask()     — öz-örtülme (Möller-Trumbore ışın-mesh testi)
  target_keypoints()  — keypoint'leri piksele projekte et (YOLO-pose etiketi)
  pose_rpy_cozum()    — 6 keypoint'ten hedefin rotasyonu (solvePnP; yalnız ÖLÇÜM)
"""
import math
import os

import numpy as np

from vision.geometry import (
    CX, CY, FX, FY, IMG_H, IMG_W, _MESH_DIR, _stl_vertices,
    camera_world_pose, project_points, rot_rpy, talon_vertices,
)

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


def rpy_cozum(kpts, iris_rpy, kpt_conf_min=0.5):
    """Keypoint'lerden hedefin DÜNYA roll/pitch/yaw'ı — PnP çözümü.
    (Eski adı `geometry.pose_rpy_cozum`.)

    NEDEN VARDI: pose modeli doğrudan rotasyon vermez, 6 keypoint verir.
    Güdüm bundan yalnız iki türev kullanıyordu (gövde ekseninin görüntüdeki
    yönü ve yandanlık) — tam 3B rotasyon hiç hesaplanmazdı, lead için
    gerekmiyordu. Bu fonksiyon yalnız ÖLÇÜM içindi: "pose modeli hedefin
    rotasyonunu ne kadar doğru kestirebilirdi" (cevap anahtarı kıyası).

    kpts     : 6×(u, v, conf) tam kare pikselde (pose_detector çıktısı)
    iris_rpy : kendi aracımızın (roll, pitch, yaw) radyan
    Dönüş    : (roll, pitch, yaw) radyan DÜNYA çerçevesinde, veya None.

    Çözüm zinciri: solvePnP hedefin KAMERAYA göre rotasyonunu verir
    (OpenCV optik çerçeve); önce link çerçevesine, sonra kamera dünya
    rotasyonuyla çarpılıp dünya çerçevesine taşınır.
    """
    try:
        import cv2
    except ImportError:
        return None
    model = talon_keypoints()
    obj, img = [], []
    for i, (u, v, c) in enumerate(kpts):
        if c is None or c < kpt_conf_min:
            continue
        if u == 0 and v == 0:              # YOLO-pose "görünmez" kuralı
            continue
        obj.append(model[i])
        img.append((u, v))
    if len(obj) < 4:                       # PnP en az 4 nokta ister
        return None
    K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
    # ITERATIVE (DLT) en az 6 nokta ister; uçuşta çoğu karede 4-5 keypoint
    # görünür (bir kanat/kuyruk gövde arkasında kalır). EPNP 4 noktayla çalışır.
    bayrak = cv2.SOLVEPNP_ITERATIVE if len(obj) >= 6 else cv2.SOLVEPNP_EPNP
    try:
        ok, rvec, _ = cv2.solvePnP(
            np.asarray(obj, np.float64),
            np.asarray(img, np.float64).reshape(-1, 1, 2),
            K, None, flags=bayrak)
    except cv2.error:
        return None
    if not ok:
        return None
    R_opt, _ = cv2.Rodrigues(rvec)         # hedef → OPTİK çerçeve

    # optik (X sağ, Y aşağı, Z ileri) → link (X ileri, Y sol, Z yukarı)
    R_link_opt = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    _, R_cam = camera_world_pose((0.0, 0.0, 0.0), iris_rpy)   # yalnız rotasyon
    R_dunya = R_cam @ R_link_opt @ R_opt

    # rot_rpy = Rz(yaw)·Ry(pitch)·Rx(roll) tersine çözümü
    pitch = math.asin(max(-1.0, min(1.0, -float(R_dunya[2, 0]))))
    if abs(math.cos(pitch)) < 1e-6:        # gimbal kilidi
        return None
    roll = math.atan2(float(R_dunya[2, 1]), float(R_dunya[2, 2]))
    yaw = math.atan2(float(R_dunya[1, 0]), float(R_dunya[0, 0]))
    return roll, pitch, yaw
