"""
vision/dogruluk.py — Pose modeli kestirimi ↔ Gazebo gerçeği karşılaştırması.

Görsel güdümün her karesinde, pose modelinin ürettiği keypoint'lerin GERÇEKTE
nerede olması gerektiğini `vision/geometry.target_keypoints` ile hesaplar ve
farkı ölçer. Çıktı yalnız CSV'ye yazılır — güdüm hesabına GİRMEZ (bkz. CLAUDE.md §8).

Neden bu karşılaştırma anlamlı: `geometry.target_keypoints` zaten pose modelinin
eğitim etiketlerini üreten fonksiyondur (`vision/capture_pose_dataset.py:163`).
Yani model tam olarak bu projeksiyona yakınsamaya çalıştı; ondan sapması doğrudan
modelin hatasıdır, farklı bir referansın değil.

Ölçülen dört soru:
  1. Keypoint'ler piksel olarak ne kadar sapıyor?        → kpt_hata_px_*
  2. Yönelim (yandanlık + eksen açısı) ne kadar doğru?   → yandanlik_gercek, eksen_aci_*
  3. Burun/kuyruk takas oluyor mu? (lead'i TERS çevirir) → burun_kuyruk_takas
  4. Menzil kestirimi ne kadar sapıyor?                  → menzil_gercek_gz_m

ÇERÇEVE: girdi pozları Gazebo dünya çerçevesi (ENU), RPY radyan —
`control/gz_truth.py` çıktısı doğrudan buraya girer.
"""

import math

import numpy as np

from vision import geometry as geo

# guidance_core ile AYNI indeksler ve AYNI oran — formüller birebir eşleşmezse
# karşılaştırma anlamsız olur. Değiştirilirse iki dosya birlikte değişmeli.
_I_BURUN, _I_KUYRUK, _I_SOLK, _I_SAGK = 0, 1, 2, 3
_GOVDE_KANAT_ORANI = 0.633        # = 0.81 / 1.28 (guidance_core.GOVDE_KANAT_ORANI)


def _gercek_kpts(hedef_pos, hedef_rpy, iris_pos, iris_rpy):
    """Keypoint'lerin HAM piksel projeksiyonu + görünürlük bayrakları.

    `geo.target_keypoints` KULLANILMAZ, çünkü o öz-örtülü noktaları (u,v)=(0,0)
    yapar — eğitim etiketi için doğru, ANALİZ için yıkıcı: kuyruktan takipte burun
    her zaman örtülü sayılır, yani `a_gercek` tam da ölçmek istediğimiz geometride
    hep boş kalırdı. Model o noktalar için yine de bir konum üretiyor ve güdüm onu
    yine de kullanıyor; dolayısıyla referans da örtülmeden bağımsız olmalı.
    Örtülme bilgisi atılmaz, ayrı bayrak olarak döner.

    Dönüş: (uv (6,2), onde (6,) bool, kadrajda (6,) bool, ortulu (6,) bool)
    """
    R_t = geo.rot_rpy(*hedef_rpy)
    world = np.asarray(hedef_pos, float) + geo.talon_keypoints() @ R_t.T
    cam_pos, R_cam = geo.camera_world_pose(iris_pos, iris_rpy)
    u, v, onde = geo.project_points(world, cam_pos, R_cam)
    kadrajda = (u >= 0) & (u < geo.IMG_W) & (v >= 0) & (v < geo.IMG_H)
    ortulu = geo.occluded_mask(cam_pos, hedef_pos, R_t)
    return np.column_stack([u, v]), onde, kadrajda, ortulu


def _eksen_aci_deg(p_burun, p_kuyruk):
    """Görüntü düzleminde gövde ekseninin açısı (derece, atan2(dv, du)).
    guidance_core'daki `d = burun - kuyruk` ile aynı yön tanımı."""
    du = p_burun[0] - p_kuyruk[0]
    dv = p_burun[1] - p_kuyruk[1]
    if abs(du) < 1e-9 and abs(dv) < 1e-9:
        return None
    return math.degrees(math.atan2(dv, du))


def _aci_farki_deg(a, b):
    """İki açı arasındaki en kısa fark, (-180, 180]."""
    if a is None or b is None:
        return None
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def olc(pose, iris_truth, hedef_truth):
    """Bir karenin doğruluk ölçümleri → CSV kolonları sözlüğü.

    pose        : pose_detector çıktısı {cx, cy, conf, kpts: 6×(u,v,conf), bbox}
    iris_truth  : gz_truth.get_iris()  — {x,y,z,roll,pitch,yaw,stamp}
    hedef_truth : gz_truth.get_hedef() — aynı biçim

    Herhangi biri None ise boş sözlük döner (CSV'de kolonlar boş kalır).
    Hesaplanamayan tek tek alanlar da atlanır — kısmi ölçüm tam ölçümden iyidir.
    """
    if pose is None or iris_truth is None or hedef_truth is None:
        return {}

    out = {}
    iris_pos = (iris_truth["x"], iris_truth["y"], iris_truth["z"])
    iris_rpy = (iris_truth["roll"], iris_truth["pitch"], iris_truth["yaw"])
    hedef_pos = (hedef_truth["x"], hedef_truth["y"], hedef_truth["z"])
    hedef_rpy = (hedef_truth["roll"], hedef_truth["pitch"], hedef_truth["yaw"])

    # ── Gerçek menzil (çerçeveden bağımsız) + hedef tutumu ──
    menzil = math.sqrt(sum((h - i) ** 2 for h, i in zip(hedef_pos, iris_pos)))
    out["menzil_gercek_gz_m"] = round(menzil, 3)
    out["hedef_roll_gercek"] = round(math.degrees(hedef_rpy[0]), 2)
    out["hedef_pitch_gercek"] = round(math.degrees(hedef_rpy[1]), 2)
    out["hedef_yaw_gercek"] = round(math.degrees(hedef_rpy[2]), 2)
    # Referansın kendi tazeliği: pose karesi ile truth örneği arasındaki sim-saat
    # farkı. Büyükse (>~30 ms) hata ölçümüne hareket bulanıklığı karışıyor demektir.
    out["truth_dt_s"] = round(abs(hedef_truth.get("stamp", 0.0)
                                  - iris_truth.get("stamp", 0.0)), 4)

    # ── Aspect açısı: hedefin burun ekseni ile hedef→iris LOS arası ──
    # yandanlik ≈ |sin(aspect)| BEKLENİR:
    #   aspect   0° → burun bize dönük (head-on)  → gövde kısalmış → yandanlik 0
    #   aspect  90° → tam yandan (broadside)      → gövde tam boy  → yandanlik 1
    #   aspect 180° → kuyruktan takip (tail chase)→ gövde kısalmış → yandanlik 0
    # Bu satır, "GPS fazı görsel fazı kör noktada mı devrediyor" sorusunun ölçüsü.
    R_h = geo.rot_rpy(*hedef_rpy)
    ileri = R_h @ np.array([1.0, 0.0, 0.0])          # hedefin burun yönü (dünya)
    los = np.array(iris_pos) - np.array(hedef_pos)   # hedef → iris
    n_los = np.linalg.norm(los)
    if n_los > 1e-6:
        kosinus = float(np.dot(ileri, los / n_los))
        aspect = math.acos(max(-1.0, min(1.0, kosinus)))
        out["aspect_gercek_deg"] = round(math.degrees(aspect), 2)
        out["sin_aspect_gercek"] = round(abs(math.sin(aspect)), 4)

    # ── Gerçek keypoint projeksiyonu (ham; örtülme ayrı bayrak) ──
    try:
        kg, onde, kadrajda, ortulu = _gercek_kpts(
            hedef_pos, hedef_rpy, iris_pos, iris_rpy)
    except Exception:
        return out                                    # projeksiyon yapılamadı
    out["kpt_kadrajda_sayi"] = int((onde & kadrajda).sum())
    out["kpt_ortulu_sayi"] = int((onde & ortulu).sum())

    kpts = pose["kpts"]
    # Keypoint başına piksel hatası. Ölçüt yalnız "kameranın ÖNÜNDE mi" —
    # kadraj dışı/örtülü noktalar da ölçülür (model onlara da konum atıyor),
    # ayrıştırma yukarıdaki sayaçlarla analizde yapılır.
    hatalar = []
    for i, ad in enumerate(geo.KEYPOINT_NAMES):
        if not onde[i]:
            continue
        h = math.hypot(kpts[i][0] - kg[i, 0], kpts[i][1] - kg[i, 1])
        out[f"kpt_hata_px_{ad}"] = round(h, 2)
        hatalar.append(h)
    if hatalar:
        out["kpt_hata_px_ort"] = round(sum(hatalar) / len(hatalar), 2)
        out["kpt_hata_px_max"] = round(max(hatalar), 2)

    # ── Gerçek ölçek/yandanlık: guidance_core ile BİREBİR aynı formül ──
    if onde[_I_BURUN] and onde[_I_KUYRUK]:
        a_g = float(np.hypot(kg[_I_BURUN, 0] - kg[_I_KUYRUK, 0],
                             kg[_I_BURUN, 1] - kg[_I_KUYRUK, 1]))
        out["a_gercek"] = round(a_g, 2)
    else:
        a_g = None
    if onde[_I_SOLK] and onde[_I_SAGK]:
        b_g = float(np.hypot(kg[_I_SOLK, 0] - kg[_I_SAGK, 0],
                             kg[_I_SOLK, 1] - kg[_I_SAGK, 1]))
        out["b_gercek"] = round(b_g, 2)
    else:
        b_g = None
    if a_g is not None and b_g is not None:
        olcek_ham_g = math.sqrt(a_g * a_g + (_GOVDE_KANAT_ORANI * b_g) ** 2)
        # ── Yükselti düzeltmesi — guidance_core.yukselti_duzeltme ile AYNI ──
        # Zorunlu: `olcek_ham = √(a² + (0.633b)²)` ancak bakış yönü gövde-kanat
        # DÜZLEMİNDE ise FX·0.81/R'ye eşittir. Avcı hedefin altından baktığı için
        # (LOS yükselişi eps) bakış o düzlemin dışına çıkar, ölçek şişer, menzil
        # olduğundan KISA görünür. Düzeltme uygulanmazsa referans, güdümün
        # kullandığı `menzil_kestirim_m` ile karşılaştırılamaz hale gelir.
        # Doğrulama (2026-07-27, sentetik): eps=25° → düzeltme 1.086;
        # düzeltmesiz 20 m → 18.4 m okunuyordu, düzeltmeyle 19.96 m.
        cam_pos, _ = geo.camera_world_pose(iris_pos, iris_rpy)
        los_cam = np.asarray(hedef_pos, float) - cam_pos
        n_cam = float(np.linalg.norm(los_cam))
        eps_g = math.asin(max(-1.0, min(1.0, los_cam[2] / n_cam))) if n_cam > 1e-6 else 0.0
        duzeltme_g = math.sqrt(1.0 + math.sin(eps_g) ** 2)
        olcek_g = olcek_ham_g / duzeltme_g
        out["eps_gercek_deg"] = round(math.degrees(eps_g), 2)
        out["olcek_gercek"] = round(olcek_g, 2)
        if olcek_g > 1e-9:
            # yandanlik: guidance_core'daki gibi DÜZELTİLMİŞ ölçeğe bölünür ve
            # aynı şekilde [0,1]'e sınırlanır (yoksa model tarafı sınırlı,
            # referans tarafı sınırsız olur → karşılaştırma yanıltır).
            out["yandanlik_gercek"] = round(min(a_g / olcek_g, 1.0), 4)
            # Ölçekten türeyen menzilin GERÇEK keypoint'lerle hali. Üç değerin
            # farkı üç ayrı hatayı ayırır:
            #   menzil_kestirim_m  vs bu   → POSE MODELİNİN hatası
            #   bu  vs menzil_gercek_gz_m  → FORMÜLÜN kendi hatası
            out["menzil_olcek_gercek_m"] = round(geo.FX * 0.81 / olcek_g, 2)

    # ── Eksen açısı: model vs gerçek (yönelim doğruluğunun asıl ölçüsü) ──
    if a_g is not None:
        aci_g = _eksen_aci_deg(kg[_I_BURUN], kg[_I_KUYRUK])
        aci_m = _eksen_aci_deg(kpts[_I_BURUN], kpts[_I_KUYRUK])
        if aci_g is not None:
            out["eksen_aci_gercek_deg"] = round(aci_g, 2)
        if aci_m is not None:
            out["eksen_aci_deg"] = round(aci_m, 2)
        fark = _aci_farki_deg(aci_m, aci_g)
        if fark is not None:
            out["eksen_aci_hata_deg"] = round(fark, 2)

        # ── 180° BELİRSİZLİĞİ: burun/kuyruk takas testi ──
        # Modelin "burun"u gerçek buruna mı yoksa gerçek kuyruğa mı daha yakın?
        # Takas olursa lead TAM TERS yöne kayar; guidance_core'un flip koruması
        # yalnız 0.2 s içindeki ANİ takası yakalar, KALICI yanlış etiketlemeyi
        # göremez. Bu kolon o kör noktayı ölçer.
        if onde[_I_BURUN] and onde[_I_KUYRUK]:
            d_burun = math.hypot(kpts[_I_BURUN][0] - kg[_I_BURUN, 0],
                                 kpts[_I_BURUN][1] - kg[_I_BURUN, 1])
            d_capraz = math.hypot(kpts[_I_BURUN][0] - kg[_I_KUYRUK, 0],
                                  kpts[_I_BURUN][1] - kg[_I_KUYRUK, 1])
            out["burun_kuyruk_takas"] = int(d_capraz < d_burun)

    # ── bbox merkezi hatası (saf takip yönünün u'sunu doğrudan belirler) ──
    bb_g = None
    try:
        bb_g = geo.target_bbox(hedef_pos, hedef_rpy, iris_pos, iris_rpy)
    except Exception:
        pass
    if bb_g is not None:
        gcx, gcy = (bb_g[0] + bb_g[2]) / 2.0, (bb_g[1] + bb_g[3]) / 2.0
        out["bbox_merkez_hata_px"] = round(
            math.hypot(pose["cx"] - gcx, pose["cy"] - gcy), 2)
        out["bbox_genislik_gercek"] = round(bb_g[2] - bb_g[0], 1)
        out["bbox_yukseklik_gercek"] = round(bb_g[3] - bb_g[1], 1)

    return out


# CSV başlığı için sabit kolon listesi — `olc()` çıktısı kısmi olabilir, ama
# DictWriter sabit alan listesi ister. Sıra okunabilirlik içindir.
KOLONLAR = (
    ["menzil_gercek_gz_m", "menzil_olcek_gercek_m", "truth_dt_s",
     "aspect_gercek_deg", "sin_aspect_gercek",
     "a_gercek", "b_gercek", "olcek_gercek", "yandanlik_gercek", "eps_gercek_deg",
     "eksen_aci_deg", "eksen_aci_gercek_deg", "eksen_aci_hata_deg",
     "burun_kuyruk_takas", "kpt_kadrajda_sayi", "kpt_ortulu_sayi",
     "kpt_hata_px_ort", "kpt_hata_px_max"]
    + [f"kpt_hata_px_{ad}" for ad in geo.KEYPOINT_NAMES]
    + ["bbox_merkez_hata_px", "bbox_genislik_gercek", "bbox_yukseklik_gercek",
       "hedef_roll_gercek", "hedef_pitch_gercek", "hedef_yaw_gercek"]
)
