"""
vision/capture_pose_dataset.py — Gazebo statik world'de OTOMATİK ETİKETLİ YOLO-POSE verisi.

Detection pipeline'ıyla (capture_dataset.py) aynı mekanik: kamera + hedef her karede
rastgele konumlanır; pozlar bilindiğinden bbox VE 6 keypoint (burun, kuyruk, sol/sağ
kanat ucu, sol/sağ V-tail ucu) projeksiyonla otomatik etiketlenir. Manuel etiket yok.

Etiket formatı (YOLO-pose): "0 cx cy w h  x1 y1 v1 ... x6 y6 v6" (normalize).
v=2 görünür, v=0 kadraj dışı/arkada (x=y=0).

Kullanım:
  # 1) Ayrı terminalde statik world (detection'la AYNI world):
  export GZ_SIM_RESOURCE_PATH=$HOME/projects/avci_sim/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models
  gz sim -r sim/gazebo_harmonic/worlds/dataset_capture.sdf
  # 2) Veri topla:
  python3 -m vision.capture_pose_dataset --count 5000 --debug-overlay
"""

import argparse
import os
import random
import time

import cv2

from gz.transport13 import Node

from vision import geometry as geo
from vision import krop as _krop
from vision.capture_dataset import (
    CAMERA, CAM_TOPIC, TARGET,
    FrameGrabber, _set_pose, random_camera_pose, sample_target_pose,
)

# Keypoint pikselden okunabilsin diye bbox alt sınırı detection'dan büyük
MIN_POSE_BOX_PX = 16.0

# Debug overlay: keypoint renkleri (BGR) + iskelet çizgileri
_KPT_COLORS = [(0, 0, 255), (255, 0, 0), (0, 255, 0),
               (0, 255, 255), (255, 0, 255), (255, 255, 0)]
_SKELETON = [(0, 1), (2, 3), (1, 4), (1, 5)]   # gövde, kanat, V-tail'ler


def write_dataset_yaml(out_dir):
    abs_out = os.path.abspath(out_dir)
    yaml_path = os.path.join(out_dir, "dataset.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"path: {abs_out}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write(f"kpt_shape: [{len(geo.KEYPOINT_NAMES)}, 3]\n")
        f.write(f"flip_idx: {geo.KEYPOINT_FLIP_IDX}\n")
        f.write("nc: 1\n")
        f.write("names:\n  0: talon\n")
    return yaml_path


def kpts_to_yolo(kpts):
    """(6,3) piksel [u,v,vis] → normalize YOLO-pose string parçası."""
    parts = []
    for u, v, vis in kpts:
        if vis < 1:
            parts.append("0 0 0")
        else:
            parts.append(f"{u / geo.IMG_W:.6f} {v / geo.IMG_H:.6f} {int(vis)}")
    return " ".join(parts)


def draw_debug(frame, bb, kpts):
    dbg = frame.copy()
    cv2.rectangle(dbg, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), (0, 255, 0), 1)
    for a, b in _SKELETON:
        if kpts[a][2] >= 1 and kpts[b][2] >= 1:
            cv2.line(dbg, (int(kpts[a][0]), int(kpts[a][1])),
                     (int(kpts[b][0]), int(kpts[b][1])), (200, 200, 200), 1)
    for i, (u, v, vis) in enumerate(kpts):
        if vis >= 1:
            cv2.circle(dbg, (int(u), int(v)), 3, _KPT_COLORS[i], -1)
            cv2.putText(dbg, geo.KEYPOINT_NAMES[i], (int(u) + 4, int(v) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, _KPT_COLORS[i], 1)
    return dbg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=5000, help="hedef örnek sayısı")
    ap.add_argument("--out", default="vision/datasets/talon_pose")
    ap.add_argument("--val-split", type=float, default=0.15)
    ap.add_argument("--settle", type=float, default=0.20,
                    help="set_pose sonrası render bekleme (s)")
    ap.add_argument("--min-kpts", type=int, default=3,
                    help="karede en az bu kadar keypoint görünür olmalı")
    ap.add_argument("--dist-min", type=float, default=None,
                    help="hedef mesafe alt sınırı m (varsayılan capture_dataset: 3)")
    ap.add_argument("--dist-max", type=float, default=None,
                    help="hedef mesafe üst sınırı m (varsayılan 15)")
    ap.add_argument("--dist-exp", type=float, default=None,
                    help="mesafe dağılım üssü: 2=yakın ağırlıklı (vars.), 1=düzgün")
    ap.add_argument("--min-box", type=float, default=MIN_POSE_BOX_PX,
                    help=f"bbox alt sınırı px (varsayılan {MIN_POSE_BOX_PX}; "
                         f"uzak mesafe partisinde düşür)")
    ap.add_argument("--tam-kare", action="store_true",
                    help="krop yerine tam 640x480 kare kaydet (eski davranış)")
    ap.add_argument("--gercek-kutu-krop", action="store_true",
                    help="krop penceresini geometry kutusundan al (ESKİ, önerilmez — "
                         "uçuşta krop DETECTION kutusundan alınır, bkz. modül başlığı)")
    ap.add_argument("--debug-overlay", action="store_true",
                    help="ilk 20 karede bbox+keypoint'leri çizip ayrı kaydet")
    args = ap.parse_args()

    # Mesafe ayarlarını örnekleyicinin (capture_dataset) modül sabitlerine uygula
    import vision.capture_dataset as _cd
    if args.dist_min is not None:
        _cd.DIST_MIN = args.dist_min
    if args.dist_max is not None:
        _cd.DIST_MAX = args.dist_max
    if args.dist_exp is not None:
        _cd.DIST_EXP = args.dist_exp
    print(f"[POSE] Mesafe: {_cd.DIST_MIN}-{_cd.DIST_MAX} m (üs {_cd.DIST_EXP}), "
          f"min kutu {args.min_box}px")

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)
    if args.debug_overlay:
        os.makedirs(os.path.join(args.out, "debug"), exist_ok=True)

    # ── KROP PENCERESİ NEREDEN GELİR (2026-08-01 ölçümü) ─────────────
    # Eskiden krop, geometry'nin GERÇEK kutusundan kesiliyordu. Uçuşta ise
    # pose_detector kropu DETECTION kutusundan keser ve detection küçük
    # hedefte kutuyu şişirir (38.9 m'de gerçek 5.5 px yerine 13.5 px —
    # 2.46×; 13 m'de yalnız 1.14×). Sonuç: hedef krop içinde eğitimdekinin
    # yarısı kalıyordu (79 px → 31.5 px) ve üstelik merkezden 17 px kayıyordu.
    # Modelin keypoint hatası bu boyuta sert bağlı (val, krop px):
    #     79 px → 2.27    55 px → 2.07    44 px → 2.14    32 px → 5.00
    # Ölçülen sonuç: model val'de 2.2 px hata yaparken uçuşta 14-28 px.
    # Sabit bir krop marjı bunu kapatamaz, çünkü şişme oranı mesafeye bağlı.
    # ÇÖZÜM: veri seti de kropu DETECTION kutusundan alsın — eğitim ile
    # çıkarım aynı pencereyi görsün. Detection bulamazsa kare atlanır;
    # uçuşta da o karede pose zaten çalışmazdı.
    _det = None
    if not args.gercek_kutu_krop and not args.tam_kare:
        from vision import detector as _detm
        _detm.load()
        _det = _detm
        print("[POSE] Krop penceresi DETECTION kutusundan alınacak "
              "(uçuşla birebir aynı yol)")

    node = Node()
    grabber = FrameGrabber(node)

    print(f"[POSE] Kamera bekleniyor ({CAM_TOPIC})...")
    t0 = time.time()
    while grabber.snapshot()[0] is None and time.time() - t0 < 15:
        time.sleep(0.3)
    if grabber.snapshot()[0] is None:
        print("[POSE] HATA: kameradan görüntü gelmedi. Gazebo (dataset_capture.sdf) çalışıyor mu?")
        return
    print(f"[POSE] Kamera hazır. {len(geo.KEYPOINT_NAMES)} keypoint: "
          f"{', '.join(geo.KEYPOINT_NAMES)}")

    # Yarıda kesilen toplamaya kaldığı yerden devam et (isim çakışması/çift
    # etiket olmasın): mevcut en büyük indeksin bir sonrasından numaralandır.
    existing = [f for sp in ("train", "val")
                for f in os.listdir(os.path.join(args.out, "images", sp))]
    saved = 1 + max((int(f.split("_")[-1].split(".")[0]) for f in existing),
                    default=-1)
    if saved:
        print(f"[POSE] Mevcut {saved} örnek bulundu — {args.count} hedefine devam ediliyor.")

    # Krop etiketleri krop koordinatındadır; gerçek mesafe bilgisi etikette
    # kaybolur. Dağılımı sonradan doğrulayabilmek için ayrı meta kaydı tutulur.
    import math as _math
    meta = open(os.path.join(args.out, "meta.csv"), "a")
    if meta.tell() == 0:
        meta.write("ad,mesafe_m,kutu_w_px,kutu_h_px,gorunur_kpt\n")

    attempts = 0
    max_attempts = args.count * 4
    while saved < args.count and attempts < max_attempts:
        attempts += 1
        iris_pos, iris_rpy = random_camera_pose()
        tpos, trpy = sample_target_pose(iris_pos, iris_rpy)
        if tpos is None:
            continue

        # ── ELEME ÖNCE, GAZEBO'YA DOKUNMADAN (2026-07-30 hız düzeltmesi) ──
        # target_bbox ve target_keypoints saf geometridir (konum+rotasyon+FOV;
        # occluded_mask da ışın-mesh testi) — simülasyona bağlı değil.
        # Eskiden modeller ÖNCE taşınıp settle beklendikten SONRA eleniyordu;
        # pose'da eleme oranı %92 olduğu için zamanın neredeyse tamamı boşa
        # gidiyordu (ölçüm: 300 kare hedefi için 1200 deneme, 288 sn).
        bb = geo.target_bbox(tpos, trpy, iris_pos, iris_rpy)
        if bb is None:
            continue
        if (bb[2] - bb[0]) < args.min_box or (bb[3] - bb[1]) < args.min_box:
            continue
        kpts = geo.target_keypoints(tpos, trpy, iris_pos, iris_rpy)
        if (kpts[:, 2] >= 1).sum() < args.min_kpts:
            continue

        # Yalnızca KABUL EDİLEN aday için modelleri taşı + render bekle
        if not _set_pose(node, CAMERA, iris_pos, iris_rpy):
            time.sleep(0.02); continue
        if not _set_pose(node, TARGET, tpos, trpy):
            time.sleep(0.02); continue
        time.sleep(args.settle)

        frame, _ = grabber.snapshot()
        if frame is None:
            continue

        split = "val" if random.random() < args.val_split else "train"
        name = f"talon_pose_{saved:05d}"

        if args.tam_kare:
            # Eski davranış: tam 640×480 kare + tam kare koordinatlı etiket
            cv2.imwrite(os.path.join(args.out, "images", split, name + ".jpg"), frame)
            cx, cy, w, h = geo.bbox_to_yolo(bb)
            etiket = f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {kpts_to_yolo(kpts)}"
        else:
            # ★ KROP MODU (varsayılan): hedefin etrafı kesilip KROP_BOYUT'a
            # büyütülür. Çıkarımda pose_detector AYNI vision.krop fonksiyonunu
            # çağırır — eğitim/çıkarım ölçeği birebir aynı olsun diye.
            # Krop penceresi: uçuşta olduğu gibi DETECTION kutusundan.
            # Etiketler yine geometry'nin gerçek keypoint'lerinden gelir —
            # değişen yalnız pencerenin nereden hesaplandığı.
            krop_bb = bb
            if _det is not None:
                _d = _det.detect_talon(frame)
                if _d is None:
                    continue            # uçuşta da pose çalışmazdı
                krop_bb = _d["bbox"]
            krop, kx1, ky1, olcek = _krop.krop_al(frame, krop_bb)
            if krop is None:
                continue
            K = _krop.KROP_BOYUT
            (bx1, by1), (bx2, by2) = _krop.noktalari_kropa_tasi(
                [(bb[0], bb[1]), (bb[2], bb[3])], kx1, ky1, olcek)
            bx1, bx2 = max(0.0, bx1), min(float(K), bx2)
            by1, by2 = max(0.0, by1), min(float(K), by2)
            if bx2 - bx1 < 2 or by2 - by1 < 2:
                continue
            kp_krop = _krop.noktalari_kropa_tasi(
                [(k[0], k[1]) for k in kpts], kx1, ky1, olcek)

            # ── SİMETRİK ÇİFTLERİ PİKSEL SIRASINA ÇEVİR ──────────────
            # "sol kanat / sağ kanat" 3B anlamdır ve küçük hedefte görsel
            # olarak ayırt edilemez; model belirsizlik altında iki ihtimalin
            # ORTASINI tahmin ediyordu (2026-07-31: karelerin %99.8'inde iki
            # kanat üst üste, hata tam açıklığın yarısı). "Görüntüde soldaki /
            # sağdaki" ise her karede kesin belirlenir. Bu düzeltmeyle val'de
            # kanat açıklığı oranı 0.05 → 1.04, V-tail 1.00, mAP50-95
            # 0.534 → 0.875, uçuşta yaw hatası 67.2° → 5.7° oldu.
            # flip_idx [0,1,3,2,5,4] bu düzenle uyumludur.
            kp_krop = list(kp_krop)
            for _a, _b in ((2, 3), (4, 5)):
                if kp_krop[_a] > kp_krop[_b]:      # (x, y) sözlük sırası
                    kp_krop[_a], kp_krop[_b] = kp_krop[_b], kp_krop[_a]
                    kpts[[_a, _b]] = kpts[[_b, _a]]

            parcalar = []
            gorunur = 0
            for (u, v), k in zip(kp_krop, kpts):
                # Krop dışına düşen nokta da görünmez sayılır — model kadrajda
                # olmayan bir noktayı tahmin etmeye çalışmasın.
                if k[2] < 1 or not (0 <= u < K and 0 <= v < K):
                    parcalar.append("0 0 0")
                else:
                    parcalar.append(f"{u / K:.6f} {v / K:.6f} 2")
                    gorunur += 1
            if gorunur < args.min_kpts:
                continue
            cv2.imwrite(os.path.join(args.out, "images", split, name + ".jpg"), krop)
            etiket = (f"0 {(bx1 + bx2) / 2 / K:.6f} {(by1 + by2) / 2 / K:.6f} "
                      f"{(bx2 - bx1) / K:.6f} {(by2 - by1) / K:.6f} "
                      + " ".join(parcalar))

        with open(os.path.join(args.out, "labels", split, name + ".txt"), "w") as f:
            f.write(etiket + "\n")
        meta.write(f"{name},{_math.dist(tpos, iris_pos):.2f},"
                   f"{bb[2]-bb[0]:.1f},{bb[3]-bb[1]:.1f},"
                   f"{int((kpts[:, 2] >= 1).sum())}\n")
        meta.flush()

        if args.debug_overlay and saved < 20:
            cv2.imwrite(os.path.join(args.out, "debug", name + "_pose.jpg"),
                        draw_debug(frame, bb, kpts))

        saved += 1
        if saved % 100 == 0:
            print(f"[POSE]   {saved}/{args.count}  (deneme {attempts})")

    yaml_path = write_dataset_yaml(args.out)
    print(f"[POSE] Bitti: {saved} örnek kaydedildi ({attempts} deneme).")
    print(f"[POSE] dataset.yaml: {yaml_path}")
    if args.debug_overlay:
        print(f"[POSE] Etiket doğrulama görselleri: {args.out}/debug/")


if __name__ == "__main__":
    main()
