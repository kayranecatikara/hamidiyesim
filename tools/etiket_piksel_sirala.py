#!/usr/bin/env python3
"""
tools/etiket_piksel_sirala.py — Simetrik keypoint çiftlerini PİKSEL sırasına çevirir.

SORUN (2026-07-31 ölçümü, 431 val karesi):
  Pose modeli karelerin %99.8'inde iki kanadı da gövde merkezine koyuyor.
  Hata 39.9 px — tam olarak kanat açıklığının yarısı. Çapraz eşleştirme
  hatayı hiç düşürmüyor (39.9 → 39.9), yani bu bir SIRA karışıklığı değil.
  Model "ortalamaya kaçıyor": hangi kanadın sol olduğunu ayırt edemediği
  için L2 kaybı altında en güvenli tahmin iki ihtimalin ortası oluyor.
  Aynı sebeple fliplr=0 eğitimi de bir şey değiştirmedi (oran 0.05 → 0.05).

ÇÖZÜM:
  Modelden "sol kanat / sağ kanat" istemeyi bırak. Bunlar 3B anlamdır ve
  192 px'lik kropta uzaktan ayırt edilemez. Yerine "GÖRÜNTÜDE solda olan /
  sağda olan" iste — bu her karede piksel x'ine bakarak kesin belirlenir,
  belirsizlik kalmaz, ortalamaya kaçacak bir şey olmaz.

  Aynı düzeltme V-tail çiftine de uygulanır.

  PnP tarafı (vision/poz_tahmin.py) buna karşılık iki permütasyonu dener ve
  yeniden-izdüşüm hatası düşük olanı seçer — 3B anlam orada geri kazanılır.

flip_idx [0,1,3,2,5,4] bu düzenle uyumludur: aynalamadan sonra soldaki nokta
sağa geçer, flip_idx takası onu yine doğru indekse koyar. Bu yüzden fliplr
augmentation'ı açık bırakmak güvenlidir (ve veri çeşitliliğine faydalıdır).

Görüntüler symlink'lenir — disk kopyası yok.
"""

import os
import shutil
import sys

KAYNAK = "/home/zeylo/projects/avci_sim/vision/datasets/talon_pose_krop"
HEDEF = "/home/zeylo/projects/avci_sim/vision/datasets/talon_pose_sirali"

# Piksel x'ine göre sıralanacak simetrik çiftler (indeks)
CIFTLER = [(2, 3), (4, 5)]          # kanatlar, V-tail'ler


def satir_donustur(satir):
    """Bir YOLO-pose etiket satırında simetrik çiftleri piksel x'ine göre sıralar."""
    p = satir.split()
    if len(p) < 5 + 6 * 3:
        return satir, False
    bas, kp = p[:5], p[5:5 + 18]
    nokta = [kp[i * 3:i * 3 + 3] for i in range(6)]
    degisti = False
    for a, b in CIFTLER:
        xa, ya = float(nokta[a][0]), float(nokta[a][1])
        xb, yb = float(nokta[b][0]), float(nokta[b][1])
        # soldaki (küçük x) önce; x eşitse yukarıdaki (küçük y) önce
        if (xa, ya) > (xb, yb):
            nokta[a], nokta[b] = nokta[b], nokta[a]
            degisti = True
    yeni = bas + [v for n in nokta for v in n] + p[5 + 18:]
    return " ".join(yeni), degisti


def main():
    if not os.path.isdir(KAYNAK):
        sys.exit(f"kaynak yok: {KAYNAK}")
    if os.path.exists(HEDEF):
        shutil.rmtree(HEDEF)

    toplam = takas = bos = 0
    for bolum in ("train", "val"):
        g_kay = os.path.join(KAYNAK, "images", bolum)
        e_kay = os.path.join(KAYNAK, "labels", bolum)
        g_hed = os.path.join(HEDEF, "images", bolum)
        e_hed = os.path.join(HEDEF, "labels", bolum)
        os.makedirs(g_hed, exist_ok=True)
        os.makedirs(e_hed, exist_ok=True)

        for ad in os.listdir(g_kay):
            os.symlink(os.path.join(g_kay, ad), os.path.join(g_hed, ad))

        for ad in os.listdir(e_kay):
            satirlar = []
            with open(os.path.join(e_kay, ad)) as f:
                for s in f:
                    s = s.strip()
                    if not s:
                        continue
                    yeni, d = satir_donustur(s)
                    satirlar.append(yeni)
                    toplam += 1
                    takas += int(d)
            if not satirlar:
                bos += 1
            with open(os.path.join(e_hed, ad), "w") as f:
                f.write("\n".join(satirlar) + ("\n" if satirlar else ""))

    with open(os.path.join(HEDEF, "dataset.yaml"), "w") as f:
        f.write(f"""path: {HEDEF}
train: images/train
val: images/val
kpt_shape: [6, 3]
# 0 burun, 1 kuyruk, 2 SOLDAKİ kanat, 3 SAĞDAKİ kanat,
# 4 SOLDAKİ V-tail, 5 SAĞDAKİ V-tail   (görüntü pikselinde, 3B anlamda değil)
flip_idx: [0, 1, 3, 2, 5, 4]
nc: 1
names:
  0: talon
""")
    print(f"  {toplam} etiket satırı işlendi")
    print(f"  {takas} satırda çift takaslandı (%{100*takas/max(toplam,1):.1f})")
    if bos:
        print(f"  {bos} boş etiket dosyası")
    print(f"  → {HEDEF}")


if __name__ == "__main__":
    main()
