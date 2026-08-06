"""
vision/krop.py — Pose için ORTAK krop mantığı (eğitim ve çıkarım aynı kodu kullanır).

Neden ayrı modül: pose modeli 35 m'de 17×10 pikselllik bir hedefi çözmek zorunda.
Tam karede (640×480) bu nesne kadrajın binde biri; model kapasitesinin neredeyse
tamamını boş göğe harcıyor. Onun yerine detection'ın bulduğu kutunun etrafından
krop alınıp sabit boyuta büyütülür — hedef kadrajı doldurur.

★ EN ÖNEMLİ KURAL: veri seti bu fonksiyonla üretilir, çıkarım da bu fonksiyonu
çağırır. İkisi ayrı hesap yaparsa model eğitimde gördüğünden farklı bir görüntüyle
karşılaşır ve HATA VERMEDEN kötü çalışır. (2026-07-30'da detection tarafında tam
bu ortaya çıktı: model 1280'de eğitilmişti, sistem 640'ta çalıştırıyordu.)
"""

import cv2
import numpy as np

# Kutunun kaç katı alan kropa girecek. 1.0 = tam kutu (kanat uçları kenara
# dayanır), 2.5 = hedefin etrafında nefes payı. Çıkarımda detection kutusu
# birkaç piksel kayabildiği için pay şart.
#
# ★ 2026-08-01 ÖLÇÜMÜ — asıl sorun marj değil, KROPUN KAYNAĞI:
# Veri seti GERÇEK (geometry) kutudan kroplanıyor, uçuşta ise DETECTION
# kutusundan. Detection küçük hedefte kutuyu şişiriyor (38.9 m'de gerçek
# 5.5 px yerine 13.5 px — 2.46×; 13 m'de yalnız 1.14×), krop penceresi de
# o oranda genişliyor ve hedef krop içinde küçülüyor:
#     eğitim (val) 79 px   |   uçuş 31.5 px, üstelik merkezden 17 px kaymış
# Modelin keypoint hatası hedef boyutuna sert bağlı (val ölçümü, krop px):
#     79 px → 2.27    55 px → 2.07    44 px → 2.14    32 px → 5.00
# Sonuç: model val'de 2.2 px hata yaparken uçuşta 14-28 px yapıyor.
#
# Marj 1.5 denendi (hedef 43.8 px): mesafe hatası 20.9 → 14.5 m düzeldi ama
# rotasyon düzelmedi (yaw 7.4° → 9.6° ham, pitch 5.3° → 9.4°) — çünkü model
# 2.5 marjla eğitildi, marjı değiştirmek onu dağılımdan bir kez daha
# uzaklaştırıyor. Şişme oranı mesafeye göre değiştiği için sabit bir marjla
# da kapatılamaz.
# DOĞRU ÇÖZÜM: veri setini DETECTION kutusundan kroplayarak üretmek — o
# zaman eğitim ile çıkarım aynı kutuyu görür. AVCI_KROP_MARGIN ile denenebilir.
import os as _os
KROP_MARGIN = float(_os.environ.get("AVCI_KROP_MARGIN", "2.5"))
# Çıktı kare boyutu. YOLO stride'ı 32; 192 = 6×32. 35 m'de kutu ~17 px →
# krop ~42 px → 192'ye 4.5× büyütme.
KROP_BOYUT = 192
# Çok küçük kroplarda upscale gürültüyü büyütür; taban koyuyoruz.
MIN_KROP_PX = 24.0


def krop_penceresi(bb):
    """bbox (x1,y1,x2,y2) → kare krop penceresi (kx1, ky1, kenar).

    Kenar daima kare; hedefin merkezine oturur. Kadraj dışına taşabilir —
    taşan kısım krop_al() içinde kenar tekrarıyla doldurulur.
    """
    x1, y1, x2, y2 = bb
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    kenar = max(x2 - x1, y2 - y1) * KROP_MARGIN
    kenar = max(kenar, MIN_KROP_PX)
    return cx - kenar / 2.0, cy - kenar / 2.0, kenar


def krop_al(frame, bb):
    """Kareden hedefin etrafını kırpıp KROP_BOYUT×KROP_BOYUT'a getirir.

    Döndürür: (krop_goruntu, kx1, ky1, olcek)
      olcek = KROP_BOYUT / kenar  → piksel dönüşümü için
    """
    kx1, ky1, kenar = krop_penceresi(bb)
    h, w = frame.shape[:2]
    x1i, y1i = int(np.floor(kx1)), int(np.floor(ky1))
    x2i, y2i = int(np.ceil(kx1 + kenar)), int(np.ceil(ky1 + kenar))

    # Kadraj dışına taşan kısmı kenar tekrarıyla doldur (siyah dolgu modele
    # sahte kenar/kontrast öğretir; replicate daha nötr).
    sol, ust = max(0, -x1i), max(0, -y1i)
    sag, alt = max(0, x2i - w), max(0, y2i - h)
    kesit = frame[max(0, y1i):min(h, y2i), max(0, x1i):min(w, x2i)]
    if kesit.size == 0:
        return None, kx1, ky1, 1.0
    if sol or sag or ust or alt:
        kesit = cv2.copyMakeBorder(kesit, ust, alt, sol, sag, cv2.BORDER_REPLICATE)

    krop = cv2.resize(kesit, (KROP_BOYUT, KROP_BOYUT), interpolation=cv2.INTER_LINEAR)
    return krop, float(x1i), float(y1i), KROP_BOYUT / float(kesit.shape[1])


def noktalari_kropa_tasi(noktalar, kx1, ky1, olcek):
    """Tam kare piksel koordinatlarını krop piksel koordinatlarına çevirir."""
    out = []
    for u, v in noktalar:
        out.append(((u - kx1) * olcek, (v - ky1) * olcek))
    return out


def noktalari_kareye_dondur(noktalar, kx1, ky1, olcek):
    """Krop koordinatlarını tam kare koordinatlarına geri çevirir (çıkarım)."""
    out = []
    for u, v in noktalar:
        out.append((u / olcek + kx1, v / olcek + ky1))
    return out
