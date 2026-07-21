# vision/color_detector.py
#
# Cessna hedef uçağı renk-tabanlı tespiti (ArduPilot + Gazebo sürümü).
# rc_cessna Gazebo modeli gri gövdeli (diffuse ~0.175) + koyu detaylar.
# Gökyüzü arka planına karşı gri-açık gri bir siluet oluşturur.
#
# PX4/Talon sürümünden farklar:
#   - Talon saf beyazdı (Val>=120); Cessna daha koyu gri → Val alt sınırı düştü
#   - Saturation toleransı biraz genişletildi (gri gövde + hafif renk tonu)
#   - Etiket "UCAK" → "CESSNA"
import cv2
import numpy as np

# Cessna gri gövde + gökyüzüne karşı siluet için HSV maskesi.
# Düşük saturation (grinin doğası), geniş value aralığı (koyu gri gövdeden
# render ışığıyla parlayan yüzeylere kadar). Mavi gökyüzü yüksek saturation
# olduğu için elenir.
CESSNA_HSV_LOWER = np.array([0,   0,  55])
CESSNA_HSV_UPPER = np.array([180, 45, 255])

# Geriye dönük uyumluluk (eski import'lar kırılmasın)
TALON_HSV_LOWER = CESSNA_HSV_LOWER
TALON_HSV_UPPER = CESSNA_HSV_UPPER


def detect_cessna(frame_bgr):
    """
    Cessna hedef uçağını renk/parlaklık maskesiyle tespit eder.

    Returns: dict | None
      dict = {"cx": int, "cy": int, "w": int, "h": int, "conf": float, "bbox": (x1,y1,x2,y2)}
    """
    h_frame, w_frame = frame_bgr.shape[:2]

    # 1. BİT DİLİMLEME (Gürültüyü, ışık yansımalarını ve gökyüzü pusunu azaltır)
    frame_sliced = np.bitwise_and(frame_bgr, 224)

    hsv = cv2.cvtColor(frame_sliced, cv2.COLOR_BGR2HSV)

    # 2. GRİ/AÇIK-GRİ MASKE (Cessna gövdesi + kontrol yüzeyleri)
    mask = cv2.inRange(hsv, CESSNA_HSV_LOWER, CESSNA_HSV_UPPER)

    # 3. KANATLARI VE GÖVDEYİ BÜTÜNLEŞTİRME
    kernel_dilate = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel_dilate, iterations=1)

    kernel_close = np.ones((15, 15), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    valid_contours = []
    for c in contours:
        x_c, y_c, w_c, h_c = cv2.boundingRect(c)
        area = cv2.contourArea(c)

        # FİLTRE 1: Nişangah/ufak gürültüler çok küçüktür (elenir)
        if area < 50:
            continue

        # FİLTRE 2: Ufuk çizgisi/zemin ekranın %85'inden fazlasını kaplar (elenir)
        if w_c > w_frame * 0.85:
            continue

        valid_contours.append(c)

    if not valid_contours:
        return None

    # Ekranda kalan nesneler içinden ALANI en büyük olanı seç.
    # Cessna (hedef uçak) gökyüzündeki en büyük gri kütledir.
    best_contour = max(valid_contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(best_contour)

    return {
        "cx": int(x + w / 2), "cy": int(y + h / 2),
        "w": int(w), "h": int(h),
        "conf": 1.0,
        "bbox": (x, y, x + w, y + h)
    }


# Geriye dönük uyumluluk — gcs_server ve diğer çağıranlar detect_talon bekliyor
detect_talon = detect_cessna


def draw_overlay(frame_bgr, det):
    if det is None:
        return frame_bgr

    draw_frame = frame_bgr.copy()
    x1, y1, x2, y2 = det["bbox"]

    cv2.rectangle(draw_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.putText(draw_frame, "HEDEF",
                (x1, max(y1 - 6, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return draw_frame
