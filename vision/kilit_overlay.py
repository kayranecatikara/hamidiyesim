"""vision/kilit_overlay.py — Kilitlenme overlay çizici (Adım 8).

SALT-OKUR tüketici. Karar mantığı YOK (kilitlenme/kilit_sure/kilit_kriteri
ellenmez). Mevcut cv2 overlay hattına EK olarak çağrılır.

Çizilen: yalnız sağ-altta t_sim ve WxH.

NOT: SARI AV çerçevesi ve KIRMIZI kilit dörtgeni artık burada (cv2/sunucu)
DEĞİL, TARAYICI SVG katmanında (#avOverlay) çiziliyor — video elemanına kilitli,
onunla birlikte hareket/ölçek. Çift çizim olmasın diye buradaki cv2 kutu çizimi
KALDIRILDI. Sol-üst gri HUD metin bloğu da (daha önce) UI paneline taşındı.
"""

import cv2

_BEYAZ = (255, 255, 255)
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def ciz(img, durum, t_sim=None):
    """Kareye yalnız t_sim + WxH etiketini çizer ve img'i döndürür (yerinde).

    durum parametresi geriye dönük uyum için korunur (artık kutu çizilmiyor;
    kutular tarayıcı SVG katmanında kilit durumundan çiziliyor)."""
    H, W = img.shape[:2]
    alt = f"{W}x{H}"
    if t_sim is not None:
        alt = f"t={t_sim:.2f}  {alt}"
    (tw, th), _ = cv2.getTextSize(alt, _FONT, 0.45, 1)
    cv2.putText(img, alt, (W - tw - 8, H - 8), _FONT, 0.45, _BEYAZ, 1, cv2.LINE_AA)
    return img
