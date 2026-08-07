"""vision/kilit_overlay.py — Kilitlenme overlay çizici (Adım 8).

SALT-OKUR tüketici. Karar mantığı YOK (kilitlenme/kilit_sure/kilit_kriteri
ellenmez). Mevcut cv2 overlay hattına EK olarak çağrılır.

Çizilen: sağ-altta t_sim + WxH, VE KIRMIZI kilitlenme dörtgeni (AH) anlık kilit
varken (6.1.4: #FF0000, kalınlık ≤ 3 px).

── 2026-08-07: KIRMIZI DÖRTGEN SUNUCUYA (cv2) GERİ ALINDI ──
Kutu tarayıcı SVG katmanındayken (500-700 ms poll) 30 Hz videoya göre GERİDEN
zıplıyordu (yeşil YOLO kutusu videoya gömülü olduğu için titremezken kırmızı
titriyordu). Videoya gömülü çizim 30 Hz senkron → titreme yok, üstelik 6.1.4
hakem videosu için doğru (kilit dörtgeni gerçek videoda olmalı). SVG'deki
lockRect kaldırıldı (çift çizim olmasın); SARI AV hâlâ SVG'de (sabit kutu,
titremez). Kutu, KilitTakip'e beslenen YUMUŞATILMIŞ ah_kutu'dur (tek kaynak, #3).
"""

import cv2

from config.kilit_sabitler import SARTNAME

_BEYAZ = (255, 255, 255)
_KIRMIZI_BGR = (0, 0, 255)          # #FF0000 (6.1.4 DORTGEN_RENK) → BGR
_FONT = cv2.FONT_HERSHEY_SIMPLEX
_KALINLIK = max(1, min(3, int(SARTNAME.CIZGI_KALINLIK_MAKS_PX)))


def ciz(img, durum, t_sim=None):
    """Kareye kırmızı kilitlenme dörtgenini (anlık kilit varken) + t_sim/WxH
    etiketini çizer, img'i döndürür (yerinde). durum: KilitTakip.guncelle çıktısı."""
    H, W = img.shape[:2]
    # ── KIRMIZI AH dörtgeni: yalnız anlık kilit varken (SVG ile aynı koşul) ──
    if durum and durum.get("anlik_kilit") and durum.get("ah_kutu"):
        x1, y1, x2, y2 = (int(v) for v in durum["ah_kutu"])
        cv2.rectangle(img, (x1, y1), (x2, y2), _KIRMIZI_BGR, _KALINLIK)
    alt = f"{W}x{H}"
    if t_sim is not None:
        alt = f"t={t_sim:.2f}  {alt}"
    (tw, th), _ = cv2.getTextSize(alt, _FONT, 0.45, 1)
    cv2.putText(img, alt, (W - tw - 8, H - 8), _FONT, 0.45, _BEYAZ, 1, cv2.LINE_AA)
    return img
