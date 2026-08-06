"""Kilitlenme görevi — oransal sabitler ve W,H'den türetme yardımcıları.

Piksel sabiti YOK. Tüm eşikler oransaldır; piksel değerleri yalnızca
çalışma anında kaynak karenin W,H değerlerinden türetilir.
Bkz. CLAUDE.md — Geometri (Şekil 2 + 6.1.4) ve Çözünürlük kuralı.
"""

# --- AV (hedef vuruş alanı, SARI) oransal sınırları ---
AV_YATAY = (0.25, 0.75)    # yatay [0.25W, 0.75W]
AV_DIKEY = (0.10, 0.90)    # dikey [0.10H, 0.90H]

# --- Kilit boyut şartı (alan oranı DEĞİL; eksenlerden en az biri) ---
ESIK_RESMI = 0.05          # şartname resmi eşiği: AH_en >= 0.05·W VEYA AH_boy >= 0.05·H
ESIK_BILDIRIM = 0.06       # iç karar/bildirim eşiği (min_boyut bunu tabanlar)
KARE_TOLERANS_ORAN = 0.05  # kilit segmentindeki kısa boşluk bütçesi (bildirilen sürenin %5'i)

# --- Kilit modu ---
KILIT_MODU = "merkez"      # AV içindelik: bbox merkezi (bbox'ın tamamı değil)

# --- Zamanlama (saniye) ---
DOGRULAMA_SN = 0.5         # çok-kareli tespit doğrulama penceresi (6.1.1)
TESPIT_TUTARLILIK_ORAN = 0.6  # doğrulama penceresinde gereken tespitli-süre oranı
PENCERE_SN = 10.0          # kayan değerlendirme penceresi
KUMULATIF_SN = 5.0         # pencere içi kümülatif kilit eşiği
KESINTISIZ_SN = 3.0        # çarpışma öncesi kesintisiz angajman (6.1.3)
ORNEK_TAVAN_SN = 0.2       # iki kilit örneği arası dt bunu aşarsa aralık kilitli
                           # SAYILMAZ (donma/atlama koruması; 200 ms kare toleransı
                           # ile hizalı, nominal kare ~35 ms olduğundan tetiklemez)

# --- Diğer ---
MARJ_REF = 1.45
RESET_POLICY = "kumulatif_korunur"

# --- Marj geri beslemeli mesafe tutucu (Adım 7; koşuda kalibre edilecek) ---
MARJ_UST = 1.75            # marj bandı üst sınırı (bant: MARJ_REF .. MARJ_UST)
MENZIL_REF_ALT = 6.0       # m; RANGE_SET alt sınırı (clamp)
MENZIL_REF_UST = 20.0      # m; RANGE_SET üst sınırı (clamp)
MENZIL_ADIM_M = 0.5        # m/tık; tek adımda menzil referansı değişim sınırı
TUTUCU_HZ = 1.5            # dış döngü frekansı (1-2 Hz)


def av_sinirlari(W, H):
    """AV dörtgeninin piksel sınırlarını (x_min, x_max, y_min, y_max) döndürür."""
    x_min = AV_YATAY[0] * W
    x_max = AV_YATAY[1] * W
    y_min = AV_DIKEY[0] * H
    y_max = AV_DIKEY[1] * H
    return (x_min, x_max, y_min, y_max)


def min_boyut(W, H):
    """Kilit için gereken minimum AH boyutunu (min_en, min_boy) piksel döndürür.

    ESIK_BILDIRIM tabanlıdır (histerezis için resmi eşiğin biraz üstü).
    """
    min_en = ESIK_BILDIRIM * W
    min_boy = ESIK_BILDIRIM * H
    return (min_en, min_boy)
