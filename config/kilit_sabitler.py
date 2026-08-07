"""Kilitlenme + görev FSM — tek merkezî config.

İki blok:
  • BLOK A — ŞARTNAME (DEĞİŞTİRİLEMEZ): yarışma şartnamesinden gelen değerler.
    `frozen=True` dataclass; kod içinde override EDİLEMEZ, env okumaz. Her alanın
    yanında şartname maddesi belirtilir.
  • BLOK B — AYARLANABİLİR: şartnamede tanımlı olmayan, simülasyonda kalibre
    edilecek değerler. Env ile override edilebilir (AVCI_* değişkenleri).

Piksel sabiti YOK; tüm eşikler oransaldır, piksel değerleri çalışma anında W,H'den
türetilir (bkz. av_sinirlari / min_boyut). Bkz. CLAUDE.md — Geometri + Çözünürlük.

Geriye dönük uyumluluk: eski modül-düzeyi isimler (PENCERE_SN, KUMULATIF_SN, …)
bu blokların değerlerine ALIAS olarak korunur; mevcut importlar bozulmaz.
"""

import os
from dataclasses import dataclass


def _envf(ad, varsayilan):
    try:
        return float(os.environ.get(ad, varsayilan))
    except (TypeError, ValueError):
        return float(varsayilan)


# ══════════════════════════════════════════════════════════════════════════
#  BLOK A — ŞARTNAME (DEĞİŞTİRİLEMEZ; env override YOK)
# ══════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class SartnameSabit:
    PENCERE_SN: float = 10.0             # 6.1.2 / 6.1.4 — kayan değerlendirme penceresi
    KUMULATIF_KILIT_SN: float = 5.0      # 6.1.2 / 6.1.4 — pencere içi kümülatif kilit
    KESINTISIZ_SN: float = 3.0           # 6.1.3 — angajman öncesi son kesintisiz kilit
    AH_HEDEF_KAPSAMA_MIN: float = 0.90   # 6.1.4 — kilit dörtgeni hedefin >= %90'ı
    AH_EKRAN_ORAN_MIN: float = 0.05      # 6.1.4 — en az bir eksende >= %5
    MERKEZ_SAPMA_YATAY: float = 0.5      # 6.1.4 — hedef genişliğinin yarısı
    MERKEZ_SAPMA_DIKEY: float = 0.5      # 6.1.4 — hedef yüksekliğinin yarısı
    FRAME_TOLERANS_ORAN: float = 0.05    # 6.1.4 — %5 (5 sn kilitte 200 ms) boşluk köprüsü
    CIZGI_KALINLIK_MAKS_PX: int = 3      # 6.1.4 — kilit dörtgeni çizgi kalınlığı üst sınırı
    DORTGEN_RENK: str = "#FF0000"        # 6.1.4 — kilit dörtgeni rengi (kırmızı)
    TELEMETRI_HZ_MIN: float = 1.0        # 6.1.6 — telemetri gönderim alt frekansı
    TELEMETRI_HZ_MAKS: float = 5.0       # 6.1.6 — telemetri gönderim üst frekansı


SARTNAME = SartnameSabit()


# ══════════════════════════════════════════════════════════════════════════
#  BLOK B — AYARLANABİLİR (şartnamede tanımlı değil; simülasyonda kalibre)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class AyarSabit:
    # Tespit doğrulama: KARE değil SÜRE tabanlı olsun (fps'ten bağımsız).
    TESPIT_DOGRULAMA_SN: float = _envf("AVCI_TESPIT_DOGRULAMA_SN", 0.30)
    # Doğrulama penceresinde gereken tespitli-süre oranı (6.1.1 tutarlılık).
    TESPIT_TUTARLILIK_ORAN: float = _envf("AVCI_TESPIT_TUTARLILIK", 0.6)
    # Kilit bu süreden uzun kaybedilirse TRACK_LOST (X).
    KILIT_KAYIP_SN: float = _envf("AVCI_KILIT_KAYIP_SN", 2.0)
    # ENGAGE'de kapanma hızı tavanı (STRIKE'ta V_KAPANMA serbest).
    V_MAX_ENGAGE: float = _envf("AVCI_V_MAX_ENGAGE", 5.0)
    # AH ekran-oran histerezisi (şartname 6.1.4 tavsiyesi: paket limiti %6+).
    AH_ORAN_GIRIS: float = _envf("AVCI_AH_ORAN_GIRIS", 0.06)   # kilide giriş eşiği
    AH_ORAN_CIKIS: float = _envf("AVCI_AH_ORAN_CIKIS", 0.052)  # histerezis, sınır titremesi
    # ── Oran regülasyonu (kapalı çevrim APPROACH/TRACK_LOCK; menzil değil ORAN) ──
    # Sistem hedefin karedeki oranını ORAN_SETPOINT'e getirene dek yaklaşır,
    # ulaşınca bandı korur. Ölçüm HAM oranın EMA'sıdır (nişan kutusu coast'ta
    # tahmin üretir → mesafe kontrolü tahmine bağlanmaz). Yalnız APPROACH ve
    # TRACK_LOCK'ta geçerli; ENGAGE/STRIKE'ta mesafe kapatılır.
    ORAN_SETPOINT: float = _envf("AVCI_ORAN_SETPOINT", 0.075)   # hedef oran
    ORAN_BANT_ALT: float = _envf("AVCI_ORAN_BANT_ALT", 0.065)   # ölü bölge alt
    ORAN_BANT_UST: float = _envf("AVCI_ORAN_BANT_UST", 0.085)   # ölü bölge üst
    ORAN_EMA_TAU: float = _envf("AVCI_ORAN_EMA_TAU", 0.3)       # s; oran EMA zaman sabiti
    # BİRİNCİL emniyet: EMA oran bunu aşarsa geri çekil (tavan tek hat değil,
    # regülasyon zaten üst bantta geri çeker).
    AH_ORAN_TAVAN: float = _envf("AVCI_AH_ORAN_TAVAN", 0.10)
    # YEDEK taban: yalnız bbox KAYBOLUNCA (oran yok, menzil güvenilmez) devreye
    # girer — bbox varken emniyet oran tavanıdır, R_MIN değil.
    R_MIN_GUVENLI: float = _envf("AVCI_R_MIN_GUVENLI", 5.0)     # m
    # bbox VARKEN RANGE_SET'in inebileceği fiziksel taban (oran tavanı asıl
    # emniyet). GNSS menzili düşük okusa bile oran setpoint'e ulaşabilsin diye
    # R_MIN'den küçük; oran tavanı (0.10) daha yakına inmeyi zaten durdurur.
    RANGE_SET_MIN: float = _envf("AVCI_RANGE_SET_MIN", 2.0)     # m
    # Yaklaşma / geri-çekilme hız tavanları (geri < ileri).
    V_MAX_APPROACH: float = _envf("AVCI_V_MAX_APPROACH", 1.5)   # m/s
    V_MAX_RETREAT: float = _envf("AVCI_V_MAX_RETREAT", 1.0)     # m/s (sarmalayıcı send_velocity)
    # ── Kilitlenme dörtgeni (AH) zamansal yumuşatma — ham YOLO kutusu gürültülü,
    # kırmızı çizim ekranda titriyor. EMA konumu/boyutu yumuşatır; boyut sıçraması
    # (sahte dev kutu) reddedilir. Kaynağa uygulanır → kriter+çizim+paket TEK
    # KAYNAK kalır (#3) ve kilit daha kararlı olur (yanıp sönme azalır). ──
    KUTU_EMA_TAU: float = _envf("AVCI_KUTU_EMA_TAU", 0.15)      # s; kutu EMA zaman sabiti
    KUTU_SICRAMA_ORAN: float = _envf("AVCI_KUTU_SICRAMA", 2.5)  # boyut sıçrama reddi (kat)
    # Ölü bölge (piksel histerezisi): çizilen/beslenen kutu ancak bu kadar
    # kayınca güncellenir → hedef dururken kutu tamamen sabit (kalan sub-2px
    # gürültü kesilir). Kaynağa uygulanır (kriter+çizim aynı kutu, #3).
    KUTU_OLU_BOLGE_PX: float = _envf("AVCI_KUTU_OLU_BOLGE", 2.0)  # px
    # STRIKE kapanma hızı — yalnız STRIKE'ta aktif, mevcut değer korunur.
    V_KAPANMA: float = _envf("AVCI_IBVS_V_KAPANMA", 25.0)
    # ── Bağıl kapanma klempi (hareketli hedef): klemp MUTLAK hıza değil KAPANMA
    # (bağıl) hızına biner. v_cmd = v_hedef_kestirimi(=v_drone+Ṙ·û) + kapanma·û. ──
    V_KAPANMA_MAX_ENGAGE: float = _envf("AVCI_V_KAPANMA_MAX_ENGAGE", 4.0)  # m/s net
    # Mutlak emniyet tavanı — nihai komut her koşulda bunun altında (platform ~25).
    V_MUTLAK_MAX: float = _envf("AVCI_V_MUTLAK_MAX", 20.0)      # m/s
    # İki kilit örneği arası dt bunu aşarsa aralık kilitli SAYILMAZ (donma/atlama
    # koruması; nominal kare ~35 ms olduğundan tetiklemez).
    ORNEK_TAVAN_SN: float = _envf("AVCI_ORNEK_TAVAN_SN", 0.2)


AYAR = AyarSabit()


# ── AV (hedef vuruş alanı, SARI) oransal sınırları — Şekil 2 (6.1.4) ──
AV_YATAY = (0.25, 0.75)    # yatay [0.25W, 0.75W]
AV_DIKEY = (0.10, 0.90)    # dikey [0.10H, 0.90H]

# ── Kilit modu ──
KILIT_MODU = "merkez"      # AV içindelik: bbox merkezi (bbox'ın tamamı değil)

# ── Marj / mesafe tutucu (Adım 7; koşuda kalibre) ──
MARJ_REF = 1.45
RESET_POLICY = "kumulatif_korunur"
MARJ_UST = 1.75            # marj bandı üst sınırı
MENZIL_REF_ALT = 6.0       # m; RANGE_SET alt sınırı
MENZIL_REF_UST = 20.0      # m; RANGE_SET üst sınırı
MENZIL_ADIM_M = 0.5        # m/tık; menzil referansı tek-adım değişim sınırı
TUTUCU_HZ = 1.5            # dış döngü frekansı


# ══════════════════════════════════════════════════════════════════════════
#  GERİYE DÖNÜK ALIAS'LAR — mevcut importları bozmadan tek kaynağa bağlar.
#  (KilitSure, TespitDogrulama, kilit_kriteri, menzil_tutucu ve testler okur.)
# ══════════════════════════════════════════════════════════════════════════
PENCERE_SN = SARTNAME.PENCERE_SN
KUMULATIF_SN = SARTNAME.KUMULATIF_KILIT_SN
KESINTISIZ_SN = SARTNAME.KESINTISIZ_SN
KARE_TOLERANS_ORAN = SARTNAME.FRAME_TOLERANS_ORAN
ESIK_RESMI = SARTNAME.AH_EKRAN_ORAN_MIN        # resmi %5
ESIK_BILDIRIM = AYAR.AH_ORAN_GIRIS             # iç karar/bildirim eşiği (%6)
DOGRULAMA_SN = AYAR.TESPIT_DOGRULAMA_SN
TESPIT_TUTARLILIK_ORAN = AYAR.TESPIT_TUTARLILIK_ORAN
ORNEK_TAVAN_SN = AYAR.ORNEK_TAVAN_SN


def av_sinirlari(W, H):
    """AV dörtgeninin piksel sınırlarını (x_min, x_max, y_min, y_max) döndürür."""
    x_min = AV_YATAY[0] * W
    x_max = AV_YATAY[1] * W
    y_min = AV_DIKEY[0] * H
    y_max = AV_DIKEY[1] * H
    return (x_min, x_max, y_min, y_max)


def min_boyut(W, H):
    """Kilit için gereken minimum AH boyutunu (min_en, min_boy) piksel döndürür.

    ESIK_BILDIRIM (%6) tabanlıdır — histerezis için resmi eşiğin (%5) biraz üstü.
    """
    min_en = ESIK_BILDIRIM * W
    min_boy = ESIK_BILDIRIM * H
    return (min_en, min_boy)
