"""senaryo_cfg.py — HEDEF senaryosunun canlı ayarları (panel düğmeleri).

NEDEN AYRI DOSYA: senaryo (`control/run_plane_scenario.py`) gcs_server'ın
başlattığı AYRI BİR SÜREÇTİR. Güdüm özelliklerinde kullanılan
"gcs_server sınıf niteliğini değiştirir, güdüm döngüsü bir sonraki karede
okur" yolu burada İŞLEMEZ — iki süreç bellek paylaşmaz.

Bu yüzden akış şöyle:
    panel  →  POST /api/gudum_ozellikleri  →  gcs_server bu sınıfı değiştirir
    senaryo süreci  →  GET /api/senaryo_ayar  (0.5 s önbellek)  →  okur

Yani düğme UÇUŞ SIRASINDA, yeniden başlatmadan etki eder (CLAUDE.md §6).
Env anahtarları otomatik kampanyalar ve başlangıç varsayılanı için durur.
"""

import os


def _bayrak(ad, varsayilan="1"):
    return os.environ.get(ad, varsayilan).lower() not in ("0", "off", "false")


class SenaryoCfg:
    # Hedefin irtifa tutucusu (düz + bekleme turu + daire fazları).
    # KAPALI = özellik eklenmeden önceki açık çevrim davranış.
    IRTIFA_TUT = _bayrak("AVCI_SCN_IRTIFA_TUT")
