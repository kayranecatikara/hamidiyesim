"""control/komut_kaydi.py — Komut vektörü yakalama (Adım 8).

send_velocity KÖPRÜ GÖVDESİNE dokunmadan, guidance modüllerinin bağlı
`send_velocity` adını AYNI İMZALI kaydeden sarmalayıcıyla değiştirir (runtime
monkeypatch; kaynak dosya düzenlenmez — CLAUDE.md "aynı imzalı sarmalayıcı"
izniyle). Sarmalayıcı son komutu (vx,vy,vz,yaw) paylaşımlı duruma yazar, sonra
"o an bağlı olan" orijinal fonksiyonu çağırır.

kur() İDEMPOTENT'tir: ikinci çağrıda çifte sarmaz. Tüm send_velocity
yönlendirmeleri (DoW köprüsü dahil) BAĞLANDIKTAN SONRA çağrılmalıdır.
"""

import threading

_lock = threading.Lock()
_son_komut = None            # (vx, vy, vz, yaw) | None
_sarildi = False             # idempotens bayrağı

# Sarılacak modüller: guidance hatlarının `send_velocity`'yi bağladığı yerler.
_HEDEF_MODULLER = (
    "control.guidance.gps_guidance",
    "control.guidance.visual_lead",
    "control.guidance.adapter_copter",
)


def get_son_komut():
    """Son gönderilen (vx, vy, vz, yaw) komutunu döndürür (veya None)."""
    with _lock:
        return _son_komut


def _sarmalayici(orijinal):
    def send_velocity(conn, vx, vy, vz, yaw):
        global _son_komut
        with _lock:
            _son_komut = (vx, vy, vz, yaw)
        return orijinal(conn, vx, vy, vz, yaw)
    send_velocity._komut_kaydi = True   # çifte-sarma tespiti için işaret
    return send_velocity


def kur():
    """Bağlı send_velocity referanslarını kaydeden sarmalayıcıyla değiştirir.

    Dönüş: sarılan modül sayısı. İdempotent — zaten sarılmışsa 0 döner.
    """
    global _sarildi
    import importlib
    with _lock:
        if _sarildi:
            return 0
        _sarildi = True
    sarilan = 0
    for ad in _HEDEF_MODULLER:
        try:
            mod = importlib.import_module(ad)
        except Exception:
            continue
        fn = getattr(mod, "send_velocity", None)
        if fn is None or getattr(fn, "_komut_kaydi", False):
            continue                     # yok ya da zaten sarılı → atla
        setattr(mod, "send_velocity", _sarmalayici(fn))
        sarilan += 1
    print(f"[KOMUT] komut kaydı {sarilan} modülü sardı")
    return sarilan
