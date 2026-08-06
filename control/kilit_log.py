"""control/kilit_log.py — CSV hakem logu (Adım 8).

Her kare için bir satır (tespitsiz karede de — bbox alanları boş). Kare hattını
BLOKLAMAMAK için: kamera thread'i satırı kuyruğa put_nowait ile bırakır (kuyruk
doluysa DÜŞÜRÜR ve sayar), AYRI daemon yazıcı thread CSV'ye toplu yazar +
periyodik flush. Gerekçe: disk stall'ı 30 Hz kare hattından izole eder; düşük
maliyetli append bile fsync stall'ında hattı riske atardı.

SALT-OKUR tüketici: karar mantığına dokunmaz.
"""

import os
import queue
import threading
import time

_ALANLAR = [
    "t_sim", "faz", "gorev_faz",
    "x1", "y1", "x2", "y2",
    "marj", "kumulatif_s", "kesintisiz_s", "kilit", "kilit_isteri_ok",
    "tespit_dogrulandi", "menzil", "menzil_ref",
    "vx", "vy", "vz", "yaw",
    "los_x", "los_y", "los_z",
]

_LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "logs")

_kuyruk = None          # queue.Queue
_thread = None
_stop = None            # threading.Event
_dusen = 0              # kuyruk dolu → düşen satır sayacı
_dusen_lock = threading.Lock()


def alanlar():
    return list(_ALANLAR)


def baslat(maxsize=512, flush_periyot=1.0):
    """Yazıcı thread'i başlatır. İdempotent (zaten çalışıyorsa yeni açmaz).
    Dönüş: CSV yolu."""
    global _kuyruk, _thread, _stop
    if _thread is not None and _thread.is_alive():
        return getattr(_thread, "_csv_yol", None)

    os.makedirs(_LOG_DIR, exist_ok=True)
    csv_yol = os.path.join(_LOG_DIR, time.strftime("kilit_%Y%m%d_%H%M%S.csv"))
    _kuyruk = queue.Queue(maxsize=maxsize)
    _stop = threading.Event()

    def _yazici():
        import csv
        with open(csv_yol, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=_ALANLAR, extrasaction="ignore")
            w.writeheader()
            son_flush = time.time()
            while not (_stop.is_set() and _kuyruk.empty()):
                try:
                    row = _kuyruk.get(timeout=0.2)
                except queue.Empty:
                    row = None
                if row is not None:
                    w.writerow(row)
                if time.time() - son_flush >= flush_periyot:
                    f.flush()
                    son_flush = time.time()
            f.flush()

    _thread = threading.Thread(target=_yazici, daemon=True)
    _thread._csv_yol = csv_yol
    _thread.start()
    print(f"[KILIT-LOG] hakem logu: {csv_yol}")
    return csv_yol


def kaydet(row):
    """Bir satırı kuyruğa bırak (NON-BLOCKING). Kuyruk doluysa düşür + say."""
    global _dusen
    if _kuyruk is None:
        return
    try:
        _kuyruk.put_nowait(row)
    except queue.Full:
        with _dusen_lock:
            _dusen += 1


def dusen_sayisi():
    with _dusen_lock:
        return _dusen


def kapat():
    """Yazıcıyı düzgün kapat (kalan kuyruğu boşaltıp flush eder)."""
    if _stop is not None:
        _stop.set()
    if _thread is not None:
        _thread.join(timeout=2.0)
