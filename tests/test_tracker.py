# tests/test_tracker.py — vision/tracker.py (HybridSORT sarmalayıcısı) birim testleri.
# Model/GPU gerektirmez; sentetik tespit dizileriyle takip davranışını doğrular.
import numpy as np
import pytest

pytest.importorskip("boxmot", reason="boxmot kurulu değil")

from vision.detector import best_det
from vision.tracker import TalonTracker, TargetLock, draw_tracks


def _frame():
    return (np.random.default_rng(7).random((480, 640, 3)) * 255).astype(np.uint8)


def _det(x, y=200, w=40, h=30, conf=0.9):
    return np.array([[x, y, x + w, y + h, conf, 0]], dtype=np.float32)


def test_kararli_id_ve_format():
    """Düzgün hareket eden tek hedef: tek ID, M×8 çıktı formatı."""
    tr, frame = TalonTracker(), _frame()
    ids, takipli = set(), 0
    for k in range(40):
        out = tr.update(_det(60 + 5 * k), frame)
        if len(out):
            assert out.shape[1] == 8
            ids.add(int(out[0][4]))
            takipli += 1
    assert len(ids) == 1          # kimlik hiç değişmedi
    assert takipli >= 30          # min_hits(3) sonrası her karede takip


def test_tespit_bosluklarinda_id_korunur():
    """max_age(30) içinde kalan 5 karelik tespit kaybı ID'yi değiştirmemeli."""
    tr, frame = TalonTracker(), _frame()
    ids_once, ids_sonra = set(), set()
    for k in range(50):
        dets = (np.empty((0, 6), dtype=np.float32) if 20 <= k < 25
                else _det(60 + 5 * k))
        out = tr.update(dets, frame)
        (ids_once if k < 20 else ids_sonra).update(int(t[4]) for t in out)
    assert len(ids_once) == 1
    assert ids_sonra == ids_once  # boşluktan sonra AYNI kimlik


def test_bos_ve_none_giris():
    """Tespitsiz kare / None / boş liste takibi düşürmemeli, (0,8) dönmeli."""
    tr, frame = TalonTracker(), _frame()
    assert tr.update(np.empty((0, 6), dtype=np.float32), frame).shape == (0, 8)
    assert tr.update(None, frame).shape == (0, 8)
    assert tr.update([], frame).shape == (0, 8)


def test_dusuk_conf_tek_basina_track_acmaz():
    """det_thresh(0.3) altındaki tespitler yeni track BAŞLATMAZ (BYTE düşük
    skorluları yalnız MEVCUT track'leri sürdürmekte kullanır)."""
    tr, frame = TalonTracker(), _frame()
    out = None
    for k in range(15):
        out = tr.update(_det(60 + 5 * k, conf=0.2), frame)
    assert out is not None and len(out) == 0


def test_active_boxes_coast_koprusu():
    """update() tespitsiz karede track YAYINLAMAZ; active_boxes ise Kalman
    tahmin kutusunu coast sayacıyla vermeli (köprü/kilit politikaları için)."""
    tr, frame = TalonTracker(), _frame()
    for k in range(10):
        tr.update(_det(60 + 5 * k), frame)
    out = tr.update(np.empty((0, 6), dtype=np.float32), frame)  # tespitsiz kare
    assert len(out) == 0                                        # yayın filtresi
    ab = tr.active_boxes(max_coast=5)
    assert len(ab) == 1
    assert ab[0]["coast"] >= 1
    assert 90 < ab[0]["bbox"][0] < 130   # tahmin hareket yönünde ilerlemiş olmalı
    assert tr.active_boxes(max_coast=0) == []                   # filtre çalışıyor


def test_draw_tracks():
    frame = _frame()
    tracks = np.array([[100, 100, 160, 140, 3, 0.8, 0, 0]], dtype=np.float32)
    assert draw_tracks(frame.copy(), tracks).shape == frame.shape
    assert draw_tracks(frame, None) is frame        # boş takipte kare aynen döner
    assert draw_tracks(frame, np.empty((0, 8))) is frame


# ─── TargetLock (kilitli-ID politikası) ───

def _adim(tr, kilit, dets, frame):
    d = best_det(dets) if dets is not None and len(dets) else None
    out = tr.update(dets if dets is not None else np.empty((0, 6), np.float32), frame)
    return kilit.step(out, d)


def test_lock_kilitlenir_ve_esleme_doner():
    tr, frame = TalonTracker(), _frame()
    kilit = TargetLock(tr)
    lock = None
    for k in range(6):
        lock = _adim(tr, kilit, _det(60 + 5 * k, conf=0.9), frame)
    assert lock is not None and lock["kaynak"] == "eslesme"
    assert kilit.relock_sayisi == 1
    ilk_id = lock["id"]
    for k in range(6, 12):
        lock = _adim(tr, kilit, _det(60 + 5 * k, conf=0.9), frame)
    assert lock["id"] == ilk_id           # kimlik değişmedi


def test_lock_dusuk_confa_kilitlenmez():
    """lock_conf(0.5) altındaki onaylı track kilit ALMAZ (FP koruması)."""
    tr, frame = TalonTracker(), _frame()
    kilit = TargetLock(tr)
    lock = None
    for k in range(8):
        lock = _adim(tr, kilit, _det(60 + 5 * k, conf=0.4), frame)
    assert lock is None and kilit.lock_id is None


def test_lock_coast_kaynagi_ve_dusme():
    """Tespit kesilince coast<=max_coast 'tahmin' döner; aşınca kilit düşer."""
    tr, frame = TalonTracker(), _frame()
    kilit = TargetLock(tr, max_coast=5)
    for k in range(8):
        _adim(tr, kilit, _det(60 + 5 * k, conf=0.9), frame)
    kaynaklar = []
    for _ in range(7):                      # 7 tespitsiz kare (max_coast=5'i aşar)
        lock = _adim(tr, kilit, None, frame)
        kaynaklar.append(None if lock is None else lock["kaynak"])
    assert kaynaklar[:5] == ["tahmin"] * 5   # köprü
    assert kaynaklar[-1] is None             # coast aşıldı → kilit düştü
    assert kilit.lock_id is None


def test_lock_gap_sonrasi_ayni_id():
    """max_coast içindeki boşlukta kilit aynı ID ile devam etmeli (relock YOK)."""
    tr, frame = TalonTracker(), _frame()
    kilit = TargetLock(tr, max_coast=5)
    for k in range(8):
        lock = _adim(tr, kilit, _det(60 + 5 * k, conf=0.9), frame)
    ilk_id = lock["id"]
    for _ in range(3):
        _adim(tr, kilit, None, frame)                       # kısa boşluk
    for k in range(11, 15):
        lock = _adim(tr, kilit, _det(60 + 5 * k, conf=0.9), frame)
    assert lock["kaynak"] == "eslesme" and lock["id"] == ilk_id
    assert kilit.relock_sayisi == 1


def test_lock_sicrama_korumasi():
    """HybridSort tek aday varken sıfır-IoU tespiti bile track'e eşleyebiliyor
    (kimlik 'ışınlanır'). Sıçrama koruması bunu yakalamalı: o kare kilit düşer
    (None → gcs det_raw'a döner), yeni konum kalıcıysa yeniden kilitlenilir."""
    tr, frame = TalonTracker(), _frame()
    kilit = TargetLock(tr)
    for k in range(8):
        lock = _adim(tr, kilit, _det(60 + 5 * k, conf=0.9), frame)
    assert lock["kaynak"] == "eslesme"
    # A kayboldu; uzakta (IoU=0) güçlü tespit — tracker aynı ID'ye ışınlıyor
    lock1 = _adim(tr, kilit, _det(400, y=380, conf=0.9), frame)
    assert lock1 is None                      # şüpheli kare: kilit çıkışı YOK
    lock2 = _adim(tr, kilit, _det(400, y=380, conf=0.9), frame)
    assert lock2 is not None and lock2["kaynak"] == "eslesme"
    assert kilit.relock_sayisi == 2           # bilinçli yeniden kilitlenme


def test_lock_celiski_tazeleme():
    """POLİTİKA düzeyinde (sentetik track satırlarıyla): kilit zayıf track'te
    kalmışken güçlü tespit ısrarla BAŞKA yerdeyse, celiski_kare sonunda kilit
    bırakılıp güçlü track'e geçilmeli (FP-saplanma güvenlik ağı). Gerçek
    tracker'da ilişkilendirme genelde kimliği güçlü kutuya kendisi taşır ve
    sıçrama koruması yakalar (test_lock_sicrama_korumasi); bu test ikinci ağı
    tracker'dan bağımsız doğrular."""
    def satir(tid, x, y, conf):
        return [x, y, x + 40, y + 30, tid, conf, 0, 0]

    kilit = TargetLock(tracker=None, celiski_kare=6)   # coast yolu kullanılmıyor
    for k in range(4):                                  # yalnız zayıf id7 → kilit
        lock = kilit.step(np.array([satir(7, 100 + 2 * k, 200, 0.6)],
                                   dtype=np.float32))
    assert lock["id"] == 7
    yeni = None
    for k in range(4, 20):                              # id9 (güçlü) başka yerde
        tracks = np.array([satir(7, 100 + 2 * k, 200, 0.6),
                           satir(9, 400, 380, 0.95)], dtype=np.float32)
        best = {"conf": 0.95, "bbox": (400, 380, 440, 410)}
        lock = kilit.step(tracks, best)
        if lock is not None and lock["id"] != 7:
            yeni = lock
            break
    assert yeni is not None and yeni["id"] == 9         # güçlü track'e geçti
    assert kilit.relock_sayisi == 2






def test_best_det_sozlesmesi():
    """best_det, detect_talon'un dict sözleşmesini birebir üretmeli."""
    dets = np.array([[10, 20, 50, 60, 0.50, 0],
                     [100, 100, 140, 130, 0.90, 0]], dtype=np.float32)
    d = best_det(dets)
    assert d["bbox"] == (100, 100, 140, 130)
    assert d["conf"] == pytest.approx(0.90)
    assert (d["cx"], d["cy"], d["w"], d["h"]) == (120, 115, 40, 30)
    assert best_det(dets, conf_min=0.95) is None
    assert best_det(np.empty((0, 6), dtype=np.float32)) is None
    assert best_det(None) is None
