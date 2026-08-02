"""
vision/tracker.py — HybridSORT (BoxMOT) hedef takipçisi.

Detection tek kare tespit eder; tracker kareler ARASI kimliği (ID) sürdürür:
kısa tespit kayıplarını Kalman öngörüsüyle köprüler (max_age), düşük güvenli
tespitleri BYTE eşleşmesiyle değerlendirir (use_byte + low_thresh). Canlı gcs
pipeline'ı ve offline video aracı (vision/hybridsort_video.py) AYNI sarmalayıcıyı
kullanır.

Kullanım:
    from vision.tracker import TalonTracker, draw_tracks
    tr = TalonTracker()                    # TRACKER_PARAMS ile kurulur
    tracks = tr.update(dets, frame)        # dets: N×6 [x1,y1,x2,y2,conf,cls]
    # tracks: M×8 [x1,y1,x2,y2,id,conf,cls,det_ind]

Sürüm notu: boxmot==19.0.0 — numpy<2 ile uyumlu SON sürüm (20+ numpy>=2.2
dayatır; ortamdaki cv2 4.9 / torch 2.5.1 numpy 1.26'ya bağlı, YÜKSELTME).
Modül yolu sürümler arasında değiştiği için import toleranslı (_import_hybridsort).
"""

import logging
import math

import numpy as np

# ─── TRACKER PARAMETRELERİ ───────────────────────────────────────────────────
# HybridSort ve BaseTracker sistem default'ları.
# Değiştirmek istediğin parametreyi buradan düzenle.
TRACKER_PARAMS = {
    # BaseTracker
    "det_thresh":    0.3,
    "max_age":       30,
    "max_obs":       50,
    "min_hits":      3,
    "iou_threshold": 0.3,
    "asso_func":     "iou",

    # HybridSort
    "with_reid":                           False,   # .pt dosyası olmadan False bırak
    "low_thresh":                          0.1,
    "delta_t":                             3,
    "inertia":                             0.05,
    "use_byte":                            True,
    "longterm_bank_length":                30,
    "alpha":                               0.9,
    "track_thresh":                        0.5,
    "EG_weight_high_score":                4.6,
    "EG_weight_low_score":                 1.3,
    "TCM_first_step":                      True,
    "TCM_byte_step":                       True,
    "TCM_byte_step_weight":                1.0,
    "high_score_matching_thresh":          0.7,
    "with_longterm_reid":                  True,
    "longterm_reid_weight":                0.0,
    "with_longterm_reid_correction":       True,
    "longterm_reid_correction_thresh":     0.4,
    "longterm_reid_correction_thresh_low": 0.4,
}
# ─────────────────────────────────────────────────────────────────────────────


def _import_hybridsort():
    """BoxMOT sürümleri arasında modül yolu değişti — sırayla dene."""
    try:                                     # v20/v21 düzeni (paket içinde modül)
        from boxmot.trackers.bbox.hybridsort.hybridsort import HybridSort
        return HybridSort
    except ImportError:
        pass
    try:                                     # v22 düzeni (bbox/hybridsort.py)
        from boxmot.trackers.bbox.hybridsort import HybridSort
        return HybridSort
    except ImportError:
        pass
    from boxmot.trackers.hybridsort.hybridsort import HybridSort   # v19 düzeni
    return HybridSort


def silence_boxmot_logs():
    """ECC "did not converge" gibi kare başı WARNING selini keser (dokusu az
    gökyüzü karelerinde her karede basar). Hatalar (ERROR) görünmeye devam eder."""
    logging.getLogger("boxmot").setLevel(logging.ERROR)


class TalonTracker:
    """HybridSort'u proje sözleşmesiyle saran ince katman.

    - update() girişte None/boş listeyi tolere eder (boş karede track'ler yaşlanır),
      çıkışı her zaman float32 M×8 ndarray'e normalize eder.
    - quiet=True kare başı WARNING loglarını susturur (canlı gcs için şart).
    """

    def __init__(self, reid_model=None, quiet=True, **overrides):
        if quiet:
            silence_boxmot_logs()
        HybridSort = _import_hybridsort()
        import sys
        self._boxmot_mod = sys.modules[HybridSort.__module__]
        self.params = dict(TRACKER_PARAMS)
        self.params.update(overrides)
        self._make = lambda: HybridSort(reid_model=reid_model, **self.params)
        self._tracker = self._make()

    def update(self, dets, frame):
        """dets: N×6 [x1,y1,x2,y2,conf,cls] (None/boş olabilir) → M×8 tracks."""
        if dets is None or len(dets) == 0:
            dets = np.empty((0, 6), dtype=np.float32)
        dets = np.asarray(dets, dtype=np.float32).reshape(-1, 6)
        out = self._tracker.update(dets, frame)
        if out is None or len(out) == 0:
            return np.empty((0, 8), dtype=np.float32)
        return np.asarray(out, dtype=np.float32)

    def active_boxes(self, max_coast=None):
        """İç track listesini yayın filtresiz okur. update() çıktısı yalnız o
        karede tespit eşleşen track'leri içerir; buradan ise tespit gelmeyen
        karede de Kalman TAHMİN kutusu okunur (köprü/kilit politikaları için).
        Dönüş: list[dict(id, bbox, coast, hits, conf)] — coast = son eşleşmeden
        beri geçen kare sayısı (0 = bu karede eşleşti)."""
        out = []
        trks = getattr(self._tracker, "active_tracks", None)
        conv = getattr(self._boxmot_mod, "convert_x_to_bbox", None)
        if trks is None:
            return out
        for t in trks:
            coast = int(getattr(t, "time_since_update", 0))
            if max_coast is not None and coast > max_coast:
                continue
            try:
                if coast < 1 and np.asarray(t.last_observation).sum() >= 0:
                    bb = np.asarray(t.last_observation[:4], dtype=float)
                elif conv is not None:
                    bb = np.asarray(conv(t.kf.x)[0][:4], dtype=float)
                else:
                    continue
            except Exception:
                continue
            out.append({"id": int(t.id) + 1,
                        "bbox": tuple(float(v) for v in bb),
                        "coast": coast,
                        "hits": int(getattr(t, "hits", 0)),
                        "conf": float(getattr(t, "conf", 0.0))})
        return out

    def reset(self):
        """Takip durumunu sıfırlar (yeni oturum). Not: HybridSort ID sayacı
        sınıf düzeyinde — ID'ler kaldığı yerden devam eder, bu bir hata değil."""
        self._tracker = self._make()


def _iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


class TargetLock:
    """Kilitli-ID hedef seçim politikası (GT'li deneyle doğrulandı — compare_tracker).

    Kural:
    - Kilit yokken: bu karede YAYINLANAN (onaylı+eşleşmiş) track'lerden
      conf >= lock_conf olanların en güvenlisine kilitlenir.
    - Kilitli ID bu karede eşleşmişse kutusu kaynak='eslesme' döner (BYTE
      düşük-conf eşleşmeleri dahil — deneyde +50 doğru kare / 9 yanlış).
    - Eşleşmemişse ve coast <= max_coast ise Kalman TAHMİN kutusu
      kaynak='tahmin' döner. Tahmin kutusu YALNIZ krop/kimlik içindir, nişan
      için DEĞİL: deneyde doğru kareler coast=5'te doydu, uzun coast yalnız
      yanlış kutu ekledi.
    - Track ölür / coast aşılırsa kilit düşer, aynı karede yeniden denenir.
    - FP-saplanma koruması: karede conf >= celiski_conf bir tespit varken kilit
      çıktısı onunla celiski_kare ARDIŞIK kare IoU < celiski_iou çelişirse
      kilit bırakılır (sonraki karede güçlü tespitin track'ine kilitlenilir).
    - SIÇRAMA koruması: HybridSort tek aday varken sıfır-IoU tespiti bile
      mevcut track'e eşleyebiliyor (zayıf-ipucu maliyetinde conf benzerliği
      IoU'yu ezebilir — sentetikte ölçüldü). Kilit kutusu tek karede fiziksel
      olarak imkânsız sıçrarsa (merkez > max(sicrama_taban, sicrama_carpan ×
      önceki köşegen)) kimlik ŞÜPHELİ: kilit düşürülür, o kare çıktı verilmez
      (gcs det_raw'a düşer = eski davranış); yeni konum kalıcıysa sonraki kare
      yeniden kilitlenilir.
    """

    def __init__(self, tracker, lock_conf=0.5, max_coast=5,
                 celiski_kare=10, celiski_iou=0.10, celiski_conf=0.5,
                 sicrama_carpan=3.0, sicrama_taban=50.0):
        self.tracker = tracker
        self.lock_conf = lock_conf
        self.max_coast = max_coast
        self.celiski_kare = celiski_kare
        self.celiski_iou = celiski_iou
        self.celiski_conf = celiski_conf
        self.sicrama_carpan = sicrama_carpan
        self.sicrama_taban = sicrama_taban
        self.lock_id = None
        self.relock_sayisi = 0
        self._celiski = 0
        self._onceki_bbox = None

    def _kilitlen(self, tracks):
        aday = tracks[tracks[:, 5] >= self.lock_conf] if len(tracks) else tracks
        if len(aday) == 0:
            return None
        t = aday[int(np.argmax(aday[:, 5]))]
        self.lock_id = int(t[4])
        self.relock_sayisi += 1
        self._celiski = 0
        return {"id": self.lock_id, "bbox": tuple(float(v) for v in t[:4]),
                "conf": float(t[5]), "kaynak": "eslesme", "coast": 0}

    def step(self, tracks, best=None):
        """Her karede tracker.update() SONRASI çağrılır.
        tracks: update() çıktısı (M×8); best: detector.best_det dict'i (çelişki
        koruması için, opsiyonel). Dönüş: dict veya None."""
        if tracks is None:
            tracks = np.empty((0, 8), dtype=np.float32)
        out = None
        if self.lock_id is not None:
            rows = tracks[tracks[:, 4] == self.lock_id] if len(tracks) else tracks
            if len(rows):
                t = rows[0]
                out = {"id": self.lock_id, "bbox": tuple(float(v) for v in t[:4]),
                       "conf": float(t[5]), "kaynak": "eslesme", "coast": 0}
            else:
                ab = {a["id"]: a for a in
                      self.tracker.active_boxes(max_coast=self.max_coast)}
                if self.lock_id in ab:
                    a = ab[self.lock_id]
                    out = {"id": self.lock_id, "bbox": a["bbox"],
                           "conf": a["conf"], "kaynak": "tahmin",
                           "coast": a["coast"]}
                else:
                    self.lock_id = None            # öldü / coast aşıldı
        if out is None and self.lock_id is None:
            out = self._kilitlen(tracks)

        # SIÇRAMA koruması: kimlik tek karede ışınlandıysa şüpheli — kilidi bırak
        if out is not None and self._onceki_bbox is not None:
            ob, nb = self._onceki_bbox, out["bbox"]
            sicrama = math.hypot((nb[0] + nb[2]) / 2 - (ob[0] + ob[2]) / 2,
                                 (nb[1] + nb[3]) / 2 - (ob[1] + ob[3]) / 2)
            izin = max(self.sicrama_taban,
                       self.sicrama_carpan * math.hypot(ob[2] - ob[0], ob[3] - ob[1]))
            if sicrama > izin:
                self.lock_id = None
                self._celiski = 0
                self._onceki_bbox = None
                return None
        if out is not None:
            self._onceki_bbox = out["bbox"]

        # FP-saplanma koruması: güçlü tespit sürekli başka yerdeyse kilidi bırak
        if (out is not None and best is not None
                and best.get("conf", 0.0) >= self.celiski_conf):
            if _iou(out["bbox"], best["bbox"]) < self.celiski_iou:
                self._celiski += 1
                if self._celiski >= self.celiski_kare:
                    self.lock_id = None
                    self._celiski = 0
                    self._onceki_bbox = None   # yeni kimlik taze başlasın
            else:
                self._celiski = 0
        return out

    def reset(self):
        self.lock_id = None
        self._celiski = 0
        self._onceki_bbox = None


def draw_tracks(frame, tracks, line=2):
    """tracks: M×8 [x1, y1, x2, y2, id, conf, cls, det_ind] → kutu + ID overlay.
    Renk ID'den deterministik üretilir (aynı ID her karede aynı renk)."""
    import cv2
    if tracks is None or len(tracks) == 0:
        return frame
    for t in tracks:
        x1, y1, x2, y2 = map(int, t[:4])
        tid = int(t[4])
        conf = float(t[5])
        color = tuple(int(c) for c in np.random.default_rng(tid).integers(80, 255, 3))
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, line)
        cv2.putText(frame, f"ID:{tid} {conf:.2f}", (x1, max(y1 - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return frame
