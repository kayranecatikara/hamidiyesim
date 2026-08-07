"""vision/kutu_yumusatici.py — Kilitlenme dörtgeni (AH) zamansal yumuşatıcı (SAF).

Ham YOLO kutusu kareler arası zıplar (ölçülen: ort 5.5 px, tepe 72 px konum
sıçraması; ara sıra 4× boyut fışkırması). Videodaki kırmızı dörtgen bu yüzden
titrer ve anlık kilit yanıp söner. Bu sınıf kutuyu EMA ile yumuşatır ve
sahte boyut sıçramalarını reddeder.

KAYNAĞA uygulanır (KilitTakip'e beslenen kutu) → kriter + çizim + hakem paketi
TEK KAYNAK kalır (#3). Tespit kaybında sıfırlanır (bayat kutu çizilmez).
I/O yok, thread yok.
"""

from config.kilit_sabitler import AYAR


class KutuYumusatici:
    """Kare kare (x1,y1,x2,y2)|None tüketir, yumuşatılmış kutu|None üretir."""

    def __init__(self, cfg=AYAR):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self._box = None      # yumuşatılmış (x1,y1,x2,y2) float
        self._cikti = None    # son EMİTLENEN int kutu (ölü bölge referansı)

    def yumusat(self, bbox, dt):
        """bbox: ham (x1,y1,x2,y2) piksel veya None. dt: kare aralığı (s).
        Dönüş: yumuşatılmış+ölü-bölgeli (x1,y1,x2,y2) int veya None (tespit yok)."""
        if bbox is None:
            self.reset()                          # tespit yok → bayat kutu tutma
            return None
        x = tuple(float(v) for v in bbox)
        if self._box is None:
            self._box = x                         # ilk kare: doğrudan
            return self._cikti_ver()
        # Boyut sıçrama reddi: yeni kutu öncekinden çok büyük/küçükse (sahte
        # dev/mini kutu) BU kareyi yok say, önceki yumuşatılmışı koru.
        w_yeni = max(1.0, x[2] - x[0]); w_eski = max(1.0, self._box[2] - self._box[0])
        r = w_yeni / w_eski
        if r > self.cfg.KUTU_SICRAMA_ORAN or r < 1.0 / self.cfg.KUTU_SICRAMA_ORAN:
            return self._cikti_ver()
        a = dt / (self.cfg.KUTU_EMA_TAU + dt) if dt > 0 else 1.0
        self._box = tuple(self._box[i] + a * (x[i] - self._box[i]) for i in range(4))
        return self._cikti_ver()

    def _cikti_ver(self):
        """EMA kutusunu int'e yuvarla; ölü bölge: son emitlenen kutudan herhangi
        bir köşe >= KUTU_OLU_BOLGE_PX kaymadıysa ESKİYİ döndür (sabit dur)."""
        aday = tuple(int(round(v)) for v in self._box)
        db = self.cfg.KUTU_OLU_BOLGE_PX
        if self._cikti is None or any(abs(aday[i] - self._cikti[i]) >= db for i in range(4)):
            self._cikti = aday
        return self._cikti
