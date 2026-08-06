"""Tespit doğrulama kapısı — SAF sınıf (şartname 6.1.1).

Tek karelik tespit YETERLİ DEĞİLDİR: son DOGRULAMA_SN penceresinde tespitin
yeterince TUTARLI olması gerekir. Kural SÜRE tabanlıdır (kare sayısı değil):

    dogrulanmis = (gecen_sure >= DOGRULAMA_SN) VE
                  (tespitli_sure / DOGRULAMA_SN >= TESPIT_TUTARLILIK_ORAN)

"Aralık tespitli" sayımı control/kilit_sure.KilitSure ile BİREBİR aynı kuraldır:
İKİ uç da tespitli VE iki örnek arası dt <= ORNEK_TAVAN_SN. Böylece tespit hattı
donduğunda (büyük dt) donma aralığı tespitli sayılmaz ve kapı açık kalmaz.

I/O yok, thread yok, ağır import yok. Tek üreticiden çağrılmalıdır.
"""

from config.kilit_sabitler import (
    DOGRULAMA_SN,
    ORNEK_TAVAN_SN,
    TESPIT_TUTARLILIK_ORAN,
)


class TespitDogrulama:
    """Kare kare tespit örneklerini tüketip doğrulama kararı (bool) üretir."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._t0 = None                # ilk örnek anı (geçen süre için)
        self._t_prev = None
        self._tespit_prev = False
        self._spans = []               # [start, end] tespitli aralıklar (bitişik)

    def guncelle(self, t, tespit_var):
        """Bir örnek işle, güncel doğrulama kararını (bool) döndür."""
        tv = bool(tespit_var)
        if self._t0 is None:
            self._t0 = t
            self._t_prev = t
            self._tespit_prev = tv
            return False               # tek örnek asla doğrulanmaz

        dt = t - self._t_prev
        if dt < 0.0:
            dt = 0.0

        # KilitSure ile aynı kural: iki uç tespitli VE dt tavanın altında.
        aralik_tespitli = self._tespit_prev and tv and dt <= ORNEK_TAVAN_SN
        if aralik_tespitli:
            if self._spans and self._spans[-1][1] == self._t_prev:
                self._spans[-1][1] = t          # bitişik aralığı uzat
            else:
                self._spans.append([self._t_prev, t])

        self._t_prev = t
        self._tespit_prev = tv
        return self._dogrulanmis(t)

    def _dogrulanmis(self, t):
        if (t - self._t0) < DOGRULAMA_SN:
            return False                        # pencere henüz dolmadı
        w = t - DOGRULAMA_SN
        while self._spans and self._spans[0][1] < w:
            self._spans.pop(0)                  # pencere dışı → bellek temizliği
        tespitli = 0.0
        for s, e in self._spans:
            a = s if s > w else w
            if e > a:
                tespitli += e - a
        return (tespitli / DOGRULAMA_SN) >= TESPIT_TUTARLILIK_ORAN
