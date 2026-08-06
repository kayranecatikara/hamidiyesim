
"""
control/kilitlenme.py — Şartname 6.1.4 "Kilitlenme Tespiti" mantığı (saf, Gazebo'suz).

Yarışma şartnamesi Şekil 2'deki geometri ve kilitlenme kurallarını uygular.
Buradaki hiçbir değer güdüme GİRMEZ — yalnız kilitlenme durumunu ölçer, çizim ve
panel için durum üretir (gösterge/skorlama katmanı). Güdüm/uçuş davranışı değişmez.

── Şekil 2 geometrisi (kadraj = %100 × %100) ──
  AK : Kamera Görüş Alanı  → tüm kadraj (640×480)
  AV : Hedef Vuruş Alanı    → SARI KUTU. Ortada; yatayda %25 sol + %25 sağ boşluk
       (genişlik = kadrajın %50'si), dikeyde %10 üst + %10 alt boşluk
       (yükseklik = kadrajın %80'i). Hedef İHA merkezinin içinde tutulması gereken bölge.
  AH : Kilitlenme Dörtgeni  → KIRMIZI (#FF0000). Hedefin (HH) etrafına çizilen kutu.
  HH : Hedef Hava Aracı

── Kilitlenme kuralları (6.1.4) ──
  • Boyut şartı: hedef, ekranın yatay VEYA dikey ekseninden en az birinde ≥ %5 kaplamalı.
    (Paket gönderme eşiğinin %6+ olması tavsiye edilir; AVCI_KILIT_BOYUT ile ayarlanır.)
  • Anlık kilit: hedef merkezi AV (sarı kutu) içinde VE boyut şartı sağlanmış.
  • 10 sn'lik kayan değerlendirme penceresinde KÜMÜLATİF kilit süresi ≥ 5 sn → kilit isteri sağlandı.
    Süre kesintili olabilir; pencere içindeki toplam süre sayılır.

── Üç faz (görsel güdüm gösterge katmanı) ──
  TAKIBE_GECIS : Hedef tespit edildikten sonra, ekranda yatay/dikey %5 kaplayana kadar
                 yaklaşılan faz.
  TAKIP        : Boyut şartı sağlandıktan sonra hedefin SARI KUTU içinde tutulduğu,
                 kümülatif kilidin biriktirildiği faz.
  TERMINAL     : Kilit isteri (≥5 sn kümülatif) karşılandıktan sonra, hedefe çarpmaya
                 kadar süren faz.
  BEKLE        : Henüz tespit yok / uzun süre temas kaybı.
"""

import collections
import os


def _envf(ad, varsayilan):
    try:
        return float(os.environ.get(ad, varsayilan))
    except (TypeError, ValueError):
        return float(varsayilan)


class KilitCfg:
    """Ayarlar (env ile geçilebilir)."""
    # AV (sarı kutu) kenar boşlukları — Şekil 2: yatay %25, dikey %10
    YATAY_BOSLUK = _envf("AVCI_KILIT_AV_YATAY", 0.25)
    DIKEY_BOSLUK = _envf("AVCI_KILIT_AV_DIKEY", 0.10)
    # Boyut şartı: kadrajın en az bu oranı (Şekil 2: %5). Paket eşiği için %6 tavsiye.
    BOYUT_ESIK = _envf("AVCI_KILIT_BOYUT", 0.05)
    # Değerlendirme penceresi ve gereken kümülatif kilit süresi (sn)
    PENCERE_S = _envf("AVCI_KILIT_PENCERE", 10.0)
    HEDEF_S = _envf("AVCI_KILIT_HEDEF", 5.0)
    # Bu süre boyunca hiç tespit yoksa fazlar/kilit sıfırlanır (görev kaybı)
    SIFIRLA_S = _envf("AVCI_KILIT_SIFIRLA", 3.0)
    # İki kare arası boşluk bundan büyükse kümülatife katkı sayılmaz (donma/atlama koruması)
    MAKS_DT = _envf("AVCI_KILIT_MAKS_DT", 0.5)


class KilitTakip:
    """Kare kare kilitlenme durumu üretir. Thread-safe DEĞİL — tek üreticiden
    (kamera işleme thread'i) çağrılmalıdır."""

    def __init__(self, img_w=640, img_h=480, cfg=KilitCfg):
        self.img_w = int(img_w)
        self.img_h = int(img_h)
        self.cfg = cfg
        # AV (sarı kutu) piksel köşeleri — ortada, Şekil 2 boşluklarıyla
        self.av = (
            int(round(cfg.YATAY_BOSLUK * self.img_w)),
            int(round(cfg.DIKEY_BOSLUK * self.img_h)),
            int(round((1.0 - cfg.YATAY_BOSLUK) * self.img_w)),
            int(round((1.0 - cfg.DIKEY_BOSLUK) * self.img_h)),
        )
        self.sifirla()

    def sifirla(self):
        """Görev başlangıcı/bitişinde çağrılır: fazları ve kümülatif pencereyi sıfırlar."""
        self._pencere = collections.deque()   # (t, kilitli_sure_s) katkıları
        self._son_t = None
        self._son_tespit_t = None
        self._takip_latch = False              # boyut şartı bir kez sağlandı mı?
        self._terminal_latch = False           # kilit isteri (≥5 sn) bir kez sağlandı mı?
        self._son_durum = self._bos_durum("BEKLE")

    # ── yardımcılar ──
    def _bos_durum(self, faz):
        return {
            "faz": faz,
            "av_kutu": self.av,
            "ah_kutu": None,
            "anlik_kilit": False,
            "tespit_var": False,
            "kaplama_x": 0.0,
            "kaplama_y": 0.0,
            "boyut_ok": False,
            "merkez_av_icinde": False,
            "kumulatif_s": 0.0,
            "kilit_isteri_ok": False,
            "pencere_s": self.cfg.PENCERE_S,
            "hedef_s": self.cfg.HEDEF_S,
        }

    def _pencere_temizle(self, now):
        alt = now - self.cfg.PENCERE_S
        p = self._pencere
        while p and p[0][0] < alt:
            p.popleft()

    def _kumulatif(self):
        return sum(sure for _, sure in self._pencere)

    def guncelle(self, bbox, now):
        """Bir kareyi işle.

        bbox : (x1, y1, x2, y2) hedef kutusu (piksel) ya da None (tespit yok).
        now  : duvar saati (time.time()).
        Döner: durum sözlüğü (çizim + telemetri için).
        """
        cfg = self.cfg
        dt = 0.0
        if self._son_t is not None:
            dt = now - self._son_t
        self._son_t = now

        tespit_var = bbox is not None
        if tespit_var:
            self._son_tespit_t = now

        # Uzun temas kaybı → görev sıfırlaması (fazlar ve pencere temizlenir)
        if (self._son_tespit_t is None or
                now - self._son_tespit_t > cfg.SIFIRLA_S):
            self._pencere.clear()
            self._takip_latch = False
            self._terminal_latch = False

        durum = self._bos_durum("BEKLE")
        durum["tespit_var"] = tespit_var

        anlik_kilit = False
        if tespit_var:
            x1, y1, x2, y2 = (float(v) for v in bbox)
            w = max(0.0, x2 - x1)
            h = max(0.0, y2 - y1)
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            kap_x = w / self.img_w
            kap_y = h / self.img_h
            boyut_ok = (kap_x >= cfg.BOYUT_ESIK) or (kap_y >= cfg.BOYUT_ESIK)
            ax1, ay1, ax2, ay2 = self.av
            merkez_ic = (ax1 <= cx <= ax2) and (ay1 <= cy <= ay2)
            anlik_kilit = boyut_ok and merkez_ic

            durum["ah_kutu"] = (int(x1), int(y1), int(x2), int(y2))
            durum["kaplama_x"] = kap_x
            durum["kaplama_y"] = kap_y
            durum["boyut_ok"] = boyut_ok
            durum["merkez_av_icinde"] = merkez_ic

            if boyut_ok:
                self._takip_latch = True

        # Kümülatif kilit penceresi: bu karede kilitliyse geçen süreyi ekle
        if anlik_kilit and 0.0 < dt <= cfg.MAKS_DT:
            self._pencere.append((now, dt))
        self._pencere_temizle(now)
        kumulatif = self._kumulatif()
        if kumulatif >= cfg.HEDEF_S:
            self._terminal_latch = True

        durum["anlik_kilit"] = anlik_kilit
        durum["kumulatif_s"] = kumulatif
        durum["kilit_isteri_ok"] = self._terminal_latch

        # ── Faz kararı (ileri-kilitlemeli) ──
        if self._terminal_latch:
            durum["faz"] = "TERMINAL"
        elif self._takip_latch:
            durum["faz"] = "TAKIP"
        elif tespit_var:
            durum["faz"] = "TAKIBE_GECIS"
        else:
            durum["faz"] = "BEKLE"

        self._son_durum = durum
        return durum

    @property
    def durum(self):
        return self._son_durum


# Faz → insan-okur etiket (UI ile ortak)
FAZ_ETIKET = {
    "BEKLE": "BEKLE",
    "TAKIBE_GECIS": "TAKİBE GEÇİŞ",
    "TAKIP": "TAKİP",
    "TERMINAL": "TERMİNAL",
}
