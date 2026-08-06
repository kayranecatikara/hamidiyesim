"""
control/kilitlenme.py — Şartname 6.1.4 "Kilitlenme Tespiti" mantığı (saf, Gazebo'suz).

Yarışma şartnamesi Şekil 2'deki geometri ve kilitlenme kurallarını uygular.
Buradaki hiçbir değer güdüme GİRMEZ — yalnız kilitlenme durumunu ölçer, çizim ve
panel için durum üretir (gösterge/skorlama katmanı). Güdüm/uçuş davranışı değişmez.

Kilit kriteri ve süre muhasebesi ayrı saf modüllere devredilmiştir:
  • Geometri/kilit koşulu : vision/kilit_kriteri.kriter_degerlendir
  • Kayan pencere/süre     : control/kilit_sure.KilitSure
Eşiklerin ve pencere sürelerinin TEK doğruluk kaynağı config/kilit_sabitler.py'dir.

── Şekil 2 geometrisi (kadraj = %100 × %100) ──
  AK : Kamera Görüş Alanı  → tüm kadraj (W×H)
  AV : Hedef Vuruş Alanı    → SARI KUTU. Ortada; yatayda %25 sol + %25 sağ boşluk
       (genişlik = kadrajın %50'si), dikeyde %10 üst + %10 alt boşluk
       (yükseklik = kadrajın %80'i). Hedef İHA merkezinin içinde tutulması gereken bölge.
  AH : Kilitlenme Dörtgeni  → KIRMIZI (#FF0000). Hedefin (HH) etrafına çizilen kutu.
  HH : Hedef Hava Aracı

── Kilitlenme kuralları (6.1.4) ──
  • Boyut şartı: hedef, ekranın yatay VEYA dikey ekseninden en az birinde ≥ eşik kaplamalı.
    (Eşik = config.ESIK_BILDIRIM; resmi %5 üstünde iç karar eşiği %6.)
  • Anlık kilit: hedef merkezi AV (sarı kutu) içinde VE boyut şartı sağlanmış.
  • 10 sn'lik kayan değerlendirme penceresinde KÜMÜLATİF kilit süresi ≥ 5 sn → kilit isteri sağlandı.
    Süre kesintili olabilir; pencere içindeki toplam süre sayılır.

── Üç faz (görsel güdüm gösterge katmanı) ──
  TAKIBE_GECIS : Hedef tespit edildikten sonra, ekranda yatay/dikey eşik kaplayana kadar
                 yaklaşılan faz.
  TAKIP        : Boyut şartı sağlandıktan sonra hedefin SARI KUTU içinde tutulduğu,
                 kümülatif kilidin biriktirildiği faz.
  TERMINAL     : Kilit isteri (≥5 sn kümülatif, pencere_ok) karşılandıktan sonra,
                 hedefe çarpmaya kadar süren faz.
  BEKLE        : Henüz tespit yok / uzun süre temas kaybı.
"""

import logging
import os

from config.kilit_sabitler import ESIK_BILDIRIM, KUMULATIF_SN, PENCERE_SN
from control.kilit_sure import KilitSure
from vision.kilit_kriteri import kriter_degerlendir
from vision.tespit_dogrulama import TespitDogrulama

_log = logging.getLogger(__name__)

# Geriye dönük uyum uyarıları: kilit kararı artık yalnız config'ten gelir; bu
# env'ler yalnız gösterimi değiştirip tutarsızlık yaratacağından okunmaz.
if "AVCI_KILIT_BOYUT" in os.environ:
    _log.warning(
        "AVCI_KILIT_BOYUT artık kullanılmıyor; kilit eşiği "
        "config.kilit_sabitler.ESIK_BILDIRIM (=%.3f) ile belirlenir.",
        ESIK_BILDIRIM,
    )
if "AVCI_KILIT_PENCERE" in os.environ or "AVCI_KILIT_HEDEF" in os.environ:
    _log.warning(
        "AVCI_KILIT_PENCERE/AVCI_KILIT_HEDEF artık kullanılmıyor; pencere ve "
        "kümülatif eşik config.kilit_sabitler.PENCERE_SN (=%.1f) / "
        "KUMULATIF_SN (=%.1f) ile belirlenir.",
        PENCERE_SN, KUMULATIF_SN,
    )


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
    # Boyut şartı eşiği — TEK doğruluk kaynağı config.ESIK_BILDIRIM (env okunmaz).
    BOYUT_ESIK = ESIK_BILDIRIM
    # Değerlendirme penceresi ve kümülatif eşik — TEK kaynak config (env okunmaz).
    PENCERE_S = PENCERE_SN
    HEDEF_S = KUMULATIF_SN
    # Bu süre boyunca hiç tespit yoksa fazlar BEKLE'ye döner (görev kaybı)
    SIFIRLA_S = _envf("AVCI_KILIT_SIFIRLA", 3.0)


class KilitTakip:
    """Kare kare kilitlenme durumu üretir. Thread-safe DEĞİL — tek üreticiden
    (kamera işleme thread'i) çağrılmalıdır."""

    def __init__(self, img_w, img_h, cfg=KilitCfg):
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
        """Görev başlangıcı/bitişinde çağrılır: AÇIK reset. Fazları ve süre
        muhasebesini (kayan pencere) sıfırlar."""
        self._sure = KilitSure()               # açık reset (RESET_POLICY=kumulatif_korunur)
        self._dogrulama = TespitDogrulama()    # 6.1.1 çok-kareli tespit doğrulama kapısı
        self._son_tespit_t = None
        self._takip_latch = False              # boyut şartı bir kez sağlandı mı?
        self._terminal_latch = False           # kilit isteri (pencere_ok) bir kez sağlandı mı?
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
            "marj": 0.0,
            "tespit_dogrulandi": False,
            "kumulatif_s": 0.0,
            "kesintisiz_s": 0.0,
            "pencere_ok": False,
            "kilit_isteri_ok": False,
            "pencere_s": self.cfg.PENCERE_S,
            "hedef_s": self.cfg.HEDEF_S,
        }

    def guncelle(self, bbox, now):
        """Bir kareyi işle.

        bbox : (x1, y1, x2, y2) hedef kutusu (piksel) ya da None (tespit yok).
        now  : sim saati (kare zaman damgası, header.stamp) ya da time.time().
        Döner: durum sözlüğü (çizim + telemetri için).
        """
        cfg = self.cfg

        tespit_var = bbox is not None
        if tespit_var:
            self._son_tespit_t = now

        # Uzun temas kaybı → fazlar BEKLE'ye döner. KilitSure.reset() ÇAĞRILMAZ:
        # kayan pencere doğal olarak unutur (RESET_POLICY=kumulatif_korunur).
        if (self._son_tespit_t is None or
                now - self._son_tespit_t > cfg.SIFIRLA_S):
            self._takip_latch = False
            self._terminal_latch = False

        # 6.1.1 doğrulama kapısı — her karede beslenir (kendi 0.5 sn penceresiyle
        # doğal unutur; SIFIRLA_S ile ayrıca sıfırlanmaz).
        dogrulanmis = self._dogrulama.guncelle(now, tespit_var)

        durum = self._bos_durum("BEKLE")
        durum["tespit_var"] = tespit_var
        durum["tespit_dogrulandi"] = dogrulanmis

        anlik_kilit = False
        if tespit_var:
            x1, y1, x2, y2 = (float(v) for v in bbox)
            w = max(0.0, x2 - x1)
            h = max(0.0, y2 - y1)
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            # Kilit koşulu (merkez∈AV VE boyut-VEYA) + marj — saf geometri modülü.
            # W,H kaynak kareden (detector çıktısı çerçevesi) alınır; sabit yok.
            kr = kriter_degerlendir(cx, cy, w, h, self.img_w, self.img_h)
            anlik_kilit = kr.kilit

            durum["ah_kutu"] = (int(x1), int(y1), int(x2), int(y2))
            durum["kaplama_x"] = w / self.img_w
            durum["kaplama_y"] = h / self.img_h
            durum["boyut_ok"] = kr.boyut_ok
            durum["merkez_av_icinde"] = kr.merkez_av_icinde
            durum["marj"] = kr.marj

            # TAKIBE_GECIS → TAKIP: boyut şartı VE doğrulanmış tespit (tek-kare
            # boyut FP'si fazı titretmesin — ş3). Kümülatif hesap değişmez.
            if kr.boyut_ok and dogrulanmis:
                self._takip_latch = True

        # Kümülatif kilit: her karede süre modülüne besle (kare sayma YOK).
        sd = self._sure.guncelle(now, anlik_kilit)
        if sd.pencere_ok:
            self._terminal_latch = True

        durum["anlik_kilit"] = anlik_kilit
        durum["kumulatif_s"] = sd.kumulatif_sn
        durum["kesintisiz_s"] = sd.kesintisiz_sn
        durum["pencere_ok"] = sd.pencere_ok
        durum["kilit_isteri_ok"] = self._terminal_latch

        # ── Faz kararı (ileri-kilitlemeli) ──
        # TAKIP → TERMINAL yalnızca pencere_ok'a bağlıdır (kesintisiz_ok DEĞİL;
        # kesintisiz kilit angajman katmanının işidir — Adım 6).
        if self._terminal_latch:
            durum["faz"] = "TERMINAL"
        elif self._takip_latch:
            durum["faz"] = "TAKIP"
        elif dogrulanmis:
            durum["faz"] = "TAKIBE_GECIS"   # BEKLE→TAKIBE_GECIS: doğrulanmış tespit
        else:
            durum["faz"] = "BEKLE"

        self._son_durum = durum
        return durum

    @property
    def durum(self):
        return self._son_durum

    def beyan_araligi(self, t):
        """SALT-OKUR: bildirim beyan aralığını KilitSure'a devreder (davranış
        değiştirmez). Dönüş: (baslangic, bitis, kumulatif) | None."""
        return self._sure.beyan_araligi(t)


# Faz → insan-okur etiket (UI ile ortak)
FAZ_ETIKET = {
    "BEKLE": "BEKLE",
    "TAKIBE_GECIS": "TAKİBE GEÇİŞ",
    "TAKIP": "TAKİP",
    "TERMINAL": "TERMİNAL",
}
