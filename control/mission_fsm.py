"""control/mission_fsm.py — Otonom görev durum makinesi (tek merkezî FSM).

Şartname 6.1 akışını KAPI'lı bir durum makinesine oturtur. Hiçbir aşama
atlanamaz; geçişler YALNIZ açık geçiş tablosu üzerinden, her geçiş kendi guard
fonksiyonuyla yapılır. Güdüm modu bu FSM'in durumunun TÜREVİDİR (bkz.
guidance-tarafı klemp) — bağımsız bir "hedef görüldü" bayrağı yoktur.

Akış (özet):
  SEARCH → APPROACH → DETECT → TRACK_LOCK → ENGAGE → STRIKE
                         ↑            │          │        │
                         └── TRACK_LOST ←────────┴────────┘   (kilit > X sn kayıp)
  STRIKE, yalnız ENGAGE içinde KESİNTİSİZ_SN kilit korunduğunda tetiklenir; bu
  süre içinde kilit koparsa STRIKE iptal → ENGAGE (tamamen kaybolursa TRACK_LOST).
  TRACK_LOST'tan asla doğrudan STRIKE'a gidilmez.

Zaman: t = kare damgası (sim monotonik saat), KARE SAYMA YOK. Kümülatif + kesintisiz
süre control.kilit_sure.KilitSure ile (6.1.4 %5 boşluk köprüsü, yalnız segment içi).
Çok-kareli tespit doğrulama vision.tespit_dogrulama.TespitDogrulama ile (6.1.1).

Bu modül SAF ve tek-thread'dir (kamera işleme thread'inden çağrılır). I/O yok
(log fonksiyonu enjekte edilir). Durum yayını gcs tarafında detection_state ile.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional

from config.kilit_sabitler import SARTNAME, AYAR
from vision.tespit_dogrulama import TespitDogrulama


class State(Enum):
    SEARCH = "SEARCH"           # bozuk GNSS ile arama deseni
    APPROACH = "APPROACH"       # hedefi AV (sarı kutu) içine GPS ile getir
    DETECT = "DETECT"           # tespit + kilit dörtgeni; N-kareli doğrulama
    TRACK_LOCK = "TRACK_LOCK"   # 10 sn pencerede kümülatif >= 5 sn
    ENGAGE = "ENGAGE"           # yönelim + mesafe azalt; çarpışma manevrası YOK
    STRIKE = "STRIKE"           # yalnız kesintisiz 3 sn kilitte
    TRACK_LOST = "TRACK_LOST"   # kilit > X sn kayıp; STRIKE'a asla gitmez


@dataclass
class Girdi:
    """Kare başına gözlem. anlik_kilit = o kare kilit geometrisi sağlanıyor mu
    (merkez AV içinde + AH ekran oranı >= eşik (histerezis) + kapsama >= %90).
    Üretimde anlik_kilit_gecerli() ile hesaplanır; testlerde doğrudan verilir."""
    t: float
    tespit_var: bool
    anlik_kilit: bool
    # Kilit SÜRELERİ TEK KAYNAKTAN (KilitTakip) — FSM'in kendi ayrı sayacı YOK.
    # Böylece UI/hakem paketi ile FSM'in ENGAGE/STRIKE kapıları BİREBİR aynı olur
    # (2026-08-07 saha bulgusu: iki ayrı KilitSure farklı sıfırlanıp ayrışıyordu).
    kumulatif_sn: float = 0.0       # 10 sn pencerede kümülatif kilit süresi
    kesintisiz_sn: float = 0.0      # anlık kesintisiz kilit süresi
    menzil: Optional[float] = None
    # Salt-gözlem (loglama için; karara girmez):
    ah_oran: float = 0.0            # max(bbox_w/W, bbox_h/H)
    merkez_sapma_x: float = 0.0     # |cx - kadraj_cx| / (bbox_w/2)
    merkez_sapma_y: float = 0.0
    kapanma_hizi: float = 0.0       # m/s (guidance'tan; ENGAGE reset logu için)


@dataclass
class FSMDurum:
    """FSM'in dışarıya yayınladığı anlık durum (salt-okur)."""
    state: State
    kumulatif_sn: float = 0.0
    kesintisiz_sn: float = 0.0
    pencere_ok: bool = False
    kesintisiz_ok: bool = False
    tespit_dogrulandi: bool = False
    kilit_kayip_sn: float = 0.0
    gecis_sayisi: int = 0


def anlik_kilit_gecerli(durum, onceki_kilit, cfg=AYAR, sart=SARTNAME):
    """KilitTakip.guncelle çıktısından şartname 6.1.4 ANLIK kilidini üretir.

    - merkez AV içinde (durum['merkez_av_icinde']),
    - AH ekran oranı histerezisi: girişte >= AH_ORAN_GIRIS, çıkışta < AH_ORAN_CIKIS
      (onceki_kilit True iken CIKIS eşiği kullanılır — sınırda titremeyi önler),
    - AH hedefi kapsama >= %90 (simülasyonda AH = hedef bbox → 1.0; gerçek
      donanımda kapsama ölçümü buraya beslenir).
    onceki_kilit: bir önceki karenin anlik_kilit değeri (histerezis için)."""
    if not durum or not durum.get("tespit_var"):
        return False
    if not durum.get("merkez_av_icinde"):
        return False
    kapsama = durum.get("ah_kapsama", 1.0)
    if kapsama < sart.AH_HEDEF_KAPSAMA_MIN:
        return False
    oran = max(durum.get("kaplama_x", 0.0), durum.get("kaplama_y", 0.0))
    esik = cfg.AH_ORAN_CIKIS if onceki_kilit else cfg.AH_ORAN_GIRIS
    return oran >= esik


class GorevFSM:
    """Merkezî görev durum makinesi. Her kare step(Girdi) çağrılır; guard'lar
    değerlendirilir, en fazla BİR geçiş yapılır, geçiş loglanır."""

    def __init__(self, log_fn: Optional[Callable[[str], None]] = None,
                 cfg=AYAR, sart=SARTNAME, reject_log_dt: float = 0.5):
        self.cfg = cfg
        self.sart = sart
        self._log_fn = log_fn if log_fn is not None else (lambda s: print(s))
        self._reject_log_dt = reject_log_dt
        self.reset()

    def reset(self):
        self.state = State.SEARCH
        # Süre sayacı FSM'de YOK — kümülatif/kesintisiz Girdi'den (KilitTakip) gelir.
        self._dogrula = TespitDogrulama()   # yalnız çok-kareli tespit doğrulama (6.1.1)
        self._son_kilit_t = None       # en son anlik_kilit True olan an
        self._son_tespit_t = None      # en son tespit_var True olan an
        self._gecis_sayisi = 0
        self._son_reject_t = -1e9
        self._son_reject_sebep = None
        self._prev_kesintisiz = 0.0
        self._son = FSMDurum(self.state)

    # ── zaman/kilit türev sinyalleri ──
    def _kilit_kayip_sure(self, t):
        if self._son_kilit_t is None:
            return float("inf")
        return max(0.0, t - self._son_kilit_t)

    def _tespit_kayip_sure(self, t):
        if self._son_tespit_t is None:
            return float("inf")
        return max(0.0, t - self._son_tespit_t)

    # ── geçiş tablosu: (state) → [(guard_adı, hedef_state), ...] öncelik sırasıyla ──
    _TABLO = {
        State.SEARCH:     [("hedef_ilk_tespit", State.APPROACH)],
        # APPROACH→SEARCH TESPİT kaybına bağlı (anlık KİLİT kaybına DEĞİL):
        # yaklaşırken hedef henüz kilitlenmemiş olması normaldir; kilit kaybı
        # guard'ı kullanılsaydı APPROACH↔SEARCH titrerdi (2026-08-07 saha logu).
        State.APPROACH:   [("tespit_kayip", State.SEARCH),
                           ("anlik_kilit_saglandi", State.DETECT)],
        State.DETECT:     [("kilit_tamamen_kayip", State.TRACK_LOST),
                           ("anlik_kilit_koptu", State.APPROACH),
                           ("tespit_dogrulandi", State.TRACK_LOCK)],
        State.TRACK_LOCK: [("kilit_tamamen_kayip", State.TRACK_LOST),
                           ("kumulatif_5s", State.ENGAGE)],
        State.ENGAGE:     [("kilit_tamamen_kayip", State.TRACK_LOST),
                           ("kesintisiz_3s", State.STRIKE)],
        # STRIKE bir kez tetiklendi mi COMMIT — dalış temasa dek sürdürülür.
        # Yakın mesafede hedef kareyi doldurup merkez AV'den çıkınca kilit
        # titrer (CLAUDE.md 6.1.3: ANGAJMAN'da merkez/%5 aranmaz), ama dalış
        # İPTAL EDİLMEZ. İptal edilebilirlik ENGAGE'deki 3 sn birikimde kalır
        # (orada kilit koparsa STRIKE hiç tetiklenmez). Yalnız TAM kayıp (kilit
        # > X sn yok = tamamen kaçtı/geçtik) STRIKE'ı bozar.
        State.STRIKE:     [("kilit_tamamen_kayip", State.TRACK_LOST)],
        State.TRACK_LOST: [("yeniden_tespit", State.APPROACH)],
    }

    # İlerleme (progress) guard'ları: reddedildiklerinde sebebiyle loglanır.
    _ILERLEME_GUARD = {"tespit_dogrulandi", "kumulatif_5s", "kesintisiz_3s"}

    def step(self, girdi: Girdi) -> State:
        t = girdi.t
        # 1) Kilit süreleri TEK KAYNAK (KilitTakip → girdi); FSM sayaç TUTMAZ.
        pencere_ok = girdi.kumulatif_sn >= self.sart.KUMULATIF_KILIT_SN
        kesintisiz_ok = girdi.kesintisiz_sn >= self.sart.KESINTISIZ_SN
        dogrulandi = self._dogrula.guncelle(t, girdi.tespit_var)
        if girdi.anlik_kilit:
            self._son_kilit_t = t
        if girdi.tespit_var:
            self._son_tespit_t = t
        kayip_sure = self._kilit_kayip_sure(t)

        # 2) ENGAGE'de kesintisiz sayaç sıfırlanışını EK logla (V_MAX_ENGAGE ayarı).
        if (self.state is State.ENGAGE and self._prev_kesintisiz > 0.0
                and girdi.kesintisiz_sn == 0.0):
            self._log_fn(
                f"[FSM] t={t:.3f} ENGAGE kesintisiz sifirlandi: "
                f"kesintisiz={self._prev_kesintisiz:.2f}s "
                f"kapanma={girdi.kapanma_hizi:.2f}m/s ah_oran={girdi.ah_oran:.3f} "
                f"sapma=({girdi.merkez_sapma_x:.2f},{girdi.merkez_sapma_y:.2f})")
        self._prev_kesintisiz = girdi.kesintisiz_sn

        # 3) guard bağlamı
        ctx = {
            "girdi": girdi, "pencere_ok": pencere_ok,
            "kesintisiz_ok": kesintisiz_ok, "dogrulandi": dogrulandi,
            "kayip_sure": kayip_sure,
        }

        # 4) Geçiş tablosunu öncelik sırasıyla değerlendir; ilk sağlanan geçiş yapılır.
        yapildi = False
        for guard_adi, hedef in self._TABLO[self.state]:
            if self._guard(guard_adi, ctx):
                self._gecis(guard_adi, hedef, t, girdi)
                yapildi = True
                break
            elif guard_adi in self._ILERLEME_GUARD:
                # İlerleme guard'ı sağlanmadı → reddi (throttled) logla.
                self._reddi_logla(guard_adi, ctx, t, girdi)

        # 5) Durumu güncelle/yayınla
        self._son = FSMDurum(
            state=self.state,
            kumulatif_sn=girdi.kumulatif_sn, kesintisiz_sn=girdi.kesintisiz_sn,
            pencere_ok=pencere_ok, kesintisiz_ok=kesintisiz_ok,
            tespit_dogrulandi=dogrulandi,
            kilit_kayip_sn=(0.0 if kayip_sure == float("inf") else kayip_sure),
            gecis_sayisi=self._gecis_sayisi,
        )
        return self.state

    @property
    def durum(self) -> FSMDurum:
        return self._son

    # ── guard'lar (her biri bool döndürür; komut ÜRETMEZ) ──
    def _guard(self, adi, ctx):
        g = ctx["girdi"]
        kayip = ctx["kayip_sure"]
        if adi == "hedef_ilk_tespit":
            return g.tespit_var
        if adi == "anlik_kilit_saglandi":
            return g.anlik_kilit
        if adi == "anlik_kilit_koptu":
            return not g.anlik_kilit
        if adi == "tespit_dogrulandi":
            return ctx["dogrulandi"] and g.anlik_kilit
        if adi == "kumulatif_5s":
            return ctx["pencere_ok"]
        if adi == "kesintisiz_3s":
            return ctx["kesintisiz_ok"]
        if adi == "kesintisiz_koptu":
            return not ctx["kesintisiz_ok"]
        if adi == "kilit_tamamen_kayip":
            return kayip > self.cfg.KILIT_KAYIP_SN
        if adi == "tespit_kayip":
            return self._tespit_kayip_sure(g.t) > self.cfg.KILIT_KAYIP_SN
        if adi == "yeniden_tespit":
            return g.tespit_var
        raise KeyError(f"bilinmeyen guard: {adi}")

    def _gecis(self, guard_adi, hedef, t, girdi):
        eski = self.state
        self.state = hedef
        self._gecis_sayisi += 1
        # Aşama-başına sıfırlamalar. NOT: kilit SÜRESİ (kümülatif/kesintisiz)
        # KilitTakip'te tutulur — FSM burada onu sıfırlamaz (tek kaynak). Kayan
        # pencere zaten doğal olarak düşürür; uzun kayıpta KilitTakip kendi sıfırlar.
        if hedef is State.TRACK_LOST:
            self._dogrula.reset()          # doğrulama yeniden birikir
            self._son_kilit_t = None
            self._prev_kesintisiz = 0.0
        elif hedef is State.DETECT and eski is State.APPROACH:
            # DETECT gerçek kapı: N-kareli doğrulama DETECT içinde ölçülsün.
            self._dogrula.reset()
        self._son_reject_sebep = None      # yeni state → reddi sıfırla
        self._log_gecis(t, eski, hedef, guard_adi, girdi)

    def _log_gecis(self, t, eski, yeni, guard, girdi):
        self._log_fn(
            f"[FSM] t={t:.3f} {eski.value}->{yeni.value} guard={guard} "
            f"kumulatif={girdi.kumulatif_sn:.2f}s kesintisiz={girdi.kesintisiz_sn:.2f}s "
            f"ah_oran={girdi.ah_oran:.3f}")

    def _reddi_logla(self, guard_adi, ctx, t, girdi):
        if guard_adi == "tespit_dogrulandi":
            sebep = "tespit dogrulanmadi"
        elif guard_adi == "kumulatif_5s":
            sebep = (f"kumulatif {girdi.kumulatif_sn:.2f}s "
                     f"< {self.sart.KUMULATIF_KILIT_SN:.2f}s")
        elif guard_adi == "kesintisiz_3s":
            sebep = (f"kesintisiz {girdi.kesintisiz_sn:.2f}s "
                     f"< {self.sart.KESINTISIZ_SN:.2f}s")
        else:
            return
        hedef_ad = dict(self._TABLO[self.state]).get(guard_adi)
        hedef_ad = hedef_ad.value if hedef_ad else guard_adi
        # Throttle: aynı sebep 30 Hz basılmasın; sebep değişince ya da
        # reject_log_dt geçince bir kez logla.
        if (sebep != self._son_reject_sebep
                or (t - self._son_reject_t) >= self._reject_log_dt):
            self._log_fn(
                f"[FSM] t={t:.3f} {hedef_ad} reddedildi: {sebep} "
                f"(durum {self.state.value}) ah_oran={girdi.ah_oran:.3f}")
            self._son_reject_t = t
            self._son_reject_sebep = sebep
