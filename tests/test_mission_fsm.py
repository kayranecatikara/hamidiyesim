"""Görev FSM birim testleri (-p no:anyio).

Sahte tespit/kilit dizileriyle FSM sürülür; hiçbir aşama atlanamaz kuralı ve
zaman kapıları (5 sn kümülatif, 3 sn kesintisiz, X sn kilit kaybı) doğrulanır.

FSM kilit SÜRESİNİ tutmaz — üretimde KilitTakip'ten (KilitSure) gelir. Test de
FSM'i eşlenik bir KilitSure ile süren Surucu üzerinden çalışır (üretimle birebir).
"""

from config.kilit_sabitler import SARTNAME, AYAR
from control.mission_fsm import GorevFSM, Girdi, State
from control.kilit_sure import KilitSure

DT = 0.05


class Surucu:
    """FSM + eşlenik KilitSure. step() ikisini birlikte sürer: KilitSure süreyi
    hesaplar, FSM'e Girdi ile verilir (gcs_server + KilitTakip ile aynı yol)."""

    def __init__(self, reject_log_dt=0.0):
        self.kayit = []
        self.fsm = GorevFSM(log_fn=self.kayit.append, reject_log_dt=reject_log_dt)
        self.sure = KilitSure()

    def step(self, t, tespit, kilit, **obs):
        sd = self.sure.guncelle(t, kilit)
        return self.fsm.step(Girdi(
            t=t, tespit_var=tespit, anlik_kilit=kilit,
            kumulatif_sn=sd.kumulatif_sn, kesintisiz_sn=sd.kesintisiz_sn, **obs))

    @property
    def state(self):
        return self.fsm.state

    @property
    def durum(self):
        return self.fsm.durum


def yeni_fsm():
    return Surucu()


def besle(s, t0, sure, tespit, kilit, **obs):
    """[t0, t0+sure] aralığında DT adımlarla sabit (tespit, kilit) besler."""
    n = max(1, int(round(sure / DT)))
    t = t0
    son = s.state
    for i in range(1, n + 1):
        t = round(t0 + i * DT, 6)
        son = s.step(t, tespit, kilit, **obs)
    return t, son


def kilitle_kumulatife(s, hedef_kumulatif, t0=0.0):
    """Sürekli kilit besleyerek kümülatif ~hedef_kumulatif olana dek sürer."""
    t = t0
    while s.durum.kumulatif_sn < hedef_kumulatif:
        t = round(t + DT, 6)
        s.step(t, True, True)
        if t > t0 + 60:
            break
    return t


# ── Senaryo 1: Tespit anında STRIKE'a (hatta ENGAGE'e) geçilemez ──
def test_tespit_aninda_strike_yok():
    s = yeni_fsm()
    s.step(DT, True, True)                     # ilk kare → SEARCH→APPROACH
    assert s.state is State.APPROACH
    gorulen = set()
    t = DT
    for _ in range(20):                        # ~1.0 sn kusursuz kilit
        t = round(t + DT, 6)
        gorulen.add(s.step(t, True, True))
    assert State.STRIKE not in gorulen
    assert State.ENGAGE not in gorulen
    assert State.DETECT in gorulen and State.TRACK_LOCK in gorulen


# ── Senaryo 2: 4.9 sn kümülatifte ENGAGE yok, 5.0 sn'de var ──
def test_kumulatif_5s_kapisi():
    s = yeni_fsm()
    besle(s, 0.0, 4.90, True, True)
    assert s.state is State.TRACK_LOCK
    assert s.durum.kumulatif_sn < SARTNAME.KUMULATIF_KILIT_SN
    assert any("reddedildi: kumulatif" in x for x in s.kayit)
    kilitle_kumulatife(s, SARTNAME.KUMULATIF_KILIT_SN, t0=4.90)
    assert s.state is State.ENGAGE


# ── Senaryo 3: ENGAGE içinde 2.9 sn kesintisizde STRIKE yok, 3.0'da var ──
def test_kesintisiz_3s_kapisi():
    s = yeni_fsm()
    t, _ = besle(s, 0.0, 4.90, True, True)
    assert s.state is State.TRACK_LOCK
    t, _ = besle(s, t, 0.5, False, False)      # kesintisizi kır (kümülatif korunur)
    assert s.state is State.TRACK_LOCK          # TRACK_LOST'a düşmedi
    while s.state is not State.ENGAGE:          # kümülatif 5'i geçince ENGAGE
        t = round(t + DT, 6)
        s.step(t, True, True)
        assert t < 8.0
    assert s.durum.kesintisiz_sn < 0.5          # ENGAGE girişinde kesintisiz taze
    gordu_29 = False
    while s.state is not State.STRIKE:
        ks = s.durum.kesintisiz_sn
        assert s.state is State.ENGAGE          # STRIKE ancak >=3 sn'de
        if 2.80 <= ks < SARTNAME.KESINTISIZ_SN:
            gordu_29 = True
        t = round(t + DT, 6)
        s.step(t, True, True)
        assert t < 15.0
    assert s.durum.kesintisiz_sn >= SARTNAME.KESINTISIZ_SN - 1e-6
    assert gordu_29
    assert any("reddedildi: kesintisiz" in x for x in s.kayit)


# ── Senaryo 3b: STRIKE için İKİSİ BİRDEN dolu olmalı (kümülatif>=5 VE kesintisiz>=3) ──
def test_strike_ikisi_birden_dolu_olmali():
    """ENGAGE'de kayan 10 sn penceresi kümülatifi 5'in ALTINA düşürürse, kesintisiz
    3'ü geçse bile STRIKE OLMAZ. Kümülatif tekrar 5'i bulunca (kesintisiz hâlâ >=3)
    STRIKE olur. 3 sn, 5 sn'nin içinde olabilir; ama vuruş anında ikisi de dolu olmalı.

    Süreler doğrudan Girdi'ye verilir (kayan-pencere kenar durumunu KilitSure'dan
    üretmek yerine deterministik kur)."""
    fsm = GorevFSM(log_fn=lambda s: None, reject_log_dt=0.0)
    t = kum = kes = 0.0
    while fsm.state is not State.ENGAGE:            # sürekli kilitle ENGAGE'e getir
        t = round(t + DT, 6); kum = round(kum + DT, 6); kes = round(kes + DT, 6)
        fsm.step(Girdi(t=t, tespit_var=True, anlik_kilit=True,
                       kumulatif_sn=kum, kesintisiz_sn=kes))
        assert t < 8.0
    # ENGAGE'deyiz. Kümülatif<5 (pencere düştü), kesintisiz>=3 → STRIKE YOK.
    t = round(t + DT, 6)
    fsm.step(Girdi(t=t, tespit_var=True, anlik_kilit=True,
                   kumulatif_sn=4.80, kesintisiz_sn=3.50))
    assert fsm.state is State.ENGAGE
    # Ters durum: kümülatif>=5 ama kesintisiz<3 → yine STRIKE YOK.
    t = round(t + DT, 6)
    fsm.step(Girdi(t=t, tespit_var=True, anlik_kilit=True,
                   kumulatif_sn=5.50, kesintisiz_sn=2.90))
    assert fsm.state is State.ENGAGE
    # İkisi de dolu → STRIKE.
    t = round(t + DT, 6)
    fsm.step(Girdi(t=t, tespit_var=True, anlik_kilit=True,
                   kumulatif_sn=5.10, kesintisiz_sn=3.60))
    assert fsm.state is State.STRIKE


def _strike_e_getir(s):
    t = kilitle_kumulatife(s, 4.55)
    t, _ = besle(s, t, 0.5, False, False)                              # kesintisizi kır
    t = kilitle_kumulatife(s, SARTNAME.KUMULATIF_KILIT_SN, t0=t)       # → ENGAGE
    t, _ = besle(s, t, 3.2, True, True)                               # kesintisiz 3s → STRIKE
    assert s.state is State.STRIKE
    return t


# ── Senaryo 4: STRIKE COMMIT — kısa kilit kaybında iptal OLMAZ (dalış sürer) ──
def test_strike_commit_kisa_kayipta_iptal_olmaz():
    s = yeni_fsm()
    t = _strike_e_getir(s)
    # Yakın mesafede kilit titrer (merkez AV'den çıkar) — STRIKE COMMIT, iptal yok.
    t, _ = besle(s, t, 0.5, False, False)
    assert s.state is State.STRIKE             # dalış taahhüt edildi
    # Ancak TAM kayıp (> X sn = hedef tamamen kaçtı) STRIKE'ı bozar → TRACK_LOST.
    t, _ = besle(s, t, AYAR.KILIT_KAYIP_SN + 0.6, False, False)
    assert s.state is State.TRACK_LOST


# ── Senaryo 5: kilit tamamen kaybolursa TRACK_LOST; sonra akış baştan (STRIKE atlanmaz) ──
def test_kilit_kaybi_ve_yeniden_akis():
    s = yeni_fsm()
    t = _strike_e_getir(s)
    t, _ = besle(s, t, AYAR.KILIT_KAYIP_SN + 0.6, False, False)   # > X kilit kaybı
    assert s.state is State.TRACK_LOST
    # Yeniden tespit → APPROACH (asla doğrudan STRIKE).
    t = round(t + DT, 6)
    s.step(t, True, True)
    assert s.state is State.APPROACH
    # Kayıp KESİNTİSİZİ sıfırladı → STRIKE için yeniden 3 sn kesintisiz gerekir.
    # Kısa kilitte STRIKE'a ASLA atlanmaz (aşama atlama yok).
    gorulen = set()
    t2, _ = besle(s, t, 0.6, True, True)
    for _ in range(int(0.6 / DT)):
        gorulen.add(s.state)
    assert s.state is not State.STRIKE
    assert s.durum.kesintisiz_sn < SARTNAME.KESINTISIZ_SN


# ── APPROACH, tespit VARKEN (henüz kilit yok) SEARCH'e titremez ──
def test_approach_tespit_varken_search_e_titremez():
    s = yeni_fsm()
    s.step(DT, True, True)
    assert s.state is State.APPROACH
    gorulen = set()
    t = DT
    for _ in range(int(6.0 / DT)):             # X'in çok üstünde, 6 sn tespit VAR kilit YOK
        t = round(t + DT, 6)
        gorulen.add(s.step(t, True, False))
    assert gorulen == {State.APPROACH}         # SEARCH'e hiç düşmedi
    t2, son = besle(s, t, AYAR.KILIT_KAYIP_SN + 0.5, False, False)
    assert s.state is State.SEARCH             # tespit gerçekten kesilince SEARCH


# ── Geçiş tablosu bütünlük: STRIKE'a giden tek yol ENGAGE ──
def test_strike_yalniz_engage_ustunden():
    hedefler = [(st, g, h) for st, gecisler in GorevFSM._TABLO.items()
                for g, h in gecisler]
    strike_kaynaklari = {st for (st, g, h) in hedefler if h is State.STRIKE}
    assert strike_kaynaklari == {State.ENGAGE}
    assert all(h is not State.STRIKE
               for (st, g, h) in hedefler if st is State.TRACK_LOST)
