"""Görev FSM birim testleri (-p no:anyio).

Sahte tespit/kilit dizileriyle FSM sürülür; hiçbir aşama atlanamaz kuralı ve
zaman kapıları (5 sn kümülatif, 3 sn kesintisiz, X sn kilit kaybı) doğrulanır.
Zaman sim monotonik saattir; kare adımı DT (< ORNEK_TAVAN_SN).
"""

from config.kilit_sabitler import SARTNAME, AYAR
from control.mission_fsm import GorevFSM, Girdi, State

DT = 0.05


def yeni_fsm():
    """Reddedilen geçiş loglarını da yakalamak için throttle'sız FSM."""
    kayit = []
    fsm = GorevFSM(log_fn=kayit.append, reject_log_dt=0.0)
    return fsm, kayit


def besle(fsm, t0, sure, tespit, kilit, **obs):
    """[t0, t0+sure] aralığında DT adımlarla sabit (tespit, kilit) besler.
    Dönüş: (son_t, son_state)."""
    n = max(1, int(round(sure / DT)))
    t = t0
    son = fsm.state
    for i in range(1, n + 1):
        t = round(t0 + i * DT, 6)
        son = fsm.step(Girdi(t=t, tespit_var=tespit, anlik_kilit=kilit, **obs))
    return t, son


def kilitle_kumulatife(fsm, hedef_kumulatif, t0=0.0):
    """Sürekli kilit besleyerek kümülatif ~hedef_kumulatif olana dek sürer.
    Dönüş: son_t. (t≈kümülatif çünkü t0'dan sürekli kilit.)"""
    t = t0
    while fsm.durum.kumulatif_sn < hedef_kumulatif:
        t = round(t + DT, 6)
        fsm.step(Girdi(t=t, tespit_var=True, anlik_kilit=True))
        if t > t0 + 60:
            break
    return t


# ── Senaryo 1: Tespit anında STRIKE'a (hatta ENGAGE'e) geçilemez ──
def test_tespit_aninda_strike_yok():
    fsm, _ = yeni_fsm()
    # İlk kare tespit+kilit: yalnız SEARCH→APPROACH olmalı.
    fsm.step(Girdi(t=DT, tespit_var=True, anlik_kilit=True))
    assert fsm.state is State.APPROACH
    # 1 sn boyunca kusursuz kilit versek bile ENGAGE/STRIKE görülmez.
    gorulen = set()
    t = DT
    for _ in range(20):                       # ~1.0 sn
        t = round(t + DT, 6)
        gorulen.add(fsm.step(Girdi(t=t, tespit_var=True, anlik_kilit=True)))
    assert State.STRIKE not in gorulen
    assert State.ENGAGE not in gorulen
    # Aşamalar sırayla yürüdü: DETECT ve TRACK_LOCK görülmeli.
    assert State.DETECT in gorulen and State.TRACK_LOCK in gorulen


# ── Senaryo 2: 4.9 sn kümülatifte ENGAGE yok, 5.0 sn'de var ──
def test_kumulatif_5s_kapisi():
    fsm, kayit = yeni_fsm()
    # 4.90 sn sürekli kilit → hâlâ TRACK_LOCK (kümülatif < 5).
    besle(fsm, 0.0, 4.90, True, True)
    assert fsm.state is State.TRACK_LOCK
    assert fsm.durum.kumulatif_sn < SARTNAME.KUMULATIF_KILIT_SN
    # Reddedilen kümülatif geçişi loglanmış olmalı.
    assert any("reddedildi: kumulatif" in s for s in kayit)
    # Kilidi sürdür → kümülatif 5.0'ı geçince ENGAGE.
    kilitle_kumulatife(fsm, SARTNAME.KUMULATIF_KILIT_SN, t0=4.90)
    assert fsm.state is State.ENGAGE


# ── Senaryo 3: ENGAGE içinde 2.9 sn kesintisizde STRIKE yok, 3.0'da var ──
def test_kesintisiz_3s_kapisi():
    fsm, kayit = yeni_fsm()
    # Kümülatifi ~4.90 sn'ye sürekli kilitle taşı (henüz ENGAGE değil, TRACK_LOCK).
    t, _ = besle(fsm, 0.0, 4.90, True, True)
    assert fsm.state is State.TRACK_LOCK
    # Kısa boşluk: kesintisiz segmenti öldür (kümülatif 4.90 korunur, < X=2 sn).
    t, _ = besle(fsm, t, 0.5, False, False)
    assert fsm.state is State.TRACK_LOCK           # TRACK_LOST'a düşmedi
    # Yeniden kilit: kümülatif 5'i az sonra geçer → ENGAGE, kesintisiz TAZE (~0).
    while fsm.state is not State.ENGAGE:
        t = round(t + DT, 6)
        fsm.step(Girdi(t, True, True))
        assert t < 8.0
    assert fsm.durum.kesintisiz_sn < 0.5           # segment ENGAGE girişinde taze
    # Kesintisiz 3.0'a çıkana dek besle: 3 sn altında ASLA STRIKE olmamalı.
    gordu_29 = False
    while fsm.state is not State.STRIKE:
        ks = fsm.durum.kesintisiz_sn
        assert fsm.state is State.ENGAGE           # STRIKE ancak >=3 sn'de
        if 2.80 <= ks < SARTNAME.KESINTISIZ_SN:
            gordu_29 = True                        # 2.9 sn'de hâlâ ENGAGE
        t = round(t + DT, 6)
        fsm.step(Girdi(t, True, True))
        assert t < 15.0
    # STRIKE anında kesintisiz gerçekten >= 3.0.
    assert fsm.durum.kesintisiz_sn >= SARTNAME.KESINTISIZ_SN - 1e-6
    assert gordu_29
    assert any("reddedildi: kesintisiz" in s for s in kayit)


def _strike_e_getir(fsm):
    """FSM'i STRIKE durumuna taşır, son t'yi döndürür."""
    t = kilitle_kumulatife(fsm, 4.55)
    t, _ = besle(fsm, t, 0.5, False, False)         # kesintisizi kır
    t = kilitle_kumulatife(fsm, SARTNAME.KUMULATIF_KILIT_SN, t0=t)  # → ENGAGE
    t, _ = besle(fsm, t, 3.2, True, True)           # kesintisiz 3s → STRIKE
    assert fsm.state is State.STRIKE
    return t


# ── Senaryo 4: STRIKE'ta kilit koparsa STRIKE iptal → ENGAGE ──
def test_strike_iptal_kesintisiz_kopunca():
    fsm, _ = yeni_fsm()
    t = _strike_e_getir(fsm)
    # Kısa kayıp (< X): kesintisiz segment ölür → STRIKE iptal, ENGAGE'e döner.
    t, _ = besle(fsm, t, 0.5, False, False)
    assert fsm.state is State.ENGAGE               # STRIKE'a değil, TRACK_LOST'a da değil


# ── Senaryo 5: kilit tamamen kaybolursa TRACK_LOST; sonra akış baştan ──
def test_kilit_kaybi_ve_yeniden_akis():
    fsm, _ = yeni_fsm()
    t = _strike_e_getir(fsm)
    # X sn'den uzun kilit kaybı → TRACK_LOST (STRIKE'tan asla ENGAGE atlayıp değil).
    t, _ = besle(fsm, t, AYAR.KILIT_KAYIP_SN + 0.6, False, False)
    assert fsm.state is State.TRACK_LOST
    assert fsm.durum.kumulatif_sn == 0.0           # taze başladı
    # Yeniden tespit → APPROACH (asla doğrudan STRIKE).
    fsm.step(Girdi(t=round(t + DT, 6), tespit_var=True, anlik_kilit=True))
    assert fsm.state is State.APPROACH
    # Akış baştan: kısa kilitte DETECT/TRACK_LOCK; ENGAGE/STRIKE için yeniden 5 sn gerek.
    t2, _ = besle(fsm, round(t + DT, 6), 0.6, True, True)
    assert fsm.state in (State.DETECT, State.TRACK_LOCK)
    assert fsm.state not in (State.ENGAGE, State.STRIKE)


# ── APPROACH, tespit VARKEN (henüz kilit yok) SEARCH'e titremez ──
def test_approach_tespit_varken_search_e_titremez():
    fsm, _ = yeni_fsm()
    fsm.step(Girdi(t=DT, tespit_var=True, anlik_kilit=True))
    assert fsm.state is State.APPROACH
    # Uzun süre tespit VAR ama anlık kilit YOK (yaklaşma sürüyor, hedef küçük):
    # eskiden kilit_kayip>X ile APPROACH↔SEARCH titriyordu. Artık APPROACH kalır.
    gorulen = set()
    t = DT
    for _ in range(int(6.0 / DT)):                 # X=2 sn'in çok üstünde, 6 sn
        t = round(t + DT, 6)
        gorulen.add(fsm.step(Girdi(t=t, tespit_var=True, anlik_kilit=False)))
    assert gorulen == {State.APPROACH}             # SEARCH'e hiç düşmedi
    # Tespit gerçekten kesilirse (X sn) SEARCH'e döner.
    t2, son = besle(fsm, t, AYAR.KILIT_KAYIP_SN + 0.5, False, False)
    assert fsm.state is State.SEARCH


# ── Geçiş tablosu bütünlük: STRIKE'a giden tek yol ENGAGE ──
def test_strike_yalniz_engage_ustunden():
    hedefler = [(s, g, h) for s, gecisler in GorevFSM._TABLO.items()
                for g, h in gecisler]
    strike_kaynaklari = {s for (s, g, h) in hedefler if h is State.STRIKE}
    assert strike_kaynaklari == {State.ENGAGE}
    # TRACK_LOST'tan STRIKE'a doğrudan geçiş YOK.
    assert all(h is not State.STRIKE
               for (s, g, h) in hedefler if s is State.TRACK_LOST)
