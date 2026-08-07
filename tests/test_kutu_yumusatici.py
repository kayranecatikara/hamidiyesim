"""AH kutusu zamansal yumuşatıcı testleri (-p no:anyio)."""

from config.kilit_sabitler import AYAR
from vision.kutu_yumusatici import KutuYumusatici

DT = 1.0 / 30.0


def test_ilk_kare_dogrudan():
    y = KutuYumusatici()
    assert y.yumusat((100, 100, 140, 130), DT) == (100, 100, 140, 130)


def test_ema_konumu_yumusatir():
    y = KutuYumusatici()
    y.yumusat((100, 100, 140, 130), DT)
    # ani 40 px kayma → çıktı kısmen hareket eder (EMA), tamamı değil
    out = y.yumusat((140, 100, 180, 130), DT)
    assert 100 < out[0] < 140                     # x1 arada kaldı (yumuşadı)


def test_konum_zamanla_yakinsar():
    y = KutuYumusatici()
    y.yumusat((100, 100, 140, 130), DT)
    out = None
    for _ in range(60):                           # sabit hedefe uzun besle
        out = y.yumusat((140, 100, 180, 130), DT)
    assert abs(out[0] - 140) <= AYAR.KUTU_OLU_BOLGE_PX  # ölü bölge içinde oturdu


def test_olu_bolge_kucuk_gurultuyu_dondurur():
    # Sabit hedef + ±1 px gürültü → çıktı kutusu HİÇ değişmemeli (taş gibi).
    y = KutuYumusatici()
    ilk = y.yumusat((300, 200, 340, 230), DT)
    for _ in range(40):
        y.yumusat((300, 200, 340, 230), DT)       # otur
    kararli = y.yumusat((300, 200, 340, 230), DT)
    ciktilar = set()
    for dx, dy in [(1, 0), (0, -1), (-1, 1), (1, 1), (0, 0), (-1, 0)]:
        ciktilar.add(y.yumusat((300 + dx, 200 + dy, 340 + dx, 230 + dy), DT))
    assert len(ciktilar) == 1                      # sub-eşik gürültüde tek (sabit) kutu


def test_boyut_sicramasi_reddedilir():
    y = KutuYumusatici()
    y.yumusat((100, 100, 140, 130), DT)           # genişlik 40
    # 4× genişlik (sahte dev kutu) → reddedilir, önceki korunur
    out = y.yumusat((100, 100, 260, 130), DT)     # genişlik 160 = 4×
    assert out == (100, 100, 140, 130)


def test_tespit_yok_sifirlar():
    y = KutuYumusatici()
    y.yumusat((100, 100, 140, 130), DT)
    assert y.yumusat(None, DT) is None
    # sıfırlandı → sonraki kutu yeniden doğrudan alınır
    assert y.yumusat((200, 200, 240, 230), DT) == (200, 200, 240, 230)
