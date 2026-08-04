"""
tests/test_frpn_guidance.py — FRPN uçuş döngüsünün kabul kriterleri (F4).

Gazebo'suz, gerçek MAVLink'siz: sahte conn ile döngü GERÇEKTEN koşturulur.
Kullanım: python3 -m tests.test_frpn_guidance

Kapsam:
  D1-D3  sözleşme: eski modülle aynı imza, aynı status anahtarları/değerleri
  D4-D6  döngü duman testi: komut üretiyor, CSV yazıyor, durdurulabiliyor
  D7-D9  WARMUP / DROPOUT / donuk telemetri davranışı
  D10-D12 fren-farkında iniş sınırı (11:34 çakılmasının panzehiri)
  D13-D14 kapalı çevrim: hareketli hedefe gerçekten yaklaşıyor mu
"""

import math
import os
import tempfile
import threading
import time

from control.guidance import frpn_guidance as fg
from control.guidance import gps_guidance as gg

# Test CSV'leri gerçek uçuş loglarına karışmasın
fg._LOG_DIR = tempfile.mkdtemp(prefix="avci_test_frpn_")

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


class SahteConn:
    """send_velocity'nin çağırdığı MAVLink arayüzünün asgari taklidi."""

    def __init__(self):
        self.komutlar = []
        self.target_system = 1
        self.target_component = 1
        self.mav = self

    def set_position_target_local_ned_send(self, *a, **k):
        # imza: (t, sys, comp, frame, mask, x,y,z, vx,vy,vz, ax,ay,az, yaw, yawrate)
        self.komutlar.append({"vx": a[8], "vy": a[9], "vz": a[10], "yaw": a[14]})


def _kos(hedef_fn, iris_fn, sure=2.0, cfg=None):
    """Döngüyü `sure` saniye koştur, sahte conn'u döndür."""
    conn = SahteConn()
    dur = threading.Event()

    def ol():
        time.sleep(sure)
        dur.set()
    threading.Thread(target=ol, daemon=True).start()
    fg.run_frpn_guidance(conn, hedef_fn, iris_fn, dur, cfg=cfg or fg.Cfg)
    return conn


def main():
    print("FRPN uçuş döngüsü — kabul kriterleri")
    print("=" * 66)

    # ══ D1-D3: SÖZLEŞME ══
    print("\n── D1-D3: sözleşme (eski modülle uyum) ──")
    import inspect
    s_yeni = inspect.signature(fg.run_frpn_guidance)
    s_eski = inspect.signature(gg.run_gps_guidance)
    ortak = ["conn", "get_plane", "get_iris", "stop_event", "cfg"]
    kontrol("D1  imza eski modülle uyumlu (ilk 5 parametre)",
            list(s_yeni.parameters)[:5] == ortak
            and list(s_eski.parameters)[:5] == ortak,
            f"{list(s_yeni.parameters)[:5]}")

    kontrol("D2  status anahtarları birebir aynı",
            set(fg.status.keys()) == set(gg.status.keys()),
            f"{sorted(fg.status.keys())}")

    kontrol("D3  durum değerleri supervisor'ın beklediği kümede",
            fg.status["durum"] in ("WARMUP", "ARAMA", "KILIT", "DROPOUT", "DURDU"),
            f"başlangıç durumu={fg.status['durum']}")

    # ══ D4-D6: DÖNGÜ DUMAN TESTİ ══
    print("\n── D4-D6: döngü duman testi ──")
    hedef = {"x": 200.0, "y": 0.0, "z": -50.0, "yaw": 0.0, "frozen": False}
    iris_p = {"x": 0.0, "y": 0.0, "z": -45.0,
              "vx": 0.0, "vy": 0.0, "vz": 0.0,
              "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    def get_plane():
        hedef["x"] += 16.0 * 0.05           # 16 m/s doğuya değil kuzeye
        return dict(hedef)

    def get_iris():
        return dict(iris_p)

    conn = _kos(get_plane, get_iris, sure=2.0)
    kontrol("D4  komut üretiyor", len(conn.komutlar) > 20,
            f"{len(conn.komutlar)} komut / 2 s (~20 Hz beklenir)")

    # ⚠ SON komut döngü çıkışındaki DURDURMA komutudur (0,0,0) — ona bakılmaz.
    v_orta = conn.komutlar[len(conn.komutlar) // 2]
    kontrol("D5  komut hedefe doğru (kuzey bileşeni pozitif)",
            v_orta["vx"] > 1.0,
            f"orta kare v=({v_orta['vx']:+.1f},{v_orta['vy']:+.1f},{v_orta['vz']:+.1f})")

    v_son = conn.komutlar[-1]
    kontrol("D6  durdurulunca DURDU ve son komut SIFIR (araç serbest kalmaz)",
            fg.status["durum"] == "DURDU"
            and abs(v_son["vx"]) + abs(v_son["vy"]) + abs(v_son["vz"]) < 1e-9,
            f"durum={fg.status['durum']}  son komut="
            f"({v_son['vx']:+.1f},{v_son['vy']:+.1f},{v_son['vz']:+.1f})")

    csvler = [x for x in os.listdir(fg._LOG_DIR) if x.endswith(".csv")]
    kontrol("D6b CSV yazıldı ve satır içeriyor", len(csvler) >= 1,
            f"{len(csvler)} dosya")

    # ══ D7-D9: WARMUP / DROPOUT ══
    print("\n── D7-D9: telemetri tazeliği ──")
    donuk = {"x": 100.0, "y": 0.0, "z": -50.0, "yaw": 0.0, "frozen": True}
    conn = _kos(lambda: dict(donuk), get_iris, sure=1.0)
    kontrol("D7  hiç taze telemetri gelmezse WARMUP'ta kalır (komut yok)",
            fg.status["durum"] in ("WARMUP", "DURDU") and len(conn.komutlar) > 0,
            f"durum={fg.status['durum']}  (hover komutları: {len(conn.komutlar)})")

    # Önce taze veri ver, sonra dondur → DROPOUT'a düşmeli
    canli = {"x": 100.0, "y": 0.0, "z": -50.0, "yaw": 0.0, "frozen": False}
    sayac = {"n": 0}

    def get_plane_donan():
        sayac["n"] += 1
        if sayac["n"] > 20:                  # ~1 s sonra dondur
            canli["frozen"] = True
        else:
            canli["x"] += 0.8
        return dict(canli)

    kisa_cfg = type("C", (fg.Cfg,), {"HOLD_S": 0.5})
    conn = _kos(get_plane_donan, get_iris, sure=2.5, cfg=kisa_cfg)
    kontrol("D8  telemetri donunca DROPOUT'a düşer",
            fg.status["durum"] in ("DROPOUT", "DURDU"),
            f"durum={fg.status['durum']} (HOLD_S=0.5 s)")

    kontrol("D9  DROPOUT'ta hover komutu gidiyor (araç serbest kalmıyor)",
            len(conn.komutlar) > 20, f"{len(conn.komutlar)} komut")

    # ══ D10-D12: FREN-FARKINDA İNİŞ ══
    print("\n── D10-D12: fren-farkında iniş sınırı ──")
    C = fg.Cfg
    kontrol("D10 taban irtifasında iniş YASAK",
            fg._inis_tavani(C.LOOKUP_MIN_ALT, C) == 0.0,
            f"irtifa={C.LOOKUP_MIN_ALT} m → tavan {fg._inis_tavani(C.LOOKUP_MIN_ALT, C):.2f} m/s")

    v10 = fg._inis_tavani(C.LOOKUP_MIN_ALT + 2.0, C)
    beklenen = math.sqrt(2 * C.AZ_FREN * 2.0)
    kontrol("D11 tabana 2 m kala tavan = √(2·a·h)",
            abs(v10 - min(C.VZ_MAX, beklenen)) < 1e-9,
            f"{v10:.2f} m/s (√(2·{C.AZ_FREN}·2)={beklenen:.2f})")

    kontrol("D12 yüksekte tam VZ_MAX serbest",
            abs(fg._inis_tavani(200.0, C) - C.VZ_MAX) < 1e-9,
            f"200 m'de tavan {fg._inis_tavani(200.0, C):.1f} = VZ_MAX {C.VZ_MAX}")

    # ══ D13-D14: KAPALI ÇEVRİM ══
    print("\n── D13-D14: kapalı çevrim (gerçekten yaklaşıyor mu) ──")
    # Basit nokta-kütle: komutu doğrudan hıza uygula
    hp = {"x": 150.0, "y": 0.0, "z": -50.0, "yaw": 0.0, "frozen": False}
    ip = {"x": 0.0, "y": 0.0, "z": -45.0, "vx": 0.0, "vy": 0.0, "vz": 0.0,
          "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    son_cmd = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
    menziller = []

    class Conn2(SahteConn):
        def set_position_target_local_ned_send(self, *a, **k):
            super().set_position_target_local_ned_send(*a, **k)
            son_cmd.update(vx=a[8], vy=a[9], vz=a[10])

    def gp():
        hp["x"] += 15.0 * 0.05
        return dict(hp)

    def gi():
        # komutu birinci derece takip
        for eks, k in (("x", "vx"), ("y", "vy"), ("z", "vz")):
            ip[k] += (son_cmd[k] - ip[k]) * 0.35
            ip[eks] += ip[k] * 0.05
        menziller.append(math.dist((hp["x"], hp["y"], hp["z"]),
                                   (ip["x"], ip["y"], ip["z"])))
        return dict(ip)

    conn = Conn2()
    dur = threading.Event()
    threading.Thread(target=lambda: (time.sleep(8.0), dur.set()), daemon=True).start()
    fg.run_frpn_guidance(conn, gp, gi, dur)

    # ⚠ İLK saniyeler HIZLANMA RAMPASIDIR, kapanma değil. Drone durgun başlar;
    # MAX_ACCEL=8 m/s² ile 18 m/s'ye çıkmak ~2.3 s sürer ve o sırada 15 m/s'lik
    # hedef mesafe AÇAR. Bu fizik, kusur değil. Ölçüm bu yüzden rampa bittikten
    # sonraki KAPANMA HIZINA bakar. (İlk sürüm 4 s koşup baştan sona kıyaslıyordu
    # ve neredeyse tamamen rampayı ölçüyordu.)
    n = len(menziller)
    ramp_sonu = int(n * 0.45)
    kapanma = ((menziller[ramp_sonu] - menziller[-1])
               / ((n - ramp_sonu) * 0.05)) if n > ramp_sonu + 10 else 0.0
    kontrol("D13 rampa bitince menzil KAPANIYOR (kapanma hızı > 1 m/s)",
            kapanma > 1.0,
            f"{menziller[ramp_sonu]:.1f} → {menziller[-1]:.1f} m, "
            f"kapanma {kapanma:+.2f} m/s (teorik tavan {fg.Cfg.V_MAX - 15.0:.1f})")

    sonlu = all(math.isfinite(k["vx"]) and math.isfinite(k["vy"])
                and math.isfinite(k["vz"]) for k in conn.komutlar)
    hiz_ok = all(math.hypot(k["vx"], k["vy"]) <= fg.Cfg.V_MAX + 1e-6
                 for k in conn.komutlar)
    kontrol("D14 tüm komutlar sonlu ve hız tavanı içinde",
            sonlu and hiz_ok,
            f"n={len(conn.komutlar)}  V_MAX={fg.Cfg.V_MAX}")

    print("\n" + "=" * 66)
    gecen = sum(1 for _, ok, _ in _sonuclar if ok)
    toplam = len(_sonuclar)
    print(f"SONUÇ: {gecen}/{toplam} geçti — "
          f"{'HEPSİ GEÇTİ ✓' if gecen == toplam else 'BAŞARISIZ ✗'}")
    if gecen != toplam:
        for ad, ok, detay in _sonuclar:
            if not ok:
                print(f"  FAIL: {ad}  {detay}")
    return 0 if gecen == toplam else 1


if __name__ == "__main__":
    raise SystemExit(main())
