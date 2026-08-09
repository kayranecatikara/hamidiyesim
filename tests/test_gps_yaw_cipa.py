"""
tests/test_gps_yaw_cipa.py — GPS yaw ÇIPALAMA deney kolunun kabul kriterleri.

Kullanım: PYTHONPATH=. python3 tests/test_gps_yaw_cipa.py
(pytest bu dosyayı ÇALIŞTIRMAZ — main() elle çağrılmalı; bkz. TODO'daki
"pytest asıl kontrolleri koşmuyor" notu.)

NEDEN AYRI DOSYA: `tests/test_gps_guidance.py` iş bölümü gereği Kayra'nın
alanı ve `origin/kayramin_super_gudumu` ile birebir aynı tutuluyor. Bu deney
kolunun testleri oraya yazılsa merge'de çakışırdı.

KAPSAM:
  Y1  KAPALIYKEN kod yolu Kayra'nınkiyle aynı — cmd_yaw yalnız bearing'i
      kovalar, aracın gerçek başlığından ETKİLENMEZ
  Y2  AÇIKKEN komut gerçek başlığa demirlenir (actual + en çok bir adım)
  Y3  Adım tavanı: |cmd − actual| hiçbir karede YAW_RATE_MAX·dt'yi aşmaz
  Y4  Ölü bant: hata ≤ YAW_DEADBAND iken komut = actual (adım 0)
  Y5  Süreklı doygunluk kapısı: araç hiç dönmezse YAW_DOYGUN_N kare sonra
      yaw susturulur
  Y6  SÜRELİ SUSMA (08-06 kilitlenme düzeltmesi): susma sonsuza kadar
      sürmez, YAW_SUS_N kare sonra yetki geri verilir
  Y7  Meşru büyük dönüş SUSTURULMAZ: hata kapanıyorsa doygunluk sayacı sıfırlanır
  Y8  Döngü duman testi: iki kol da komut üretir, çıpalama kolunda komut
      aracın başlığına yapışık kalır
"""

import math
import os
import threading
import time

# Test CSV'leri gerçek uçuş loglarına karışmasın
import tempfile as _tf

from control.guidance import gps_guidance as gg
from control.guidance.common import clamp, normalize_angle

gg._LOG_DIR = _tf.mkdtemp(prefix="avci_test_yawcipa_")

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


class _Cfg(gg.Cfg):
    """Deney kolu açık bir kopya — env'e dokunmadan A/B kurulabilsin."""
    YAW_CIPA = True


# ── Güdümdeki yaw bloğunun BİREBİR aynısı, tek karelik saf fonksiyon olarak.
# Döngünün tamamını çalıştırmadan mantığı sınamak için; blok değişirse burası
# da değişmeli (Y8 duman testi ikisinin uyumunu uçtan uca doğrular).
class _YawSim:
    def __init__(self, cfg):
        self.cfg = cfg
        self.cmd_yaw = None
        self.yaw_doygun_n = 0
        self.yaw_sus_n = 0
        self.yaw_ref = None

    def adim(self, bearing, iyaw, dt):
        cfg = self.cfg
        if cfg.YAW_CIPA:
            yaw_err = normalize_angle(bearing - iyaw)
            adim_ham = yaw_err
            tavan = cfg.YAW_RATE_MAX * dt
            adim = clamp(adim_ham, -tavan, tavan)
            yaw_doygun = abs(adim_ham) > tavan
            if yaw_doygun:
                ilerleme = (None if self.yaw_ref is None
                            else self.yaw_ref - abs(yaw_err))
                if ilerleme is not None and ilerleme > 0.25 * abs(adim):
                    self.yaw_doygun_n = 0
                else:
                    self.yaw_doygun_n += 1
                self.yaw_ref = abs(yaw_err)
            else:
                self.yaw_doygun_n = 0
                self.yaw_ref = None
            if self.yaw_doygun_n > cfg.YAW_DOYGUN_N:
                adim = 0.0
                self.yaw_sus_n += 1
                if self.yaw_sus_n >= cfg.YAW_SUS_N:
                    self.yaw_doygun_n, self.yaw_ref, self.yaw_sus_n = 0, None, 0
            else:
                self.yaw_sus_n = 0
            if abs(yaw_err) <= cfg.YAW_DEADBAND:
                adim = 0.0
            self.cmd_yaw = normalize_angle(iyaw + adim)
        else:
            if self.cmd_yaw is None:
                self.cmd_yaw = bearing
            yaw_err = normalize_angle(bearing - self.cmd_yaw)
            if abs(yaw_err) > cfg.YAW_DEADBAND:
                step = clamp(yaw_err, -cfg.YAW_RATE_MAX * dt,
                             cfg.YAW_RATE_MAX * dt)
                self.cmd_yaw = normalize_angle(self.cmd_yaw + step)
        return self.cmd_yaw


def main():
    print("GPS yaw çıpalama — kabul kriterleri")
    print("=" * 60)
    DT = 0.05                       # 20 Hz
    KAPALI, ACIK = gg.Cfg, _Cfg
    tavan = gg.Cfg.YAW_RATE_MAX * DT

    # ── Y0: VARSAYILAN KAPALI ── deney kolu kazara açık kalmasın.
    kontrol("Y0  varsayılan KAPALI (Kayra'nın hattı bozulmadı)",
            gg.Cfg.YAW_CIPA is False,
            f"YAW_CIPA={gg.Cfg.YAW_CIPA}")

    # ── Y1: KAPALIYKEN gerçek başlıktan ETKİLENMEZ ──
    # Aynı bearing dizisi, taban tabana zıt iki iyaw geçmişi → aynı komut.
    s_a, s_b = _YawSim(KAPALI), _YawSim(KAPALI)
    hedef = math.radians(90.0)
    for i in range(40):
        s_a.adim(hedef, 0.0, DT)                       # araç hiç dönmüyor
        s_b.adim(hedef, math.radians((i * 37) % 360), DT)   # araç fırıl fırıl
    kontrol("Y1  KAPALI: komut aracın başlığından bağımsız (Kayra hattı)",
            abs(normalize_angle(s_a.cmd_yaw - s_b.cmd_yaw)) < 1e-9,
            f"A={math.degrees(s_a.cmd_yaw):.2f}° B={math.degrees(s_b.cmd_yaw):.2f}°")

    # ── Y2: AÇIKKEN komut gerçek başlığa demirlenir ──
    s = _YawSim(ACIK)
    iyaw = math.radians(10.0)
    cmd = s.adim(math.radians(90.0), iyaw, DT)         # 80° hata, tavan 6°
    kontrol("Y2  AÇIK: komut = actual + kırpılmış adım",
            abs(normalize_angle(cmd - (iyaw + tavan))) < 1e-9,
            f"cmd={math.degrees(cmd):.2f}° beklenen={math.degrees(iyaw + tavan):.2f}°")

    # ── Y3: adım tavanı — araç KOMUTA HİÇ UYMASA BİLE komut kaçmaz ──
    # 08-05 kaçağının tam senaryosu: bearing sürekli 180° ötede, araç dönmüyor.
    s = _YawSim(ACIK)
    en_kotu = 0.0
    for i in range(200):
        c = s.adim(math.radians(179.0), 0.0, DT)       # actual sabit 0
        en_kotu = max(en_kotu, abs(normalize_angle(c - 0.0)))
    kontrol("Y3  AÇIK: araç dönmese de |cmd − actual| ≤ tavan (kaçak imkânsız)",
            en_kotu <= tavan + 1e-9,
            f"en kötü={math.degrees(en_kotu):.2f}° tavan={math.degrees(tavan):.2f}°")

    # Aynı senaryo KAPALI kolda: komut serbestçe kaçar (kıyas için)
    s = _YawSim(KAPALI)
    for _ in range(200):
        c = s.adim(math.radians(179.0), 0.0, DT)
    kacak = abs(normalize_angle(c - 0.0))
    kontrol("Y3b KAPALI: aynı senaryoda komut kaçar (kıyas — kusur değil, fark)",
            kacak > tavan * 5,
            f"kaçak={math.degrees(kacak):.1f}° (tavan {math.degrees(tavan):.1f}°)")

    # ── Y4: ölü bant ──
    s = _YawSim(ACIK)
    iyaw = math.radians(45.0)
    cmd = s.adim(iyaw + gg.Cfg.YAW_DEADBAND * 0.5, iyaw, DT)
    kontrol("Y4  AÇIK: ölü bant içinde adım 0 → cmd = actual",
            abs(normalize_angle(cmd - iyaw)) < 1e-9,
            f"cmd−actual={math.degrees(normalize_angle(cmd - iyaw)):.4f}°")

    # ── Y5: sürekli doygunluk kapısı — araç dönmüyorsa yaw susturulur ──
    s = _YawSim(ACIK)
    sus_karesi = None
    for i in range(1, 60):
        c = s.adim(math.radians(179.0), 0.0, DT)       # actual hiç ilerlemiyor
        if abs(normalize_angle(c - 0.0)) < 1e-9 and sus_karesi is None:
            sus_karesi = i
    # Sayaç kaçıncı karede taşar: yaw_doygun_n her karede +1, kapı `> N` olduğu
    # için N. karede henüz açılmaz, N+1'de açılır.
    kontrol("Y5  AÇIK: hata kapanmıyorsa YAW_DOYGUN_N sonrası yaw susar",
            sus_karesi is not None and sus_karesi == gg.Cfg.YAW_DOYGUN_N + 1,
            f"susma {sus_karesi}. karede (YAW_DOYGUN_N={gg.Cfg.YAW_DOYGUN_N})")

    # ── Y6: SÜRELİ SUSMA — 93 saniyelik ölü kilit bir daha olmasın ──
    # Susma başladıktan sonra en fazla YAW_SUS_N kare sürer, sonra yetki döner.
    s = _YawSim(ACIK)
    en_uzun = akan = 0
    donme_karesi = 0
    N = 600                                            # 30 s @20 Hz
    for _ in range(N):
        c = s.adim(math.radians(179.0), 0.0, DT)
        if abs(normalize_angle(c - 0.0)) < 1e-9:
            akan += 1
            en_uzun = max(en_uzun, akan)
        else:
            akan = 0
            donme_karesi += 1
    oran = 100.0 * donme_karesi / N
    kontrol("Y6  AÇIK: susma SÜRELİ — en uzun susma ≤ YAW_SUS_N, yetki geri geliyor",
            en_uzun <= gg.Cfg.YAW_SUS_N and donme_karesi > 0,
            f"en uzun susma={en_uzun} kare (tavan {gg.Cfg.YAW_SUS_N}), "
            f"dönme yetkisi %{oran:.0f}")

    # ── Y7: meşru büyük dönüş SUSTURULMAZ ──
    # Araç komutu izliyor (her karede tavan kadar dönüyor) → hata kapanıyor.
    s = _YawSim(ACIK)
    iyaw = 0.0
    hedef = math.radians(170.0)
    hic_susmadi = True
    for _ in range(40):
        c = s.adim(hedef, iyaw, DT)
        if abs(normalize_angle(c - iyaw)) < 1e-9 and abs(
                normalize_angle(hedef - iyaw)) > gg.Cfg.YAW_DEADBAND:
            hic_susmadi = False
        iyaw = c                                       # araç komuta tam uyuyor
    kontrol("Y7  AÇIK: hata kapanıyorsa büyük dönüş susturulmaz",
            hic_susmadi and abs(normalize_angle(hedef - iyaw)) < math.radians(5),
            f"kalan hata={math.degrees(abs(normalize_angle(hedef - iyaw))):.2f}°")

    # ── Y8: DÖNGÜ DUMAN TESTİ — gerçek run_gps_guidance, iki kol da koşar ──
    class _FakeMav:
        def __init__(s): s.last = None
        def set_position_target_local_ned_send(s, *a): s.last = a

    class _FakeConn:
        target_system = 1
        target_component = 1
        def __init__(s): s.mav = _FakeMav()
        def recv_match(s, **k): return None

    def kos(cfg, drone_yaw):
        """Hedef sağda duruyor, drone burnu drone_yaw'da KİLİTLİ (hiç dönmüyor).
        Çıpalama kolunda komut bu başlığa yapışmalı; kapalı kolda kaçmalı."""
        conn = _FakeConn()
        def get_plane():
            return {"x": 200.0, "y": 200.0, "z": -100.0, "yaw": 0.0, "frozen": False}
        def get_iris():
            return {"x": 0.0, "y": 0.0, "z": -100.0,
                    "roll": 0.0, "pitch": 0.0, "yaw": drone_yaw,
                    "vx": 0.0, "vy": 0.0, "vz": 0.0}
        stop = threading.Event()
        th = threading.Thread(target=gg.run_gps_guidance,
                              args=(conn, get_plane, get_iris, stop),
                              kwargs={"cfg": cfg}, daemon=True)
        th.start()
        time.sleep(0.9)
        sent = conn.mav.last
        stop.set(); th.join(2.0)
        # set_position_target_local_ned_send arg sırası (common.send_velocity):
        # 0 t · 1-2 sys/comp · 3 frame · 4 typemask · 5-7 pos · 8-10 vel ·
        # 11-13 accel · 14 YAW · 15 yaw_rate
        return sent[14] if sent else None

    drone_yaw = math.radians(-120.0)       # bearing +45°, yani ~165° hata
    cmd_acik = kos(ACIK, drone_yaw)
    cmd_kapali = kos(KAPALI, drone_yaw)
    ok_acik = (cmd_acik is not None
               and abs(normalize_angle(cmd_acik - drone_yaw)) <= tavan + 1e-6)
    kontrol("Y8  döngü/AÇIK: komut aracın başlığına yapışık kaldı",
            ok_acik,
            f"cmd={math.degrees(cmd_acik):.1f}° actual={math.degrees(drone_yaw):.1f}° "
            f"fark={math.degrees(abs(normalize_angle(cmd_acik - drone_yaw))):.2f}° "
            f"(tavan {math.degrees(tavan):.1f}°)" if cmd_acik is not None else "komut yok")
    kontrol("Y8b döngü/KAPALI: komut üretiliyor (Kayra kolu sağlam)",
            cmd_kapali is not None,
            f"cmd={math.degrees(cmd_kapali):.1f}°" if cmd_kapali is not None else "komut yok")

    print("=" * 60)
    gecen = sum(1 for _, ok, _ in _sonuclar if ok)
    print(f"{gecen}/{len(_sonuclar)} GEÇTİ")
    for ad, ok, detay in _sonuclar:
        if not ok:
            print(f"  ✗ {ad}  {detay}")
    return 0 if gecen == len(_sonuclar) else 1


if __name__ == "__main__":
    raise SystemExit(main())
