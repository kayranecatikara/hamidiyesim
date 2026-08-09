#!/usr/bin/env python3
"""ArduPilot kara kutusundan (.BIN) YAW titremesinin GERÇEK spektrumu.

Neden bu araç var
-----------------
Güdüm CSV'si 20 Hz. Titreme 4-5 Hz civarında ölçülmüştü ama 20 Hz örneklemede
Nyquist 10 Hz — 4.3 Hz zar zor çözülür, üstündeki her şey ALIAS olarak
görünür. Yanlış frekansa göre filtre seçmek zaman kaybı.

.BIN içindeki ATT kaydı 50-400 Hz. Bu araç:
  * ATT.Yaw (gerçek) ve ATT.DesYaw (komut) serilerini çeker
  * yaw HATASI = Yaw - DesYaw (sarma düzeltilmiş) hesaplar
  * hatanın genlik spektrumunu (DFT) çıkarır → tepe frekans
  * RATE.R/RATE.RDes varsa yaw hızı çevrimini de aynı şekilde inceler

Kullanım:
    PYTHONPATH=. python3 tools/yaw_spektrum.py son      # en yeni COPTER kaydı
    PYTHONPATH=. python3 tools/yaw_spektrum.py son2     # son iki kayıt = A/B
    PYTHONPATH=. python3 tools/yaw_spektrum.py A.BIN B.BIN

⚠ Bu araç SADECE OKUR. Hiçbir parametre yazmaz, hiçbir uçuşu etkilemez.
"""
from __future__ import annotations

import glob
import math
import os
import sys

_LOG_DIZIN = os.path.expanduser("~/ardupilot/logs")


def _copter_mu(yol: str) -> bool:
    """BIN ArduCopter'a mı ait? (aynı dizinde Plane kayıtları da var.)

    Uçuş başında yazılan MSG metinlerinde firmware adı geçer. Yalnız ilk
    birkaç yüz kaydı tarar — 20 MB'lık dosyayı baştan sona okumaz.
    """
    from pymavlink import mavutil

    try:
        m = mavutil.mavlink_connection(yol)
    except Exception:
        return False
    for _ in range(400):
        msg = m.recv_match(type=["MSG"], blocking=False)
        if msg is None:
            break
        metin = str(getattr(msg, "Message", ""))
        if "ArduCopter" in metin:
            return True
        if "ArduPlane" in metin:
            return False
    return False


def _son_copter(n: int = 1):
    """En yeni n adet COPTER .BIN yolunu ESKİDEN YENİYE döndür (A, B sırası)."""
    hepsi = sorted(glob.glob(os.path.join(_LOG_DIZIN, "*.BIN")),
                   key=os.path.getmtime, reverse=True)
    bulunan = []
    for y in hepsi[: 4 * n + 8]:
        if _copter_mu(y):
            bulunan.append(y)
        if len(bulunan) >= n:
            break
    return list(reversed(bulunan))


def _sar(d: float) -> float:
    """Açı farkını (-180, 180] aralığına sar."""
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


def _oku(yol: str):
    """BIN'den ATT (açı) ve IMU (gyroZ) serilerini çek.

    ASIL KANAL IMU.GyrZ'dir: ATT varsayılan LOG_BITMASK'ta yalnız 10 Hz
    kaydediliyor ve 8 Hz'lik çevrimi 2 Hz'e alias'lıyor. IMU daha hızlı.
    (LOG_BITMASK 442367 ile ikisi de ana döngü hızında — bkz. avci_copter.parm.)
    """
    from pymavlink import mavutil

    m = mavutil.mavlink_connection(yol)
    att_t, att_y, att_d = [], [], []
    imu_t, imu_gz = [], []
    while True:
        msg = m.recv_match(type=["ATT", "IMU"], blocking=False)
        if msg is None:
            break
        t = getattr(msg, "TimeUS", None)
        if t is None:
            continue
        t = t * 1e-6
        if msg.get_type() == "ATT":
            att_t.append(t)
            att_y.append(float(msg.Yaw))
            att_d.append(float(msg.DesYaw))
        elif getattr(msg, "I", 0) == 0:
            # Yalnız 0. IMU — örnekleri karıştırmak sahte frekans üretir.
            imu_t.append(t)
            imu_gz.append(math.degrees(float(msg.GyrZ)))
    return (att_t, att_y, att_d), (imu_t, imu_gz)


def _spektrum(t, x, fmin=0.3, fmax=None):
    """Eşit aralığa yeniden örnekle, ortalamayı çıkar, DFT genliği döndür.

    Döner: (fs, [(frekans, genlik), ...] fmin..fmax bandında)
    """
    n = len(t)
    if n < 64:
        return 0.0, []
    dt = (t[-1] - t[0]) / (n - 1)
    fs = 1.0 / dt
    if fmax is None:
        fmax = fs / 2.0
    # Ortalamayı çıkar (DC tepesi bandı bastırmasın)
    ort = sum(x) / n
    y = [v - ort for v in x]
    # Hann penceresi — sızıntıyı kes
    y = [v * 0.5 * (1.0 - math.cos(2.0 * math.pi * i / (n - 1))) for i, v in enumerate(y)]

    # Yalnız ilgi bandındaki binleri hesapla (tam FFT gereksiz, N büyük)
    k_bas = max(1, int(fmin * n / fs))
    k_son = min(n // 2, int(fmax * n / fs))
    if k_son <= k_bas:
        return fs, []
    # Bin sayısı çok büyükse seyrelt (görüntüleme için yeterli çözünürlük)
    adim = max(1, (k_son - k_bas) // 400)
    cikti = []
    for k in range(k_bas, k_son, adim):
        w = 2.0 * math.pi * k / n
        re = im = 0.0
        for i, v in enumerate(y):
            a = w * i
            re += v * math.cos(a)
            im -= v * math.sin(a)
        # Hann penceresi genliği 2 kat zayıflatır → 4/n ölçek
        cikti.append((k * fs / n, 4.0 * math.hypot(re, im) / n))
    return fs, cikti


def _spektrum_hizli(t, x, fmin=0.3, fmax=None):
    """_spektrum ile aynı ama numpy varsa onu kullanır (çok daha hızlı)."""
    try:
        import numpy as np
    except ImportError:
        return _spektrum(t, x, fmin, fmax)
    n = len(t)
    if n < 64:
        return 0.0, []
    dt = (t[-1] - t[0]) / (n - 1)
    fs = 1.0 / dt
    if fmax is None:
        fmax = fs / 2.0
    y = np.asarray(x, dtype=float)
    y = y - y.mean()
    y = y * np.hanning(n)
    Y = np.fft.rfft(y)
    f = np.fft.rfftfreq(n, dt)
    g = 4.0 * np.abs(Y) / n
    sec = (f >= fmin) & (f <= fmax)
    return fs, list(zip(f[sec].tolist(), g[sec].tolist()))


def _tepe(spek, n=5):
    return sorted(spek, key=lambda p: -p[1])[:n]


def _bant_gucu(spek, f1, f2):
    """f1..f2 bandındaki RMS katkısı (genliklerin karekök toplamı)."""
    s = sum(g * g for f, g in spek if f1 <= f < f2)
    return math.sqrt(s / 2.0)


def _sabit_komut_dilimi(t, des, min_sure=8.0, tol=1.0):
    """Komutun ~sabit olduğu en uzun dilimi bul — güdümü suçlamamak için.

    Titremeyi ölçerken komut da hareket ediyorsa, gerçek yaw'ın hareketi
    normaldir. Yalnız komut duruyorken oynuyorsa kabahat araçtadır.
    """
    en_iyi = (0, 0)
    i = 0
    n = len(t)
    while i < n:
        j = i + 1
        while j < n and abs(_sar(des[j] - des[i])) <= tol:
            j += 1
        if t[j - 1] - t[i] > t[en_iyi[1] - 1] - t[en_iyi[0]] if en_iyi[1] > en_iyi[0] else True:
            if j - i > 2:
                en_iyi = (i, j)
        i = j
    if t[en_iyi[1] - 1] - t[en_iyi[0]] < min_sure:
        return None
    return en_iyi


def incele(yol: str):
    ad = yol.split("/")[-1]
    (at, ay, ad_), (it, igz) = _oku(yol)
    if len(at) < 200:
        print(f"  {ad}: ATT kaydı yok/az ({len(at)}) — bu araç olmayabilir")
        return None

    dt = (at[-1] - at[0]) / (len(at) - 1)
    print(f"\n=== {ad} ===")
    print(f"  ATT  : {len(at)} kayıt, {at[-1]-at[0]:.1f} s, {1.0/dt:.0f} Hz")

    dil = _sabit_komut_dilimi(at, ad_)
    if dil is None:
        print("  ⚠ komutun sabit kaldığı ≥8 s'lik dilim yok — tüm kayıt kullanılıyor")
        i, j = 0, len(at)
    else:
        i, j = dil
        print(f"  sabit-komut dilimi: {at[i]:.1f}-{at[j-1]:.1f} s ({at[j-1]-at[i]:.1f} s), "
              f"komut {ad_[i]:.1f}°")

    hata = [_sar(ay[k] - ad_[k]) for k in range(i, j)]
    tt = at[i:j]
    ort = sum(hata) / len(hata)
    std = math.sqrt(sum((h - ort) ** 2 for h in hata) / len(hata))
    print(f"  yaw hatası: ort {ort:+.2f}°  std {std:.2f}°  "
          f"tepe {max(abs(h) for h in hata):.2f}°")

    fs, spek = _spektrum_hizli(tt, hata, fmin=0.3, fmax=min(60.0, 1.0/dt/2.0))
    if not spek:
        return None
    print(f"  en güçlü frekanslar (yaw hatası):")
    for f, g in _tepe(spek):
        print(f"      {f:6.2f} Hz   genlik {g:.3f}°")
    print(f"  bant RMS:  0.3-2 Hz {_bant_gucu(spek,0.3,2):.2f}°   "
          f"2-6 Hz {_bant_gucu(spek,2,6):.2f}°   "
          f"6-15 Hz {_bant_gucu(spek,6,15):.2f}°   "
          f"15+ Hz {_bant_gucu(spek,15,999):.2f}°")

    sonuc = {"ad": ad, "std": std, "tepe": _tepe(spek, 1)[0],
             "b_dusuk": _bant_gucu(spek, 0.3, 2), "b_orta": _bant_gucu(spek, 2, 6),
             "b_yuksek": _bant_gucu(spek, 6, 15), "b_cok": _bant_gucu(spek, 15, 999),
             "sure": at[j-1] - at[i], "gyro_rms": float("nan"),
             "gyro_tepe": (float("nan"), float("nan")), "g_cevrim": float("nan")}

    # ── ASIL KANAL: yaw gyro'su ──────────────────────────────────────────
    # ATT'nin 10 Hz'i 8 Hz'i 2 Hz'e alias'lıyor. IMU daha hızlı örnekliyor,
    # üstelik gyro çevrimi ham hâliyle görür (açı entegrasyonu 8 Hz'i 50 kat
    # bastırır — açıya bakarak "titreme yok" sanmak bu yüzden kolay).
    ig = [k for k, t in enumerate(it) if tt[0] <= t <= tt[-1]]
    if len(ig) > 200:
        gt = [it[k] for k in ig]
        gx = [igz[k] for k in ig]
        gfs, gspek = _spektrum_hizli(gt, gx, fmin=0.3, fmax=None)
        if gspek:
            gort = sum(gx) / len(gx)
            grms = math.sqrt(sum((v - gort) ** 2 for v in gx) / len(gx))
            nyq = gfs / 2.0
            print(f"  gyroZ: {gfs:.0f} Hz örnekleme (Nyquist {nyq:.1f} Hz), "
                  f"RMS {grms:.2f} °/s, tepe {max(abs(v-gort) for v in gx):.1f} °/s")
            print(f"  en güçlü frekanslar (yaw gyro):")
            for f, g in _tepe(gspek, 5):
                print(f"      {f:6.2f} Hz   genlik {g:6.2f} °/s")
            bantlar = [(0.3, 2), (2, 6), (6, 10), (10, 15), (15, 30), (30, 999)]
            print("  bant RMS (°/s): " + "  ".join(
                f"{a:g}-{b if b < 900 else '∞'}:{_bant_gucu(gspek, a, b):.1f}"
                for a, b in bantlar if a < nyq))
            sonuc["gyro_rms"] = grms
            sonuc["gyro_tepe"] = _tepe(gspek, 1)[0]
            # 8 Hz çevriminin kendisi — A/B'nin ASIL ÖLÇÜTÜ
            sonuc["g_cevrim"] = _bant_gucu(gspek, 6.0, 10.0)
            if nyq < 10.0:
                print("  ⚠ Nyquist < 10 Hz — 8 Hz çevrimi ALIAS'lanmış olabilir. "
                      "LOG_BITMASK 442367 (ATTITUDE_FAST + IMU_FAST) şart.")
    return sonuc


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    hedefler = []
    for a in argv[1:]:
        if a.startswith("son"):
            n = int(a[3:] or "1")
            bulunan = _son_copter(n)
            if not bulunan:
                print(f"⚠ {_LOG_DIZIN} içinde ArduCopter .BIN bulunamadı")
                return 1
            hedefler.extend(bulunan)
        else:
            hedefler.append(a)

    sonuclar = [s for s in (incele(y) for y in hedefler) if s]
    if len(sonuclar) >= 2:
        print("\n" + "=" * 70)
        print("KIYAS  (A = eski/ilk kol, B = yeni/son kol)")
        print(f"{'log':<16}{'açı std°':>10}{'gyro RMS':>10}{'gyro tepe':>11}"
              f"{'6-10 Hz':>10}")
        for s in sonuclar:
            print(f"{s['ad']:<16}{s['std']:>10.2f}{s['gyro_rms']:>10.2f}"
                  f"{s['gyro_tepe'][0]:>8.2f} Hz{s['g_cevrim']:>10.2f}")
        a, b = sonuclar[0], sonuclar[-1]
        if a["g_cevrim"] == a["g_cevrim"] and a["g_cevrim"] > 1e-6:
            oran = b["g_cevrim"] / a["g_cevrim"]
            print(f"\n8 Hz ÇEVRİMİ: {a['g_cevrim']:.1f} → {b['g_cevrim']:.1f} °/s "
                  f"({oran:.2f}×, %{100*(oran-1):+.0f})")
            print("YORUM: 0.5×'ten küçükse çentik tuttu; ~1.0 ise etkisiz; "
                  ">1.2 ise KÖTÜLEŞTİ → geri al.")
        print("\n⚠ Bu tablo YALNIZ titremeyi ölçer. Vuruş/menzil kıyası için "
              "tools/gudum_karne.py ve iki kolun hedef irtifası ayrıca bakılmalı.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
