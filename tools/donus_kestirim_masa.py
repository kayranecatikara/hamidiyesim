#!/usr/bin/env python3
"""tools/donus_kestirim_masa.py — Hedefin dönüşünü YALNIZ KAMERADAN kestirme,
ÇEVRİMDIŞI doğrulama (uçmadan önce masada).

NEDEN: 2026-08-08'de üç deney çürüdü (lead kapısı, lead tavanı, kapanma hızı).
Kök neden: yatay ivme kamera yüzünden 4 m/s²'de sabit → 22 m/s'te dönüş yarıçapı
110 m, hedefin dairesi 27.5 m. Hiçbir SABİT hız kurtarmıyor. Kapı: hedefin ŞU AN
olduğu yere değil OLACAĞI yere nişan almak — düz çizgi ivme gerektirmez.
Bunun için hedefin dönüş yarıçapı lazım ve TELEMETRİDEN ALINAMAZ (yarışmada GPS
karıştırılacak — kullanıcı kararı 08-08). Yalnız kamera + aracın kendi durumu.

YÖNTEM: gerçek uçuş yörüngeleri (gps_guidance CSV'leri) alınır, kameranın ne
GÖRECEĞİ ölçülmüş gürültüyle sentezlenir, kestirimci onun üstünde koşturulur ve
cevap anahtarıyla kıyaslanır. Uçuş harcamadan "bu iş oluyor mu" sorusu.

ÖLÇÜLMÜŞ ALGI GÜRÜLTÜSÜ (08-08, 7830 'ok' karesi, bbox vs cevap anahtarı):
    açı (yaw)   : medyan −0.54°, p10/p90 −2.74/+0.63  → σ ≈ 1.7°
    açı (elev)  : medyan +0.30°, p10/p90 −1.26/+1.24  → σ ≈ 1.25°
    MENZİL      : oran medyan 1.19× (%19 YANLI), p10/p90 0.44/1.91 → ±%60
Yani açı iyi, menzil berbat. Kestirimci buna göre kurulur.

SONUÇ (2026-08-08, masada — UÇUŞ HARCANMADI):

  1. KESTİRİMCİ ÇALIŞIYOR. Nişan açısı hatası τ=1 s ufukta 3.1°, saf takipte
     26.4°. ⚠ ÖLÇÜT AÇI OLMALI, METRE DEĞİL: menzil ±%60 hatalı ama nişan
     AÇISI = kayma ÷ menzil olduğu için ölçek sadeleşir — menzili mükemmel
     yapınca hata yalnız 3.1° → 2.7° iyileşiyor. Yani kestirimci kötü menzile
     RAĞMEN çalışır, bu da yarışma şartı (GPS karıştırılacak).

  2. AMA KESİŞME NİŞANI ASIL DERDİ ÇÖZMÜYOR. Nişanın DÖNME hızı (0.5 s tabanlı,
     gürültü katkısı ~5 °/s):

        menzil     saf takip   KESİŞME    araç (22 m/s'te 10.4 °/s)
         8-20 m     19.2 °/s   19.5 °/s   yetmiyor
        20-35 m     25.2 °/s   24.8 °/s   yetmiyor
        35-55 m     15.2 °/s   19.6 °/s   yetmiyor
        55-80 m     10.7 °/s   21.9 °/s   yetmiyor
        80-130 m     6.6 °/s   12.9 °/s   yetmiyor

     Kesişme nişanı HİÇBİR menzilde saf takipten iyi değil, uzakta daha kötü.
     "Hedefin olacağı yere nişan al" fikri masada ÇÜRÜDÜ.

  3. ⚠ İKİ ÖLÇÜM TUZAĞI (tekrarlama): (a) CT modelinde w=0 dalı ayrı yazılırsa
     ω'ya duyarlılık sıfır olur ve filtre ω'yı ASLA öğrenmez (gürültüsüzde bile
     hata 20.4 °/s). Tek sürekli formül + seri açılımı şart. (b) Gürültülü açıyı
     kare-arası (0.05 s) türevlemek σ=1.7°'de ~34 °/s SAHTE dönme hızı üretir;
     her menzil bandı aynı çıkar. Türev tabanı ≥0.5 s olmalı.

Kullanım:
    python3 tools/donus_kestirim_masa.py                 # bugünkü daire uçuşları
    python3 tools/donus_kestirim_masa.py logs/gps_*.csv  # belirli loglar
"""
import csv
import glob
import math
import statistics as st
import sys

import numpy as np

# ── Ölçülmüş gürültü (yukarıdaki tablo) ──
ACI_SIGMA_DEG = 1.7        # LOS açısı gürültüsü (yaw); elev biraz daha iyi
MENZIL_YANLILIK = 1.19     # bbox menzili sistematik %19 fazla gösteriyor
MENZIL_SIGMA_ORAN = 0.45   # çarpımsal gürültü (p10/p90 ±%60'a karşılık gelir)

DT_NOM = 0.05              # 20 Hz GPS logu


class DonusKestirimi:
    """Sabit-dönüş-hızı (CT) modeli, açı-ağırlıklı EKF.

    Durum: [x, y, vx, vy, ω]  (yatay düzlem, NED x-y)
    Ölçüm: LOS azimutu (İYİ) + menzil (KÖTÜ)

    Menzile küçük ağırlık verilir; asıl bilgi açıdan ve aracın KENDİ
    hareketinden gelir (kendi hareketi menzili gözlenebilir kılar — açı-tabanlı
    hedef hareket analizi, bearings-only TMA). Menzil yine de tamamen atılmaz,
    ölçek demirini o veriyor.
    """

    def __init__(self, aci_sigma_deg=ACI_SIGMA_DEG, menzil_sigma_oran=MENZIL_SIGMA_ORAN):
        self.x = None                      # durum
        self.P = None                      # kovaryans
        self.r_aci = math.radians(aci_sigma_deg) ** 2
        self.menzil_sigma_oran = menzil_sigma_oran
        # süreç gürültüsü: hedef manevra yapabilir, ω yavaş değişir
        self.q_ivme = 8.0 ** 2             # m/s² — hedefin ivme belirsizliği
        self.q_omega = math.radians(20.0) ** 2   # (rad/s)/s — ω'nın değişim hızı

    def baslat(self, p_own, az, menzil):
        tx = p_own[0] + menzil * math.cos(az)
        ty = p_own[1] + menzil * math.sin(az)
        self.x = np.array([tx, ty, 0.0, 0.0, 0.0])
        self.P = np.diag([25.0, 25.0, 400.0, 400.0, math.radians(60.0) ** 2])

    def _ilerlet(self, dt):
        yeni = self._ham_ilerlet(dt)
        # Sayısal Jacobian. ⚠ _ham_ilerlet w=0'da DA türevlenebilir olmalı;
        # ilk sürümde |w|<1e-4 için ayrı bir "düz uçuş" dalı vardı ve o dalın
        # w'ya duyarlılığı SIFIRDI → ω sıfırda başlayınca sonsuza kadar sıfırda
        # kalıyordu (gürültü sıfırken bile ω hatası 20.4 °/s, yani kestirim yok).
        # Şimdi tek sürekli formül + küçük w için seri açılımı kullanılıyor.
        F = np.eye(5)
        tut = self.x.copy()
        for j in range(5):
            eps = 1e-5 if j < 4 else 1e-4      # ω için biraz daha büyük adım
            p = tut.copy(); p[j] += eps
            self.x = p
            F[:, j] = (self._ham_ilerlet(dt) - yeni) / eps
        self.x = tut
        Q = np.diag([
            self.q_ivme * dt ** 4 / 4, self.q_ivme * dt ** 4 / 4,
            self.q_ivme * dt ** 2, self.q_ivme * dt ** 2,
            self.q_omega * dt])
        self.x = yeni
        self.P = F @ self.P @ F.T + Q

    def _ham_ilerlet(self, dt):
        """Sabit dönüş hızı ilerletmesi — w=0'da TEKİL DEĞİL (seri açılımı)."""
        x, y, vx, vy, w = self.x
        wdt = w * dt
        if abs(wdt) < 1e-4:
            # sin(wdt)/w ve (1-cos(wdt))/w'nin w→0 limitleri
            A = dt - w * w * dt ** 3 / 6.0
            B = w * dt * dt / 2.0
        else:
            A = math.sin(wdt) / w
            B = (1.0 - math.cos(wdt)) / w
        c, s = math.cos(wdt), math.sin(wdt)
        return np.array([x + vx * A - vy * B,
                         y + vx * B + vy * A,
                         vx * c - vy * s,
                         vx * s + vy * c,
                         w])

    def guncelle(self, p_own, az_olc, menzil_olc, dt):
        if self.x is None:
            self.baslat(p_own, az_olc, menzil_olc)
            return
        self._ilerlet(max(1e-3, dt))

        dx = self.x[0] - p_own[0]
        dy = self.x[1] - p_own[1]
        r2 = dx * dx + dy * dy
        r = math.sqrt(max(r2, 1e-6))

        # ── 1) AÇI ölçümü (güvenilir) ──
        H = np.array([[-dy / r2, dx / r2, 0.0, 0.0, 0.0]])
        y_art = (az_olc - math.atan2(dy, dx) + math.pi) % (2 * math.pi) - math.pi
        self._kalman(H, np.array([y_art]), np.array([[self.r_aci]]))

        # ── 2) MENZİL ölçümü (kötü — çarpımsal gürültü, yanlılık düzeltilmiş) ──
        dx = self.x[0] - p_own[0]; dy = self.x[1] - p_own[1]
        r = math.hypot(dx, dy) or 1e-6
        H = np.array([[dx / r, dy / r, 0.0, 0.0, 0.0]])
        menzil_duz = menzil_olc / MENZIL_YANLILIK       # bilinen yanlılık çıkarılır
        R = np.array([[(self.menzil_sigma_oran * menzil_duz) ** 2]])
        self._kalman(H, np.array([menzil_duz - r]), R)

    def _kalman(self, H, y, R):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        I = np.eye(5)
        self.P = (I - K @ H) @ self.P

    # ── çıktılar ──
    def omega_dps(self):
        return math.degrees(self.x[4])

    def hiz(self):
        return math.hypot(self.x[2], self.x[3])

    def yaricap(self):
        w = abs(self.x[4])
        return self.hiz() / w if w > 1e-3 else float("inf")

    def gelecek_konum(self, tau):
        """tau saniye sonra hedef nerede olacak (CT modeliyle ileri sarım)."""
        tut_x, tut_P = self.x.copy(), self.P.copy()
        self.x = self._ham_ilerlet(tau)
        p = (self.x[0], self.x[1])
        self.x, self.P = tut_x, tut_P
        return p


def gercek_daire(xs, ys):
    """Pencereye çember oturt (cevap anahtarı yarıçapı). Dönüş: (cx, cy, R)."""
    A = np.c_[2 * np.array(xs), 2 * np.array(ys), np.ones(len(xs))]
    b = np.array(xs) ** 2 + np.array(ys) ** 2
    try:
        c, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = c[0], c[1]
    R = math.sqrt(max(0.0, c[2] + cx * cx + cy * cy))
    return cx, cy, R


def _f(r, k):
    v = r.get(k, "")
    try:
        return float(v) if v not in ("", "None") else None
    except ValueError:
        return None


def _omega_serisi(rows, ema=0.15):
    """Hedefin açısal hızı (°/s, işaretli) — tgt_vx/tgt_vy başlık türevinden.

    ⚠ GPS CSV'sine sütun EKLENMEZ (iş bölümü: gps_guidance Kayra'nın alanı).
    Bu büyüklük gerekiyorsa MEVCUT sütunlardan türetilir; `tgt_omega_dps`
    eklemek GPS'e dokunmak demektir.
    """
    out = [None] * len(rows)
    ph = pt = None
    w = 0.0
    for i, r in enumerate(rows):
        t = _f(r, "t")
        vx, vy = _f(r, "tgt_vx"), _f(r, "tgt_vy")
        if None in (t, vx, vy) or math.hypot(vx, vy) < 3.0:
            out[i] = math.degrees(w)
            continue
        h = math.atan2(vy, vx)
        if ph is not None and 0 < t - pt < 0.5:
            d = (h - ph + math.pi) % (2 * math.pi) - math.pi
            w = ema * (d / (t - pt)) + (1 - ema) * w
        ph, pt = h, t
        out[i] = math.degrees(w)
    return out


def kos(yol, rng, tau=1.5):
    rows = list(csv.DictReader(open(yol)))
    if len(rows) < 400:
        return None

    om_ger = _omega_serisi(rows)
    kf = DonusKestirimi()
    hatalar_w = []      # ω hatası (°/s)
    hatalar_R = []      # yarıçap bağıl hatası
    hatalar_p = []      # tau sonrası konum kestirim hatası (m)
    ham_w = []          # kıyas: ham ölçümden sonlu-fark ω (filtresiz)
    n_kullanilan = 0

    gecmis = []         # cevap anahtarı çember oturtma penceresi
    onceki_t = None
    bekleyen = []       # (t, gercek_gelecek_konum) için

    for _i, r in enumerate(rows):
        t = _f(r, "t")
        ix, iy = _f(r, "iris_x"), _f(r, "iris_y")
        tx, ty = _f(r, "tgt_ham_x"), _f(r, "tgt_ham_y")
        if tx is None:
            tx, ty = _f(r, "tgt_x"), _f(r, "tgt_y")
        if None in (t, ix, iy, tx, ty):
            continue

        dt = DT_NOM if onceki_t is None else max(1e-3, t - onceki_t)
        onceki_t = t

        # ── KAMERANIN GÖRECEĞİ: gerçek geometri + ölçülmüş gürültü ──
        az_ger = math.atan2(ty - iy, tx - ix)
        menzil_ger = math.hypot(tx - ix, ty - iy)
        az_olc = az_ger + math.radians(rng.normal(0.0, ACI_SIGMA_DEG))
        menzil_olc = menzil_ger * MENZIL_YANLILIK * math.exp(
            rng.normal(0.0, MENZIL_SIGMA_ORAN) - MENZIL_SIGMA_ORAN ** 2 / 2)

        kf.guncelle((ix, iy), az_olc, menzil_olc, dt)

        gecmis.append((t, tx, ty))
        gecmis[:] = [g for g in gecmis if t - g[0] <= 3.0]
        bekleyen.append((t, kf.gelecek_konum(tau)))

        # cevap anahtarı: son 3 s'ye çember oturt
        if len(gecmis) > 40:
            cd = gercek_daire([g[1] for g in gecmis], [g[2] for g in gecmis])
            # ⚠ GPS CSV'sine sütun EKLEMİYORUZ (iş bölümü: gps_guidance Kayra'da).
            # Hedefin açısal hızı mevcut tgt_vx/tgt_vy'den TÜRETİLİR.
            w_ger = om_ger[_i]
            if cd and w_ger is not None and abs(w_ger) > 5.0 and cd[2] < 200:
                n_kullanilan += 1
                hatalar_w.append(abs(kf.omega_dps()) - abs(w_ger))
                Rk = kf.yaricap()
                if math.isfinite(Rk) and cd[2] > 1:
                    hatalar_R.append((Rk - cd[2]) / cd[2])

        # tau saniye önce yapılan konum kestirimi şimdi doğrulanabilir
        while bekleyen and t - bekleyen[0][0] >= tau:
            _, tahmin = bekleyen.pop(0)
            hatalar_p.append(math.hypot(tahmin[0] - tx, tahmin[1] - ty))

    if n_kullanilan < 100:
        return None
    return {
        "n": n_kullanilan,
        "w_hata_med": st.median(hatalar_w),
        "w_hata_abs": st.median([abs(h) for h in hatalar_w]),
        "R_hata_med": st.median(hatalar_R) if hatalar_R else float("nan"),
        "R_hata_abs": st.median([abs(h) for h in hatalar_R]) if hatalar_R else float("nan"),
        "p_hata_med": st.median(hatalar_p) if hatalar_p else float("nan"),
        "p_hata_p90": (sorted(hatalar_p)[9 * len(hatalar_p) // 10]
                       if hatalar_p else float("nan")),
    }


def main():
    yollar = sys.argv[1:] or sorted(glob.glob("logs/gps_guidance_20260808_*.csv"))
    rng = np.random.default_rng(20260808)
    tau = 1.5

    print(f"MASA TESTİ — hedefin dönüşünü yalnız kameradan kestirme")
    print(f"  gürültü: açı σ={ACI_SIGMA_DEG}°, menzil ×{MENZIL_YANLILIK} yanlı ±%{MENZIL_SIGMA_ORAN*100:.0f}")
    print(f"  kestirim ufku τ={tau} s\n")
    print(f"{'log':22s} {'n':>6s} {'ω hata':>10s} {'yarıçap hata':>13s} "
          f"{'τ sonrası konum':>17s}")

    ozet = {"w": [], "R": [], "p": [], "p90": []}
    for y in yollar:
        s = kos(y, rng, tau)
        if s is None:
            continue
        print(f"{y.split('/')[-1][:22]:22s} {s['n']:6d} "
              f"{s['w_hata_abs']:8.1f}°/s {100*s['R_hata_abs']:11.0f}% "
              f"{s['p_hata_med']:12.1f} m (p90 {s['p_hata_p90']:.1f})")
        ozet["w"].append(s["w_hata_abs"]); ozet["R"].append(s["R_hata_abs"])
        ozet["p"].append(s["p_hata_med"]); ozet["p90"].append(s["p_hata_p90"])

    if not ozet["w"]:
        print("\n  (yeterli veri içeren log bulunamadı)")
        return 1
    print(f"\n{'='*70}")
    print(f"  ω kestirim hatası (medyan mutlak)   : {st.median(ozet['w']):.1f} °/s")
    print(f"  dönüş yarıçapı hatası               : %{100*st.median(ozet['R']):.0f}")
    print(f"  {tau} s sonrasının konum hatası        : {st.median(ozet['p']):.1f} m "
          f"(p90 {st.median(ozet['p90']):.1f} m)")
    print(f"{'='*70}")
    print("\nÖLÇÜT: nişanı kesişme noktasına koymak için τ sonrası konum hatası")
    print("       hedefin boyundan (≈2 m) küçük olmalı; 5 m üstü kullanılamaz.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
