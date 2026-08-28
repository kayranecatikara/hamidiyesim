"""
hedef_kestirim.py — Hedef durum kestirimi: IMM (CV + CA). Saf matematik.

NEDEN VAR (ölçüm, 2026-08-04): mevcut kestirici EMA konum + sonlu fark hızdır.
gps_guidance_20260804_122519.csv'de gerçekle (2 s pencereli) kıyaslandığında:
    gerçek hız medyan 16.00 m/s  ↔  kestirim medyan 19.30 m/s   → 1.21× ŞİŞİK
    gerçek std       0.82        ↔  kestirim std       2.59     → 3.2× GÜRÜLTÜLÜ
    mutlak sapma medyan 3.11 m/s, en kötü 12.82 m/s
Bu hız güdümün ÜÇ teriminin de içine giriyor (ileri besleme, istasyon yönü,
göreli hız terimi) — yani gürültü doğrudan komuta sızıyor.

CTU Prag (arXiv 2405.13542) aynı problemde IMM (CV+CA) kullanıp tek modelli
Kalman'a göre kestirim hatasını %58 düşürmüş. Burada uygulanan da o.

TASARIM — İKİ MODEL, ORTAK DURUM UZAYI:
Klasik IMM'de modeller farklı boyutlu olur (CV 6, CA 9) ve karıştırma adımında
boyut dönüşümü gerekir. Burada her iki model de 9 boyutlu ortak durumu
kullanır: x = [p(3), v(3), a(3)]. Fark yalnız geçiş matrisi ve süreç
gürültüsündedir:
    CV: ivmeyi taşımaz (a sönümlenir), süreç gürültüsü hıza biner   → σ_a
    CA: ivmeyi taşır,   süreç gürültüsü ivmeye biner (jerk)         → σ_j
Böylece karıştırma düz ağırlıklı ortalama olur; boyut dönüşümü ve onun
getirdiği hata kaynağı ortadan kalkar.

Ölçüm yalnız KONUM (telemetriden gelen): H = [I 0 0].

Kullanım:
    kf = IMM()
    kf.guncelle((x, y, z), dt)   → {"p":…, "v":…, "a":…, "w_cv":…, "w_ca":…}
    kf.tahmin(ileri_s)           → ölçüm yokken ileriye taşı (bayat telemetri)
"""

import math

import numpy as np


class Cfg:
    # ── Süreç gürültüsü (F3'te grid-search ile ayarlanacak) ──
    # SIGMA_A, CV modelinin "hedef ivmelenebilir" toleransıdır ve MODEL AYRIM
    # GÜCÜNÜ o belirler. Ölçüldü (2026-08-04): 2.0 ile modeller ayrışmıyordu,
    # ağırlıklar 0.44-0.59 bandında takılı kalıyordu. Sebep fiziksel: gerçek
    # daire deseninin merkezcil ivmesi 5.9 m/s² (R=39 m, v=15.2), ama 20 Hz'de
    # bir adımda ürettiği konum farkı ½·5.9·0.05² = 0.0074 m — 1.5 m'lik ölçüm
    # gürültüsünün 200'de biri, yani görünmez. CV'yi katılaştırmadan ayrım
    # olmuyor. 0.5'te CV manevrayı açıklayamıyor ve IMM CA'ya geçiyor.
    SIGMA_A = 0.5        # m/s²  ; CV modelinin ivme toleransı (KATI olmalı)
    SIGMA_J = 8.0        # m/s³  ; CA modelinin jerk toleransı

    # ── Ölçüm gürültüsü ──
    SIGMA_Z = 1.5        # m ; telemetri konum gürültüsü (GPS + ağ)

    # ── IMM geçiş olasılıkları ──
    # Yapışkan: model bir kez seçilince kolay kolay bırakmasın (gürültüyle
    # model zıplaması komutu titretir). 0.95 kalma, 0.05 geçme.
    P_KAL = 0.95

    # ── Sayısal koruma ──
    DT_MIN = 1e-3
    DT_MAX = 2.0         # bundan uzun boşlukta filtre SIFIRLANIR (bayat veri)


def _F_cv(dt):
    """Sabit hız: p += v·dt. İvme durumu taşınmaz (0'a çekilir)."""
    F = np.eye(9)
    F[0:3, 3:6] = np.eye(3) * dt
    F[6:9, 6:9] = np.zeros((3, 3))       # a → 0
    return F


def _F_ca(dt):
    """Sabit ivme: p += v·dt + ½a·dt², v += a·dt."""
    F = np.eye(9)
    F[0:3, 3:6] = np.eye(3) * dt
    F[0:3, 6:9] = np.eye(3) * (0.5 * dt * dt)
    F[3:6, 6:9] = np.eye(3) * dt
    return F


def _Q_cv(dt, sigma_a):
    """Gürültü hıza binen sürekli-beyaz ivme modeli."""
    Q = np.zeros((9, 9))
    q = sigma_a ** 2
    Q[0:3, 0:3] = np.eye(3) * (q * dt ** 4 / 4.0)
    Q[0:3, 3:6] = np.eye(3) * (q * dt ** 3 / 2.0)
    Q[3:6, 0:3] = np.eye(3) * (q * dt ** 3 / 2.0)
    Q[3:6, 3:6] = np.eye(3) * (q * dt ** 2)
    Q[6:9, 6:9] = np.eye(3) * 1e-6       # a kullanılmıyor; tekil olmasın
    return Q


def _Q_ca(dt, sigma_j):
    """Gürültü ivmeye binen (jerk) model."""
    Q = np.zeros((9, 9))
    q = sigma_j ** 2
    Q[0:3, 0:3] = np.eye(3) * (q * dt ** 6 / 36.0)
    Q[0:3, 3:6] = np.eye(3) * (q * dt ** 5 / 12.0)
    Q[0:3, 6:9] = np.eye(3) * (q * dt ** 4 / 6.0)
    Q[3:6, 0:3] = np.eye(3) * (q * dt ** 5 / 12.0)
    Q[3:6, 3:6] = np.eye(3) * (q * dt ** 4 / 4.0)
    Q[3:6, 6:9] = np.eye(3) * (q * dt ** 3 / 2.0)
    Q[6:9, 0:3] = np.eye(3) * (q * dt ** 4 / 6.0)
    Q[6:9, 3:6] = np.eye(3) * (q * dt ** 3 / 2.0)
    Q[6:9, 6:9] = np.eye(3) * (q * dt ** 2)
    return Q


_H = np.zeros((3, 9))
_H[0:3, 0:3] = np.eye(3)


class _Model:
    """Tek bir Kalman süzgeci (CV ya da CA)."""

    def __init__(self, F_fn, Q_fn, gurultu):
        self._F_fn, self._Q_fn, self._gurultu = F_fn, Q_fn, gurultu
        self.x = np.zeros(9)
        self.P = np.eye(9) * 1e3

    def tahmin(self, dt):
        F = self._F_fn(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q_fn(dt, self._gurultu)

    def guncelle(self, z, R):
        """Dönüş: bu ölçümün bu modele göre olabilirliği (likelihood)."""
        y = z - _H @ self.x                       # yenilik
        S = _H @ self.P @ _H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return 1e-12
        K = self.P @ _H.T @ S_inv
        self.x = self.x + K @ y
        I_KH = np.eye(9) - K @ _H
        # Joseph formu: simetri ve pozitif tanımlılık sayısal olarak korunur
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        det = np.linalg.det(S)
        if det <= 0:
            return 1e-12
        ust = float(-0.5 * y.T @ S_inv @ y)
        L = math.exp(max(-700.0, ust)) / math.sqrt(((2 * math.pi) ** 3) * det)
        return max(L, 1e-12)


class IMM:
    """Interacting Multiple Model: CV + CA.

    guncelle(z, dt) her yeni TELEMETRİ ölçümünde çağrılır. Ölçüm gelmeyen
    karelerde tahmin(dt) ile ileri taşınabilir (yarışmada telemetri 1-2 Hz,
    güdüm 20 Hz → aradaki kareler bununla doldurulur).
    """

    def __init__(self, cfg=Cfg):
        self.cfg = cfg
        self._kur()

    def _kur(self):
        self.cv = _Model(_F_cv, _Q_cv, self.cfg.SIGMA_A)
        self.ca = _Model(_F_ca, _Q_ca, self.cfg.SIGMA_J)
        self.w = np.array([0.5, 0.5])            # model olasılıkları [cv, ca]
        p, q = self.cfg.P_KAL, 1.0 - self.cfg.P_KAL
        self.Pi = np.array([[p, q], [q, p]])     # geçiş matrisi
        self.baslatildi = False
        self.R = np.eye(3) * (self.cfg.SIGMA_Z ** 2)

    def sifirla(self):
        self._kur()

    # ── karıştırma (IMM'in "interacting" kısmı) ──
    def _karistir(self):
        c = self.Pi.T @ self.w                   # normalizasyon
        c = np.maximum(c, 1e-12)
        mu = (self.Pi * self.w[:, None]) / c[None, :]   # mu[i,j]: i→j ağırlığı
        modeller = [self.cv, self.ca]
        x_yeni, P_yeni = [], []
        for j in range(2):
            xj = sum(mu[i, j] * modeller[i].x for i in range(2))
            Pj = np.zeros((9, 9))
            for i in range(2):
                d = (modeller[i].x - xj).reshape(-1, 1)
                Pj += mu[i, j] * (modeller[i].P + d @ d.T)
            x_yeni.append(xj)
            P_yeni.append(Pj)
        for j, m in enumerate(modeller):
            m.x, m.P = x_yeni[j], P_yeni[j]
        return c

    # ── API: TAHMİN ve ÖLÇÜM AYRI ──
    # Bunlar bilerek ayrıldı. Tek bir guncelle(z, dt) çağrısı hem ilerletip hem
    # ölçüm uygularsa, 20 Hz döngüde 1-2 Hz telemetriyle çalışan çağıran taraf
    # zamanı ÇİFT SAYAR: ara kareleri tahmin(dt) ile ilerletir, sonra ölçüm
    # geldiğinde guncelle(z, Δt_ölçüm) aynı süreyi bir daha ilerletir.
    # Ölçüldü (2026-08-04): bu hata 1 Hz telemetride hız hatasını 0.9 m/s'den
    # 7.9 m/s'ye çıkarıyordu — EMA'dan 9 kat KÖTÜ.
    # Doğru kullanım (gerçek güdüm döngüsü):
    #     her kare:            kf.tahmin(dt_kare)
    #     telemetri geldiyse:  kf.olcum(z)

    def tahmin(self, dt):
        """Zamanı ilerlet (ölçüm yok). Her güdüm karesinde çağrılır."""
        if not self.baslatildi or dt is None or dt <= 0:
            return self.durum()
        if dt > self.cfg.DT_MAX:
            self.baslatildi = False          # bayat: hız/ivme artık geçersiz
            return self.durum()
        self.cv.tahmin(max(dt, self.cfg.DT_MIN))
        self.ca.tahmin(max(dt, self.cfg.DT_MIN))
        return self.durum()

    def olcum(self, z):
        """Yeni konum ölçümü uygula (zaman İLERLETMEZ)."""
        z = np.asarray(z, dtype=float)
        if not self.baslatildi:
            for m in (self.cv, self.ca):
                m.x = np.zeros(9)
                m.x[0:3] = z
                m.P = np.eye(9) * 1e2
                m.P[0:3, 0:3] = self.R * 4.0
            self.w = np.array([0.5, 0.5])
            self.baslatildi = True
            return self.durum()

        c = self._karistir()
        L = np.array([self.cv.guncelle(z, self.R),
                      self.ca.guncelle(z, self.R)])
        w_yeni = L * c
        toplam = w_yeni.sum()
        self.w = (w_yeni / toplam) if toplam > 1e-300 else np.array([0.5, 0.5])
        # tam 0/1'e kilitlenmesin — kilitlenirse diğer model bir daha uyanamaz
        self.w = np.clip(self.w, 1e-4, 1.0 - 1e-4)
        self.w /= self.w.sum()
        return self.durum()

    def guncelle(self, z, dt):
        """Kolaylık: tahmin(dt) + olcum(z). Ara karelerde tahmin() ÇAĞIRMAYAN
        basit kullanım için (telemetri hızı = güdüm hızı olduğunda)."""
        self.tahmin(dt)
        return self.olcum(z)

    def durum(self):
        """Birleştirilmiş kestirim (model olasılıklarıyla ağırlıklı)."""
        if not self.baslatildi:
            return {"p": (0.0, 0.0, 0.0), "v": (0.0, 0.0, 0.0),
                    "a": (0.0, 0.0, 0.0), "w_cv": 0.5, "w_ca": 0.5,
                    "hazir": False}
        x = self.w[0] * self.cv.x + self.w[1] * self.ca.x
        return {"p": tuple(x[0:3]), "v": tuple(x[3:6]), "a": tuple(x[6:9]),
                "w_cv": float(self.w[0]), "w_ca": float(self.w[1]),
                "hazir": True}
