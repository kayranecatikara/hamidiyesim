#!/usr/bin/env python3
"""
tools/frpn_replay.py — KAPALI ÇEVRİM GÜDÜM TEZGÂHI (F3).

    python3 tools/frpn_replay.py                # tüm kollar × tüm yörüngeler
    python3 tools/frpn_replay.py --tara         # FRPN katsayı grid-search'ü
    python3 tools/frpn_replay.py --kol D        # tek kol

NEDEN KONTROLLÜ KARŞILAŞTIRMA (ve neden "FRPN iyi, eski kötü" DEĞİL):
Kullanıcının sorusu üzerine iki yasanın cebiri açıldı ve AYNI ÜÇ TERİMLİ YAPIDA
oldukları görüldü:

  mevcut : v_cmd = v_hedef + KP_H·Δp·û            + KD_H·Δv     (0.8 / 0.20)
  FRPN   : v_cmd = v_hedef + (V_c + K_ZEM|Δv|)·û  + K_ZEM·Δv    (—   / 1.00)

Yani mevcut güdümde de lead VAR (KD_H terimi), sadece 5× zayıf. Dahası denge
analizi (v_d = v_cmd varsayımıyla) Δv = −c/(1+K)·û verir, yani K≥0 için her
iki yasa da çarpışma rotasına oturur; K yalnız NE KADAR HIZLI oturduğunu
belirler. Bu durumda "FRPN gerekli mi yoksa KD_H'yi büyütmek yeter mi?"
sorusunun cevabı ölçümle verilmelidir — bu tezgâh onun için var.

DRONE MODELİ: nokta-kütle, ölçülmüş zarfla (2026-08-04, 212407 uçuşu):
    düz uçuşta tepe hız      ~18 m/s   (log: medyan 17.3, %95 18.7)
    dairede tepe hız         ~14.4 m/s (log: medyan 14.2, %95 14.4)
    yanal ivme tavanı         9.31 m/s² (log %95; ANGLE_MAX 45° → teorik 9.81)
    komut gecikmesi           1 kare @20 Hz
Hız tavanı sabit değil, ivme bütçesinden TÜRETİLİR: dönüşte merkezcil ivme
bütçeyi yediği için ileri hız kendiliğinden düşer — gerçek aracın davranışı bu.
"""

import argparse
import math
import statistics as st

from control.guidance import frpn
from control.guidance.hedef_kestirim import IMM

# ═══════════════════════════ ARAÇ MODELİ ═══════════════════════════

class Arac:
    A_MAX = 9.31          # m/s²  toplam yatay ivme bütçesi (ölçüldü)
    V_MAX = 18.0          # m/s   düz uçuşta tepe hız (ölçüldü)
    VZ_MAX = 6.0
    AZ_MAX = 2.5          # m/s²  WPNAV_ACCEL_Z
    JERK = 4.0            # m/s³  WPNAV_JERK
    GECIKME_KARE = 1      # komut gecikmesi

    def __init__(self, p, v):
        self.p = list(p)
        self.v = list(v)
        self.a = [0.0, 0.0, 0.0]
        self._kuyruk = []

    def adim(self, v_cmd, dt):
        # komut gecikmesi
        self._kuyruk.append(list(v_cmd))
        if len(self._kuyruk) <= self.GECIKME_KARE:
            v_hedef = list(self.v)
        else:
            v_hedef = self._kuyruk.pop(0)

        # hız tavanı (yatay/dikey ayrı)
        vh = math.hypot(v_hedef[0], v_hedef[1])
        if vh > self.V_MAX and vh > 1e-9:
            s = self.V_MAX / vh
            v_hedef[0] *= s
            v_hedef[1] *= s
        v_hedef[2] = max(-self.VZ_MAX, min(self.VZ_MAX, v_hedef[2]))

        # istenen ivme
        a_ist = [(v_hedef[i] - self.v[i]) / dt for i in range(3)]
        ah = math.hypot(a_ist[0], a_ist[1])
        if ah > self.A_MAX and ah > 1e-9:
            s = self.A_MAX / ah
            a_ist[0] *= s
            a_ist[1] *= s
        a_ist[2] = max(-self.AZ_MAX, min(self.AZ_MAX, a_ist[2]))

        # jerk sınırı
        for i in range(3):
            d = a_ist[i] - self.a[i]
            tavan = self.JERK * dt
            if d > tavan:
                d = tavan
            elif d < -tavan:
                d = -tavan
            self.a[i] += d

        for i in range(3):
            self.v[i] += self.a[i] * dt
            self.p[i] += self.v[i] * dt
        return math.hypot(self.a[0], self.a[1])


# ═══════════════════════════ HEDEF YÖRÜNGELERİ ═══════════════════════════
# Ölçülmüş gerçek değerlerle: hedef 15.2-16 m/s, daire R=39 m (212407 uçuşu).

def yr_duz(t):
    return (16.0 * t, 0.0, -50.0), (16.0, 0.0, 0.0)


def yr_daire(t, R=39.0, v=15.2):
    w = v / R
    return ((R * math.cos(w * t), R * math.sin(w * t), -50.0),
            (-v * math.sin(w * t), v * math.cos(w * t), 0.0))


# ⚠ YÖRÜNGELER HEDEFİN GERÇEK ZARFININ İÇİNDE OLMALI.
# İlk sürümde zikzak (T=8 s, A=40 m) hedeften 24.7 m/s² yanal ivme istiyordu;
# gerçek hedef uçak ölçüldüğünde 15.2 m/s'de %90 dilimi 34.6°/s = 9.2 m/s².
# Yapamayacağı bir manevrayı kovalayamamak güdümün kusuru değildir — o test
# hiçbir kolu ayırt etmeden hepsini birden düşürüyordu. Değerler hedefin
# ölçülmüş zarfına çekildi:
#     daire  R=39 v=15.2  → 5.92 m/s²   (gerçek uçuştan birebir)
#     sekiz  R=55 v=15.5  → 7.6  m/s²
#     zikzak T=12 A=25    → 6.85 m/s²

def yr_sekiz(t, R=55.0, v=15.5):
    w = v / R
    return ((R * math.sin(w * t), R * math.sin(w * t) * math.cos(w * t), -50.0),
            (R * w * math.cos(w * t),
             R * w * math.cos(2 * w * t), 0.0))


def yr_zikzak(t, v=15.5, periyot=12.0, genlik=25.0):
    w = 2 * math.pi / periyot
    return ((v * t, genlik * math.sin(w * t), -50.0),
            (v, genlik * w * math.cos(w * t), 0.0))


YORUNGELER = [("düz", yr_duz), ("daire", yr_daire),
              ("sekiz", yr_sekiz), ("zikzak", yr_zikzak)]


# ═══════════════════════════ GÜDÜM KOLLARI ═══════════════════════════
# Her kol: (p_hedef, v_hedef, p_drone, v_drone, durum) → v_cmd

class KolMevcut:
    """A — bugünkü gps_guidance.py yasası: istasyon 1000 m'den itibaren,
    KP_H=0.8, KD_H=0.20."""
    ad = "A mevcut (KD=0.20, istasyon hep)"
    KP, KD = 0.8, 0.20
    GECIS = False

    def __init__(self):
        self.de = [0.0, 0.0, 0.0]
        self.e_prev = None

    def _istasyon(self, p_h, v_h, p_d):
        cfg = frpn.Cfg
        ist = math.radians(cfg.ISTASYON_ELEV_DEG)
        fark = [p_h[i] - p_d[i] for i in range(3)]
        menzil = math.sqrt(sum(x * x for x in fark))
        r_eff = min(menzil, cfg.RANGE_SET)
        w = 1.0
        if self.GECIS:
            if menzil >= cfg.GECIS_BASLA:
                w = 0.0
            elif menzil > cfg.GECIS_TAM:
                w = (cfg.GECIS_BASLA - menzil) / (cfg.GECIS_BASLA - cfg.GECIS_TAM)
        vh = math.hypot(v_h[0], v_h[1])
        if vh >= cfg.TRACK_MIN_SPD:
            b = (-v_h[0] / vh, -v_h[1] / vh, 0.0)
        else:
            dh = math.hypot(fark[0], fark[1]) or 1.0
            b = (-fark[0] / dh, -fark[1] / dh, 0.0)
        d_arka = r_eff * math.cos(ist) * w
        d_alt = r_eff * math.sin(ist) * w
        return (p_h[0] + b[0] * d_arka, p_h[1] + b[1] * d_arka, p_h[2] + d_alt)

    def komut(self, p_h, v_h, p_d, v_d, dt):
        st_p = self._istasyon(p_h, v_h, p_d)
        e = [st_p[i] - p_d[i] for i in range(3)]
        if self.e_prev is not None:
            for i in range(3):
                self.de[i] = 0.8 * self.de[i] + 0.2 * ((e[i] - self.e_prev[i]) / dt)
        self.e_prev = e
        return [v_h[i] + self.KP * e[i] + self.KD * self.de[i] for i in range(3)]


class KolGecis(KolMevcut):
    """B — mevcut yasa + menzile bağlı istasyon geçişi (tek değişken)."""
    ad = "B mevcut + menzil geçişi"
    GECIS = True


class KolLeadArtti(KolMevcut):
    """C — mevcut yasa + geçiş + LEAD kazancı büyütülmüş (ucuz alternatif).
    FRPN'e hiç geçmeden aynı etki alınabiliyor mu sorusunu sınar."""
    ad = "C mevcut + geçiş + KD=1.0"
    KD = 1.0
    GECIS = True


class KolFRPN:
    """D — FRPN hız formu."""
    ad = "D FRPN"

    def komut(self, p_h, v_h, p_d, v_d, dt):
        p_s, _ = frpn.sanal_hedef(p_h, v_h, p_d)
        dp = tuple(p_s[i] - p_d[i] for i in range(3))
        dv = tuple(v_h[i] - v_d[i] for i in range(3))
        return list(frpn.komut(dp, dv, v_h)["v_cmd"])


KOLLAR = {"A": KolMevcut, "B": KolGecis, "C": KolLeadArtti, "D": KolFRPN}


# ═══════════════════════════ KOŞUM ═══════════════════════════

DEVIR_MENZIL = 12.0        # m — başarı: bu menzile inmek (yeni devir hedefi)


def kos(kol_sinifi, yorunge_fn, sure=120.0, dt=0.05, telemetri_hz=None,
        kestirici=None, baslangic_menzil=200.0):
    """Tek koşum. Dönüş: metrikler."""
    kol = kol_sinifi()
    p_h0, v_h0 = yorunge_fn(0.0)
    # drone hedefin gerisinde, biraz altında başlar
    yon = math.atan2(v_h0[1], v_h0[0])
    p_d = [p_h0[0] - baslangic_menzil * math.cos(yon),
           p_h0[1] - baslangic_menzil * math.sin(yon),
           p_h0[2] + 5.0]
    arac = Arac(p_d, [v_h0[0], v_h0[1], 0.0])

    kf = IMM() if kestirici == "imm" else None
    ema_p, ema_v = None, [0.0, 0.0, 0.0]
    son_olcum_t = -1e9
    olcum_araligi = (1.0 / telemetri_hz) if telemetri_hz else dt

    menziller, doygunluk, t_devir = [], [], None
    n = int(sure / dt)
    for k in range(n):
        t = k * dt
        p_h, v_h = yorunge_fn(t)

        # ── kestirim ──
        if kf is not None:
            kf.tahmin(dt)
            if t - son_olcum_t >= olcum_araligi - 1e-9:
                kf.olcum(p_h)
                son_olcum_t = t
            d = kf.durum()
            p_kes, v_kes = (d["p"], d["v"]) if d["hazir"] else (p_h, v_h)
        else:
            # mevcut kestirici: EMA konum + sonlu fark hız
            if t - son_olcum_t >= olcum_araligi - 1e-9:
                if ema_p is None:
                    ema_p = list(p_h)
                else:
                    dtm = t - son_olcum_t
                    yeni = [0.4 * p_h[i] + 0.6 * ema_p[i] for i in range(3)]
                    if dtm > 1e-3:
                        for i in range(3):
                            ema_v[i] = (0.3 * ((yeni[i] - ema_p[i]) / dtm)
                                        + 0.7 * ema_v[i])
                    ema_p = yeni
                son_olcum_t = t
            p_kes, v_kes = tuple(ema_p or p_h), tuple(ema_v)

        v_cmd = kol.komut(p_kes, v_kes, arac.p, arac.v, dt)
        a = arac.adim(v_cmd, dt)
        doygunluk.append(a > Arac.A_MAX * 0.95)

        menzil = math.sqrt(sum((p_h[i] - arac.p[i]) ** 2 for i in range(3)))
        menziller.append(menzil)
        if t_devir is None and menzil <= DEVIR_MENZIL:
            t_devir = t

    # TUTUNMA SÜRESİ — asıl başarı ölçütü.
    # "12 m'ye bir kez değmek" yetmez: görsel fazın devralabilmesi için o
    # bantta KALMAK gerekir (20:52 uçuşunda devir oldu ama faz 3 saniyede
    # koptu). Bu yüzden 15 m altında geçirilen toplam süre de ölçülür.
    yakin_kare = sum(1 for m in menziller if m <= 15.0)
    return {"t_devir": t_devir, "min_menzil": min(menziller),
            "son_menzil": menziller[-1], "medyan_menzil": st.median(menziller),
            "sure_yakin": yakin_kare * dt,
            "doygunluk": 100.0 * sum(doygunluk) / len(doygunluk)}


def tablo(kestirici, telemetri_hz, baslik):
    print(f"\n{'═'*94}")
    print(f"  {baslik}")
    print(f"{'═'*94}")
    print(f"  {'kol':<32}", end="")
    for ad, _ in YORUNGELER:
        print(f"{ad:>15}", end="")
    print()
    print(f"  {'':<32}" + "".join(f"{'devir | tutunma':>15}" for _ in YORUNGELER))
    print(f"  {'-'*92}")
    puanlar = {}
    for anahtar, sinif in KOLLAR.items():
        print(f"  {sinif.ad:<32}", end="")
        tutunma_top = 0.0
        for _, yfn in YORUNGELER:
            r = kos(sinif, yfn, kestirici=kestirici, telemetri_hz=telemetri_hz)
            tutunma_top += r["sure_yakin"]
            t = f"{r['t_devir']:.0f}s" if r["t_devir"] is not None else f"{r['min_menzil']:.0f}m✗"
            print(f"{t:>7} |{r['sure_yakin']:>6.1f}s", end="")
        print()
        puanlar[anahtar] = tutunma_top
    print(f"  {'-'*92}")
    print("  devir  = 12 m'ye ilk inme süresi ('34m✗' = hiç inemedi, en yakın menzil)")
    print("  tutunma = 15 m altında geçirilen TOPLAM süre — asıl başarı ölçütü")
    return puanlar


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kol", help="tek kol koş (A/B/C/D)")
    ap.add_argument("--tara", action="store_true", help="FRPN katsayı taraması")
    args = ap.parse_args()

    if args.kol:
        sinif = KOLLAR[args.kol]
        print(f"\n{sinif.ad}")
        for ad, yfn in YORUNGELER:
            r = kos(sinif, yfn, kestirici="imm")
            t = f"{r['t_devir']:.1f}s" if r["t_devir"] else "—"
            print(f"  {ad:<8} devir={t:<8} en yakın={r['min_menzil']:6.1f} m  "
                  f"doygunluk %{r['doygunluk']:.0f}")
        return 0

    if args.tara:
        # AMAÇ FONKSİYONU = TOPLAM TUTUNMA SÜRESİ.
        # İlk sürümde "12 m'ye kaç yörüngede inildi" kullanılmıştı; o ölçüt
        # yanıltıcıydı — bir kez değip hemen uzaklaşan bir kol, yakın menzilde
        # uzun süre kalan koldan iyi görünüyordu. Görsel fazın ihtiyacı olan
        # PENCERE olduğu için tutunma süresi doğru ölçüt.
        # Ayrıca 20 Hz VE 1 Hz koşulunun ikisinde birden ölçülür: yalnız birini
        # iyileştiren katsayı yarışmada işe yaramaz.
        print("\nFRPN KATSAYI TARAMASI — amaç: toplam tutunma süresi (15 m altı)")
        print(f"  {'K_C':>6} {'V_C_MAX':>8} {'K_ZEM':>6}   {'20Hz':>8} {'1Hz':>8} {'toplam':>9}")
        eski = (frpn.Cfg.K_C, frpn.Cfg.V_C_MAX, frpn.Cfg.K_ZEM)
        sonuclar = []
        for kc in (0.15, 0.25, 0.4, 0.6):
            for vmax in (12.0, 16.0, 20.0):
                for kz in (0.3, 0.6, 1.0, 1.4):
                    frpn.Cfg.K_C, frpn.Cfg.V_C_MAX, frpn.Cfg.K_ZEM = kc, vmax, kz
                    t20 = sum(kos(KolFRPN, y, kestirici="imm")["sure_yakin"]
                              for _, y in YORUNGELER)
                    t01 = sum(kos(KolFRPN, y, kestirici="imm",
                                  telemetri_hz=1.0)["sure_yakin"]
                              for _, y in YORUNGELER)
                    sonuclar.append((t20 + t01, t20, t01, kc, vmax, kz))
        sonuclar.sort(reverse=True)
        for top, t20, t01, kc, vmax, kz in sonuclar[:10]:
            print(f"  {kc:>6.2f} {vmax:>8.1f} {kz:>6.2f}   "
                  f"{t20:>7.1f}s {t01:>7.1f}s {top:>8.1f}s")
        frpn.Cfg.K_C, frpn.Cfg.V_C_MAX, frpn.Cfg.K_ZEM = eski
        top, t20, t01, kc, vmax, kz = sonuclar[0]
        print(f"\n  EN İYİ: K_C={kc}  V_C_MAX={vmax}  K_ZEM={kz}")
        print(f"          20 Hz {t20:.1f}s + 1 Hz {t01:.1f}s = {top:.1f}s")
        print(f"  (mevcut varsayılan: K_C={eski[0]} V_C_MAX={eski[1]} K_ZEM={eski[2]})")
        return 0

    p1 = tablo(None, None, "1) MEVCUT KESTİRİCİ (EMA), telemetri = güdüm hızı (20 Hz)")
    p2 = tablo("imm", None, "2) IMM KESTİRİCİ, telemetri = güdüm hızı (20 Hz)")
    p3 = tablo("imm", 1.0, "3) IMM KESTİRİCİ, telemetri 1 Hz (YARIŞMA KOŞULU)")

    print(f"\n{'═'*94}")
    print("  ÖZET — 4 yörüngede TOPLAM tutunma süresi (15 m altı, saniye)")
    print("         Yüksek = iyi. Görsel fazın devralabileceği pencere bu.")
    print(f"{'═'*94}")
    print(f"  {'kol':<32}{'EMA/20Hz':>12}{'IMM/20Hz':>12}{'IMM/1Hz':>12}")
    for anahtar, sinif in KOLLAR.items():
        print(f"  {sinif.ad:<32}{p1[anahtar]:>11.1f}s{p2[anahtar]:>11.1f}s"
              f"{p3[anahtar]:>11.1f}s")
    en_iyi = max(p3, key=lambda k: p3[k])
    print(f"\n  Yarışma koşulunda (IMM/1Hz) en iyi kol: "
          f"{KOLLAR[en_iyi].ad}  ({p3[en_iyi]:.1f}s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
