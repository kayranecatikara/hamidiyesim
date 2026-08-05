"""tools/gecis_analiz.py — kara kutu geometrisinin saf hesabı.

    python3 -m tests.olcum_araclari.test_gecis_analiz

Bu araç "drone nereden ıskaladı" sorusunun hakemi: iki aracın `.BIN` kaydını
GPS haftası saatiyle hizalayıp aradaki yatay/dikey mesafeyi çıkarıyor. BIN
okuma kısmı burada test edilemez (kayıtlar depoda yok), ama hizalama sonrası
geometri saf hesap — sentetik pozlarla doğrulanır.

E1-E3  _geometri     mesafe, zaman enterpolasyonu, telemetri boşluğu
E4     DİKEY işareti sözleşmesi (alttan mı üstten mi geçti)
E5-E6  _yaklasmalar  yerel minimum yakalama + aynı geçişin tekrarını eleme
E7     _wrap         açı sarmalama
E8     _titreme      yaw takip hatası istatistiği
"""

import math

from tests.olcum_araclari.ortak import kontrol, ozet, sifirla
from tools import gecis_analiz as ga

M_LAT = 111320.0            # aracın kendi sabiti; testler aynısını kullanır


def _pos(t, kuzey_m=0.0, dogu_m=0.0, irtifa=0.0, lat0=0.0):
    """Metre ofsetinden (t, lat, lng, alt) üret — aracın ters dönüşümü."""
    mlon = M_LAT * math.cos(math.radians(lat0))
    return (t, lat0 + kuzey_m / M_LAT, dogu_m / mlon, irtifa)


def main():
    sifirla()
    print("Ölçüm aracı: gecis_analiz (kara kutu geometrisi)")
    print("=" * 60)

    # ── E1: kopter orijinde, uçak 10 m kuzeyde ve 3 m yukarıda ──
    K = {"pos": [_pos(1.0, irtifa=100.0)], "att": []}
    U = {"pos": [_pos(0.8, kuzey_m=10.0, irtifa=103.0),
                 _pos(1.2, kuzey_m=10.0, irtifa=103.0)], "att": []}
    G = ga._geometri(K, U)
    t_bagil, menzil, yatay, dikey = G[0]
    kontrol("E1  yatay/dikey/menzil sentetik pozdan doğru çıkıyor",
            len(G) == 1 and abs(yatay - 10.0) < 0.01 and abs(dikey - 3.0) < 0.01
            and abs(menzil - math.hypot(10.0, 3.0)) < 0.01,
            f"yatay={yatay:.3f} m  dikey={dikey:+.3f} m  menzil={menzil:.3f} m")

    # ── E2: uçak pozu kopter zaman damgasına ENTERPOLE edilmeli ──
    # İki SITL aynı anda örneklemiyor; hizalama yanlışsa menzil kayar.
    K = {"pos": [_pos(1.0, irtifa=100.0)], "att": []}
    U = {"pos": [_pos(0.8, kuzey_m=0.0, irtifa=100.0),
                 _pos(1.2, kuzey_m=20.0, irtifa=100.0)], "att": []}
    G = ga._geometri(K, U)
    kontrol("E2  uçak pozu kopter saatine enterpole ediliyor",
            len(G) == 1 and abs(G[0][2] - 10.0) < 0.01,
            f"t=1.0'da 0↔20 m arası → {G[0][2]:.3f} m (beklenen 10.000)")

    # ── E3: 0.5 s'den büyük telemetri boşluğu ATLANMALI ──
    # Boşluğa enterpolasyon uydurmak sessizce yanlış menzil üretirdi.
    K = {"pos": [_pos(1.0, irtifa=100.0)], "att": []}
    U = {"pos": [_pos(0.4, kuzey_m=0.0, irtifa=100.0),
                 _pos(1.4, kuzey_m=100.0, irtifa=100.0)], "att": []}
    kontrol("E3  1.0 s'lik telemetri boşluğu enterpole EDİLMİYOR",
            ga._geometri(K, U) == [], "kare atlandı (uydurma menzil yok)")

    # ── E4: DİKEY İŞARET SÖZLEŞMESİ — belgedeki okuma kuralının kendisi ──
    # + = uçak yukarıda = drone ALTTAN geçti. Ters dönerse her "alttan/üstten
    # geçti" hükmü tersine döner ve kimse fark etmez.
    K = {"pos": [_pos(1.0, irtifa=100.0)], "att": []}
    U_ust = {"pos": [_pos(0.9, irtifa=105.0), _pos(1.1, irtifa=105.0)], "att": []}
    U_alt = {"pos": [_pos(0.9, irtifa=95.0), _pos(1.1, irtifa=95.0)], "att": []}
    d_ust = ga._geometri(K, U_ust)[0][3]
    d_alt = ga._geometri(K, U_alt)[0][3]
    kontrol("E4  DİKEY işareti: + = uçak yukarıda (drone alttan geçti)",
            d_ust > 0 and d_alt < 0,
            f"uçak 5 m üstte → {d_ust:+.1f}   uçak 5 m altta → {d_alt:+.1f}")

    # ── E5: yerel menzil minimumu yakalanmalı, eşik üstü yok sayılmalı ──
    G = [(i * 1.0, m, m, 0.0) for i, m in
         enumerate([20.0, 15.0, 2.0, 15.0, 20.0, 15.0, 2.0, 15.0, 20.0])]
    y = ga._yaklasmalar(G, esik=12.0)
    kontrol("E5  yerel minimumlar yakalanıyor, 12 m üstü eleniyor",
            len(y) == 2 and abs(y[0][1] - 2.0) < 1e-9 and abs(y[1][1] - 2.0) < 1e-9,
            f"{len(y)} yaklaşma: " + ", ".join(f"t={a[0]:.1f}s→{a[1]:.1f}m" for a in y))

    # ── E6: 2 s'den yakın iki minimum AYNI geçiştir, tekrar sayılmamalı ──
    G_sik = [(i * 0.4, m, m, 0.0) for i, m in
             enumerate([20.0, 15.0, 2.0, 15.0, 20.0, 15.0, 2.0, 15.0, 20.0])]
    y_sik = ga._yaklasmalar(G_sik, esik=12.0)
    kontrol("E6  2 s içindeki ikinci minimum aynı geçiş sayılıyor",
            len(y_sik) == 1,
            f"aynı dizi 0.4 s aralıkla → {len(y_sik)} geçiş (1.0 s aralıkla 2 idi)")

    # ── E7: açı sarmalama — 350° aslında −10°'dir ──
    # Aralık YARI AÇIK: [−180, 180). Tam 180° −180'e düşer; ±180 civarındaki
    # yaw hatalarında işaret buradan geliyor.
    kontrol("E7  _wrap açıyı [−180, 180) aralığına taşıyor",
            abs(ga._wrap(350.0) + 10.0) < 1e-9 and abs(ga._wrap(-350.0) - 10.0) < 1e-9
            and abs(ga._wrap(180.0) + 180.0) < 1e-9 and abs(ga._wrap(0.0)) < 1e-9,
            f"350°→{ga._wrap(350.0):+.1f}°  −350°→{ga._wrap(-350.0):+.1f}°  "
            f"180°→{ga._wrap(180.0):+.1f}°")

    # ── E8: yaw takip hatası istatistiği (seyir penceresi 20-45 s) ──
    # DesYaw sabit, gerçek yaw ±2° salınıyor → std 2.0, tepe 4.0, her adımda
    # ortalamayı kestiği için hz = kare/2/süre.
    t0 = 1000.0
    att = [(t0 + 20.0 + k * 0.25, 100.0 + (2.0 if k % 2 == 0 else -2.0), 100.0)
           for k in range(100)]
    ttr = ga._titreme(att, t0)
    sure = 99 * 0.25
    kontrol("E8  _titreme yaw takip hatasını doğru özetliyor",
            ttr is not None and abs(ttr["std"] - 2.0) < 1e-6
            and abs(ttr["tepe"] - 4.0) < 1e-6 and abs(ttr["komut"]) < 1e-9
            and abs(ttr["gercek"] - 4.0) < 1e-6
            and abs(ttr["hz"] - 99 / 2 / sure) < 1e-6,
            f"std={ttr['std']:.2f}° tepe={ttr['tepe']:.2f}° "
            f"komut={ttr['komut']:.2f}°/kare gerçek={ttr['gercek']:.2f}°/kare")

    return ozet("gecis_analiz")


if __name__ == "__main__":
    import sys
    gecen, toplam = main()
    sys.exit(0 if gecen == toplam else 1)
