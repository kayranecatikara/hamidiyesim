#!/usr/bin/env python3
"""tools/yorunge_isaret_testi.py — Yörünge kamerası KARAR DENEYİ + işaret ölçümü.

⚠ ŞU AN KOŞULAMAZ: dünya SDF'inde yorunge_iris / yorunge_talon modelleri YOK
(o değişiklik geri alındı). Bu araç, modeller dünyaya eklenip Gazebo yeniden
başlatıldığında koşulmak üzere hazır duruyor. Modeller yokken 1. adımda
"KAMERA RENDER ETMİYOR" ya da 4. adımda "set_pose REDDEDİLDİ" diyerek
temiz biçimde çıkar; hiçbir şeyi bozmaz.

İki işi birden yapar:

  A) KARAR DENEYİ — <static>true</static> bir modele set_pose uygulandığında
     sahne GERÇEKTEN güncelleniyor mu? Bütün yörünge işi buna bağlı.
     Geçmezse model dinamik yapılmalı (yerçekimi kapalı, çarpışma yok).

  B) İŞARET ÖLÇÜMÜ — AZIMUT_ISARETI ve YUKSELIS_ISARETI'ni deneyerek bulur.
     Kabul ölçütü: fareyi SAĞA sürüklerken uzak zemin/ufuk SAĞA kaymalı
     ("sahneyi tutup sürükleme" hissi = Gazebo GUI'sinin orbit davranışı).

⚠ Yalnız EKRANA basar; hiçbir dosya oluşturmaz, hiçbir şeyi değiştirmez.
⚠ AVCI_YORUNGE=0 ile koşun ki sürücü iş parçacıkları kamerayı geri çekmesin.
⚠ Yörünge matematiği bu dosyada KOPYALANMAZ — control.yorunge_kamera'dan
  içe aktarılır, yani test SEVK EDİLEN kodu sınar.

Kullanım:
    cd ~/projects/hamidiyesim && source /opt/ros/humble/setup.bash
    AVCI_YORUNGE=0 python3 tools/yorunge_isaret_testi.py --panel talon_chase
"""

import argparse
import hashlib
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from control import yorunge_kamera as Y

TOPIKLER = {"iris_chase": "/iris_chase/image", "talon_chase": "/talon_chase/image"}


class KareAlici:
    """En son kareyi tutar + sayaç. 'Oturdu' kararı sayaçla verilir: kaç kare
    geçtiğini bilmeden absdiff karşılaştırmak, eski kareyle yeni pozu
    karşılaştırma riskini taşır."""

    def __init__(self):
        self.kilit = threading.Lock()
        self.kare = None
        self.n = 0

    def cb(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
        except Exception:
            return
        with self.kilit:
            self.kare = arr.copy()
            self.n += 1

    def al(self):
        with self.kilit:
            return (None, self.n) if self.kare is None else (self.kare.copy(), self.n)


def _poz_yaz(node, Pose, Boolean, model, pos, quat, istek_ms=500):
    msg = Pose()
    msg.name = model
    msg.position.x, msg.position.y, msg.position.z = pos
    msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = quat
    ok, rep = node.request(f"/world/{Y.DUNYA}/set_pose", msg, Pose, Boolean, istek_ms)
    return bool(ok) and bool(rep.data)


def _poz_oku(node, Pose_V, model, sure=3.0):
    """pose/info'dan modelin pozunu oku (set_pose yankı doğrulaması)."""
    sonuc = {}
    ev = threading.Event()

    def cb(msg):
        for p in msg.pose:
            if p.name == model:
                sonuc["pos"] = (p.position.x, p.position.y, p.position.z)
                ev.set()

    node.subscribe(Pose_V, f"/world/{Y.DUNYA}/pose/info", cb)
    ev.wait(sure)
    return sonuc.get("pos")


def _otur(alici, min_kare=3, min_s=0.4, azami_s=4.0):
    """Yeni pozun render'a yansımasını bekle: >=min_kare YENİ kare VE >=min_s."""
    _, n0 = alici.al()
    t0 = time.time()
    while time.time() - t0 < azami_s:
        time.sleep(0.05)
        kare, n = alici.al()
        if n - n0 >= min_kare and time.time() - t0 >= min_s:
            return kare
    return alici.al()[0]


def _mavi_direk(kare):
    """`axes` modelinin 10 m'lik MAVİ direğinin görüntüdeki ağırlık merkezi.

    Direk <emissive> — ışıktan bağımsız ve doygun, bu yüzden eşikleme
    güvenilir. Piksel ağırlık merkezinde İŞARET BELİRSİZLİĞİ YOKTUR:
    +x sağ, +y aşağıdır (tanım gereği). Ölçümün gücü buradan geliyor.
    Kare RGB (gz sırası)."""
    r = kare[:, :, 0].astype(np.int16)
    g = kare[:, :, 1].astype(np.int16)
    b = kare[:, :, 2].astype(np.int16)
    maske = (b > 120) & (b > r + 60) & (b > g + 60)
    if maske.sum() < 50:
        return None
    ys, xs = np.nonzero(maske)
    return float(xs.mean()), float(ys.mean()), int(maske.sum())


def _sablon_kaydir(a, b):
    """Yedek ölçüm: çimen dokusunda yatay kayma (matchTemplate).

    ⚠ ÖNCE KENDİNİ KALİBRE EDER: bilinen bir kaydırmada (+10 px) doğru işareti
    döndürdüğü sınanır. Bu adım atlanırsa klasik işaret tuzağına düşülür."""
    H, W = a.shape[:2]
    y0, y1 = int(H * 0.60), int(H * 0.92)
    x0, x1 = W // 2 + 40, W // 2 + 200
    yama = a[y0:y1, x0:x1]
    if yama.size == 0:
        return None

    def _kay(hedef):
        res = cv2.matchTemplate(hedef, yama, cv2.TM_CCOEFF_NORMED)
        _, _, _, loc = cv2.minMaxLoc(res)
        return loc[0] - x0

    kalib = _kay(np.roll(a, +10, axis=1))
    if abs(kalib - 10) > 2:
        print(f"    [!] şablon kalibrasyonu BAŞARISIZ (bekl +10, ölçülen {kalib}) "
              f"— bu ölçüm yok sayılıyor")
        return None
    return _kay(b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--panel", default="talon_chase", choices=list(TOPIKLER))
    ap.add_argument("--adim", type=float, default=6.0, help="azimut/yükseliş probu (derece)")
    ap.add_argument("--yaricap", type=float, default=15.0)
    args = ap.parse_args()

    panel = args.panel
    model, anahtar = Y.MODELLER[panel]
    topik = TOPIKLER[panel]

    from gz.transport13 import Node
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.image_pb2 import Image
    from control import sim_truth

    print("═" * 72)
    print(f"YÖRÜNGE KARAR DENEYİ — panel={panel} model={model} dünya={Y.DUNYA}")
    print("═" * 72)

    node = Node()
    alici = KareAlici()
    if not node.subscribe(Image, topik, alici.cb):
        print(f"✗ {topik} aboneliği kurulamadı")
        return 2

    # ── 1) RENDER KANITI ────────────────────────────────────────────────
    print(f"\n[1] Render kanıtı — {topik} kare veriyor mu? (15 s)")
    t0 = time.time()
    while time.time() - t0 < 15 and alici.al()[0] is None:
        time.sleep(0.2)
    kare, n = alici.al()
    if kare is None:
        print(f"    ✗ KAMERA RENDER ETMİYOR — 15 s'de tek kare yok.")
        print(f"    → Gazebo yörünge modelleri eklenmiş dünyayla başlatıldı mı?")
        print(f"      gz topic -l | grep {panel}")
        return 1
    print(f"    ✓ kare geldi: {kare.shape[1]}x{kare.shape[0]}, {n} kare")

    std = float(kare.std())
    print(f"[2] İçerik sağlaması — std={std:.1f}")
    if std <= 5:
        print(f"    ✗ Kare neredeyse düz (boş gökyüzü?) — karşılaştırmalar anlamsız olur.")
        print(f"    → Kızağın başlangıç <pose>'unu düzelt.")
        return 1
    print(f"    ✓ dokulu görüntü")

    # ── 3) Hedef aracın pozu ────────────────────────────────────────────
    sim_truth.start()
    time.sleep(1.5)
    arac, yaw, kaynak = Y._arac_pozu(sim_truth, anahtar)
    hedef = arac["pos"]
    print(f"[3] Hedef araç ({anahtar}): pos={tuple(round(v,2) for v in hedef)} "
          f"yaw={yaw if yaw is None else round(yaw,1)}° kaynak={kaynak}")

    # ── 4) Dört poz: A(baz) B(azimut+) C(yükseliş+) D(başa dönüş) ───────
    r = args.yaricap
    A_AZ, A_EL = 90.0, 20.0
    d = args.adim
    print(f"\n[4] Dört poz (r={r} m, adım={d}°) — set_pose + oturma bekleme")

    kareler = {}
    for etiket, az, el in (("A", A_AZ, A_EL), ("B", A_AZ + d, A_EL),
                           ("C", A_AZ, A_EL + d), ("D", A_AZ, A_EL)):
        pos, quat = Y._poz_hesapla(hedef, az, el, r)
        pos = (pos[0], pos[1], max(pos[2], Y.KAMERA_MIN_Z))
        ok = _poz_yaz(node, Pose, Boolean, model, pos, quat)
        if not ok:
            print(f"    ✗ {etiket}: set_pose REDDEDİLDİ (ok/rep.data false)")
            print(f"    → Sahnede '{model}' yok. Eski dünya mı koşuyor?")
            return 1
        kareler[etiket] = _otur(alici)
        yanki = _poz_oku(node, Pose_V, model) if etiket == "A" else None
        ek = ""
        if yanki:
            sapma = math.dist(yanki, pos)
            ek = f" | pose/info yankısı sapma {sapma*1000:.2f} mm"
            if sapma > 1e-3:
                ek += "  ⚠ >1 mm"
        print(f"    {etiket}: az={az:6.1f}° el={el:5.1f}° → "
              f"({pos[0]:7.2f},{pos[1]:7.2f},{pos[2]:6.2f}){ek}")

    if any(k is None for k in kareler.values()):
        print("    ✗ Bazı pozlarda kare alınamadı")
        return 1

    # ── 5) KARAR ────────────────────────────────────────────────────────
    def _fark(u, v):
        return float(cv2.absdiff(u, v).mean())

    def _md5(u):
        return hashlib.md5(u.tobytes()).hexdigest()[:12]

    ab, ad = _fark(kareler["A"], kareler["B"]), _fark(kareler["A"], kareler["D"])
    ac = _fark(kareler["A"], kareler["C"])
    print(f"\n[5] KARAR")
    print(f"    A/B farkı (azimut değişti)  = {ab:6.2f}   (geçmesi için > 5.0)")
    print(f"    A/C farkı (yükseliş değişti)= {ac:6.2f}")
    print(f"    A/D farkı (AYNI poza dönüş) = {ad:6.2f}   (geçmesi için < 2.0)")
    print(f"    md5: A={_md5(kareler['A'])} B={_md5(kareler['B'])} D={_md5(kareler['D'])}")
    print(f"    ⓘ A/D kontrolü şart: 'avci' canlı bir dünya, görüntü kendiliğinden")
    print(f"      de değişir. 'Aynı poz → aynı kare' olmadan 'kamera oynadı' ile")
    print(f"      'dünya oynadı' ayrılamaz.")

    gecti = ab > 5.0 and _md5(kareler["A"]) != _md5(kareler["B"]) and ad < 2.0
    if not gecti:
        print(f"\n    ✗ KALDI — statik model set_pose ile RENDER'da HAREKET ETMİYOR")
        print(f"    → avci_harmonic.sdf'te iki yörünge modelini dinamik yap:")
        print(f"        <static>false</static>  ve <link> İÇİNDE <gravity>false</gravity>")
        print(f"        (⚠ <gravity> bir LINK öğesidir; model düzeyinde sessizce yok sayılır)")
        print(f"        mass 0.01, çarpışma geometrisi YOK")
        print(f"    → Gazebo'yu yeniden başlat ve bu testi tekrar koş.")
        return 1
    print(f"\n    ✓ GEÇTİ — <static>true</static> + set_pose sahneyi güncelliyor.")

    # ── 6) İŞARET ÖLÇÜMÜ ────────────────────────────────────────────────
    print(f"\n[6] İŞARET ÖLÇÜMÜ")
    mA, mB, mC = (_mavi_direk(kareler[k]) for k in "ABC")
    if mA and mB and mC:
        dx, dy = mB[0] - mA[0], mC[1] - mA[1]
        print(f"    yöntem: `axes` mavi direği (piksel ağırlık merkezi)")
        print(f"    A=({mA[0]:.1f},{mA[1]:.1f}) {mA[2]}px  "
              f"B=({mB[0]:.1f},{mB[1]:.1f})  C=({mC[0]:.1f},{mC[1]:.1f})")
    else:
        dx = _sablon_kaydir(kareler["A"], kareler["B"])
        dy = None
        print(f"    yöntem: çimen şablonu (mavi direk kadrajda değil)")
        if dx is None:
            print(f"    ✗ İşaret ölçülemedi — kamerayı direği görecek şekilde konumla")
            print(f"      (--panel talon_chase dene: `axes` onun arka planında)")
            return 1

    print(f"    azimut +{d}° → arka plan yatay kayma dx = {dx:+.1f} px "
          f"({'SAĞA' if dx > 0 else 'SOLA'})")
    az_isaret = +1 if dx > 0 else -1
    print(f"    → sağa sürüklemede (dx>0) arka plan sağa kayması için "
          f"AZIMUT_ISARETI = {az_isaret:+d}")

    if dy is not None:
        print(f"    yükseliş +{d}° → dikey kayma dy = {dy:+.1f} px "
              f"({'AŞAĞI' if dy > 0 else 'YUKARI'})")
        yuk_isaret = -1 if dy < 0 else +1
        print(f"    → 'aşağı sürükle (dy>0) → kamera aracın altına alçalsın' için "
              f"YUKSELIS_ISARETI = {yuk_isaret:+d}")
    else:
        yuk_isaret = None
        print(f"    yükseliş ölçülemedi (şablon yöntemi yalnız yatay)")

    print(f"\n{'═'*72}")
    print(f"SONUÇ  AZIMUT_ISARETI = {az_isaret:+d}"
          + (f"   YUKSELIS_ISARETI = {yuk_isaret:+d}" if yuk_isaret else ""))
    print(f"       şu andaki kod:  AZIMUT_ISARETI = {Y.AZIMUT_ISARETI:+d}   "
          f"YUKSELIS_ISARETI = {Y.YUKSELIS_ISARETI:+d}")
    uyum = (az_isaret == Y.AZIMUT_ISARETI
            and (yuk_isaret is None or yuk_isaret == Y.YUKSELIS_ISARETI))
    print(f"       {'✓ koddaki sabitler ölçümle UYUMLU' if uyum else '⚠ SABİTLER GÜNCELLENMELİ'}")
    print(f"{'═'*72}")
    print(f"\nNihai kabul sayısal değil İNSANİDİR: paneli aç, sağa sürükle, zeminin")
    print(f"sağa kaydığını gör; Gazebo GUI'sini yanına açıp aynı sürüklemenin aynı")
    print(f"yönü ürettiğini doğrula.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
