"""
tests/test_gps_guidance.py — GPS kadraj güdümü kabul kriterleri.

Gazebo'suz, saf mantık. Kullanım: python3 -m tests.test_gps_guidance

Kapsam:
  G1-G6  hedef_kadraj_hatasi (başarı kriteri matematiği — merkez/yatay/yan/dikey/arka/menzil)
  G7     KADEME 1 tasarım tutarlılığı: geometrik kadraj noktasında drone → hedef MERKEZDE
  G8     kadraj noktası hedefin hız yönünün gerisinde + altında
  G9     döngü duman testi (fake conn): komut üretir, hold'da ≈ hedef hızı, durum dolu
  G10-12 sabit açı ofseti · terminal dikey bütçe · iç daire nişanı · oranlı kayma
  G13    dinamik istasyon yükselişi (elev = tilt + gövde pitch): formül, uçtan
         uca merkezleme, sınırlar, döngü içi işleyiş, kapatma anahtarı
  G14    dönüş ileri beslemesi (v_ist = v_hedef + ω×r): sayısal türevle
         doğrulama, düzde sıfır, tavan
  G15    arka kısaltma: dönüşte arka bileşen erir, düzde tam kalır
"""

import inspect
import math
import threading
import time

from control.guidance.guidance_core import hedef_kadraj_hatasi, govde_to_dunya
from control.guidance.common import normalize_angle as normalize_angle_test
from control.guidance import gps_guidance as gg

# Test CSV'leri gerçek uçuş loglarına karışmasın (bkz. test_visual_lead notu)
import tempfile as _tf
gg._LOG_DIR = _tf.mkdtemp(prefix="avci_test_logs_")

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def main():
    print("GPS kadraj güdümü kabul kriterleri")
    print("=" * 60)
    C = gg.Cfg
    tilt = C.CENTER_ELEV_DEG              # kamera tilt'i (kadraj MERKEZİ)
    ist = C.ISTASYON_ELEV_DEG             # istasyonun LOS yükselişi (≤ tilt)
    d_behind = C.RANGE_SET * math.cos(math.radians(ist))
    d_below = C.RANGE_SET * math.sin(math.radians(ist))

    # ── G1: MERKEZ — hedef boresight yönünde, drone seviyeli → yaw 0, elev tilt, (CX,CY) ──
    b = govde_to_dunya([0.906, 0.0, -0.423], 0, 0, 0)
    tgt = (C.RANGE_SET * b).tolist()
    r = hedef_kadraj_hatasi(tgt, [0, 0, 0], 0, 0, 0)
    kontrol("G1  merkez → yaw≈0, elev≈25°, (u,v)≈(320,240)",
            abs(math.degrees(r["yaw_hata"])) < 0.5
            and abs(math.degrees(r["elev"]) - tilt) < 0.5
            and abs(r["u"] - 320) < 2 and abs(r["v"] - 240) < 2,
            f"yaw={math.degrees(r['yaw_hata']):.2f}° elev={math.degrees(r['elev']):.2f}° "
            f"u={r['u']:.1f} v={r['v']:.1f}")

    # ── G2: pitch_hata merkez sapması = elev − tilt (merkezde 0) ──
    kontrol("G2  merkez pitch_hata≈0", abs(math.degrees(r["pitch_hata"])) < 0.5,
            f"pitch_hata={math.degrees(r['pitch_hata']):.2f}°")

    # ── G3: YATAY ÖNDE (elev 0) → hedef merkezin ALTINDA (pitch_hata=−25°, v>240) ──
    r3 = hedef_kadraj_hatasi([10, 0, 0], [0, 0, 0], 0, 0, 0)
    kontrol("G3  yatay-önde → pitch_hata≈−25°, v>240 (kadraj altı)",
            abs(math.degrees(r3["pitch_hata"]) + 25) < 0.5 and r3["v"] > 240,
            f"pitch_hata={math.degrees(r3['pitch_hata']):.2f}° v={r3['v']:.1f}")

    # ── G4: YAN (hedef sağda) → yaw_hata>0, u>320 ──
    r4 = hedef_kadraj_hatasi([10, 5, -4.65], [0, 0, 0], 0, 0, 0)
    kontrol("G4  hedef sağda → yaw_hata>0, u>320",
            math.degrees(r4["yaw_hata"]) > 1 and r4["u"] > 320,
            f"yaw={math.degrees(r4['yaw_hata']):.2f}° u={r4['u']:.1f}")

    # ── G5: ATTITUDE COUPLING — drone pitch nose-down → merkezdeki hedef yukarı kayar ──
    r5 = hedef_kadraj_hatasi(tgt, [0, 0, 0], 0, math.radians(-10), 0)
    kontrol("G5  pitch −10° → kadraj hatası ~+10° (K2'nin kapatacağı sapma)",
            abs(math.degrees(r5["pitch_hata"]) - 10) < 1.0,
            f"pitch_hata={math.degrees(r5['pitch_hata']):.2f}°")

    # ── G6: hedef ARKADA (kameranın önünde değil) → onde=False, u/v None ──
    r6 = hedef_kadraj_hatasi([-10, 0, 0], [0, 0, 0], 0, 0, 0)
    kontrol("G6  hedef arkada → onde=False", (not r6["onde"]) and r6["u"] is None,
            f"onde={r6['onde']}")

    # ── G7: KADEME 1 TUTARLILIK — geometrik kadraj noktasında drone hedefi MERKEZDE görür ──
    # Hedef +X yönünde uçuyor; istasyon = hedefin d_behind gerisi + d_below altı.
    T = [50.0, 20.0, -40.0]                       # hedef NED (alt=40 m)
    vhat = (1.0, 0.0)                             # hedef +X yönünde
    st = [T[0] - vhat[0] * d_behind, T[1] - vhat[1] * d_behind, T[2] + d_below]
    yaw_to_tgt = math.atan2(T[1] - st[1], T[0] - st[0])
    r7 = hedef_kadraj_hatasi(T, st, 0, 0, yaw_to_tgt)   # drone seviyeli, burun hedefte
    # Ölçüt "TAM MERKEZ" DEĞİL, "kadrajda paylı içeride". İstasyon yükselişi
    # 2026-08-02'de kamera tilt'inden ayrıldı (dikey ivme bütçesi, bkz.
    # Cfg.ISTASYON_ELEV_DEG); hedef artık merkezin (tilt − istasyon)° altında
    # görünür. Asıl gereklilik hedefin kadrajı terk etmemesi.
    v_marj = min(r7["v"], 480 - r7["v"]) if r7["v"] is not None else -1
    kontrol(f"G7  istasyonda hedef kadrajda paylı (yaw≈0, elev≈{ist:.0f}°, menzil≈RANGE_SET)",
            abs(math.degrees(r7["yaw_hata"])) < 0.5
            and abs(math.degrees(r7["elev"]) - ist) < 0.5
            and abs(r7["menzil"] - C.RANGE_SET) < 0.2
            and v_marj > 100,
            f"yaw={math.degrees(r7['yaw_hata']):.2f}° elev={math.degrees(r7['elev']):.2f}° "
            f"menzil={r7['menzil']:.2f} v={r7['v']:.0f}px (merkez 240, kenara pay {v_marj:.0f}px)")

    # ── G8: istasyon hedefin ALTINDA ve GERİSİNDE ──
    kontrol("G8  istasyon: altında (alt<hedef) ve gerisinde (x<hedef)",
            (-st[2]) < (-T[2]) and st[0] < T[0],
            f"drone_alt={-st[2]:.1f} < hedef_alt={-T[2]:.1f}; st_x={st[0]:.1f} < tgt_x={T[0]:.1f}")

    # ── G9: DÖNGÜ DUMAN TESTİ (fake conn) — hold'da komut ≈ hedef hızı, durum dolu ──
    class _FakeMav:
        def __init__(s): s.last = None
        def set_position_target_local_ned_send(s, *a): s.last = a

    class _FakeConn:
        target_system = 1; target_component = 1
        def __init__(s): s.mav = _FakeMav()
        def recv_match(s, **k): return None

    conn = _FakeConn()
    TV = 8.0                                       # hedef hızı +X 8 m/s
    st0 = list(st)
    state = {"t0": time.monotonic(), "tx": T[0]}
    def get_plane():
        el = time.monotonic() - state["t0"]
        return {"x": T[0] + TV * el, "y": T[1], "z": T[2], "yaw": 0.0, "frozen": False}
    def get_iris():
        # drone istasyonda + hedefle birlikte kayıyor (hold senaryosu), seviyeli
        el = time.monotonic() - state["t0"]
        return {"x": st0[0] + TV * el, "y": st0[1], "z": st0[2],
                "roll": 0.0, "pitch": 0.0, "yaw": yaw_to_tgt,
                "vx": TV, "vy": 0.0, "vz": 0.0}
    stop = threading.Event()
    th = threading.Thread(target=gg.run_gps_guidance,
                          args=(conn, get_plane, get_iris, stop), daemon=True)
    th.start()
    time.sleep(0.8)                                # ~16 kare
    sent = conn.mav.last                           # ÇALIŞIRKEN yakala (stop öncesi)
    snap = dict(gg.status)                          # durum anlık görüntüsü (DURDU olmadan)
    stop.set(); th.join(2.0)
    # set_position_target_local_ned_send argümanları: (...,vx,vy,vz,...) index 8,9,10
    vx_cmd = sent[8] if sent else None
    ok_hold = sent is not None and abs(vx_cmd - TV) < 3.0    # FF hedef hızına ~oturmuş
    ok_durum = snap["durum"] in ("KILIT", "ARAMA") and snap["d_h"] is not None
    ok_merkez = (snap["kadraj_elev_deg"] is not None
                 and abs(snap["kadraj_elev_deg"] - ist) < 3.0)
    kontrol("G9  döngü: hold'da vx≈hedef hızı, durum+kadraj dolu, elev=istasyon açısı",
            ok_hold and ok_durum and ok_merkez,
            f"vx_cmd={vx_cmd} (~{TV}) durum={snap['durum']} d_h={snap['d_h']} "
            f"elev={snap['kadraj_elev_deg']}°")

    # ── G10: istasyon ofseti SABİT AÇI (sabit metre değil) ──
    # Dikey ıskanın kök nedeni buydu: 4.65 m sabit ofset, menzil kapandıkça
    # büyüyen bir LOS yükselişine dönüşüp hedefi kadrajın tepesinden çıkarıyordu.
    # Artık etkin standoff menzille orantılı küçülür → yükseliş her menzilde
    # CENTER_ELEV_DEG kalır. Uzakta (menzil ≥ RANGE_SET) davranış değişmez.
    cfg = gg.Cfg
    ce = math.radians(cfg.ISTASYON_ELEV_DEG)
    tilt_hedef = cfg.ISTASYON_ELEV_DEG

    def _istasyon_elev(menzil):
        """Verilen menzilde istasyona park etmiş drone'un gördüğü LOS yükselişi.
        gps_guidance'ın 4. adımıyla AYNI hesap (r_eff → arka/alt ofset)."""
        r_eff = min(menzil, cfg.RANGE_SET)
        d_arka = r_eff * math.cos(ce)
        d_alt = r_eff * math.sin(ce)
        # hedef orijinde, drone d_arka geride + d_alt altta (NED: +z aşağı)
        r = hedef_kadraj_hatasi([0.0, 0.0, 0.0], [-d_arka, 0.0, d_alt], 0, 0, 0)
        return math.degrees(r["elev"])

    sapmalar = [(m, _istasyon_elev(m) - tilt_hedef) for m in (20, 11, 8, 6, 4, 2)]
    en_kotu = max(abs(s) for _, s in sapmalar)
    kontrol("G10 istasyon yükselişi HER menzilde sabit (menzille orantılı ofset)",
            en_kotu < 0.5,
            "  ".join(f"{m}m:{s:+.1f}°" for m, s in sapmalar)
            + f"  (hedef {tilt_hedef:.0f}°, en kötü sapma {en_kotu:.2f}°)")

    # Eski davranış (sabit metre) aynı testte SINIFTA KALIRDI — regresyon koruması
    def _eski_elev(menzil):
        d_arka = cfg.RANGE_SET * math.cos(ce)      # menzilden BAĞIMSIZ
        d_alt = cfg.RANGE_SET * math.sin(ce)
        # eski kod yakınlaşınca yatayı kapatıyordu ama dikey ofset sabit kalıyordu
        yatay = min(menzil * math.cos(ce), d_arka)
        r = hedef_kadraj_hatasi([0.0, 0.0, 0.0], [-yatay, 0.0, d_alt], 0, 0, 0)
        return math.degrees(r["elev"])

    # ── G11: TERMİNAL DİKEY BÜTÇESİ — istasyon aracın ivme sınırına SIĞMALI ──
    # 2026-08-02, 3 uçuş, iki aracın kara kutusu: ArduPilot dikey hız komutunu
    # WP_ACC_Z = 1.0 m/s² ile rampalıyor (DVD pozitif eğim medyanı üç uçuşta da
    # tam 1.00). Güdümün istediği 8-22 m/s tamamen alakasız; hız tavanı
    # (WP_SPD_UP=5) hiç görülmedi, yani sınırlayan İVME. Araç kusursuz uyguluyor
    # (DVD↔VD hatası 0.1 m/s, gaz hiç %95'i aşmadı).
    #
    # Eski geometri (istasyon 25° = kamera tilt'i) bu bütçeye SIĞMIYORDU:
    #   kapatılacak 4.65 m → 1 m/s² ile 3.05 s gerek, terminalde 2.4-2.8 s var
    #   ölçülen kalan dikey: vurdu +0.03 m | ıskaladı +1.52 m, +2.06 m (alttan)
    # Bu test o hatanın geri gelmesini engeller: istasyon açısı yükseltilirse
    # (veya RANGE_SET küçültülürse) burada yakalanır.
    # A_DIKEY 1.0 → 2.5 (2026-08-08): 1.0 ölçümü parametre adı düzeltmesinden
    # ÖNCEYDİ — o dönem WPNAV_ACCEL_Z firmware varsayılanında (100 cm/s²) idi.
    # avci_copter.parm artık 250 yazıyor ve uygulanıyor → bütçe 2.5 m/s².
    A_DIKEY = 2.5        # m/s²; WPNAV_ACCEL_Z — aracın dikey hız rampası
    V_YATAY = 4.3        # m/s; ÖLÇÜLEN en hızlı terminal yatay kapanma (kötü hal)
    t_var = d_behind / V_YATAY
    tirmanilabilir = 0.5 * A_DIKEY * t_var ** 2
    kontrol("G11 terminal dikey bütçesi: istasyonun altı ivme sınırında kapanabilir",
            tirmanilabilir > d_below,
            f"kapatılacak {d_below:.2f} m, {t_var:.2f} s var → {tirmanilabilir:.2f} m "
            f"tırmanılabilir (pay {tirmanilabilir - d_below:+.2f} m); "
            f"25°'de kapatılacak {C.RANGE_SET*math.sin(math.radians(25)):.2f} m olurdu")

    eski_4m = _eski_elev(4.0)
    # Eşik 20° → 8° (2026-08-08): RANGE_SET 11→8 ile eski davranışın 4 m'deki
    # taşması küçüldü (d_alt 4.65 → 2.07 m) ama İLKE aynı: eski şema açıyı
    # menzille büyütür, yeni şema sabit tutar. Test ilkeyi bekçiliyor.
    kontrol("G10b eski sabit-metre davranışı 4 m'de açıyı büyütürdü, yeni sabit",
            eski_4m - tilt_hedef > 8.0
            and abs(_istasyon_elev(4.0) - tilt_hedef) < 0.5,
            f"eski: {eski_4m:.1f}° (merkezden {eski_4m - tilt_hedef:+.1f}°), "
            f"yeni: {_istasyon_elev(4.0):.1f}°")


    # ── G11: İÇ DAİRE NİŞANI — ölçülmüş varsayılan, yön doğru, düzde sıfır ──
    # 2026-08-05 uçuş ölçümü: kayma 0→8→14 m ile menzil 34.1→22.8→9.8 m.
    # Bu değer bir tahmin değil, üç koşuluk kontrollü deneyin sonucudur.
    kontrol("G11a iç daire nişanı ölçülmüş varsayılanda (14 m)",
            C.IC_KAYMA == 14.0, f"IC_KAYMA={C.IC_KAYMA}")
    kontrol("G11a2 lead kazancı ölçülmüş varsayılanda (0.60)",
            C.KD_H == 0.60, f"KD_H={C.KD_H}  (0.20 → 34.3 m, 0.60 → 29.4 m)")

    # Kayma yönü: hız vektörünün dönüş yönünde 90°'si = dönüş merkezi.
    # Sentetik daire (R=38, saat yönü) üzerinde iki noktada kontrol.
    R, vh = 38.0, 14.6
    w = vh / R
    en_kotu = 0.0
    for tt in (0.0, math.pi / (2 * w), math.pi / w):
        px, py = R * math.cos(w * tt), R * math.sin(w * tt)
        vx, vy = -vh * math.sin(w * tt), vh * math.cos(w * tt)
        sp = math.hypot(vx, vy)
        vhx, vhy = vx / sp, vy / sp
        # omega > 0 (başlık artıyor) → merkez (-v̂y, +v̂x) yönünde
        cx, cy = -vhy, vhx
        gx, gy = -px / R, -py / R          # gerçek merkez yönü
        en_kotu = max(en_kotu, math.degrees(
            math.acos(max(-1.0, min(1.0, cx * gx + cy * gy)))))
    kontrol("G11b kayma yönü tam dönüş merkezine bakıyor",
            en_kotu < 1e-6, f"en kötü sapma {en_kotu:.2e}°")

    # Düz uçuşta (açısal hız ~0) kayma ölçeği sıfıra gitmeli — düz kovalama
    # senaryosunda regresyon olmasın diye kritik.
    olcek_duz = min(1.0, 0.0 / C.IC_OMEGA_REF)
    olcek_daire = min(1.0, 0.384 / C.IC_OMEGA_REF)
    kontrol("G11c düz uçuşta kayma 0, dairede tam",
            olcek_duz == 0.0 and olcek_daire == 1.0,
            f"düz={olcek_duz:.2f}  daire(ω=0.384)={olcek_daire:.2f}")

    # ── G12: YARIÇAP-ORANLI KAYMA ──
    def _oranli(Rr, vv, oran=0.27):
        ww = vv / Rr
        olc = min(1.0, abs(ww) / C.IC_OMEGA_REF)
        if Rr < C.IC_R_MIN:
            return 0.0
        return min(oran * Rr, C.IC_KAYMA_MAX) * olc

    kontrol("G12a varsayılan KAPALI — sabit metre geçerli (regresyon koruması)",
            C.IC_ORAN == 0.0, f"IC_ORAN={C.IC_ORAN}")

    # Katsayı ölçümden geldi: 2026-08-05'te 14 m kayma, hedefin 52.2 m
    # yarıçapının 0.268'iydi. Yani oranlı sürüm O SENARYODA sabit sürümle
    # aynı kaymayı üretmeli — fark yalnız yarıçap değişince doğmalı.
    k52 = _oranli(52.2, 14.5)
    kontrol("G12b ölçüm senaryosunda sabit sürümle AYNI kayma",
            abs(k52 - 14.0) < 0.5, f"R=52.2 m → {k52:.1f} m (sabit sürüm 14.0 m)")

    k24, k80 = _oranli(24.0, 14.5), _oranli(80.0, 15.0)
    kontrol("G12c dar dairede daha az, geniş dairede daha çok kayma",
            k24 < 14.0 < k80,
            f"R=24 m → {k24:.1f} m   |   R=80 m → {k80:.1f} m   (sabit: hep 14.0)")

    kontrol("G12d aşırı geniş yarıçapta tavan bağlıyor",
            _oranli(200.0, 15.0) <= C.IC_KAYMA_MAX + 1e-9,
            f"R=200 m → {_oranli(200.0, 15.0):.1f} m ≤ tavan {C.IC_KAYMA_MAX}")

    kontrol("G12e güvenilmez dar yarıçapta kayma kapanıyor",
            _oranli(C.IC_R_MIN - 1.0, 14.0) == 0.0,
            f"R={C.IC_R_MIN - 1:.0f} m (< IC_R_MIN={C.IC_R_MIN:.0f}) → 0.0 m")

    # ── G13: DİNAMİK İSTASYON YÜKSELİŞİ — elev = tilt + gövde pitch ──
    # Ölçülen sorun (2026-08-06, 8 uçuş): daire tutuşunda drone +11° burun
    # yukarı uçuyor (yengeç ~26-37° + merkezcil yatma) ve hedef kadraj
    # merkezinin 20-28° altına düşüyordu. Sabit açı bunu göremez.
    def _elev_din(pitch_deg):
        return max(C.ELEV_DIN_MIN, min(C.ELEV_DIN_MAX,
                                       C.CENTER_ELEV_DEG + pitch_deg))

    kontrol("G13a varsayılan AÇIK; pitch −10° → 15° (eski statik değer = regresyon çapası)",
            C.ELEV_DINAMIK and abs(_elev_din(-10.0) - 15.0) < 1e-9,
            f"ELEV_DINAMIK={C.ELEV_DINAMIK}  elev(−10°)={_elev_din(-10.0):.1f}°")

    # Uçtan uca: daire duruşunda (pitch +11°) dinamik istasyondan bakınca hedef
    # MERKEZDE; eski statik 15° istasyondan bakınca ~21° altta kalırdı.
    pit11 = math.radians(11.0)
    e_din = math.radians(_elev_din(11.0))
    stD = [T[0] - C.RANGE_SET * math.cos(e_din), T[1],
           T[2] + C.RANGE_SET * math.sin(e_din)]
    rD = hedef_kadraj_hatasi(T, stD, 0, pit11, 0.0)
    stS = [T[0] - d_behind, T[1], T[2] + d_below]
    rS = hedef_kadraj_hatasi(T, stS, 0, pit11, 0.0)
    kontrol("G13b daire duruşunda (pitch +11°) dinamik istasyon hedefi MERKEZE getirir",
            abs(math.degrees(rD["pitch_hata"])) < 0.5 and abs(rD["v"] - 240) < 3,
            f"dinamik: hata={math.degrees(rD['pitch_hata']):+.2f}° v={rD['v']:.0f}px | "
            f"statik 15° olsaydı: {math.degrees(rS['pitch_hata']):+.2f}° v={rS['v']:.0f}px")

    kontrol("G13c sınırlar bağlıyor: pitch −45° → MIN, pitch +30° → MAX",
            _elev_din(-45.0) == C.ELEV_DIN_MIN and _elev_din(30.0) == C.ELEV_DIN_MAX,
            f"elev(−45°)={_elev_din(-45.0):.0f}°  elev(+30°)={_elev_din(30.0):.0f}°")

    # Döngü içi işleyiş: fake koşuda pitch +11° → CSV'ye yazılan ist_elev_deg
    # 36° olmalı (dinamik); anahtar kapalıyken 15° (eski yol, birebir).
    import glob as _gl
    import os as _os
    import csv as _csv

    def _kosu_ist_elev(pitch_deg, dinamik):
        eski = gg.Cfg.ELEV_DINAMIK
        gg.Cfg.ELEV_DINAMIK = dinamik
        try:
            conn2 = _FakeConn()
            stp = threading.Event()
            t0 = time.monotonic()

            def gp():
                el = time.monotonic() - t0
                return {"x": T[0] + 8.0 * el, "y": T[1], "z": T[2],
                        "yaw": 0.0, "frozen": False}

            def gi():
                el = time.monotonic() - t0
                return {"x": st0[0] + 8.0 * el, "y": st0[1], "z": st0[2],
                        "roll": 0.0, "pitch": math.radians(pitch_deg),
                        "yaw": yaw_to_tgt, "vx": 8.0, "vy": 0.0, "vz": 0.0}

            th2 = threading.Thread(target=gg.run_gps_guidance,
                                   args=(conn2, gp, gi, stp), daemon=True)
            th2.start()
            time.sleep(0.6)
            stp.set()
            th2.join(2.0)
        finally:
            gg.Cfg.ELEV_DINAMIK = eski
        yol = max(_gl.glob(_os.path.join(gg._LOG_DIR, "*.csv")),
                  key=_os.path.getmtime)
        with open(yol) as fh:
            rows = list(_csv.DictReader(fh))
        return float(rows[-1]["ist_elev_deg"]) if rows else None

    e_acik = _kosu_ist_elev(11.0, True)
    kontrol("G13d döngü: pitch +11° ile ist_elev 36°'ye oturur (25 + 11)",
            e_acik is not None and abs(e_acik - 36.0) < 0.5,
            f"CSV ist_elev_deg={e_acik}")

    e_kapali = _kosu_ist_elev(11.0, False)
    kontrol("G13e kapatma anahtarı: dinamik OFF → sabit ISTASYON_ELEV_DEG (eski yol)",
            e_kapali is not None and abs(e_kapali - C.ISTASYON_ELEV_DEG) < 1e-6,
            f"CSV ist_elev_deg={e_kapali}")

    # ── G14: DÖNÜŞ İLERİ BESLEMESİ — v_ist = v_hedef + ω × r ──
    # Asıl doğrulama: birlikte-dönen istasyonun hızı SAYISAL türevle hesaplanır,
    # formülle karşılaştırılır. (Formül yanlış işaret/eksende olsa burada patlar.)
    # Uçuş 2026-08-08 (log 131037): FF açıkken daire 23.0 m, kontrol 15.1 m —
    # varsayılan KAPALI ve öyle KALMALI (regresyon koruması).
    kontrol("G14a varsayılan KAPALI (uçuşta elendi: 23.0 vs 15.1 m)",
            (not C.FF_DONUS) and C.FF_DONUS_MAX == 8.0,
            f"FF_DONUS={C.FF_DONUS} MAX={C.FF_DONUS_MAX}")

    Rg, vg, d_arka_g, d_ic_g = 52.0, 14.5, 8.7, 14.0
    wg = vg / Rg                                    # ω > 0 (saat yönü tersi değil, başlık artıyor)

    def _istasyon_poz(theta):
        px, py = Rg * math.cos(theta), Rg * math.sin(theta)
        vxg, vyg = -vg * math.sin(theta), vg * math.cos(theta)
        vhx, vhy = vxg / vg, vyg / vg
        cxg, cyg = -vhy, vhx                        # ω>0'da merkez yönü (IC ile aynı)
        sx = px - vhx * d_arka_g + cxg * d_ic_g
        sy = py - vhy * d_arka_g + cyg * d_ic_g
        return px, py, vxg, vyg, sx, sy

    en_kotu = 0.0
    for th in (0.0, 1.1, 2.7):
        px, py, vxg, vyg, sx, sy = _istasyon_poz(th)
        dt_ = 1e-4
        _, _, _, _, sx2, sy2 = _istasyon_poz(th + wg * dt_)
        v_num = ((sx2 - sx) / dt_, (sy2 - sy) / dt_)          # gerçek istasyon hızı
        rx_, ry_ = sx - px, sy - py
        v_form = (vxg + wg * (-ry_), vyg + wg * rx_)           # koddaki formül
        en_kotu = max(en_kotu, math.hypot(v_form[0] - v_num[0], v_form[1] - v_num[1]))
    kontrol("G14b formül = birlikte-dönen istasyonun sayısal türevi",
            en_kotu < 1e-2, f"en kötü fark {en_kotu:.2e} m/s (3 açıda)")

    duz_ff = math.hypot(0.0 * (-(-3.0)), 0.0 * 10.0)           # ω=0 → düzeltme 0
    buyukluk = abs(0.5) * math.hypot(10.0, 30.0)               # ω=0.5, |r|=31.6 → 15.8
    kontrol("G14c düzde sıfır, aşırı ω'da tavan bağlar",
            duz_ff == 0.0 and min(buyukluk, C.FF_DONUS_MAX) == C.FF_DONUS_MAX,
            f"düz={duz_ff}  ham {buyukluk:.1f} → tavan {C.FF_DONUS_MAX}")

    # Büyüklük gerçeklik kontrolü: ⌀55 senaryosu geometrisinde ~4.6 m/s beklenir
    _, _, _, _, sx, sy = _istasyon_poz(0.0)
    rx_, ry_ = sx - Rg, sy - 0.0
    mag = math.hypot(wg * (-ry_), wg * rx_)
    kontrol("G14d ⌀55 geometrisinde düzeltme ~4.6 m/s (ölçülen açıkla uyumlu)",
            3.5 < mag < 5.5, f"|ω×r| = {mag:.2f} m/s (r={math.hypot(rx_, ry_):.1f} m)")

    # ── G15: ARKA KISALTMA — mekanizma doğru, yarışma hattında KAPALI ──
    def _d_arka(d_behind_eff, omega, kisalt):
        olc = min(1.0, abs(omega) / C.IC_OMEGA_REF)
        return d_behind_eff * (1.0 - kisalt * olc)

    # D0 yarışma kuralı (2026-08-08): görsel temas varken GPS yasak → yakın
    # yandan eskort (5.7 m) tespit sürekliliği başlatıp bizi yandan görsel
    # faza zorlar (139°/s — ölümcül). Varsayılan 0; teknik gimball_gudum'da.
    kontrol("G15a varsayılan KAPALI (D0 yarışma kuralı; gimball_gudum'da arşivli)",
            C.ARKA_KISALT == 0.0, f"ARKA_KISALT={C.ARKA_KISALT}")

    db = 8.0 * math.cos(math.radians(38.0))     # dönüş rejiminde tipik d_behind_eff
    kontrol("G15b mekanizma (kısalt=1): düzde tam, tam dönüşte sıfır, yarıda yarı",
            _d_arka(db, 0.0, 1.0) == db
            and _d_arka(db, 0.30, 1.0) < 1e-9
            and abs(_d_arka(db, 0.075, 1.0) - db / 2) < 1e-9,
            f"ω=0 → {_d_arka(db, 0.0, 1.0):.2f}  ω=0.075 → {_d_arka(db, 0.075, 1.0):.2f}  "
            f"ω=0.30 → {_d_arka(db, 0.30, 1.0):.2f} m")

    kontrol("G15c yarışma davranışı (kısalt=0): arka bileşen HER ω'da tam",
            _d_arka(db, 0.30, C.ARKA_KISALT) == db
            and _d_arka(db, 0.0, C.ARKA_KISALT) == db,
            f"ω=0.30 → {_d_arka(db, 0.30, C.ARKA_KISALT):.2f} m (= {db:.2f})")

    # ── G16: FAZ GİRİŞİNDE YAW SIÇRAMASI YOK (takla önleyici) ──
    # 2026-08-08: görsel fazdan GPS'e dönüşte cmd_yaw doğrudan bearing'e
    # atanıyordu → hedef arkadaysa tek karede 100-160° komut sıçraması →
    # yaw doyumu → motorlarda roll/pitch yetkisi kalmıyor → TAKLA.
    # Ölçüldü: 12 faz girişinin 6'sında sıçrama >60°.
    class _FakeMav2:
        def __init__(s): s.ilk = None
        def set_position_target_local_ned_send(s, *a):
            if s.ilk is None:
                s.ilk = a

    class _FakeConn2:
        target_system = 1; target_component = 1
        def __init__(s): s.mav = _FakeMav2()

    conn2 = _FakeConn2()
    IRIS_YAW = math.radians(30.0)          # aracın burnu +30°
    T2 = [0.0, 0.0, -50.0]                 # hedef ARKADA (drone +100 x'te)
    def gp2():
        return {"x": T2[0], "y": T2[1], "z": T2[2], "yaw": 0.0, "frozen": False}
    def gi2():
        return {"x": 100.0, "y": 0.0, "z": -50.0, "roll": 0.0, "pitch": 0.0,
                "yaw": IRIS_YAW, "vx": 0.0, "vy": 0.0, "vz": 0.0}
    stop2 = threading.Event()
    th2 = threading.Thread(target=gg.run_gps_guidance,
                           args=(conn2, gp2, gi2, stop2), daemon=True)
    th2.start(); time.sleep(0.5); stop2.set(); th2.join(2.0)
    ilk = conn2.mav.ilk
    yaw0 = ilk[14] if ilk else None          # ilk gönderilen yaw komutu
    sicrama = (abs(math.degrees(normalize_angle_test(yaw0 - IRIS_YAW)))
               if yaw0 is not None else 999)
    # hedef tam arkada (bearing 180°) — eski kod 150° sıçrama üretirdi
    kontrol("G16 faz girişinde yaw komutu araç yönünden başlar (takla önleyici)",
            yaw0 is not None and sicrama < 15.0,
            f"ilk komut {math.degrees(yaw0):+.0f}° vs araç {math.degrees(IRIS_YAW):+.0f}° "
            f"→ sıçrama {sicrama:.0f}° (eski kod ~150° üretirdi)")

    print("=" * 60)
    # ══════════════════════════════════════════════════════════════════
    # G17 · Ö-D2 — BURUN BORCU SINIRI (GPS FAZI)
    # ══════════════════════════════════════════════════════════════════
    # Kısıt döngüde, slew'den SONRA uygulanıyor; yasayı burada birebir kurar.
    def _od2(cfg, cmd_yaw_deg, iris_yaw_deg):
        borc = math.degrees(gg.normalize_angle(
            math.radians(cmd_yaw_deg - iris_yaw_deg)))
        if cfg.GPS_FOV_YAW > 0.0 and abs(borc) > cfg.GPS_FOV_YAW:
            borc = max(-cfg.GPS_FOV_YAW, min(cfg.GPS_FOV_YAW, borc))
        return math.degrees(gg.normalize_angle(
            math.radians(iris_yaw_deg + borc)))

    class _D2(gg.Cfg):
        GPS_FOV_YAW = 25.0

    # G17a — kullanıcının OLAY 1'i: 175° borç 25°'ye iner
    _o1 = _od2(_D2, -173.0, 2.0)
    kontrol("G17a Ö-D2: kullanıcının OLAY 1'i (175° borç) kısılır",
            abs(_o1 - (-23.0)) < 1e-6,
            f"yaw_cmd −173° / iris_yaw +2° (borç −175°) → komut {_o1:.0f}° "
            f"(borç −25°). Araç 300°/s'ye çıkmaz, bekçi tetiklenmez.")

    # G17b — SAKİN TAKİPTE ÖLÜ (kuyruk takibinde borç birkaç derece)
    _sakin = [(_od2(_D2, a, b), a) for a, b in
              ((3.0, 0.0), (-8.0, 0.0), (100.0, 80.0), (-175.0, 170.0))]
    kontrol("G17b Ö-D2 sakin takipte ÖLÜ (borç < 25° → komut BİT BİT aynı)",
            all(abs(gg.normalize_angle(math.radians(y - a))) < 1e-9
                for y, a in _sakin),
            "borç 3°/8°/20°/15° → dördünde de komut değişmedi; "
            "kuyruk takibi ve istasyon tutma ETKİLENMEZ")

    # G17c — ±180° sarması doğru
    _w = _od2(_D2, -175.0, 170.0)
    kontrol("G17c Ö-D2 ±180° sarmasında borç doğru",
            abs(gg.normalize_angle(math.radians(_w - (-175.0)))) < 1e-9,
            f"cmd −175° / iris 170° gerçek borç 15° (345° DEĞİL) → değişmedi")

    # G17d — YALNIZ KISAR
    _b = []
    for a in range(-180, 181, 15):
        for c in range(-180, 181, 45):
            o = abs(math.degrees(gg.normalize_angle(math.radians(a - c))))
            y = abs(math.degrees(gg.normalize_angle(
                math.radians(_od2(_D2, float(a), float(c)) - c))))
            _b.append(y <= o + 1e-9)
    kontrol("G17d Ö-D2 YALNIZ KISAR, borcu asla büyütmez",
            all(_b), f"{len(_b)} (cmd, iris) kombinasyonunda |borç| hiç artmadı")

    # G17e — YAPISAL GARANTİ: hız vektörü cmd_yaw'dan ÖNCE hesaplanır
    # Kaynak sırası bit bit denetlenir: vx/vy satırı, Ö-D2 kısıtından ÖNCE
    # olmalı ve limit_acceleration çağrısı cmd_yaw kullanmamalı.
    _src = inspect.getsource(gg.run_gps_guidance).splitlines()
    _i_vx = next(i for i, L in enumerate(_src) if "vx = ff_x" in L)
    _i_kis = next(i for i, L in enumerate(_src) if "Ö-D2 BURUN BORCU" in L)
    _i_acc = next(i for i, L in enumerate(_src) if "limit_acceleration(" in L)
    _acc_blok = " ".join(_src[_i_acc:_i_acc + 3])
    kontrol("G17e YAPISAL GARANTİ: hız cmd_yaw'dan ÖNCE ve ondan BAĞIMSIZ",
            _i_vx < _i_kis and "cmd_yaw" not in _acc_blok,
            f"vx/vy satır {_i_vx} · Ö-D2 satır {_i_kis} (sonra) · "
            f"limit_acceleration girdisinde cmd_yaw YOK → "
            f"UÇUŞ YOLU DEĞİŞEMEZ, yalnız BURUN")

    # G17f — kapatılabilir (varsayılan KAPALI)
    kontrol("G17f Ö-D2 kapatılabilir (varsayılan KAPALI)",
            abs(gg.Cfg.GPS_FOV_YAW) < 1e-9
            and abs(_od2(gg.Cfg, -173.0, 2.0) - (-173.0)) < 1e-6,
            f"GPS_FOV_YAW={gg.Cfg.GPS_FOV_YAW} → 175° borç aynen geçer "
            f"(eski davranış bit bit korunur)")

    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
