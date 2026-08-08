"""
tests/test_bbox_ibvs.py — SAF bbox IBVS görsel güdüm kabul kriterleri.

Gazebo'suz, saf mantık. Kullanım: python3 -m tests.test_bbox_ibvs

Kapsam:
  B1-B4  komut yasası: merkez, sağ/sol yaw, yakın/uzak kapanma, alt/üst dikey
  B5     ⚠ D0 KURAL UYUMU (yapısal): görsel döngünün CANLI GPS'e erişimi YOK —
         taşıyıcı sayı üçlüsü olarak geçilir, callback değil
  B6-B7  kutu geçerliliği: düşük conf / küçük kutu elenir
  B8     döngü duman testi (fake conn): kutu akışında komut üretir
  B9     kayıp: kayip_kare_esik ardışık kutusuz → 'kayip'
  B10    DONDURULMUŞ TAŞIYICI: hedefin seyir hızını üstlenir (uçuş dersi:
         taşıyıcısız 8 m/s üretiyordu, hedef 15 → kutu kaybı)
  B11    toplam hız tavanı bağlar
  B12    kutu yokken taşıyıcı SÜRER (kısa boşluk kalıcı kayba dönmesin)
"""

import math
import threading
import time

from vision import geometry as geo
from control.guidance import bbox_ibvs as ib

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def main():
    print("SAF bbox IBVS kabul kriterleri")
    print("=" * 60)
    C = ib.Cfg
    CX, CY, FX, FY = geo.CX, geo.CY, geo.FX, geo.FY

    # ── B1: MERKEZDE nişan — yaw ≈ mevcut, dikey ≈ 0 ──
    # cx=CX (yatay merkez), cy=CY_NISAN (dikey nişan) → sapma yok.
    vx, vy, vz, yaw, t = ib.komut(CX, C.CY_NISAN, 40, 40, 0.0, cfg=C)
    kontrol("B1  nişan noktasında: yaw≈0, vz≈0",
            abs(math.degrees(yaw)) < 0.5 and abs(vz) < 0.05,
            f"yaw={math.degrees(yaw):.2f}° vz={vz:.3f}")

    # ── B2: hedef SAĞDA (cx>CX) → yaw komutu POZİTİF (sağa dön) ──
    vx, vy, vz, yaw_sag, t = ib.komut(CX + 100, C.CY_NISAN, 40, 40, 0.0, cfg=C)
    _, _, _, yaw_sol, _ = ib.komut(CX - 100, C.CY_NISAN, 40, 40, 0.0, cfg=C)
    kontrol("B2  hedef sağda → yaw>0, solda → yaw<0",
            yaw_sag > 0.05 and yaw_sol < -0.05,
            f"sağ yaw={math.degrees(yaw_sag):+.1f}° sol yaw={math.degrees(yaw_sol):+.1f}°")

    # ── B3: KAPANMA — küçük kutu (uzak) tam kapanma, büyük kutu (yakın) geri ──
    _, _, _, _, t_uzak = ib.komut(CX, C.CY_NISAN, 5, 5, 0.0, cfg=C)
    _, _, _, _, t_yakin = ib.komut(CX, C.CY_NISAN, 60, 60, 0.0, cfg=C)
    _, _, _, _, t_denge = ib.komut(CX, C.CY_NISAN, C.BOYUT_REF, C.BOYUT_REF,
                                   0.0, cfg=C)
    kontrol("B3  uzak kutu tam kapanma, REF'te 0, yakın kutu geri",
            t_uzak["v_kapanma"] > 4.0 and abs(t_denge["v_kapanma"]) < 1e-6
            and t_yakin["v_kapanma"] < 0,
            f"5px→{t_uzak['v_kapanma']:+.1f}  REF({C.BOYUT_REF:.0f}px)→"
            f"{t_denge['v_kapanma']:+.1f}  60px→{t_yakin['v_kapanma']:+.1f} m/s")

    # ── B4: DİKEY — hedef kadrajda AŞAĞIDA (cy>nişan) → ALÇAL (vz>0, NED down+) ──
    _, _, vz_asa, _, _ = ib.komut(CX, C.CY_NISAN + 120, 40, 40, 0.0, cfg=C)
    _, _, vz_yuk, _, _ = ib.komut(CX, C.CY_NISAN - 120, 40, 40, 0.0, cfg=C)
    kontrol("B4  hedef altta → vz>0 (alçal), üstte → vz<0 (tırman)",
            vz_asa > 0.1 and vz_yuk < -0.1,
            f"altta vz={vz_asa:+.2f}  üstte vz={vz_yuk:+.2f}")

    # ── B5: ⚠ D0 KURAL UYUMU — YAPISAL GARANTİ ──
    # Kural: görsel temas varken CANLI GPS güdümde kullanılamaz. Bu testin
    # iddiası "kullanmıyoruz" değil, "KULLANAMAYIZ": görsel döngü hedefe dair
    # tek bilgiyi devirde bir kez, SAYI olarak alır. Callable (canlı kaynak)
    # geçilirse burada patlar.
    import inspect
    kp = list(inspect.signature(ib.komut).parameters)
    dp = inspect.signature(ib.run_bbox_ibvs).parameters
    # döngüde hedef verisi taşıyabilecek tek parametre ff_hiz; o da sayı üçlüsü
    hedef_param = [p for p in dp if any(
        k in p.lower() for k in ("plane", "truth", "menzil", "gercek", "tgt", "gps"))]
    ff_default = dp["ff_hiz"].default
    ff_sayi = (isinstance(ff_default, tuple) and len(ff_default) == 3
               and all(isinstance(v, (int, float)) for v in ff_default))
    r1 = ib.komut(CX + 50, CY, 60, 40, 1.2, (5.0, 1.0, 0.0), C)
    r2 = ib.komut(CX + 50, CY, 60, 40, 1.2, (5.0, 1.0, 0.0), C)
    kontrol("B5  D0: döngüde canlı hedef kaynağı YOK, taşıyıcı sayı üçlüsü",
            not hedef_param and ff_sayi and r1[:4] == r2[:4],
            f"run parametreleri={list(dp)}  ff varsayılan={ff_default}")

    # ── B6: düşük conf kutusu elenir ──
    dusuk = ib._kutu_gecerli({"bbox": (300, 220, 340, 260), "conf": 0.1}, C)
    yuksek = ib._kutu_gecerli({"bbox": (300, 220, 340, 260), "conf": 0.9}, C)
    kontrol("B6  conf < CONF_MIN kutusu None, üstü geçerli",
            dusuk is None and yuksek is not None,
            f"conf 0.1 → {dusuk}, conf 0.9 → geçerli")

    # ── B7: çok küçük kutu (gürültü) elenir ──
    minik = ib._kutu_gecerli({"bbox": (320, 240, 323, 243), "conf": 0.9}, C)
    kontrol("B7  boyut < BOYUT_MIN kutusu elenir",
            minik is None, f"3×3 px kutu → {minik}")

    # ── B8: DÖNGÜ DUMAN TESTİ (fake conn) — kutu akışında komut üretir ──
    class _FakeMav:
        def __init__(s): s.last = None
        def set_position_target_local_ned_send(s, *a): s.last = a

    class _FakeConn:
        target_system = 1; target_component = 1
        def __init__(s): s.mav = _FakeMav()

    conn = _FakeConn()
    st = {"seq": 0}
    def wait_pose(son_seq, timeout=0.5):
        st["seq"] += 1
        # hedef sağda + biraz altta, orta boy kutu
        return {"seq": st["seq"],
                "pose": {"bbox": (360, 250, 400, 285), "conf": 0.8},
                "stamp": None, "wall_recv": None, "lock": None}
    def get_iris():
        return {"yaw": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0}
    stop = threading.Event()
    import tempfile
    ib._LOG_DIR = tempfile.mkdtemp(prefix="avci_ibvs_test_")
    th = threading.Thread(target=ib.run_bbox_ibvs,
                          args=(conn, get_iris, wait_pose, stop), daemon=True)
    th.start()
    time.sleep(0.4)
    sent = conn.mav.last
    stop.set(); th.join(2.0)
    # set_position_target_local_ned_send: vx,vy,vz index 8,9,10; yaw index 14
    ok_komut = sent is not None
    yaw_cmd = sent[14] if sent else None
    kontrol("B8  döngü kutu akışında komut üretir, hedef sağda → yaw>0",
            ok_komut and yaw_cmd is not None and yaw_cmd > 0.02,
            f"yaw_cmd={yaw_cmd}")

    # ── B9: KAYIP — kayip_kare_esik ardışık kutusuz kare → 'kayip' ──
    conn2 = _FakeConn()
    st2 = {"seq": 0}
    def wait_pose_bos(son_seq, timeout=0.5):
        st2["seq"] += 1
        return {"seq": st2["seq"], "pose": None,
                "stamp": None, "wall_recv": None, "lock": None}
    stop2 = threading.Event()
    sonuc = {"r": None}
    def kosu():
        sonuc["r"] = ib.run_bbox_ibvs(conn2, get_iris, wait_pose_bos, stop2,
                                      kayip_kare_esik=5)
    th2 = threading.Thread(target=kosu, daemon=True)
    th2.start(); th2.join(3.0)
    stop2.set()
    kontrol("B9  N ardışık kutusuz kare → 'kayip' (GPS'e dönüş sinyali)",
            sonuc["r"] == "kayip", f"dönüş={sonuc['r']}")

    # ── B10: DONDURULMUŞ TAŞIYICI — hedefin seyrini üstlenir ──
    # 2026-08-08 uçuş dersi: taşıyıcısız yasa 12 m'de (kutu 12px) yalnız
    # ~8 m/s üretiyordu; hedef 15 m/s → drone geride kaldı, faz 3.5 s'de koptu.
    # Taşıyıcıyla toplam hız hedefin hızını AŞMALI (aksi halde asla kapanmaz).
    HEDEF_V = 15.0
    vx0, vy0, _, _, _ = ib.komut(CX, C.CY_NISAN, 12, 12, 0.0, (0.0, 0.0, 0.0), C)
    vx1, vy1, _, _, _ = ib.komut(CX, C.CY_NISAN, 12, 12, 0.0, (HEDEF_V, 0.0, 0.0), C)
    kontrol("B10 taşıyıcısız hedefin altında, taşıyıcıyla ÜSTÜNDE",
            math.hypot(vx0, vy0) < HEDEF_V and math.hypot(vx1, vy1) > HEDEF_V,
            f"taşıyıcısız {math.hypot(vx0, vy0):.1f} m/s  |  "
            f"taşıyıcılı {math.hypot(vx1, vy1):.1f} m/s  (hedef {HEDEF_V:.0f})")

    # ── B11: toplam yatay hız tavanı bağlar ──
    vx2, vy2, _, _, _ = ib.komut(CX, C.CY_NISAN, 5, 5, 0.0, (17.0, 0.0, 0.0), C)
    kontrol("B11 toplam hız V_TOPLAM_MAX ile tavanlı",
            math.hypot(vx2, vy2) <= C.V_TOPLAM_MAX + 1e-6,
            f"17+kapanma → {math.hypot(vx2, vy2):.2f} ≤ {C.V_TOPLAM_MAX}")

    # ── B12: kutu yokken TAŞIYICI SÜRER (sıfır komut kalıcı kayıp yapar) ──
    conn3 = _FakeConn()
    st3 = {"seq": 0}
    def wait_pose_bos2(son_seq, timeout=0.5):
        st3["seq"] += 1
        return {"seq": st3["seq"], "pose": None,
                "stamp": None, "wall_recv": None, "lock": None}
    stop3 = threading.Event()
    th3 = threading.Thread(
        target=ib.run_bbox_ibvs,
        args=(conn3, get_iris, wait_pose_bos2, stop3, ib.Cfg, 50, (14.0, 3.0, 0.0)),
        daemon=True)
    th3.start(); time.sleep(0.3); s3 = conn3.mav.last; stop3.set(); th3.join(2.0)
    kontrol("B12 kutu yokken taşıyıcı sürüyor (0 komut verilmiyor)",
            s3 is not None and abs(s3[8] - 14.0) < 0.01 and abs(s3[9] - 3.0) < 0.01,
            f"komut vx={s3[8] if s3 else None} vy={s3[9] if s3 else None} (taşıyıcı 14.0, 3.0)")

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
