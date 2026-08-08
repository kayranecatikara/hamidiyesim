"""
tests/test_bbox_ibvs.py — SAF bbox IBVS görsel güdüm kabul kriterleri.

Gazebo'suz, saf mantık. Kullanım: python3 -m tests.test_bbox_ibvs

Kapsam:
  B1-B4  komut yasası: merkez, sağ/sol yaw, yakın/uzak ileri, alt/üst dikey
  B5     GPS BAĞIMSIZLIĞI: yasa yalnız (cx,cy,w,h,yaw) — hedef GPS'i girmiyor
  B6-B7  kutu geçerliliği: düşük conf / küçük kutu elenir
  B8     döngü duman testi (fake conn): kutu akışında komut üretir
  B9     kayıp: kayip_kare_esik ardışık kutusuz → 'kayip'
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
    vx, vy, vz, yaw, t = ib.komut(CX, C.CY_NISAN, 40, 40, 0.0, C)
    kontrol("B1  nişan noktasında: yaw≈0, vz≈0",
            abs(math.degrees(yaw)) < 0.5 and abs(vz) < 0.05,
            f"yaw={math.degrees(yaw):.2f}° vz={vz:.3f}")

    # ── B2: hedef SAĞDA (cx>CX) → yaw komutu POZİTİF (sağa dön) ──
    vx, vy, vz, yaw_sag, t = ib.komut(CX + 100, C.CY_NISAN, 40, 40, 0.0, C)
    _, _, _, yaw_sol, _ = ib.komut(CX - 100, C.CY_NISAN, 40, 40, 0.0, C)
    kontrol("B2  hedef sağda → yaw>0, solda → yaw<0",
            yaw_sag > 0.05 and yaw_sol < -0.05,
            f"sağ yaw={math.degrees(yaw_sag):+.1f}° sol yaw={math.degrees(yaw_sol):+.1f}°")

    # ── B3: İLERİ — küçük kutu (uzak) hızlı, büyük kutu (yakın) yavaş/geri ──
    _, _, _, _, t_uzak = ib.komut(CX, C.CY_NISAN, 20, 20, 0.0, C)
    _, _, _, _, t_yakin = ib.komut(CX, C.CY_NISAN, 200, 200, 0.0, C)
    kontrol("B3  uzak kutu ileri hızlı, yakın kutu geri/yavaş",
            t_uzak["v_fwd"] > 5.0 and t_yakin["v_fwd"] < t_uzak["v_fwd"],
            f"uzak(20px) v_fwd={t_uzak['v_fwd']:.1f}  yakın(200px) v_fwd={t_yakin['v_fwd']:.1f}")

    # ── B4: DİKEY — hedef kadrajda AŞAĞIDA (cy>nişan) → ALÇAL (vz>0, NED down+) ──
    _, _, vz_asa, _, _ = ib.komut(CX, C.CY_NISAN + 120, 40, 40, 0.0, C)
    _, _, vz_yuk, _, _ = ib.komut(CX, C.CY_NISAN - 120, 40, 40, 0.0, C)
    kontrol("B4  hedef altta → vz>0 (alçal), üstte → vz<0 (tırman)",
            vz_asa > 0.1 and vz_yuk < -0.1,
            f"altta vz={vz_asa:+.2f}  üstte vz={vz_yuk:+.2f}")

    # ── B5: GPS BAĞIMSIZLIĞI — yasa imzası yalnız piksel+yaw; GPS argümanı YOK ──
    # komut(cx, cy, w, h, iris_yaw, cfg) — hedef konumu/hızı/menzili parametre
    # DEĞİL. İki farklı "dünya"da aynı kutu aynı komutu üretir (yasa görüntüye
    # bakar, sahneye değil).
    import inspect
    parametreler = list(inspect.signature(ib.komut).parameters)
    yasak = [p for p in parametreler if any(
        k in p.lower() for k in ("gps", "ned", "menzil", "hedef_", "plane", "tgt"))]
    r1 = ib.komut(CX + 50, CY, 60, 40, 1.2, C)
    r2 = ib.komut(CX + 50, CY, 60, 40, 1.2, C)
    kontrol("B5  yasa GPS'siz: imzada hedef konum/menzil yok, deterministik",
            not yasak and r1[:4] == r2[:4],
            f"parametreler={parametreler}")

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

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
