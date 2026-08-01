"""
tests/test_gps_guidance.py — GPS kadraj güdümü kabul kriterleri.

Gazebo'suz, saf mantık. Kullanım: python3 -m tests.test_gps_guidance

Kapsam:
  G1-G6  hedef_kadraj_hatasi (başarı kriteri matematiği — merkez/yatay/yan/dikey/arka/menzil)
  G7     KADEME 1 tasarım tutarlılığı: geometrik kadraj noktasında drone → hedef MERKEZDE
  G8     kadraj noktası hedefin hız yönünün gerisinde + altında
  G9     döngü duman testi (fake conn): komut üretir, hold'da ≈ hedef hızı, durum dolu
"""

import math
import os
import tempfile
import threading
import time

# CSV'yi geçici dizine yönlendir — testler logs/ altına sahte uçuş dosyası
# bırakmasın. _LOG_DIR import anında okunur, bu satır importlardan ÖNCE olmalı.
os.environ.setdefault("AVCI_LEAD_LOG_DIR", tempfile.mkdtemp(prefix="avci_test_gps_"))

from control.guidance.guidance_core import hedef_kadraj_hatasi, govde_to_dunya
from control.guidance import gps_guidance as gg

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def main():
    print("GPS kadraj güdümü kabul kriterleri")
    print("=" * 60)
    C = gg.Cfg
    tilt = C.CENTER_ELEV_DEG
    d_behind = C.RANGE_SET * math.cos(math.radians(tilt))
    d_below = C.RANGE_SET * math.sin(math.radians(tilt))

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
    kontrol("G7  geometrik istasyonda hedef merkezde (yaw≈0, elev≈25°, menzil≈RANGE_SET)",
            abs(math.degrees(r7["yaw_hata"])) < 0.5
            and abs(math.degrees(r7["elev"]) - tilt) < 0.5
            and abs(r7["menzil"] - C.RANGE_SET) < 0.2,
            f"yaw={math.degrees(r7['yaw_hata']):.2f}° elev={math.degrees(r7['elev']):.2f}° "
            f"menzil={r7['menzil']:.2f}")

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
                 and abs(snap["kadraj_elev_deg"] - tilt) < 3.0)
    kontrol("G9  döngü: hold'da vx≈hedef hızı, durum+kadraj dolu ve merkezde",
            ok_hold and ok_durum and ok_merkez,
            f"vx_cmd={vx_cmd} (~{TV}) durum={snap['durum']} d_h={snap['d_h']} "
            f"elev={snap['kadraj_elev_deg']}°")

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
