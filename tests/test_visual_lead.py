"""
tests/test_visual_lead.py — IBVS lead pursuit kabul kriterleri (T1-T21).

Gazebo'dan ÖNCE geçmeli. Sentetik üreteç (master spec):
  a = fx * GOVDE_BOYU_M / R * sin(aspect)
  b = fx * KANAT_ACIKLIGI_M / R * cos(aspect)

Kullanım: python3 -m tests.test_visual_lead
"""

import math

import numpy as np

from control.guidance.adapter_copter import CopterAdapter
from control.guidance.adapter_fixedwing import FixedWingAdapter
from control.guidance.guidance_core import (
    GOVDE_BOYU_M, KANAT_ACIKLIGI_M, LeadPursuitCore, cfg_copy,
    govde_to_dunya, yukselti_duzeltme)
from vision import geometry as geo

FX, FY, CX, CY = geo.FX, geo.FY, geo.CX, geo.CY

# Kamerayı yatay yapan attitude (tilt 25° yukarı → pitch -25° = eps 0, merkez hedef)
ATT_KAMERA_YATAY = (0.0, math.radians(-25.0), 0.0)

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def make_pose(R, aspect_deg, cx=CX, cy=CY, d_aci_deg=0.0, conf=1.0,
              kpt_conf=1.0, swap=False, a_ovr=None, b_ovr=None):
    """Sentetik pose: bbox merkezi (cx,cy); gövde ekseni d_aci yönünde a px,
    kanat ona dik b px. a/b master spec üreteciyle (veya override) hesaplanır."""
    asp = math.radians(aspect_deg)
    a = a_ovr if a_ovr is not None else FX * GOVDE_BOYU_M / R * math.sin(asp)
    b = b_ovr if b_ovr is not None else FX * KANAT_ACIKLIGI_M / R * math.cos(asp)
    th = math.radians(d_aci_deg)
    dx, dy = math.cos(th), math.sin(th)
    burun = (cx + dx * a / 2, cy + dy * a / 2, kpt_conf)
    kuyruk = (cx - dx * a / 2, cy - dy * a / 2, kpt_conf)
    if swap:
        burun, kuyruk = kuyruk, burun
    solk = (cx - dy * b / 2, cy + dx * b / 2, kpt_conf)
    sagk = (cx + dy * b / 2, cy - dx * b / 2, kpt_conf)
    vt = (cx, cy, kpt_conf)
    boy = max(a, b, 4.0)
    return {"cx": cx, "cy": cy, "conf": conf,
            "bbox": (cx - boy / 2, cy - boy / 2, cx + boy / 2, cy + boy / 2),
            "kpts": [burun, kuyruk, solk, sagk, vt, vt]}


def tek_kare(cfg, pose, att=ATT_KAMERA_YATAY, stamp=0.0):
    return LeadPursuitCore(cfg).process(pose, stamp, att)


def main():
    print("IBVS lead pursuit kabul kriterleri (T1-T21)")
    print("=" * 60)

    # ── T1: aspect=90, R=5/8/12 → lead üçünde de AYNI (menzil bağımsızlık) ──
    # Kalite kapısı TASARIMI GEREĞİ menzile bağlı (14px≈9.6m); ölçüm zincirinin
    # menzil bağımsızlığını test etmek için kapı nötrlenir (kapının kendisi T4'te).
    def cfg_kapisiz():
        c = cfg_copy(); c.OLCEK_KAPALI_PX = 0.0; c.OLCEK_TAM_PX = 0.5
        return c
    leads = [tek_kare(cfg_kapisiz(), make_pose(R, 90))["lead_deg"] for R in (5, 8, 12)]
    kontrol("T1  menzil bağımsız lead (aspect=90)",
            max(leads) - min(leads) < 0.01, f"leads={[round(l,3) for l in leads]}")

    # ── T2: aspect=30, R=5/8/12 → aynı ──
    leads = [tek_kare(cfg_kapisiz(), make_pose(R, 30))["lead_deg"] for R in (5, 8, 12)]
    kontrol("T2  menzil bağımsız lead (aspect=30)",
            max(leads) - min(leads) < 0.01, f"leads={[round(l,3) for l in leads]}")

    # ── T3: aspect=5 → lead < 3 deg ──
    lead = tek_kare(cfg_copy(), make_pose(5, 5))["lead_deg"]
    kontrol("T3  karşıdan hedefte lead söner", lead < 3.0, f"lead={lead:.2f}°")

    # ── T4: R=30 m → kalite=0, lead=0 ──
    r = tek_kare(cfg_copy(), make_pose(30, 90))
    kontrol("T4  uzakta kalite kapısı", r["kalite"] == 0.0 and r["lead_deg"] == 0.0,
            f"olcek={r['olcek']:.1f}px kalite={r['kalite']} lead={r['lead_deg']}")

    # ── T5: K_LEAD=0 → lead=0, çıkış birebir saf takip (u_nisan == u) ──
    cfg = cfg_copy(); cfg.K_LEAD = 0.0
    r = tek_kare(cfg, make_pose(8, 90, cx=450, cy=300))
    sapma = math.degrees(math.acos(min(1.0, float(np.dot(r["u"], r["u_nisan"])))))
    kontrol("T5  K_LEAD=0 → saf takip", r["lead_deg"] == 0.0 and sapma < 1e-9,
            f"lead={r['lead_deg']} u·u_nisan sapma={sapma:.2e}°")

    # ── T6: burun/kuyruk takas → flip yakalanır, yön korunur, WARN ──
    cfg = cfg_copy()
    core = LeadPursuitCore(cfg)
    r1 = core.process(make_pose(8, 90), 0.0, ATT_KAMERA_YATAY)
    r2 = core.process(make_pose(8, 90, swap=True), 1.0 / 30.0, ATT_KAMERA_YATAY)
    yon_korundu = float(np.dot(r2["d_birim"], r1["d_birim"])) > 0.5
    kontrol("T6  flip koruması", core.flip_sayaci == 1 and yon_korundu
            and "flip" in r2["warn"],
            f"flip_sayaci={core.flip_sayaci} dot={np.dot(r2['d_birim'], r1['d_birim']):.2f}")

    # ── T7: aspect=90, K=0.5, kalite=1, eps=0 → lead = 26.57° ──
    r = tek_kare(cfg_copy(), make_pose(5, 90))
    kontrol("T7  arctan(0.5) leadi", abs(r["lead_deg"] - 26.565) < 0.01,
            f"lead={r['lead_deg']:.3f}° kalite={r['kalite']} eps={r['eps_deg']:.2f}")

    # ── T8: aynı geometri, bbox_cx=320/400/500/600 → AÇISAL lead aynı ──
    acilar = []
    for cx in (320, 400, 500, 600):
        r = tek_kare(cfg_copy(), make_pose(8, 90, cx=cx))
        acilar.append(math.degrees(math.acos(
            max(-1.0, min(1.0, float(np.dot(r["u"], r["u_nisan"])))))))
    kontrol("T8  ekran konumundan bağımsız açısal lead",
            max(acilar) - min(acilar) < 0.01,
            f"açısal kaydırmalar={[round(x,4) for x in acilar]}")

    # ── T9: menzil kestirimi, eps=0 merkez → hata < %2 ──
    hatalar = []
    for R in (5, 8, 12, 15):
        r = tek_kare(cfg_copy(), make_pose(R, 60))
        hatalar.append(abs(r["menzil_kestirim_m"] - R) / R * 100)
    kontrol("T9  menzil kestirimi <%2 (SADECE LOG)", max(hatalar) < 2.0,
            f"max hata=%{max(hatalar):.3f}")

    # ── T10: filtre oturma süresi SANİYE cinsinden 10 vs 30 Hz aynı ──
    def oturma(hz):
        cfg = cfg_copy()
        core = LeadPursuitCore(cfg)
        dt, t = 1.0 / hz, 0.0
        for _ in range(int(1.0 * hz)):          # 1 s düşük yandanlık
            core.process(make_pose(8, 5), t, ATT_KAMERA_YATAY); t += dt
        y0 = core.yandanlik_f
        hedef = y0 + 0.95 * (1.0 - y0)
        t0 = t
        for _ in range(int(3.0 * hz)):          # adım: aspect 90
            r = core.process(make_pose(8, 90), t, ATT_KAMERA_YATAY); t += dt
            if r["yandanlik_f"] >= hedef:
                return t - t0
        return float("inf")
    s10, s30 = oturma(10), oturma(30)
    kontrol("T10 filtre oturması Hz'den bağımsız",
            abs(s10 - s30) / max(s10, s30) < 0.20, f"10Hz={s10:.3f}s 30Hz={s30:.3f}s")

    # ── T11: görüntü merkezi → pitch_hata=+25.00, yaw_hata=0.00 ──
    # Tilt dönüşümü testi: lead kapalı (K_LEAD=0) — saf yön zinciri ölçülür.
    cfg = cfg_copy(); cfg.K_LEAD = 0.0
    r = tek_kare(cfg, make_pose(8, 90), att=(0.0, 0.0, 0.0))
    kontrol("T11 tilt telafisi (merkez → +25°)",
            abs(r["pitch_hata_deg"] - 25.0) < 0.01 and abs(r["yaw_hata_deg"]) < 0.01,
            f"pitch={r['pitch_hata_deg']:.2f}° yaw={r['yaw_hata_deg']:.2f}°")

    # ── T12: görüş zarfı (lead'i kapatarak saf yön dönüşümü test edilir) ──
    cfg = cfg_copy(); cfg.K_LEAD = 0.0
    ust = tek_kare(cfg, make_pose(8, 90, cy=0), att=(0, 0, 0))
    alt = tek_kare(cfg, make_pose(8, 90, cy=480), att=(0, 0, 0))
    sag = tek_kare(cfg, make_pose(8, 90, cx=640), att=(0, 0, 0))
    kontrol("T12 görüş zarfı üst kenar +80.2°",
            abs(ust["pitch_hata_deg"] - 80.2) < 0.1, f"{ust['pitch_hata_deg']:.2f}°")
    # Not: tam değer 25 − atan(240/166.6) = −30.24°; spec'teki −30.1 yuvarlama
    # tutarsızlığı (üst kenarı 80.24→80.2 diye doğru vermiş) → tolerans 0.2.
    kontrol("T12 görüş zarfı alt kenar -30.1°",
            abs(alt["pitch_hata_deg"] + 30.1) < 0.2, f"{alt['pitch_hata_deg']:.2f}°")
    kontrol("T12 görüş zarfı sağ kenar yaw +64.7°",
            abs(sag["yaw_hata_deg"] - 64.7) < 0.1, f"{sag['yaw_hata_deg']:.2f}°")

    # ── T13: KAMERA_TILT_DEG=0 → merkez pitch_hata=0 (tilt kapsüllemesi) ──
    cfg = cfg_copy(); cfg.KAMERA_TILT_DEG = 0.0
    r = tek_kare(cfg, make_pose(8, 90), att=(0.0, 0.0, 0.0))
    kontrol("T13 tilt=0 kapsülleme", abs(r["pitch_hata_deg"]) < 0.01,
            f"pitch={r['pitch_hata_deg']:.3f}°")

    # ── T14: yükselti düzeltme katsayıları ──
    beklenen = [(25.0, 1.086), (38.7, 1.179), (56.4, 1.302)]
    ok = all(abs(yukselti_duzeltme(math.radians(e)) - d) < 0.001 for e, d in beklenen)
    kontrol("T14 düzeltme katsayıları", ok,
            f"{[(e, round(yukselti_duzeltme(math.radians(e)), 3)) for e, d in beklenen]}")

    # ── T15: tam alttan (eps=90) seviyeli hedef → yandanlık=1.00 ──
    # Seviyeli hedefe dikey LOS: gövde VE kanat izdüşümü tam boy (a=fL/R, b=fW/R).
    # Kamera dik yukarı: pitch 65° + tilt 25° = 90°.
    att_dik = (0.0, math.radians(65.0), 0.0)
    R = 8.0
    pose_alttan = make_pose(R, 0, a_ovr=FX * GOVDE_BOYU_M / R,
                            b_ovr=FX * KANAT_ACIKLIGI_M / R)
    r = tek_kare(cfg_copy(), pose_alttan, att=att_dik)
    kontrol("T15 eps=90 → yandanlık 1.00 (düzeltmeli)",
            abs(r["yandanlik_ham"] - 1.0) < 0.01 and abs(r["eps_deg"] - 90.0) < 0.1,
            f"yandanlik={r['yandanlik_ham']:.4f} eps={r['eps_deg']:.1f}° "
            f"duzeltme={r['duzeltme']:.3f}")

    # ── T16: YUKSELTI_DUZELT=False → aynı senaryo 0.707 ──
    cfg = cfg_copy(); cfg.YUKSELTI_DUZELT = False
    r = tek_kare(cfg, pose_alttan, att=att_dik)
    kontrol("T16 düzeltme kapalı → 0.707",
            abs(r["yandanlik_ham"] - 0.707) < 0.01,
            f"yandanlik={r['yandanlik_ham']:.4f}")

    # ── T17: copter adaptörü hız+yaw üretir, roll/pitch ÜRETMEZ ──
    cfg = cfg_copy()
    ad = CopterAdapter(cfg)
    out = ad.compute(np.array([0.9, 0.1, -0.4]), 0.1, (0, 0, 0), 1.0 / 30.0, 0.0)
    kontrol("T17 copter çıkışı hız+yaw",
            "v_cmd" in out and "yaw_cmd" in out
            and "roll_cmd" not in out and "pitch_cmd" not in out,
            f"alanlar={sorted(out.keys())}")

    # ── T18: |v_cmd| = V_KAPANMA (±%1), yönü u_dunya ile aynı ──
    ad = CopterAdapter(cfg)
    u_g = np.array([0.9, 0.1, -0.42]); u_g = u_g / np.linalg.norm(u_g)
    out = None
    for i in range(200):                      # rampa otursun
        out = ad.compute(u_g, 0.0, (0, 0, 0), 1.0 / 30.0, 0.0)
    v = np.array(out["v_cmd"]); vn = np.linalg.norm(v)
    yon = float(np.dot(v / vn, out["u_dunya"]))
    kontrol("T18 |v|=V_KAPANMA ve yön=u_dunya",
            abs(vn - cfg.V_KAPANMA) / cfg.V_KAPANMA < 0.01 and yon > 0.9999,
            f"|v|={vn:.3f} m/s yön·u_dunya={yon:.6f}")

    # ── T19: fixedwing stub sessizce GEÇMEZ ──
    try:
        FixedWingAdapter(cfg).compute()
        kontrol("T19 fixedwing stub NotImplementedError", False, "istisna atmadı!")
    except NotImplementedError:
        kontrol("T19 fixedwing stub NotImplementedError", True)

    # ── T20: ivme rampası — hız sıçramasında uygulanan ivme ≤ IVME_TAVAN ──
    ad = CopterAdapter(cfg)                   # v_onceki = 0
    dt = 1.0 / 30.0
    out = ad.compute(np.array([1.0, 0.0, 0.0]), 0.0, (0, 0, 0), dt, 0.0)
    ivme = np.linalg.norm(out["v_cmd"]) / dt
    kontrol("T20 ivme tavanı", ivme <= cfg.IVME_TAVAN * (1 + 1e-9),
            f"uygulanan={ivme:.2f} m/s² tavan={cfg.IVME_TAVAN}")

    # ── T21: K_LEAD=1.5, yandanlık≈0.9 → durum='cozumsuz' + WARN ──
    cfg = cfg_copy(); cfg.K_LEAD = 1.5
    r = tek_kare(cfg, make_pose(5, 64.2))     # sin(64.2°)≈0.90
    kontrol("T21 çözümsüzlük işareti",
            r["durum"] == "cozumsuz" and "cozumsuz" in r["warn"]
            and r["lead_deg"] <= cfg.MAX_LEAD_DEG,
            f"durum={r['durum']} yandanlik={r['yandanlik_ham']:.3f} "
            f"lead={r['lead_deg']:.1f}°")

    # ── T22: supervisor geçiş zinciri GPS→VISUAL→(kayıp)→GPS→VISUAL→durdur ──
    import threading
    import time as _t
    import control.guidance.supervisor as sup
    olaylar = []
    _orij_gps, _orij_vis = sup.run_gps_approach, sup.run_visual_lead

    def fake_gps(conn, gp, gi, stop_event):
        olaylar.append("gps")
        stop_event.wait(5.0)          # izci görsel kilitle tetikleyene kadar

    def fake_visual(conn, wait_pose, gpt, stop_event, cfg=None, kayip_kare_esik=None):
        olaylar.append("visual")
        return "kayip" if olaylar.count("visual") == 1 else "durduruldu"

    sayac = {"seq": 0}
    def fake_wait(son_seq, timeout=0.5):
        _t.sleep(0.002)
        sayac["seq"] += 1
        return {"seq": sayac["seq"], "pose": {"conf": 0.9},
                "stamp": sayac["seq"] / 30.0, "wall_recv": _t.time()}

    try:
        sup.run_gps_approach, sup.run_visual_lead = fake_gps, fake_visual
        sup._ga.status["handoff"] = True          # menzil kapısı açık
        stop = threading.Event()
        th = threading.Thread(
            target=sup.run_hybrid,
            args=(None, None, None, fake_wait, None, stop), daemon=True)
        th.start()
        th.join(10.0)
        kontrol("T22 supervisor geçiş zinciri",
                olaylar == ["gps", "visual", "gps", "visual"]
                and sup.status["faz"] == "DURDU" and sup.status["gecis_sayisi"] == 2,
                f"olaylar={olaylar} faz={sup.status['faz']} "
                f"geçiş={sup.status['gecis_sayisi']}")
    finally:
        sup.run_gps_approach, sup.run_visual_lead = _orij_gps, _orij_vis
        sup._ga.status["handoff"] = False

    # ── T23: supervisor 'vuruldu' → görev biter, faz=VURULDU ──
    olaylar2 = []
    _og, _ov = sup.run_gps_approach, sup.run_visual_lead

    def fake_gps2(conn, gp, gi, stop_event):
        olaylar2.append("gps"); stop_event.wait(5.0)

    def fake_visual_vurus(conn, wp, gpt, stop_event, cfg=None, kayip_kare_esik=None):
        olaylar2.append("visual"); return "vuruldu"

    try:
        sup.run_gps_approach, sup.run_visual_lead = fake_gps2, fake_visual_vurus
        sup._ga.status["handoff"] = True
        stop = threading.Event()
        th = threading.Thread(target=sup.run_hybrid,
                              args=(None, None, None, fake_wait, None, stop),
                              daemon=True)
        th.start(); th.join(10.0)
        kontrol("T23 supervisor VURULDU → görev biter",
                olaylar2 == ["gps", "visual"] and sup.status["faz"] == "VURULDU",
                f"olaylar={olaylar2} faz={sup.status['faz']}")
    finally:
        sup.run_gps_approach, sup.run_visual_lead = _og, _ov
        sup._ga.status["handoff"] = False

    # ── T24/T25: visual_lead terminal (kör dalış → vuruş / süre dolunca ıska) ──
    import control.guidance.visual_lead as vlmod

    class _Msg:
        def __init__(s, t, **kw): s._t = t; s.__dict__.update(kw)
        def get_type(s): return s._t
        def get_srcSystem(s): return 1

    class _FakeMav:
        def __init__(s): s.gonderilen = []
        def set_position_target_local_ned_send(s, *a): s.gonderilen.append(a)

    class _FakeConn:
        target_system = 1; target_component = 1
        def __init__(s): s.mav = _FakeMav(); s._q = []
        def durum_yaz(s, pos):
            s._q = [_Msg("ATTITUDE", roll=0.0, pitch=0.0, yaw=0.0),
                    _Msg("HEARTBEAT", custom_mode=4),
                    _Msg("LOCAL_POSITION_NED", x=pos[0], y=pos[1], z=pos[2])]
        def recv_match(s, type=None, blocking=False):
            return s._q.pop(0) if s._q else None

    def terminal_kosusu(cfg, menzil_dizisi, pose_var_dizisi):
        """menzil_dizisi: her karede iris'in hedefe uzaklığı; pose_var_dizisi:
        o karede pose geldi mi (True) / tespit yok (False). get_plane_truth
        orijinde, iris (r,0,0)."""
        conn = _FakeConn()
        durum = {"i": 0}
        def wp(son_seq, timeout=0.5):
            i = durum["i"]
            if i >= len(menzil_dizisi):
                return None
            durum["i"] += 1
            r = menzil_dizisi[i]
            conn.durum_yaz((r, 0.0, 0.0))
            pose = make_pose(8, 90) if pose_var_dizisi[i] else None
            return {"seq": i + 1, "pose": pose, "stamp": (i + 1) / 30.0,
                    "wall_recv": _t.time()}
        gpt = lambda: {"x": 0.0, "y": 0.0, "z": 0.0}
        stop = threading.Event()
        return vlmod.run_visual_lead(conn, wp, gpt, stop, cfg=cfg,
                                     kayip_kare_esik=20)

    cfgT = cfg_copy()
    cfgT.TERMINAL_MENZIL = 8.0; cfgT.VURUS_MENZIL = 1.5; cfgT.TERMINAL_SURE = 2.0

    # T24: yaklaş (10→6, pose var), sonra tespit KESİL ama menzil kapanmaya devam
    # (6→1.2): kör dalış devreye girer, menzil<1.5 → VURULDU
    menz = [10, 9, 8, 7, 6.5, 6] + [5, 4, 3, 2, 1.2]
    posev = [True] * 6 + [False] * 5
    sonuc = terminal_kosusu(cfgT, menz, posev)
    kontrol("T24 kör dalış → VURULDU",
            sonuc == "vuruldu", f"sonuç={sonuc}")

    # T25: yaklaş sonra tespit kesil, menzil kapanıyor ama VURUS'a inmeden süre
    # dolar (TERMINAL_SURE kısa) → ıska (kayip)
    cfgT2 = cfg_copy()
    cfgT2.TERMINAL_MENZIL = 8.0; cfgT2.VURUS_MENZIL = 1.5; cfgT2.TERMINAL_SURE = 0.08
    menz2 = [10, 8, 7, 6, 5.5, 5] + [4.9, 4.8, 4.7, 4.6, 4.5, 4.4, 4.3, 4.2, 4.1, 4.0]
    posev2 = [True] * 6 + [False] * 10
    sonuc2 = terminal_kosusu(cfgT2, menz2, posev2)
    kontrol("T25 kör dalış süresi dolunca ıska (kayip)",
            sonuc2 == "kayip", f"sonuç={sonuc2}")

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
