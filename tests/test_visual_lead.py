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
from control.guidance.guidance_core import (
    GOVDE_BOYU_M, KANAT_ACIKLIGI_M, LeadPursuitCore, cfg_copy,
    govde_to_dunya, hedef_kadraj_hatasi, yukselti_duzeltme)
from control.guidance.visual_lead import _cevap_anahtari, _vurus_oldu
from control.guidance.common import normalize_angle as _norm
from vision import geometry as geo

# Test CSV'leri GERÇEK uçuş loglarına karışmasın → tmp'ye yaz (2026-08-02)
import tempfile as _tf
import control.guidance.visual_lead as _vlmod_logfix
import control.guidance.gps_guidance as _ggmod_logfix
_vlmod_logfix._LOG_DIR = _tf.mkdtemp(prefix="avci_test_logs_")
_ggmod_logfix._LOG_DIR = _vlmod_logfix._LOG_DIR

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

    # ── T20: ivme rampası — hız sıçramasında uygulanan ivme ≤ IVME_TAVAN ──
    # Tavan env ile override edilebildiği için testte açıkça sabitlenir.
    # KP_KADRAJ=0: kadraj tutma düzeltmesi kapatılır, yoksa yatay nişan
    # kadraj merkezinden (25°) saptığı için DİKEY bileşen doğar ve ölçülen
    # ivme yatay tavanla kıyaslanamaz hâle gelir (bu test yatayı ölçüyor).
    cfg = cfg_copy(); cfg.IVME_TAVAN = 4.0; cfg.KP_KADRAJ = 0.0
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
    _orij_gps, _orij_vis = sup.run_gps_guidance, sup.run_visual_lead

    def fake_gps(conn, gp, gi, stop_event):
        olaylar.append("gps")
        stop_event.wait(5.0)          # izci görsel kilitle tetikleyene kadar

    def fake_visual(conn, wait_pose, gpt, stop_event, cfg=None, kayip_kare_esik=None,
                    **kw):                                # get_temas/get_menzil uyumu
        olaylar.append("visual")
        return "kayip" if olaylar.count("visual") == 1 else "durduruldu"

    sayac = {"seq": 0}
    def fake_wait(son_seq, timeout=0.5):
        _t.sleep(0.002)
        sayac["seq"] += 1
        return {"seq": sayac["seq"], "pose": {"conf": 0.9},
                "stamp": sayac["seq"] / 30.0, "wall_recv": _t.time()}

    try:
        sup.run_gps_guidance, sup.run_visual_lead = fake_gps, fake_visual
        sup._ga.status["d_h"] = 10.0          # menzil kapısı açık
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
        sup.run_gps_guidance, sup.run_visual_lead = _orij_gps, _orij_vis
        sup._ga.status["d_h"] = None

    # ── T23: supervisor 'vuruldu' → görev biter, faz=VURULDU ──
    olaylar2 = []
    _og, _ov = sup.run_gps_guidance, sup.run_visual_lead

    def fake_gps2(conn, gp, gi, stop_event):
        olaylar2.append("gps"); stop_event.wait(5.0)

    def fake_visual_vurus(conn, wp, gpt, stop_event, cfg=None, kayip_kare_esik=None,
                          **kw):                          # get_temas/get_menzil uyumu
        olaylar2.append("visual"); return "vuruldu"

    try:
        sup.run_gps_guidance, sup.run_visual_lead = fake_gps2, fake_visual_vurus
        sup._ga.status["d_h"] = 10.0
        stop = threading.Event()
        th = threading.Thread(target=sup.run_hybrid,
                              args=(None, None, None, fake_wait, None, stop),
                              daemon=True)
        th.start(); th.join(10.0)
        kontrol("T23 supervisor VURULDU → görev biter",
                olaylar2 == ["gps", "visual"] and sup.status["faz"] == "VURULDU",
                f"olaylar={olaylar2} faz={sup.status['faz']}")
    finally:
        sup.run_gps_guidance, sup.run_visual_lead = _og, _ov
        sup._ga.status["d_h"] = None

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

    # ── T26: GÜRÜLTÜLÜ menzil (zıplayan) — kilit tutmalı, vuruşu yakalamalı.
    # Eski kapaniyor-bayraklı mantık bu senaryoda kör dalışı düşürüyordu. ──
    cfgT3 = cfg_copy()
    cfgT3.TERMINAL_MENZIL = 8.0; cfgT3.VURUS_MENZIL = 3.0; cfgT3.TERMINAL_SURE = 1.0
    # yaklaş (pose var) → tespit kesil → menzil canlıdaki gibi ZIPLASIN
    menz3 = [10, 8, 7, 6, 5.5, 5] + [8.6, 3.8, 7.1, 2.7, 5.3, 2.75]
    posev3 = [True] * 6 + [False] * 6
    sonuc3 = terminal_kosusu(cfgT3, menz3, posev3)
    kontrol("T26 gürültülü menzilde kilit tutar + vuruş",
            sonuc3 == "vuruldu", f"sonuç={sonuc3} (2.7/2.75<3.0 yakalanmalı)")

    # ── T27: DİKEY PN — tırmanan hedef (yükseliş artıyor) → aim YUKARI kayar ──
    # Saf takip altından geçiyordu; yumuşatılmış yükseliş oranından PN lead çıkış
    # yükselişini girişin belirgin üstüne iter (yumuşatma lag'ine rağmen).
    cfg = cfg_copy()
    ad = CopterAdapter(cfg)
    dt = 1.0 / 30.0
    out = None
    girdi_elev = None
    for e_deg in range(10, 25):                        # +1°/kare 15 kare → sabit tırmanış
        e = math.radians(e_deg)
        ug = np.array([math.cos(e), 0.0, -math.sin(e)])   # gövde: ileri+yukarı
        out = ad.compute(ug, 0.0, (0, 0, 0), dt, 0.0)
        girdi_elev = e_deg
    cikis_elev = math.degrees(math.asin(max(-1.0, min(1.0, -float(out["u_dunya"][2])))))
    kontrol("T27 dikey PN tırmanan hedefte aim'i yukarı kaydırır",
            cikis_elev > girdi_elev + 2.0 and out["pn_dikey_deg"] > 3.0,
            f"girdi={girdi_elev}° çıkış={cikis_elev:.2f}° pn={out['pn_dikey_deg']:.2f}°")

    # ── T28: TERMİNAL CO-ALTITUDE — sabit hedefte terminal=True aim'i ~COALT yukarı ──
    cfg = cfg_copy()
    e = math.radians(20.0)
    ug = np.array([math.cos(e), 0.0, -math.sin(e)])
    base = CopterAdapter(cfg).compute(ug, 0.0, (0, 0, 0), dt, 0.0, terminal=False)
    term = CopterAdapter(cfg).compute(ug, 0.0, (0, 0, 0), dt, 0.0, terminal=True)
    e_base = math.degrees(math.asin(-float(base["u_dunya"][2])))
    e_term = math.degrees(math.asin(-float(term["u_dunya"][2])))
    kontrol("T28 terminal co-altitude yukarı yanlılık",
            abs((e_term - e_base) - cfg.TERMINAL_COALT_DEG) < 0.5
            and abs(term["coalt_deg"] - cfg.TERMINAL_COALT_DEG) < 1e-6,
            f"base={e_base:.2f}° term={e_term:.2f}° Δ={e_term-e_base:.2f}° "
            f"(beklenen {cfg.TERMINAL_COALT_DEG}°)")

    # ── T29: DİKEY AIM YUMUŞATMA — tek karede dev yükseliş sıçraması KIRPILIR ──
    # (kpt bimodal ~49° zıplama vz'yi chatter'a sokuyordu). Komut yükselişi ham
    # 70°'ye fırlamamalı (slew kırpma) ve |v|=V_KAPANMA korunmalı.
    # Rampa BURADA kapatılır: ölçülen şey dikey aim yumuşatması, ivme tavanı değil
    # (tavan açıkken |v| 4 karede V_KAPANMA'ya oturamaz — T20 rampayı ayrıca test eder).
    # VZ_TERMINAL_MAX da kapatılır: 70° yükselişte dikey bileşen 23.5 m/s olur ve
    # tavan onu kırpınca |v| < V_KAPANMA olur — bu YENİ ve İSTENEN davranış
    # (T56 ayrıca test eder), ama bu testin ölçtüğü şey değil.
    cfg = cfg_copy(); cfg.IVME_TAVAN = 1e6; cfg.IVME_TAVAN_DIKEY = 1e6
    cfg.VZ_TERMINAL_MAX = 0.0
    ad = CopterAdapter(cfg)
    for e_deg in (20.0, 20.5, 21.0):                  # sakin seyir → yumuşatma otursun
        e = math.radians(e_deg)
        ad.compute(np.array([math.cos(e), 0.0, -math.sin(e)]), 0.0, (0, 0, 0), dt, 0.0)
    e = math.radians(70.0)                            # ANİ ~49° sıçrama (bimodal gürültü)
    out = ad.compute(np.array([math.cos(e), 0.0, -math.sin(e)]), 0.0, (0, 0, 0), dt, 0.0)
    cikis_elev = math.degrees(math.asin(max(-1.0, min(1.0, -float(out["u_dunya"][2])))))
    vn = np.linalg.norm(np.array(out["v_cmd"]))
    # ÖLÇÜLEN ŞEY YUMUŞATILMIŞ AIM — PN ve co-altitude çıkarılır. Aksi hâlde test
    # PN tavanına bağlı kalır ve PN her ayarlandığında yanlış alarm verir
    # (2026-07-31: PN 15°→30° olunca eski sabit 45° sınırı kırılmıştı; yumuşatma
    # bozulduğu için değil, teste PN de dâhil olduğu için).
    yumusatilmis = (cikis_elev - out["pn_dikey_deg"] - out["coalt_deg"]
                    - out["kadraj_duz_deg"])
    kontrol("T29 dikey aim yumuşatma sıçramayı kırpar",
            yumusatilmis < 30.0                       # ham 70° → yumuşatılmış ≪
            and abs(out["pn_dikey_deg"]) <= cfg.PN_DIKEY_MAX_DEG + 1e-6
            and abs(vn - cfg.V_KAPANMA) / cfg.V_KAPANMA < 0.01,
            f"ham=70° yumuşatılmış={yumusatilmis:.2f}° (komut {cikis_elev:.2f}° "
            f"− pn {out['pn_dikey_deg']:.2f}° − coalt {out['coalt_deg']:.2f}° "
            f"− kadraj {out['kadraj_duz_deg']:.2f}°) |v|={vn:.2f}")

    # ══ T30-T33: AZİMUT TEKİLLİĞİ (2026-07-31 kendi-etrafında-dönme düzeltmesi) ══
    # 141017 uçuşu, ardışık üç kare — nişan dikeye yaklaşınca atan2(y,x) tanımsız:
    #   u_govde (+0.190,+0.403,-0.895) yatay 0.446 → yaw_hata  +64.7°
    #   u_govde (-0.024,-0.012,-0.9996) yatay 0.027 → yaw_hata -154.2°
    #   u_govde (-0.102,+0.118,-0.988)  yatay 0.156 → yaw_hata +130.8°
    # Sonuç: tek karede 136° yaw komut sıçraması, 4.1 s'te 637° dönüş.

    # ── T30: azimut_kalite yükselişle 1→0 iner, tekil karede TAM 0 ──
    cfg = cfg_copy(); cfg.K_LEAD = 0.0
    def _kalite(ug):
        """u_govde'yi doğrudan Adım 8 geometrisinden geçir (pose üretmeden)."""
        yatay = math.hypot(ug[0], ug[1])
        yt = math.cos(math.radians(cfg.AZIMUT_TAM_YUKSELIS_DEG))
        yk = math.cos(math.radians(cfg.AZIMUT_TEKIL_YUKSELIS_DEG))
        return max(0.0, min(1.0, (yatay - yk) / (yt - yk)))
    k_saglam = _kalite((0.9, 0.1, -0.42))          # yükseliş ~25° → sağlam
    k_sinir  = _kalite((0.190, 0.403, -0.895))     # yatay 0.446, yükseliş ~63°
    k_tekil  = _kalite((-0.0245, -0.0118, -0.9996))  # yatay 0.027, yükseliş ~88°
    kontrol("T30 azimut_kalite: sağlam=1, tekil=0",
            abs(k_saglam - 1.0) < 1e-9 and k_tekil == 0.0 and 0.0 < k_sinir < 1.0,
            f"25°={k_saglam:.2f} 63°={k_sinir:.2f} 88°={k_tekil:.2f}")

    # ── T31: process() azimut_kalite üretir ve tekilde 'azimut_tekil' uyarır ──
    # DİKKAT: u_govde GÖVDE çerçevesindedir (kamera_to_govde), attitude'dan
    # BAĞIMSIZ. Tekillik attitude'la değil, hedefin KADRAJ TEPESİNE çıkmasıyla
    # oluşur: cy=0 → kamera açısı +55.2°, +25° tilt ile gövde yükselişi 80.2°
    # (> AZİMUT_TEKIL 75°). Canlıda lead bunu 88°'ye kadar itmişti.
    cfg = cfg_copy(); cfg.K_LEAD = 0.0
    r_dik = tek_kare(cfg, make_pose(8, 90, cy=0), att=ATT_KAMERA_YATAY)
    r_duz = tek_kare(cfg_copy(), make_pose(8, 90), att=ATT_KAMERA_YATAY)
    kontrol("T31 tekil geometride azimut_kalite=0 + uyarı",
            r_dik["azimut_kalite"] == 0.0 and "azimut_tekil" in r_dik["warn"]
            and r_duz["azimut_kalite"] == 1.0,
            f"dik={r_dik['azimut_kalite']:.2f} warn={r_dik['warn']} "
            f"düz={r_duz['azimut_kalite']:.2f}")

    # ── T32: ASIL DÜZELTME — tekil karede yaw adımı SUSAR ──
    # Aynı ±154°'lik saçma yaw_hata, kapı olmadan tavana çakılır; kapıyla 0 olur.
    cfg = cfg_copy()
    dt = 1.0 / 30.0
    yaw_hata_saçma = math.radians(-154.2)
    kapali = CopterAdapter(cfg).compute(
        np.array([-0.0245, -0.0118, -0.9996]), yaw_hata_saçma,
        (0, 0, 0), dt, 0.0, azimut_kalite=1.0)          # kapı YOK (eski davranış)
    acik = CopterAdapter(cfg).compute(
        np.array([-0.0245, -0.0118, -0.9996]), yaw_hata_saçma,
        (0, 0, 0), dt, 0.0, azimut_kalite=0.0)          # kapı VAR
    kontrol("T32 tekil karede yaw adımı susar",
            abs(acik["yaw_adim_deg"]) < 1e-9 and abs(kapali["yaw_adim_deg"]) > 1.0,
            f"kapısız={kapali['yaw_adim_deg']:.2f}°/kare "
            f"kapılı={acik['yaw_adim_deg']:.2f}°/kare")

    # ── T33: yaw slew tavanı geri geldi + yaw_cmd ±π'ye sarmalı ──
    # 1080°/s iken tavan 36°/kare idi (fiilen kapalı); 90°/s ile 3°/kare.
    cfg = cfg_copy()
    out = CopterAdapter(cfg).compute(np.array([0.9, 0.1, -0.42]),
                                     math.radians(150.0), (0, 0, 0), dt, 0.0)
    tavan_kare = cfg.YAW_HIZ_MAX * dt
    # sarmalama: mevcut_yaw π'ye yakınken komut ±π dışına TAŞMAMALI
    out_sarma = CopterAdapter(cfg).compute(np.array([0.9, 0.1, -0.42]),
                                           math.radians(150.0), (0, 0, 0), dt,
                                           math.pi - 0.01)
    kontrol("T33 yaw slew tavanı + ±π sarmalama",
            abs(out["yaw_adim_deg"]) <= tavan_kare + 1e-9 and out["yaw_doygun"]
            and abs(out_sarma["yaw_cmd"]) <= math.pi + 1e-9,
            f"adım={out['yaw_adim_deg']:.2f}° tavan={tavan_kare:.2f}°/kare "
            f"sarmalı_cmd={math.degrees(out_sarma['yaw_cmd']):.1f}°")

    # ══ T34-T35: LİMİT dt TAVANI (2026-07-31 kare-boşluğu savrulması) ══
    # 160249 uçuşu satır 60: 15 kare kör dalıştan sonra dt=0.825 s geldi.
    # Tavan YAW_HIZ_MAX*dt = 90*0.825 = 74.2° olmuş, 74°'lik adımın tamamı
    # tek MAVLink mesajında gitmişti. DT_TAVAN_S bunu kırpar.

    # ── T34: şişmiş dt'de yaw adımı DT_TAVAN_S payıyla sınırlı ──
    cfg = cfg_copy()
    dt_sismis = 0.825                                  # canlıdaki gerçek değer
    beklenen_tavan = cfg.YAW_HIZ_MAX * cfg.DT_TAVAN_S  # 90 * 0.1 = 9°
    out = CopterAdapter(cfg).compute(np.array([0.9, 0.1, -0.42]),
                                     math.radians(77.8),    # canlıdaki yaw_hata
                                     (0, 0, 0), dt_sismis, 0.0)
    eski_tavan = cfg.YAW_HIZ_MAX * dt_sismis           # kırpma olmasa 74.2°
    kontrol("T34 şişmiş dt'de yaw adımı kırpılır",
            abs(out["yaw_adim_deg"]) <= beklenen_tavan + 1e-9
            and eski_tavan > 5 * beklenen_tavan,
            f"dt={dt_sismis}s adım={out['yaw_adim_deg']:.2f}° "
            f"(kırpmalı tavan {beklenen_tavan:.1f}°, kırpmasız olsaydı {eski_tavan:.1f}°)")

    # ── T35: aynı kırpma ivme rampasında da geçerli ──
    # v_onceki=0'dan V_KAPANMA'ya sıçrama: uygulanan ivme IVME_TAVAN'ı aşmamalı
    # VE tek karede alınan hız DT_TAVAN_S payını geçmemeli.
    cfg = cfg_copy(); cfg.KP_KADRAJ = 0.0              # bkz. T20 gerekçesi
    ad = CopterAdapter(cfg)                            # v_onceki = (0,0,0)
    out = ad.compute(np.array([1.0, 0.0, 0.0]), 0.0, (0, 0, 0), dt_sismis, 0.0)
    dv = np.linalg.norm(np.array(out["v_cmd"]))
    beklenen_dv = cfg.IVME_TAVAN * cfg.DT_TAVAN_S      # 4 * 0.1 = 0.4 m/s
    kirpmasiz_dv = cfg.IVME_TAVAN * dt_sismis          # 3.3 m/s
    kontrol("T35 şişmiş dt'de ivme rampası kırpılır",
            dv <= beklenen_dv + 1e-6 and kirpmasiz_dv > 5 * beklenen_dv,
            f"Δv={dv:.3f} m/s (kırpmalı sınır {beklenen_dv:.2f}, "
            f"kırpmasız olsaydı {kirpmasiz_dv:.2f})")

    # ══ T36-T37: YATAY/DİKEY AYRI İVME TAVANI (2026-07-31 dikey ıska) ══
    # Tek 3B tavan, kameranın YATAY kısıtını (burun eğimi → gökyüzü kaybı)
    # dikeye de dayatıyordu. Dikey ivme burun eğimi gerektirmez; sınır itkidir.
    # Ölçüm: dikey PN %27-78 oranında 15° tavanında "yukarı" derken v_doygun
    # %93-99 idi — komut rampada yok oluyor, drone hedefin ~2 m altından geçiyordu.

    # ── T36: dikey ivme YATAY tavandan bağımsız, DİKEY tavanla sınırlı ──
    cfg = cfg_copy()
    dt = 1.0 / 30.0
    ad = CopterAdapter(cfg)                      # v_onceki = (0,0,0)
    # Saf DİKEY nişan (gövde ileri-yukarı 90°): tüm hız değişimi z ekseninde
    out = ad.compute(np.array([0.0, 0.0, -1.0]), 0.0, (0, 0, 0), dt, 0.0)
    dvz = abs(out["v_cmd"][2])
    kontrol("T36 dikey ivme DİKEY tavanı kullanır",
            abs(dvz - cfg.IVME_TAVAN_DIKEY * dt) < 1e-6
            and cfg.IVME_TAVAN_DIKEY > cfg.IVME_TAVAN,
            f"Δvz={dvz:.3f} m/s (dikey tavan {cfg.IVME_TAVAN_DIKEY}×dt="
            f"{cfg.IVME_TAVAN_DIKEY*dt:.3f}; yatay tavan olsaydı "
            f"{cfg.IVME_TAVAN*dt:.3f})")

    # ── T37: yatay ivme DEĞİŞMEDİ — kamera kısıtı korunuyor ──
    # (Burun eğimi yatay ivmeyle belirlenir; bu tavan gevşerse kamera yere bakar.)
    cfg = cfg_copy()
    ad = CopterAdapter(cfg)
    out = ad.compute(np.array([1.0, 0.0, 0.0]), 0.0, (0, 0, 0), dt, 0.0)
    dvh = math.hypot(out["v_cmd"][0], out["v_cmd"][1])
    kontrol("T37 yatay ivme tavanı korunur (kamera kısıtı)",
            abs(dvh - cfg.IVME_TAVAN * dt) < 1e-6,
            f"Δv_yatay={dvh:.3f} m/s (tavan {cfg.IVME_TAVAN}×dt="
            f"{cfg.IVME_TAVAN*dt:.3f})")

    # ══ T38: MENZİL MAKULLÜK KAPISI (2026-07-31 sahte VURULDU düzeltmesi) ══
    # 193559 uçuşu "vuruldu" diye bitti; bir önceki karede menzil 10.4 m'ydi.
    # 33 ms'de 22.4 → 6.6 m = 479 m/s — fiziksel olarak imkânsız. Doğrulanan
    # 7 gerçek uçuş vuruşunun 1'i bu şekilde sahteydi. Sinyal yalnız log değil:
    # kör dalış tetiğini ve terminal co-altitude kilidini de besliyor.
    # ── T38: menzil makullük kapısı — imkânsız sıçrama reddedilir ──
    cfg = cfg_copy()
    kapi = vlmod._MenzilKapisi(cfg)
    kapi.ekle(22.4, None)                       # ilk örnek: kabul
    d1, ok1 = kapi.ekle(21.7, 1.0 / 30.0)       # makul (21 m/s) → kabul
    # 193559 uçuşundaki gerçek sıçrama: 33 ms'de 22.4 → 6.6 m = 479 m/s
    d2, ok2 = kapi.ekle(6.6, 1.0 / 30.0)
    kontrol("T38 imkânsız menzil sıçraması reddedilir",
            ok1 and not ok2 and abs(d2 - 21.7) < 1e-9 and kapi.red_sayaci == 1,
            f"makul {d1:.1f}m kabul={ok1}; sıçrama sonrası menzil {d2:.1f}m "
            f"(korundu), red={kapi.red_sayaci}")

    # ── T38b: ısrarlı sapma → yeni seviyeye SENKRONİZE ol (bayat değere kilitlenme)
    for _ in range(cfg.MENZIL_RESENK_N):
        d3, ok3 = kapi.ekle(6.6, 1.0 / 30.0)
    kontrol("T38b ısrarlı sapmada yeniden senkronize",
            abs(d3 - 6.6) < 1e-9 and ok3,
            f"{cfg.MENZIL_RESENK_N} ardışık red sonrası menzil {d3:.1f}m")

    # ══ T39: KADRAJ TUTMA — "metre altta kal" değil "AÇI altta kal" ══
    # GPS fazı hedefin SABİT 4.65 m altında park ediyor. Sabit metre, kapanan
    # menzilde sabit açı değildir: 11 m'de 25° (kadraj merkezi), 6 m'de 51°,
    # 4 m'de kadraj DIŞI (+80.2°). Görsel faz hedefi merkeze geri çekmeli.
    cfg = cfg_copy()
    dt = 1.0 / 30.0
    merkez = math.radians(cfg.KAMERA_TILT_DEG)     # kadraj merkezi = tilt açısı

    def _kadraj(elev_govde_deg):
        e = math.radians(elev_govde_deg)
        ad = CopterAdapter(cfg)
        return ad.compute(np.array([math.cos(e), 0.0, -math.sin(e)]), 0.0,
                          (0, 0, 0), dt, 0.0)

    tam_merkez = _kadraj(cfg.KAMERA_TILT_DEG)      # hedef TAM merkezde
    yukarida = _kadraj(cfg.KAMERA_TILT_DEG + 25.0)  # merkezden 25° yukarıda
    asagida = _kadraj(cfg.KAMERA_TILT_DEG - 15.0)   # merkezden 15° aşağıda
    kontrol("T39 kadraj merkezinde düzeltme YOK",
            abs(tam_merkez["kadraj_hata_deg"]) < 1e-6
            and abs(tam_merkez["kadraj_duz_deg"]) < 1e-6,
            f"hata={tam_merkez['kadraj_hata_deg']:.3f}° "
            f"düzeltme={tam_merkez['kadraj_duz_deg']:.3f}°")
    kontrol("T39b hedef yukarı kaçınca nişan YUKARI itilir",
            yukarida["kadraj_duz_deg"] > 0
            and abs(yukarida["kadraj_duz_deg"]
                    - cfg.KP_KADRAJ * yukarida["kadraj_hata_deg"]) < 1e-6,
            f"hata={yukarida['kadraj_hata_deg']:+.1f}° → "
            f"düzeltme={yukarida['kadraj_duz_deg']:+.1f}° (KP={cfg.KP_KADRAJ})")
    kontrol("T39c simetrik: hedef aşağıdayken nişan AŞAĞI iner",
            asagida["kadraj_duz_deg"] < 0,
            f"hata={asagida['kadraj_hata_deg']:+.1f}° → "
            f"düzeltme={asagida['kadraj_duz_deg']:+.1f}°")
    # tavan: aşırı sapmada düzeltme KADRAJ_MAX_DEG'i aşmaz
    asiri = _kadraj(cfg.KAMERA_TILT_DEG + 80.0)
    kontrol("T39d düzeltme tavanı",
            abs(asiri["kadraj_duz_deg"]) <= cfg.KADRAJ_MAX_DEG + 1e-6,
            f"hata={asiri['kadraj_hata_deg']:+.1f}° → "
            f"düzeltme={asiri['kadraj_duz_deg']:+.1f}° (tavan {cfg.KADRAJ_MAX_DEG}°)")

    # ── T40-T43: CEVAP ANAHTARI (ground-truth ölçüm sütunları) ──
    # Kritik risk: pose zinciri (kamera→gövde) ile saf geometri (dünya→gövde)
    # farklı konvansiyon kullanırsa sapma sütunları SABİT bir yanlılık gösterir
    # ve olmayan bir algı hatası kovalanır. T40 iki zincirin aynı dili
    # konuştuğunu kanıtlar.
    class _Aras:
        def __init__(self, pos, att):
            self.pos, self.attitude = pos, att

    cfg = cfg_copy()
    drone, att0 = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    def _anahtar(hedef, cx_kaydir=0.0, res_ver=True):
        t = hedef_kadraj_hatasi(hedef, drone, *att0)
        satir, res = {}, None
        if res_ver and t["u"] is not None:
            res = tek_kare(cfg, make_pose(11.0, 45.0, cx=t["u"] + cx_kaydir,
                                          cy=t["v"]), att=att0)
        _cevap_anahtari(satir, hedef, _Aras(drone, att0), res)
        return satir, t

    # hedef 10 m ileri, 4.65 m yukarıda → gerçek yükseliş ~24.9° (kadraj merkezi)
    s40, t40 = _anahtar((10.0, 0.0, -4.65))
    kontrol("T40 mükemmel pose'ta cevap anahtarı sapması ≈ 0",
            abs(s40["pose_yaw_sapma_deg"]) < 0.01
            and abs(s40["pose_elev_sapma_deg"]) < 0.01,
            f"yaw sapma={s40['pose_yaw_sapma_deg']:+.4f}° "
            f"elev sapma={s40['pose_elev_sapma_deg']:+.4f}° "
            f"(gercek elev={s40['gercek_elev_deg']:.1f}°)")

    # bbox'ı 30 px SAĞA kaydır → gerçek bir azimut sapması görünmeli (sağ +)
    s41, _ = _anahtar((10.0, 0.0, -4.65), cx_kaydir=30.0)
    bekle41 = math.degrees(math.atan(30.0 / FX))
    kontrol("T41 kaydırılmış pose sapma olarak ölçülür (sağ +)",
            s41["pose_yaw_sapma_deg"] > 0
            and abs(s41["pose_yaw_sapma_deg"] - bekle41) < 2.0,
            f"sapma={s41['pose_yaw_sapma_deg']:+.2f}° (beklenen ~{bekle41:+.2f}°)")

    # Problem 1 geometrisi: sabit 4.65 m dikey ofset yakın menzilde kadrajı taşırır
    s42a, _ = _anahtar((10.0, 0.0, -4.65), res_ver=False)
    s42b, t42b = _anahtar((0.5, 0.0, -4.65), res_ver=False)
    kontrol("T42 kadraj içi/dışı ayrımı (dikey ıska imzası)",
            s42a["gercek_kadraj_ici"] == 1 and s42b["gercek_kadraj_ici"] == 0,
            f"10 m'de içeride (elev {s42a['gercek_elev_deg']:.1f}°), "
            f"0.5 m'de DIŞARIDA (elev {s42b['gercek_elev_deg']:.1f}°)")

    # hedef ARKADA: piksel izdüşümü yok, kadraj_ici 0 olmalı (None'a düşmemeli)
    s43, _ = _anahtar((-10.0, 0.0, 0.0), res_ver=False)
    kontrol("T43 hedef arkadayken önde=0 ve kadraj dışı",
            s43["gercek_onde"] == 0 and s43["gercek_kadraj_ici"] == 0
            and "gercek_u_px" not in s43,
            f"onde={s43['gercek_onde']} kadraj_ici={s43['gercek_kadraj_ici']} "
            f"yaw={s43['gercek_yaw_deg']:.1f}°")

    # ── T44-T45: YAW KAÇAK KAPISI (kendi etrafında dönme) ──
    # 2026-08-01 kara kutu ölçümü: araç 443 s'de 33.6 TUR döndü, attitude hedefi
    # (DesYaw) bu dönüşü birebir takip etti (yani komut buydu) ve motorlar HİÇ
    # doymadı (%0.0). Ortalama 91.5 °/s ≈ YAW_HIZ_MAX(90) — "her karede tavan
    # adımı" imzası. Uçuş CSV'lerinde yaw_doygun %91-100, adımlar tek yönlü.
    # Mekanizma: yaw_hata kapanmıyor (bayat/hatalı algı) ama komut her karede
    # aracı bir tavan adımı daha çeviriyor → araç sürekli dönüyor.
    # Kapı bunu kesmeli AMA büyük meşru dönüşleri kesmemeli.
    def _yaw_kapali_cevrim(bayat, hedef_deg=60.0, n=900, tau=0.033):
        """Aracın komutu (neredeyse mükemmel) takip ettiği kapalı çevrim.
        bayat=True: algı, dönüşe rağmen AYNI gövde hatasını bildirir (arıza)."""
        ad = CopterAdapter(cfg_copy())
        yaw, toplam = 0.0, 0.0
        H = math.radians(hedef_deg)
        sabit = _norm(H)
        for _ in range(n):
            yh = sabit if bayat else _norm(H - yaw)
            u = np.array([math.cos(yh), math.sin(yh),
                          -math.tan(math.radians(25.0))])
            u = u / np.linalg.norm(u)
            out = ad.compute(u, yh, (0.0, 0.0, yaw), 1.0 / 30, yaw)
            y0 = yaw
            yaw = _norm(yaw + _norm(out["yaw_cmd"] - yaw) * min(1.0, (1.0 / 30) / tau))
            toplam += _norm(yaw - y0)
        return math.degrees(toplam), math.degrees(_norm(H - yaw))

    d60, k60 = _yaw_kapali_cevrim(False, 60.0)
    d150, k150 = _yaw_kapali_cevrim(False, 150.0)
    kontrol("T44 sağlıklı algıda büyük dönüş TAM yapılır (kapı yanlış tetiklenmez)",
            abs(k60) < 2.0 and abs(k150) < 2.0,
            f"hedef 60° → {d60:+.1f}° (kalan {k60:+.2f}°); "
            f"hedef 150° → {d150:+.1f}° (kalan {k150:+.2f}°)")

    dbayat, _ = _yaw_kapali_cevrim(True, 60.0)
    kapisiz = 90.0 * 30.0        # YAW_HIZ_MAX × 30 s, kapı olmasaydı
    kontrol("T45 bayat algıda yaw kaçağı sınırlanır (sürekli dönme bitiyor)",
            abs(dbayat) < 90.0,
            f"30 s'de {abs(dbayat):.0f}° ({abs(dbayat)/360:.2f} tur) — "
            f"kapısız {kapisiz:.0f}° ({kapisiz/360:.1f} tur) olurdu")

    # ══════════════════════════════════════════════════════════
    #  A5 — GERÇEK ÇARPIŞMA ÖLÇÜTÜ (T46-T48)
    #  _vurus_oldu, temas kaynağını dışarıdan alır; sahte kaynakla test edilir.
    # ══════════════════════════════════════════════════════════
    class SahteKaynak:
        def __init__(self, temas, kaynak):
            self._t, self._k = temas, kaynak
        def temas_var(self):  return self._t
        def kaynak_var(self): return self._k

    c = cfg_copy()

    # ── T46: temas kaynağı ÇALIŞIYOR, temas YOK → yakınlık vuruş SAYILMAZ ──
    # Ölçülen gerçek koşu: tek geçişte 0.61/0.69/0.75/1.06/1.16/1.20 m yaklaşma.
    # Eski ölçütle 6 sahte vuruş; yenisiyle sıfır olmalı.
    yakin = [0.61, 0.69, 0.75, 1.06, 1.16, 1.20]
    kaynak = SahteKaynak(temas=False, kaynak=True)
    sahte = [m for m in yakin if _vurus_oldu(m, c, kaynak)[0]]
    kontrol("T46 temas yokken yakınlık VURUŞ DEĞİL (sahte vuruş elenir)",
            not sahte,
            f"{len(yakin)} yaklaşma ({min(yakin)}-{max(yakin)} m) → "
            f"{len(sahte)} vuruş; eski ölçütle {len(yakin)} sahte vuruş olurdu")

    # ── T47: gerçek temas geldi → menzil ne olursa olsun VURULDU ──
    # Menzil bilinmiyor (None) olsa bile temas temastır; menzil kestirimi
    # kopmuşken de vuruş raporlanabilmeli.
    kaynak = SahteKaynak(temas=True, kaynak=True)
    v_uzak, g_uzak = _vurus_oldu(9.9, c, kaynak)
    v_yok, g_yok = _vurus_oldu(None, c, kaynak)
    kontrol("T47 gerçek temas → menzilden bağımsız VURULDU",
            v_uzak and v_yok and g_uzak == "gercek_temas" == g_yok,
            f"menzil 9.9 m → {g_uzak}; menzil yok → {g_yok}")

    # ── T48: temas kaynağı YOKSA yakınlık yedeği devreye girer ──
    # gz-transport kurulu değil / AVCI_HASAR=0 kurulumlarında vuruş hiç
    # raporlanamaz olmamalı; ama gerekçe 'yedek_menzil' diye işaretlenmeli.
    kaynak = SahteKaynak(temas=False, kaynak=False)
    v_ic, g_ic = _vurus_oldu(c.VURUS_MENZIL - 0.1, c, kaynak)
    v_dis, _ = _vurus_oldu(c.VURUS_MENZIL + 0.1, c, kaynak)
    kontrol("T48 temas kaynağı yokken yakınlık YEDEĞİ çalışır",
            v_ic and (not v_dis) and g_ic == "yedek_menzil",
            f"{c.VURUS_MENZIL-0.1:.2f} m → vuruldu ({g_ic}); "
            f"{c.VURUS_MENZIL+0.1:.2f} m → ıska")

    # ══════════════════════════════════════════════════════════
    # GT ROTASYON MODU (AVCI_GT_ROT=on) — T49-T54
    # Güdümün algı girdisi pose yerine Gazebo gerçek pozu. Bu testlerin işi:
    # GT yolunun pose yoluyla AYNI yasayı çalıştırdığını göstermek.
    # ══════════════════════════════════════════════════════════
    IRIS_POS = (0.0, 0.0, 5.0)
    IRIS_RPY = (0.0, 0.0, 0.0)          # seviyeli drone → kamera 25° yukarı

    def gt_sahne(R, hedef_rpy, sapma=(0.0, 0.0, 0.0)):
        """Hedefi kameranın optik ekseninden R m ileriye koy (+ sapma m).
        Dönüş: (gt_sozluk, hedef_pos)."""
        cam_pos, R_cam = geo.camera_world_pose(IRIS_POS, IRIS_RPY)
        eksen = R_cam @ np.array([1.0, 0.0, 0.0])
        hedef_pos = cam_pos + R * eksen + np.asarray(sapma, float)
        return (geo.rot_gt_goruntu(hedef_pos, hedef_rpy, IRIS_POS, IRIS_RPY),
                hedef_pos)

    # ── T49: GT modu optik eksendeki hedefi kadraj merkezine koyar ──
    gt, _ = gt_sahne(20.0, (0.0, 0.0, math.radians(90)))
    r = LeadPursuitCore(cfg_copy()).process(None, 0.0, ATT_KAMERA_YATAY, gt=gt)
    merkez_sapma = math.hypot(gt["uv"][0] - CX, gt["uv"][1] - CY)
    kontrol("T49 GT modu: optik eksendeki hedef kadraj merkezinde",
            merkez_sapma < 0.5 and r["gt_modu"] is True,
            f"uv=({gt['uv'][0]:.1f},{gt['uv'][1]:.1f}) beklenen=({CX:.0f},{CY:.0f}) "
            f"sapma={merkez_sapma:.3f}px")

    # ── T50: GT modu pose'suz çalışır (pose=None) ve keypoint kapıları susar ──
    # Pose modunda düşük kpt güveni 'kanat_dusuk'/'kpt_dusuk' üretirdi; GT'de
    # ölçüm güveni diye bir şey yok → durum temiz, lead sönmüyor.
    gt, _ = gt_sahne(10.0, (0.0, 0.0, math.radians(90)))
    r = LeadPursuitCore(cfg_copy()).process(None, 0.0, ATT_KAMERA_YATAY, gt=gt)
    kontrol("T50 GT modu pose=None ile çalışır, kpt kapıları devre dışı",
            r["durum"] == "ok" and r["guven"] == 1.0 and r["lead_deg"] > 0.0,
            f"durum={r['durum']} guven={r['guven']} lead={r['lead_deg']:.2f}°")

    # ── T51: GT ölçeği GERÇEK menzilden türer (olcek = fx·L/R) ──
    # Sonuç: menzil_kestirim_m gerçek menzile eşit; yükselti düzeltmesi GT'de
    # UYGULANMAZ (gerçeğe uygulanırsa hata ekler) → duzeltme = 1.0.
    for R_ger in (8.0, 15.0, 25.0):
        gt, _ = gt_sahne(R_ger, (0.0, 0.0, math.radians(90)))
        r = LeadPursuitCore(cfg_copy()).process(None, 0.0, ATT_KAMERA_YATAY, gt=gt)
        if abs(r["menzil_kestirim_m"] - R_ger) > 0.01 or r["duzeltme"] != 1.0:
            break
    kontrol("T51 GT ölçeği gerçek menzilden türer, düzeltme uygulanmaz",
            abs(r["menzil_kestirim_m"] - R_ger) < 0.01 and r["duzeltme"] == 1.0,
            f"R={R_ger:.0f}m → kestirim={r['menzil_kestirim_m']:.3f}m "
            f"olcek={r['olcek']:.2f}px duzeltme={r['duzeltme']}")

    # ── T52: yandanlık gerçek yönelimi izler (yandan >> karşıdan) ──
    gt_yan, _ = gt_sahne(12.0, (0.0, 0.0, math.radians(90)))    # LOS'a dik
    gt_kars, _ = gt_sahne(12.0, (0.0, 0.0, math.radians(180)))  # bize doğru
    y_yan = LeadPursuitCore(cfg_copy()).process(
        None, 0.0, ATT_KAMERA_YATAY, gt=gt_yan)["yandanlik_ham"]
    y_kars = LeadPursuitCore(cfg_copy()).process(
        None, 0.0, ATT_KAMERA_YATAY, gt=gt_kars)["yandanlik_ham"]
    # Karşıdan gelen hedef YATAY uçuyor, biz 25° yukarı bakıyoruz → artık sıfır
    # değil sin(25°)≈0.42 (geometrik olarak doğru, bakış açısı farkı).
    kontrol("T52 GT yandanlık gerçek yönelimi izler",
            y_yan > 0.99 and abs(y_kars - math.sin(math.radians(25))) < 0.02,
            f"yandan={y_yan:.3f} (≈1) karşıdan={y_kars:.3f} "
            f"(≈sin25°={math.sin(math.radians(25)):.3f})")

    # ── T53: KRİTİK — GT yolu ile KUSURSUZ pose yolu aynı yere nişan alır ──
    # Aynı 3B sahneden hem gt hem (geometry ile projekte edilmiş) MÜKEMMEL pose
    # üretilir; iki yol aynı yasayı çalıştırdığı için sonuç yakın olmalı.
    # Geometri seçimi: hedef SEVİYELİ (roll=0) ve 6 keypoint'in tamamı görünür.
    # 25° alttan bakışta çoğu açıda bir kanat ucu gövde arkasında kalıyor —
    # kusursuz-pose kıyası ancak örtülme yokken anlamlıdır (bkz. T53b).
    def gt_pose_kiyas(R, hedef_rpy, sapma=(0.0, 0.0, 0.0)):
        gt_, hp_ = gt_sahne(R, hedef_rpy, sapma)
        kp_ = geo.target_keypoints(hp_, hedef_rpy, IRIS_POS, IRIS_RPY)
        bb_ = geo.target_bbox(hp_, hedef_rpy, IRIS_POS, IRIS_RPY)
        if not all(kp_[i][2] > 0 for i in range(4)) or bb_ is None:
            return None
        pz_ = {"cx": (bb_[0] + bb_[2]) / 2, "cy": (bb_[1] + bb_[3]) / 2,
               "conf": 1.0, "bbox": bb_,
               "kpts": [(float(k[0]), float(k[1]), 1.0) for k in kp_]}
        return (LeadPursuitCore(cfg_copy()).process(None, 0.0, ATT_KAMERA_YATAY, gt=gt_),
                LeadPursuitCore(cfg_copy()).process(pz_, 0.0, ATT_KAMERA_YATAY))

    SEVIYELI = (0.0, 0.0, math.pi)     # seviyeli, burnu bize dönük (örtülme yok)
    en_kotu = {"yaw": 0.0, "pit": 0.0, "yan": 0.0, "lead": 0.0}
    kiyas_sayisi = 0
    for _sap in ((0.0, 0.0, 0.0), (0.0, 1.5, 0.0), (0.0, -2.0, 1.0)):
        _k = gt_pose_kiyas(12.0, SEVIYELI, _sap)
        if _k is None:
            continue
        kiyas_sayisi += 1
        _g, _p = _k
        en_kotu["yaw"] = max(en_kotu["yaw"],
                             abs(math.degrees(_norm(_g["yaw_hata"] - _p["yaw_hata"]))))
        en_kotu["pit"] = max(en_kotu["pit"],
                             abs(math.degrees(_g["pitch_hata"] - _p["pitch_hata"])))
        en_kotu["yan"] = max(en_kotu["yan"],
                             abs(_g["yandanlik_ham"] - _p["yandanlik_ham"]))
        en_kotu["lead"] = max(en_kotu["lead"], abs(_g["lead_deg"] - _p["lead_deg"]))
    kontrol("T53 GT yolu ≈ kusursuz pose yolu (aynı yasa, aynı nişan)",
            kiyas_sayisi == 3 and en_kotu["yaw"] < 2.0 and en_kotu["pit"] < 2.5
            and en_kotu["yan"] < 0.02 and en_kotu["lead"] < 3.0,
            f"{kiyas_sayisi}/3 sahne — en kötü: Δyaw={en_kotu['yaw']:.2f}° "
            f"Δpitch={en_kotu['pit']:.2f}° Δyandanlik={en_kotu['yan']:.4f} "
            f"Δlead={en_kotu['lead']:.2f}°")

    # ── T53b: hedef YATIRINCA iki yol BİLEREK ayrışır (GT daha doğru) ──
    # pose yandanlığı a/olcek'tir ve 'hedef seviyeli uçuyor' varsayar; hedef
    # bank yaparken kanat izdüşümü büyük kalır → yandanlık DÜŞÜK ölçülür.
    # GT ise gövde ekseni ile LOS arasındaki gerçek |sin θ|'yı verir. Bu fark
    # GT modunun ölçtüğü asıl şeydir: pose modundaki manevra körlüğü.
    YATIK = (math.radians(45), 0.0, math.radians(90))   # 45° bank, tam yandan
    _k = gt_pose_kiyas(12.0, YATIK)
    if _k is None:
        kontrol("T53b yatık hedefte GT ile pose ayrışır (manevra körlüğü)",
                False, "kıyas geometrisi örtülü — sahne seçimi gözden geçirilmeli")
    else:
        _g, _p = _k
        kontrol("T53b yatık hedefte GT ile pose ayrışır (manevra körlüğü)",
                _g["yandanlik_ham"] > 0.95 and _p["yandanlik_ham"] < 0.85,
                f"GT yandanlik={_g['yandanlik_ham']:.3f} (gerçek, tam yandan) "
                f"pose={_p['yandanlik_ham']:.3f} (seviyeli varsayımı) → "
                f"lead {_g['lead_deg']:.1f}° vs {_p['lead_deg']:.1f}°")

    # ── T54: REGRESYON — gt=None verilince davranış birebir eski pose modu ──
    # GT dalı eklenirken pose yolunun bozulmadığının kanıtı.
    p = make_pose(9.0, 60.0, cx=380.0, cy=260.0, d_aci_deg=20.0)
    r_a = LeadPursuitCore(cfg_copy()).process(p, 0.0, ATT_KAMERA_YATAY)
    r_b = LeadPursuitCore(cfg_copy()).process(p, 0.0, ATT_KAMERA_YATAY, gt=None)
    ayni = all(abs(r_a[k] - r_b[k]) < 1e-12 for k in
               ("a", "b", "olcek", "yandanlik_ham", "lead_deg", "yaw_hata",
                "pitch_hata", "kalite", "duzeltme"))
    kontrol("T54 gt=None → pose modu birebir korunur (regresyon)",
            ayni and r_a["gt_modu"] is False,
            f"lead={r_a['lead_deg']:.3f}° yaw={math.degrees(r_a['yaw_hata']):.3f}° "
            f"olcek={r_a['olcek']:.2f}px")

    # ── T55: MENZİL KAYNAĞI — zaman hizalı gz, telemetriye TERCİH edilir ──
    # 2026-08-04'e kadar _menzil_olc() tanımlıydı ama hiç çağrılmıyordu; menzil
    # sessizce telemetriden geliyordu ve loglarda karelerin %37'si donuktu.
    # Bu test kaynağın gerçekten gz olduğunu ve DEĞERİN gz'den geldiğini
    # doğrular (ikisi farklı olacak şekilde kurgulanır).
    import csv as _csv
    import glob as _glob
    import os as _os

    cfgM = cfg_copy()
    cfgM.TERMINAL_MENZIL = 2.0      # terminal/vuruş yollarına hiç girme
    cfgM.VURUS_MENZIL = 0.5
    _conn = _FakeConn()
    _menz_telem = [20.0, 19.0, 18.0, 17.0, 16.0]
    _GZ_FARK = 2.0                  # gz telemetriden 2 m FARKLI okusun
    _sim = {"i": 0}

    def _wp_m(son_seq, timeout=0.5):
        i = _sim["i"]
        if i >= len(_menz_telem):
            return None
        _sim["i"] += 1
        _conn.durum_yaz((_menz_telem[i], 0.0, 0.0))
        return {"seq": i + 1, "pose": make_pose(8, 90), "stamp": (i + 1) / 30.0,
                "wall_recv": _t.time()}

    def _gz_menzil():
        j = max(0, _sim["i"] - 1)
        return _menz_telem[j] - _GZ_FARK

    vlmod.run_visual_lead(_conn, _wp_m, lambda: {"x": 0.0, "y": 0.0, "z": 0.0},
                          threading.Event(), cfg=cfgM, kayip_kare_esik=20,
                          get_menzil=_gz_menzil)
    _son_csv = max(_glob.glob(_os.path.join(vlmod._LOG_DIR, "visual_lead_*.csv")),
                   key=_os.path.getmtime)
    _R = [r for r in _csv.DictReader(open(_son_csv)) if r.get("menzil_ham_m")]
    _kaynaklar = {r.get("menzil_kaynak") for r in _R}
    _degerler_gz = all(
        abs(float(r["menzil_ham_m"]) - (float(_menz_telem[i]) - _GZ_FARK)) < 1e-6
        for i, r in enumerate(_R) if i < len(_menz_telem))
    kontrol("T55 menzil kaynağı gz'yi tercih eder (telemetri yalnız yedek)",
            len(_R) > 0 and _kaynaklar == {"gz"} and _degerler_gz,
            f"{len(_R)} satır, kaynak={_kaynaklar}, "
            f"değerler telemetriden {_GZ_FARK} m farklı: {_degerler_gz}")

    # T55b: gz yoksa telemetriye düşer (kaynak sütunu bunu dürüstçe söyler)
    _sim["i"] = 0
    vlmod.run_visual_lead(_conn, _wp_m, lambda: {"x": 0.0, "y": 0.0, "z": 0.0},
                          threading.Event(), cfg=cfgM, kayip_kare_esik=20,
                          get_menzil=lambda: None)
    _son_csv2 = max(_glob.glob(_os.path.join(vlmod._LOG_DIR, "visual_lead_*.csv")),
                    key=_os.path.getmtime)
    _R2 = [r for r in _csv.DictReader(open(_son_csv2)) if r.get("menzil_ham_m")]
    kontrol("T55b gz yokken telemetri yedeği + kaynak dürüst etiketlenir",
            len(_R2) > 0 and {r.get("menzil_kaynak") for r in _R2} == {"telem"}
            and abs(float(_R2[0]["menzil_ham_m"]) - _menz_telem[0]) < 1e-6,
            f"{len(_R2)} satır, kaynak=telem, ilk menzil={_R2[0]['menzil_ham_m']}")

    # ══════════════════════════════════════════════════════════
    # T56-T59 — YAKLAŞMA ALT-FAZI (2026-08-05)
    # Görsel faz devirden itibaren sabit V_KAPANMA ile dalıyordu; ölçüldü ki
    # kapanma hızı menzilden bağımsız (0-2 m'de bile 24.4 m/s = kare başına
    # 0.81 m, ıska mesafesi medyanının tamamı). Artık devirden sonra önce
    # YAKLAŞMA: yavaşla + hedefle irtifayı eşitle, sonra terminal.
    # ══════════════════════════════════════════════════════════
    dtA = 1.0 / 30.0
    cfgA = cfg_copy()
    cfgA.IVME_TAVAN = 1e6; cfgA.IVME_TAVAN_DIKEY = 1e6   # rampa testi değil
    e20 = math.radians(20.0)
    ug20 = np.array([math.cos(e20), 0.0, -math.sin(e20)])

    # ── T56: terminalde dikey tavan — nişan dikeye yaklaşınca vz kırpılır ──
    e70 = math.radians(70.0)
    ug70 = np.array([math.cos(e70), 0.0, -math.sin(e70)])
    r_t = CopterAdapter(cfgA).compute(ug70, 0.0, (0, 0, 0), dtA, 0.0,
                                      menzil=cfgA.TERMINAL_MENZIL - 1.0)
    vz_ham = cfgA.V_KAPANMA * math.sin(e70)
    kontrol("T56 terminalde dikey hız tavanı (yukarı fırlama kesiliyor)",
            r_t["alt_faz"] == "terminal"
            and abs(r_t["v_cmd"][2]) <= cfgA.VZ_TERMINAL_MAX + 1e-6
            and vz_ham > cfgA.VZ_TERMINAL_MAX,
            f"tavansız {vz_ham:.1f} m/s olurdu → {abs(r_t['v_cmd'][2]):.1f} m/s "
            f"(tavan {cfgA.VZ_TERMINAL_MAX})")

    # ── T57: YAKLAŞMA alt-fazı yavaşlatır ──
    r_y = CopterAdapter(cfgA).compute(ug20, 0.0, (0, 0, 0), dtA, 0.0,
                                      menzil=cfgA.TERMINAL_MENZIL + 10.0)
    yatay_y = math.hypot(r_y["v_cmd"][0], r_y["v_cmd"][1])
    r_tt = CopterAdapter(cfgA).compute(ug20, 0.0, (0, 0, 0), dtA, 0.0,
                                       menzil=cfgA.TERMINAL_MENZIL - 1.0)
    yatay_t = math.hypot(r_tt["v_cmd"][0], r_tt["v_cmd"][1])
    kontrol("T57 yaklaşmada yatay hız V_YAKLASMA'ya iner, terminalde tam hız",
            r_y["alt_faz"] == "yaklasma" and r_tt["alt_faz"] == "terminal"
            and abs(yatay_y - cfgA.V_YAKLASMA) < 0.1 and yatay_t > yatay_y * 1.5,
            f"yaklaşma {yatay_y:.1f} m/s (hedef {cfgA.V_YAKLASMA}) · "
            f"terminal {yatay_t:.1f} m/s")

    # ── T58: yaklaşmada dikey, hedefle İRTİFA FARKINI kapatır ──
    # Hedef 20° yukarıda, 20 m menzilde → dikey fark ≈ 6.8 m → drone TIRMANMALI
    # (NED'de vz negatif). Ters durumda (hedef aşağıda) alçalmalı.
    m20 = 20.0
    r_up = CopterAdapter(cfgA).compute(ug20, 0.0, (0, 0, 0), dtA, 0.0, menzil=m20)
    ug_dn = np.array([math.cos(e20), 0.0, +math.sin(e20)])      # hedef AŞAĞIDA
    r_dn = CopterAdapter(cfgA).compute(ug_dn, 0.0, (0, 0, 0), dtA, 0.0, menzil=m20)
    kontrol("T58 yaklaşmada dikey irtifa farkını kapatır (yön doğru, tavanlı)",
            r_up["v_cmd"][2] < 0 and r_dn["v_cmd"][2] > 0
            and abs(r_up["v_cmd"][2]) <= cfgA.VZ_YAKLASMA + 1e-6,
            f"hedef yukarıda → vz={r_up['v_cmd'][2]:+.2f} (tırmanma) · "
            f"aşağıda → vz={r_dn['v_cmd'][2]:+.2f} · tavan {cfgA.VZ_YAKLASMA}")

    # ── T59: REGRESYON — menzil verilmezse davranış BİREBİR eski ──
    cfgB = cfg_copy(); cfgB.IVME_TAVAN = 1e6; cfgB.IVME_TAVAN_DIKEY = 1e6
    cfgB.VZ_TERMINAL_MAX = 0.0                    # eski: dikey tavan yok
    r_a = CopterAdapter(cfgB).compute(ug20, 0.0, (0, 0, 0), dtA, 0.0)
    r_b = CopterAdapter(cfgB).compute(ug20, 0.0, (0, 0, 0), dtA, 0.0, menzil=None)
    vn = np.linalg.norm(np.array(r_a["v_cmd"]))
    kontrol("T59 menzil=None → eski davranış birebir (regresyon)",
            r_a["alt_faz"] == "terminal"
            and all(abs(x - y) < 1e-12 for x, y in zip(r_a["v_cmd"], r_b["v_cmd"]))
            and abs(vn - cfgB.V_KAPANMA) / cfgB.V_KAPANMA < 0.01,
            f"|v|={vn:.2f} (V_KAPANMA={cfgB.V_KAPANMA})")

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
