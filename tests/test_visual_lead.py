"""
tests/test_visual_lead.py — IBVS lead pursuit kabul kriterleri (T1-T21).

Gazebo'dan ÖNCE geçmeli. Sentetik üreteç (master spec):
  a = fx * GOVDE_BOYU_M / R * sin(aspect)
  b = fx * KANAT_ACIKLIGI_M / R * cos(aspect)

Kullanım: python3 -m tests.test_visual_lead
"""

import math
import os
import tempfile

import numpy as np

# Güdüm CSV'lerini geçici dizine yönlendir — testler logs/ altına sahte uçuş
# dosyası bırakmasın (analiz scriptleri onları gerçek uçuş sanardı).
# _LOG_DIR import anında okunduğu için bu satır importlardan ÖNCE olmalı.
os.environ.setdefault("AVCI_LEAD_LOG_DIR", tempfile.mkdtemp(prefix="avci_test_lead_"))

from control.guidance.adapter_copter import CopterAdapter
from control.guidance.guidance_core import (
    GOVDE_BOYU_M, KANAT_ACIKLIGI_M, LeadPursuitCore, cfg_copy,
    govde_to_dunya, yukselti_duzeltme)
from vision import geometry as geo

FX, FY, CX, CY = geo.FX, geo.FY, geo.CX, geo.CY

# Kamerayı yatay yapan attitude (tilt 25° yukarı → pitch -25° = eps 0, merkez hedef)
ATT_KAMERA_YATAY = (0.0, math.radians(-25.0), 0.0)

_sonuclar = []


def cfg_sabit_montaj():
    """Gimbal'sız (kamera gövdeye çakılı) Cfg kopyası.

    Sentetik sahne üreten testler gövde attitude'unu kamera attitude'u kabul
    eder — bu, sabit montajın geometrisidir. Gimbal ikisini ayırdığı için o
    testler bu kopyayı kullanır; gimbal'ın KENDİ davranışı ayrıca sınanır
    (T44-T46, ve test_gps_guidance G5/G5b)."""
    c = cfg_copy()
    c.GIMBAL_AKTIF = False
    return c


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
    # cfg_sabit: sentetik sahne gövde attitude'unu KAMERA attitude'u sayar,
    # yani sabit montaj geometrisidir. Gimbal ikisini ayırdığından bu testte
    # kapatılır (ölçülen şey montajdan bağımsız: ölçek → menzil dönüşümü).
    hatalar = []
    for R in (5, 8, 12, 15):
        r = tek_kare(cfg_sabit_montaj(), make_pose(R, 60))
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
    r = tek_kare(cfg_sabit_montaj(), pose_alttan, att=att_dik)   # bkz. T9 notu
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
    # 2026-08-04: nişanın dikey bileşeni KÜÇÜLTÜLDÜ (−0.42 → −0.20). Sebep,
    # testin zayıflatılması değil: artık dikey hız VZ_TAVAN ile kırpılıyor
    # (Cfg.VZ_TAVAN, çöküş gerekçesi orada). −0.42 ile vz = 25·0.42 = 10.5 m/s
    # olur, yani 6 m/s tavanın üstünde — o durumda |v| = V_KAPANMA olmaması
    # DOĞRU davranıştır ve ayrıca T42/T43'te sınanır. Bu test, kırpmanın
    # DEVREDE OLMADIĞI zarf içinde adaptörün tam kapanma hızını verdiğini
    # doğrular: 25·0.20 = 5.0 m/s < 6.
    ad = CopterAdapter(cfg)
    u_g = np.array([0.9, 0.1, -0.20]); u_g = u_g / np.linalg.norm(u_g)
    out = None
    for i in range(200):                      # rampa otursun
        out = ad.compute(u_g, 0.0, (0, 0, 0), 1.0 / 30.0, 0.0)
    v = np.array(out["v_cmd"]); vn = np.linalg.norm(v)
    yon = float(np.dot(v / vn, out["u_dunya"]))
    kontrol("T18 |v|=V_KAPANMA ve yön=u_dunya",
            abs(vn - cfg.V_KAPANMA) / cfg.V_KAPANMA < 0.01 and yon > 0.9999,
            f"|v|={vn:.3f} m/s yön·u_dunya={yon:.6f}")

    # ── T20: ivme rampası — hız sıçramasında uygulanan ivme ≤ IVME_TAVAN ──
    # Not: varsayılan IVME_TAVAN "limitsiz test" için çok yükseğe çekildi;
    # rampa MEKANİZMASINI test etmek için burada açıkça sonlu bir değere sabitle.
    cfg = cfg_copy(); cfg.IVME_TAVAN = 4.0
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

    def fake_visual(conn, wait_pose, gpt, stop_event, cfg=None,
                    kayip_kare_esik=None, gercek=None):
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

    def fake_visual_vurus(conn, wp, gpt, stop_event, cfg=None,
                          kayip_kare_esik=None, gercek=None):
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
    cfg = cfg_copy()
    ad = CopterAdapter(cfg)
    for e_deg in (20.0, 20.5, 21.0):                  # sakin seyir → yumuşatma otursun
        e = math.radians(e_deg)
        ad.compute(np.array([math.cos(e), 0.0, -math.sin(e)]), 0.0, (0, 0, 0), dt, 0.0)
    e = math.radians(70.0)                            # ANİ ~49° sıçrama (bimodal gürültü)
    out = ad.compute(np.array([math.cos(e), 0.0, -math.sin(e)]), 0.0, (0, 0, 0), dt, 0.0)
    cikis_elev = math.degrees(math.asin(max(-1.0, min(1.0, -float(out["u_dunya"][2])))))
    vn = np.linalg.norm(np.array(out["v_cmd"]))
    # |v| = V_KAPANMA koşulu YATAY bileşene taşındı: bu testin konusu dikey aim
    # yumuşatma, ve 39.5°'lik yükselişte dikey bileşen (25·sin39.5 = 15.9 m/s)
    # artık VZ_TAVAN ile kırpılıyor — bu kasıtlı (bkz. Cfg.VZ_TAVAN). Kırpmanın
    # YATAY kapanmayı bozmadığını doğrulamak, testin asıl güvencesidir.
    v_yatay = math.hypot(out["v_cmd"][0], out["v_cmd"][1])
    yatay_bekl = cfg.V_KAPANMA * math.cos(math.radians(cikis_elev))
    kontrol("T29 dikey aim yumuşatma sıçramayı kırpar",
            cikis_elev < 45.0 and abs(out["pn_dikey_deg"]) <= cfg.PN_DIKEY_MAX_DEG + 1e-6
            and abs(v_yatay - yatay_bekl) / yatay_bekl < 0.01,
            f"ham=70° komut_yükseliş={cikis_elev:.2f}° pn={out['pn_dikey_deg']:.2f}° "
            f"|v_yatay|={v_yatay:.2f} (beklenen {yatay_bekl:.2f})")

    # ══ T30-T33: DOĞRULUK REFERANSI (vision/dogruluk.py) ══
    # Bu testler ANALİZ ZEMİNİNİ doğrular: MÜKEMMEL bir pose modeli taklit edilip
    # (gerçek keypoint projeksiyonu aynen pose olarak verilir) ölçüm sıfır hata
    # vermeli. Vermezse uçuş verisinde göreceğimiz her sapma referansın kendi
    # hatasıyla karışır ve analiz anlamını yitirir.
    from vision import dogruluk

    def sentetik_sahne(menzil_m, hedef_yaw):
        """iris orijinde ve düz; hedef optik eksende `menzil_m` ötede.
        Dönüş: (pose, iris_truth, hedef_truth)."""
        iris_pos, iris_rpy = (0.0, 0.0, 10.0), (0.0, 0.0, 0.0)
        cam_pos, R_cam = geo.camera_world_pose(iris_pos, iris_rpy)
        tgt = cam_pos + menzil_m * (R_cam @ np.array([1.0, 0.0, 0.0]))
        rpy = (0.0, 0.0, hedef_yaw)
        kg, _onde, _kad, _ort = dogruluk._gercek_kpts(tgt, rpy, iris_pos, iris_rpy)
        bb = geo.target_bbox(tgt, rpy, iris_pos, iris_rpy)
        pose = {"cx": (bb[0] + bb[2]) / 2, "cy": (bb[1] + bb[3]) / 2, "conf": 0.9,
                "bbox": tuple(int(v) for v in bb),
                "kpts": [(kg[i, 0], kg[i, 1], 1.0) for i in range(6)]}
        it = {"x": iris_pos[0], "y": iris_pos[1], "z": iris_pos[2],
              "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "stamp": 1.0}
        ht = {"x": tgt[0], "y": tgt[1], "z": tgt[2],
              "roll": 0.0, "pitch": 0.0, "yaw": hedef_yaw, "stamp": 1.0}
        return pose, it, ht

    # T30: mükemmel model → keypoint hatası sıfır (boru hattı doğru bağlanmış)
    p, it, ht = sentetik_sahne(20.0, math.pi / 2)
    r = dogruluk.olc(p, it, ht)
    kontrol("T30 mükemmel model → kpt hatası ~0",
            r["kpt_hata_px_ort"] < 0.01 and r["kpt_hata_px_max"] < 0.01,
            f"ort={r['kpt_hata_px_ort']} max={r['kpt_hata_px_max']}")

    # T31: ölçekten menzil, GERÇEK menzile %2 içinde — yükselti düzeltmesi şart.
    # Düzeltme uygulanmazsa 25° bakışta ~%8 kısa okunur (2026-07-27 ölçümü).
    sapmalar = []
    for R in (10.0, 20.0, 30.0):
        for yaw in (0.0, math.pi / 4, math.pi / 2, math.pi):
            p, it, ht = sentetik_sahne(R, yaw)
            rr = dogruluk.olc(p, it, ht)
            sapmalar.append(abs(rr["menzil_olcek_gercek_m"] - R) / R)
    kontrol("T31 ölçek→menzil sapması < %2 (her açı/menzil)",
            max(sapmalar) < 0.02, f"maks sapma=%{100 * max(sapmalar):.1f}")

    # T32: yandanlik_gercek ≈ |sin(aspect)| — guidance_core lead büyüklüğünü
    # bu ilişkiye dayandırıyor; bozulursa lead yasasının varsayımı çöker.
    farklar = []
    for yaw in (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4, math.pi):
        p, it, ht = sentetik_sahne(15.0, yaw)
        rr = dogruluk.olc(p, it, ht)
        farklar.append(abs(rr["yandanlik_gercek"] - rr["sin_aspect_gercek"]))
    kontrol("T32 yandanlik_gercek ≈ sin(aspect)",
            max(farklar) < 0.06, f"maks fark={max(farklar):.3f}")

    # T33: burun/kuyruk takas tespiti — kasıtlı takas edilmiş pose yakalanmalı.
    # Bu kolon guidance_core'un flip korumasının GÖREMEDİĞİ kalıcı yanlış
    # etiketlemeyi ölçer; çalışmazsa 180° belirsizliği sessiz kalır.
    p, it, ht = sentetik_sahne(15.0, math.pi / 2)
    kpts_takas = list(p["kpts"])
    kpts_takas[0], kpts_takas[1] = kpts_takas[1], kpts_takas[0]
    p_takas = dict(p, kpts=kpts_takas)
    r_norm = dogruluk.olc(p, it, ht)
    r_takas = dogruluk.olc(p_takas, it, ht)
    kontrol("T33 burun/kuyruk takası tespit edilir",
            r_norm["burun_kuyruk_takas"] == 0 and r_takas["burun_kuyruk_takas"] == 1,
            f"normal={r_norm['burun_kuyruk_takas']} takas={r_takas['burun_kuyruk_takas']}")

    # ══════════════════════════════════════════════════════════
    #  T34-T37  LOS KAPISI (adapter_copter.kapanma_hizi)
    # ══════════════════════════════════════════════════════════
    cfgL = cfg_copy()
    adL = CopterAdapter(cfgL)

    # T34 KUYRUK takibi: yandanlik≈0, lead≈0 → LOS neredeyse dönmüyor, kapı
    # AÇILMALI (tam V_KAPANMA). Stash'teki ilk sürüm burada da frenliyordu.
    v_kuyruk = adL.kapanma_hizi(10.0, 0.0, 0.0)
    kontrol("T34 kuyruktan takipte LOS kapısı kısıtlamaz (tam V_KAPANMA)",
            abs(v_kuyruk - cfgL.V_KAPANMA) < 1e-9,
            f"v={v_kuyruk:.2f} (V_KAPANMA={cfgL.V_KAPANMA})")

    # T35 YANDAN geçiş, 8 m: 2026-08-01 uçuşunun tam geometrisi
    # (yandanlik_f≈0.95, lead≈23°, menzil_kestirim≈8 m). Kapı BAĞLAMALI.
    lead23 = math.radians(23.0)
    v_yan = adL.kapanma_hizi(8.0, 0.95, lead23)
    kontrol("T35 yandan geçişte (8 m, yandanlık 0.95) kapı frenler",
            v_yan < cfgL.V_KAPANMA and v_yan >= cfgL.V_KAPANMA_MIN,
            f"v={v_yan:.2f} < {cfgL.V_KAPANMA} ve ≥ taban {cfgL.V_KAPANMA_MIN}")

    # T36 ASIL AMAÇ: kapıdan çıkan hızla LOS açısal hızı, aracın izleyebildiği
    # bandın İÇİNDE kalmalı. Kapı olmadan aynı geometride 155 °/s çıkıyordu
    # (ölçüm: hedef 0.30 s'de kadrajı terk etti).
    def omega_los(v, menzil, yandanlik, lead):
        return math.degrees(v * (cfgL.K_LEAD * yandanlik + math.sin(lead)) / menzil)
    w_kapili = omega_los(v_yan, 8.0, 0.95, lead23)
    w_kapisiz = omega_los(cfgL.V_KAPANMA, 8.0, 0.95, lead23)
    kontrol("T36 kapılı LOS hızı izlenebilir bandın içinde, kapısız değil",
            w_kapili <= cfgL.LOS_YAW_IZLENEBILIR and w_kapisiz > cfgL.LOS_YAW_IZLENEBILIR,
            f"kapılı={w_kapili:.0f}°/s, kapısız={w_kapisiz:.0f}°/s, "
            f"araç {cfgL.LOS_YAW_IZLENEBILIR:.0f}°/s")

    # T37 menzil kestirimi YOKSA kapı uygulanmaz (yanlış sıfır yüzünden aracı
    # durdurmak, hızlı gitmekten kötü) — ve kapatma anahtarı çalışır.
    cfgOff = cfg_copy(); cfgOff.LOS_KAPISI = False
    v_none = adL.kapanma_hizi(None, 1.0, lead23)
    v_off = CopterAdapter(cfgOff).kapanma_hizi(8.0, 0.95, lead23)
    kontrol("T37 menzil None → kapı kapalı; LOS_KAPISI=False → kapı kapalı",
            abs(v_none - cfgL.V_KAPANMA) < 1e-9 and abs(v_off - cfgOff.V_KAPANMA) < 1e-9,
            f"none={v_none:.1f} off={v_off:.1f}")

    # ══════════════════════════════════════════════════════════
    #  T38-T40  HAREKET TUTARLILIĞI (kalıcı burun/kuyruk takası)
    # ══════════════════════════════════════════════════════════
    # T38 Model ısrarla TERS etiketliyor ama hedef kadrajda +x yönünde kayıyor.
    # Uçak burnu ileri uçtuğuna göre eksen +x olmalı; çekirdek bunu düzeltmeli.
    core_t = LeadPursuitCore(cfg_copy())
    d_son, warn_son = None, []
    for i in range(8):
        p = make_pose(12.0, 90.0, cx=CX - 40 + 10.0 * i, cy=CY,
                      d_aci_deg=0.0, swap=True)       # etiketler TERS
        r = core_t.process(p, i / 30.0, ATT_KAMERA_YATAY)
        d_son, warn_son = r["d_birim"], r["warn"]
    kontrol("T38 kalıcı takas: akış tutarlılığı ekseni hareket yönüne çevirir",
            float(d_son[0]) > 0.9 and core_t.takas_sayaci > 0,
            f"d_birim=({d_son[0]:+.2f},{d_son[1]:+.2f}) hareket=+x, "
            f"takas_sayaci={core_t.takas_sayaci}, warn={warn_son}")

    # T39 Etiketler DOĞRUYKEN kapı asla devreye girmemeli (yanlış pozitif yok).
    core_ok = LeadPursuitCore(cfg_copy())
    for i in range(8):
        p = make_pose(12.0, 90.0, cx=CX - 40 + 10.0 * i, cy=CY, d_aci_deg=0.0)
        r_ok = core_ok.process(p, i / 30.0, ATT_KAMERA_YATAY)
    kontrol("T39 doğru etiketlemede takas kapısı susar (yanlış pozitif yok)",
            core_ok.takas_sayaci == 0 and float(r_ok["d_birim"][0]) > 0.9,
            f"takas_sayaci={core_ok.takas_sayaci} akis_skor={core_ok.akis_skor:+.2f}")

    # T40 KUYRUK TAKİBİ: hedef kadrajda neredeyse durgun (akış eşiğin altında)
    # → oy verilmez, skor 0'da kalır, hiçbir şey çevrilmez. Kapının kendi
    # kendine bozmadığının garantisi.
    core_dur = LeadPursuitCore(cfg_copy())
    for i in range(10):
        p = make_pose(12.0, 20.0, cx=CX + 0.02 * i, cy=CY, d_aci_deg=0.0)
        core_dur.process(p, i / 30.0, ATT_KAMERA_YATAY)
    kontrol("T40 durgun kadrajda (kuyruk takibi) kapı oy vermez",
            core_dur.takas_sayaci == 0 and abs(core_dur.akis_skor) < 1e-9,
            f"takas_sayaci={core_dur.takas_sayaci} akis_skor={core_dur.akis_skor:+.3f}")

    # T41 yandanlık fiziksel sınırı: yükselti düzeltmesi aşırı telafi etse bile
    # a/olcek 1'i aşamaz (ölçüldü: 1.0269 ve 1.21). Aşarsa lead şişer ve LOS
    # kapısının paydası fizik dışı büyür.
    core_y = LeadPursuitCore(cfg_copy())
    p_asiri = make_pose(12.0, 90.0, a_ovr=60.0, b_ovr=0.0)   # kanat yok, gövde uzun
    r_y = core_y.process(p_asiri, 0.0, (0.0, math.radians(-70.0), 0.0))
    kontrol("T41 yandanlık ≤ 1.0 (fiziksel sınır dayatılır)",
            r_y["yandanlik_ham"] <= 1.0 + 1e-12,
            f"yandanlik_ham={r_y['yandanlik_ham']:.4f} "
            f"(a/olcek ham={r_y['a']/r_y['olcek']:.4f})")

    # ══════════════════════════════════════════════════════════
    #  T42-T43  DİKEY HIZ TAVANI (2026-08-04 çöküşü)
    # ══════════════════════════════════════════════════════════
    # T42 Dikliğine yukarı nişanda vz komutu VZ_TAVAN'ı aşmamalı. Kırpma
    # OLMADAN 25 m/s'lik kapanma hızı doğrudan 25 m/s tırmanış komutu olur;
    # ölçülen çöküşte vz_cmd −24.3 m/s'ye çıkmıştı.
    cfgV = cfg_copy()
    adV = CopterAdapter(cfgV)
    # gövde-FRD'de dik yukarı: (0, 0, -1). Attitude seviyeli → dünya NED'de de yukarı.
    outV = adV.compute([0.0, 0.0, -1.0], 0.0, (0.0, 0.0, 0.0), 1.0 / 30.0, 0.0)
    kontrol("T42 dikey hız komutu VZ_TAVAN ile kırpılır",
            abs(outV["v_cmd"][2]) <= cfgV.VZ_TAVAN + 1e-9,
            f"vz_cmd={outV['v_cmd'][2]:+.2f} m/s, tavan ±{cfgV.VZ_TAVAN} "
            f"(kırpmasız {-cfgV.V_KAPANMA:+.1f} olurdu)")

    # T43 Kırpma YATAY bileşeni bozmamalı — nişan düzleşir, yavaşlamaz.
    # 45° yukarı nişan: yatay ve dikey bileşen eşit (25/√2 = 17.7 m/s).
    adV2 = CopterAdapter(cfg_copy())
    s2 = math.sqrt(0.5)
    outH = adV2.compute([s2, 0.0, -s2], 0.0, (0.0, 0.0, 0.0), 1.0 / 30.0, 0.0)
    yatay_bekl = cfgV.V_KAPANMA * s2
    kontrol("T43 dikey kırpma yatay kapanmayı azaltmaz",
            abs(outH["v_cmd"][0] - yatay_bekl) < 0.1
            and abs(outH["v_cmd"][2]) <= cfgV.VZ_TAVAN + 1e-9,
            f"vx={outH['v_cmd'][0]:.2f} (beklenen {yatay_bekl:.2f}), "
            f"vz={outH['v_cmd'][2]:+.2f}")

    # ══════════════════════════════════════════════════════════
    #  T44-T46  GIMBAL: KAMERA DÜNYA POZU (bağımsız değişmez)
    # ══════════════════════════════════════════════════════════
    # NEDEN AYRI BİR TEST: T30 ("mükemmel model → kpt hatası ~0") sahneyi
    # camera_world_pose ile KURUP yine onunla ÖLÇÜYOR. Dönüşüm yanlış olsa
    # bile hata iki tarafta da aynı olduğu için sıfırlanır — nitekim
    # camera_world_pose gimbal için hatalı yazıldığında T30 0.0 px verirken
    # UÇUŞTA 164 px ölçüldü. Kendi kendine referanslı test bunu göremez.
    # Bu üç test BAĞIMSIZ bir değişmezi sınar: gimbal varken kameranın DÜNYA
    # çerçevesindeki roll'ü 0, pitch'i CAM_TILT olmalıdır — gövde ne yaparsa
    # yapsın. Ölçüt dönüşümün kendisinden türetilmiyor.
    from vision import geometry as _g

    def _cam_dunya_rpy(roll, pitch, yaw):
        _pos, R = _g.camera_world_pose([0, 0, 50], (roll, pitch, yaw))
        # R'den (roll, pitch) çıkar — Gazebo X-ileri/Y-sol/Z-yukarı çerçevesi
        p = math.asin(max(-1.0, min(1.0, -float(R[2, 0]))))
        rr = math.atan2(float(R[2, 1]), float(R[2, 2]))
        return math.degrees(rr), math.degrees(p)

    T = math.degrees(-_g.CAM_TILT_RAD)          # +25° (yukarı, pozitif okunur)
    sapmalar = []
    for rb, pb in [(0, 0), (30, 0), (-45, 0), (0, 30), (0, -40), (40, 35), (-50, -30)]:
        rc, pc = _cam_dunya_rpy(math.radians(rb), math.radians(pb), math.radians(20))
        sapmalar.append((rb, pb, rc, -pc))
    en_kotu_roll = max(abs(s[2]) for s in sapmalar)
    en_kotu_pitch = max(abs(s[3] - T) for s in sapmalar)
    kontrol("T44 gimbal: kamera dünya ROLL'ü gövdeden bağımsız ≈ 0",
            en_kotu_roll < 0.5,
            f"en kötü |roll_dünya|={en_kotu_roll:.2f}° "
            f"(gövde roll ±50°, pitch ±40° tarandı)")
    kontrol("T45 gimbal: kamera dünya PITCH'i gövdeden bağımsız ≈ CAM_TILT",
            en_kotu_pitch < 0.5,
            f"en kötü sapma={en_kotu_pitch:.2f}° (hedef {T:.0f}°)")

    # T46 gimbal KAPALIYKEN kamera gövdeyi izlemeli — eski davranış korunuyor mu
    _eski = _g._gimbal_aktif
    _g._gimbal_aktif = lambda: False
    try:
        rc, pc = _cam_dunya_rpy(0.0, math.radians(30), 0.0)
    finally:
        _g._gimbal_aktif = _eski
    kontrol("T46 gimbal kapalı → kamera gövde pitch'ini İZLER (eski davranış)",
            abs(-pc - (T - 30)) < 0.5,
            f"gövde pitch +30° → kamera dünya pitch {-pc:+.1f}° "
            f"(beklenen {T-30:+.0f}°)")

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
