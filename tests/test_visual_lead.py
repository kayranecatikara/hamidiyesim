"""
tests/test_visual_lead.py — BBOX IBVS kabul kriterleri.

Gazebo'dan ÖNCE geçmeli. Sentetik üreteç:
  w_px = fx * BBOX_L_ETKIN_M / R        (kutu genişliği; ölçek sinyali)

── 2026-08-06: POSE ÇIKARILDI ──
T1-T21 bloğu yeniden yazıldı: keypoint/yandanlık/şekil-lead testlerinin yerini
bbox ölçeği ve AZİMUT-ORANI lead testleri aldı. T22+ numaraları korundu
(davranışları değişmedi, yalnız girdi üreteci pose→det oldu).
Sökülen pose testleri: POSEA_GERI_DONMEK_ISTERSENIZ/gudum_anlik_goruntu/

Kullanım: python3 -m tests.test_visual_lead
"""

import math

import numpy as np

from control.guidance.adapter_copter import CopterAdapter
from control.guidance.guidance_core import (
    Cfg, LeadPursuitCore, cfg_copy,
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


# Ölçülen kutu en-boy oranı (08-06, n=1917 `ok` karesi): medyan 1.95
_EN_BOY = 1.95


def make_det(R, cx=CX, cy=CY, conf=1.0, w_ovr=None):
    """Sentetik detection: kutu merkezi (cx,cy), genişliği w = fx·L_ETKIN/R."""
    w = w_ovr if w_ovr is not None else FX * Cfg.BBOX_L_ETKIN_M / R
    h = w / _EN_BOY
    return {"cx": cx, "cy": cy, "w": w, "h": h, "conf": conf,
            "bbox": (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)}


def tek_kare(cfg, det, att=ATT_KAMERA_YATAY, stamp=0.0):
    return LeadPursuitCore(cfg).process(det, stamp, att)


def _az_kosusu(cfg, az_dizisi_deg, elev_deg=20.0, dt=1.0 / 30.0, kalite=1.0):
    """Adaptörü verilen DÜNYA azimut dizisiyle sür; son karenin çıktısını döndür.
    u_govde attitude=(0,0,0) ile doğrudan dünya azimut/yükselişine karşılık gelir."""
    ad = CopterAdapter(cfg)
    e = math.radians(elev_deg)
    out = None
    for az_deg in az_dizisi_deg:
        a = math.radians(az_deg)
        ug = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a),
                       -math.sin(e)])
        out = ad.compute(ug, 0.0, (0, 0, 0), dt, 0.0, kalite=kalite)
    return ad, out


def main():
    print("BBOX IBVS kabul kriterleri")
    print("=" * 60)

    # ══ T1-T4: BBOX ÖLÇEĞİ (pose keypoint ölçeğinin halefi) ══

    # ── T1: kutu genişliğinden menzil kestirimi (SADECE LOG) ──
    hatalar = [abs(tek_kare(cfg_copy(), make_det(R))["menzil_kestirim_m"] - R) / R * 100
               for R in (5, 8, 12, 20)]
    kontrol("T1  bbox ölçeğinden menzil kestirimi <%1 (SADECE LOG)",
            max(hatalar) < 1.0, f"max hata=%{max(hatalar):.4f}")

    # ── T2: kalite rampası menzille — uzakta 0, yakında 1 ──
    r_uzak = tek_kare(cfg_copy(), make_det(30))
    r_yakin = tek_kare(cfg_copy(), make_det(8))
    kontrol("T2  kalite rampası: uzakta 0, yakında 1",
            r_uzak["kalite"] == 0.0 and r_yakin["kalite"] == 1.0,
            f"R=30m → olcek={r_uzak['olcek']:.1f}px kalite={r_uzak['kalite']} · "
            f"R=8m → olcek={r_yakin['olcek']:.1f}px kalite={r_yakin['kalite']}")

    # ── T3: çekirdek SAF TAKİP — u_nisan bakış yönüyle BİREBİR aynı ──
    # Lead artık burada değil adaptörde üretiliyor (bkz. T5-T8). Çekirdeğin
    # nişanı kaydırmadığı garanti altına alınmalı, yoksa lead iki kez binerdi.
    r = tek_kare(cfg_copy(), make_det(8, cx=450, cy=300))
    fark = float(np.max(np.abs(np.asarray(r["u"]) - np.asarray(r["u_nisan"]))))
    kontrol("T3  çekirdek saf takip (u_nisan ≡ u, lead adaptörde)",
            fark == 0.0, f"bileşen bazında en büyük fark={fark:.2e}")

    # ── T4: kutu MIN_BBOX_PX altında → ölçek güvenilmez, kalite susar ──
    r = tek_kare(cfg_copy(), make_det(8, w_ovr=1.0))
    kontrol("T4  çok küçük kutu → durum=kutu_kucuk, kalite=0",
            r["durum"] == "kutu_kucuk" and r["kalite"] == 0.0
            and "kutu_kucuk" in r["warn"],
            f"durum={r['durum']} kalite={r['kalite']}")

    # ══ T5-T8: AZİMUT-ORANI LEAD (pose şekil-lead'inin halefi) ══

    # ── T5: azimut SABİT → lead yok (saf takip) ──
    _, out = _az_kosusu(cfg_copy(), [0.0] * 40)
    kontrol("T5  sabit azimutta yatay lead = 0",
            abs(out["yatay_lead_deg"]) < 1e-9 and abs(out["az_rate_dps"]) < 1e-6,
            f"lead={out['yatay_lead_deg']:.6f}° oran={out['az_rate_dps']:.6f}°/s")

    # ── T6: azimut DÖNÜYOR → lead aynı yönde, tavanla sınırlı ──
    # Hedef sağa geçiyorsa (azimut artıyor) nişan de sağa öne kaymalı.
    cfg = cfg_copy()
    _, out_sag = _az_kosusu(cfg, [i * 1.0 for i in range(60)])    # +30 °/s
    _, out_sol = _az_kosusu(cfg, [-i * 1.0 for i in range(60)])   # -30 °/s
    _, out_hizli = _az_kosusu(cfg, [i * 8.0 for i in range(60)])  # +240 °/s
    kontrol("T6  azimut oranıyla orantılı lead (işaret + tavan)",
            out_sag["yatay_lead_deg"] > 1.0 and out_sol["yatay_lead_deg"] < -1.0
            and abs(out_hizli["yatay_lead_deg"]) <= cfg.PN_YATAY_MAX_DEG + 1e-6,
            f"+30°/s → {out_sag['yatay_lead_deg']:+.2f}° · "
            f"−30°/s → {out_sol['yatay_lead_deg']:+.2f}° · "
            f"+240°/s → {out_hizli['yatay_lead_deg']:+.2f}° "
            f"(tavan {cfg.PN_YATAY_MAX_DEG}°)")

    # ── T7: GERİ DÖNÜŞ YOLU — PN_YATAY_SURE=0 → birebir saf takip ──
    cfg0 = cfg_copy(); cfg0.PN_YATAY_SURE = 0.0
    _, out0 = _az_kosusu(cfg0, [i * 2.0 for i in range(40)])
    kontrol("T7  PN_YATAY_SURE=0 → yatay lead tamamen kapanır",
            out0["yatay_lead_deg"] == 0.0,
            f"lead={out0['yatay_lead_deg']}° (tek env ile saf takibe dönüş)")

    # ── T8: AZİMUT DAİRESEL — ±180° geçişi sahte oran üretmemeli ──
    # normalize_angle olmasaydı tek karede 360°/dt ≈ 10800 °/s oran çıkar ve
    # lead anında tavana çakılırdı (ters yönde!).
    cfg = cfg_copy()
    _, out_sar = _az_kosusu(cfg, [176.0, 178.0, 180.0, -178.0, -176.0, -174.0])
    kontrol("T8  ±180° sarmalamada sahte azimut oranı yok",
            out_sar["yatay_lead_deg"] > 0.0
            and abs(out_sar["az_rate_dps"]) < 200.0,
            f"oran={out_sar['az_rate_dps']:+.1f}°/s lead={out_sar['yatay_lead_deg']:+.2f}° "
            f"(sarmalamasız ~10800 °/s olurdu)")

    # ── T9: kadraj merkezi → pitch_hata=+25.00, yaw_hata=0.00 (tilt telafisi) ──
    r = tek_kare(cfg_copy(), make_det(8), att=(0.0, 0.0, 0.0))
    kontrol("T9  tilt telafisi (merkez → +25°)",
            abs(r["pitch_hata_deg"] - 25.0) < 0.01 and abs(r["yaw_hata_deg"]) < 0.01,
            f"pitch={r['pitch_hata_deg']:.2f}° yaw={r['yaw_hata_deg']:.2f}°")

    # ── T10: görüş zarfı ──
    ust = tek_kare(cfg_copy(), make_det(8, cy=0), att=(0, 0, 0))
    alt = tek_kare(cfg_copy(), make_det(8, cy=480), att=(0, 0, 0))
    sag = tek_kare(cfg_copy(), make_det(8, cx=640), att=(0, 0, 0))
    kontrol("T10 görüş zarfı üst kenar +80.2°",
            abs(ust["pitch_hata_deg"] - 80.2) < 0.1, f"{ust['pitch_hata_deg']:.2f}°")
    kontrol("T10 görüş zarfı alt kenar -30.1°",
            abs(alt["pitch_hata_deg"] + 30.1) < 0.2, f"{alt['pitch_hata_deg']:.2f}°")
    kontrol("T10 görüş zarfı sağ kenar yaw +64.7°",
            abs(sag["yaw_hata_deg"] - 64.7) < 0.1, f"{sag['yaw_hata_deg']:.2f}°")

    # ── T11: KAMERA_TILT_DEG=0 → merkez pitch_hata=0 (tilt kapsüllemesi) ──
    cfg = cfg_copy(); cfg.KAMERA_TILT_DEG = 0.0
    r = tek_kare(cfg, make_det(8), att=(0.0, 0.0, 0.0))
    kontrol("T11 tilt=0 kapsülleme", abs(r["pitch_hata_deg"]) < 0.01,
            f"pitch={r['pitch_hata_deg']:.3f}°")

    # ── T12: yükselti düzeltme katsayıları ──
    beklenen = [(25.0, 1.086), (38.7, 1.179), (56.4, 1.302)]
    ok = all(abs(yukselti_duzeltme(math.radians(e)) - d) < 0.001 for e, d in beklenen)
    kontrol("T12 düzeltme katsayıları", ok,
            f"{[(e, round(yukselti_duzeltme(math.radians(e)), 3)) for e, d in beklenen]}")

    # ── T13: yükselti düzeltmesi ölçeği KÜÇÜLTÜR (perspektif kısalması telafisi) ──
    # Kamera dik yukarı bakarken (eps=90°) aynı kutu daha uzak sayılmalı.
    att_dik = (0.0, math.radians(65.0), 0.0)
    d13 = make_det(8)
    r_duz = tek_kare(cfg_copy(), d13, att=att_dik)
    cfg = cfg_copy(); cfg.YUKSELTI_DUZELT = False
    r_ham = tek_kare(cfg, d13, att=att_dik)
    kontrol("T13 yükselti düzeltmesi ölçeği kısaltma payıyla düzeltir",
            abs(r_duz["eps_deg"] - 90.0) < 0.1
            and abs(r_duz["duzeltme"] - math.sqrt(2.0)) < 0.01
            and r_duz["olcek"] < r_ham["olcek"],
            f"eps={r_duz['eps_deg']:.1f}° duzeltme={r_duz['duzeltme']:.3f} "
            f"olcek {r_ham['olcek']:.1f} → {r_duz['olcek']:.1f} px")

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
    # KP_KADRAJ=0: kadraj tutma düzeltmesi kapatılır, yoksa yatay nişan kadraj
    # merkezinden (25°) saptığı için DİKEY bileşen doğar ve ölçülen ivme yatay
    # tavanla kıyaslanamaz hâle gelir (bu test yatayı ölçüyor).
    cfg = cfg_copy(); cfg.IVME_TAVAN = 4.0; cfg.KP_KADRAJ = 0.0
    ad = CopterAdapter(cfg)                   # v_onceki = 0
    dt = 1.0 / 30.0
    out = ad.compute(np.array([1.0, 0.0, 0.0]), 0.0, (0, 0, 0), dt, 0.0)
    ivme = np.linalg.norm(out["v_cmd"]) / dt
    kontrol("T20 ivme tavanı", ivme <= cfg.IVME_TAVAN * (1 + 1e-9),
            f"uygulanan={ivme:.2f} m/s² tavan={cfg.IVME_TAVAN}")

    # ── T22: FSM-güdümlü dispatch zinciri GPS→VISUAL→(kayıp)→GPS→VISUAL→durdur ──
    # Supervisor artık devir kapısı değil, görev FSM durumunu (get_gorev_state)
    # okuyor. Test FSM durumunu set_gorev_durum ile sürer: SEARCH→GPS, DETECT→
    # görsel. fake_gps "hedef edinildi"yi (FSM DETECT) simüle eder.
    import threading
    import time as _t
    import control.guidance.supervisor as sup
    from control.mission_fsm import State as _St, FSMDurum as _FD
    from vision.detection_state import set_gorev_durum as _setst
    olaylar = []
    _orij_gps, _orij_vis = sup.run_gps_guidance, sup.run_visual_lead

    def fake_gps(conn, gp, gi, stop_event):
        olaylar.append("gps")
        _setst(_FD(_St.ENGAGE))       # kilit tamam → FSM görsel kümeye (ENGAGE)
        stop_event.wait(5.0)          # gps_izci ENGAGE'i görüp faz_stop'u kırar

    def fake_visual(conn, wait_kare, gpt, stop_event, cfg=None, kayip_kare_esik=None,
                    **kw):                                # get_temas/get_menzil/state uyumu
        olaylar.append("visual")
        if olaylar.count("visual") == 1:
            _setst(_FD(_St.TRACK_LOST))   # kilit kaybı → GPS kümesine dön
            return "kayip"
        return "durduruldu"

    def fake_wait(son_seq, timeout=0.5):
        _t.sleep(0.002)
        return None

    try:
        sup.run_gps_guidance, sup.run_visual_lead = fake_gps, fake_visual
        _setst(_FD(_St.SEARCH))               # başlangıç: GPS kümesi
        stop = threading.Event()
        th = threading.Thread(
            target=sup.run_hybrid,
            args=(None, None, None, fake_wait, None, stop), daemon=True)
        th.start()
        # İkinci görsele girince (gecis=2) görevi durdur → zincir kapansın.
        for _ in range(500):
            if olaylar.count("visual") >= 2:
                stop.set(); break
            _t.sleep(0.01)
        th.join(10.0)
        kontrol("T22 supervisor geçiş zinciri",
                olaylar[:4] == ["gps", "visual", "gps", "visual"]
                and sup.status["faz"] == "DURDU" and sup.status["gecis_sayisi"] == 2,
                f"olaylar={olaylar} faz={sup.status['faz']} "
                f"geçiş={sup.status['gecis_sayisi']}")
    finally:
        sup.run_gps_guidance, sup.run_visual_lead = _orij_gps, _orij_vis
        _setst(None)

    # ── T23: supervisor 'vuruldu' → görev biter, faz=VURULDU ──
    olaylar2 = []
    _og, _ov = sup.run_gps_guidance, sup.run_visual_lead

    def fake_gps2(conn, gp, gi, stop_event):
        olaylar2.append("gps")
        _setst(_FD(_St.ENGAGE)); stop_event.wait(5.0)

    def fake_visual_vurus(conn, wp, gpt, stop_event, cfg=None, kayip_kare_esik=None,
                          **kw):                          # get_temas/get_menzil/state uyumu
        olaylar2.append("visual"); return "vuruldu"

    try:
        sup.run_gps_guidance, sup.run_visual_lead = fake_gps2, fake_visual_vurus
        _setst(_FD(_St.SEARCH))
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
        _setst(None)

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

    def terminal_kosusu(cfg, menzil_dizisi, det_var_dizisi):
        """menzil_dizisi: her karede iris'in hedefe uzaklığı; det_var_dizisi:
        o karede tespit geldi mi (True) / gelmedi mi (False). get_plane_truth
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
            d = make_det(8) if det_var_dizisi[i] else None
            return {"seq": i + 1, "det": d, "stamp": (i + 1) / 30.0,
                    "wall_recv": _t.time()}
        gpt = lambda: {"x": 0.0, "y": 0.0, "z": 0.0}
        stop = threading.Event()
        return vlmod.run_visual_lead(conn, wp, gpt, stop, cfg=cfg,
                                     kayip_kare_esik=20)

    cfgT = cfg_copy()
    cfgT.TERMINAL_MENZIL = 8.0; cfgT.VURUS_MENZIL = 1.5; cfgT.TERMINAL_SURE = 2.0

    # T24: yaklaş (10→6, tespit var), sonra tespit KESİL ama menzil kapanmaya devam
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
    # yaklaş (tespit var) → tespit kesil → menzil canlıdaki gibi ZIPLASIN
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
    cfg = cfg_copy()
    def _kalite(ug):
        """u_govde'yi doğrudan hata-açısı geometrisinden geçir (kutu üretmeden)."""
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
    # (> AZİMUT_TEKIL 75°).
    r_dik = tek_kare(cfg_copy(), make_det(8, cy=0), att=ATT_KAMERA_YATAY)
    r_duz = tek_kare(cfg_copy(), make_det(8), att=ATT_KAMERA_YATAY)
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
    # Kritik risk: bbox zinciri (kamera→gövde) ile saf geometri (dünya→gövde)
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
            res = tek_kare(cfg, make_det(11.0, cx=t["u"] + cx_kaydir,
                                        cy=t["v"]), att=att0)
        _cevap_anahtari(satir, hedef, _Aras(drone, att0), res)
        return satir, t

    # hedef 10 m ileri, 4.65 m yukarıda → gerçek yükseliş ~24.9° (kadraj merkezi)
    s40, t40 = _anahtar((10.0, 0.0, -4.65))
    kontrol("T40 mükemmel kutuda cevap anahtarı sapması ≈ 0",
            abs(s40["bbox_yaw_sapma_deg"]) < 0.01
            and abs(s40["bbox_elev_sapma_deg"]) < 0.01,
            f"yaw sapma={s40['bbox_yaw_sapma_deg']:+.4f}° "
            f"elev sapma={s40['bbox_elev_sapma_deg']:+.4f}° "
            f"(gercek elev={s40['gercek_elev_deg']:.1f}°)")

    # bbox'ı 30 px SAĞA kaydır → gerçek bir azimut sapması görünmeli (sağ +)
    s41, _ = _anahtar((10.0, 0.0, -4.65), cx_kaydir=30.0)
    bekle41 = math.degrees(math.atan(30.0 / FX))
    kontrol("T41 kaydırılmış kutu sapma olarak ölçülür (sağ +)",
            s41["bbox_yaw_sapma_deg"] > 0
            and abs(s41["bbox_yaw_sapma_deg"] - bekle41) < 2.0,
            f"sapma={s41['bbox_yaw_sapma_deg']:+.2f}° (beklenen ~{bekle41:+.2f}°)")

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
            abs(dbayat) < kapisiz * 0.30,
            f"30 s'de {abs(dbayat):.0f}° ({abs(dbayat)/360:.2f} tur) — "
            f"kapısız {kapisiz:.0f}° ({kapisiz/360:.1f} tur) olurdu")

    # ── T45b: SUSMA KALICI OLMAMALI (2026-08-06 kilitlenme düzeltmesi) ──
    # Eski kapı bir KİLİTTİ: adım 0 → araç dönmez → hata kapanmaz → kapı hiç
    # açılmaz. ÖLÇÜLDÜ (124 346 GPS karesi): karelerin %7.8'inde burun >20°
    # sapmışken adım tam 0; en uzun kesintisiz susma 1867 kare = 93 SANİYE.
    # Sözleşme: susma en fazla YAW_SUS_N kare sürer, sonra yetki geri gelir.
    cfgS = cfg_copy()
    adS = CopterAdapter(cfgS)
    yhS = math.radians(60.0)            # hiç kapanmayan (bayat) hata
    uS = np.array([math.cos(yhS), math.sin(yhS), -math.tan(math.radians(25.0))])
    uS = uS / np.linalg.norm(uS)
    adimlarS = []
    for _ in range(cfgS.YAW_DOYGUN_N + cfgS.YAW_SUS_N + 10):
        adimlarS.append(abs(adS.compute(uS, yhS, (0, 0, 0), 1.0 / 30, 0.0)["yaw_adim_deg"]))
    _sustu = any(a < 1e-9 for a in adimlarS)
    _en_uzun_susma = _mevcut = 0
    for a in adimlarS:
        _mevcut = _mevcut + 1 if a < 1e-9 else 0
        _en_uzun_susma = max(_en_uzun_susma, _mevcut)
    _geri_geldi = adimlarS[-1] > 1e-9
    kontrol("T45b yaw susması SÜRELİ — kilitlenmez, yetki geri gelir",
            _sustu and _geri_geldi and _en_uzun_susma <= cfgS.YAW_SUS_N,
            f"en uzun susma {_en_uzun_susma} kare (tavan {cfgS.YAW_SUS_N}), "
            f"son adım {adimlarS[-1]:.2f}° — eski kodda sonsuza dek 0 kalırdı")

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
    # GT MODU (AVCI_GT_ROT=on) — T49-T54
    # Güdümün algı girdisi YOLO kutusu yerine Gazebo gerçek pozu. Bu testlerin
    # işi: GT yolunun bbox yoluyla AYNI yasayı çalıştırdığını göstermek.
    # ══════════════════════════════════════════════════════════
    IRIS_POS = (0.0, 0.0, 5.0)
    IRIS_RPY = (0.0, 0.0, 0.0)          # seviyeli drone → kamera 25° yukarı

    def gt_sahne(R, hedef_rpy, sapma=(0.0, 0.0, 0.0)):
        """Hedefi kameranın optik ekseninden R m ileriye koy (+ sapma m).
        Dönüş: (gt_sozluk, hedef_pos)."""
        cam_pos, R_cam = geo.camera_world_pose(IRIS_POS, IRIS_RPY)
        eksen = R_cam @ np.array([1.0, 0.0, 0.0])
        hedef_pos = cam_pos + R * eksen + np.asarray(sapma, float)
        return (geo.bbox_gt_goruntu(hedef_pos, hedef_rpy, IRIS_POS, IRIS_RPY),
                hedef_pos)

    # ── T49: GT modu optik eksendeki hedefi kadraj merkezine koyar ──
    gt, _ = gt_sahne(20.0, (0.0, 0.0, math.radians(90)))
    r = LeadPursuitCore(cfg_copy()).process(None, 0.0, ATT_KAMERA_YATAY, gt=gt)
    merkez_sapma = math.hypot(gt["uv"][0] - CX, gt["uv"][1] - CY)
    kontrol("T49 GT modu: optik eksendeki hedef kadraj merkezinde",
            merkez_sapma < 0.5 and r["gt_modu"] is True,
            f"uv=({gt['uv'][0]:.1f},{gt['uv'][1]:.1f}) beklenen=({CX:.0f},{CY:.0f}) "
            f"sapma={merkez_sapma:.3f}px")

    # ── T50: GT modu det=None ile çalışır; tespit güveni diye bir şey yok ──
    gt, _ = gt_sahne(10.0, (0.0, 0.0, math.radians(90)))
    r = LeadPursuitCore(cfg_copy()).process(None, 0.0, ATT_KAMERA_YATAY, gt=gt)
    kontrol("T50 GT modu det=None ile çalışır, tespit kapıları devre dışı",
            r["durum"] == "ok" and r["guven"] == 1.0 and r["kalite"] > 0.0,
            f"durum={r['durum']} guven={r['guven']} kalite={r['kalite']:.2f}")

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

    # ── T52: GT gerçek kutu boyutunu da taşır (ölçüm/teşhis sütunları) ──
    # Kutu YÖNELİME duyarlı: tam yandan bakışta gövde boyu görünür, karşıdan
    # bakışta kanat açıklığı — ikisi farklı genişlik verir.
    gt_yan, _ = gt_sahne(12.0, (0.0, 0.0, math.radians(90)))    # LOS'a dik
    gt_kars, _ = gt_sahne(12.0, (0.0, 0.0, math.radians(180)))  # bize doğru
    kontrol("T52 GT kutusu yönelimle değişir (w/h dolu ve anlamlı)",
            gt_yan["w"] is not None and gt_kars["w"] is not None
            and gt_yan["w"] > 0 and gt_kars["w"] > 0
            and abs(gt_yan["w"] - gt_kars["w"]) > 1.0,
            f"yandan w={gt_yan['w']:.1f}px · karşıdan w={gt_kars['w']:.1f}px")

    # ── T53: KRİTİK — GT yolu ile KUSURSUZ bbox yolu aynı yere nişan alır ──
    # Aynı 3B sahneden hem gt hem (geometry ile projekte edilmiş) MÜKEMMEL kutu
    # üretilir; iki yol aynı yasayı çalıştırdığı için nişan aynı olmalı.
    def gt_bbox_kiyas(R, hedef_rpy, sapma=(0.0, 0.0, 0.0)):
        gt_, hp_ = gt_sahne(R, hedef_rpy, sapma)
        bb_ = geo.target_bbox(hp_, hedef_rpy, IRIS_POS, IRIS_RPY)
        if gt_ is None or bb_ is None:
            return None
        dz_ = {"cx": (bb_[0] + bb_[2]) / 2, "cy": (bb_[1] + bb_[3]) / 2,
               "w": bb_[2] - bb_[0], "h": bb_[3] - bb_[1],
               "conf": 1.0, "bbox": bb_}
        return (LeadPursuitCore(cfg_copy()).process(None, 0.0, ATT_KAMERA_YATAY, gt=gt_),
                LeadPursuitCore(cfg_copy()).process(dz_, 0.0, ATT_KAMERA_YATAY))

    SEVIYELI = (0.0, 0.0, math.pi)     # seviyeli, burnu bize dönük
    en_kotu = {"yaw": 0.0, "pit": 0.0}
    kiyas_sayisi = 0
    for _sap in ((0.0, 0.0, 0.0), (0.0, 1.5, 0.0), (0.0, -2.0, 1.0)):
        _k = gt_bbox_kiyas(12.0, SEVIYELI, _sap)
        if _k is None:
            continue
        kiyas_sayisi += 1
        _g, _p = _k
        en_kotu["yaw"] = max(en_kotu["yaw"],
                             abs(math.degrees(_norm(_g["yaw_hata"] - _p["yaw_hata"]))))
        en_kotu["pit"] = max(en_kotu["pit"],
                             abs(math.degrees(_g["pitch_hata"] - _p["pitch_hata"])))
    kontrol("T53 GT yolu ≈ kusursuz bbox yolu (aynı yasa, aynı nişan)",
            kiyas_sayisi == 3 and en_kotu["yaw"] < 2.0 and en_kotu["pit"] < 2.5,
            f"{kiyas_sayisi}/3 sahne — en kötü: Δyaw={en_kotu['yaw']:.2f}° "
            f"Δpitch={en_kotu['pit']:.2f}°")

    # ── T54: REGRESYON — gt=None verilince davranış birebir bbox modu ──
    d54 = make_det(9.0, cx=380.0, cy=260.0)
    r_a = LeadPursuitCore(cfg_copy()).process(d54, 0.0, ATT_KAMERA_YATAY)
    r_b = LeadPursuitCore(cfg_copy()).process(d54, 0.0, ATT_KAMERA_YATAY, gt=None)
    ayni = all(abs(r_a[k] - r_b[k]) < 1e-12 for k in
               ("olcek", "yaw_hata", "pitch_hata", "kalite", "duzeltme"))
    kontrol("T54 gt=None → bbox modu birebir korunur (regresyon)",
            ayni and r_a["gt_modu"] is False,
            f"yaw={math.degrees(r_a['yaw_hata']):.3f}° olcek={r_a['olcek']:.2f}px")

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
        return {"seq": i + 1, "det": make_det(8), "stamp": (i + 1) / 30.0,
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
    # Yaklaşma bandının İÇİNDE bir menzil seç (TERMINAL_MENZIL < m ≤ MAX)
    _m_yak = (cfgA.TERMINAL_MENZIL + cfgA.YAKLASMA_MAX_MENZIL) / 2.0
    r_y = CopterAdapter(cfgA).compute(ug20, 0.0, (0, 0, 0), dtA, 0.0,
                                      menzil=_m_yak)
    yatay_y = math.hypot(r_y["v_cmd"][0], r_y["v_cmd"][1])
    r_tt = CopterAdapter(cfgA).compute(ug20, 0.0, (0, 0, 0), dtA, 0.0,
                                       menzil=cfgA.TERMINAL_MENZIL - 1.0)
    yatay_t = math.hypot(r_tt["v_cmd"][0], r_tt["v_cmd"][1])
    kontrol("T57 yaklaşmada yatay hız V_YAKLASMA'ya iner, terminalde tam hız",
            r_y["alt_faz"] == "yaklasma" and r_tt["alt_faz"] == "terminal"
            and abs(yatay_y - cfgA.V_YAKLASMA) < 0.1 and yatay_t > yatay_y,
            f"yaklaşma {yatay_y:.1f} m/s (hedef {cfgA.V_YAKLASMA}) · "
            f"terminal {yatay_t:.1f} m/s")

    # ── T58: yaklaşmada dikey, hedefle İRTİFA FARKINI kapatır ──
    # Hedef 20° yukarıda, 20 m menzilde → dikey fark ≈ 6.8 m → drone TIRMANMALI
    # (NED'de vz negatif). Ters durumda (hedef aşağıda) alçalmalı.
    m20 = _m_yak                       # yaklaşma bandının içinde
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

    # ══════════════════════════════════════════════════════════
    # B5 — FLY-PAST + FAZ SONU KOMUT SIFIRLAMA (T60-T63, 2026-08-06)
    # Ölçüm: ıska sonrası hız vektörü ile burun arası açı medyan 54.4°, p90
    # 138.9° (ıska öncesi 25.0° / 84.1°); log 00000108'de vuruş anından
    # itibaren kesintisiz 14.57 tur dönüş, irtifa 21.8 → 2.0 m. Sebep: MAVLink
    # hız komutu KALICI — güdüm CSV'yi kapatıyor ama komut araçta yaşıyor.
    # ══════════════════════════════════════════════════════════
    cfgF = cfg_copy()
    cfgF.TERMINAL_MENZIL = 0.0        # kör dalış yoluna hiç girme
    cfgF.VURUS_MENZIL = 0.1
    cfgF.FLYPAST_MENZIL = 8.0
    cfgF.FLYPAST_BUYUME_M = 1.5

    # ── T60: menzil DÖNÜNCE görsel faz bırakılır (fly-past) ──
    # 10→3 m yaklaş, sonra 3→6 m uzaklaş: en yakın 3.0, +1.5 eşiği 4.5'te aşılır.
    menzF = [10.0, 8.0, 6.0, 4.0, 3.0, 3.5, 4.0, 4.6, 5.0, 6.0]
    sonucF = terminal_kosusu(cfgF, menzF, [True] * len(menzF))
    kontrol("T60 fly-past: menzil dönünce görsel faz bırakılır",
            sonucF == "kayip", f"sonuç={sonucF} (en yakın 3.0 m → 4.6 m'de kesilmeli)")

    # ── T61: menzil DÜZGÜN KAPANIRKEN fly-past tetiklenmez (yanlış alarm yok) ──
    # 10→1 m kesintisiz kapanma: hiçbir karede 'gecildi' yazılmamalı ve faz
    # fly-past yüzünden erken bırakılmamalı (kareler bitince normal biter).
    menzG = [10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0]
    terminal_kosusu(cfgF, menzG, [True] * len(menzG))
    _csvG = max(_glob.glob(_os.path.join(vlmod._LOG_DIR, "visual_lead_*.csv")),
                key=_os.path.getmtime)
    _satG = list(_csv.DictReader(open(_csvG)))
    _durG = {r["durum"] for r in _satG}
    kontrol("T61 sürekli kapanan menzilde fly-past YOK (yanlış alarm)",
            "gecildi" not in _durG and len(_satG) == len(menzG),
            f"{len(_satG)}/{len(menzG)} kare işlendi, durumlar={sorted(_durG)}")

    # ── T62: FAZ SONU KOMUT SIFIRLAMA — son mesaj (0,0,0) olmalı ──
    # _FakeMav gönderilen tüm SET_POSITION_TARGET paketlerini biriktiriyor;
    # sözleşme: son üç paket sıfır hız (UDP kaybına karşı 3 kez).
    connZ = _FakeConn()
    durumZ = {"i": 0}
    menzZ = [10.0, 8.0, 6.0, 4.0, 3.0, 4.0, 5.0, 6.0]

    def wpZ(son_seq, timeout=0.5):
        i = durumZ["i"]
        if i >= len(menzZ):
            return None
        durumZ["i"] += 1
        connZ.durum_yaz((menzZ[i], 0.0, 0.0))
        return {"seq": i + 1, "det": make_det(8), "stamp": (i + 1) / 30.0,
                "wall_recv": _t.time()}

    vlmod.run_visual_lead(connZ, wpZ, lambda: {"x": 0.0, "y": 0.0, "z": 0.0},
                          threading.Event(), cfg=cfgF, kayip_kare_esik=20)
    # send_velocity paketinin hız alanları: (..., vx, vy, vz, ...) — konum
    # sözleşmesini bilmek yerine "son paketlerde sıfırdan farklı hız yok"
    # ölçütü kullanılır (paket düzeni değişse de test anlamlı kalır).
    _sonlar = connZ.mav.gonderilen[-3:]
    _sifir = all(all(abs(float(x)) < 1e-9 for x in pkt[5:8]) for pkt in _sonlar)
    kontrol("T62 faz sonunda hız komutu SIFIRLANIR (komut araçta yaşamasın)",
            len(connZ.mav.gonderilen) >= 3 and _sifir,
            f"{len(connZ.mav.gonderilen)} paket, son 3'ünün hızı sıfır={_sifir}")

    # ── T63: nişan GERİYİ gösterince (hedef arkada) fly-past ──
    # u_govde[0] < 0 → |yaw_hata| > 90°. Kutu kadrajın çok kenarında ve
    # attitude aracı çevirmiş olsa bile bu imza bağımsız çalışmalı.
    cfgB5 = cfg_copy()
    cfgB5.FLYPAST_ARKA = True
    r_arka = tek_kare(cfgB5, make_det(8, cx=640), att=(0.0, math.radians(-25.0), 0.0))
    # cx=640 kenarı yaw_hata ≈ +64.7° verir (arka DEĞİL) — imzanın yanlış
    # tetiklenmediği de test edilmeli:
    kontrol("T63 kadraj kenarı 'arka' sayılmaz (yanlış alarm yok)",
            float(r_arka["u_govde"][0]) > 0,
            f"u_govde[0]={float(r_arka['u_govde'][0]):+.3f} "
            f"yaw_hata={r_arka['yaw_hata_deg']:+.1f}°")

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
