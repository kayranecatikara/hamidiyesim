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

    # ── B1: NİŞANDA — yaw ≈ mevcut, dikey ≈ 0 ──
    # ⚠ 2026-08-09: tutuş dikey hedefi artık sabit piksel DEĞİL, atalet
    # yükselişi (ELEV_HEDEF_DEG). Nişan pikseli pitch'e göre değişir; pitch=0
    # için: cy = CY + FY·tan(kamera_tilt − ELEV_HEDEF).
    _cy_nisan_atalet = geo.CY + geo.FY * math.tan(
        math.radians(25.0 - C.ELEV_HEDEF_DEG))
    vx, vy, vz, yaw, _I, t, _eI = ib.komut(CX, _cy_nisan_atalet, 40, 40, 0.0, 0.0,
                                      0.05, C)
    kontrol("B1  nişan noktasında: yaw≈0, vz≈0",
            abs(math.degrees(yaw)) < 0.5 and abs(vz) < 0.05,
            f"yaw={math.degrees(yaw):.2f}° vz={vz:.3f}")

    # ── B2: hedef SAĞDA (cx>CX) → yaw komutu POZİTİF (sağa dön) ──
    vx, vy, vz, yaw_sag, _I, t, _eI = ib.komut(CX + 100, C.CY_NISAN, 40, 40, 0.0, 0.0, 0.05, C)
    _, _, _, yaw_sol, _, _, _eI = ib.komut(CX - 100, C.CY_NISAN, 40, 40, 0.0, 0.0, 0.05, C)
    kontrol("B2  hedef sağda → yaw>0, solda → yaw<0",
            yaw_sag > 0.05 and yaw_sol < -0.05,
            f"sağ yaw={math.degrees(yaw_sag):+.1f}° sol yaw={math.degrees(yaw_sol):+.1f}°")

    # ── B3: HIZ — küçük kutu (uzak) hızlı, REF'te integral kadar, yakın geri ──
    _, _, _, _, _, t_uzak, _eI = ib.komut(CX, C.CY_NISAN, 5, 5, 0.0, 0.0, 0.05, C)
    _, _, _, _, _, t_yakin, _eI = ib.komut(CX, C.CY_NISAN, 60, 60, 0.0, 0.0, 0.05, C)
    _, _, _, _, _, t_denge, _eI = ib.komut(CX, C.CY_NISAN, C.BOYUT_REF, C.BOYUT_REF,
                                      0.0, 0.0, 0.05, C)
    # Yakın kutuda hız 0'a iner ama NEGATİF OLMAZ (V_MIN=0, geri gitme yok —
    # 2026-08-08 kullanıcı kararı: fren vuruşu engelliyordu).
    kontrol("B3  uzak kutu hızlı, REF'te integral kadar, yakında 0 (geri YOK)",
            t_uzak["v_los"] > 4.0 and abs(t_denge["v_los"]) < 1e-6
            and t_yakin["v_los"] == 0.0,
            f"5px→{t_uzak['v_los']:+.1f}  REF({C.BOYUT_REF:.0f}px, I=0)→"
            f"{t_denge['v_los']:+.1f}  60px→{t_yakin['v_los']:+.1f} m/s")

    # ── B4: DİKEY — hedef kadrajda AŞAĞIDA (cy>nişan) → ALÇAL (vz>0, NED down+) ──
    _, _, vz_asa, _, _, _, _eI = ib.komut(CX, C.CY_NISAN + 120, 40, 40, 0.0, 0.0, 0.05, C)
    _, _, vz_yuk, _, _, _, _eI = ib.komut(CX, C.CY_NISAN - 120, 40, 40, 0.0, 0.0, 0.05, C)
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
    r1 = ib.komut(CX + 50, CY, 60, 40, 1.2, 8.0, 0.05, C)
    r2 = ib.komut(CX + 50, CY, 60, 40, 1.2, 8.0, 0.05, C)
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
    vx0, vy0, _, _, _, _, _eI = ib.komut(CX, C.CY_NISAN, 12, 12, 0.0, 0.0, 0.05, C)
    vx1, vy1, _, _, _, _, _eI = ib.komut(CX, C.CY_NISAN, 12, 12, 0.0, HEDEF_V, 0.05, C)
    kontrol("B10 integral sıcak başlangıcı: hedefin hızını aşan komut",
            math.hypot(vx0, vy0) < HEDEF_V and math.hypot(vx1, vy1) > HEDEF_V,
            f"I=0 → {math.hypot(vx0, vy0):.1f} m/s  |  "
            f"I=15 → {math.hypot(vx1, vy1):.1f} m/s  (hedef {HEDEF_V:.0f})")

    # ── B11: toplam yatay hız tavanı bağlar ──
    vx2, vy2, _, _, _, _, _eI = ib.komut(CX, C.CY_NISAN, 5, 5, 0.0, 17.0, 0.05, C)
    kontrol("B11 toplam hız V_TOPLAM_MAX ile tavanlı",
            math.hypot(vx2, vy2) <= C.V_TOPLAM_MAX + 1e-6,
            f"17+kapanma → {math.hypot(vx2, vy2):.2f} ≤ {C.V_TOPLAM_MAX}")

    # ── B12: kutu boşluğunda SON KOMUT sürer (sıfır komut kalıcı kayıp yapar) ──
    # ⚠ Eski sürüm burada DONDURULMUŞ NED taşıyıcıyı basıyordu; 2026-08-08
    # uçuşunda o taşıyıcı hedef döndükçe drone'u yana savurdu (aspect 7°→70°,
    # mesafe 8.7→66.6 m). Artık son LOS komutu sürdürülür — yön hedefin son
    # görüldüğü yöndedir, sabit bir NED vektörü değil.
    conn3 = _FakeConn()
    st3 = {"seq": 0}
    def wait_pose_karisik(son_seq, timeout=0.5):
        st3["seq"] += 1
        # ilk 8 kare kutu var, sonra boşluk
        p = ({"bbox": (300, 290, 320, 305), "conf": 0.8}
             if st3["seq"] <= 8 else None)
        return {"seq": st3["seq"], "pose": p,
                "stamp": None, "wall_recv": None, "lock": None}
    # Devir gerçeği: drone zaten uçuyor (ivme sınırlayıcı oradan başlar)
    def get_iris_ucan():
        return {"yaw": 0.0, "vx": 14.0, "vy": 3.0, "vz": 0.0}
    stop3 = threading.Event()
    th3 = threading.Thread(
        target=ib.run_bbox_ibvs,
        args=(conn3, get_iris_ucan, wait_pose_karisik, stop3, ib.Cfg, 50,
              (14.0, 3.0, 0.0)),
        daemon=True)
    th3.start(); time.sleep(0.6); s3 = conn3.mav.last; stop3.set(); th3.join(2.0)
    hiz3 = math.hypot(s3[8], s3[9]) if s3 else 0.0
    kontrol("B12 kutu boşluğunda son komut sürüyor (sıfırlanmıyor)",
            s3 is not None and hiz3 > 5.0,
            f"boşlukta komut hızı {hiz3:.1f} m/s (sıfır olmamalı)")

    # ── B13: TERMİNAL HÜCUM — fren yok, tam taahhüt ──
    # Kullanıcı kararı (2026-08-08): "o freni koymasan aracı vurabiliyoruz."
    _, _, _, _, _, t_tut, _eI = ib.komut(CX, C.CY_NISAN, 60, 60, 0.0, 0.0, 0.05,
                                    C, False)
    _, _, _, _, _, t_ter, _eI = ib.komut(CX, C.CY_NISAN, 60, 60, 0.0, 0.0, 0.05,
                                    C, True)
    kontrol("B13 terminalde fren yok: v = V_TERMINAL (tut modunda ise 0)",
            abs(t_ter["v_los"] - C.V_TERMINAL) < 1e-6 and t_tut["v_los"] < 1.0,
            f"tut modu {t_tut['v_los']:+.1f} → terminal {t_ter['v_los']:+.1f} m/s "
            f"(yaklaşma tavanı {C.V_TOPLAM_MAX:.0f} ayrı)")

    kontrol("B14 geri gitme YOK (V_MIN=0) ve tavan 24 m/s",
            C.V_MIN == 0.0 and C.V_TOPLAM_MAX >= 24.0,
            f"V_MIN={C.V_MIN}  V_TOPLAM_MAX={C.V_TOPLAM_MAX}")

    # ── B15: ÇARPMA SENSÖRÜ — temas gelince 'vuruldu' ──
    conn4 = _FakeConn()
    st4 = {"seq": 0}
    def wait_pose4(son_seq, timeout=0.5):
        st4["seq"] += 1
        return {"seq": st4["seq"],
                "pose": {"bbox": (310, 295, 330, 310), "conf": 0.8},
                "stamp": None, "wall_recv": None, "lock": None}
    temas_durum = {"v": False}
    def get_temas():
        st4["seq"] > 5 and temas_durum.update(v=True)
        return temas_durum["v"]
    stop4 = threading.Event()
    son4 = {"r": None}
    def kosu4():
        son4["r"] = ib.run_bbox_ibvs(conn4, get_iris, wait_pose4, stop4,
                                     ib.Cfg, 50, (14.0, 0.0, 0.0), get_temas)
    th4 = threading.Thread(target=kosu4, daemon=True)
    th4.start(); th4.join(3.0); stop4.set()
    kontrol("B15 Talon çarpma sensörü → 'vuruldu' ile biter",
            son4["r"] == "vuruldu", f"dönüş={son4['r']}")

    # ── B16: ⚠ KÖR HÜCUM SÜRE SINIRLI — sınırsız kalırsa araç kaçar ──
    # 2026-08-08 hatası: süre sınırı yoktu; drone ıskaladıktan sonra son
    # komutu 260 s bastı ve 1032 m uzağa uçtu, faz hiç 'kayip' dönmedi.
    class _KisaCfg(ib.Cfg):
        TERMINAL_SURE = 0.4        # test için kısa
        TERMINAL_BOYUT = 20.0      # hemen terminale girsin
    conn5 = _FakeConn()
    st5 = {"seq": 0}
    def wait_pose5(son_seq, timeout=0.5):
        st5["seq"] += 1
        # ilk 5 kare BÜYÜK kutu (terminal tetiklenir), sonra hiç kutu yok
        p = ({"bbox": (300, 285, 340, 315), "conf": 0.9}
             if st5["seq"] <= 5 else None)
        return {"seq": st5["seq"], "pose": p,
                "stamp": None, "wall_recv": None, "lock": None}
    stop5 = threading.Event()
    son5 = {"r": None}
    def kosu5():
        son5["r"] = ib.run_bbox_ibvs(conn5, get_iris, wait_pose5, stop5,
                                     _KisaCfg, 10, (14.0, 0.0, 0.0))
    th5 = threading.Thread(target=kosu5, daemon=True)
    th5.start(); th5.join(5.0); stop5.set()
    kontrol("B16 kör hücum süre dolunca ISKA → 'kayip' (sonsuz kör YOK)",
            son5["r"] == "kayip", f"dönüş={son5['r']} (thread canlı mı: {th5.is_alive()})")

    # ── B17: piksel → LOS yükselişi geometrisi (25° tilt) ──
    e_bore = math.degrees(ib.piksel_elev(geo.CY, C))
    e_seviye = math.degrees(ib.piksel_elev(geo.CY + geo.FY * math.tan(math.radians(25)), C))
    kontrol("B17 kadraj merkezi → +25° (boresight), seviye hedef pikseli → 0°",
            abs(e_bore - 25.0) < 0.2 and abs(e_seviye) < 0.3,
            f"cy=240 → {e_bore:+.1f}°   cy=318 → {e_seviye:+.1f}°")

    # ── B18: TERMİNAL KESİŞİM — hız vektörü hedefe DOĞRU bakar ──
    # 2026-08-08 ölçümü: ıskanın baskın bileşeni DİKEYDİ (0.5-1.1 m). Sebep:
    # terminalde bile dikey kanal "tutuş" yasasıydı → hedefin altından geçiyorduk.
    cy_ust = geo.CY + geo.FY * math.tan(math.radians(15))   # hedef 10° yukarıda
    _, _, vz_tut, _, _, _, _eI = ib.komut(CX, cy_ust, 30, 30, 0.0, 10.0, 0.05, C,
                                     False, (0.0, 0.0), 0.0)
    _, _, vz_ter, _, _, _, _eI = ib.komut(CX, cy_ust, 30, 30, 0.0, 10.0, 0.05, C,
                                     True, (0.0, 0.0), 0.0)
    # hedef 10° yukarıda → kesişim için TIRMANMALI (vz<0, NED)
    kontrol("B18 terminalde hedef yukarıdayken TIRMANIR (tutuş modu tırmanmıyordu)",
            vz_ter < -0.5 and vz_ter < vz_tut,
            f"tutuş vz={vz_tut:+.2f}  →  terminal vz={vz_ter:+.2f} m/s")

    # ── B19: LEAD yalnız TERMİNALDE ve LOS dönüyorken ──
    _, _, _, yaw_ldsz, _, t_ldsz, _eI = ib.komut(CX, C.CY_NISAN, 30, 30, 0.0, 10.0,
                                            0.05, C, True, (0.0, 0.0), 0.0)
    _, _, _, yaw_ld, _, t_ld, _eI = ib.komut(CX, C.CY_NISAN, 30, 30, 0.0, 10.0,
                                        0.05, C, True, (0.5, 0.0), 0.0)
    _, _, _, _, _, t_tut2, _eI = ib.komut(CX, C.CY_NISAN, 30, 30, 0.0, 10.0,
                                     0.05, C, False, (0.5, 0.0), 0.0)
    kontrol("B19 lead: LOS dönerken terminalde nişan öne alınır, tutuşta ALINMAZ",
            abs(t_ldsz["lead_az"]) < 1e-9 and t_ld["lead_az"] > 0.1
            and abs(t_tut2["lead_az"]) < 1e-9,
            f"LOS=0 → {math.degrees(t_ldsz['lead_az']):.1f}°  "
            f"LOS=0.5 rad/s → {math.degrees(t_ld['lead_az']):.1f}°  "
            f"tutuş → {math.degrees(t_tut2['lead_az']):.1f}°")

    # ── B22: DİKEY BÜTÇE KISITI — hız vektörü hedefe BAKABİLMELİ ──
    # Kullanıcı gözlemi (2026-08-09): "hele dikeyde çok kaçırıyor".
    # Mekanizma: v=18, vz tavanı 5 → vektör en fazla 15.5° yukarı bakabilir;
    # hedef daha yukarıdaysa kesişim imkânsız. Ölçüldü: terminal karelerinin
    # %22-49'unda vz doymuştu. Çözüm: yatayı kıs, vektör hedefe baksın.
    def _cy_icin(elev_deg):
        """Verilen LOS yükselişini üretecek piksel (boresight 25° yukarıda)."""
        return geo.CY + geo.FY * math.tan(
            math.radians(geo_tilt := 25.0) - math.radians(elev_deg))

    # 20° — eski sürümün 15.5° tavanının ÜSTÜNDE, ama hız tabanının bağladığı
    # noktanın altında: vektör hedefe TAM bakabilmeli.
    cy20 = _cy_icin(20.0)
    vx_d, vy_d, vz_d, _, _, _, _eI = ib.komut(CX, cy20, 30, 30, 0.0, 10.0, 0.05,
                                         C, True, (0.0, 0.0), 0.0)
    vyatay = math.hypot(vx_d, vy_d)
    elev_vektor = math.degrees(math.atan2(-vz_d, vyatay)) if vyatay > 1e-6 else 0.0
    kontrol("B22 dik hedefte yatay kısılır, hız vektörü hedefe TAM bakar",
            vyatay < C.V_TERMINAL - 1.0 and abs(elev_vektor - 20.0) < 2.0,
            f"hedef 20° yukarıda → yatay {C.V_TERMINAL:.0f}→{vyatay:.1f} m/s, "
            f"vektör {elev_vektor:.1f}° (eski sürüm 15.5°'de takılırdı)")

    # 35° — taban bağlar; vektör hedefe tam bakamaz ama eski 15.5°'den DİK
    cy35 = _cy_icin(35.0)
    vx_e, vy_e, vz_e, _, _, _, _eI = ib.komut(CX, cy35, 30, 30, 0.0, 10.0, 0.05,
                                         C, True, (0.0, 0.0), 0.0)
    vyat_e = math.hypot(vx_e, vy_e)
    elev_e = math.degrees(math.atan2(-vz_e, vyat_e)) if vyat_e > 1e-6 else 0.0
    # Taban 14 m/s (hedefin 14.5'ine yakın — geride kalmamak için). Bu tabanla
    # ulaşılabilen en dik açı atan(VZ_MAX_TERM/14) ≈ 19.7°; eski tavan 15.5°'ydi.
    _ulasilabilir = math.degrees(math.atan(C.VZ_MAX_TERM / C.V_TERM_MIN))
    kontrol("B23 aşırı dikte hız tabanı bağlar (hedefi büsbütün kaçırmamak için)",
            abs(vyat_e - C.V_TERM_MIN) < 1e-6
            and abs(elev_e - _ulasilabilir) < 1.5 and elev_e > 15.5,
            f"hedef 35° → yatay taban {vyat_e:.1f} m/s, vektör {elev_e:.1f}° "
            f"(taban sınırı {_ulasilabilir:.1f}°; eski sürüm 15.5°'de takılırdı)")

    # ── B24: TERMİNAL DİKEY SÖNÜMLEME — "üstten geçme" önleyici ──
    # Kullanıcının manuel uçuş kaydı (log 081132): hedef TAM nişandayken
    # (dikey hata −2.2°) vz komutu −4.2 m/s; sonra kutu 294→456 px kaydı,
    # yani hedefin üstünden geçildi. Sebep: dikey kanalda türev/sönümleme
    # terimi yoktu, araç tırmanma momentumu kazanıp geç sönüyordu.
    cy_bir_az_ust = geo.CY + geo.FY * math.tan(math.radians(25 - 8))
    _, _, vz_durgun, _, _, _, _eI = ib.komut(CX, cy_bir_az_ust, 30, 30, 0.0, 10.0,
                                        0.05, C, True, (0.0, 0.0), 0.0, 0.0)
    _, _, vz_tirmanan, _, _, _, _eI = ib.komut(CX, cy_bir_az_ust, 30, 30, 0.0, 10.0,
                                          0.05, C, True, (0.0, 0.0), 0.0, -4.0)
    kontrol("B24 zaten tırmanan araçta dikey komut GERİ ÇEKİLİR (sönümleme)",
            vz_durgun < -2.0 and vz_tirmanan > vz_durgun + 1.5,
            f"araç durgunken {vz_durgun:+.2f} → 4 m/s tırmanırken "
            f"{vz_tirmanan:+.2f} m/s (fark {vz_tirmanan - vz_durgun:+.2f})")

    # ── B25: İRTİFA EŞİTLEME — gövde pitch'i hesaba katılır (jiroskop) ──
    # Kullanıcı fikri (2026-08-09): önce irtifayı eşitle, sonra dal.
    # Aynı PİKSEL, farklı gövde pitch'i → FARKLI gerçek yükseliş. Eski yasa
    # (sabit piksel) bunu göremiyordu; dikey limit çevriminin kaynağı buydu.
    _cy_test = geo.CY + geo.FY * math.tan(math.radians(25.0 - C.ELEV_HEDEF_DEG))
    _, _, vz_p0, _, _, t_p0, _eI = ib.komut(CX, _cy_test, 40, 40, 0.0, 0.0, 0.05, C,
                                       False, (0.0, 0.0), 0.0, 0.0)
    # aynı piksel ama araç 10° burun YUKARI → hedef gerçekte 10° daha yukarıda
    _, _, vz_p10, _, _, t_p10, _eI = ib.komut(CX, _cy_test, 40, 40, 0.0, 0.0, 0.05, C,
                                         False, (0.0, 0.0), math.radians(10.0), 0.0)
    kontrol("B25 aynı piksel + burun yukarı → TIRMANMA komutu (pitch hesaba katılıyor)",
            abs(vz_p0) < 0.05 and vz_p10 < -0.5,
            f"pitch 0° → vz {vz_p0:+.2f} | pitch +10° → vz {vz_p10:+.2f} m/s "
            f"(eski sabit-piksel yasası ikisinde de aynı verirdi)")

    kontrol("B26 terminal kapısı irtifa şartı içeriyor (eşik tanımlı)",
            C.TERMINAL_ELEV_ESIK > 0 and C.ELEV_ATALET,
            f"yükseliş hedefi {C.ELEV_HEDEF_DEG:.0f}°, hücum eşiği "
            f"±{C.TERMINAL_ELEV_ESIK:.0f}° — irtifa oturmadan hücum açılmaz")

    # ── B21: ⚠ YAW SLEW SINIRI — takla önleyici ──
    # 2026-08-09: görsel fazda yaw komutu 876 °/s'ye çıkıyordu (araç ~120);
    # yaw doyumu roll/pitch yetkisini yiyor → takla. Ölçülen medyan 12-38 °/s,
    # yani sınır normal takibi KISITLAMAZ, yalnız fly-past'ta bağlar.
    conn6 = _FakeConn()
    st6 = {"seq": 0}
    def wait_pose_savrulan(son_seq, timeout=0.5):
        st6["seq"] += 1
        # kutu kadrajı sağdan sola SÜPÜRÜYOR (fly-past): her karede ±uç
        cxx = 600 if st6["seq"] % 2 else 40
        return {"seq": st6["seq"],
                "pose": {"bbox": (cxx - 15, 290, cxx + 15, 315), "conf": 0.9},
                "stamp": None, "wall_recv": None, "lock": None}
    stop6 = threading.Event()
    th6 = threading.Thread(
        target=ib.run_bbox_ibvs,
        args=(conn6, get_iris, wait_pose_savrulan, stop6, ib.Cfg, 100,
              (14.0, 0.0, 0.0)),
        daemon=True)
    th6.start(); time.sleep(0.7); stop6.set(); th6.join(2.0)
    # log'dan yaw komutu değişim hızını ölç
    import glob as _g2, os as _o2, csv as _c2
    _y6 = max(_g2.glob(_o2.path.join(ib._LOG_DIR, "*.csv")), key=_o2.path.getmtime)
    _r6 = [r for r in _c2.DictReader(open(_y6)) if r["durum"] in ("IBVS", "TERMINAL")]
    _hiz6 = []
    for a, b in zip(_r6, _r6[1:]):
        try:
            _dt = float(b["t"]) - float(a["t"])
            _d = (float(b["yaw_cmd_deg"]) - float(a["yaw_cmd_deg"]) + 180) % 360 - 180
            if 1e-3 < _dt < 0.5:
                _hiz6.append(abs(_d / _dt))
        except (ValueError, KeyError):
            pass
    _enb = max(_hiz6) if _hiz6 else 0.0
    _sinir = math.degrees(C.YAW_RATE_MAX)
    kontrol("B21 yaw komut hızı slew sınırında kalır (fly-past'ta takla yok)",
            len(_hiz6) > 5 and _enb <= _sinir * 1.15,
            f"savrulan kutuda en hızlı yaw komutu {_enb:.0f}°/s ≤ sınır "
            f"{_sinir:.0f}°/s (sınırsız sürüm 876°/s üretmişti)")

    kontrol("B20 lead açısı tavanla sınırlı (gürültülü LOS savurmasın)",
            abs(ib.komut(CX, C.CY_NISAN, 30, 30, 0.0, 10.0, 0.05, C, True,
                         (50.0, 0.0), 0.0)[5]["lead_az"])
            <= math.radians(C.LEAD_MAX_DEG) + 1e-9,
            f"LOS=50 rad/s → tavan {C.LEAD_MAX_DEG:.0f}°")

    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
