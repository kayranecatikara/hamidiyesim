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
from control.guidance.guidance_core import Cfg as GeoCfg

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def main():
    print("SAF bbox IBVS kabul kriterleri")

    C = ib.Cfg
    CX, CY, FX, FY = geo.CX, geo.CY, geo.FX, geo.FY

    # ── B1: MERKEZDE nişan — yaw ≈ mevcut, dikey ≈ 0 ──
    # cx=CX (yatay merkez), cy=CY_NISAN (dikey nişan) → sapma yok.
    vx, vy, vz, yaw, _I, t = ib.komut(CX, C.CY_NISAN, 40, 40, 0.0, 0.0, 0.05, C)
    kontrol("B1  nişan noktasında: yaw≈0, vz≈0",
            abs(math.degrees(yaw)) < 0.5 and abs(vz) < 0.05,
            f"yaw={math.degrees(yaw):.2f}° vz={vz:.3f}")

    # ── B2: hedef SAĞDA (cx>CX) → yaw komutu POZİTİF (sağa dön) ──
    vx, vy, vz, yaw_sag, _I, t = ib.komut(CX + 100, C.CY_NISAN, 40, 40, 0.0, 0.0, 0.05, C)
    _, _, _, yaw_sol, _, _ = ib.komut(CX - 100, C.CY_NISAN, 40, 40, 0.0, 0.0, 0.05, C)
    kontrol("B2  hedef sağda → yaw>0, solda → yaw<0",
            yaw_sag > 0.05 and yaw_sol < -0.05,
            f"sağ yaw={math.degrees(yaw_sag):+.1f}° sol yaw={math.degrees(yaw_sol):+.1f}°")

    # ── B3: HIZ — küçük kutu (uzak) hızlı, REF'te integral kadar, yakın geri ──
    _, _, _, _, _, t_uzak = ib.komut(CX, C.CY_NISAN, 5, 5, 0.0, 0.0, 0.05, C)
    _, _, _, _, _, t_yakin = ib.komut(CX, C.CY_NISAN, 60, 60, 0.0, 0.0, 0.05, C)
    _, _, _, _, _, t_denge = ib.komut(CX, C.CY_NISAN, C.BOYUT_REF, C.BOYUT_REF,
                                      0.0, 0.0, 0.05, C)
    # Yakın kutuda hız 0'a iner ama NEGATİF OLMAZ (V_MIN=0, geri gitme yok —
    # 2026-08-08 kullanıcı kararı: fren vuruşu engelliyordu).
    kontrol("B3  uzak kutu hızlı, REF'te integral kadar, yakında 0 (geri YOK)",
            t_uzak["v_los"] > 4.0 and abs(t_denge["v_los"]) < 1e-6
            and t_yakin["v_los"] == 0.0,
            f"5px→{t_uzak['v_los']:+.1f}  REF({C.BOYUT_REF:.0f}px, I=0)→"
            f"{t_denge['v_los']:+.1f}  60px→{t_yakin['v_los']:+.1f} m/s")

    # ── B4: DİKEY — hedef kadrajda AŞAĞIDA (cy>nişan) → ALÇAL (vz>0, NED down+) ──
    _, _, vz_asa, _, _, _ = ib.komut(CX, C.CY_NISAN + 120, 40, 40, 0.0, 0.0, 0.05, C)
    _, _, vz_yuk, _, _, _ = ib.komut(CX, C.CY_NISAN - 120, 40, 40, 0.0, 0.0, 0.05, C)
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
    vx0, vy0, _, _, _, _ = ib.komut(CX, C.CY_NISAN, 12, 12, 0.0, 0.0, 0.05, C)
    vx1, vy1, _, _, _, _ = ib.komut(CX, C.CY_NISAN, 12, 12, 0.0, HEDEF_V, 0.05, C)
    kontrol("B10 integral sıcak başlangıcı: hedefin hızını aşan komut",
            math.hypot(vx0, vy0) < HEDEF_V and math.hypot(vx1, vy1) > HEDEF_V,
            f"I=0 → {math.hypot(vx0, vy0):.1f} m/s  |  "
            f"I=15 → {math.hypot(vx1, vy1):.1f} m/s  (hedef {HEDEF_V:.0f})")

    # ── B11: toplam yatay hız tavanı bağlar ──
    vx2, vy2, _, _, _, _ = ib.komut(CX, C.CY_NISAN, 5, 5, 0.0, 17.0, 0.05, C)
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
    _, _, _, _, _, t_tut = ib.komut(CX, C.CY_NISAN, 60, 60, 0.0, 0.0, 0.05,
                                    C, False)
    _, _, _, _, _, t_ter = ib.komut(CX, C.CY_NISAN, 60, 60, 0.0, 0.0, 0.05,
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
    _, _, vz_tut, _, _, _ = ib.komut(CX, cy_ust, 30, 30, 0.0, 10.0, 0.05, C,
                                     False, (0.0, 0.0), 0.0)
    _, _, vz_ter, _, _, _ = ib.komut(CX, cy_ust, 30, 30, 0.0, 10.0, 0.05, C,
                                     True, (0.0, 0.0), 0.0)
    # hedef 10° yukarıda → kesişim için TIRMANMALI (vz<0, NED)
    kontrol("B18 terminalde hedef yukarıdayken TIRMANIR (tutuş modu tırmanmıyordu)",
            vz_ter < -0.5 and vz_ter < vz_tut,
            f"tutuş vz={vz_tut:+.2f}  →  terminal vz={vz_ter:+.2f} m/s")

    # ── B19: LEAD yalnız TERMİNALDE ve LOS dönüyorken (VARSAYILAN davranış) ──
    # 2026-08-09/M3: kapıyı kaldırmak DENENDİ ve uçuşta ölçülüp GERİ ALINDI —
    # kadrajda tutuş düzeldi ama yaklaşma bozuldu (bkz. Cfg.LEAD_ERKEN, B33-B37).
    # Varsayılan yol yine "yalnız terminal"; bu test onu bekçiliyor.
    _, _, _, yaw_ldsz, _, t_ldsz = ib.komut(CX, C.CY_NISAN, 30, 30, 0.0, 10.0,
                                            0.05, C, True, (0.0, 0.0), 0.0)
    _, _, _, yaw_ld, _, t_ld = ib.komut(CX, C.CY_NISAN, 30, 30, 0.0, 10.0,
                                        0.05, C, True, (0.5, 0.0), 0.0)
    _, _, _, _, _, t_tut2 = ib.komut(CX, C.CY_NISAN, 30, 30, 0.0, 10.0,
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
    vx_d, vy_d, vz_d, _, _, _ = ib.komut(CX, cy20, 30, 30, 0.0, 10.0, 0.05,
                                         C, True, (0.0, 0.0), 0.0)
    vyatay = math.hypot(vx_d, vy_d)
    elev_vektor = math.degrees(math.atan2(-vz_d, vyatay)) if vyatay > 1e-6 else 0.0
    kontrol("B22 dik hedefte yatay kısılır, hız vektörü hedefe TAM bakar",
            vyatay < C.V_TERMINAL - 1.0 and abs(elev_vektor - 20.0) < 2.0,
            f"hedef 20° yukarıda → yatay {C.V_TERMINAL:.0f}→{vyatay:.1f} m/s, "
            f"vektör {elev_vektor:.1f}° (eski sürüm 15.5°'de takılırdı)")

    # 35° — taban bağlar; vektör hedefe tam bakamaz ama eski 15.5°'den DİK
    cy35 = _cy_icin(35.0)
    vx_e, vy_e, vz_e, _, _, _ = ib.komut(CX, cy35, 30, 30, 0.0, 10.0, 0.05,
                                         C, True, (0.0, 0.0), 0.0)
    vyat_e = math.hypot(vx_e, vy_e)
    elev_e = math.degrees(math.atan2(-vz_e, vyat_e)) if vyat_e > 1e-6 else 0.0
    kontrol("B23 aşırı dikte hız tabanı bağlar (hedefi büsbütün kaçırmamak için)",
            abs(vyat_e - C.V_TERM_MIN) < 1e-6 and elev_e > 20.0,
            f"hedef 35° → yatay taban {vyat_e:.1f} m/s, vektör {elev_e:.0f}° "
            f"(eski sürüm 15.5°)")

    # ── B24: TERMİNAL DİKEY SÖNÜMLEME — "üstten geçme" önleyici ──
    # Kullanıcının manuel uçuş kaydı (log 081132): hedef TAM nişandayken
    # (dikey hata −2.2°) vz komutu −4.2 m/s; sonra kutu 294→456 px kaydı,
    # yani hedefin üstünden geçildi. Sebep: dikey kanalda türev/sönümleme
    # terimi yoktu, araç tırmanma momentumu kazanıp geç sönüyordu.
    cy_bir_az_ust = geo.CY + geo.FY * math.tan(math.radians(25 - 8))
    _, _, vz_durgun, _, _, _ = ib.komut(CX, cy_bir_az_ust, 30, 30, 0.0, 10.0,
                                        0.05, C, True, (0.0, 0.0), 0.0, 0.0)
    _, _, vz_tirmanan, _, _, _ = ib.komut(CX, cy_bir_az_ust, 30, 30, 0.0, 10.0,
                                          0.05, C, True, (0.0, 0.0), 0.0, -4.0)
    kontrol("B24 zaten tırmanan araçta dikey komut GERİ ÇEKİLİR (sönümleme)",
            vz_durgun < -2.0 and vz_tirmanan > vz_durgun + 1.5,
            f"araç durgunken {vz_durgun:+.2f} → 4 m/s tırmanırken "
            f"{vz_tirmanan:+.2f} m/s (fark {vz_tirmanan - vz_durgun:+.2f})")

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
    _sinir = C.YAW_RATE_MAX_DEG
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
    # ── B25: DİKEY KOMUT KAPANMA HIZIYLA ÖLÇEKLENİR ──
    # Kullanıcı uçuşta gördü: "tam vuracağı sırada yukarı manevra yapıp
    # aracın üstünden geçiyoruz." Kök neden tek çarpandı: dikey komut
    # DRONE'un hızıyla (18 m/s) ölçekleniyordu, oysa dikey farkı kapatmak
    # için olan süreyi KAPANMA hızı belirler (hedef kaçtığı için ~2 m/s).
    # Ölçüldü (4 hücum): 3.67 m'de 0.89 m'lik fark için −5.00 m/s komut
    # ediliyordu, gereken −0.37 m/s — 13.7 KAT fazla.
    class _Eski(ib.Cfg):
        KAPANMA = False
    _cy_yukari = geo.CY + geo.FY * math.tan(math.radians(11.0))   # hedef yukarıda
    _eski = ib.komut(CX, _cy_yukari, 40, 40, 0.0, 14.0, 0.05, _Eski, True,
                     (0.0, 0.0), 0.0, 0.0)[2]
    _yeni = ib.komut(CX, _cy_yukari, 40, 40, 0.0, 14.0, 0.05, C, True,
                     (0.0, 0.0), 0.0, 0.0, 2.0)          # ṙ = 2 m/s
    _vz = _yeni[2]
    kontrol("B25 dikey komut KAPANMA hızıyla ölçeklenir (üstten geçme kök nedeni)",
            abs(_vz) < abs(_eski) / 3.0 and abs(_vz) > 0.05,
            f"aynı geometri: eski yasa {_eski:+.2f} m/s → kapanma hızıyla "
            f"{_vz:+.2f} m/s (ṙ=2 m/s). Eski yasa drone'un 18 m/s'siyle "
            f"ölçekliyordu.")

    # B26: ölçek tabanı — kapanma dursa bile dikey düzeltme büsbütün ölmesin
    _dur = ib.komut(CX, _cy_yukari, 40, 40, 0.0, 14.0, 0.05, C, True,
                    (0.0, 0.0), 0.0, 0.0, 0.0)[2]        # ṙ = 0
    kontrol("B26 kapanma dursa bile dikey düzeltme ölmez (taban)",
            abs(_dur) > 0.05,
            f"ṙ=0 → vz {_dur:+.2f} m/s (taban {C.KAPANMA_MIN} m/s ile)")

    # B27: geri dönüş yolu — AVCI_IBVS_KAPANMA=0 eski davranışı aynen getirir
    kontrol("B27 kapanma ölçeklemesi kapatılabilir (eski davranış geri gelir)",
            abs(ib.komut(CX, _cy_yukari, 40, 40, 0.0, 14.0, 0.05, _Eski, True,
                         (0.0, 0.0), 0.0, 0.0, 2.0)[2] - _eski) < 1e-9,
            "KAPANMA=False iken kapanma hızı verilse bile YOK SAYILIR")


    print("=" * 60)
    # ── T1a: YATAY AÇI ROLL/PITCH TELAFİSİ ──
    # Kullanıcı uçuşta gördü: "düz uçuşta ıskalamıyor ama hedef manevra
    # yapınca görsel güdüm sapıtıyor, yatayda çok salınım oluyor."
    # Kök neden: atan((cx−CX)/FX) KAMERA azimutudur, araç yattığında SEVİYE
    # azimutu değildir. 5869 kare gerçek uçuş verisinde ölçüldü:
    #     yatış 0-9° → 0.6° hata,  20-29° → 11.0°,  30-39° → 13.9°
    class _RollYok(ib.Cfg):
        ROLL_TELAFI = False

    _t = math.radians(GeoCfg.KAMERA_TILT_DEG)

    def _pikselle(az, elev, roll, pitch):
        """SEVİYE çerçevesindeki (az, elev) + duruş → (cx, cy). los_seviye'nin
        TERSİ: yuvarlak gidiş-dönüş testi için bağımsız ileri izdüşüm."""
        ca, sa = math.cos(az), math.sin(az)
        ce, se = math.cos(elev), math.sin(elev)
        lx, ly, lz = ca * ce, sa * ce, -se           # seviye çerçevesi
        # seviye → gövde: Rx(−roll)·Ry(−pitch)
        cp, sp = math.cos(-pitch), math.sin(-pitch)
        bx0, by0, bz0 = lx * cp + lz * sp, ly, -lx * sp + lz * cp
        cr, sr = math.cos(-roll), math.sin(-roll)
        bx, by, bz = bx0, by0 * cr - bz0 * sr, by0 * sr + bz0 * cr
        # gövde → kamera (25° yukarı tilt)
        ct, st = math.cos(_t), math.sin(_t)
        ileri, sag, asagi = ct * bx - st * bz, by, st * bx + ct * bz
        return (geo.CX + geo.FX * sag / ileri,
                geo.CY + geo.FY * asagi / ileri)

    # B28: gidiş-dönüş — telafili okuma gerçek geometriyi GERİ VERİR
    _az0, _el0, _ro, _pi = math.radians(8.0), math.radians(15.0), \
        math.radians(40.0), math.radians(10.0)
    _cxp, _cyp = _pikselle(_az0, _el0, _ro, _pi)
    _azg, _elg = ib.los_seviye(_cxp, _cyp, _ro, _pi, C)
    kontrol("B28 roll/pitch telafisi gerçek yatay yönü geri verir (gidiş-dönüş)",
            abs(_azg - _az0) < math.radians(0.05)
            and abs(_elg - _el0) < math.radians(0.05),
            f"gerçek az={math.degrees(_az0):.1f}° (yatış 40°, pitch 10°) → "
            f"telafili okuma {math.degrees(_azg):.2f}°")

    # B29: GERÇEK manevra karesinde telafisiz okuma büyük sapar.
    # Geometri loglardan: manevra koşularında kutu kadrajda YUKARIDA duruyor
    # (cy medyan 261, p10 185) ve araç ANGLE_MAX'a (45°) dayanıyor. Hata
    # hedefin kadraj merkezinin ne kadar ÜSTÜNDE olduğuyla büyür:
    #     cy=300 (nişan) @40° → 3.3°   |   cy=240 @40° → 16.7°
    #     cy=220        @40° → ~19°    |   cy=200 @40° → 27.1°
    _cxm, _cym, _rollm = 350.0, 220.0, math.radians(40.0)
    _az_d, _ = ib.los_seviye(_cxm, _cym, _rollm, math.radians(5.0), C)
    _az_h = math.atan((_cxm - C.CX_NISAN) / geo.FX)
    _sapma = abs(math.degrees(_az_d - _az_h))
    kontrol("B29 telafisiz okuma yatışta büyük sapar (manevra bozulmasının kaynağı)",
            _sapma > 10.0,
            f"gerçek manevra karesi (cx={_cxm:.0f}, cy={_cym:.0f}, yatış 40°): "
            f"telafisiz ↔ telafili arasında {_sapma:.1f}° fark "
            f"(uçuş ölçümü 30-39° yatışta ort. 13.9°)")

    # B30: DÜZ UÇUŞ BOZULMAZ — kullanıcının doğrulanmış davranışı korunmalı.
    # roll≈0'da telafili ve telafisiz okuma pratikte aynı kalmalı.
    _enb_duz = 0.0
    for _cxd in (280.0, 300.0, 320.0, 340.0, 360.0):
        for _cyd in (260.0, 300.0, 340.0):
            _a, _ = ib.los_seviye(_cxd, _cyd, 0.0, math.radians(5.0), C)
            _h = math.atan((_cxd - C.CX_NISAN) / geo.FX)
            _enb_duz = max(_enb_duz, abs(math.degrees(_a - _h)))
    kontrol("B30 düz uçuşta (yatış≈0) telafi davranışı değiştirmez",
            _enb_duz < 2.5,
            f"roll=0, pitch=5°, tüm kadraj: en büyük fark {_enb_duz:.2f}° "
            f"(uçuş ölçümü: yatış<10°'de 0.6°). Kalan fark kamera 25° tilt "
            f"eşleniği — ham formül onu da yanlış yapıyordu")

    # B31: T1a YALNIZ YATAY — dikey komut bit bit aynı kalmalı (tek değişken)
    _arg = (350.0, 220.0, 40, 40, 0.3, 14.0, 0.05)
    _ile = ib.komut(*_arg, C, True, (0.2, 0.1), math.radians(5.0), 0.5, 2.0,
                    _rollm)
    _siz = ib.komut(*_arg, _RollYok, True, (0.2, 0.1), math.radians(5.0), 0.5,
                    2.0, _rollm)
    kontrol("B31 T1a dikey kanala DOKUNMAZ (vz birebir aynı)",
            abs(_ile[2] - _siz[2]) < 1e-12,
            f"yatış 40°'de vz telafili {_ile[2]:+.4f} = telafisiz "
            f"{_siz[2]:+.4f} m/s — dikey ayrı testin konusu (T1b)")

    # B32: geri dönüş yolu — AVCI_IBVS_ROLL=0 eski yatay yasayı aynen getirir
    kontrol("B32 roll telafisi kapatılabilir (eski yatay davranış geri gelir)",
            abs(_siz[5]["eps_yaw"] - _siz[5]["eps_yaw_ham"]) < 1e-12
            and abs(_ile[5]["eps_yaw"] - _ile[5]["eps_yaw_ham"]) > math.radians(10.0),
            "ROLL_TELAFI=False → eps_yaw = ham okuma; True iken 40° yatışta "
            f"{math.degrees(abs(_ile[5]['eps_yaw'] - _ile[5]['eps_yaw_ham'])):.1f}° ayrışır")

    print("=" * 60)
    # ── M3: LEAD ERKEN BAŞLASIN ──
    # Uçuş ölçümü (4473 kutulu kare, daire koşuları): lead 0-5 m dışında HER
    # bantta tam 0.0° — çünkü `if terminal:` kapısı 6.4 m'de kapanıyordu.
    # 8-13 m'de karelerin %88'i zaten aracın fiziksel tavanı (g·tan45 =
    # 9.81 m/s²) üstünde dönüş istiyor; düzeltmenin ucuz olduğu 13-35 m
    # bandında ise lead hiç yoktu.
    # UÇUŞ SONUCU: kapı kalkınca kadrajda tutuş düzeldi ama YAKLAŞMA bozuldu
    # (8 m'ye giriş 4→2 kez, lead karelerin %27'sinde 25° tavanında). Varsayılan
    # KAPALI; bu testler mekanizmayı ve geri/ileri anahtarı koruyor.
    class _LeadGec(ib.Cfg):
        LEAD_ERKEN = False

    class _LeadErk(ib.Cfg):
        LEAD_ERKEN = True

    # B33: terminal DEĞİLKEN lead artık uygulanabiliyor (kapı anahtarlı)
    _lyaw = 0.4
    _argL = (360.0, 300.0, 14, 10, _lyaw, 16.0, 0.05)   # boyut≈11.8 → ≈13 m
    _erk = ib.komut(*_argL, _LeadErk, False, (0.6, 0.0), 0.0, 0.0, 2.0, 0.0)
    _gec = ib.komut(*_argL, _LeadGec, False, (0.6, 0.0), 0.0, 0.0, 2.0, 0.0)
    kontrol("B33 lead terminal DIŞINDA da uygulanır (M3 kapısı kalktı)",
            abs(_erk[5]["lead_az"]) > math.radians(5.0)
            and abs(_gec[5]["lead_az"]) < 1e-12,
            f"13 m'de λ̇=0.6 rad/s: erken lead "
            f"{math.degrees(_erk[5]['lead_az']):.1f}° — eski kapıda 0.0° "
            f"(uçuşta ölçülen: 13-20 m bandında lead medyanı 0.0°)")

    # B34: DÜZ UÇUŞ BOZULMAZ — λ̇≈0 iken lead ≈ 0, yaw komutu aynı kalır
    _argD = (340.0, 300.0, 14, 10, _lyaw, 16.0, 0.05)
    _dz_e = ib.komut(*_argD, _LeadErk, False, (0.0, 0.0), 0.0, 0.0, 2.0, 0.0)
    _dz_g = ib.komut(*_argD, _LeadGec, False, (0.0, 0.0), 0.0, 0.0, 2.0, 0.0)
    kontrol("B34 düz uçuşta (λ̇=0) M3 yaw komutunu DEĞİŞTİRMEZ",
            abs(_dz_e[3] - _dz_g[3]) < 1e-12 and abs(_dz_e[0] - _dz_g[0]) < 1e-9,
            "λ̇=0 → lead = LEAD_SURE·0 = 0; yaw ve hız komutu bit bit aynı "
            "(kullanıcının düz uçuşta doğruladığı davranış korunur)")

    # B35: M3 YALNIZ YATAY — dikey komut bit bit aynı (tek değişken kuralı)
    kontrol("B35 M3 dikey kanala DOKUNMAZ (vz birebir aynı)",
            abs(_erk[2] - _gec[2]) < 1e-12,
            f"λ̇=0.6 rad/s'de vz erken {_erk[2]:+.4f} = geç {_gec[2]:+.4f} m/s "
            "— dikey lead terminal tutuşunda bırakıldı")

    # B36: tavan hâlâ geçerli — büyük λ̇ lead'i LEAD_MAX_DEG'de kesmeli
    _sat = ib.komut(*_argL, _LeadErk, False, (5.0, 0.0), 0.0, 0.0, 2.0, 0.0)
    kontrol("B36 erken lead tavanı aşmaz (LEAD_MAX_DEG)",
            abs(math.degrees(_sat[5]["lead_az"])) <= _LeadErk.LEAD_MAX_DEG + 1e-9,
            f"λ̇=5 rad/s (uçuşta görülen p90 4.81) → lead "
            f"{math.degrees(_sat[5]['lead_az']):.1f}° ≤ "
            f"{_LeadErk.LEAD_MAX_DEG:.0f}° tavan")

    # B37: geri dönüş yolu — AVCI_IBVS_LEAD_ERKEN=0 eski davranışı getirir
    _eski_t = ib.komut(*_argL, _LeadGec, True, (0.6, 0.0), 0.0, 0.0, 2.0, 0.0)
    kontrol("B37 M3 kapatılabilir (eski 'yalnız terminal' davranışı geri gelir)",
            abs(_gec[5]["lead_az"]) < 1e-12
            and abs(_eski_t[5]["lead_az"]) > math.radians(5.0),
            "LEAD_ERKEN=False → terminal dışında lead 0, terminalde "
            f"{math.degrees(_eski_t[5]['lead_az']):.1f}° (eski yol aynen)")

    print("=" * 60)
    # ── Ö1: KAÇIŞ TELAFİSİ ──
    # 10 kaçamak koşusunun İSTİSNASIZ hepsinde, kaçamaktan sonraki 15 s'de
    # drone 7.7-13.9 m/s'ye düşüyor (hedef 15.4-16.3) ve 48-147 m açılıyor.
    # Hız yasası saf menzil düzenleyicisi; "hedef uzaklaşıyor mu" girdisi yok.
    class _Kacis(ib.Cfg):
        KACIS_KD = 1.0

    _argK = (CX, C.CY_NISAN, 12, 12, 0.0, 8.0, 0.05)   # boyut 12 → uzak, seyir
    # ṙ<0 = hedef UZAKLAŞIYOR (kaçamak sonrası hâli)
    _kac = ib.komut(*_argK, _Kacis, False, (0.0, 0.0), 0.0, 0.0, -6.0, 0.0)
    _yok = ib.komut(*_argK, C, False, (0.0, 0.0), 0.0, 0.0, -6.0, 0.0)
    kontrol("B38 hedef uzaklaşırken (ṙ<0) kaçış telafisi hızı ARTIRIR",
            _kac[5]["v_los"] > _yok[5]["v_los"] + 3.0
            and abs(_kac[5]["kacis_ek"] - 6.0) < 1e-6,
            f"ṙ=-6 m/s: telafisiz {_yok[5]['v_los']:.1f} → telafili "
            f"{_kac[5]['v_los']:.1f} m/s (ek {_kac[5]['kacis_ek']:.1f})")

    # B39: YAKLAŞIRKEN terim SIFIR — fren yok (kullanıcı kararı: geri çekilme yok)
    _yak_k = ib.komut(*_argK, _Kacis, False, (0.0, 0.0), 0.0, 0.0, +6.0, 0.0)
    _yak_y = ib.komut(*_argK, C, False, (0.0, 0.0), 0.0, 0.0, +6.0, 0.0)
    kontrol("B39 yaklaşırken (ṙ>0) kaçış telafisi ASLA yavaşlatmaz",
            abs(_yak_k[5]["kacis_ek"]) < 1e-12
            and abs(_yak_k[5]["v_los"] - _yak_y[5]["v_los"]) < 1e-12,
            f"ṙ=+6 m/s: ek {_yak_k[5]['kacis_ek']:.2f}, v_los telafili "
            f"{_yak_k[5]['v_los']:.2f} = telafisiz {_yak_y[5]['v_los']:.2f} m/s")

    # B40: TERMİNAL hücum yasasına DOKUNMAZ (tek değişken)
    _t_k = ib.komut(CX, C.CY_NISAN, 30, 30, 0.0, 8.0, 0.05, _Kacis, True,
                    (0.0, 0.0), 0.0, 0.0, -6.0, 0.0)
    kontrol("B40 kaçış telafisi TERMİNALE dokunmaz (v = V_TERMINAL)",
            abs(_t_k[5]["v_los"] - C.V_TERMINAL) < 1e-9
            and abs(_t_k[5]["kacis_ek"]) < 1e-12,
            f"terminalde ṙ=-6 olsa bile v_los {_t_k[5]['v_los']:.1f} = "
            f"V_TERMINAL {C.V_TERMINAL:.1f} m/s")

    # B41: DİKEY kanala dokunmaz
    kontrol("B41 kaçış telafisi dikey komutu DEĞİŞTİRMEZ",
            abs(_kac[2] - _yok[2]) < 1e-12,
            f"vz telafili {_kac[2]:+.4f} = telafisiz {_yok[2]:+.4f} m/s")

    # B42: tavan bağlar + kapatılabilir
    _buyuk = ib.komut(*_argK, _Kacis, False, (0.0, 0.0), 0.0, 0.0, -30.0, 0.0)
    kontrol("B42 terim KACIS_MAX ile sınırlı ve AVCI_IBVS_KD=0 ile kapanır",
            abs(_buyuk[5]["kacis_ek"] - _Kacis.KACIS_MAX) < 1e-9
            and abs(_yok[5]["kacis_ek"]) < 1e-12,
            f"ṙ=-30 → ek {_buyuk[5]['kacis_ek']:.1f} m/s "
            f"(tavan {_Kacis.KACIS_MAX:.0f}); KD=0 → ek 0.00")

    print("=" * 60)
    # ── Ö8: YANAL KOMUT AÇIYLA DEĞİL, KAÇIRMA MESAFESİYLE ──
    # Ölçüldü (O7A, 1.5 m): eps_yaw 58°, yaw komutu 120°/s'de doymuş.
    # 1.5 m'de 58° = yalnız 1.3 m yanal kaçırma; güdüm mesafeyi değil açıyı
    # görüyordu. Ö8 hız vektörünü kaçırma mesafesine göre ölçekler.
    class _Yanal(ib.Cfg):
        YANAL_K = 3.0

    # YAKIN + BÜYÜK AÇI: kutu 105 px (≈1.5 m), hedef kadrajın sağında
    _cx_uzak = CX + 300.0                       # eps_yaw ≈ 61°
    _argY = (_cx_uzak, C.CY_NISAN, 105, 105, 0.0, 18.0, 0.05)
    _y_ac = ib.komut(*_argY, _Yanal, True, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
    _y_ka = ib.komut(*_argY, C, True, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
    _ac_deg = math.degrees(_y_ac[5]["eps_hiz"])
    _ka_deg = math.degrees(_y_ka[5]["eps_hiz"])
    kontrol("B43 yakında Ö8 hız komutunu KISAR (58° slam biter)",
            abs(_ac_deg) < abs(_ka_deg) * 0.7,
            f"1.5 m'de hız yönü: kapalı {_ka_deg:.0f}° → açık {_ac_deg:.0f}° "
            f"(uçuşta ölçülen slam: 58°, yaw komutu 120°/s'de doymuş)")

    # B44: BURUN kısılmaz — kamera hedefi izlemeye devam etmeli
    kontrol("B44 Ö8 BURNU kısmaz (kamera hedefi kaybetmesin)",
            abs(_y_ac[3] - _y_ka[3]) < 1e-9,
            f"yaw komutu açık {math.degrees(_y_ac[3]):.1f}° = "
            f"kapalı {math.degrees(_y_ka[3]):.1f}° — burun tam hedefte")

    # B45: UZAKTA aşırı kısmamalı (25 m tetikte 4/6 isabet var, bozulmasın)
    _argU = (CX + 60.0, C.CY_NISAN, 8, 8, 0.0, 18.0, 0.05)   # ≈20 m, eps≈20°
    _u_ac = ib.komut(*_argU, _Yanal, False, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
    _u_ka = ib.komut(*_argU, C, False, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
    _oran = (abs(_u_ac[5]["eps_hiz"]) / max(abs(_u_ka[5]["eps_hiz"]), 1e-9))
    kontrol("B45 uzakta Ö8 HİÇ kısmaz (menzil kapısı — 25 m tetik bozulmasın)",
            _oran > 0.999,
            f"20 m'de hız yönü kapalıya oranı %{100*_oran:.0f} "
            f"({math.degrees(_u_ka[5]['eps_hiz']):.1f}° → "
            f"{math.degrees(_u_ac[5]['eps_hiz']):.1f}°)")

    # B46: ASLA BÜYÜTMEZ — yalnız kısan bir sınır
    _buyutme = False
    for _cxt in (CX + 20, CX + 100, CX + 200, CX + 300):
        for _bt in (8, 20, 60, 105):
            _a = ib.komut(_cxt, C.CY_NISAN, _bt, _bt, 0.0, 18.0, 0.05,
                          _Yanal, True, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
            _k = ib.komut(_cxt, C.CY_NISAN, _bt, _bt, 0.0, 18.0, 0.05,
                          C, True, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
            if abs(_a[5]["eps_hiz"]) > abs(_k[5]["eps_hiz"]) + 1e-9:
                _buyutme = True
    kontrol("B46 Ö8 komutu ASLA büyütmez (yalnız kısan sınır)",
            not _buyutme, "16 kombinasyonda hiçbirinde büyütme yok")

    # B47: kapatılabilir — AVCI_IBVS_YANAL=0 eski davranışı aynen getirir
    kontrol("B47 Ö8 kapatılabilir (varsayılan KAPALI)",
            abs(C.YANAL_K) < 1e-9
            and abs(_y_ka[5]["eps_hiz"] - _y_ka[5]["eps_yaw"]) < 1e-12,
            f"YANAL_K={C.YANAL_K} → eps_hiz = eps_yaw (bit bit aynı)")

    print("=" * 60)
    # ── Ö9: YATAY KANALA SÖNÜMLEME (D terimi) ──
    # Kullanıcı uçuşu 185753: hafif bir manevrada (aileron 1733) araç 4.3 m'den
    # 25 m'ye savruldu, arada iki kez gidip geldi. Yatay kanal SAF ORANSAL;
    # gecikmeli sistemde saf-P zorunlu olarak salınır.
    class _Sonum(ib.Cfg):
        SONUM_T = 0.30

    _argS = (CX + 100.0, C.CY_NISAN, 20, 20, 0.0, 14.0, 0.05)
    # araç SAĞA dönüyor (yaw_hizi>0) ve hedef de sağda → komut GERİ ÇEKİLMELİ
    _s_var = ib.komut(*_argS, _Sonum, False, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0, 0.8)
    _s_yok = ib.komut(*_argS, C, False, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0, 0.8)
    kontrol("B48 araç zaten dönüyorken sönümleme komutu GERİ ÇEKER",
            _s_var[3] < _s_yok[3] - math.radians(5.0),
            f"yaw hızı 0.8 rad/s: sönümsüz {math.degrees(_s_yok[3]):.1f}° → "
            f"sönümlü {math.degrees(_s_var[3]):.1f}° "
            f"(fark {math.degrees(_s_yok[3]-_s_var[3]):.1f}°)")

    # B49: araç DÖNMÜYORSA (yaw_hizi=0) sönümleme etkisiz — düz uçuş korunur
    _d_var = ib.komut(*_argS, _Sonum, False, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0, 0.0)
    _d_yok = ib.komut(*_argS, C, False, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0, 0.0)
    kontrol("B49 araç dönmüyorken (ω=0) sönümleme yaw'ı DEĞİŞTİRMEZ",
            abs(_d_var[3] - _d_yok[3]) < 1e-12,
            "düz takipte ω≈0 → sönümleme terimi 0; kullanıcının doğruladığı "
            "düz uçuş davranışı bit bit korunur")

    # B50: TERS yönde dönerken komutu İLERİ iter (simetrik, tek yönlü değil)
    _t_var = ib.komut(*_argS, _Sonum, False, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0, -0.8)
    kontrol("B50 sönümleme simetriktir (ters dönüşte komutu ileri iter)",
            _t_var[3] > _s_yok[3] + math.radians(5.0),
            f"yaw hızı −0.8 rad/s → {math.degrees(_t_var[3]):.1f}° "
            f"(sönümsüz {math.degrees(_s_yok[3]):.1f}°)")

    # B51: tavan bağlar — sönümleme komutu ters yöne çevirmesin
    _b_var = ib.komut(*_argS, _Sonum, False, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0, 9.0)
    kontrol("B51 sönümleme SONUM_MAX_DEG tavanında kesilir",
            abs(math.degrees(_b_var[5]["sonum"])) <= _Sonum.SONUM_MAX_DEG + 1e-9,
            f"ω=9 rad/s → sönüm {math.degrees(_b_var[5]['sonum']):.1f}° "
            f"≤ {_Sonum.SONUM_MAX_DEG:.0f}° tavan")

    # B52: kapatılabilir
    kontrol("B52 Ö9 kapatılabilir (varsayılan KAPALI)",
            abs(C.SONUM_T) < 1e-9 and abs(_s_yok[5]["sonum"]) < 1e-12,
            f"SONUM_T={C.SONUM_T} → sönüm terimi 0.00°")

    print("=" * 60)
    # ── Ö5: DÖNÜŞ-FARKINDA HIZ TAVANI ──
    # Kullanıcı ölçütüyle bulundu: SAĞA AŞIM 8-47 m (drone hedefin YANINA
    # savruluyor), "önde %" ~0. Yarıçap R=V²/a; 18 m/s'de 33 m, hedef 13 m.
    # Tek kaldıraç hızı kısmak (R, V'nin KARESİYLE düşer).
    class _Donus(ib.Cfg):
        DONUS_A = 9.0

    # DÖNÜŞTE (λ̇ büyük) hız kısılmalı: λ̇=1.2 → tavan 9.0/1.2 = 7.5 → taban 10
    _argD5 = (CX + 40.0, C.CY_NISAN, 20, 20, 0.0, 20.0, 0.05)
    _d_ac = ib.komut(*_argD5, _Donus, True, (1.2, 0.0), 0.0, 0.0, 3.0, 0.0)
    _d_ka = ib.komut(*_argD5, C, True, (1.2, 0.0), 0.0, 0.0, 3.0, 0.0)
    kontrol("B53 sert dönüşte (λ̇=1.2) Ö5 hızı KISAR",
            _d_ac[5]["v_los"] < _d_ka[5]["v_los"] - 3.0,
            f"λ̇=1.2 rad/s: tavansız {_d_ka[5]['v_los']:.1f} → "
            f"tavanlı {_d_ac[5]['v_los']:.1f} m/s "
            f"(yarıçap {_d_ka[5]['v_los']**2/9.81:.0f} → "
            f"{_d_ac[5]['v_los']**2/9.81:.0f} m)")

    # B54: DÜZ UÇUŞTA (λ̇≈0) etkisiz — kullanıcının doğruladığı davranış
    _z_ac = ib.komut(*_argD5, _Donus, True, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
    _z_ka = ib.komut(*_argD5, C, True, (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
    kontrol("B54 düz uçuşta (λ̇≈0) Ö5 hızı DEĞİŞTİRMEZ",
            abs(_z_ac[5]["v_los"] - _z_ka[5]["v_los"]) < 1e-12
            and _z_ac[5]["donus_tavan"] is None,
            f"λ̇=0 → tavan uygulanmadı, v_los {_z_ac[5]['v_los']:.1f} m/s "
            "(düz uçuş davranışı bit bit korunur)")

    # B55: TABAN bağlar — hedeften tamamen kopmayalım
    _b_ac = ib.komut(*_argD5, _Donus, True, (5.0, 0.0), 0.0, 0.0, 3.0, 0.0)
    kontrol("B55 Ö5 hızı DONUS_V_MIN altına indirmez",
            _b_ac[5]["v_los"] >= _Donus.DONUS_V_MIN - 1e-9,
            f"λ̇=5 rad/s (tavan {_Donus.DONUS_A/5:.1f}) → v_los "
            f"{_b_ac[5]['v_los']:.1f} ≥ taban {_Donus.DONUS_V_MIN:.0f} m/s")

    # B56: ASLA HIZLANDIRMAZ — yalnız kısan bir tavan
    _hizli = False
    for _lam in (0.05, 0.2, 0.5, 1.0, 2.0):
        for _b in (8, 20, 40):
            _a = ib.komut(CX + 40.0, C.CY_NISAN, _b, _b, 0.0, 20.0, 0.05,
                          _Donus, True, (_lam, 0.0), 0.0, 0.0, 3.0, 0.0)
            _k = ib.komut(CX + 40.0, C.CY_NISAN, _b, _b, 0.0, 20.0, 0.05,
                          C, True, (_lam, 0.0), 0.0, 0.0, 3.0, 0.0)
            if _a[5]["v_los"] > _k[5]["v_los"] + 1e-9:
                _hizli = True
    kontrol("B56 Ö5 hızı ASLA artırmaz (yalnız kısan tavan)",
            not _hizli, "15 kombinasyonda hiçbirinde hızlanma yok")

    # B57: kapatılabilir
    kontrol("B57 Ö5 kapatılabilir (varsayılan KAPALI)",
            abs(C.DONUS_A) < 1e-9 and _d_ka[5]["donus_tavan"] is None,
            f"DONUS_A={C.DONUS_A} → tavan hiç uygulanmaz")

    print("=" * 60)
    # ── T1b: DİKEY KANALDA ROLL/PITCH TELAFİSİ ──
    # Gece ölçümü: kesişim 10-40 cm'ye çözülüyor, isabet/ıska farkı DİKEYDE.
    # Zarf yatayda ±0.65 m ama dikeyde +0.29/−0.13 m — 5 kat dar.
    class _DikeyT(ib.Cfg):
        DIKEY_ROLL = True

    # DÜZ UÇUŞ: roll=pitch=0'da telafi dikey hatayı DEĞİŞTİRMEMELİ
    _enb = 0.0
    for _cyd in (200.0, 260.0, 300.0, 340.0, 400.0):
        _a = ib.komut(CX, _cyd, 20, 20, 0.0, 12.0, 0.05, _DikeyT, False,
                      (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
        _k = ib.komut(CX, _cyd, 20, 20, 0.0, 12.0, 0.05, C, False,
                      (0.0, 0.0), 0.0, 0.0, 3.0, 0.0)
        _enb = max(_enb, abs(_a[2] - _k[2]))
    kontrol("B58 T1b düz uçuşta (roll=pitch=0) dikey komutu DEĞİŞTİRMEZ",
            _enb < 0.02,
            f"tüm kadrajda en büyük vz farkı {_enb:.4f} m/s "
            "(kullanıcının doğruladığı dikey davranış korunur)")

    # YATIŞTA: okuma ciddi biçimde DEĞİŞMELİ (uçuşta 33° sapma ölçülmüştü)
    _rl = math.radians(30.0)
    _ay = ib.komut(350.0, 234.0, 20, 20, 0.0, 12.0, 0.05, _DikeyT, False,
                   (0.0, 0.0), math.radians(5.0), 0.0, 3.0, _rl)
    _ky = ib.komut(350.0, 234.0, 20, 20, 0.0, 12.0, 0.05, C, False,
                   (0.0, 0.0), math.radians(5.0), 0.0, 3.0, _rl)
    _fark = math.degrees(abs(_ay[5]["eps_elev"] - _ky[5]["eps_elev"]))
    # ⚠ ÖNCEKİ İDDİA DÜZELTİLDİ: "dikeyde 33° sapma" iki FARKLI büyüklüğü
    # kıyaslıyordu (seviye yükselişi ↔ piksel farkı hatası) — geçersizdi.
    # Gerçek roll kaynaklı sapma hedefin kadraj merkezinden uzaklığıyla büyür:
    #   yatış 30°: cx=320'de 3.5° | cx=420'de 18.0° | cx=500'de 23.5°
    # Terminal yaklaşmada hedef merkeze yakın (|cx−320| medyan 14-18 px) ve
    # araç neredeyse düz (yatış medyan 4°) olduğu için gerçek uçuş
    # kayıtlarında düzeltme MEDYANI −0.06° çıktı — pratikte sıfır.
    kontrol("B59 T1b yatışta dikey okumayı düzeltir (etki cx ile büyür)",
            _fark > 3.0,
            f"yatış 30°, cx=350, cy=234: ham {math.degrees(_ky[5]['eps_elev']):+.1f}° → "
            f"telafili {math.degrees(_ay[5]['eps_elev']):+.1f}° (fark {_fark:.1f}°). "
            "⚠ Terminal fazda gerçek düzeltme medyanı −0.06° — bu özellik "
            "santimetrelik dikey ıskanın çaresi DEĞİL (uçmadan ölçüldü)")

    # T1b YATAY kanala dokunmamalı (tek değişken)
    kontrol("B60 T1b YATAY kanala DOKUNMAZ (yaw komutu birebir aynı)",
            abs(_ay[3] - _ky[3]) < 1e-12,
            f"yaw telafili {math.degrees(_ay[3]):.2f}° = "
            f"telafisiz {math.degrees(_ky[3]):.2f}°")

    # kapatılabilir
    kontrol("B61 T1b kapatılabilir (varsayılan KAPALI)",
            not C.DIKEY_ROLL
            and abs(_ky[5]["eps_elev"] - _ky[5]["eps_elev_ham"]) < 1e-12,
            f"DIKEY_ROLL={C.DIKEY_ROLL} → eps_elev = ham okuma")

    print("=" * 60)
    # ── Ö11: ISKA SONRASI DÖNÜŞ İÇİN YAVAŞLAMA ──
    # Ölçüldü: aşım her koşuda tetikten +7 s sonra, 66-69 m = 2R (minimum
    # dönüş çemberi). Kazanç değil GEOMETRİ; çare hızı kısmak (R ∝ V²).
    class _DY(ib.Cfg):
        DONUS_YAVAS = 9.0

    _uzak = CX + 300.0          # eps_yaw ≈ 61° → dönmemiz gerekiyor
    _yakin_aci = CX + 40.0      # eps_yaw ≈ 13.5° → hedefe nişanlıyız
    _argDY = (_uzak, C.CY_NISAN, 12, 12, 0.0, 16.0, 0.05)

    # GEÇTİK + DÖNMEMİZ GEREK → hız kısılmalı
    _g_ac = ib.komut(*_argDY, _DY, False, (0.0, 0.0), 0.0, 0.0, -10.0, 0.0)
    _g_ka = ib.komut(*_argDY, C, False, (0.0, 0.0), 0.0, 0.0, -10.0, 0.0)
    _R = lambda v: v * v / 9.81
    kontrol("B62 hedefi geçince (ṙ<−5) ve dönüş gerekince Ö11 hızı KISAR",
            _g_ac[5]["v_los"] < _g_ka[5]["v_los"] - 3.0
            and _g_ac[5]["donus_yavas"],
            f"ṙ=−10, açı 61°: {_g_ka[5]['v_los']:.1f} → {_g_ac[5]['v_los']:.1f} m/s "
            f"(U-dönüşü 2R: {2*_R(_g_ka[5]['v_los']):.0f} → "
            f"{2*_R(_g_ac[5]['v_los']):.0f} m — uçuşta ölçülen aşım 66 m)")

    # YAKLAŞIRKEN (ṙ>0) etkisiz — düz takip bozulmaz
    _y_ac = ib.komut(*_argDY, _DY, False, (0.0, 0.0), 0.0, 0.0, +3.0, 0.0)
    _y_ka = ib.komut(*_argDY, C, False, (0.0, 0.0), 0.0, 0.0, +3.0, 0.0)
    kontrol("B63 yaklaşırken (ṙ>0) Ö11 ETKİSİZ (düz takip bozulmaz)",
            abs(_y_ac[5]["v_los"] - _y_ka[5]["v_los"]) < 1e-12
            and not _y_ac[5]["donus_yavas"],
            f"ṙ=+3 → hız {_y_ac[5]['v_los']:.1f} m/s, kısma yok")

    # HEDEFE NİŞANLIYKEN (küçük açı) serbest bırakır — kendiliğinden çıkış
    _n_ac = ib.komut(_yakin_aci, C.CY_NISAN, 12, 12, 0.0, 16.0, 0.05,
                     _DY, False, (0.0, 0.0), 0.0, 0.0, -10.0, 0.0)
    _n_ka = ib.komut(_yakin_aci, C.CY_NISAN, 12, 12, 0.0, 16.0, 0.05,
                     C, False, (0.0, 0.0), 0.0, 0.0, -10.0, 0.0)
    kontrol("B64 hedefe nişan alınca (açı<45°) Ö11 KENDİLİĞİNDEN bırakır",
            abs(_n_ac[5]["v_los"] - _n_ka[5]["v_los"]) < 1e-12
            and not _n_ac[5]["donus_yavas"],
            f"açı 13.5° → hız {_n_ac[5]['v_los']:.1f} m/s, durum tutulmuyor "
            "(dönüş ilerledikçe kendiliğinden serbest)")

    # ASLA HIZLANDIRMAZ
    _hizlandi = False
    for _cxq in (CX + 40, CX + 150, CX + 300):
        for _bq in (8, 20, 40):
            for _kq in (-12.0, -6.0, 0.0, 4.0):
                _a = ib.komut(_cxq, C.CY_NISAN, _bq, _bq, 0.0, 16.0, 0.05,
                              _DY, False, (0.0, 0.0), 0.0, 0.0, _kq, 0.0)
                _k = ib.komut(_cxq, C.CY_NISAN, _bq, _bq, 0.0, 16.0, 0.05,
                              C, False, (0.0, 0.0), 0.0, 0.0, _kq, 0.0)
                if _a[5]["v_los"] > _k[5]["v_los"] + 1e-9:
                    _hizlandi = True
    kontrol("B65 Ö11 hızı ASLA artırmaz (yalnız kısan tavan)",
            not _hizlandi, "36 kombinasyonda hiçbirinde hızlanma yok")

    kontrol("B66 Ö11 kapatılabilir (varsayılan KAPALI)",
            abs(C.DONUS_YAVAS) < 1e-9 and not _g_ka[5]["donus_yavas"],
            f"DONUS_YAVAS={C.DONUS_YAVAS} → hiç devreye girmez")

    print("=" * 60)
    # ── Ö12: YAKIN MENZİLDE YAW SLEW TAVANI (kendi ekseninde dönme çaresi) ──
    # 30 koşunun 10'unda KURTARMA tetiklenmiş; öncesinde yaw komutu tavanda
    # sürekli kaçıyor (122/118/122 °/s) ve aracın gerçek yaw hızı 300°/s'yi
    # aşıyor. Menzil küçüldükçe hedefin açısal hızı 1/R ile patlıyor.
    class _YawM(ib.Cfg):
        YAW_MENZIL_REF = 15.0

    # B67 — YAPISAL GARANTİ: yaw slew sınırı komut() çıktısına HİÇ DOKUNMAZ.
    # Hız vektörü ve nişan yönü bu sınırdan geçmez; sınır yalnız döngüdeki
    # BURUN slew'ine uygulanır. Bu, "başka durumu bozmama" şartının kanıtı.
    _bozdu = False
    for _cxz in (CX - 200, CX, CX + 120, CX + 300):
        for _bz in (8, 25, 60, 100):
            for _tz in (False, True):
                _a = ib.komut(_cxz, C.CY_NISAN, _bz, _bz, 0.3, 14.0, 0.05,
                              _YawM, _tz, (0.5, 0.1), 0.0, 0.0, 2.0, 0.2)
                _k = ib.komut(_cxz, C.CY_NISAN, _bz, _bz, 0.3, 14.0, 0.05,
                              C, _tz, (0.5, 0.1), 0.0, 0.0, 2.0, 0.2)
                if any(abs(_a[i] - _k[i]) > 1e-12 for i in range(4)):
                    _bozdu = True
    kontrol("B67 Ö12 komut() çıktısını HİÇ DEĞİŞTİRMEZ (hız vektörü korunur)",
            not _bozdu,
            "32 kombinasyonda vx/vy/vz/yaw bit bit aynı — sınır yalnız "
            "döngüdeki BURUN slew'ine uygulanır, uçuş yolu etkilenmez")

    # B68 — tavan menzille ölçeklenir: yakında kısılır
    def _tavan(cfg, menzil):
        b = ib.Cfg.MENZIL_PX_M / menzil
        t = cfg.YAW_RATE_MAX_DEG
        if cfg.YAW_MENZIL_REF > 0.0:
            t *= max(cfg.YAW_MIN_KAT, min(menzil / cfg.YAW_MENZIL_REF, 1.0))
        return t
    _t20, _t8, _t3 = (_tavan(_YawM, 20.0), _tavan(_YawM, 8.0),
                      _tavan(_YawM, 3.0))
    kontrol("B68 yaw tavanı yakın menzilde KISILIR",
            _t8 < _YawM.YAW_RATE_MAX_DEG * 0.8 and _t3 < _t8 + 1e-9,
            f"20 m → {_t20:.0f}°/s   8 m → {_t8:.0f}°/s   3 m → {_t3:.0f}°/s "
            f"(taban {_YawM.YAW_MIN_KAT:.2f}× = {_YawM.YAW_RATE_MAX_DEG*_YawM.YAW_MIN_KAT:.0f}°/s)")

    # B69 — UZAK menzilde tavan AYNEN kalır (normal takip bozulmaz)
    kontrol("B69 uzak menzilde (R ≥ ref) yaw tavanı DEĞİŞMEZ",
            abs(_t20 - _YawM.YAW_RATE_MAX_DEG) < 1e-9,
            f"20 m ≥ ref 15 m → tavan {_t20:.0f}°/s = varsayılan "
            f"{_YawM.YAW_RATE_MAX_DEG:.0f}°/s (uzak takip aynen)")

    # B70 — kapatılabilir
    kontrol("B70 Ö12 kapatılabilir (varsayılan KAPALI)",
            abs(C.YAW_MENZIL_REF) < 1e-9
            and abs(_tavan(C, 3.0) - C.YAW_RATE_MAX_DEG) < 1e-9,
            f"YAW_MENZIL_REF={C.YAW_MENZIL_REF} → 3 m'de bile tavan "
            f"{_tavan(C, 3.0):.0f}°/s, hiç kısılmaz")

    # ══════════════════════════════════════════════════════════════════

    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
