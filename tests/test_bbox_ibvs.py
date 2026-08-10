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

    # ── B19: LEAD yalnız TERMİNALDE ve LOS dönüyorken ──
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

    # ══ PN YATAY LEAD KİLL-SWITCH (2026-08-09, arastirma Aday #2) ══
    # Kök bulgu: los_hiz[0] = d/dt[iyaw + atan((cx−CX)/FX)] ZATEN ego-temiz
    # (atalet LOS hızı). Kill-switch KAPALI (varsayılan) → native lead bit-aynı;
    # AÇIK → PN lead (N·λ̇·t_go) yatay/yaw kanalında, dikey vz'ye DOKUNMAZ.
    class _PNoff(ib.Cfg):
        PN = False                 # varsayılanla aynı; açıkça
    class _PNon(ib.Cfg):
        PN = True
        PN_N = 3.0
        PN_TGO_MAX = 3.0
        PN_KAP_MIN = 1.5

    # Crosser geometrisi: hedef merkezde (yaw hatası izole edilsin), yatay LOS
    # DÖNÜYOR (los_hiz[0]>0), kapanma 3 m/s, kutu 18px (≈8.9 m — ölçülen
    # terminal-latch menzili). Native lead vs PN lead karşılaştır.
    LAM = 0.3                       # rad/s atalet yatay LOS hızı (gerçek crosser)
    _off = ib.komut(CX, C.CY_NISAN, 18, 18, 0.0, 10.0, 0.05, _PNoff, True,
                    (LAM, 0.0), 0.0, 0.0, 3.0)[5]
    _on = ib.komut(CX, C.CY_NISAN, 18, 18, 0.0, 10.0, 0.05, _PNon, True,
                   (LAM, 0.0), 0.0, 0.0, 3.0)[5]

    # ── B28: kill-switch KAPALI → native lead yolu (kaynak='native') ──
    # Native = LEAD_SURE·lead_olcek·λ̇. Değer, kill-switch eklemeden ÖNCEKİ
    # formülle bire bir aynı olmalı (bit-aynı native davranış).
    _native_beklenen = max(-math.radians(C.LEAD_MAX_DEG),
        min(math.radians(C.LEAD_MAX_DEG),
            C.LEAD_SURE * min(1.0, C.BOYUT_REF / 18.0) * LAM))
    kontrol("B28 PN kapalı → native lead formülü bit-aynı (kaynak=native)",
            _off["lead_kaynak"] == "native"
            and abs(_off["lead_az"] - _native_beklenen) < 1e-12,
            f"native lead={math.degrees(_off['lead_az']):.3f}° "
            f"(beklenen {math.degrees(_native_beklenen):.3f}°)")

    # ── B29: PN AÇIK crosser'da lead DAHA BÜYÜK ve aynı işarette ──
    # PN = N·λ̇·t_go; t_go = menzil/kapanma = (160/18)/3 ≈ 2.96 s → N·λ̇·t_go
    # tavana (25°) oturur. Native ≈ 6.9°. PN, native'den belirgin büyük.
    kontrol("B29 PN açık: crosser'da yatay lead native'den BÜYÜK, aynı işaret",
            _on["lead_kaynak"] == "pn"
            and _on["lead_az"] > _off["lead_az"] > 0.0
            and abs(_on["lead_az"]) > abs(_off["lead_az"]) * 1.5,
            f"native {math.degrees(_off['lead_az']):.2f}° → "
            f"PN {math.degrees(_on['lead_az']):.2f}° "
            f"({math.degrees(_on['lead_az'])/max(math.degrees(_off['lead_az']),1e-6):.1f}×)")

    # ── B30: PN lead İŞARETİ LOS dönüş yönüyle uyumlu (sol crosser → sol lead) ──
    _on_neg = ib.komut(CX, C.CY_NISAN, 18, 18, 0.0, 10.0, 0.05, _PNon, True,
                       (-LAM, 0.0), 0.0, 0.0, 3.0)[5]
    kontrol("B30 PN lead işareti λ̇ ile döner (λ̇<0 → lead<0)",
            _on_neg["lead_az"] < 0.0
            and abs(_on_neg["lead_az"] + _on["lead_az"]) < 1e-9,
            f"λ̇=+{LAM} → {math.degrees(_on['lead_az']):+.1f}°, "
            f"λ̇=−{LAM} → {math.degrees(_on_neg['lead_az']):+.1f}°")

    # ── B31: PN lead LEAD_MAX_DEG ile tavanlı (gürültülü λ̇ savurmasın) ──
    _on_big = ib.komut(CX, C.CY_NISAN, 18, 18, 0.0, 10.0, 0.05, _PNon, True,
                       (5.0, 0.0), 0.0, 0.0, 3.0)[5]     # λ̇=5 rad/s absürt
    kontrol("B31 PN lead LEAD_MAX_DEG tavanında kalır",
            abs(_on_big["lead_az"]) <= math.radians(C.LEAD_MAX_DEG) + 1e-9,
            f"λ̇=5 rad/s → {math.degrees(_on_big['lead_az']):.1f}° "
            f"≤ {C.LEAD_MAX_DEG:.0f}°")

    # ── B32: PN yalnız YATAY/YAW kanalını değiştirir — DİKEY vz AYNEN KALIR ──
    # Aynı geometri, PN off vs on: vz komutu (index 2) bit-aynı olmalı.
    cy_ust_pn = geo.CY + geo.FY * math.tan(math.radians(15))   # hedef 10° yukarıda
    _vz_off = ib.komut(CX, cy_ust_pn, 18, 18, 0.0, 10.0, 0.05, _PNoff, True,
                       (LAM, 0.4), 0.0, 0.0, 3.0)[2]
    _vz_on = ib.komut(CX, cy_ust_pn, 18, 18, 0.0, 10.0, 0.05, _PNon, True,
                      (LAM, 0.4), 0.0, 0.0, 3.0)[2]
    kontrol("B32 PN dikey vz'ye DOKUNMAZ (yalnız yaw/yatay kanal)",
            abs(_vz_off - _vz_on) < 1e-12,
            f"vz PN-off {_vz_off:+.3f} = PN-on {_vz_on:+.3f} m/s")

    # ── B33: EGO-DÜZELTME kancası — los_hiz[0] zaten ego-temiz; PN_EGO çift-
    # çıkarma yapar. Kanca AÇIKKEN yaw_rate=λ̇ verilince lead ~0'a düşmeli
    # (çift-düzeltme sinyali sıfırlar) → los_hiz[0]'ın gerçekten ego-temiz
    # olduğunun ispatı: ondan yaw_rate çıkınca crosser sinyali kaybolur.
    class _PNego(_PNon):
        PN_EGO = True
    _ego = ib.komut(CX, C.CY_NISAN, 18, 18, 0.0, 10.0, 0.05, _PNego, True,
                    (LAM, 0.0), 0.0, 0.0, 3.0, LAM)[5]      # yaw_rate=LAM
    kontrol("B33 PN_EGO çift-çıkarma: yaw_rate=λ̇ → lead≈0 (los_hiz ego-temiz kanıtı)",
            abs(_ego["lead_az"]) < 1e-9,
            f"PN_EGO=1, yaw_rate=λ̇=λ̇ → lead {math.degrees(_ego['lead_az']):.3f}° "
            f"(los_hiz[0] ZATEN ego-temiz olduğu için çift-çıkarma sıfırlıyor)")

    # ══ KESTİRİM + COAST KİLL-SWITCH (PRED) (2026-08-09, arastirma Aday #3/#5) ══
    # Ölçülen problem: hedef-kaybı %73.7, kör hücumda son komut 2 s DONULUYORDU.
    # PRED KAPALI (varsayılan) → komut yolu + kör davranış bit-aynı; PRED AÇIK →
    # görüntü-düzlemi (cx,cy) kestirimiyle coast (donmuş yerine ilerleyen nişan)
    # + normal takipte küçük görüntü-lead. Dikey/yatay YASAYA dokunulmaz.
    class _PREDon(ib.Cfg):
        PRED = True
        PRED_MAXS = 0.6
        PRED_LEAD = 0.3
        PRED_ALPHA = 0.5
        PRED_BETA = 0.1

    # ── B34: PRED KAPALI → komut() yolu BİT-AYNI (kestirim yalnız döngüde) ──
    # komut imzası değişmedi; PRED off cfg ile default cfg özdeş sonuç vermeli.
    class _PREDoff(ib.Cfg):
        PRED = False
    _a = ib.komut(CX + 60, C.CY_NISAN + 40, 30, 30, 0.1, 8.0, 0.05, C,
                  False, (0.2, 0.1), 0.05, 1.0, 2.0, 0.0)
    _b = ib.komut(CX + 60, C.CY_NISAN + 40, 30, 30, 0.1, 8.0, 0.05, _PREDoff,
                  False, (0.2, 0.1), 0.05, 1.0, 2.0, 0.0)
    kontrol("B34 PRED kapalı → komut() çıktısı bit-aynı (yasa değişmedi)",
            _a[:5] == _b[:5] and _a[5]["lead_az"] == _b[5]["lead_az"],
            f"vx/vy/vz/yaw/I ve lead_az özdeş (PRED yalnız döngüde beslemeyi kaydırır)")

    # ── B35: KESTİRİCİ hareketli hedefin bir sonraki konumunu DOĞRU tahminler ──
    # Sabit hızla (px/frame) ilerleyen hedefe estimator'ı besle; ölçümsüz ileri-
    # tahmin gerçek yörüngeye yakınsamalı (alpha-beta hızı öğrenir).
    est = ib.HedefKestirim(_PREDon)
    _dt = 0.05
    _vx_px, _vy_px = 40.0, -20.0      # px/s (gerçek hedef görüntü hızı)
    cx_t, cy_t = 300.0, 260.0
    for _k in range(12):              # 12 kare besle → hız otursun
        cx_t += _vx_px * _dt
        cy_t += _vy_px * _dt
        est.guncelle(cx_t, cy_t, _dt)
    v_hata = math.hypot(est.vcx - _vx_px, est.vcy - _vy_px)
    # bir sonraki karenin GERÇEK konumu vs estimator ileri-tahmini
    cx_gercek, cy_gercek = cx_t + _vx_px * _dt, cy_t + _vy_px * _dt
    cx_tah, cy_tah = est.tahmin_ileri(_dt)
    tah_hata = math.hypot(cx_tah - cx_gercek, cy_tah - cy_gercek)
    kontrol("B35 estimator hareketli hedefin hızını+sonraki konumunu doğru kestirir",
            v_hata < 3.0 and tah_hata < 3.0,
            f"öğrenilen hız ({est.vcx:.1f},{est.vcy:.1f}) vs gerçek "
            f"({_vx_px:.0f},{_vy_px:.0f}) px/s; ileri-tahmin hatası {tah_hata:.2f} px")

    # ── B36: COAST — boşlukta tahmin edilen cx/cy hız yönünde İLERLER (donmaz) ──
    # Estimator hareketli hedefe oturtulduktan sonra art arda ileri-tahmin
    # çağır; cx her adımda vcx·dt kadar artmalı (frozen olsaydı sabit kalırdı).
    est2 = ib.HedefKestirim(_PREDon)
    cx_s, cy_s = 320.0, 300.0
    for _k in range(10):
        cx_s += 30.0 * _dt            # 30 px/s sağa
        est2.guncelle(cx_s, cy_s, _dt)
    cx0, _ = est2.cx, est2.cy
    est2.tahmin_ileri(_dt)
    cx1 = est2.cx
    est2.tahmin_ileri(_dt)
    cx2 = est2.cx
    kontrol("B36 coast: tahmin edilen cx hız yönünde ilerler (donmuş DEĞİL)",
            cx1 > cx0 + 1.0 and cx2 > cx1 + 1.0
            and abs((cx1 - cx0) - est2.vcx * _dt) < 1e-6,
            f"cx {cx0:.1f} → {cx1:.1f} → {cx2:.1f} (adım ≈ vcx·dt = "
            f"{est2.vcx * _dt:.2f} px; frozen olsaydı sabit)")

    # ── DÖNGÜ COAST SÜRÜCÜSÜ (B37/B38 paylaşır) ──
    # ⚠ Gerçekçi kare zamanlaması: gerçek uçuşta wait_pose bir SONRAKİ kamera
    # karesine dek BLOKLAR (~30 fps); döngünün kutu-yok yolu bu yüzden kendi
    # uyumaz (kaynak temposunu verir). Fake wait_pose'a küçük bir uyku koyup
    # bunu modelliyoruz — yoksa boşluk döngüsü μs-dt ile spin eder (dt=0.001
    # sınırında kalır) ve coast'ın ilerleyişi kare başına sub-mrad olur.
    def _kosu_coast(cfg, buyuk_kutu, sure=1.2, dt_kare=0.03, kutu_sayi=12):
        """Kutu akan sonra kesilen döngü koşusu; YALNIZ boşluk (coast) komutlarını
        döner. Her komutu duvar-zamanıyla damgalar; son kutunun servis anından
        SONRAKİ komutlar = boşluk (takip fazı dışlanır, izolasyon net)."""
        conn_c = _FakeConn()
        _sig = []                                    # (wall_t, args)
        class _M:
            def set_position_target_local_ned_send(s, *a):
                _sig.append((time.monotonic(), a))
        conn_c.mav = _M()
        stc = {"seq": 0, "son_kutu_t": None}
        def wp(son_seq, timeout=0.5):
            time.sleep(dt_kare)                       # kamera karesi temposu
            stc["seq"] += 1
            if stc["seq"] <= kutu_sayi:
                cxx = 300 + stc["seq"] * 5            # sağa kayan hedef
                if buyuk_kutu:                        # terminal latch'i tetikler
                    p = {"bbox": (cxx - 20, 285, cxx + 20, 320), "conf": 0.9}
                else:                                 # küçük: normal takip yolu
                    p = {"bbox": (cxx - 7, 296, cxx + 7, 304), "conf": 0.9}
                stc["son_kutu_t"] = time.monotonic()  # son gerçek kutu servis anı
            else:
                p = None
            return {"seq": stc["seq"], "pose": p,
                    "stamp": None, "wall_recv": None, "lock": None}
        stopc = threading.Event()
        def gi():
            return {"yaw": 0.0, "pitch": 0.0, "roll": 0.0,
                    "vx": 10.0, "vy": 0.0, "vz": 0.0}
        thc = threading.Thread(target=ib.run_bbox_ibvs,
                               args=(conn_c, gi, wp, stopc, cfg, 500,
                                     (10.0, 0.0, 0.0)),
                               daemon=True)
        thc.start(); time.sleep(sure); stopc.set(); thc.join(2.0)
        # boşluk = son kutu servis anından SONRAKİ komutlar (takip fazı hariç).
        t0 = stc["son_kutu_t"]
        if t0 is None:
            return [a for _t, a in _sig]
        return [a for _t, a in _sig if _t > t0 + 1e-4]

    # ── B37: DÖNGÜDE COAST + EXPIRY — PRED-on boşlukta İLERLER sonra DONAR,
    #        PRED-off boşlukta HİÇ ilerlemez (donmuş). Kör hücum DIŞI yol. ──
    # Kısa PRED_MAXS ile ufuk hızla dolar; ilerleme sonra durur (frozen'a düşer).
    class _PREDexp(_PREDon):
        PRED_MAXS = 0.25             # kısa ufuk: coast erken biter
    # uzun boşluk (sure) ki ufuk dolduktan SONRA bol frozen kare kalsın.
    _sig_off = _kosu_coast(_PREDoff, buyuk_kutu=False, sure=2.6)   # boşluk komutları
    _sig_exp = _kosu_coast(_PREDexp, buyuk_kutu=False, sure=2.6)
    _yoff = [s[14] for s in _sig_off]      # boşluktaki yaw komutları
    _yexp = [s[14] for s in _sig_exp]
    _span_off = (max(_yoff) - min(_yoff)) if _yoff else 0.0
    _span_exp = (max(_yexp) - min(_yexp)) if _yexp else 0.0
    # EXPIRY: son ~30 boşluk komutu (ufuk çoktan dolmuş) yaw sabit olmalı.
    _kuyruk = _yexp[-30:] if len(_yexp) >= 30 else _yexp
    _kuyruk_sabit = (max(_kuyruk) - min(_kuyruk) < 1e-4) if _kuyruk else False
    kontrol("B37 coast ufuk (PRED_MAXS) dolunca donar; PRED-off hiç ilerlemez",
            len(_yoff) > 3 and len(_yexp) > 3
            and _span_off < 1e-4 and _span_exp > 1e-3 and _kuyruk_sabit,
            f"boşlukta yaw yayılımı: PRED-off {_span_off:.5f} (donuk) | "
            f"PRED-exp {_span_exp:.4f} rad ilerledi, ufuk sonrası kuyruk sabit "
            f"({(max(_kuyruk)-min(_kuyruk)) if _kuyruk else 0:.6f})")

    # ── B38: KÖR HÜCUM (terminal) COAST — PRED-on donmuş komut yerine hareketli
    #        hedefi izler; PRED-off son komutu DONDURUR (native TERM_KOR). ──
    # Ölçülen problemin doğrudan karşılığı: kör terminal hücumda crosser yana
    # kaçarken PRED-on nişanı sürdürür, PRED-off düz uçar.
    class _TermUzun_off(ib.Cfg):
        TERMINAL_BOYUT = 30.0; TERMINAL_SURE = 5.0; PRED = False
    class _TermUzun_on(_PREDon):
        TERMINAL_BOYUT = 30.0; TERMINAL_SURE = 5.0
    _sig_toff = _kosu_coast(_TermUzun_off, buyuk_kutu=True)   # kör hücum boşluğu
    _sig_ton = _kosu_coast(_TermUzun_on, buyuk_kutu=True)
    def _yaw_yay(sig):
        vals = [s[14] for s in sig]
        return (max(vals) - min(vals)) if vals else 0.0
    _yay_toff = _yaw_yay(_sig_toff)
    _yay_ton = _yaw_yay(_sig_ton)
    kontrol("B38 kör hücum coast: PRED-on nişanı ilerletir, PRED-off dondurur",
            len(_sig_toff) > 3 and len(_sig_ton) > 3
            and _yay_toff < 1e-4 and _yay_ton > _yay_toff + 1e-3,
            f"kör hücumda yaw yayılımı: PRED-off {_yay_toff:.5f} (donuk komut) → "
            f"PRED-on {_yay_ton:.4f} rad (tahmin edilen hedefe nişan)")

    # ── B39: NORMAL TAKİPTE görüntü-LEAD — nişanı hedefin gittiği yöne öne alır ──
    # Hareketli hedefe oturmuş estimator ile normal (terminal DIŞI) takipte
    # komut()'a beslenen cx, ham cx'ten hedefin hız yönünde kaymalı. Durgun
    # hedefte lead≈0 (PN'i ezmeyen modest ofset).
    est_l = ib.HedefKestirim(_PREDon)
    cx_l = 320.0
    for _k in range(10):
        cx_l += 50.0 * _dt            # sağa 50 px/s
        est_l.guncelle(cx_l, 300.0, _dt)
    dcx, dcy = est_l.lead_ofset()
    # durgun estimator (hız 0) → ofset 0
    est_d = ib.HedefKestirim(_PREDon)
    for _k in range(5):
        est_d.guncelle(320.0, 300.0, _dt)
    dcx0, _ = est_d.lead_ofset()
    kontrol("B39 normal takip görüntü-lead: hareketli hedefte öne, durgunda ~0",
            dcx > 1.0 and abs(dcx0) < 1e-6
            and abs(dcx - est_l.vcx * _PREDon.PRED_LEAD * _PREDon.PRED_MAXS) < 1e-9,
            f"hareketli → dcx {dcx:.2f} px (=vcx·LEAD·MAXS), durgun → {dcx0:.2f} px")

    # ── B40: PRED açık DİKEY/YATAY YASAYA dokunmaz — coast dışı normal karede,
    # estimator henüz hız öğrenmemişken (tek kutu) komut ham (cx,cy) ile aynı.
    # (lead_ofset=0 → cx_giris=cx). Yasa reversibilitesi kanıtı.
    est_z = ib.HedefKestirim(_PREDon)
    est_z.guncelle(CX + 60, C.CY_NISAN + 40, _dt)     # tek ölçüm → hız 0
    _dz = est_z.lead_ofset()
    kontrol("B40 tek ölçümde lead ofseti 0 (hız yok) → komut ham (cx,cy) ile özdeş",
            abs(_dz[0]) < 1e-9 and abs(_dz[1]) < 1e-9,
            f"tek kutu → lead ofset {_dz} px (hız kestirimi yokken kaydırma yok)")

    # ══ KESİŞİM-NOKTASI TAAHHÜDÜ KİLL-SWITCH (INTERCEPT) (2026-08-10) ══
    # Ölçülen problem (log 092502): en yakın 3.00 m, 0 vuruş, 97 kör epizodu;
    # terminal yatay lead %95 doymuş, hedef GİDECEĞİ yerin gerisine nişanlanıyor.
    # INTERCEPT AÇIK → terminal son-birleşmede PRED hızıyla (vcx,vcy) hesaplanan
    # KESİŞİM noktasına nişanla (cx_aim = cx + vcx·t_go·K). Yalnız (cx,cy)
    # beslemesi; komut yasası değişmez. KAPALI (varsayılan) → besleme bit-aynı.
    class _INToff(ib.Cfg):
        PRED = True; INTERCEPT = False        # PRED açık, INTERCEPT kapalı
    class _INTon(ib.Cfg):
        PRED = True; INTERCEPT = True
        INTERCEPT_K = 1.0; INTERCEPT_MENZIL = 6.0; INTERCEPT_MAX_PX = 200.0
        INTERCEPT_KAPMIN = 1.5; INTERCEPT_TGO_MAX = 2.0

    def _crosser_est(cfg, vpx=120.0, vpy=0.0, n=15):
        """Sabit hızla yana kayan hedefe oturmuş estimator (vcx≈vpx px/s)."""
        e = ib.HedefKestirim(cfg)
        cx_e, cy_e = 320.0, 300.0
        for _ in range(n):
            cx_e += vpx * _dt
            cy_e += vpy * _dt
            e.guncelle(cx_e, cy_e, _dt)
        return e

    # ── B41: INTERCEPT KAPALI → besleme ham/PRED-lead ile BİT-AYNI (native) ──
    # Aynı geometri + estimator, INTERCEPT-off vs saf PRED: _besleme_nisan
    # terminalde de kesişim SEÇMEZ, PRED-lead yoluna düşer → çıktı özdeş.
    _e_off = _crosser_est(_INToff)
    _boyut_yakin = 52.0           # menzil = 160/52 ≈ 3.08 m (< INTERCEPT_MENZIL)
    _kap = 3.0
    _cxa_off, _cya_off, _src_off = ib._besleme_nisan(
        320.0, 300.0, _boyut_yakin, _kap, True, _e_off, _INToff)
    # saf PRED (INTERCEPT niteliği yok) ile aynı estimator → aynı lead_ofset
    _dcx_pred, _dcy_pred = _e_off.lead_ofset()
    kontrol("B41 INTERCEPT kapalı → terminal beslemesi PRED-lead ile bit-aynı",
            _src_off == "pred_lead"
            and abs(_cxa_off - (320.0 + _dcx_pred)) < 1e-12
            and abs(_cya_off - (300.0 + _dcy_pred)) < 1e-12,
            f"INTERCEPT-off terminal cx 320 → {_cxa_off:.2f} (PRED-lead {_dcx_pred:+.2f}px), "
            f"kesişim SEÇİLMEDİ (kaynak={_src_off})")

    # ── B42: INTERCEPT AÇIK — crosser'da nişan KESİŞİME öne alınır (≈vcx·t_go) ──
    # t_go = menzil/max(kapanma,KAPMIN) = 3.08/3.0 ≈ 1.03 s; dcx ≈ vcx·t_go·K.
    _e_on = _crosser_est(_INTon)
    _cxa_on, _cya_on, _src_on = ib._besleme_nisan(
        320.0, 300.0, _boyut_yakin, _kap, True, _e_on, _INTon)
    _dcx_i, _dcy_i, _tgo_i = _e_on.intercept_ofset(_boyut_yakin, _kap)
    _beklenen_dcx = _e_on.vcx * _tgo_i * _INTon.INTERCEPT_K   # tavan altındaysa bu
    kontrol("B42 INTERCEPT açık: crosser'da nişan kesişime öne alınır (~vcx·t_go)",
            _src_on == "intercept"
            and _cxa_on > 320.0 + 20.0            # belirgin öne kayma (sağ crosser)
            and abs((_cxa_on - 320.0) - _dcx_i) < 1e-9
            and _dcx_i > 0.0,
            f"vcx={_e_on.vcx:.0f}px/s, t_go={_tgo_i:.2f}s → cx 320 → {_cxa_on:.1f} "
            f"(kesişim {_dcx_i:+.1f}px, ham vcx·t_go={_beklenen_dcx:+.1f}px)")

    # ── B43: INTERCEPT yalnız INTERCEPT_MENZIL menzilinin ALTINDA uygulanır ──
    # UZAK kutu (boyut 10 → menzil 16 m > 6): INTERCEPT açık olsa da kesişim
    # SEÇİLMEZ, PRED-lead yoluna düşer (terminal son-anına özgü olsun).
    _boyut_uzak = 10.0            # menzil = 160/10 = 16 m (> INTERCEPT_MENZIL)
    _cxa_uz, _cya_uz, _src_uz = ib._besleme_nisan(
        320.0, 300.0, _boyut_uzak, _kap, True, _e_on, _INTon)
    kontrol("B43 INTERCEPT yalnız yakın menzilde (uzakta kesişim YOK, PRED-lead)",
            _src_uz == "pred_lead"
            and (160.0 / _boyut_uzak) > _INTon.INTERCEPT_MENZIL,
            f"menzil {160.0/_boyut_uzak:.0f}m > {_INTon.INTERCEPT_MENZIL:.0f}m → "
            f"kaynak={_src_uz} (yakın olsaydı 'intercept')")

    # ── B44: KESİŞİM kayması INTERCEPT_MAX_PX ile TAVANLI (gürültü savurmasın) ──
    # Absürt hız → dcx patlar; büyüklük INTERCEPT_MAX_PX'e kırpılır (yön korunur).
    _e_hizli = _crosser_est(_INTon, vpx=5000.0)
    _dcx_b, _dcy_b, _tgo_b = _e_hizli.intercept_ofset(_boyut_yakin, _kap)
    _mag = math.hypot(_dcx_b, _dcy_b)
    kontrol("B44 kesişim kayması INTERCEPT_MAX_PX ile tavanlı",
            abs(_mag - _INTon.INTERCEPT_MAX_PX) < 1e-6,
            f"vcx={_e_hizli.vcx:.0f}px/s → |kayma|={_mag:.1f}px = tavan "
            f"{_INTon.INTERCEPT_MAX_PX:.0f}px")

    # ── B45: KESİŞİM hem YAW hem DİKEYİ etkiler (2B ofset) — dikey crosser ──
    # Ölçüm: ıskanın baskın bileşeni dikeydi. Hedef DİKEYDE kayıyorsa (vcy≠0)
    # kesişim cy'yi de öne almalı → komut() dikey kanalı da geleceğe nişanlar.
    _e_dik = _crosser_est(_INTon, vpx=0.0, vpy=100.0)   # aşağı kayan hedef
    _cxa_d, _cya_d, _src_d = ib._besleme_nisan(
        320.0, 300.0, _boyut_yakin, _kap, True, _e_dik, _INTon)
    kontrol("B45 kesişim 2B: dikey crosser'da cy de öne alınır (yaw+dikey)",
            _src_d == "intercept" and _cya_d > 300.0 + 20.0 and _e_dik.vcy > 0.0,
            f"vcy={_e_dik.vcy:.0f}px/s → cy 300 → {_cya_d:.1f} (dikey kesişim "
            f"{_cya_d-300.0:+.1f}px)")

    # ── B46: t_go kapanma tabanı (KAPMIN) ve tavanı (TGO_MAX) ile sınırlı ──
    # kapanma≈0 → KAPMIN tabana oturur, t_go patlamaz; ve TGO_MAX ile tavanlı.
    _dcx0, _dcy0, _tgo0 = _e_on.intercept_ofset(_boyut_yakin, 0.0)  # kapanma 0
    _dcxN, _dcyN, _tgoN = _e_on.intercept_ofset(2.0, 0.0)  # çok yakın+kapanma 0
    kontrol("B46 t_go kapanma tabanı+tavanıyla sınırlı (yavaş kapanmada patlamaz)",
            0.0 <= _tgo0 <= _INTon.INTERCEPT_TGO_MAX + 1e-9
            and _tgoN <= _INTon.INTERCEPT_TGO_MAX + 1e-9,
            f"kapanma=0 → t_go {_tgo0:.2f}s (≤ TGO_MAX {_INTon.INTERCEPT_TGO_MAX})")

    # ── B47: INTERCEPT_K nişan kayma büyüklüğünü ölçekler (0.5 → yarı öne) ──
    class _INThalf(_INTon):
        INTERCEPT_K = 0.5
        INTERCEPT_MAX_PX = 1000.0     # tavanı kaldır ki K etkisi görünsün
    class _INTtam(_INTon):
        INTERCEPT_K = 1.0
        INTERCEPT_MAX_PX = 1000.0
    _e_k = _crosser_est(_INTtam, vpx=60.0)     # tavan altında kalacak hız
    _dxh = _crosser_est(_INThalf, vpx=60.0).intercept_ofset(_boyut_yakin, _kap)[0]
    _dxt = _e_k.intercept_ofset(_boyut_yakin, _kap)[0]
    kontrol("B47 INTERCEPT_K nişan kaymasını ölçekler (K=0.5 → yarısı)",
            _dxt > 1.0 and abs(_dxh - _dxt * 0.5) < 1e-6,
            f"K=1.0 → {_dxt:.2f}px, K=0.5 → {_dxh:.2f}px (yarısı)")

    # ── B48: kör hücum ufku INTERCEPT açıkken KISALIR (37 m fly-past kesilir) ──
    # INTERCEPT-off ufuk = TERMINAL_SURE; INTERCEPT-on ufuk = min(TERMINAL_SURE,
    # INTERCEPT_KOR_MAXS). Döngü kodundaki _kor_ufuk seçimini doğrula.
    class _KorOff(ib.Cfg):
        INTERCEPT = False; TERMINAL_SURE = 2.0; INTERCEPT_KOR_MAXS = 0.8
    class _KorOn(ib.Cfg):
        INTERCEPT = True; TERMINAL_SURE = 2.0; INTERCEPT_KOR_MAXS = 0.8
    _ufuk_off = (min(_KorOff.TERMINAL_SURE, _KorOff.INTERCEPT_KOR_MAXS)
                 if getattr(_KorOff, "INTERCEPT", False) else _KorOff.TERMINAL_SURE)
    _ufuk_on = (min(_KorOn.TERMINAL_SURE, _KorOn.INTERCEPT_KOR_MAXS)
                if getattr(_KorOn, "INTERCEPT", False) else _KorOn.TERMINAL_SURE)
    kontrol("B48 kör hücum ufku INTERCEPT açıkken kısalır (fly-past taşması azalır)",
            abs(_ufuk_off - 2.0) < 1e-9 and abs(_ufuk_on - 0.8) < 1e-9
            and _ufuk_on < _ufuk_off,
            f"INTERCEPT-off ufuk {_ufuk_off:.1f}s → INTERCEPT-on {_ufuk_on:.1f}s "
            f"(18 m/s × {_ufuk_off:.1f}s = {18*_ufuk_off:.0f}m → "
            f"{18*_ufuk_on:.0f}m fly-past)")

    # ── B49: INTERCEPT açık + PRED KAPALI → besleme kesişim SEÇMEZ (PRED şart) ──
    # INTERCEPT PRED'in vcx/vcy'sine muhtaç; PRED yoksa est kullanılmaz → native.
    class _INTnoPRED(ib.Cfg):
        PRED = False; INTERCEPT = True
    _cxa_np, _cya_np, _src_np = ib._besleme_nisan(
        320.0, 300.0, _boyut_yakin, _kap, True, _e_on, _INTnoPRED)
    kontrol("B49 INTERCEPT açık ama PRED kapalı → besleme ham (PRED şart)",
            _src_np == "native" and abs(_cxa_np - 320.0) < 1e-12,
            f"PRED=off → kaynak={_src_np}, cx değişmedi ({_cxa_np:.1f}) "
            f"(INTERCEPT PRED'in vcx/vcy'sine muhtaç)")


    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
