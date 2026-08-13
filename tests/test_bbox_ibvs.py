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
    def wait_kare(son_seq, timeout=0.5):
        st["seq"] += 1
        # hedef sağda + biraz altta, orta boy kutu
        return {"seq": st["seq"],
                "det": {"bbox": (360, 250, 400, 285), "conf": 0.8},
                "stamp": None, "wall_recv": None, "lock": None}
    def get_iris():
        return {"yaw": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0}
    stop = threading.Event()
    import tempfile
    ib._LOG_DIR = tempfile.mkdtemp(prefix="avci_ibvs_test_")
    th = threading.Thread(target=ib.run_bbox_ibvs,
                          args=(conn, get_iris, wait_kare, stop), daemon=True)
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
    def wait_kare_bos(son_seq, timeout=0.5):
        st2["seq"] += 1
        return {"seq": st2["seq"], "det": None,
                "stamp": None, "wall_recv": None, "lock": None}
    stop2 = threading.Event()
    sonuc = {"r": None}
    def kosu():
        sonuc["r"] = ib.run_bbox_ibvs(conn2, get_iris, wait_kare_bos, stop2,
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
    def wait_kare_karisik(son_seq, timeout=0.5):
        st3["seq"] += 1
        # ilk 8 kare kutu var, sonra boşluk
        p = ({"bbox": (300, 290, 320, 305), "conf": 0.8}
             if st3["seq"] <= 8 else None)
        return {"seq": st3["seq"], "det": p,
                "stamp": None, "wall_recv": None, "lock": None}
    # Devir gerçeği: drone zaten uçuyor (ivme sınırlayıcı oradan başlar)
    def get_iris_ucan():
        return {"yaw": 0.0, "vx": 14.0, "vy": 3.0, "vz": 0.0}
    stop3 = threading.Event()
    th3 = threading.Thread(
        target=ib.run_bbox_ibvs,
        args=(conn3, get_iris_ucan, wait_kare_karisik, stop3, ib.Cfg, 50,
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
    def wait_kare4(son_seq, timeout=0.5):
        st4["seq"] += 1
        return {"seq": st4["seq"],
                "det": {"bbox": (310, 295, 330, 310), "conf": 0.8},
                "stamp": None, "wall_recv": None, "lock": None}
    temas_durum = {"v": False}
    def get_temas():
        st4["seq"] > 5 and temas_durum.update(v=True)
        return temas_durum["v"]
    stop4 = threading.Event()
    son4 = {"r": None}
    def kosu4():
        son4["r"] = ib.run_bbox_ibvs(conn4, get_iris, wait_kare4, stop4,
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
    def wait_kare5(son_seq, timeout=0.5):
        st5["seq"] += 1
        # ilk 5 kare BÜYÜK kutu (terminal tetiklenir), sonra hiç kutu yok
        p = ({"bbox": (300, 285, 340, 315), "conf": 0.9}
             if st5["seq"] <= 5 else None)
        return {"seq": st5["seq"], "det": p,
                "stamp": None, "wall_recv": None, "lock": None}
    stop5 = threading.Event()
    son5 = {"r": None}
    def kosu5():
        son5["r"] = ib.run_bbox_ibvs(conn5, get_iris, wait_kare5, stop5,
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
    def wait_kare_savrulan(son_seq, timeout=0.5):
        st6["seq"] += 1
        # kutu kadrajı sağdan sola SÜPÜRÜYOR (fly-past): her karede ±uç
        cxx = 600 if st6["seq"] % 2 else 40
        return {"seq": st6["seq"],
                "det": {"bbox": (cxx - 15, 290, cxx + 15, 315), "conf": 0.9},
                "stamp": None, "wall_recv": None, "lock": None}
    stop6 = threading.Event()
    th6 = threading.Thread(
        target=ib.run_bbox_ibvs,
        args=(conn6, get_iris, wait_kare_savrulan, stop6, ib.Cfg, 100,
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

    # ══ B43-B48: Ö5 ANİ KAÇIŞ — TESPİT + DÖNÜŞ-FARKINDA HIZ TAVANI ══
    # Kullanıcı fikri (2026-08-10); tespit ölçütü ölçümle piksel hızından
    # yanal hıza (|λ̇·R|) çevrildi — piksel hızı 1/R ile patlıyor ve yakın
    # geçişte hedef manevra yapmasa bile 1666 px/s üretiyor.
    class _Man(C):
        MANEVRA = True
        MANEVRA_VYAN = 8.0
        MANEVRA_R = 12.0
        MANEVRA_ACI = 45.0
        MANEVRA_VMIN = 6.0

    class _ManKapali(_Man):
        MANEVRA = False

    # boyut 20 px → R = 160/20 = 8 m (< MANEVRA_R). λ̇ = 1.5 rad/s → v_yan = 12 m/s
    _argM = (CX, C.CY_NISAN, 20, 20, 0.0, 18.0, 0.05)
    _man = ib.komut(*_argM, _Man, False, (1.5, 0.0), 0.0, 0.0, None, 0.0)
    _kap = ib.komut(*_argM, _ManKapali, False, (1.5, 0.0), 0.0, 0.0, None, 0.0)

    kontrol("B43 yakın + hızlı yanal kayma → MANEVRA tespit edilir",
            _man[5]["manevra"] is True and _kap[5]["manevra"] is False,
            f"v_yanal {_man[5]['v_yanal']:.1f} m/s (eşik {_Man.MANEVRA_VYAN:.0f}), "
            f"R=8 m; MANEVRA=0 iken tespit yok")

    _bek = 9.81 * math.tan(math.radians(45.0)) / 1.5      # = 6.54 m/s
    kontrol("B44 hız tavanı v ≤ g·tan(açı)/λ̇ formülünü uygular",
            abs(_man[5]["v_los"] - _bek) < 1e-6,
            f"λ̇=1.5 rad/s → tavan {_bek:.2f} m/s, komut {_man[5]['v_los']:.2f}; "
            f"kapalıyken {_kap[5]['v_los']:.2f} m/s")

    # UZAK hedef: boyut 8 px → R = 20 m > MANEVRA_R → tetiklenmemeli
    _uzak = ib.komut(CX, C.CY_NISAN, 8, 8, 0.0, 18.0, 0.05, _Man, False,
                     (1.5, 0.0), 0.0, 0.0, None, 0.0)
    kontrol("B45 UZAK hedefte tetiklenmez (menzil kapısı)",
            _uzak[5]["manevra"] is False,
            f"R=20 m > {_Man.MANEVRA_R:.0f} m → tespit yok, "
            f"v_los {_uzak[5]['v_los']:.1f} m/s serbest")

    # YAKIN ama SAKİN: λ̇ = 0.1 → v_yan = 0.8 m/s → tetiklenmemeli
    _sakin = ib.komut(*_argM, _Man, False, (0.1, 0.0), 0.0, 0.0, None, 0.0)
    kontrol("B46 yakın ama SAKİN takipte tetiklenmez (normal uçuş korunur)",
            _sakin[5]["manevra"] is False and _sakin[5]["v_yanal"] < 1.0,
            f"λ̇=0.1 rad/s → v_yanal {_sakin[5]['v_yanal']:.2f} m/s < "
            f"{_Man.MANEVRA_VYAN:.0f} → tespit yok")

    # VMIN tabanı: λ̇ çok büyükse tavan VMIN'in altına inmemeli
    _sert = ib.komut(*_argM, _Man, False, (5.0, 0.0), 0.0, 0.0, None, 0.0)
    kontrol("B47 tavan MANEVRA_VMIN'in altına İNMEZ (araç durmaz)",
            abs(_sert[5]["v_los"] - _Man.MANEVRA_VMIN) < 1e-9,
            f"λ̇=5.0 rad/s → ham tavan {9.81/5.0:.2f} m/s, "
            f"uygulanan {_sert[5]['v_los']:.2f} = VMIN")

    # Yatış açısı büyürse tavan YÜKSELİR (Ö6 ile birlikte anlamlı)
    class _Man55(_Man):
        MANEVRA_ACI = 55.0
    _m55 = ib.komut(*_argM, _Man55, False, (1.5, 0.0), 0.0, 0.0, None, 0.0)
    kontrol("B48 yatış açısı 45→55° olunca tavan YÜKSELİR (Ö6 ile birlikte)",
            _m55[5]["v_los"] > _man[5]["v_los"] + 1.0,
            f"45° → {_man[5]['v_los']:.2f} m/s, 55° → {_m55[5]['v_los']:.2f} m/s")

    # ══ B49-B53: M3b — SEYİR FAZI İÇİN AYRI LEAD TAVANI ══
    # 08-10 kampanyası (158 koşu): lead karelerin %71-77'sinde TAM SIFIR,
    # çünkü kapı `if terminal or LEAD_ERKEN` ve LEAD_ERKEN varsayılan kapalı.
    # 08-09'da erken lead terminal tavanıyla (25°) denenip geri alınmıştı —
    # yön doğru, genlik yanlıştı. Tavan artık faza göre AYRI.
    class _LeadKapali(C):
        LEAD_ERKEN = False
        LEAD_MAX_SEYIR_DEG = 8.0

    class _LeadAcik(_LeadKapali):
        LEAD_ERKEN = True

    class _LeadAcik20(_LeadAcik):
        LEAD_MAX_SEYIR_DEG = 20.0

    # boyut 12 px → R ≈ 13 m (seyir, terminal DEĞİL). λ̇ = 1.0 rad/s.
    _argL = (CX, C.CY_NISAN, 12, 12, 0.0, 18.0, 0.05)
    _lk = ib.komut(*_argL, _LeadKapali, False, (1.0, 0.0), 0.0, 0.0, None, 0.0)
    _la = ib.komut(*_argL, _LeadAcik,   False, (1.0, 0.0), 0.0, 0.0, None, 0.0)

    kontrol("B49 SEYİRDE lead erken KAPALIYKEN sıfırdır (ölçülen kusur)",
            abs(_lk[5]["lead_az"]) < 1e-12,
            f"terminal=False, LEAD_ERKEN=0 → lead {_lk[5]['lead_az']:.3f} rad "
            f"— kampanyada karelerin %71-77'sinde görülen durum")

    kontrol("B50 SEYİRDE lead erken AÇIKKEN sıfırdan farklıdır",
            abs(_la[5]["lead_az"]) > 1e-6,
            f"λ̇=1.0 rad/s → lead {math.degrees(_la[5]['lead_az']):.2f}°")

    kontrol("B51 SEYİR tavanı LEAD_MAX_SEYIR_DEG'i AŞMAZ",
            abs(math.degrees(_la[5]["lead_az"]))
            <= _LeadAcik.LEAD_MAX_SEYIR_DEG + 1e-9,
            f"|lead| {abs(math.degrees(_la[5]['lead_az'])):.2f}° ≤ "
            f"{_LeadAcik.LEAD_MAX_SEYIR_DEG:.0f}° (terminal tavanı "
            f"{C.LEAD_MAX_DEG:.0f}° DEĞİL)")

    # Aynı girdide seyir tavanı 8→20 olunca lead büyümeli (tavan bağlayıcı)
    _l20 = ib.komut(*_argL, _LeadAcik20, False, (1.0, 0.0), 0.0, 0.0, None, 0.0)
    kontrol("B52 seyir tavanı büyütülünce lead BÜYÜR (tavan gerçekten bağlıyor)",
            abs(_l20[5]["lead_az"]) > abs(_la[5]["lead_az"]) + 1e-6,
            f"8° → {math.degrees(_la[5]['lead_az']):.2f}°, "
            f"20° → {math.degrees(_l20[5]['lead_az']):.2f}°")

    # TERMİNALDE tavan hâlâ LEAD_MAX_DEG (25°) — seyir tavanı onu KISMAMALI
    _term = ib.komut(*_argL, _LeadKapali, True, (3.0, 0.0), 0.0, 0.0, None, 0.0)
    kontrol("B53 TERMİNAL tavanı seyir tavanından etkilenmez (25° korunur)",
            abs(math.degrees(_term[5]["lead_az"]))
            > _LeadKapali.LEAD_MAX_SEYIR_DEG + 1e-6,
            f"terminal lead {abs(math.degrees(_term[5]['lead_az'])):.2f}° > "
            f"seyir tavanı {_LeadKapali.LEAD_MAX_SEYIR_DEG:.0f}° — "
            f"08-09'da doğrulanmış terminal davranışı bozulmadı")

    # ══ B54-B58: M4 — SEYİR FAZI DİKEY SÖNÜMLEMESİ ══
    # 165 koşuluk M3b kampanyası: kaçamak sonrası avcı hedefin ÜSTÜNE çıkıyor
    # (capraz +16.2 m, yatay +11.9 m). Uçuş anı kareleri (durum=IBVS, yani BU
    # yasa komut ediyordu): kutu küçülürken vz komutu −1.22 → −2.54 m/s
    # büyüyor = sönümlemesiz P imzası. Terminalde türev terimi vardı, seyirde
    # yoktu.
    class _SeyirSonumsuz(C):
        K_VZ_D_SEYIR = 0.0          # eski davranış
    class _SeyirSonumlu(C):
        K_VZ_D_SEYIR = 0.6

    # Hedef nişanın ÜSTÜNDE (cy < CY_NISAN) → tırmanma komutu (NED: vz<0).
    # boyut 12 px → R≈13 m, terminal DEĞİL (seyir dalı).
    _cy_ust = C.CY_NISAN - 40
    _argV = (CX, _cy_ust, 12, 12, 0.0, 18.0, 0.05)

    _v_dur = ib.komut(*_argV, _SeyirSonumsuz, False, (0.0, 0.0), 0.0, 0.0,
                      None, 0.0)          # iris_vz=0: araç henüz tırmanmıyor
    kontrol("B54 sönümleme KAPALI iken seyir dikey komutu eski formülle aynı",
            abs(_v_dur[2] - ib.clamp(C.K_VZ * C.V_NOM
                                     * math.atan((_cy_ust - C.CY_NISAN) / geo.FY),
                                     -C.VZ_MAX, C.VZ_MAX)) < 1e-9,
            f"vz={_v_dur[2]:.3f} m/s — K_VZ·V_NOM·eps_elev ile birebir")

    # Araç ZATEN komuttan hızlı tırmanıyorsa (iris_vz çok negatif) sönümleme
    # komutu geri çekmeli — asıl aranan davranış bu.
    _hizli = -4.0                          # m/s; NED'de yukarı
    _s_kapali = ib.komut(*_argV, _SeyirSonumsuz, False, (0.0, 0.0), 0.0,
                         _hizli, None, 0.0)
    _s_acik = ib.komut(*_argV, _SeyirSonumlu, False, (0.0, 0.0), 0.0,
                       _hizli, None, 0.0)
    kontrol("B55 araç fazla tırmanıyorken sönümleme komutu GERİ ÇEKER",
            _s_acik[2] > _s_kapali[2] + 0.5,
            f"iris_vz={_hizli:.1f} m/s iken vz: kapalı {_s_kapali[2]:.2f} → "
            f"açık {_s_acik[2]:.2f} (daha az tırmanma)")

    kontrol("B56 sönümleme KAPALI iken araç hızı komutu HİÇ etkilemez",
            abs(_s_kapali[2] - _v_dur[2]) < 1e-9,
            f"iris_vz 0 → {_v_dur[2]:.2f}, iris_vz {_hizli:.1f} → "
            f"{_s_kapali[2]:.2f} (saf P: araç hızına kör)")

    # Araç tırmanması gerekirken tırmanmıyorsa komut BÜYÜMELİ (tek yönlü değil)
    _yavas = ib.komut(*_argV, _SeyirSonumlu, False, (0.0, 0.0), 0.0, 0.0,
                      None, 0.0)
    kontrol("B57 araç yavaş kalmışsa sönümleme komutu BÜYÜTÜR (simetrik)",
            _yavas[2] < _v_dur[2] - 1e-9,
            f"iris_vz=0 iken kapalı {_v_dur[2]:.2f} → açık {_yavas[2]:.2f}")

    # Terminal dalı BU değişiklikten etkilenmemeli (tek değişken kuralı)
    _t_kapali = ib.komut(*_argV, _SeyirSonumsuz, True, (0.0, 0.0), 0.0,
                         _hizli, None, 0.0)
    _t_acik = ib.komut(*_argV, _SeyirSonumlu, True, (0.0, 0.0), 0.0,
                       _hizli, None, 0.0)
    kontrol("B58 TERMİNAL dikey komutu M4'ten ETKİLENMEZ (dokunulmadı)",
            abs(_t_kapali[2] - _t_acik[2]) < 1e-9,
            f"terminal vz: {_t_kapali[2]:.3f} = {_t_acik[2]:.3f} — "
            f"terminal sönümlemesi K_VZ_D ayrı kalıyor")

    # ══ B59-B66: KAYRA'NIN DÖRT ÖZELLİĞİ (2026-08-13 port) ══
    # Hepsi VARSAYILAN KAPALI. En kritik bekçi B59: dördü de kapalıyken
    # komut çıktısı port ÖNCESİ değerlerle BİREBİR aynı olmalı — yoksa
    # ölçülmemiş bir davranış sessizce uçmaya başlar.
    class _Kapali(C):
        YANAL_K = 0.0; SONUM_T = 0.0; DONUS_A = 0.0; DIKEY_ROLL = False

    _argK = (CX + 60, C.CY_NISAN - 30, 14, 14, 0.0, 18.0, 0.05)
    _k = ib.komut(*_argK, _Kapali, False, (0.5, 0.0), 0.0, 0.0, 8.0, 0.35, 0.8)
    # Elle beklenen: eps_hiz == eps_yaw, sonum == 0, eps_elev == ham
    kontrol("B59 dördü KAPALIYKEN yasa eski hâline birebir indirgenir",
            abs(_k[5]["eps_hiz"] - _k[5]["eps_yaw"]) < 1e-12
            and abs(_k[5]["sonum"]) < 1e-12
            and abs(_k[5]["eps_elev"] - _k[5]["eps_elev_ham"]) < 1e-12
            and _k[5]["donus_tavan"] is None,
            "eps_hiz=eps_yaw, sonum=0, eps_elev=ham, donus_tavan=None")

    # ── Ö8: yakında AÇI büyük ama KAÇIRMA küçükse komut kısılır ──
    class _O8(_Kapali):
        YANAL_K = 1.0
    # ⚠ SEYİR fazı olmalı: boyut < TERMINAL_BOYUT(25). Ayrıca V_MIN=0 olduğu
    # için çok yakında v_los sıfıra iniyor ve Ö8'in v_los>0.1 koruması devreye
    # giriyor — o rejim zaten TERMİNAL'dir. boyut 24 px → R ≈ 6.7 m.
    _a8 = (CX + 150, C.CY_NISAN, 24, 24, 0.0, 18.0, 0.05)
    _o8k = ib.komut(*_a8, _Kapali, False, (0.0, 0.0), 0.0, 0.0, 5.0, 0.0)
    _o8a = ib.komut(*_a8, _O8,     False, (0.0, 0.0), 0.0, 0.0, 5.0, 0.0)
    kontrol("B60 Ö8 YAKINDA komutu kısar (6.7 m'de 40° = yalnız 4.3 m ıska)",
            abs(_o8a[5]["eps_hiz"]) < abs(_o8k[5]["eps_hiz"]) - 1e-6,
            f"R≈{160/24:.1f} m: eps_yaw {math.degrees(_o8k[5]['eps_yaw']):.1f}° → "
            f"eps_hiz {math.degrees(_o8a[5]['eps_hiz']):.1f}°")

    # UZAKTA aynı açı GERÇEK bir kaçırmadır — kısılmamalı
    _a8u = (CX + 150, C.CY_NISAN, 8, 8, 0.0, 18.0, 0.05)   # R = 20 m
    _o8u = ib.komut(*_a8u, _O8, False, (0.0, 0.0), 0.0, 0.0, 2.0, 0.0)
    kontrol("B61 Ö8 UZAKTA kısmaz (20 m'de aynı açı 13 m ıska demek)",
            abs(_o8u[5]["eps_hiz"] - _o8u[5]["eps_yaw"]) < 1e-9,
            f"R={160/8:.0f} m > YANAL_MENZIL={C.YANAL_MENZIL:.0f} → ağırlık 0")

    kontrol("B62 Ö8 burun yönünü (yaw_cmd) DEĞİŞTİRMEZ — yalnız hız vektörü",
            abs(_o8a[3] - _o8k[3]) < 1e-12,
            f"yaw_cmd {_o8k[3]:.6f} = {_o8a[3]:.6f}; ayrışan şey hiz_yonu")

    # ── Ö9: araç dönerken komut geri çekilir ──
    class _O9(_Kapali):
        SONUM_T = 0.3
    _a9 = (CX + 60, C.CY_NISAN, 14, 14, 0.0, 18.0, 0.05)
    _o9k = ib.komut(*_a9, _Kapali, False, (0.0, 0.0), 0.0, 0.0, None, 0.0, 1.0)
    _o9a = ib.komut(*_a9, _O9,     False, (0.0, 0.0), 0.0, 0.0, None, 0.0, 1.0)
    kontrol("B63 Ö9 araç dönerken yaw komutunu GERİ ÇEKER",
            _o9a[3] < _o9k[3] - 1e-6
            and abs(_o9a[5]["sonum"] - 0.3) < 1e-9,
            f"yaw_hızı=1.0 rad/s → sönüm {math.degrees(_o9a[5]['sonum']):.1f}°, "
            f"yaw_cmd {_o9k[3]:.3f} → {_o9a[3]:.3f}")

    _o9d = ib.komut(*_a9, _O9, False, (0.0, 0.0), 0.0, 0.0, None, 0.0, 0.0)
    kontrol("B64 Ö9 DÜZ uçuşta etkisiz (araç dönmüyorsa terim sıfır)",
            abs(_o9d[3] - _o9k[3]) < 1e-12,
            "yaw_hızı=0 → sönüm 0, komut değişmez")

    # ── Ö5-kapısız: λ̇ büyükse hız kısılır, kapı YOK ──
    class _O5B(_Kapali):
        DONUS_A = 9.0
    # UZAK hedef (R=20 m) — bizim eski Ö5 buna kapı yüzünden DOKUNMAZDI
    _a5 = (CX, C.CY_NISAN, 8, 8, 0.0, 18.0, 0.05)
    # λ̇=0.5 → ham tavan 9.0/0.5 = 18.0 m/s (DONUS_V_MIN=10 tabanının üstünde,
    # yani formülün kendisi sınanıyor, taban değil).
    _o5k = ib.komut(*_a5, _Kapali, False, (0.5, 0.0), 0.0, 0.0, None, 0.0)
    _o5a = ib.komut(*_a5, _O5B,    False, (0.5, 0.0), 0.0, 0.0, None, 0.0)
    kontrol("B65 Ö5-kapısız UZAKTA da kısar (eski sürüm kapı yüzünden kısmazdı)",
            _o5a[5]["v_los"] < _o5k[5]["v_los"] - 1e-6
            and abs(_o5a[5]["donus_tavan"] - 18.0) < 1e-9,
            f"λ̇=0.5 → tavan {_o5a[5]['donus_tavan']:.1f} m/s (=DONUS_A/λ̇); "
            f"v_los {_o5k[5]['v_los']:.1f} → {_o5a[5]['v_los']:.1f}, R=20 m "
            f"(eski MANEVRA sürümü R≤12 kapısı yüzünden dokunmazdı)")

    # ── T1b: yatıkken dikey okuma düzelir, düz uçuşta değişmez ──
    class _T1B(_Kapali):
        DIKEY_ROLL = True
    _at = (CX + 120, C.CY_NISAN - 20, 14, 14, 0.0, 18.0, 0.05)
    _t_duz = ib.komut(*_at, _T1B, False, (0.0, 0.0), 0.0, 0.0, None, 0.0)
    kontrol("B66 T1b DÜZ uçuşta (roll=0) dikey okumayı DEĞİŞTİRMEZ",
            abs(_t_duz[5]["eps_elev"] - _t_duz[5]["eps_elev_ham"]) < 1e-12,
            "roll=0 → telafi farkı tam sıfır")

    _t_yat = ib.komut(*_at, _T1B, False, (0.0, 0.0), 0.0, 0.0, None,
                      math.radians(35.0))
    _t_ham = ib.komut(*_at, _Kapali, False, (0.0, 0.0), 0.0, 0.0, None,
                      math.radians(35.0))
    kontrol("B67 T1b 35° yatışta dikey okumayı DÜZELTİR (kenardaki hedef)",
            abs(_t_yat[5]["eps_elev"] - _t_ham[5]["eps_elev"]) > math.radians(3.0),
            f"|dx|=120 px, roll=35° → ham "
            f"{math.degrees(_t_ham[5]['eps_elev']):.1f}° vs telafili "
            f"{math.degrees(_t_yat[5]['eps_elev']):.1f}°")

    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
