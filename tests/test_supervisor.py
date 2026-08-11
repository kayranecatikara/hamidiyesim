"""
tests/test_supervisor.py — faz hakemliği (GPS ↔ görsel devir) kabul kriterleri.

Gazebo'suz, saf mantık. Kullanım:
    PYTHONPATH=. python3 tests/test_supervisor.py

⚠ `pytest tests/` bu dosyanın kontrollerini ÇALIŞTIRMAZ (koşucu `main()`
içinde). Ayrı çağrılmalı.

Kapsam:
  S1-S4  GPS FAZINDA VURUŞ (2026-08-09, kullanıcı isteği). Vuruş hep görsel
         fazdan sonra olur, ama avcı hedefe kameranın göremeyeceği kadar
         yaklaşınca faz 'kayip' ile biter ve ÇARPMA GPS fazına düşer. O
         pencerede olan temas eskiden hiçbir yerde raporlanmıyordu — görev
         sonsuza kadar dönüyordu. Ölçüt YALNIZ Gazebo contact sensörü.
  S5     devir kapısı: kilit + menzil sağlanınca görsel faza geçilir
"""

import threading

from control import carpisma_state
from control.guidance import supervisor as sv

_sonuclar = []


def kontrol(ad, kosul, detay=""):
    _sonuclar.append((ad, bool(kosul), detay))
    print(f"  {'PASS' if kosul else 'FAIL'}  {ad}  {detay}")


def _cfg(**kw):
    """SupCfg'nin kopyası — sınıf niteliklerini bozmadan kol değiştirmek için."""
    ad = {k: getattr(sv.SupCfg, k) for k in dir(sv.SupCfg)
          if not k.startswith("__")}
    ad.update(kw)
    return type("SupCfgT", (), ad)


class _Kosum:
    """run_gps_guidance / run_bbox_ibvs yerine geçen sahte fazlar.

    Gerçek güdüm döngüleri MAVLink ve kamera ister; buradaki testler yalnız
    supervisor'ın FAZ HAKEMLİĞİNİ sınıyor, o yüzden fazlar taklit edilir.

    ⚠ 2026-08-10: görsel yasa TEK — run_bbox_ibvs. Eski `lead` kolu arşive
    alındı (POSEA_GERI_DONMEK_ISTERSENIZ/gudum_anlik_goruntu/), bu yüzden
    yalnız bbox sahtelenir.
    """

    def __init__(s, gorsel_sonuc="kayip"):
        s.gps_cagri = 0
        s.vis_cagri = 0
        s.gorsel_sonuc = gorsel_sonuc

    def gps(s, conn, get_plane, get_iris, faz_stop):
        s.gps_cagri += 1
        faz_stop.wait(timeout=2.0)      # izci kırana kadar bekle

    def bbox(s, conn, get_iris, wait_kare, stop_event, **kw):
        # run_bbox_ibvs imzası: get_plane_truth YOK (D0 — canlı GPS girmez).
        s.vis_cagri += 1
        return s.gorsel_sonuc


def _kos(cfg, kosum, det_var=True, d_h=5.0, sure=3.0, conf=0.9, desen=None):
    """run_hybrid'i sahte fazlarla koştur; (faz, son_sebep) döndür.

    conf  : sahte tespitin güveni (D0 eşik testleri için)
    desen : kare_no -> bool; None ise det_var sabiti kullanılır (kesintili
            tespit üretip ARDIŞIK vs KAYAN PENCERE ayrımını sınamak için)
    """
    eski_gps, eski_bbox = sv.run_gps_guidance, sv.run_bbox_ibvs
    eski_dh = sv._ga.status.get("d_h")
    sv.run_gps_guidance = kosum.gps
    sv.run_bbox_ibvs = kosum.bbox
    sv._ga.status["d_h"] = d_h
    sv._ga.status["durum"] = "KILIT"
    stop = threading.Event()

    def wait_kare(son_seq, timeout=0.5):
        # Her çağrıda bir "kare": tespit var/yok
        n = son_seq + 1
        var = desen(n) if desen else det_var
        return {"seq": n, "det": ({"conf": conf} if var else None)}

    t = threading.Thread(
        target=sv.run_hybrid,
        args=(None, lambda: None, lambda: None, wait_kare, lambda: None, stop),
        kwargs={"sup_cfg": cfg}, daemon=True)
    t.start()
    t.join(timeout=sure)
    stop.set()
    t.join(timeout=2.0)
    sv.run_gps_guidance, sv.run_bbox_ibvs = eski_gps, eski_bbox
    sv._ga.status["d_h"] = eski_dh
    return sv.status.get("faz"), sv.status.get("son_sebep")


def main():
    print("Faz hakemliği (supervisor) kabul kriterleri")
    print("=" * 60)

    # ── S1: GPS fazında GERÇEK TEMAS → VURULDU ──
    # Kullanıcı (08-09): "görselden gpse anlık geçişlerde, avcı hedefe aşırı
    # yaklaştığı için artık göremediğinden gpse geçişte ... o durumlarda da
    # vuruldu saysın EĞER GERÇEKTEN ÇARPMA VE HASAR TESPİTİ DOĞRU ÇALIŞIYORSA"
    carpisma_state.sifirla()
    carpisma_state.kaynak_bildir(True)
    carpisma_state.temas_bildir("test: iris ↔ talon")
    k = _Kosum()
    faz, sebep = _kos(_cfg(), k, det_var=False)      # tespit YOK: devir olmasın
    kontrol("S1  GPS fazında gerçek temas → VURULDU",
            faz == "VURULDU" and sebep == "vuruldu_gps" and k.vis_cagri == 0,
            f"faz={faz} sebep={sebep} görsel faz çağrısı={k.vis_cagri}")

    # ── S2: temas VAR ama KAYNAK YOK → vuruş sayılmaz ──
    # Kullanıcının şartı bu: temas dinleyicisi çalışmıyorsa "temas gelmedi" ile
    # "temas olmadı" ayırt edilemez. GPS fazında yakınlık yedeği BİLEREK yok
    # (faz zaten hedefin 8-10 m gerisinde durmak üzere kurulu).
    carpisma_state.sifirla()
    carpisma_state.kaynak_bildir(False)
    carpisma_state.temas_bildir("test")
    k = _Kosum()
    faz, _ = _kos(_cfg(), k, det_var=False, sure=1.5)
    kontrol("S2  kaynak yokken temas bayrağı VURULDU saymaz",
            faz != "VURULDU",
            f"faz={faz} (temas var ama dinleyici yok → karar verilemez)")

    # ── S3: temas YOKKEN sahte vuruş üretilmemeli ──
    carpisma_state.sifirla()
    carpisma_state.kaynak_bildir(True)
    k = _Kosum()
    faz, _ = _kos(_cfg(), k, det_var=False, sure=1.5)
    kontrol("S3  temas yokken VURULDU raporlanmaz (sahte vuruş yok)",
            faz != "VURULDU", f"faz={faz}")

    # ── S4: kill-switch eski davranışı geri getirir ──
    carpisma_state.sifirla()
    carpisma_state.kaynak_bildir(True)
    carpisma_state.temas_bildir("test")
    k = _Kosum()
    faz, _ = _kos(_cfg(GPS_VURUS=False), k, det_var=False, sure=1.5)
    kontrol("S4  AVCI_GPS_VURUS=off eski davranışa döner",
            faz != "VURULDU", f"faz={faz} (kill-switch kapalı)")

    # ── S5: devir kapısı — kilit + menzil sağlanınca görsel faza geçilir ──
    # (Vuruş kontrolünün normal devri ENGELLEMEDİĞİNİ de doğrular.)
    carpisma_state.sifirla()
    carpisma_state.kaynak_bildir(True)
    k = _Kosum(gorsel_sonuc="vuruldu")
    faz, sebep = _kos(_cfg(), k, det_var=True, d_h=5.0)
    kontrol("S5  kilit + menzil kapısı sağlanınca görsel faza devredilir",
            k.vis_cagri >= 1 and faz == "VURULDU" and sebep == "vuruldu",
            f"görsel faz çağrısı={k.vis_cagri} faz={faz} sebep={sebep}")

    # ══ S6-S8: D0 DEVİR ÖLÇÜTÜ (Kayra 2b8d68c'den taşındı, 2026-08-11) ══
    # S6 ASIL KURAL BEKÇİSİ: dedektörün kabul eşiği bbox_ibvs.CONF_MIN=0.35.
    # Supervisor'ın kendi eşiği ondan YÜKSEK olursa, arada kalan bantta model
    # "gördüm" derken güdüm GPS'te kalır — D0 tam olarak bunu yasaklıyor.
    carpisma_state.sifirla()
    carpisma_state.kaynak_bildir(True)
    k = _Kosum(gorsel_sonuc="vuruldu")
    faz, _ = _kos(_cfg(), k, det_var=True, d_h=5.0, conf=0.40)
    kontrol("S6  conf 0.40 (dedektör eşiği 0.35 üstü) DEVREDİYOR — D0 uyumu",
            k.vis_cagri >= 1 and faz == "VURULDU",
            f"görsel faz çağrısı={k.vis_cagri}; supervisor eşiği "
            f"{sv.SupCfg.KILIT_CONF_MIN:.2f} ≤ dedektör eşiği 0.35 olmalı")

    # S7: eski 0.5 eşiği geri alınınca aynı tespit devretmez (ihlalin kanıtı)
    carpisma_state.sifirla()
    k = _Kosum(gorsel_sonuc="vuruldu")
    faz, _ = _kos(_cfg(KILIT_CONF_MIN=0.5), k, det_var=True, d_h=5.0, conf=0.40)
    kontrol("S7  AVCI_HYBRID_CONF=0.5 geri alınırsa AYNI tespit devretMEZ",
            k.vis_cagri == 0,
            f"görsel faz çağrısı={k.vis_cagri} — 0.35-0.50 bandı eskiden "
            f"görsel temas varken GPS'te kalıyordu (giderilen ihlal)")

    # S8: ARDIŞIK sayaç tek tespitsiz karede sıfırlanır; kayan pencere sıfırlamaz.
    # Desen: her 10. kare tespitsiz → ardışık en fazla 9 (KILIT_N=10'a ulaşmaz),
    # kayan pencerede ise 15 karenin ~13'ü dolu → devreder.
    kesintili = lambda n: (n % 10) != 0
    carpisma_state.sifirla()
    k_ard = _Kosum(gorsel_sonuc="vuruldu")
    _kos(_cfg(KILIT_ARDISIK=True), k_ard, d_h=5.0, desen=kesintili, sure=2.5)
    carpisma_state.sifirla()
    k_pen = _Kosum(gorsel_sonuc="vuruldu")
    _kos(_cfg(KILIT_ARDISIK=False), k_pen, d_h=5.0, desen=kesintili, sure=2.5)
    kontrol("S8  ARDIŞIK kesintili tespitte devretmez, KAYAN PENCERE devreder",
            k_ard.vis_cagri == 0 and k_pen.vis_cagri >= 1,
            f"her 10. kare tespitsiz → ardışık={k_ard.vis_cagri} çağrı, "
            f"kayan pencere={k_pen.vis_cagri} çağrı (ARDIŞIK'ın bilinen riski: "
            f"gürültülü tespitte devir GECİKİR)")

    carpisma_state.sifirla()
    print("=" * 60)
    fails = [ad for ad, ok, _ in _sonuclar if not ok]
    print(f"SONUÇ: {len(_sonuclar) - len(fails)}/{len(_sonuclar)} geçti"
          + (f" — KALAN: {fails}" if fails else " — HEPSİ GEÇTİ ✓"))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
