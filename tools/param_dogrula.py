#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
PARAMETRE DOĞRULAYICI — "dosyada yazan, araçta gerçekten var mı?"
================================================================================
NİYE VAR (2026-09-06'da yakalanan gerçek arıza):

  `sim/ardupilot_params/avci_copter.parm` dosyası eğim tavanı için 7000
  (=70°) yazıyordu. Araç 6000 (=60°) uyguluyordu. Fark 2026-08-17'den beri
  vardı ve o tarihten sonraki BÜTÜN kampanya raporları "araç 70°'ye kadar
  yatabiliyor" varsayımı üzerine kuruldu.

  KÖK NEDEN — ArduPilot'un parametre önceliği:
    sim_vehicle.py bu dosyayı `--add-param-file` ile verir.
    O yol  AP_Param::set_default_by_name()  çağırır (AP_Param.cpp:2723).
    Adı üstünde: yalnız VARSAYILANI değiştirir.
    Bir parametrenin ~/ardupilot/eeprom.bin içinde KAYITLI (saved) değeri
    varsa — biri bir kez GCS'ten/MAVProxy'den yazdıysa — KAYITLI DEĞER
    KAZANIR ve dosya sessizce yok sayılır. Hata mesajı YOKTUR.

  Bu tam olarak CLAUDE.md §5.1'in ("özellik gerçekten çalıştı mı?")
  önlemeye çalıştığı hata sınıfıdır — ve deney disiplininin altını oyar.

KULLANIM:
    python3 tools/param_dogrula.py            # yalnız RAPOR (hiçbir şey yazmaz)
    python3 tools/param_dogrula.py --yaz      # uyuşmayanları MAVLink ile YAZ
    python3 tools/param_dogrula.py --dosya sim/ardupilot_params/avci_plane.parm \\
                                   --adres udpin:127.0.0.1:14542

ÇIKIŞ KODU:  0 = hepsi uyuşuyor (ya da --yaz ile düzeltildi),  1 = uyuşmazlık var
================================================================================
"""
import argparse
import os
import sys
import time

from pymavlink import mavutil

VARSAYILAN_DOSYA = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "sim", "ardupilot_params", "avci_copter.parm")


def istekleri_oku(yol):
    """parm dosyasından (ad, değer) çiftlerini çıkar. Yorum satırları atlanır."""
    cikti = []
    for ham in open(yol, encoding="utf-8"):
        s = ham.split("#")[0].strip()
        if not s:
            continue
        parca = s.replace(",", " ").split()
        if len(parca) >= 2:
            try:
                cikti.append((parca[0], float(parca[1])))
            except ValueError:
                pass
    return cikti


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dosya", default=VARSAYILAN_DOSYA)
    ap.add_argument("--adres", default="udpin:127.0.0.1:14541")
    ap.add_argument("--yaz", action="store_true",
                    help="uyuşmayanları araca YAZ (kalıcı kaydeder)")
    ap.add_argument("--sessiz", action="store_true",
                    help="yalnız uyuşmazlıkları yaz")
    a = ap.parse_args()

    istekler = istekleri_oku(a.dosya)
    if not istekler:
        print("⛔ dosyada parametre bulunamadı: %s" % a.dosya)
        return 1

    c = mavutil.mavlink_connection(a.adres)
    c.wait_heartbeat()

    def oku(ad):
        for _ in range(3):
            c.mav.param_request_read_send(c.target_system, c.target_component,
                                          ad.encode(), -1)
            t0 = time.time()
            while time.time() - t0 < 2.0:
                m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.8)
                if m and m.param_id.strip("\x00") == ad:
                    return m.param_value
        return None

    def yaz(ad, deger):
        c.mav.param_set_send(c.target_system, c.target_component, ad.encode(),
                             float(deger), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if m and m.param_id.strip("\x00") == ad:
                return m.param_value
        return None

    print("=" * 74)
    print("  PARAMETRE DOĞRULAMA — %s" % os.path.basename(a.dosya))
    print("  araç: sys=%d  (%s)" % (c.target_system, a.adres))
    print("=" * 74)

    uyusmaz, eksik, duzeltilen = [], [], []
    for ad, istenen in istekler:
        v = oku(ad)
        if v is None:
            eksik.append(ad)
            print("  %-18s  dosya %12.5f   araç        —   ⛔ PARAMETRE YOK" % (ad, istenen))
            continue
        tolerans = max(1e-4, abs(istenen) * 1e-4)
        if abs(v - istenen) <= tolerans:
            if not a.sessiz:
                print("  %-18s  dosya %12.5f   araç %11.5f   ✔" % (ad, istenen, v))
            continue
        uyusmaz.append((ad, istenen, v))
        print("  %-18s  dosya %12.5f   araç %11.5f   ⛔ UYUŞMUYOR" % (ad, istenen, v))
        if a.yaz:
            yeni = yaz(ad, istenen)
            if yeni is not None and abs(yeni - istenen) <= tolerans:
                duzeltilen.append(ad)
                print("  %-18s  → YAZILDI, geri okuma %.5f" % ("", yeni))
            else:
                print("  %-18s  → ⛔ YAZILAMADI (geri okuma %s)" % ("", yeni))

    print("-" * 74)
    print("  toplam %d parametre · uyuşmayan %d · araçta olmayan %d"
          % (len(istekler), len(uyusmaz), len(eksik)))
    if uyusmaz and not a.yaz:
        print()
        print("  ⛔ UYUŞMAZLIK VAR. En olası sebep: bu parametrelerin")
        print("     ~/ardupilot/eeprom.bin içinde KAYITLI değeri var ve o kazanıyor.")
        print("     Düzeltmek için:  python3 tools/param_dogrula.py --yaz")
        print("     (ya da SITL'i `sim_vehicle.py -w` ile bir kez sıfırla)")
    if a.yaz and duzeltilen:
        print("  ✔ düzeltilen: %s" % ", ".join(duzeltilen))
    return 1 if (uyusmaz and not a.yaz) or eksik else 0


if __name__ == "__main__":
    sys.exit(main())
