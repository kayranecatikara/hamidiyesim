#!/usr/bin/env python3
# Kaynak: ekip PR#4 (hamidiyesim, Kübra Nur Tiryaki) — birebir alındı 2026-08-02.
"""parm_denetle.py — .parm dosyalarındaki her parametrenin SITL'e GERÇEKTEN
uygulanıp uygulanmadığını denetler.

NEDEN: ArduPilot tanımadığı parametre adını SESSİZCE yok sayar — ne hata basar
ne uyarır. 2026-08-01'de bu yüzden avci_copter.parm'daki 9 parametrenin 7'si
aylarca uygulanmamıştı (firmware adları yeniden adlandırıp cm/cdeg yerine SI
birimine geçmişti). Araç ATC_ANGLE_MAX=30° varsayılanıyla uçuyordu: yatay ivme
tavanı 5.7 m/s², oysa güdüm 12 m/s² komut ediyor. "Manevra yapamıyor, hedefi
kaçırıyor" belirtisinin doğrudan sebebi buydu.

KULLANIM — uçuştan SONRA (SITL param dökümü o zaman oluşur):

    python3 tools/parm_denetle.py

Döküm dosyaları sim_vehicle.py tarafından ~/ardupilot/mav_<sysid>_1.parm
olarak yazılır (copter sysid 5, plane sysid 2).

Çıkış kodu: tanınmayan/uyuşmayan parametre varsa 1, hepsi tamamsa 0.
"""
import os
import sys

_KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_AP = os.path.expanduser("~/ardupilot")

# (etiket, proje .parm dosyası, SITL döküm dosyası)
CIFTLER = [
    ("COPTER (avcı)", f"{_KOK}/sim/ardupilot_params/avci_copter.parm",
     f"{_AP}/mav_5_1.parm"),
    ("PLANE (Talon)", f"{_KOK}/sim/ardupilot_params/avci_plane.parm",
     f"{_AP}/mav_2_1.parm"),
]


def _yukle(yol):
    """.parm / döküm dosyasını {AD: değer} sözlüğüne çevirir."""
    d = {}
    with open(yol, encoding="utf-8") as f:
        for satir in f:
            satir = satir.strip()
            if not satir or satir.startswith("#"):
                continue
            parca = satir.split()
            if len(parca) >= 2:
                d[parca[0].upper()] = parca[1]
    return d


def denetle(etiket, parm_yol, dump_yol):
    """Tek bir çifti denetler. Dönüş: sorunlu parametre sayısı."""
    if not os.path.exists(parm_yol):
        print(f"  ! proje dosyası yok: {parm_yol}")
        return 0
    if not os.path.exists(dump_yol):
        print(f"  ! SITL dökümü yok: {dump_yol}")
        print(f"    (bir kez uçuş yapın; sim_vehicle.py dökümü o zaman yazar)")
        return 0

    parm, sitl = _yukle(parm_yol), _yukle(dump_yol)
    sorun = 0
    print(f"\n=== {etiket} ===")
    print(f"{'PARAMETRE':<20}{'istenen':>10}{'SITL':>12}   durum")
    print("-" * 62)
    for ad, deger in parm.items():
        if ad not in sitl:
            print(f"{ad:<20}{deger:>10}{'YOK':>12}   ✗ TANINMADI (ad yanlış?)")
            sorun += 1
            continue
        try:
            uyum = abs(float(sitl[ad]) - float(deger)) <= 1e-6
        except ValueError:
            uyum = sitl[ad] == deger
        if uyum:
            print(f"{ad:<20}{deger:>10}{sitl[ad]:>12}   ✓")
        else:
            print(f"{ad:<20}{deger:>10}{sitl[ad]:>12}   ✗ UYUŞMUYOR")
            sorun += 1
    print("-" * 62)
    print(f"{len(parm) - sorun}/{len(parm)} uygulandı"
          + (f"  —  {sorun} SORUNLU" if sorun else "  —  hepsi tamam ✓"))
    return sorun


def main():
    print("ArduPilot parametre denetimi — proje .parm dosyaları vs SITL dökümü")
    toplam = sum(denetle(*c) for c in CIFTLER)
    print()
    if toplam:
        print(f"SONUÇ: {toplam} parametre uygulanmıyor.")
        print("Doğru adı bulmak için SITL dökümünde arayın, ör:")
        print("    grep -i 'angle' ~/ardupilot/mav_5_1.parm")
        print("Firmware SI birimine geçmiş olabilir (cm→m, cdeg→deg).")
        return 1
    print("SONUÇ: tüm parametreler uygulanmış ✓")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
