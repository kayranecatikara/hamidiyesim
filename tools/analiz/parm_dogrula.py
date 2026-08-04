"""
parm_dogrula.py — .parm dosyalarındaki parametre adlarını SITL dökümüyle sınar.

NEDEN VAR: ArduPilot bilinmeyen bir parametre adını SESSİZCE yok sayar — hata
vermez, log'a yazmaz. Bu proje bu tuzağa İKİ KEZ düştü:

  2026-08-01  avci_plane.parm: TRIM_ARSPD_CM / ARSPD_FBW_MIN / ARSPD_FBW_MAX
              yeniden adlandırılmıştı (AIRSPEED_CRUISE / _MIN / _MAX, birim
              cm/s → m/s). Hedefin hız ayarı haftalarca hiçbir şey yapmadı.
  2026-08-04  avci_copter.parm: ANGLE_MAX / WPNAV_SPEED / WPNAV_ACCEL /
              WPNAV_SPEED_UP / WPNAV_SPEED_DN / LOIT_SPEED / PSC_VELXY_P —
              HEPSİ ölü ad. Avcı, WP_ACC'nin varsayılanı olan 2.5 m/s² ivme
              tavanıyla uçuyordu; menzil hiç kapanmadı.

Referans, SITL'in kendi kaydettiği param dökümüdür (mav_<sysid>_1.parm). Orada
olmayan bir ad uçakta/kopterde de YOKTUR — dokümantasyon değil, bu dosya
esastır.

Kullanım (salt okuma, simülasyona bağlanmaz):
    python3 -m tools.analiz.parm_dogrula
    python3 -m tools.analiz.parm_dogrula sim/ardupilot_params/avci_copter.parm

Döküm bulunamazsa: sistemi bir kez başlat (SITL dökümü kendisi yazar), sonra
tekrar çalıştır. AVCI_AP_DIR ile ardupilot dizini değiştirilebilir.
"""

import glob
import os
import re
import sys

# Hangi .parm hangi araca ait — döküm dosyası sysid'e göre adlanır.
# copter sysid 5 → mav_5_1.parm, plane sysid 2 → mav_2_1.parm (CLAUDE.md §6).
_ESLEME = {"avci_copter.parm": "mav_5_1.parm",
           "avci_plane.parm": "mav_2_1.parm"}

_PROJ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PARM_DIZIN = os.path.join(_PROJ, "sim", "ardupilot_params")


def _ap_dizin():
    for aday in (os.environ.get("AVCI_AP_DIR"),
                 os.path.expanduser("~/ardupilot"),
                 os.path.expanduser("~/Masaüstü/ardupilot"),
                 os.path.expanduser("~/Desktop/ardupilot")):
        if aday and os.path.isdir(aday):
            return aday
    return None


def _adlari_oku(yol):
    """Bir .parm dosyasından (satır_no, ad, deger) üçlüleri. Yorumlar atlanır."""
    cikti = []
    with open(yol, encoding="utf-8") as f:
        for i, satir in enumerate(f, 1):
            s = satir.strip()
            if not s or s.startswith("#"):
                continue
            parca = re.split(r"[\s,]+", s, maxsplit=1)
            if len(parca) == 2:
                cikti.append((i, parca[0].strip(), parca[1].strip()))
    return cikti


def _dokum_oku(yol):
    """SITL dökümünden {AD: değer}."""
    d = {}
    with open(yol, encoding="utf-8", errors="replace") as f:
        for satir in f:
            parca = re.split(r"[\s,]+", satir.strip(), maxsplit=1)
            if len(parca) == 2 and parca[0] and not parca[0].startswith("#"):
                d[parca[0].upper()] = parca[1].strip()
    return d


def _benzer(ad, dokum_adlari, en_fazla=4):
    """Ölü bir ad için olası yeni adları öner: ortak parça eşleşmesi."""
    parcalar = [p for p in re.split(r"_", ad.upper()) if len(p) >= 3]
    puanli = []
    for aday in dokum_adlari:
        puan = sum(1 for p in parcalar if p in aday)
        # kısaltılmış biçimler: WPNAV_ACCEL → WP_ACC, PSC_VELXY_P → PSC_NE_VEL_P
        for p in parcalar:
            if len(p) > 3 and any(a.startswith(p[:3]) for a in aday.split("_")):
                puan += 0.5
        if puan:
            puanli.append((puan, aday))
    puanli.sort(key=lambda x: (-x[0], len(x[1])))
    return [a for _, a in puanli[:en_fazla]]


def dogrula(parm_yol, dokum_yol):
    print(f"\n▸ {os.path.basename(parm_yol)}  ↔  {os.path.basename(dokum_yol)}")
    dokum = _dokum_oku(dokum_yol)
    satirlar = _adlari_oku(parm_yol)
    olu, sapan = [], []
    for satir_no, ad, deger in satirlar:
        if ad.upper() not in dokum:
            olu.append((satir_no, ad, deger))
            continue
        # Ad var — değeri gerçekten uygulanmış mı? (yükleme sırası hataları)
        try:
            if abs(float(dokum[ad.upper()]) - float(deger)) > 1e-6:
                sapan.append((satir_no, ad, deger, dokum[ad.upper()]))
        except ValueError:
            pass

    print(f"  {len(satirlar)} parametre okundu, {len(satirlar) - len(olu)} tanesi "
          f"firmware'de VAR.")
    if olu:
        print(f"\n  ✗ ÖLÜ AD ({len(olu)}) — ArduPilot bunları SESSİZCE yok sayıyor:")
        for satir_no, ad, deger in olu:
            oneri = _benzer(ad, dokum)
            ek = ("  → olabilir: " + ", ".join(
                f"{o}={dokum[o]}" for o in oneri)) if oneri else ""
            print(f"      satır {satir_no:>3}  {ad} {deger}{ek}")
        print("      DİKKAT: önerilerin BİRİMİ değişmiş olabilir "
              "(cm/s→m/s, centi-derece→derece).")
    if sapan:
        print(f"\n  ⚠ UYGULANMAMIŞ ({len(sapan)}) — ad geçerli ama döküm başka "
              "değer gösteriyor:")
        for satir_no, ad, istenen, gercek in sapan:
            print(f"      satır {satir_no:>3}  {ad}: istenen {istenen}, "
                  f"dökümde {gercek}")
        print("      Sebep genelde --add-param-file SIRASIDIR: proje dosyası "
              "en sonda olmalı.")
    if not olu and not sapan:
        print("  ✓ Tüm adlar geçerli ve değerler dökümle uyuşuyor.")
    return len(olu) + len(sapan)


def main(argv):
    print("=" * 78)
    print("  PARAMETRE ADI DOĞRULAMA — ölü adlar sessizce yok sayılır")
    print("=" * 78)
    ap = _ap_dizin()
    if ap is None:
        print("HATA: ardupilot dizini bulunamadı (AVCI_AP_DIR ile verilebilir).")
        return 1

    yollar = argv or sorted(glob.glob(os.path.join(_PARM_DIZIN, "*.parm")))
    if not yollar:
        print(f"HATA: {_PARM_DIZIN} altında .parm yok.")
        return 1

    toplam, bakilan = 0, 0
    for yol in yollar:
        dokum_ad = _ESLEME.get(os.path.basename(yol))
        if dokum_ad is None:
            print(f"\n▸ {os.path.basename(yol)} — hangi araca ait bilinmiyor, "
                  "atlandı (_ESLEME'ye ekle).")
            continue
        dokum_yol = os.path.join(ap, dokum_ad)
        if not os.path.exists(dokum_yol):
            print(f"\n▸ {os.path.basename(yol)} — döküm yok ({dokum_yol}).")
            print("    Sistemi bir kez başlat: SITL bu dosyayı kendisi yazar.")
            continue
        toplam += dogrula(yol, dokum_yol)
        bakilan += 1

    print("\n" + "=" * 78)
    if not bakilan:
        print("SONUÇ: doğrulanabilen dosya yok (döküm eksik).")
        return 1
    print(f"SONUÇ: {toplam} sorun" if toplam else "SONUÇ: temiz ✓")
    print("NOT: döküm, parametreleri SİSTEMİN SON ÇALIŞTIĞI andaki haliyle "
          "gösterir.\n     .parm'ı değiştirdikten sonra bir kez uçurup tekrar bak.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
