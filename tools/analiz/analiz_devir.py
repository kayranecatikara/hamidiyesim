"""
analiz_devir.py — GPS fazı görsel fazı DOĞRU GEOMETRİDE mi devrediyor?

Planın omurgasındaki soru: GPS fazı hedefin tam arkasına yerleşiyor
(`APPROACH_STANDOFF=10 m`, 5 m alt). Tam kuyruktan bakışta gövde ekseni
kısalır → `yandanlik → 0` → `lead → 0`. Yani görsel faz, lead yasasının hiç
bilgi alamadığı geometride devralıyor olabilir.

Ölçüm yolu: `run_visual_lead` her görsel faz girişinde YENİ bir CSV açar.
Dolayısıyla her dosya = bir devir; dosyanın İLK satırları = devir anındaki
geometri. Son satır ve `durum` kolonu da o denemenin sonucunu verir.

Not: `LOOKUP_ELEV_DEG` + `APPROACH_ALT_OFFSET` avcıyı hedefin ALTINA koyduğu
için "tam kuyruk" aslında saf 180° değildir; alttan bakış aspect'i 180°'den
uzaklaştırır ve yandanlığı kendiliğinden yükseltir. Bu scriptin ölçtüğü şey,
o kurtarmanın YETERLİ olup olmadığıdır.

Kullanım:  python3 -m tools.analiz.analiz_devir [logs/visual_lead_*.csv]
"""

import os
import sys

from tools.analiz import ortak

# Devir anı = ilk bu kadar geçerli kare (30 Hz'te ~0.5 s)
DEVIR_KARE = 15


def _ilk_gecerli(satirlar, n):
    """İlk n adet, doğruluk ölçümü dolu kare."""
    out = []
    for s in satirlar:
        if s.get("yandanlik_gercek") is not None:
            out.append(s)
            if len(out) >= n:
                break
    return out


def analiz(yol, satirlar):
    ad = os.path.basename(yol)
    if not ortak.dosya_ozeti(yol, satirlar):
        return None

    devir = _ilk_gecerli(satirlar, DEVIR_KARE)
    if not devir:
        print("   Devir anında doğruluk ölçümü yok — atlanıyor.")
        return None

    # ── Devir anı geometrisi ──
    asp = ortak.ist(ortak.kolon(devir, "aspect_gercek_deg"))
    yan_g = ortak.ist(ortak.kolon(devir, "yandanlik_gercek"))
    yan_m = ortak.ist(ortak.kolon(devir, "yandanlik_filtreli"))
    lead = ortak.ist(ortak.kolon(devir, "lead_deg"))
    menz = ortak.ist(ortak.kolon(devir, "menzil_gercek_gz_m"))
    olc = ortak.ist(ortak.kolon(devir, "olcek"))
    a_px = ortak.ist(ortak.kolon(devir, "a_gercek"))
    kal = ortak.ist(ortak.kolon(devir, "kalite"))

    ortak.altbaslik(f"DEVİR ANI (ilk {len(devir)} kare)")
    ortak.ist_yaz("menzil (gerçek)", menz, " m")
    ortak.ist_yaz("aspect açısı", asp, "°")
    ortak.ist_yaz("yandanlik (gerçek)", yan_g)
    ortak.ist_yaz("yandanlik (model)", yan_m)
    ortak.ist_yaz("a  — gövde ekseni px", a_px, " px")
    ortak.ist_yaz("ölçek (düzeltilmiş) px", olc, " px")
    ortak.ist_yaz("kalite kapısı", kal)
    ortak.ist_yaz("ÜRETİLEN LEAD", lead, "°")

    # ── Teşhis ──
    print()
    if a_px and a_px["ort"] < 2.0:
        print("  ⚠ a < MIN_GOVDE_PX(2.0): lead deadband'e düşüyor → SAF TAKİP.")
    if kal and kal["ort"] < 0.05:
        print("  ⚠ kalite ≈ 0: ölçek OLCEK_KAPALI_PX(6 px) altında → lead sönük.")
    if lead and lead["ort"] < 1.0:
        print("  ⚠ Devirde lead ≈ 0 — görsel faz saf takiple başlıyor.")
    if yan_g and yan_g["ort"] > 0.3 and lead and lead["ort"] < 1.0:
        print("  ⚠ Geometri lead'e MÜSAİT (yandanlik>0.3) ama lead üretilmiyor:")
        print("    darboğaz geometri değil, kalite/güven kapıları.")
    if yan_g and yan_g["ort"] < 0.2:
        print("  → Geometri gerçekten dejenere (kuyruk kör noktası).")
        print("    Çare: istasyonu yana kaydır (çeyrek yaklaşma) veya alttan")
        print("    bakış ofsetini artır (APPROACH_ALT_OFFSET / LOOKUP_ELEV_DEG).")

    # ── Faz boyunca ──
    tum = [s for s in satirlar if s.get("yandanlik_gercek") is not None]
    ortak.altbaslik("GÖRSEL FAZ BOYUNCA")
    ortak.ist_yaz("aspect açısı", ortak.ist(ortak.kolon(tum, "aspect_gercek_deg")), "°")
    ortak.ist_yaz("yandanlik (gerçek)", ortak.ist(ortak.kolon(tum, "yandanlik_gercek")))
    ortak.ist_yaz("lead", ortak.ist(ortak.kolon(tum, "lead_deg")), "°")
    ortak.ist_yaz("kapanma hızı", ortak.ist(ortak.kolon(satirlar, "kapanma_hizi_ms")), " m/s")

    en_yakin = min((s["menzil_gercek_gz_m"] for s in satirlar
                    if s.get("menzil_gercek_gz_m") is not None), default=None)
    durumlar = [s.get("durum") for s in satirlar]
    sonuc = ("VURULDU" if "vuruldu" in durumlar
             else "kör dalış + ıska" if "kor_dalis" in durumlar
             else "temas kaybı/durduruldu")
    print(f"\n  SONUÇ: {sonuc}"
          + (f"   en yakın menzil: {en_yakin:.2f} m" if en_yakin is not None else ""))

    return {"ad": ad, "aspect": asp["ort"] if asp else None,
            "yandanlik": yan_g["ort"] if yan_g else None,
            "lead": lead["ort"] if lead else None,
            "menzil": menz["ort"] if menz else None,
            "sonuc": sonuc}


def main(argv):
    ortak.baslik("DEVİR GEOMETRİSİ — GPS fazı görsel fazı nerede devrediyor?")
    ozetler = []
    for yol, satirlar in ortak.yukle_hepsi(argv):
        o = analiz(yol, satirlar)
        if o:
            ozetler.append(o)

    if len(ozetler) > 1:
        ortak.baslik("TÜM DEVİRLER — özet")
        print(f"  {'dosya':<34}{'menzil':>8}{'aspect':>8}{'yandan':>8}"
              f"{'lead':>7}  sonuç")
        for o in ozetler:
            print(f"  {o['ad'][:33]:<34}{(o['menzil'] or 0):>8.1f}"
                  f"{(o['aspect'] or 0):>8.1f}{(o['yandanlik'] or 0):>8.2f}"
                  f"{(o['lead'] or 0):>7.1f}  {o['sonuc']}")
        vurus = sum(1 for o in ozetler if o["sonuc"] == "VURULDU")
        print(f"\n  Vuruş oranı: {vurus}/{len(ozetler)}")
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
