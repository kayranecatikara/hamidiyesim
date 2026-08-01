"""
analiz_menzil.py — Ölçekten türeyen MENZİL kestirimi ne kadar doğru?

Üç değeri ayrı ayrı karşılaştırır; farkları üç FARKLI hatayı ayırır:

  menzil_kestirim_m       modelin keypoint'lerinden (guidance_core hesaplar)
  menzil_olcek_gercek_m   GERÇEK keypoint'lerden, aynı formülle
  menzil_gercek_gz_m      Gazebo fiziği (hata payı sıfır)

    kestirim − olcek_gercek  →  POSE MODELİNİN hatası
    olcek_gercek − gz        →  FORMÜLÜN kendi hatası (sentetikte < %2)
    kestirim − gz            →  toplam hata (güdümün gördüğü)

Ayrıca `menzil_gercek_m` (MAVLink telemetrisi) ile `menzil_gercek_gz_m`
karşılaştırılır — aradaki fark EKF + çerçeve kalibrasyon hatasıdır ve vuruş
tespitinin dayandığı veri odur.

Kullanım:  python3 -m tools.analiz.analiz_menzil [logs/visual_lead_*.csv]
"""

import sys

from tools.analiz import ortak

MENZIL_KENARLARI = [0, 5, 10, 15, 20, 30, 50, 1000]


def _fark(satirlar, a, b):
    """a - b farkları (ikisi de doluysa)."""
    out = []
    for s in satirlar:
        x, y = s.get(a), s.get(b)
        if x is not None and y is not None:
            out.append(x - y)
    return out


def _bagil(satirlar, a, b):
    """(a - b) / b yüzde farkları."""
    out = []
    for s in satirlar:
        x, y = s.get(a), s.get(b)
        if x is not None and y is not None and abs(y) > 1e-6:
            out.append(100.0 * (x - y) / y)
    return out


def analiz(yol, satirlar):
    if not ortak.dosya_ozeti(yol, satirlar):
        return
    kareler = [s for s in satirlar if s.get("menzil_kestirim_m") is not None
               and s.get("menzil_gercek_gz_m") is not None]
    if not kareler:
        print("   Menzil karşılaştırması olan kare yok — atlanıyor.")
        return

    ortak.altbaslik("Hata ayrıştırması (metre)")
    ortak.ist_yaz("POSE MODELİ  (kestirim−ölçek_ger)",
                  ortak.ist(_fark(kareler, "menzil_kestirim_m", "menzil_olcek_gercek_m")), " m")
    ortak.ist_yaz("FORMÜL       (ölçek_ger−gz)",
                  ortak.ist(_fark(kareler, "menzil_olcek_gercek_m", "menzil_gercek_gz_m")), " m")
    ortak.ist_yaz("TOPLAM       (kestirim−gz)",
                  ortak.ist(_fark(kareler, "menzil_kestirim_m", "menzil_gercek_gz_m")), " m")
    ortak.ist_yaz("TELEMETRİ    (mavlink−gz)",
                  ortak.ist(_fark(kareler, "menzil_gercek_m", "menzil_gercek_gz_m")), " m")

    ortak.altbaslik("Bağıl hata (%) — menzil bandına göre")
    print(f"  {'menzil(m)':<12}{'kare':>6}{'toplam%':>10}{'model%':>10}"
          f"{'formül%':>10}{'ölçek_px':>10}")
    for etiket, grup in ortak.bantla(kareler, "menzil_gercek_gz_m", MENZIL_KENARLARI):
        if not grup:
            continue
        t = ortak.ist(_bagil(grup, "menzil_kestirim_m", "menzil_gercek_gz_m"))
        m = ortak.ist(_bagil(grup, "menzil_kestirim_m", "menzil_olcek_gercek_m"))
        f = ortak.ist(_bagil(grup, "menzil_olcek_gercek_m", "menzil_gercek_gz_m"))
        o = ortak.ist(ortak.kolon(grup, "olcek"))
        print(f"  {etiket:<12}{len(grup):>6}"
              f"{(t['ort'] if t else 0):>+10.1f}{(m['ort'] if m else 0):>+10.1f}"
              f"{(f['ort'] if f else 0):>+10.1f}{(o['ort'] if o else 0):>10.1f}")

    ortak.altbaslik("Menzilin güdüme dolaylı etkisi — `kalite` kapısı")
    # kalite = (olcek - 6) / (14 - 6); 0 iken lead TAMAMEN sönük.
    kal = [s.get("kalite") for s in kareler if s.get("kalite") is not None]
    if kal:
        sifir = 100.0 * sum(1 for k in kal if k <= 0.001) / len(kal)
        tam = 100.0 * sum(1 for k in kal if k >= 0.999) / len(kal)
        print(f"  kalite = 0 (lead TAMAMEN sönük) : %{sifir:.1f} kare")
        print(f"  kalite = 1 (tam lead)           : %{tam:.1f} kare")
        ortak.ist_yaz("kalite", ortak.ist(kal))
        if sifir > 50:
            print("  ⚠ Karelerin yarısından fazlasında lead sönük — OLCEK_KAPALI_PX "
                  "(6 px ≈ 22 m) bandı bu uçuş için fazla dar olabilir.")

    ortak.altbaslik("Yükselti düzeltmesi (Adım 3) çalışıyor mu")
    ortak.ist_yaz("eps (model, LOS yükselişi)", ortak.ist(ortak.kolon(kareler, "eps_deg")), "°")
    ortak.ist_yaz("eps (gerçek)", ortak.ist(ortak.kolon(kareler, "eps_gercek_deg")), "°")
    ortak.ist_yaz("düzeltme katsayısı", ortak.ist(ortak.kolon(kareler, "duzeltme")))


def main(argv):
    ortak.baslik("MENZİL DOĞRULUĞU — ölçekten kestirim vs Gazebo gerçeği")
    for yol, satirlar in ortak.yukle_hepsi(argv):
        analiz(yol, satirlar)
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
