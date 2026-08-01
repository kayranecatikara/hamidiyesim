"""
analiz_yonelim.py — Pose modelinin YÖNELİM kestirimi ne kadar doğru?

Cevapladığı sorular (bkz. CLAUDE.md §9):
  • Keypoint'ler piksel olarak ne kadar sapıyor, menzille nasıl bozuluyor?
  • `yandanlik` (lead'in BÜYÜKLÜĞÜNÜ belirleyen skaler) ne kadar doğru?
  • Gövde ekseni açısı (lead'in YÖNÜNÜ belirleyen) kaç derece sapıyor?
  • Burun/kuyruk ne sıklıkla takas oluyor? — takas lead'i TAM TERS çevirir ve
    guidance_core'un flip koruması KALICI takası göremez.

Kullanım:  python3 -m tools.analiz.analiz_yonelim [logs/visual_lead_*.csv]
"""

import sys

from tools.analiz import ortak
from vision import geometry as geo

# Menzil bantları: guidance_core'un kalite kapısı 6-14 px ölçek bandına denk
# gelir (≈22 m → ≈10 m). Bantlar o geçişi görecek şekilde seçildi.
MENZIL_KENARLARI = [0, 5, 10, 15, 20, 30, 50, 1000]


def analiz(yol, satirlar):
    if not ortak.dosya_ozeti(yol, satirlar):
        return

    # Yalnız gerçek bir tespitin olduğu kareler anlamlı
    kareler = [s for s in satirlar if s.get("kpt_hata_px_ort") is not None]
    if not kareler:
        print("   Yönelim ölçümü olan kare yok — atlanıyor.")
        return

    ortak.altbaslik("Keypoint piksel hatası (menzil bandına göre)")
    print(f"  {'menzil(m)':<12}{'kare':>6}{'hata_ort':>10}{'hata_med':>10}"
          f"{'hata_max':>10}{'kutu_px':>9}")
    for etiket, grup in ortak.bantla(kareler, "menzil_gercek_gz_m", MENZIL_KENARLARI):
        if not grup:
            continue
        h = ortak.ist(ortak.kolon(grup, "kpt_hata_px_ort"))
        hm = ortak.ist(ortak.kolon(grup, "kpt_hata_px_max"))
        olc = ortak.ist(ortak.kolon(grup, "olcek_gercek"))
        print(f"  {etiket:<12}{h['n']:>6}{h['ort']:>10.2f}{h['med']:>10.2f}"
              f"{hm['ort']:>10.2f}{(olc['ort'] if olc else 0):>9.1f}")

    ortak.altbaslik("Keypoint bazında hata (hangi nokta güvenilmez?)")
    for ad in geo.KEYPOINT_NAMES:
        ortak.ist_yaz(ad, ortak.ist(ortak.kolon(kareler, f"kpt_hata_px_{ad}")), " px")
    print("\n  Not: sol_vtail/sag_vtail güdümde KULLANILMIYOR (yalnız overlay).")
    print("  Burun/kuyruk hatası doğrudan lead YÖNÜNÜ, kanat hatası ÖLÇEĞİ bozar.")

    ortak.altbaslik("Yandanlık — lead'in BÜYÜKLÜĞÜ (model vs gerçek)")
    farklar = []
    for s in kareler:
        m, g = s.get("yandanlik_filtreli"), s.get("yandanlik_gercek")
        if m is not None and g is not None:
            farklar.append(m - g)
    ortak.ist_yaz("yandanlik (model)", ortak.ist(ortak.kolon(kareler, "yandanlik_filtreli")))
    ortak.ist_yaz("yandanlik (gerçek)", ortak.ist(ortak.kolon(kareler, "yandanlik_gercek")))
    ortak.ist_yaz("HATA (model - gerçek)", ortak.ist(farklar))
    ortak.ist_yaz("sin(aspect) [teorik]", ortak.ist(ortak.kolon(kareler, "sin_aspect_gercek")))

    ortak.altbaslik("Gövde ekseni açısı — lead'in YÖNÜ (derece)")
    ortak.ist_yaz("eksen açı hatası", ortak.ist(ortak.kolon(kareler, "eksen_aci_hata_deg")), "°")
    ah = [abs(x) for x in ortak.kolon(kareler, "eksen_aci_hata_deg") if x is not None]
    if ah:
        for esik in (5, 10, 20, 45, 90):
            oran = 100.0 * sum(1 for x in ah if x > esik) / len(ah)
            print(f"    |hata| > {esik:3d}° : %{oran:5.1f}")

    ortak.altbaslik("180° BELİRSİZLİĞİ — burun/kuyruk takası")
    tk = [s.get("burun_kuyruk_takas") for s in kareler
          if s.get("burun_kuyruk_takas") is not None]
    if not tk:
        print("  ölçüm yok")
    else:
        oran = 100.0 * sum(tk) / len(tk)
        print(f"  takas oranı: %{oran:.2f}  ({int(sum(tk))}/{len(tk)} kare)")
        # En kritik soru: takas YAKINDA mı oluyor? Yakında lead büyük, ters
        # lead doğrudan ıskaya götürür.
        for etiket, grup in ortak.bantla(kareler, "menzil_gercek_gz_m", MENZIL_KENARLARI):
            g = [s["burun_kuyruk_takas"] for s in grup
                 if s.get("burun_kuyruk_takas") is not None]
            if g:
                print(f"    {etiket:>8} m : %{100.0 * sum(g) / len(g):5.2f} "
                      f"({int(sum(g))}/{len(g)})")
        if oran > 2.0:
            print("  ⚠ %2'nin üstünde takas: lead işareti güvenilmez. v_tail "
                  "keypoint'leriyle burun/kuyruk ayrımı doğrulanmalı.")

    ortak.altbaslik("Model tarafının flip sayacı (guidance_core koruması)")
    fs = ortak.kolon(kareler, "flip_sayaci")
    fs = [x for x in fs if x is not None]
    if fs:
        print(f"  toplam flip düzeltmesi: {int(max(fs))} "
              f"({100.0 * max(fs) / len(kareler):.1f}% kare başına)")


def main(argv):
    ortak.baslik("YÖNELİM DOĞRULUĞU — pose modeli vs Gazebo gerçeği")
    for yol, satirlar in ortak.yukle_hepsi(argv):
        analiz(yol, satirlar)
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
