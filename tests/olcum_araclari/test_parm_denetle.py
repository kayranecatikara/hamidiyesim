"""tools/parm_denetle.py — parametre uygulandı mı denetimi.

    python3 -m tests.olcum_araclari.test_parm_denetle

Bu araç, ArduPilot'un tanımadığı parametre adını SESSİZCE yok saymasına karşı
yazıldı (9 parametrenin 7'si aylarca uygulanmamıştı). Kendisi sessizce
yanılırsa aynı arıza geri döner — bu yüzden test ediliyor.

P1-P2  _yukle    yorum/boş satır ayıklama, ad normalleştirme
P3     denetle   biçim farkı UYUŞMAZLIK DEĞİL (45 ↔ 45.000000)
P4     denetle   tanınmayan ad yakalanıyor
P5     denetle   değer uyuşmazlığı yakalanıyor
P6     denetle   dosya yoksa sessizce 0 (uçuş öncesi normal durum)
"""

import os
import tempfile

from tests.olcum_araclari.ortak import kontrol, ozet, sifirla
from tools import parm_denetle as pd


def _yaz(dizin, ad, icerik):
    yol = os.path.join(dizin, ad)
    with open(yol, "w", encoding="utf-8") as f:
        f.write(icerik)
    return yol


def main():
    sifirla()
    print("Ölçüm aracı: parm_denetle (parametre uygulandı mı)")
    print("=" * 60)
    tmp = tempfile.mkdtemp(prefix="avci_parm_test_")

    # ── P1: yorumlar ve boş satırlar ayıklanmalı ──
    yol = _yaz(tmp, "yorumlu.parm",
               "# başlık yorumu\n\nATC_ANGLE_MAX 45\n"
               "  # girintili yorum\nWP_ACC_Z 1.0\n\n")
    d = pd._yukle(yol)
    kontrol("P1  yorum ve boş satırlar ayıklanıyor",
            d == {"ATC_ANGLE_MAX": "45", "WP_ACC_Z": "1.0"},
            f"2 parametre okundu: {sorted(d)}")

    # ── P2: parametre adları büyük harfe normalleştirilmeli ──
    # SITL dökümü ile proje dosyası farklı yazımda olabilir; normalleşmezse
    # her parametre "TANINMADI" görünür ve araç sahte alarm verir.
    yol = _yaz(tmp, "kucuk.parm", "atc_angle_max 45\nWp_Acc_Z 1.0\n")
    d = pd._yukle(yol)
    kontrol("P2  parametre adları büyük harfe normalleşiyor",
            d == {"ATC_ANGLE_MAX": "45", "WP_ACC_Z": "1.0"},
            f"küçük harfli giriş → {sorted(d)}")

    # ── P3: SITL dökümü 45.000000 yazar, proje dosyası 45 — SORUN DEĞİL ──
    # Metin karşılaştırması yapılsaydı her parametre "UYUŞMUYOR" derdi.
    parm = _yaz(tmp, "a.parm", "ATC_ANGLE_MAX 45\nWP_ACC_Z 1.0\n")
    dump = _yaz(tmp, "a_dump.parm", "ATC_ANGLE_MAX 45.000000\nWP_ACC_Z 1.000000\n")
    kontrol("P3  sayısal biçim farkı uyuşmazlık sayılmıyor",
            pd.denetle("P3", parm, dump) == 0, "45 ↔ 45.000000 → sorun yok")

    # ── P4: dökümde HİÇ olmayan ad = firmware tanımadı (asıl yakalanacak arıza) ──
    parm = _yaz(tmp, "b.parm", "ATC_ANGLE_MAX 45\nESKI_AD_YOK 3\n")
    dump = _yaz(tmp, "b_dump.parm", "ATC_ANGLE_MAX 45.0\n")
    kontrol("P4  tanınmayan parametre adı yakalanıyor",
            pd.denetle("P4", parm, dump) == 1, "1 parametre dökümde yok → 1 sorun")

    # ── P5: ad tanınmış ama değer tutmuyor (parm dosyası hiç yüklenmemiş olabilir) ──
    parm = _yaz(tmp, "c.parm", "ATC_ANGLE_MAX 45\n")
    dump = _yaz(tmp, "c_dump.parm", "ATC_ANGLE_MAX 30.0\n")
    kontrol("P5  değer uyuşmazlığı yakalanıyor",
            pd.denetle("P5", parm, dump) == 1, "istenen 45, SITL'de 30 → 1 sorun")

    # ── P6: uçuştan önce döküm dosyası yoktur — sahte alarm vermemeli ──
    kontrol("P6  döküm dosyası yokken sorun raporlanmıyor",
            pd.denetle("P6", parm, os.path.join(tmp, "olmayan.parm")) == 0,
            "uçuş öncesi normal durum, 0 sorun")

    return ozet("parm_denetle")


if __name__ == "__main__":
    import sys
    gecen, toplam = main()
    sys.exit(0 if gecen == toplam else 1)
