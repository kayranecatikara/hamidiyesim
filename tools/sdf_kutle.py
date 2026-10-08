#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SDF model ağacındaki TÜM <mass> değerlerini toplar (include'lar dahil).

NİYE: `--kutle` argümanını elle yazmak hata kaynağı. Model değişince toplam
kütle de değişir; ölçüm aracına yanlış kütle verilirse T_max yanlış çıkar.
"""
import os
import re
import sys

KOK = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "sim", "gazebo_harmonic", "models")


def topla(model, gorulen=None, derinlik=0):
    gorulen = gorulen if gorulen is not None else set()
    if model in gorulen:
        return 0.0
    gorulen.add(model)
    yol = os.path.join(KOK, model, "model.sdf")
    if not os.path.exists(yol):
        print("  ⛔ bulunamadı: %s" % yol)
        return 0.0
    ham = open(yol, encoding="utf-8").read()
    # yorum bloklarını at — iris_cam'de dev bir <!-- ... --> hata ayıklama bloğu var
    net = re.sub(r"<!--.*?-->", "", ham, flags=re.S)
    toplam = 0.0
    for eslesme in re.finditer(r"<mass>\s*([0-9.eE+-]+)\s*</mass>", net):
        toplam += float(eslesme.group(1))
    print("%s%-24s : %.4f kg (kendi linkleri)" % ("  " * derinlik, model, toplam))
    for alt in re.findall(r"<uri>model://([^/<]+)</uri>", net):
        toplam += topla(alt, gorulen, derinlik + 1)
    return toplam


if __name__ == "__main__":
    ad = sys.argv[1] if len(sys.argv) > 1 else "iris_cam"
    print("=" * 60)
    t = topla(ad)
    print("=" * 60)
    print("  TOPLAM KÜTLE: %.4f kg" % t)
