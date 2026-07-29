#!/usr/bin/env python3
"""Balistik dunyasinin bir kopyasini, ag askisi verilen tilt acisiyla egilmis
halde uretir.

Atis yonu artik NetLauncherPlugin tarafindan NAMLUNUN YONELIMINDEN okunuyor
(disaridan aci parametresi yok). Aci taramasi yapabilmek icin askinin
kendisini egiyoruz - gercek taretin yaptigi sey de bu.

Kullanim:  ./tilt_world.py <kaynak.sdf> <hedef.sdf> <tilt_derece>
"""
import math
import re
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    src, dst, tilt_deg = Path(sys.argv[1]), Path(sys.argv[2]), float(sys.argv[3])

    # tilt(-) = namlu yukari. SDF'te pitch(+) burnu ASAGI indirir,
    # dolayisiyla isaret dogrudan gecer.
    pitch = math.radians(tilt_deg)

    s = src.read_text(encoding="utf-8")
    pattern = r'(<model name="net_anchor">\s*<static>true</static>\s*)<pose>[^<]*</pose>'
    new, n = re.subn(pattern,
                     lambda m: f'{m.group(1)}<pose>0 0 10 0 {pitch:.6f} 0</pose>',
                     s)
    if n != 1:
        print(f"HATA: net_anchor pose'u bulunamadi (eslesme={n})")
        return 1

    dst.write_text(new, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
