#!/usr/bin/env python3
"""Taret eklem acilarini derece cinsinden okur (dogrulama araci).

Kullanim:  ./turret_state.py [dunya_adi] [model_adi]
           ./turret_state.py bullet_net_test bullet_net_interceptor
"""
import math
import re
import subprocess
import sys

VARSAYILAN_MODEL = "avci_net_interceptor"


def blocks(txt: str):
    """Dengeli parantezle ust seviye { ... } bloklarini ayirir.

    Duz regex kullanilamaz: joint bloklari ic ice (axis1 { position: ... }).
    """
    depth, start = 0, None
    for i, ch in enumerate(txt):
        if ch == "{":
            if depth == 0:
                start = i
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0 and start is not None:
                yield txt[start:i + 1]
                start = None


def read_angles(world: str, model: str) -> dict[str, float]:
    topic = f"/world/{world}/model/{model}/joint_state"
    proc = subprocess.run(["gz", "topic", "-e", "-t", topic, "-n", "1"],
                          capture_output=True, text=True, timeout=10)
    out = {}
    for b in blocks(proc.stdout):
        n = re.search(r'name:\s*"([^"]+)"', b)
        p = re.search(r"position:\s*([-\d.eE+]+)", b)
        if n and p and "turret" in n.group(1):
            out[n.group(1)] = math.degrees(float(p.group(1)))
    return out


def main() -> int:
    world = sys.argv[1] if len(sys.argv) > 1 else "net_test"
    model = sys.argv[2] if len(sys.argv) > 2 else VARSAYILAN_MODEL
    try:
        angles = read_angles(world, model)
    except subprocess.TimeoutExpired:
        print("HATA: joint_state topic'inden veri gelmedi (sim calisiyor mu?)")
        return 1
    if not angles:
        print("HATA: taret eklemi bulunamadi")
        return 1
    for name, deg in angles.items():
        print(f"  {name:22s} {deg:+8.2f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
