#!/usr/bin/env python3
"""Yorunge CSV'sinden ozet sayilar cikarir: menzil, ucus suresi, tepe z, cikis hizi.

Cikti tek satir, bosluk ayrilmis:  <menzil_m> <sure_s> <tepe_z_m> <v0_ms>
41_range_test.sh bunu `read` ile okur.
"""
import csv
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print("0 0 0 0")
        return 2
    path = Path(sys.argv[1])
    if not path.exists():
        print("0 0 0 0")
        return 1

    rows = list(csv.DictReader(path.open()))
    if len(rows) < 2:
        print("0 0 0 0")
        return 1

    t = [float(r["t_sim_s"]) for r in rows]
    x = [float(r["x_m"]) for r in rows]
    z = [float(r["z_m"]) for r in rows]

    # Yere degdigi ilk ornekte kes (sekmeleri menzile katma)
    son = len(x) - 1
    for i in range(1, len(z)):
        if z[i] <= 0.05:
            son = i
            break

    menzil = x[son] - x[0]
    sure = t[son] - t[0]
    tepe = max(z[:son + 1])

    # Cikis hizi: atisin hemen ardindaki ilk 0.1 sn
    v0 = 0.0
    t_atis = None
    for i in range(1, len(x)):
        if abs(x[i] - x[0]) > 0.01:
            t_atis = t[i - 1]
            break
    if t_atis is not None:
        pencere = [(tt, xx) for tt, xx in zip(t, x) if t_atis <= tt <= t_atis + 0.1]
        if len(pencere) >= 2:
            dt = pencere[-1][0] - pencere[0][0]
            if dt > 1e-6:
                v0 = (pencere[-1][1] - pencere[0][1]) / dt

    print(f"{menzil:.2f} {sure:.2f} {tepe:.2f} {v0:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
