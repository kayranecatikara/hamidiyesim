#!/usr/bin/env python3
"""Tareti gz-transport uzerinden nisanlar (Asama 3).

Otopilottan bagimsiz: eklem aci komutlarini dogrudan
gz-sim-joint-position-controller-system'e yollar.

Kullanim:
    ./turret_aim.py <pan_derece> <tilt_derece>
    ./turret_aim.py 20 -10          # 20 derece saga, 10 derece yukari
    ./turret_aim.py --hedef 10 3 1.5   # hedefin govde cerceve x y z'sine nisanla
    ./turret_aim.py 0 -8 --model bullet_net_interceptor   # mermi govdeli surum

Aci sozlesmesi:
    pan  (+) : sola/saat yonunun tersi (Z ekseni)
    tilt (-) : namlu yukari  (SDF'te +pitch burnu asagi indirir)
"""
import argparse
import math
import subprocess
import sys

VARSAYILAN_MODEL = "bullet_net_interceptor"
YAW_LIMIT_DEG = 100.0
TILT_MIN_DEG, TILT_MAX_DEG = -60.0, 30.0

# Namlunun govde cercevesindeki konumu. Eklem limitleri iki govdede ayni,
# ama namlu yeri farkli: --hedef ile nokta nisanlamasi bunu kullanir.
#   bullet_net_interceptor: dikey mermi govde, taret TEPEDE
NAMLU_KONUMU = {
    "bullet_net_interceptor": (0.11, 0.0, 0.310),
}


def cmd_topic(model: str, joint: str) -> str:
    return f"/model/{model}/joint/{joint}/0/cmd_pos"


def publish(model: str, joint: str, radians: float) -> bool:
    topic = cmd_topic(model, joint)
    proc = subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double",
         "-p", f"data: {radians:.6f}"],
        capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"  HATA {topic}: {proc.stderr.strip()}", file=sys.stderr)
        return False
    print(f"  {joint:20s} <- {math.degrees(radians):+7.2f} deg  ({topic})")
    return True


def clamp(val: float, lo: float, hi: float, adi: str) -> float:
    if val < lo or val > hi:
        print(f"  UYARI: {adi} {val:+.1f} deg sinir disi, [{lo}, {hi}] araligina kirpildi")
    return max(lo, min(hi, val))


def aim_at_point(model: str, x: float, y: float, z: float) -> tuple[float, float]:
    """Govde cercevesindeki bir noktaya nisan acilarini hesaplar."""
    mx, my, mz = NAMLU_KONUMU[model]
    dx, dy, dz = x - mx, y - my, z - mz
    pan = math.degrees(math.atan2(dy, dx))
    # tilt: yukari bakmak icin negatif
    tilt = -math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    print(f"  hedef ({x}, {y}, {z}) -> menzil {math.sqrt(dx*dx+dy*dy+dz*dz):.2f} m")
    return pan, tilt


def main() -> int:
    ap = argparse.ArgumentParser(description="Taret nisanlama")
    ap.add_argument("a", type=float, nargs="?", help="pan derece (veya --hedef ile x)")
    ap.add_argument("b", type=float, nargs="?", help="tilt derece (veya --hedef ile y)")
    ap.add_argument("c", type=float, nargs="?", help="--hedef ile z")
    ap.add_argument("--hedef", action="store_true",
                    help="acilar yerine govde cercevesinde x y z noktasi ver")
    ap.add_argument("--model", default=VARSAYILAN_MODEL,
                    choices=sorted(NAMLU_KONUMU),
                    help=f"hedef model (varsayilan: {VARSAYILAN_MODEL})")
    args = ap.parse_args()

    if args.hedef:
        if args.c is None:
            ap.error("--hedef icin uc deger gerekli: x y z")
        pan_deg, tilt_deg = aim_at_point(args.model, args.a, args.b, args.c)
    else:
        if args.b is None:
            ap.error("iki deger gerekli: pan tilt (derece)")
        pan_deg, tilt_deg = args.a, args.b

    pan_deg = clamp(pan_deg, -YAW_LIMIT_DEG, YAW_LIMIT_DEG, "pan")
    tilt_deg = clamp(tilt_deg, TILT_MIN_DEG, TILT_MAX_DEG, "tilt")

    print(f"Nisan [{args.model}]: pan {pan_deg:+.2f} deg, tilt {tilt_deg:+.2f} deg")
    ok = publish(args.model, "turret_yaw_joint", math.radians(pan_deg))
    ok &= publish(args.model, "turret_pitch_joint", math.radians(tilt_deg))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
