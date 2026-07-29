#!/usr/bin/env python3
"""MRS jinja drone sablonlarini duz SDF'e render eder.

MRS'in kendi spawner'i ROS2 dugumu icinde calisiyor; biz sadece sablonu
render etmek istiyoruz. Loader kokunu models/ yapip drone sablonunu cagiriyoruz.

Kullanim:
    ./11_render_jinja.py x500 t650 m690
"""
import math
import sys
from pathlib import Path

from jinja2 import Environment, FileSystemLoader

ROOT = Path(__file__).resolve().parent.parent
MRS_MODELS = ROOT / "repos" / "mrs_uav_gazebo_simulator" / "models"
OUT_DIR = ROOT / "models" / "interceptors"

# MRS spawner'in sablona verdigi baglam. Sablonlarda kullanilan tek sozluk
# `spawner_args`; anahtarlari: 'name' ve sensor acma/kapama icin 'spawner_keyword'.
# Tum sensorleri kapali biraktik - bize cip ciplak govde + pervaneler lazim,
# kamerayi/taretini kendimiz ekleyecegiz.
def make_context(model_name: str) -> dict:
    return {"spawner_args": {"name": model_name}}


def render(drone: str) -> Path | None:
    tpl_path = f"mrs_robots_description/sdf/drones/{drone}.sdf.jinja"
    if not (MRS_MODELS / tpl_path).exists():
        print(f"  [YOK] {tpl_path}")
        return None

    model_name = f"cand_mrs_{drone}"
    env = Environment(
        loader=FileSystemLoader(str(MRS_MODELS)),
        # MRS'in kendisi de varsayilan (non-strict) Undefined kullaniyor.
        # x500.sdf.jinja:883 tanimsiz `ouster` degiskenine bakiyor (upstream hata);
        # StrictUndefined ile render patliyor, non-strict'te bos sayilip else dalina
        # dusuyor - upstream davranisi bu.
        autoescape=False,
        trim_blocks=True,
        lstrip_blocks=True,
    )
    # MRS'in kendi ortami da math'i global veriyor:
    # mrs_uav_gazebo_simulator/core/jinja_template_manager.py:147
    env.globals["math"] = math
    try:
        sdf = env.get_template(tpl_path).render(**make_context(model_name))
    except Exception as exc:  # noqa: BLE001 - sablon hatalarini rapora yaziyoruz
        print(f"  [HATA] {drone}: {type(exc).__name__}: {exc}")
        return None

    dest_dir = OUT_DIR / f"cand_mrs_{drone}"
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / "model.sdf"
    dest.write_text(sdf, encoding="utf-8")

    (dest_dir / "model.config").write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>cand_mrs_{drone}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>
    CTU MRS {drone} govdesi, jinja sablonundan render edildi.
    Kaynak: ctu-mrs/mrs_uav_gazebo_simulator (ros2 branch)
    Kiyas adayi - ArduPilot'a baglanmadi (PX4 motor modeli ile geliyor).
  </description>
</model>
""",
        encoding="utf-8",
    )
    print(f"  [OK] {drone}: {len(sdf.splitlines())} satir -> {dest.relative_to(ROOT)}")
    return dest


def main() -> int:
    drones = sys.argv[1:] or ["x500", "t650", "m690", "f450", "naki"]
    print(f"MRS jinja render ({MRS_MODELS.name}):")
    rendered = [d for d in drones if render(d)]
    print(f"\n{len(rendered)}/{len(drones)} sablon render edildi.")
    return 0 if rendered else 1


if __name__ == "__main__":
    raise SystemExit(main())
