#!/usr/bin/env python3
"""ArduPilot SITL_Models'tan alinan skycat_tvbs'i incelemek icin vitrin uretir.

skycat_tvbs bir TVBS (Thrust Vectoring Bi-copter Tailsitter) quadplane; SDF'i
ArduPilotPlugin + 9 adet lift-drag + 6 apply-joint-force eklentisi tasiyor.
Bu eklentiler ArduPilot SITL baglanmadan lock_step yuzunden fizigi ilerletmiyor,
dolayisiyla kamera da kare uretmiyor. Bu yuzden 50_showcase.py'deki desen:
eklentileri sokulmus, statik bir GORUNTU KOPYASI uretiliyor.

Uretilenler:
  models/showcase/skycat_tvbs_display/   (eklentisiz + statik kopya)
  worlds/skycat_view.sdf                 (3 kamerali vitrin dunyasi)

Kullanim:
    ./70_skycat_view.py
    gz sim -r ../worlds/skycat_view.sdf              # canli GUI
    ./51_capture_shots.py --topic /skycat/yan --cikti skycat_yan.png
"""
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "models" / "platforms" / "skycat_tvbs"
DST = ROOT / "models" / "showcase" / "skycat_tvbs_display"
WORLD = ROOT / "worlds" / "skycat_view.sdf"

# (topic, pose, yatay_fov, genislik, yukseklik, aciklama)
KAMERALAR = [
    ("/skycat/yan",   "-0.10 -2.30 0.62 0 0.06 1.5708", 0.75, 1600, 1100, "yandan"),
    ("/skycat/on",    " 2.20  0.00 0.62 0 0.06 3.1416", 0.80, 1400, 1100, "onden"),
    ("/skycat/ust",   " 0.00  0.00 2.60 0 1.5708 0",    0.95, 1400, 1400, "usten"),
]


def strip_plugins(sdf: str) -> str:
    sdf = re.sub(r"<plugin\b.*?</plugin>", "", sdf, flags=re.DOTALL)
    sdf = re.sub(r"<plugin\b[^>]*/>", "", sdf)
    return sdf


def make_display_copy() -> None:
    if DST.exists():
        shutil.rmtree(DST)
    DST.mkdir(parents=True)
    # mesh + doku yollari model://skycat_tvbs_display/... olacagi icin
    # varliklar kopyalaniyor ve URI'ler yeniden yazilliyor.
    shutil.copytree(SRC / "meshes", DST / "meshes")
    shutil.copytree(SRC / "materials", DST / "materials")

    sdf = (SRC / "model.sdf").read_text(encoding="utf-8")
    sdf = strip_plugins(sdf)
    sdf = sdf.replace("model://skycat_tvbs/", "model://skycat_tvbs_display/")
    sdf = re.sub(r"<model name='skycat_tvbs'>",
                 "<model name='skycat_tvbs_display'>", sdf, count=1)
    sdf = re.sub(r"<static>0</static>", "<static>true</static>", sdf, count=1)
    (DST / "model.sdf").write_text(sdf, encoding="utf-8")
    (DST / "model.config").write_text(
        """<?xml version="1.0"?>
<model>
  <name>skycat_tvbs_display</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>skycat_tvbs vitrin kopyasi: eklentiler cikarildi, statik yapildi.</description>
</model>
""", encoding="utf-8")
    print(f"  [OK] {DST.relative_to(ROOT)}")


def kamera_modeli(topic: str, pose: str, fov: float, w: int, h: int, ad: str) -> str:
    isim = "cam_" + topic.rsplit("/", 1)[-1]
    return f"""
    <!-- {ad} -->
    <model name="{isim}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>{topic}</topic>
          <camera>
            <horizontal_fov>{fov}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>200</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


def make_world() -> None:
    kameralar = "".join(kamera_modeli(*k) for k in KAMERALAR)
    WORLD.write_text(f"""<?xml version="1.0" ?>
<!--
  SKYCAT TVBS VITRINI - scripts/70_skycat_view.py tarafindan uretildi.

  ArduPilot/SITL_Models'taki skycat_tvbs modelinin statik kopyasi, uc kameradan
  (yan / on / ust) incelenmek uzere kaide uzerinde asili.

  Kullanim:
    source scripts/env.sh
    gz sim -r worlds/skycat_view.sdf
    python3 scripts/51_capture_shots.py --topic /skycat/yan --cikti skycat_yan.png
-->
<sdf version="1.9">
  <world name="skycat_view">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.70 0.70 0.74 1</ambient>
      <background>0.55 0.68 0.85 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.95 0.95 0.92 1</diffuse>
      <specular>0.35 0.35 0.35 1</specular>
      <direction>-0.4 0.35 -0.85</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.40 0.40 0.46 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.6 -0.5 -0.6</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material>
            <ambient>0.32 0.34 0.36 1</ambient>
            <diffuse>0.42 0.44 0.46 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Kaide: modeli 0.6 m'de asili tutan gorsel silindir -->
    <model name="pedestal">
      <static>true</static>
      <pose>0 0 0.20 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.09</radius><length>0.40</length></cylinder></geometry>
          <material>
            <ambient>0.15 0.45 0.75 1</ambient>
            <diffuse>0.20 0.55 0.85 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <include>
      <pose>0 0 0.60 0 0 0</pose>
      <uri>model://skycat_tvbs_display</uri>
    </include>
{kameralar}
  </world>
</sdf>
""", encoding="utf-8")
    print(f"  [OK] {WORLD.relative_to(ROOT)}")


if __name__ == "__main__":
    print("skycat_tvbs vitrini uretiliyor:")
    make_display_copy()
    make_world()
