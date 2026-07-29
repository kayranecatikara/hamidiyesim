#!/usr/bin/env python3
"""Tum interceptor adaylarini yan yana dizen bir 'vitrin' dunyasi uretir.

Adaylar SDF'lerinde ArduPilotPlugin, motor eklentileri vb. tasiyor. Ayni
dunyada yan yana koyunca hepsi ayni FDM portuna baglanmaya calisiyor ve
konsol hata yagiyor; ustelik modeller yerde savruluyor. Bu yuzden vitrin icin
her adayin GORUNTU KOPYASI uretiliyor:
  - tum <plugin> bloklari cikariliyor
  - <static>true</static> ekleniyor  (havada asili dursunlar)

Ayrica sahneye bir kamera konuyor; 51_capture_shots.py bu kameradan
ekran goruntusu aliyor.

Kullanim:  ./50_showcase.py
"""
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
INTERCEPTORS = ROOT / "models" / "interceptors"
SHOWCASE = ROOT / "models" / "showcase"
WORLD = ROOT / "worlds" / "showcase.sdf"

# (klasor_adi, ekranda gorunecek etiket, renk RGBA)
ADAYLAR = [
    ("avci_net_interceptor", "AVCI NET INTERCEPTOR (taretli+ag)", "0.9 0.4 0.05 1"),
    ("cand_iris",            "cand_iris (secilen govde)",          "0.2 0.7 0.2 1"),
    ("cand_iq_camera",       "cand_iq_camera (iq_sim)",            "0.2 0.5 0.9 1"),
    ("cand_mrs_x500",        "cand_mrs_x500 (CTU MRS)",            "0.9 0.8 0.1 1"),
    ("cand_mrs_t650",        "cand_mrs_t650 (CTU MRS)",            "0.9 0.8 0.1 1"),
    ("cand_mrs_m690",        "cand_mrs_m690 (CTU MRS)",            "0.9 0.8 0.1 1"),
    ("cand_mrs_f450",        "cand_mrs_f450 (CTU MRS)",            "0.9 0.8 0.1 1"),
    ("cand_mrs_naki",        "cand_mrs_naki (CTU MRS)",            "0.9 0.8 0.1 1"),
]

ARALIK = 1.9   # adaylar arasi mesafe [m]
YUKSEKLIK = 1.2


def strip_plugins(sdf: str) -> str:
    """Tum <plugin ...>...</plugin> bloklarini ve tekil <plugin .../> etiketlerini siler."""
    sdf = re.sub(r"<plugin\b.*?</plugin>", "", sdf, flags=re.DOTALL)
    sdf = re.sub(r"<plugin\b[^>]*/>", "", sdf)
    return sdf


def make_display_copy(name: str) -> bool:
    src = INTERCEPTORS / name
    if not (src / "model.sdf").exists():
        print(f"  [YOK] {name}")
        return False

    dst = SHOWCASE / f"{name}_display"
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True)

    sdf = (src / "model.sdf").read_text(encoding="utf-8")
    sdf = strip_plugins(sdf)

    # Model adini degistir + statik yap
    sdf = re.sub(r'(<model name=")([^"]+)(">)',
                 lambda m: f'{m.group(1)}{name}_display{m.group(3)}\n    <static>true</static>',
                 sdf, count=1)

    # DIS modeli statik yapmak YETMIYOR: iris turevleri govdelerini <include>
    # ile getiriyor ve o alt model dinamik kaldigi icin kaideden yere dusuyordu
    # (ilk vitrin denemesinde ilk uc model zeminde cikti).
    sdf = re.sub(r'(<include>)', r'\1\n      <static>true</static>', sdf)

    (dst / "model.sdf").write_text(sdf, encoding="utf-8")
    (dst / "model.config").write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>{name}_display</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{name} vitrin kopyasi: eklentiler cikarildi, statik yapildi.</description>
</model>
""", encoding="utf-8")
    print(f"  [OK] {name}_display")
    return True


def make_net_copy() -> bool:
    """Agin vitrin kopyasi: eklentiler cikarilmis, statik."""
    src = ROOT / "models" / "net_launchers" / "net_cone" / "model.sdf"
    if not src.exists():
        return False
    dst = SHOWCASE / "net_cone_display"
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True)
    sdf = strip_plugins(src.read_text(encoding="utf-8"))
    sdf = sdf.replace('<model name="net_cone">',
                      '<model name="net_cone_display">\n    <static>true</static>', 1)
    (dst / "model.sdf").write_text(sdf, encoding="utf-8")
    (dst / "model.config").write_text(
        '''<?xml version="1.0"?>
<model>
  <name>net_cone_display</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>net_cone vitrin kopyasi (statik, eklentisiz).</description>
</model>
''', encoding="utf-8")
    print("  [OK] net_cone_display")
    return True


def build_world(hazir: list[tuple[str, str, str]]) -> None:
    # Adaylari X ekseninde sirala, ortalanmis
    n = len(hazir)
    x0 = -(n - 1) * ARALIK / 2

    parcalar = []
    for i, (name, etiket, renk) in enumerate(hazir):
        x = x0 + i * ARALIK
        if name == "avci_net_interceptor":
            # Ag, namlunun ucuna: model cercevesinde namlu ucu x=0.27,
            # ag (koni TEPESI) 0.01 m onune -> x=0.28, z=+0.045
            parcalar.append(f"""
    <!-- Ag, taretin namlusunda -->
    <include>
      <static>true</static>
      <uri>model://net_cone_display</uri>
      <name>net_cone_display</name>
      <pose>{x + 0.28:.2f} 0 {YUKSEKLIK + 0.045:.3f} 0 0 0</pose>
    </include>""")
        parcalar.append(f"""
    <!-- {etiket} -->
    <include>
      <uri>model://{name}_display</uri>
      <name>{name}_display</name>
      <pose>{x:.2f} 0 {YUKSEKLIK} 0 0 0</pose>
    </include>

    <!-- {name} kaidesi ve renk kodu -->
    <model name="{name}_pedestal">
      <static>true</static>
      <pose>{x:.2f} 0 0 0 0 0</pose>
      <link name="link">
        <visual name="post">
          <pose>0 0 {YUKSEKLIK/2:.2f} 0 0 0</pose>
          <geometry><cylinder><radius>0.03</radius><length>{YUKSEKLIK:.2f}</length></cylinder></geometry>
          <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material>
        </visual>
        <visual name="base">
          <pose>0 0 0.02 0 0 0</pose>
          <geometry><cylinder><radius>0.45</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>{renk}</ambient><diffuse>{renk}</diffuse></material>
        </visual>
      </link>
    </model>""")

    world = f"""<?xml version="1.0" ?>
<!--
  VITRIN DUNYASI - scripts/50_showcase.py tarafindan uretildi, elle duzenlemeyin.

  Tum interceptor adaylari yan yana, 1.2 m yukseklikte asili.
  Soldan saga (kaide rengi):
{chr(10).join(f"    {i+1}. {e}" for i, (_, e, _) in enumerate(hazir))}

  Kullanim:
    source scripts/env.sh
    gz sim -r worlds/showcase.sdf          # canli GUI
    python3 scripts/51_capture_shots.py    # ekran goruntusu
-->
<sdf version="1.9">
  <world name="showcase">
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

    <gravity>0 0 -9.8</gravity>

    <scene>
      <ambient>0.75 0.75 0.78 1</ambient>
      <background>0.55 0.68 0.85 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.95 0.95 0.92 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.45 0.45 0.5 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.6 -0.5 -0.6</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.42 0.44 0.46 1</ambient>
            <diffuse>0.42 0.44 0.46 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
{"".join(parcalar)}

    <!-- Vitrin kamerasi: 51_capture_shots.py bundan goruntu alir -->
    <model name="showcase_camera">
      <static>true</static>
      <pose>0 -11.5 1.75 0 0.035 1.5708</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>/showcase/wide</topic>
          <camera>
            <horizontal_fov>1.25</horizontal_fov>
            <image><width>1800</width><height>620</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>200</far></clip>
          </camera>
        </sensor>
      </link>
    </model>

    <!-- Yakin plan kamera: taretli interceptor -->
    <model name="closeup_camera">
      <static>true</static>
      <!-- Taretli interceptor en solda: x = -(n-1)*ARALIK/2 = -6.65 -->
      <pose>-6.45 -2.05 1.30 0 -0.01 1.5708</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>/showcase/closeup</topic>
          <camera>
            <horizontal_fov>0.80</horizontal_fov>
            <image><width>1280</width><height>1000</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>100</far></clip>
          </camera>
        </sensor>
      </link>
    </model>

  </world>
</sdf>
"""
    WORLD.write_text(world, encoding="utf-8")


def main() -> int:
    SHOWCASE.mkdir(parents=True, exist_ok=True)
    print("Vitrin kopyalari uretiliyor (eklentiler cikariliyor, statik yapiliyor):")
    hazir = [a for a in ADAYLAR if make_display_copy(a[0])]
    make_net_copy()
    build_world(hazir)
    print(f"\nDunya: {WORLD.relative_to(ROOT)}  ({len(hazir)} aday)")
    print("Goruntulemek icin:")
    print("  source scripts/env.sh && gz sim -r worlds/showcase.sdf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
