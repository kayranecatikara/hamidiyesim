#!/usr/bin/env python3
"""Secilen govdeden (cand_iris) taretli ag atici interceptor'i uretir.

Neden script? cand_iris/model.sdf 883 satir ve kaynagi avci_sim'de gelisiyor.
Taret blogunu elle yapistirmak yerine enjekte ediyoruz ki govde guncellenince
tek komutla yeniden uretilebilsin.

Uretilen: models/interceptors/avci_net_interceptor/{model.sdf,model.config}

Taret mimarisi (docs/SECIM_KARARI.md kutle butcesi):
    iris_with_standoffs::base_link
      +-- turret_mount_joint (fixed)   -> turret_base_link   0.10 kg
            +-- turret_yaw_joint (Z)   -> turret_yaw_link    0.08 kg
                  +-- turret_pitch_joint (Y) -> turret_pitch_link 0.10 kg
                        +-- muzzle_joint (fixed) -> muzzle_link   0.07 kg
                                                          toplam  0.35 kg
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "models" / "interceptors" / "cand_iris" / "model.sdf"
DEST_DIR = ROOT / "models" / "interceptors" / "avci_net_interceptor"

MODEL_NAME = "avci_net_interceptor"
# iris govdesine gore taret montaj noktasi.
# base_link modelin merkezinde; on rotorlar x=+0.13, y=+-0.22'de.
# x=0.16 govdenin onunde ama rotor dairelerinin arasinda kaliyor.
MOUNT_X, MOUNT_Y, MOUNT_Z = 0.16, 0.0, 0.0

# avci_sim ayni anda calisabilsin diye ayri FDM portu (avci_sim: 9002 / 9012)
FDM_PORT = 9022

TURRET_BLOCK = f"""
    <!-- ================================================================ -->
    <!-- TARET + AG FIRLATICI                                             -->
    <!-- scripts/20_build_interceptor.py tarafindan eklendi               -->
    <!-- ================================================================ -->

    <!-- Taret tabani: govdeye sabit -->
    <link name="turret_base_link">
      <pose>{MOUNT_X} {MOUNT_Y} {MOUNT_Z} 0 0 0</pose>
      <inertial>
        <mass>0.10</mass>
        <inertia>
          <ixx>4e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>4e-5</iyy><iyz>0</iyz>
          <izz>4e-5</izz>
        </inertia>
      </inertial>
      <collision name="turret_base_collision">
        <geometry><box><size>0.05 0.05 0.03</size></box></geometry>
      </collision>
      <visual name="turret_base_visual">
        <geometry><box><size>0.05 0.05 0.03</size></box></geometry>
        <material>
          <ambient>0.15 0.15 0.18 1</ambient>
          <diffuse>0.15 0.15 0.18 1</diffuse>
        </material>
      </visual>
    </link>

    <joint name="turret_mount_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>turret_base_link</child>
    </joint>

    <!-- 1. eksen: PAN (yaw, Z ekseni) -->
    <link name="turret_yaw_link">
      <pose>{MOUNT_X} {MOUNT_Y} {MOUNT_Z + 0.03} 0 0 0</pose>
      <inertial>
        <mass>0.08</mass>
        <inertia>
          <ixx>3e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>3e-5</iyy><iyz>0</iyz>
          <izz>3e-5</izz>
        </inertia>
      </inertial>
      <collision name="turret_yaw_collision">
        <geometry><cylinder><radius>0.025</radius><length>0.025</length></cylinder></geometry>
      </collision>
      <visual name="turret_yaw_visual">
        <geometry><cylinder><radius>0.025</radius><length>0.025</length></cylinder></geometry>
        <material>
          <ambient>0.25 0.25 0.28 1</ambient>
          <diffuse>0.25 0.25 0.28 1</diffuse>
        </material>
      </visual>
    </link>

    <joint name="turret_yaw_joint" type="revolute">
      <parent>turret_base_link</parent>
      <child>turret_yaw_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <!-- +-100 derece: govde donmeden genis tarama -->
          <lower>-1.745</lower>
          <upper>1.745</upper>
          <effort>25.0</effort>
          <velocity>6.0</velocity>
        </limit>
        <dynamics><damping>0.02</damping></dynamics>
      </axis>
    </joint>

    <!-- 2. eksen: TILT (pitch, Y ekseni) -->
    <link name="turret_pitch_link">
      <pose>{MOUNT_X} {MOUNT_Y} {MOUNT_Z + 0.045} 0 0 0</pose>
      <inertial>
        <mass>0.10</mass>
        <inertia>
          <ixx>4e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>6e-5</iyy><iyz>0</iyz>
          <izz>6e-5</izz>
        </inertia>
      </inertial>
      <collision name="turret_pitch_collision">
        <geometry><box><size>0.04 0.05 0.03</size></box></geometry>
      </collision>
      <visual name="turret_pitch_visual">
        <geometry><box><size>0.04 0.05 0.03</size></box></geometry>
        <material>
          <ambient>0.35 0.35 0.38 1</ambient>
          <diffuse>0.35 0.35 0.38 1</diffuse>
        </material>
      </visual>
    </link>

    <joint name="turret_pitch_joint" type="revolute">
      <parent>turret_yaw_link</parent>
      <child>turret_pitch_link</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <!-- -60 (yukari) .. +30 (asagi) derece; SDF'te +pitch burnu asagi indirir -->
          <lower>-1.047</lower>
          <upper>0.524</upper>
          <effort>25.0</effort>
          <velocity>6.0</velocity>
        </limit>
        <dynamics><damping>0.02</damping></dynamics>
      </axis>
    </joint>

    <!-- Namlu: agin cikis noktasi. Ekseni +X. -->
    <link name="muzzle_link">
      <pose degrees="true">{MOUNT_X + 0.06} {MOUNT_Y} {MOUNT_Z + 0.045} 0 90 0</pose>
      <inertial>
        <mass>0.07</mass>
        <inertia>
          <ixx>2e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>2e-5</iyy><iyz>0</iyz>
          <izz>1e-5</izz>
        </inertia>
      </inertial>
      <collision name="muzzle_collision">
        <geometry><cylinder><radius>0.028</radius><length>0.10</length></cylinder></geometry>
      </collision>
      <visual name="muzzle_visual">
        <geometry><cylinder><radius>0.028</radius><length>0.10</length></cylinder></geometry>
        <material>
          <ambient>0.7 0.25 0.05 1</ambient>
          <diffuse>0.7 0.25 0.05 1</diffuse>
        </material>
      </visual>
    </link>

    <joint name="muzzle_joint" type="fixed">
      <parent>turret_pitch_link</parent>
      <child>muzzle_link</child>
    </joint>

    <!-- ================ Taret kontrolu ================ -->
    <!-- Aci komutu: gz topic -t /model/{MODEL_NAME}/joint/turret_yaw_joint/0/cmd_pos
                       -m gz.msgs.Double -p 'data: 0.5'  -->
    <plugin filename="gz-sim-joint-position-controller-system"
      name="gz::sim::systems::JointPositionController">
      <joint_name>turret_yaw_joint</joint_name>
      <p_gain>8.0</p_gain>
      <i_gain>1.0</i_gain>
      <d_gain>0.4</d_gain>
      <i_max>2.0</i_max>
      <i_min>-2.0</i_min>
      <cmd_max>20.0</cmd_max>
      <cmd_min>-20.0</cmd_min>
    </plugin>

    <plugin filename="gz-sim-joint-position-controller-system"
      name="gz::sim::systems::JointPositionController">
      <joint_name>turret_pitch_joint</joint_name>
      <!--
        Tilt ekseni yerçekimine karsi calisiyor: namlu + AGIN kutlesi
        (net_cone 0.15 kg, tilt ekleminden ~0.27 m ileride) yaklasik
        0.40 N.m devirici moment uretiyor. P=2 ile kalici ~9 derece hata
        olculdu; kazanclar bu yuke gore buyutuldu ve I terimi eklendi.
      -->
      <p_gain>30.0</p_gain>
      <i_gain>25.0</i_gain>
      <d_gain>1.5</d_gain>
      <i_max>15.0</i_max>
      <i_min>-15.0</i_min>
      <cmd_max>20.0</cmd_max>
      <cmd_min>-20.0</cmd_min>
    </plugin>

    <!-- ================ Ag tutucu / firlatici ================ -->
    <!--
      Kendi eklentimiz: agi namluya kilitler ve TEK komutla atar.
      Once hazir sistemler denendi (gz-sim-detachable-joint-system +
      gz-sim-apply-link-wrench-system) ama ayirma ve itki AYRI topic'lerden
      geldigi icin aralarindaki gecikme kontrol edilemiyordu: ayni
      parametrelerle menzil kosumlar arasi 2 m ile 108 m arasinda oynadi.
      NetLauncherPlugin ikisini ayni fizik adiminda yapar ve impuls yerine
      dogrudan cikis hizi verir (SetLinearVelocity) - sonuc tekrarlanabilir.

      Ates:  gz topic -t /{MODEL_NAME}/net/fire -m gz.msgs.Double -p 'data: 20'
    -->
    <plugin filename="NetLauncherPlugin" name="avci::NetLauncherPlugin">
      <muzzle_link>muzzle_link</muzzle_link>
      <net_model>net_cone</net_model>
      <net_link>net_link</net_link>
      <fire_topic>/{MODEL_NAME}/net/fire</fire_topic>
      <muzzle_speed>20.0</muzzle_speed>
      <!-- muzzle_link pitch=90 ile duruyor; kendi +Z'si govde +X'ine bakar -->
      <launch_axis>0 0 1</launch_axis>
    </plugin>
"""


def build() -> int:
    if not SRC.exists():
        print(f"HATA: {SRC} yok. Once ./10_stage_candidates.py calistirin.")
        return 1

    sdf = SRC.read_text(encoding="utf-8")
    sdf = sdf.replace('<model name="cand_iris">', f'<model name="{MODEL_NAME}">', 1)

    # avci_sim ile port cakismasini onle
    sdf, n = re.subn(r"<fdm_port_in>\d+</fdm_port_in>",
                     f"<fdm_port_in>{FDM_PORT}</fdm_port_in>", sdf)
    if n != 1:
        print(f"UYARI: fdm_port_in {n} kez degistirildi (1 bekleniyordu)")

    # Taret blogunu modelin sonuna, </model>'den hemen once enjekte et
    marker = "\n  </model>"
    if marker not in sdf:
        print("HATA: </model> kapanisi bulunamadi, enjeksiyon yapilamadi.")
        return 1
    sdf = sdf.replace(marker, TURRET_BLOCK + marker, 1)

    DEST_DIR.mkdir(parents=True, exist_ok=True)
    (DEST_DIR / "model.sdf").write_text(sdf, encoding="utf-8")
    (DEST_DIR / "model.config").write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Ag atan taretli interceptor. Govde: cand_iris (avci_sim iris_cam).
    2 eksenli taret + namlu + net_cone tutucu. FDM portu {FDM_PORT}.
    Uretici: scripts/20_build_interceptor.py
  </description>
</model>
""", encoding="utf-8")

    print(f"Uretildi: {(DEST_DIR / 'model.sdf').relative_to(ROOT)}  "
          f"({len(sdf.splitlines())} satir, FDM portu {FDM_PORT})")
    return 0


if __name__ == "__main__":
    raise SystemExit(build())
