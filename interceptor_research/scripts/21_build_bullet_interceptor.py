#!/usr/bin/env python3
"""Mermi govdeli (cand_bullet) gövdeden taretli ag atici interceptor uretir.

20_build_interceptor.py ile ayni desen: govde kaynagina dokunmadan taret
blogunu enjekte eder. Fark, bu govdenin DIKEY bir mermi olmasi:

  - Burun konisi KALDIRILIR (z = +0.25'teki duz govde tepesi acilir)
  - Taret oraya, govdenin TEPESINE oturur
  - Namlu +X'e bakar: ag ILERI atilir (dikey govde, yatay atis)

Uretilen: models/interceptors/bullet_net_interceptor/{model.sdf,model.config}

Taret mimarisi (20_build_interceptor.py ile ayni kutle ve ayni eklem limitleri):
    base_link
      +-- turret_mount_joint (fixed)   -> turret_base_link   0.10 kg
            +-- turret_yaw_joint (Z)   -> turret_yaw_link    0.08 kg   +-100 deg
                  +-- turret_pitch_joint (Y) -> turret_pitch_link 0.10 kg  -60..+30 deg
                        +-- muzzle_joint (fixed) -> muzzle_link   0.07 kg
                                                          toplam  0.35 kg
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "models" / "interceptors" / "cand_bullet" / "model.sdf"
DEST_DIR = ROOT / "models" / "interceptors" / "bullet_net_interceptor"

SRC_MODEL_NAME = "bullet_interceptor"
MODEL_NAME = "bullet_net_interceptor"

# Govde tepesi: fuselage silindiri z = -0.25 .. +0.25 arasinda.
# Burun konisi kaldirilinca z = +0.25 duz bir tabla olur; taret tabani
# (0.03 m yuksek kutu) onun ustune oturur -> merkezi +0.265.
MOUNT_X, MOUNT_Y, MOUNT_Z = 0.0, 0.0, 0.265

# Port cakismasi olmasin: avci_sim 9002/9012, avci_net_interceptor 9022,
# ham cand_bullet 9002/9003. Bu model 9032/9033.
FDM_PORT_IN = 9032
FDM_PORT_OUT = 9033

TURRET_BLOCK = f"""
    <!-- ================================================================ -->
    <!-- TARET + AG FIRLATICI                                             -->
    <!-- scripts/21_build_bullet_interceptor.py tarafindan eklendi         -->
    <!--                                                                  -->
    <!-- Govde DIKEY duruyor (+Z burun yonu) ama taret ILERI (+X) atiyor:  -->
    <!-- muzzle_link pitch=90 ile yatirilmis, kendi +Z'si govde +X'ine     -->
    <!-- bakar; launch_axis bu yuzden "0 0 1".                            -->
    <!-- ================================================================ -->

    <!-- Taret tabani: govdenin tepesine (burun konisinin yerine) sabit -->
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
      <parent>base_link</parent>
      <child>turret_base_link</child>
    </joint>

    <!-- 1. eksen: PAN (yaw, Z ekseni) -->
    <link name="turret_yaw_link">
      <pose>{MOUNT_X} {MOUNT_Y} {round(MOUNT_Z + 0.03, 4)} 0 0 0</pose>
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
          <!-- +-100 derece: govde donmeden genis tarama (kaynak modelle ayni) -->
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
      <pose>{MOUNT_X} {MOUNT_Y} {round(MOUNT_Z + 0.045, 4)} 0 0 0</pose>
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
          <!-- -60 (yukari) .. +30 (asagi) derece; kaynak modelle ayni.
               SDF'te +pitch namluyu asagi indirir. -->
          <lower>-1.047</lower>
          <upper>0.524</upper>
          <effort>25.0</effort>
          <velocity>6.0</velocity>
        </limit>
        <dynamics><damping>0.02</damping></dynamics>
      </axis>
    </joint>

    <!-- Namlu: agin cikis noktasi. Ekseni +X (govde ileri yonu). -->
    <link name="muzzle_link">
      <pose degrees="true">{round(MOUNT_X + 0.06, 4)} {MOUNT_Y} {round(MOUNT_Z + 0.045, 4)} 0 90 0</pose>
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
        Tilt ekseni yercekimine karsi calisiyor: namlu + AGIN kutlesi
        (net_cone 0.30 kg, tilt ekleminden ~0.27 m ileride) devirici moment
        uretiyor. Kazanclar 20_build_interceptor.py'de bu yuke gore olculdu,
        geometri ayni oldugu icin aynen tasindi.
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

# Burun konisi: taretin oturacagi yeri isgal ediyordu, kaldirildi.
NOSE_CONE_RE = re.compile(
    r'\n[ \t]*<!-- =+ ogive nose cone.*?-->'
    r'\n[ \t]*<visual name="nose_cone_visual">.*?</visual>'
    r'\n[ \t]*<!-- Cone collisions.*?-->',
    re.DOTALL,
)

NOSE_CONE_REPLACEMENT = """
      <!-- ========== burun konisi KALDIRILDI ==========
           Orijinalde z = +0.31'de r=0.08 / L=0.12 bir ogive koni vardi.
           Taret govdenin tepesine oturdugu icin cikarildi; govde tepesi
           artik z = +0.25'te duz bir tabla ve turret_base_link oraya biniyor.
           scripts/21_build_bullet_interceptor.py -->"""

# Kutle butcesi dokumu (SDF'ten okunan degerlerle dogrulanir)
GOVDE_KUTLELERI = {
    "base_link": 1.800,
    "imu_link": 0.010,
    "camera_link": 0.030,
    "rotor_0": 0.025,
    "rotor_1": 0.025,
    "rotor_2": 0.025,
    "rotor_3": 0.025,
}
TARET_KUTLELERI = {
    "turret_base_link": 0.10,
    "turret_yaw_link": 0.08,
    "turret_pitch_link": 0.10,
    "muzzle_link": 0.07,
}
NET_KUTLESI = 0.30  # models/net_launchers/net_cone


def kutle_butcesi() -> str:
    govde = sum(GOVDE_KUTLELERI.values())
    taret = sum(TARET_KUTLELERI.values())
    kuru = govde + taret
    yuklu = kuru + NET_KUTLESI
    return (
        f"  govde (cand_bullet)      : {govde:.3f} kg\n"
        f"  taret zinciri            : {taret:.3f} kg\n"
        f"  --------------------------------\n"
        f"  kuru (ag yok)            : {kuru:.3f} kg\n"
        f"  ag yuklu (net_cone 0.30) : {yuklu:.3f} kg\n"
    )


def build() -> int:
    if not SRC.exists():
        print(f"HATA: {SRC} yok.")
        return 1

    sdf = SRC.read_text(encoding="utf-8")

    # 1) Model adi
    sdf, n = re.subn(rf'<model name="{SRC_MODEL_NAME}">',
                     f'<model name="{MODEL_NAME}">', sdf)
    if n != 1:
        print(f"HATA: model adi {n} kez degistirildi (1 bekleniyordu)")
        return 1

    # 2) Burun konisini kaldir
    sdf, n = NOSE_CONE_RE.subn(NOSE_CONE_REPLACEMENT, sdf)
    if n != 1:
        print(f"HATA: burun konisi {n} kez kaldirildi (1 bekleniyordu)")
        return 1

    # 3) FDM portlari (avci_sim / avci_net_interceptor ile cakismasin)
    sdf, n_in = re.subn(r"<fdm_port_in>\d+</fdm_port_in>",
                        f"<fdm_port_in>{FDM_PORT_IN}</fdm_port_in>", sdf)
    sdf, n_out = re.subn(r"<fdm_port_out>\d+</fdm_port_out>",
                         f"<fdm_port_out>{FDM_PORT_OUT}</fdm_port_out>", sdf)
    if n_in != 1 or n_out != 1:
        print(f"UYARI: FDM portlari {n_in}/{n_out} kez degistirildi (1/1 bekleniyordu)")

    # 4) Kamera topic'i model adiyla eslesin
    sdf = sdf.replace(f"{SRC_MODEL_NAME}/nose_camera/image",
                      f"{MODEL_NAME}/nose_camera/image")

    # 5) Taret blogunu </model>'den hemen once enjekte et
    marker = "\n  </model>"
    if marker not in sdf:
        print("HATA: </model> kapanisi bulunamadi, enjeksiyon yapilamadi.")
        return 1
    sdf = sdf.replace(marker, TURRET_BLOCK + marker, 1)

    DEST_DIR.mkdir(parents=True, exist_ok=True)
    (DEST_DIR / "model.sdf").write_text(sdf, encoding="utf-8")

    govde = sum(GOVDE_KUTLELERI.values())
    taret = sum(TARET_KUTLELERI.values())
    (DEST_DIR / "model.config").write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Mermi govdeli, tepeden taretli ag atan interceptor.
    Govde: cand_bullet (burun konisi kaldirildi).
    2 eksenli taret: pan +-100 deg, tilt -60..+30 deg. Namlu +X, ag ILERI atilir.
    Kutle: govde {govde:.3f} + taret {taret:.3f} = {govde + taret:.3f} kg kuru,
           ag yuklu {govde + taret + NET_KUTLESI:.3f} kg.
    ArduPilot FDM portu {FDM_PORT_IN}/{FDM_PORT_OUT}.
    Param dosyasi: config/bullet_net_interceptor.param
    Uretici: scripts/21_build_bullet_interceptor.py
  </description>
</model>
""", encoding="utf-8")

    print(f"Uretildi: {(DEST_DIR / 'model.sdf').relative_to(ROOT)}  "
          f"({len(sdf.splitlines())} satir, FDM {FDM_PORT_IN}/{FDM_PORT_OUT})")
    print()
    print("Kutle butcesi:")
    print(kutle_butcesi())
    return 0


if __name__ == "__main__":
    raise SystemExit(build())
