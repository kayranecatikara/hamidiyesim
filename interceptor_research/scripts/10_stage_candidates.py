#!/usr/bin/env python3
"""Aday interceptor govdelerini models/interceptors/ altina hazirlar.

- cand_iris        : avci_sim'in Harmonic-native iris'i (donusum yok)
- cand_iq_camera   : iq_sim/drone_with_camera, Gazebo Classic -> Harmonic donusumu
- cand_d2d_x500    : d2dtracker'in x500_d435 sarmalayicisi (bagimlilik analizi)
- cand_mrs_*       : 11_render_jinja.py tarafindan uretilir (burada uretilmez)

Kullanim:  ./10_stage_candidates.py
"""
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REPOS = ROOT / "repos"
AVCI = Path("/home/kubra/projects/avci_sim/sim/gazebo_harmonic/models")
OUT = ROOT / "models" / "interceptors"

# Gazebo Classic -> Harmonic (gz-sim 8) eklenti adi karsiliklari.
# Kaynak: gz-sim migration rehberi + avci_sim/models/iris_cam/model.sdf (calisan ornek)
CLASSIC_TO_HARMONIC = {
    "libLiftDragPlugin.so": "gz-sim-lift-drag-system",
    "libArduPilotPlugin.so": "ArduPilotPlugin",
    "libgazebo_ros_camera.so": None,          # gz'de ayri plugin yok, sensor yeterli
    "libgazebo_ros_ray_sensor.so": None,
    "libgazebo_ros_imu_sensor.so": None,
}
PLUGIN_CLASS = {
    "gz-sim-lift-drag-system": "gz::sim::systems::LiftDrag",
    "ArduPilotPlugin": "ArduPilotPlugin",
}

# Harmonic, Classic'in <material><script>Gazebo/X</script></material> bicimini
# desteklemiyor ("A <script> element is missing a child <uri>" hatasi).
# Ogre materyal adlarini duz RGBA'ya ceviriyoruz.
GAZEBO_MATERIALS = {
    "Gazebo/Blue": "0.0 0.0 0.8 1",
    "Gazebo/Red": "0.8 0.0 0.0 1",
    "Gazebo/Green": "0.0 0.8 0.0 1",
    "Gazebo/Grey": "0.5 0.5 0.5 1",
    "Gazebo/DarkGrey": "0.2 0.2 0.2 1",
    "Gazebo/FlatBlack": "0.05 0.05 0.05 1",
    "Gazebo/Black": "0.0 0.0 0.0 1",
    "Gazebo/White": "1.0 1.0 1.0 1",
    "Gazebo/Orange": "1.0 0.5 0.0 1",
    "Gazebo/Yellow": "1.0 1.0 0.0 1",
}
DEFAULT_RGBA = "0.5 0.5 0.5 1"


def write_config(dest_dir: Path, name: str, desc: str) -> None:
    (dest_dir / "model.config").write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{desc}</description>
</model>
""",
        encoding="utf-8",
    )


def stage_iris() -> None:
    """avci_sim'in iris'i - zaten Harmonic + ArduPilot, oldugu gibi kopyalanir."""
    dest = OUT / "cand_iris"
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)

    sdf = (AVCI / "iris_cam" / "model.sdf").read_text(encoding="utf-8")
    # Model adini aday adiyla degistir (dunyada birden fazla aday yan yana durabilsin)
    sdf = sdf.replace('<model name="iris_with_ardupilot">', '<model name="cand_iris">', 1)
    (dest / "model.sdf").write_text(sdf, encoding="utf-8")
    write_config(dest, "cand_iris",
                 "avci_sim iris_cam govdesi. Harmonic-native, ArduPilotPlugin bagli, donusum gerekmedi.")

    # Bagimlilik: iris_with_standoffs (mesh + govde). Kendi kopyamizi alalim ki
    # avci_sim'e bagimli olmayalim.
    dep_src = AVCI / "iris_with_standoffs"
    dep_dst = OUT / "iris_with_standoffs"
    if dep_dst.exists():
        shutil.rmtree(dep_dst)
    shutil.copytree(dep_src, dep_dst)
    print(f"  [OK] cand_iris  (+ bagimlilik iris_with_standoffs, {len(list(dep_dst.rglob('*')))} dosya)")


def convert_classic_sdf(sdf: str) -> tuple[str, list[str]]:
    """Classic plugin adlarini Harmonic karsiliklariyla degistirir.

    None esleyen eklentiler (ros_camera vb.) tum <plugin>...</plugin> blogu
    ile birlikte silinir - sadece acilis etiketini degistirmek bozuk XML uretir.
    """
    notes: list[str] = []

    # 1) Once tamamen kaldirilacak eklenti bloklarini sil
    def drop(m: re.Match) -> str:
        fname = m.group("file")
        if CLASSIC_TO_HARMONIC.get(fname, "keep") is None:
            notes.append(f"blok kaldirildi (Harmonic'te gereksiz): {fname}")
            return f"<!-- Harmonic: {fname} blogu kaldirildi, <sensor> tanimi yeterli -->"
        return m.group(0)

    sdf = re.sub(
        r'<plugin\s+name="[^"]+"\s+filename="(?P<file>[^"]+)".*?</plugin>',
        drop, sdf, flags=re.DOTALL)

    # 2) Kalanlarin filename/name ozniteliklerini Harmonic karsiligiyla degistir
    def rename(m: re.Match) -> str:
        pname, fname = m.group("name"), m.group("file")
        new = CLASSIC_TO_HARMONIC.get(fname)
        if new is None:  # sozlukte yok -> dokunma
            if fname not in CLASSIC_TO_HARMONIC:
                notes.append(f"bilinmeyen Classic eklentisi birakildi: {fname}")
            return m.group(0)
        notes.append(f"{fname} -> {new}")
        return f'<plugin name="{PLUGIN_CLASS.get(new, pname)}" filename="{new}"'

    sdf = re.sub(
        r'<plugin\s+name="(?P<name>[^"]+)"\s+filename="(?P<file>[^"]+)"',
        rename, sdf)

    # 3) Classic materyal script'lerini Harmonic RGBA'ya cevir
    def material(m: re.Match) -> str:
        names = re.findall(r"<name>([^<]+)</name>", m.group(0))
        mat = names[-1].strip() if names else ""
        rgba = GAZEBO_MATERIALS.get(mat)
        if rgba is None:
            rgba = DEFAULT_RGBA
            notes.append(f"bilinmeyen materyal '{mat}' -> varsayilan gri")
        else:
            notes.append(f"materyal {mat} -> RGBA {rgba}")
        return f"<ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"

    sdf = re.sub(r"<script>.*?</script>", material, sdf, flags=re.DOTALL)
    return sdf, notes


def stage_iq_camera() -> None:
    """iq_sim/drone_with_camera - Classic'ten Harmonic'e cevrilir."""
    src_dir = REPOS / "iq_sim" / "models"
    dest = OUT / "cand_iq_camera"
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)

    sdf = (src_dir / "drone_with_camera" / "model.sdf").read_text(encoding="utf-8")
    sdf = sdf.replace('<model name="drone_with_camera">', '<model name="cand_iq_camera">', 1)
    sdf = sdf.replace('<sdf version="1.6" >', '<sdf version="1.9">', 1)
    sdf, notes = convert_classic_sdf(sdf)
    (dest / "model.sdf").write_text(sdf, encoding="utf-8")
    write_config(dest, "cand_iq_camera",
                 "iq_sim drone_with_camera. Gazebo Classic -> Harmonic eklenti donusumu uygulandi.")

    # Bagimlilik: iris_base
    dep_src = src_dir / "iris_base"
    dep_dst = OUT / "iris_base"
    if dep_dst.exists():
        shutil.rmtree(dep_dst)
    shutil.copytree(dep_src, dep_dst)
    base = (dep_dst / "model.sdf").read_text(encoding="utf-8")
    base, base_notes = convert_classic_sdf(base)
    (dep_dst / "model.sdf").write_text(base, encoding="utf-8")

    (dest / "DONUSUM_NOTLARI.txt").write_text(
        "drone_with_camera:\n  " + "\n  ".join(notes) +
        "\n\niris_base:\n  " + "\n  ".join(base_notes) + "\n",
        encoding="utf-8")
    print(f"  [OK] cand_iq_camera  ({len(notes)} eklenti donusumu, notlar DONUSUM_NOTLARI.txt'de)")


def stage_d2d_x500() -> None:
    """d2dtracker x500_d435 - PX4 x500 modeline bagimli, bagimlilik analizi yapilir."""
    dest = OUT / "cand_d2d_x500"
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)

    src = REPOS / "d2dtracker_sim" / "models" / "x500_d435" / "model.sdf"
    sdf = src.read_text(encoding="utf-8")
    (dest / "model.sdf").write_text(sdf, encoding="utf-8")
    write_config(dest, "cand_d2d_x500",
                 "d2dtracker x500_d435 sarmalayici. PX4 x500 modeline bagimli.")

    deps = re.findall(r"<uri>model://([^<]+)</uri>", sdf)
    local = {p.name for p in (REPOS / "d2dtracker_sim" / "models").iterdir()}
    missing = [d.split("/")[0] for d in deps if d.split("/")[0] not in local]
    (dest / "BAGIMLILIK.txt").write_text(
        f"include edilen modeller: {deps}\n"
        f"d2dtracker deposunda BULUNMAYAN: {missing}\n"
        f"-> bunlar PX4-Autopilot/Tools/simulation/gz'den gelir, depoda yok.\n",
        encoding="utf-8")
    print(f"  [OK] cand_d2d_x500  (eksik bagimlilik: {missing or 'yok'})")


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    print("Adaylar hazirlaniyor:")
    stage_iris()
    stage_iq_camera()
    stage_d2d_x500()
    print("\nMRS adaylari icin: ./11_render_jinja.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
