#!/usr/bin/env python3
"""Aday govdelerin SDF'lerinden kutle/geometri cikarir, bench sonuclariyla
birlestirip docs/KIYAS_RAPORU.md uretir.

Olculen buyuklukler:
  - toplam kutle (include edilen alt modeller dahil)
  - govde sinir kutusu (collision geometrilerinden)
  - burun acikligi: +X yonunde en ileri collision noktasi
    (taret buraya monte edilecek, ne kadar ileri gitmemiz gerektigi)
  - mesh collision sayisi (olculemeyen geometri -> guven notu)
"""
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "models" / "interceptors"
BENCH = ROOT / "docs" / "bench_raw"
REPORT = ROOT / "docs" / "KIYAS_RAPORU.md"

# model:// cozumlemesi icin aranacak dizinler (12_bench.sh ile ayni sira)
SEARCH_PATHS = [
    MODELS,
    ROOT / "repos" / "mrs_uav_gazebo_simulator" / "models",
    ROOT / "repos" / "d2dtracker_sim" / "models",
    ROOT / "repos" / "iq_sim" / "models",
]


def find_model(uri: str) -> Path | None:
    name = uri.replace("model://", "").split("/")[0]
    for base in SEARCH_PATHS:
        cand = base / name / "model.sdf"
        if cand.exists():
            return cand
    return None


def parse_pose(elem: ET.Element | None) -> tuple[float, float, float]:
    if elem is None or not (elem.text or "").strip():
        return (0.0, 0.0, 0.0)
    parts = elem.text.split()
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except (ValueError, IndexError):
        return (0.0, 0.0, 0.0)


def geom_halfextent(geom: ET.Element) -> tuple[float, float, float] | None:
    """Collision geometrisinin yari-boyutlari. Mesh ise None (olculemez)."""
    if (box := geom.find("box/size")) is not None and box.text:
        d = [float(v) for v in box.text.split()]
        return (d[0] / 2, d[1] / 2, d[2] / 2)
    if (cyl := geom.find("cylinder")) is not None:
        r = float(cyl.findtext("radius", "0"))
        ln = float(cyl.findtext("length", "0"))
        return (r, r, ln / 2)
    if (sph := geom.find("sphere/radius")) is not None and sph.text:
        r = float(sph.text)
        return (r, r, r)
    return None


def walk_model(sdf_path: Path, offset=(0.0, 0.0, 0.0), depth=0, seen=None):
    """SDF'i (include'lari izleyerek) gezer; kutle ve collision kutularini toplar."""
    seen = seen or set()
    if depth > 4 or sdf_path in seen:
        return 0.0, [], 0
    seen.add(sdf_path)

    try:
        root = ET.parse(sdf_path).getroot()
    except ET.ParseError:
        return 0.0, [], 0

    mass_total = 0.0
    boxes: list[tuple[float, float, float, float, float, float]] = []
    mesh_count = 0

    for model in root.iter("model"):
        for link in model.findall("link"):
            lx, ly, lz = parse_pose(link.find("pose"))
            lx, ly, lz = lx + offset[0], ly + offset[1], lz + offset[2]

            if (m := link.findtext("inertial/mass")) is not None:
                try:
                    mass_total += float(m)
                except ValueError:
                    pass

            for col in link.findall("collision"):
                cx, cy, cz = parse_pose(col.find("pose"))
                geom = col.find("geometry")
                if geom is None:
                    continue
                he = geom_halfextent(geom)
                if he is None:
                    mesh_count += 1
                    continue
                ox, oy, oz = lx + cx, ly + cy, lz + cz
                boxes.append((ox - he[0], oy - he[1], oz - he[2],
                              ox + he[0], oy + he[1], oz + he[2]))

        for inc in model.findall("include"):
            uri = inc.findtext("uri", "")
            ipose = parse_pose(inc.find("pose"))
            child = find_model(uri)
            if child is None:
                continue
            cm, cb, cmesh = walk_model(
                child,
                (offset[0] + ipose[0], offset[1] + ipose[1], offset[2] + ipose[2]),
                depth + 1, seen)
            mass_total += cm
            boxes.extend(cb)
            mesh_count += cmesh

    return mass_total, boxes, mesh_count


def analyse(cand_dir: Path) -> dict:
    sdf = cand_dir / "model.sdf"
    mass, boxes, meshes = walk_model(sdf)

    if boxes:
        xmin = min(b[0] for b in boxes); xmax = max(b[3] for b in boxes)
        ymin = min(b[1] for b in boxes); ymax = max(b[4] for b in boxes)
        zmin = min(b[2] for b in boxes); zmax = max(b[5] for b in boxes)
        bbox = (xmax - xmin, ymax - ymin, zmax - zmin)
    else:
        xmax = ymax = zmax = 0.0
        bbox = (0.0, 0.0, 0.0)

    txt = sdf.read_text(encoding="utf-8")
    # Kamera/ArduPilot include edilen alt modellerde olabilir (iris_cam'in kamerasi
    # iris_with_standoffs icinde) - bagimliliklarin metnini de tarayalim.
    all_txt = txt
    for uri in re.findall(r"<uri>(model://[^<]+)</uri>", txt):
        dep = find_model(uri)
        if dep is not None:
            all_txt += dep.read_text(encoding="utf-8")

    return {
        "kutle_kg": round(mass, 3),
        "bbox_m": [round(v, 3) for v in bbox],
        "burun_x_m": round(xmax, 3),
        "olculemeyen_mesh_collision": meshes,
        "ardupilot_bagli": "ArduPilotPlugin" in all_txt,
        "kamera_var": 'type="camera"' in all_txt,
        "satir": len(txt.splitlines()),
    }


def main() -> int:
    rows = []
    for cand_dir in sorted(MODELS.glob("cand_*")):
        name = cand_dir.name
        geo = analyse(cand_dir)
        bench_file = BENCH / f"{name}.json"
        bench = json.loads(bench_file.read_text()) if bench_file.exists() else {}
        rows.append({"aday": name, **geo, **bench})

    REPORT.parent.mkdir(parents=True, exist_ok=True)
    with REPORT.open("w", encoding="utf-8") as f:
        f.write(HEADER)
        f.write("| Aday | Durum | RTF | Kütle (kg) | Sınır kutusu XYZ (m) | Burun +X (m) | ArduPilot | Kamera | Hata |\n")
        f.write("|---|---|---|---|---|---|---|---|---|\n")
        for r in rows:
            bbox = "×".join(str(v) for v in r["bbox_m"])
            rtf = r.get("rtf")
            rtf_s = "—" if rtf in (None, "null") else f"{rtf}"
            f.write(
                f"| `{r['aday']}` | {r.get('durum','?')} | {rtf_s} | {r['kutle_kg']} | {bbox} | "
                f"{r['burun_x_m']} | {'✅' if r['ardupilot_bagli'] else '❌'} | "
                f"{'✅' if r['kamera_var'] else '❌'} | {r.get('hata_sayisi','?')} |\n")

        f.write("\n### Adaya özel notlar\n\n")
        for r in rows:
            f.write(f"**`{r['aday']}`**\n")
            if r.get("eksik_modeller", "").strip():
                f.write(f"- Eksik bağımlılık: `{r['eksik_modeller'].strip()}`\n")
            if r.get("eklenti_hatalari", "").strip():
                f.write(f"- Eklenti hatası: `{r['eklenti_hatalari'].strip()[:200]}`\n")
            if r["olculemeyen_mesh_collision"]:
                f.write(f"- {r['olculemeyen_mesh_collision']} adet mesh collision ölçülemedi "
                        f"→ sınır kutusu/burun değeri **alt sınır**, gerçeği daha büyük\n")
            f.write(f"- SDF {r['satir']} satır\n\n")

        f.write(FOOTER)

    print(f"Rapor yazildi: {REPORT}")
    for r in rows:
        print(f"  {r['aday']:20s} {r.get('durum','?'):12s} kutle={r['kutle_kg']:6.3f}kg "
              f"burun_x={r['burun_x_m']:6.3f}m")
    return 0


HEADER = """# Aday Interceptor Gövdeleri — Kıyas Raporu

`scripts/12_bench.sh` (Gazebo Harmonic headless, 3000 iterasyon) +
`scripts/13_report.py` (SDF geometri analizi) tarafından üretildi.

**Ölçüm notları:**
- **RTF**: gerçek zaman faktörü. Dünya yüklenmediyse `—`.
  ⚠️ Adaylar arası doğrudan kıyaslanamaz — `cand_iris` ve `cand_iq_camera`
  çalışan ArduPilotPlugin + kamera render'ı taşıyor, MRS adaylarının motor
  eklentisi yüklenemediği için fizik yükü daha hafif.
- **Burun +X**: gövdenin +X (ileri) yönündeki en uzak collision noktası.
  Taret bunun ötesine monte edilecek. Mesh collision'lar ölçülemez → alt sınır.
- **Kütle**: `include` edilen alt modeller dahil, tüm linklerin `<inertial><mass>` toplamı.

---

"""

FOOTER = """---

## Sonuç

Karar `docs/SECIM_KARARI.md` dosyasında.
"""


if __name__ == "__main__":
    raise SystemExit(main())
