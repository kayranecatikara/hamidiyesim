#!/usr/bin/env python3
"""AVCI SİM dokümantasyon sitesini tek dosyalık HTML olarak üretir."""
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from md2html import convert, _slug  # noqa: E402

# Yolları bu dosyanın konumundan türet (dokumantasyon/build/ -> dokumantasyon/)
_HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(_HERE)
OUT = os.path.join(SRC, "site", "index.html")

# ── Gezinti ağacı: proje yapısını yansıtır, doc bölümlerine bağlanır ────────
# (etiket, dosya, h1_metni|None, katman)   h1_metni None ise dokümanın tamamı
TREE = [
    ("group", "GENEL BAKIŞ", None, [
        ("doc", "Proje Yapısı", "01_PROJE_YAPISI.md", None, "genel"),
        ("doc", "Mimari & Veri Akışı", "02_MIMARI_VE_VERI_AKISI.md", None, "genel"),
    ]),
    ("group", "control/", "Uçuş kontrolü ve güdüm", [
        ("doc", "mav_common.py", "10_KOD_control_cekirdek.md", "mav_common.py", "kontrol"),
        ("doc", "drone_functions.py", "10_KOD_control_cekirdek.md", "drone_functions.py", "kontrol"),
        ("doc", "plane_functions.py", "10_KOD_control_cekirdek.md", "plane_functions.py", "kontrol"),
        ("doc", "plane_patterns.py", "10_KOD_control_cekirdek.md", "plane_patterns.py", "kontrol"),
        ("doc", "run_plane_scenario.py", "11_KOD_control_senaryo_demo.md", "run_plane_scenario.py", "kontrol"),
        ("doc", "arm_diag.py", "11_KOD_control_senaryo_demo.md", "arm_diag.py", "kontrol"),
        ("doc", "demos/", "11_KOD_control_senaryo_demo.md", "control/demos/", "kontrol"),
    ]),
    ("group", "control/guidance/", "★ Hibrit güdüm — projenin kalbi", [
        ("doc", "common.py", "12_KOD_guidance.md", "common.py", "gudum"),
        ("doc", "guidance_core.py", "12_KOD_guidance.md", "guidance_core.py", "gudum"),
        ("doc", "adapter_copter.py", "12_KOD_guidance.md", "adapter_copter.py", "gudum"),
        ("doc", "visual_lead.py", "12_KOD_guidance.md", "visual_lead.py", "gudum"),
        ("doc", "gps_guidance.py", "12_KOD_guidance.md", "gps_guidance.py", "gudum"),
        ("doc", "supervisor.py", "12_KOD_guidance.md", "supervisor.py", "gudum"),
    ]),
    ("group", "Yer Kontrol İstasyonu", None, [
        ("doc", "gcs_server.py", "13_KOD_gcs_server.md", None, "arayuz"),
        ("doc", "gcs_ui/", "14_KOD_gcs_ui.md", None, "arayuz"),
    ]),
    ("group", "vision/", "Görüntü işleme ve eğitim", [
        ("doc", "geometry.py", "15_KOD_vision.md", "geometry.py", "goru"),
        ("doc", "detector.py", "15_KOD_vision.md", "detector.py", "goru"),
        ("doc", "pose_detector.py", "15_KOD_vision.md", "pose_detector.py", "goru"),
        ("doc", "detection_state.py", "15_KOD_vision.md", "detection_state.py", "goru"),
        ("doc", "Veri toplama", "15_KOD_vision.md", "Veri toplama betikleri", "goru"),
        ("doc", "Eğitim", "15_KOD_vision.md", "Eğitim betikleri", "goru"),
    ]),
    ("group", "DESTEK", None, [
        ("doc", "sim/ — Simülasyon varlıkları", "16_KOD_sim_varliklari.md", None, "sim"),
        ("doc", "scripts/ tests/ tools/", "17_KOD_scripts_tools_tests.md", None, "arac"),
        ("doc", "Temizlik Kaydı", "90_TEMIZLIK_KAYDI.md", None, "kayit"),
    ]),
]

KATMAN = {
    "genel": ("GENEL", "n"), "kontrol": ("KONTROL", "a"), "gudum": ("GÜDÜM", "b"),
    "arayuz": ("ARAYÜZ", "c"), "goru": ("GÖRÜ", "d"), "sim": ("SİMÜLASYON", "e"),
    "arac": ("ARAÇ", "f"), "kayit": ("KAYIT", "n"),
}

LINK_MAP = {}   # md dosya adı → panel id (dosya bazlı bağlantılar için)


def split_by_h1(md):
    """Markdown'ı h1 başlıklarına göre böler → [(baslik|None, govde)]."""
    lines = md.split("\n")
    parts, cur_title, buf, in_code = [], None, [], False
    for ln in lines:
        if ln.startswith("```"):
            in_code = not in_code
        if not in_code:
            m = re.match(r"^#\s+(.*)$", ln)
            if m:
                if buf or cur_title is not None:
                    parts.append((cur_title, "\n".join(buf)))
                cur_title = m.group(1).strip()
                buf = []
                continue
        buf.append(ln)
    parts.append((cur_title, "\n".join(buf)))
    return parts


def clean(t):
    return re.sub(r"`", "", t or "").strip()


def main():
    raw = {}
    for fn in sorted(os.listdir(SRC)):
        if fn.endswith(".md"):
            with open(os.path.join(SRC, fn), encoding="utf-8") as f:
                raw[fn] = f.read()

    # panel id ataması
    panels, order = {}, []
    for kind, label, sub, items in TREE:
        for entry in items:
            _, lbl, fn, h1, katman = entry
            pid = _slug(f"{fn}-{h1 or 'all'}", set(panels.keys()))
            panels[pid] = {"label": lbl, "file": fn, "h1": h1, "katman": katman,
                           "group": label, "note": sub}
            order.append(pid)
            if h1 is None:
                LINK_MAP.setdefault(fn, pid)

    seen_ids = set()
    out_html, index = [], []

    for pid in order:
        p = panels[pid]
        md = raw[p["file"]]
        parts = split_by_h1(md)

        if p["h1"] is None:
            # doküman başlığı (ilk h1) + tüm gövde
            doc_title = clean(parts[0][0]) if parts[0][0] else p["label"]
            body_md = "\n\n".join(
                (f"# {t}\n{b}" if t and i else b) for i, (t, b) in enumerate(parts))
        else:
            doc_title, body_md = None, None
            for t, b in parts:
                # başlık "control/demos/ — Bağımsız uçuş demoları" gibi ek metin
                # taşıyabilir; önek eşleşmesi yeterli
                if t and clean(t).startswith(p["h1"]):
                    doc_title, body_md = clean(t), b
                    break
            if body_md is None:
                doc_title, body_md = p["label"], "_(bölüm bulunamadı)_"

        body, heads = convert(body_md, LINK_MAP, seen_ids, id_prefix=f"{pid}-")

        # sağ rayda gösterilecek başlıklar (h2 = fonksiyon / bölüm)
        subs = [h for h in heads if h["level"] == 2]
        katman_ad, _ = KATMAN[p["katman"]]

        # İlk panel statik HTML'de GÖRÜNÜR kalır: JS çalışmayan ortamlarda
        # (iOS Dosyalar/QuickLook önizlemesi gibi) sayfa boş kalmasın.
        gizli = "" if pid == order[0] else " hidden"
        out_html.append(
            f'<article class="panel" id="{pid}"{gizli}>'
            f'<header class="panel-hd">'
            f'<div class="eyebrow"><span class="layer layer-{p["katman"]}">{katman_ad}</span>'
            f'<span class="src">{p["file"]}</span></div>'
            f'<h1 class="panel-title">{doc_title}</h1>'
            f'</header>'
            f'<div class="prose">{body}</div>'
            f'</article>'
        )

        # arama indeksi: başlıklar + düz metin
        plain = re.sub(r"<[^>]+>", " ", body)
        plain = re.sub(r"\s+", " ", plain)
        index.append({
            "id": pid, "t": p["label"], "g": p["group"], "k": katman_ad,
            "h": [{"i": h["id"], "t": h["text"], "l": h["level"]} for h in heads
                  if h["level"] in (2, 3)],
            "s": [{"i": h["id"], "t": h["text"]} for h in subs],
            "x": plain[:14000].lower(),
        })

    # ── gezinti HTML ──
    nav = []
    for kind, label, sub, items in TREE:
        nav.append(f'<div class="nav-group"><div class="nav-group-hd">{label}'
                   + (f'<span class="nav-note">{sub}</span>' if sub else "")
                   + "</div><ul class=\"nav-list\">")
        for entry in items:
            _, lbl, fn, h1, katman = entry
            pid = next(k for k, v in panels.items()
                       if v["label"] == lbl and v["file"] == fn and v["h1"] == h1)
            nav.append(f'<li><button class="nav-item" data-nav="{pid}">'
                       f'<span class="nav-dot layer-{katman}"></span>{lbl}</button></li>')
        nav.append("</ul></div>")

    stats = {
        "panel": len(order),
        "satir": sum(len(v.split("\n")) for v in raw.values()),
        "dosya": len(raw),
    }

    tpl_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "shell.html")
    with open(tpl_path, encoding="utf-8") as f:
        shell = f.read()

    page = (shell
            .replace("__NAV__", "\n".join(nav))
            .replace("__PANELS__", "\n".join(out_html))
            .replace("__INDEX__", json.dumps(index, ensure_ascii=False, separators=(",", ":")))
            .replace("__FIRST__", order[0])
            .replace("__STAT_PANEL__", str(stats["panel"]))
            .replace("__STAT_SATIR__", f'{stats["satir"]:,}'.replace(",", "."))
            )

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(page)
    print(f"✓ {OUT}")
    print(f"  {stats['panel']} panel · {stats['satir']} kaynak satır · "
          f"{os.path.getsize(OUT)/1024:.0f} KB")


if __name__ == "__main__":
    main()
