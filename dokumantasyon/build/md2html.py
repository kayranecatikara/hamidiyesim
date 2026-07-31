#!/usr/bin/env python3
"""Markdown → HTML dönüştürücü (bağımlılıksız, bu projenin kullandığı alt küme)."""
import html
import re

_INLINE_CODE = re.compile(r"`([^`]+)`")
_BOLD = re.compile(r"\*\*([^*]+)\*\*")
_LINK = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")


def _inline(text, link_map=None):
    """Satır içi markdown → HTML. Önce escape, sonra işaretleme."""
    out = html.escape(text, quote=False)
    # kod önce: içindeki * ve [ bold/link olarak yorumlanmasın
    holes = []

    def _stash(m):
        holes.append(f"<code>{m.group(1)}</code>")
        return f"\x00{len(holes)-1}\x00"

    out = _INLINE_CODE.sub(_stash, out)
    out = _BOLD.sub(r"<strong>\1</strong>", out)

    def _link(m):
        label, target = m.group(1), m.group(2)
        if link_map and target in link_map:
            return f'<a href="#{link_map[target]}" data-nav="{link_map[target]}">{label}</a>'
        if target.startswith("http"):
            return f'<a href="{target}" target="_blank" rel="noopener">{label}</a>'
        return f'<span class="ref">{label}</span>'

    out = _LINK.sub(_link, out)
    out = re.sub(r"\x00(\d+)\x00", lambda m: holes[int(m.group(1))], out)
    return out


def _slug(text, seen):
    s = re.sub(r"`", "", text)
    s = re.sub(r"[^\w\s.-]", "", s, flags=re.UNICODE).strip().lower()
    s = re.sub(r"\s+", "-", s)
    tr = str.maketrans("çğıöşü", "cgiosu")
    s = s.translate(tr)[:60] or "b"
    n, base = 1, s
    while s in seen:
        n += 1
        s = f"{base}-{n}"
    seen.add(s)
    return s


def convert(md, link_map=None, seen=None, id_prefix=""):
    """Markdown metnini (HTML, başlık_listesi) olarak döndürür."""
    seen = seen if seen is not None else set()
    lines = md.split("\n")
    out, heads = [], []
    i, n = 0, len(lines)

    def flush_para(buf):
        if buf:
            out.append(f"<p>{_inline(' '.join(buf), link_map)}</p>")
            buf.clear()

    para = []
    while i < n:
        ln = lines[i]

        # ---- fenced code ----
        if ln.startswith("```"):
            flush_para(para)
            lang = ln[3:].strip() or "text"
            i += 1
            body = []
            while i < n and not lines[i].startswith("```"):
                body.append(lines[i])
                i += 1
            i += 1
            code = html.escape("\n".join(body), quote=False)
            out.append(
                f'<div class="code-wrap"><div class="code-lang">{lang}</div>'
                f'<pre><code>{code}</code></pre></div>'
            )
            continue

        # ---- table ----
        if ln.startswith("|") and i + 1 < n and re.match(r"^\|[\s:|-]+\|?\s*$", lines[i + 1]):
            flush_para(para)
            hdr = [c.strip() for c in ln.strip().strip("|").split("|")]
            aligns = []
            for c in lines[i + 1].strip().strip("|").split("|"):
                c = c.strip()
                aligns.append("right" if c.endswith(":") and not c.startswith(":")
                              else "center" if c.startswith(":") and c.endswith(":")
                              else "left")
            i += 2
            rows = []
            while i < n and lines[i].startswith("|"):
                rows.append([c.strip() for c in lines[i].strip().strip("|").split("|")])
                i += 1
            th = "".join(
                f'<th style="text-align:{aligns[k] if k < len(aligns) else "left"}">'
                f"{_inline(c, link_map)}</th>" for k, c in enumerate(hdr))
            trs = []
            for r in rows:
                tds = "".join(
                    f'<td style="text-align:{aligns[k] if k < len(aligns) else "left"}">'
                    f"{_inline(c, link_map)}</td>" for k, c in enumerate(r))
                trs.append(f"<tr>{tds}</tr>")
            out.append(
                f'<div class="table-wrap"><table><thead><tr>{th}</tr></thead>'
                f'<tbody>{"".join(trs)}</tbody></table></div>'
            )
            continue

        # ---- heading ----
        m = re.match(r"^(#{1,6})\s+(.*)$", ln)
        if m:
            flush_para(para)
            lvl, txt = len(m.group(1)), m.group(2).strip()
            hid = id_prefix + _slug(txt, seen)
            heads.append({"level": lvl, "text": re.sub(r"`", "", txt), "id": hid})
            out.append(f'<h{lvl} id="{hid}" class="hd hd{lvl}">{_inline(txt, link_map)}</h{lvl}>')
            i += 1
            continue

        # ---- blockquote ----
        if ln.startswith(">"):
            flush_para(para)
            body = []
            while i < n and (lines[i].startswith(">") or
                             (lines[i].strip() and body and not lines[i].startswith("#"))):
                if not lines[i].startswith(">"):
                    break
                body.append(re.sub(r"^>\s?", "", lines[i]))
                i += 1
            inner, _ = convert("\n".join(body), link_map, seen, id_prefix)
            tone = "note"
            joined = " ".join(body)
            if "⚠️" in joined or "DİKKAT" in joined:
                tone = "warn"
            elif "⛔" in joined:
                tone = "stop"
            out.append(f'<blockquote class="cal cal-{tone}">{inner}</blockquote>')
            continue

        # ---- hr ----
        if re.match(r"^-{3,}\s*$", ln):
            flush_para(para)
            out.append('<hr class="rule">')
            i += 1
            continue

        # ---- lists ----
        m = re.match(r"^(\s*)([-*]|\d+\.)\s+(.*)$", ln)
        if m:
            flush_para(para)
            ordered = not m.group(2) in ("-", "*")
            tag = "ol" if ordered else "ul"
            items, base_ind = [], len(m.group(1))
            while i < n:
                mm = re.match(r"^(\s*)([-*]|\d+\.)\s+(.*)$", lines[i])
                if mm and len(mm.group(1)) >= base_ind:
                    items.append(mm.group(3))
                    i += 1
                    # devam satırları
                    while (i < n and lines[i].strip()
                           and not re.match(r"^(\s*)([-*]|\d+\.)\s+", lines[i])
                           and not lines[i].startswith(("#", "|", "```", ">"))
                           and lines[i].startswith("  ")):
                        items[-1] += " " + lines[i].strip()
                        i += 1
                else:
                    break
            lis = "".join(f"<li>{_inline(x, link_map)}</li>" for x in items)
            out.append(f"<{tag}>{lis}</{tag}>")
            continue

        # ---- blank / paragraph ----
        if not ln.strip():
            flush_para(para)
        else:
            para.append(ln.strip())
        i += 1

    flush_para(para)
    return "\n".join(out), heads
