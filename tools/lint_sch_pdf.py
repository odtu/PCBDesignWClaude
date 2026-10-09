"""Schematic text lint on the exported PDFs (lab rule 1.8: no overlapping text).

    python -I tools/lint_sch_pdf.py docs/schematic_signal.pdf [...]

Reports text spans whose boxes overlap another span, or that a wire/symbol line
crosses (title block and frame excluded). Needs PyMuPDF.
"""
import sys

import pymupdf

SHRINK = 0.6        # pt: ignore touching boxes


def spans(page):
    out = []
    for blk in page.get_text("dict")["blocks"]:
        for line in blk.get("lines", []):
            for s in line["spans"]:
                t = s["text"].strip()
                if t and t != "~":
                    r = pymupdf.Rect(s["bbox"])
                    out.append((t, pymupdf.Rect(r.x0 + SHRINK, r.y0 + SHRINK, r.x1 - SHRINK, r.y1 - SHRINK)))
    seen, uniq = set(), []                  # KiCad writes every text twice
    for t, r in out:
        k = (t, round(r.x0), round(r.y0))
        if k not in seen:
            seen.add(k)
            uniq.append((t, r))
    return uniq


def lint(path):
    doc = pymupdf.open(path)
    n = 0
    for pno, page in enumerate(doc):
        W, H = page.rect.width, page.rect.height
        frame = pymupdf.Rect(W * 0.59, H * 0.83, W, H)          # title block area
        ss = [(t, r) for t, r in spans(page) if not r.intersects(frame) and r.y0 > H * 0.04
              and r.x0 > W * 0.03 and r.x1 < W * 0.97 and r.y1 < H * 0.96]
        for i in range(len(ss)):
            for j in range(i + 1, len(ss)):
                if ss[i][1].intersects(ss[j][1]):
                    print(f"p{pno + 1} TEXT/TEXT  {ss[i][0]!r} x {ss[j][0]!r}  at {ss[i][1].x0:.0f},{ss[i][1].y0:.0f}")
                    n += 1
        lines = []
        for d in page.get_drawings():
            for it in d["items"]:
                if it[0] == "l":
                    lines.append((it[1], it[2]))
                elif it[0] == "re":
                    r = it[1]
                    lines += [(r.tl, r.tr), (r.tr, r.br), (r.br, r.bl), (r.bl, r.tl)]
        # wires, pins and symbol outlines are >= 6 pt; shorter strokes are stroke-font glyphs
        lines = [(a, b) for a, b in lines if 6 <= max(abs(a.x - b.x), abs(a.y - b.y)) < W * 0.5]
        for t, r in ss:
            big = pymupdf.Rect(r.x0 - 1.5, r.y0 - 1.5, r.x1 + 1.5, r.y1 + 1.5)
            for a, b in lines:
                if big.contains(a) and big.contains(b):
                    continue                                       # the text's own glyphs
                if _seg_hits(a, b, r):
                    print(f"p{pno + 1} TEXT/LINE  {t!r} at {r.x0:.0f},{r.y0:.0f}")
                    n += 1
                    break
    print(f"{path}: {n} issue(s)")
    return n


def _seg_hits(a, b, r):
    """Segment a-b crosses rect r (Liang-Barsky clip)."""
    t0, t1 = 0.0, 1.0
    dx, dy = b.x - a.x, b.y - a.y
    for p, q in ((-dx, a.x - r.x0), (dx, r.x1 - a.x), (-dy, a.y - r.y0), (dy, r.y1 - a.y)):
        if p == 0:
            if q <= 0:
                return False
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 >= t1:
            return False
    return True


if __name__ == "__main__":
    total = sum(lint(p) for p in sys.argv[1:])
    sys.exit(1 if total else 0)
