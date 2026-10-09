"""Silkscreen clean-up (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/silk.py <board.kicad_pcb>

- All reference texts: 1.0 mm high, 0.15 mm stroke (lab minimum, rule 2.4).
- ICs, connectors, test points, LED, inductor, holes: reference on silkscreen.
  Each one goes to the nearest free spot, searched outward from the part centre,
  reading at 0 or 90 deg (rule 3.6). A spot is free when the text box keeps
  0.15 mm from every pad, from all part silk and bodies (its own included: no
  reference under a part), from already placed
  references and board texts, from fiducials (1.5 mm), and stays 0.5 mm inside
  the round edge and the shaft hole.
- Passives (R, C, FB, NT, FID): reference moved to the Fab layer. 0603 parts
  sit 0.5-1 mm apart, too close for legible silk references; the assembly
  drawing (Fab layers) carries them instead.
"""
import math
import os
import sys

import pcbnew

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "InductiveEncoder.kicad_pcb")
SILK_PREFIX = ("U", "J", "TP", "D", "L", "H")
CX, CY = 100.0, 100.0          # board centre
R_EDGE, R_HOLE = 38.0, 8.0     # outline and shaft hole radii
EDGE_GAP, GAP = 0.5, 0.15
STEP, SEARCH = 0.25, 7.0
mm, MM = pcbnew.ToMM, pcbnew.FromMM


def box(b, grow=0.0):
    return (mm(b.GetLeft()) - grow, mm(b.GetTop()) - grow, mm(b.GetRight()) + grow, mm(b.GetBottom()) + grow)


def hit(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def inside_ring(r):
    for x, y in ((r[0], r[1]), (r[2], r[1]), (r[0], r[3]), (r[2], r[3])):
        d = math.hypot(x - CX, y - CY)
        if d > R_EDGE - EDGE_GAP or d < R_HOLE + EDGE_GAP:
            return False
    return True


def obstacles(board, back, skip):
    """Pad / silk / body boxes on one side of the board, per footprint."""
    silk = pcbnew.B_SilkS if back else pcbnew.F_SilkS
    cu = pcbnew.B_Cu if back else pcbnew.F_Cu
    obs = {}
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        items = []
        for p in fp.Pads():
            if p.IsOnLayer(cu):
                grow = 1.5 if ref.startswith("FID") else GAP
                items.append(box(p.GetBoundingBox(), grow))
        for g in fp.GraphicalItems():
            if g.GetLayer() == silk:
                items.append(box(g.GetBoundingBox(), GAP))
        if fp.IsFlipped() == back and ref not in skip and not ref.startswith("G"):
            # the body of a part on this side (pads + outlines, texts excluded):
            # no references under it; not every library part has a Fab outline
            items.append(box(fp.GetBoundingBox(False), GAP))
        obs[ref] = items
    for d in board.GetDrawings():
        if d.GetLayer() == silk:
            obs.setdefault("_board", []).append(box(d.GetBoundingBox(), GAP))
    return obs


def place_ref(fp, obs, placed, search=SEARCH):
    t = fp.Reference()
    ref = fp.GetReference()
    # its own pads, silk and body count too: no reference under a part (rule 3.6)
    others = [b for bs in obs.values() for b in bs] + placed
    c = fp.GetBoundingBox(False).GetCenter()
    cx, cy = mm(c.x), mm(c.y)
    ang = fp.GetOrientationDegrees() % 180
    angles = [0, 90] if (ang < 45 or ang > 135) else [90, 0]
    near = [b for b in others if hit(b, (cx - search - 4, cy - search - 4, cx + search + 4, cy + search + 4))]
    shapes = {}
    for a in angles:
        t.SetTextAngleDegrees(a)
        t.SetPosition(pcbnew.VECTOR2I(0, 0))
        shapes[a] = box(t.GetBoundingBox())
    n = int(search / STEP)
    cands = sorted(((i * STEP, j * STEP) for i in range(-n, n + 1) for j in range(-n, n + 1)),
                   key=lambda d: math.hypot(*d))
    for dx, dy in cands:
        x, y = cx + dx, cy + dy
        for a in angles:
            s = shapes[a]
            r = (s[0] + x, s[1] + y, s[2] + x, s[3] + y)
            if not inside_ring(r):
                continue
            if any(hit(r, b) for b in near):
                continue
            t.SetTextAngleDegrees(a)
            t.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
            return r
    return None


def main():
    board = pcbnew.LoadBoard(PCB)
    fps = list(board.GetFootprints())
    moved = kept = 0
    silk_fps = []
    for fp in fps:
        ref = fp.GetReference()
        t = fp.Reference()
        t.SetTextSize(pcbnew.VECTOR2I(MM(1.0), MM(1.0)))
        t.SetTextThickness(MM(0.15))
        back = fp.IsFlipped()
        prefix = ref.rstrip("0123456789")
        if prefix in SILK_PREFIX:
            t.SetLayer(pcbnew.B_SilkS if back else pcbnew.F_SilkS)
            silk_fps.append(fp)
        else:
            # assembly drawing: small reference on the body, along the part
            t.SetLayer(pcbnew.B_Fab if back else pcbnew.F_Fab)
            t.SetTextSize(pcbnew.VECTOR2I(MM(0.5), MM(0.5)))
            t.SetTextThickness(MM(0.08))
            t.SetPosition(fp.GetPosition())
            ang = fp.GetOrientationDegrees() % 180
            t.SetTextAngleDegrees(0 if (ang < 45 or ang > 135) else 90)
            moved += 1
        # library footprints carry a "${REFERENCE}" user text: small ones on silk are
        # below the lab minimum (1.0 mm / 0.15 mm) and go to the Fab layer. On Fab it
        # labels the body in the assembly drawing; then a Fab reference field would
        # print the reference twice, so that field is hidden.
        fab_ref = False
        for item in fp.GraphicalItems():
            if item.Type() != pcbnew.PCB_TEXT_T:
                continue
            if item.GetText() == "${REFERENCE}":
                if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
                    item.SetLayer(pcbnew.B_Fab if item.GetLayer() == pcbnew.B_SilkS else pcbnew.F_Fab)
                fab_ref = fab_ref or item.GetLayer() in (pcbnew.F_Fab, pcbnew.B_Fab)
            elif item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
                if item.GetTextHeight() < MM(1.0):
                    item.SetLayer(pcbnew.B_Fab if item.GetLayer() == pcbnew.B_SilkS else pcbnew.F_Fab)
                elif item.GetTextThickness() < MM(0.15):
                    item.SetTextThickness(MM(0.15))
        t.SetVisible(not (fab_ref and t.GetLayer() in (pcbnew.F_Fab, pcbnew.B_Fab)))
    # most constrained first: ICs, then connectors, test points, the rest
    order = {"U": 0, "D": 1, "L": 1, "J": 2, "TP": 3, "H": 4}
    silk_fps.sort(key=lambda f: order.get(f.GetReference().rstrip("0123456789"), 5))
    placed = {False: [], True: []}
    obs = {side: obstacles(board, side, ()) for side in (False, True)}
    failed = []
    for fp in silk_fps:
        side = fp.IsFlipped()
        r = place_ref(fp, obs[side], placed[side]) or place_ref(fp, obs[side], placed[side], 12.0)
        if r is None:
            failed.append(fp.GetReference())
            fp.Reference().SetLayer(pcbnew.B_Fab if side else pcbnew.F_Fab)
            fp.Reference().SetPosition(fp.GetPosition())
        else:
            placed[side].append(tuple(v + (GAP if i > 1 else -GAP) for i, v in enumerate(r)))
            kept += 1
    board.Save(PCB)
    print(f"silk refs placed {kept}, moved to Fab {moved}, no free spot (-> Fab): {failed}")


if __name__ == "__main__":
    main()
