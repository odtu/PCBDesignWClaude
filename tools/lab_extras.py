"""Lab-rule extras on the signal board (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/lab_extras.py

- Local fiducial pair diagonally across U5 (MSPM0, 0.5 mm pitch; rule 3.2):
  the nearest pair of free 3 mm keepout spots on opposite sides of the part.
- GND stitching vias (rule 3.4) on a 5 mm grid: a grid point (or a spot within
  1.5 mm of it) gets a 0.3/0.6 mm GND via when it keeps 0.55 mm from other-net
  copper on every layer, 1 mm from other vias and holes, and stays off silk,
  fiducial keepouts, the edge/shaft-hole keepouts and the pour slot. After a
  zone fill, vias that don't land in the GND fill on at least two layers are
  removed again.
- J1 voltage/pin labels on the bottom silk (rule 3.6).
Re-running replaces what an earlier run added (the vias in group STITCH,
FID4/FID5, the J1 label texts).
"""
import math
import os

import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from coilgen import _top_level_items  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb")
FID_LIB = r"D:\Belgeler\METUPowerLab\PowerLabKiCadLibraries\footprints\METUPowerLab_Mechanicals_Fiducials.pretty"
FID_FP = "Fiducial_1mm_Mask2mm"
CX, CY = 100.0, 100.0
R_MIN, R_MAX = 9.6, 36.4            # inside HOLE/EDGE keepouts with margin
SLOT_HALF = 1.3                     # pour slot at 90 deg (x = 100, y < 100)
VIA_D, VIA_DRILL = 0.6, 0.3
PITCH = 5.0
J1_LABELS = ["+VIN 12-24V: 1,2   GND: 3,4,11",
             "RX 5  TX 6  SYNC 7  NRST 8",
             "SWDIO 9  SWCLK 10  +3V3 12"]
# texts of earlier runs, removed by text edit (SWIG can't delete board texts)
OLD_LABEL_PREFIXES = ("+VIN 12-24V", "RX 5", "SWDIO 9", "1,2 +VIN", "5 RX")
mm, MM = pcbnew.ToMM, pcbnew.FromMM


def V(x, y):
    return pcbnew.VECTOR2I(MM(x), MM(y))


def rect_dist(x, y, b):
    dx = max(b[0] - x, 0, x - b[2])
    dy = max(b[1] - y, 0, y - b[3])
    return math.hypot(dx, dy)


def seg_dist(x, y, a, b):
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    L = vx * vx + vy * vy
    t = 0 if L == 0 else max(0, min(1, ((x - ax) * vx + (y - ay) * vy) / L))
    return math.hypot(x - ax - t * vx, y - ay - t * vy)


def bbox(item, grow=0.0):
    b = item.GetBoundingBox()
    return (mm(b.GetLeft()) - grow, mm(b.GetTop()) - grow, mm(b.GetRight()) + grow, mm(b.GetBottom()) + grow)


class Copper:
    """Copper on each layer: pads as boxes, tracks as segments, vias as circles."""

    def __init__(self, board):
        self.layers = {n: board.GetLayerID(n) for n in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu")}
        self.pads = []      # (box, layer ids, net)
        self.segs = []      # (a, b, half width, layer, net)
        self.vias = []      # (x, y, r, net)
        for fp in board.GetFootprints():
            for p in fp.Pads():
                lay = [l for l in self.layers.values() if p.IsOnLayer(l)]
                if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                    lay = list(self.layers.values())
                self.pads.append((bbox(p), lay, p.GetNetname()))
        for t in board.GetTracks():
            if t.Type() == pcbnew.PCB_VIA_T:
                self.vias.append((mm(t.GetPosition().x), mm(t.GetPosition().y), mm(t.GetWidth(pcbnew.F_Cu)) / 2,
                                  t.GetNetname()))
            else:
                a, b = t.GetStart(), t.GetEnd()
                self.segs.append(((mm(a.x), mm(a.y)), (mm(b.x), mm(b.y)), mm(t.GetWidth()) / 2, t.GetLayer(),
                                  t.GetNetname()))

    def clear(self, x, y, r, layers, net=None, gap=0.2):
        """Circle (x, y, r) keeps gap from all copper on `layers` not on `net`."""
        for b, lay, n in self.pads:
            if n != net or net is None:
                if set(lay) & set(layers) and rect_dist(x, y, b) < r + gap:
                    return False
        for a, b, hw, l, n in self.segs:
            if (n != net or net is None) and l in layers and seg_dist(x, y, a, b) < r + hw + gap:
                return False
        for vx, vy, vr, n in self.vias:
            if (n != net or net is None) and math.hypot(x - vx, y - vy) < r + vr + gap:
                return False
        return True


def silk_boxes(board, layer, grow):
    out = []
    for fp in board.GetFootprints():
        for g in list(fp.GraphicalItems()) + [fp.Reference()]:
            if g.GetLayer() == layer and (g.Type() != pcbnew.PCB_FIELD_T or g.IsVisible()):
                out.append(bbox(g, grow))
    for d in board.GetDrawings():
        if d.GetLayer() == layer:
            out.append(bbox(d, grow))
    return out


def in_ring(x, y, rmin=R_MIN, rmax=R_MAX):
    r = math.hypot(x - CX, y - CY)
    return rmin <= r <= rmax and not (abs(x - CX) < SLOT_HALF and y < CY)


def remove_previous():
    """Text edit: drop the STITCH group with its vias, FID4/FID5 and old J1 labels."""
    text = open(PCB, encoding="utf-8").read()
    items = _top_level_items(text)
    stitch = set()
    for it in items:
        if it.startswith('(group "STITCH"'):
            stitch = set(re.findall(r'"([0-9a-f-]{36})"', it))

    def old(it):
        if it.startswith('(group "STITCH"'):
            return True
        if it.startswith("(via"):
            m = re.search(r'\(uuid "([0-9a-f-]{36})"\)', it)
            return bool(m and m.group(1) in stitch)
        if it.startswith("(footprint") and re.search(r'\(property "Reference" "FID[45]"', it):
            return True
        if it.startswith("(gr_text"):
            m = re.match(r'\(gr_text "([^"]*)"', it)
            return bool(m and m.group(1).startswith(OLD_LABEL_PREFIXES))
        return False

    keep = [it for it in items if not old(it)]
    head = text[:text.index("(", 1)]
    open(PCB, "w", encoding="utf-8").write(head + "\n\t".join(keep) + "\n)\n")
    print(f"removed {len(items) - len(keep)} items of an earlier run")


def fiducials(board, cu):
    u5 = next(f for f in board.GetFootprints() if f.GetReference() == "U5")
    c = u5.GetPosition()
    ux, uy = mm(c.x), mm(c.y)
    silk = silk_boxes(board, pcbnew.F_SilkS, 0.0)
    free = []
    for i in range(-48, 49):
        for j in range(-48, 49):
            x, y = ux + i * 0.25, uy + j * 0.25
            d = math.hypot(x - ux, y - uy)
            if not 3.5 <= d <= 12 or not in_ring(x, y, 10.0, 36.0):
                continue
            # 1 mm pad + 1 mm keepout clearance; no silk inside the 3 mm keepout
            if not cu.clear(x, y, 0.5, [cu.layers["F.Cu"]], gap=1.05):
                continue
            if any(rect_dist(x, y, b) < 1.55 for b in silk):
                continue
            free.append((x, y, math.atan2(y - uy, x - ux), d))
    best = None
    for a in free:
        for b in free:
            dang = abs((a[2] - b[2] + math.pi) % (2 * math.pi) - math.pi)
            if dang < math.radians(150) or math.hypot(a[0] - b[0], a[1] - b[1]) < 6:
                continue
            # diagonal across the part: neither spot on a pin-row axis
            if min(abs(math.sin(2 * a[2])), abs(math.sin(2 * b[2]))) < 0.5:
                continue
            score = a[3] + b[3]
            if best is None or score < best[0]:
                best = (score, a, b)
    if best is None:
        print("no free diagonal fiducial pair near U5")
        return []
    placed = []
    for ref, (x, y, _, d) in zip(("FID4", "FID5"), best[1:]):
        fp = pcbnew.FootprintLoad(FID_LIB, FID_FP)
        fp.SetFPID(pcbnew.LIB_ID("METUPowerLab_Mechanicals_Fiducials", FID_FP))
        fp.SetReference(ref)
        fp.SetBoardOnly(True)
        fp.SetExcludedFromBOM(True)
        fp.SetExcludedFromPosFiles(True)
        fp.SetPosition(V(x, y))
        fp.Reference().SetLayer(pcbnew.F_Fab)
        board.Add(fp)
        placed.append((x, y))
        print(f"{ref} at ({x:.2f}, {y:.2f}), {d:.1f} mm from U5")
    return placed


def j1_labels(board):
    # text edit can't be done through SWIG removal reliably: only add when missing
    have = {d.GetText() for d in board.GetDrawings() if d.Type() == pcbnew.PCB_TEXT_T}
    j1 = next(f for f in board.GetFootprints() if f.GetReference() == "J1")
    x = sum(mm(p.GetPosition().x) for p in j1.Pads()) / len(list(j1.Pads()))    # origin is pin 1
    top = min(mm(p.GetPosition().y) for p in j1.Pads())     # row toward the board centre
    for i, s in enumerate(J1_LABELS):
        if s in have:
            continue
        t = pcbnew.PCB_TEXT(board)
        t.SetText(s)
        t.SetLayer(pcbnew.B_SilkS)
        t.SetMirrored(True)
        t.SetTextSize(V(1.0, 1.0))
        t.SetTextThickness(MM(0.15))
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
        t.SetPosition(V(x, top - 6.0 + i * 1.6))
        board.Add(t)


def stitching(board, cu, fids):
    gnd = board.FindNet("GND")
    silk = silk_boxes(board, pcbnew.F_SilkS, 0.25) + silk_boxes(board, pcbnew.B_SilkS, 0.25)
    all_layers = list(cu.layers.values())
    group = pcbnew.PCB_GROUP(board)
    group.SetName("STITCH")
    board.Add(group)
    added = []
    n = int(R_MAX / PITCH) + 1
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            gx, gy = CX + i * PITCH + PITCH / 2, CY + j * PITCH + PITCH / 2
            spots = sorted(((gx + a * 0.25, gy + b * 0.25) for a in range(-6, 7) for b in range(-6, 7)),
                           key=lambda p: math.hypot(p[0] - gx, p[1] - gy))
            for x, y in spots:
                if not in_ring(x, y):
                    continue
                if any(math.hypot(x - fx, y - fy) < 1.5 + 0.3 + 0.3 for fx, fy in fids):
                    continue
                if any(rect_dist(x, y, b) < VIA_D / 2 for b in silk):
                    continue
                # every pad counts (no vias in or next to pads, thermal-via fields included)
                if not cu.clear(x, y, VIA_D / 2, all_layers, net=None, gap=0.25):
                    continue
                # hole-to-hole and spacing to every via (any net), 1 mm centre to centre
                if any(math.hypot(x - vx, y - vy) < 1.0 for vx, vy, _, _ in cu.vias):
                    continue
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(V(x, y))
                v.SetWidth(MM(VIA_D))
                v.SetDrill(MM(VIA_DRILL))
                v.SetNet(gnd)
                board.Add(v)
                group.AddItem(v)
                cu.vias.append((x, y, VIA_D / 2, "GND"))
                added.append(v)
                break
    return added


def connected_layers(board, via):
    p = via.GetPosition()
    n = 0
    for z in board.Zones():
        if z.GetNetname() != "GND" or z.GetIsRuleArea():
            continue
        for l in z.GetLayerSet().Seq():
            if z.GetFilledPolysList(l).Contains(p):
                n += 1
    return n


def main():
    remove_previous()
    board = pcbnew.LoadBoard(PCB)
    cu = Copper(board)
    fids = fiducials(board, cu)
    j1_labels(board)
    cu = Copper(board)
    vias = stitching(board, cu, fids)
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    weak = [v for v in vias if connected_layers(board, v) < 2]
    for v in weak:
        board.Remove(v)
    filler.Fill(board.Zones())
    board.Save(PCB)
    print(f"stitching vias: {len(vias) - len(weak)} added ({len(weak)} dropped: not in the GND fill on 2+ layers)")


if __name__ == "__main__":
    main()
