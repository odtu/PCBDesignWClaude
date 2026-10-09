"""Signal-board placement, connectivity driven (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/signal_place.py

1. Fixed parts: J2/J3 sockets at the stator headers' exact X/Y (checked pin
   by pin), J1 on the bottom at the lower edge, M3 holes, fiducials.
2. ICs at their block positions (floorplan below), each turned to whichever
   of the four rotations puts its pins closest to what is already placed
   (LDC inputs toward the sockets, op-amp inputs toward the LDC outputs, ...).
3. Every passive, in priority order (decoupling first): all candidate spots
   and rotations around its IC are scored by the distance from each of its
   pads to the nearest already-placed pad of the same net (GND ignored: it
   goes to the plane through a via). The best free spot wins.

Floorplan (board centre 100/100, top view; sockets at the top edge):
  LDC1 U6 under J2 (right)   LDC2 U8 under J3 (left)
  U7 right of the shaft hole U9 left of the shaft hole
  MCU U5 below the hole      buck lower left, LDOs / reference lower right
"""
import math
import os

import pcbnew

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb")
STATOR = os.path.join(ROOT, "InductiveEncoder.kicad_pcb")
LIB = r"D:\Belgeler\METUPowerLab\PowerLabKiCadLibraries\footprints"
CX, CY = 100.0, 100.0
R_MAX = 37.0
R_HOLE = 8.0 + 0.8
HOLES = [(34.0, a) for a in (30, 150, 210, 330)]
HOLE_KEEP = 3.4
GAP = 0.6                   # body-to-body; gives ~1 mm pad-to-pad on 0603
STEP = 0.25
SEARCH = 13.0               # search radius around the block anchor (mm)
J1_POS = (100.0, 134.5)

# IC, anchor (x, y), passives in placement order (decoupling first)
BLOCKS = [
    ("U6", (110.5, 82.0), ["C32", "C31", "C30", "C34", "C33", "FB2",            # VCC / VREG decoupling
                           "C24", "C25",                                         # LC tank at LCIN / LCOUT
                           "C26", "C27", "C28", "C29", "R10", "R11", "R12", "R13",  # input filter
                           "C35", "C36", "C37", "C38",                           # OUTx caps
                           "R14", "R15", "R16", "R17"]),
    ("U8", (89.5, 82.0), ["C54", "C53", "C52", "C56", "C55", "FB3",
                          "C46", "C47",
                          "C48", "C49", "C50", "C51", "R30", "R31", "R32", "R33",
                          "C57", "C58", "C59", "C60",
                          "R34", "R35", "R36", "R37"]),
    ("U7", (122.0, 97.0), ["C39", "R19", "C40", "R18", "R20", "R21", "R22", "C41",
                           "R25", "C43", "R24", "R26", "R27", "R28", "C44",
                           "R23", "C42", "R29", "C45"]),
    ("U9", (78.0, 97.0), ["C61", "R39", "C62", "R38", "R40", "R41", "R42", "C63",
                          "R45", "C65", "R44", "R46", "R47", "R48", "C66",
                          "R43", "C64", "R49", "C67"]),
    ("U5", (100.0, 116.0), ["C19", "C18", "C17", "FB1", "C20", "C23", "C22", "R7", "R6", "C21",
                            "R8", "TP7", "R9", "D2", "TP8", "TP9"]),
    ("U4", (121.0, 112.5), ["C13", "C14", "C16", "C15", "TP5"]),
    ("U2", (116.5, 120.0), ["C7", "C9", "C8", "TP3"]),
    ("U3", (124.5, 121.0), ["C10", "C12", "C11", "TP4"]),
    ("U1", (82.0, 116.5), ["C1", "C2", "C3", "L1", "C4", "C5", "R2", "R1", "C6", "D1", "TP1", "TP2", "TP6"]),
    ("J1", J1_POS, ["R3", "R4", "R5"]),
]


def V(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def mm(v):
    return pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)


def bbox(fp):
    b = fp.GetBoundingBox(False)
    return pcbnew.ToMM(b.GetLeft()), pcbnew.ToMM(b.GetTop()), pcbnew.ToMM(b.GetRight()), pcbnew.ToMM(b.GetBottom())


def fits(box):
    x0, y0, x1, y1 = box
    for x, y in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)):
        if math.hypot(x - CX, y - CY) > R_MAX:
            return False
    for cx, cy, rr in [(CX, CY, R_HOLE)] + [(CX + r * math.cos(math.radians(a)), CY - r * math.sin(math.radians(a)),
                                              HOLE_KEEP) for r, a in HOLES]:
        nx, ny = min(max(cx, x0), x1), min(max(cy, y0), y1)
        if math.hypot(nx - cx, ny - cy) < rr:
            return False
    return True


def free(box, placed):
    for o in placed:
        if not (box[2] + GAP <= o[0] or o[2] + GAP <= box[0] or box[3] + GAP <= o[1] or o[3] + GAP <= box[1]):
            return False
    return True


class Placer:
    def __init__(self, board):
        self.board = board
        self.placed = []                 # bboxes
        self.pads = {}                   # net -> [(x, y)]

    def commit(self, fp):
        self.placed.append(bbox(fp))
        for p in fp.Pads():
            n = p.GetNetname()
            if n and n != "GND" and not n.startswith("unconnected"):
                self.pads.setdefault(n, []).append(mm(p.GetPosition()))

    def shape(self, fp, ang):
        """Pad offsets and bbox relative to the footprint position for a rotation."""
        fp.SetOrientationDegrees(ang)
        fp.SetPosition(V(0, 0))
        pads = [(p.GetNetname(), mm(p.GetPosition())) for p in fp.Pads()]
        return pads, bbox(fp)

    @staticmethod
    def weight(net):
        """Supply / switching nets pull 3x harder: decoupling right at the pin,
        tight buck loop."""
        n = net.split("/")[-1]
        if n.startswith("+") or any(k in n for k in ("VCC", "VREG", "VCORE", "BUCK_SW", "BUCK_CB", "BUCK_FB")):
            return 3.0
        return 1.0

    def cost(self, pads, x, y):
        c, linked = 0.0, 0
        for net, (px, py) in pads:
            pts = self.pads.get(net)
            if not pts:
                continue
            c += self.weight(net) * min(math.hypot(px + x - qx, py + y - qy) for qx, qy in pts)
            linked += 1
        return c, linked

    def best_spot(self, fp, anchor, angles=(0, 90, 180, 270), search=SEARCH):
        cands = []
        ax, ay = anchor
        shapes = {a: self.shape(fp, a) for a in angles}
        n = int(search / STEP)
        for a, (pads, bb) in shapes.items():
            cx, cy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
            for i in range(-n, n + 1):
                for j in range(-n, n + 1):
                    x, y = ax + i * STEP - cx, ay + j * STEP - cy      # footprint position
                    c, linked = self.cost(pads, x, y)
                    d = math.hypot(i * STEP, j * STEP)
                    cands.append((c + 0.2 * d if linked else d, a, x, y))
        cands.sort()
        for c, a, x, y in cands:
            pads, bb = shapes[a]
            box = (bb[0] + x, bb[1] + y, bb[2] + x, bb[3] + y)
            if fits(box) and free(box, self.placed):
                fp.SetOrientationDegrees(a)
                fp.SetPosition(V(x, y))
                return True
        return False


def main():
    board = pcbnew.LoadBoard(PCB)
    stator = pcbnew.LoadBoard(STATOR)
    P = Placer(board)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    log = []

    # sockets at the stator headers (mirrored header -> rotation found by pin matching)
    for ref in ("J2", "J3"):
        hs = stator.FindFootprintByReference(ref)
        pos, ang = hs.GetPosition(), hs.GetOrientationDegrees()
        want = {p.GetNumber(): mm(p.GetPosition()) for p in hs.Pads()}
        fp = fps[ref]
        if fp.IsFlipped():
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
        best = None
        for a in (ang, ang + 180):
            fp.SetOrientationDegrees(((a + 180) % 360) - 180)
            fp.SetPosition(pos)
            got = {p.GetNumber(): mm(p.GetPosition()) for p in fp.Pads()}
            dx, dy = want["1"][0] - got["1"][0], want["1"][1] - got["1"][1]
            e = max(math.dist(want[k], (got[k][0] + dx, got[k][1] + dy)) for k in want)
            if best is None or e < best[0]:
                best = (e, a, dx, dy)
        e, a, dx, dy = best
        fp.SetOrientationDegrees(((a + 180) % 360) - 180)
        fp.SetPosition(pcbnew.VECTOR2I(pos.x + pcbnew.FromMM(dx), pos.y + pcbnew.FromMM(dy)))
        got = {p.GetNumber(): mm(p.GetPosition()) for p in fp.Pads()}
        err = max(math.dist(want[k], got[k]) for k in want)
        if err > 0.01:
            raise SystemExit(f"{ref}: socket pins do not line up with the stator header ({err:.2f} mm)")
        fp.SetLocked(True)
        P.commit(fp)
        log.append(f"{ref}: socket pin match error {err:.3f} mm")

    # mechanical: holes, fiducials
    mech = [(f"H{i + 1}", "METUPowerLab_Mechanicals_MountingHoles", "M3_NPTH", CX + r * math.cos(math.radians(a)),
             CY - r * math.sin(math.radians(a))) for i, (r, a) in enumerate(HOLES)]
    mech += [("FID1", "METUPowerLab_Mechanicals_Fiducials", "Fiducial_1mm_Mask2mm", 100.0, 89.0),
             ("FID2", "METUPowerLab_Mechanicals_Fiducials", "Fiducial_1mm_Mask2mm", 70.5, 110.0),
             ("FID3", "METUPowerLab_Mechanicals_Fiducials", "Fiducial_1mm_Mask2mm", 131.0, 89.0)]
    for ref, lib, name, x, y in mech:
        fp = fps.get(ref)
        if fp is None:
            fp = pcbnew.FootprintLoad(os.path.join(LIB, lib + ".pretty"), name)
            fp.SetFPID(pcbnew.LIB_ID(lib, name))
            fp.SetReference(ref)
            fp.SetBoardOnly(True)
            board.Add(fp)
            fps[ref] = fp
        fp.SetPosition(V(x, y))
        P.commit(fp)

    # J1 on the bottom, lower edge, long axis horizontal
    j1 = fps["J1"]
    if not j1.IsFlipped():
        j1.Flip(j1.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
    for a in (0, 90):
        j1.SetOrientationDegrees(a)
        b = bbox(j1)
        if b[2] - b[0] > b[3] - b[1]:
            break
    b = bbox(j1)
    p = j1.GetPosition()
    j1.SetPosition(pcbnew.VECTOR2I(p.x + pcbnew.FromMM(J1_POS[0] - (b[0] + b[2]) / 2),
                                   p.y + pcbnew.FromMM(J1_POS[1] - (b[1] + b[3]) / 2)))
    P.commit(j1)

    for ic, anchor, passives in BLOCKS:
        if ic != "J1":
            fp = fps[ic]
            if fp.IsFlipped():
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
            if not P.best_spot(fp, anchor, search=3.0):
                log.append(f"{ic}: NO ROOM near {anchor}")
            P.commit(fp)
        for ref in passives:
            fp = fps[ref]
            if fp.IsFlipped():
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
            if not P.best_spot(fp, anchor):
                log.append(f"{ref}: NO ROOM near {ic}")
                continue
            P.commit(fp)

    for ref in ("U1", "U2", "U3", "U4", "U5", "U6", "U7", "U8", "U9"):
        fps[ref].SetAllowSolderMaskBridges(True)
    board.Save(PCB)
    print("\n".join(log))


if __name__ == "__main__":
    main()
