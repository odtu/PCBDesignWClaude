"""Stator board: coils + net ties + J2/J3 coil headers (two-board version).

    python -I tools/stator_layout.py strip                                   # system python
    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/stator_layout.py build

strip: remove every footprint except the net ties NT1-6 and holes H1-4, every
       zone, and every track/via that is not generated coil copper (text edit).
build: add the 2x6 male headers J2/J3 on the bottom at the coil exits, place
       the net ties on the N leads, and hand-route each coil terminal:

  P / A lead (In2.Cu) -> arc r = 31.6 mm -> inner-row pin (odd pin) of its column
  N / B lead (B.Cu)   -> net tie pad 1; pad 2 -> arc r = 32.9 mm -> up between
                         two inner-row pins -> arc r = 36.54 mm -> outer-row pin
  GND pins            -> F.Cu arc between the two pin rows (70 deg span, open)

Each coil has its own angular range, so nothing crosses. Column assignment
(also used by the signal-board sockets):
  J2: col0 EXC1 (pins 1/2), col2 SIN1 (5/6), col4 SIN2 (9/10)
  J3: col0 COS1 (pins 1/2), col2 COS2 (5/6), col4 EXC2 (9/10); cols 1/3/5 GND
"""
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(ROOT, "InductiveEncoder.kicad_pcb")
LIB = r"D:\Belgeler\METUPowerLab\PowerLabKiCadLibraries\footprints"
CX, CY = 100.0, 100.0
W = 0.25
R_IN_ROW, R_OUT_ROW = 33.0, 35.54     # pin rows at the header centre (straight header: ends ~0.6 mm further out)
R_C = (R_IN_ROW + R_OUT_ROW) / 2
DCOL = math.degrees(2.54 / R_C)
R_P, R_N = 31.6, 31.9
KEEP = {"NT1", "NT2", "NT3", "NT4", "NT5", "NT6", "H1", "H2", "H3", "H4"}

#            lead angle, lead end r, NT, P net, N net, header, column, N gap side (+1/-1)
COILS = {
    "EXC1": (70.0, 71.97, 31.2, "NT3", "EXC1_A", "EXC1_B", "J2", 0, +1),
    "SIN1": (78.0, 78.0, 30.6, "NT1", "SIN1_P", "SIN1_N", "J2", 2, +1),
    "SIN2": (83.63, 83.63, 30.6, "NT4", "SIN2_P", "SIN2_N", "J2", 4, +1),
    "COS1": (94.88, 94.88, 30.6, "NT2", "COS1_P", "COS1_N", "J3", 0, +1),
    "COS2": (100.5, 100.5, 30.6, "NT5", "COS2_P", "COS2_N", "J3", 2, +1),
    "EXC2": (106.13, 106.13, 30.6, "NT6", "EXC2_A", "EXC2_B", "J3", 4, -1),
}
COL0 = {"J2": 66.0}
COL0["J3"] = COL0["J2"] + 6.8 * DCOL      # +0.8 pitch: the straight bodies would touch at the inner corners
PINS = {"J2": {0: ("1", "2"), 2: ("5", "6"), 4: ("9", "10")},
        "J3": {0: ("1", "2"), 2: ("5", "6"), 4: ("9", "10")}}
NETS = {"J2": {"1": "EXC1_A", "2": "EXC1_B", "5": "SIN1_P", "6": "SIN1_N", "9": "SIN2_P", "10": "SIN2_N"},
        "J3": {"1": "COS1_P", "2": "COS1_N", "5": "COS2_P", "6": "COS2_N", "9": "EXC2_A", "10": "EXC2_B"}}
for d in NETS.values():
    d.update({k: "GND" for k in ("3", "4", "7", "8", "11", "12")})


def col_theta(hdr, k):
    return COL0[hdr] + k * DCOL


# ---------------------------------------------------------------------------
def strip():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from coilgen import _top_level_items
    text = open(PCB, encoding="utf-8").read()
    items = _top_level_items(text)
    coil = set()
    for it in items:
        if it.startswith('(group "COILS"'):
            coil = set(re.findall(r'"([0-9a-f-]{36})"', it))

    def uid(it):
        m = re.search(r'\(uuid "([0-9a-f-]{36})"\)', it)
        return m.group(1) if m else None

    keep = []
    for it in items:
        if it.startswith("(footprint"):
            ref = re.search(r'\(property "Reference" "([^"]+)"', it)
            if not ref or ref.group(1) not in KEEP:
                continue
        elif it.startswith("(zone"):
            continue
        elif it.startswith(("(segment", "(via", "(arc")) and uid(it) not in coil:
            continue
        keep.append(it)
    head = text[:text.index("(", 1)]
    open(PCB, "w", encoding="utf-8").write(head + "\n\t".join(keep) + "\n)\n")
    print(f"strip: kept {len(keep)} of {len(items)} items")


# ---------------------------------------------------------------------------
def build():
    import pcbnew

    def xy(r, th):
        t = math.radians(th)
        return CX + r * math.cos(t), CY - r * math.sin(t)

    def V(p):
        return pcbnew.VECTOR2I(pcbnew.FromMM(p[0]), pcbnew.FromMM(p[1]))

    def pol(v):
        x, y = pcbnew.ToMM(v.x) - CX, pcbnew.ToMM(v.y) - CY
        return math.hypot(x, y), math.degrees(math.atan2(-y, x)) % 360

    board = pcbnew.LoadBoard(PCB)
    lay = {"F": pcbnew.F_Cu, "B": pcbnew.B_Cu, "L2": board.GetLayerID("In1.Cu"), "L3": board.GetLayerID("In2.Cu")}

    def net(name):
        n = board.FindNet(name)
        if n is None:
            n = pcbnew.NETINFO_ITEM(board, name)
            board.Add(n)
        return n

    # headers
    pads = {}
    for hdr in ("J2", "J3"):
        fp = pcbnew.FootprintLoad(os.path.join(LIB, "METUPowerLab_Connectors_Headers.pretty"), "61301221121")
        fp.SetFPID(pcbnew.LIB_ID("METUPowerLab_Connectors_Headers", "61301221121"))
        fp.SetReference(hdr)
        fp.SetValue("2x6")
        board.Add(fp)
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
        thc = col_theta(hdr, 2.5)
        best = None
        for orient in (thc, thc + 90, thc + 180, thc + 270):
            fp.SetOrientationDegrees(((orient + 180) % 360) - 180)
            fp.SetPosition(V((0, 0)))
            c = {p.GetNumber(): p.GetPosition() for p in fp.Pads()}
            mx = sum(v.x for v in c.values()) / len(c)
            my = sum(v.y for v in c.values()) / len(c)
            tx, ty = xy(R_C, thc)
            fp.SetPosition(pcbnew.VECTOR2I(int(pcbnew.FromMM(tx) - mx), int(pcbnew.FromMM(ty) - my)))
            pp = {p.GetNumber(): pol(p.GetPosition()) for p in fp.Pads()}
            ok = pp["1"][0] < pp["2"][0] and pp["3"][1] > pp["1"][1]   # pin 1 inner, numbering CCW
            if ok:
                best = orient
                break
        if best is None:
            raise SystemExit(f"{hdr}: no orientation with pin 1 inner and CCW numbering")
        for p in fp.Pads():
            p.SetNet(net(NETS[hdr][p.GetNumber()]))
            pads[(hdr, p.GetNumber())] = (pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y))

    # net ties tangentially on the N leads: pad 1 on the lead, pad 2 toward the header gap
    nt_pads = {}
    for name, (ta, tb, r_end, nt, pn, nn, hdr, k, side) in COILS.items():
        fp = board.FindFootprintByReference(nt)
        if not fp.IsFlipped():
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
        gap = col_theta(hdr, k) + side * DCOL / 2
        d = 1 if gap >= tb else -1
        rc = r_end + 0.55
        thc_nt = tb + d * math.degrees(0.5 / rc)
        for extra in (0, 180):
            fp.SetOrientationDegrees(((thc_nt + 90 + extra + 180) % 360) - 180)
            x, y = xy(rc, thc_nt)
            fp.SetPosition(V((x, y)))
            pp = {p.GetNumber(): p.GetPosition() for p in fp.Pads()}
            if (pol(pp["2"])[1] - pol(pp["1"])[1]) * d > 0:
                break
        nt_pads[nt] = {p.GetNumber(): (pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y))
                       for p in fp.Pads()}

    n_seg = 0

    def track(layer, netname, pts):
        nonlocal n_seg
        for a, b in zip(pts, pts[1:]):
            if math.dist(a, b) < 1e-4:
                continue
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(V(a))
            t.SetEnd(V(b))
            t.SetWidth(pcbnew.FromMM(W))
            t.SetLayer(lay[layer])
            t.SetNet(net(netname))
            t.SetLocked(True)
            board.Add(t)
            n_seg += 1

    def arc(r, t0, t1, step=0.3):
        n = max(1, int(abs(math.radians(t1 - t0)) * r / step) + 1)
        return [xy(r, t0 + (t1 - t0) * i / n) for i in range(n + 1)]

    def mid(a, b):
        return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

    def inner(hdr, col):
        return pads[(hdr, str(2 * col + 1))]

    def outer(hdr, col):
        return pads[(hdr, str(2 * col + 2))]

    for name, (ta, tb, r_end, nt, pn, nn, hdr, k, side) in COILS.items():
        thc = col_theta(hdr, k)
        p_pin, n_pin = PINS[hdr][k]
        # P / A lead on In2.Cu: arc inside the pin rows, then straight to the inner-row pin
        track("L3", pn, [xy(r_end, ta)] + arc(R_P, ta, thc) + [pads[(hdr, p_pin)]])
        # N / B lead on B.Cu -> net tie -> between two inner pins -> along the outer row to its pin
        track("B", pn, [xy(r_end, tb), nt_pads[nt]["1"]])
        g_in = mid(inner(hdr, k), inner(hdr, k + side))
        g_out = mid(outer(hdr, k), outer(hdr, k + side))
        r2, t2 = pol(V(nt_pads[nt]["2"]))
        rg, tg = pol(V(g_in))
        track("B", nn, [nt_pads[nt]["2"]] + arc(R_N, t2, tg) + [g_in, g_out, pads[(hdr, n_pin)]])

    # GND pins: F.Cu polyline along the header centre lines (between the rows),
    # with stubs to the GND pins; it spans J2 col1 .. J3 col5 only (no loop)
    cols = [("J2", c) for c in range(1, 6)] + [("J3", c) for c in range(0, 6)]
    centre = [mid(inner(h, c), outer(h, c)) for h, c in cols]
    # bridge the J2-J3 gap along each header's own centre line (not diagonally past pin 1)
    j2_end, j2_prev = centre[4], centre[3]
    j3_start, j3_next = centre[5], centre[6]
    ext2 = (j2_end[0] + 0.5 * (j2_end[0] - j2_prev[0]), j2_end[1] + 0.5 * (j2_end[1] - j2_prev[1]))
    ext3 = (j3_start[0] - 0.5 * (j3_next[0] - j3_start[0]), j3_start[1] - 0.5 * (j3_next[1] - j3_start[1]))
    track("F", "GND", centre[:5] + [ext2, ext3] + centre[5:])
    for (h, c), m in zip(cols, centre):
        if c % 2 == 1:
            track("F", "GND", [m, inner(h, c)])
            track("F", "GND", [m, outer(h, c)])

    board.Save(PCB)
    print(f"build: headers J2/J3 placed, {n_seg} hand-routed segments")


if __name__ == "__main__":
    {"strip": strip, "build": build}[sys.argv[1]]()
