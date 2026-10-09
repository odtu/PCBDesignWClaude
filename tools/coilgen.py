"""Inductive encoder coil generator (stator coils + rotating target).

Run with KiCad's Python (needs pcbnew):
    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/coilgen.py            # stator
    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/coilgen.py --target   # target disc
    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/coilgen.py --preview  # PNG only

Concept (after TI TIDA-010961): two coil tracks with coprime period counts
(16 outer / 15 inner) form a Nonius pair for absolute angle. Each track has a
sine and a cosine receive coil plus one excitation ring driven by its own
LDC5072. The two tank circuits run at different frequencies so the tracks
do not beat against each other.

Layer plan (4-layer stator, target faces L1):
  L1/L2  receive coils and excitation rings.
         Receive coils alternate layers every half period:
           SIN coil: outer half -> L1, inner half -> L2
           COS coil: outer half -> L2, inner half -> L1
         so a SIN trace only ever meets a COS trace on the other layer.
  L3/L4  coil leads, run radially as broadside P/N pairs in "lanes"
         midway between the receive-coil via columns.

Every coil and both its leads carry the P-side net (e.g. SIN1_P). The
N-lead ends in the electronics ring where a NetTie (the coil's schematic
symbol) joins it to the N-side net. That keeps DRC clean.

Re-running replaces everything in the COILS / TARGET group, so it is safe to
run again after footprints and other routing are added.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ----------------------------------------------------------------------------
# Parameters (mm, degrees)
# ----------------------------------------------------------------------------
CX, CY = 100.0, 100.0          # board centre in KiCad coordinates
TRACK_W = 0.20                 # coil track width (lab default)
CLEAR = 0.20                   # lab default clearance
VIA_D, VIA_DRILL = 0.60, 0.30  # lab signal via
PITCH = TRACK_W + CLEAR        # excitation spiral pitch

BOARD_R = 38.0                 # stator outline radius (Ø76)
SHAFT_R = 8.0                  # shaft hole radius (Ø16)
R_EXIT = 30.6                  # where coil leads end, for routing to the AFEs

OUTER = dict(name="1", N=16, R=24.0, A=2.8)   # outer track, 16 periods
INNER = dict(name="2", N=15, R=13.9, A=2.8)   # inner track, 15 periods

# Excitation rings: (r_lo, turns per layer, terminal side)
EXC1 = dict(r_lo=27.7, n=3, side="out")       # outer track, outside its band
EXC2 = dict(r_lo=18.1, n=5, side="in")        # inner track, between the tracks
TERM_GAP = 1.0                                 # arc gap between ring terminals

FEED_THETA = 78.0              # mechanical angle of the first outer lead lane
EXC1_THETA = 70.0              # ring 1 terminal angle
R_JOG = 19.4                   # radius where inner-track leads shift lanes

# Target disc
TARGET_R = 28.0                # Ø56 outline
TARGET_MARGIN = 0.3            # lobe overhang past the receive band


# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------
def xy(r, th_deg):
    t = math.radians(th_deg)
    return (CX + r * math.cos(t), CY - r * math.sin(t))


def arc_pts(r, th0, th1, max_seg=0.15):
    n = max(1, int(math.ceil(abs(math.radians(th1 - th0)) * r / max_seg)))
    return [xy(r, th0 + (th1 - th0) * i / n) for i in range(n + 1)]


def polar_path(wps, max_seg=0.15):
    """wps: [(r, th), ...]; equal r -> arc, otherwise straight line."""
    pts = [xy(*wps[0])]
    for (r0, t0), (r1, t1) in zip(wps, wps[1:]):
        if abs(r0 - r1) < 1e-9 and abs(t0 - t1) > 1e-9:
            pts += arc_pts(r0, t0, t1, max_seg)[1:]
        else:
            pts.append(xy(r1, t1))
    return pts


def via_offset(trk):
    """Electrical offset (deg) of the layer-switch vias after each crossing.

    Near a crossing the two curves are lines of slope +/-k (dr/ds). The vias
    must clear each other and the opposite curve by pad + track + clearance.
    """
    k = trk["A"] * trk["N"] / trk["R"]
    need = VIA_D / 2 + TRACK_W / 2 + CLEAR
    x = need * math.sqrt(1 + k * k) / (2 * k)          # arc distance (mm)
    x = max(x, (VIA_D + CLEAR) / (2 * k)) * 1.1          # via-to-via + 10 %
    return math.degrees(x / trk["R"]) * trk["N"]


class Geo:
    """Abstract primitives, emitted to pcbnew and to the matplotlib preview."""

    def __init__(self):
        self.tracks = []   # (layer, net, [pts], width)
        self.vias = []     # (net, (x, y))
        self.polys = []    # (layer, [pts])

    def track(self, layer, net, pts, w=TRACK_W):
        self.tracks.append((layer, net, pts, w))

    def via(self, net, p):
        self.vias.append((net, p))


# ----------------------------------------------------------------------------
# Receive coils
# ----------------------------------------------------------------------------
def theta_of(trk, u):
    return trk["theta0"] + u / trk["N"]


def receive_coil(g, trk, kind, net):
    """One sine-shaped receive coil, fed at a crossing.

    Local electrical angle v: curve a = R + A sin v, curve b = R - A sin v.
    SIN coil: v = u, P side = curve a.  COS coil: v = u - 270, P side = curve a
    (= R + A cos u). COS is fed at its 270 deg crossing so neither feed turnaround
    via lands inside the other coil's lead lane.
    Feed crossing at v = 0.
    Returns (P via (r, th), N via (r, th)).
    """
    N, R, A, d = trk["N"], trk["R"], trk["A"], trk["delta"]
    v0 = 0.0 if kind == "SIN" else 270.0
    lay_outer, lay_inner = ("L1", "L2") if kind == "SIN" else ("L2", "L1")
    vend = 360.0 * N
    step = 2.0
    ends = {}
    for sigma in (+1, -1):
        def rad(v):
            return R + sigma * A * math.sin(math.radians(v))
        # segments between consecutive layer switches at v = k*180 + d
        for k in range(2 * N):
            va = k * 180.0 + d
            vb = min((k + 1) * 180.0 + d, vend)
            outer = sigma * (1 if k % 2 == 0 else -1) > 0
            layer = lay_outer if outer else lay_inner
            n = max(2, int(math.ceil((vb - va) / step)))
            pts = [xy(rad(va + (vb - va) * i / n), theta_of(trk, v0 + va + (vb - va) * i / n))
                   for i in range(n + 1)]
            g.track(layer, net, pts)
            if k > 0:
                g.via(net, xy(rad(va), theta_of(trk, v0 + va)))
        ends[sigma] = (rad(d), theta_of(trk, v0 + d))
        g.via(net, xy(*ends[sigma]))
    # turnaround at the feed crossing (v = 360N == 0): both curves meet at R
    g.via(net, xy(R, theta_of(trk, v0 + vend)))
    p_sigma = +1
    return ends[p_sigma], ends[-p_sigma]


def lane_theta(trk, k):
    """Mechanical angle of lead lane k (midway between via columns)."""
    return theta_of(trk, trk["delta"] + 45.0 + 90.0 * k)


def feed_lane(trk, kind):
    return lane_theta(trk, 0 if kind == "SIN" else 3)


# ----------------------------------------------------------------------------
# Excitation rings
# ----------------------------------------------------------------------------
def spiral_pts(r0, r1, th0, span, max_seg=0.15):
    rm = max(r0, r1)
    n = max(4, int(math.ceil(math.radians(abs(span)) * rm / max_seg)))
    return [xy(r0 + (r1 - r0) * i / n, th0 + span * i / n) for i in range(n + 1)]


def exc_ring(g, ring, net, th_term, th_trans_target, trk_facing):
    """Two-layer spiral: L1 from the terminal edge to the far edge, transition
    via, L2 back to the terminal edge. Both terminals end on vias just outside
    the terminal edge. The transition via sits outside the far edge at an angle
    where the facing receive band is furthest away (u = 45 deg mod 90).
    """
    n = ring["n"]
    width = (n + 0.5) * PITCH
    r_lo, r_hi = ring["r_lo"], ring["r_lo"] + width
    if ring["side"] == "out":
        r_t, r_f, out = r_hi, r_lo, +1
    else:
        r_t, r_f, out = r_lo, r_hi, -1
    # transition angle: nearest u = 45 mod 90 of the facing track, ~half a turn away
    best = None
    for kk in range(-400, 400):
        th = theta_of(trk_facing, 45.0 + 90.0 * kk)
        f = ((th - th_term) % 360.0) / 360.0
        if 0.30 <= f <= 0.50 and (best is None or abs(f - 0.5) < abs(best[0] - 0.5)):
            best = (f, th)
    f, _ = best if th_trans_target is None else (((th_trans_target - th_term) % 360) / 360, None)
    s1 = 360.0 * (n + f)
    gap_deg = math.degrees(TERM_GAP / r_t)
    s2 = 360.0 * (n - f) + gap_deg
    th_tr = th_term + s1
    th_b = th_tr + s2
    jog = VIA_D / 2 + TRACK_W / 2 + CLEAR + 0.1          # 0.6 -> 0.7 mm
    # L1: terminal A -> far edge
    g.track("L1", net, polar_path([(r_t + out * jog, th_term), (r_t, th_term)]))
    g.track("L1", net, spiral_pts(r_t, r_f, th_term, s1))
    g.track("L1", net, polar_path([(r_f, th_tr), (r_f - out * jog, th_tr)]))
    g.via(net, xy(r_f - out * jog, th_tr))
    # L2: far edge -> terminal B
    g.track("L2", net, polar_path([(r_f - out * jog, th_tr), (r_f, th_tr)]))
    g.track("L2", net, spiral_pts(r_f, r_t, th_tr, s2))
    g.track("L2", net, polar_path([(r_t, th_b), (r_t + out * jog, th_b)]))
    a = (r_t + out * jog, th_term)
    b = (r_t + out * jog, th_b % 360.0)
    g.via(net, xy(*a))
    g.via(net, xy(*b))
    turns = [(r_t + (r_f - r_t) * (i + 0.5) / (n + f), 0.0) for i in range(int(round(n + f)))]
    turns += [(r_f + (r_t - r_f) * (i + 0.5) / (n - f), 0.2) for i in range(int(round(n - f)))]
    return a, b, (r_lo, r_hi), turns


# ----------------------------------------------------------------------------
# Build the stator coil set
# ----------------------------------------------------------------------------
def build_stator():
    g = Geo()
    for t in (OUTER, INNER):
        t["delta"] = via_offset(t)
    # outer track phase: its SIN lane (lane 0) at FEED_THETA
    OUTER["theta0"] = 0.0
    OUTER["theta0"] = FEED_THETA - (lane_theta(OUTER, 0) - 0.0)
    # inner track phase: its SIN lane coincides with outer lane 2
    INNER["theta0"] = 0.0
    INNER["theta0"] = lane_theta(OUTER, 1) - lane_theta(INNER, 0)

    report = []
    # --- receive coils + leads ------------------------------------------------
    for trk in (OUTER, INNER):
        for kind in ("SIN", "COS"):
            net = f"{kind}{trk['name']}_P"
            (rp, tp), (rn, tn) = receive_coil(g, trk, kind, net)
            lane = feed_lane(trk, kind)
            if trk is OUTER:
                out_lane = lane
                jog = None
            else:
                out_lane = lane_theta(OUTER, 1 if kind == "SIN" else 4)
                jog = None if abs(out_lane - lane) < 1e-6 else R_JOG
            for (r0, t0, layer) in ((rp, tp, "L3"), (rn, tn, "L4")):
                wps = [(r0, t0), (r0, lane)]
                if jog is not None:
                    wps += [(jog, lane), (jog, out_lane)]
                wps += [(R_EXIT, out_lane)]
                g.track(layer, net, polar_path(wps))
            report.append(f"{kind}{trk['name']}: N={trk['N']} R={trk['R']} A={trk['A']} "
                          f"via offset={trk['delta']:.1f} deg el, lane {out_lane:.2f} deg")

    # --- excitation ring 1 (outer track, terminals outward) -------------------
    a1, b1, band1, turns1 = exc_ring(g, EXC1, "EXC1_A", EXC1_THETA, None, OUTER)
    for (r0, t0), layer in ((a1, "L3"), (b1, "L4")):
        g.track(layer, "EXC1_A", polar_path([(r0, t0), (R_EXIT + 0.6, t0)]))

    # --- excitation ring 2 (inner track, between the tracks, terminals inward)
    th_e2 = lane_theta(OUTER, 5)
    a2, b2, band2, turns2 = exc_ring(g, EXC2, "EXC2_A", th_e2, None, OUTER)
    # A lead straight out on L3; B lead steps over the A via, then runs under A on L4
    g.track("L3", "EXC2_A", polar_path([a2, (R_EXIT, th_e2)]))
    r_step = a2[0] + VIA_D / 2 + TRACK_W / 2 + CLEAR + 0.2
    g.track("L4", "EXC2_A", polar_path([b2, (r_step, b2[1]), (r_step, th_e2), (R_EXIT, th_e2)]))

    # --- inductance estimate --------------------------------------------------
    sys.path.insert(0, HERE)
    try:
        from inductance import coil_l
    except ImportError:              # KiCad's Python has no scipy
        return g, report
    for name, turns in (("EXC1", turns1), ("EXC2", turns2)):
        L = coil_l([(r * 1e-3, z * 1e-3, 1) for r, z in turns], TRACK_W * 1e-3)
        report.append(f"{name}: band {band1 if name == 'EXC1' else band2}, "
                      f"{len(turns)} turns total, L = {L * 1e6:.2f} uH")
        for c in (330e-12, 390e-12, 470e-12, 560e-12):
            f = 1 / (2 * math.pi * math.sqrt(L * c / 2))
            report.append(f"    C1=C2={c * 1e12:.0f} pF -> f = {f / 1e6:.2f} MHz")
    return g, report


# ----------------------------------------------------------------------------
# Target disc
# ----------------------------------------------------------------------------
def build_target():
    g = Geo()
    for trk in (OUTER, INNER):
        r0 = trk["R"] - trk["A"] - TARGET_MARGIN
        r1 = trk["R"] + trk["A"] + TARGET_MARGIN
        w = 180.0 / trk["N"]
        for k in range(trk["N"]):
            t0 = k * 360.0 / trk["N"]
            outline = arc_pts(r1, t0, t0 + w) + arc_pts(r0, t0 + w, t0)
            for layer in ("L1", "L4"):
                g.polys.append((layer, outline))
    return g


# ----------------------------------------------------------------------------
# Preview (matplotlib, no KiCad needed)
# ----------------------------------------------------------------------------
COLORS = {"L1": "#c83434", "L2": "#c8a000", "L3": "#2a8f2a", "L4": "#3050d0"}


def preview(g, path, title, outline_r):
    try:
        import matplotlib
    except ImportError:              # KiCad's Python: skip the preview
        return
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 10), dpi=150)
    for layer, _, pts, w in g.tracks:
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=COLORS[layer], lw=0.5, alpha=0.85)
    for layer, pts in g.polys:
        xs, ys = zip(*pts)
        ax.fill(xs, ys, color=COLORS[layer], alpha=0.5, lw=0)
    for _, (x, y) in g.vias:
        ax.add_patch(plt.Circle((x, y), VIA_D / 2, color="k", fill=False, lw=0.4))
    for r in outline_r:
        ax.add_patch(plt.Circle((CX, CY), r, color="0.4", fill=False, lw=0.8))
    for layer, c in COLORS.items():
        ax.plot([], [], color=c, label=layer)
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_title(title)
    fig.savefig(path, bbox_inches="tight")
    print("preview:", path)


# ----------------------------------------------------------------------------
# KiCad output
# ----------------------------------------------------------------------------
LAYER_NAME = {"L1": "F.Cu", "L2": "In1.Cu", "L3": "In2.Cu", "L4": "B.Cu"}


def _top_level_items(text):
    """Split the body of (kicad_pcb ...) into top-level item strings."""
    items, depth, start, in_str, esc = [], 0, None, False, False
    for i, ch in enumerate(text):
        if in_str:
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == "(":
            depth += 1
            if depth == 2:
                start = i
        elif ch == ")":
            if depth == 2:
                items.append(text[start:i + 1])
            depth -= 1
    return items


def emit(g, pcb_path, group_name, outline_r):
    """Write g into the board file, replacing the previous run.

    Edits the s-expression text directly: KiCad 10's SWIG bindings cannot
    enumerate group members, but the file lists them by UUID.
    """
    import re
    import uuid
    text = open(pcb_path, encoding="utf-8").read()
    items = _top_level_items(text)
    names = {group_name, "OUTLINE"}
    drop = set()
    for it in items:
        m = re.match(r'\(group "([^"]*)"', it)
        if m and m.group(1) in names:
            drop.update(re.findall(r'"([0-9a-f-]{36})"', it))
    keep = []
    for it in items:
        m = re.match(r'\(group "([^"]*)"', it)
        if m and m.group(1) in names:
            continue
        u = re.search(r'\(uuid "([0-9a-f-]{36})"\)', it)
        if u and u.group(1) in drop and not it.startswith("(footprint"):
            continue
        keep.append(it)

    def f(x):
        return f"{x:.6f}".rstrip("0").rstrip(".")

    new, members = [], []

    def add(s):
        u = str(uuid.uuid4())
        members.append(u)
        new.append(s.replace("@UUID@", u))

    for layer, nname, pts, w in g.tracks:
        for p0, p1 in zip(pts, pts[1:]):
            if math.dist(p0, p1) < 1e-4:
                continue
            add(f'(segment (start {f(p0[0])} {f(p0[1])}) (end {f(p1[0])} {f(p1[1])}) '
                f'(width {f(w)}) (layer "{LAYER_NAME[layer]}") (net "{nname}") (uuid "@UUID@"))')
    for nname, p in g.vias:
        add(f'(via (at {f(p[0])} {f(p[1])}) (size {f(VIA_D)}) (drill {f(VIA_DRILL)}) '
            f'(layers "F.Cu" "B.Cu") (net "{nname}") (uuid "@UUID@"))')
    for layer, pts in g.polys:
        xy_s = " ".join(f"(xy {f(x)} {f(y)})" for x, y in pts)
        add(f'(gr_poly (pts {xy_s}) (stroke (width 0) (type solid)) (fill yes) '
            f'(layer "{LAYER_NAME[layer]}") (uuid "@UUID@"))')
    coil_members = members
    groups = [f'(group "{group_name}" (uuid "{uuid.uuid4()}") (locked yes) (members '
              + " ".join(f'"{u}"' for u in coil_members) + "))"]
    members = []
    for r in outline_r:
        add(f'(gr_circle (center {f(CX)} {f(CY)}) (end {f(CX + r)} {f(CY)}) '
            f'(stroke (width 0.1) (type solid)) (fill no) (layer "Edge.Cuts") (uuid "@UUID@"))')
    groups.append(f'(group "OUTLINE" (uuid "{uuid.uuid4()}") (locked yes) (members '
                  + " ".join(f'"{u}"' for u in members) + "))")

    head = text[:text.index("(", 1)]          # "(kicad_pcb\n\t"
    body = "\n\t".join(keep + new + groups)
    open(pcb_path, "w", encoding="utf-8").write(head + body + "\n)\n")
    print(f"wrote {pcb_path}: {len(coil_members)} copper items "
          f"({len(g.tracks)} polylines, {len(g.vias)} vias, {len(g.polys)} polys)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", action="store_true", help="generate the target disc")
    ap.add_argument("--preview", action="store_true", help="PNG preview only")
    args = ap.parse_args()
    if args.target:
        g, rep = build_stator()          # sets track phases (not needed, but consistent)
        g = build_target()
        out = os.path.join(ROOT, "target")
        os.makedirs(out, exist_ok=True)
        preview(g, os.path.join(ROOT, "docs", "target_preview.png"), "Target (L1/L4 copper)",
                (TARGET_R, SHAFT_R))
        if not args.preview:
            emit(g, os.path.join(out, "EncoderTarget.kicad_pcb"), "TARGET", (TARGET_R, SHAFT_R))
        return
    g, rep = build_stator()
    print("\n".join(rep))
    os.makedirs(os.path.join(ROOT, "docs"), exist_ok=True)
    preview(g, os.path.join(ROOT, "docs", "coils_preview.png"), "Stator coils", (BOARD_R, SHAFT_R, 29.0))
    if not args.preview:
        emit(g, os.path.join(ROOT, "InductiveEncoder.kicad_pcb"), "COILS", (BOARD_R, SHAFT_R))


if __name__ == "__main__":
    main()
