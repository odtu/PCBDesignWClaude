"""Power and GND fan-out before signal routing (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/fanout.py [--dry]

Lab rules 3.2 / 3.3 / 3.5 (and odtu/PowerLabKiCadAssistant issue #21):
- GND is never routed as tracks between pads: the GND pours connect the pads.
  Only decoupling caps (a cap whose other pad is a supply) get their own via to
  the In1 plane right at the GND pad (plane -> via -> cap -> pin, rule 3.2).
  Pads the pour can't reach (boxed in by tracks) get a via after routing, when
  DRC shows them unconnected.
- Fine-pitch IC pins (pitch < 0.8 mm) on Power/GND nets neck down at the pin:
  GND pins next to a GND exposed pad join it with an inward stub; other GND
  pins get a stub to a via just outside the pin row; power pins get a short
  stub out of the pin row, so the router continues at the 0.5 mm class width.
- Fan-out vias keep a 1.5 mm lane in front of every fine-pitch IC pin free,
  so the signal pins can still escape.
- Power pins that sit close together (3+ pads of one rail within 2.5 mm of
  each other) share a local F.Cu copper area instead of tracks, when no pad
  of another net lies inside it.
The new copper is locked so Freerouting keeps it. Run on a board without
tracks (autoroute.strip_tracks).
"""
import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb")
CLR = 0.2                        # Power/GND/Default clearance
VIA_PWR = (0.8, 0.4)             # lab power/GND via (pad, drill)
VIA_FINE = (0.6, 0.3)            # fan-out via at a fine-pitch pin (the part forces it)
TRACK = 0.5                      # Power/GND class width
NECK = 0.25                      # neck-down width at fine-pitch pins
FINE_PITCH = 0.8
ESCAPE = 1.5                     # free lane in front of fine-pitch pins (no fan-out vias)
CLUSTER_GAP = 2.5                # pad centre distance for a "power pin group"
CX, CY, R_EDGE, R_HOLE = 100.0, 100.0, 37.35, 8.6
mm, MM = pcbnew.ToMM, pcbnew.FromMM


def V(x, y):
    return pcbnew.VECTOR2I(MM(x), MM(y))


def is_power(net):
    return net.startswith(("+", "-", "VBUS"))


def rect(pad):
    b = pad.GetBoundingBox()
    return (mm(b.GetLeft()), mm(b.GetTop()), mm(b.GetRight()), mm(b.GetBottom()))


def rect_dist(x, y, r):
    return math.hypot(max(r[0] - x, 0, x - r[2]), max(r[1] - y, 0, y - r[3]))


def seg_dist_pt(px, py, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L = vx * vx + vy * vy
    t = 0 if L == 0 else max(0, min(1, ((px - a[0]) * vx + (py - a[1]) * vy) / L))
    return math.hypot(px - a[0] - t * vx, py - a[1] - t * vy)


def seg_rect_dist(a, b, r):
    # sample: segments here are short (< 3 mm)
    n = max(2, int(math.dist(a, b) / 0.05))
    return min(rect_dist(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, r) for k in range(n + 1))


def seg_seg_dist(a, b, c, d):
    return min(seg_dist_pt(*a, c, d), seg_dist_pt(*b, c, d), seg_dist_pt(*c, a, b), seg_dist_pt(*d, a, b))


class World:
    """Copper on F.Cu (and through-hole objects) for clearance checks."""

    def __init__(self, board):
        self.board = board
        self.pads = []         # (rect, net, pad, through)
        for fp in board.GetFootprints():
            for p in fp.Pads():
                th = p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
                if p.IsOnLayer(pcbnew.F_Cu) or th:
                    self.pads.append((rect(p), p.GetNetname(), p, th))
        self.keep = []          # (x, y, r) circles: fiducial keepouts, mounting holes
        for fp in board.GetFootprints():
            ref = fp.GetReference()
            if ref.startswith("FID"):
                self.keep.append((mm(fp.GetPosition().x), mm(fp.GetPosition().y), 1.5))
            if ref.startswith("H"):
                self.keep.append((mm(fp.GetPosition().x), mm(fp.GetPosition().y), 3.5))
        self.segs = []          # (a, b, half width, net) on F.Cu
        self.vias = []          # (x, y, r, net)
        # escape lanes: 1.5 mm in front of every fine-pitch IC pin stay free of
        # fan-out vias, so each signal pin can still leave its IC (and drop a via)
        self.lanes = []         # (rect, net)
        for fp in board.GetFootprints():
            if not fp.GetReference().startswith("U"):
                continue
            for p in fp.Pads():
                if p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD or pin_pitch(fp, p) >= FINE_PITCH:
                    continue
                (ux, uy), pr = outward(fp, p)
                if ux > 0:
                    lane = (pr[2], pr[1], pr[2] + ESCAPE, pr[3])
                elif ux < 0:
                    lane = (pr[0] - ESCAPE, pr[1], pr[0], pr[3])
                elif uy > 0:
                    lane = (pr[0], pr[3], pr[2], pr[3] + ESCAPE)
                else:
                    lane = (pr[0], pr[1] - ESCAPE, pr[2], pr[1])
                self.lanes.append((lane, p.GetNetname()))

    def via_ok(self, x, y, r, net, own_pad=None):
        rr = math.hypot(x - CX, y - CY)
        if rr > R_EDGE - 0.5 - r or rr < R_HOLE + 0.5 + r:
            return False
        if any(math.hypot(x - kx, y - ky) < kr + r + CLR for kx, ky, kr in self.keep):
            return False
        if any(n != net and rect_dist(x, y, lane) < r + 0.1 for lane, n in self.lanes):
            return False
        for pr, n, p, th in self.pads:
            # no via in or touching any SMD pad (mask web), other nets keep the clearance
            if rect_dist(x, y, pr) < r + CLR:
                return False
        for a, b, hw, n in self.segs:
            if n != net and seg_dist_pt(x, y, a, b) < r + hw + CLR:
                return False
        for vx, vy, vr, n in self.vias:
            # hole to hole 0.4 mm and the clearance to other nets
            need = (vr + r + CLR) if n != net else (vr + r + 0.25)
            if math.hypot(x - vx, y - vy) < need:
                return False
        return True

    def seg_ok(self, a, b, hw, net, own=()):
        for pr, n, p, th in self.pads:
            if n != net and seg_rect_dist(a, b, pr) < hw + CLR:
                return False
        for c, d, w2, n in self.segs:
            if n != net and seg_seg_dist(a, b, c, d) < hw + w2 + CLR:
                return False
        for vx, vy, vr, n in self.vias:
            if n != net and seg_dist_pt(vx, vy, a, b) < vr + hw + CLR:
                return False
        return True

    def add_track(self, a, b, w, net, layer=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(self.board)
        t.SetStart(V(*a))
        t.SetEnd(V(*b))
        t.SetWidth(MM(w))
        t.SetLayer(layer)
        t.SetNet(self.board.FindNet(net))
        t.SetLocked(True)
        self.board.Add(t)
        self.segs.append((a, b, w / 2, net))

    def add_via(self, x, y, size, net):
        v = pcbnew.PCB_VIA(self.board)
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetPosition(V(x, y))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetWidth(MM(size[0]))
        v.SetDrill(MM(size[1]))
        v.SetNet(self.board.FindNet(net))
        v.SetLocked(True)
        self.board.Add(v)
        self.vias.append((x, y, size[0] / 2, net))


def pin_pitch(fp, pad):
    px, py = mm(pad.GetX()), mm(pad.GetY())
    d = [math.hypot(mm(q.GetX()) - px, mm(q.GetY()) - py) for q in fp.Pads()
         if q.GetNumber() != pad.GetNumber() and q.GetNumber() not in ("", "EP")
         and q.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]
    return min(d) if d else 99


def outward(fp, pad):
    """Unit vector along the pad's long axis, away from the footprint centre."""
    pr = rect(pad)
    px, py = (pr[0] + pr[2]) / 2, (pr[1] + pr[3]) / 2
    pts = [(mm(q.GetX()), mm(q.GetY())) for q in fp.Pads()]
    cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
    if pr[3] - pr[1] > pr[2] - pr[0]:
        return (0.0, 1.0 if py > cy else -1.0), pr
    return (1.0 if px > cx else -1.0, 0.0), pr


def fine_pitch_pins(world):
    """Neck-downs at fine-pitch IC pins of Power/GND nets."""
    done, failed = set(), []
    for fp in world.board.GetFootprints():
        if not fp.GetReference().startswith("U"):
            continue
        ep = [q for q in fp.Pads() if q.GetNetname() == "GND" and q.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
              and rect(q)[2] - rect(q)[0] > 1.5]
        for p in fp.Pads():
            net = p.GetNetname()
            if not (net == "GND" or is_power(net)) or p in ep or p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                continue
            if pin_pitch(fp, p) >= FINE_PITCH:
                continue
            (ux, uy), pr = outward(fp, p)
            cx, cy = (pr[0] + pr[2]) / 2, (pr[1] + pr[3]) / 2
            half_len = (pr[3] - pr[1]) / 2 if uy else (pr[2] - pr[0]) / 2
            w = min(NECK, (pr[2] - pr[0]) if uy else (pr[3] - pr[1]))
            key = (fp.GetReference(), p.GetNumber())
            if net == "GND" and ep:
                # join the exposed GND pad: stub inward across the gap
                e = rect(ep[0])
                ex = min(max(cx, e[0] + 0.1), e[2] - 0.1)
                ey = min(max(cy, e[1] + 0.1), e[3] - 0.1)
                # straight across the gap (a corner pin's stub overlaps the pad's edge,
                # which KiCad connects; an angled stub would pass the neighbour pin)
                if uy:
                    ex = cx
                else:
                    ey = cy
                world.add_track((cx, cy), (ex, ey), w, net)
                done.add(key)
                continue
            toe = (cx + ux * half_len, cy + uy * half_len)
            if net == "GND":
                r = VIA_FINE[0] / 2
                ok = False
                for d in [0.2 + r + k * 0.1 for k in range(0, 15)]:
                    vx, vy = toe[0] + ux * d, toe[1] + uy * d
                    if world.via_ok(vx, vy, r, net) and world.seg_ok((cx, cy), (vx, vy), w / 2, net):
                        world.add_track((cx, cy), (vx, vy), w, net)
                        world.add_via(vx, vy, VIA_FINE, net)
                        ok = True
                        break
                if not ok:
                    # out of the pin row first, then a 45 deg jog to a free via site
                    knee = (toe[0] + ux * 0.45, toe[1] + uy * 0.45)
                    for d in [0.6 + k * 0.1 for k in range(12)]:
                        for side in (1, -1):
                            jx, jy = ux - side * uy, uy + side * ux        # 45 deg off the axis
                            n2 = math.hypot(jx, jy)
                            vx, vy = knee[0] + jx / n2 * d, knee[1] + jy / n2 * d
                            if world.via_ok(vx, vy, r, net) and world.seg_ok((cx, cy), knee, w / 2, net)                                     and world.seg_ok(knee, (vx, vy), w / 2, net):
                                world.add_track((cx, cy), knee, w, net)
                                world.add_track(knee, (vx, vy), w, net)
                                world.add_via(vx, vy, VIA_FINE, net)
                                ok = True
                                break
                        if ok:
                            break
                if not ok:
                    # the decoupling cap's GND pad right at the pin (it has its own via): join it
                    near = sorted((F_dist, q) for q, F_dist in
                                  ((q, rect_dist(cx, cy, r2)) for r2, n2, q, th in world.pads
                                   if n2 == "GND" and q.GetParentFootprint().GetReference() != fp.GetReference()))
                    for dist, q in near[:3]:
                        if dist > 2.0:
                            break
                        qr = rect(q)
                        tx = min(max(cx, qr[0] + 0.1), qr[2] - 0.1)
                        ty = min(max(cy, qr[1] + 0.1), qr[3] - 0.1)
                        if world.seg_ok((cx, cy), (tx, ty), w / 2, net):
                            world.add_track((cx, cy), (tx, ty), w, net)
                            ok = True
                            break
                        # straight out of the pin row first, then into the cap pad
                        knee = (toe[0] + ux * 0.3, toe[1] + uy * 0.3)
                        if world.seg_ok((cx, cy), knee, w / 2, net) and world.seg_ok(knee, (tx, ty), w / 2, net):
                            world.add_track((cx, cy), knee, w, net)
                            world.add_track(knee, (tx, ty), w, net)
                            ok = True
                            break
                (done.add(key) if ok else failed.append(key))
            else:
                end = (toe[0] + ux * 0.5, toe[1] + uy * 0.5)
                if world.seg_ok((cx, cy), end, w / 2, net):
                    world.add_track((cx, cy), end, w, net)
                    done.add(key)
                else:
                    failed.append(key)
    return done, failed


def is_decap(fp):
    """A capacitor between GND and a supply (rail, VCC, VREG, VCORE, REF_NR)."""
    nets = [p.GetNetname() for p in fp.Pads()]
    return fp.GetReference().startswith("C") and "GND" in nets and         any(n.startswith("+") or any(k in n for k in ("VCC", "VREG", "VCORE", "REF_NR")) for n in nets)


def gnd_vias(world, skip):
    """A GND via right at each decoupling cap's GND pad (plane -> via -> cap -> pin).
    Every other GND pad connects through the pour on its own layer."""
    placed, failed = 0, []
    r = VIA_PWR[0] / 2
    for fp in world.board.GetFootprints():
        ref = fp.GetReference()
        if not is_decap(fp):
            continue
        pts = [(mm(q.GetX()), mm(q.GetY())) for q in fp.Pads()]
        fcx, fcy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
        for p in fp.Pads():
            if p.GetNetname() != "GND" or (ref, p.GetNumber()) in skip:
                continue
            if p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD or not p.IsOnLayer(pcbnew.F_Cu):
                continue                     # through-hole GND pins reach every plane already
            pr = rect(p)
            if ref.startswith("U") and pr[2] - pr[0] > 1.5 and pr[3] - pr[1] > 1.5:
                continue                     # IC exposed pads carry their own thermal vias
            cx, cy = (pr[0] + pr[2]) / 2, (pr[1] + pr[3]) / 2
            w = min(TRACK, pr[2] - pr[0], pr[3] - pr[1])
            best = None
            for k in range(72):
                a = math.radians(k * 5)
                ux, uy = math.cos(a), math.sin(a)
                # leave the pad through its edge, via 0.2 mm (mask web) beyond it
                t_exit = min(((pr[2] - cx) / ux if ux > 1e-9 else (pr[0] - cx) / ux if ux < -1e-9 else 1e9),
                             ((pr[3] - cy) / uy if uy > 1e-9 else (pr[1] - cy) / uy if uy < -1e-9 else 1e9))
                for extra in (0.0, 0.15, 0.3, 0.5):
                    d = t_exit + CLR + r + extra
                    vx, vy = cx + ux * d, cy + uy * d
                    # prefer leaving away from the part (not between its own pads)
                    away = (vx - fcx) * ux + (vy - fcy) * uy
                    cost = d - 0.3 * (away > 0)
                    if best and cost >= best[0]:
                        continue
                    start = (cx + ux * max(t_exit - w / 2, 0), cy + uy * max(t_exit - w / 2, 0))
                    if world.via_ok(vx, vy, r, "GND") and world.seg_ok(start, (vx, vy), w / 2, "GND"):
                        best = (cost, vx, vy, start)
                        break
            if best is None:
                failed.append(f"{ref}.{p.GetNumber()}")
                continue
            _, vx, vy, start = best
            world.add_track(start, (vx, vy), w, "GND")
            world.add_via(vx, vy, VIA_PWR, "GND")
            placed += 1
    return placed, failed


def power_groups(world, dry=False):
    """Local F.Cu copper for clusters of power pins of one rail."""
    nets = {}
    for pr, n, p, th in world.pads:
        if is_power(n) and not th and p.IsOnLayer(pcbnew.F_Cu):
            nets.setdefault(n, []).append((pr, p))
    made = []
    for net, items in nets.items():
        # single-linkage clusters
        groups = []
        for it in items:
            c = ((it[0][0] + it[0][2]) / 2, (it[0][1] + it[0][3]) / 2)
            hit = [g for g in groups if any(math.dist(c, ((q[0][0] + q[0][2]) / 2, (q[0][1] + q[0][3]) / 2))
                                            <= CLUSTER_GAP for q in g)]
            merged = [it]
            for g in hit:
                merged += g
                groups.remove(g)
            groups.append(merged)
        for g in groups:
            if len(g) < 3:
                continue
            # skip pads of fine-pitch ICs (they neck down instead)
            g = [it for it in g if not (it[1].GetParentFootprint().GetReference().startswith("U")
                                        and pin_pitch(it[1].GetParentFootprint(), it[1]) < FINE_PITCH)]
            if len(g) < 3:
                continue
            pts = []
            for pr, p in g:
                m = 0.3
                pts += [(pr[0] - m, pr[1] - m), (pr[2] + m, pr[1] - m), (pr[2] + m, pr[3] + m), (pr[0] - m, pr[3] + m)]
            hull = convex_hull(pts)
            # no other-net pad inside (it would be cut off); GND pads are fine,
            # each has its own via to the In1 plane
            inside = [n for pr, n, p, th in world.pads if n not in (net, "GND") and poly_hits_rect(hull, pr, CLR)]
            refs = sorted({p.GetParentFootprint().GetReference() + "." + p.GetNumber() for _, p in g})
            if inside:
                print(f"  {net}: group {refs} not poured (other nets inside: {sorted(set(inside))[:4]})")
                continue
            made.append((net, refs))
            if dry:
                continue
            z = pcbnew.ZONE(world.board)
            z.SetLayer(pcbnew.F_Cu)
            z.SetNet(world.board.FindNet(net))
            z.SetZoneName(f"PWR_{net}_{len(made)}")
            z.SetAssignedPriority(10)
            z.SetLocalClearance(MM(CLR))
            z.SetMinThickness(MM(0.25))
            z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)     # small currents, but no spokes needed
            o = z.Outline()
            o.NewOutline()
            for x, y in hull:
                o.Append(MM(x), MM(y))
            world.board.Add(z)
    return made


def convex_hull(pts):
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def poly_hits_rect(poly, r, grow):
    """Polygon overlaps rect r grown by `grow` (sampled: pads are small)."""
    x0, y0, x1, y1 = r[0] - grow, r[1] - grow, r[2] + grow, r[3] + grow
    for k in range(5):
        for j in range(5):
            x, y = x0 + (x1 - x0) * k / 4, y0 + (y1 - y0) * j / 4
            if point_in_poly(x, y, poly):
                return True
    return False


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def main():
    dry = "--dry" in sys.argv
    if not dry:
        # start from a board without tracks/vias and without our local power zones
        from autoroute import strip_tracks, _items
        strip_tracks(PCB)
        text = open(PCB, encoding="utf-8").read()
        items = _items(text)
        keep = [it for it in items if not (it.startswith("(zone") and '(name "PWR_' in it)]
        head = text[:text.index("(", 1)]
        open(PCB, "w", encoding="utf-8").write(head + "\n\t".join(keep) + "\n)\n")
    board = pcbnew.LoadBoard(PCB)
    world = World(board)
    done, failed = fine_pitch_pins(world)
    print(f"fine-pitch Power/GND pins: {len(done)} necked down, failed: {failed}")
    groups = power_groups(world, dry)
    for net, refs in groups:
        print(f"  local {net} copper: {refs}")
    n, gfail = gnd_vias(world, done)
    print(f"GND pad vias: {n}, no room (pour only): {gfail}")
    if not dry:
        board.Save(PCB)


if __name__ == "__main__":
    main()
