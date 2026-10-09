"""Track clean-up after autorouting (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/track_opt.py <board.kicad_pcb>

Lab rule 3.3 (keep traces short and direct, 45 deg corners, never 90 deg;
odtu/PowerLabKiCadAssistant issue #21). Works on every unlocked run of track
between two anchors (pad, via, T-junction, layer change or locked fan-out),
one net and layer at a time:
1. Shortcut: from each point, jump to the farthest later point of the run that
   a clear octilinear path reaches (one straight segment, or a straight + a
   45 deg segment), so detours and staircase corners disappear.
2. Pad entry: a run ends at the pad centre and, where clearance allows, its
   last segment enters the pad straight along the pad's axis (no track
   grazing a pad edge or corner, no pour slivers next to the pad).
3. Any corner of 90 deg or less left (also between runs of different width)
   is chamfered to two 45 deg corners.
4. Duplicated and dangling segments (unused neck-down stubs) are removed, and
   Power/GND tracks are widened to their class width where clearance allows.
Interior angles below 90 deg are never created. Every new segment keeps the
clearance (0.2 mm, or a pad's own larger clearance) to other-net pads, tracks
and vias on its layer, 0.5 mm to the board edge and the shaft hole, and stays
out of other nets' local power zones.
"""
import math
import sys
from collections import defaultdict

import pcbnew

PCB = sys.argv[1]
CLR = 0.2
CX, CY, R_EDGE, R_HOLE, EDGE_CLR = 100.0, 100.0, 38.0, 8.0, 0.5
CELL = 2.0
mm, MM = pcbnew.ToMM, pcbnew.FromMM
EPS = 1e-4


def P(v):
    return (round(mm(v.x), 4), round(mm(v.y), 4))


def pt_seg(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L = vx * vx + vy * vy
    t = 0 if L == 0 else max(0, min(1, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L))
    return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)


def seg_cross(a, b, c, d):
    def o(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    d1, d2, d3, d4 = o(c, d, a), o(c, d, b), o(a, b, c), o(a, b, d)
    return (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0)


def seg_seg(a, b, c, d):
    if seg_cross(a, b, c, d):
        return 0.0
    return min(pt_seg(a, c, d), pt_seg(b, c, d), pt_seg(c, a, b), pt_seg(d, a, b))


def pt_rect(p, r):
    return math.hypot(max(r[0] - p[0], 0, p[0] - r[2]), max(r[1] - p[1], 0, p[1] - r[3]))


def seg_rect(a, b, r):
    corners = [(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]
    for k in range(4):
        if seg_cross(a, b, corners[k], corners[(k + 1) % 4]):
            return 0.0
    if r[0] <= a[0] <= r[2] and r[1] <= a[1] <= r[3]:
        return 0.0
    return min(pt_rect(a, r), pt_rect(b, r), *(pt_seg(c, a, b) for c in corners))


def in_poly(p, poly):
    x, y = p
    inside = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


class Obstacles:
    """Copper per layer in a bucket grid; tracks can be removed and added."""

    def __init__(self, board):
        self.board = board
        self.cu = [board.GetLayerID(n) for n in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu")]
        self.pads = defaultdict(list)          # layer -> [(rect, net, clr)]
        self.vias = []                         # (p, r, net)
        self.tracks = {}                       # id -> (a, b, hw, layer, net)
        self.bucket = defaultdict(set)         # (layer, i, j) -> track ids
        self.zones = []                        # (layer, net, polygon)
        for fp in board.GetFootprints():
            for p in fp.Pads():
                b = p.GetBoundingBox()
                r = (mm(b.GetLeft()), mm(b.GetTop()), mm(b.GetRight()), mm(b.GetBottom()))
                clr = max(CLR, mm(p.GetLocalClearance() or 0))
                if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                    clr = max(clr, 0.25)
                for l in self.cu:
                    if p.IsOnLayer(l) or p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                        self.pads[l].append((r, p.GetNetname(), clr, p))
        self.keepouts = []                     # (layer set, outer polygon, hole polygons) of rule areas
        for z in board.Zones():
            if z.GetIsRuleArea():
                if z.GetDoNotAllowTracks():
                    ol = z.Outline()
                    outer = ol.Outline(0)
                    poly = [(mm(outer.CPoint(i).x), mm(outer.CPoint(i).y)) for i in range(outer.PointCount())]
                    holes = []
                    for h in range(ol.HoleCount(0)):
                        hc = ol.Hole(0, h)
                        holes.append([(mm(hc.CPoint(i).x), mm(hc.CPoint(i).y)) for i in range(hc.PointCount())])
                    lays = {l for l in self.cu if z.IsOnLayer(l)}
                    self.keepouts.append((lays, poly, holes))
                continue
            if not z.GetZoneName().startswith("PWR_"):
                continue
            o = z.Outline().Outline(0)
            poly = [(mm(o.CPoint(i).x), mm(o.CPoint(i).y)) for i in range(o.PointCount())]
            self.zones.append((z.GetLayer(), z.GetNetname(), poly))
        self.next_id = 0

    def add_via(self, v):
        self.vias.append((P(v.GetPosition()), mm(v.GetWidth(pcbnew.F_Cu)) / 2, v.GetNetname()))

    def add_track(self, a, b, hw, layer, net):
        tid = self.next_id
        self.next_id += 1
        self.tracks[tid] = (a, b, hw, layer, net)
        for key in self._cells(a, b, hw, layer):
            self.bucket[key].add(tid)
        return tid

    def remove_track(self, tid):
        a, b, hw, layer, net = self.tracks.pop(tid)
        for key in self._cells(a, b, hw, layer):
            self.bucket[key].discard(tid)

    def _cells(self, a, b, hw, layer):
        m = hw + CLR + 1.0
        i0, i1 = int((min(a[0], b[0]) - m) // CELL), int((max(a[0], b[0]) + m) // CELL)
        j0, j1 = int((min(a[1], b[1]) - m) // CELL), int((max(a[1], b[1]) + m) // CELL)
        return [(layer, i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)]

    def clear(self, a, b, hw, layer, net, ignore=()):
        # board edge and shaft hole
        for p in (a, b):
            if math.hypot(p[0] - CX, p[1] - CY) > R_EDGE - EDGE_CLR - hw:
                return False
        if pt_seg((CX, CY), a, b) < R_HOLE + EDGE_CLR + hw + 0.15:     # HOLE_KEEPOUT is r 8.6
            return False
        n = max(2, int(math.dist(a, b) / 0.1))
        samples = [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)]
        for lays, poly, holes in self.keepouts:
            if layer in lays and any(in_poly(q, poly) and not any(in_poly(q, h) for h in holes) for q in samples):
                return False
        for r, n, clr, _ in self.pads[layer]:
            if n != net and seg_rect(a, b, r) < hw + clr - EPS:
                return False
        for p, vr, n in self.vias:
            if n != net and pt_seg(p, a, b) < vr + hw + CLR - EPS:
                return False
        seen = set()
        for key in self._cells(a, b, hw, layer):
            for tid in self.bucket.get(key, ()):
                if tid in seen or tid in ignore:
                    continue
                seen.add(tid)
                c, d, hw2, l2, n2 = self.tracks[tid]
                if n2 != net and seg_seg(a, b, c, d) < hw + hw2 + CLR - EPS:
                    return False
        for zl, zn, poly in self.zones:
            if zl == layer and zn != net:
                n = max(2, int(math.dist(a, b) / 0.2))
                if any(in_poly((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n), poly)
                       for k in range(n + 1)):
                    return False
        return True


def octi_paths(a, b):
    """Octilinear ways from a to b: direct when 0/45/90 deg, else straight+diagonal both ways."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    adx, ady = abs(dx), abs(dy)
    if adx < EPS or ady < EPS or abs(adx - ady) < EPS:
        return [[a, b]]
    sx, sy = math.copysign(1, dx), math.copysign(1, dy)
    m = min(adx, ady)
    k1 = (a[0] + sx * m, a[1] + sy * m)                 # diagonal first
    k2 = (b[0] - sx * m, b[1] - sy * m)                 # straight first
    return [[a, k1, b], [a, k2, b]]


def turn_ok(p0, p1, p2):
    """Interior angle at p1 at least 90 deg (no acute corners)."""
    v1 = (p0[0] - p1[0], p0[1] - p1[1])
    v2 = (p2[0] - p1[0], p2[1] - p1[1])
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    if l1 < EPS or l2 < EPS:
        return True
    return (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2) <= 1e-6


def length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def pad_point(padref, toward, hw):
    """The pad centre: a track enters a pad straight and ends at its centre (lab
    rule 3.3, review of issue #21). Ending on the pad edge is connected, but it
    leaves odd copper and pour slivers next to the pad."""
    rect, pad = padref
    c = pad.GetPosition()
    return (round(mm(c.x), 4), round(mm(c.y), 4))


def into_pad_straight(a, b, rect):
    """Does the last segment a->b run along the pad's axis (square to its edge)?"""
    dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
    wide = rect[2] - rect[0] >= rect[3] - rect[1]
    return dx < EPS or dy < EPS if abs((rect[2] - rect[0]) - (rect[3] - rect[1])) < EPS else         (dy < EPS if wide else dx < EPS)


def snap45(a, b):
    """Is a-b already 0/45/90 deg?"""
    dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
    return dx < EPS or dy < EPS or abs(dx - dy) < 1e-3


def optimise_chain(obs, pts, hw, layer, net, ids, start_pad, end_pad):
    """Return a new point list for the run (or None if nothing better)."""
    ign = set(ids)
    clear = lambda a, b: obs.clear(a, b, hw, layer, net, ign)   # noqa: E731
    out = [pts[0]]
    i = 0
    n = len(pts) - 1
    while i < n:
        cur = out[-1]
        done = False
        for k in range(n, i, -1):
            targets = [pts[k]]
            if k == n and end_pad is not None:
                targets.insert(0, pad_point(end_pad, cur, hw))
            starts = [cur]
            if len(out) == 1 and start_pad is not None:
                starts.insert(0, pad_point(start_pad, pts[k], hw))
            best = None
            for s in starts:
                for t in targets:
                    for path in octi_paths(s, t):
                        cand = (out[:-1] + [s] if len(out) == 1 else out) + path[1:]
                        if len(cand) >= 3 and not all(turn_ok(cand[j - 1], cand[j], cand[j + 1])
                                                      for j in range(max(1, len(out) - 1), len(cand) - 1)):
                            continue
                        segs = list(zip(path, path[1:]))
                        if all(clear(a, b) for a, b in segs if math.dist(a, b) > EPS):
                            L = length(path)
                            # prefer finishing straight into the end pad (square to its edge)
                            if k == n and end_pad is not None and not into_pad_straight(path[-2], path[-1], end_pad[0]):
                                L += 0.3
                            if len(out) == 1 and start_pad is not None and not into_pad_straight(path[1], path[0], start_pad[0]):
                                L += 0.3
                            if best is None or L < best[0]:
                                best = (L, s, path)
            if best:
                if len(out) == 1:
                    out[0] = best[1]
                out += best[2][1:]
                i = k
                done = True
                break
        if not done:
            # keep the original segment (it was legal)
            out.append(pts[i + 1])
            i += 1
    out = chamfer(obs, drop_jogs(obs, dedupe(out), hw, layer, net, ign), hw, layer, net, ign)
    return out


def drop_jogs(obs, pts, hw, layer, net, ign):
    """Remove interior points closer than 0.1 mm to a neighbour (tiny jogs left by
    pad entry), when the merged segment keeps clearance."""
    k = 1
    while k < len(pts) - 1:
        if min(math.dist(pts[k], pts[k - 1]), math.dist(pts[k], pts[k + 1])) < 0.1 and                 obs.clear(pts[k - 1], pts[k + 1], hw, layer, net, ign):
            pts.pop(k)
        else:
            k += 1
    return pts


def dedupe(pts):
    res = [pts[0]]
    for p in pts[1:]:
        if math.dist(p, res[-1]) > EPS:
            res.append(p)
    # drop collinear points
    k = 1
    while k < len(res) - 1:
        a, b, c = res[k - 1], res[k], res[k + 1]
        if abs((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])) < 1e-6 and \
                (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]) > 0:
            res.pop(k)
        else:
            k += 1
    return res


def chamfer(obs, pts, hw, layer, net, ign):
    k = 1
    while k < len(pts) - 1:
        a, b, c = pts[k - 1], pts[k], pts[k + 1]
        v1 = (a[0] - b[0], a[1] - b[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        l1, l2 = math.hypot(*v1), math.hypot(*v2)
        if l1 > EPS and l2 > EPS and abs(v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2) < 1e-3:
            d = min(0.5, l1 / 2, l2 / 2)
            p1 = (b[0] + v1[0] / l1 * d, b[1] + v1[1] / l1 * d)
            p2 = (b[0] + v2[0] / l2 * d, b[1] + v2[1] / l2 * d)
            if obs.clear(p1, p2, hw, layer, net, ign):
                pts[k:k + 1] = [p1, p2]
                k += 2
                continue
        k += 1
    return pts


def dangling(board):
    """Rule 3.3: no stubs and no dangling traces. UUIDs of every track with an end
    that touches no pad, via or other (live) track of its net on its layer, found
    repeatedly (e.g. a neck-down stub the router didn't use)."""
    pads = defaultdict(list)
    for fp in board.GetFootprints():
        for p in fp.Pads():
            b = p.GetBoundingBox()
            lay = [l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if p.IsOnLayer(l)]
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                lay += [board.GetLayerID("In1.Cu"), board.GetLayerID("In2.Cu")]
            pads[p.GetNetname()].append(((mm(b.GetLeft()), mm(b.GetTop()), mm(b.GetRight()), mm(b.GetBottom())), lay))
    recs, vias = [], defaultdict(list)
    for t in board.GetTracks():          # read everything inside the iteration (SWIG)
        if t.Type() == pcbnew.PCB_VIA_T:
            vias[t.GetNetname()].append((P(t.GetPosition()), mm(t.GetWidth(pcbnew.F_Cu)) / 2))
        elif t.Type() == pcbnew.PCB_TRACE_T:
            recs.append((t.m_Uuid.AsString(), P(t.GetStart()), P(t.GetEnd()), t.GetNetname(), t.GetLayer(),
                         mm(t.GetWidth()) / 2))
    by = defaultdict(list)
    for k, r in enumerate(recs):
        by[(r[3], r[4])].append(k)
    dead = set()
    while True:
        new = set()
        for k, (uid, a, b, net, layer, hw) in enumerate(recs):
            if k in dead:
                continue
            for end in (a, b):
                # a track end within its half width of the pad overlaps it (KiCad connects it)
                ok = any(r[0] - hw <= end[0] <= r[2] + hw and r[1] - hw <= end[1] <= r[3] + hw and layer in lay
                         for r, lay in pads[net])
                ok = ok or any(math.dist(end, vp) <= vr for vp, vr in vias[net])
                # KiCad joins track ends only to track ends (not to the middle of a track)
                ok = ok or any(j != k and j not in dead and min(math.dist(end, recs[j][1]), math.dist(end, recs[j][2]))
                               <= max(hw, recs[j][5]) for j in by[(net, layer)])
                if not ok:
                    new.add(k)
                    break
        if not new:
            return {recs[k][0] for k in dead}
        dead |= new


def drop_items(path, uuids):
    """Remove top-level board items by UUID (text edit; SWIG Remove() breaks later lookups)."""
    import os
    import re
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from coilgen import _top_level_items
    text = open(path, encoding="utf-8").read()
    items = _top_level_items(text)
    keep = []
    for it in items:
        m = re.search(r'\(uuid "([0-9a-f-]{36})"\)', it) if it.startswith(("(segment", "(via", "(arc")) else None
        if not (m and m.group(1) in uuids):
            keep.append(it)
    head = text[:text.index("(", 1)]
    open(path, "w", encoding="utf-8").write(head + "\n\t".join(keep) + "\n)\n")
    return len(items) - len(keep)


def main():
    board = pcbnew.LoadBoard(PCB)
    dup = duplicates(board)
    if dup:
        drop_items(PCB, dup)
        print(f"duplicated track segments removed: {len(dup)}")
        board = pcbnew.LoadBoard(PCB)
    kill = dangling(board)
    if kill:
        print(f"dangling track segments: {len(kill)}")
    obs = Obstacles(board)
    items, uid = {}, {}
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            obs.add_via(t)
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_TRACE_T and t.m_Uuid.AsString() not in kill:
            a, b = P(t.GetStart()), P(t.GetEnd())
            tid = obs.add_track(a, b, mm(t.GetWidth()) / 2, t.GetLayer(), t.GetNetname())
            items[tid] = (t.GetNetname(), t.GetLayer(), round(mm(t.GetWidth()), 4), t.IsLocked())
            uid[tid] = t.m_Uuid.AsString()
    via_pts = {p for p, r, n in obs.vias}
    before_len = sum(math.dist(obs.tracks[i][0], obs.tracks[i][1]) for i in items)
    by = defaultdict(list)                 # (net, layer, width) -> track ids
    locked_pts = set()
    for tid, (net, layer, w, locked) in items.items():
        if locked:
            locked_pts |= {obs.tracks[tid][0], obs.tracks[tid][1]}
        else:
            by[(net, layer, w)].append(tid)
    changed = 0
    for (net, layer, w), tids in sorted(by.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        # degree of every point over ALL copper of this net and layer
        deg = defaultdict(int)
        for tid, (a, b, hw, l, n) in obs.tracks.items():
            if n == net and l == layer:
                deg[a] += 1
                deg[b] += 1
        pads = [(r, p) for r, n, clr, p in obs.pads[layer] if n == net]

        def pad_at(p):
            for r, pad in pads:
                if r[0] - EPS <= p[0] <= r[2] + EPS and r[1] - EPS <= p[1] <= r[3] + EPS \
                        and pad.HitTest(pcbnew.VECTOR2I(MM(p[0]), MM(p[1]))):
                    return (r, pad)
            return None

        def anchor(p):
            return deg[p] != 2 or p in via_pts or p in locked_pts or pad_at(p) is not None

        adj = defaultdict(list)
        for tid in tids:
            a, b = obs.tracks[tid][0], obs.tracks[tid][1]
            adj[a].append((tid, b))
            adj[b].append((tid, a))
        used = set()
        for tid in tids:
            if tid in used:
                continue
            # grow the run in both directions until anchors
            a, b = obs.tracks[tid][0], obs.tracks[tid][1]
            chain_ids, pts = [tid], [a, b]
            used.add(tid)
            for direction in (0, 1):
                while True:
                    end = pts[-1] if direction == 0 else pts[0]
                    if anchor(end):
                        break
                    nxt = [(t2, q) for t2, q in adj[end] if t2 not in used]
                    if len(nxt) != 1:
                        break
                    t2, q = nxt[0]
                    used.add(t2)
                    chain_ids.append(t2)
                    if direction == 0:
                        pts.append(q)
                    else:
                        pts.insert(0, q)
            old_len = length(pts)
            new = optimise_chain(obs, pts, w / 2, layer, net, chain_ids, pad_at(pts[0]), pad_at(pts[-1]))
            if new is None:
                continue
            if length(new) > old_len - 0.01 and len(new) >= len(pts) and \
                    all(snap45(x, y) for x, y in zip(pts, pts[1:])):
                continue
            # replace the run: new tracks now, the old ones leave by UUID after saving
            for t2 in chain_ids:
                obs.remove_track(t2)
                items.pop(t2)
                kill.add(uid.pop(t2))
            for x, y in zip(new, new[1:]):
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(pcbnew.VECTOR2I(MM(x[0]), MM(x[1])))
                t.SetEnd(pcbnew.VECTOR2I(MM(y[0]), MM(y[1])))
                t.SetWidth(MM(w))
                t.SetLayer(layer)
                t.SetNet(board.FindNet(net))
                board.Add(t)
                items[obs.add_track(x, y, w / 2, layer, net)] = (net, layer, w, False)
            changed += 1
    after_len = sum(math.dist(obs.tracks[i][0], obs.tracks[i][1]) for i in items)
    board.Save(PCB)
    n = drop_items(PCB, kill)
    print(f"runs rewritten: {changed} ({n} old segments removed); track length {before_len:.0f} -> {after_len:.0f} mm")


def duplicates(board):
    """UUIDs of track segments that repeat another one (same net, layer and ends
    within 1 um; KiCad's SES import shifts copies by a few nm)."""
    seen, dup = {}, set()
    for t in board.GetTracks():
        if t.Type() != pcbnew.PCB_TRACE_T:
            continue
        a, b = sorted([(round(mm(t.GetStart().x), 3), round(mm(t.GetStart().y), 3)),
                       (round(mm(t.GetEnd().x), 3), round(mm(t.GetEnd().y), 3))])
        k = (t.GetNetname(), t.GetLayer(), a, b)
        uid = t.m_Uuid.AsString()
        if k in seen:
            # keep the locked copy
            if t.IsLocked() and not seen[k][1]:
                dup.add(seen[k][0])
                seen[k] = (uid, True)
            else:
                dup.add(uid)
        else:
            seen[k] = (uid, t.IsLocked())
    return dup


def corners():
    """Every corner of 90 deg or less (also where the width changes, between runs)
    gets a 45 deg chamfer: both segments are shortened and joined by a diagonal."""
    board = pcbnew.LoadBoard(PCB)
    obs = Obstacles(board)
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            obs.add_via(t)
    recs = {}
    ends = defaultdict(list)
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_TRACE_T:
            a, b = P(t.GetStart()), P(t.GetEnd())
            tid = obs.add_track(a, b, mm(t.GetWidth()) / 2, t.GetLayer(), t.GetNetname())
            recs[tid] = [t, a, b]
            ends[(t.GetNetname(), t.GetLayer(), a)].append((tid, 0))
            ends[(t.GetNetname(), t.GetLayer(), b)].append((tid, 1))
    via_pts = {p for p, r, n in obs.vias}
    done = 0
    for (net, layer, p), lst in list(ends.items()):
        if len(lst) != 2 or p in via_pts:
            continue
        if any(r[0] - EPS <= p[0] <= r[2] + EPS and r[1] - EPS <= p[1] <= r[3] + EPS
               for r, n, clr, pad in obs.pads[layer] if n == net):
            continue
        (t1, e1), (t2, e2) = lst
        if t1 == t2 or recs[t1][0].IsLocked() and recs[t2][0].IsLocked():
            continue
        q1 = recs[t1][2 - e1] if e1 == 0 else recs[t1][1]      # far ends
        q1 = recs[t1][2] if e1 == 0 else recs[t1][1]
        q2 = recs[t2][2] if e2 == 0 else recs[t2][1]
        v1, v2 = (q1[0] - p[0], q1[1] - p[1]), (q2[0] - p[0], q2[1] - p[1])
        l1, l2 = math.hypot(*v1), math.hypot(*v2)
        if l1 < EPS or l2 < EPS:
            continue
        cos = (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)
        if cos < -0.02:                       # interior angle above ~91 deg: fine
            continue
        w = min(mm(recs[t1][0].GetWidth()), mm(recs[t2][0].GetWidth()))
        for d in (0.5, 0.35, 0.2, 0.1, 0.05):
            d = min(d, 0.45 * l1, 0.45 * l2)
            a1 = (round(p[0] + v1[0] / l1 * d, 4), round(p[1] + v1[1] / l1 * d, 4))
            a2 = (round(p[0] + v2[0] / l2 * d, 4), round(p[1] + v2[1] / l2 * d, 4))
            if obs.clear(a1, a2, w / 2, layer, net, {t1, t2}):
                for tid, e, new in ((t1, e1, a1), (t2, e2, a2)):
                    t = recs[tid][0]
                    if t.IsLocked():
                        break
                (recs[t1][0].SetStart if e1 == 0 else recs[t1][0].SetEnd)(pcbnew.VECTOR2I(MM(a1[0]), MM(a1[1])))
                (recs[t2][0].SetStart if e2 == 0 else recs[t2][0].SetEnd)(pcbnew.VECTOR2I(MM(a2[0]), MM(a2[1])))
                recs[t1][1 + e1] = a1
                recs[t2][1 + e2] = a2
                n = pcbnew.PCB_TRACK(board)
                n.SetStart(pcbnew.VECTOR2I(MM(a1[0]), MM(a1[1])))
                n.SetEnd(pcbnew.VECTOR2I(MM(a2[0]), MM(a2[1])))
                n.SetWidth(MM(w))
                n.SetLayer(layer)
                n.SetNet(board.FindNet(net))
                board.Add(n)
                obs.add_track(a1, a2, w / 2, layer, net)
                done += 1
                break
    board.Save(PCB)
    print(f"corners of 90 deg or less chamfered: {done}")


def is_power(net):
    return net.startswith(("+", "-", "VBUS")) or "GND" in net


def widen():
    """Power/GND tracks to their 0.5 mm class width (Freerouting routes them at the
    default width): each segment takes the widest of 0.5/0.4/0.3/0.25 mm that keeps
    clearance, so it only necks down where a part forces it (lab rule 2.6).
    Stubs left unused by the shortcuts (e.g. a neck-down the new path bypasses)
    are removed first."""
    board = pcbnew.LoadBoard(PCB)
    dead = dangling(board)
    if dead:
        drop_items(PCB, dead)
        print(f"removed {len(dead)} stubs left unused")
        subprocess.run([sys.executable, __file__, PCB, "--widen"], check=True)
        return
    obs = Obstacles(board)
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            obs.add_via(t)
    recs = []
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_TRACE_T:
            a, b = P(t.GetStart()), P(t.GetEnd())
            tid = obs.add_track(a, b, mm(t.GetWidth()) / 2, t.GetLayer(), t.GetNetname())
            recs.append((t, tid, a, b))
    hist = defaultdict(int)
    narrow = 0
    for t, tid, a, b in recs:
        net = t.GetNetname()
        if not is_power(net) and mm(t.GetWidth()) < 0.15 - EPS:
            # Freerouting necks down to 0.10-0.12 mm at fine-pitch pins: PCBWay's floor is 0.15 mm
            if obs.clear(a, b, 0.075, t.GetLayer(), net, {tid}):
                obs.remove_track(tid)
                obs.add_track(a, b, 0.075, t.GetLayer(), net)
                t.SetWidth(MM(0.15))
            else:
                narrow += 1
            continue
        if not is_power(net) or t.IsLocked():
            continue
        w0 = mm(t.GetWidth())
        for w in (0.5, 0.4, 0.3, 0.25):
            if w <= w0 + EPS:
                break
            if obs.clear(a, b, w / 2, t.GetLayer(), net, {tid}):
                obs.remove_track(tid)
                obs.add_track(a, b, w / 2, t.GetLayer(), net)
                t.SetWidth(MM(w))
                hist[w] += 1
                break
        else:
            hist[w0] += 0
    board.Save(PCB)
    print("power/GND segments widened:", dict(sorted(hist.items())),
          f"; still below 0.15 mm: {narrow}" if narrow else "")


import subprocess  # noqa: E402

if __name__ == "__main__":
    if "--widen" in sys.argv:
        widen()
    elif "--corners" in sys.argv:
        corners()
    else:
        main()
        # each pass in a fresh process (SWIG state after the text edits)
        subprocess.run([sys.executable, __file__, PCB, "--corners"], check=True)
        subprocess.run([sys.executable, __file__, PCB, "--widen"], check=True)
