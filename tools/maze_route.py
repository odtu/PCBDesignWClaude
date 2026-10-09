"""Route a few leftover connections with a grid A* (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/maze_route.py <board> NET:REF.PAD[@width] ...
        [reserve=REF.PAD,...] [--no-lanes]

--no-lanes drops the general escape lanes and IC-body blocking (for the last
few connections, when most pins have escaped); reserve= keeps the escape of
specific still-unrouted pins free instead.

Each argument routes from that pad to the nearest copper item of the same
net that is not yet connected to the pad. Obstacles are every pad, track and via of other nets, inflated by their
size plus clearance; GND pours are ignored (they flow around the new tracks
when refilled). Layers: F.Cu, In2.Cu, B.Cu (In1.Cu stays a solid GND plane).
Net-class sizes: Power/GND nets (+*, -*, VBUS*, *GND*) 0.5 mm tracks and
0.4/0.8 mm vias, other nets 0.2 mm and 0.3/0.6 mm. 0.1 mm grid.
"""
import heapq
import math
import sys

import pcbnew

G = 0.1                     # grid (mm)
W = 0.2                     # new track width
CLR = 0.2                   # clearance
VIA_R = 0.3                 # via pad radius
LAYERS = ["F.Cu", "In2.Cu", "B.Cu"]
VIA_COST = 30               # in grid steps
CX, CY, R_EDGE, R_HOLE = 100.0, 100.0, 37.4, 8.6


def mm(v):
    return pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)


class Grid:
    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0 = x0, y0
        self.nx, self.ny = int((x1 - x0) / G) + 1, int((y1 - y0) / G) + 1
        self.block = [bytearray(self.nx * self.ny) for _ in LAYERS]    # track centre forbidden
        self.vblock = bytearray(self.nx * self.ny)                     # via centre forbidden

    def idx(self, i, j):
        return j * self.nx + i

    def cell(self, x, y):
        return int(round((x - self.x0) / G)), int(round((y - self.y0) / G))

    def xy(self, i, j):
        return self.x0 + i * G, self.y0 + j * G

    def mark_rect(self, arrs, x0, y0, x1, y1, r):
        """Mark cells within r of the rectangle (pads are rectangles: a circle model
        misses the corners)."""
        lo_i, lo_j = self.cell(x0 - r, y0 - r)
        hi_i, hi_j = self.cell(x1 + r, y1 + r)
        for j in range(max(lo_j, 0), min(hi_j, self.ny - 1) + 1):
            for i in range(max(lo_i, 0), min(hi_i, self.nx - 1) + 1):
                x, y = self.xy(i, j)
                dx = max(x0 - x, 0.0, x - x1)
                dy = max(y0 - y, 0.0, y - y1)
                if dx * dx + dy * dy <= r * r:
                    k = self.idx(i, j)
                    for arr in arrs:
                        arr[k] = 1

    def mark_seg(self, arrs, a, b, r):
        """Mark cells within r of segment a-b in the given bytearrays."""
        (ax, ay), (bx, by) = a, b
        lo_i, lo_j = self.cell(min(ax, bx) - r, min(ay, by) - r)
        hi_i, hi_j = self.cell(max(ax, bx) + r, max(ay, by) + r)
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        for j in range(max(lo_j, 0), min(hi_j, self.ny - 1) + 1):
            for i in range(max(lo_i, 0), min(hi_i, self.nx - 1) + 1):
                x, y = self.xy(i, j)
                t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
                if math.hypot(x - ax - t * dx, y - ay - t * dy) <= r:
                    k = self.idx(i, j)
                    for arr in arrs:
                        arr[k] = 1


LANES = "--no-lanes" not in sys.argv


def build(board, net, anchor=None):
    x0, y0 = CX - 38.5, CY - 38.5
    if anchor:      # align the grid to the start pad so the narrow escape between fine-pitch pins is on grid
        x0 = anchor[0] - round((anchor[0] - x0) / G) * G
        y0 = anchor[1] - round((anchor[1] - y0) / G) * G
    grid = Grid(x0, y0, x0 + 77.0, y0 + 77.0)
    lid = {board.GetLayerID(n): k for k, n in enumerate(LAYERS)}
    tr = W / 2 + CLR                       # keep-off for a track centre
    vr = VIA_R + CLR                       # keep-off for a via centre
    for t in board.GetTracks():
        if t.GetNetname() == net:
            continue
        if t.Type() == pcbnew.PCB_VIA_T:
            p = mm(t.GetPosition())
            r = pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)) / 2
            grid.mark_seg(grid.block, p, p, r + tr)
            grid.mark_seg([grid.vblock], p, p, r + vr)
        else:
            k = lid.get(t.GetLayer())
            w = pcbnew.ToMM(t.GetWidth()) / 2
            a, b = mm(t.GetStart()), mm(t.GetEnd())
            if k is not None:
                grid.mark_seg([grid.block[k]], a, b, w + tr)
            grid.mark_seg([grid.vblock], a, b, w + vr)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname() == net:
                continue
            bb = pad.GetBoundingBox()
            x0, y0 = pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop())
            x1, y1 = pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())
            c = ((x0 + x1) / 2, (y0 + y1) / 2)
            half = max(x1 - x0, y1 - y0) / 2
            layers = [k for l, k in lid.items() if pad.IsOnLayer(l)]
            extra = max(0.0, pcbnew.ToMM(pad.GetLocalClearance() or 0) - CLR)    # fiducial keepouts
            grid.mark_rect([grid.block[k] for k in layers], x0, y0, x1, y1, tr + extra)
            grid.mark_rect([grid.vblock], x0, y0, x1, y1, vr + extra)
            if pad.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                grid.mark_seg(grid.block, c, c, half + tr)
    # escape lanes: 1.2 mm in front of every fine-pitch IC pin of another net stay free of
    # tracks on all layers (no track along a pin row blocks its neighbours or their escape
    # vias; no-connect pins need no lane)
    for fp in board.GetFootprints():
        if not fp.GetReference().startswith("U") or not LANES:
            continue
        pads = [p for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]
        # no track of another net under a fine-pitch IC (between its pin rows): the
        # pins need that room to escape inwards
        body = [pcbnew.ToMM(v) for v in (min(p.GetBoundingBox().GetLeft() for p in pads),
                                          min(p.GetBoundingBox().GetTop() for p in pads),
                                          max(p.GetBoundingBox().GetRight() for p in pads),
                                          max(p.GetBoundingBox().GetBottom() for p in pads))]
        if not any(p.GetNetname() == net for p in pads):
            side = [k for l, k in lid.items() if pads[0].IsOnLayer(l)]
            grid.mark_rect([grid.block[k] for k in side], *body, W / 2 + CLR)
        cxs = [pcbnew.ToMM(p.GetX()) for p in pads]
        cys = [pcbnew.ToMM(p.GetY()) for p in pads]
        fcx, fcy = sum(cxs) / len(cxs), sum(cys) / len(cys)
        for p in pads:
            n = p.GetNetname()
            if n == net or n.startswith("unconnected-") or n == "":
                continue
            px, py = mm(p.GetPosition())
            pitch = min((math.hypot(pcbnew.ToMM(q.GetX()) - px, pcbnew.ToMM(q.GetY()) - py)
                         for q in pads if q.GetNumber() != p.GetNumber()), default=9)
            bb = p.GetBoundingBox()
            x0, y0, x1, y1 = (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                              pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))
            if pitch >= 0.8 or (x1 - x0 > 1.5 and y1 - y0 > 1.5):
                continue
            L = 1.2
            if y1 - y0 > x1 - x0:          # pad long axis vertical: lane above or below
                lane = (x0, y1, x1, y1 + L) if py > fcy else (x0, y0 - L, x1, y0)
            else:
                lane = (x1, y0, x1 + L, y1) if px > fcx else (x0 - L, y0, x0, y1)
            # tracks of other nets stay out of the lane on every layer (the pin escapes and
            # drops its via here); vias only keep their clearance, so escape vias can stagger
            grid.mark_rect(grid.block, *lane, W / 2 + CLR / 2)
    # board edge and shaft hole
    for j in range(grid.ny):
        for i in range(grid.nx):
            x, y = grid.xy(i, j)
            r = math.hypot(x - CX, y - CY)
            if r > R_EDGE - tr or r < R_HOLE + tr:
                k = grid.idx(i, j)
                for arr in grid.block:
                    arr[k] = 1
                grid.vblock[k] = 1
    return grid


def _key(item):
    """Stable identity (SWIG hands out a new wrapper object on every access)."""
    if isinstance(item, pcbnew.PAD):
        return ("pad", item.GetParentFootprint().GetReference(), item.GetNumber())
    return ("trk", item.m_Uuid.AsString())


def connected_to(board, net, start_pad):
    """Keys of the copper of `net` already connected to `start_pad`."""
    pads = [p for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetname() == net]
    tracks = [t for t in board.GetTracks() if t.GetNetname() == net]

    def touches(item, x, y, layer):
        """KiCad's rule: an end joins a pad it lies in, a via it lies on, or another
        track END (not the middle of a track)."""
        if isinstance(item, pcbnew.PAD):
            return (layer is None or item.IsOnLayer(layer)) and                 item.HitTest(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
        if item.Type() == pcbnew.PCB_VIA_T:
            vx, vy = mm(item.GetPosition())
            return math.hypot(vx - x, vy - y) <= pcbnew.ToMM(item.GetWidth(pcbnew.F_Cu)) / 2
        if layer is not None and item.GetLayer() != layer:
            return False
        return any(math.hypot(px - x, py - y) <= 0.01 for px, py in (mm(item.GetStart()), mm(item.GetEnd())))

    def ends(item):
        if isinstance(item, pcbnew.PAD) or item.Type() == pcbnew.PCB_VIA_T:
            return [(mm(item.GetPosition()), None)]
        return [(mm(item.GetStart()), item.GetLayer()), (mm(item.GetEnd()), item.GetLayer())]

    items = pads + tracks
    seen = {_key(start_pad)}
    todo = [start_pad]
    while todo:
        cur = todo.pop()
        for other in items:
            k = _key(other)
            if k in seen:
                continue
            if any(touches(other, x, y, l) for (x, y), l in ends(cur)) or \
                    any(touches(cur, x, y, l) for (x, y), l in ends(other)):
                seen.add(k)
                todo.append(other)
    return seen


def targets(board, net, exclude_pad, start_pad=None):
    """Copper of `net` that is NOT yet connected to the start pad."""
    conn = connected_to(board, net, start_pad) if start_pad is not None else set()
    pts = []
    for t in board.GetTracks():
        if t.GetNetname() == net and _key(t) not in conn:
            if t.Type() == pcbnew.PCB_VIA_T:
                pts.append((mm(t.GetPosition()), None))
            else:
                pts.append((mm(t.GetStart()), t.GetLayer()))
                pts.append((mm(t.GetEnd()), t.GetLayer()))
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname() == net and _key(pad) not in conn and \
                    (fp.GetReference(), pad.GetNumber()) != exclude_pad:
                pts.append((mm(pad.GetPosition()), "pad", pad))
    return pts


def astar(grid, start_cells, goal_cells):
    goal_set = set(goal_cells)
    gi = [(c[0], c[1]) for c in goal_cells]

    def h(i, j):
        return min(abs(i - a) + abs(j - b) for a, b in gi[:50]) if gi else 0

    openh = []
    came, gcost = {}, {}
    for s in start_cells:
        gcost[s] = 0
        heapq.heappush(openh, (h(s[0], s[1]), 0, s))
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    while openh:
        f, g, cur = heapq.heappop(openh)
        if cur in goal_set:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        if g > gcost.get(cur, 1e18):
            continue
        i, j, l = cur
        nbrs = []
        for di, dj, c in moves:
            ni, nj = i + di, j + dj
            if 0 <= ni < grid.nx and 0 <= nj < grid.ny and not grid.block[l][grid.idx(ni, nj)]:
                nbrs.append(((ni, nj, l), c))
        if not grid.vblock[grid.idx(i, j)]:
            for nl in range(len(LAYERS)):
                if nl != l:
                    nbrs.append(((i, j, nl), VIA_COST))
        for nxt, c in nbrs:
            ng = g + c
            if ng < gcost.get(nxt, 1e18):
                gcost[nxt] = ng
                came[nxt] = cur
                heapq.heappush(openh, (ng + h(nxt[0], nxt[1]), ng, nxt))
    return None


def main():
    board = pcbnew.LoadBoard(sys.argv[1])
    lids = [board.GetLayerID(n) for n in LAYERS]
    specs = [a for a in sys.argv[2:] if not a.startswith(("reserve=", "--"))]
    reserve = [r for a in sys.argv[2:] if a.startswith("reserve=") for r in a[8:].split(",")]
    global W, VIA_R
    for spec in specs:
        width = None
        if "@" in spec:                 # NET:REF.PAD@0.25 = neck-down width out of a fine-pitch pin
            spec, width = spec.split("@")
        net, padref = spec.split(":")
        power = net.startswith(("+", "-", "VBUS")) or "GND" in net
        W, VIA_R = (0.5, 0.4) if power else (0.2, 0.3)
        if width:
            W = float(width)
        ref, num = padref.split(".")
        fp = board.FindFootprintByReference(ref)
        pad = next(p for p in fp.Pads() if p.GetNumber() == num)
        sx, sy = mm(pad.GetPosition())
        grid = build(board, net, anchor=(sx, sy))
        # start: pad cells on its copper layers
        si, sj = grid.cell(sx, sy)
        starts = [(si, sj, k) for k, l in enumerate(lids) if pad.IsOnLayer(l)]
        # keep the escape corridor of reserved pins (other nets) free: 1.6 mm outward from the pad
        for rr in reserve:
            rref, rnum = rr.split(".")
            rfp = board.FindFootprintByReference(rref)
            rp = next(p for p in rfp.Pads() if p.GetNumber() == rnum)
            if rp.GetNetname() == net:
                continue
            px, py = mm(rp.GetPosition())
            pp = [mm(q.GetPosition()) for q in rfp.Pads()]
            cx, cy = sum(x for x, _ in pp) / len(pp), sum(y for _, y in pp) / len(pp)   # pad centroid
            # escape direction: along the pad's long axis, away from the IC (not radial)
            bb = rp.GetBoundingBox()
            if bb.GetHeight() > bb.GetWidth():
                ux, uy = 0.0, (1.0 if py > cy else -1.0)
            else:
                ux, uy = (1.0 if px > cx else -1.0), 0.0
            ex, ey = px + ux * 1.6, py + uy * 1.6
            for k, l in enumerate(lids):
                if rp.IsOnLayer(l):
                    grid.mark_seg([grid.block[k]], (px, py), (ex, ey), W / 2 + CLR + 0.1)
            # and a via site 0.9 mm out, so the reserved pin can drop to an inner layer
            vx, vy = px + ux * 0.9, py + uy * 0.9
            grid.mark_seg(grid.block, (vx, vy), (vx, vy), VIA_R + CLR + W / 2)
        for s in starts:
            grid.block[s[2]][grid.idx(si, sj)] = 0
        # goals: other copper of the same net (outside this pad)
        goals = []
        exact = {}                      # goal cell -> true target point (track ends are off grid)
        for item in targets(board, net, (ref, num), pad):
            (x, y) = item[0]
            if math.hypot(x - sx, y - sy) < 0.3:
                continue
            gi, gj = grid.cell(x, y)
            if item[1] is None or item[1] == "pad":
                p2 = item[2] if item[1] == "pad" else None
                for k, l in enumerate(lids):
                    if p2 is None or p2.IsOnLayer(l):
                        goals.append((gi, gj, k))
                        grid.block[k][grid.idx(gi, gj)] = 0
                        if p2 is None:
                            exact[(gi, gj, k)] = (x, y)
            elif item[1] in lids:
                k = lids.index(item[1])
                goals.append((gi, gj, k))
                grid.block[k][grid.idx(gi, gj)] = 0
                exact[(gi, gj, k)] = (x, y)
        if not goals:
            print(f"{net}: already connected")
            continue
        path = astar(grid, starts, goals)
        if not path:
            print(f"{net}: no path found")
            continue
        # emit: segments per layer run, vias at layer changes
        pts = [(grid.xy(i, j), l) for i, j, l in path]
        pts[0] = ((sx, sy), pts[0][1])
        if path[-1] in exact:           # end exactly on the target track end / via (KiCad joins ends only)
            pts[-1] = (exact[path[-1]], pts[-1][1])
        n_seg = n_via = 0
        run = [pts[0]]

        def flush(run):
            nonlocal n_seg
            # drop collinear points
            simp = [run[0][0]]
            for k in range(1, len(run) - 1):
                (x0, y0), (x1, y1), (x2, y2) = simp[-1], run[k][0], run[k + 1][0]
                if abs((x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)) > 1e-9:
                    simp.append(run[k][0])
            simp.append(run[-1][0])
            for a, b in zip(simp, simp[1:]):
                if math.dist(a, b) < 1e-6:
                    continue
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(a[0]), pcbnew.FromMM(a[1])))
                t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(b[0]), pcbnew.FromMM(b[1])))
                t.SetWidth(pcbnew.FromMM(W))
                t.SetLayer(lids[run[0][1]])
                t.SetNet(board.FindNet(net))
                board.Add(t)
                n_seg += 1

        for p in pts[1:]:
            if p[1] != run[-1][1]:
                flush(run)
                v = pcbnew.PCB_VIA(board)
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(p[0][0]), pcbnew.FromMM(p[0][1])))
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetDrill(pcbnew.FromMM(0.4 if VIA_R > 0.35 else 0.3))
                v.SetWidth(pcbnew.FromMM(2 * VIA_R))
                v.SetNet(board.FindNet(net))
                board.Add(v)
                n_via += 1
                run = [p]
            else:
                run.append(p)
        flush(run)
        print(f"{net}: routed, {n_seg} segments, {n_via} vias, {len(path)} grid steps")
    board.Save(sys.argv[1])


if __name__ == "__main__":
    main()
