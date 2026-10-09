"""Autoroute the electronics ring with Freerouting 2.4.1 (Java 25).

Two stages, because the 38k coil segments must not go through Freerouting:

  1. (system python)  python -I tools/autoroute.py prepare
     -> docs/route/route.kicad_pcb: the board without the generated coil copper
  2. (KiCad python)   "C:/Program Files/KiCad/10.0/bin/python.exe" tools/autoroute.py route [passes]
     -> adds a keepout over the coil area, exports DSN (net classes from the
        .kicad_pro), leaves GND to the pours and the coil nets alone (they are
        already complete through the hand routes), runs Freerouting, imports SES
  3. (system python)  python -I tools/autoroute.py merge
     -> copies the routed tracks/vias back into InductiveEncoder.kicad_pcb,
        keeping the coil copper untouched
  Signal board:  KiCad python tools/fanout.py, then tools/autoroute.py signal [passes],
     then tools/track_opt.py

Uses the PowerLab KiCad Assistant's Freerouting install and its DSN helpers.
"""
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = os.path.join(ROOT, "InductiveEncoder.kicad_pcb")
WORK = os.path.join(ROOT, "docs", "route")
COPY = os.path.join(WORK, "route.kicad_pcb")
LOCAL = os.environ.get("LOCALAPPDATA", "")
ASSIST = os.path.join(LOCAL, "PowerLabKiCadAssistant")
JAVA = os.path.join(ASSIST, "java-25.0.4.1", "jdk-25.0.4.1+1-jre", "bin", "java.exe")
JAR = os.path.join(ASSIST, "freerouting", "freerouting-2.4.1.jar")
MCP_PY = os.path.join(ASSIST, "KiCAD-MCP-Server", "python")
POUR_NETS = ["GND"]
COIL_NETS = ["SIN1_P", "COS1_P", "SIN2_P", "COS2_P", "EXC1_A", "EXC1_B", "EXC2_A"]
KEEPOUT_R = 30.3

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _items(text):
    from coilgen import _top_level_items
    return _top_level_items(text)


def coil_uuids(items):
    for it in items:
        if it.startswith('(group "COILS"'):
            return set(re.findall(r'"([0-9a-f-]{36})"', it))
    return set()


def item_uuid(it):
    m = re.search(r'\(uuid "([0-9a-f-]{36})"\)', it)
    return m.group(1) if m else None


def prepare():
    os.makedirs(WORK, exist_ok=True)
    text = open(MAIN, encoding="utf-8").read()
    items = _items(text)
    coil = coil_uuids(items)
    keep = [it for it in items if not (item_uuid(it) in coil and it.startswith(("(segment", "(via")))
            and not it.startswith('(group "COILS"')]
    head = text[:text.index("(", 1)]
    open(COPY, "w", encoding="utf-8").write(head + "\n\t".join(keep) + "\n)\n")
    shutil.copy(os.path.join(ROOT, "InductiveEncoder.kicad_pro"), os.path.join(WORK, "route.kicad_pro"))
    print(f"prepared {COPY}: dropped {len(items) - len(keep)} coil items")


def edge_keepout(board, r_in=37.35, r_out=38.6, name="EDGE_KEEPOUT"):
    """Rule-area ring along the round edge: Freerouting doesn't get KiCad's
    0.5 mm copper-to-edge rule from the DSN, but it does respect keepouts."""
    import math
    import pcbnew
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetZoneName(name)
    z.SetDoNotAllowTracks(True)
    z.SetDoNotAllowVias(True)
    z.SetDoNotAllowZoneFills(False)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    ls = pcbnew.LSET()
    for name in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu"):
        ls.AddLayer(board.GetLayerID(name))
    z.SetLayerSet(ls)
    o = z.Outline()
    o.NewOutline()
    for k in range(180):
        t = 2 * math.pi * k / 180
        o.Append(pcbnew.FromMM(100 + r_out * math.cos(t)), pcbnew.FromMM(100 - r_out * math.sin(t)))
    if r_in <= 0:
        board.Add(z)
        return z
    o.NewHole()
    for k in range(180):
        t = -2 * math.pi * k / 180
        o.Append(pcbnew.FromMM(100 + r_in * math.cos(t)), pcbnew.FromMM(100 - r_in * math.sin(t)), -1, 0)
    board.Add(z)
    return z


def strip_tracks(pcb):
    """Remove every track/via/arc (text edit) for a fresh routing run."""
    text = open(pcb, encoding="utf-8").read()
    items = _items(text)
    keep = [it for it in items if not it.startswith(("(segment", "(via", "(arc"))]
    head = text[:text.index("(", 1)]
    open(pcb, "w", encoding="utf-8").write(head + "\n\t".join(keep) + "\n)\n")
    print(f"stripped {len(items) - len(keep)} track items")


def route(passes, pcb=COPY, pro=None, coil_keepout=True, leave_out=None, edge=False):
    import math
    import pcbnew
    sys.path.insert(0, MCP_PY)
    from commands.pour_nets import leave_to_pours
    from utils.project_netclasses import apply_net_classes_to_board, load_project_net_classes
    from commands.freerouting import _reconcile_ses_net_names

    pro = pro or os.path.join(WORK, "route.kicad_pro")
    leave_out = POUR_NETS + COIL_NETS if leave_out is None else leave_out
    keep = open(pro, "rb").read()          # saving the board below rewrites the project file
    board = pcbnew.LoadBoard(pcb)
    apply_net_classes_to_board(board, load_project_net_classes(pro))
    if not coil_keepout:
        if edge:
            zs = list(board.Zones())
            names = [z.GetZoneName() for z in zs]
            # Freerouting treats every pour as a solid plane: give it only the In1 GND plane,
            # so each GND pad gets a via down to it (the other pours are re-added by zones.py)
            for z in zs:
                if z.GetZoneName().startswith("GND_RING") and z.GetZoneName() != "GND_RING_In1.Cu":
                    board.Remove(z)
            if "EDGE_KEEPOUT" not in names:
                edge_keepout(board)
            if "HOLE_KEEPOUT" not in names:
                edge_keepout(board, r_in=0.0, r_out=8.6, name="HOLE_KEEPOUT")
        try:
            return _run(board, pcb, passes, leave_out, None)
        finally:
            open(pro, "wb").write(keep)
    # rule area over the coils: no tracks or vias may be added there
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetDoNotAllowTracks(True)
    z.SetDoNotAllowVias(True)
    z.SetDoNotAllowZoneFills(True)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    ls = pcbnew.LSET()
    for name in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu"):
        ls.AddLayer(board.GetLayerID(name))
    z.SetLayerSet(ls)
    o = z.Outline()
    o.NewOutline()
    for k in range(120):
        t = 2 * math.pi * k / 120
        o.Append(pcbnew.FromMM(100 + KEEPOUT_R * math.cos(t)), pcbnew.FromMM(100 - KEEPOUT_R * math.sin(t)))
    board.Add(z)
    return _run(board, pcb, passes, leave_out, z)


PLANE_LAYERS = ["In1.Cu"]      # solid GND plane (lab rule 3.1): no tracks on it
ROUTE_LAYERS = ["F.Cu", "In2.Cu", "B.Cu"]


def copper_keepouts(board, nets, clearance=0.2):
    """DSN keepouts covering the existing copper of `nets` (plus clearance).

    Nets left out of the DSN lose their wiring there. Their vias and tracks
    (e.g. the GND fan-out) must still block the other nets, so each becomes a
    keepout of its own size plus the clearance, on every routing layer.
    """
    import pcbnew
    out = []
    um = lambda v: pcbnew.ToMM(v) * 1000      # noqa: E731
    for t in board.GetTracks():
        if t.GetNetname() not in nets:
            continue
        if t.Type() == pcbnew.PCB_VIA_T:
            p = t.GetPosition()
            d = um(t.GetWidth(pcbnew.F_Cu)) + 2000 * clearance
            for l in ROUTE_LAYERS:
                out.append(f'    (keepout "" (circle {l} {d:.0f} {um(p.x):.1f} {-um(p.y):.1f}))')
        else:
            name = board.GetLayerName(t.GetLayer())
            if name not in ROUTE_LAYERS:
                continue
            a, b = t.GetStart(), t.GetEnd()
            w = um(t.GetWidth()) + 2000 * clearance
            out.append(f'    (keepout "" (path {name} {w:.0f} {um(a.x):.1f} {-um(a.y):.1f} '
                       f'{um(b.x):.1f} {-um(b.y):.1f}))')
    # local copper areas (zones) of the left-out nets
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() not in nets or z.GetZoneName().startswith("GND_RING"):
            continue
        o = z.Outline().Outline(0)
        pts = " ".join(f"{um(o.CPoint(i).x):.1f} {-um(o.CPoint(i).y):.1f}" for i in range(o.PointCount()))
        name = board.GetLayerName(z.GetLayer())
        if name in ROUTE_LAYERS:
            out.append(f'    (keepout "" (polygon {name} {2000 * clearance:.0f} {pts}))')
    # pads with their own, larger clearance (fiducial keepouts): Freerouting ignores it
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            lc = pcbnew.ToMM(pad.GetLocalClearance() or 0)
            if lc > clearance:
                p = pad.GetPosition()
                d = max(um(pad.GetSize().x), um(pad.GetSize().y)) + 2000 * lc
                for l in ROUTE_LAYERS:
                    if pad.IsOnLayer(board.GetLayerID(l)):
                        out.append(f'    (keepout "" (circle {l} {d:.0f} {um(p.x):.1f} {-um(p.y):.1f}))')
    return out


def _run(board, pcb, passes, leave_out, keepout):
    import pcbnew
    from commands.pour_nets import leave_to_pours
    from commands.freerouting import _reconcile_ses_net_names
    os.makedirs(WORK, exist_ok=True)
    dsn = os.path.join(WORK, "route.dsn")
    ses = os.path.join(WORK, "route.ses")
    def geo(t):
        if t.Type() == pcbnew.PCB_VIA_T:
            return ("v", t.GetPosition().x // 1000, t.GetPosition().y // 1000, t.GetNetname())
        a_, b_ = sorted([(t.GetStart().x // 1000, t.GetStart().y // 1000), (t.GetEnd().x // 1000, t.GetEnd().y // 1000)])
        return ("t", a_, b_, t.GetLayer(), t.GetNetname())
    locked = {geo(t) for t in board.GetTracks() if t.IsLocked()}     # the SES import drops the flag
    # the left-out nets' own (unlocked) copper must survive the import: lock it for now
    tmp = set()
    for t in board.GetTracks():
        if t.GetNetname() in set(leave_out) and not t.IsLocked():
            t.SetLocked(True)
            tmp.add(geo(t))
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        raise SystemExit("DSN export failed")
    txt = open(dsn, encoding="utf-8").read()
    txt, dropped = leave_to_pours(txt, leave_out)
    if leave_out:
        ko = copper_keepouts(board, set(leave_out))
        i = txt.index("(boundary", txt.index("(structure"))
        i = txt.rindex("\n", 0, i) + 1
        txt = txt[:i] + "\n".join(ko) + "\n" + txt[i:]
        print(f"{len(ko)} keepouts for the fixed copper of {sorted(leave_out)}")
    for name in PLANE_LAYERS:
        # a "power" layer in the DSN is a plane: Freerouting routes no tracks on it
        txt = re.sub(r"(\(layer %s\s*\(type )signal\)" % re.escape(name), r"\1power)", txt)
    open(dsn, "w", encoding="utf-8").write(txt)
    print(f"DSN {os.path.getsize(dsn)} bytes, {dropped} nets left out")
    if os.path.exists(ses):
        os.remove(ses)
    cmd = [JAVA, "-jar", JAR, "-de", dsn, "-do", ses, "--gui.enabled=false", "-mp", str(passes)]
    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=WORK)
    open(os.path.join(WORK, "freerouting.log"), "w", encoding="utf-8").write(proc.stdout + proc.stderr)
    if proc.returncode != 0 or not os.path.exists(ses):
        raise SystemExit(f"Freerouting failed ({proc.returncode}), see docs/route/freerouting.log")
    names = [board.GetNetInfo().GetNetItem(i).GetNetname() for i in range(board.GetNetCount())]
    s, _ = _reconcile_ses_net_names(open(ses, encoding="utf-8").read(), names)
    open(ses, "w", encoding="utf-8").write(s)
    if keepout is not None:
        board.Remove(keepout)
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise SystemExit("SES import failed")
    # the SES carries the via diameter only; KiCad takes the drill from the net class,
    # which gives 0.6 mm vias a 0.4 mm drill: restore the lab pairs 0.6/0.3 and 0.8/0.4
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            d = round(pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)), 2)
            t.SetDrill(pcbnew.FromMM(0.3 if d < 0.7 else 0.4))
    relocked = 0
    for t in board.GetTracks():
        if geo(t) in locked:
            t.SetLocked(True)
            relocked += 1
        elif geo(t) in tmp:
            t.SetLocked(False)
    print(f"re-locked {relocked} of {len(locked)} fan-out items")
    # KiCad keeps locked copper on import and the SES brings it back as well: drop exact copies
    seen, dup = set(), []
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            k = ("v", round(t.GetPosition().x, -3), round(t.GetPosition().y, -3), t.GetNetCode())
        else:      # 1 um tolerance: the SES copy of locked copper is shifted by a few nm
            a_, b_ = sorted([(round(t.GetStart().x, -3), round(t.GetStart().y, -3)),
                             (round(t.GetEnd().x, -3), round(t.GetEnd().y, -3))])
            k = ("t", a_, b_, t.GetLayer(), t.GetNetCode())
        if k in seen:
            dup.append(t)
        else:
            seen.add(k)
    for t in dup:
        board.Remove(t)
    if dup:
        print(f"removed {len(dup)} duplicated tracks/vias after import")
    board.Save(pcb)
    print(f"routed board saved: {pcb}")


def merge():
    routed = _items(open(COPY, encoding="utf-8").read())
    new_tracks = [it for it in routed if it.startswith(("(segment", "(via", "(arc"))]
    text = open(MAIN, encoding="utf-8").read()
    items = _items(text)
    coil = coil_uuids(items)
    keep = [it for it in items
            if not (it.startswith(("(segment", "(via", "(arc")) and item_uuid(it) not in coil)]
    # the coil group (and its members) stay; everything else that is copper track comes from the copy
    head = text[:text.index("(", 1)]
    groups_at = next(i for i, it in enumerate(keep) if it.startswith("(group"))
    out = keep[:groups_at] + new_tracks + keep[groups_at:]
    open(MAIN, "w", encoding="utf-8").write(head + "\n\t".join(out) + "\n)\n")
    print(f"merged {len(new_tracks)} routed tracks/vias into {MAIN}")


if __name__ == "__main__":
    stage = sys.argv[1]
    if stage == "prepare":
        prepare()
    elif stage == "route":
        route(int(sys.argv[2]) if len(sys.argv) > 2 else 30)
    elif stage == "merge":
        merge()
    elif stage == "signal-continue":
        # continue from the current routing (rip-up and retry the open connections)
        sig = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb")
        route(int(sys.argv[2]) if len(sys.argv) > 2 else 30, pcb=sig,
              pro=os.path.join(ROOT, "signal", "EncoderSignal.kicad_pro"),
              coil_keepout=False, leave_out=["GND"], edge=True)
    elif stage == "signal":
        # signal board, after tools/fanout.py and tools/power_route.py: every GND pad
        # reaches the In1 plane through its own via and the rails are routed at their
        # class width. GND and the rails stay out of Freerouting (it would draw GND
        # tracks, and it routes every net at the default width); their copper becomes
        # keepouts. Freerouting 2.4.1 also drops "fix" vias of a plane net and routes
        # over them, so keeping them in the DSN doesn't work.
        sig = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb")
        import pcbnew
        nets = [n for n in pcbnew.LoadBoard(sig).GetNetsByName().keys()]
        rails = sorted({str(n) for n in nets if str(n).startswith(("+", "-", "VBUS"))})
        route(int(sys.argv[2]) if len(sys.argv) > 2 else 30, pcb=sig,
              pro=os.path.join(ROOT, "signal", "EncoderSignal.kicad_pro"),
              coil_keepout=False, leave_out=["GND"] + rails, edge=True)
