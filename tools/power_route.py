"""Route the power rails before the signals (KiCad python, runs tools/maze_route.py).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/power_route.py

Lab rule 3.3 routing order: high-current paths first. Freerouting 2.4.1
ignores the net-class width (it routes everything at the default 0.2 mm), so
the rails are routed here at their 0.5 mm Power class width, pad by pad from
the regulator outwards, each pad joining the nearest copper of its rail it is
not yet connected to, repeated until each rail is one piece. Where a corridor is too narrow, the width steps down
(0.4, 0.3, 0.25 mm) for that connection only; tools/track_opt.py widens again
wherever clearance allows. Fine-pitch IC pins are skipped: tools/fanout.py
gave them neck-down stubs, which the rail joins.
"""
import math
import os
import subprocess
import sys

import pcbnew

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb")
MAZE = os.path.join(ROOT, "tools", "maze_route.py")
# rail -> source pin (regulator output / input connector)
RAILS = [("+VIN", "J1.1"), ("+6V_BUCK", "L1.2"), ("+5V_AFE", "U2.5"), ("+3V3_SYS", "U3.5"),
         ("+3V3_REF", "U4.6"), ("+3V3_MCU", "FB1.2")]
WIDTHS = (0.5, 0.4, 0.3, 0.25)
FINE_PITCH = 0.8
env = dict(os.environ, MSYS_NO_PATHCONV="1")


def pads_of(board, net, fine=False):
    """Pads of the rail; fine=True returns only the fine-pitch IC pins instead."""
    out = []
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        for p in fp.Pads():
            if p.GetNetname() != net:
                continue
            if ref.startswith("U"):
                others = [q for q in fp.Pads() if q.GetNumber() != p.GetNumber()]
                pitch = min(math.hypot(pcbnew.ToMM(q.GetX() - p.GetX()), pcbnew.ToMM(q.GetY() - p.GetY()))
                            for q in others)
                if (pitch < FINE_PITCH) != fine:
                    continue
            elif fine:
                continue
            out.append((f"{ref}.{p.GetNumber()}", pcbnew.ToMM(p.GetX()), pcbnew.ToMM(p.GetY())))
    return out


def in_power_zone(board, net, x, y):
    for z in board.Zones():
        if z.GetNetname() == net and z.GetZoneName().startswith("PWR_") and \
                z.Outline().Contains(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))):
            return True
    return False


def main():
    board = pcbnew.LoadBoard(PCB)
    plan = []
    for net, src in RAILS:
        pads = pads_of(board, net)
        s = next(p for p in pads if p[0] == src)
        # pads inside the rail's local copper area reach it through the zone fill: only the source joins
        pads = [p for p in pads if p[0] == src or not in_power_zone(board, net, p[1], p[2])]
        pads.sort(key=lambda p: math.hypot(p[1] - s[1], p[2] - s[2]))
        fine = [p[0] for p in pads_of(board, net, fine=True)]
        plan.append((net, [p[0] for p in pads[1:]], [p[0] for p in pads[:1]] + fine))
    del board
    failed = []

    def run(net, ref, widths):
        for w in widths:
            r = subprocess.run([sys.executable, MAZE, PCB, f"{net}:{ref}@{w}"], capture_output=True,
                               text=True, env=env)
            line = [l for l in r.stdout.splitlines() if l.startswith(net)]
            if line and "already connected" in line[-1]:
                return "connected"
            if line and "routed" in line[-1]:
                print(f"{net} {ref}: {w} mm, {line[-1].split(': ', 1)[1]}")
                return "routed"
        return "failed"

    for net, refs, extra in plan:
        # every pad joins the copper it isn't connected to yet; the fine-pitch pins'
        # neck-down stubs join at 0.25 mm. Repeat until the rail is one piece.
        for _ in range(3):
            changed, failed_now = False, []
            for ref in refs + extra:
                widths = (0.25,) if ref in extra[1:] else WIDTHS
                res = run(net, ref, widths)
                changed |= res == "routed"
                if res == "failed":
                    failed_now.append(f"{net}:{ref}")
            if not changed:
                break
        failed += failed_now
    print("not routed:", failed)


if __name__ == "__main__":
    main()
