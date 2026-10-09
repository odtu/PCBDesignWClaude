"""Report the net of every coil lead end (KiCad python). Sanity check that
board edits did not rename coil copper."""
import math
import sys

import pcbnew

b = pcbnew.LoadBoard(sys.argv[1] if len(sys.argv) > 1 else "InductiveEncoder.kicad_pcb")
seen = set()
for t in b.GetTracks():
    if t.Type() != pcbnew.PCB_TRACE_T or abs(pcbnew.ToMM(t.GetWidth()) - 0.2) > 1e-6:
        continue
    for p in (t.GetStart(), t.GetEnd()):
        x, y = pcbnew.ToMM(p.x) - 100, pcbnew.ToMM(p.y) - 100
        r = math.hypot(x, y)
        if 30.5 < r < 31.3:
            seen.add((round(math.degrees(math.atan2(-y, x)) % 360, 1), b.GetLayerName(t.GetLayer()), t.GetNetname()))
for s in sorted(seen):
    print(s)
