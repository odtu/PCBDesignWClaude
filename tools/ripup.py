"""Rip up the tracks/vias of some nets near a point (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/ripup.py <board> x y radius NET [NET ...]

Removes unlocked tracks and vias of the given nets with an end within `radius`
mm of (x, y), so tools/maze_route.py can route a blocked pin first and then
reconnect these nets from their pads to the remaining copper.
"""
import math
import sys

import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
x, y, r = map(float, sys.argv[2:5])
nets = set(sys.argv[5:])
gone = []
for t in list(board.GetTracks()):
    if t.GetNetname() not in nets or t.IsLocked():
        continue
    pts = [t.GetPosition()] if t.Type() == pcbnew.PCB_VIA_T else [t.GetStart(), t.GetEnd()]
    if any(math.hypot(pcbnew.ToMM(p.x) - x, pcbnew.ToMM(p.y) - y) <= r for p in pts):
        gone.append(t)
for t in gone:
    board.Remove(t)
board.Save(sys.argv[1])
print(f"ripped up {len(gone)} items of {sorted(nets)}")
