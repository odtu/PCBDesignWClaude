"""GND pours in the electronics ring (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/zones.py

One C-shaped GND zone per layer (F.Cu, In1.Cu, B.Cu, In2.Cu) covering the
ring r = 30.9 .. 38 mm. Each zone is cut open by a radial slot at the 30 deg
mounting hole: a closed copper ring around the excitation coil would act as
a shorted secondary turn, damping the LC tank and weakening the field.
Nothing is poured inside r = 30.9 mm (coil area). Re-running replaces the
zones named GND_RING_*.

    tools/zones.py signal   -> GND on all four layers of the signal board,
                               slotted at 90 deg (no closed loop around the axis)
"""
import math
import os
import sys

import pcbnew

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(ROOT, "InductiveEncoder.kicad_pcb")
CX, CY = 100.0, 100.0
R_IN, R_OUT = 30.9, 38.5          # outer beyond the edge; the fill stops at the outline
CUT_TH, CUT_W = 30.0, 1.0         # slot angle (deg) and width (mm)
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]


def pt(r, th):
    t = math.radians(th)
    return pcbnew.VECTOR2I(pcbnew.FromMM(CX + r * math.cos(t)), pcbnew.FromMM(CY - r * math.sin(t)))


def main():
    global R_IN, R_OUT, CUT_TH, CUT_W
    pcb = PCB
    if len(sys.argv) > 1 and sys.argv[1] == "signal":
        # signal board: full disc (the shaft hole clips the fill), slot at 90 deg in the
        # free corridor between the two AFE blocks
        pcb = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb")
        R_IN, R_OUT, CUT_TH, CUT_W = 7.0, 38.5, 90.0, 0.6
    board = pcbnew.LoadBoard(pcb)
    old = [z for z in board.Zones() if z.GetZoneName().startswith("GND_RING")]
    for z in old:
        board.Remove(z)
    gnd = board.FindNet("GND")
    for i, lay in enumerate(LAYERS):
        z = pcbnew.ZONE(board)
        z.SetLayer(board.GetLayerID(lay))
        z.SetNetCode(gnd.GetNetCode())
        z.SetZoneName(f"GND_RING_{lay}")
        a0 = CUT_TH + math.degrees(CUT_W / 2 / R_IN)
        a1 = CUT_TH + 360 - math.degrees(CUT_W / 2 / R_IN)
        poly = z.Outline()
        poly.NewOutline()
        n = 180
        for k in range(n + 1):
            p = pt(R_OUT, a0 + (a1 - a0) * k / n)
            poly.Append(p.x, p.y)
        for k in range(n, -1, -1):
            p = pt(R_IN, a0 + (a1 - a0) * k / n)
            poly.Append(p.x, p.y)
        z.SetLocalClearance(pcbnew.FromMM(0.25))
        z.SetMinThickness(pcbnew.FromMM(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(pcbnew.FromMM(0.3))
        z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.3))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        z.SetAssignedPriority(i)
        board.Add(z)
    board.Save(pcb)
    print("zones:", [z.GetZoneName() for z in board.Zones()])


if __name__ == "__main__":
    main()
