"""Board information on the bottom silkscreen (lab rule 3.6) (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/board_info.py

Signal board: METU PowerLab logo, QR code (github.com/odtu), project / rev /
date / designer / website text. Stator: two text lines in the outer ring
(the coil area is full). Text uses ${TITLE}/${REVISION}/${ISSUE_DATE} from the
board title block and ${PROJECT_NAME}/${DESIGNER} from the project text
variables (tools/lab_setup.py), so board and schematic stay in sync.
Re-running replaces these texts (text edit: SWIG can't delete board texts)
and moves the logo footprints; other bottom-silk texts (J1 labels) stay.
"""
import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from coilgen import _top_level_items  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = r"D:\Belgeler\METUPowerLab\PowerLabKiCadLibraries\footprints\METUPowerLab_Graphics.pretty"
SIGNAL_LINES = ["${PROJECT_NAME}", "Signal Board  Rev ${REVISION}", "${ISSUE_DATE}", "Design: ${DESIGNER}",
                "power.eee.odtu.edu.tr", "github.com/odtu"]
STATOR_LINES = ["${PROJECT_NAME} - Stator  Rev ${REVISION}", "Design: ${DESIGNER}", "METU PowerLab  ${ISSUE_DATE}"]
# earlier versions of these lines, removed on re-run
OLD_LINES = {"Inductive Encoder", "Design: Ekrem Turan Fırat", "INDUCTIVE ENCODER STATOR  Rev ${REVISION}",
             "METU PowerLab  ${ISSUE_DATE}", "METU PowerLab  ${ISSUE_DATE}  Design: ${DESIGNER}"}


def V(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def title_block(board, title):
    tb = board.GetTitleBlock()
    tb.SetTitle(title)
    tb.SetRevision("A")
    tb.SetDate("2026-10-09")
    tb.SetCompany("METU PowerLab")


def clear_texts(path):
    """Drop the board-info gr_texts of an earlier run (text edit)."""
    ours = set(SIGNAL_LINES) | set(STATOR_LINES) | OLD_LINES
    text = open(path, encoding="utf-8").read()
    items = _top_level_items(text)

    def mine(it):
        m = re.match(r'\(gr_text "((?:[^"\\]|\\.)*)"', it)
        return bool(m and m.group(1) in ours and '(layer "B.SilkS")' in it)

    keep = [it for it in items if not mine(it)]
    head = text[:text.index("(", 1)]
    open(path, "w", encoding="utf-8").write(head + "\n\t".join(keep) + "\n)\n")


def text(board, s, x, y, size=1.2, angle=0, just=pcbnew.GR_TEXT_H_ALIGN_CENTER):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetLayer(pcbnew.B_SilkS)
    t.SetMirrored(True)
    t.SetTextSize(V(size, size))
    t.SetTextThickness(pcbnew.FromMM(max(0.15, size * 0.15)))
    t.SetHorizJustify(just)
    t.SetTextAngleDegrees(angle)
    t.SetPosition(V(x, y))
    board.Add(t)


def graphic(board, ref, name, x, y):
    fp = next((f for f in board.GetFootprints() if f.GetReference() == ref), None)
    if fp is None:
        fp = pcbnew.FootprintLoad(LIB, name)
        fp.SetFPID(pcbnew.LIB_ID("METUPowerLab_Graphics", name))
        fp.SetReference(ref)
        fp.SetBoardOnly(True)
        fp.SetExcludedFromBOM(True)
        fp.SetExcludedFromPosFiles(True)
        board.Add(fp)
    if not fp.IsFlipped():
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    fp.SetPosition(V(x, y))


def signal():
    path = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb")
    clear_texts(path)
    b = pcbnew.LoadBoard(path)
    title_block(b, "Inductive Encoder - Signal Board")
    graphic(b, "G1", "METUPowerLab_Logo_30mm", 100.0, 80.0)
    graphic(b, "G2", "METUPowerLab_QRCode_13.3mm", 118.0, 118.0)
    for i, s in enumerate(SIGNAL_LINES):          # clear of the MCU exposed pad (bottom copy) and J1
        text(b, s, 84.0, 115.2 + i * 1.8, size=1.2 if i == 0 else 1.0)
    b.Save(path)
    print("signal board info placed")


def stator():
    path = os.path.join(ROOT, "InductiveEncoder.kicad_pcb")
    clear_texts(path)
    b = pcbnew.LoadBoard(path)
    title_block(b, "Inductive Encoder - Stator")
    for i, s in enumerate(STATOR_LINES):      # bottom of the outer ring, below the coils (r > 31.4)
        text(b, s, 100.0, 131.4 + i * 1.7, size=1.0)
    b.Save(path)
    print("stator board info placed")


if __name__ == "__main__":
    signal()
    stator()
