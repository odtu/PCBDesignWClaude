"""Close-up PNG of a board region for review (system python + PyMuPDF).

    python -I tools/crop_view.py <board.kicad_pcb> <layers> x0 y0 x1 y1 <out.png> [dpi]

Exports the layers with kicad-cli (one page, board coordinates in mm) and
crops the region.
"""
import os
import subprocess
import sys
import tempfile

import pymupdf

CLI = os.path.join(r"C:\Program Files", "KiCad", "10.0", "bin", "kicad-cli.exe")
pcb, layers = sys.argv[1], sys.argv[2]
x0, y0, x1, y1 = map(float, sys.argv[3:7])
out = sys.argv[7]
dpi = int(sys.argv[8]) if len(sys.argv) > 8 else 600
with tempfile.TemporaryDirectory() as td:
    pdf = os.path.join(td, "v.pdf")
    subprocess.run([CLI, "pcb", "export", "pdf", "--layers", layers, "--mode-single", "-o", pdf, pcb],
                   check=True, capture_output=True)
    k = 72 / 25.4
    pymupdf.open(pdf)[0].get_pixmap(dpi=dpi, clip=pymupdf.Rect(x0 * k, y0 * k, x1 * k, y1 * k)).save(out)
print(out)
