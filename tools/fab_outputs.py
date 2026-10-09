"""PCBWay output package for both boards (lab rule 3.7).

    python -I tools/fab_outputs.py

fab/<board>/
  <board>_gerbers.zip     Gerber RS-274X (4 Cu, mask, paste, silk, edge) + Excellon (PTH/NPTH)
  <board>_bom.csv         grouped BOM: refs, qty, value, manufacturer, MPN, package, mounting
  <board>_pos.csv         SMD centroids (mm), both sides
  <board>_assembly.pdf    assembly drawings (top/bottom): Fab + mask (pad openings) + outline
Refill the zones (DRC with refill) before running; run again after any board change.
"""
import glob
import os
import shutil
import subprocess
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = os.path.join(r"C:\Program Files", "KiCad", "10.0", "bin", "kicad-cli.exe")
BOARDS = {
    "stator": (os.path.join(ROOT, "InductiveEncoder.kicad_pcb"), os.path.join(ROOT, "InductiveEncoder.kicad_sch")),
    "signal": (os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb"), os.path.join(ROOT, "signal", "EncoderSignal.kicad_sch")),
}
LAYERS = "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
BOM_FIELDS = "Reference,QUANTITY,Value,Manufacturer,Manufacturer Number,Package,Mounting Type,Footprint"
BOM_LABELS = "Designator,Qty,Value,Manufacturer,MPN,Package,Mounting,Footprint"


def run(*args):
    subprocess.run([CLI, *args], check=True, capture_output=True, text=True)


def main():
    for name, (pcb, sch) in BOARDS.items():
        out = os.path.join(ROOT, "fab", name)
        gdir = os.path.join(out, "gerbers")
        shutil.rmtree(gdir, ignore_errors=True)
        os.makedirs(gdir)
        run("pcb", "export", "gerbers", "--layers", LAYERS, "--no-protel-ext", "-o", gdir + os.sep, pcb)
        run("pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th", "--excellon-units", "mm",
            "--generate-map", "--map-format", "gerberx2", "-o", gdir + os.sep, pcb)
        zpath = os.path.join(out, f"{name}_gerbers.zip")
        with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(glob.glob(os.path.join(gdir, "*"))):
                z.write(f, os.path.basename(f))
        run("sch", "export", "bom", "--fields", BOM_FIELDS, "--labels", BOM_LABELS,
            "--group-by", "Value,Footprint,Manufacturer Number", "--exclude-dnp",
            "-o", os.path.join(out, f"{name}_bom.csv"), sch)
        run("pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both", "--smd-only",
            "--exclude-dnp", "-o", os.path.join(out, f"{name}_pos.csv"), pcb)
        run("pcb", "export", "pdf", "--layers", "F.Fab,F.Mask,Edge.Cuts", "--common-layers", "",
            "--mode-single", "--black-and-white", "-o", os.path.join(out, f"{name}_assembly_top.pdf"), pcb)
        run("pcb", "export", "pdf", "--layers", "B.Fab,B.Mask,Edge.Cuts", "--mirror",
            "--mode-single", "--black-and-white", "-o", os.path.join(out, f"{name}_assembly_bottom.pdf"), pcb)
        print(f"{name}: {len(os.listdir(gdir))} gerber/drill files -> {os.path.relpath(zpath, ROOT)}")


if __name__ == "__main__":
    main()
