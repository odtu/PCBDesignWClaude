"""Lab-specific project settings (odtu/Powerlab KiCAD/PCB_DESIGN_RULES.md §1.1, §1.6).

    python -I tools/lab_setup.py

- Copies the METU PowerLab drawing sheet (latest from the local odtu/Powerlab
  clone's origin/master, read-only) next to each project and sets it with a
  relative path for the schematic and the board.
- Defines the title-block text variables ${PROJECT_NAME}, ${DESIGNER},
  ${PROJECTNUMBER}.
- GND / Power net classes: blue / red, 0.3 mm schematic wires.
"""
import json
import os
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POWERLAB = os.path.join(os.path.dirname(ROOT), "Powerlab")
WKS = "METUPowerLab_KiCadSchematicTemplate.kicad_wks"
TEXT_VARS = {
    "PROJECT_NAME": "Inductive Encoder",
    "DESIGNER": "Ekrem Turan Fırat",
    "PROJECTNUMBER": "TBD",
}
PROJECTS = [
    os.path.join(ROOT, "InductiveEncoder.kicad_pro"),
    os.path.join(ROOT, "signal", "EncoderSignal.kicad_pro"),
]
STYLE = {"GND": "rgb(0, 0, 255)", "Power": "rgb(255, 0, 0)"}


def template():
    try:
        return subprocess.run(["git", "-C", POWERLAB, "show", f"origin/master:KiCAD/{WKS}"],
                              capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return open(os.path.join(ROOT, WKS), "rb").read()     # offline: keep the copy we have


def main():
    wks = template()
    for pro in PROJECTS:
        d = os.path.dirname(pro)
        open(os.path.join(d, WKS), "wb").write(wks)
        cfg = json.load(open(pro, encoding="utf-8"))
        cfg["text_variables"] = dict(TEXT_VARS)
        cfg.setdefault("schematic", {})["page_layout_descr_file"] = WKS
        cfg.setdefault("pcbnew", {})["page_layout_descr_file"] = WKS
        for c in cfg["net_settings"]["classes"]:
            if c["name"] in STYLE:
                c["schematic_color"] = STYLE[c["name"]]
                c["wire_width"] = 12          # mils: 0.3 mm
        json.dump(cfg, open(pro, "w", encoding="utf-8"), indent=2, ensure_ascii=False)
        print("lab setup:", os.path.relpath(pro, ROOT))


if __name__ == "__main__":
    main()
