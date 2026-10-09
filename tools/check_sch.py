"""Regenerate the schematic, run ERC and export the PDF.

    python -I tools/check_sch.py
"""
import collections
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"
IGNORE = {"lib_symbol_issues", "footprint_link_issues"}

subprocess.run([sys.executable, "-I", os.path.join(ROOT, "tools", "encoder_circuit.py")], check=True)
for name, sch in (("stator", os.path.join(ROOT, "InductiveEncoder.kicad_sch")),
                  ("signal", os.path.join(ROOT, "signal", "EncoderSignal.kicad_sch"))):
    rep = os.path.join(ROOT, "docs", f"erc_{name}.json")
    subprocess.run([CLI, "sch", "erc", "--severity-all", "--format", "json", "-o", rep, sch],
                   check=True, capture_output=True)
    d = json.load(open(rep, encoding="utf-8"))
    v = [x for s in d["sheets"] for x in s["violations"]]
    print(f"ERC {name}:", dict(collections.Counter((x["severity"], x["type"]) for x in v)))
    for x in v:
        if x["type"] not in IGNORE:
            print(" ", x["severity"], x["type"], x["description"][:80], [i["description"][:50] for i in x["items"]][:2])
    pdf = os.path.join(ROOT, "docs", f"schematic_{name}.pdf")
    subprocess.run([CLI, "sch", "export", "pdf", "-o", pdf, sch], check=True, capture_output=True)
    print("PDF:", os.path.relpath(pdf, ROOT))
