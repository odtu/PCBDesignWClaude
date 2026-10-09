"""Copy schematic symbol data onto the PCB footprints (KiCad python).

    "C:/Program Files/KiCad/10.0/bin/python.exe" tools/sync_fields.py stator|signal

For every schematic symbol (by reference): footprint path (symbol link),
value, all extra fields (hidden), DNP and exclude-from-BOM. This is the field
part of "Update PCB from Schematic" without touching nets or tracks (the MCP
sync renumbers nets and leaves existing tracks on stale net codes).
"""
import glob
import os
import re
import subprocess
import sys

import pcbnew

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARDS = {"stator": (ROOT, "InductiveEncoder"),
          "signal": (os.path.join(ROOT, "signal"), "EncoderSignal")}
CLI = os.path.join(r"C:\Program Files", "KiCad", "10.0", "bin", "kicad-cli.exe")
SKIP_FIELDS = {"Reference", "Value", "Footprint", "Intersheetrefs"}


def items(text):
    out, depth, start, ins, esc = [], 0, None, False, False
    for i, ch in enumerate(text):
        if ins:
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                ins = False
            continue
        if ch == '"':
            ins = True
        elif ch == "(":
            depth += 1
            if depth == 2:
                start = i
        elif ch == ")":
            if depth == 2:
                out.append(text[start:i + 1])
            depth -= 1
    return out


def unq(s):
    return s.replace('\\"', '"').replace("\\n", "\n").replace("\\\\", "\\")


def symbols(folder):
    for path in glob.glob(os.path.join(folder, "*.kicad_sch")):
        for it in items(open(path, encoding="utf-8").read()):
            if not it.startswith("(symbol") or "(lib_id" not in it:
                continue
            uid = re.search(r'\(uuid "([0-9a-f-]{36})"\)', it).group(1)
            inst = re.search(r'\(path "([^"]+)"\s*\(reference "([^"]+)"\)', it)
            if not inst or inst.group(2).startswith("#"):
                continue
            props = {m.group(1): unq(m.group(2))
                     for m in re.finditer(r'\(property "((?:[^"\\]|\\.)*)" "((?:[^"\\]|\\.)*)"', it)}
            yield dict(ref=inst.group(2), path=inst.group(1) + "/" + uid, props=props,
                       in_bom="(in_bom no)" not in it, dnp="(dnp yes)" in it)


def netlist_pins(folder, name):
    """(ref, pin) -> net name from the schematic netlist (kicad-cli)."""
    out = os.path.join(folder, name + "_sync.net")
    subprocess.run([CLI, "sch", "export", "netlist",
                    "--format", "kicadsexpr", "-o", out, os.path.join(folder, name + ".kicad_sch")],
                   check=True, capture_output=True)
    text = open(out, encoding="utf-8").read()
    os.remove(out)
    pins = {}
    for m in re.finditer(r'\(net\s+\(code "\d+"\)\s+\(name "([^"]*)"\)', text):
        end = text.find("(net (code", m.end())
        body = text[m.end(): end if end > 0 else len(text)]
        for n in re.finditer(r'\(node\s+\(ref "([^"]+)"\)\s+\(pin "([^"]+)"\)', body):
            pins[(n.group(1), n.group(2))] = m.group(1)
    return pins


def main():
    folder, name = BOARDS[sys.argv[1]]
    board = pcbnew.LoadBoard(os.path.join(folder, name + ".kicad_pcb"))
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    # pad nets follow the schematic netlist (pin swaps; unconnected pins get
    # KiCad's "unconnected-(...)" nets)
    pins = netlist_pins(folder, name)
    for ref, fp in fps.items():
        for pad in fp.Pads():
            want = pins.get((ref, pad.GetNumber()))
            if want and pad.GetNetname() != want:
                ni = board.FindNet(want)
                if ni is None:
                    ni = pcbnew.NETINFO_ITEM(board, want)
                    board.Add(ni)
                print(f"{ref}.{pad.GetNumber()}: {pad.GetNetname() or '(none)'} -> {want}")
                pad.SetNet(ni)
    n = 0
    for sym in symbols(folder):
        fp = fps.get(sym["ref"])
        if fp is None:
            print("missing footprint for", sym["ref"])
            continue
        fp.SetPath(pcbnew.KIID_PATH(sym["path"]))
        fp.SetValue(sym["props"].get("Value", fp.GetValue()))
        for k, v in sym["props"].items():
            if k in SKIP_FIELDS:
                continue
            fp.SetField(k, v)
            f = fp.GetField(k)
            if f is not None:
                f.SetVisible(False)
        fp.SetDNP(sym["dnp"])
        fp.SetExcludedFromBOM(not sym["in_bom"])
        n += 1
    board.Save(os.path.join(folder, name + ".kicad_pcb"))
    print(f"{sys.argv[1]}: synced {n} footprints")


if __name__ == "__main__":
    main()
