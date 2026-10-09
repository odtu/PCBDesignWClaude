"""Signal-board routing chain (system python; runs the KiCad-python tools).

    python -I tools/route_signal.py [--from-power]

1. tools/fanout.py       GND vias at every GND pad, neck-downs, local power copper
2. tools/power_route.py  rails at their 0.5 mm class width (routing order: power first)
3. tools/autoroute.py    Freerouting for the signals (GND and rails as keepouts)
4. tools/maze_route.py   whatever DRC still reports open, from the IC pin first
5. tools/track_opt.py    shortcuts, 45 deg corners, pad entry, stubs, power widths
Snapshots go to docs/route/. Start from a placed board (tracks are stripped).
"""
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pcb")
KPY = os.path.join(r"C:\Program Files", "KiCad", "10.0", "bin", "python.exe")
CLI = os.path.join(r"C:\Program Files", "KiCad", "10.0", "bin", "kicad-cli.exe")
T = os.path.join(ROOT, "tools")
SNAP = os.path.join(ROOT, "docs", "route")
env = dict(os.environ, MSYS_NO_PATHCONV="1")


PRO = os.path.join(ROOT, "signal", "EncoderSignal.kicad_pro")
PRO_KEEP = open(PRO, "rb").read()      # board saves from python rewrite the project file


def k(script, *args):
    r = subprocess.run([KPY, os.path.join(T, script), *args], capture_output=True, text=True, env=env)
    open(PRO, "wb").write(PRO_KEEP)
    out = [l for l in (r.stdout + r.stderr).splitlines() if "image handler" not in l and "memory leak" not in l]
    if r.returncode:
        print("\n".join(out[-15:]))
        raise SystemExit(f"{script} failed")
    return out


def drc():
    rep = os.path.join(SNAP, "drc.json")
    subprocess.run([CLI, "pcb", "drc", "--schematic-parity", "--refill-zones", "--format", "json",
                    "-o", rep, PCB], capture_output=True)
    return json.load(open(rep, encoding="utf-8"))


def open_pads(d):
    """Unconnected pairs -> candidate start pads (IC pins first)."""
    jobs = []
    for u in d["unconnected_items"]:
        cands = []
        for it in u["items"]:
            desc = it["description"]
            if "Pad " in desc and " of " in desc:
                num = desc.split("Pad ")[1].split(" ")[0]
                net = desc.split("[")[1].split("]")[0]
                ref = desc.split(" of ")[1].split(" ")[0]
                cands.append((0 if ref.startswith("U") else 1, f"{net}:{ref}.{num}"))
        jobs.append([c for _, c in sorted(cands)])
    return jobs


def main():
    if "--from-power" not in sys.argv:
        print("\n".join(k("fanout.py")[-4:]))
        shutil.copy(PCB, os.path.join(SNAP, "signal_fanout.kicad_pcb"))
        print(k("power_route.py")[-1])
        shutil.copy(PCB, os.path.join(SNAP, "signal_power.kicad_pcb"))
    print(k("autoroute.py", "signal", "40")[-1])
    shutil.copy(PCB, os.path.join(SNAP, "signal_fr_raw.kicad_pcb"))
    for rnd in range(2):
        jobs = open_pads(drc())
        print(f"open connections: {len(jobs)}")
        for cands in jobs:
            for spec in cands:
                out = k("maze_route.py", PCB, spec)
                if any("routed" in l or "already connected" in l for l in out):
                    print("  ", out[-1])
                    break
            else:
                print("   no path:", cands)
    shutil.copy(PCB, os.path.join(SNAP, "signal_routed_raw.kicad_pcb"))
    print("\n".join(k("track_opt.py", PCB)))
    d = drc()
    errs = [v for v in d["violations"] if v["severity"] == "error"]
    print(f"DRC: {len(errs)} errors, {len(d['unconnected_items'])} unconnected, "
          f"{len(d['schematic_parity'])} parity")
    for v in errs:
        print("  ", v["type"], " || ".join(i["description"][:60] for i in v["items"]))
    for u in d["unconnected_items"]:
        print("   open:", " || ".join(i["description"][:60] for i in u["items"]))


if __name__ == "__main__":
    main()
