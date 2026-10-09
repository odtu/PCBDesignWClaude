"""Schematic writer for the inductive encoder (KiCad 10 .kicad_sch).

Builds a hierarchical schematic from a Python description: every part comes
from PowerLabKiCadLibraries, every connected pin gets its own short wire
stub ending in a net label, power symbol or no-connect flag (lab rule 1.4:
no pin-to-pin connections). Inter-sheet nets use global labels (rule 1.2);
supply rails use +V_RAIL power symbols renamed to the rail (rule 1.5).

The circuit itself lives in tools/encoder_circuit.py.
"""
import base64
import math
import os
import re
import uuid

LIB_ROOT = r"D:\Belgeler\METUPowerLab\PowerLabKiCadLibraries\symbols"
PROJECT = "InductiveEncoder"
STUB = 2.54


def uid():
    return str(uuid.uuid4())


def stable_uid(*parts):
    """Deterministic UUID, so regenerated schematics keep the links to the PCB."""
    return str(uuid.uuid5(uuid.NAMESPACE_URL, "inductive-encoder/" + "/".join(parts)))


def q(s):
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'


def f(x):
    x = round(x, 4)
    return ("%.4f" % x).rstrip("0").rstrip(".") if x != int(x) else str(int(x))


# ----------------------------------------------------------------------------
# minimal s-expression parser (for symbol libraries)
# ----------------------------------------------------------------------------
_TOK = re.compile(r'\s+|\(|\)|"(?:[^"\\]|\\.)*"|[^\s()"]+', re.S)


def parse(text):
    stack, cur = [], []
    for m in _TOK.finditer(text):
        t = m.group(0)
        if t.isspace():
            continue
        if t == "(":
            stack.append(cur)
            cur = []
        elif t == ")":
            done = cur
            cur = stack.pop()
            cur.append(done)
        elif t.startswith('"'):
            cur.append(("str", bytes(t[1:-1], "utf-8").decode("unicode_escape").encode("latin-1").decode("utf-8")))
        else:
            cur.append(t)
    return cur


def sval(x):
    return x[1] if isinstance(x, tuple) else x


class LibSymbol:
    def __init__(self, lib, name, text):
        self.lib, self.name, self.text = lib, name, text
        tree = parse(text)[0]
        self.props = {}
        self.pins = []          # (unit, number, name, type, x, y, angle)
        self.power = any(isinstance(e, list) and e and e[0] == "power" for e in tree)
        self.in_bom = not any(isinstance(e, list) and e[:2] == ["in_bom", "no"] for e in tree)
        for e in tree:
            if isinstance(e, list) and e and e[0] == "property":
                at = next((a for a in e if isinstance(a, list) and a and a[0] == "at"), None)
                hide = any(isinstance(a, list) and a and a[0] == "effects" and
                           any(isinstance(b, list) and b and b[0] == "hide" and "yes" in b for b in a)
                           for a in e)
                self.props[sval(e[1])] = (sval(e[2]), [float(v) for v in at[1:4]] if at else [0, 0, 0], hide)
            if isinstance(e, list) and e and e[0] == "symbol":
                sub = sval(e[1])
                unit = int(sub.rsplit("_", 2)[1])
                for p in e:
                    if isinstance(p, list) and p and p[0] == "pin":
                        at = next(a for a in p if isinstance(a, list) and a[0] == "at")
                        nm = next(a for a in p if isinstance(a, list) and a[0] == "name")
                        nu = next(a for a in p if isinstance(a, list) and a[0] == "number")
                        self.pins.append((unit, sval(nu[1]), sval(nm[1]), p[1],
                                          float(at[1]), float(at[2]), float(at[3])))

    def embedded(self):
        """Library text renamed to Lib:Name for the schematic's lib_symbols."""
        t = self.text.replace(f'(symbol "{self.name}"', f'(symbol "{self.lib}:{self.name}"', 1)
        return t


_lib_cache = {}


def load_symbol(lib, name):
    key = (lib, name)
    if key in _lib_cache:
        return _lib_cache[key]
    path = os.path.join(LIB_ROOT, lib + ".kicad_sym")
    text = open(path, encoding="utf-8").read()
    m = re.search(r'\n\t\(symbol "' + re.escape(name) + r'"\n.*?\n\t\)', text, re.S)
    if not m:
        raise KeyError(f"{lib}:{name} not in library")
    s = LibSymbol(lib, name, m.group(0).strip("\n"))
    _lib_cache[key] = s
    return s


# ----------------------------------------------------------------------------
# geometry
# ----------------------------------------------------------------------------
def xform(x, y, at, rot):
    r = math.radians(rot)
    return (at[0] + x * math.cos(r) - y * math.sin(r), at[1] - (x * math.sin(r) + y * math.cos(r)))


def _on_seg(p, a, b):
    (x, y), (x0, y0), (x1, y1) = p, a, b
    if abs((x1 - x0) * (y - y0) - (y1 - y0) * (x - x0)) > 1e-6:
        return False
    return min(x0, x1) - 1e-6 <= x <= max(x0, x1) + 1e-6 and min(y0, y1) - 1e-6 <= y <= max(y0, y1) + 1e-6


def out_dir(pin_angle, rot):
    """Unit vector (schematic coords, y down) pointing away from the body."""
    a = math.radians(pin_angle + 180 + rot)
    return (round(math.cos(a)), round(-math.sin(a)))


# ----------------------------------------------------------------------------
# sheet model
# ----------------------------------------------------------------------------
class Part:
    def __init__(self, sheet, ref, lib, sym, at, rot, pins, fp, value, unit, dnp, fields):
        self.sheet, self.ref, self.sym = sheet, ref, load_symbol(lib, sym)
        self.at, self.rot, self.pins, self.fp = at, rot, pins, fp
        self.value, self.unit, self.dnp, self.fields = value, unit, dnp, fields
        self.uuid = stable_uid(sheet.key, ref)          # stable across regenerations

    def pin_list(self):
        return [p for p in self.sym.pins if p[0] in (0, self.unit)]

    def resolve(self):
        """Map each connection key (pin number or name) to a pin."""
        out = []
        by_num = {p[1]: p for p in self.pin_list()}
        by_name = {}
        for p in self.pin_list():
            by_name.setdefault(p[2], []).append(p)
        used = set()
        for key, net in self.pins.items():
            if key in by_num:
                cand = [by_num[key]]
            elif key in by_name:
                cand = by_name[key]
            else:
                raise KeyError(f"{self.ref}: pin {key!r} not in {self.sym.name} "
                               f"(pins: {sorted(by_num)} / {sorted(by_name)})")
            for p in cand:
                out.append((p, net))
                used.add(p[1])
        missing = [p for p in self.pin_list() if p[1] not in used]
        return out, missing


class Sheet:
    def __init__(self, name, filename, page, title):
        self.name, self.filename, self.page, self.title = name, filename, page, title
        self.parts, self.texts, self.images = [], [], []
        self.key = name
        self.uuid = stable_uid(name, "sheet")      # sheet symbol uuid (in root)
        self.file_uuid = stable_uid(name, "file")

    def part(self, ref, lib, sym, at, rot=0, pins=None, fp=None, value=None, unit=1, dnp=False, pstub=None,
             val_at=None, **fields):
        at = (round(at[0] / 2.54) * 2.54, round(at[1] / 2.54) * 2.54)    # 100 mil grid
        p = Part(self, ref, lib, sym, at, rot, pins or {}, fp, value, unit, dnp, fields)
        p.pstub = pstub          # longer stub for GND/supply pins (crowded connectors)
        p.val_at = val_at        # Value field offset from the symbol origin (overrides the library spot)
        self.parts.append(p)
        return p

    def text(self, s, at, size=1.27, bold=False):
        self.texts.append((s, at, size, bold))

    def image(self, path, at, scale):
        self.images.append((path, at, scale))


class Design:
    def __init__(self, root_dir, title, rev, date, company, globals_, project=PROJECT):
        self.root_dir, self.title, self.rev, self.date, self.company = root_dir, title, rev, date, company
        self.project = project
        self.globals = set(globals_)
        self.sheets = []
        self.root = Sheet("Root", self.project + ".kicad_sch", 1, "Overview")
        self.root.key = f"{self.project}/Root"
        self.root.file_uuid = None
        self.sheet_blocks = []          # (sheet, at, size)
        self.pwr_count = 0
        self.warnings = []

    def sheet(self, name, filename, title):
        s = Sheet(name, filename, len(self.sheets) + 2, title)
        s.key = f"{self.project}/{name}"
        s.uuid = stable_uid(s.key, "sheet")
        s.file_uuid = stable_uid(s.key, "file")
        self.sheets.append(s)
        return s

    def place_sheet(self, sheet, at, size):
        self.sheet_blocks.append((sheet, at, size))

    # -- writing -------------------------------------------------------------
    def _title_block(self, sheet):
        return (f'\t(title_block\n\t\t(title {q(self.title)})\n\t\t(date {q(self.date)})\n'
                f'\t\t(rev {q(self.rev)})\n\t\t(company {q(self.company)})\n'
                f'\t\t(comment 1 {q(sheet.title)})\n\t)\n')

    GROUP_GAP = 10.16      # max distance between stub ends that get chained

    def _layout_nets(self, sheet):
        """Stub wires, chains, attachments and no-connects for one sheet.

        Stub ends of the same net that line up (same direction, same row or
        column, within GROUP_GAP of each other) are chained with a wire and
        share one label / power symbol; inner chain points get a junction.
        """
        pins, wires, nc, ends = [], [], [], []
        for part in sheet.parts:
            conns, missing = part.resolve()
            for p in missing:
                if not part.sym.power:
                    self.warnings.append(f"{sheet.name}: {part.ref} pin {p[1]} ({p[2]}) unassigned -> NC")
                    conns.append((p, "NC"))
            for (unit, num, name, ptype, x, y, a), net in conns:
                px, py = xform(x, y, part.at, part.rot)
                tag = f"{part.ref}.{num}"
                if net == "NC":
                    pins.append((f"NC:{tag}", (px, py), tag))
                    nc.append((px, py))
                    continue
                pins.append((net, (px, py), tag))
                dx, dy = out_dir(a, part.rot)
                if isinstance(part.pstub, dict):            # per-pin stub lengths
                    L = part.pstub.get(num, STUB)
                else:
                    L = part.pstub if (part.pstub and (net == "GND" or net.startswith("+"))) else STUB
                e = (px + dx * L, py + dy * L)
                wires.append((net, (px, py), e, tag))
                ends.append(dict(net=net, p=e, d=(dx, dy), tag=tag))
        others = [(n, p) for n, p, t in pins] + [(e["net"], e["p"]) for e in ends]
        groups = {}
        for e in ends:
            vertical_stub = e["d"][0] == 0
            key = (e["net"], e["d"], round(e["p"][1] if vertical_stub else e["p"][0], 3))
            groups.setdefault(key, []).append(e)
        attach, junctions = [], []
        for (net, d, _), grp in groups.items():
            axis = 0 if d[0] == 0 else 1          # chain along x for vertical stubs
            grp.sort(key=lambda e: e["p"][axis])
            chains, cur = [], [grp[0]]
            for e in grp[1:]:
                blocked = any(n2 != net and _on_seg(p2, cur[-1]["p"], e["p"]) for n2, p2 in others)
                if e["p"][axis] - cur[-1]["p"][axis] <= self.GROUP_GAP + 1e-6 and not blocked:
                    cur.append(e)
                else:
                    chains.append(cur)
                    cur = [e]
            chains.append(cur)
            for ch in chains:
                for a, b in zip(ch, ch[1:]):
                    wires.append((net, a["p"], b["p"], a["tag"] + "~" + b["tag"]))
                for e in ch[1:-1]:
                    junctions.append(e["p"])
                # horizontal stubs stacked vertically: GND symbol on the lowest end
                head = ch[-1] if (d[0] != 0 and net == "GND") else ch[0]
                attach.append((net, head["p"], d, head["tag"]))
        self._check_collisions(sheet, pins, wires)
        return wires, attach, junctions, nc

    def _check_collisions(self, sheet, pins, wires):
        """Every pin end and wire end must touch only copper of its own net.

        KiCad connects any wire end / pin end that lands on another wire or
        pin, so overlapping placements silently short nets together.
        """
        pts = list(pins) + [(n, a, t) for n, a, b, t in wires] + [(n, b, t) for n, a, b, t in wires]

        def on_seg(p, a, b):
            (x, y), (x0, y0), (x1, y1) = p, a, b
            if abs((x1 - x0) * (y - y0) - (y1 - y0) * (x - x0)) > 1e-6:
                return False
            return min(x0, x1) - 1e-6 <= x <= max(x0, x1) + 1e-6 and min(y0, y1) - 1e-6 <= y <= max(y0, y1) + 1e-6

        hits = set()
        for net, p, tag in pts:
            for net2, a, b, tag2 in wires:
                if net2 != net and on_seg(p, a, b):
                    hits.add(tuple(sorted((f"{tag}[{net}]", f"{tag2}[{net2}]"))))
        for a, b in sorted(hits):
            self.warnings.append(f"{sheet.name}: COLLISION {a} <-> {b}")

    def _net_items(self, sheet, path, out):
        """Write stubs, chains, labels / power symbols, junctions, no-connects."""
        wires, attach, junctions, nc = self._layout_nets(sheet)
        for x, y in nc:
            out.append(f'\t(no_connect\n\t\t(at {f(x)} {f(y)})\n\t\t(uuid "{uid()}")\n\t)\n')
        for net, a, b, tag in wires:
            out.append(f'\t(wire\n\t\t(pts\n\t\t\t(xy {f(a[0])} {f(a[1])}) (xy {f(b[0])} {f(b[1])})\n\t\t)\n'
                       f'\t\t(stroke\n\t\t\t(width 0)\n\t\t\t(type default)\n\t\t)\n\t\t(uuid "{uid()}")\n\t)\n')
        for x, y in junctions:
            out.append(f'\t(junction\n\t\t(at {f(x)} {f(y)})\n\t\t(diameter 0)\n\t\t(color 0 0 0 0)\n\t\t(uuid "{uid()}")\n\t)\n')
        for net, (ex, ey), (dx, dy), tag in attach:
            if net == "GND" or net.startswith("+"):
                if net == "GND" and dy < 0:
                    self.warnings.append(f"{sheet.name}: {tag} GND stub points up")
                if net != "GND" and dy > 0:
                    self.warnings.append(f"{sheet.name}: {tag} {net} stub points down")
                out.append(self._power_symbol(net, (ex, ey), path))
                continue
            ang = {(1, 0): 0, (-1, 0): 180, (0, -1): 90, (0, 1): 270}[(dx, dy)]
            just = "left" if ang in (0, 90) else "right"
            if net in self.globals:
                out.append(f'\t(global_label {q(net)}\n\t\t(shape bidirectional)\n\t\t(at {f(ex)} {f(ey)} {ang})\n'
                           f'\t\t(fields_autoplaced yes)\n\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n\t\t\t)\n'
                           f'\t\t\t(justify {just})\n\t\t)\n\t\t(uuid "{uid()}")\n'
                           f'\t\t(property "Intersheetrefs" "${{INTERSHEET_REFS}}"\n\t\t\t(at {f(ex)} {f(ey)} 0)\n'
                           f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t\t(hide yes)\n\t\t\t)\n\t\t)\n\t)\n')
            else:
                out.append(f'\t(label {q(net)}\n\t\t(at {f(ex)} {f(ey)} {ang})\n\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n\t\t\t)\n'
                           f'\t\t\t(justify {just} bottom)\n\t\t)\n\t\t(uuid "{uid()}")\n\t)\n')

    def _power_symbol(self, net, at, path):
        self.pwr_count += 1
        ref = f"#PWR{self.pwr_count:03d}"
        sym = "GND" if net == "GND" else "+V_RAIL"
        s = load_symbol("METUPowerLab_Schematic_PowerSymbols", sym)
        self._used.add(s)
        vy = at[1] + (3.81 if sym == "GND" else -3.81)
        return (f'\t(symbol\n\t\t(lib_id "METUPowerLab_Schematic_PowerSymbols:{sym}")\n\t\t(at {f(at[0])} {f(at[1])} 0)\n'
                f'\t\t(unit 1)\n\t\t(exclude_from_sim no)\n\t\t(in_bom no)\n\t\t(on_board no)\n\t\t(dnp no)\n'
                f'\t\t(uuid "{uid()}")\n'
                f'\t\t(property "Reference" {q(ref)}\n\t\t\t(at {f(at[0])} {f(at[1])} 0)\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t\t(hide yes)\n\t\t\t)\n\t\t)\n'
                f'\t\t(property "Value" {q(net)}\n\t\t\t(at {f(at[0])} {f(vy)} 0)\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
                f'\t\t(property "Footprint" ""\n\t\t\t(at {f(at[0])} {f(at[1])} 0)\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t\t(hide yes)\n\t\t\t)\n\t\t)\n'
                f'\t\t(pin "1"\n\t\t\t(uuid "{uid()}")\n\t\t)\n'
                f'\t\t(instances\n\t\t\t(project {q(self.project)}\n\t\t\t\t(path {q(path)}\n\t\t\t\t\t(reference {q(ref)})\n\t\t\t\t\t(unit 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n')

    @staticmethod
    def _vertical(part):
        ps = [xform(p[4], p[5], part.at, part.rot) for p in part.pin_list()]
        return len(ps) == 2 and abs(ps[0][0] - ps[1][0]) < 1e-6

    def _field_pos(self, part, key):
        rot = part.rot % 360
        x, y = part.at
        if part.sym.name in () or not part.sym.pins:
            return x, y
        # passives (2 pins) and test points: fields beside the body, text always horizontal
        if len(part.sym.pins) <= 3 and not part.sym.power:
            if self._vertical(part) or len(part.sym.pins) == 1:
                return (x + 2.54, y - 1.27) if key == "Reference" else (x + 2.54, y + 1.27)
            return (x, y - 3.81) if key == "Reference" else (x, y + 3.81)
        if key == "Value" and part.val_at:
            return x + part.val_at[0], y + part.val_at[1]
        val, at, _ = part.sym.props.get(key, ("", [0, 0, 0], False))
        return xform(at[0], at[1], part.at, part.rot)

    def _symbol(self, part, path):
        s = part.sym
        self._used.add(s)
        props = []
        value = part.value or s.props.get("Value", ("",))[0]
        fp = part.fp if part.fp is not None else s.props.get("Footprint", ("",))[0]
        fields = {"Reference": part.ref, "Value": value, "Footprint": fp}
        for k, (v, at, hide) in s.props.items():
            if k not in fields and not k.startswith("ki_"):
                fields[k] = v
        fields.update(part.fields)
        rot = part.rot % 360
        for k, v in fields.items():
            if k in ("Reference", "Value"):
                x, y = self._field_pos(part, k)
                beside = len(s.pins) <= 3 and (self._vertical(part) or len(s.pins) == 1)
                # KiCad mirrors field justification with the symbol at 180 deg
                just = ("\n\t\t\t\t(justify " + ("right" if rot in (90, 180) else "left") + ")") if beside else ""
                # power-flag references (#FLGxx) stay hidden
                hide = "\n\t\t\t\t(hide yes)" if (k == "Reference" and part.ref.startswith("#")) else ""
                fa = 90 if rot in (90, 270) else 0
                props.append(f'\t\t(property {q(k)} {q(v)}\n\t\t\t(at {f(x)} {f(y)} {fa})\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t){just}{hide}\n\t\t\t)\n\t\t)\n')
            else:
                props.append(f'\t\t(property {q(k)} {q(v)}\n\t\t\t(at {f(part.at[0])} {f(part.at[1])} 0)\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t\t(hide yes)\n\t\t\t)\n\t\t)\n')
        pins = "".join(f'\t\t(pin {q(p[1])}\n\t\t\t(uuid "{uid()}")\n\t\t)\n' for p in part.pin_list())
        return (f'\t(symbol\n\t\t(lib_id "{s.lib}:{s.name}")\n\t\t(at {f(part.at[0])} {f(part.at[1])} {part.rot})\n'
                f'\t\t(unit {part.unit})\n\t\t(exclude_from_sim no)\n\t\t(in_bom {"yes" if (s.in_bom and not s.power) else "no"})\n'
                f'\t\t(on_board yes)\n\t\t(dnp {"yes" if part.dnp else "no"})\n\t\t(uuid "{part.uuid}")\n'
                + "".join(props) + pins +
                f'\t\t(instances\n\t\t\t(project {q(self.project)}\n\t\t\t\t(path {q(path)}\n\t\t\t\t\t(reference {q(part.ref)})\n'
                f'\t\t\t\t\t(unit {part.unit})\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n')

    def _texts(self, sheet):
        out = []
        for s, at, size, bold in sheet.texts:
            b = "\n\t\t\t\t(bold yes)" if bold else ""
            out.append(f'\t(text {q(s)}\n\t\t(exclude_from_sim no)\n\t\t(at {f(at[0])} {f(at[1])} 0)\n'
                       f'\t\t(effects\n\t\t\t(font\n\t\t\t\t(size {f(size)} {f(size)}){b}\n\t\t\t)\n\t\t\t(justify left top)\n\t\t)\n'
                       f'\t\t(uuid "{uid()}")\n\t)\n')
        for path, at, scale in sheet.images:
            data = base64.b64encode(open(path, "rb").read()).decode()
            chunks = " ".join(q(data[i:i + 76]) for i in range(0, len(data), 76))
            out.append(f'\t(image\n\t\t(at {f(at[0])} {f(at[1])})\n\t\t(scale {f(scale)})\n\t\t(uuid "{uid()}")\n'
                       f'\t\t(data {chunks})\n\t)\n')
        return out

    def write(self):
        root_uuid = None
        root_path = os.path.join(self.root_dir, self.project + ".kicad_sch")
        if os.path.exists(root_path):
            m = re.search(r'\(uuid "?([0-9a-f-]{36})"?\)', open(root_path, encoding="utf-8").read())
            root_uuid = m.group(1) if m else None
        root_uuid = root_uuid or uid()
        for sheet in [self.root] + self.sheets:
            self._used = set()
            path = "/" + root_uuid + ("" if sheet is self.root else "/" + sheet.uuid)
            body = []
            self._net_items(sheet, path, body)
            syms = [self._symbol(p, path) for p in sheet.parts]
            body = syms + body + self._texts(sheet)
            if sheet is self.root:
                for sh, at, size in self.sheet_blocks:
                    body.append(
                        f'\t(sheet\n\t\t(at {f(at[0])} {f(at[1])})\n\t\t(size {f(size[0])} {f(size[1])})\n'
                        f'\t\t(exclude_from_sim no)\n\t\t(in_bom yes)\n\t\t(on_board yes)\n\t\t(dnp no)\n'
                        f'\t\t(stroke\n\t\t\t(width 0.1524)\n\t\t\t(type solid)\n\t\t)\n\t\t(fill\n\t\t\t(color 0 0 0 0.0000)\n\t\t)\n'
                        f'\t\t(uuid "{sh.uuid}")\n'
                        f'\t\t(property "Sheetname" {q(sh.name)}\n\t\t\t(at {f(at[0])} {f(at[1] - 0.7)} 0)\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.524 1.524)\n\t\t\t\t)\n\t\t\t\t(justify left bottom)\n\t\t\t)\n\t\t)\n'
                        f'\t\t(property "Sheetfile" {q(sh.filename)}\n\t\t\t(at {f(at[0])} {f(at[1] + size[1] + 0.6)} 0)\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t\t(justify left top)\n\t\t\t)\n\t\t)\n'
                        f'\t\t(instances\n\t\t\t(project {q(self.project)}\n\t\t\t\t(path {q("/" + root_uuid)}\n\t\t\t\t\t(page {q(str(sh.page))})\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n')
            libs = "".join("\t\t" + s.embedded().replace("\n", "\n\t") + "\n" for s in
                           sorted(self._used, key=lambda s: (s.lib, s.name)))
            file_uuid = root_uuid if sheet is self.root else sheet.file_uuid
            text = (f'(kicad_sch\n\t(version 20260101)\n\t(generator "eeschema")\n\t(generator_version "10.0")\n'
                    f'\t(uuid "{file_uuid}")\n\t(paper "A4")\n' + self._title_block(sheet) +
                    f'\t(lib_symbols\n{libs}\t)\n' + "".join(body))
            if sheet is self.root:
                text += '\t(sheet_instances\n\t\t(path "/"\n\t\t\t(page "1")\n\t\t)\n\t)\n'
            text += '\t(embedded_fonts no)\n)\n'
            open(os.path.join(self.root_dir, sheet.filename), "w", encoding="utf-8").write(text)
        for w in self.warnings:
            print("WARN", w)
        print("wrote", [s.filename for s in [self.root] + self.sheets])
