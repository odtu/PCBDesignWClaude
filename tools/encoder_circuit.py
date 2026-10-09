"""Inductive encoder circuit (TIDA-010961 architecture, METU PowerLab parts).

Run:  python -I tools/encoder_circuit.py      (writes the .kicad_sch files of both boards)

Two boards, stacked on M3 standoffs (same 76 mm outline and holes):
  Stator  (InductiveEncoder.kicad_sch, project root): coils as net ties + J2/J3
          male 2x6 headers carrying the coil terminals (GND rows between pairs)
  Signal  (signal/EncoderSignal.kicad_sch): everything below + J2/J3 sockets

Signal-board sheets
  1 Overview      root page: description, key specs, coil render, sheet blocks
  2 Power         12-24 V in -> LMR51610 buck (6.1 V) -> TPS70950 (5 V AFE)
                  + TPS70933 (3.3 V) ; REF35330 3.3 V ADC reference
  3 Interface     2x6 header: supply, UART, SYNC, SWD, NRST
  4 MCU           MSPM0G3507 (2x 12-bit ADC sampled simultaneously)
  5 AFE outer     LDC5072 + TLV9062 for the 16-period track (SIN1/COS1/EXC1)
  6 AFE inner     LDC5072 + TLV9062 for the 15-period track (SIN2/COS2/EXC2)
  7 Coil conn.    J2/J3 sockets from the stator
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from schgen import Design  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# library parts
# ---------------------------------------------------------------------------
R_LIB = "METUPowerLab_Resistors_ChipResistors"
C_LIB = "METUPowerLab_Capacitors_Ceramics_SurfaceMounts"
R_FP = R_LIB + ":0603_Medium"
C_FP = C_LIB + ":0603_Medium"
SYM = {
    "LDC5072": ("METUPowerLab_Sensors_Encoders", "LDC5072EPWRQ1"),
    "MSPM0": ("METUPowerLab_Microcontrollers_MSPM0", "MSPM0G3507SRHBR"),
    "TLV9062": ("METUPowerLab_Amplifiers_OpAmps", "TLV9062IDGKR"),
    "REF35": ("METUPowerLab_References_VoltageReferences", "REF35330QDBVR"),
    "LDO33": ("METUPowerLab_Regulators_LDOs", "TPS70933DBVR"),
    "LDO50": ("METUPowerLab_Regulators_LDOs", "TPS70950DBVR"),
    "BUCK": ("METUPowerLab_Regulators_BuckConverters", "LMR51610XDBVR"),
    "HDR2x6": ("METUPowerLab_Connectors_Headers", "2x6"),
    "SKT2x6": ("METUPowerLab_Connectors_Headers", "2x6 Female"),
    "NETTIE": ("METUPowerLab_NetTies_SurfaceMounts", "NetTie_2_0.5mm"),
    "TP": ("METUPowerLab_TestPoints_SurfaceMounts", "TestPoint_Pad"),
    "TVS": ("METUPowerLab_Diodes_TVSDiodes", "SMBJ36CA-13-F"),
    "L33U": ("METUPowerLab_Inductors_SurfaceMounts", "33µH"),
    "FB": ("METUPowerLab_CircuitProtections_FerriteBeads", "BLM18EG221SN1D"),
    "LED_G": ("METUPowerLab_LEDs_SurfaceMounts", "Green "),
}
CAP_FP = {"0603": C_FP, "0805": C_LIB + ":0805_Medium", "1210": C_LIB + ":1210_Medium"}
CAP = {  # value -> (symbol name, package)
    "100 nF": ("100 nF", "0603"), "1 µF": ("1 µF 10V", "0603"), "10 µF": ("10 µF 25V", "0603"),
    "22 µF": ("22 µF", "0603"), "1 nF": ("1 nF ", "0603"), "10 nF": ("10 nF", "0603"),
    "220 pF": ("220 pF", "0603"), "390 pF": ("390 pF", "0603"), "470 nF": ("470 nF", "0603"),
    "22 pF": ("22 pF", "0603"), "12 pF": ("12 pF", "0603"), "100 nF 100V": ("100 nF 100V", "0603"),
    "2.2 µF 100V": ("2.2 µF 100V 0805", "0805"),
}
GLOBALS = ["SIN1", "COS1", "SIN2", "COS2", "UART_TX", "UART_RX", "SYNC", "NRST", "SWDIO", "SWCLK",
           # coil nets: global so the names match the generated coil copper exactly
           "SIN1_P", "SIN1_N", "COS1_P", "COS1_N", "EXC1_A", "EXC1_B",
           "SIN2_P", "SIN2_N", "COS2_P", "COS2_N", "EXC2_A", "EXC2_B"]

D = Design(os.path.join(ROOT, "signal"), "Inductive Encoder - Signal Board", "A", "2026-10-09",
           "METU PowerLab", GLOBALS, project="EncoderSignal")
DS = Design(ROOT, "Inductive Encoder - Stator (coils)", "A", "2026-10-09", "METU PowerLab", GLOBALS,
            project="InductiveEncoder")
# board-to-board coil connectors (same pinout on stator header and signal-board socket)
COIL_CONN = {
    "J2": {"1": "EXC1_A", "2": "EXC1_B", "5": "SIN1_P", "6": "SIN1_N", "9": "SIN2_P", "10": "SIN2_N"},
    "J3": {"1": "COS1_P", "2": "COS1_N", "5": "COS2_P", "6": "COS2_N", "9": "EXC2_A", "10": "EXC2_B"},
}
for _pins in COIL_CONN.values():
    _pins.update({k: "GND" for k in ("3", "4", "7", "8", "11", "12")})
_ref = {}


def ref(prefix):
    _ref[prefix] = _ref.get(prefix, 0) + 1
    return f"{prefix}{_ref[prefix]}"


def R(S, val, at, n1, n2, rot=0, dnp=False, sym=None):
    """Resistor; rot 0: pin1 left/pin2 right, rot 270: pin1 top/pin2 bottom."""
    return S.part(ref("R"), R_LIB, sym or val, at, rot, {"1": n1, "2": n2}, fp=R_FP, dnp=dnp)


def C(S, val, at, n1, n2, rot=0):
    sym, pkg = CAP[val]
    return S.part(ref("C"), C_LIB, sym, at, rot, {"1": n1, "2": n2}, fp=CAP_FP[pkg])


def decap_row(S, x0, y, rail, vals, dx=12.7):
    for i, v in enumerate(vals):
        C(S, v, (x0 + i * dx, y), rail, "GND")


def TP(S, net, at, label=None):
    return S.part(ref("TP"), *SYM["TP"], at, 180 if net.startswith("+") else 0, {"1": net},
                  fp="METUPowerLab_TestPoints_SurfaceMounts:Pad_D1.5mm", value=net)


# ---------------------------------------------------------------------------
# Sheet 2: Power
# ---------------------------------------------------------------------------
P = D.sheet("Power", "power.kicad_sch", "Power supply: 12-24 V input, buck, LDOs, ADC reference")
P.text("12-24 V INPUT PROTECTION", (20, 25), 2, True)
P.part(ref("D"), *SYM["TVS"], (35, 55), 270, {"1": "+VIN", "2": "GND"})
C(P, "2.2 µF 100V", (60, 55), "+VIN", "GND")
C(P, "100 nF 100V", (72.7, 55), "+VIN", "GND")
P.text("LAYOUT: D1 and C1/C2 right at the J1 power pins.\n"
       "Note: no reverse-polarity protection - keyed cable required.", (20, 75))

P.text("BUCK  6.1 V / 400 kHz (LMR51610X, PFM)", (95, 25), 2, True)
P.part(ref("U"), *SYM["BUCK"], (115, 55), 0,
       {"5": "+VIN", "4": "+VIN", "2": "GND", "1": "BUCK_CB", "6": "BUCK_SW", "3": "BUCK_FB"})
C(P, "100 nF", (149.86, 48.26), "BUCK_CB", "BUCK_SW")
P.part(ref("L"), *SYM["L33U"], (170.18, 71.12), 0, {"1": "BUCK_SW", "2": "+6V_BUCK"})
R(P, "147 kOhm", (187.96, 50.8), "+6V_BUCK", "BUCK_FB", rot=270)
R(P, "22.1 kOhm", (187.96, 86.36), "BUCK_FB", "GND", rot=270)
decap_row(P, 205.74, 60.96, "+6V_BUCK", ["22 µF", "22 µF", "10 µF"])
P.text("Vout = 0.8 V x (1 + 147k/22.1k) = 6.12 V\n"
       "L = 33 µH, Cout = 2x22 µF + 10 µF (datasheet table 8-1, 400 kHz)\n"
       "LAYOUT: keep the VIN-cap / SW / GND loop tight; SW node small.\n"
       "LAYOUT: keep the buck away from the coils and the AFE analog parts.", (95, 85))

P.text("LDOs", (20, 110), 2, True)
P.part(ref("U"), *SYM["LDO50"], (45, 135), 0, {"1": "+6V_BUCK", "3": "+6V_BUCK", "2": "GND",
                                                  "5": "+5V_AFE", "4": "NC"})
C(P, "1 µF", (25, 140), "+6V_BUCK", "GND")
decap_row(P, 85, 140, "+5V_AFE", ["10 µF", "100 nF"])
P.part(ref("U"), *SYM["LDO33"], (45, 170), 0, {"1": "+6V_BUCK", "3": "+6V_BUCK", "2": "GND",
                                                  "5": "+3V3_SYS", "4": "NC"})
C(P, "1 µF", (25, 175), "+6V_BUCK", "GND")
decap_row(P, 85, 175, "+3V3_SYS", ["10 µF", "100 nF"])
P.text("+5V_AFE: both LDC5072 (5 V supply mode).  +3V3_SYS: MCU, op-amps.", (20, 190))

P.text("ADC REFERENCE  3.3 V", (140, 110), 2, True)
P.part(ref("U"), *SYM["REF35"], (180, 135), 0, val_at=(12.7, 7.62), pins=
       {"4": "+5V_AFE", "3": "+5V_AFE", "1": "GND", "2": "GND", "5": "REF_NR", "6": "+3V3_REF"})
C(P, "100 nF", (145, 140), "+5V_AFE", "GND")
C(P, "100 nF", (205, 145), "REF_NR", "GND")
decap_row(P, 220, 145, "+3V3_REF", ["1 µF", "100 nF"])
P.text("+3V3_REF: MSPM0 VREF+ and the op-amp 1.65 V bias dividers.\n"
       "LAYOUT: C on VREF right at the reference and at MSPM0 VREF+.", (140, 160))

P.text("TEST POINTS", (248, 25), 2, True)
for i, net in enumerate(["+VIN", "+6V_BUCK", "+5V_AFE"]):
    TP(P, net, (250, 45 + i * 15))
for i, net in enumerate(["+3V3_SYS", "+3V3_REF"]):
    TP(P, net, (270, 45 + i * 15))
TP(P, "GND", (270, 72))
P.text("POWER FLAGS", (248, 95), 1.5, True)
for i, net in enumerate(["+VIN", "+6V_BUCK"]):
    P.part(f"#FLG0{i + 1}", "METUPowerLab_Schematic_PowerSymbols", "PWR_FLAG", (252 + i * 15, 112), 180,
           {"1": net})
P.part("#FLG03", "METUPowerLab_Schematic_PowerSymbols", "PWR_FLAG", (275, 125), 0, {"1": "GND"})

# ---------------------------------------------------------------------------
# Sheet 3: Interface
# ---------------------------------------------------------------------------
I = D.sheet("Interface", "interface.kicad_sch", "Host interface connector")
I.text("ENCODER CONNECTOR  J1 (2x6, 2.54 mm)", (30, 30), 2, True)
I.part(ref("J"), *SYM["HDR2x6"], (140, 90), 0, pstub=22.86, pins={
    "1": "+VIN", "2": "+VIN", "3": "GND", "4": "GND", "5": "J_UART_RX", "6": "J_UART_TX",
    "7": "J_SYNC", "8": "NRST", "9": "SWDIO", "10": "SWCLK", "11": "GND", "12": "+3V3_SYS"})
R(I, "22 Ohm", (60, 70), "J_UART_RX", "UART_RX")
R(I, "22 Ohm", (60, 85), "UART_TX", "J_UART_TX")
R(I, "22 Ohm", (60, 100), "J_SYNC", "SYNC")
I.text("Pinout\n 1,2  +VIN 12-24 V        3,4,11  GND\n 5  UART_RX (host -> encoder)   6  UART_TX (encoder -> host)\n"
       " 7  SYNC (sample trigger in)     8  NRST\n 9  SWDIO   10  SWCLK   12  +3V3 (debugger VTref, out)\n"
       "UART: 2.5 Mbaud, 8N1 (TIDA-010961 protocol). The MSPM0 ROM bootloader\n"
       "also uses UART0 (PA10/PA11): hold BSL (PA18) high at reset to flash over J1.",
       (30, 130))
I.text("LAYOUT: J1 on the bottom side (away from the target), at the board edge.", (30, 175))

# ---------------------------------------------------------------------------
# Sheet 4: MCU
# ---------------------------------------------------------------------------
M = D.sheet("MCU", "mcu.kicad_sch", "MSPM0G3507 angle computation")
M.part(ref("U"), *SYM["MSPM0"], (140, 100), 0, pstub={"25": 15.24, "27": 5.08, "4": 15.24}, pins={
    "4": "+3V3_MCU", "5": "GND", "32": "VCORE", "3": "NRST", "6": "ROSC",
    "27": "+3V3_REF", "25": "GND",                     # VREF+ / VREF-
    # SIN1 on A0_12/PA14 (pin 18, next to COS1): the outer AFE is on the right side of
    # the board; A0_0/PA27 (pin 31) sat on the far side of the MCU
    "18": "SIN1", "19": "COS1", "30": "SIN2", "20": "COS2",   # A0_12 / A1_0 / A0_1 / A1_1
    "14": "UART_TX", "15": "UART_RX", "16": "SYNC", "23": "SWDIO", "24": "SWCLK",
    "1": "LED_STATUS", "12": "TEST_A", "13": "TEST_B", "22": "BSL",
    "2": "NC", "7": "NC", "8": "NC", "9": "NC", "10": "NC", "11": "NC", "17": "NC", "31": "NC",
    "21": "NC", "26": "NC", "28": "NC", "29": "NC", "33": "GND"})
M.part(ref("FB"), *SYM["FB"], (35, 35), 0, {"1": "+3V3_SYS", "2": "+3V3_MCU"},
       fp="METUPowerLab_CircuitProtections_FerriteBeads:0603_Medium")
decap_row(M, 55, 45, "+3V3_MCU", ["10 µF", "1 µF", "100 nF"])
C(M, "470 nF", (100, 45), "VCORE", "GND")
R(M, "47 kOhm", (30, 70), "+3V3_MCU", "NRST", rot=270)
C(M, "10 nF", (45, 70), "NRST", "GND")
R(M, "100 kOhm 0.1%", (30, 105), "ROSC", "GND", rot=270)
decap_row(M, 45, 110, "+3V3_REF", ["1 µF", "100 nF"])
R(M, "47 kOhm", (30, 135), "BSL", "GND", rot=270)
TP(M, "BSL", (45, 135))
R(M, "1 kOhm", (215, 45), "+3V3_MCU", "LED_ANODE", rot=270)
M.part(ref("D"), *SYM["LED_G"], (240, 40), 180, {"1": "LED_STATUS", "2": "LED_ANODE"},
       fp="METUPowerLab_LEDs_SurfaceMounts:0603_Medium")
TP(M, "TEST_A", (230, 60))
TP(M, "TEST_B", (245, 60))
M.part("#FLG04", "METUPowerLab_Schematic_PowerSymbols", "PWR_FLAG", (65, 80), 180, {"1": "+3V3_MCU"})
M.text("ADC: SIN on ADC0, COS on ADC1 of the same track -> sampled simultaneously.\n"
       "SIN1 A0_12/PA14, COS1 A1_0/PA15, SIN2 A0_1/PA26, COS2 A1_1/PA16.\n"
       "VREF+ = REF35330 (3.3 V), VREF- = GND. ROSC 100k 0.1 % for SYSOSC FCL.\n"
       "LAYOUT: decoupling caps at VDD/VSS and VCORE pins, VREF caps at PA23.\n"
       "LAYOUT: ROSC resistor right at PA2, short ground return.", (160, 150))

# ---------------------------------------------------------------------------
# Sheets 5/6: AFE (one per track)
# ---------------------------------------------------------------------------
def afe(name, file, k, periods, f_osc):
    A = D.sheet(name, file, f"LDC5072 front end, {periods}-period track")
    n = lambda s: s.replace("#", str(k))  # noqa: E731
    A.text(n(f"COILS - track #: {periods} periods"), (15, 20), 1.5, True)
    A.text(n("The coils are on the stator board. Their terminals arrive through\n"
             "J2/J3 (sheet Coil_Connectors): SIN#_P/N, COS#_P/N, EXC#_A/B."), (15, 30))
    # tank
    A.text(n(f"LC TANK  f = {f_osc}"), (15, 85), 1.5, True)
    C(A, "390 pF", (25, 110), n("EXC#_A"), "GND")
    C(A, "390 pF", (40, 110), n("EXC#_B"), "GND")
    A.text("C0G, 1 %.\nLAYOUT: at LCIN/LCOUT pins.", (50, 108))
    # receive filters
    A.text("INPUT EMC FILTER 120R / 220 pF", (15, 130), 1.5, True)
    for i, (coil, pin) in enumerate([("SIN#_P", "IN0P"), ("SIN#_N", "IN0N"), ("COS#_P", "IN1P"), ("COS#_N", "IN1N")]):
        y = 140 + i * 10
        R(A, "120 Ohm", (30, y), n(coil), n(f"LDC#_{pin}"))
        C(A, "220 pF", (62 + i * 12.7, 150), n(f"LDC#_{pin}"), "GND")
    # LDC5072
    A.part(ref("U"), *SYM["LDC5072"], (105, 100), 0, {
        "1": n("EXC#_A"), "2": n("EXC#_B"), "3": n("LDC#_TM0"), "9": n("LDC#_TOUT"),
        "4": n("LDC#_IN0P"), "6": n("LDC#_IN0N"), "5": n("LDC#_IN1P"), "7": n("LDC#_IN1N"),
        "8": n("LDC#_AGC"), "16": n("LDC#_VCC"), "15": "GND", "14": n("LDC#_VREG"),
        "13": n("LDC#_OUT0P"), "12": n("LDC#_OUT0N"), "11": n("LDC#_OUT1P"), "10": n("LDC#_OUT1N")})
    A.part(ref("FB"), *SYM["FB"], (90, 30), 0, {"1": "+5V_AFE", "2": n("LDC#_VCC")},
           fp="METUPowerLab_CircuitProtections_FerriteBeads:0603_Medium")
    decap_row(A, 114.3, 45.72, n("LDC#_VCC"), ["1 µF", "100 nF", "1 nF"])
    A.part(f"#FLG0{4 + k}", "METUPowerLab_Schematic_PowerSymbols", "PWR_FLAG", (75, 45), 0,
           {"1": n("LDC#_VCC")})
    decap_row(A, 154.94, 45.72, n("LDC#_VREG"), ["1 µF", "1 nF"])
    R(A, "10 kOhm", (115, 165), n("LDC#_TM0"), "GND", rot=270)
    R(A, "10 kOhm", (127, 165), n("LDC#_TOUT"), "GND", rot=270)
    R(A, "1.5 kOhm", (140, 165), n("LDC#_AGC"), "GND", rot=270)
    R(A, "10 kOhm", (155, 165), n("LDC#_VREG"), n("LDC#_AGC"), rot=90, dnp=True)   # rot 90: pin 1 bottom
    A.text("AGC mode: R(AGC_EN->GND) = 1.5k, pull-up DNP.\nFixed gain: fit pull-up, 0.1 % parts (datasheet 9.2.2.2.3).",
           (15, 180))
    for i, out in enumerate(["OUT0P", "OUT0N", "OUT1P", "OUT1N"]):
        C(A, "10 nF", (139.7 + i * 12.7, 124.46), n(f"LDC#_{out}"), "GND")
    A.text("COUT 10 nF. LAYOUT: at the OUTx pins.", (138, 140))
    # differential -> single ended, gain 0.51, 1.65 V bias
    A.text("DIFF -> SINGLE-ENDED  G = 5.1k/10k, bias 1.65 V", (175, 20), 1.5, True)
    A.part(ref("U"), *SYM["TLV9062"], (228.6, 134.62), 0, {
        "8": "+3V3_SYS", "4": "GND",
        "1": n("AMP#_SIN"), "2": n("AMP#_SIN_N"), "3": n("AMP#_SIN_P"),
        "7": n("AMP#_COS"), "6": n("AMP#_COS_N"), "5": n("AMP#_COS_P")})
    C(A, "100 nF", (271.78, 137.16), "+3V3_SYS", "GND")
    for j, (sig, op, on) in enumerate([("SIN", "OUT0P", "OUT0N"), ("COS", "OUT1P", "OUT1N")]):
        y0 = 38.1 + j * 50.8        # keep the R/R/C creation order: it sets the references
        R(A, "10 kOhm", (190.5, y0), n(f"LDC#_{on}"), n(f"AMP#_{sig}_N"))
        R(A, "5.1 kOhm", (231.14, y0), n(f"AMP#_{sig}_N"), n(f"AMP#_{sig}"))
        C(A, "22 pF", (231.14, y0 - 10.16), n(f"AMP#_{sig}_N"), n(f"AMP#_{sig}"), rot=90)
        R(A, "10 kOhm", (190.5, y0 + 12.7), n(f"LDC#_{op}"), n(f"AMP#_{sig}_P"))
        R(A, "10 kOhm", (243.84, y0 + 15.24), "+3V3_REF", n(f"AMP#_{sig}_P"), rot=270)
        R(A, "10 kOhm", (271.78, y0 + 15.24), n(f"AMP#_{sig}_P"), "GND", rot=270)
        C(A, "12 pF", (259.08, y0 + 15.24), n(f"AMP#_{sig}_P"), "GND")
        R(A, "100 Ohm", (185 + j * 45, 160), n(f"AMP#_{sig}"), n(f"{sig}#"))
        C(A, "1 nF", (205 + j * 45, 168), n(f"{sig}#"), "GND")
    A.text("Out = 0.51 x (OUTxP - OUTxN) + 1.65 V. 100R/1 nF anti-alias (1.6 MHz)\n"
           "LAYOUT: RC right at the MSPM0 ADC pins. Keep SIN/COS routing symmetric.", (100, 190))
    return A


afe("AFE_Outer", "afe_outer.kicad_sch", 1, 16, "~4.7 MHz (5.8 µH)")
afe("AFE_Inner", "afe_inner.kicad_sch", 2, 15, "~3.85 MHz (8.8 µH)")

# ---------------------------------------------------------------------------
# Sheet 7: coil connectors (signal board side)
# ---------------------------------------------------------------------------
K = D.sheet("Coil_Connectors", "coil_connectors.kicad_sch", "Board-to-board sockets from the stator")
K.text("COIL CONNECTORS  J2 / J3  (2x6 2.54 mm sockets, mate with the stator headers)", (20, 25), 2, True)
K.part("J2", *SYM["SKT2x6"], (90, 90), 0, pstub=15.24, pins=COIL_CONN["J2"])
K.part("J3", *SYM["SKT2x6"], (190, 90), 0, pstub=15.24, pins=COIL_CONN["J3"])
K.text("J2: EXC1 (outer excitation), SIN1, SIN2.  J3: COS1, COS2, EXC2.\n"
       "Every second pin pair is GND to separate the coil pairs.\n"
       "LAYOUT: J2/J3 on the TOP side, at the same X/Y as the stator headers\n"
       "(stacked on M3 standoffs). Keep each P/N pair together to the AFE.", (20, 140))

# ---------------------------------------------------------------------------
# Root page
# ---------------------------------------------------------------------------
Rt = D.root
Rt.text("INDUCTIVE ENCODER - SIGNAL BOARD  (rev A)", (15, 15), 3, True)
Rt.text("Contactless absolute rotary encoder after TI TIDA-010961 (TIDUFF8).\n"
        "Dual-track Nonius coils (16 / 15 periods) on a 4-layer Ø76 mm stator, Ø56 mm\n"
        "copper target disc, 0.5 mm air gap. Two LDC5072-Q1 AFEs excite the coils and\n"
        "demodulate sin/cos; MSPM0G3507 samples both ADCs simultaneously and computes\n"
        "the absolute angle (Nonius sector + fine angle), sent over UART.", (15, 24), 1.5)
Rt.text("KEY SPECS\n"
        "Input: 12-24 V (4-65 V buck), ~0.5 W\n"
        "Coil area: Ø58 mm, shaft hole Ø16 mm\n"
        "Tracks: 16 periods (outer), 15 periods (inner)\n"
        "Excitation: ~4.7 MHz (outer), ~3.85 MHz (inner)\n"
        "MCU: MSPM0G3507, 2x 12-bit 4 Msps ADC\n"
        "Interface: UART 2.5 Mbaud, SYNC input, SWD\n"
        "Target accuracy (TI ref.): < 0.04°, 16+ ENOB", (15, 55), 1.5)
Rt.image(os.path.join(ROOT, "docs", "coils_preview_small.png"), (215, 75), 1.0)
Rt.text("Stator coils (generated)", (200, 97), 1.27)
blocks = [(P, (20, 120)), (I, (65, 120)), (M, (110, 120)), (D.sheets[3], (20, 155)), (D.sheets[4], (65, 155)),
          (K, (110, 155))]
for sh, at in blocks:
    D.place_sheet(sh, at, (35, 20))
Rt.text("REVISION HISTORY\nA  2026-10-09  first issue", (160, 160), 1.27)

# ---------------------------------------------------------------------------
# Stator board: coils + coil connectors
# ---------------------------------------------------------------------------
S = DS.sheet("Coils", "coils.kicad_sch", "PCB coils (net ties) and coil connectors")
S.text("COILS  (PCB copper generated by tools/coilgen.py)", (20, 20), 2, True)
for i, (refn, p, m, what) in enumerate([("NT1", "SIN1_P", "SIN1_N", "outer SIN receive coil, 16 periods"),
                                        ("NT2", "COS1_P", "COS1_N", "outer COS receive coil, 16 periods"),
                                        ("NT3", "EXC1_A", "EXC1_B", "outer excitation ring, 5.8 uH"),
                                        ("NT4", "SIN2_P", "SIN2_N", "inner SIN receive coil, 15 periods"),
                                        ("NT5", "COS2_P", "COS2_N", "inner COS receive coil, 15 periods"),
                                        ("NT6", "EXC2_A", "EXC2_B", "inner excitation ring, 8.8 uH")]):
    S.part(refn, *SYM["NETTIE"], (40, 40 + i * 15), 0, {"1": p, "2": m}, value=f"COIL_{p.split('_')[0]}")
    S.text(what, (75, 38 + i * 15))
S.text("Each coil is drawn as a net tie: the coil copper and both of its leads carry the\n"
       "_P/_A net; the tie at the end of the N lead joins the _N/_B net.\n"
       "LAYOUT: net ties at the coil lead exits; coil terminals are hand-routed\n"
       "(tools/stator_layout.py) because both lead ends belong to the same net.", (20, 140))
S.part("J2", *SYM["HDR2x6"], (170, 70), 0, pstub=15.24, pins=COIL_CONN["J2"])
S.part("J3", *SYM["HDR2x6"], (250, 70), 0, pstub=15.24, pins=COIL_CONN["J3"])
S.part("#FLG01", "METUPowerLab_Schematic_PowerSymbols", "PWR_FLAG", (210, 120), 0, {"1": "GND"})
S.text("J2/J3: 2x6 2.54 mm male headers on the BOTTOM side at the coil exits.\n"
       "Mate with the signal-board sockets J2/J3 (same pinout). GND comes from\n"
       "the signal board; it only shields the pairs inside the connector.", (150, 140))
St = DS.root
St.text("INDUCTIVE ENCODER - STATOR (COIL) BOARD  (rev A)", (15, 15), 3, True)
St.text("4-layer 76 mm stator with the dual-track Nonius coils (16 / 15 periods,\n"
        "58 mm coil area, 16 mm shaft hole). The target disc rotates 0.5 mm above L1.\n"
        "The signal board (signal/EncoderSignal) stacks behind it on M3 standoffs and\n"
        "plugs into J2/J3. No GND copper on this board: a closed ring around the\n"
        "excitation coils would act as a shorted turn.", (15, 24), 1.5)
St.image(os.path.join(ROOT, "docs", "coils_preview_small.png"), (215, 75), 1.0)
DS.place_sheet(S, (20, 120), (35, 20))
St.text("REVISION HISTORY\nA  2026-10-09  first issue", (110, 160), 1.27)

if __name__ == "__main__":
    os.makedirs(os.path.join(ROOT, "signal"), exist_ok=True)
    D.write()
    DS.write()
