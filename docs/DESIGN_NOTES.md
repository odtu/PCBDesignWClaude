# Inductive Absolute Encoder (rev A): design notes

Paths below are relative to the repository root.

Contactless absolute rotary encoder after TI **TIDA-010961** ([TIDUFF8](https://www.ti.com/lit/pdf/tiduff8)),
built with METU PowerLab library parts. There are three PCBs, like the TI reference:

| Board | Project | What's on it |
|---|---|---|
| **Stator (coil board)** | `InductiveEncoder.kicad_pro` (this folder) | Ø76 mm, 4 layers. Dual-track coils, 6 net ties, J2/J3 2×6 male headers on the bottom at the coil exits. No other parts and no GND pour (no shorted turn). |
| **Signal board** | `signal/EncoderSignal.kicad_pro` | Ø76 mm, 4 layers, same outline and M3 holes. 2 × LDC5072, 2 × TLV9062, MSPM0G3507, power, J1 host connector. J2/J3 sockets on the top mate with the stator headers. |
| **Target** | `target/EncoderTarget.kicad_pcb` | Ø56 mm, 2 layers. Copper segments (16 + 15) on both layers, rotates 0.5 mm (max ~1.2 mm) above the stator top. |

**Stack:** target → 0.5 mm air gap → stator (coils on L1 facing the target) → J2/J3 header + socket (~11 mm, WR-PHD 8.5 mm socket + header) → signal board.
All three boards share the Ø16 mm shaft hole. Stator and signal board share 4 × M3 NPTH at r = 34 mm (30/150/210/330°), so one set of standoffs holds both.

| | |
|---|---|
| AFE | 2 × LDC5072-Q1 (5 V supply mode, AGC), each with TLV9062 diff-to-single-ended (G = 0.51, 1.65 V bias) |
| MCU | MSPM0G3507 (VQFN-32): SIN on ADC0, COS on ADC1 of each track → simultaneous sampling; VREF+ = REF35330 (3.3 V) |
| Power | 12–24 V → LMR51610X buck (6.12 V, 400 kHz, 33 µH) → TPS70950 (+5V_AFE) / TPS70933 (+3V3_SYS) |
| Host interface | J1 2×6 2.54 mm (signal board, bottom): +VIN ×2, GND ×3, UART RX/TX (2.5 Mbaud), SYNC, NRST, SWDIO, SWCLK, +3V3 (VTref) |
| Coil connectors | J2: EXC1 (pins 1/2), SIN1 (5/6), SIN2 (9/10). J3: COS1 (1/2), COS2 (5/6), EXC2 (9/10). Pins 3/4, 7/8, 11/12 are GND, shielding the pairs. |

## Coil design (`tools/coilgen.py`)

| | Outer track | Inner track |
|---|---|---|
| Periods | 16 | 15 |
| Mean radius / amplitude | 24.0 / 2.8 mm | 13.9 / 2.8 mm |
| Receive band | r 21.2 – 26.8 mm | r 11.1 – 16.7 mm |
| Excitation ring | r 27.7 – 29.1 mm, 3 turns/layer on L1+L2, **5.8 µH** | r 18.1 – 20.3 mm, 5 turns/layer, **8.8 µH** |
| Tank (C1 = C2, signal board) | 390 pF C0G → **~4.7 MHz** | 390 pF C0G → **~3.85 MHz** |

- Different tank frequencies keep the two LDC5072 oscillators from beating into each other (LDC5072 range 2.4–5 MHz).
- **Layer plan:** receive coils on L1/L2, closest to the target. SIN uses L1 for its outer half and L2 for its inner half; COS is the other way round, so a SIN trace only ever meets a COS trace on the other layer. Layer changes use 0.3/0.6 mm vias just after each self-crossing.
- **Leads:** run radially on L3 (P) and L4 (N) in lanes midway between the via columns. COS coils are fed at their 270° crossing so no feed turnaround via sits inside a lead lane.
- **Net ties:** each coil plus both leads is one net (`SIN1_P`, `EXC1_A`, …). The coil's net tie sits on its N lead and joins it to the `_N`/`_B` net. That's why the coil terminals are **hand-routed** to J2/J3 (`tools/stator_layout.py`): an autorouter can't tell the two lead ends apart.
- **No closed copper loop around the coils:** the stator has no GND pour. The signal-board GND pours are slotted at 90° so they don't form a shorted turn around the axis.
- **Inductance:** coil inductance is computed with a filament model (`tools/inductance.py`). Measure the real tank frequency on LCIN and trim C24/C25, C46/C47 if needed.

## Regenerating

```bash
python -I tools/lab_setup.py                   # drawing sheet, title-block text variables, net-class colours
python -I tools/coilgen.py                     # stator coils (text-edits the board; re-runnable)
python -I tools/coilgen.py --target            # target disc
python -I tools/check_sch.py                   # both schematics from tools/encoder_circuit.py + ERC + PDFs
python -I tools/lint_sch_pdf.py docs/schematic_signal.pdf docs/schematic_stator.pdf   # no overlapping text
python -I tools/stator_layout.py strip         # then KiCad python: stator_layout.py build (headers, net ties, coil routes)
"C:/Program Files/KiCad/10.0/bin/python.exe" tools/signal_place.py    # signal-board placement
"C:/Program Files/KiCad/10.0/bin/python.exe" tools/sync_fields.py stator|signal  # symbol fields, pad nets -> footprints
python -I tools/route_signal.py                # signal-board routing chain (see "Signal-board routing")
"C:/Program Files/KiCad/10.0/bin/python.exe" tools/zones.py signal    # slotted GND pours (after routing)
"C:/Program Files/KiCad/10.0/bin/python.exe" tools/lab_extras.py      # U5 local fiducials, GND stitching vias, J1 labels
"C:/Program Files/KiCad/10.0/bin/python.exe" tools/board_info.py      # logo, QR, project/rev/date/designer texts
"C:/Program Files/KiCad/10.0/bin/python.exe" tools/silk.py <board>    # reference placement (both boards)
python -I tools/fab_outputs.py                 # fab/<board>/: gerbers+drill zip, BOM, pos, assembly drawings
```

### Signal-board routing (odtu/PowerLabKiCadAssistant issue #21)

`tools/route_signal.py` strips the tracks and runs the chain below.

1. **`fanout.py`:**
   - **GND:** the GND pours connect the GND pads; there are no GND tracks between pads. A via to the In1 plane goes only:
     - at each decoupling cap's GND pad (0.4/0.8 mm, lab rule 3.2);
     - at fine-pitch IC GND pins;
     - on the 8 pads the pour can't reach because tracks box them in. These were added after routing, where DRC showed the pad unconnected.
   - **Fine-pitch power/GND pins:** they get 0.25 mm neck-downs. MCU GND pins join the exposed pad.
   - **Escape lanes:** fan-out vias stay out of a 1.5 mm lane in front of every fine-pitch pin.
   - **Power pin groups:** they get local copper when no pad of another net lies inside the group (here only +5V_AFE at U2/C8/C9).
2. **`power_route.py`:** the rails come first (lab routing order), at their 0.5 mm class width. Freerouting 2.4.1 ignores net-class widths, so the rails aren't left to it.
3. **`autoroute.py signal`:**
   - Freerouting routes the signals only.
   - GND and the rails go in as keepouts, so it can't draw GND tracks or route over the fan-out.
   - In1 is a plane layer, so no signal routes on it.
4. **`maze_route.py`:** routes what is still open, starting from the IC pin.
   - It keeps escape lanes free and doesn't route under fine-pitch ICs.
   - `ripup.py` frees a congested pin row when needed.
5. **`track_opt.py`:**
   - Shortcuts detours with straight or 45° segments.
   - Enters pads from the nearest side.
   - Chamfers every corner of 90° or less.
   - Removes stubs and duplicates.
   - Widens Power/GND to 0.5 mm where clearance allows; nothing is narrower than 0.15 mm.

**Result:**
- **Widths:** +VIN, +6V_BUCK and +5V_AFE are 100 % at 0.5 mm. +3V3_REF and +3V3_MCU are 0.5 mm apart from the 0.25 mm pin neck-downs; +3V3_SYS is 70 % at 0.5 mm and 0.3–0.4 mm in tight spots.
- **GND:** GND tracks are only the short stubs from decoupling caps and IC pins to their vias.
- **Corners:** no corner of 90° or less outside pads and vias.
- **Track length:** −10 % compared with the raw routing.

**Layout changes made for routing** (no functional change):
- SIN1 moved from PA27/A0_0 (pin 31) to PA14/A0_12 (pin 18), next to COS1; it is still on ADC0 for simultaneous sampling.
- The MCU block (U5, C17–C23, R6, R7, R9, FB1, D2) moved 1.5 mm away from the shaft hole. Its top pin row had only 1.2 mm of room to escape.
- TP8/TP9 moved 4–5 mm away from the MCU's right pin row.

> **Warning:** the schematic `.kicad_sch` files are **generated**. Edit `tools/encoder_circuit.py`, not the sheets,
> or stop regenerating once you start editing by hand. The same applies to the layout scripts: re-running them moves parts and reroutes.
> Symbol UUIDs are stable, so regenerating keeps the board links.

## Library

New parts (LDC5072EPWRQ1, MSPM0G3507SRHBR, TLV9062IDGKR, REF35330QDBVR, TPS70933/50DBVR, 2×6 header, 2×6 socket,
100 pF/390 pF/470 nF caps, 4.7 Ω, 100 kΩ 0.1 %) are committed on the local branch
`library/inductive-encoder-parts` of `PowerLabKiCadLibraries` and **not pushed**.
The schematics and boards carry embedded copies of every symbol and footprint, so the project opens without
them. To update parts from the library, add the PowerLabKiCadLibraries release that contains these parts.

## Lab rules (odtu/Powerlab `KiCAD/PCB_DESIGN_RULES.md`)

Checked against the lab-specific file (origin/master, 2026-10-09).

- **Schematics:**
  - Every sheet uses the METU PowerLab drawing sheet (copied next to each project, relative path).
  - The title-block variables `${PROJECT_NAME}`, `${DESIGNER}` and `${PROJECTNUMBER}` are project text variables.
  - GND nets are drawn blue and Power nets red, both 0.3 mm. There is no overlapping text (`lint_sch_pdf.py`).
  - ERC: 0 errors.
- **Boards:**
  - Board minimums are at the PCBWay 0.15 mm floor. Board-level mask expansion is 0.05 mm and the minimum web 0.1 mm.
  - References sit at 1.0 / 0.15 mm, off pads and outside part bodies.
  - The local fiducial pair FID4/FID5 sits diagonally across U5 (0.5 mm pitch).
  - GND stitching vias run on a 5 mm grid.
  - The board info uses text variables. J1 carries voltage and pin labels.
  - Both boards: DRC 0 errors, 0 unconnected, 0 parity issues.

### Deviations (documented)

- **4 layers** on both stator and signal board (approved by the user).
- **Power/GND net classes:** 0.5 mm tracks and 0.4/0.8 mm vias as in the lab table, but 0.2 mm clearance instead of 0.3 mm. All nets are below 30 V; IPC-2221 asks for 0.1 mm.
  - Neck-downs to 0.25 mm sit only at fine-pitch pins.
  - Some +3V3_SYS sections are 0.3–0.4 mm where the op-amp area leaves no room (rails carry < 0.2 A).
  - Fan-out vias at fine-pitch pins are 0.3/0.6 mm.
- **`*.kicad_dru`:** allows 0.15 mm pad-to-pad clearance inside each fine-pitch IC (TSSOP/VSSOP/VQFN pin gaps are 0.20–0.25 mm).
- **Solder-mask bridges allowed inside U1–U9:** the 0.1 mm mask margin in these library footprints merges the fine-pitch apertures (9 `lib_footprint_mismatch` warnings). Consider giving these library footprints a smaller margin.
- **Minimum thermal spokes = 1** (`min_resolved_spokes`): some GND pads are boxed in by tracks.
- **Passive references on the Fab layer:** 0603 parts sit 0.5–1 mm apart, too close for 1.0 mm silk text. They are on the assembly drawings in `fab/`.
- **Local fiducials 8–9 mm from U5:** the nearest free 3 mm keepout spots.
- **J1 pin-1 silk circle** sits 0.2 mm from the round edge (1 `silk_edge_clearance` warning, library graphic).
- **Round boards:** PCBWay assembly needs a panel with breakaway rails (panelization by the user).
- **No reverse-polarity protection** on +VIN (only a bidirectional TVS): use a keyed cable.
- **No test points on the SIN/COS ADC nets:** probe the anti-alias capacitors (C42/C45, C64/C67) instead. Supplies, GND, BSL and TEST_A/B have test points; UART, SYNC, NRST and SWD are on J1.

## Open items before ordering

- **Teardrops (rule 3.3):** KiCad 10 has no scripting API for them. Open each board, run *Edit → Edit Teardrops → Add teardrops* (pads and vias), refill zones (B), save, then re-run `tools/fab_outputs.py`.
- **`${PROJECTNUMBER}`** is `TBD` in `tools/lab_setup.py`: set the lab project number.
- **C4/C5** (22 µF 0603, 10 V X5R) on the 6.1 V buck output lose most of their capacitance to DC bias. A 16–25 V part in 0805/1206 is safer. This needs a library part.
- Verify the coil/target dimensions and the board-to-board stack height against your motor's mechanics.
- **Firmware isn't part of this project.** It covers ADC sampling at 32 kHz with 8× oversampling, offset/gain calibration, Nonius sector computation (TIDUFF8 §3.2) and UART protocol.
- Measure the tank frequency and coil Q on the first board.
- Print both boards 1:1 and check footprints; check the Gerbers in the Gerber viewer.
