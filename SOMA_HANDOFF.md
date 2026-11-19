# SOMA hand-off: SCX30 short-course / trophy-truck kit

Repo: `/Users/johndionaldo/GitHub/scx30-sct`. Frame: X forward, Y left, Z up, mm. Last commit is v8 (`c380a8d`). About 181 files are uncommitted (v9 candidate). Do not commit or push unless the owner asks.

## 1. Read this first: SOMA and this repo do not match

- SOMA (`/Applications/SOMA.app` v0.11.0) is a CadQuery app. Its projects are `model.py`, `parts/`, `AGENTS.md`, `CLAUDE.md`, `NOTES.md` and `.soma/skills/`.
- This repo is NOT CadQuery. Geometry is built by Python scripts using trimesh and manifold3d. The outputs are per-body STLs in `stl/v3` (front end) and `stl/v4` (tub, cage, shell, rear). A SolidCraft file, `scx30_sct_107mm_kit.sc` (Git LFS), is regenerated from those STLs.
- So do not "port" the model blindly. Pick one of these:
  - **Mode A (recommended): open the repo folder as the SOMA project and let the agents edit the existing scripts.** Add the `AGENTS.md` below to the repo root. SOMA's viewer shows the STLs.
  - **Mode B: rebuild individual parts in CadQuery** inside a new SOMA project, one part at a time, using the STL as the dimensional reference and the numbers in section 3. Only do this for parts that need real parametric edits (shell, tub). Verify each against the STL volume and bounding box.
- Codex was not found in `/Applications`. Use the Codex CLI (`codex`) in a terminal or whatever SOMA's agent picker offers. Antigravity.app is installed. Confirm both appear in SOMA's agent selector before relying on them.

## 2. Suggested `AGENTS.md` for the SOMA project (Codex and Antigravity both read it)

```
# SCX30 SCT kit
- Units mm; X fwd, Y left, Z up. All four tyre contact patches lie on Z = -16.5.
- Never read .stl/.3mf/.sc as text. Use trimesh to measure.
- Rebuild order (run from repo root, with a venv that has trimesh, manifold3d, python-fcl, scipy, matplotlib, pymupdf):
    scripts/18_tub.py -> scripts/19_shell_v4.py -> scripts/18_tub.py -> USE_V4=1 scripts/16_integrate_check.py
    scripts/13_front_v3.py                      (front end, writes stl/v3)
    print/make_plates.py -> print/verify_plates.py -> print/gen_parts_list.py -> docs/make_diagrams.py
- After any geometry change: python scripts/checks/battery.py (collisions, leftovers, mesh quality)
  and scripts/checks/hw.py (floating fasteners).
- Max 3 attempts per fix. Same error twice = stop and report.
- Do not commit or push. Do not edit print/parts or print/oriented by hand: make_plates.py owns them.
- Edit a variable, not a whole file. Scripts over 300 lines: ask before restructuring.
```

## 3. What has been done (state at v9 working tree)

Design intent: 1/30 SCX30 short-course / trophy truck (Tacoma Trophy shell), fully printable, Injora and SCX30 parts, with a working front diff and IFS, steering, servo, suspension and drivetrain.

| Area | State |
|---|---|
| Wheelbase / tyres | 116.5 mm, O36 x 13.5; all four contact patches on one plane (Z -16.5) |
| Tub | Shortened to the minimum, raised 5.8 mm with the gearbox, motor, ESC, RX and servo (stack raise) so it meets the front diff bottom. Suspension geometry unchanged. Floor top Z +0.9, underside Z -0.9 |
| Bulkhead | Floor web at Z +0.5 (0.8 at screw seats). +0.9 would rub the diff ring gear (bottom Z 0.85) |
| Shell | Tacoma Trophy, lifted 11 mm (roof Z 46.1) with intact arches (not cut). Front arch tightened, nose to X 63.5 (+8 mm), hood no longer short |
| Rear cage | Rebuilt as one part: chamfered SQUARE tube (2.4 mm side, 0.5 chamfer), one hoop plus tail frame, X brace, sway-bar saddles. Floats on the rear, prints inverted |
| Cabin cage | New `Cabin_Cage` (2154 mm3) under the shell, M1.6 bolts. Hood-area bracing: front bumper (bull bar + skid oval) and Y-brace |
| Rear suspension | Trailing arms; upper link with 3 anti-squat pilots (top default 22.8 percent, middle 54.7, lowest 85.9, at assumed CG 21.5 mm). Droop stop limits the rear shaft to 13 deg. Total rear travel 6.84 mm incl. ~4 mm bump stop (was 11.9 before the raise) |
| Shocks | Rear: Injora 36 mm big-bore (eye-to-eye 36, assumed 7-9 mm travel). Front: Injora SCX30-08 (26 mm eye-to-eye, 7.7 mm travel) |
| Front end | Injora double-joint shafts, front travel about 3.1 bump / 3.0 droop, bump stops stop bottoming on the tub |
| Removed | Motor clamp (motor mounts on the transmission), old round-tube cages, rail, slider, spare-tyre carrier, obsolete hardware bodies |
| Electronics | ESC 30x17x6, RX 20x15x7 (user-measured); ESC/RX shelves resized, countersunk lid screws, 2.8 mm antenna slot |
| Print set | 33 printed parts on 6 Bambu plates (assumed 256 x 256 bed, H3C), oriented for strength and speed, easy-peel supports. verify_plates: 0 failures |
| Docs | `ASSEMBLY.md`, `parts_list.md/.csv`, `docs/` diagrams and `SHELL_CLIPS.md`, `print/PLATES.md`, `print/PRINT_SETTINGS.md`, `TUB_V4.md` (Rev 28-43), `FRONT_V3.md` (updates 11-14), `YETI_REFERENCE.md`, `GEARBOX_FIT.md` |
| `.sc` | 144 bodies, names match the STLs exactly (144/144), regenerated from a checksummed snapshot of stl/v3 + stl/v4 |
| Checks run | No non-hardware overlaps; 70 hardware bodies seated; front envelope to shell 1.5-1.9 mm; rear wheel to shell clear to about 8 mm axle rise; fastener audit run (`scripts/checks/hw.py`: only nuts/screws that touch a single body are flagged, which is expected) |

Check scripts are saved in `scripts/checks/` (`battery.py`, `hw.py`, `bump.py`, `wall.py`). They are untracked, so commit them with v9 if wanted. `wall.py` over-reports at sharp edges.

## 4. What is left (priority order)

1. **Front shock clearance**: shock to wheel envelope 0.49 mm, shock to bulkhead 0.19 mm. Target at least 1.0 mm. Move the shock pin, trim the bulkhead, or shorten the upper mount.
2. **Rear travel**: the raise cut total travel to 6.84 mm. The owner wants maximum travel. Options: lower the droop-stop limit (needs a driveshaft angle check), move the rear arm pivot, or relocate the rear shock mount. Weigh against the 13 deg shaft limit.
3. **Slicer thin-wall review**: 0.7 mm retainer plates, ESC shelves (about 0.4 mm), tie rods, and the front-arch fender junction. Open them in Bambu Studio and fix anything flagged. Also test Cabin_Cage print orientation.
4. **Body mount**: tilt-open mount is not modelled. Clip hooks are specified in `docs/SHELL_CLIPS.md` but not built.
5. **Bulkhead floor screws**: only about 1.8 mm engagement. Add a boss or lengthen the screws.
6. **Sweeps to re-run on the final raised geometry**: full steering lock, droop, and compression sweeps; the fastener-by-hole audit.
7. **Unmeasured assumptions to confirm with real parts**: pinion mesh, motor/gearbox screw pattern, servo wire exit, real shock and spring ODs and travel, Injora PRO servo size, printer model and bed size, the "12x25x28" transmission size meaning.
8. **Open questions for the owner**: rear shock lean (28.6 deg now), open cockpit vs Tacoma (assumed Tacoma), battery in the rear box (assumed no).
9. **Housekeeping**: `AUDIT.md` is stale (pre-v4). Arch-rim chamfer and rear arches on the shell are untouched. The 70 MB old `.sc` blob is in LFS history (a rewrite needs the owner's approval). The `.sc` has not been opened in SolidCraft by anyone yet; open it once and spot-check.

## 5. Working rules for both agents

- **Ownership**: one agent per file set at a time. Suggested split: Codex takes scripts and checks (`scripts/`, `print/`); Antigravity takes docs and renders (`docs/`, `*.md`, diagrams). Neither edits `print/parts` or `print/oriented` directly.
- **Do not run my old `plate4.py` approach** (an ad-hoc plate builder). It overwrote the real plates once. Use only `print/make_plates.py`.
- Regenerate the `.sc` only after `stl/v3` and `stl/v4` have finished writing. Snapshot first, then diff live body names against the STL names.
- STL float32 round trips can break watertightness. Check with manifold3d before shipping a part.
- Every fix: state the theory, show the evidence, fix, then run `battery.py` once. If it fails twice, stop and report.
- Ask the owner before commit or push. Commit trailer: `Co-Authored-By: Claude Sonnet 5.5 <noreply@anthropic.com>` only when Claude commits.

## 6. First steps in SOMA

1. Open SOMA, create or open a project pointed at `/Users/johndionaldo/GitHub/scx30-sct`.
2. Add the section 2 `AGENTS.md` to the repo root (SOMA also reads `CLAUDE.md` and `NOTES.md`; keep `NOTES.md` as the running log of section 4 items done).
3. Pick the agent in SOMA's selector (Codex or Antigravity). Give it ONE item from section 4 at a time, for example: "Item 1: raise front shock to wheel clearance to at least 1.0 mm; edit the shock mount in scripts/13_front_v3.py; rebuild; run scripts/checks/battery.py; report overlaps".
4. After each item, run the rebuild order from section 2 and the checks.
5. Hand results back to the owner for the commit decision.
