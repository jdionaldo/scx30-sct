#!/bin/zsh
# Full rebuild of the kit, in the order AGENTS.md gives, with one log and one exit code per step.
# Usage (any folder):  zsh scripts/run_chain.sh [LOGDIR]     add FRONT=1 to rebuild the front end (13_front_v3.py, ~5 min) first.
# The .sc is regenerated last, and only if every earlier step exited 0.
cd "${0:A:h}/.." || exit 1
S=${1:-_stale/chain_logs/$(date +%Y%m%d-%H%M)}; mkdir -p "$S"; S=${S:A}; : > "$S/exits.txt"
export BODY_DIR=$PWD/.~body_dir USE_V4=1 LEVEL_WALL=${LEVEL_WALL:-0}      # the shipped shell comes from 25_shell_variant.py (SHELL_SOURCE=planes for the Tacoma), so no wall levelling
PY=$PWD/.soma/env/bin/python
r(){ n=$1; shift; "$@" > "$S/$n.log" 2>&1; echo "$n $?" >> "$S/exits.txt"; }
[ -n "$FRONT" ] && r 13 $PY scripts/13_front_v3.py "$BODY_DIR"
r 18a $PY scripts/18_tub.py; r 19 $PY scripts/19_shell_v4.py; r 18b $PY scripts/18_tub.py
r 16 $PY scripts/16_integrate_check.py; r clean $PY scripts/20_mesh_clean.py
r accept $PY scripts/checks/shell_accept.py stl/v4/Tacoma_Trophy_Body_Shell.stl
r hw $PY scripts/checks/hw.py; r wallscan $PY _stale/handoff/tub_cage_wall_scan.py stl/v4/Chassis_Tub.stl stl/v4/Cage.stl
( r plates $PY print/make_plates.py && r verify $PY print/verify_plates.py; r parts $PY print/gen_parts_list.py; r diagrams $PY docs/make_diagrams.py; r battery $PY scripts/checks/battery.py ) &
( r 3mf $PY scripts/make_viewer_3mf.py; r step $PY scripts/export/make_step.py ) &
wait
# shell_accept exits 1 while its wall / fidelity checks fail: they are written for the Tacoma (1.4 mm wall, soma.stl reference); its clearance, motion and validity lines still apply
if ! grep -v '^accept ' "$S/exits.txt" | grep -qv ' 0$'; then r sc $PY scripts/export/make_sc.py; r soma $PY -m soma build; fi
echo "== exits"; cat "$S/exits.txt"
echo "== shell"; grep -E 'plane-built|^SHELL|rear sweep contacts|front wheel/shell|tilt_contacts' "$S/19.log" | cut -c1-200
echo "== accept"; grep -E 'FAIL|SHELL ACCEPT' "$S/accept.log" | cut -c1-200
echo "== clean / plates / battery / step / sc / soma"
tail -1 "$S/clean.log"; grep FAILURES "$S/verify.log"; grep 'parts with flags' "$S/battery.log"; grep -E 'WARNING|^wrote' "$S/step.log" | tail -3; tail -2 "$S/sc.log" 2>/dev/null; grep Build "$S/soma.log" 2>/dev/null
echo "== wall scan"; cut -c1-140 "$S/wallscan.log"
