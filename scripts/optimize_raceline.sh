#!/usr/bin/env zsh
# Generate an optimal raceline for an existing recorded track.
#
# Pipeline:
#   1. scripts/export_centerline.py  ->  TUM tracks/<name>.csv
#   2. main_globaltraj.py            ->  outputs/traj_race_cl.csv
#   3. copy result back as           ->  maps/<name>/waypoints_optimal.csv
#
# Usage: ./scripts/optimize_raceline.sh <track_name> [mincurv|mincurv_iqp|shortest_path]

set -e

TRACK="${1:?usage: $0 <track_name> [opt_type]}"
OPT_TYPE="${2:-mincurv}"
WS="${0:a:h:h}"
TUM="$WS/tools/global_racetrajectory_optimization"

echo "[1/3] Extracting medial-axis centerline from maps/$TRACK ..."
python3 "$WS/scripts/extract_centerline.py" "$TRACK"

echo "[2/3] Running TUM optimizer (opt_type=$OPT_TYPE) ..."
cd "$TUM"
python3 main_f1tenth.py "$TRACK" "$OPT_TYPE"

echo "[3/3] Copying raceline to maps/$TRACK/waypoints_optimal.csv ..."
OUT="$TUM/outputs/traj_race_cl.csv"
DST="$WS/maps/$TRACK/waypoints_optimal.csv"
if [ ! -f "$OUT" ]; then
    echo "[ERROR] TUM did not produce $OUT"
    exit 1
fi
# Convert TUM output (s,x,y,psi,kappa,vx,ax) -> our format (x,y,yaw,v)
python3 - "$OUT" "$DST" <<'PY'
import sys, numpy as np
src, dst = sys.argv[1], sys.argv[2]
data = np.loadtxt(src, delimiter=';', comments='#')
# columns: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
out = np.column_stack([data[:, 1], data[:, 2], data[:, 3], data[:, 5]])
np.savetxt(dst, out, delimiter=',', fmt='%.4f')
print(f'  -> {dst}  ({len(out)} points, v range {out[:,3].min():.2f}-{out[:,3].max():.2f} m/s)')
PY

echo "Done. Run with:"
echo "  ./scripts/run_pf_pp.sh  $TRACK  <speed>   # or tweak config to use waypoints_optimal.csv"
