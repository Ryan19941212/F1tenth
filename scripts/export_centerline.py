#!/usr/bin/env python3
"""Export recorded waypoints + occupancy map to TUM raceline format.

Input:
    maps/<track>/map.{pgm,yaml}    occupancy grid
    maps/<track>/waypoints.csv     recorded lap (x, y, yaw[, v])

Output:
    tools/global_racetrajectory_optimization/inputs/tracks/<track>.csv
        columns: x_m, y_m, w_tr_right_m, w_tr_left_m

Method:
    1. Load centerline (recorded waypoints smoothed + resampled to uniform arc length).
    2. For each centerline point, compute the unit normal (perp to tangent).
    3. Ray-cast in +normal / -normal across the occupancy grid until a wall hit.
    4. Emit one CSV row per point.

Usage:
    python3 scripts/export_centerline.py <track_name> [--step 0.1] [--smooth 5]
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


WS = Path(__file__).resolve().parent.parent
TUM_TRACKS_DIR = WS / 'tools' / 'global_racetrajectory_optimization' / 'inputs' / 'tracks'


def load_map(track_dir: Path):
    """Return (grid, resolution, origin_x, origin_y).

    grid[row, col] is True for occupied (wall), False for free/unknown.
    row 0 is the bottom of the map (flipped from image convention).
    """
    with open(track_dir / 'map.yaml') as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(track_dir / 'map.pgm'))
    # ROS convention: occupied if pixel < free_thresh*255 (default 0.25 -> 64)
    # slam_toolbox defaults: occupied_thresh=0.65, free_thresh=0.196
    occ_thresh = meta.get('occupied_thresh', 0.65)
    grid = img < (occ_thresh * 255)
    # .pgm image origin is top-left; ROS map origin is bottom-left -> flip rows
    grid = np.flipud(grid)
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    return grid, res, ox, oy


def world_to_grid(x, y, res, ox, oy):
    col = int((x - ox) / res)
    row = int((y - oy) / res)
    return row, col


def raycast(grid, row, col, dr, dc, max_steps):
    """Walk from (row, col) in direction (dr, dc) until we hit an occupied cell
    or step off the grid. Return the step count (≈ distance in cells)."""
    h, w = grid.shape
    r, c = float(row), float(col)
    for k in range(1, max_steps + 1):
        r += dr
        c += dc
        ri, ci = int(r), int(c)
        if ri < 0 or ri >= h or ci < 0 or ci >= w:
            return k - 1
        if grid[ri, ci]:
            return k
    return max_steps


def smooth_and_resample(xy: np.ndarray, step: float, window: int) -> np.ndarray:
    """Moving-average smooth (circular) then resample to uniform arc length."""
    n = len(xy)
    if window > 1:
        w = np.ones(window) / window
        xs = np.convolve(np.r_[xy[-window:, 0], xy[:, 0], xy[:window, 0]], w, mode='same')
        ys = np.convolve(np.r_[xy[-window:, 1], xy[:, 1], xy[:window, 1]], w, mode='same')
        xy = np.column_stack([xs[window:window + n], ys[window:window + n]])

    # cumulative arc length (closed loop)
    d = np.hypot(np.diff(xy[:, 0], append=xy[0, 0]),
                 np.diff(xy[:, 1], append=xy[0, 1]))
    s = np.concatenate([[0.0], np.cumsum(d)])
    total = s[-1]
    target = np.arange(0.0, total, step)
    xs = np.interp(target, s, np.r_[xy[:, 0], xy[0, 0]])
    ys = np.interp(target, s, np.r_[xy[:, 1], xy[0, 1]])
    return np.column_stack([xs, ys])


def tangent_normals(xy: np.ndarray) -> np.ndarray:
    """Unit normal (right of travel direction) for each point, closed loop."""
    dx = np.diff(xy[:, 0], append=xy[0, 0])
    dy = np.diff(xy[:, 1], append=xy[0, 1])
    L = np.hypot(dx, dy)
    L[L == 0] = 1.0
    # right-hand normal: rotate tangent by -90deg -> (dy, -dx)
    nx = dy / L
    ny = -dx / L
    return np.column_stack([nx, ny])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track', help='track name under maps/')
    ap.add_argument('--step', type=float, default=0.1, help='resample step (m)')
    ap.add_argument('--smooth', type=int, default=5, help='moving average window')
    ap.add_argument('--max-width', type=float, default=3.0,
                    help='max ray distance to search (m)')
    ap.add_argument('--min-width', type=float, default=0.3,
                    help='min accepted side width (m); below this the wp is pruned')
    args = ap.parse_args()

    track_dir = WS / 'maps' / args.track
    if not track_dir.is_dir():
        sys.exit(f'[ERROR] {track_dir} not found')

    grid, res, ox, oy = load_map(track_dir)
    print(f'Map: {grid.shape} cells @ {res:.3f} m/cell, origin=({ox:.2f},{oy:.2f})')

    wp = np.loadtxt(track_dir / 'waypoints.csv', delimiter=',', ndmin=2)
    print(f'Waypoints: {len(wp)} raw')

    centerline = smooth_and_resample(wp[:, :2], step=args.step, window=args.smooth)
    print(f'Centerline: {len(centerline)} points @ {args.step} m step')

    normals = tangent_normals(centerline)
    max_steps = int(args.max_width / res)

    rows = []
    pruned = 0
    for (x, y), (nx, ny) in zip(centerline, normals):
        r, c = world_to_grid(x, y, res, ox, oy)
        if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
            pruned += 1
            continue
        # right = along +normal ; left = along -normal
        steps_right = raycast(grid, r, c, ny / res * res, nx / res * res, max_steps)
        # simpler: use dr, dc as unit step vector in grid coords
        # one grid cell = res meters, so moving `1` cell along normal = 1/res meters... we want per-cell step
        # redo: walk one cell at a time in grid coords
        # dr, dc represent how many cells to move per iteration; we want unit magnitude
        pass  # replaced below

    # simpler ray: step by 1 cell along the unit normal in grid coordinates
    rows = []
    pruned = 0
    min_w_cells = args.min_width / res
    for (x, y), (nx, ny) in zip(centerline, normals):
        r, c = world_to_grid(x, y, res, ox, oy)
        if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]) or grid[r, c]:
            pruned += 1
            continue
        # normal in grid: row grows with +y, col grows with +x -> (ny, nx)
        steps_right = raycast(grid, r, c, ny, nx, max_steps)
        steps_left = raycast(grid, r, c, -ny, -nx, max_steps)
        w_right = steps_right * res
        w_left = steps_left * res
        if w_right < args.min_width or w_left < args.min_width:
            pruned += 1
            continue
        rows.append([x, y, w_right, w_left])

    if not rows:
        sys.exit('[ERROR] all points pruned — check map alignment and waypoint coords')

    out = np.array(rows)
    stats = (out[:, 2:].min(), out[:, 2:].mean(), out[:, 2:].max())
    print(f'Kept {len(out)} points ({pruned} pruned); '
          f'width min/mean/max = {stats[0]:.2f} / {stats[1]:.2f} / {stats[2]:.2f} m')

    TUM_TRACKS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = TUM_TRACKS_DIR / f'{args.track}.csv'
    with open(out_path, 'w') as f:
        f.write('# x_m,y_m,w_tr_right_m,w_tr_left_m\n')
        np.savetxt(f, out, delimiter=',', fmt='%.4f')
    print(f'Wrote {out_path}')


if __name__ == '__main__':
    main()
