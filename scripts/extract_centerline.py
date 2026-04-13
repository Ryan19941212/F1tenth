#!/usr/bin/env python3
"""Extract a proper centerline from an occupancy map via medial axis (skeleton).

Method:
  1. Load map.pgm -> binary free/occupied
  2. Medial axis transform: get skeleton + distance-to-obstacle at each skel pixel
  3. Prune short branches (spurs from wall concavities)
  4. Order skeleton pixels into a single closed loop (degree-2 graph walk)
  5. Resample uniformly, convert to world coords
  6. Width at each point = distance-transform value (radius of inscribed circle)
  7. Write TUM format: x_m, y_m, w_tr_right_m, w_tr_left_m

Usage:
    python3 scripts/extract_centerline.py <track_name> [--prune 5] [--step 0.1]
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import label, binary_dilation, binary_opening
from skimage.morphology import medial_axis

WS = Path(__file__).resolve().parent.parent
TUM_TRACKS_DIR = WS / 'tools' / 'global_racetrajectory_optimization' / 'inputs' / 'tracks'


def load_map(track_dir: Path):
    with open(track_dir / 'map.yaml') as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(track_dir / 'map.pgm'))
    occ_thresh = meta.get('occupied_thresh', 0.65)
    free_thresh = meta.get('free_thresh', 0.196)
    # free cell: > free_thresh ; occupied: < occ_thresh ; unknown in between
    free = img > (255 * (1.0 - free_thresh))  # slam_toolbox writes 255=free, 0=occ
    # safer: free where pixel >= ~205 (slam_toolbox convention)
    free = img >= 205
    free = np.flipud(free)
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    return free, res, ox, oy


def prune_endpoints(skel: np.ndarray) -> np.ndarray:
    """Iteratively remove all degree-1 pixels (spurs) until stable."""
    from scipy.signal import convolve2d
    skel = skel.copy()
    k = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.uint8)
    while True:
        neigh = convolve2d(skel.astype(np.uint8), k, mode='same', boundary='fill')
        endpoints = skel & (neigh == 1)
        if not endpoints.any():
            break
        skel[endpoints] = False
    return skel


def longest_cycle(skel: np.ndarray) -> list:
    """Find the longest simple cycle in the skeleton graph.

    Builds a graph with skeleton pixels as nodes (8-connected), finds cycles,
    returns the longest one as an ordered list of (row, col).
    """
    import networkx as nx
    pts = np.argwhere(skel)
    if len(pts) == 0:
        raise RuntimeError('empty skeleton')

    G = nx.Graph()
    pt_set = {tuple(p) for p in pts}
    for p in pts:
        r, c = int(p[0]), int(p[1])
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                q = (r + dr, c + dc)
                if q in pt_set:
                    G.add_edge((r, c), q)

    # Pick the largest connected component first
    comps = sorted(nx.connected_components(G), key=len, reverse=True)
    sub = G.subgraph(comps[0]).copy()

    # Find the fundamental cycle basis and pick the longest cycle
    try:
        cycles = nx.cycle_basis(sub)
    except nx.NetworkXNoCycle:
        raise RuntimeError('no cycle in skeleton — track not closed?')
    if not cycles:
        raise RuntimeError('cycle_basis returned empty')
    cycle = max(cycles, key=len)

    # cycle_basis returns nodes in traversal order but may not be geometrically
    # contiguous at the junctions — reorder by greedy nearest-neighbor walk
    remaining = set(cycle)
    start = next(iter(remaining))
    ordered = [start]
    remaining.discard(start)
    while remaining:
        r, c = ordered[-1]
        nxt = None
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            cand = (r + dr, c + dc)
            if cand in remaining:
                nxt = cand
                break
        if nxt is None:
            break
        ordered.append(nxt)
        remaining.discard(nxt)
    return ordered


def resample_loop(xy: np.ndarray, step: float) -> np.ndarray:
    """Resample closed polyline to uniform arc length."""
    closed = np.vstack([xy, xy[:1]])
    d = np.hypot(np.diff(closed[:, 0]), np.diff(closed[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(d)])
    total = s[-1]
    target = np.arange(0.0, total, step)
    xs = np.interp(target, s, closed[:, 0])
    ys = np.interp(target, s, closed[:, 1])
    return np.column_stack([xs, ys])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track', help='track name under maps/')
    ap.add_argument('--prune', type=int, default=0,
                    help='(unused — endpoints are pruned iteratively to stability)')
    ap.add_argument('--step', type=float, default=0.1, help='output step (m)')
    ap.add_argument('--safety', type=float, default=0.12,
                    help='safety margin to subtract from each side width (m)')
    args = ap.parse_args()

    track_dir = WS / 'maps' / args.track
    if not track_dir.is_dir():
        sys.exit(f'[ERROR] {track_dir} not found')

    free, res, ox, oy = load_map(track_dir)
    print(f'Map: {free.shape} cells @ {res:.3f} m/cell '
          f'({free.sum()} free, {(~free).sum()} occupied)')

    # Morphological cleanup: remove single-pixel noise islands in BOTH walls
    # and free space (SLAM maps often have salt-and-pepper artefacts that create
    # fake chokepoints in the medial axis).
    free = binary_opening(free, iterations=1)
    print(f'After morphological opening: {free.sum()} free cells')

    # Restrict free region to the connected component containing the recorded
    # waypoints (kills outside-the-track free space that medial_axis would otherwise skeletonize).
    wp_path = track_dir / 'waypoints.csv'
    if wp_path.exists():
        wp = np.loadtxt(wp_path, delimiter=',', ndmin=2)
        seed_row = int((wp[0, 1] - oy) / res)
        seed_col = int((wp[0, 0] - ox) / res)
        labels, n = label(free)
        seed_lbl = labels[seed_row, seed_col]
        if seed_lbl == 0:
            print(f'[WARN] waypoint seed ({seed_row},{seed_col}) not on free cell; '
                  f'falling back to largest connected component')
            sizes = np.bincount(labels.ravel())
            sizes[0] = 0
            seed_lbl = int(np.argmax(sizes))
        free = (labels == seed_lbl)
        print(f'Flood-fill from waypoint seed: {free.sum()} cells kept '
              f'({n} components total)')

    skel, dist = medial_axis(free, return_distance=True)
    print(f'Medial axis: {skel.sum()} skeleton pixels')

    skel = prune_endpoints(skel)
    print(f'After endpoint pruning: {skel.sum()} skeleton pixels')

    ordered = longest_cycle(skel)
    print(f'Longest cycle: {len(ordered)} pixels')

    # to world coords
    rows_cols = np.array(ordered)
    xs = rows_cols[:, 1] * res + ox
    ys = rows_cols[:, 0] * res + oy
    xy = np.column_stack([xs, ys])

    # widths: distance-transform value * res = radius to nearest wall (m)
    widths = dist[rows_cols[:, 0], rows_cols[:, 1]] * res
    # shrink by safety margin (and floor at 1 cell)
    widths = np.maximum(widths - args.safety, res)

    # Pre-smooth the pixel-aligned skeleton with a closed-loop moving average
    # so the TUM spline approximation doesn't have to fight the jaggedness.
    win = 5
    kernel = np.ones(win) / win
    xy_pad = np.vstack([xy[-win:], xy, xy[:win]])
    xy = np.column_stack([
        np.convolve(xy_pad[:, 0], kernel, mode='same')[win:win + len(xy)],
        np.convolve(xy_pad[:, 1], kernel, mode='same')[win:win + len(xy)],
    ])

    # resample; also interpolate width
    closed_xy = np.vstack([xy, xy[:1]])
    closed_w = np.r_[widths, widths[:1]]
    d = np.hypot(np.diff(closed_xy[:, 0]), np.diff(closed_xy[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(d)])
    total = s[-1]
    target = np.arange(0.0, total, args.step)
    xs = np.interp(target, s, closed_xy[:, 0])
    ys = np.interp(target, s, closed_xy[:, 1])
    ws = np.interp(target, s, closed_w)

    print(f'Resampled: {len(xs)} points over {total:.2f} m loop')
    print(f'Width (per side): min {ws.min():.2f} / mean {ws.mean():.2f} / max {ws.max():.2f} m')

    TUM_TRACKS_DIR.mkdir(parents=True, exist_ok=True)
    out = TUM_TRACKS_DIR / f'{args.track}.csv'
    arr = np.column_stack([xs, ys, ws, ws])  # symmetric: medial axis is equidistant from both walls
    with open(out, 'w') as f:
        f.write('# x_m,y_m,w_tr_right_m,w_tr_left_m\n')
        np.savetxt(f, arr, delimiter=',', fmt='%.4f')
    print(f'Wrote {out}')

    # also save a debug plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.imshow(np.flipud(free), cmap='gray',
                  extent=(ox, ox + free.shape[1] * res, oy, oy + free.shape[0] * res))
        ax.plot(xs, ys, 'r-', lw=1.5, label='centerline')
        ax.plot(xs[0], ys[0], 'go', ms=8, label='start')
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title(f'{args.track}: medial-axis centerline ({len(xs)} pts, {total:.1f}m)')
        debug_dir = track_dir / 'debug'
        debug_dir.mkdir(exist_ok=True)
        fig.savefig(debug_dir / 'centerline.png', dpi=120, bbox_inches='tight')
        print(f'Debug plot -> {debug_dir}/centerline.png')
    except Exception as e:
        print(f'(plot skipped: {e})')


if __name__ == '__main__':
    main()
