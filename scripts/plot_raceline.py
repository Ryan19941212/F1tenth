#!/usr/bin/env python3
"""Plot recorded waypoints vs optimized raceline on the map."""
import sys
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
from PIL import Image

WS = Path(__file__).resolve().parent.parent


def main(track):
    td = WS / 'maps' / track
    meta = yaml.safe_load(open(td / 'map.yaml'))
    img = np.array(Image.open(td / 'map.pgm'))
    res, ox, oy = meta['resolution'], meta['origin'][0], meta['origin'][1]
    h, w = img.shape

    orig = np.loadtxt(td / 'waypoints.csv', delimiter=',', ndmin=2)
    opt = np.loadtxt(td / 'waypoints_optimal.csv', delimiter=',', ndmin=2)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # Left: path comparison
    ax = axes[0]
    ax.imshow(img, cmap='gray',
              extent=(ox, ox + w * res, oy, oy + h * res))
    ax.plot(orig[:, 0], orig[:, 1], 'b-', lw=1.2, alpha=0.7,
            label=f'Recorded ({len(orig)} pts)')
    ax.plot(opt[:, 0], opt[:, 1], 'r-', lw=1.8,
            label=f'Optimal mincurv ({len(opt)} pts)')
    ax.plot(opt[0, 0], opt[0, 1], 'go', ms=8, label='start')
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    ax.set_title(f'{track} — recorded vs optimal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')

    # Right: raceline colored by speed
    ax = axes[1]
    ax.imshow(img, cmap='gray',
              extent=(ox, ox + w * res, oy, oy + h * res))
    if opt.shape[1] >= 4:
        speeds = opt[:, 3]
        sc = ax.scatter(opt[:, 0], opt[:, 1], c=speeds, cmap='plasma',
                        s=20, vmin=speeds.min(), vmax=speeds.max())
        cbar = fig.colorbar(sc, ax=ax, shrink=0.8)
        cbar.set_label('v [m/s]')
        ax.set_title(f'Raceline speed profile (v: {speeds.min():.2f}–{speeds.max():.2f} m/s)')
    else:
        ax.plot(opt[:, 0], opt[:, 1], 'r-', lw=1.8)
        ax.set_title('Raceline (no speed column)')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')

    out = td / 'debug' / 'raceline_comparison.png'
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=140, bbox_inches='tight')
    print(f'Saved {out}')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'track_20260411_223212')
