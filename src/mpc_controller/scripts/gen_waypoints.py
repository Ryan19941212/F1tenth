#!/usr/bin/env python3
"""
Generate example waypoint CSV files for MPC testing.

Usage:
    python3 gen_waypoints.py --shape circle  --output ../waypoints/example_circle.csv
    python3 gen_waypoints.py --shape oval    --output ../waypoints/example_oval.csv
    python3 gen_waypoints.py --shape figure8 --output ../waypoints/example_figure8.csv
"""

import argparse
import math
import os

import numpy as np


def _write_csv(waypoints: np.ndarray, path: str, speed: float) -> None:
    """waypoints: (N, 3) columns [x, y, yaw].  Appends speed column."""
    out = np.column_stack([waypoints, np.full(len(waypoints), speed)])
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savetxt(path, out, delimiter=',',
               header='x,y,yaw,v', comments='', fmt='%.6f')
    print(f'Wrote {len(out)} waypoints → {path}')


def gen_circle(
    radius: float = 5.0,
    n_points: int = 100,
    speed: float = 0.8,
    path: str = '../waypoints/example_circle.csv',
) -> None:
    """
    Clockwise circle in XY plane, centred at (0, -radius).
    The car starts at the origin heading East (+X) — matches raw /odom
    frame where startup pose is (0, 0, yaw=0).
    """
    thetas = np.linspace(0, 2 * math.pi, n_points, endpoint=False)
    xs   = radius * np.sin(thetas)
    ys   = -radius * (1.0 - np.cos(thetas))
    yaws = -thetas                                 # tangent = -theta
    _write_csv(np.column_stack([xs, ys, yaws]), path, speed)


def gen_oval(
    length: float = 8.0,
    width: float = 4.0,
    n_points: int = 150,
    speed: float = 0.8,
    path: str = '../waypoints/example_oval.csv',
) -> None:
    """
    Oval (two straights + two semicircles).
    Total perimeter ≈ 2*length + 2*pi*(width/2)
    """
    R = width / 2.0
    # Split points proportionally to arc length
    straight_len   = length
    arc_len        = math.pi * R
    total          = 2 * straight_len + 2 * arc_len
    n_straight     = max(2, int(n_points * straight_len / total))
    n_arc          = max(2, int(n_points * arc_len    / total))

    pts = []

    # Bottom straight: (0,0) → (length,0), heading East
    for x in np.linspace(0, length, n_straight, endpoint=False):
        pts.append((x, 0.0, 0.0))

    # Right arc: centre (length, R), from South to North
    for t in np.linspace(-math.pi / 2, math.pi / 2, n_arc, endpoint=False):
        pts.append((length + R * math.cos(t),
                    R + R * math.sin(t),
                    t + math.pi / 2))

    # Top straight: (length, width) → (0, width), heading West
    for x in np.linspace(length, 0, n_straight, endpoint=False):
        pts.append((x, width, math.pi))

    # Left arc: centre (0, R), from North to South
    for t in np.linspace(math.pi / 2, 3 * math.pi / 2, n_arc, endpoint=False):
        pts.append((R * math.cos(t),
                    R + R * math.sin(t),
                    t + math.pi / 2))

    wp = np.array(pts)
    wp[:, 2] = (wp[:, 2] + math.pi) % (2 * math.pi) - math.pi
    _write_csv(wp, path, speed)


def gen_figure8(
    radius: float = 4.0,
    n_points: int = 200,
    speed: float = 0.8,
    path: str = '../waypoints/example_figure8.csv',
) -> None:
    """Figure-8 using a lemniscate-like parametrisation."""
    t = np.linspace(0, 2 * math.pi, n_points, endpoint=False)
    xs   = radius * np.sin(t)
    ys   = radius * np.sin(t) * np.cos(t)
    # yaw from finite differences
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    yaws = np.arctan2(dy, dx)
    _write_csv(np.column_stack([xs, ys, yaws]), path, speed)


# ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--shape',  default='circle',
                        choices=['circle', 'oval', 'figure8'])
    parser.add_argument('--output', default='',
                        help='Output CSV path (default: auto)')
    parser.add_argument('--speed',  type=float, default=0.8)
    args = parser.parse_args()

    base = os.path.join(os.path.dirname(__file__), '..', 'waypoints')

    if args.shape == 'circle':
        out = args.output or os.path.join(base, 'example_circle.csv')
        gen_circle(speed=args.speed, path=out)
    elif args.shape == 'oval':
        out = args.output or os.path.join(base, 'example_oval.csv')
        gen_oval(speed=args.speed, path=out)
    elif args.shape == 'figure8':
        out = args.output or os.path.join(base, 'example_figure8.csv')
        gen_figure8(speed=args.speed, path=out)
