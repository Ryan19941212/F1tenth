#!/usr/bin/env python3
"""Benchmark range_libc CPU vs GPU on a real map.

Usage: python3 scripts/bench_rangelibc.py <track_name> [num_particles]
"""
import sys, time
from types import SimpleNamespace
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
import range_libc

WS = Path(__file__).resolve().parent.parent


def make_occgrid(track):
    td = WS / 'maps' / track
    meta = yaml.safe_load(open(td / 'map.yaml'))
    img = np.array(Image.open(td / 'map.pgm'))
    H, W = img.shape
    # slam_toolbox: 255=free, 0=occupied; PyOMap expects >10 = blocked
    data = np.where(img < 205, 100, 0).astype(np.int8).ravel().tolist()
    msg = SimpleNamespace(
        info=SimpleNamespace(
            width=W, height=H,
            resolution=meta['resolution'],
            origin=SimpleNamespace(
                position=SimpleNamespace(x=meta['origin'][0], y=meta['origin'][1]),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=data,
    )
    return msg, meta['resolution'], H, W


def bench(name, rm, queries, angles, ranges, n_iter=50):
    for _ in range(3):
        rm.calc_range_repeat_angles(queries, angles, ranges)
    t0 = time.perf_counter()
    for _ in range(n_iter):
        rm.calc_range_repeat_angles(queries, angles, ranges)
    dt = (time.perf_counter() - t0) / n_iter
    print(f'  {name:28s} {1e3*dt:8.2f} ms/iter   {1.0/dt:8.1f} Hz')
    return dt


def main():
    track = sys.argv[1] if len(sys.argv) > 1 else 'track_20260411_223212'
    N = int(sys.argv[2]) if len(sys.argv) > 2 else 4000
    msg, res, H, W = make_occgrid(track)
    print(f'Map: {track}  {W}x{H}  res={res}')
    print(f'CUDA available: {range_libc.SHOULD_USE_CUDA}')

    omap = range_libc.PyOMap(msg)
    MAX_RANGE_PX = int(8.0 / res)
    NUM_RAYS = 61

    rng = np.random.default_rng(0)
    q = np.column_stack([
        rng.uniform(2, W - 2, N),
        rng.uniform(2, H - 2, N),
        rng.uniform(-np.pi, np.pi, N),
    ]).astype(np.float32)
    angles = np.linspace(-2.35, 2.35, NUM_RAYS).astype(np.float32)
    ranges = np.zeros(N * NUM_RAYS, dtype=np.float32)

    print(f'Particles: {N}  rays: {NUM_RAYS}  max_range_px: {MAX_RANGE_PX}')

    print('\n--- CPU ---')
    rm_cpu = range_libc.PyRayMarching(omap, MAX_RANGE_PX)
    t_cpu = bench('PyRayMarching', rm_cpu, q, angles, ranges)

    print('\n--- GPU ---')
    rm_gpu = range_libc.PyRayMarchingGPU(omap, MAX_RANGE_PX)
    t_gpu = bench('PyRayMarchingGPU', rm_gpu, q, angles, ranges)

    print(f'\nSpeedup GPU/CPU: {t_cpu / t_gpu:.2f}x')
    print(f'At 50 Hz PF loop, CPU budget {1000/50:.1f} ms, GPU uses {1e3*t_gpu:.2f} ms')


if __name__ == '__main__':
    main()
