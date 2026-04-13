#!/usr/bin/env python3
"""
Stage 2 verification: MPCSolver standalone tests.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

from mpc_controller.mpc_solver import MPCSolver
from f1tenth_utils.constants import WHEELBASE


def make_solver():
    return MPCSolver(
        v=0.8,
        L=WHEELBASE,
        dt=0.1,
        N=12,
        Q=np.diag([14.0, 8.0]),
        R=np.array([[0.08]]),
        Rd=np.array([[1.2]]),
        delta_max=0.40,
        ddelta_max=0.08,
    )


def test_init():
    solver = make_solver()
    assert solver.problem is not None
    assert solver.A_nominal.shape == (2, 2)
    assert solver.B_nominal.shape == (2, 1)
    print('[PASS] Test 1: solver initialises correctly')
    return solver


def test_zero_error(solver):
    delta, status, _ = solver.solve(np.array([0.0, 0.0]), delta_prev=0.0, speed=0.8)
    assert status in ('optimal', 'optimal_inaccurate'), f'status={status}'
    assert abs(delta) < 1e-3, f'Expected delta≈0, got {delta:.6f}'
    print(f'[PASS] Test 2: zero error → delta={delta:.6f} rad')


def test_positive_ey(solver):
    delta, status, _ = solver.solve(np.array([0.20, 0.0]), delta_prev=0.0, speed=0.8)
    assert status in ('optimal', 'optimal_inaccurate'), f'status={status}'
    assert delta < 0, f'Expected delta < 0, got {np.degrees(delta):.2f} deg'
    print(f'[PASS] Test 3: e_y=+0.20 m → delta={np.degrees(delta):+.2f} deg')


def test_positive_epsi(solver):
    delta, status, _ = solver.solve(np.array([0.0, 0.15]), delta_prev=0.0, speed=0.8)
    assert status in ('optimal', 'optimal_inaccurate'), f'status={status}'
    assert delta < 0, f'Expected delta < 0, got {np.degrees(delta):.2f} deg'
    print(f'[PASS] Test 4: e_psi=+0.15 rad → delta={np.degrees(delta):+.2f} deg')


def test_solve_time(solver):
    times = []
    for ey in np.linspace(-0.3, 0.3, 10):
        _, _, ms = solver.solve(np.array([ey, 0.0]), delta_prev=0.0, speed=0.8)
        times.append(ms)

    avg_ms = np.mean(times)
    max_ms = np.max(times)
    assert max_ms < 50.0, f'Solve too slow: max={max_ms:.1f} ms'
    print(f'[PASS] Test 5: solve time avg={avg_ms:.1f} ms max={max_ms:.1f} ms')


def test_trajectory_reduces_error(solver):
    x0 = np.array([0.3, 0.1])
    _, status, _ = solver.solve(x0, delta_prev=0.0, speed=0.8)
    assert status in ('optimal', 'optimal_inaccurate'), f'status={status}'

    A, B = solver._compute_dynamics(0.8)
    x = x0.copy()
    for k in range(solver.N):
        x = A @ x + B.flatten() * solver.U.value[0, k]

    err0 = float(np.linalg.norm(x0))
    errN = float(np.linalg.norm(x))
    assert errN < err0, f'Error should decrease: |x0|={err0:.4f} → |xN|={errN:.4f}'
    print(f'[PASS] Test 6: error {err0:.4f} → {errN:.4f}')


def test_steering_slew_limit(solver):
    delta, status, _ = solver.solve(np.array([0.4, 0.2]), delta_prev=0.30, speed=0.8)
    assert status in ('optimal', 'optimal_inaccurate'), f'status={status}'
    assert abs(delta - 0.30) <= solver.ddelta_max + 1e-4, (
        f'slew violation: prev=0.30, delta={delta:.4f}, max_step={solver.ddelta_max:.4f}'
    )
    print(f'[PASS] Test 7: slew-limited delta={np.degrees(delta):+.2f} deg')


def test_speed_parameter_changes_dynamics(solver):
    _, status_slow, _ = solver.solve(
        np.array([0.2, 0.1]), delta_prev=0.0, speed=0.5
    )
    b_slow = solver.B_param.value.copy()
    _, status_fast, _ = solver.solve(
        np.array([0.2, 0.1]), delta_prev=0.0, speed=1.5
    )
    b_fast = solver.B_param.value.copy()
    assert status_slow in ('optimal', 'optimal_inaccurate')
    assert status_fast in ('optimal', 'optimal_inaccurate')
    assert not np.allclose(b_slow, b_fast), 'Speed parameter should update the dynamics matrices'
    print(
        '[PASS] Test 8: speed-aware dynamics '
        f'(B[1,0]={b_slow[1,0]:.4f} @0.5 m/s vs {b_fast[1,0]:.4f} @1.5 m/s)'
    )


if __name__ == '__main__':
    print('=' * 55)
    print('Stage 2: MPC Solver Tests')
    print('=' * 55)

    try:
        solver = test_init()
        test_zero_error(solver)
        test_positive_ey(solver)
        test_positive_epsi(solver)
        test_solve_time(solver)
        test_trajectory_reduces_error(solver)
        test_steering_slew_limit(solver)
        test_speed_parameter_changes_dynamics(solver)
        print('=' * 55)
        print('ALL TESTS PASSED — Stage 2 complete')
        print('=' * 55)
        sys.exit(0)
    except AssertionError as e:
        print(f'[FAIL] {e}')
        sys.exit(1)
