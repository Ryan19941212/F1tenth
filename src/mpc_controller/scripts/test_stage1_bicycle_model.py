#!/usr/bin/env python3
"""
Stage 1 verification: Kinematic Bicycle Model

What this tests:
  1. Nonlinear step() produces physically sensible motion
  2. Linearized (A, B, c) matrices match numerical Jacobian
  3. Linearization error is small near the reference point

Run:
    python3 test_stage1_bicycle_model.py
Expected: all tests PASS, plot shows circular arc
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib
matplotlib.use('Agg')   # headless — saves PNG instead of GUI window
import matplotlib.pyplot as plt

from mpc_controller.bicycle_model import BicycleModel
from f1tenth_utils.constants import WHEELBASE


# ──────────────────────────────────────────────────────────────────────
# Test 1: Straight-line motion (delta = 0)
# ──────────────────────────────────────────────────────────────────────
def test_straight_line():
    model = BicycleModel(v=1.5, dt=0.1)
    s = np.array([0.0, 0.0, 0.0])   # start at origin, heading East
    s_next = model.step(s, u=0.0)
    # Should move ~0.15 m in X, no Y or psi change
    expected_x = 1.5 * 0.1
    assert abs(s_next[0] - expected_x) < 1e-9, f"X mismatch: {s_next[0]:.6f} vs {expected_x:.6f}"
    assert abs(s_next[1]) < 1e-9,              f"Y should be 0, got {s_next[1]:.6f}"
    assert abs(s_next[2]) < 1e-9,              f"psi should be 0, got {s_next[2]:.6f}"
    print("[PASS] Test 1: straight-line motion")


# ──────────────────────────────────────────────────────────────────────
# Test 2: Turning motion (delta != 0 → should curve)
# ──────────────────────────────────────────────────────────────────────
def test_turning():
    model = BicycleModel(wheelbase=WHEELBASE, v=1.5, dt=0.1)
    s = np.array([0.0, 0.0, 0.0])
    # Apply constant steering for 1 second (10 steps)
    delta = 0.2  # radians
    for _ in range(10):
        s = model.step(s, u=delta)
    # After turning, heading should have changed
    psi_expected = 10 * (1.5 / WHEELBASE) * np.tan(delta) * 0.1
    assert abs(s[2] - psi_expected) < 0.05, \
        f"psi mismatch: got {np.degrees(s[2]):.2f} deg, expected {np.degrees(psi_expected):.2f} deg"
    assert s[0] > 0, "Car should have moved forward"
    assert s[1] > 0, "Car should have moved left (positive Y for left turn)"
    print(f"[PASS] Test 2: turning — final psi={np.degrees(s[2]):.2f} deg, pos=({s[0]:.3f}, {s[1]:.3f})")


# ──────────────────────────────────────────────────────────────────────
# Test 3: Linearization accuracy (vs. numerical Jacobian)
# ──────────────────────────────────────────────────────────────────────
def test_linearization_accuracy():
    model = BicycleModel(wheelbase=WHEELBASE, v=1.5, dt=0.1)
    s_ref = np.array([1.0, 0.5, 0.3])
    u_ref = 0.15

    A, B, c = model.linearize(s_ref, u_ref)

    # Numerical Jacobian for A
    eps = 1e-5
    A_num = np.zeros((3, 3))
    for i in range(3):
        s_plus  = s_ref.copy(); s_plus[i]  += eps
        s_minus = s_ref.copy(); s_minus[i] -= eps
        A_num[:, i] = (model.step(s_plus, u_ref) - model.step(s_minus, u_ref)) / (2 * eps)

    # Numerical Jacobian for B
    f_plus  = model.step(s_ref, u_ref + eps)
    f_minus = model.step(s_ref, u_ref - eps)
    B_num = ((f_plus - f_minus) / (2 * eps)).reshape(3, 1)

    A_err = np.max(np.abs(A - A_num))
    B_err = np.max(np.abs(B - B_num))

    assert A_err < 1e-6, f"A Jacobian error too large: {A_err:.2e}"
    assert B_err < 1e-6, f"B Jacobian error too large: {B_err:.2e}"
    print(f"[PASS] Test 3: linearization — A_err={A_err:.2e}, B_err={B_err:.2e}")


# ──────────────────────────────────────────────────────────────────────
# Test 4: Linearized model prediction error (small perturbation)
# ──────────────────────────────────────────────────────────────────────
def test_linearized_prediction():
    model = BicycleModel(wheelbase=WHEELBASE, v=1.5, dt=0.1)
    s_ref = np.array([0.0, 0.0, 0.1])
    u_ref = 0.1

    A, B, c = model.linearize(s_ref, u_ref)

    # Small perturbation from reference
    ds = np.array([0.05, 0.05, 0.02])
    du = 0.02
    s0  = s_ref + ds
    u0  = u_ref + du

    s_nonlinear = model.step(s0, u0)
    s_linear    = A @ s0 + B.flatten() * u0 + c

    error = np.abs(s_nonlinear - s_linear)
    print(f"[INFO] Test 4: linearization prediction error: {error}")
    assert np.max(error) < 0.01, \
        f"Linearized prediction error too large for small perturbation: {error}"
    print(f"[PASS] Test 4: prediction error < 0.01 m/rad for small perturbation")


# ──────────────────────────────────────────────────────────────────────
# Visual check: simulate circular arc and save plot
# ──────────────────────────────────────────────────────────────────────
def plot_circular_arc():
    model = BicycleModel(wheelbase=WHEELBASE, v=1.5, dt=0.1)
    s = np.array([0.0, 0.0, 0.0])
    delta = 0.25  # constant steering

    xs, ys = [s[0]], [s[1]]
    for _ in range(60):   # 6 seconds
        s = model.step(s, u=delta)
        xs.append(s[0])
        ys.append(s[1])

    plt.figure(figsize=(5, 5))
    plt.plot(xs, ys, 'b-o', markersize=3)
    plt.plot(xs[0], ys[0], 'go', markersize=8, label='Start')
    plt.plot(xs[-1], ys[-1], 'ro', markersize=8, label='End')
    plt.axis('equal')
    plt.grid(True)
    plt.title(f'Bicycle Model Arc (v=1.5 m/s, δ={np.degrees(delta):.1f}°)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    out = '/tmp/stage1_arc.png'
    plt.savefig(out, dpi=100, bbox_inches='tight')
    print(f"[INFO] Arc plot saved → {out}")


# ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("=" * 50)
    print("Stage 1: Bicycle Model Tests")
    print("=" * 50)

    try:
        test_straight_line()
        test_turning()
        test_linearization_accuracy()
        test_linearized_prediction()
        plot_circular_arc()
        print("=" * 50)
        print("ALL TESTS PASSED — Stage 1 complete")
        print("Check /tmp/stage1_arc.png for visual verification")
        print("=" * 50)
        sys.exit(0)
    except AssertionError as e:
        print(f"[FAIL] {e}")
        sys.exit(1)
