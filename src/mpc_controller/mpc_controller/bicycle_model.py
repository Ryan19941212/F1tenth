"""
Linearized Kinematic Bicycle Model for MPC.

State:  s = [x, y, psi]     (position x, position y, heading)
Input:  u = [delta]          (front steering angle, radians)

Discrete-time nonlinear dynamics (Euler):
    x[k+1]   = x[k]   + v * cos(psi[k]) * dt
    y[k+1]   = y[k]   + v * sin(psi[k]) * dt
    psi[k+1] = psi[k] + v / L * tan(delta[k]) * dt

Linearized around reference point (s_ref, u_ref):
    s[k+1] ≈ A * s[k] + B * u[k] + c

where c = f(s_ref, u_ref) - A @ s_ref - B @ u_ref
"""

import numpy as np


class BicycleModel:
    """
    Kinematic bicycle model with linearization support.

    Parameters
    ----------
    wheelbase : float
        Distance between front and rear axles [m].  F1Tenth = 0.3302 m
    dt : float
        Discretization timestep [s].
    v : float
        Constant longitudinal speed [m/s].
    """

    # Safety limits (hardware)
    DELTA_MAX = 0.40   # rad  (~23 deg) — conservative for real car
    DELTA_MIN = -0.40

    def __init__(self, wheelbase: float = 0.3302, dt: float = 0.1, v: float = 1.5):
        self.L  = wheelbase
        self.dt = dt
        self.v  = v
        self.n_states = 3   # [x, y, psi]
        self.n_inputs = 1   # [delta]

    # ------------------------------------------------------------------
    # Nonlinear forward simulation (for testing / reference generation)
    # ------------------------------------------------------------------
    def step(self, s: np.ndarray, u: float) -> np.ndarray:
        """
        Simulate one step with the nonlinear model.

        Parameters
        ----------
        s : np.ndarray, shape (3,)   [x, y, psi]
        u : float                    steering angle delta [rad]

        Returns
        -------
        s_next : np.ndarray, shape (3,)
        """
        u = np.clip(u, self.DELTA_MIN, self.DELTA_MAX)
        x, y, psi = s
        x_next   = x   + self.v * np.cos(psi) * self.dt
        y_next   = y   + self.v * np.sin(psi) * self.dt
        psi_next = psi + self.v / self.L * np.tan(u) * self.dt
        return np.array([x_next, y_next, psi_next])

    # ------------------------------------------------------------------
    # Linearization around a reference point
    # ------------------------------------------------------------------
    def linearize(
        self,
        s_ref: np.ndarray,
        u_ref: float,
    ):
        """
        Compute (A, B, c) for the linearized model at (s_ref, u_ref).

        Returns
        -------
        A : np.ndarray, shape (3, 3)
        B : np.ndarray, shape (3, 1)
        c : np.ndarray, shape (3,)   — affine offset
        """
        psi_r   = s_ref[2]
        delta_r = np.clip(u_ref, self.DELTA_MIN, self.DELTA_MAX)
        v, L, dt = self.v, self.L, self.dt

        # Jacobian w.r.t. state
        A = np.array([
            [1.0,  0.0, -v * np.sin(psi_r) * dt],
            [0.0,  1.0,  v * np.cos(psi_r) * dt],
            [0.0,  0.0,  1.0                    ],
        ])

        # Jacobian w.r.t. input
        cos2_delta = np.cos(delta_r) ** 2
        # Guard against cos(delta) ≈ 0 (shouldn't happen within ±40 deg)
        cos2_delta = max(cos2_delta, 1e-6)
        B = np.array([
            [0.0],
            [0.0],
            [v * dt / (L * cos2_delta)],
        ])

        # Affine correction: c = f(s_ref, u_ref) - A @ s_ref - B @ u_ref
        f_ref = self.step(s_ref, delta_r)
        c = f_ref - A @ s_ref - B.flatten() * delta_r

        return A, B, c

    # ------------------------------------------------------------------
    # Angle wrapping utility
    # ------------------------------------------------------------------
    @staticmethod
    def wrap_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi
