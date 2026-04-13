"""
Linear MPC solver for the F1Tenth lateral error model.

State:   x = [e_y, e_psi]
Control: u = [delta]

The solver supports:
  - parameterized speed-dependent dynamics
  - steering magnitude limits
  - steering slew-rate limits
  - steering slew-rate cost for smoother commands
"""

import time
from typing import Tuple

import cvxpy as cp
import numpy as np

try:
    from scipy.linalg import solve_discrete_are as _dare
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


class MPCSolver:
    """Builds one CVXPY QP and updates parameters at runtime."""

    def __init__(
        self,
        v: float,
        L: float,
        dt: float,
        N: int,
        Q: np.ndarray,
        R: np.ndarray,
        Rd: np.ndarray | None = None,
        delta_max: float = 0.40,
        ddelta_max: float | None = None,
    ):
        self.v = float(v)
        self.L = float(L)
        self.dt = float(dt)
        self.N = int(N)
        self.n = 2
        self.m = 1
        self.delta_max = float(delta_max)
        self.ddelta_max = None if ddelta_max is None else float(ddelta_max)

        self.Q = np.asarray(Q, dtype=float)
        self.R = np.asarray(R, dtype=float)
        self.Rd = None if Rd is None else np.asarray(Rd, dtype=float)

        A0, B0 = self._compute_dynamics(self.v)
        self.A_nominal = A0
        self.B_nominal = B0
        self.P = self._compute_terminal_cost(A0, B0, self.Q, self.R)

        self._build_problem()
        self._warm_start()

    def _compute_dynamics(self, speed: float) -> Tuple[np.ndarray, np.ndarray]:
        v = float(speed)
        A = np.array([
            [1.0, v * self.dt],
            [0.0, 1.0],
        ])
        B = np.array([
            [0.0],
            [v * self.dt / self.L],
        ])
        return A, B

    def _compute_terminal_cost(
        self,
        A: np.ndarray,
        B: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
    ) -> np.ndarray:
        if _HAS_SCIPY:
            try:
                P = _dare(A, B, Q, R)
                return (P + P.T) / 2.0
            except Exception:
                pass
        return Q * float(self.N)

    def _build_problem(self) -> None:
        n, m, N = self.n, self.m, self.N

        self.X = cp.Variable((n, N + 1))
        self.U = cp.Variable((m, N))

        self.x0_param = cp.Parameter(n)
        self.u_prev_param = cp.Parameter(m)
        self.A_param = cp.Parameter((n, n))
        self.B_param = cp.Parameter((n, m))

        cost = 0.0
        constraints = [self.X[:, 0] == self.x0_param]

        for k in range(N):
            cost += cp.quad_form(self.X[:, k], self.Q)
            cost += cp.quad_form(self.U[:, k], self.R)

            if self.Rd is not None:
                if k == 0:
                    du = self.U[:, k] - self.u_prev_param
                else:
                    du = self.U[:, k] - self.U[:, k - 1]
                cost += cp.quad_form(du, self.Rd)
                if self.ddelta_max is not None:
                    constraints.append(du[0] <= self.ddelta_max)
                    constraints.append(du[0] >= -self.ddelta_max)

            constraints.append(
                self.X[:, k + 1] == self.A_param @ self.X[:, k] + self.B_param @ self.U[:, k]
            )
            constraints.append(self.U[0, k] <= self.delta_max)
            constraints.append(self.U[0, k] >= -self.delta_max)

        cost += cp.quad_form(self.X[:, N], self.P)
        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def _warm_start(self) -> None:
        self.x0_param.value = np.zeros(self.n)
        self.u_prev_param.value = np.zeros(self.m)
        self.A_param.value = self.A_nominal
        self.B_param.value = self.B_nominal
        self.problem.solve(
            solver=cp.OSQP,
            warm_start=True,
            eps_abs=1e-4,
            eps_rel=1e-4,
            verbose=False,
        )

    def solve(
        self,
        x0: np.ndarray,
        delta_prev: float = 0.0,
        speed: float | None = None,
    ) -> Tuple[float, str, float]:
        """
        Solve MPC for the given state.

        Parameters
        ----------
        x0 : np.ndarray
            Initial error state [e_y, e_psi].
        delta_prev : float
            Previously applied steering angle [rad].
        speed : float | None
            Speed used in the prediction model. Falls back to nominal speed.
        """
        model_speed = self.v if speed is None else max(float(speed), 1e-3)
        A, B = self._compute_dynamics(model_speed)

        self.x0_param.value = np.asarray(x0, dtype=float)
        self.u_prev_param.value = np.array([float(delta_prev)], dtype=float)
        self.A_param.value = A
        self.B_param.value = B

        t0 = time.perf_counter()
        try:
            self.problem.solve(
                solver=cp.OSQP,
                warm_start=True,
                eps_abs=1e-4,
                eps_rel=1e-4,
                max_iter=4000,
                polish=False,
                verbose=False,
            )
        except cp.SolverError as exc:
            return 0.0, f"SolverError:{exc}", 0.0

        solve_ms = (time.perf_counter() - t0) * 1e3

        if self.problem.status in ("optimal", "optimal_inaccurate") and self.U.value is not None:
            delta = float(self.U.value[0, 0])
            delta = float(np.clip(delta, -self.delta_max, self.delta_max))
            return delta, self.problem.status, solve_ms

        return 0.0, self.problem.status or "unknown", solve_ms

    @property
    def predicted_trajectory(self) -> np.ndarray:
        if self.X.value is None:
            return np.zeros((self.n, self.N + 1))
        return self.X.value
