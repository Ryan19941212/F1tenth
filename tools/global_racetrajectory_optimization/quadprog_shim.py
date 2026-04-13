"""Drop-in replacement for `quadprog.solve_qp` using cvxopt.

The aarch64 PyPI wheel for quadprog 0.1.7 ships with an undefined C++ symbol
(_Z7qpgen2_...), so `import quadprog` crashes on Jetson. TPH only calls
`quadprog.solve_qp(G, a, C, b, meq)` in exactly one place (opt_min_curv.py),
so we install this shim as `sys.modules['quadprog']` before TPH is imported.

quadprog.solve_qp signature:
    minimize     0.5 x^T G x - a^T x
    subject to   C^T x >= b        (first meq rows are equalities)

cvxopt.solvers.qp signature:
    minimize     0.5 x^T P x + q^T x
    subject to   G_cvx x <= h_cvx
                 A_cvx x == b_cvx

Mapping:
    P = G,  q = -a
    equalities  : A_cvx = C[:, :meq].T,  b_cvx = b[:meq]
    inequalities: G_cvx = -C[:, meq:].T, h_cvx = -b[meq:]
"""

import numpy as np
import cvxopt

cvxopt.solvers.options['show_progress'] = False
cvxopt.solvers.options['abstol'] = 1e-8
cvxopt.solvers.options['reltol'] = 1e-7


def _m(a):
    return cvxopt.matrix(np.asarray(a, dtype=float))


def solve_qp(G, a, C=None, b=None, meq=0, factorized=False):
    P = _m(G)
    q = _m(-np.asarray(a, dtype=float))
    args = [P, q]
    if C is not None and C.size > 0:
        C = np.asarray(C, dtype=float)
        b = np.asarray(b, dtype=float)
        G_cvx = -C[:, meq:].T if C.shape[1] > meq else None
        h_cvx = -b[meq:] if b.size > meq else None
        A_cvx = C[:, :meq].T if meq > 0 else None
        b_cvx = b[:meq] if meq > 0 else None
        if G_cvx is not None and G_cvx.size:
            args += [_m(G_cvx), _m(h_cvx)]
        else:
            args += [None, None]
        if A_cvx is not None and A_cvx.size:
            args += [_m(A_cvx), _m(b_cvx)]
    sol = cvxopt.solvers.qp(*args)
    if sol['status'] != 'optimal':
        print(f"[quadprog_shim] cvxopt status: {sol['status']}")
    x = np.array(sol['x']).ravel()
    # quadprog returns (x, f, xu, iters, lagr, iact) — TPH only uses [0]
    return x, 0.0, x, 0, np.zeros(0), np.zeros(0)
