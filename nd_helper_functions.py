"""
Contains helper functions for obtaining a low-dimensional Bezier curve which approximates a higher-dimensional
one.
"""
import numpy as np
import quadprog

def obtain_pts_time(n_samples, bezier_high_dim):
    ptsTime = [(bezier_high_dim(t / n_samples), t / n_samples) for t in range(n_samples + 1)]
    return ptsTime

def quadprog_solve_qp(P, q, G=None, h=None, C=None, d=None, verbose=False):
    """
    min (1/2)x' P x + q' x
    subject to  G x <= h
    subject to  C x  = d
    """
    qp_G = 0.5 * (P + P.T)  # make sure P is symmetric
    qp_a = -q
    qp_C = None
    qp_b = None
    meq = 0
    if C is not None:
        if G is not None:
            qp_C = -np.vstack([C, G]).T
            qp_b = -np.hstack([d, h])
        else:
            qp_C = -C.transpose()
            qp_b = -d
        meq = C.shape[0]
    elif G is not None:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
    res = quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)
    if verbose:
        return res
    # print('qp status ', res)
    return res[0]

# least square form of ||Ax-b||**2
def to_least_square(A, b):
    return np.dot(A.T, A), -np.dot(A.T, b)

def genCost(variableBezier, ptsTime):
    # first evaluate variableBezier for each time sampled
    allsEvals = [(variableBezier(time), pt) for (pt, time) in ptsTime]
    # then compute the least square form of the cost for each points
    allLeastSquares = [to_least_square(el.B(), -el.c() + pt) for (el, pt) in allsEvals]
    # and finally sum the costs
    Ab = [sum(x) for x in zip(*allLeastSquares)]
    return Ab[0], Ab[1]