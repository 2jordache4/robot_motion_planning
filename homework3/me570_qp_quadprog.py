"""
Functions for implementing the Quadratic Programs used for CBFs and CLFs
"""

import numpy as np
import quadprog


def qp_supervisor(a_barrier, b_barrier, u_ref=None):
    """
    Solves the QP min_u ||u-u_ref||^2 subject to a_barrier*u+b_barrier<=0
    """
    dim = 2
    if u_ref is None:
        u_ref = np.zeros((dim, 1))
    p_qp = np.eye(2)
    q_qp = -u_ref
    if a_barrier is None:
        g_qp = None
    else:
        g_qp = np.double(a_barrier)
    if b_barrier is None:
        h_qp = None
    else:
        h_qp = -np.double(b_barrier)
    alphas, objective = quadprog.solve_qp(p_qp, q_qp, g_qp, h_qp, [], [])
    breakpoint()
    #solution = cvx.solvers.qp(p_qp, q_qp, G=g_qp, h=h_qp)
    return np.array(solution['x'])


def qp_supervisor_test():
    """
    Simple test showing how to use the function qp_supervisor
    """
    a_barrier = np.diag([-1, 1])
    b_barrier = np.zeros((2, 1))
    u_ref = np.ones((2, 1))
    u_opt = qp_supervisor(a_barrier, b_barrier, u_ref)
    u_expected = np.array([[1], [0]])
    print('u_expected')
    print(u_expected)
    print('u_optimal')
    print(u_opt)

    a_barrier = np.array([[1, 0], [-1, 0]])
    b_barrier = np.ones((2, 1))

    print('Trying to solve an infeasible problem ...')
    try:
        qp_supervisor(a_barrier, b_barrier, u_ref)
    except ValueError:
        print('\tas expected, raises a ValueError exception')


if __name__ == '__main__':
    qp_supervisor_test()
