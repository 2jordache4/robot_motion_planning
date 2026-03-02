'''
Supporting functions for testing Planner.run
'''

import numpy as np


def planner_function(x_eval):
    '''
    Define a potential as the distance from the origin
    '''
    return np.linalg.norm(x_eval)


def planner_control(x_eval):
    '''
    Define a field using a stable dynamical system
    '''
    a_matrix = np.array([[-0.1, -2], [1, -0.1]])
    return a_matrix @ (x_eval)
