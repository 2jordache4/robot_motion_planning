"""
Classes to define potential and potential planner for the sphere world
"""

import numpy as np
from matplotlib import pyplot as plt
from scipy import io as scio
import math

import me570_geometry


class SphereWorld:
    """ Class for loading and plotting a 2-D sphereworld. """

    def __init__(self):
        """
        Load the sphere world from the provided file sphereworld.mat, and sets the
    following attributes:
     -  world: a  nb_spheres list of  Sphere objects defining all the spherical obstacles in the
    sphere world.
     -  x_start, a [2 x nb_start] array of initial starting locations (one for each column).
     -  x_goal, a [2 x nb_goal] vector containing the coordinates of different goal locations (one
    for each column).
        """
        data = scio.loadmat(
            '/Users/jordancousineau/Desktop/Graduate/Fall/ME570/Homework/homework3(17)/sphereworld.mat'
        )

        self.world = []
        for sphere_args in np.reshape(data['world'], (-1, )):
            sphere_args[1] = sphere_args[1].item()
            sphere_args[2] = sphere_args[2].item()
            self.world.append(me570_geometry.Sphere(*sphere_args))

        self.x_goal = data['xGoal']
        self.x_start = data['xStart']
        self.theta_start = data['thetaStart']

    def plot(self, axes=None):
        """
        Uses Sphere.plot to draw the spherical obstacles together with a  * marker at the goal
        location.
        """

        if axes is None:
            axes = plt.gca()

        for sphere in self.world:
            sphere.plot('r')

        plt.scatter(self.x_goal[0, :], self.x_goal[1, :], c='g', marker='*')

        plt.xlim([-11, 11])
        plt.ylim([-11, 11])
        plt.axis('equal')


class RepulsiveSphere:
    """ Repulsive potential for a sphere """

    def __init__(self, sphere):
        """
        Save the arguments to internal attributes
        """
        self.sphere = sphere

    def eval(self, x_eval):
        """s
        Evaluate the repulsive potential from  sphere at the location x= x_eval. The function returns
    the repulsive potential as given by      (  eq:repulsive  ).
        """
        distance = self.sphere.distance(x_eval)

        distance_influence = self.sphere.distance_influence
        if (distance > distance_influence):
            u_rep = 0
        elif (distance_influence > distance > 0):
            u_rep = ((distance**-1 - distance_influence**-1)**2) / 2.
            u_rep = u_rep.item()
        else:
            u_rep = math.nan
        return u_rep

    def grad(self, x_eval):
        """
        Compute the gradient of U_ rep for a single sphere, as given by (eq:repulsive-gradient).
        """
        #expression for grad repulsive potential is
        #{ -( (1/d_xi) - (1/grad_d_xi) ) * 1/d_xi^2*grad_d_xi if 0 < d_xi < d_influence
        #{ 0 if d_xi > grad_d_xi
        #{ infinite otherwise
        # distance influence is a class member
        d_xi = self.sphere.distance(x_eval)
        grad_d_xi = self.sphere.distance_grad(x_eval)
        dist_inf = self.sphere.distance_influence
        if (d_xi > dist_inf):
            grad_u_rep = np.zeros((2, 1))
        elif (0 < d_xi < dist_inf):
            grad_u_rep = -(1 / d_xi - 1 / dist_inf) * (1 / d_xi**2) * grad_d_xi
        else:
            grad_u_rep = np.full((2, 1), np.nan)
        return grad_u_rep


class Attractive:
    """ Repulsive potential for a sphere """

    def __init__(self, potential):
        """
        Save the arguments to internal attributes
        """
        self.potential = potential

    def eval(self, x_eval):
        """
        Evaluate the attractive potential  U_ attr at a point  xEval with respect to a goal location
    potential.xGoal given by the formula: If  potential.shape is equal to  'conic', use p=1. If
    potential.shape is equal to  'quadratic', use p=2.
        """
        x_goal = self.potential['x_goal']
        shape = self.potential['shape']
        if (shape == 'conic'):
            expo = 1
        else:
            expo = 2
        u_attr = np.linalg.norm(x_eval - x_goal)**expo
        return u_attr

    def grad(self, x_eval):
        """
        Evaluate the gradient of the attractive potential  U_ attr at a point  xEval. The gradient
        is given by the formula If  potential['shape'] is equal to 'conic', use p=1; if it is
        equal to 'quadratic', use p=2.
        """
        # For p = 1, (1/norm(x-x_goal)*(x-x_goal))
        # For p = 2, 2(x-x_goal)
        shape = self.potential['shape']
        x_goal = self.potential['x_goal']
        if (shape == 'conic'):
            grad_u_attr = (1 / np.linalg.norm(x_eval - x_goal)) * (x_eval -
                                                                   x_goal)
        else:
            grad_u_attr = 2 * (x_eval - x_goal)

        return grad_u_attr


class Total:
    """ Combines attractive and repulsive potentials """

    def __init__(self, world, potential):
        """
        Save the arguments to internal attributes
        """
        self.world = world
        self.potential = potential

    def eval(self, x_eval):
        """
        Compute the function U=U_attr+a*iU_rep,i, where a is given by the variable
    potential.repulsiveWeight
        """
        # U = U_attr + alpha*Sum(U_rep,i), where alpha = potential.repulsiveWeight
        alpha = self.potential['repulsive_weight']
        attra = Attractive(self.potential)
        U_repi = 0
        for sphere in self.world.world:
            U_repi = U_repi + RepulsiveSphere(sphere).eval(x_eval)
        U_attr = attra.eval(x_eval)
        u_eval = U_attr + alpha * U_repi
        return u_eval

    def grad(self, x_eval):
        """
        Compute the gradient of the total potential,  U=U_ attr+a*U_rep,i, where a is given by
        the variable  potential.repulsiveWeight
        """
        alpha = self.potential['repulsive_weight']
        U_repi = np.zeros((2, 1))
        attra = Attractive(self.potential)

        for sphere in self.world.world:
            U_repi = U_repi + RepulsiveSphere(sphere).grad(x_eval)

        grad_u_eval = attra.grad(x_eval) + alpha * U_repi

        return grad_u_eval


class Planner:
    """
    A class implementing a generic potential planner and plot the results.
    """

    def __init__(self, function, control, epsilon, nb_steps):
        """
        Save the arguments to internal attributes
        """
        self.function = function  # computing value of potential function
        self.control = control  # compute direction (negative gradient of the potential function)
        self.epsilon = epsilon  # value for step size
        self.nb_steps = nb_steps  # total number of steps

    def run(self, x_start):
        """
        This function uses a given function (given by  control) to implement a
        generic potential-based planner with step size  epsilon, and evaluates
        the cost along the returned path. The planner must stop when either the
        number of steps given by  nb_stepsis reached, or when the norm of the
        vector given by  control is less than 5 10^-3 (equivalently,  5e-3).
        """
        nb_steps = self.nb_steps
        epsilon = self.epsilon
        x_path = np.zeros((2, nb_steps))
        x_path[:, 0] = x_start.T
        u_path = np.zeros((1, nb_steps))


        # u_path[0] = self.function(x_start)
        for i in range(nb_steps - 1):
            x_i = np.vstack(x_path[:, i])
            control_i = self.control(x_i)
            print(control_i)
            vec_norm = np.linalg.norm(control_i)
            print(vec_norm)

            if (vec_norm < 5e-3):
                x_path[:, i] = np.nan
                u_path[i] = np.nan
            else:
                var = epsilon * control_i
                x_path[:, i + 1] = (x_i + var).T

                u_path[:, i] = self.function(x_i)

        if (vec_norm > 5e-3):
            x_i = np.vstack(x_path[:, nb_steps - 1])
            u_path[:, nb_steps - 1] = self.function(x_i)
        print(x_path)
        print(u_path)

        return x_path, u_path
   

class Clfcbf_Control:
    """
    A class implementing a CLF-CBF-based control framework.
    """

    def __init__(self, world, potential):
        """
        Save the arguments to internal attributes, and create an attribute
        attractive with an object of class  Attractive using the argument
        potential.
        """
        self.world = world
        self.potential = potential
        self.attractive = Attractive(potential)

    def function(self, x_eval):
        """
        Evaluate the CLF (i.e.,  self.attractive.eval()) at the given input.
        """
        return self.attractive.eval(x_eval)

    def control(self, x_eval):
        """
        Compute u^* according to      (  eq:clfcbf-qp  ).
        """
        #u_ref = -c gradV where v is the clf (attractive)
        #a-Barrier = gradient of the distance to the obstacle
        #b_barrier = minimal distance from the object * the repulsion
        c_h = self.potential['repulsive_weight']
        world = self.world
        attra = Attractive(self.potential)
        nb_obstacles = len(world.world)
        a_barrier = np.zeros((nb_obstacles, 2))
        b_barrier = np.zeros((nb_obstacles, 1))
        u_ref = -(attra.grad(x_eval))

        for obst_num, sphere in enumerate(world.world):
            a_barrier[obst_num, :] = -(sphere.distance_grad(x_eval)).T
            b_barrier[obst_num, 0] = -(c_h * sphere.distance(x_eval))

        u_opt = qp_supervisor(a_barrier, b_barrier, u_ref)
        return u_opt


"""
Functions for implementing the Quadratic Programs used for CBFs and CLFs
"""

import warnings

import cvxopt as cvx
import numpy as np

cvx.solvers.options['show_progress'] = False


def qp_supervisor(a_barrier, b_barrier, u_ref=None, solver='cvxopt'):
    """
    Solves the QP min_u ||u-u_ref||^2 subject to a_barrier*u+b_barrier<=0
    For the list of supported solvers, see https://pypi.org/project/qpsolvers/
    """
    dim = 2
    threshold_a_barrier = 1e-5

    if u_ref is None:
        u_ref = np.zeros((dim, 1))
    p_qp = cvx.matrix(np.eye(2))
    q_qp = cvx.matrix(-u_ref)
    if a_barrier is None:
        g_qp = None
    else:
        g_qp = cvx.matrix(np.double(a_barrier))
    if b_barrier is None:
        h_qp = None
    else:
        h_qp = -cvx.matrix(np.double(b_barrier))
    if a_barrier is not None:
        # Check norm of rows of a_barrier
        a_barrier_norms = np.sqrt(sum(a_barrier.transpose()**2))
        if any(a_barrier_norms < threshold_a_barrier):
            warnings.warn(
                'At least one of the rows of ABarrier has small norm. The results of the QP solver might be inaccurate.',
                RuntimeWarning)
    solution = cvx.solvers.qp(p_qp, q_qp, G=g_qp, h=h_qp, solver=solver)
    return np.array(solution['x'])


def qp_supervisor_test():
    """
    Simple test showing how to use the function qp_supervisor
    """
    a_barrier = np.diag([-1, 1])
    b_barrier = np.zeros((2, 1))
    u_ref = np.ones((2, 1))
    u_opt = qp_supervisor(a_barrier, b_barrier, u_ref)
    u_opt_quadprog = qp_supervisor(a_barrier,
                                   b_barrier,
                                   u_ref,
                                   solver='quadprog')
    u_expected = np.array([[1], [0]])
    print('u_expected')
    print(u_expected)
    print('u_optimal')
    print(u_opt)
    print('u_optimal with another solver')
    print(u_opt_quadprog)

    print('Solving a problem with a non-well-conditioned row,',
          'should issue a warning')
    cond_mat = np.diag([1e-6, 1])
    qp_supervisor(cond_mat @ a_barrier, cond_mat @ b_barrier, u_ref)

    a_barrier = np.array([[1, 0], [-1, 0]])
    b_barrier = np.ones((2, 1))

    print('Trying to solve an infeasible problem ...')
    try:
        qp_supervisor(a_barrier, b_barrier, u_ref)
    except ValueError:
        print('\tas expected, raises a ValueError exception')


# if __name__ == '__main__':
#     qp_supervisor_test()

# center = np.array([[3.75],[0]])
# point = np.array([[-7.5],[1.5]])
# sphere = me570_geometry.Sphere(center, 3,9)
# sphere1 = RepulsiveSphere(sphere)
# # print(sphere.distance_grad(point))
# # print(sphere.distance(point))
# print(sphere1.grad(point))
# planner = Planner(Total.eval,Total.grad,1e-3,100)
# planner.run_plot()
