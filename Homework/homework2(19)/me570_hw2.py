"""
Main file for ME570 HW2
"""

import math

import matplotlib.pyplot as plt
import numpy as np
from scipy import io as scio

import me570_geometry as geo
import me570_robot as robot


def twolink_plot_collision_test():
    """
    This function generates 30 random configurations, loads the  points variable from the file
    twolink_testData.mat (provided with the homework), and then display the results
    using  twolink_plotCollision to plot the manipulator in red if it is in collision, and green
    otherwise.
    """
    nb_configurations = 7
    two_link = robot.TwoLink()
    theta_random = 2 * math.pi * np.random.rand(2, nb_configurations)
    test_data = scio.loadmat('./Homework/homework2(19)/twolink_testData.mat')
    obstacle_points = test_data['obstaclePoints']
    plt.plot(obstacle_points[0, :], obstacle_points[1, :], 'r*')
    for i_theta in range(0, nb_configurations):
        theta = theta_random[:, i_theta:i_theta + 1]
        two_link.plot_collision(theta, obstacle_points)


def grid_eval_example():
    """ Example of the use of Grid.mesh and Grid.eval functions"""

    def fun(x_vec):
        return math.sin(x_vec[0])

    example_grid = geo.Grid(np.linspace(-3, 3), np.linspace(-3, 3))
    fun_eval = example_grid.eval(fun)
    print(fun_eval)
    [xx_grid, yy_grid] = example_grid.mesh()
    print(xx_grid, yy_grid)
    fig = plt.figure()
    axis = fig.add_subplot(111, projection='3d')
    axis.plot_surface(xx_grid, yy_grid, fun_eval)
    plt.show()


def torus_twolink_plot_jacobian():
    """
    For each one of the curves used in Question~ q:torusDrawChartsCurves, do the following:
 - Use Line.linspace to compute the array  thetaPoints for the curve;
 - For each one of the configurations given by the columns of  thetaPoints:
 - Use Twolink.plot to plot the two-link manipulator.
 - Use Twolink.jacobian to compute the velocity of the end effector, and then use quiver to draw
that velocity as an arrow starting from the end effector's position.   The function should produce a
total of four windows (or, alternatively, a single window with four subplots), each window (or
subplot) showing all the configurations of the manipulator superimposed on each other. You can use
matplotlib.pyplot.ion and insert a time.sleep command in the loop for drawing the manipulator, in
order to obtain a ``movie-like'' presentation of the motion.
    """
    a_lines = [
        np.array([[3 / 4 * math.pi], [0]]),
        np.array([[3 / 4 * math.pi], [3 / 4 * math.pi]]),
        np.array([[-3 / 4 * math.pi], [3 / 4 * math.pi]]),
        np.array([[0], [-3 / 4 * math.pi]])
    ]

    b_line = np.array([[-1], [-1]])

    # axis = geo.gca_3d()
    nb_points = 7
    min = 0
    max = 1
    twolink = robot.TwoLink()
    theta_dot = np.array([[.75],[.5]])
    for idx in range(len(a_lines)):
        theta_points = geo.line_linspace(a_lines[idx], b_line, min, max, nb_points)
        
        for i in range(nb_points):
            theta = theta_points[:,i].reshape(2,1)
            twolink.plot(theta_points[:, i],'g')
            end = twolink.kinematic_map(theta_points[:, i])
            end_position = end[0]
            velocity = twolink.jacobian(theta,theta_dot)
            plt.quiver(end_position[0],end_position[1],velocity[0],velocity[1], angles='xy', scale_units='xy',scale=1,color='r') 
            
             # Compute Jacobian (velocity)
        plt.axis('equal')
        plt.show()

    pass  # Substitute with your code


torus_twolink_plot_jacobian()
# twolink_plot_collision_test()
plt.show()
