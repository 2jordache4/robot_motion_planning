"""
MainfileforME570HW2"""

import math

import matplotlib.pyplot as plt
import numpy as np
from scipy import io as scio

import me570_geometry
import me570_robot


def twolink_plot_collision_test():
    """
    Thisfunctiongenerates30randomconfigurations loadsthe pointsvariablefromthefile!70DarkSeaGreen2twolink_testDatamat(providedwiththehomework, andthendisplaytheresultsusing twolink_plotCollisiontoplotthemanipulatorinredifitisincollision andgreenotherwise
    """
    nb_configurations = 3
    two_link = me570_robot.TwoLink()
    theta_random = 2 * math.pi * np.random.rand(2, nb_configurations)
    test_data = scio.loadmat('twolink_testData.mat')
    obstacle_points = test_data['obstaclePoints']
    plt.plot(obstacle_points[0, :], obstacle_points[1, :], 'r*')
    for i_theta in range(0, nb_configurations):
        theta = theta_random[:, i_theta:i_theta + 1]
        two_link.plot_collision(theta, obstacle_points)

    plt.show()


def grid_eval_example():
    """ Example of the use of Grid.mesh and Grid.eval function"""
    def fun(x_vec):
        return math.sin(x_vec[0])

    example_grid = me570_geometry.Grid(np.linspace(-3, 3), np.linspace(-3, 3))
    fun_eval = example_grid.eval(fun)
    [xx_grid, yy_grid] = example_grid.mesh()
    fig = plt.figure()
    axis = fig.add_subplot(111, projection='3d')
    axis.plot_surface(xx_grid, yy_grid, fun_eval)
    plt.show()


def torus_twolink_plot_jacobian():
    """
    ForeachoneofthecurvesusedinQuestion qtorusDrawChartsCurves dothefollowing
 - UseLinelinspacetocomputethearray thetaPointsforthecurve
 - Foreachoneoftheconfigurationsgivenbythecolumnsof thetaPoints
 - UseTwolinkplottoplotthetwolinkmanipulator
 - UseTwolinkjacobiantocomputethevelocityoftheendeffector andthenusequivertodrawthatvelocityasanarrowstartingfromtheendeffectorsposition   Thefunctionshouldproduceatotaloffourwindows(or alternatively asinglewindowwithfoursubplots, eachwindow(orsubplot showingalltheconfigurationsofthemanipulatorsuperimposedoneachother Youcanusematplotlibpyplotionandinsertatimesleepcommandintheloopfordrawingthemanipulator inordertoobtaina``movielike' presentationofthemotion
    """
    a_line = [None] * 4
    labels = [None] * 4

    a_line[0] = np.array([[3 / 4 * math.pi], [0]])
    labels[0] = '[[3/4*pi],[0]]'

    a_line[1] = np.array([[3 / 4 * math.pi], [3 / 4 * math.pi]])
    labels[1] = '[[3/4*pi],[3/4*pi]]'

    a_line[2] = np.array([[-3 / 4 * math.pi], [3 / 4 * math.pi]])
    labels[2] = '[[-3/4*pi],[3/4*pi]]'

    a_line[3] = np.array([[0], [3 / 4 * math.pi]])
    labels[3] = '[[0],[3/4*pi]]'

    b_line = np.array([[-1], [-1]])

    nb_points = 7
    two_link = me570_robot.TwoLink()
    t_min = 0
    t_max = 1

    for idx_line in range(4):
        theta_points = me570_geometry.line_linspace(a_line[idx_line], b_line,
                                                    t_min, t_max, nb_points)
        vertex_effector_dot = two_link.jacobian(theta_points, a_line[idx_line])

        fig = plt.figure()
        axes = fig.gca()
        for i_theta in range(0, nb_points):
            theta = theta_points[:, i_theta:i_theta + 1]
            two_link.plot(theta, 'black')
            [vertex_effector_transf, _, _] = two_link.kinematic_map(theta)
            axes.quiver(vertex_effector_transf[0],
                        vertex_effector_transf[1],
                        vertex_effector_dot[0, i_theta],
                        vertex_effector_dot[1, i_theta],
                        color='blue',
                        angles='xy',
                        scale_units='xy',
                        scale=2)
        axes.set_title(labels[0])
        axes.set_aspect('equal', adjustable='box')
        plt.xlim([-10, 15])
        plt.ylim([-10, 15])

# 
