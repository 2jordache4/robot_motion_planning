import math

import numpy as np
import scipy as sp
from matplotlib import cm
from matplotlib import pyplot as plt

import me570_geometry as gm
import me570_potential as pot


def distance(points, center, radius):
    d_points_sphere = np.linalg.norm(points - center, axis=0) - abs(radius)
    if (radius < 0):
        d_points_sphere *= -1
    return d_points_sphere


def distance_and_grad_distTesting():

    points = np.array([[0, 2, 4], [0, 2, 4]])
    center = np.array([[1], [1]])
    radius = 2

    d = distance(points, center, radius)
    print(d)
    l2_norm = np.linalg.norm(points - center, axis=0)
    center_diff = points - center
    print(center_diff)
    print(l2_norm)
    print(center_diff / l2_norm)


def world_plotTesting():
    fig, ax = plt.subplots(1)
    ax.set_aspect('equal', adjustable='box')
    world = pot.SphereWorld()
    world.plot()
    plt.show()


def test_fieldPlot():
    fig, ax = plt.subplots(1)
    ax.set_aspect('equal', adjustable='box')

    sphere_world = pot.SphereWorld()
    x_goal = sphere_world.x_goal[:, 1]
    x_goal = np.reshape(x_goal, (2, 1))
    potential = {
        'x_goal': x_goal,
        'repulsive_weight': 1 / 4,
        'shape': 'quadratic'
    }
    #potential = {'x_goal': x_goal, 'repulsive_weight': .01, 'shape': 'conic'}
    attr = pot.Attractive(potential)
    #gm.field_plot_threshold(attr.eval)

    for sphere in sphere_world.world:
        repul = pot.RepulsiveSphere(sphere)
        #gm.field_plot_threshold(repul.eval,threshold = 100,nb_grid=150)
        #plt.show()

    clfcbf_grad = lambda x_eval: pot.clfcbf_control(x_eval, sphere_world.world,
                                                    potential)
    nb_steps = 250
    planned_parameters = {
        'U': attr.eval,
        'control': clfcbf_grad,
        'epsilon': .06,
        'nb_steps': nb_steps
    }

    total = pot.Total(sphere_world, potential)

    #gm.field_plot_threshold(attr.eval,threshold = 300,nb_grid=100)
    sphere_world.plot(ax)
    gm.field_plot_threshold(clfcbf_grad, threshold=300, nb_grid=60)

    plt.xlim([-11, 11])
    plt.ylim([-11, 11])
    plt.show()


def test_runPlanner():
    planner = pot.Planner()
    planner.run_plot()


def test_distance():
    sphere_world = pot.SphereWorld()
    sphere1 = sphere_world.world[0]
    gm.field_plot_threshold(sphere1.distance)
    plt.show()


def test_distanceGrad():
    sphere_world = pot.SphereWorld()
    sphere1 = sphere_world.world[0]
    gm.field_plot_threshold(sphere1.distance_grad)
    plt.show()


#test_distanceGrad()

#test_distance()
#test_runPlanner()
test_fieldPlot()

#sphere_world = pot.SphereWorld()
#for sphere in sphere_world.world:
#    gm.field_plot_threshold(sphere.distance_grad,threshold=300)
#    plt.show()
'''
fig, ax = plt.subplots(1)
ax.set_aspect('equal', adjustable = 'box')
world = pot.SphereWorld()
sphere = world.world[1]
sphere.plot('b',ax)
plt.xlim([-11, 11])
plt.ylim([-11, 11])
plt.show()
'''
