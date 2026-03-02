'''
Functions to generate plots for the report
'''

import numpy as np
from matplotlib import pyplot as plt

import me570_geometry as gm
import me570_potential as pot


def repulsive_sphere_grad_test():
    """
    Use the provided function Grid.plot_threshold ( ) to visualize ( ) for the
    first two spheres in the sphere world, and overlap the plot of the sphere
    in each plot.
    """
    plt.close('all')
    xx_ticks = np.linspace(-11, 11, 51)
    grid = gm.Grid(xx_ticks, xx_ticks)
    world = pot.SphereWorld()
    for i_sphere in range(2):
        sphere = world.world[i_sphere]
        repulsive_sphere = pot.RepulsiveSphere(sphere)
        plt.figure()
        sphere.plot('k')
        grid.plot_threshold(repulsive_sphere.grad, 1)


def planner_run_plot_test():
    """
    Show the results of Planner.run_plot for each goal location in
    world.xGoal, and for different interesting combinations of
    potential['repulsive_weight'],  potential['shape'],  epsilon, and
    nb_steps. In each case, for the object of class  Planner should have the
    attribute  function set to  Total.eval, and the attribute  control set
    to the negative of  Total.grad.
    """

    world = pot.SphereWorld()
    nb_goals = world.x_goal.shape[1]
    for pars in [{
            'repulsive_weight': 0.1,
            'shape': 'conic',
            'epsilon': 0.2,
            'nb_steps': 200,
    }, {
            'repulsive_weight': 2,
            'shape': 'quadratic',
            'epsilon': 0.015,
            'nb_steps': 200,
    }]:
        for i_goal in range(nb_goals):
            x_goal = world.x_goal[:, [i_goal]]
            pars['x_goal'] = x_goal
            potential = pot.Total(world, pars)
            planner = pot.Planner(epsilon=pars['epsilon'],
                                  nb_steps=pars['nb_steps'],
                                  function=potential.eval,
                                  control=potential.grad)
            planner.run_plot()


def clfcbf_run_plot_test():
    """
    Show the results of Planner.run_plot for one combination of
    repulsive_weight and  epsilon that makes the planner work reliably.
    """
    plt.close('all')
    plt.ion()
    world = pot.SphereWorld()
    nb_goals = world.x_goal.shape[1]
    pars = {
        'repulsive_weight': 0.1,
        'shape': 'conic',
        'epsilon': 0.3,
        'nb_steps': 150,
    }
    # We need to store the potential objects in a lambda,
    # hence it is necessary to wrap the entire body of the loop
    # in a separate function
    for i_goal in range(nb_goals):
        x_goal = world.x_goal[:, [i_goal]]
        pars['x_goal'] = x_goal

        clfcbf_control = pot.Clfcbf_Control(world, pars)

        planner = pot.Planner(epsilon=pars['epsilon'],
                              nb_steps=pars['nb_steps'],
                              function=clfcbf_control.function,
                              control=clfcbf_control.control)
        planner.run_plot()
