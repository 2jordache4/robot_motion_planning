import matplotlib.pyplot as plt
import numpy as np

import me570_geometry
import me570_potential

# def sphere_test_collision():
#     """
#     Generates one figure with a sphere (with arbitrary parameters) and
#     nb_points=100 random points that are colored according to the sign of
#     their distance from the sphere (red for negative, green for positive).
#     Generates a second figure in the same way (and the same set of points)
#     but flipping the sign of the radius  r of the sphere. For each sampled
#     point, plot also the result of the output  pointsSphere.
#     """
#     pass  # Substitute with your code


def clfcbf_control_test_singlesphere():
    """
    Use the provided function Grid.plot_threshold ( )
    to visualize the CLF-CBF control field for a single filled-in sphere
    """
    # A single sphere whose edge intersects the origin
    world = me570_potential.SphereWorld()
    world.world = [
        me570_geometry.Sphere(center=np.array([[0], [-2]]),
                              radius=2,
                              distance_influence=1)
    ]
    world.x_goal = np.array([[0], [-6]])
    pars = {
        'repulsive_weight': 0.1,
        'x_goal': np.array([[0], [-6]]),
        'shape': 'conic'
    }

    xx_ticks = np.linspace(-10, 10, 23)
    grid = me570_geometry.Grid(xx_ticks, xx_ticks)

    clfcbf = me570_potential.Clfcbf_Control(world, pars)
    plt.figure()
    world.plot()
    grid.plot_threshold(clfcbf.control, 1)


def planner_run_plot_test():
    """
    Show the results of Planner.run_plot for each goal location in
    world.xGoal, and for different interesting combinations of
    potential['repulsive_weight'],  potential['shape'],  epsilon, and
    nb_steps. In each case, for the object of class  Planner should have the
    attribute  function set to  Total.eval, and the attribute  control set
    to the negative of  Total.grad.
    """
    world = me570_potential.SphereWorld()  # Cargar el mundo esférico
    
    epsilon = 1e-2 # Step size values
    repulsive_weight = 0.60  # Repulsive weights
    shape = 'conic' # Potential shapes
    nb_steps = 100  # Number of steps
    plt.rcParams["figure.figsize"] = (8, 8)
    colors = plt.cm.get_cmap('hsv', world.x_start.shape[1] + 1)
    _, axs = plt.subplots(world.x_goal.shape[1], 2)
    plt.subplots_adjust(hspace=0.305)

    # Step 2: Iterate through each combination of epsilon, shape, nb_steps
    for i, x_goal in enumerate(world.x_goal.T):
        potential = {
                            'x_goal': np.vstack(x_goal),
                            'repulsive_weight': repulsive_weight,
                            'shape': shape
                        }
        total = me570_potential.Total(world, potential)
        planner = me570_potential.Planner(
                                function=lambda x: total.eval(x),
                                control= lambda x: -total.grad(x),  # Negative gradient for control
                                epsilon=epsilon,
                                nb_steps=nb_steps
                            )
        for color_num, curr_start in enumerate(world.x_start.T):
                curr_start = np.vstack(curr_start)
                x_path, u_path = planner.run(curr_start)
                # Make sure we are plotting below the world we are concerned with
                plt.sca(axs[i, 0])
                plt.plot(x_path[0, :], x_path[1, :], color=colors(color_num))

                # Plotting the potential on the right-hand subplot
                plt.sca(axs[i, 1])
                plt.title(f"Goal {i}, Potential")
                plt.xlabel('# steps')
                plt.ylabel('U')
                arr = np.arange(0, nb_steps)
                arr = arr.reshape(1,-1)
                new_u = u_path.reshape(nb_steps)
                plt.semilogy(np.arange(0, nb_steps),new_u,
                             color=colors(color_num))
        plt.sca(axs[i, 0])
        plt.title(f"Goal {i}, Quadratic")
        world.plot()



def clfcbf_run_plot_test():
    """
    Use the function Planner.run_plot to run the planner based on the
    CLF-CBF framework, and show the results for one combination of
    repulsive_weight and  epsilon that makes the planner work reliably.
    """
    world = me570_potential.SphereWorld()  # Cargar el mundo esférico
    
    epsilon = 3e-1 # Step size values
    repulsive_weight = 0.10  # Repulsive weights
    shape = 'conic' # Potential shapes
    nb_steps = 100  # Number of steps
    plt.rcParams["figure.figsize"] = (8, 8)
    colors = plt.cm.get_cmap('hsv', world.x_start.shape[1] + 1)
    _, axs = plt.subplots(world.x_goal.shape[1], 2)
    plt.subplots_adjust(hspace=0.305)

    # Step 2: Iterate through each combination of epsilon, shape, nb_steps
    for i, x_goal in enumerate(world.x_goal.T):
        potential = {
                            'x_goal': np.vstack(x_goal),
                            'repulsive_weight': repulsive_weight,
                            'shape': shape
                        }
        total = me570_potential.Total(world, potential)
        potent = me570_potential.Attractive(potential)
        clf = me570_potential.Clfcbf_Control(world,potential)
        planner = me570_potential.Planner(
                                function=lambda x: potent.eval(x),
                                control= lambda x: clf.control(x),  # Negative gradient for control
                                epsilon=epsilon,
                                nb_steps=nb_steps
                            )
        for color_num, curr_start in enumerate(world.x_start.T):
                curr_start = np.vstack(curr_start)
                x_path, u_path = planner.run(curr_start)
                # Make sure we are plotting below the world we are concerned with
                plt.sca(axs[i, 0])
                plt.plot(x_path[0, :], x_path[1, :], color=colors(color_num))

                # Plotting the potential on the right-hand subplot
                plt.sca(axs[i, 1])
                plt.title(f"Goal {i}, Potential")
                plt.xlabel('# steps')
                plt.ylabel('U')
                arr = np.arange(0, nb_steps)
                arr = arr.reshape(1,-1)
                new_u = u_path.reshape(nb_steps)
                plt.semilogy(np.arange(0, nb_steps),new_u,
                             color=colors(color_num))
        plt.sca(axs[i, 0])
        plt.title(f"Goal {i}, CLFCBF")
        world.plot()


# clfcbf_control_test_singlesphere()
# x0plt.show()
# clfcbf_run_plot_test()
# plt.show()

xx_ticks=np.linspace(-10,10,60)
grid = me570_geometry.Grid(xx_ticks,xx_ticks)
# # center = np.ones((2,1))

# # # point = np.array([[-7.5],[1.5]])
# # sphere = me570_geometry.Sphere(center, -2,3)
# # def f_handle(point):
# #     return sphere.distance(point)
world = me570_potential.SphereWorld()
world.world = [
    me570_geometry.Sphere(center=np.array([[0], [-2]]),
                            radius=2,
                            distance_influence=1)
]
epsilon = 1e-2 # Step size values
repulsive_weight = 0.60  # Repulsive weights
shape = 'conic' # Potential shapes
nb_steps = 400  # Number of steps
x_goal = world.x_goal
potential = {
                            'x_goal': np.vstack(x_goal),
                            'repulsive_weight': repulsive_weight,
                            'shape': shape
                        }
total = me570_potential.Total(world, potential)
clf = me570_potential.Clfcbf_Control(world,potential)
planner = me570_potential.Planner(
                                function=lambda x: total.eval(x),
                                control= lambda x: clf.control(x),  # Negative gradient for control
                                epsilon=epsilon,
                                nb_steps=nb_steps
                            )
plt.figure(1)
grid.plot_threshold(planner.function, 10)
plt.title("Total Potential U")
plt.show()

# Plot 2: Gradient of the potential ∇U
plt.figure(2)
grid.plot_threshold(total.grad)  # Gradient magnitude

world.plot()
plt.title("Gradient ∇U of the Potential")
plt.show()
# sphere.plot('k')
# plt.show()
# sphere_world = me570_potential.SphereWorld()
# sphere_world.world[0]
# rep=me570_potential.RepulsiveSphere(sphere_world.world[0])
# sphere_world.world[0].plot('r')
# grid.plot_threshold(rep.grad)
# plt.show()

# planner_run_plot_test()
# plt.show()