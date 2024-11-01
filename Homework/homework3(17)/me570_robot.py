"""
 Please merge the functions and classes from this file with the same file from the previous
 homework assignment
"""
import numpy as np
import matplotlib.pyplot as plt
import me570_potential as pot
import me570_geometry as geometry


class TwoLink:
    """ See description from previous homework assignments. """

    def kinematic_map(self, theta):
        """
        The function returns the coordinate of the end effector, plus the vertices of the links, all
    transformed according to  _1, _2.
        """
        theta1, theta2 = theta
        b_1, b_2 = polygons_generate()  #this is body 1and2
        # t_1 = np.array([[0], [0]])  #translation from w to b1
        t_2 = np.array([[5], [0]])  #translation from w to b2
        r_1 = geometry.rot2d(theta1)  #rotation from w to b1
        r_2 = geometry.rot2d(theta2)  #rotation from b1 to b2
        vertex_effector_transf = np.dot(r_1, np.dot(r_2, t_2) + t_2)
        polygon1_transf = geometry.Polygon(np.dot(r_1, b_1.vertices))
        polygon2_transf = geometry.Polygon(
            np.dot(r_1,
                   np.dot(r_2, b_2.vertices) + t_2))
        # rewrite to match the formula better
        return vertex_effector_transf, polygon1_transf, polygon2_transf

    def plot(self, theta, color):
        """
        This function should use TwoLink.kinematic_map from the previous question together with
        the method Polygon.plot from Homework 1 to plot the manipulator.
        """
        [vertex_effector_transf, polygon1_transf,
         polygon2_transf] = self.kinematic_map(theta)
        print(polygon1_transf)
        print(polygon2_transf)
        polygon1_transf.plot(color)
        polygon2_transf.plot(color)

    def is_collision(self, theta, points):
        """
        For each specified configuration, returns  True if  any of the links of the manipulator
        collides with  any of the points, and  False otherwise. Use the function
        Polygon.is_collision to check if each link of the manipulator is in collision.
        """
        vertex_effector_transf, polygon1_transf, polygon2_transf = self.kinematic_map(
            theta)
        flag_theta = False
        p_1_collide = polygon1_transf.is_collision(points)
        p_2_collide = polygon2_transf.is_collision(points)

        for i in range(len(p_1_collide)):
            if p_1_collide[i] or p_2_collide[i]:
                flag_theta = True
                break

        return flag_theta

    def plot_collision(self, theta, points):
        """
        This function should:
     - Use TwoLink.is_collision for determining if each configuration is a collision or not.
     - Use TwoLink.plot to plot the manipulator for all configurations, using a red color when the
    manipulator is in collision, and green otherwise.
     - Plot the points specified by  points as black asterisks.
        """
        if self.is_collision(theta, points):
            color = 'r'
        else:
            color = 'g'

        self.plot(theta, color)

        for point in points:
            plt.plot(point[0], point[1], 'k')

    def jacobian(self, theta, theta_dot):
        """
        Implement the map for the Jacobian of the position of the end effector with respect to the
        joint angles as derived in Question~ q:jacobian-effector.
        """
        nb_theta = theta.shape[1]
        vertex_effector_dot = np.zeros((2, nb_theta))
        jacobian = np.zeros((2, 2))

        for i in range(nb_theta):
            theta1 = theta[0, i]
            theta2 = theta[
                1,
            ]

            rot_1 = np.array([[np.cos(theta1), -np.sin(theta1)],
                              [np.sin(theta1), np.cos(theta1)]])
            rot_2 = np.array([[np.cos(theta2), -np.sin(theta2)],
                              [np.sin(theta2), np.cos(theta2)]])

            j_11 = (-5 * np.sin(theta1) - 5 * np.sin(theta1 + theta2))
            j_12 = (-5 * np.sin(theta1 + theta2))
            j_21 = (5 * np.cos(theta1) + 5 * np.cos(theta1 + theta2))
            j_22 = (5 * np.cos(theta1 + theta2))
            print(jacobian)
            jacobian[0][0] = j_11
            jacobian[0][1] = j_12
            jacobian[1][0] = j_21
            jacobian[1][1] = j_22
            print(jacobian)

            vertex_effector_dot[:, i] = np.dot(jacobian, theta_dot[:, i])

        return vertex_effector_dot

    def jacobian_matrix(self, theta):
        """
        Compute the matrix representation of the Jacobian of the position of the end effector with
        respect to the joint angles as derived in Question~ q:jacobian-matrix.
        """
        nb_theta = theta.shape[1]
        vertex_effector_dot = np.zeros((2, nb_theta))
        jacobian = np.zeros((2, 2))

        for i in range(nb_theta):
            theta1 = theta[0, i]
            theta2 = theta[
                1,
            ]

            rot_1 = np.array([[np.cos(theta1), -np.sin(theta1)],
                              [np.sin(theta1), np.cos(theta1)]])
            rot_2 = np.array([[np.cos(theta2), -np.sin(theta2)],
                              [np.sin(theta2), np.cos(theta2)]])

            j_11 = (-5 * np.sin(theta1) - 5 * np.sin(theta1 + theta2))
            j_12 = (-5 * np.sin(theta1 + theta2))
            j_21 = (5 * np.cos(theta1) + 5 * np.cos(theta1 + theta2))
            j_22 = (5 * np.cos(theta1 + theta2))
            # print(jacobian)
            jacobian[0][0] = j_11
            jacobian[0][1] = j_12
            jacobian[1][0] = j_21
            jacobian[1][1] = j_22
            # print(jacobian)
            j_theta = jacobian

        return j_theta

    def animate(self, theta):
        """
        Draw the two-link manipulator for each column in theta with 
        a small pause between each drawing operation
        """
        theta_steps = theta.shape[1]
        for i_theta in range(0, theta_steps, 15):
            self.plot(theta[:, [i_theta]], 'k')


"""
Representation of a simple robot used in the assignments (from hw1)
"""


def polygons_add_x_reflection(vertices):
    """
    Given a sequence of vertices, adds other vertices by reflection
    along the x axis
    """
    vertices = np.hstack([vertices, np.fliplr(np.diag([1, -1]).dot(vertices))])
    return vertices


def polygons_generate():
    """
    Generate the polygons to be used for the two-link manipulator
    """
    vertices1 = np.array([[0, 5], [-1.11, -0.511]])
    vertices1 = polygons_add_x_reflection(vertices1)
    vertices2 = np.array([[0, 3.97, 4.17, 5.38, 5.61, 4.5],
                          [-0.47, -0.5, -0.75, -0.97, -0.5, -0.313]])
    vertices2 = polygons_add_x_reflection(vertices2)
    return (geometry.Polygon(vertices1), geometry.Polygon(vertices2))


class TwoLinkPotential:
    """ Combines attractive and repulsive potentials """

    def __init__(self, world, potential):
        """
        Save the arguments to internal attributes
        """
        self.world = world
        self.potential = potential

    def eval(self, theta_eval):
        """
        Compute the potential U pulled back through the kinematic map of the two-link manipulator,
        i.e., U(Wp_eff(theta)), where U is defined as in Question~q:total-potential, and
        Wp_ eff(theta) is the position of the end effector in the world frame as a function of 
        the joint angles   = _1\\ _2.
        """
        robot = TwoLink()
        transf_end_effector, _, _ = robot.kinematic_map(theta_eval)  
        #underscores can be used to ignore the other returned
        total = pot.Total(self.world, self.potential)
        u_eval_theta = total.eval(transf_end_effector)

        return u_eval_theta

    def grad(self, theta_eval):
        """
        Compute the gradient of the potential U pulled back through the kinematic map of the
        two-link manipulator, i.e., grad U(  Wp_ eff(  )).
        """
        robot = TwoLink()
        trans_end_effector, _, _ = robot.kinematic_map(theta_eval)
        total = pot.Total(self.world, self.potential)

        grad_u_eval_theta = (robot.jacobian_matrix(theta_eval).T @ total.grad(trans_end_effector))
        return grad_u_eval_theta

    def run_plot(self, epsilon, nb_steps):
        """
        This function performs the same steps as Planner.run_plot in
        Question~q:potentialPlannerTest, except for the following:
     - In step  it:grad-handle:  planner_parameters['U'] should be set to  @twolink_total, and
    planner_parameters['control'] to the negative of  @twolink_totalGrad.
     - In step  it:grad-handle: Use the contents of the variable  thetaStart instead of  xStart to
    initialize the planner, and use only the second goal  x_goal[:,1].
     - In step  it:plot-plan: Use Twolink.plotAnimate to plot a decimated version of the results of
    the planner. Note that the output  xPath from Potential.planner will really contain a sequence
    of join angles, rather than a sequence of 2-D points. Plot only every 5th or 10th column of
    xPath (e.g., use  xPath(:,1:5:end)). To avoid clutter, plot a different figure for each start.
        """
        sphere_world = pot.SphereWorld()

        nb_starts = sphere_world.theta_start.shape[1]

        planner = pot.Planner(function=self.eval,
                              control=self.grad,
                              epsilon=epsilon,
                              nb_steps=nb_steps)

        two_link = TwoLink()

        for start in range(0, nb_starts):
            # Run the planner
            theta_start = sphere_world.theta_start[:, start]
            theta_path, u_path = planner.run(theta_start)

            # Plots
            _, axes = plt.subplots(ncols=2)
            axes[0].set_aspect('equal', adjustable='box')
            plt.sca(axes[0])
            sphere_world.plot()
            two_link.animate(theta_path)
            axes[1].plot(u_path.T)

world = pot.SphereWorld()
world.world = [
    geometry.Sphere(center=np.array([[0], [-2]]),
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
total = pot.Total(world, potential)
clf = pot.Clfcbf_Control(world,potential)
planner = pot.Planner(
                                function=lambda x: total.eval(x),
                                control= lambda x: clf.control(x),  # Negative gradient for control
                                epsilon=epsilon,
                                nb_steps=nb_steps
                            )
twolink = TwoLinkPotential(world,potential)
twolink.run_plot(epsilon,nb_steps)
plt.show()