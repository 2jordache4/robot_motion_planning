"""
 Please merge the functions and classes from this file with the same file from the previous
 homework assignment
"""

import numbers
import math
import numpy as np
from matplotlib import cm
from matplotlib import pyplot as plt
import me570_potential as pot


def numel(var):
    """
    Counts the number of entries in a numpy array, or returns 1 for fundamental
    numerical types
    """
    if isinstance(var, numbers.Number):
        size = int(1)
    elif isinstance(var, np.ndarray):
        size = var.size
    else:
        raise NotImplementedError(f'number of elements for type {type(var)}')
    return size

class Sphere:
    """ Class for plotting and computing distances to spheres (circles, in 2-D). """

    def __init__(self, center, radius, distance_influence):
        """
        Save the parameters describing the sphere as internal attributes.
        """
        self.center = center
        self.radius = radius
        self.distance_influence = distance_influence

    def plot(self, color):
        """
        This function draws the sphere (i.e., a circle) of the given radius, 
        and the specified color, and then draws another circle in gray 
        with radius equal to the distance of influence.
        """
        # Get current axes
        ax = plt.gca()

        # Add circle as a patch
        if self.radius > 0:
            # Circle is filled in
            kwargs = {'facecolor': (0.3, 0.3, 0.3)}
            radius_influence = self.radius + self.distance_influence
        else:
            # Circle is hollow
            kwargs = {'fill': False}
            radius_influence = -self.radius - self.distance_influence

        center = (self.center[0, 0], self.center[1, 0])
        ax.add_patch(
            plt.Circle(center,
                       radius=abs(self.radius),
                       edgecolor=color,
                       **kwargs))

        ax.add_patch(
            plt.Circle(center,
                       radius=radius_influence,
                       edgecolor=(0.7, 0.7, 0.7),
                       fill=False))

    def distance(self, points):
        """
        Computes the signed distance between points and the sphere, while taking
        into account whether the sphere is hollow or filled in.
        """
        # FORMULA: norm(point - center) - abs(r)

        # length = points.shape[1]
        # d_points_sphere = np.zeros(length)
        radius = self.radius
        center = self.center

        # for idx in range(length):
        #     dCenter = np.linalg.norm(points[:, idx] - center)
        #     d_points_sphere[idx] = dCenter - abs(radius)
        d_points_sphere = np.linalg.norm(points - center, axis=0) - abs(radius)
        # instead of a for loop this does all the matrix mult in one go

        if radius < 0:
            d_points_sphere *= -1

        return d_points_sphere

    def distance_grad(self, points):
        """
        Computes the gradient of the signed distance between points and the
        sphere, consistently with the definition of Sphere.distance.
        """
        #FORMULA: (point - center)/norm(point-center)
        length = points.shape[1]
        grad_d_points_sphere = np.zeros((2, length))
        radius = self.radius
        center = self.center
        denominator = np.linalg.norm(points - center, axis=0)
        if not all(denominator):
            # make sure we don't divide by zero
            return np.zeros((2, 1))

        grad_d_points_sphere = (points - center) / np.linalg.norm(
            points - center, axis=0)

        if radius < 0:
            grad_d_points_sphere *= -1

        return grad_d_points_sphere


def clip(val, threshold):
    """
    If val is a scalar, threshold its value; if it is a vector, normalized it
    """
    if isinstance(val, np.ndarray):
        val_norm = np.linalg.norm(val)
        if val_norm > threshold:
            val = val * threshold / val_norm
    elif isinstance(val, numbers.Number):
        if np.isnan(val):
            val = threshold
        else:
            val = min(val, threshold)
    else:
        raise ValueError('Numeric format not recognized')

    return val


"""
 Please merge the functions and classes from this file with the same file from the previous
 homework assignment
"""


def gca_3d():
    """
    Get current Matplotlib axes, and if they do not support 3-D plotting,
    add new axes that support it
    """
    fig = plt.gcf()
    if len(fig.axes) == 0 or not hasattr(plt.gca(), 'plot3D'):
        axis = fig.add_subplot(111, projection='3d')
    else:
        axis = plt.gca()
    return axis


def numel(var):
    """
    Counts the number of entries in a numpy array, or returns 1 for fundamental numerical
    types
    """
    if isinstance(var, numbers.Number):
        size = int(1)
    elif isinstance(var, np.ndarray):
        size = var.size
    else:
        raise NotImplementedError(f'number of elements for type {type(var)}')
    return size


def rot2d(theta):
    """
    Create a 2-D rotation matrix from the angle theta according to (1).
    """
    rot_theta = np.array([[math.cos(theta), -math.sin(theta)],
                          [math.sin(theta), math.cos(theta)]])
    return rot_theta


def line_linspace(a_line, b_line, t_min, t_max, nb_points):
    """
    Generates a discrete number of  nb_points points along the curve
    (t)=( a(1)t + b(1), a(2)t + b(2))  R^2 for t ranging from  tMin to  tMax.
    """
    t_sequence = np.linspace(t_min, t_max, nb_points)
    theta_points = a_line * t_sequence + b_line
    return theta_points


class Grid:
    """
    A function to store the coordinates of points on a 2-D grid and evaluate arbitrary
    functions on those points.
    """

    def __init__(self, xx_grid, yy_grid):
        """
        Stores the input arguments in attributes.
        """
        self.xx_grid = xx_grid
        self.yy_grid = yy_grid

    def eval(self, fun):
        """
        This function evaluates the function  fun (which should be a function)
        on each point defined by the grid.
        """

        dim_domain = [numel(self.xx_grid), numel(self.yy_grid)]
        dim_range = [numel(fun(np.array([[0], [0]])))]
        fun_eval = np.nan * np.ones(dim_domain + dim_range)
        for idx_x in range(0, dim_domain[0]):
            for idx_y in range(0, dim_domain[1]):
                x_eval = np.array([[self.xx_grid[idx_x]],
                                   [self.yy_grid[idx_y]]])
                fun_eval[idx_x, idx_y, :] = np.reshape(fun(x_eval),
                                                       [1, 1, dim_range[0]])

        # If the last dimension is a singleton, remove it
        if dim_range == [1]:
            fun_eval = np.reshape(fun_eval, dim_domain)

        return fun_eval

    def mesh(self):
        """
        Shorhand for calling meshgrid on the points of the grid
        """
        return np.meshgrid(self.xx_grid, self.yy_grid)

    def plot_threshold(self, f_handle, threshold=10):
        """
        The function evaluates the function  f_handle on points placed on the grid.
        """

        def f_handle_clip(val):
            return clip(f_handle(val), threshold)

        f_eval = self.eval(f_handle_clip)

        [xx_mesh, yy_mesh] = self.mesh()
        f_dim = numel(f_handle_clip(np.zeros((2, 1))))
        if f_dim == 1:
            # scalar field
            fig = plt.gcf()
            axis = fig.add_subplot(111, projection='3d')

            axis.plot_surface(xx_mesh,
                              yy_mesh,
                              f_eval.transpose(),
                              cmap=cm.gnuplot2)
            axis.set_zlim(0, threshold)
        elif f_dim == 2:
            # vector field

            # grid.eval gives the result transposed with respect to
            # what meshgrid expects
            f_eval = f_eval.transpose((1, 0, 2))
            # vector field
            plt.quiver(xx_mesh,
                       yy_mesh,
                       f_eval[:, :, 0],
                       f_eval[:, :, 1],
                       angles='xy',
                       scale_units='xy',
                       scale=1)
            axis = plt.gca()
        else:
            raise NotImplementedError(
                'Field plotting for dimension greater than two not implemented'
            )

        axis.set_xlim(-15, 15)
        axis.set_ylim(-15, 15)
        plt.xlabel('x')
        plt.ylabel('y')


class Torus:
    """
    A class that holds functions to compute the embedding and display a torus and curves on it.
    """

    def phi(self, theta):
        """
        Implements equation (eq:chartTorus).
        """
        theta1, theta2 = theta
        theta = np.atleast_2d(theta)
        if theta.shape[1] != 1:
            theta = theta.T
        num = theta.shape[1]
        x_torus = np.zeros((3, num))
        rot_2d = rot2d(theta2)
        rot_3d = np.eye(3)
        rot_3d[:2, :2] = rot_2d
        r = 3

        for i in range(num):
            circle_phi = Torus.circle_phi(self, theta[0, i], r)
            x_torus[:, i] = np.squeeze(np.matmul(rot_3d, circle_phi))

        return x_torus

    def plot(self):
        """
        For each one of the chart domains U_i from the previous question:
        - Fill a  grid structure with fields  xx_grid and  yy_grid that define a grid of regular
          point in U_i. Use nb_grid=33.
        - Call the function Grid.eval with argument Torus.phi.
        - Plots the surface described by the previous step using the the Matplotlib function
        ax.plot_surface (where  ax represents the axes of the current figure) in a separate figure.
        Plot a final additional figure showing all the charts at the same time.   To better show
        the overlap between the charts, you can use different colors each one of them,
        and making them slightly transparent.
        """
        nb_grid = 33
        xx_grid = np.linspace(0, 2 * math.pi, nb_grid)
        yy_grid = np.linspace(0, 2 * math.pi, nb_grid)
        grid = Grid(xx_grid, yy_grid)
        fun_eval = grid.eval(self.phi)
        [xx_grid, yy_grid] = grid.mesh()
        axis = gca_3d()
        axis.plot_surface(fun_eval[:, :, 0],
                          fun_eval[:, :, 1],
                          fun_eval[:, :, 2],
                          alpha=0.4)
        # plt.show()

    def phi_push_curve(self, a_line, b_line):
        """
        This function evaluates the curve x(t)= phi_torus ( phi(t) )  R^3 at  nb_points=31 points
        generated along the curve phi(t) using Line.linspace with  tMin=0 and  tMax=1,
        and a, b as given in the input arguments.
        """
        t_min = 0
        t_max = 1
        nb_points = 31
        x_points = np.zeros((3, nb_points))
        theta_points = line_linspace(a_line, b_line, t_min, t_max, nb_points)
        for idx in range(nb_points):
            x_points[:, idx] = self.phi(theta_points[:, idx]).flatten()
        return x_points

    def plot_curves(self):
        """
        The function should iterate over the following four curves:
        - 3/4*pi0
        - 3/4*pi3/4*pi
        - -3/4*pi3/4*pi
        - 0 -3/4*pi  and  b=np.array([[-1],[-1]]).
        The function should show an overlay containing:
        - The output of Torus.plotCharts;
        - The output of the functions torus_pushCurveTorus.pushCurve for each one of the curves.
        """
        a_lines = [
            np.array([[3 / 4 * math.pi], [0]]),
            np.array([[3 / 4 * math.pi], [3 / 4 * math.pi]]),
            np.array([[-3 / 4 * math.pi], [3 / 4 * math.pi]]),
            np.array([[0], [-3 / 4 * math.pi]])
        ]

        b_line = np.array([[-1], [-1]])

        axis = gca_3d()
        for a_line in a_lines:
            x_points = self.phi_push_curve(a_line, b_line)
            axis.plot(x_points[0, :], x_points[1, :], x_points[2, :])
        self.plot()

    def phi_test(self):
        """
        Uses the function phi to plot two perpendicular rings
        """
        nb_points = 200
        theta_ring = np.linspace(0, 15 / 8 * np.pi, nb_points)
        theta_zeros = np.zeros((1, nb_points))
        data = [
            np.vstack((theta_ring, theta_zeros)),
            np.vstack((theta_zeros, theta_ring))
        ]
        axis = gca_3d()
        for theta in data:
            ring = np.zeros((3, nb_points))
            for idx in range(nb_points):
                ring[:, [idx]] = self.phi(theta[:, [idx]])
            axis.plot(ring[0, :], ring[1, :], ring[2, :])

    def circle_phi(self, theta, r):
        """ 
        This is a function I made that just does the inside () for torus phi
        """
        circle = np.array([[math.cos(theta) + r], [0], [math.sin(theta)]])

        return circle


class Polygon:
    """
    Class for plotting, drawing, checking visibility and collision with
    polygons.
    """

    def __init__(self, vertices):
        """
        Save the input coordinates to the internal attribute  vertices.
        """
        self.vertices = vertices

    @property
    def nb_vertices(self):
        """ Number of vertices """
        return self.vertices.shape[1]

    def flip(self):
        """
        Reverse the order of the vertices (i.e., transform the polygon from
        filled in to hollow and viceversa).
        """
        self.vertices = np.fliplr(self.vertices)

    def plot(self, style):
        """
        Plot the polygon using Matplotlib.
        """
        if len(style) == 0:
            style = 'k'

        directions = np.diff(self.vertices_loop)
        plt.quiver(self.vertices[0, :],
                   self.vertices[1, :],
                   directions[0, :],
                   directions[1, :],
                   color=style,
                   angles='xy',
                   scale_units='xy',
                   scale=1.)

    @property
    def vertices_loop(self):
        """
        Returns self.vertices with the first vertex repeated at the end
        """
        return np.hstack((self.vertices, self.vertices[:, [0]]))

    def is_filled(self):
        """
        Checks the ordering of the vertices, and returns whether the polygon is
        filled in or not.
        """

        # Iterates over the columns of the 2D Matrix to perform the calculation
        # sum((x_2 - x_1) * (y_2 + y_1))
        # If the sum is negative, the polygon is oriented counter-clockwise,
        # clockwise otherwise.

        num_cols = self.vertices.shape[1]
        running_sum = 0

        for i in range(num_cols - 1):
            x_vals = self.vertices[0, :]
            y_vals = self.vertices[1, :]

            # modulus is for the last element to be compared with the first
            # to close the shape
            running_sum += (x_vals[(i+1) % num_cols] - x_vals[i]) * \
                (y_vals[i] + y_vals[(i+1) % num_cols])

        return running_sum < 0

    def is_self_occluded(self, idx_vertex, point):
        """
        Given the corner of a polygon, checks whether a given point is
        self-occluded or not by that polygon (i.e., if it is ``inside'' the
        corner's cone or not). Points on boundary (i.e., on one of the sides of
        the corner) are not considered self-occluded. Note that to check
        self-occlusion, we just need a vertex index  idx_vertex. From this, one
        can obtain the corresponding  vertex, and the  vertex_prev and
        vertex_next that precede and follow that vertex in the polygon. This
        information is sufficient to determine self-occlusion. To convince
        yourself, try to complete the corners shown in Figure~
        fig:self-occlusion with clockwise and counterclockwise polygons, and
        you will see that, for each example, only one of these cases can be
        consistent with the arrow directions.
        """
        vertex = self.vertices[:, [idx_vertex]]
        vertex_next = self.vertices[:, [(idx_vertex + 1) % self.nb_vertices]]
        vertex_prev = self.vertices[:, [(idx_vertex - 1) % self.nb_vertices]]

        # The point is occluded if, measuring angles using p-vertex as the
        # "zero angle", the angle for vertex_prev is smaller than the one
        # for vertex_next. Using the 'unsigned' angles means that we do not
        # have to worry separately about negative angles
        angle_p_prev = angle(vertex, point, vertex_prev, 'unsigned')
        angle_p_next = angle(vertex, point, vertex_next, 'unsigned')

        return angle_p_prev < angle_p_next

    def is_visible(self, idx_vertex, test_points):
        """
        Checks whether a point p is visible from a vertex v of a polygon. In
        order to be visible, two conditions need to be satisfied:
         - The point p should not be self-occluded with respect to the vertex
        v\\ (see Polygon.is_self_occluded).
         - The segment p--v should not collide with  any of the edges of the
        polygon (see Edge.is_collision).
        """
        nb_test_points = test_points.shape[1]
        nb_vertices = self.vertices.shape[1]

        # Initial default: all flags are True
        flag_points = [True] * nb_test_points
        vertex = self.vertices[:, [idx_vertex]]
        for idx_point in range(0, nb_test_points):
            point = test_points[:, [idx_point]]

            # If it is self occluded, bail out
            if self.is_self_occluded(idx_vertex, point):
                flag_points[idx_point] = False
            else:
                # Build the vertex-point edge (it is the same for all other
                # edges)
                edge_vertex_point = Edge(np.hstack([point, vertex]))
                # Then iterate over all edges in the polygon
                for idx_vertex_collision in range(0, self.nb_vertices):
                    edge_vertex_vertex = Edge(self.vertices[:, [
                        idx_vertex_collision,
                        (idx_vertex_collision + 1) % nb_vertices
                    ]])
                    # The final result is the and of all the checks with
                    # individual edges
                    flag_points[
                        idx_point] &= not edge_vertex_point.is_collision(
                            edge_vertex_vertex)

                    # Early bail out after one collision
                    if not flag_points[idx_point]:
                        break

        return flag_points

    def is_collision(self, test_points):
        """
        Checks whether the a point is in collsion with a polygon (that is,
        inside for a filled in polygon, and outside for a hollow polygon). In
        the context of this homework, this function is best implemented using
        Polygon.is_visible.
        """
        flag_points = [False] * test_points.shape[1]
        # We iterate over the polygon vertices, and process all the test points
        # in parallel
        for idx_vertex in range(0, self.nb_vertices):
            flag_points_vertex = self.is_visible(idx_vertex, test_points)
            # Accumulate the new flags with the previous ones
            flag_points = [
                flag_prev or flag_new
                for flag_prev, flag_new in zip(flag_points, flag_points_vertex)
            ]
        flag_points = [not flag for flag in flag_points]
        return flag_points


class Edge:
    """ Class for storing edges and checking collisions among them. """

    def __init__(self, vertices):
        """
        Save the input coordinates to the internal attribute  vertices.
        """
        self.vertices = vertices

    @property
    def direction(self):
        """ Difference between tip and base """
        return self.vertices[:, [1]] - self.vertices[:, [0]]

    @property
    def base(self):
        """ Coordinates of the first vertex"""
        return self.vertices[:, [0]]

    def plot(self, *args, **kwargs):
        """ Plot the edge """
        plt.plot(self.vertices[0, :], self.vertices[1, :], *args, **kwargs)

    def is_collision(self, edge):
        """
         Returns  True if the two edges intersect.  Note: if the two edges
        overlap but are colinear, or they overlap only at a single endpoint,
        they are not considered as intersecting (i.e., in these cases the
        function returns  False). If one of the two edges has zero length, the
        function should always return the result that edges are
        non-intersecting.
        """

        # Write the lines from the two edges as
        # x_i(t_i)=edge_base+edge.direction*t_i
        # Then finds the parameters for the intersection by solving the linear
        # system obtained from x_1(t_1)=x_2(t_2)

        # Tolerance for cases involving parallel lines and endpoints
        tol = 1e-6

        # The matrix of the linear system
        a_directions = np.hstack([self.direction, -edge.direction])
        if abs(np.linalg.det(a_directions)) < tol:
            # Lines are practically parallel
            return False
        # The vector of the linear system
        b_bases = np.hstack([edge.base - self.base])

        # Solve the linear system
        t_param = np.linalg.solve(a_directions, b_bases)
        t_self = t_param[0, 0]
        t_other = t_param[1, 0]

        # Check that collision point is strictly between endpoints of each edge
        flag_collision = tol < t_self < 1.0 - tol and tol < t_other < 1.0 - tol

        return flag_collision


def angle(vertex0, vertex1, vertex2, angle_type='unsigned'):
    """
    Compute the angle between two edges  vertex0-- vertex1 and  vertex0--
    vertex2 having an endpoint in common. The angle is computed by starting
    from the edge  vertex0-- vertex1, and then ``walking'' in a
    counterclockwise manner until the edge  vertex0-- vertex2 is found.
    """
    # tolerance to check for coincident points
    tol = 2.22e-16

    # compute vectors corresponding to the two edges, and normalize
    vec1 = vertex1 - vertex0
    vec2 = vertex2 - vertex0

    norm_vec1 = np.linalg.norm(vec1)
    norm_vec2 = np.linalg.norm(vec2)
    if norm_vec1 < tol or norm_vec2 < tol:
        # vertex1 or vertex2 coincides with vertex0, abort
        edge_angle = math.nan
        return edge_angle

    vec1 = vec1 / norm_vec1
    vec2 = vec2 / norm_vec2

    # Transform vec1 and vec2 into flat 3-D vectors,
    # so that they can be used with np.inner and np.cross
    vec1flat = np.vstack([vec1, 0]).flatten()
    vec2flat = np.vstack([vec2, 0]).flatten()

    c_angle = np.inner(vec1flat, vec2flat)
    s_angle = np.inner(np.array([0, 0, 1]), np.cross(vec1flat, vec2flat))

    edge_angle = math.atan2(s_angle, c_angle)

    angle_type = angle_type.lower()
    if angle_type == 'signed':
        # nothing to do
        pass
    elif angle_type == 'unsigned':
        edge_angle = (edge_angle + 2 * math.pi) % (2 * math.pi)
    else:
        raise ValueError('Invalid argument angle_type')

    return edge_angle


