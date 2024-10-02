"""
Classes and functions for Polygons and Edges
"""

import math

import numpy as np
from matplotlib import pyplot as plt
import me570_robot as robot


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

        fig, ax = plt.subplots()

        xCoords = self.vertices[0]
        yCoords = self.vertices[1]
        length = len(self.vertices[0]) - 1

        num = 0
        while num < length:
            ax.quiver(xCoords[num],
                      yCoords[num],
                      xCoords[num + 1] - xCoords[num],
                      yCoords[num + 1] - yCoords[num],
                      angles='xy',
                      scale_units='xy',
                      scale=1,
                      color=style)
            num += 1

        ax.quiver(xCoords[num],
                  yCoords[num],
                  xCoords[0] - xCoords[num],
                  yCoords[0] - yCoords[num],
                  angles='xy',
                  scale_units='xy',
                  scale=1,
                  color=style)

        ax.set_xlim(-1, 8)
        ax.set_ylim(-3, 6)
        # plt.axis('off')
        # plt.show()

    def is_filled(self):
        """
        Checks the ordering of the vertices, and returns whether the polygon is
        filled in or not.
        """

        bFlag = False

        return bFlag

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
        bOccluded = False

        previous_Index = (idx_vertex - 1) % len(self.vertices[0])
        next_Index = (idx_vertex + 1) % len(self.vertices[0])

        current_Vertex = self.vertices[:, [idx_vertex]]
        previous_Vertex = self.vertices[:, [previous_Index]]
        next_Vertex = self.vertices[:, [next_Index]]

        angle1 = angle(current_Vertex,
                       previous_Vertex,
                       next_Vertex,
                       angle_type='unsigned')
        angle2 = angle(current_Vertex,
                       previous_Vertex,
                       point,
                       angle_type='unsigned')
        difference = angle1 - angle2

        if difference < 0:
            bOccluded = True
        else:
            bOccluded = False

        return bOccluded

    def is_visible(self, idx_vertex, test_points):
        """
        Checks whether a point p is visible from a vertex v of a polygon. In
        order to be visible, two conditions need to be satisfied:
         - The point p should not be self-occluded with respect to the vertex
        v\\ (see Polygon.is_self_occluded).
         - The segment p--v should not collide with  any of the edges of the
        polygon (see Edge.is_collision).
        """

        vertex = self.vertices[:, idx_vertex]
        coords_Conversion = [[0], [0]]
        # total_points = np.hstack((test_points, self.vertices))
        flag_points = []
        # print(vertex)

        # self.plot('k')

        for i, coords in enumerate(test_points.T):
            bOcclude = True
            bCollide = True

            coords_Conversion[0][0] = coords[0]
            coords_Conversion[1][0] = coords[1]
            # print(coords_Conversion)


            if (coords_Conversion[0][0] != vertex[0] or \
                coords_Conversion[1][0] != vertex[1]):
                if self.is_self_occluded(idx_vertex, coords_Conversion):
                    bOcclude = False
                    # print("bOccluded")


                vertex_point = np.array([[vertex[0],coords_Conversion[0][0]],\
                                            [vertex[1],coords_Conversion[1][0]]])
                vertex_point_edge = Edge(vertex_point)
                if self.edge_polygon_collision(vertex_point_edge):
                    bCollide = False
                    # print("bCollided")
            else:
                bCollide = False

            if bCollide and bOcclude:
                # plt.plot([vertex[0], coords_Conversion[0][0]],
                #          [vertex[1], coords_Conversion[1][0]], 'g-')
                flag_points.append(True)
            else:
                # plt.plot([vertex[0], coords_Conversion[0][0]],
                #          [vertex[1], coords_Conversion[1][0]], 'r-')
                flag_points.append(False)

        #SWITCH SHAPES IN HW.PY

        return flag_points

    def edge_polygon_collision(polygon, edge):

        bCollide = False

        for i in range(len(polygon.vertices[0])):
            current_Index = polygon.vertices[:, i]
            next_Index = polygon.vertices[:,
                                          (i + 1) % len(polygon.vertices[0])]
            polygon_array = np.array([[current_Index[0], next_Index[0]],
                                      [current_Index[1], next_Index[1]]])
            polygon_edge = Edge(polygon_array)

            if polygon_edge.is_collision(edge):
                bCollide = True

        return bCollide

    def is_in_polygon(self, point):
        """
        I wrote this ray tracing algo to tell if a point is inside a 
        a polygon or not. I used a youtube video by MagellanicMath, 
        psuedocode from rosettacode, and http://www.philliplemons.com/posts/ray-casting-algorithm
        """
        bInside = False
        total_vertices = len(self.vertices[0])

        num_crosses = 0

        for i in range(total_vertices):
            polygon_Point = self.vertices[:, i]
            polygon_Point2 = self.vertices[:, (i + 1) % total_vertices]

            if (polygon_Point[1] > point[1]) != (polygon_Point2[1] > point[1]):
                intersect = (polygon_Point[0] - point[0]) * (point[1] - polygon_Point[1]) / \
                    (polygon_Point2[1] - polygon_Point[1]) + polygon_Point[0]
                if point[0] < intersect:
                    num_crosses += 1

        if (num_crosses % 2) != 0:
            bInside = True

        return bInside

    def is_collision(self, test_points):
        """
        Checks whether the a point is in collsion with a polygon (that is,
        inside for a filled in polygon, and outside for a hollow polygon). In
        the context of this homework, this function is best implemented using
        Polygon.is_visible.
        """
        bCollision = False
        vertex = self.vertices[:, 0]
        coords_Conversion = [[0], [0]]
        # total_points = np.hstack((test_points, self.vertices))
        flag_points = []
        # print(vertex)

        # self.plot('k')

        for i, coords in enumerate(test_points.T):
            bOcclude = True
            bCollide = True

            coords_Conversion[0][0] = coords[0]
            coords_Conversion[1][0] = coords[1]
            # print(coords_Conversion)


            if (coords_Conversion[0][0] != vertex[0] or \
                coords_Conversion[1][0] != vertex[1]):
                if self.is_self_occluded(0, coords_Conversion):
                    bOcclude = False
                    # print("bOccluded")


                vertex_point = np.array([[vertex[0],coords_Conversion[0][0]],\
                                            [vertex[1],coords_Conversion[1][0]]])
                vertex_point_edge = Edge(vertex_point)
                if self.edge_polygon_collision(vertex_point_edge):
                    bCollide = False
                    # print("bCollided")
            else:
                bCollide = False

            if bCollide:
                # plt.plot([vertex[0], coords_Conversion[0][0]],
                #          [vertex[1], coords_Conversion[1][0]], 'g-')
                flag_points.append(True)
            else:
                # plt.plot([vertex[0], coords_Conversion[0][0]],
                #          [vertex[1], coords_Conversion[1][0]], 'r-')
                flag_points.append(False)

        #SWITCH SHAPES IN HW.PY

        return flag_points


class Edge:
    """
    Class for storing edges and checking collisions among them.
    """

    def __init__(self, vertices):
        """
        Save the input coordinates to the internal attribute  vertices.
        """
        self.vertices = vertices

    def is_collision(self, edge):
        """
         Returns  True if the two edges intersect.  Note: if the two edges
        overlap but are colinear, or they overlap only at a single endpoint,
        they are not considered as intersecting (i.e., in these cases the
        function returns  False). If one of the two edges has zero length, the
        function should always return the result that edges are
        non-intersecting.
        """
        bCollide = False

        edge1_Start = self.vertices[:, 0]
        edge1_End = self.vertices[:, 1]
        edge2_Start = edge.vertices[:, 0]
        edge2_End = edge.vertices[:, 1]
        dir1 = self.ori(edge1_Start, edge1_End, edge2_Start)
        dir2 = self.ori(edge1_Start, edge1_End, edge2_End)
        dir3 = self.ori(edge2_Start, edge2_End, edge1_Start)
        dir4 = self.ori(edge2_Start, edge2_End, edge1_End)

        if ((self.edgeLength(edge1_Start, edge1_End)
             or self.edgeLength(edge2_Start, edge2_End)) == 0):
            bCollide = True
            # This checks if either of the segments is length 0

        if (dir1 > 0 and dir2 < 0 or dir1 < 0 and dir2 > 0) \
            and (dir3 > 0 and dir4 < 0 or dir3 < 0 and dir4 > 0):
            bCollide = True

        return bCollide

    def plot(self, *args, **kwargs):
        """ Plot the edge """
        plt.plot(self.vertices[0, :], self.vertices[1, :], *args, **kwargs)

    def ori(self, edge1_Start, edge1_End, edge2_Point):
        """
        This will calculate the orientation of three points. 
        Returns negative if edge2_Point is left of edge1, postive if right.
        Used geeksforgeeks to understand the algorithm 
        """
        return (((edge1_End[1] - edge1_Start[1]) * \
               (edge2_Point[0] - edge1_End[0])) - \
               ((edge1_End[0] - edge1_Start[0]) * \
               (edge2_Point[1] - edge1_End[1])))

    def edgeLength(self, edge_Start, edge_End):
        """
        Helper Function to check edge length in is_collision
        """
        return np.linalg.norm(np.array(edge_End) - np.array(edge_Start))


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


# vertices = np.array([[0, 0, 5, 5], [0, 5, 5, 0]])
# vertices2 = np.array([[0, 0], [0, 5]])
# vertices3 = np.array([[0, 5], [5, 5]])

# # print(vertices.T)
# # square = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
# myPoints = np.array([[10, 20, 30, 40], [10, 20, 30, 40]])
# myPoly = Polygon(vertices)
# myPoly.flip()
# myPoly.plot('k')
# plt.show()
# # squarePoly = Polygon(square)
# # print(squarePoly.is_visible(0, myPoints))
# print(myPoly.is_visible(1,myPoints))
# plt.show()



