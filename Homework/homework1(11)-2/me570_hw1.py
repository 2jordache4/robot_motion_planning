"""
Test functions for HW1
"""

import matplotlib.pyplot as plt
import numpy as np
from numpy import cos, pi, sin

import me570_geometry as geometry
import me570_queue as queue
import me570_robot as robot


def edge_is_collision_test():
    """
    The function creates an edge from  [0;0] to
    [1,1] and a second random edge with endpoints
    contained in the square [0,1] [0,1], and plots them in green if they do
    not overlap, and in red otherwise.
    """

    vertices = [np.array([[0, 1], [0, 1]]), np.random.rand(2, 2)]
    edges = [geometry.Edge(x) for x in vertices]
    flag_collision = edges[0].is_collision(edges[1])
    if flag_collision:
        style = 'r'
    else:
        style = 'g'
    style = style + '-o'
    for edge in edges:
        print(edge.vertices)
        edge.plot(style)


def polygon_is_self_occluded_test():
    """
    Visually test the function polygon_isSelfOccluded by picking random
    arrangements for  vertexPrev and  vertexNext, and systematically picking
    the position of  point. The meaning of the green and red lines are
    similar to those shown in  fig:self-occlusion.
    """
    vertex = np.zeros((2, 1))
    # print(vertex)
    angles_test = np.random.rand(2) * 2 * pi

    vertex_prev = np.array([[cos(angles_test[0])], [sin(angles_test[0])]])
    vertex_next = np.array([[cos(angles_test[1])], [sin(angles_test[1])]])

    nb_points = 61
    angle_point = np.linspace(0, 2 * pi, nb_points)
    point = np.vstack([cos(angle_point), sin(angle_point)])

    polygon = geometry.Polygon(np.hstack([vertex_prev, vertex, vertex_next]))
    polygon.plot('k')

    for i_point in range(nb_points):
        # check collision against the second vertex
        # (index 1, corresponding to vertex)
        flag_occluded = polygon.is_self_occluded(1, point[:, [i_point]])
        if flag_occluded:
            style = 'r'
        else:
            style = 'g'
        plt.plot([0, point[0, i_point]], [0, point[1, i_point]], style)
    plt.xlim(-1, 1)
    plt.ylim(-1, 1)
    plt.gca().axis('equal')
    plt.text(vertex_prev[0], vertex_prev[1], 'vertex_prev')
    plt.text(vertex_next[0], vertex_next[1], 'vertex_next')
    plt.text(vertex[0], vertex[1], 'vertex')


def polygon_is_visible_test():
    """
    This function should perform the following operations:
     - Create an array  test_points with dimensions [2 x 5] containing
    points generated uniformly at random using np.random.rand and scaled to
    approximately occupy the rectangle [0,5] [-2,2] (i.e., the x coordinates
    of the points should fall between 0 and 5, while the y coordinates
    between -2 and 2).
     - Obtain the polygons  polygon1 and  polygon2 from TwoLink.Polygons.
     - item:test-polygon For each polygon  polygon1,  polygon2, display a
    separate figure using the following:
     - Create the array  test_points_with_polygon by concatenating
    test_points with the coordinates of the polygon (i.e., the coordinates
    of the polygon become also test points).
     - Plot the polygon (use Polygon.plot).
     - item:test-visibility For each vertex v in the polygon:
     - Compute the visibility of each point in  test_points_with_polygon
    with respect to that polygon (using Polygon.is_visible).
     - Plot lines from the vertex v to each point in
    test_points_with_polygon in green if the corresponding point is visible,
    and in red otherwise.
     - Reverse the order of the vertices in the two polygons using
    Polygon.flip.
     - Repeat item item:test-polygon above with the reversed polygons.
    """
    random_points = np.random.rand(2, 5)

    x_min, x_max = 0, 5
    y_min, y_max = -2, 2

    x_coords = random_points[0] * (x_max - x_min) + x_min
    y_coords = random_points[1] * (y_max - y_min) + y_min

    test_points = np.vstack((x_coords, y_coords))
    polygon1, polygon2 = robot.polygons_generate()

    polygons = [polygon1, polygon2]
    # for i in range(len(polygon1.vertices[0])):

    for polygon in polygons:
        # Create test points with polygon vertices

        test_points_with_polygon = np.hstack((test_points, polygon.vertices))

        polygon.plot('k')
        for i in range(len(polygon.vertices[0])):
            vertex = polygon.vertices[:, i]
            for j in range(len(test_points_with_polygon[0])):
                if (polygon.is_visible(i, test_points_with_polygon)[j]):
                    plt.plot([vertex[0], test_points_with_polygon[0][j]],
                             [vertex[1], test_points_with_polygon[1][j]], 'g-')
                else:
                    plt.plot([vertex[0], test_points_with_polygon[0][j]],
                             [vertex[1], test_points_with_polygon[1][j]], 'r-')

        polygon.flip()

        test_points_with_polygon = np.hstack((test_points, polygon.vertices))

        polygon.plot('k')
        for i in range(len(polygon.vertices[0])):
            vertex = polygon.vertices[:, i]
            for j in range(len(test_points_with_polygon[0])):
                if (polygon.is_visible(i, test_points_with_polygon)[j]):
                    plt.plot([vertex[0], test_points_with_polygon[0][j]],
                             [vertex[1], test_points_with_polygon[1][j]], 'g-')
                else:
                    plt.plot([vertex[0], test_points_with_polygon[0][j]],
                             [vertex[1], test_points_with_polygon[1][j]], 'r-')


def polygon_is_collision_test_plot(polygon, test_points):
    """
    Helper function for polygon_is_collision_test to run tests and plot the
    results for a single polygon
    """
    test_points_with_polygon = np.hstack((polygon.vertices, test_points))
    # plt.figure()
    polygon.plot('k')

    green_x = []
    green_y = []
    red_x = []
    red_y = []

    flag_points = polygon.is_collision(test_points_with_polygon)
    for i, point in enumerate(test_points_with_polygon.T):
        x_point = point[0]
        y_point = point[1]
        if flag_points[i] is True:
            red_x.append(x_point)
            red_y.append(y_point)
        else:
            green_x.append(x_point)
            green_y.append(y_point)

    plt.scatter(green_x, green_y, color='g')
    plt.scatter(red_x, red_y, color='r')

    plt.show()


def polygon_is_collision_test():
    """
    This function is the same as polygon_is_visible_test, but instead of
    step  item:test-visibility, use the following:
     - Compute whether each point in  test_points_with_polygon is in
    collision with the polygon or not using Polygon.is_collision.
     - Plot each point in  test_points_with_polygon in green if it is not in
    collision, and red otherwise.  Moreover, increase the number of test
    points from 5 to 100 (i.e.,  testPoints should have dimension [2 x
    100]).
    """
    test_points = np.random.rand(2, 100)

    # Scale x coordinates to uniformly cover [0, 5)
    test_points[0, :] *= 5

    # Scale y coordinates to uniformly cover [-2, 2)
    #   formula used: low + ((high - low) * random_value)
    test_points[1, :] *= 4  # high - low
    test_points[1, :] -= 2  # low

    # Loop over polygon1 and polygon2
    for polygon in robot.polygons:
        polygon_is_collision_test_plot(polygon, test_points)
        polygon.flip()
        polygon_is_collision_test_plot(polygon, test_points)


def priority_test():
    """
    The function should perform the following steps:  enumerate
     - Initialize an empty queue as the object  p_queue.
     - Add three elements (as shown in Table~tab:priority-test-inputs and in
    that order) to that queue.
     - Extract a minimum element.
     - Add another element (as shown in Table~tab:priority-test-inputs).
     - Check if an element (which is in the queue) is present.
     - Check if an element (which is  not in the queue) is present.
     - Remove all elements by repeated extractions.  enumerate After each
    step, display the content of  p_queue.
    """
    my_queue = queue.PriorityQueue()

    print('priority_test: Add "Oranges", "Apples", "Bananas"')
    my_queue.insert("Oranges", 4.5)
    my_queue.insert("Apples", 1)
    my_queue.insert("Bananas", 2.7)
    print(*my_queue.queue_list, sep=", ")

    key, cost = my_queue.min_extract()
    print(f"priority_test: Extracted ({key}, {cost})")

    print('priority_test: Add "Cantaloupe"')
    my_queue.insert("Cantaloupe", 3)
    print(*my_queue.queue_list, sep=", ")

    print("priority_test: Oranges is in queue? -->" +
          f"{my_queue.is_member('Oranges')}")
    print(f"priority_test: Milk is in queue? --> {my_queue.is_member('Milk')}")

    while True:
        (key, value) = my_queue.min_extract()
        if (key is None and value is None):
            break

        print(f"priority_test: Removed ({key}, {value}) --> remaining: ",
              end=" ")
        print(*my_queue.queue_list, sep=", ")


# edge_is_collision_test()
# polygon_is_self_occluded_test()
polygon_is_visible_test()
plt.show()
