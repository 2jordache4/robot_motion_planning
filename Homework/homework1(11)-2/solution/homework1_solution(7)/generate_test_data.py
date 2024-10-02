#!/usr/bin/env python
'''
Generate autograder test data for Homework1
'''

import os
import pickle
import sys

import matplotlib.pyplot as plt
import numpy as np
import typer

import me570_geometry as geometry
import me570_queue as queue

app = typer.Typer()

sys.path.insert(0, '../common')

import test_data

DIR_AUTOGRADER = '../../pythonAutograders/homework1'

EPS = np.finfo(float).eps


@app.command()
def edge_is_collision_easy():
    '''
    Same case as the provided test
    '''

    vertices_base = np.array([[0, 1], [0, 1]])
    edge_base = geometry.Edge(vertices_base)
    vertices = [np.random.rand(2, 2) for count in range(10)]
    edges = [(geometry.Edge(x), ) for x in vertices]

    test_data.generate_save_pairs_object(DIR_AUTOGRADER,
                                         edge_base,
                                         geometry.Edge.is_collision,
                                         edges,
                                         file_name='Edge.is_collision_easy')


@app.command()
def edge_is_collision_overlap():
    '''
    Overlap
    '''
    vertices_base = np.array([[0, 1], [0, 1]])
    edge_base = geometry.Edge(vertices_base)
    edges = [(geometry.Edge(vertices_base + 0.5 + EPS), )]
    test_data.generate_save_pairs_object(DIR_AUTOGRADER,
                                         edge_base,
                                         geometry.Edge.is_collision,
                                         edges,
                                         file_name='Edge.is_collision_overlap')


@app.command()
def edge_is_collision_corners():
    '''
    Corners
    '''
    vertices_base = np.random.randn(2, 2)
    edge_base = geometry.Edge(vertices_base)

    def build_corner_vertices():
        return np.hstack((vertices_base[:, [1]], np.random.randn(2, 1)))

    edges = [(geometry.Edge(build_corner_vertices()), ) for count in range(5)]
    test_data.generate_save_pairs_object(DIR_AUTOGRADER,
                                         edge_base,
                                         geometry.Edge.is_collision,
                                         edges,
                                         file_name='Edge.is_collision_corners')


@app.command()
def edge_all():
    '''Generate all tests'''
    edge_is_collision_easy()
    edge_is_collision_overlap()
    edge_is_collision_corners()


def polygon_point_test_data():
    '''
    Generate a polygon and points around each vertex as test data
    '''

    vertices = np.array([[0., 2., 1., .5], [0, .5, 1., 2]])
    polygon = geometry.Polygon(vertices)
    nb_vertices = vertices.shape[1]
    nb_test_points_vertex = 33
    angle = np.linspace(0, 2 * np.pi, nb_test_points_vertex)
    test_points_base = np.vstack((np.cos(angle), np.sin(angle)))
    test_points_base = np.hstack((test_points_base, 5 * test_points_base))
    inputs = []
    for idx_vertex in range(nb_vertices):
        test_points_list = [
            vertices[:, [idx_vertex]] + test_points_base[:, [idx_point]]
            for idx_point in range(nb_test_points_vertex)
        ]
        inputs += [(idx_vertex, point) for point in test_points_list]

    return polygon, inputs


@app.command()
def polygon_is_self_occluded():
    '''
    Use common test data for the function named above
    '''
    polygon, inputs = polygon_point_test_data()
    test_data.generate_save_pairs_object(DIR_AUTOGRADER,
                                         polygon,
                                         geometry.Polygon.is_self_occluded,
                                         inputs,
                                         file_name='Polygon.is_self_occluded')


@app.command()
def polygon_is_self_occluded_hollow():
    '''
    Hollow version
    '''
    polygon, inputs = polygon_point_test_data()
    polygon.flip()
    test_data.generate_save_pairs_object(
        DIR_AUTOGRADER,
        polygon,
        geometry.Polygon.is_self_occluded,
        inputs,
        file_name='Polygon.is_self_occluded_hollow')


@app.command()
def polygon_is_visible():
    '''
    Use common test data for the function named above
    '''
    polygon, inputs = polygon_point_test_data()
    test_data.generate_save_pairs_object(DIR_AUTOGRADER,
                                         polygon,
                                         geometry.Polygon.is_visible,
                                         inputs,
                                         file_name='Polygon.is_visible')


@app.command()
def polygon_is_visible_hollow():
    '''
    Hollow case
    '''
    polygon, inputs = polygon_point_test_data()
    polygon.flip()
    test_data.generate_save_pairs_object(DIR_AUTOGRADER,
                                         polygon,
                                         geometry.Polygon.is_visible,
                                         inputs,
                                         file_name='Polygon.is_visible_hollow')


@app.command()
def polygon_is_collision(flag_flip, flag_visualize=False):
    '''
    Use random points to test collision
    '''
    polygon, _ = polygon_point_test_data()
    filename = 'Polygon.is_collision'
    if flag_flip:
        polygon.flip()
        filename += '_hollow'

    nb_points = 55
    test_points = 2 * np.random.rand(2, nb_points)
    inputs = [(test_points[:, [idx]], ) for idx in range(nb_points)]
    test_pairs = test_data.generate_save_pairs_object(
        DIR_AUTOGRADER,
        polygon,
        geometry.Polygon.is_collision,
        inputs,
        file_name=filename)

    points_collision = np.hstack(
        [input[1] for (input, output) in test_pairs if output[0]])
    points_no_collision = np.hstack(
        [input[1] for (input, output) in test_pairs if not output[0]])
    print(f'Collisions over total: {points_collision.shape[1]}/{nb_points}')

    if flag_visualize:
        polygon.plot('k')
        plt.scatter(points_collision[0, :], points_collision[1, :], color='r')
        plt.scatter(points_no_collision[0, :],
                    points_no_collision[1, :],
                    color='g')
        plt.show()


@app.command()
def polygon_all():
    '''
    Generate all data for testing polygons
    '''
    polygon_is_self_occluded()
    polygon_is_visible()
    polygon_is_self_occluded_hollow()
    polygon_is_visible_hollow()
    polygon_is_collision(False)
    polygon_is_collision(True)


@app.command()
def queue_data():
    '''
    Generate dictionary with two model queues: one with keys of a single type,
    the other with keys of different types
    '''
    mono_queue = queue.PriorityQueue()
    mono_queue.insert("Oranges", 4.5)
    mono_queue.insert("Apples", 1)
    mono_queue.insert("Bananas", 2.7)

    bi_queue = queue.PriorityQueue()
    bi_queue.insert(10, 4.5)
    bi_queue.insert("Apples", 1)
    bi_queue.insert(12.5, 2.7)
    bi_queue.insert(True, 4)
    bi_queue.insert(None, 3.7)

    small_queue = queue.PriorityQueue()
    small_queue.insert('Single', -20)

    queue_dict = {
        'mono': mono_queue,
        'bi': bi_queue,
        'small': small_queue,
        'empty': queue.PriorityQueue()
    }

    filename = os.path.join(DIR_AUTOGRADER, 'queue_data.pickle')
    print('Saving test data to', filename)
    with open(filename, 'wb') as fout:
        pickle.dump(queue_dict, fout)

    return queue_dict


@app.command()
def queue_min_extract(q_type='mono', depth=None):
    '''
    Test extraction on a model queue
    '''

    p_queue = queue_data()[q_type]
    if depth is None:
        depth = len(p_queue.queue_list)
    inputs = [() for _ in range(depth)]
    test_data.generate_save_pairs_object(
        DIR_AUTOGRADER,
        p_queue,
        queue.PriorityQueue.min_extract,
        inputs,
        file_name='PriorityQueue.min_extract_' + q_type)


@app.command()
def queue_is_member(q_type='mono'):
    '''
    Test presence of keys on a model queue
    '''

    p_queue = queue_data()[q_type]
    inputs = (("Apples", ), ("Onions", ), (12.5, ), (True, ), (False, ))
    test_data.generate_save_pairs_object(DIR_AUTOGRADER,
                                         p_queue,
                                         queue.PriorityQueue.is_member,
                                         inputs,
                                         file_name='PriorityQueue.is_member' +
                                         q_type)


@app.command()
def queue_insert():
    '''
    Test insertion in the queue
    '''

    p_queue = queue_data()['empty']
    inputs = (("Apples", 1), ("Onions", -1), (12.5, 0), (True, 0.5), (False,
                                                                      0.7))

    def is_member_as_set(obj, *args):
        obj.insert(*args)
        return set(obj.queue_list)

    test_data.generate_save_pairs_object(DIR_AUTOGRADER,
                                         p_queue,
                                         is_member_as_set,
                                         inputs,
                                         file_name='PriorityQueue.insert')


@app.command()
def queue_all():
    '''
    All test data for PriorityQueue
    '''
    queue_min_extract('mono')
    queue_min_extract('bi')
    queue_min_extract('small', 2)
    queue_min_extract('empty', 2)
    queue_is_member('mono')
    queue_is_member('bi')


if __name__ == '__main__':
    app()
