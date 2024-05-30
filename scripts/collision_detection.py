''' Praticle and Fast Collision detection for autonomous driving.
objects: Rectangle and circle.
According to the implemetation difficulty. 3 method are used.
methods:  seperating axis therom; double circle method; particle model;

Ref:

'''
import numpy as np
import os, math, yaml, time
from math import cos, sin, sqrt
from collision_geometry import Rectangle, Circle
import matplotlib.pyplot as plt

# path_cfg = os.path.join("configs", "control.yaml")
# f_cfg = open(path_cfg)
# car_cfg = yaml.load(f_cfg)


def collision_rect_and_rect(rect1: Rectangle, rect2: Rectangle):
    ''' Use seperating axis therom (Reference to apollo). \
        Judge collision between 2 rectangles.

    '''
    shift_x = rect2.x - rect1.x
    shift_y = rect2.y - rect1.y

 
    cos_v = math.cos(rect1.yaw)
    sin_v = math.sin(rect1.yaw)
    cos_o = math.cos(rect2.yaw)
    sin_o = math.sin(rect2.yaw)
    half_l_v = rect1.length/2
    half_w_v = rect1.width/2
    half_l_o = rect2.length/2
    half_w_o = rect2.width/2

    dx1 = cos_v * rect1.length/2
    dy1 = sin_v * rect1.length/2
    dx2 = sin_v * rect1.width/2
    dy2 = -cos_v * rect1.width/2

    dx3 = cos_o * rect2.length/2
    dy3 = sin_o * rect2.length/2
    dx4 = sin_o * rect2.width/2
    dy4 = -cos_o * rect2.width/2

    # use seperating axis therom
    return ((abs(shift_x * cos_v + shift_y * sin_v) <=
             abs(dx3 * cos_v + dy3 * sin_v) + abs(dx4 * cos_v + dy4 * sin_v) + half_l_v)
            and (abs(shift_x * sin_v - shift_y * cos_v) <=
                 abs(dx3 * sin_v - dy3 * cos_v) + abs(dx4 * sin_v - dy4 * cos_v) + half_w_v)
            and (abs(shift_x * cos_o + shift_y * sin_o) <=
                 abs(dx1 * cos_o + dy1 * sin_o) + abs(dx2 * cos_o + dy2 * sin_o) + half_l_o)
            and (abs(shift_x * sin_o - shift_y * cos_o) <=
                 abs(dx1 * sin_o - dy1 * cos_o) + abs(dx2 * sin_o - dy2 * cos_o) + half_w_o))


def collision_circle_and_rect(c:Circle, r:Rectangle):
    ''' dectect 2 + 1 axis. https://www.sevenson.com.au/programming/sat/
    '''
    # find the nearest vertices to circle
    if not hasattr(r, 'vertices'):
        vertices = r.calc_vertices()
    d_min = np.Inf
    ind = -1

    for i_v in range(vertices.shape[0]):
        d = ((vertices[i_v, 0] - c.x)**2 + (vertices[i_v, 1] - c.y)**2)**0.5 - c.r
        if d < d_min:
            d_min = d
            ind = i_v
    
    if d_min<0:
        return True
    
    circle_center = np.array([c.x, c.y])
    
    yaw = r.yaw
    axes = [[cos(yaw), sin(yaw)], [-sin(yaw), cos(yaw)]]

    axis_point = vertices[ind, :] - circle_center
    axis_point = normalize(axis_point)

    axes.append(axis_point)
    for axis in axes:
        projection_a = project(vertices, axis)
        projection_c = project([circle_center], axis)[0]
        projection_b = [projection_c-c.r, projection_c+c.r]

        overlapping = overlap(projection_a, projection_b)
        # if exist axis to seperate, then no collision.
        if not overlapping:
            return False

    return True
    

def normalize(vector):
    """
    :return: The vector scaled to a length of 1
    """
    norm = sqrt(vector[0] ** 2 + vector[1] ** 2)
    return vector[0] / norm, vector[1] / norm


def dot(vector1, vector2):
    """
    :return: The dot (or scalar) product of the two vectors
    """
    return vector1[0] * vector2[0] + vector1[1] * vector2[1]


def edge_direction(point0, point1):
    """
    :return: A vector going from point0 to point1
    """
    return point1[0] - point0[0], point1[1] - point0[1]


def orthogonal(vector):
    """
    :return: A new vector which is orthogonal to the given vector
    """
    return vector[1], -vector[0]


def vertices_to_edges(vertices):
    """
    :return: A list of the edges of the vertices as vectors
    """
    return [edge_direction(vertices[i], vertices[(i + 1) % len(vertices)])
            for i in range(len(vertices))]


def project(vertices, axis):
    """
    :return: A vector showing how much of the vertices lies along the axis
    """
    dots = [dot(vertex, axis) for vertex in vertices]
    return [min(dots), max(dots)]



def overlap(projection1, projection2):
    """
    :return: Boolean indicating if the two projections overlap
    """
    return min(projection1) <= max(projection2) and \
        min(projection2) <= max(projection1)


def separating_axis_theorem(vertices_a, vertices_b):
    ''' slow but unified function for convex polygon collision dectection.
    '''
    edges = vertices_to_edges(vertices_a) + vertices_to_edges(vertices_b)
    axes = [normalize(orthogonal(edge)) for edge in edges]

    for axis in axes:
        projection_a = project(vertices_a, axis)
        projection_b = project(vertices_b, axis)

        overlapping = overlap(projection_a, projection_b)

        # if exist axis to seperate, then no collision.
        if not overlapping:
            return False

    return True


def get_dist_circle_collision(c1: Circle, c2: Circle):
    # d_c1_to_c2 - (r1 + r2)
    return ((c1.x - c2.x)**2 + (c1.y - c2.y)**2)**0.5 - (c1.r + c2.r)

def calc_dis(x1, y1, x2, y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def is_collision(o1, o2):
    ''' use seperating axis theorem
    '''
    if type(o1) == Circle and type(o2) == Circle:
        return get_dist_circle_collision(o1, o2)<0
    elif type(o1) == Rectangle  and type(o2) == Rectangle:
        return collision_rect_and_rect(o1, o2)
    else:
        (c, r) = (o1, o2) if type(o1)==Circle else (o2, o1)
        return collision_circle_and_rect(c, r)


def is_collision_double_circle(obj1, obj2):
    ''' detect distance between 2 objects. 
    Use double circles to represent a rectangle.
    [abandoned]: Slower than the not simple version.
    '''
    if type(obj1) == Circle and type(obj2) == Circle:
        return get_dist_circle_collision(obj1, obj2)<0
    elif type(obj1) == Rectangle  and type(obj2) == Rectangle:
        xf1, yf1, xr1, yr1 = get_disc_positions(obj1.x, obj1.y, obj1.yaw, obj1.length)
        xf2, yf2, xr2, yr2 = get_disc_positions(obj2.x, obj2.y, obj2.yaw, obj2.length)
        r1 = obj1.r
        r2 = obj2.r
        i1 =calc_dis(xf1, yf1, xf2, yf2) -(r1 + r2) < 0
        i2 =calc_dis(xf1, yf1, xr2, yr2) -(r1 + r2) < 0
        i3 =calc_dis(xr1, yr1, xf2, yf2) -(r1 + r2) < 0
        i4 =calc_dis(xr1, yr1, xr2, yr2) -(r1 + r2) < 0
        return i1 | i2 | i3 | i4

    else:
        (c, r) = (obj1, obj2) if type(obj1)==Circle else (obj2, obj1)

        return collision_circle_and_rect(c, r)
    pass


def get_disc_positions(x, y, theta, L):
    """_summary_
    Args:
        L (double): car length
    Returns:
        _type_: _description_
    """
    if type(x) is np.ndarray:
        cos, sin = np.cos, np.sin
    else:
        cos, sin = math.cos, math.sin

    f2x = 1/4 * L
    r2x = - 1/4 * L

    xf = x + f2x * cos(theta)
    xr = x + r2x * cos(theta)
    yf = y + f2x * sin(theta)
    yr = y + r2x * sin(theta)
    return xf, yf, xr, yr

def get_disc_positions_from_real_axis_center(x, y, theta, car_len, Lr):
    """
    Lr : rear axis center to vehicle backend
    """
    if type(x) is np.ndarray:
        cos, sin = np.cos, np.sin
    else:
        cos, sin = math.cos, math.sin

    f2x = 1/4 * 3*car_len - Lr
    r2x = 1/4 * car_len - 3*Lr/4

    xf = x + f2x * cos(theta)
    xr = x + r2x * cos(theta)
    yf = y + f2x * sin(theta)
    yr = y + r2x * sin(theta)
    return xf, yf, xr, yr



