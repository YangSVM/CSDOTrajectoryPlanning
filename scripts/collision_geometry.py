''' Praticle Collision detection for autonomous driving.
Rectangle and circle.
'''
import numpy as np
import os, yaml, time
from math import cos, sin


class Rectangle:
    def __init__(self, x, y, yaw, length=3, width=2, pos='center', lb=1):

        '''
        Params:
            pos: whether the point (x,y) represents the 'center' or 'rear_axle_center'
            LB: if pos is 'rear_axle_center'. lb is the distance between rear_axle_center and rear.
        '''
        self.yaw = yaw
        self.length = length
        self.width = width
        if pos == 'center':
            self.x = x
            self.y = y
        elif pos == 'rear_axle_center':
            d = length/2-lb
            self.x = x + d*cos(yaw)
            self.y = y+ d*sin(yaw)
            self.xr = x     # rear axle x
            self.yr = y     # rear axle


    def calc_vertices(self):
        '''
        '''
        # vertices relative displacement to center
        v_d = np.array([[self.length, self.width],\
                    [-self.length, self.width],\
                    [-self.length, -self.width],\
                    [self.length, -self.width]])
        v_d = v_d/2
        
        yaw = self.yaw
        rotate_matrix = np.array([[cos(yaw), -sin(yaw)],\
            [sin(yaw), cos(yaw)]])
        
        v_d = v_d @ rotate_matrix.T

        # add the displacement
        v = v_d + np.array([self.x, self.y])
        self.vertice = v
        return v

class Circle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

