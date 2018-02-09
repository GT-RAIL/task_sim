#!/usr/bin/env python

# Python
from random import random

# ROS
from geometry_msgs.msg import Point

class GraspState():

    def __init__(self, neighbors = 0, position = Point(0, 0, 0)):
        self.neighbors = None
        self.position = None
        self.updateGraspRate(neighbors, position)

    def updateGraspRate(self, neighbors, position):
        if neighbors == self.neighbors and position == self.position:
            return
        self.graspable = random() < pow(2, -.25*neighbors)
        self.neighbors = neighbors
        self.position = position