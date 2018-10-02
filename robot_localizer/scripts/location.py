#!/usr/bin/env python

class Location(object):
    """ The class that contains information for a location object"""
    def __init__(self, x, y, theta, relative):
        self.x = x
        self.y = y
        self.theta = theta
        self.relative = relative