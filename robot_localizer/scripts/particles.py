#!/usr/bin/env python
from typing import List
from geometry_msgs.msg import Vector3
from occupancy_field import OccupancyField
from constants import NUM_INITIAL_PARTICLES, PROB_THRESHOLD
import numpy as np

class Particles():
    """ The class that represents ROS Node to store the particles and each of their potential
    locations and probability values """
    def __init__(self):
        self.occupancy_field = OccupancyField()
        self.locations = {} # location to be stored as tuple(x, y, theta): confidence
        self.confidence_func = lambda new_confidence, old_confidence: (new_confidence + old_confidence) / 2

    def vector_to_tuple(v):
        return (v.x, v.y, v.z)

    def initialize_locations(self):
        # function to initialize the locations when the particle filter begins to run
        initial_conf = 1.0/NUM_INITIAL_PARTICLES
        for i in range(NUM_INITIAL_PARTICLES):
            x = (np.random.randint(100)-50)/10.0 # gives range of -5 to 5 m
            y = (np.random.randint(100)-50)/10.0
            theta = random.randint(359)
            self.locations[(x,y,theta)] = initial_conf

    def update_locations(potential_locations):
        # just assuming the structure of potential_locations to be a list of tuples (cur_loc(vector3), prev_loc, confidence)
        # going to average old confidence with new confidence
        new_locations = {}
        for cur_loc, prev_loc, confidence in potential_locations:
            # new_conf = (confidence + self.locations[self.vector_to_tuple(prev_loc)]) / 2
            new_conf = self.confidence_func(confidence, self.locations[self.vector_to_tuple(prev_loc)])
            if new_conf >= PROB_THRESHOLD:
                new_locations[self.vector_to_tuple(cur_loc)] = new_conf
        

    def display_particle_markers(self, location)
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.ARROW
        marker.pose.position.x = location[0]
        marker.pose.position.y = location[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = location[2]
        marker.pose.orientation.w = 0
        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .1
        marker.color.g = 1

    def get_locations():
        return self.locations
    


