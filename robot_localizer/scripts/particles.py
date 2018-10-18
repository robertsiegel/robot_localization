#!/usr/bin/env python
from geometry_msgs.msg import Vector3, PoseArray, Pose
from occupancy_field import OccupancyField
from constants import NUM_INITIAL_PARTICLES, CUTOFF_THRESHOLD
import numpy as np
import math, rospy, random

class Particles():
    """ The class that represents ROS Node to store the particles and each of their potential
    locations and probability values """
    def __init__(self):
        self.occupancy_field = OccupancyField()
        self.locations = {} # location to be stored as tuple(x, y, theta): confidence
        self.confidence_func = lambda new_confidence, old_confidence: (new_confidence + old_confidence) / 2
        self.pc_pub = rospy.Publisher('/particlecloud', PoseArray, queue_size=10)

    def vector_to_tuple(self, v):
        return v

    def initialize_particles(self):
        # function to initialize the locations when the particle filter begins to run

        # determine max height and width from map
        height = self.occupancy_field.map.info.height * self.occupancy_field.map.info.resolution * .2
        width = self.occupancy_field.map.info.width * self.occupancy_field.map.info.resolution * .2
        
        initial_prob = 1.0/NUM_INITIAL_PARTICLES # equal probability weight for each initial particle

        # initialize a number of particles given by constant
        for i in range(NUM_INITIAL_PARTICLES):
            x = np.random.normal(loc=0, scale=1.5)
            y = np.random.normal(loc=0, scale=1.5)
            theta = math.radians(np.random.randint(359))
            self.locations[(x,y,theta)] = initial_prob

    def add_particle(self, location, prob):
        # adds a particle to the locations array
        self.locations[location] = prob

    def update_locations(self, potential_locations):
        # just assuming the structure of potential_locations to be a list of tuples (cur_loc(vector3), prev_loc, confidence)
        # going to average old confidence with new confidence
        new_locations = [(cur_loc, prev_loc, self.confidence_func(confidence, self.locations[self.vector_to_tuple(prev_loc)])) for cur_loc, prev_loc, confidence in potential_locations]
        self.locations = {self.vector_to_tuple(cur_loc): confidence for cur_loc, prev_loc, confidence in sorted(new_locations, key=lambda x: x[2], reverse=True)[:int(len(new_locations) * CUTOFF_THRESHOLD)]}

        self.publish_particle_markers()

    def publish_particle_markers(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        for location in self.locations:
            pose = Pose()
            pose.position.x = -location[0]
            pose.position.y = -location[1]
            pose.orientation.z = location[2]
            pose_array.poses.append(pose)

        self.pc_pub.publish(pose_array)

    def get_locations(self):
        return self.locations
    


