#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField

from particles import Particles
from constants import NUM_INITIAL_PARTICLES

class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()

        self.particles = Particles()
        self.particles.initialize_locations()

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

        # TODO this should be deleted before posting
        self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        # initialize your particle filter based on the xy_theta tuple

    def add_noise_to_particles(self):
        current_particles = self.particles.getLocations()
        if len(current_particles) < NUM_INITIAL_PARTICLES:
            for loc_tuple in potential_locations:
                # ADD 4 new noise locations for each particle



    def calculate_particle probs(self):
        # iterate through particles, determine likelihood of each
        potential_locations = self.locations.getLocations()
        for loc_tuple in potential_locations
            prior_conf = potential_locations[loc_tuple]


    def get_closest_obstacle_from_laserscan(self, ranges):
        # this function is used to calculate probabilties of each particle
        closest_laserscan = (0,0)
        for i in ranges():
            scan_val = ranges[i]
            if scan_val > 0 and (scan_val < closest_laserscan[1] or closest_laserscan[1] == 0):
                closest_laserscan = (i, ranges[i])
        return closest_laserscan

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
