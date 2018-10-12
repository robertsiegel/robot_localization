#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from sensor_msgs.msg import LaserScan

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
        rospy.Subscriber("initialpose"
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
        self.particles.initialize_particles()

        self.ranges = []

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
        
        # setting min/max values for noise to be added
        # TODO: UPDATE THIS TO BE BASED ON A NORMAL DISTRIBUTION
        # min_dist_noise = .1
        # max_dist_noise = 2
        # theta_noise = 1
        if len(current_particles) < NUM_INITIAL_PARTICLES:
            for loc_tuple in potential_locations:
                for i in range(4)
                    x = loc_tuple[0] + math.random.normal(loc=0, scale=0.75)
                    y = loc_tuple[1] + math.random.normal(loc=0, scale=0.75)
                    theta = loc_tuple[2] + math.random.normal(loc=0, scale=0.5)
                    # total_delta = math.random.uniform(min_dist_noise, max_dist_noise)
                    # x = loc_tuple[0] + math.random.uniform(0, math.sqrt(total_delta))
                    # y = loc_tuple[1] + math.sqrt(total_delta**2 - x_delta**2)
                    # theta = loc_tuple[2] + math.random.uniform(-theta_noise, theta_noise)
                    self.particles.add_location((x,y,theta), 0)

    def calculate_particle probs(self):
        # iterate through particles, determine likelihood of each
        total_weight = 0
        particles = self.locations.getLocations()
        for loc_tuple in potential_locations
            prior_conf = potential_locations[loc_tuple]
            measured_distance = get_closest_obstacle_from_laserscan(self.ranges)
            map_distance = occupancy_field.get_closest_obstacle_distance(loc_tuple[0], loc_tuple[1])
            
            if measured_distance == 0 and map_distance == 0
                new_weight == 500
            elif measured_distance == 0 or map_distance == 0
                new_weight == 0
            elif measured_distance-map_distance != 0:
                new_weight = 1/abs(measured_distance-map_distance)
            else:
                new_weight = 500
            particles[loc_tuple] = new_weight
            total_weight += new_weight

        # normalize the weight to be a probability
        for loc_tuple in potential_locations
            if total_weight != 0:
                particles[loc_tuple] = particles[loc_tuple]/total_weight


    def get_closest_obstacle_from_laserscan(self):
        # this function is used to calculate probabilties of each particle
        closest_laserscan = (0,0)
        for i in self.ranges():
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
        
    def update_ranges(self, msg):
        # callback function from listener
        self.ranges = msg.ranges

    def listener(self):
        scan_sub = rospy.Subscriber('/scan', LaserScan, self.update_ranges)


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
