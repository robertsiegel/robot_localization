#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from sensor_msgs.msg import LaserScan

from helper_functions import TFHelper
from occupancy_field import OccupancyField

from particles import Particles
from constants import NUM_INITIAL_PARTICLES, NEW_PARTICLES

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

    def add_noise_to_particles(self, position_change):
        # OUTPUT FORMAT : List[Tuple(x, y, theta, original particle position)]
        current_particles = self.particles.get_locations()
        new_particles = []

        for particle in current_particles:
            for i in NEW_PARTICLES:
                x_noise = math.random.normal(loc=0, scale=.75)
                y_noise = math.random.normal(loc=0, scale=.75)
                theta_noise = math.random.normal(loc=0, scale=45)
                # appending (x, y, theta, original particle position)
                new_particles.append((particle[0] + position_change.x + x_noise, particle[1] + position_change.y + y_noise, (particle[2] + position_change.z + theta_noise) % 360, particle))


        
        # # setting min/max values for noise to be added
        # # TODO: UPDATE THIS TO BE BASED ON A NORMAL DISTRIBUTION
        # # min_dist_noise = .1
        # # max_dist_noise = 2
        # # theta_noise = 1
        # if len(current_particles) < NUM_INITIAL_PARTICLES:
        #     for loc_tuple in potential_locations:
        #         for i in range(4)
        #             x = loc_tuple[0] + math.random.normal(loc=0, scale=0.75)
        #             y = loc_tuple[1] + math.random.normal(loc=0, scale=0.75)
        #             theta = loc_tuple[2] + math.random.normal(loc=0, scale=0.5)
        #             # total_delta = math.random.uniform(min_dist_noise, max_dist_noise)
        #             # x = loc_tuple[0] + math.random.uniform(0, math.sqrt(total_delta))
        #             # y = loc_tuple[1] + math.sqrt(total_delta**2 - x_delta**2)
        #             # theta = loc_tuple[2] + math.random.uniform(-theta_noise, theta_noise)
        #             self.particles.add_location((x,y,theta), 0)

        # each output particle in list is of fomrmat (x, y, theta, original particle position)
        return new_particles

    def calculate_particle_probs(self, particles):
        # OUTPUT FORMAT : List[Tuple(cur_loc, prev_loc, confidence weight)]
        # iterate through particles, determine likelihood of each
        total_weight = 0
        # particles = self.locations.getLocations()
        potential_locations = []
        for loc_tuple in particles:
            # prior_conf = particles[loc_tuple]
            measured_distance = get_closest_obstacle_from_laserscan(self.ranges)
            map_distance = occupancy_field.get_closest_obstacle_distance(loc_tuple[0], loc_tuple[1])
            
            if measured_distance == 0 and map_distance == 0:
                new_weight == 500
            elif measured_distance == 0 or map_distance == 0:
                new_weight == 0
            elif measured_distance-map_distance != 0:
                new_weight = 1/abs(measured_distance-map_distance)
            else:
                new_weight = 500
            # particles[loc_tuple] = new_weight
            # appending in format (cur_loc, prev_loc, confidence weight)
            potential_locations.append(((loc_tuple[0], loc_tuple[1], loc_tuple[2]), loc_tuple[3], new_weight))
            total_weight += new_weight

        # normalize the weight to be a probability
        # for loc_tuple in particles
        #     if total_weight != 0:
        #         particles[loc_tuple] = particles[loc_tuple]/total_weight
        if total_weight:
            return [(cur_loc, prev_loc, new_weight / total_weight) for cur_loc, prev_loc, new_weight in potential_locations]
        return potential_locations


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
        scan_sub = rospy.Subscriber('/scan', LaserScan, self.update_ranges)
        scan_sub = rospy.Subscriber('/pos_change', Vector3, self.position_update_listener)
        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()
        
    def position_update_listener(self, msg):
        pos_change = msg
        new_particles = self.add_noise_to_particles(pos_change)
        potential_locations = self.calculate_particle_probs(new_particles)
        self.particles.update_locations(potential_locations)


    def update_ranges(self, msg):
        # callback function from listener
        self.ranges = msg.ranges


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
