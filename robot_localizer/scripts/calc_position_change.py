#!/usr/bin/env python
from sensor_msgs.msg import LaserScan
from location import Location
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseWithCovariance, Pose, Point, Quaternion, TwistWithCovariance
from helper_functions import TFHelper
from constants import TIME_THROTTLE
import rospy, time

class CalcPosNode(object):
    def __init__(self):
        rospy.init_node('calc_pos_node')
        self.pos_change_pub = rospy.Publisher('/pos_change', Vector3, queue_size=10)
        # self.prev_pos = Location(0, 0, 0) # previous odometry of position and angle
        self.prev_pos = Vector3(0, 0, 0) # previous odometry of position and angle
        self.prev_time = time.time()

    def odometry_callback(self, msg):
        # operate on the msg
        # set self.prev_pos to the last used position
        if time.time() - self.prev_time < TIME_THROTTLE:
            return
        self.prev_time = time.time()
        # storing and working with absolute odometry, publishing relative changes
        new_pos = Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z)
        self.pos_change_pub.publish(new_pos.x - self.prev_pos.x, new_pos.y - self.prev_pos.y, new_pos.z - self.prev_pos.z)
        self.prev_pos = new_pos
        
    def run(self):
        # read in the data (odometry)
        # calculate the change in position (difference between current and last position)
        rospy.Subscriber('/odometry', Odometry, self.odometry_callback) # find the topic odometry is being published to


if __name__ == '__main__':
    CalcPosNode().run()