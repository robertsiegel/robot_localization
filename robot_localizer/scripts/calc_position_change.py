#!/usr/bin/env python
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseWithCovariance, Pose, Point, Quaternion, TwistWithCovariance
from helper_functions import TFHelper
from constants import TIME_THROTTLE
import rospy, time
import math

class CalcPosNode(object):
    def __init__(self):
        rospy.init_node('calc_pos_node')
        self.pos_change_pub = rospy.Publisher('/pos_change', Vector3, queue_size=10)
        self.prev_pos = Vector3(0, 0, 0) # previous odometry of position and angle
        self.prev_time = time.time()
        self.transform_helper = TFHelper()

    def odometry_callback(self, msg):
        # operate on the msg
        # set self.prev_pos to the last used position
        if time.time() - self.prev_time < TIME_THROTTLE:
            return
        self.prev_time = time.time()
        # storing and working with absolute odometry, publishing relative changes
        transformed_pos = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose) # transform to x,y,yaw
        new_pos = Vector3(transformed_pos[0], transformed_pos[1], math.degrees(transformed_pos[2]))
        self.pos_change_pub.publish(new_pos.x - self.prev_pos.x, new_pos.y - self.prev_pos.y, (new_pos.z - self.prev_pos.z) % 360)
        self.prev_pos = new_pos
        
    def run(self):
        # read in the data (odometry)
        # calculate the change in position (difference between current and last position)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    CalcPosNode().run()