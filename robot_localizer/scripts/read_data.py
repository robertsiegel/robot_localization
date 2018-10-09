from sensor_msgs.msg import LaserScan
from location import Location
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseWithCovariance, Pose, Point, Quarternion, TwistWithCovariance
from helper_functions import TFHelper
import rospy

class ReadDataNode():
    def __init__(self):
        rospy.init_node('read_data_node')
        self.pub = rospy.Publisher('/input_data', )
        
    def run(self):

