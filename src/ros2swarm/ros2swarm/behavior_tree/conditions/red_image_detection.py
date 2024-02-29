import numpy as np
import py_trees
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from py_trees.common import Status
import cv2
from cv_bridge import CvBridge




class RedImageDetection(py_trees.behaviour.Behaviour):
    """
    If not enough red pixels are detetected the condition returns SUCCESS, else FAILURE"""
    def __init__(self):
        super().__init__("avoid")
        self.RedDetected = False

    def setup(self):
            self.subscription = self.create_subscription(
            Image,
            '/Turtlebot4/rgb_camera',
            self.image_callback,
            1
        )

    def initialise(self) -> None:
        self.RedDetected = False

    def update(self) -> Status:
        if self.RedDetected:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
    
    def image_callback(self, msg):

        lower_color_bound = np.array([0, 100, 100])
        upper_color_bound = np.array([10, 255, 255])

        bridge = CvBridge()

        #converting to opencv and hsv frame
        rgb_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        hsv_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)

        # color mask
        colored_mask = cv2.inRange(hsv_frame, lower_color_bound, upper_color_bound)

        # using color mask on image
        colored_regions = cv2.bitwise_and(rgb_frame, rgb_frame, mask=colored_mask)

        #filter non red points
        red_points = rgb_frame[np.nonzero(colored_mask)]

        red_point_count = red_points.shape[0]


        if red_point_count>100:
            self.RedDetected = True;
        else:
            self.RedDetected = False;
            