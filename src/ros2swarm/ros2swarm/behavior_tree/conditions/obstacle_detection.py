import py_trees
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from py_trees.common import Status

from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions



class Obstacle_detection(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("avoid")
        self.obstacle_free = False

    def setup(self):
        self.range_data_subscription= self.create_subscription(
            HazardDetectionVector,
            self.get_namespace() + '/range_data',
            self.swarm_command_controlled(self.range_data_callback),
            qos_profile=qos_profile_sensor_data
        )
    def initialise(self) -> None:
        self.obstacle_free = False

        self.param_max_range = float(self.get_parameter(
            "obstacle_detection_max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "obstacle_detection_min_range").get_parameter_value().double_value
        self.param_threshold = self.get_parameter(
            "obstacle_detection_threshold").get_parameter_value().integer_value

    def update(self) -> Status:
        if self.obstacle_free:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
    
    def range_data_callback(self, msg):
       
        ranges = ScanCalculationFunctions.adjust_ranges(msg.ranges, self.param_min_range, self.param_max_range)
        self.obstacle_free = ScanCalculationFunctions.is_obstacle_free(self.param_max_range, ranges, self.param_threshold)