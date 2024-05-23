import py_trees
from rclpy.qos import qos_profile_sensor_data
from communication_interfaces.msg import RangeData

from py_trees.common import Status
from rclpy.node import Node
import rclpy


from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions



class Obstacle_detection(py_trees.behaviour.Behaviour, Node):
    def __init__(self):
        Node.__init__(self,"obstacle_detection")
        py_trees.behaviour.Behaviour.__init__(self,"obstacle_detection")
        self.obstacle_free = False
        self.declare_parameters(
            namespace='',
            parameters=[
                ('obstacle_detection_max_range', 0.0),
                ('obstacle_detection_min_range', 0.0),
                ('obstacle_detection_threshold', 0)
            ])

    def setup(self):
        self.range_data_subscription= self.create_subscription(
            RangeData,
            self.get_namespace() + '/range_data',
            self.range_data_callback,
            qos_profile=qos_profile_sensor_data
        )
    def initialise(self) -> None:

        self.param_max_range = float(self.get_parameter(
            "obstacle_detection_max_range").get_parameter_value().double_value)
        self.param_min_range = float(self.get_parameter(
            "obstacle_detection_min_range").get_parameter_value().double_value)
        self.param_threshold = int(self.get_parameter(
            "obstacle_detection_threshold").get_parameter_value().integer_value)

    def update(self):
        rclpy.spin_once(self, timeout_sec=0)
        
        if self.obstacle_free:
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.FAILURE
    
    def range_data_callback(self, msg):
        # self.get_logger().info(msg.ranges)
        ranges = ScanCalculationFunctions.adjust_ranges(msg.ranges, self.param_min_range, self.param_max_range)
        before  = self.obstacle_free
        self.obstacle_free = ScanCalculationFunctions.is_obstacle_free(self.param_max_range, ranges, self.param_threshold)
        if before != self.obstacle_free:
            self.get_logger().info("It is now obstacle free: ", self.obstacle_free)
        