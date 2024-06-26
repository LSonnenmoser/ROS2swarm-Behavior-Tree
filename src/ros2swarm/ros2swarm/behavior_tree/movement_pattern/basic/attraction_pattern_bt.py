
import py_trees

import rclpy


from geometry_msgs.msg import Twist
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions
from ros2swarm.behavior_tree.movement_pattern.movement_pattern_bt import MovementPatternBT



class AttractionPatternBT(MovementPatternBT, py_trees.behaviour.Behaviour):

    def __init__(self):

        """Initialize the attraction pattern node."""
        MovementPatternBT.__init__(self,'attraction_pattern')
        py_trees.behaviour.Behaviour.__init__(self,'attraction_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('attraction_max_range', 0.0),
                ('attraction_min_range', 0.0),
                ('attraction_front_attraction', 0.0),
                ('attraction_threshold', 0),
                ('attraction_linear_if_alone', 0.0),
                ('attraction_angular_if_alone', 0.0),
                ('max_translational_velocity', 0.0),
                ('max_rotational_velocity', 0.0)
            ])
        self.direction_if_alone = Twist()




    def setup(self):
        """Initialize the attraction pattern node."""
        self.get_logger().debug("  %s [Attraction::setup()]" % self.name)

        self.scan_subscription = self.create_subscription(
            RangeData,
            self.get_namespace() + '/range_data',
            self.range_data_callback,
            qos_profile=qos_profile_sensor_data
        )
        

    def initialise(self):

        """Initialize the attraction pattern node."""
        self.get_logger().debug("  %s [Attraction::initialise()]" % self.name)
        

        self.param_max_range = float(
            self.get_parameter("attraction_max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "attraction_min_range").get_parameter_value().double_value
        self.param_front_attraction = self.get_parameter(
            "attraction_front_attraction").get_parameter_value().double_value
        self.param_threshold = self.get_parameter(
            "attraction_threshold").get_parameter_value().integer_value
        self.param_linear_if_alone = self.get_parameter(
            "attraction_linear_if_alone").get_parameter_value().double_value
        self.param_angular_if_alone = self.get_parameter(
            "attraction_angular_if_alone").get_parameter_value().double_value
        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value

        self.direction_if_alone = Twist()
        self.direction_if_alone.linear.x = self.param_linear_if_alone
        self.direction_if_alone.angular.z = self.param_angular_if_alone




    def update(self):

        """

        spin node once

        """

        self.get_logger().debug("  %s [Attraction::update()]" % self.name)

        self.feedback_message = "spin attraction pattern once"
        rclpy.spin_once(self, timeout_sec=0)

        return py_trees.common.Status.RUNNING





    def terminate(self, new_status):

        """

        destroy node

        """

        self.get_logger().debug("  %s [Attraction::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


    def range_data_callback(self, incoming_msg):
        """Call back if a new scan msg is available."""
        direction = self.vector_calc(incoming_msg)
        self.command_publisher.publish(direction)

    def vector_calc(self, current_msg):
        """Calculate the direction vector for the current range data."""
        if current_msg is None:
            return Twist()

        direction, alone = ScanCalculationFunctions.repulsion_field(
            self.param_front_attraction,
            self.param_max_range,
            self.param_max_rotational_velocity,
            self.param_max_translational_velocity,
            self.param_min_range,
            self.param_threshold,
            current_msg.ranges,
            current_msg.angles)

        direction = self.direction_if_alone if alone else direction

        return direction
    

