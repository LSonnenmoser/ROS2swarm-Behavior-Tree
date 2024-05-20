#!/usr/bin/env python3
#    Copyright 2020 Marian Begemann
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

from geometry_msgs.msg import Twist
import rclpy
import py_trees

from ros2swarm.utils import setup_node
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.behavior_tree.movement_pattern.movement_pattern_bt import MovementPatternBT
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions


class AttractionPattern2BT(MovementPatternBT, py_trees.behaviour.Behaviour):
    """
    Pattern to reach an group of the participating robots in the available area.

    The dispersion pattern node creates drive commands based on the laser scan messages it receive

    2 = using attraction_field instead of repulsion_field

    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """Initialize the attraction pattern node."""
        MovementPatternBT.__init__(self, 'attraction_pattern2')
        py_trees.behaviour.Behaviour.__init__(self,'attraction_pattern2')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('attraction2_max_range', 0.0),
                ('attraction2_min_range', 0.0),
                ('attraction2_front_attraction', 0.0),
                ('attraction2_threshold', 0),
                ('attraction2_linear_if_alone', 0.0),
                ('attraction2_angular_if_alone', 0.0),
                ('max_translational_velocity', 0.0),
                ('max_rotational_velocity', 0.0),
            ])
        self.direction_if_alone = Twist()


    def setup(self): 
        """Initialize the aggregation pattern node.""" 

        self.get_logger().debug("  %s [Attraction2::setup()]" % self.name)

    def initialise(self):
        """Initialize the attraction pattern node."""
        self.get_logger().debug("  %s [Attraction2::initialise()]" % self.name)

        self.scan_subscription = self.create_subscription(
            RangeData,
            self.get_namespace() + '/range_data',
            self.range_data_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.param_max_range = float(
            self.get_parameter("attraction2_max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "attraction2_min_range").get_parameter_value().double_value
        self.param_front_attraction = self.get_parameter(
            "attraction2_front_attraction").get_parameter_value().double_value
        self.param_threshold = self.get_parameter(
            "attraction2_threshold").get_parameter_value().integer_value
        self.param_linear_if_alone = self.get_parameter(
            "attraction2_linear_if_alone").get_parameter_value().double_value
        self.param_angular_if_alone = self.get_parameter(
            "attraction2_angular_if_alone").get_parameter_value().double_value
        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value
        
        self.direction_if_alone = Twist()
        self.direction_if_alone.linear.x = self.param_linear_if_alone
        self.direction_if_alone.angular.z = self.param_angular_if_alone

            
    def update(self):

        """ spin node once """

        self.get_logger().debug("  %s [Attraction2::update()]" % self.name)

        rclpy.spin_once(self, timeout_sec=0)

        self.feedback_message = "spin attraction pattern 2 once"

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):

        """ destroy node """

        self.get_logger().debug("  %s [Attraction2::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        
        # AttractionPattern2BT.destroy_node()


    def range_data_callback(self, incoming_msg):
        """Call back if a new scan msg is available."""
        direction = self.vector_calc(incoming_msg)
        self.command_publisher.publish(direction)

    def vector_calc(self, current_msg):
        """Calculate the direction vector for the current scan."""
        if current_msg is None:
            return Twist()

        direction, alone = ScanCalculationFunctions.attraction_field(
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


