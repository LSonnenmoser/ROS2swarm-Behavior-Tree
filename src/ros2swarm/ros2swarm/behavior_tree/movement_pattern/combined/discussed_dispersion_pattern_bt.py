#!/usr/bin/env python3
#    Copyright 2021 Marian Begemann
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
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from communication_interfaces.msg import OpinionMessage, DoubleMessage


class DiscussedDispersionBT(MovementPattern, py_trees.behaviour.Behaviour):
    """
    Pattern to perform a dispersion of the swarm based a common first to discuss distance value.
    Keep distance at 1,2, or 3 meters, depending on common opinion 1 to 3.
    """

    def __init__(self):
        """Initialize the discussed dispersion pattern node."""
        MovementPattern.__init__(self,'discussed_dispersion_pattern')
        py_trees.behaviour.Behaviour.__init__(self,'discussed_dispersion_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_translational_velocity', 0.0),
                ('max_rotational_velocity', 0.0),
                ('discussed_dispersion_timer_period', 0.0),
                ('discussed_dispersion_discussion_base_distance', 0.0),
                ('discussed_dispersion_discussion_time', 0.0),
                ('discussed_dispersion_discussion_opinion_multiply', 0.0),
            ])
        self.counter = 0
        self.dispersion_latest = Twist()
        self.opinion_latest = OpinionMessage()
        self.message = DoubleMessage()


    def setup(self): 
        """Initialize the aggregation pattern node.""" 

        self.logger.debug("  %s [Foo::setup()]" % self.name)

        # messages from subpattern subscriptions
        self.dispersion_subpattern = self.create_subscription(
            Twist,
            self.get_namespace() + '/drive_command_dispersion_pattern',
            self.command_callback_dispersion,
            10)
            
        self.majority_rule_subpattern = self.create_subscription(
            OpinionMessage,
            self.get_namespace() + '/opinion',
            self.command_callback_majority_rule,
            10)

        self.dispersion_distance_publisher = self.create_publisher(DoubleMessage,
                                                       self.get_namespace() + '/dispersion_distance', 10)

    def initialise(self):
        """Initialize the attraction pattern node."""
        self.logger.debug("  %s [Foo::initialise()]" % self.name)

        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value
        timer_period = self.get_parameter(
            "discussed_dispersion_timer_period").get_parameter_value().double_value
        discussion_time = self.get_parameter(
            "discussed_dispersion_discussion_time").get_parameter_value().double_value
        self.base_distance = self.get_parameter(
            "discussed_dispersion_discussion_base_distance").get_parameter_value().double_value
        self.opinion_multiply = self.get_parameter(
            "discussed_dispersion_discussion_opinion_multiply").get_parameter_value().double_value

        self.timer = self.create_timer(timer_period,
                                       self.swarm_command_controlled_timer(self.discussed_dispersion_callback))
        self.switch = discussion_time / timer_period



    def update(self):

        """ spin node once """

        self.logger.debug("  %s [Foo::update()]" % self.name)

        self.feedback_message = "spin drive pattern once"

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):

        """ destroy node """

        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        
        DiscussedDispersionBT.destroy_node()

    def command_callback_dispersion(self, incoming_msg: Twist):
        """Assign the message to variable"""
        self.dispersion_latest = incoming_msg

    def command_callback_majority_rule(self, incoming_msg: OpinionMessage):
        """Assign the message to variable"""
        self.opinion_latest = incoming_msg

    def discussed_dispersion_callback(self):
        """ Controls the discussed dispersion pattern. """

        if self.counter <= self.switch:
            self.get_logger().debug('Opinion is: "{}"'.format(self.opinion_latest.opinion))
            self.counter += 1
        else:
            distance = self.base_distance + (self.opinion_latest.opinion * self.opinion_multiply)
            self.message.data = distance if distance <= 3.5 else 3.5 # TODO or zero?
            self.dispersion_distance_publisher.publish(self.message)
            self.command_publisher.publish(self.dispersion_latest)
