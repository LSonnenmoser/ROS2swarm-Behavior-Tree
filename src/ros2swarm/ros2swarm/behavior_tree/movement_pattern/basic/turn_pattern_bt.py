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
import py_trees
import rclpy


from geometry_msgs.msg import Twist
from ros2swarm.behavior_tree.movement_pattern.movement_pattern_bt import MovementPatternBT



class TurnPatternBT(MovementPatternBT, py_trees.behaviour.Behaviour):
    """
    A simple pattern for driving a constant direction vector.

    Which is configured in with the parameters of this pattern.
    How often the direction is published is configured in the timer period parameter.
    """

    def __init__(self):
        """Initialize the turn pattern."""
        py_trees.behaviour.Behaviour.__init__(self,'turn_pattern')
        MovementPatternBT.__init__(self,'turn_pattern')


        self.declare_parameters(
            namespace='',
            parameters=[
                ('turn_timer_period', 0.0),
                ('turn_linear', 0.0),
                ('turn_angular', 0.0),
            ])
    
    def setup(self): 
        """Initialize the aggregation pattern node.""" 

        self.logger.debug("  %s [TurnPatternBT::setup()]" % self.name)

        timer_period = float(
            self.get_parameter("turn_timer_period").get_parameter_value().double_value)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.param_x = float(self.get_parameter("turn_linear").get_parameter_value().double_value)
        self.param_z = float(
            self.get_parameter("turn_angular").get_parameter_value().double_value)

    def initialise(self):
        """Initialize the attraction pattern node."""
        self.get_logger().debug("  %s [TurnPatternBT::initialise()]" % self.name)



    def update(self):

        """ spin node once """

        self.get_logger().debug("  %s [TurnPatternBT::update()]" % self.name)

        self.feedback_message = "spin turn pattern once"
        rclpy.spin_once(self, timeout_sec=0)

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):

        """ destroy node """

        self.get_logger().debug("  %s [TurnPatternBT::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        
    
    def timer_callback(self):
        """Publish the configured twist message when called."""

        msg = Twist()
        # command to publish the message in the terminal by hand
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{
        # linear: {x: 0.26, y: 0.0, z: 0.0},
        # angular: {x: 0.0, y: 0.0, z: 0.0}
        # }"
        msg.linear.x = self.param_x
        msg.angular.z = self.param_z
        self.command_publisher.publish(msg)
        self.get_logger().info('Publishing {}:"{}"'.format(self.i, msg))
        self.i += 1