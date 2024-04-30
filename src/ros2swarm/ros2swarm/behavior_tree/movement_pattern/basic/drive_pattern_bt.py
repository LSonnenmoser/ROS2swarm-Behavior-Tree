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



class DrivePatternBT(MovementPatternBT, py_trees.behaviour.Behaviour):
    """
    A simple pattern for turning a constant time with a constant speed.

    Which is configured in with the parameters of this pattern.
    """

    def __init__(self):
        """Initialize the drive pattern."""
        py_trees.behaviour.Behaviour.__init__(self,'drive_pattern')
        MovementPatternBT.__init__(self,'drive_pattern')


        self.declare_parameters(
            namespace='',
            parameters=[
                ('drive_timer_period', 0.0),
                ('drive_linear', 0.0),
                ('drive_angular', 0.0),
            ])
    
    def setup(self): 
        """Initialize the aggregation pattern node.""" 

        self.logger.debug("  %s [DrivePatternBT::setup()]" % self.name)

    def initialise(self):
        """Initialize the pattern node."""
        self.get_logger().debug("  %s [DrivePatternBT::initialise()]" % self.name)


        timer_period = float(
            self.get_parameter("drive_timer_period").get_parameter_value().double_value)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.param_x = float(self.get_parameter("drive_linear").get_parameter_value().double_value)
        self.param_z = float(
            self.get_parameter("drive_angular").get_parameter_value().double_value)


    def update(self):

        """ spin node once """

        self.get_logger().debug("  %s [DrivePatternBT::update()]" % self.name)

        self.feedback_message = "spin drive pattern once"
        rclpy.spin_once(self, timeout_sec=0)

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):

        """ destroy node """

        self.get_logger().debug("  %s [DrivePatternBT::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

        self.timer.destroy();
        
    
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