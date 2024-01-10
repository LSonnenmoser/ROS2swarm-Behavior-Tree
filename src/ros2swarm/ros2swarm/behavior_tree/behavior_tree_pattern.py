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
import py_trees
import rclpy 
import time

from ros2swarm.abstract_pattern import AbstractPattern
from ros2swarm.behavior_tree.movement_pattern.basic.attraction_pattern2_bt import AttractionPattern2BT
from ros2swarm.utils import setup_node


class BehaviorTreePattern(AbstractPattern):
    """The class for behavior tree patterns"""

    def __init__(self):
        """
        Initialize the common movement pattern functions.

        Passes the node name to the super node and creates the drive command topic,
        which is available for all movement patterns
        """
        super().__init__('behavior_tree')
        rclpy.init()
        root = py_trees.composites.Sequence("root", False)
        action = AttractionPattern2BT()
        root.add_child(action)

        action.setup()



        for _unused_i in range(0, 12):

            root.tick_once()

            time.sleep(0.5)


    def swarm_command_false_case(self):
        """If swarm command is false the robot should not move."""
        self.command_publisher.publish(Twist())

    def destroy_node(self):
        """Send a stop twist message and calls the super destroy method."""
        self.command_publisher.publish(Twist())
        super().destroy_node()

def main(args=None):
    """Create a node for the attraction pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, AttractionPattern2BT)


if __name__ == '__main__':
    main()