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

from rclpy.node import Node

from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node
from communication_interfaces.msg import Int8Message
from ros2swarm.utils.swarm_controll import SwarmState

from ros2swarm.behavior_tree.movement_pattern.basic.aggregation_pattern_bt import AggregationPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.attraction_pattern_bt import AttractionPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.attraction_pattern2_bt import AttractionPattern2BT
from ros2swarm.behavior_tree.movement_pattern.basic.dispersion_pattern_bt import DispersionPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.drive_pattern_bt import DrivePatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.magnetometer_pattern_bt import MagnetometerPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.minimalist_flocking_pattern_bt import MinimalistFlockingPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.random_walk_pattern_bt import RandomWalkPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.rat_search_pattern_bt import RatSearchPatternBT



class BehaviorTreePattern(Node):
    """The class for behavior tree patterns"""

    def __init__(self):
        """
        Initialize the common movement pattern functions.

        Passes the node name to the super node and creates the drive command topic,
        which is available for all movement patterns
        """
        
        super().__init__('behavior_tree')

        self.get_logger().info('Publishing : Init.')
        self.start_flag = False

        self.formBehaviorTree()

        
        self.current_msg = Twist()
        self.i = 0

        self.timer= self.create_timer(0.001, self.timer_callback)

        self.swarm_command_subscription = \
            self.create_subscription(Int8Message, '/swarm_command',
                                     self.swarm_command_callback, 10)

        




    def timer_callback(self):
            """Publish the configured twist message when called."""
            if self.start_flag:
                self.root.tick_once()

                self.get_logger().info('Ticking'+ str(self.i))
                self.i += 1

    def swarm_command_callback(self, msg: Int8Message):
        """
        Set the start flag to true if on the /swarm_command topic a SwarmState.START
        and to false if SwarmState.STOP is revised.If the flag is ture the callback is executed.

        ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"

        ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 0}"
        """
        self.get_logger().debug('Robot "{}" received command "{}"'.format(self.get_namespace(), msg.data))
        if msg.data == int(SwarmState.START):
            self.start_flag = True
        if msg.data == int(SwarmState.STOP):
            self.start_flag = False


    def formBehaviorTree(self):
        """ Create and setup the behavior tree
        BT nodes:
            py_trees.composites.Sequence(name, memory)
            py_trees.composites.Selector(name, memory)
            py.trees.composites.Parallel(name, policy)
            RandomwalkPatternBT()
        """
        patterns=[AttractionPatternBT(),
                  AggregationPatternBT(),  
                  AttractionPattern2BT(), 
                  DispersionPatternBT(), 
                  DrivePatternBT(), 
                  MagnetometerPatternBT(), 
                  MinimalistFlockingPatternBT(),
                  RandomWalkPatternBT(), 
                #   RatSearchPatternBT()
                  ]
        action2 = RandomWalkPatternBT()
        action3 = AggregationPatternBT()
        action = DrivePatternBT()
        action1 = DrivePatternBT()
        # self.root.add_child(action)
        self.root = py_trees.composites.Sequence("root", False, patterns)
        # self.root.add_child(action3)

        self.get_logger().info('Publishing : Setup.')

        for pattern in patterns:
            pattern.setup()

def main(args=None):
    """Create a node for the attraction pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, BehaviorTreePattern)

if __name__ == '__main__':
    main()