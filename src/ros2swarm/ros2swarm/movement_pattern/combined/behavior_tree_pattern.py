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

from ros2swarm.utils import setup_node


from ros2swarm.behavior_tree.movement_pattern.basic.aggregation_pattern_bt import AggregationPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.attraction_pattern_bt import AttractionPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.attraction_pattern2_bt import AttractionPattern2BT
from ros2swarm.behavior_tree.movement_pattern.basic.dispersion_pattern_bt import DispersionPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.drive_pattern_bt import DrivePatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.magnetometer_pattern_bt import MagnetometerPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.minimalist_flocking_pattern_bt import MinimalistFlockingPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.random_walk_pattern_bt import RandomWalkPatternBT
from ros2swarm.behavior_tree.movement_pattern.basic.rat_search_pattern_bt import RatSearchPatternBT
from ros2swarm.behavior_tree.conditions.obstacle_detection import Obstacle_detection
from ros2swarm.behavior_tree.conditions.timer import Timer
from ros2swarm.behavior_tree.movement_pattern.basic.turn_pattern_bt import TurnPatternBT
from ros2swarm.abstract_pattern import AbstractPattern



class BehaviorTreePattern(AbstractPattern):
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
        
        self.timer= self.create_timer(0, self.swarm_command_controlled_timer(self.timer_callback))




    def timer_callback(self):
            """Publish the configured twist message when called."""
            if self.i==0:
                 self.get_logger().info('started')
            self.root.tick_once()

            # self.get_logger().info('Ticking'+ str(self.i))
            self.i += 1

    def formBehaviorTree(self):
        """ Create and setup the behavior tree
        BT nodes:
            py_trees.composites.Sequence(name, memory)
            py_trees.composites.Selector(name, memory)
            py.trees.composites.Parallel(name, policy)
            RandomwalkPatternBT()
        """
        patterns=[
                  AttractionPatternBT(),
                  DrivePatternBT(), 
                  TurnPatternBT()
                  ]
        condition = Timer()
        condition2 = Obstacle_detection()
        drive = py_trees.composites.Selector('drive', False, children=[condition, patterns[0]])
        # self.root.add_child(action)
        # self.root = py_trees.composites.Sequence('root', False, children=[drive, patterns[1]])
        self.root = py_trees.composites.Sequence("root", False, children=[condition2,patterns[1]])
        self.get_logger().info('Publishing : Setup.')

        condition.setup()

        for pattern in patterns:
            pattern.setup()





def main(args=None):
    """Create a node for the attraction pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, BehaviorTreePattern)

if __name__ == '__main__':
    main()