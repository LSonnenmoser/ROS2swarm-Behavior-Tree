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

from ros2swarm.behavior_tree.conditions.red_image_detection import RedImageDetection
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
        Initialize behavor treee node

        Passes the node name to the super node and subscribes to the swarm command
        """
        
        super().__init__('behavior_tree')

        self.get_logger().info('Publishing : Init.')
        self.start_flag = False

        self.formBehaviorTree()

        
        self.current_msg = Twist()
        self.i = 0
        
        #starts callback after swarm command
        self.timer= self.create_timer(0, self.swarm_command_controlled_timer(self.timer_callback))




    def timer_callback(self):
            """Tick the behaviortree."""
            if self.i==0:
                 self.get_logger().info('started')
            self.root.tick_once()

            # self.get_logger().info('Ticking'+ str(self.i))
            self.i += 1


    def formBehaviorTree(self):
        """ Create and setup the behavior tree
        BT nodes:
            py_trees.composites.Sequence(name, memory, children)
            py_trees.composites.Selector(name, memory, children)
            py.trees.composites.Parallel(name, policy, children)

            RandomwalkPatternBT()
            AggregationPatternBT(),
            AttractionPatternBT(),
            AttractionPattern2BT(),
            DispersionPatternBT(),
            DrivePatternBT(), 
            MagnetometerPatternBT(),
            MinimalistFlockingPatternBT(),
            RandomWalkPatternBT(),
            RatSearchPatternBT(),
            TurnPatternBT(),

            Timer()
            Obstacle_detection()
            RedImageDetection()


            For a new Behavior add it like the example_pattern_bt.py to the behaviortree folder and add the parameters to the config file of the behaviortrees for every robot.


            every node/leaf used in the behavior tree needs to be a unique node/object
            every node needs to be initializied and setup before they are getting ticked
        """
        self.get_logger().info('Publishing : Setup.')

        # initialize
        behaviors=[
                AggregationPatternBT(),
                AttractionPatternBT(),
                AttractionPattern2BT(),
                DispersionPatternBT(),
                DrivePatternBT(), 
                MagnetometerPatternBT(),
                MinimalistFlockingPatternBT(),
                RandomWalkPatternBT(),
                RatSearchPatternBT(),
                TurnPatternBT(),
                ]
        conditions = [Timer(5),
                      Obstacle_detection(),
                      RedImageDetection()
        ]

        # setup
        for pattern in behaviors:
            pattern.setup()
        
        for condition in conditions:
             condition.setup() 

        # form behavior tree
        """
        exampe behavior tree:
        drive straight until obstacle is detected, as long as there is a obstacle turn on place,
        keep in mind that there is a hardware protection layer, you can change the parameters like the range in the conifg file

                                    ||
                        ->                      Turn
                obstacle    Drive               Pattern
                detection   pattern
        """

        self.root = py_trees.composites.Selector("root", False, children=[drive,behaviors[9]])
        drive = py_trees.composites.Sequence('drive', False, children=[conditions[1], behaviors[4]])




def main(args=None):
    """Create a node for the behavior tree and ticks it"""
    setup_node.init_and_spin(args, BehaviorTreePattern)

if __name__ == '__main__':
    main()