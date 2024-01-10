import time
import py_trees
import rclpy

import py_trees.display
from movement_pattern.basic.attraction_pattern2_bt import AttractionPattern2BT
from movement_pattern.basic.drive_pattern_bt import DrivePatternBT


if __name__ == '__main__':


    rclpy.init()
    root = py_trees.composites.Sequence("root", False)
    action = AttractionPattern2BT()
    root.add_child(action)

    action.setup()



    try:

        for _unused_i in range(0, 12):

            root.tick_once()

            time.sleep(0.5)

        print("\n")

    except KeyboardInterrupt:

        pass
    rclpy.shutdown()
