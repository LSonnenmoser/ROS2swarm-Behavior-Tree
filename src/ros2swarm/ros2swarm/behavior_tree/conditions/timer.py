import py_trees
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from py_trees.common import Status



class Timer(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("avoid")
        self.timer=0
        self.timer2 = 0

    def setup(self):
        pass
    def initialise(self) -> None:
        pass

    def update(self) -> Status:
        if self.timer2 < 0:
            self.timer=0
        self.timer+=1
        if self.timer < 10000:
            self.timer2 += 1
            return py_trees.common.Status.FAILURE
        self.timer2 -= 1
        return py_trees.common.Status.SUCCESS
    