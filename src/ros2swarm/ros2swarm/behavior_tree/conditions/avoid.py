import py_trees
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from py_trees.common import Status



class Avoid(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("avoid")
        self.hazard = False

    def setup(self):
        self.hazard_detection_subscription = self.create_subscription(
            HazardDetectionVector,
            self.get_namespace() + '/hazard_detection',
            self.swarm_command_controlled(self.hazard_detection_callback),
            qos_profile=qos_profile_sensor_data
        )
    def initialise(self) -> None:
        self.hazard = False

    def update(self) -> Status:
        if self.hazard:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
    
    def hazard_detection_callback(self, msg):
        if msg.detections:
            self.hazard = True
        else:
            self.hazard = False