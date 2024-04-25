import py_trees
import rclpy
from rclpy.node import Node



class Timer(py_trees.behaviour.Behaviour, Node):

    def __init__(self):
        py_trees.behaviour.Behaviour.__init__(self, "timer")
        Node.__init__(self, "timer")
        self.timer_period=5
        self.flag = False

    def setup(self):
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def initialise(self):
        pass

    def update(self):
        rclpy.spin_once(self, timeout_sec=0)
        
        if self.flag:
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.FAILURE
    
    def timer_callback(self):
        self.flag = not self.flag