
from geometry_msgs.msg import Twist
import rclpy
import py_trees

from rclpy.qos import qos_profile_sensor_data
from ros2swarm.behavior_tree.movement_pattern.movement_pattern_bt import MovementPatternBT
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions


class ExamplePatternBT(MovementPatternBT, py_trees.behaviour.Behaviour):
    """
    Example Pattern 
    """

    def __init__(self):
        """Declare the pattern node."""
        MovementPatternBT.__init__(self, 'example_pattern')
        py_trees.behaviour.Behaviour.__init__(self,'example_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('example_pattern_1', 0.0),
                ('example_pattern_2', 0.0),
                ('example_pattern_3', 0.0),
            ])
        self.normal_movement = Twist()



    def setup(self): 
        """setup the pattern node.
        
        Will only be executed beofre the first tick.

        needs to be called
        
        init here ros subscriptions and publisher

        as well as timer
        """ 

        self.scan_subscription = self.create_subscription(
            int,
            self.get_namespace() + '/topic_name',
            self.example_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.get_logger().debug("  %s [Example::setup()]" % self.name)




    def initialise(self):
        """Initialize the pattern node.

        Will always be called when the pattern wasn't called directly before.

        initialize parameters here
        
        """
        self.get_logger().debug("  %s [Example::initialise()]" % self.name)


        self.example_pattern_1 = self.get_parameter(
            "example_pattern_1").get_parameter_value().double_value
        self.example_pattern_2 = self.get_parameter(
            "example_pattern_2").get_parameter_value().double_value
        self.example_pattern_3 = self.get_parameter(
            "example_pattern_3").get_parameter_value().double_value
    
        self.normal_movement.linear.x = self.example_pattern_2
        self.normal_movement.angular.z = self.example_pattern_3

            
    def update(self):

        """ spin node once -- the callbac function is called once"""


        self.get_logger().debug("  %s [Example::update()]" % self.name)

        rclpy.spin_once(self, timeout_sec=0)

        self.feedback_message = "spin example pattern  once"

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):

        """ destroy node """

        self.get_logger().debug("  %s [Example::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        
        # AttractionPattern2BT.destroy_node()


    def example_callback(self, incoming_msg):
        """Call back if a new msg is available."""
        self.normal_movement.linear.x *= incoming_msg
        direction = self.normal_movement
        self.command_publisher.publish(direction)


