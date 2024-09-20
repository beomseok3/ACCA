# ros2
import rclpy
from rclpy.node import Node

from rclpy.qos import  qos_profile_system_default

class Test(Node):
    def __init__(self):
        super().__init__("test")
        self.declare_parameters("",[('te',5)])
        
        self.a=self.get_parameter('te').get_parameter_value().integer_value
        
        self.timer = self.create_timer(1.0,self.callback_param)
    def callback_param(self):
        self.a=self.get_parameter('te').get_parameter_value().integer_value
        self.get_logger().info(f"{self.a}") 
  
        
def main(args = None):
    rclpy.init(args=args)
    node = Test()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

main()
    