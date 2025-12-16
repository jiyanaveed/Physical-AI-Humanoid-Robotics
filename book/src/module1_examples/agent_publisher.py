import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 

class ControlAgent(Node):
    def __init__(self):
        super().__init__('control_agent')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.timer = self.create_timer(0.1, self.publish_command) 
        self.linear_speed = 0.3 
        self.angular_speed = 0.0
        self.get_logger().info('Control Agent ready to publish commands.')

    def publish_command(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published: V_lin={msg.linear.x:.2f}, V_ang={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ControlAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Control Agent shut down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()