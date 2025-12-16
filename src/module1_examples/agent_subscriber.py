import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class PerceptionAgent(Node):
    def __init__(self):
        super().__init__('perception_agent')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.lidar_callback,
            10)
        self.min_distance = float('inf')

    def lidar_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
        else:
            self.min_distance = msg.range_max 

        self.get_logger().info(f'Processed: Nearest obstacle: {self.min_distance:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Perception Agent shut down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()