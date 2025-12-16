import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor_node')
        self.subscription = self.create_subscription(
            String,
            'robot_telemetry',
            self.listener_callback,
            10)
        self.get_logger().info('Status Monitor started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard status update: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber shut down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()