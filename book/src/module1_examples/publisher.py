import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('telemetry_publisher_node')
        self.publisher_ = self.create_publisher(String, 'robot_telemetry', 10)
        self.count = 0
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_telemetry)
        self.get_logger().info('Telemetry Publisher started.')

    def publish_telemetry(self):
        msg = String()
        msg.data = f'Robot status: Running. Message ID: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publisher shut down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()