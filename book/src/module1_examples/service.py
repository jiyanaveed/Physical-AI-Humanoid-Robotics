import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

# --- Service Server Node ---
class ResetServer(Node):
    def __init__(self):
        super().__init__('reset_server_node')
        # Create a service using the standard Trigger type
        self.srv = self.create_service(
            Trigger, 
            'reset_robot', 
            self.reset_callback)
        self.get_logger().info('Reset Server ready.')

    def reset_callback(self, request, response):
        self.get_logger().info('Executing robot state reset...')
        time.sleep(1.0) # Simulate a long operation
        response.success = True
        response.message = 'Robot state reset successfully.'
        return response

# --- Service Client Node ---
class CommanderClient(Node):
    def __init__(self):
        super().__init__('commander_client_node')
        self.cli = self.create_client(Trigger, 'reset_robot')
        self.req = Trigger.Request()
        
    def send_reset_request(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset service not available, waiting...')
        
        self.get_logger().info('Service is up. Sending reset request.')
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    
    server_node = ResetServer()
    client_node = CommanderClient()

    # Spin the server asynchronously
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(server_node)
    executor.add_node(client_node)

    # Client sends request and spins until complete
    future = client_node.send_reset_request()
    executor.spin_until_future_complete(future)

    if future.result() is not None:
        result = future.result()
        client_node.get_logger().info(f'Result: Success={result.success}, Message="{result.message}"')
    else:
        client_node.get_logger().error('Service call failed.')

    server_node.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()