import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci 
import time

# --- Action Server Node (The Controller Side) ---
class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci_sequence',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())

    def execute_callback(self, goal_handle):
        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
                
            sequence.append(sequence[i] + sequence[i-1])
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        return result

# --- Action Client Node (The Agent Side) ---
class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        group = MutuallyExclusiveCallbackGroup()
        self._action_client = ActionClient(
            self, 
            Fibonacci, 
            'fibonacci_sequence', 
            callback_group=group)

    def send_goal(self, order):
        self._action_client.wait_for_server()
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self._feedback_callback)
            
        self.send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.sequence[-1]}')

    def _get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Result received: {future.result().result.sequence}')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor() 

    server = FibonacciActionServer()
    client = FibonacciActionClient()
    
    executor.add_node(server)
    executor.add_node(client)

    client.send_goal(order=10) 

    try:
        executor.spin() 
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()