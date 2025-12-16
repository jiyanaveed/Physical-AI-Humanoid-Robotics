---
title: "Chapter 1.2: Python Agents and ROS 2 Development"
description: "Best practices for implementing object-oriented Python agents for complex ROS 2 tasks."
sidebar_position: 2
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 1.2: Python Agents and ROS 2 Development

In the previous chapter, you mastered the fundamental communication patterns of ROS 2. Now, it's time to bridge that knowledge with the power of intelligent, autonomous systems. This chapter introduces the concept of a **Python Agent**—an object-oriented software component that encapsulates perception, decision-making, and action. You will learn how to design and implement these agents within the ROS 2 ecosystem, creating sophisticated, task-driven robotic behaviors.

:::tip[Learning Objectives]
*   Understand the architecture of an object-oriented Python agent for robotics.
*   Implement an agent that subscribes to sensor data topics for perception.
*   Implement an agent that publishes control commands to actuator topics.
*   Master the use of **ROS 2 Actions** for managing long-running, goal-oriented tasks with feedback.
*   Learn about `rclpy` executors and callback management for complex applications.
*   Build a complete pick-and-place agent that integrates perception, logic, and control.
:::

## From Nodes to Agents: A Paradigm Shift

While nodes are the fundamental *executables* in ROS 2, it's often beneficial to structure the logic *within* a node using a more sophisticated design pattern. An **Agent** is a class-based structure that represents an autonomous entity responsible for a specific set of tasks.

| Aspect         | Standard ROS 2 Node (Functional Approach)                               | Python Agent (Object-Oriented Approach)                                           |
| -------------- | ----------------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| **Structure**  | Often a collection of functions and callbacks.                          | A class with methods, properties, and a clear state.                              |
| **State**      | State is often managed with global variables or passed between functions. | State is encapsulated within the class instance (`self.state`).                   |
| **Reusability**| Can be difficult to reuse parts of the node's logic.                    | Highly reusable and extensible through inheritance and composition.               |
| **Complexity** | Becomes hard to manage as the number of publishers/subscribers grows.   | Organizes complexity by grouping related perception, logic, and action components.|

By adopting an agent-based approach, we create code that is more modular, testable, and easier to reason about, especially for complex robots.

## 1.2.1 The Agent Architecture: Sense, Think, Act

A common and effective architecture for a robotic agent is the **Sense-Think-Act** loop.

1.  **Sense**: The agent gathers information about its environment. In ROS 2, this is primarily done by subscribing to topics that publish sensor data (e.g., `/camera/image_raw`, `/scan`, `/odom`).
2.  **Think**: The agent processes the sensory input, updates its internal state, and makes a decision. This is the "brain" of the agent, where your custom logic, algorithms, or even a trained machine learning model resides.
3.  **Act**: The agent executes its decision by interacting with the world. In ROS 2, this usually means publishing command messages to topics (e.g., `/cmd_vel`, `/arm_controller/joint_trajectory`) or making service/action requests.

<!-- 
![Sense-Think-Act Diagram](../assets/sense_think_act.svg)
*The classic sense-think-act loop for autonomous agents.*
-->

### Code Example: Agent Subscribing to Sensor Data (Sense)

Let's design an agent that "senses" its environment by subscribing to a sensor topic. This agent will monitor a laser scanner and determine if an obstacle is too close.

```python
# T015: Agent subscribing to a sensor topic
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleAvoiderAgent(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_agent')
        # SENSE: Subscribe to the /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Agent state
        self.is_obstacle_too_close = False
        self.min_safe_distance = 0.5  # meters

    def scan_callback(self, msg: LaserScan):
        # THINK: Process the laser scan data
        min_distance = min(msg.ranges)
        if min_distance < self.min_safe_distance:
            self.is_obstacle_too_close = True
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m! State: DANGER')
        else:
            self.is_obstacle_too_close = False
            self.get_logger().info(f'Path is clear. Min distance: {min_distance:.2f}m. State: SAFE')

def main(args=None):
    rclpy.init(args=args)
    agent = ObstacleAvoiderAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Example: Agent Publishing Control Commands (Act)

Now, let's extend our agent to act on its decision. If an obstacle is detected, it will publish a command to stop the robot.

```python
# T016: Agent publishing a control command
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoiderAgent(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_agent')
        # SENSE
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # ACT
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Agent state
        self.is_obstacle_too_close = False
        self.min_safe_distance = 0.5

    def scan_callback(self, msg: LaserScan):
        # THINK
        min_distance = min(msg.ranges)
        if min_distance < self.min_safe_distance:
            self.is_obstacle_too_close = True
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m! Stopping robot.')
            # ACT: Publish a zero-velocity command
            stop_command = Twist()
            self.publisher_.publish(stop_command)
        else:
            self.is_obstacle_too_close = False
            # In a real system, you would publish a forward velocity command here.
            # For simplicity, we do nothing if the path is clear.
            self.get_logger().info(f'Path is clear. Min distance: {min_distance:.2f}m.')

# ... (main function remains the same)
```

## 1.2.2 Actions: For Long-Running, Goal-Oriented Tasks

Topics are great for continuous data, and Services are great for quick, synchronous transactions. But what about tasks that take a long time to complete, like "navigate to the kitchen," "pick up the red ball," or "rotate 360 degrees"?

For these scenarios, ROS 2 provides **Actions**.

An Action is similar to a Service, but with three key differences:
1.  **Asynchronous & Non-Blocking**: A client sends a *goal* to an Action Server and doesn't have to wait for the result. It can continue with other tasks.
2.  **Feedback**: The Action Server can periodically send updates (feedback) to the client about the progress of the task.
3.  **Cancellable**: The client can request to cancel the goal at any time.

### Anatomy of an Action
*   **Action Server**: A node that advertises the action, accepts goals, provides feedback, and produces a result.
*   **Action Client**: A node that sends goals, monitors feedback, and receives the final result.
*   **Action Definition (`.action` file)**: A file that defines the structure for the Goal, Result, and Feedback messages.

```
# Goal definition
---
# Result definition
---
# Feedback definition
```

### Code Example: A Fibonacci Action Server

Let's create an action that calculates a Fibonacci sequence. The goal will be the `order` of the sequence. The feedback will be the current sequence being calculated. The result will be the final sequence.

<Tabs>
  <TabItem value="action-server" label="src/module1_examples/fibonacci_action_server.py" default>
```python
# T017: A basic action server node in rclpy
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_robot_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

# ... (main function)
```
  </TabItem>
  <TabItem value="action-client" label="src/module1_examples/fibonacci_action_client.py">
```python
# T017: A basic action client node in rclpy
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.partial_sequence}')

# ... (main function with argument parsing for order)
```
  </TabItem>
</Tabs>


## Hands-on Exercises

1.  **Advanced Obstacle Avoider**:
    *   Improve the `ObstacleAvoiderAgent` from the example.
    *   If the path is clear, it should publish a `Twist` message with a constant forward linear velocity.
    *   If an obstacle is detected, it should stop, and then rotate until the path is clear again.
2.  **Vending Machine Service**:
    *   Create a service that simulates a vending machine.
    *   The request should contain the `item_name` (string) and `payment` (float).
    *   The server should have a predefined list of items and prices.
    *   The response should return the item if the payment is sufficient, or an error message if not.
3.  **Robot Arm Control Action**:
    *   Define an action `MoveArm.action` with a `geometry_msgs/Pose` as the goal.
    *   The feedback should be the current `distance_to_goal` (float).
    *   The result should be a `bool` indicating success.
    *   Implement a mock Action Server that simulates moving an arm over 5 seconds, publishing feedback every second.

## Capstone Project: Autonomous Pick-and-Place Agent

**Goal**: Build an agent that can detect an object, pick it up, and place it at a target location using ROS 2 Actions.

**System Components**:
1.  **`perception_node`**:
    *   Publishes the location of a "widget" as a `geometry_msgs/PoseStamped` message to the `/widget_pose` topic. (You can just publish a static pose for this simulation).
2.  **`pick_and_place_agent`**:
    *   An agent that subscribes to `/widget_pose`.
    *   It should have two Action Clients: one for a `/move_to` action and one for a `/gripper_control` action.
3.  **`mock_arm_controller`**:
    *   An Action Server for `/move_to` (of type `MoveArm` from the exercise). It simulates moving the arm to the requested pose.
4.  **`mock_gripper_controller`**:
    *   An Action Server for `/gripper_control`. The goal should be a boolean (`true` for close, `false` for open). It simulates opening or closing a gripper.

**Agent Logic**:
1.  Wait for the `/widget_pose`.
2.  Send a goal to the `/move_to` action server to move the arm above the widget.
3.  Once that's done, send a goal to `/gripper_control` to close the gripper.
4.  Send another goal to `/move_to` to lift the arm.
5.  Send a final goal to `/move_to` to place the widget at a predefined drop-off location.
6.  Send a final goal to `/gripper_control` to open the gripper.

This project will challenge you to orchestrate multiple actions and subscriptions, creating a truly autonomous and stateful robotic agent.

---
**End of Chapter 1.2**
