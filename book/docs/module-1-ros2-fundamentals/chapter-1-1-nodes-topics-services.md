---
title: "Chapter 1.1: ROS 2 Nodes, Topics, and Services"
description: "Learn the fundamental communication mechanisms in ROS 2."
sidebar_position: 1
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 1.1: ROS 2 Nodes, Topics, and Services

**Welcome to your first step in mastering Physical AI!** This chapter lays the essential groundwork for all your future work in robotics. We will explore the three fundamental pillars of ROS 2 communication: **Nodes**, **Topics**, and **Services**. By the end of this chapter, you will not only understand these concepts theoretically but will have also built your first multi-node ROS 2 application.

:::tip[Learning Objectives]
*   Explain what a ROS 2 Node is and its role in a distributed robotics system.
*   Implement a Python-based publisher and subscriber node using `rclpy`.
*   Understand the publish/subscribe model and the role of Topics for asynchronous data streaming.
*   Implement a synchronous request/response pattern using Services.
*   Distinguish when to use Topics vs. Services in a real-world robotics scenario.
*   Build and run a complete, multi-node ROS 2 system that integrates these concepts.
:::

## The ROS 2 Graph: A Network of Nodes

Imagine a complex robot like a humanoid. It has to perform many tasks simultaneously: read from cameras (perception), listen for commands (communication), maintain balance (control), and move its limbs (actuation). A monolithic program to handle all this would be a nightmare to write, debug, and maintain.

ROS 2 solves this complexity by using a **distributed network of specialized processes** called **Nodes**. This network is called the **ROS 2 Graph**.

<!--
![ROS 2 Graph Diagram](../assets/ros2_graph.svg)
*A simplified ROS 2 Graph showing how different nodes for perception, control, and actuation communicate.*
-->

### 1.1.1 What is a Node?

A **Node** is the smallest, most fundamental executable unit in a ROS 2 system. Think of it as a small, independent program that does one thing well. For example, you might have:
*   A `/camera_driver` node that captures images from a camera.
*   A `/image_processor` node that detects objects in those images.
*   A `/motor_controller` node that moves a robotic arm.
*   A `/user_interface` node that displays information to an operator.

Each node runs as a separate process on the operating system. They can be started, stopped, and restarted independently, making the entire system robust and fault-tolerant. Crucially, they can run on different computers across a network, allowing you to build large-scale, distributed systems.

All ROS 2 nodes are created using a client library. In this book, we will use **`rclpy`**, the officially supported Python client library.

## 1.1.2 Topics: The Asynchronous Data Bus

Now that we have nodes, how do they talk to each other? The most common way is through **Topics**.

A Topic is a named "bus" or "channel" that acts as a conduit for messages. Nodes can publish (send) messages to a topic, and any number of other nodes can subscribe to (receive) that topic. This is a **one-to-many, asynchronous** communication model.

*   **One-to-many**: One publisher can have many subscribers.
*   **Asynchronous**: The publisher doesn't wait for subscribers to receive the message. It just sends the data and continues its work.

This model is perfect for continuous data streams, like sensor readings, odometry, or camera feeds.

### Anatomy of a Topic
*   **Publisher**: A node that writes data to a topic.
*   **Subscriber**: A node that reads data from a topic.
*   **Message**: The data itself. Every topic has a specific **message type** (e.g., `String`, `Int64`, `Twist`), which defines the data structure.

### Code Example: A Simple Publisher and Subscriber

Let's see how this works in practice. Here we have a `talker` node that publishes "Hello World" messages and a `listener` node that subscribes to them.

<Tabs>
  <TabItem value="publisher" label="src/module1_examples/publisher.py" default>

```python
# T009: A basic publisher node in rclpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher on the 'topic' topic with message type String
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # Create a timer to call the timer_callback function every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    # Spin the node so the callback function is called.
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
  </TabItem>
  <TabItem value="subscriber" label="src/module1_examples/subscriber.py">
```python
# T010: A basic subscriber node in rclpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create a subscriber on the 'topic' topic.
        # The message type must match the publisher's.
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    # Spin the node to keep it alive and allow callbacks to be processed.
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
  </TabItem>
</Tabs>

## 1.1.3 Services: Synchronous Request & Response

What if a node needs to ask another node a direct question and wait for an answer? This is where **Services** come in.

A Service provides a **synchronous request/response** communication model. It's a **one-to-one** interaction.
*   **Service Server**: A node that advertises a service and provides a response when called.
*   **Service Client**: A node that calls the service with a request and waits for the response.

This model is ideal for remote procedure calls (RPCs), such as:
*   Asking a `/navigation` node to "plan a path to coordinate (X, Y)".
*   Requesting a `/robot_state` node to "get the current battery level".
*   Triggering a `/camera` node to "save the current image to a file".

### Anatomy of a Service
*   **Server**: A node that offers the service.
*   **Client**: A node that uses the service.
*   **Service Type**: Defines the structure of *both* the request and the response. It's usually composed of two parts separated by `---`.

### Code Example: An "Add Two Ints" Service

Here, we create a service that adds two integers. The client sends two numbers, and the server returns their sum.

<Tabs>
  <TabItem value="service-server" label="src/module1_examples/service_server.py" default>

```python
# T011: A basic service server node in rclpy
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        # Create a service with the 'add_two_ints' name and AddTwoInts type
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        # Calculate the sum and store it in the response object
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
  </TabItem>
  <TabItem value="service-client" label="src/module1_examples/service_client.py">
```python
# T011: A basic service client node in rclpy
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
import sys

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    # Spin until the future is complete (response is received)
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(f'Service call failed {e}')
            else:
                minimal_client.get_logger().info(
                    f'Result of add_two_ints: for {minimal_client.req.a} + {minimal_client.req.b} = {response.sum}')
            break

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
  </TabItem>
</Tabs>

## Hands-on Exercises

Theory is great, but robotics is a hands-on discipline. Let's put your new knowledge to the test.

1.  **Your First Publisher/Subscriber**:
    *   Create a Python package for your ROS 2 nodes.
    *   Write a `talker.py` node that publishes a `std_msgs/String` message to a topic named `/chatter` every second.
    *   Write a `listener.py` node that subscribes to `/chatter` and prints the received message to the console.
    *   Use a `launch` file to run both nodes at the same time.

2.  **Custom Message Types**:
    *   Define a custom message file `Student.msg` that contains `string name` and `uint32 student_id`.
    *   Modify your publisher to send messages of this new type.
    *   Modify your subscriber to receive and log the custom messages.

3.  **Simple Calculator Service**:
    *   Create a service that can perform addition, subtraction, multiplication, and division.
    *   The service request should contain two numbers and an operator (`+`, `-`, `*`, `/`).
    *   The service response should contain the result.
    *   Write a client that calls this service from the command line.

## Capstone Project: Autonomous Robot Monitor

**Goal**: Build a simple monitoring system for a simulated robot that reports its status and can be commanded to perform a self-check.

**System Components**:
1.  **`robot_simulator` Node**:
    *   Publishes the robot's battery level (`Float32`) to a `/battery_status` topic every 5 seconds (simulate a decreasing battery).
    *   Publishes the robot's hardware status (`String`, e.g., "OK", "WARN", "ERROR") to a `/hardware_status` topic every 20 seconds.
2.  **`status_monitor` Node**:
    *   Subscribes to `/battery_status` and `/hardware_status`.
    *   If the battery level drops below 20%, it should print a `CRITICAL BATTERY LOW` warning.
    *   It should log any status message that is not "OK".
3.  **`self_check_server` Node**:
    *   Provides a service named `/trigger_self_check` of type `std_srvs/Trigger`.
    *   When called, it should simulate a 5-second self-check and return a success message with details like "Self-check complete. All systems nominal."

This project will test your ability to integrate nodes, topics, and services into a cohesive, functional robotics application. Good luck!

---
**End of Chapter 1.1**
