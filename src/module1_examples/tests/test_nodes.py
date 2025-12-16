# src/module1_examples/tests/test_nodes.py
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Import the nodes we want to test
from module1_examples.publisher import TelemetryPublisher
from module1_examples.subscriber import StatusMonitor
from module1_examples.service import CommanderClient

# --- Fixtures for Setup and Teardown ---

@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    """Initializes and shuts down rclpy once per module."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture(scope='function')
def minimal_node():
    """Provides a basic node for spinning futures."""
    node = Node('minimal_test_node')
    yield node
    node.destroy_node()

# --- Test Cases for Topics ---

def test_telemetry_publisher_creation():
    """T023: Test if the publisher node is correctly instantiated."""
    node = TelemetryPublisher()
    # Check if a publisher was correctly created on the specified topic
    pub_count = len(node.publishers_)
    assert pub_count == 1
    assert node.publishers_[0].topic_name == '/robot_telemetry'
    node.destroy_node()

def test_status_monitor_creation():
    """T023: Test if the subscriber node is correctly instantiated."""
    node = StatusMonitor()
    # Check if a subscription was correctly created
    sub_count = len(node.subscriptions_)
    assert sub_count == 1
    assert node.subscriptions_[0].topic_name == '/robot_telemetry'
    node.destroy_node()

def test_topic_communication(minimal_node):
    """T023: Test data flow between publisher and subscriber."""
    # Setup: Create a publisher and a subscriber
    pub_node = TelemetryPublisher()
    
    # We use a custom test subscriber to capture the data and assert its content
    received_msg = None
    
    def test_callback(msg):
        nonlocal received_msg
        received_msg = msg
        minimal_node.get_logger().info(f"Test Subscriber received: {msg.data}")

    test_sub = minimal_node.create_subscription(String, 'robot_telemetry', test_callback, 10)
    
    # Act: Manually call the publisher's timer callback and spin the minimal node
    pub_node.publish_telemetry()
    
    # Spin the node to process the pending publish/subscribe callbacks
    # Use a timeout to prevent infinite blocking
    timeout_sec = 1.0
    start_time = minimal_node.get_clock().now()
    
    # Wait until a message is received or timeout
    while rclpy.ok() and received_msg is None and (minimal_node.get_clock().now() - start_time).nanoseconds / 1e9 < timeout_sec:
        rclpy.spin_once(minimal_node, timeout_sec=0.1)

    # Assert: Check if the message was received and contains expected content
    assert received_msg is not None, "Did not receive message from publisher"
    assert "Robot status: Running" in received_msg.data

    # Cleanup
    minimal_node.destroy_subscription(test_sub)
    pub_node.destroy_node()

# --- Test Cases for Services ---

def test_service_client_creation():
    """T023: Test if the service client is correctly instantiated."""
    node = CommanderClient()
    # Check if the client was correctly created on the specified service
    assert node.cli is not None
    assert node.cli.srv_name == '/reset_robot'
    node.destroy_node()
    
# Note: Testing the full Service call requires running the Server node, 
# which is best handled by launch_testing (Task T024). 
# This test only verifies the client side setup.