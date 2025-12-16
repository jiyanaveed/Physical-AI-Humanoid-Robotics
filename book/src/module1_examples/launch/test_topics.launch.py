# launch/test_topics.launch.py
import os
import unittest
import launch
from launch_ros.actions import Node
from launch_testing.actions import LaunchTestService
from ament_index_python.packages import get_package_share_directory

# --- 1. Define the Launch Description ---

def generate_launch_description():
    """
    Launch description for testing the Publisher and Subscriber nodes.
    """
    # Nodes from the module1_examples package
    publisher_node = Node(
        package='module1_examples',
        executable='publisher.py',
        name='telemetry_publisher',
        output='screen'
    )

    subscriber_node = Node(
        package='module1_examples',
        executable='subscriber.py',
        name='status_monitor',
        output='screen'
    )

    return launch.LaunchDescription([
        publisher_node,
        subscriber_node,
        
        # Launch the launch_testing service at the end
        LaunchTestService(
            test_runners=[
                # Use a specific test runner defined below
                ('test_publisher_subscriber_communication', TestTelemetry), 
            ]
        ),
    ])

# --- 2. Define the Test Runner Class ---

class TestTelemetry(unittest.TestCase):
    """
    T024: Test class to check for successful Topic communication 
    between the Publisher and Subscriber nodes.
    """
    
    def test_topic_echo(self, launch_service):
        """
        Check the log output of the subscriber to confirm messages were received.
        """
        
        # Wait for a few seconds to allow the nodes to initialize and exchange messages
        launch_service.wait_for_condition(
            condition=lambda: self.find_topic_message(launch_service),
            timeout=5.0,
            message="Subscriber did not receive the expected message within 5 seconds."
        )

    def find_topic_message(self, launch_service):
        """Helper function to search the subscriber's log output."""
        
        # Find the log output for the 'status_monitor' node
        subscriber_output = launch_service.get_node_output('status_monitor')
        
        # Check if the subscriber successfully logged receiving the message
        # The subscriber log should contain "I heard status update: " and the data
        if "I heard status update: " in subscriber_output:
            return True
        return False

# --- 3. Optional: Define a function for the command line (for simplicity) ---
# Note: Actual execution usually involves 'pytest-launch' or 'ros2 launch'

if __name__ == '__main__':
    # This block is primarily for demonstrating the structure; 
    # the actual launch testing is run via the ROS 2 command line.
    pass
