import os
import sys
import time
import unittest
import pytest

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import rclpy
from rclpy.action import ActionClient
from dms_pkg.action import MyAction  # Import your action definition

# Launch feature node
@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)
    action_server_node = launch_ros.actions.Node(
        package='dms_pkg',
        executable='action_server',  # Assuming this is the executable name
        name='action_server_node',
        output='screen'  # For debugging output
    )

    return (
        launch.LaunchDescription([
            action_server_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'action_server': action_server_node
        }
    )

# Test class
class TestThinkingAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Create a test node for the action client
        self.node = rclpy.create_node('test_thinking_client')

    def tearDown(self):
        self.node.destroy_node()

    def test_thinking_action(self, action_server, proc_output):
        # Create an action client for MyAction
        action_client = ActionClient(
            self.node,
            MyAction,
            'thinking'
        )

        # Wait for the action server to be available
        self.assertTrue(
            action_client.wait_for_server(timeout_sec=5.0),
            "Action server not available after 5 seconds"
        )

        # Send a goal (e.g., duration of 3 seconds)
        goal_msg = MyAction.Goal()
        goal_msg.duration = 3  # Set goal duration to 3 seconds

        # Store feedback for verification
        feedback_received = []

        # Define feedback callback
        def feedback_callback(feedback_msg):
            feedback_received.append(feedback_msg.feedback.progress)
            print(f"Feedback progress: {feedback_msg.feedback.progress}")

        # Send goal asynchronously with feedback callback
        goal_handle_future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback
        )
        rclpy.spin_until_future_complete(self.node, goal_handle_future, timeout_sec=2.0)
        self.assertTrue(goal_handle_future.done(), "Goal submission timed out")
        goal_handle = goal_handle_future.result()
        self.assertTrue(goal_handle.accepted, "Goal was not accepted by server")

        # Get the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
        self.assertTrue(result_future.done(), "Result retrieval timed out")

        # Verify the result
        result = result_future.result().result
        self.assertTrue(result.success, "Action did not succeed")

        # Verify feedback (progress should approach 3.0)
        self.assertGreater(len(feedback_received), 0, "No feedback received")
        self.assertAlmostEqual(
            feedback_received[-1],
            3.0,
            delta=0.5,  # Allow some timing variance
            msg=f"Last feedback {feedback_received[-1]} not close to goal duration 3.0"
        )

        # Check server output
        proc_output.assertWaitFor(
            expected_output="Goal succeeded",
            process=action_server,
            timeout=5.0
        )