#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal
import rosunit
import unittest
import rostest
PKG = 'tortoisebot_waypoints'
NAME = 'test_action_client'

class WaypointClient:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('waypoint_client')
        # Create an action client
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        # Wait for the action server to start
        self.client.wait_for_server()
        
        # results
        self.success_pos = False
        self.success_yaw = False

    def get_result_pos(self): return self.success_pos
    
    def get_result_yaw(self): return self.success_yaw
    
    def clean_results(self):
        self.success_pos = False
        self.success_yaw = False

    def send_goal(self, x, y, z):
        self.clean_results()
        # Create a goal object
        goal = WaypointActionGoal()
        goal.position = Point(x, y, z)  # Set your target waypoint here

        # Define a feedback callback
        def feedback_cb(feedback):
            rospy.loginfo('Feedback: %s' % feedback)
        # Send the goal to the action server
        self.client.send_goal(goal, feedback_cb=feedback_cb)

        # Wait for the server to finish performing the action
        self.client.wait_for_result()

        # Get the result of the action
        result = self.client.get_result()
        self.success_pos = result.success_pos
        self.success_yaw = result.success_yaw
        # Log the result
        # rospy.loginfo("Result Pos: %s" % ("Success" if self.success_pos else "Failure"))
        # rospy.loginfo("Result Yaw: %s" % ("Success" if self.success_yaw else "Failure"))

class TestWaypointClient(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_waypoint_client')
        self.waypoint_client = WaypointClient()

    def test_send_goal(self):
        test_cases = [
            ((0.5, 0.0, 0.0), True, True),
            ((0.0, 0.5, 0.0), True, True),
            # Add more test cases here as needed
        ]

        for args, expected_pos, expected_yaw in test_cases:
            self.waypoint_client.send_goal(*args)
            self.assertEqual(self.waypoint_client.get_result_pos(), expected_pos)
            self.assertEqual(self.waypoint_client.get_result_yaw(), expected_yaw)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointClient)
