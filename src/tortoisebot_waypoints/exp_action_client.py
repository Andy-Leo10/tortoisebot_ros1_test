#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal

class WaypointClient:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('waypoint_client')

        # Create an action client
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)

        # Wait for the action server to start
        self.client.wait_for_server()

    def send_goal(self, x, y, z):
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

        # Log the result
        result1, result2 = self.client.get_result()
        rospy.loginfo(result1)
        rospy.loginfo(result2)

if __name__ == '__main__':
    try:
        waypoint_client = WaypointClient()
        rospy.loginfo("Moving to waypoint: (0.5, 0.0, 0.0)")
        waypoint_client.send_goal(0.5, 0.0, 0.0)
        rospy.loginfo("Moving to waypoint: (0.0, 0.5, 0.0)")
        waypoint_client.send_goal(0.0, 0.5, 0.0)
    except rospy.ROSInterruptException:
        pass