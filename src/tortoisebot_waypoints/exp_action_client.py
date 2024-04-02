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

        # Send the goal to the action server
        self.client.send_goal(goal)

        # Wait for the server to finish performing the action
        self.client.wait_for_result()

        # Log the result
        rospy.loginfo(self.client.get_result())

if __name__ == '__main__':
    try:
        waypoint_client = WaypointClient()
        rospy.loginfo("Moving to waypoint: (0.5, 0.5, 0.0)")
        waypoint_client.send_goal(0.5, 0.5, 0.0)
        rospy.loginfo("Moving to waypoint: (0.25, 0.25, 0.0)")
        waypoint_client.send_goal(0.25, 0.25, 0.0)
        rospy.loginfo("Moving to waypoint: (-0.25, -0.25, 0.0)")
        waypoint_client.send_goal(-0.25, -0.25, 0.0)
    except rospy.ROSInterruptException:
        pass