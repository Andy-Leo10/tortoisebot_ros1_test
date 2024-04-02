#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal

def send_goal():
    # Initialize the ROS node
    rospy.init_node('waypoint_client')

    # Create an action client
    client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)

    # Wait for the action server to start
    client.wait_for_server()

    # Create a goal object
    goal = WaypointActionGoal()
    goal.position = Point(0.5, 0.5, 0.0)  # Set your target waypoint here

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the server to finish performing the action
    client.wait_for_result()

    # Print the result
    print(client.get_result())

if __name__ == '__main__':
    try:
        send_goal()
    except rospy.ROSInterruptException:
        pass