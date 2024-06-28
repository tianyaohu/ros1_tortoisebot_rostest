#! /usr/bin/env python3

import rospy
from tortoisebot_waypoints.srv import WayptAction, WaypoingActionRequest
import sys

rospy.init_node('waypoint_as_client')
rospy.wait_for_service('/tortoisebot_as')
waypoint_srv = rospy.ServiceProxy('/tortoisebot_as', WayptAction)
srv_msg = WaypoingActionRequest()
srv_msg.position.X = 0.5
srv_msg.angle_d = 0.5
result = waypoint_srv(srv_msg)