#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import euler_from_quaternion
import rospy
import unittest
import rostest
import time
import math
import actionlib
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal
from std_srvs.srv import Empty

PKG = 'tortoisebot_waypoints'
NAME = 'waypoint_integration_test'

class TestWaypoint(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_orientation = Quaternion()
        self.current_position = Point()
        self.init_position = None
        self.init_yaw = None

        self.wait_for_odom()
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.client.wait_for_server()

        #get x,y,yaw
        self.goal_x = rospy.get_param('~x', 0.5)
        self.goal_y = rospy.get_param('~y', 0.5)

        self.send_goal(self.goal_x,self.goal_y)

    def odom_callback(self, msg):
        self.current_orientation = msg.pose.pose.orientation
        self.current_position = msg.pose.pose.position

    def euler_to_yaw(self, orientation):
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def wait_for_odom(self):
        rospy.loginfo("Waiting for initial odometry message...")
        while self.current_position == Point():
            time.sleep(0.1)
        self.init_position = self.current_position
        self.init_yaw = self.euler_to_yaw(self.current_orientation)
        rospy.loginfo(f"Initial position: {self.init_position}, Initial yaw: {self.init_yaw}")

    def send_goal(self, x, y):
        goal = WaypointActionGoal()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = 0 

        self.client.send_goal(goal)
        self.client.wait_for_result()

    def test_correct_position(self):
        final_position = self.current_position
        rospy.loginfo(f"Final position: {final_position}")

        position_tolerance = 0.1
        self.assertAlmostEqual(self.goal_x, final_position.x, delta=position_tolerance, msg="X position out of tolerance")
        self.assertAlmostEqual(self.goal_y, final_position.y, delta=position_tolerance, msg="Y position out of tolerance")

    def test_correct_yaw(self):
        final_yaw = self.euler_to_yaw(self.current_orientation)
        rospy.loginfo(f"Final yaw: {final_yaw}")

        desired_yaw = math.atan2(self.goal_y - self.current_position.y, self.goal_x - self.current_position.x)

        yaw_tolerance = 0.5
        self.assertAlmostEqual(desired_yaw, final_yaw, delta=yaw_tolerance, msg="Yaw out of tolerance")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypoint)
