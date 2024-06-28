#! /usr/bin/env python

from robot_control.rotate_robot import RobotControl
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import rospy
import rosunit
import unittest
import rostest
import time
PKG = 'robot_control'
NAME = 'rotate_robot_integration_test'

class TestRobotControl(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_node')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_orientation = Quaternion()
        self.init_yaw = 0
        self.final_yaw = 0

    def odom_callback(self, msg):

        self.current_orientation = msg.pose.pose.orientation

    def euler_to_quaternion(self, msg):

        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def test_correct_rotation(self):
        print ("Current Orientation:")
        print self.current_orientation
        time.sleep(2)
        #rospy.wait_for_message("/odom", Odometry, timeout=2)
        self.final_yaw = self.euler_to_quaternion(self.current_orientation)
        print ("Final Yaw:")
        print self.final_yaw
        yaw_diff = self.init_yaw - self.final_yaw
        print ("Yaw Diff:")
        print (yaw_diff )
        self.assertTrue((1.3 <= yaw_diff <= 2.1), "Integration error. Rotation was not between the expected values.")


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRobotControl)