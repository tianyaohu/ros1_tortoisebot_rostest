#!/usr/bin/env python3
import rospy
import time
import actionlib
import math

from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

class WaypointActionClass(object):

    _feedback = WaypointActionFeedback()
    _result = WaypointActionResult()

    _pub_cmd_vel = None
    _sub_odom = None

    _position = Point()
    _yaw = 0
    _state = 'idle'
    _des_pos = Point()
    _yaw_precision = math.pi / 90
    _dist_precision = 0.05

    def __init__(self):
        self._as = actionlib.SimpleActionServer("tortoisebot_as", WaypointActionAction, self.goal_callback, False)
        self._as.start()

        self._rate = rospy.Rate(25)
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._sub_odom = rospy.Subscriber('/odom', Odometry, self._clbk_odom)
        rospy.loginfo("Action server started")

    def _clbk_odom(self, msg):
        self._position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self._yaw = euler[2]

    def goal_callback(self, goal):
        rospy.loginfo("goal %s received" % str(goal))

        success = True
        self._des_pos = goal.position
        desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
        err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
        err_yaw = desired_yaw - self._yaw

        while err_pos > self._dist_precision and success:
            desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
            err_yaw = desired_yaw - self._yaw
            err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
            
            if self._as.is_preempt_requested():
                rospy.loginfo("The goal has been cancelled/preempted")
                self._as.set_preempted()
                success = False
            else:
                twist_msg = Twist()
                # Proportional controller for yaw correction with increased gain
                angular_velocity = 1.0 * err_yaw
                
                # Ensure minimum angular speed for significant yaw error
                if abs(err_yaw) > self._yaw_precision * 5:
                    angular_velocity = max(0.6, abs(angular_velocity)) * (1 if err_yaw > 0 else -1)
                
                twist_msg.angular.z = angular_velocity

                # Combine linear and angular movements
                if math.fabs(err_yaw) < self._yaw_precision * 5:  # Allow some yaw error tolerance for forward movement
                    twist_msg.linear.x = 0.2 * err_pos  # Slow down as it approaches the goal

                self._pub_cmd_vel.publish(twist_msg)

            self._feedback.position = self._position
            self._feedback.state = self._state
            self._as.publish_feedback(self._feedback)
            self._rate.sleep()

        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self._pub_cmd_vel.publish(twist_msg)

        if success:
            self._result.success = True
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('tortoisebot_as')
    WaypointActionClass()
    rospy.spin()
