#! /usr/bin/env python

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import copy
import math
import tf.transformations as tft
import numpy as np


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """
    THRESHOLD = .025

    def __init__(self):
        self.pub = rospy.Publisher("base_controller/command", Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
	self._last_odom_msg = None

    def _odom_callback(self, msg):
        #TODO: do something
        self._last_odom_msg = msg
      

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.pub.publish(twist)
        

    def stop(self):
        """Stops the mobile base from moving.
        """
        self.move(0, 0)


    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        while (self._last_odom_msg == None):
	    rospy.sleep(1.0)
        start = copy.deepcopy(self._last_odom_msg.pose.pose.position)
        rate = rospy.Rate(10)
        while self.distance_fn(self._last_odom_msg.pose.pose.position, start) < math.fabs(distance):
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()

    def distance_fn(self, pt1, pt2):
        xDist = pt1.x - pt2.x
        yDist = pt1.y - pt2.y
        zDist = pt1.z - pt2.z
        return math.sqrt(math.pow(xDist, 2) + math.pow(yDist, 2) + math.pow(zDist, 2))

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.
    
        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        while (self._last_odom_msg == None):
            rospy.sleep(1.0)
        start = copy.deepcopy(self._last_odom_msg.pose.pose.orientation)
        curr_yaw = self.quaternion_to_yaw(start)
        rate = rospy.Rate(10)
        direction = -1 if (angular_distance < 0) else 1
        angular_distance = angular_distance % (2 * math.pi)
        goal_angle = curr_yaw + angular_distance
        goalPos = self.rad_to_coor(goal_angle)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        curPos = self.rad_to_coor(curr_yaw) #self.quaternion_to_yaw(self._last_odom_msg.pose.pose.orientation)
        while not self.reached_goal_state(curPos, goalPos):#distance_to_goal(curr_yaw, goal_yaw, direction) > 0:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            self.move(0, direction * speed)
            curr_yaw = self.quaternion_to_yaw(self._last_odom_msg.pose.pose.orientation)
            curPos = self.rad_to_coor(curr_yaw)
            rate.sleep()

    def reached_goal_state(self, curPos, goalPos):
        curx, cury = curPos
        goalx, goaly = goalPos
        return math.fabs(curx - goalx) < self.THRESHOLD and math.fabs(cury - goaly) < self.THRESHOLD


    def quaternion_to_yaw(self, q):
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = m[0, 0]
        y = m[1, 0]
        return math.atan2(y, x) % (2 * math.pi)


    def rad_to_coor(self, rad):
        return (math.cos(rad), math.sin(rad))













