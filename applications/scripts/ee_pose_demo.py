#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg


def main():
    rospy.init_node("ee_demo")
    wait_for_time()
    listener = tf.TransformListener()
    """
    Read from frames:
       l_gripper_finger_link
       r_gripper_finger_link
       gripper_link
       wrist_roll_link
    """
    while True:
        rospy.sleep(1000)
if __name__ == '__main__':
    main

