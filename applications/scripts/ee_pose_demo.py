#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    print "lol"
    rospy.init_node("ee_demo")
    wait_for_time()
    listener = tf.TransformListener()
    rate = rospy.Rate(1.0)
    """
    Read from frames:
       l_gripper_finger_link
       r_gripper_finger_link
       gripper_link
       wrist_roll_link
    """
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('gripper_link', 'base_link', rospy.Time(0))
            rospy.loginfo("%s %s", trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == '__main__':
    main()

