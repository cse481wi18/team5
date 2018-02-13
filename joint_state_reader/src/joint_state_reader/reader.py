#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState



class JointStateReader(object):
    """Listens to /joint_states and provides the latest joint angles.

    Usage:
        joint_reader = JointStateReader()
        rospy.sleep(0.1)
        joint_reader.get_joint('shoulder_pan_joint')
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])
    """

    def __init__(self):
        self.jointSubscriber = rospy.Subscriber("/joint_states", JointState, self.update_data)
        self.jointStates = {}

    def update_data(self, message):
        for i in range(len(message.name)):
            self.jointStates[message.name[i]] = message.position[i]

    def get_joint(self, name):
        """Gets the latest joint value.

        Args:
            name: string, the name of the joint whose value we want to read.

        Returns: the joint value, or None if we do not have a value yet.
        """
        if name in self.jointStates.keys():
            return self.jointStates[name]
        else:
            return None

    def get_joints(self, names=None):
        """Gets the latest values for a list of joint names.

        Args:
            name: list of strings, the names of the joints whose values we want
                to read.

        Returns: A list of the joint values. Values may be None if we do not
            have a value for that joint yet.
        """
        if not names:
            return copy.deepcopy(self.jointStates)

        result = []
        for name in names:
            if name in self.jointStates.keys():
                result.append(self.jointStates[name])
            else:
                result.append(None)
        return result
