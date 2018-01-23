#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse
from web_teleop.srv import SetHeadPanTilt, SetHeadPanTiltResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._head = fetch_api.Head()

    def handle_set_torso(self, request):
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_head_pan_tilt(self, request):
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadPanTiltResponse()

    def handle_set_arm(self, request):
        arm = fetch_api.Arm()
        arm.move_to_joints(fetch_api.ArmJoints.from_list([request.shoulder_pan, request.shoulder_lift, request.upperarm_roll, request.elbow_flex, request.forearm_roll, request.wrist_flex, request.wrist_roll]))
        return SetArmResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    head_service = rospy.Service('web_teleop/set_head_pan_tilt', SetHeadPanTilt,
                                  server.handle_set_head_pan_tilt)
    rospy.spin()


if __name__ == '__main__':
    main()
