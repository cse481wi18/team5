#! usr/bin/bash

import fetch_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def get_marker(pose):
        gripper_m = Marker()
        gripper_m.type = marker.mesh_resource
        gripper_m.mesh_resource = GRIPPER_MESH
        gripper_m.pose = pose

        finger_left_m = Marker()
        finger_left_m.type = marker.mesh_resource
        finger_left_m.mesh_resource = L_FINGER_MESH
        finger_left_m.pose = pose

        finger_right_m = Marker()
        finger_right_m.type = marker.mesh_resource
        finger_right_m.mesh_resource = R_FINGER_MESH
        finger_right_m.pose = pose

        gripper_control = InteractiveMarkerControl()
        gripper_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        gripper_control.orientation = pose.orientation
        gripper_control.always_visible = True
        gripper_control.markers.append(gripper_m)
        gripper_control.markers.append(finger_left_m)
        gripper_control.markers.append(finger_right_m)

        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "wrist_roll_link"
        gripper_im.name = "GripperMarker"
        gripper_im.description = "Gripper Interactive Marker"
        gripper_im.controls.append(gripper_control)

        return gripper_im

    def start(self):
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # obj_im = InteractiveMarker() ...
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


def main():
    """
    ...
    """
    rospy.init_node('gripper_teleop')
    wait_for_time()

    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()

    rospy.spin()
