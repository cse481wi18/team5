#! /usr/bin/env python

import fetch_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from geometry_msgs.msg import PoseStamped, Point
import copy

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

def wait_for_time():
    while rospy.Time.now().to_sec() == 0:
        pass

def make_6dof_controls():
    controls = []
    
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))
    
    return controls

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        pose_st = PoseStamped()
        pose_st.header.frame_id = "wrist_roll_link"
        pose_st.pose.position = Point(0, 0, 0)
        pose_st.pose.orientation.w = 1
        self._gripper_im = self.get_marker(pose_st)

    def get_marker(self, pose_st):
        gripper_m = Marker()
        gripper_m.header = pose_st.header
        gripper_m.type = Marker.MESH_RESOURCE
        gripper_m.mesh_resource = GRIPPER_MESH
        gripper_m.pose = copy.deepcopy(pose_st.pose)
        gripper_m.pose.position.x += 0.166
        gripper_m.scale.x = 1.0
        gripper_m.scale.y = 1.0
        gripper_m.scale.z = 1.0

        finger_left_m = Marker()
        finger_left_m.header = pose_st.header
        finger_left_m.type = Marker.MESH_RESOURCE
        finger_left_m.mesh_resource = L_FINGER_MESH
        finger_left_m.pose = copy.deepcopy(pose_st.pose)
        finger_left_m.pose.position.x += 0.166
        finger_left_m.pose.position.y -= 0.054
        finger_left_m.scale.x = 1.0
        finger_left_m.scale.y = 1.0
        finger_left_m.scale.z = 1.0

        finger_right_m = Marker()
        finger_right_m.header = pose_st.header
        finger_right_m.type = Marker.MESH_RESOURCE
        finger_right_m.mesh_resource = R_FINGER_MESH
        finger_right_m.pose = copy.deepcopy(pose_st.pose)
        finger_right_m.pose.position.x += 0.166
        finger_right_m.pose.position.y += 0.054
        finger_right_m.scale.x = 1.0
        finger_right_m.scale.y = 1.0
        finger_right_m.scale.z = 1.0

        gripper_control = InteractiveMarkerControl()
        gripper_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        gripper_control.orientation = pose_st.pose.orientation
        gripper_control.always_visible = True
        gripper_control.markers.append(gripper_m)
        gripper_control.markers.append(finger_left_m)
        gripper_control.markers.append(finger_right_m)

        for m in gripper_control.markers:
            m.color.r = 0.0
            m.color.g = 0.5
            m.color.b = 0.5
            m.color.a = 1.0

        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "wrist_roll_link"
        gripper_im.name = "GripperMarker"
        gripper_im.description = "Gripper Interactive Marker"
        gripper_im.controls.append(gripper_control)
        gripper_im.controls.extend(make_6dof_controls())
        gripper_im.scale = 0.2
        return gripper_im

    def start(self):
        self._im_server.insert(self._gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        rospy.loginfo("gripper clicked")


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        obj_im = InteractiveMarker()
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
    #auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    #auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    #auto_pick.start()

    rospy.spin()

if __name__=="__main__":
    main()
