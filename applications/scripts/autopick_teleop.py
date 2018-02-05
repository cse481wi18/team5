#! /usr/bin/env python

import copy

import fetch_api
import rospy
from geometry_msgs.msg import PoseStamped, Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker, \
    MenuEntry

from fetch_api import Gripper

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

# Menu entry ids
PICKUP_ENTRY = 1
OPEN_ENTRY = 2
CLOSE_ENTRY = 3


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


def create_menu():
    menu_entries = []

    pickup_object_entry = MenuEntry()
    open_gripper_entry = MenuEntry()
    close_gripper_entry = MenuEntry()
    # go to pose
    pickup_object_entry.id = PICKUP_ENTRY
    pickup_object_entry.command_type = MenuEntry.FEEDBACK
    pickup_object_entry.title = "Pickup object"
    menu_entries.append(pickup_object_entry)

    # open gripper
    open_gripper_entry.id = OPEN_ENTRY
    open_gripper_entry.command_type = MenuEntry.FEEDBACK
    open_gripper_entry.title = "Open gripper"
    menu_entries.append(open_gripper_entry)

    # close gripper
    close_gripper_entry.id = CLOSE_ENTRY
    close_gripper_entry.command_type = MenuEntry.FEEDBACK
    close_gripper_entry.title = "Close gripper"
    menu_entries.append(close_gripper_entry)

    return menu_entries


def get_gripper_markers(pose_st):
    """
    Returns a set of 3 markers to represent a single gripper.
    :param pose_st: the center-back ('base') of the gripper
    :return:
    """
    gripper_m = Marker()
    gripper_m.type = Marker.MESH_RESOURCE
    gripper_m.mesh_resource = GRIPPER_MESH
    gripper_m.pose = copy.deepcopy(pose_st.pose)
    gripper_m.pose.position.x += 0.166
    gripper_m.scale.x = 1.0
    gripper_m.scale.y = 1.0
    gripper_m.scale.z = 1.0

    finger_left_m = Marker()
    finger_left_m.type = Marker.MESH_RESOURCE
    finger_left_m.mesh_resource = L_FINGER_MESH
    finger_left_m.pose = copy.deepcopy(pose_st.pose)
    finger_left_m.pose.position.x += 0.166
    finger_left_m.pose.position.y -= 0.054
    finger_left_m.scale.x = 1.0
    finger_left_m.scale.y = 1.0
    finger_left_m.scale.z = 1.0

    finger_right_m = Marker()
    finger_right_m.type = Marker.MESH_RESOURCE
    finger_right_m.mesh_resource = R_FINGER_MESH
    finger_right_m.pose = copy.deepcopy(pose_st.pose)
    finger_right_m.pose.position.x += 0.166
    finger_right_m.pose.position.y += 0.054
    finger_right_m.scale.x = 1.0
    finger_right_m.scale.y = 1.0
    finger_right_m.scale.z = 1.0

    return [gripper_m, finger_left_m, finger_right_m]


def get_object_marker(pose_st):
    """
    Returns an object marker at the given pose.
    :param pose_st:
    :return:
    """
    object_m = Marker()
    object_m.type = Marker.CUBE
    object_m.pose = copy.deepcopy(pose_st.pose)
    object_m.scale.x = 0.05
    object_m.scale.y = 0.05
    object_m.scale.z = 0.05

    return object_m


def create_im(pose_st, markers):
    """
    Creates an interactive marker.
    :return:
    """
    gripper_control = InteractiveMarkerControl()
    gripper_control.interaction_mode = InteractiveMarkerControl.MENU
    gripper_control.orientation = pose_st.pose.orientation
    gripper_control.always_visible = True
    gripper_control.markers.extend(markers)

    # set every marker in the gripper IM to green
    for m in gripper_control.markers:
        m.color = ColorRGBA(0, 1, 0, 1)

    gripper_im = InteractiveMarker()
    gripper_im.header = copy.deepcopy(pose_st.header)
    gripper_im.name = "GripperMarker"
    gripper_im.description = "Gripper AutoPick Interactive Marker"
    gripper_im.controls.append(gripper_control)
    gripper_im.controls.extend(make_6dof_controls())
    gripper_im.scale = 0.2
    gripper_im.menu_entries.extend(create_menu())
    return gripper_im


POSE_OFFSET_PRE = 0
POSE_OFFSET_GRASP = 1
POSE_OFFSET_LIFT = 2

POSE_OFFSET_GRASP_X = .175


def get_pose_offset(pose_st, type):
    """
    TODO: make this rotation-friendly
    :param pose_st:
    :param type:
    :return:
    """
    new_pose_st = copy.deepcopy(pose_st)
    if type == POSE_OFFSET_PRE:
        new_pose_st.pose.position.x -= 0.4
    elif type == POSE_OFFSET_GRASP:
        new_pose_st.pose.position.x -= POSE_OFFSET_GRASP_X
    elif type == POSE_OFFSET_LIFT:
        new_pose_st.pose.position.x -= POSE_OFFSET_GRASP_X
        new_pose_st.pose.position.z += 0.3
    return new_pose_st


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

        # This pose represents the center of our object, and is what we use for the control
        pose_st = PoseStamped()
        pose_st.header.frame_id = "base_link"
        pose_st.pose.position = Point(0, 0, 0)
        pose_st.pose.orientation.w = 1

        # Instead of just one gripper marker, we now have three
        pre_pose_st = get_pose_offset(pose_st, POSE_OFFSET_PRE)
        grasp_pose_st = get_pose_offset(pose_st, POSE_OFFSET_GRASP)
        lift_pose_st = get_pose_offset(pose_st, POSE_OFFSET_LIFT)

        all_markers = get_gripper_markers(pre_pose_st) + get_gripper_markers(grasp_pose_st) + get_gripper_markers(
            lift_pose_st)
        all_markers.append(get_object_marker(pose_st))
        self._gripper_im = create_im(pose_st, all_markers)

    def start(self):
        self._im_server.insert(self._gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def get_unreachable_markers(self, header):
        """
        Returns a list of the *unreachable* markers in the current orientation
        :param header: the header from the feedback callback method
        """
        unreachables = []
        for marker in self._gripper_im.controls[0].markers:
            pose_st = PoseStamped()
            pose_st.pose = marker.pose
            pose_st.header = header
            if not self._arm.compute_ik(pose_st):
                unreachables.append(marker)
        return unreachables

    def handle_feedback(self, feedback):
        new_pose_st = PoseStamped()
        new_pose_st.pose = feedback.pose
        new_pose_st.header = feedback.header
        feedback.header.stamp = rospy.Time(0)
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == PICKUP_ENTRY:
                self._gripper.open()
                self._arm.move_to_pose(get_pose_offset(new_pose_st, POSE_OFFSET_PRE))
                self._arm.move_to_pose(get_pose_offset(new_pose_st, POSE_OFFSET_GRASP))
                self._gripper.close(Gripper.MAX_EFFORT)
                self._arm.move_to_pose(get_pose_offset(new_pose_st, POSE_OFFSET_LIFT))
            elif feedback.menu_entry_id == OPEN_ENTRY:
                self._gripper.open()
            elif feedback.menu_entry_id == CLOSE_ENTRY:
                self._gripper.close(self._gripper.MAX_EFFORT)
            else:
                pass
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # update color of the gripper based on if pose is possible
            markers = self._gripper_im.controls[0].markers
            self._gripper_im.pose = new_pose_st.pose

            # Check each gripper and color accordingly
            green, red = ColorRGBA(0, 1, 0, 1), ColorRGBA(1, 0, 0, 1)

            is_valid = True

            if self._arm.compute_ik(get_pose_offset(new_pose_st, POSE_OFFSET_PRE)):
                markers[0].color = green
                markers[1].color = green
                markers[2].color = green
            else:
                is_valid = False
                markers[0].color = red
                markers[1].color = red
                markers[2].color = red

            if self._arm.compute_ik(get_pose_offset(new_pose_st, POSE_OFFSET_GRASP)):
                markers[3].color = green
                markers[4].color = green
                markers[5].color = green
            else:
                is_valid = False
                markers[3].color = red
                markers[4].color = red
                markers[5].color = red

            if self._arm.compute_ik(get_pose_offset(new_pose_st, POSE_OFFSET_LIFT)):
                markers[6].color = green
                markers[7].color = green
                markers[8].color = green
            else:
                is_valid = False
                markers[6].color = red
                markers[7].color = red
                markers[8].color = red

            if is_valid:
                markers[9].color = green
            else:
                markers[9].color = red
        self.start()


def main():
    """
    Runs the autopick teleop
    """
    rospy.init_node('autopick_teleop')
    wait_for_time()

    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    auto_pick.start()

    rospy.spin()


if __name__ == "__main__":
    main()
