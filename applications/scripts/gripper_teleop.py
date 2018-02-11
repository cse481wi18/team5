#! /usr/bin/env python

import fetch_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker, \
    MenuEntry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
import copy

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

# Menu entry ids
GOTO_ENTRY = 1
OPEN_ENTRY = 2
CLOSE_ENTRY = 3


def wait_for_time():
    while rospy.Time.now().to_sec() == 0:
        pass

class GripperTeleop(object):

    # Menu entry ids
    GOTO_ENTRY = 1
    OPEN_ENTRY = 2
    CLOSE_ENTRY = 3
    
    def __init__(self, arm, gripper, im_server, feedback=None):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        pose_st = PoseStamped()
        pose_st.header.frame_id = "base_link"
        pose_st.pose.position = Point(0, 0, 0)
        pose_st.pose.orientation.w = 1
        self._gripper_im = self.get_marker(pose_st)
        if feedback == None:
            self.feedback_fn = self.handle_feedback
        else:
            self.feedback_fn = feedback

    def get_marker(self, pose_st):
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

        gripper_control = InteractiveMarkerControl()
        gripper_control.interaction_mode = InteractiveMarkerControl.MENU
        gripper_control.orientation = pose_st.pose.orientation
        gripper_control.always_visible = True
        gripper_control.markers.append(gripper_m)
        gripper_control.markers.append(finger_left_m)
        gripper_control.markers.append(finger_right_m)

        # set every marker in the gripper IM to green
        for m in gripper_control.markers:
            m.color = ColorRGBA(1, 0, 0, 1)

        gripper_im = InteractiveMarker()
        gripper_im.header = copy.deepcopy(pose_st.header)
        gripper_im.name = "GripperMarker"
        gripper_im.description = "Gripper Interactive Marker"
        gripper_im.controls.append(gripper_control)
        gripper_im.controls.extend(self.make_6dof_controls())
        gripper_im.scale = 0.2
        gripper_im.menu_entries.extend(self.create_menu())
        return gripper_im

    def start(self):
        self._im_server.insert(self._gripper_im, feedback_cb=self.feedback_fn)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        new_pose_st = PoseStamped()
        new_pose_st.pose = feedback.pose
        new_pose_st.header = feedback.header
        feedback.header.stamp = rospy.Time(0)
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == GOTO_ENTRY:
                self._arm.move_to_pose(new_pose_st)
            elif feedback.menu_entry_id == OPEN_ENTRY:
                self._gripper.open()
            elif feedback.menu_entry_id == CLOSE_ENTRY:
                self._gripper.close(self._gripper.MAX_EFFORT)
            else:
                pass
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # update color of the gripper based on if pose is possible
            gripper_control = self._gripper_im.controls[0]
            self._gripper_im.pose = new_pose_st.pose
            if self._arm.compute_ik(new_pose_st):
                for m in gripper_control.markers:
                    m.color = ColorRGBA(0, 1, 0, 1)
            else:
                for m in gripper_control.markers:
                    m.color = ColorRGBA(1, 0, 0, 1)
        self._im_server.insert(self._gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def make_6dof_controls(self):
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
    
    
    def create_menu(self):
        menu_entries = []
    
        goto_pose_entry = MenuEntry()
        open_gripper_entry = MenuEntry()
        close_gripper_entry = MenuEntry()
        # go to pose
        goto_pose_entry.id = self.GOTO_ENTRY
        goto_pose_entry.command_type = MenuEntry.FEEDBACK
        goto_pose_entry.title = "Go to pose"
        menu_entries.append(goto_pose_entry)
    
        # open gripper
        open_gripper_entry.id = self.OPEN_ENTRY
        open_gripper_entry.command_type = MenuEntry.FEEDBACK
        open_gripper_entry.title = "Open gripper"
        menu_entries.append(open_gripper_entry)
    
        # close gripper
        close_gripper_entry.id = self.CLOSE_ENTRY
        close_gripper_entry.command_type = MenuEntry.FEEDBACK
        close_gripper_entry.title = "Close gripper"
        menu_entries.append(close_gripper_entry)
    
        return menu_entries

def main():
    """
    ...
    """
    rospy.init_node('gripper_teleop')
    wait_for_time()

    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    # auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    # auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    # auto_pick.start()

    rospy.spin()


if __name__ == "__main__":
    main()
