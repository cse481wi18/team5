#!/usr/bin/env python

from gripper_teleop import GripperTeleop
import fetch_api
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker, MenuEntry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
import copy
import pickle
import tf
FILE_NAME = "program_by_demo_saved.pickle"

class ProgramByDemo(GripperTeleop):

    GOTO_ENTRY = 1
    OPEN_ENTRY = 2
    CLOSE_ENTRY = 3
    CREATE_PROGRAM = 4
    SAVE_POSE = 5
    SAVE_PROGRAM = 6
    CHANGE_FRAME = 7
    BASE_LINK_FRAME = 8
    TAG_OFFSET = 9

    def __init__(self, arm, gripper, im_server, data_file=FILE_NAME):
        self._ar_tags = []
        super(ProgramByDemo, self).__init__(arm, gripper, im_server, self.handle_feedback)
        self.action_list = []
        # frame marker of none implies base_link frame
        self._frame = "base_link"
        self._gripper_open = True
        self._programming = False
        self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.tags_callback)
        self._data_file = data_file


    def tags_callback(self, feedback):
        self._ar_tags = feedback.markers
        self._gripper_im.menu_entries = self.create_menu()


    def create_menu(self):
        menu_entries = super(ProgramByDemo, self).create_menu()
        
        create_program_entry = MenuEntry()
        save_pose_entry = MenuEntry()
        save_program_entry = MenuEntry()
        change_frame_entry = MenuEntry()
        base_link_frame_entry = MenuEntry()
        # create program
        create_program_entry.id = self.CREATE_PROGRAM
        create_program_entry.command_type = MenuEntry.FEEDBACK
        create_program_entry.title = "Create program"
        menu_entries.append(create_program_entry)
        # save pose
        save_pose_entry.id = self.SAVE_POSE
        save_pose_entry.command_type = MenuEntry.FEEDBACK
        save_pose_entry.title = "Save pose"
        menu_entries.append(save_pose_entry)
         # create program
        save_program_entry.id = self.SAVE_PROGRAM
        save_program_entry.command_type = MenuEntry.FEEDBACK
        save_program_entry.title = "Save program"
        menu_entries.append(save_program_entry)
        # change frame
        change_frame_entry.id = self.CHANGE_FRAME
        change_frame_entry.command_type = MenuEntry.FEEDBACK
        change_frame_entry.title = "Change frame"
        menu_entries.append(change_frame_entry)
        # base link frame
        base_link_frame_entry.id = self.BASE_LINK_FRAME
        base_link_frame_entry.parent_id = self.CHANGE_FRAME
        base_link_frame_entry.command_type = MenuEntry.FEEDBACK
        base_link_frame_entry.title = "base_link frame"
        menu_entries.append(base_link_frame_entry)

        for marker in self._ar_tags:
            tag_entry = MenuEntry()
            tag_entry.id = self.TAG_OFFSET + marker.id
            tag_entry.parent_id = self.CHANGE_FRAME
            tag_entry.command_type = MenuEntry.FEEDBACK
            tag_entry.title = "Tag %d" % marker.id
            menu_entries.append(tag_entry)
        return menu_entries


    def handle_feedback(self, feedback):
         new_pose_st = PoseStamped()
         new_pose_st.pose = feedback.pose
         new_pose_st.header = feedback.header
         feedback.header.stamp = rospy.Time(0)
         if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
             if feedback.menu_entry_id == self.GOTO_ENTRY:
                 self._arm.move_to_pose(new_pose_st)
             elif feedback.menu_entry_id == self.OPEN_ENTRY:
                 self._gripper.open()
                 self._gripper_open = True
             elif feedback.menu_entry_id == self.CLOSE_ENTRY:
                 self._gripper.close()
                 self._gripper_open = False
             
             elif feedback.menu_entry_id == self.CREATE_PROGRAM:
                 self._programming = True
             elif feedback.menu_entry_id == self.SAVE_POSE:
                 if self._programming:
                     self.action_list.append("open" if self._gripper_open else "close")
                     rospy.loginfo(new_pose_st)
                     rospy.loginfo(tf.TransformListener().transformPose("gripper_link", new_pose_st)) 
                     new_pose_st = tf.TransformListener().transformPose(self._frame, new_pose_st)
                     rospy.loginfo(new_pose_st)
                     self.action_list.append(new_pose_st)
             elif feedback.menu_entry_id == self.SAVE_PROGRAM:
                 if self._programming:
                     self._programming = False
                     print self.action_list
                     # Pickle that shit
                     pickle.dump(self.action_list, open(self._data_file, "wb"))
                     self.action_list = []
             elif feedback.menu_entry_id >= self.BASE_LINK_FRAME:
                 if feedback.menu_entry_id == self.BASE_LINK_FRAME:
                     self._frame_marker = None
                 else:
                     tag_id = feedback.menu_entry_id - self.TAG_OFFSET
                     for marker in self._ar_tags:
                         if marker.id == tag_id:
                             self._frame = "/ar_marker_" + str(marker.id)
                             break
         elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
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


    def get_action_list(self):
        return copy.deepcopy(self.action_list)


def wait_for_time():
    while rospy.Time.now().to_sec() == 0:
        pass


def main():
    rospy.init_node('program_by_demo')
    wait_for_time()

    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    teleop = ProgramByDemo(arm, gripper, im_server)
    teleop.start()

    rospy.spin()


if __name__== "__main__":
    main()

