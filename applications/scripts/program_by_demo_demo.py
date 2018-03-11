#!/usr/bin/env python

import copy
import json

import actionlib
import pickle
from fetch_api import Arm, Gripper, ArmJoints
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from joint_state_reader import JointStateReader
from robot_controllers_msgs.msg import QueryControllerStatesGoal, ControllerState, QueryControllerStatesAction
from tf import TransformListener, TransformerROS
from geometry_msgs.msg import PoseStamped
from map_annotator import Annotator


FIELD_GRIP_STATE = "grip_state"

FIELD_POSITION = "position"
''' 
    The save action in current program would save a tuple that consists
    of some sort of frame identifier, the position relative to the frame,
    and the open/close state of the gripper. Good practice in use would be
    to save right after closing a gripper even if position doesn't change
    so that the ordering of arm movement and gripper movement is preserved.

    One thing I changed, but you can feel free to revert is that I made
    getting the frames part of the pbd helper. This seemed to make sense because
    we dynamically want to generate ar tag's. A potential change you might have
    to make because of this is that you'd have to save the action in the same
    iteration of the while loop as when you decide which frame to save in (currently
    you do it in the next iteration when the program mode changes), just in case
    the arm's state changes between iterations and the result from get_ar_tag_markers()
    is different from the previous iteration's.
'''

RVIZ = True
FILE_NAME = "program_by_demo_saved.pickle"
DEFAULT_FRAME = "base_link"

HELP_MESSAGE_OLD = """ This program lets you save a list of poses the gripper can take
for future use. These commands do not manipulate the robot.
Commands:
    start: If a programmed pose is not currently being created, start creation of one
    save: save the current pose, which includes the state of the gripper (opened or closed)
        - After saving, a numbered list of frames to save the pose in are provided. Provide
        the appropriate number to make a frame selection
        - If you want to be particular about how you order arm movement with opening/closing
        of the gripper, order your saving of poses accordingly (e.g. save once after moving,
        and again after closing the gripper even if the arm position doesn't change)
    finish <program_name>: save your program with the name "program_name"
    run <program_name>: run a previously saved program "program_name"
    exit: quits the program
    ***<program_name> cannot have spaces***
"""

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 1


def export_program(name, program):
    pickle.dump(program, open("pbd_%s.pickle" % name, 'wb'))


def import_program(name):
    return pickle.load(open("pbd_%s.pickle" % name, 'rb'))


class ProgramByDemoHelper:
    """
    Helper class for all robot info.
    """

    # Need to subscribe to topics that publish:
    # poses of the ar tags
    # pose of the gripper
    # poses of the gripper fingers
    def __init__(self):
        self._ar_tags = []
        self._ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._ar_feedback)
        self._joint_states_reader = JointStateReader()
        self._controller_client = actionlib.SimpleActionClient('query_controller_states', QueryControllerStatesAction)
        self._gripper = Gripper()
        self._tfl = TransformListener()
        self._tfr = TransformerROS()
        self._arm = Arm()

    def get_ar_tag_markers(self):
        return copy.deepcopy(self._ar_tags)

    # CALLBACKS

    def _ar_feedback(self, msg):
        self._ar_tags = msg.markers

    # ACTIONS

    def _send_arm_goal(self, controller_state, rviz):
        """
        Helper function for sending an arm goal
        :param state: RUNNING or STOPPED
        """
        if RVIZ:
            return

        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = controller_state
        goal.updates.append(state)

        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    def relax_arm(self):
        self._send_arm_goal(ControllerState.STOPPED, RVIZ)

    def start_arm(self):
        self._send_arm_goal(ControllerState.RUNNING, RVIZ)

    def close_gripper(self):
        self._gripper.close(self._gripper.MAX_EFFORT)

    def open_gripper(self):
        self._gripper.open()

    def get_position(self, frame):
        """
        Returns a map
        - joints -> list of the 7 joints we save
        - gripper -> state of gripper
        """
        pose_st = PoseStamped()
        pose_st.header.frame_id = frame
        curr_time = rospy.Time(0)
        # first
        self._tfl.waitForTransform(frame, "wrist_roll_link", curr_time, rospy.Duration(10))
        p, o = self._tfl.lookupTransform(frame, "wrist_roll_link", curr_time)
        pose_st.pose.position.x = p[0]
        pose_st.pose.position.y = p[1]
        pose_st.pose.position.z = p[2]
        pose_st.pose.orientation.x = o[0]
        pose_st.pose.orientation.y = o[1]
        pose_st.pose.orientation.z = o[2]
        pose_st.pose.orientation.w = o[3]
        return {
            FIELD_POSITION: pose_st,
            FIELD_GRIP_STATE: self._get_gripper_state()
        }

    def run_program(self, program):
        """
        Runs the given program
        :param program: list of positions
        :return:
        """
        for pos in program:
            saved_pos = pos["position"]
            # transform back into the base frame when running
            base_pose = self._tfl.transformPose(DEFAULT_FRAME, saved_pos)
            self._arm.move_to_pose(base_pose)
            if pos["grip_state"] == GRIPPER_OPEN and self._get_gripper_state() == GRIPPER_CLOSE:
                self._gripper.open()
            elif pos["grip_state"] == GRIPPER_CLOSE and self._get_gripper_state() == GRIPPER_OPEN:
                self._gripper.close()

    def _get_gripper_state(self):
        if abs(0.05 - self._joint_states_reader.get_joint('l_gripper_finger_joint')) < 0.002:
            return GRIPPER_OPEN
        else:
            return GRIPPER_CLOSE


MODE_MAIN = 0
MODE_PROGRAM = 1
MODE_SELECT_FRAME = 2

MODES = ["main", "program", "select_frame"]
COMMANDS = {
    "main": ["start", "list", "run", "export", "import"],
    "program": ["save", "closegripper", "opengripper", "len", "finish"],
    "select_frame": ["<N>"]
}


class PbdCli:
    def __init__(self):
        self._mode = MODE_MAIN  # defines the mode of the CLI application
        self._pbd = ProgramByDemoHelper()
        self._current_program = []
        self._programs = {}
        self._current_ar_tags = None
        #self._map_annotator = Annotator()

    def run(self):
        while True:
            self._handle_command(self._get_command())

    def _handle_command(self, command):
        if command[0] == "exit":
            exit(0)
        elif command[0] == "help":
            print json.dumps(COMMANDS)
            return

        if self._mode is MODE_MAIN:
            def two_args(command):
                if len(command) < 2:
                    print "provide program name"
                    return False
                if command[1] not in self._programs:
                    print "invalid program"
                    return False
                return True

            if command[0] == "start":
                self._pbd.relax_arm()
                self._mode = MODE_PROGRAM
            elif command[0] == "list":
                for name in self._programs:
                    print name
            elif command[0] == "run":
                if not two_args(command):
                    return

                print "running program %s" % command[1]
                self._pbd.run_program(self._programs[command[1]])
            elif command[0] == "export":
                if not two_args(command):
                    return
                print "exporting program %s to disk" % command[1]
                print self._programs[command[1]]
                export_program(command[1], self._programs[command[1]])
            elif command[0] == "import":
                if len(command) < 2:
                    print "provide program name"
                    return
                self._programs[command[1]] = import_program(command[1])
            else:
                print "bad command"
        elif self._mode is MODE_PROGRAM:
            if command[0] == "save":
                # Save the current ar tags so that there's no inconsistency in the tag
                self._current_ar_tags = self._pbd.get_ar_tag_markers()
                self._print_choose_frame_text()
                self._mode = MODE_SELECT_FRAME
            elif command[0] == "closegripper":
                self._pbd.close_gripper()
            elif command[0] == "opengripper":
                self._pbd.open_gripper()
            elif command[0] == "len":
                print len(self._current_program)
            elif command[0] == "finish":
                if len(command) < 2:
                    print "provide program name"
                    return

                self._programs[command[1]] = self._current_program
                self._current_program = []
                self._pbd.start_arm()
                self._mode = MODE_MAIN
            else:
                print "bad command"
        elif self._mode is MODE_SELECT_FRAME:
            try:
                index = int(command[0])
            except ValueError:
                print "provide valid index"
                return

            # Special case for base link
            if -1 <= index < len(self._current_ar_tags):
                frame_string = DEFAULT_FRAME
                if index >= 0:
                    frame_string = "ar_marker_%d" % self._current_ar_tags[index].id
                self._current_program.append(self._pbd.get_position(frame_string))
                self._mode = MODE_PROGRAM
            else:
                print "illegal frame"
        else:
            raise Exception("Illegal pbd mode")

    def _get_command(self):
        return raw_input('[%s] > ' % MODES[self._mode]).split(" ")

    def _print_choose_frame_text(self):
        """
        Prints the helper text to choose a frame
        """
        choose_frame_text = "choose a frame by index:\n"
        choose_frame_text += ("-1: %s; " % DEFAULT_FRAME)
        for i, tag in enumerate(self._current_ar_tags):
            choose_frame_text += ('%d: %s; ' % (i, tag.id))

        print choose_frame_text


def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    print json.dumps(COMMANDS)
    rospy.init_node('program_by_demo')

    cli = PbdCli()
    cli.run()


if __name__ == "__main__":
    main()
