#!/usr/bin/env python

import sys
import copy

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesGoal, ControllerState, QueryControllerStatesAction
import actionlib
import fetch_api
from joint_state_reader import JointStateReader

FILE_NAME = "program_by_demo_saved.pickle"

HELP_MESSAGE = """ This program lets you save a list of poses the gripper can take
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
        self._gripper = fetch_api.Gripper()

    def get_ar_tag_markers(self):
        return copy.deepcopy(self._ar_tags)

    def get_default_frame(self):
        return "base_link"

    def get_choose_frame_text(self):
        choose_frame_text = "choose a frame by index:\n"
        choose_frame_text += ("-1: %s; " % self.get_default_frame())
        for i, tag in enumerate(self._ar_tags):
            choose_frame_text += ('%d: %s; ' % (i, tag.id))

        return choose_frame_text

    # CALLBACKS

    def _ar_feedback(self, msg):
        self._ar_tags = msg.markers

    def _arm_feedback(self, msg):
        self._joints = msg

    # ACTIONS

    def _send_arm_goal(self, controller_state):
        """
        Helper function for sending an arm goal
        :param state: RUNNING or STOPPED
        """
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = controller_state
        goal.updates.append(state)

        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    def relax_arm(self):
        self._send_arm_goal(ControllerState.STOPPED)

    def start_arm(self):
        self._send_arm_goal(ControllerState.RUNNING)

    def close_gripper(self):
        gripper.close(gripper.MAX_EFFORT)

    def open_gripper(self):
        gripper.open()


class Program:
    """
    This defines a program that the user can define by manually setting the end-effector positions
    and saving them with the command-line application.
    """
    def __init__(self):
        self._creating = False
        self._poses = []

    def save_pose(self, pose, gripper_state, frame_index):
        self._poses.append({"pose": None, "gripper_state": gripper_state, "frame": FRAMES[frame_index]})

    def get_poses(self):
        return copy.deepcopy(self._poses)

    def run(self):
        pass


MODE_MAIN = 0
MODE_PROGRAM = 1
MODE_SELECT_FRAME = 2

MODES = ["main", "program", "select_frame"]

class PbdCli:
    def __init__(self):
        self._mode = MODE_MAIN  # defines the mode of the CLI application
        self._pbd = ProgramByDemoHelper()
        self._current_program = None
        self._programs = {}

    def run(self):
        while True:
            self._handle_command(self._get_command())

    def _handle_command(self, command):
        if command[0] == "exit":
            exit(0)

        if self._mode is MODE_MAIN:
            if command[0] == "start":
                self._current_program = Program()
                self._pbd.relax_arm()
                self._mode = MODE_PROGRAM
            elif command[0] == "run":
                if len(command) < 2:
                    print "provide program name"
                    return
                if command[1] not in self._programs:
                    print "invalid program"
                    return

                self._programs[command[1]].run()
            else:
                print HELP_MESSAGE
        elif self._mode is MODE_PROGRAM:
            if command[0] == "save":
                print self._pbd.get_choose_frame_text()
                self._mode = MODE_SELECT_FRAME
            elif command[0] == "closegripper":
                self._pbd.close_gripper()
            elif command[0] == "opengripper":
                self._pbd.open_gripper()
            elif command[0] == "finish":
                if len(command) < 2:
                    print "provide program name"
                    return

                self._programs[command[1]] = self._current_program
                self._current_program = None
                self._pbd.start_arm()
                self._mode = MODE_MAIN
            else:
                print HELP_MESSAGE
        elif self._mode is MODE_SELECT_FRAME:
            try:
                index = int(command[0])
            except ValueError:
                print "provide valid index"
                return
            ''' Sarang: so what I've been thinking is that pbd's get action would do
                all the heavy lifting and return a tuple that hold's the pose data
                and the gripper open/close state respective to the frame passed in

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
            if index == -1:
                self._current_program.save_pose(self._pbd.get_action("base_link"))
            elif 0 <= index < len(FRAMES):
                self._current_program.save_pose(self._pbd.get_action())
                self._mode = MODE_PROGRAM
            else:
                print "illegal frame"
        else:
            raise Exception("Illegal pbd mode")

    def _get_command(self):
        return raw_input('[%s] > ' % MODES[self._mode]).split(" ")


def wait_for_time():
    while rospy.Time.now().to_sec() == 0:
        pass


def main():
    print HELP_MESSAGE
    rospy.init_node('program_by_demo')

    cli = PbdCli()
    cli.run()


if __name__ == "__main__":
    main()
