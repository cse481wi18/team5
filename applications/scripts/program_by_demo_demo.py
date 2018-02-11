#!/usr/bin/env python

import sys
import copy

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesGoal, ControllerState

FILE_NAME = "program_by_demo_saved.pickle"

HELP_MESSAGE = """ This program lets you save a list of actions the gripper can take
for future use. These commands do not manipulate the robot.
Commands:
    start: If a programmed action is not currently being created, start creation of one
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


class Action:
    """
    TODO: use this class to save information about end-effector poses as part of a Program
    """

    def __init__(self):
        pass


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
        self._controller_client = None  # TODO create action client to relax arm and call

    def getARTagMarkers(self):
        return self._ar_tags

    def getGripperPose(self):
        pass

    def isGripperOpen(self):
        pass

    def isGripperClosed(self):
        pass

    def runProgram(self, prog):
        pass

    #### Need a bunch of handlers for each subcriber/actionclient callback

    def _ar_feedback(self, msg):
        self._ar_tags = msg.markers

    def _send_arm_goal(self, state):
        """
        Helper function for sending an arm goal
        :param state: RUNNING or STOPPED
        """
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = state
        goal.updates.append(state)

        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    def relax_arm(self):
        self._send_arm_goal(ControllerState.STOPPED)

    def start_arm(self):
        self._send_arm_goal(ControllerState.RUNNING)

    def get_action(self):
        pass


class Program:
    """
    This defines a program that the user can define by manually setting the end-effector positions
    and saving them with the command-line application.
    """

    def __init__(self):
        self._creating = False
        self._actions = []

    def save_action(self, frame_index):
        # TODO implement
        self._actions.append({"action": None, "frame": FRAMES[frame_index]})

    def get_actions(self):
        return copy.deepcopy(self._actions)

    def run(self):
        for action in self._actions:
            # TODO this
            action.run()


MODE_MAIN = 0
MODE_PROGRAM = 1
MODE_SELECT_FRAME = 2

MODES = ["main", "program", "select_frame"]
FRAMES = ["frame a"]  # TODO populate


class PdbCli:
    def __init__(self):
        self._mode = MODE_MAIN  # defines the mode of the CLI application
        self._pdb = ProgramByDemoHelper()
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
                print "choose a frame by index:"
                for i, frame in enumerate(FRAMES):
                    sys.stdout.write('%d: %s; ' % (i, frame))
                print ""
                self._mode = MODE_SELECT_FRAME
            elif command[0] == "finish":
                if len(command) < 2:
                    print "provide program name"
                    return

                self._programs[command[1]] = self._current_program
                self._current_program = None
                self._mode = MODE_MAIN
            else:
                print HELP_MESSAGE
        elif self._mode is MODE_SELECT_FRAME:
            try:
                index = int(command[0])
            except ValueError:
                print "provide valid index"
                return

            if 0 <= index < len(FRAMES):
                self._current_program.save_action(self._pdb.get_action(), index)
                self._mode = MODE_PROGRAM
            else:
                print "illegal frame"
        else:
            raise Exception("Illegal Pdb mode")

    def _get_command(self):
        return raw_input('[%s] > ' % MODES[self._mode]).split(" ")


def wait_for_time():
    while rospy.Time.now().to_sec() == 0:
        pass


def main():
    print HELP_MESSAGE
    rospy.init_node('program_by_demo')

    cli = PdbCli()
    cli.run()


if __name__ == "__main__":
    main()
