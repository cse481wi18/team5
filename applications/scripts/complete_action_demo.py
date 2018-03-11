#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Int32
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import copy
import os
import threading
import pickle
import json
import math
from program_by_demo_demo import PbdCli, ProgramByDemoHelper
import numpy as np
#from annotator import Annotator
import fetch_api
import actionlib
from map_annotator import Annotator
from eos_visual_guide import Leash

MODE_MAIN = 0
MODE_PROGRAM = 1
MODE_SELECT_FRAME = 2
MODE_NAV = 3

MODES = ["main", "program", "select_frame", "nav"]
COMMANDS = {
    "main": ["start", "list", "run", "export", "import", "nav", "help", "exit", "save_all"],
    "program": ["savegrip", "saveloc", "finish", "exit"],
    "select_frame": ["<N>", "exit"],
    "nav": ["move_one <location>", "move <start location> <end location>", "list", "done", "exit"]
}

FIELD_GRIP_STATE = "grip_state"

FIELD_POSITION = "position"

RVIZ = True
FILE_NAME = "program_by_demo_saved.pickle"
DEFAULT_FRAME = "base_link"

# We want to have a file that stores utility gripper program that would contain
# an action that we always want to have available
PRELOADED_PROGRAMS_FILE_NAME = "preload"

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 1
EPS = 0.05
EPS_BIG = 0.2


def export_program(name, program):
    pickle.dump(program, open("ad_%s.pickle" % name, "wb"))

def import_program(name):
    return pickle.load(open("ad_%s.pickle" % name, "rb"))

class ActionByDemoHelper(ProgramByDemoHelper):
    def __init__(self):
        ProgramByDemoHelper.__init__(self)
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.nav_client.wait_for_result()
        self._fsr_sub = rospy.Subscriber("fsr", Int32, callback=self._move_callback)
        self._move = False
        self._head = fetch_api.Head()
        self._leash = Leash(self._leash_callback)
        self._curr_goal = None
        self._should_move = False
        self._active = False

    def _leash_callback(self):
        if (self._active):
            self._should_move = not self._should_move
            if (self._should_move):
                self.nav_client.send_goal(self._curr_goal)
            else:
                self.nav_client.cancel_all_goals()

    def _move_callback(self, msg):
        if(msg.data > 850):
            self._move = not self._move

    def go_to(self, loc_msg):
        goal = MoveBaseGoal()
        new_msg = PoseStamped()
        new_msg.header = loc_msg.header
        new_msg.pose = loc_msg.pose.pose
        goal.target_pose = new_msg
        self._should_move = True
        self._curr_goal = goal
        self._active = True
        self.nav_client.send_goal(goal)
        # This means we only check 1 time every second
        r = rospy.Rate(1)
        while not rospy.is_shutdown(): 
            state = self.nav_client.get_state()
            if (state == GoalStatus.SUCCEEDED):
                break
            elif (state == GoalStatus.REJECTED or state == GoalStatus.ABORTED):
                self.nav_client.send_goal(goal)
            r.sleep()
        # Always reset the pan/tilt to 0 0 after navigation completes
        print "Done moving for now!"
        # TODO wait some time before doing this
        self._head.pan_tilt(0, 0)
        self._active = False
        self._curr_goal = None
        return True

    def run_program(self, program):
        """
        Runs the given program
        :param program: list of positions
        :return:
        """
        for component, pos in program:
            if component == "arm":
                saved_pos = pos["position"]
                # transform back into the base frame when running
                now = rospy.Time(0)
                self._tfl.waitForTransform(DEFAULT_FRAME, saved_pos.header.frame_id, now, rospy.Duration(60))
                base_pose = self._tfl.transformPose(DEFAULT_FRAME, saved_pos)
                self._arm.move_to_pose(base_pose)
                if pos["grip_state"] == GRIPPER_OPEN and self._get_gripper_state() == GRIPPER_CLOSE:
                    self._gripper.open()
                elif pos["grip_state"] == GRIPPER_CLOSE and self._get_gripper_state() == GRIPPER_OPEN:
                    self._gripper.close()
            elif component == "torso":
                self.go_to(pos)

class ActionDemoCli:
    def __init__(self):
        self._mode = MODE_MAIN  # defines the mode of the CLI application
        self._abd = ActionByDemoHelper()
        self._current_program = []
        try:
            self._programs = import_program(PRELOADED_PROGRAMS_FILE_NAME)
        except:
            self._programs = {}
        self._current_ar_tags = None
        self._annotator = Annotator()
    
    def run(self):
        while True:
            self._handle_command(self._get_command())

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

    def _mode_main(self, command):
        def valid_prog(command):
            if command[1] not in self._programs:
                print "invalid program"
                return False
            return True
        if command[0] == "start":
            print "calling start"
            self._abd.relax_arm()
            self._mode = MODE_PROGRAM
        elif command[0] == "list":
            for name in self._programs:
                print name
        elif command[0] == "run":
            if not self.n_args(command, 2) or not valid_prog(command):
                return
            print "running program %s" % command[1]
            self._abd.run_program(self._programs[command[1]])
        elif command[0] == "export":
            if not self.n_args(command, 2) or not valid_prog(command):
                return
            print "exporting program %s to disk" % command[1]
            print self._programs[command[1]]
            export_program(command[1], self._programs[command[1]])
        elif command[0] == "import":
            if not self.n_args(command,  n):
                return
            self._programs[command[1]] = import_program(command[1])
        elif command[0] == "nav":
            self._mode = MODE_NAV
        elif command[0] == "demo":
            """
            We need a new frame describing the creation of a demo
            """
            prog = []
            locs = self._annotator.get_saved_msgs()
            prog.append(("torso", locs["table"]))
            prog.extend(self._programs["first"])
            prog.append(("torso", locs["start"]))
            prog.extend(self._programs["second"])
            self._abd.run_program(prog)
        elif command[0] == "help":
            json.dumps(COMMANDS)
        elif command[0] == "save_all":
            pickle.dump(self._programs, open("ad_preload.pickle", "wb"))
        else:
            print "bad command"

    def _mode_program(self, command):
        if command[0] == "savegrip":
            # Save the current ar tags so that there's no inconsistency in the tag
            self._current_ar_tags = self._abd.get_ar_tag_markers()
            self._print_choose_frame_text()
            self._mode = MODE_SELECT_FRAME
        elif command[0] == "finish":
            if not self.n_args(command, 2):
                return
            self._programs[command[1]] = self._current_program
            self._current_program = []
            self._abd.start_arm()
            self._mode = MODE_MAIN
        else:
            print "bad command"

    def _mode_select_frame(self, command):
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
            self._current_program.append(("arm", self._abd.get_position(frame_string)))
            self._mode = MODE_PROGRAM
        else:
            print "illegal frame"

    def _mode_nav(self, command):
        if command[0] == "move_one":
            locs = self._annotator.get_saved_msgs()
            prog = []
            prog.append(("torso", locs[command[1]]))
            self._abd.run_program(prog)
        elif command[0] == "move":
            if len(command) < 3:
                print "usage: move <starting pose name> <ending pose name>"
                return
            locs = self._annotator.get_saved_msgs()
            if command[1] not in locs.keys():
                print "invalid starting position"
                return
            if command[2] not in locs.keys():
                print "invalid ending position"
                return
            prog = []
            prog.append(("torso", locs[command[1]]))
            prog.append(("torso", locs[command[2]]))
            self._abd.run_program(prog)
        elif command[0] == "list":
            locs = self._annotator.get_saved_msgs()
            for loc in locs.keys():
                print "\t", loc
        elif command[0] == "done":
            self._mode = MODE_MAIN
        else:
            print "bad command"

    def n_args(self, command, n):
        if len(command) < n:
            print "provide program name"
            return False
        return True

    def _handle_command(self, command):
        if command[0] == "exit":
            exit(0)
        elif command[0] == "help":
            print json.dumps(COMMANDS)
            return
        if self._mode is MODE_MAIN:
            self._mode_main(command)
        elif self._mode is MODE_PROGRAM:
            self._mode_program(command)
        elif self._mode is MODE_SELECT_FRAME:
            self._mode_select_frame(command)
        elif self._mode is MODE_NAV:
            self._mode_nav(command)
        else:
            raise Exception("Illegal abd mode")


def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    print json.dumps(COMMANDS)
    rospy.init_node('action_by_demo')
    wait_for_time()

    print "Loading..."

    cli = ActionDemoCli()
    cli.run()


if __name__ == "__main__":
    main()

