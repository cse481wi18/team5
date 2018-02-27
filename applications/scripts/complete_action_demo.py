#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import copy
import os
import threading
import pickle
import json
import math
from program_by_demo_demo import PbdCli, ProgramByDemoHelper
import numpy as np
from annotator import Annotator

MODE_MAIN = 0
MODE_PROGRAM = 1
MODE_SELECT_FRAME = 2
MODE_NAV = 3

MODES = ["main", "program", "select_frame", "nav"]
COMMANDS = {
    "main": ["start", "list", "run", "export", "import", "nav", "help"],
    "program": ["savegrip", "saveloc", "finish"],
    "select_frame": ["<N>"],
    "nav": ["move <start location> <end location>", "list", "finish"]
}

FIELD_GRIP_STATE = "grip_state"

FIELD_POSITION = "position"

RVIZ = True
FILE_NAME = "program_by_demo_saved.pickle"
DEFAULT_FRAME = "base_link"

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 1
EPS = 0.05

def export_program(name, program):
    pickle.dump(program, open("ad_%s.pickle" % name, "wb"))

def import_program(name):
    return pickle.load(open("ad_%s.pickle" % name, "rb"))

class ActionByDemoHelper(ProgramByDemoHelper):
    def __init__(self):
        ProgramByDemoHelper.__init__(self)
        self._loc_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self._loc_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback=self._loc_callback)
        self._curr_loc = None

    def _loc_callback(self, msg):
        self._curr_loc = msg

    def euclidean_distance(self, p1, p2):
        dist_squared = math.pow(p1.pose.position.x - p2.pose.position.x, 2)
        + math.pow(p1.pose.position.y - p2.pose.position.y, 2)
        + math.pow(p1.pose.position.z - p2.pose.position.z, 2)
        return math.sqrt(dist_squared)
   
    def quaternion_dist(self, q1, q2):
        v1 = [q1.x, q1.y, q1.z, q1.w]
        v2 = [q2.x, q2.y, q2.z, q2.w]
        return math.fabs(np.dot(v1, v2))                

    def go_to(self, loc_msg):
        new_msg = PoseStamped()
        new_msg.header = loc_msg.header
        new_msg.pose = loc_msg.pose.pose
        self._loc_pub.publish(new_msg)
        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            print("pos dist: " + str(self.euclidean_distance(new_msg, self._curr_loc.pose)))
            print("orientation dist: " + str(self.quaternion_dist(new_msg.pose.orientation, self._curr_loc.pose.pose.orientation)))
            print()
            if (self.euclidean_distance(new_msg, self._curr_loc.pose) < 0.5
                    and self.quaternion_dist(new_msg.pose.orientation, self._curr_loc.pose.pose.orientation) > 1 - EPS):
                break
            r.sleep()
        return True

    def get_location(self):
        return copy.deepcopy(self._curr_loc)

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
        self._programs = {}
        self._current_ar_tags = None
        self._annotator = Annotator()

    def run(self):
        print (self._annotator.get_saved_msgs())
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
                self._abd.relax_arm()
                self._mode = MODE_PROGRAM
            elif command[0] == "list":
                for name in self._programs:
                    print name
            elif command[0] == "run":
                if not two_args(command):
                    return
                print "running program %s" % command[1]
                self._abd.run_program(self._programs[command[1]])
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
            elif command[0] == "nav":
                self._mode = MODE_NAV
            elif command[0] == "help":
                json.dumps(COMMANDS)
            elif command[0] == "use_anno":
                positions = ["table", "table2", "start", "green"]
                prog = []
                locs = self._annotator.get_saved_msgs()
                for pos in positions:
                    prog.append(("torso", locs[pos]))
                self._abd.run_program(prog)
            else:
                print "bad command"
        elif self._mode is MODE_PROGRAM:
            if command[0] == "savegrip":
                # Save the current ar tags so that there's no inconsistency in the tag
                self._current_ar_tags = self._abd.get_ar_tag_markers()
                self._print_choose_frame_text()
                self._mode = MODE_SELECT_FRAME
            elif command[0] == "saveloc":
                self._current_program.append(("torso", self._abd.get_location()))
            elif command[0] == "finish":
                if len(command) < 2:
                    print "provide program name"
                    return

                self._programs[command[1]] = self._current_program
                self._current_program = []
                self._abd.start_arm()
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
                self._current_program.append(("arm", self._abd.get_position(frame_string)))
                self._mode = MODE_PROGRAM
            else:
                print "illegal frame"
        elif self._mode is MODE_NAV:
            if command[0] == "move":
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
                self._current_program.append(("torso", locs[command[1]]))
                self._current_program.append(("torso", locs[command[2]]))
            elif command[0] == "list":
                locs = self._annotator.get_saved_msgs()
                for loc in locs.keys():
                    print "\t", loc
            elif command[0] == "finish":
                if len(command) < 2:
                    print "provide program name"
                    return
                self._programs[command[1]] = self._current_program
                self._current_program = []
                self._mode = MODE_MAIN
            else:
                print "bad command"
        else:
            raise Exception("Illegal abd mode")


def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    print json.dumps(COMMANDS)
    rospy.init_node('action_by_demo')

    cli = ActionDemoCli()
    cli.run()


if __name__ == "__main__":
    main()

