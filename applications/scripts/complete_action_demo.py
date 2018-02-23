import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import copy
import os
import threading
import pickle

# COMMANDS = """
# start: start considering a series of actions to save
# savegrip: save the position of the gripper
# saveloc: save the location of the robot
# finish <name>: finish and save the series of actions as a program
# run <name>: run the saved program
# list: list the names of all saved programs
# exit: quit the program
# help: print the commands
# """

MODES = ["main", "program", "select_frame"]
COMMANDS = {
    "main": ["start", "list", "run", "export", "import"],
    "program": ["savegrip", "goto <N>", "listloc", "closegripper", "opengripper",
                "len", "finish"],
    "select_frame": ["<N>"]
}


class ActionDemoCli(PbdCli):
    def __init__(self):
        self._mode = MODE_MAIN  # defines the mode of the CLI application
        self._pbd = ProgramByDemoHelper()
        self._current_program = []
        self._programs = {}
        self._current_ar_tags = None
        self._map_annotator = Annotator()

    def _get_pose_strings(self):
        msgs = self._map_annotator.get_saved_msgs()
        print msgs
        
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
            def two_args(command):
                if len(command) < 2:
                    print "provide goto location"
                    return False
                if command[1] not in self._get_pose_strings: # todo: change self._locations to be a list of location strings
                    print "invalid location"
                    return False
                return True
            if command[0] == "savegrip":
                # Save the current ar tags so that there's no inconsistency in the tag
                self._current_ar_tags = self._pbd.get_ar_tag_markers()
                self._print_choose_frame_text()
                self._mode = MODE_SELECT_FRAME
            elif command[0] == "goto":
                if not two_args(command):
                    return
                print "moving to location %s" % command[1]
                # todo add a goto thing
                pass
            elif command[0] == "listloc":
                for name in self._get_pose_strings: # todo: change self._locations to be a list of locations
                    print name
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


def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    print json.dumps(COMMANDS)
    rospy.init_node('program_by_demo')

    cli = ActionDemoCli()
    cli.run()


if __name__ == "__main__":
    main()

