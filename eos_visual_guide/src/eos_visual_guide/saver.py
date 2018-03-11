import pickle

from .gripper_wrapper import GripperWrapper
from map_annotator import Annotator
from .constants import GRIPPER, LOCATION, PROGRAM

SAVE = "save"
SHOW = "show"
LIST = "list"

GRIPPERS_FILE = "grippers.pickle"
PROGRAMS_FILE = "programs.pickle"



class Saver:
    def __init__(self):
        try:
            self._grippers = pickle.load(open(GRIPPERS_FILE, 'rb'))
        except IOError:
            self._grippers = {}

        try:
            self._programs = pickle.load(open(PROGRAMS_FILE, 'rb'))
        except IOError:
            self._programs = {}

        self._gripper_wrapper = GripperWrapper()
        self._annotator = Annotator()

    def run_command(self, c):
        """
        Run a save command. Depending on the type of the save action,
        save to gripper poses or locations.
        """
        if c.command == SAVE:
            if c.payload["type"] == GRIPPER:
                self._grippers[c.payload["name"]] = self._gripper_wrapper.get_gripper_pose(
                    c.payload["frame_id"])
                print "Saved grippers"
                pickle.dump(self._grippers, open(GRIPPERS_FILE, 'wb'))
            elif c.payload["type"] == LOCATION:
                self._annotator.save_pose(c.payload["name"])
            elif c.payload["type"] == PROGRAM:
                self._programs[c.payload["name"]] = c.payload["actions"]
                print "Saved programs"
                pickle.dump(self._programs, open(PROGRAMS_FILE, 'wb'))
        elif c.command == LIST:
            print "Gripper poses: "
            print self.get_saved_gripper_names()
            print "Locations: "
            print self.get_saved_location_names()
            print "Programs: "
            print self.get_saved_program_names()
        elif c.command == SHOW:
            if c.payload["type"] == GRIPPER:
                print self.get_saved_gripper(c.payload["name"])
            if c.payload["type"] == LOCATION:
                print self.get_saved_location(c.payload["name"])
            if c.payload["type"] == PROGRAM:
                print self.get_saved_program(c.payload["name"])
        else:
            raise RuntimeError("Unsupported save command")

    def get_saved_gripper(self, name):
        return self._grippers[name]

    def get_saved_location(self, name):
        return self._annotator.get_pose(name)

    def get_saved_program(self, name):
        return self._programs[name]

    def get_saved_location_names(self):
        return self._annotator.get_saved_msgs().keys()

    def get_saved_gripper_names(self):
        return self._grippers.keys()

    def get_saved_program_names(self):
        return self._programs.keys()
