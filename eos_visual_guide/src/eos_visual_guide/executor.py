from .constants import GRIPPER, LOCATION, PROGRAM
from .gripper_wrapper import GripperWrapper
from .navigator import Navigator

class Executor:
    def __init__(self, saver):
        self._gripper_wrapper = GripperWrapper()
        self._navigator = Navigator()
        self._saver = saver

    def _run_gripper(self, name):
        self._gripper_wrapper.go_to_gripper_pose(self._saver.get_saved_gripper(name))

    def _run_location(self, name):
        self._navigator.go_to(self._saver.get_saved_location(name))

    def run_command(self, c):
        """
        The "data" field in the payload has been filled with the information
        saved in the Saver.
        :param c:
        :return:
        """
        if c.payload["type"] == GRIPPER:
            self._run_gripper(c.payload["name"])
        elif c.payload["type"] == LOCATION:
            self._run_location(c.payload["name"])
        elif c.payload["type"] == PROGRAM:
            actions = self._saver.get_saved_program(c.payload["name"])
            for action in actions:
                print action
                if action[0] == GRIPPER:
                    self._run_gripper(action[1])
                elif action[0] == LOCATION:
                    self._run_location(action[1])
                else:
                    raise RuntimeError("Invalid program")
        else:
            raise RuntimeError("Unsupported save command")

    def leash_pulled(self):
        self._navigator.leash_pulled()
