import rospy

from .constants import GRIPPER, LOCATION, PROGRAM
from .arm_wrapper import ArmWrapper, RELAX, STIFF
from .gripper_wrapper import GripperWrapper
from .navigator import Navigator
from .user_input import COMMAND_EXECUTE, COMMAND_ARM


class Executor:
    def __init__(self, saver):
        self._arm_wrapper = ArmWrapper()
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
        if c.command == COMMAND_EXECUTE:
            if c.payload["type"] == GRIPPER:
                self._run_gripper(c.payload["name"])
            elif c.payload["type"] == LOCATION:
                self._run_location(c.payload["name"])
            elif c.payload["type"] == PROGRAM:
                actions = self._saver.get_saved_program(c.payload["name"])
                for action in actions:
                    print action
                    rospy.sleep(rospy.Duration(2))
                    if action[0] == GRIPPER:
                        self._run_gripper(action[1])
                    elif action[0] == LOCATION:
                        self._run_location(action[1])
                    else:
                        raise RuntimeError("Invalid program")
        elif c.command == COMMAND_ARM:
            if c.payload["type"] == RELAX:
                self._arm_wrapper.arm_relax()
            elif c.payload["type"] == STIFF:
                self._arm_wrapper.arm_stiff()
        else:
            raise RuntimeError("Unsupported command")

    def leash_pulled(self):
        self._navigator.leash_pulled()
