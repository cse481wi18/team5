from .constants import GRIPPER, PROGRAM
from .user_input import COMMAND_SAVE, COMMAND_EXECUTE, COMMAND_EXIT, COMMAND_LIST, COMMAND_SHOW
from .cli import Cli
from .saver import Saver
from .executor import Executor
from .leash import Leash
from .ar_helper import ArTags


class EosController:
    def __init__(self):
        self._ui = Cli()
        self._saver = Saver()
        self._executor = Executor(self._saver)
        self._leash = Leash(self._leash_pulled_cb)
        self._ar = ArTags()

    def _leash_pulled_cb(self):
        """
        Pass the move state to our executor to cancel or whatever
        :param new_move_state:
        :return:
        """
        self._executor.leash_pulled()

    def run(self):
        while True:
            c = self._ui.get_command()

            if c.command == COMMAND_EXIT:
                return
            elif c.command == COMMAND_SAVE:
                # Here we have special behavior for gripper saving
                if c.payload["type"] == GRIPPER:
                    # Get frame ID
                    c.payload["frame_id"] = self._ui.get_frame_id(self._ar.get_ar_tags())
                elif c.payload["type"] == PROGRAM:
                    # Get all the grippers and locations we want
                    c.payload["actions"] = self._ui.get_actions(self._saver.get_saved_location_names(),
                                                                      self._saver.get_saved_gripper_names())
                self._saver.run_command(c)
            elif c.command == COMMAND_LIST:
                self._saver.run_command(c)
            elif c.command == COMMAND_SHOW:
                self._saver.run_command(c)
            elif c.command == COMMAND_EXECUTE:
                self._executor.run_command(c)
