COMMAND_SAVE = "save"
COMMAND_EXECUTE = "execute"
COMMAND_LIST = "list"
COMMAND_SHOW = "show"
COMMAND_EXIT = "exit"
COMMAND_ARM = "arm"

INTERFACE_CLI = "cli"
INTERFACE_APP = "app"


class UserInput:
    def __init__(self):
        pass

    def get_command(self):
        raise NotImplementedError

    def get_frame_id(self, ar_tags):
        raise NotImplementedError

    def get_actions(self, locations, grippers):
        raise NotImplementedError


class UserCommand:
    def __init__(self, command, payload=None):
        self.command = command
        self.payload = payload
