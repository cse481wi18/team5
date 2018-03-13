import rospy
from std_msgs.msg import Char
import sys

from .constants import LOCATION, PROGRAM
from .user_input import UserInput, UserCommand, COMMAND_EXECUTE, INTERFACE_APP

# Static locations
LOCATIONS_ARRAY = [
    (1, 'sieg_324'),
    (2, 'sieg_325'),
    (3, 'sieg_326'),
    (4, 'sieg_327'),
    (5, 'sieg_328'),
]
MESSAGE_MAP = {
    key: UserCommand(COMMAND_EXECUTE, {
        "type": LOCATION,
        "name": value
    }) for (key, value) in LOCATIONS_ARRAY
}

# Saving any programs that we want to save
MESSAGE_MAP[6] = UserCommand(COMMAND_EXECUTE, {
    'type': PROGRAM,
    'name': 'demo_2'
})


class MobileApp(UserInput):
    """
    This app has a 1-element bounded buffer for commands; until a command has been processed
    by the controller, any inbound commands are dropped.
    """
    def __init__(self):
        UserInput.__init__(self, INTERFACE_APP)
        self._app_sub = rospy.Subscriber("/eos_mobile_app", Char, self._handle_receive)
        self._current_command = None

    def get_command(self):
        sys.stdout.write("> ")
        sys.stdout.flush()
        while not self._current_command and not rospy.is_shutdown():
            rospy.sleep(rospy.Duration(1))

        # On shutdown
        if not self._current_command:
            return None

        c = MESSAGE_MAP[self._current_command]
        print c.payload

        self._current_command = None
        return c

    def _handle_receive(self, msg):
        if self._current_command is None:
            self._current_command = msg.data
