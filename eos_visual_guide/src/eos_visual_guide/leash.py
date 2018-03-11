import rospy
from std_msgs.msg import Int32

LEASH_THRESHOLD = 800


class Leash:
    def __init__(self, move_changed_cb):
        self._fsr_sub = rospy.Subscriber("/fsr", Int32, self._fsr_sub_cb)
        self._in_threshold = False
        self._move_changed_cb = move_changed_cb

    def _fsr_sub_cb(self, msg):
        """
        This is called when the leash's data changes
        :param msg: this contains the data from the analog force sensitive resistor (FSR)
        """
        if msg.data > LEASH_THRESHOLD:
            if not self._in_threshold:
                self._in_threshold = True
                self._move_changed_cb()
        else:  # leash tension below threshold
            if self._in_threshold:  # last time we checked we were above threshold
                self._in_threshold = False
