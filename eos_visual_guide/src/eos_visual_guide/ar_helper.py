import copy

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers


class ArTags:
    def __init__(self):
        self._ar_tags = []
        self._ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._ar_feedback)

    def get_ar_tags(self):
        return copy.deepcopy(self._ar_tags)

    def _ar_feedback(self, msg):
        self._ar_tags = msg.markers
