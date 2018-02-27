import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import copy
import os
import threading
import pickle

FILE_NAME = "annotator-poses.pickle"

class Annotator(object):
    def __init__(self, data_file=FILE_NAME):
        self.pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback=self._pose_callback)
        self._curr_msg = None
        self._data_file = data_file
        try:
            self._saved_msgs = pickle.load(open(data_file, "rb"))
        except IOError:
            # Create empty file
            self._saved_msgs = {}
            self.write_dump()

    def _pose_callback(self, msg):
        self._curr_msg = msg

    def get_msg(self):
        return copy.deepcopy(self._curr_pose)

    def get_saved_msgs(self):
        return copy.deepcopy(self._saved_msgs)

    def get_saved_loc(self, loc):
        if loc not in self._saved_msgs:
            return None
        return copy.deepcopy(self._saved_msgs[loc])

    def delete_pose(self, pose_name):
        if pose_name not in self._saved_msgs:
            return False
        else:
            del(self._saved_msgs[pose_name])
            self.write_dump()
            return True

    def save_pose(self, pose_name):
        self._saved_msgs[pose_name] = self._curr_msg
        self.write_dump()
        return self._curr_msg

    def update_pose(self, pose_name, pose):
        if pose_name not in self._saved_msgs:
            return False
        self._saved_msgs[pose_name].pose.pose = pose
        self.write_dump()

    def write_dump(self):
        if (self._data_file != None):
            pickle.dump(self._saved_msgs, open(self._data_file, "wb"))

    def go_to(self, pose_name):
        if pose_name not in self._saved_msgs:
            return False
        saved_msg = self._saved_msgs[pose_name]
        new_msg = PoseStamped()
        new_msg.header = saved_msg.header
        new_msg.pose = saved_msg.pose.pose
        self.pub.publish(new_msg)
        return True

