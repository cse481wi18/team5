#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
import fetch_api
import rospy
import random
def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

def main():
    rospy.init_node("hallucinated_reach_demo")
    wait_for_time()

    start = PoseStamped()
    start.header.frame_id = "base_link"
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    arm = fetch_api.Arm()
    arm.move_to_pose(start)

    reader = ArTagReader()
    sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, reader.callback)

    while len(reader.markers) == 0:
        rospy.sleep(0.1)

    random.shuffle(reader.markers)
    for marker in reader.markers:
        rospy.loginfo(marker.pose.header.frame_id)
        new_pose = PoseStamped()
        new_pose.header.frame_id = "base_link"
        new_pose.pose.position = marker.pose.pose.position
        error = arm.move_to_pose(new_pose)
        marker_id = marker.id
        if error is None:
            rospy.loginfo("Moved to marker " + str(marker_id))
            return
        else:
            rospy.logwarn("Failed to move to " + str(marker_id))
            rospy.logwarn(error)
    rospy.logerr("Failed to move to any markers!")

if __name__ == "__main__":
    main()
