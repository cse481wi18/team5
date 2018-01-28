#!/usr/bin/env python
        
import rospy
from annotator import Annotator
from map_annotator.msg import PoseNames, UserAction

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class MapAnnotatorServer(object):
    def __init__(self):
        self.poses_pub = rospy.Publisher('map_annotator/pose_names', PoseNames, queue_size=10, latch=True)
        self.user_actions_sub = rospy.Subscriber('map_annotator/user_actions', UserAction, self.handle_user_action)
        self._poses = ["Test 69", "Test 2"] # annotator.get_poses()
        #self._annotator = Annotator()
        self.poses_pub.publish(PoseNames(self._poses))

    def handle_user_action(self, request):
        command = request.command
        name = request.name
        updated_name = request.updated_name
        print "command: " + command + ", name: " + name
        # TODO replace with real code
        if command == "create":
            self._poses.append(name)
        else:
            self._poses.remove(name)

        self.poses_pub.publish(PoseNames(self._poses))


def main():
    rospy.init_node('map_annotator_server')
    wait_for_time()

    print "Starting server"

    server = MapAnnotatorServer()

    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    rospy.spin()


if __name__ == '__main__':
    main()
