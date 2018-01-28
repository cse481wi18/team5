#!/usr/bin/env python
        
import rospy
from annotator import Annotator
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import PoseNames, UserAction

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class MapAnnotatorServer(object):
    def __init__(self):
        self._pose_names_pub = rospy.Publisher('map_annotator/pose_names', PoseNames, queue_size=10, latch=True)
        self._user_actions_sub = rospy.Subscriber('map_annotator/user_actions', UserAction, self.handle_user_action)
        self._interactive_marker_server = InteractiveMarkerServer("map_annotator/map_poses")

        # TODO read initial poses from Annotator
        self._annotator = Annotator("db_file")
        self.publish_names()

    def publish_names(self):
        self._pose_names_pub.publish(PoseNames(self._annotator.get_saved_msgs().keys()))

    def handle_viz_input(self, input):
        if (input.event_type == InteractiveMarkerFeedback.POSE_UPDATE):
            self._annotator.update_pose(input.marker_name, input.pose)
        else:
            rospy.loginfo('Cannot handle this InteractiveMarker event')

    def handle_user_action(self, request):
        if request.command == "create":
            self.handle_create(request)
        elif request.command == "delete":
            self.handle_delete(request)
        else:
            self.handle_goto(request)

    def handle_create(self, request):
        self.create_marker(request, self._annotator.save_pose(request.name))
        self.publish_names()

    def create_marker(self, request, pose):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = request.name
        int_marker.description = request.name
        int_marker.pose = pose.pose.pose

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        # TODO this can be a lot cleaner
        arrow_marker.scale.x = 1
        arrow_marker.scale.y = 0.15
        arrow_marker.scale.z = 0.15
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        # Control #1
        move_control = InteractiveMarkerControl()
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.orientation.w = 1
        move_control.orientation.x = 0
        move_control.orientation.y = 1
        move_control.orientation.z = 0
        move_control.always_visible = True
        move_control.markers.append(arrow_marker)
        int_marker.controls.append(move_control)

        # TODO add another control
        ring_marker = Marker()
        ring_marker.type = Marker.CYLINDER
        ring_marker.pose.orientation.w = 1
        ring_marker.scale.x = 1
        ring_marker.scale.y = 1
        ring_marker.scale.z = 0.01
        ring_marker.color.r = 0.5
        ring_marker.color.g = 0.5
        ring_marker.color.b = 0.0
        ring_marker.color.a = 1.0

        rotate_control = InteractiveMarkerControl()
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 1
        rotate_control.orientation.z = 0
        rotate_control.always_visible = True
        rotate_control.markers.append(ring_marker)
        int_marker.controls.append(rotate_control)

        self._interactive_marker_server.insert(int_marker, self.handle_viz_input)
        self._interactive_marker_server.applyChanges()

    def handle_delete(self, request):
        self._annotator.delete_pose(request.name)
        self._interactive_marker_server.erase(request.name)
        self._interactive_marker_server.applyChanges()
        self.publish_names()

    def handle_goto(self, request):
        self._annotator.go_to(request.name)


def main():
    rospy.init_node('map_annotator_server')
    wait_for_time()

    server = MapAnnotatorServer()

    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    rospy.spin()


if __name__ == '__main__':
    main()
