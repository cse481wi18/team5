import sys
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from fetch_api import Head

MAX_TRIES = 3


class Navigator:
    def __init__(self):
        self._should_move = False
        self._curr_goal = None
        self._active = False
        self._nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # TODO remove this?
        # self._nav_client.wait_for_result()
        self._head = Head()

    def go_to(self, loc_msg):
        """
        Go to a particular location given a pose message. Blocks on this call.
        """
        goal = MoveBaseGoal()
        new_msg = PoseStamped()
        new_msg.header = loc_msg.header
        new_msg.pose = loc_msg.pose.pose
        goal.target_pose = new_msg

        self._should_move = True
        self._curr_goal = goal
        self._active = True
        self._nav_client.send_goal(goal)

        print "Navigating..."

        tries = 0

        # This means we only check 1 time every second
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            state = self._nav_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                break
            elif state == actionlib.GoalStatus.REJECTED:
                tries += 1
                print "State was updated to REJECTED"
                if tries < MAX_TRIES:
                    self._nav_client.send_goal(goal)
                else:
                    print "Ran out of tries :("
            elif state == actionlib.GoalStatus.ABORTED:
                tries += 1
                print "State was updated to ABORTED"
                if tries < MAX_TRIES:
                    self._nav_client.send_goal(goal)
                else:
                    print "Ran out of tries :("
            r.sleep()
        print "done!"

        sys.stdout.write("Waiting for navigation to wrap up...")
        rospy.sleep(rospy.Duration(5))
        print "done!"

        sys.stdout.write("Setting head pan/tilt...")
        self._head.pan_tilt(0, 0)
        print "done!"

        self._active = False
        self._curr_goal = None
        return True

    def leash_pulled(self):
        if self._active:
            self._should_move = not self._should_move
            if self._should_move:
                self._nav_client.send_goal(self._curr_goal)
            else:
                self._nav_client.cancel_all_goals()
