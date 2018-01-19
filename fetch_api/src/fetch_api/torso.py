#!/usr/bin/env python

# TODO: import ?????????
import actionlib
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
ACTION_NAME = "/torso_controller/follow_joint_trajectory"
JOINT_NAME = "torso_lift_joint"
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
	self.client  = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
	self.client.wait_for_server()
        #pass

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
	if (height >= self.MIN_HEIGHT and height <= self.MAX_HEIGHT):
        	# TODO: Create a trajectory point
		point = JointTrajectoryPoint()
        	# TODO: Set position of trajectory point
		endPoint = []
		endPoint.append(height)
		point.positions = endPoint
        	# TODO: Set time of trajectory point
		point.time_from_start = rospy.Duration(5.0)

        	# TODO: Create goal
		trajectory = JointTrajectory()
        	# TODO: Add joint name to list
		joints = []
		joints.append(JOINT_NAME)
		trajectory.joint_names = joints
        	# TODO: Add the trajectory point created above to trajectory
		trajectory.points = [point]
        	# TODO: Send goal
                goal = FollowJointTrajectoryGoal()
		goal.trajectory = trajectory
		self.client.send_goal(goal)
		self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        	# TODO: Wait for result
        #rospy.logerr('Not implemented.')
