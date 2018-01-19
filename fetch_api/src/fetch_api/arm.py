import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy

from .arm_joints import ArmJoints

ACTION_NAME = "/arm_controller/follow_joint_trajectory"

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        self.client.wait_for_server()
        #pass

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        point = JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        point.positions = arm_joints.values()
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(5.0)
        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()
        trajectory.joint_names = arm_joints.names()
        trajectory.points = [point]
        goal.trajectory = trajectory
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        # TODO: Send goal
        # TODO: Wait for result
        #rospy.logerr('Not implemented.')
