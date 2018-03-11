import rospy
from fetch_api import Gripper, Arm
from geometry_msgs.msg import PoseStamped
from tf import TransformListener, TransformerROS
from joint_state_reader import JointStateReader

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 1

DEFAULT_FRAME = 'base_link'


class GripperWrapper:
    def __init__(self):
        self._gripper = Gripper()
        self._arm = Arm()
        self._tfl = TransformListener()
        self._tfr = TransformerROS()
        self._joint_states_reader = JointStateReader()

    def get_gripper_pose(self, frame_id):
        """
        Returns a transformed gripper position.
        :param frame_id: ID of the frame
        :return: a tuple of (pose, gripper state)
        """
        pose_st = PoseStamped()
        pose_st.header.frame_id = frame_id
        curr_time = rospy.Time(0)
        # first parameter defines the ORIGIN, second parameter is the FRAME WE WANT TO CONVERT INTO THE NEW ORIGIN
        self._tfl.waitForTransform(frame_id, "wrist_roll_link", curr_time, rospy.Duration(10))
        p, o = self._tfl.lookupTransform(frame_id, "wrist_roll_link", curr_time)
        pose_st.pose.position.x = p[0]
        pose_st.pose.position.y = p[1]
        pose_st.pose.position.z = p[2]
        pose_st.pose.orientation.x = o[0]
        pose_st.pose.orientation.y = o[1]
        pose_st.pose.orientation.z = o[2]
        pose_st.pose.orientation.w = o[3]
        return pose_st, self._get_gripper_state()

    def go_to_gripper_pose(self, saved_pose):
        """
        :param saved_pose: tuple of the form (pose, gripper state)
        :return:
        """
        # transform back into the base frame when running
        pose, grip_state = saved_pose
        now = rospy.Time(0)
        self._tfl.waitForTransform(DEFAULT_FRAME, pose.header.frame_id, now, rospy.Duration(60))
        base_pose = self._tfl.transformPose(DEFAULT_FRAME, pose)
        self._arm.move_to_pose(base_pose)
        if grip_state == GRIPPER_OPEN and self._get_gripper_state() == GRIPPER_CLOSE:
            self._gripper.open()
        elif grip_state == GRIPPER_CLOSE and self._get_gripper_state() == GRIPPER_OPEN:
            self._gripper.close()

    def _get_gripper_state(self):
        if abs(0.05 - self._joint_states_reader.get_joint('l_gripper_finger_joint')) < 0.002:
            return GRIPPER_OPEN
        else:
            return GRIPPER_CLOSE
