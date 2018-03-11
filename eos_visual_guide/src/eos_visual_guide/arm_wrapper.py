import actionlib
from robot_controllers_msgs.msg import ControllerState, QueryControllerStatesGoal, QueryControllerStatesAction

RELAX = "relax"
STIFF = "stiff"


class ArmWrapper:
    def __init__(self, is_sim=False):
        self._controller_client = actionlib.SimpleActionClient('query_controller_states', QueryControllerStatesAction)
        self._is_sim = is_sim

    def _send_arm_goal(self, controller_state):
        """
        Helper function for sending an arm goal
        :param state: RUNNING or STOPPED
        """
        if self._is_sim:
            return

        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = controller_state
        goal.updates.append(state)

        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    def arm_relax(self):
        self._send_arm_goal(ControllerState.STOPPED)

    def arm_stiff(self):
        self._send_arm_goal(ControllerState.RUNNING)
