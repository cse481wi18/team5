#! /usr/bin/env python

import fetch_api
import rospy
from moveit_python import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import OrientationConstraint

RUN_WITH_OC_CONSTRAINT = True # RUN with orientation constraint if True else Don't

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_obstacle_demo')
    wait_for_time()

    planning_scene = PlanningSceneInterface('base_link')
    # Create table obstacle
    planning_scene.removeCollisionObject('table')
    table_size_x = 0.5
    table_size_y = 1
    table_size_z = 0.03
    table_x = 0.8
    table_y = 0
    table_z = 0.4
    if (RUN_WITH_OC_CONSTRAINT):
        table_z = 0.6   
        
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                          table_x, table_y, table_z)

    # Create divider obstacle
    if (not RUN_WITH_OC_CONSTRAINT):
        planning_scene.removeCollisionObject('divider')
        size_x = 0.3 
        size_y = 0.01
        size_z = 0.4 
        x = table_x - (table_size_x / 2) + (size_x / 2)
        y = 0 
        z = table_z + (table_size_z / 2) + (size_z / 2)
        planning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)

    pose1 = PoseStamped()
    pose1.header.frame_id = 'base_link'
    pose1.pose.position.x = 0.6 # 0.7 is pretty reliable
    if (RUN_WITH_OC_CONSTRAINT):
        pose1.pose.position.x = 0.5
    pose1.pose.position.y = -0.3
    pose1.pose.position.z = 0.75
    pose1.pose.orientation.w = 1

    pose2 = PoseStamped()
    pose2.header.frame_id = 'base_link'
    pose2.pose.position.x = 0.6 # 0.7 is pretty reliable
    if (RUN_WITH_OC_CONSTRAINT):
        pose2.pose.position.x = 0.5
    pose2.pose.position.y = 0.3
    pose2.pose.position.z = 0.75
    pose2.pose.orientation.w = 1

    arm = fetch_api.Arm()

    oc = OrientationConstraint()
    oc.header.frame_id = 'base_link'
    oc.link_name = 'wrist_roll_link'
    oc.orientation.w = 1
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = 3.14
    oc.weight = 1.0

    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    kwargs = {
        'allowed_planning_time': 15,
        'execution_timeout': 10,
        'num_planning_attempts': 5,
        'replan': False,
    }

    if (RUN_WITH_OC_CONSTRAINT):
        kwargs['orientation_constraint'] = oc
    else:
        planning_scene.removeAttachedObject('tray')

    error = arm.move_to_pose(pose1, **kwargs)
    if error is not None:
        rospy.logerr('Pose 1 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 1 succeeded')
        if (not RUN_WITH_OC_CONSTRAINT):
            frame_attached_to = 'gripper_link'
            frames_okay_to_collide_with = [
                'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
            ]
            planning_scene.attachBox('tray', 0.3, 0.07, 0.01, 0.05, 0, 0,
                                     frame_attached_to, frames_okay_to_collide_with)
            planning_scene.setColor('tray', 1, 0, 1)
            planning_scene.sendColors()
    rospy.sleep(1)
    error = arm.move_to_pose(pose2, **kwargs)
    if error is not None:
        rospy.logerr('Pose 2 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 2 succeeded')

    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('divider')
    if (not RUN_WITH_OC_CONSTRAINT):
        planning_scene.removeAttachedObject('tray')

    

if __name__ == '__main__':
    main()
