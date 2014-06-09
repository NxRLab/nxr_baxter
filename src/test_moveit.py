#!/usr/bin/env python

# Adam Barber
# June 2014

# Node for testing moveit stuff

import rospy
import moveit_commander
import moveit_msgs.msg
import subprocess

from geometry_msgs.msg import PoseStamped


if __name__=='__main__':
    rospy.loginfo("Starting joint trajectory action server")
    cmd = 'rosrun baxter_interface joint_trajectory_action_server.py'
    subprocess.Popen(cmd,shell=True)
    rospy.loginfo("Starting test_moveit node")
    rospy.init_node('test_moveit', log_level=rospy.INFO)

    both_arms_group = moveit_commander.MoveGroupCommander("both_arms")
    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
    left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
    both_arms_group.allow_replanning(True)
    left_arm_group.allow_replanning(True)
    right_arm_group.allow_replanning(True)

    both_arms_group.set_goal_position_tolerance(0.01)
    both_arms_group.set_goal_orientation_tolerance(0.01)
    left_arm_group.set_goal_position_tolerance(0.01)
    left_arm_group.set_goal_orientation_tolerance(0.01)
    right_arm_group.set_goal_position_tolerance(0.01)
    right_arm_group.set_goal_orientation_tolerance(0.01)

    #Try setting workspace bounds, instead of maybe checking joint limits.
    both_arms_group.set_workspace([-10, -51*2.54/100.0, -10, 10, 53*2.54/100.0, 10])
    right_arm_group.set_workspace([-10, -51*2.54/100.0, -10, 10, 53*2.54/100.0, 10])
    left_arm_group.set_workspace([-10, -51*2.54/100.0, -10, 10, 53*2.54/100.0, 10])

    scene = moveit_commander.PlanningSceneInterface()

    rospy.loginfo('Sitting here.')
    rospy.sleep(10.0)

    robot = moveit_commander.RobotCommander()

    both_arms_group.set_pose_reference_frame('base')
    left_arm_group.set_pose_reference_frame('base')
    right_arm_group.set_pose_reference_frame('base')

    # Add in objects
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 1.0
    p.pose.position.y = 0.025
    p.pose.position.z = -0.6
    # scene.add_box("table", p, (0.8, 1.25, 0.8)) # Bigger box

    # scene.add_box("table", p, (0.75, 1.25, 0.68)) # Good box, but check this later

    pose = [0.805, -1.02, 0.318, 0.276, 0.649, -0.27, 0.656]

    right_arm_group.set_pose_target(pose)

    right_arm_group.plan()
