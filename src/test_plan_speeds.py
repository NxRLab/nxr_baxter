#!/usr/bin/env python

# Adam Barber

# Test the time of a trajectory, then we can change the joint limits and see if it changes.

import rospy

import baxter_interface
import baxter_dataflow

import moveit_commander

import moveit_msgs.msg

import trajectory_msgs.msg


if __name__=='__main__':
    rospy.init_node('planning_test', log_level=rospy.INFO)
    rospy.loginfo("Testing plan speeds for moveit")
    
    self.mime_l_angles = {'left_s0': 0.35, 'left_s1': 0.00, 'left_e0': 0.00, 'left_e1': 0.00, 'left_w0': 0.00, 'left_w1': 0.00, 'left_w2': 0.00}
    self.mime_r_angles = {'right_s0': -0.25, 'right_s1': 0.00, 'right_e0': 0.00, 'right_e1': 0.00, 'right_w0': 0.00, 'right_w1': 0.00, 'right_w2': 0.00}

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    both_arms_group = moveit_commander.MoveGroupCommander("both_arms")

    rospy.loginfo("Get planning time: %d", both_arms_group.get_planning_time())

    both_arms_group.set_joint_value_target(moveit_both_arms_group.set_joint_value_target(dict(crane_r_angles, **crane_l_angles)))

    traj = both_arms_group.plan()
    lastTime = traj.joint_trajectory.points[len(traj.joint_trajectory.points)-1].time_from_start
    rospy.loginfo("Trajectory time is: %d", lastTime)
