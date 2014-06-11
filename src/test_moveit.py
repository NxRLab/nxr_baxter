#!/usr/bin/env python

# Adam Barber
# June 2014

# Node for testing moveit stuff

import rospy
import moveit_commander
import moveit_msgs.msg

if __name__=='__main__':
    rospy.loginfo("Starting test_moveit node")
    rospy.init_node('test_moveit', log_level=rospy.INFO)

    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

    # pose = [0.28, -0.62, -0.32, 3.14/4, -3.14/2, 0]
    pose = [0.815, -1.01, 0.321, 0.271, 0.653, -0.271, 0.653]

    print right_arm_group.get_current_pose()

    right_arm_group.set_pose_target(pose)

    right_arm_group.plan()
