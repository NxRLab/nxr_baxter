#!/usr/bin/env python

# Adam Barber
# April 2014

# Node for interfacing with moveit

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
# import geometry_msg.msg
from nxr_baxter_msgs.srv import *

if __name__=='__main__':
    rospy.loginfo("Starting moveit_interface node")
    rospy.init_node('moveit_interface', log_level=rospy.INFO)

    both_arms_group = moveit_commander.MoveGroupCommander("both_arms")

    while not rospy.is_shutdown():
        rospy.wait_for_service('joint_values')
        try:
            joint_service_proxy = rospy.ServiceProxy('joint_values', JointValues)
            joint_resp = joint_service_proxy()
            if joint_resp.new_values:
                joints = dict(zip(joint_resp.joint_names,
                                  joint_resp.joint_values))
                both_arms_group.set_joint_value_target(joints)
                both_arms_group.go()
            else:
                rospy.logdebug('Got old values')
                rospy.sleep(0.001)
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s', e)

    rospy.loginfo("moveit_interface shutting down")
