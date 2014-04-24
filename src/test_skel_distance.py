#!/usr/bin/env python

# Adam Barber
# April 2014

# This script is to figure out how far the skeletons are from the middle of the
# Asus frame and that way we can start "clipping" the frame. This means we don't
# have to worry about people leaving the frame, hopefully.


# ROS Imports
import rospy

# Other imports
import os
import sys
import subprocess

# NU imports
from skeletonmsgs_nu.msg import Skeletons
from skeletonmsgs_nu.msg import Skeleton
from skeletonmsgs_nu.msg import SkeletonJoint

def skeletonCallback(data):
    for skel in data.skeletons:
        # print "skel :", skel.userid
        x = skel.torso.transform.translation.x
        y = skel.torso.transform.translation.y
        z = skel.torso.transform.translation.z
        if x > skeletonCallback.max_x:
            skeletonCallback.max_x = x
            rospy.loginfo("Max x: %f", skeletonCallback.max_x)
        if x < skeletonCallback.min_x:
            skeletonCallback.min_x = x
            rospy.loginfo("Min x: %f", skeletonCallback.min_x)
        if y > skeletonCallback.max_y:
            skeletonCallback.max_y = y
            rospy.loginfo("Max y: %f", skeletonCallback.max_y)
        if y < skeletonCallback.min_y:
            skeletonCallback.min_y = y
            rospy.loginfo("Min y: %f", skeletonCallback.min_y)
        if z > skeletonCallback.max_z:
            skeletonCallback.max_z = z
            rospy.loginfo("Max z: %f", skeletonCallback.max_z)
        if z < skeletonCallback.min_z:
            skeletonCallback.min_z = z
            rospy.loginfo("Min z: %f", skeletonCallback.min_z)


if __name__=="__main__":
    rospy.loginfo("Starting test_skel_distance")
    rospy.loginfo("Did you start openni launch and skeleton tracker?")

    rospy.init_node('test_skel_distance', log_level=rospy.INFO)

    skeletonCallback.max_x = 0.0
    skeletonCallback.min_x = 0.0
    skeletonCallback.max_y = 0.0
    skeletonCallback.min_y = 0.0
    skeletonCallback.max_z = 0.0
    skeletonCallback.min_z = 0.0

    rospy.Subscriber("skeletons", Skeletons, skeletonCallback)

    rospy.spin()
    print "Max x: ", skeletonCallback.max_x
    print "Min x: ", skeletonCallback.min_x
    print "Max y: ", skeletonCallback.max_y
    print "Min y: ", skeletonCallback.min_y
    print "Max z: ", skeletonCallback.max_z
    print "Min z: ", skeletonCallback.min_z

    
