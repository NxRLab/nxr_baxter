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

def skeletonCallback(self, data):
    for skeleton in data.skeletons:
        print "skel :" skel.userid
        print "x: ", skel.torso.transform.translation.x
        print "y: ", skel.torso.transform.translation.y
        print "z: ", skel.torso.transform.translation.z


if __name__=="__main__":
    rospy.loginfo("Starting test_skel_distance")
    rospy.loginfo("Did you start openni launch and skeleton tracker?")

    rospy.init_node('test_skel_distance', log_level=rospy.INFO)

    rospy.Subscriber("skeletons", Skeletons, skeletonCallback)

    rospy.spin()
