#!/usr/bin/env python

import getopt
import os
import sys

import roslib
roslib.load_manifest('nxr_baxter')
import rospy

import cv
import cv_bridge

from std_msgs.msg import (
    UInt16)

import baxter_interface
import iodevices

class Camera_Arm():
    """
    Positions left arm so that camera points at user

    """
    def __init__(self):

        self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        self.left_arm = baxter_interface.limb.Limb("left")
        self.right_arm = baxter_interface.limb.Limb("right")
        self.head = baxter_interface.Head()
        self.left_cam = baxter_interface.CameraController('left_hand_camera')
        self.head_cam = baxter_interface.CameraController('head_camera')

        # Set joint state publishing to 1000 Hz
        self.pub_rate.publish(1000)

    def set_arm(self):
        """
        Positions left arm
	
        """
        print("Positioning left arm. One sec.")
        angles = dict(zip(
            ['left_s0',   'left_s1',   'left_e0',   'left_e1',   'left_w0',   'left_w1',   'w2'],
            [-1.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0]))
        self.left_arm.move_to_joint_positions(angles)

    def prepare_cameras(self):
	"""
	Opens/closes cameras as necessary

	"""
	print("Preparing cameras. Say cheese.")
	self.head_cam.close()
	self.left_cam.close()
	self.left_cam._settings.width = 1280
	self.left_cam._settings.height = 800
	self.left_cam.open()

    def put_it_all_together(self):
        """
        Calls all functions and waits for termination input
        """
	self.set_arm()
	self.prepare_cameras()
	print("Press any key to stop...")
	done = False
	while not done and not rospy.is_shutdown():
	    if iodevices.getch():
		done = True
		self.left_cam.close()

if __name__=='__main__':
    print("Initializing node...")
    rospy.init_node("Baxter_presentation")
    print("Getting robot state...")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot...")
    rs.enable()

    presenter = Camera_Arm()
    presenter.put_it_all_together()

    print("Disabling robot...")
    rs.disable()
    print("done.")
		
		
