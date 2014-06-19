#!/usr/bin/env python


import getopt
import os
import sys

import signal

import sys

import roslib
roslib.load_manifest('nxr_baxter')
import rospy

import sensor_msgs.msg

from std_msgs.msg import (
    UInt16,)
from sensor_msgs.msg import (
    JointState,)

import baxter_interface
import baxter_external_devices

import math


class Mover():
    """
    Completes random tasks to test different components of
    Baxter's functionality

    """
    def __init__(self, limb):        
        """
        Initializes left and right arm along with the joint names
        """
        self.limb = limb
        if limb == 'left':
            self.arm = baxter_interface.limb.Limb('left')
        elif limb =='right':
            self.arm = baxter_interface.limb.Limb('right')
        elif limb == 'both':
            self.left_arm = baxter_interface.limb.Limb('left')
            self.right_arm = baxter_interface.limb.Limb('right')

    def move(self):
        """
        Positions Baxter's arms
        """ 
        print "Moving..."
        # Max Joint Range
        #     (  s0,      s1,    e0,     e1,     w0,     w1,      w2)
        #     ( 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059)
        # Min Joint Range
        #     (  s0,     s1,     e0,     e1,      w0,    w1,     w2) 
        #     (-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059)
        l_angles = {                # OBSERVER PERSPECTIVE
            'left_s0':-0.60,      # (+) right      (-) left       (inverted)
            'left_s1': 0.40,      # (+) down       (-) left       (same)
            'left_e0':-1.40,     # (+) CCW        (-) CW         (inverted)
            'left_e1': 0.80,      # (+) more bent  (-) less bent  (same)
            'left_w0': 0.00,      # (+) CCW        (-) CW         (inverted)
            'left_w1': 0.50,      # (+) more bent  (-) less bent  (same)
            'left_w2': 0.00      # (+) CCW        (-) CW         (same)
        }
        r_angles = {                # OBSERVER PERSPECTIVE 
            'right_s0': 0.70,    # (+) right      (-) left        (inverted)
            'right_s1': 0.00,     # (+) down       (-) left        (same)
            'right_e0': 0.60,     # (+) CCW        (-) CW          (inverted)
            'right_e1': 1.70,    # (+) more bent  (-) less bent   (same)
            'right_w0': 0.00,      # (+) CCW        (-) CW          (inverted)
            'right_w1': 0.30,     # (+) more bent  (-) less bent   (same)
            'right_w2': 0.00      # (+) CCW        (-) CW          (same)
        }
        if self.limb == 'left':
            angles = l_angles
            self.arm.move_to_joint_positions(angles)
        elif self.limb =='right':
            angles = r_angles
            self.arm.move_to_joint_positions(angles)
        elif self.limb == 'both':
            self.left_arm.move_to_joint_positions(l_angles)
            self.right_arm.move_to_joint_positions(r_angles)

if __name__ == '__main__':
    print("Welcome.")
    rospy.init_node("rethink_rsdk_joint_velocity")
    #print("Getting Baxter's state... ")
    rs = baxter_interface.RobotEnable()
    print("Attempting to enable Baxter")
    rs.enable()

    task_completer = Mover('both')
    task_completer.move()

    print("Completing tasks. Press Esc to quit.")
    done = False
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
            else:
                print("Press Esc to quit.")

    print("All tasks completed.")
    print("Disabling Baxter... ")
    rs.disable()
    print("done.") 
