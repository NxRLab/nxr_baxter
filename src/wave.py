#!/usr/bin/env python

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import getopt
import os
import sys

import signal
import math
import random

import roslib
roslib.load_manifest('joint_velocity')
roslib.load_manifest('head_control')
import rospy

import cv
import cv_bridge

import sensor_msgs.msg

from std_msgs.msg import (
    UInt16,)
from sensor_msgs.msg import (
    JointState,)
from baxter_msgs.msg import (
    JointVelocities,
    JointCommandMode,)

import baxter_interface
import iodevices

def send_image(path):
        img = cv.LoadImage(path)
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
        pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
        pub.publish(msg)
    # Even with the latch, we seem to need to wait a bit before exiting to
    # make sure that the message got sent.  Using a service may be a better
    # idea.
    # See also: http://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/
        rospy.Rate(1).sleep()


class Waver():

    def __init__(self):
        """
        'Waves' right arm by driving the joint velocities to sinusoid functions

        """
        self._pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._joint_names = ['s0','s1','e0','e1','w0','w1','w2',]
        self._head = baxter_interface.Head()

        # set joint state publishing to 1000Hz
        self._pub_rate.publish(1000)

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose

        """
        print("Moving to neutral pose...")

        angles_l = dict(zip(
            ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'],
            [.3,     0,   0,   1.57,  0.0,  0.0,  0.0]))
        angles_r = dict(zip(
            ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'],
            [-.3,  0.0,  3.14,  1.57,  1.57,  0.0,  0.0]))
        
        self._left_arm.set_joint_positions(angles_l)
        self._right_arm.set_joint_positions(angles_r)
  
    def revert_relax(self):
        """
        Relaxes both arms        

        """
        print("Reverting to relaxed position... ")

        angles_l = dict(zip(
            ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'],
            [.3, 0, 0, 1.57, 0.0, 0.0, 0.0]))
        angles_r = dict(zip(
            ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'],
            [-.3, 0, 0, 1.57, 0.0, 0.0, 0.0]))
    
        self._left_arm.move_to_joint_positions(angles_l)
        self._right_arm.move_to_joint_positions(angles_r)

    def wave(self):
        self.set_neutral()
        """
        Performs the wobbling of both arms

        """
        rate = rospy.Rate(1000);
        start = rospy.Time.now()

        def make_v_func():
            """
            returns a randomly parameterized cos function to control a
            specific joint

            """
            period_factor = random.uniform(0.3, 0.5)
            amplitude_factor = random.uniform(0.2, 0.9)
            def v_func(elapsed):
                return math.cos(period_factor * elapsed.to_sec() * math.pi * 2) * amplitude_factor
            return v_func

        v_funcs = [make_v_func() for x in range(len(self._joint_names))]
        done = False
        print("Waving. Press any key to stop...")
        while not done and not rospy.is_shutdown():
            if iodevices.getch():
                done = True
            else:
                self._pub_rate.publish(1000)
                elapsed = rospy.Time.now() - start
                cmd = dict(zip(self._joint_names, [-v_funcs[i](elapsed) for i in range(len(self._joint_names))]))
                cmd['s0'] = 0
                cmd['s1'] = 0
                cmd['e0'] = 0
                cmd['w1'] = 0
                self._right_arm.set_joint_velocities(cmd)
                rate.sleep()

        rate = rospy.Rate(100);
        if not rospy.is_shutdown():
            for i in range(100):
                if rospy.is_shutdown():
                    return False
                self._left_arm.set_joint_position_mode()
                self._right_arm.set_joint_position_mode()
                self._pub_rate.publish(100)
                rate.sleep()
            #return to normal
            self.revert_relax()
            return True


if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_velocity")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()
    
    # send_image('/home/nxr-baxter/catkin_ws/src/nxr_baxter/images/nxr.jpg')
    waver = Waver()
    waver.wave()

    print("Disabling robot... ")
    rs.disable()
    print("done.")
