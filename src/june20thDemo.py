#!/usr/bin/env python

import math
import random

import rospy

from std_msgs.msg import (
    UInt16,
)

import baxter_interface
from baxter_interface import CHECK_VERSION


class Dancer(object):

    def __init__(self):
        """
        Dances the macarena!
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_joint_names = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()

        #control parameters
        self._rate = 500.0 #Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        #set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

    def _reset_control_modes(self):
        """
        Resets RJ's control mode
        """
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._right_arm.exit_control_mode()
            self._pub_rate.publish(100) #100Hz default joint state rate
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose
        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        self._right_arm.move_to_neutral()

    def clean_shutdown(self):
        """
        Disables RJ's motors safely
        """
        print("\nExiting example...")
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def macarena(self):
        """
        Dances the Macarena
        """
        self.set_neutral()

        num_positions = 3
        right_times = [1, 2, 3, 4]
        left_times = [1, 2, 3, 4]
        right_positions = [[ 0.60,  0.00,  0.00,  0.00,  2.75,  0.00,  0.00], 
                          [  0.60,  0.40,  2.90,  2.10,  0.00,  1.20,  0.00],
                          [ -0.60, -0.20,  0.00,  2.00,  0.00,  0.50,  0.00]]
        left_positions = [[ -0.60,  0.00,  0.00,  0.00, -2.75,  0.00,  0.00], 
                          [ -0.60,  0.40, -2.90,  2.10,  0.00,  1.20,  0.00],
                          [  0.60, -0.20,  0.00,  2.00,  0.00,  0.50,  0.00]]

        print("Dancing. Press Ctrl-c to stop...")
        for i in range(num_positions):
            while not rospy.is_shutdown():
                right_angles = dict(zip(self._right_joint_names, right_positions[i]))
                left_angles = dict(zip(self._left_joint_names, left_positions[i]))

                self._right_arm.move_to_joint_positions(right_angles)
                rospy.sleep(right_times[i])

                self._left_arm.move_to_joint_positions(left_angles)
                rospy.sleep(left_times[i])



def main():
    """
    Performs the functions for the macarena dancer
    """
    print("Initializing node... ")
    rospy.init_node("macarena_dancer")

    dancer = Dancer()
    rospy.on_shutdown(dancer.clean_shutdown)
    dancer.macarena()

    print ("Done.")

if __name__ == '__main__':
    main()