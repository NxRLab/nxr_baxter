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
        #image
        img = cv.LoadImage("../images/Macarena.png")
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        pub.publish(msg)



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
        self._right_arm.set_joint_position_speed(0.8)
        self._left_arm.set_joint_position_speed(0.8)

        num_positions = 6
        right_positions =[[  0.40,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00], #arms out, palms down
                          [  0.40,  0.00,  0.00,  0.00,  2.75,  0.00,  0.00], #arms out, palms up
                          [  0.40,  0.40,  1.90,  2.10,  0.00,  0.50,  0.00], #arms crossed, palms on shoulders
                          [  0.50,  0.40,  2.90,  1.90,  0.00,  1.20,  0.00], #arms uncrossed, palms on head
                          [  0.70,  0.00,  0.60,  1.70,  0.00,  0.30,  0.00], #arms crossed, palms on waist
                          [ -0.60, -0.20,  0.00,  2.00,  0.00,  0.50,  0.00]]

        left_positions = [[ -0.40,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00], #arms out, palms down
                          [ -0.40,  0.00,  0.00,  0.00, -2.75,  0.00,  0.00], #arms out, palms up
                          [ -0.40,  0.40, -1.80,  1.57,  0.00,  0.00,  0.00], #arms crossed, palms on shoulders
                          [ -0.50,  0.40, -2.90,  1.90,  0.00,  1.20,  0.00], #arms uncrossed, palms on head
                          [ -0.60,  0.40, -1.40,  0.80,  0.00,  0.50,  0.00], #arms crossed, palms on waist
                          [  0.60, -0.20,  0.00,  2.00,  0.00,  0.50,  0.00]]

        wiggle_right_positions = [[ -0.60, -0.20,  0.00,  2.00,  0.00,  0.50,  0.00],
                                  [ -0.50, -0.20,  0.00,  2.00,  0.00,  0.50,  0.00]]
        wiggle_left_positions = [[  0.60, -0.20,  0.00,  2.00,  0.00,  0.50,  0.00],
                                 [ -0.50, -0.20,  0.00,  2.00,  0.00,  0.50,  0.00]]

        print("Dancing. Press Ctrl-c to stop...")
        while not rospy.is_shutdown():
            for i in range(num_positions):
                if not rospy.is_shutdown():
                    right_angles = dict(zip(self._right_joint_names, right_positions[i]))
                    left_angles = dict(zip(self._left_joint_names, left_positions[i]))

                    self._right_arm.move_to_joint_positions(right_angles, threshold=0.1)
                    self._left_arm.move_to_joint_positions(left_angles, threshold=0.1)

                    if(i==5):
                        rospy.sleep(3.5)
                    



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