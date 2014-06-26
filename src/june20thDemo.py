#!/usr/bin/env python

import math
import random

import rospy

import cv
import cv_bridge

from std_msgs.msg import (
    UInt16,
)
from sensor_msgs.msg import (
    Image,
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

    def macarena(self, reps=0):
        """
        Dances the Macarena
        reps is number of times to repeat. reps=0 does infinite
        """
        #image
        img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Macarena.png")
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        pub.publish(msg)

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
                        #image
                        img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Hey-Macarena.png")
                        msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
                        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
                        pub.publish(msg)
                        rospy.sleep(3.5)
                        #image
                        img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Macarena.png")
                        msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
                        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
                        pub.publish(msg)
            if reps > 0:
                reps -= 1
                if reps == 0:
                    return None


class Arm_Wobbler(object):

    def __init__(self):
        """
        'Wobbles' both arms by commanding joint velocities sinusoidally.
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_joint_names = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()

        # control parameters
        self._rate = 500.0  # Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._right_arm.exit_control_mode()
            self._pub_rate.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        self._right_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def wobble(self):
        self.set_neutral()
        """
        Performs the wobbling of both arms.
        """
        rate = rospy.Rate(self._rate)
        start = rospy.Time.now()

        def make_v_func():
            """
            returns a randomly parameterized cosine function to control a
            specific joint.
            """
            period_factor = random.uniform(0.3, 0.5)
            amplitude_factor = random.uniform(0.1, 0.2)

            def v_func(elapsed):
                w = period_factor * elapsed.to_sec()
                return amplitude_factor * math.cos(w * 2 * math.pi)
            return v_func

        v_funcs = [make_v_func() for _ in self._right_joint_names]

        def make_cmd(joint_names, elapsed):
            return dict([(joint, v_funcs[i](elapsed))
                         for i, joint in enumerate(joint_names)])

        print("Wobbling. Press Ctrl-C to stop...")
        while rospy.get_time() - start.secs < 15:
            self._pub_rate.publish(self._rate)
            elapsed = rospy.Time.now() - start
            cmd = make_cmd(self._left_joint_names, elapsed)
            self._left_arm.set_joint_velocities(cmd)
            cmd = make_cmd(self._right_joint_names, elapsed)
            self._right_arm.set_joint_velocities(cmd)
            rate.sleep()

class Head_Wobbler(object):

    def __init__(self):
        """
        'Wobbles' the head
        """
        self._done = False
        self._head = baxter_interface.Head()

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting example...")
        if self._done:
            self.set_neutral()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def set_neutral(self):
        """
        Sets the head back into a neutral pose
        """
        self._head.set_pan(0.0)

    def wobble(self):
        self.set_neutral()
        """
        Performs the wobbling
        """
        self._head.command_nod()
        command_rate = rospy.Rate(1)
        control_rate = rospy.Rate(100)
        start = rospy.get_time()
        while not rospy.is_shutdown() and (rospy.get_time() - start < 10.0):
            angle = random.uniform(-1.5, 1.5)
            while (not rospy.is_shutdown() and
                   not (abs(self._head.pan() - angle) <=
                       baxter_interface.HEAD_PAN_ANGLE_TOLERANCE)):
                self._head.set_pan(angle, speed=30, timeout=0)
                control_rate.sleep()
            command_rate.sleep()

        self._done = True
        self._head.set_pan(0.0)

class Waver():

    def __init__(self):
        """
        'Waves' right arm by driving the joint velocities to sinusoid functions

        """
        self._pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._right_joint_names = self._right_arm.joint_names()
        self._left_joint_names = self._left_arm.joint_names()
        self._head = baxter_interface.Head()

        # set joint state publishing to 500Hz
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()
        self._pub_rate.publish(500)

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose

        """
        print("Moving to neutral pose...")

        angles_l = dict(zip(
            self._left_joint_names,
            [.3,     0,   0,   1.57,  0.0,  0.0,  0.0]))
        angles_r = dict(zip(
            self._right_joint_names,
            [-.3,  0.0,  3.14,  1.57,  1.57,  0.0,  0.0]))
        
        self._left_arm.move_to_joint_positions(angles_l)
        self._right_arm.move_to_joint_positions(angles_r)
        # self._left_arm.move_to_neutral()
        # self._right_arm.move_to_neutral() 
  
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
    
        self._left_arm.move_to_joint_positions(angles_l,timeout=5.0)
        self._right_arm.move_to_joint_positions(angles_r,timeout=5.0)

    def wave(self):
        self.set_neutral()
        """
        Performs the wobbling of both arms

        """
        rate = rospy.Rate(500);
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

        v_funcs = [make_v_func() for x in range(len(self._right_joint_names))]
        done = False
        while not done and not rospy.is_shutdown():
            self._pub_rate.publish(500)
            elapsed = rospy.Time.now() - start
            cmd = dict(zip(self._right_joint_names, [-v_funcs[i](elapsed) for i in range(len(self._right_joint_names))]))
            cmd['right_s0'] = 0
            cmd['right_s1'] = 0
            cmd['right_e0'] = 0
            cmd['right_w1'] = 0
            self._right_arm.set_joint_velocities(cmd)
            rate.sleep()
            if elapsed.secs > 15:
                self._right_arm.move_to_neutral()
                return True

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

def main():
    """
    Performs the functions for the macarena dancer
    """
    print("Initializing node... ")
    rospy.init_node("macarena_dancer")

    img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Graduation-Congratulations.png")
    msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    pub.publish(msg)

    head_wobbler = Head_Wobbler()
    dancer = Dancer()
    arm_wobbler = Arm_Wobbler()
    waver = Waver()

    while not rospy.is_shutdown():
        head_wobbler.wobble()
        rospy.sleep(30.)
        dancer.macarena(1)
        rospy.sleep(5.)
        pub.publish(msg)
        rospy.sleep(25.)
        arm_wobbler.wobble()
        rospy.sleep(30.)
        dancer.macarena(1)
        rospy.sleep(5.)
        pub.publish(msg)
        rospy.sleep(25.)
        waver.wave()
        rospy.sleep(30.)

    print ("Done.")

if __name__ == '__main__':
    main()
