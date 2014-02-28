#!/usr/bin/env python

# Jon Rovira
# Summer 2013

################
# ROS IMPORTS: #
################
import rospy
import sensor_msgs.msg

####################
# RETHINK IMPORTS: #
####################
import baxter_interface

###############
# NU IMPORTS: #
###############
from baxter_controller import Baxter_Controller

##################
# OTHER IMPORTS: #
##################
import subprocess
import threading
import os
import time
import signal
import roslaunch
import psutil
import cv
import cv_bridge


def terminate_process_and_children(p):
    # subprocess.Popen("pkill -f camera_nodelet_manager")
    p2 = subprocess.Popen("rosnode kill /camera_nodelet_manager", shell=True, stdout=subprocess.PIPE)
    print "sleeping"
    rospy.sleep(2.0)
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    p.terminate()

class Launcher():
    """
    Launcher class

    This class is the overall controller of the Baxter display. In order
    for the display to run indefinitely, the skeleton tracker and trajectory
    controllers need to be killed and restarted occasionally. This launcher
    takes care of this process.
    """

    def __init__(self):
        """
        Every x seconds after all threads have been created,
        every thread is killed and ROS sleeps for y seconds.
        Repeat...
        """
        done = False
        while not done and not rospy.is_shutdown():
            self.rs = baxter_interface.RobotEnable()
            self.left_arm = baxter_interface.limb.Limb('left')
            self.right_arm = baxter_interface.limb.Limb('right')
            self.restart_l_angles = {'left_s0': 0.25, 'left_s1': 0.00, 'left_e0': 0.00, 'left_e1': 1.57, 'left_w0': 0.00, 'left_w1': 0.00, 'left_w2': 0.00}
            self.restart_r_angles = {'right_s0': -0.25, 'right_s1': 0.00, 'right_e0': 0.00, 'right_e1': 1.57, 'right_w0': 0.00, 'right_w1': 0.00, 'right_w2': 0.00}

            self.create_threads()
            rospy.sleep(20 * 60.0)

            # Restarting message on screen
            img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-Restarting.jpg")
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
            pub.publish(msg)

            # Terminate each thread and its children
            terminate_process_and_children(self.bax_controller_process)
            rospy.sleep(10.0)
            self.left_arm.move_to_joint_positions(self.restart_l_angles)
            self.right_arm.move_to_joint_positions(self.restart_r_angles)
            self.rs.disable()
            terminate_process_and_children(self.skel_tracker_process)
            rospy.sleep(20.0)

    def create_threads(self):
        """
        Creates each necessary thread for display
        """
        # Creates each process thread
        self.skel_tracker_thread = threading.Thread(target=self.skel_tracker, name="skel")
        self.bax_controller_thread = threading.Thread(target=self.bax_controller, name="bax")

        # Necessary so that the threads can be killed manually
        self.skel_tracker_thread.daemon = False
        self.bax_controller_thread.daemon = False

        # Commences each thread
        self.skel_tracker_thread.start()
        self.bax_controller_thread.start()

    def skel_tracker(self):
        """
        Runs skeleton tracker and then kills
        """
        #self.asus_cleanup = subprocess.Popen("sudo /home/nxr-baxter/groovyws/src/nxr_baxter/src/usbreset /dev/bus/usb/002/002", shell=True)
        self.skel_tracker_process = subprocess.Popen("roslaunch skeletontracker_nu nu_skeletontracker.launch", stdout=subprocess.PIPE, shell=True)

    def bax_controller(self):
        """
        Runs Baxter controller
        """
        rospy.sleep(10)
        self.bax_controller_process = subprocess.Popen("rosrun nxr_baxter baxter_controller.py", stdout=subprocess.PIPE, shell=True)



if __name__=='__main__':
    print "\nInitializing Launcher node... "
    rospy.init_node('Launcher')
    Launcher()