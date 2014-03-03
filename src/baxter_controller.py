#!/usr/bin/env python

# Jon Rovira & Kyle Lueptow
# Summer 2013
# Adam Barber edits March 2014

################
# ROS IMPORTS: #
################
import rospy
import sensor_msgs.msg

####################
# RETHINK IMPORTS: #
####################
import baxter_interface
import baxter_dataflow

###############
# NU IMPORTS: #
###############
from skeletonmsgs_nu.msg import Skeletons
from skeletonmsgs_nu.msg import Skeleton
from skeletonmsgs_nu.msg import SkeletonJoint
from mimic import Mime
from crane import Crane
from vector_operations import (make_vector_from_POINTS,
                               angle_between_vectors,
                               vector_projection_onto_plane,
                               shortest_vector_from_point_to_vector)
import skeleton_filter as sf

####################
# OTHER IMPORTS: #
####################
import os
import traceback
import threading
import Queue
import math
import cv
import cv_bridge
import numpy as np


DOWN_SAMPLE = 5

class Baxter_Controller:
    """
    Baxter Controller

    This class functions as the macro controller of Baxter's actions during 
    Natural Interaction with people. A primer user is chosen, the user chooses
    an action by gesturing, and can command Baxter to switch to a different
    application at any time. While not completing any action or tracking 
    people, Baxter performs an idle function.

    In conjunction to running the Baxter Controller, the NU SkeletonTracker
    and the Trajectory Controller must also be running. To do so:

        Different terminal:
        roslaunch skeletontracker_nu nu_skeletontracker.launch

        Another different terminal:
        rosrun baxter_interface trajectory_controller.py

        Should be handled in launch file, no?
    """

    def __init__(self):
        """
        Enables robot, initializes booleans, subscribes to skeletons
        """
        print "Getting robot state..."
        self.rs = baxter_interface.RobotEnable() #RS is a wrapper for the robot state
        print "Enabling robot... "
        self.rs.enable()

        self.left_arm = baxter_interface.limb.Limb('left')
        self.right_arm = baxter_interface.limb.Limb('right')
        self.mime_l_angles = {'left_s0': 0.35, 'left_s1': 0.00, 'left_e0': 0.00, 'left_e1': 0.00, 'left_w0': 0.00, 'left_w1': 0.00, 'left_w2': 0.00}
        self.mime_r_angles = {'right_s0': -0.25, 'right_s1': 0.00, 'right_e0': 0.00, 'right_e1': 0.00, 'right_w0': 0.00, 'right_w1': 0.00, 'right_w2': 0.00}
        self.crane_l_angles = {'left_s0': 0.35, 'left_s1': 0.00, 'left_e0': 0.00, 'left_e1': 1.57, 'left_w0': 0.00, 'left_w1': 0.00, 'left_w2': 0.00}
        self.crane_r_angles = {'right_s0': -0.25, 'right_s1': 0.00, 'right_e0': 0.00, 'right_e1': 0.00, 'right_w0': 0.00, 'right_w1': 0.00, 'right_w2': 0.00}

        # Booleans used throughout controller
        self.userid_almost_chosen = False
        self.userid_chosen = False
        self.user_positioned = False
        self.action_chosen = False
        self.action_id = 0
        self.display_top = True
        self.display_mime_prep = True
        self.display_crane_prep = True
        self.display_mime = True
        self.display_crane = True

        # self.l_sh_c = 0
        # self.l_el_c = 0
        # self.l_ha_c = 0
        # self.r_sh_c = 0
        # self.r_el_c = 0
        # self.r_ha_c = 0

        # Screen images
        # Let's make a new class that handles image changing, inside it will have two timers, one that runs at ~1/3hz to switch between images or whatever and one that runs more frequently to check which mode it should be in.
        # Can also have it read image names from a file or param server instead of hardcoded
        # Seems like it would make more sense and might fix the weird issues we've had with it
        # Need to make sure we "init node" which it seems like we do
        img_thread = threading.Thread(target=self.img_change,
                                      name='Top',
                                      args=(['top']))
        img_thread.daemon = True
        img_thread.start()

        # skeletonCallback called whenever skeleton received
        rospy.Subscriber("skeletons", Skeletons, self.skeletonCallback)
        # instantiate a skeleton filter
        self.skel_filt = sf.SkeletonFilter(sf.joints)
        self.first_filt_flag = True

    def img_change(self, type):
        """
        Changes images on head screen
        """
        # Declare pub at the top
        # I want to rewrite this anyway
        if(type == 'top'):
            img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-Top.png")
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
            pub.publish(msg)
        elif(type == 'mime_prep'):
            for x in range(1, 3):
                if(self.display_mime_prep == True):
                    img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-Mime-Prep-" + str(x) + ".png")
                    msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
                    pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
                    pub.publish(msg)
                    rospy.sleep(3.0)
        elif(type == 'crane_prep'):
            for x in range(1, 3):
                if(self.display_crane_prep == True):
                    img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-Crane-Prep-" + str(x) + ".png")
                    msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
                    pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
                    pub.publish(msg)
                    rospy.sleep(3.0)
        elif(type == 'mime'):
            img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-Mime.png")
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
            pub.publish(msg)
            rospy.sleep(3.0)
        elif(type == 'crane'):
            img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-Crane.png")
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
            pub.publish(msg)
            rospy.sleep(3.0)

    def choose_user(self, skeletons):
        """
        Selects primary user to avoid ambiguity
        """
        for skeleton in skeletons:
            lh = skeleton.left_hand.transform.translation.y
            rh = skeleton.right_hand.transform.translation.y
            h = skeleton.head.transform.translation.y
            if h - lh > 0.11 or h - rh > 0.11: # What units are these? should we make this higher or fix the filtering?
                self.userid_almost_chosen = True
                self.main_userid = skeleton.userid
                print "\n\nMain user chosen.\nUser %s, please proceed.\n" % str(self.main_userid)
                self.user_starting_position = skeleton.torso.transform.translation
                if h - lh > 0.11:
                    # Prep user images, will need to change these also
                    img_thread = threading.Thread(target=self.img_change,
                                                   name="Crane_Prep",
                                                   args=(['crane_prep']))
                    img_thread.daemon = True
                    img_thread.start()
                    #Let's try and speed this stuff up
                    self.left_arm.move_to_joint_positions(self.crane_l_angles)
                    self.right_arm.move_to_joint_positions(self.crane_r_angles)
                    self.userid_chosen = True
                    return 'left'
                elif h - rh > 0.11:
                    # Prep user images
                    img_thread = threading.Thread(target=self.img_change,
                                                   name="Mime_Prep",
                                                   args=(['mime_prep']))
                    img_thread.daemon = True
                    img_thread.start()
                    self.left_arm.move_to_joint_positions(self.mime_l_angles)
                    self.right_arm.move_to_joint_positions(self.mime_r_angles)
                    self.userid_chosen = True
                    return 'right'

    def position_user(self, skeleton, choice):
        lh_y = skeleton.left_hand.transform.translation.y
        lh_x = skeleton.left_hand.transform.translation.x
        rh_y = skeleton.right_hand.transform.translation.y
        rh_x = skeleton.right_hand.transform.translation.y
        tor_y = skeleton.torso.transform.translation.y
        tor_x = skeleton.torso.transform.translation.x
        if (choice == 'left'):
            dy = math.fabs(tor_y - lh_y)
            dx = math.fabs(lh_x - tor_x)
            #These tolerances have been causing problems
            if(dy < 0.08 and dx > 0.4):
                self.display_crane_prep = False # will be redone with new image switcher class
                img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-User-Positioned.png")
                msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
                pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
                pub.publish(msg)
                rospy.sleep(2.0)
                return True
        elif (choice == 'right'):
            dy = math.fabs(tor_y - lh_y) + math.fabs(tor_y - rh_y)
            dx = math.fabs(lh_x - tor_x) + math.fabs(rh_x - tor_x)
            if(dy < 0.20 and dx > 0.8):
                self.display_mime_prep = False
                img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-User-Positioned.png")
                msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
                pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
                pub.publish(msg)
                rospy.sleep(2.0) #why do we sleep for 2 seconds?
                return True
        return False



    def choose_action(self, skeleton, choice):
        """
        Determines which action for Baxter to perform based on gestures
        """

        # Action not chosen yet
        if self.action_id == 0:
            # MIME
            if choice == 'right':
                self.action_id = 1
                action = 'Mime'
            elif choice == 'left':
                self.action_id = 2
                action = 'Crane'
            # ACTION CHOSEN
            print "Action chosen: %s\nProceed?\n" % action
            self.initialize_actions(self.action_id)

    def initialize_actions(self, action):
        """
        Creates action objects and initializes their state
        """
        self.actions = {1: self.mime_go,
                        2: self.crane_go}
        if action==1:
            print "    Action chosen: Mime\n"
            self.mime = Mime()
            self.action = 'mime_go'
            self.mime_count = 0
            self.action_chosen = True

            # Screen images
            img_thread = threading.Thread(target=self.img_change,
                                           name="Mime",
                                           args=(['mime']))
            img_thread.daemon = True
            img_thread.start()


        elif action == 2:
            print "    Action chosen: Crane\n"
            self.crane = Crane()
            self.crane_count = 0
            self.action_chosen = True

            # Screen images
            img_thread = threading.Thread(target=self.img_change,
                                           name="Crane",
                                           args=(['crane']))
            img_thread.daemon = True
            img_thread.start()

    def reset_booleans(self):
        """
        Resets booleans when user is done with action
        """
        print "\n**Booleans reset**\n"
        img = cv.LoadImage("/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-Booleans-Reset.jpg")
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
        pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
        pub.publish(msg)

        #Why do we disable, reset, and enable when resetting user stuff?
        self.rs.disable()
        self.rs.reset()
        self.rs.enable()

        self.userid_almost_chosen = False
        self.userid_chosen = False
        self.user_positioned = False
        self.action_chosen = False
        self.action_id = 0
        self.display_top = True
        self.display_mime_prep = True
        self.display_crane_prep = True
        self.display_mime = True
        self.display_crane = True
        rospy.sleep(2.0)
        
        # Screen images
        img_thread = threading.Thread(target=self.img_change,
                                       name="Top",
                                       args=(['top']))
        img_thread.daemon = True
        img_thread.start()

    #=========================================================#
    #                        ACTIONS:                         #
    #=========================================================#

    def mime_go(self, skeleton):
        """
        Progresses mime when skeletons are passed into controller
        """
        # For passing certain percentage of skeletons to mime
        self.mime_count+=1
        # Skeleton values
        l_sh = skeleton.left_shoulder.transform.translation
        l_el = skeleton.left_elbow.transform.translation
        l_ha = skeleton.left_hand.transform.translation
        r_sh = skeleton.right_shoulder.transform.translation
        r_el = skeleton.right_elbow.transform.translation
        r_ha = skeleton.right_hand.transform.translation
        

        if self.mime_count % DOWN_SAMPLE == 0:
            self.mime.move(l_sh, l_el, l_ha, r_sh, r_el, r_ha)

    def crane_go(self, skeleton):
        """
        Progresses crane when skeletons are passed into controller
        """
        # For passing certain percentage of skeletons to crane
        self.crane_count+=1
        # Skeleton values
        l_sh = skeleton.left_shoulder.transform.translation
        l_el = skeleton.left_elbow.transform.translation
        l_ha = skeleton.left_hand.transform.translation
        r_sh = skeleton.right_shoulder.transform.translation
        r_el = skeleton.right_elbow.transform.translation
        r_ha = skeleton.right_hand.transform.translation

        if self.crane_count % DOWN_SAMPLE == 0:
            self.crane.move(l_sh, l_el, l_ha, r_sh, r_el, r_ha)
    

    #######################
    # SUBSCRIBER CALLBACK #
    #######################
    def skeletonCallback(self, data):
        """
        Runs every time a skeleton is received from the tracker
        This seems to do everything
        This function is somewhat confusing, I'll go through later and document it better
        """
        # Chooses correct user
        if self.userid_chosen == True:
            found = False
            for skeleton in data.skeletons:
                if skeleton.userid == self.main_userid:
                    skel_raw = skeleton
                    if self.first_filt_flag:
                        self.skel_filt.reset_filters(skel_raw)
                        skel = skel_raw
                        self.first_filt_flag = False
                    else:
                        skel = self.skel_filt.filter_skeletons(skel_raw)
                    found = True


        # Chooses and sticks to one main user throughout
        if self.userid_chosen == False:
            self.choice = self.choose_user(data.skeletons)
            self.first_filt_flag = True

        elif self.user_positioned == False and found:
            p1_x = self.user_starting_position.x
            p1_z = self.user_starting_position.z
            p2_x = skel.torso.transform.translation.x
            p2_z = skel.torso.transform.translation.z

            y_LH = skel.left_hand.transform.translation.y
            y_RH = skel.right_hand.transform.translation.y
            y_torso = skel.torso.transform.translation.y
            left_ratio = (y_LH - y_torso) / y_torso
            right_ratio = (y_RH - y_torso) / y_torso


            dx = p2_x - p1_x
            dz = p2_z - p1_z
            if not (math.fabs(dx) > 0.10 and math.fabs(dz) > 0.10 and left_ratio > 0.5 and right_ratio > 0.5):
                self.user_positioned = self.position_user(skel, self.choice)
            else: self.reset_booleans()
    
        # Chooses action to complete
        elif self.action_chosen == False and found:
            self.choose_action(skel, self.choice)

        # If user doesn't leave, completes action
        elif found:
            p1_x = self.user_starting_position.x
            p1_z = self.user_starting_position.z
            p2_x = skel.torso.transform.translation.x
            p2_z = skel.torso.transform.translation.z

            y_LH = skel.left_hand.transform.translation.y
            y_RH = skel.right_hand.transform.translation.y
            y_torso = skel.torso.transform.translation.y
            left_ratio = (y_LH - y_torso) / y_torso
            right_ratio = (y_RH - y_torso) / y_torso


            dx = p2_x - p1_x
            dz = p2_z - p1_z
            if not (math.fabs(dx) > 0.10 and math.fabs(dz) > 0.10 and left_ratio > 0.5 and right_ratio > 0.5):
                self.actions[self.action_id](skel)
            else: self.reset_booleans()

        elif not found:           
            self.reset_booleans()
    



if __name__=='__main__':
    print("\nInitializing Baxter Controller node... ")
    rospy.init_node('Baxter_Controller', log_level=rospy.INFO)
    rospy.logdebug("node starting")
    Baxter_Controller()

    done = False # when does this get flipped?
    while not done and not rospy.is_shutdown():
        rospy.spin()

    print("\n\nDone.")
