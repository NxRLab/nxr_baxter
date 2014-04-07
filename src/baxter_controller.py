#!/usr/bin/env python

# Jon Rovira & Kyle Lueptow
# Summer 2013
# Adam Barber edits March 2014

################
# ROS IMPORTS: #
################
import rospy
from std_msgs.msg import Empty

####################
# RETHINK IMPORTS: #
####################
import baxter_interface
import baxter_dataflow

####################
# OTHER IMPORTS: #
####################
import os
import traceback
import threading
import Queue
import math
import numpy as np

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
import image_switcher as imgswitch

#Messages for meat-mode
from nxr_baxter_msgs.msg import MetaMode
from nxr_baxter_msgs.srv import ChangeMetaMode

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
        rospy.logdebug("Calling Baxter_Controller.__init__()")
        # print "Getting robot state..."
        self.rs = baxter_interface.RobotEnable() #RS is a wrapper for the robot state
        # print "Enabling robot... "
        rospy.loginfo("Enabling motors...")
        self.rs.enable()

        self.left_arm = baxter_interface.limb.Limb('left')
        self.right_arm = baxter_interface.limb.Limb('right')
        self.gripper = baxter_interface.Gripper('right')
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
        self.left_hand_timer = 0
        self.right_hand_timer = 0
        self.main_userid = 0

        #Pull the required filename from the parameter server
        try:
            img_files_filename = rospy.get_param('img_files_filename')
        except KeyError:
            sys.exit("img_files_filename not set in ROS parameter server")

        # Set up our ImageSwitcher object to do our first set of images
        # Note for the top mode there is only one image (for now) so we
        # don't need to set a period other than 0 which is a one-shot
        self.img_switch = imgswitch.ImageSwitcher(img_files_filename, mode='top',
                                                  image_period=0)

        # skeletonCallback called whenever skeleton received
        rospy.Subscriber("skeletons", Skeletons, self.skeletonCallback)
        # instantiate a skeleton filter
        self.skel_filt = sf.SkeletonFilter(sf.joints)
        self.first_filt_flag = True

        # Set up subscriber to meta-mode controller
        self.internal_mode = MetaMode.IDLE_ENABLED
        rospy.Subscriber("meta_mode", MetaMode, self.meta_mode_callback)

    # what does timeout do?
    def setup_move_thread(self, limb, mode, queue, timeout=15.0):
        if mode == 'crane':
            if limb == 'left':
                self.left_arm.move_to_joint_positions(self.crane_l_angles)
            elif limb == 'right':
                self.right_arm.move_to_joint_positions(self.crane_r_angles)
        elif mode == 'mime':
            if limb == 'left':
                self.left_arm.move_to_joint_positions(self.mime_l_angles)
            elif limb == 'right':
                self.right_arm.move_to_joint_positions(self.mime_r_angles)

    # What does tiemout do?
    def setup_gripper_thread(self, timeout=15.0):
        self.gripper.reboot()
        self.gripper.calibrate()

    def disable_move_thread(self, limb, queue, timeout=15.0):
        """
        Handles moving to disable position for each arm's thread
        """
        l_angles = {'left_s0': 0.25, 'left_s1': 0.00, 'left_e0': 0.00, 'left_e1': 1.57, 'left_w0': 0.00, 'left_w1': 0.00, 'left_w2': 0.00}
        r_angles = {'right_s0': -0.25, 'right_s1': 0.00, 'right_e0': 0.00, 'right_e1': 1.57, 'right_w0': 0.00, 'right_w1': 0.00, 'right_w2': 0.00}
        if limb == 'left':
            self.left_arm.move_to_joint_positions(l_angles)
        elif limb == 'right':
            self.right_arm.move_to_joint_positions(r_angles)

    def choose_user(self, skeletons):
        """
        Selects primary user to avoid ambiguity
        """
        rospy.logdebug("Calling choose_user")
        for skeleton in skeletons:
            lh = skeleton.left_hand.transform.translation.y
            rh = skeleton.right_hand.transform.translation.y
            h = skeleton.head.transform.translation.y

            # PersonX is raising a hand
            if h - lh > 0.11 or h - rh > 0.11: # What units are these? A: Meters (I think) - Jon
                # PersonX is main user
                if skeleton.userid == self.main_userid:
                    # PersonX is raising LEFT hand
                    if h - lh > 0.11:
                        # Makes sure that PersonX's right hand wasn't recently raised
                        if self.right_hand_timer == 0:
                            # Increment LEFT hand timer and set up RJ if it's been 3 seconds
                            self.left_hand_timer += 0.1
                            if self.left_hand_timer > 3:
                                self.left_hand_timer = 0
                                self.user_starting_position = skeleton.torso.transform.translation
                                self.change_mode_service(MetaMode.CRANE)
                        # LEFT hand is risen. Reset right hand timer
                        else: right_hand_timer = 0

                    # PersonX is raising RIGHT hand
                    elif h - rh > 0.11:
                        # Makes sure that PersonX's left hand wasn't recently raised
                        if self.left_hand_timer == 0:
                            # Increment RIGHT hand timer and set up RJ if it's been 3 seconds
                            self.right_hand_timer += 0.1
                            if self.right_hand_timer > 3:
                                self.right_hand_timer = 0
                                self.user_starting_position = skeleton.torso.transform.translation
                                self.change_mode_service(MetaMode.MIME)
                        # RIGHT hand is risen. Reset left hand timer
                        else: left_hand_timer = 0
                    return
                # Not main user.
                # If there isn't a main user, set PersonX as main
                elif self.left_hand_timer == 0 and self.right_hand_timer == 0:
                    self.main_userid = skeleton.userid
                    return
            # No hand risen and this is the main user
            # Reset both hand timers
            elif skeleton.userid == self.main_userid:
                self.left_hand_timer = 0
                self.right_hand_timer = 0
        return

    def choose_crane(self):
        rospy.logdebug("Calling choose_crane...")

        self.img_switch.change_mode('crane_prep',3)

        left_queue = Queue.Queue()
        right_queue = Queue.Queue()
        gripper_queue = Queue.Queue()
        left_thread = threading.Thread(target=self.setup_move_thread,
                                       args=('left', 'crane', left_queue))
        right_thread = threading.Thread(target=self.setup_move_thread,
                                        args=('right', 'crane', right_queue))
        gripper_thread = threading.Thread(target=self.setup_gripper_thread)
        left_thread.daemon = True
        right_thread.daemon = True
        gripper_thread.daemon = True
        left_thread.start()
        right_thread.start()
        gripper_thread.start()
        baxter_dataflow.wait_for(
            lambda: not (left_thread.is_alive() or right_thread.is_alive() or
                         gripper_thread.is_alive()), timeout=20.0,
                         timeout_msg=(
                            "Timeout while waiting for arm move threads to finish"),
                            rate=10,
            )
        left_thread.join()
        right_thread.join()
        gripper_thread.join()

        self.userid_chosen = True
        rospy.loginfo("Main user chosen.\nUser %s, please proceed.", str(self.main_userid))

    def choose_mime(self):
        rospy.logdebug("Calling choose_crane...")
        self.img_switch.change_mode('mime_prep', 3)
        left_queue = Queue.Queue()
        right_queue = Queue.Queue()
        left_thread = threading.Thread(target=self.setup_move_thread,
                                       args=('left', 'mime', left_queue))
        right_thread = threading.Thread(target=self.setup_move_thread,
                                        args=('right', 'mime', right_queue))
        left_thread.daemon = True
        right_thread.daemon = True
        left_thread.start()
        right_thread.start()
        baxter_dataflow.wait_for(
            lambda: not (left_thread.is_alive() or
                        right_thread.is_alive()),
                        timeout=20.0,
                        timeout_msg=(
                            "Timeout while waiting for arm move threads to finish"
                            ),
                        rate=10,
        )
        left_thread.join()
        right_thread.join()

        self.userid_chosen = True
        rospy.loginfo("Main user chosen.\nUser %s, please proceed.", str(self.main_userid))

    def change_mode_service(self, mode_type):
        rospy.logdebug("Calling change_mode_service...")
        rospy.wait_for_service('change_meta_mode')
        try:
            change_mode = rospy.ServiceProxy('change_meta_mode', ChangeMetaMode)

            change_resp = change_mode(mode_type)
            # if not change_success.error:
            if not change_resp.error:
                rospy.logerr("Tried to go back to idle mode, failed!")
            else:
                rospy.logdebug("Mode changed.")
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)

    def position_user(self, skeleton):
        rospy.logdebug("Calling position_user")
        lh_y = skeleton.left_hand.transform.translation.y
        lh_x = skeleton.left_hand.transform.translation.x
        rh_y = skeleton.right_hand.transform.translation.y
        rh_x = skeleton.right_hand.transform.translation.y
        tor_y = skeleton.torso.transform.translation.y
        tor_x = skeleton.torso.transform.translation.x
        # if choice == 'left':
        if self.internal_mode == MetaMode.CRANE:
            dy = math.fabs(tor_y - lh_y)
            dx = math.fabs(lh_x - tor_x)
            #These tolerances have been causing problems
            # if dy < 0.08 and dx > 0.4:
            if dy < 0.1 and dx > 0.2:
                self.img_switch.change_mode('positioned',0)
                rospy.sleep(0.5) # try a 1/2 second delay
                return True
        # elif (choice == 'right'):
        elif self.internal_mode == MetaMode.MIME:
            dy = math.fabs(tor_y - lh_y) + math.fabs(tor_y - rh_y)
            dx = math.fabs(lh_x - tor_x) + math.fabs(rh_x - tor_x)
            # if dy < 0.20 and dx > 0.8:
            if dy < 0.22 and dx > 0.6:
                self.img_switch.change_mode('positioned',0)
                rospy.sleep(0.5) # try 1/2 second delay
                return True
        return False


    def reset_booleans(self):
        """
        Resets booleans when user is done with action
        """
        rospy.logdebug("Calling reset_booleans")
        self.img_switch.change_mode('bool_reset',3)

        self.change_mode_service(MetaMode.IDLE_ENABLED)


    #=========================================================#
    #                        ACTIONS:                         #
    #=========================================================#

    def mime_go(self, skeleton):
        """
        Progresses mime when skeletons are passed into controller
        """
        rospy.logdebug("Calling mime_go")
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
            self.mime_count = 0

    def crane_go(self, skeleton):
        """
        Progresses crane when skeletons are passed into controller
        """
        rospy.logdebug("Calling crane_go")
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
            self.crane_count = 0
    

    #######################
    # SUBSCRIBER CALLBACK #
    #######################
    def skeletonCallback(self, data):
        """
        Runs every time a skeleton is received from the tracker
        This seems to do everything
        This function is somewhat confusing, I'll go through later and document it better
        """
        rospy.logdebug("Calling skeletonCallback")
        # Chooses correct user
        if self.userid_chosen == True:
            #A user has been chosen by a previous run
            found = False
            #Check if they are still in the frame by running through all the skeletons in the frame
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
            # We haven't previously chosen a user.
            # self.choice = self.choose_user(data.skeletons)
            self.choose_user(data.skeletons)
            # Don't think we need self.choice
            self.first_filt_flag = True

        elif self.user_positioned == False and found:
            # There IS a user, we found them still in the frame, and they are not in position yet to do the task
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
                self.user_positioned = self.position_user(skel)
            else:
                self.reset_booleans()
    
        # Chooses action to complete
        # Instead of self.userid_choice == False
        elif self.action_chosen == False and found:
            #There is a user, they are in position, time to start the actual action
            # self.choose_action(skel, self.choice)
            if self.internal_mode == MetaMode.MIME:
                rospy.loginfo("    Action chosen: Mime\n")
                self.mime = Mime()
                self.mime_count = 0
                self.action_chosen = True
                # Screen images
                self.img_switch.change_mode('mime',3)
            elif self.internal_mode == MetaMode.CRANE:
                rospy.loginfo("    Action chosen: Crane\n")
                self.crane = Crane()
                self.crane_count = 0
                self.action_chosen = True
                # Screen images
                self.img_switch.change_mode('crane',3)

        # Means found == True and user_positioned == True and action_chosen == True
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
                if self.internal_mode == MetaMode.MIME:
                    self.mime_go(skel)
                elif self.internal_mode == MetaMode.CRANE:
                    self.crane_go(skel)
            else:
                self.reset_booleans()

        elif not found:
            rospy.loginfo("not found")
            self.reset_booleans()

    def meta_mode_callback(self, data):
        if self.internal_mode != data.mode:
            rospy.logdebug("New mode: %d", data.mode)
            self.internal_mode = data.mode
            if data.mode == MetaMode.MIME:
                self.choose_mime()
            elif data.mode == MetaMode.CRANE:
                self.choose_crane()
            elif data.mode == MetaMode.IDLE_DISABLED:
            #Disable everything
                self.disable()
            elif data.mode == MetaMode.IDLE_ENABLED:
            #Enable everything
                self.enable()
            elif data.mode == MetaMode.RESTART_KINECT:
            #Disable everything while kinect is restarting?
                self.img_switch.change_mode('kinect_reset',0)
                self.disable()
            else:
                rospy.logerr("Got a mode that doesn't exist...")
        else:
            rospy.logdebug("Already in mode: %d", data.mode)

    def disable(self):
        rospy.logdebug("Calling disable...")
        #Disable motors
        rospy.loginfo("Disabling baxter_controller.py")
        rospy.loginfo("TODO: Make an image for being disabled")
        self.rs.disable()
        #Reset booleans, won't call the function because that does other stuff
        self.bool_reset()

    def enable(self):
        rospy.logdebug("Calling enable...")
        #Enable motors
        rospy.loginfo("Enabling baxter_controller.py")
        left_queue = Queue.Queue()
        right_queue = Queue.Queue()
        
        left_thread = threading.Thread(target=self.disable_move_thread, args=('left', 'mime', left_queue))
        right_thread = threading.Thread(target=self.disable_move_thread, args=('right', 'mime', right_queue))
        
        left_thread.daemon = True
        right_thread.daemon = True
        
        left_thread.start()
        right_thread.start()
        
        baxter_dataflow.wait_for(
            lambda: not (left_thread.is_alive() or right_thread.is_alive()),
            timeout=20.0,
            timeout_msg=("Timeout while waiting for arm move threads to finish"),
            rate=10,
        )
        
        left_thread.join()
        right_thread.join()
        self.rs.reset()
        self.rs.enable()
        rospy.sleep(2.0)
        self.bool_reset()
        self.img_switch.change_mode('top',3)

    def bool_reset(self):
        rospy.loginfo("\n**Booleans reset**\n")
        self.userid_chosen = False
        self.user_positioned = False
        self.action_chosen = False
        self.action_id = 0
        self.left_hand_timer = 0
        self.right_hand_timer = 0

if __name__=='__main__':
    print("\nInitializing Baxter Controller node... ")
    rospy.init_node('Baxter_Controller', log_level=rospy.INFO)
    rospy.logdebug("node starting")
    Baxter_Controller()

    done = False # when does this get flipped?
    while not done and not rospy.is_shutdown():
        rospy.spin()

    print("\n\nDone.")
