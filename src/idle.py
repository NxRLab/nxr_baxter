#!usr/bin/env python

# Kyle Lueptow
# Summer 2013

################
# ROS IMPORTS: #
################
import roslib
roslib.load_manifest('nxr_baxter')
import rospy
from std_msgs.msg import UInt16

####################
# RETHINK IMPORTS: #
####################
import baxter_interface

##################
# OTHER IMPORTS: #
##################
import math
import Queue
import threading
import os
import traceback


class PanHead():
    """
    PanHead class controls how Baxter "watches" people as they walk by
    """
    def __init__(self):
        self.head = baxter_interface.Head()
    
    def pan(self, headx, headz, timeout=15.0):
        """
        Calculates the angle at which Baxter's head should look
        using the x and z coordinates of the person's head
        """
        theta = math.atan(headx / headz)
        self.head.set_pan(theta, 50, 5)
        
   
class PickandPlace():
    """
    PickandPlace class controls Baxter's idle pick and place operations,
    as he moves two objects regularly through three different positions.
    """
    def __init__(self):
        left = baxter_interface.Gripper('left')
        left.reboot()
        self.left_arm = baxter_interface.Limb('left')
        self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        self.pub_rate.publish(500)
        """
        The following 6 dictionaries are positions for Baxter's left arm built such that
        each will put Baxter's arm over (for overposition dictionaries) one of the three
        bowls, or into (for inposition dictionaries) one of the bowls.
        """
        self.overposition1 = dict(zip())     #We need to write all of these dictionaries.
        self.overposition2 = dict(zip())
        self.overposition3 = dict(zip())
        self.inposition1 = dict(zip())
        self.inposition2 = dict(zip())
        self.inposition3 = dict(zip())

    def idle(self, emptybowl, queue, timeout=15.0):
        """
        Decides which bowl is empty to determine where it wil be picking
        and placing from and to.
        """
        
        if emptybowl == 1:
            emptybowl = pickplace(3)
        elif emptybowl == 2:
            emptybowl = pickplace(1)
        elif emptybowl == 3:
            emptybowl = pickplace(2)
        else:
            emptybowl = pickplace(1)
                
        queue.put(None)
        return emptybowl
    
    def pickplace(self, pick):

        """
        This checks which bowl is empty, and then assigns the pre-defined dictionaries
        to the two sets of two positions for positioning over the bowls, and then
        moving the grippers into the bowls.
        """
        if pick == 1:
            overpickpos = self.overposition1
            overplacepos = self.overposition2
            pickpos = self.inposition1
            placepos = self.inposition2
        elif pick == 2:
            overpickpos = self.overposition2
            overplacepos = self.overposition3
            pickpos = self.inposition2
            placepos = self.inposition3
        elif pick == 3:
            overpickpos = self.overposition3
            overplacepos = self.overposition1
            pickpos = self.inposition3
            placepos = self.inposition1
            
        """
        The following defines the order in which Baxter moves to positions to pick up
        the object, then place it in the next bowl. The function then returns the value
        of the now-empty bowl.
        """

        self.left_arm.move_to_joint_positions(self.overpickpos)
        self.left_arm.move_to_joint_positions(self.pickpos)
        left.close()
        self.left_arm.move_to_joint_positions(self.overpickpos)
        self.left_arm.move_to_joint_positions(self.overplacepos)
        self.left_arm.move_to_joint_positions(self.placepos)
        left.open()
        self.left_arm.move_to_joint_positions(self.overplacepos)
        
        return pick