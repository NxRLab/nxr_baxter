#!/usr/bin/env python
import rospy
import tf
from skeletonmsgs_nu.msg import Skeleton
from skeletonmsgs_nu.msg import SkeletonJoint
import copy
import numpy as np


# Global constants:

# define a dict for the joints that we care about, and some short strings to
# represent them
joints = {
    # right hand:
    'rhx' : 'right_hand.transform.translation.x',
    'rhy' : 'right_hand.transform.translation.y',
    'rhz' : 'right_hand.transform.translation.z',
    # left hand:
    'lhx' : 'left_hand.transform.translation.x',
    'lhy' : 'left_hand.transform.translation.y',
    'lhz' : 'left_hand.transform.translation.z',
    # right elbow:
    'rex' : 'right_elbow.transform.translation.x',
    'rey' : 'right_elbow.transform.translation.y',
    'rez' : 'right_elbow.transform.translation.z',
    # left elbow:
    'lex' : 'left_elbow.transform.translation.x',
    'ley' : 'left_elbow.transform.translation.y',
    'lez' : 'left_elbow.transform.translation.z',
    # right shoulder:
    'rsx' : 'right_shoulder.transform.translation.x',
    'rsy' : 'right_shoulder.transform.translation.y',
    'rsz' : 'right_shoulder.transform.translation.z',
    # left shoulder:
    'lsx' : 'left_shoulder.transform.translation.x',
    'lsy' : 'left_shoulder.transform.translation.y',
    'lsz' : 'left_shoulder.transform.translation.z',
}


# define a class for the Adaptive Double Exponential Smoothing Filter
class ADESFilter:
    # filter parameters:
    falpha = 0.25
    gamma = 0.01
    a_low = 0.01
    a_high = 0.35
    v_high = 0.008
    v_low = 0.01 # could be maybe a bit lower
    bn = 0.0
    prev_bn = 0.0
    prev_x = 0.0

    def update_filter(self, val):
        vn = np.abs(val - self.prev_x)
        if vn < self.v_low:
            self.falpha = self.a_low
        elif self.v_low <= vn <= self.v_high:
            self.falpha = self.a_high + ((vn-self.v_high)/(self.v_low-self.v_high))*\
              (self.a_low-self.a_high)
        elif vn > self.v_high:
            self.falpha = self.a_high
        else:
            self.falpha = (self.a_high+self.a_low)/2.0

        val = self.falpha*val + (1-self.falpha)*(self.prev_x+self.bn)
        self.bn = self.gamma*(val-self.prev_x) + (1-self.gamma)*self.prev_bn
        self.prev_bn = self.bn
        self.prev_x = val
        return val

    def reset_filter(self, val):
        # put filter "state" at a specified val
        self.bn = 0
        self.prev_bn = 0
        self.prev_x = val


# define a class for filtering Skeleton message types using the ADESFilter:
class SkeletonFilter( object ):
    def __init__(self, joints):
        """
        must pass a dict with joints we care about as items and some form of
        names as keys
        """
        self.joints = joints
        # now let's create a filter for each joint:
        self.filts = {}
        for key in self.joints.keys():
            self.filts[key] = ADESFilter()
        return

    def skel_getter(self, skel, joint):
        """
        take in a Skeleton message and a string as in the joints dict, and then
        return the val of the attribute that the string points to
        """
        js = joint.split('.')
        obj = skel
        while js:
            obj = obj.__getattribute__(js[0])
            js.pop(0)
        return obj

    def skel_setter(self, skel, joint, val):
        """
        take in a Skeleton message and a string as in the joints dict, and a
        value, and then set that value
        """
        js = joint.split('.')
        obj = skel
        while len(js) > 1:
            obj = obj.__getattribute__(js[0])
            js.pop(0)
        obj.__setattr__(js[0], val)
    
    def filter_skeletons(self, skel):
        """
        This function takes in a Skeleton message, and returns a Skeleton
        message that has been filtered:
        """
        skelf = copy.deepcopy(skel)
        for key,joint in self.joints.iteritems():
             val = self.filts[key].update_filter( self.skel_getter(skel, joint) )
             self.skel_setter(skelf, joint, val)
        return skelf

    def reset_filters(self, skel):
        for key,joint in self.joints.iteritems():
            val = self.filts[key].update_filter( self.skel_getter(skel, joint) )
        return


# tvec = np.linspace(0,10,1000)
# skel = Skeleton()
# xo = np.zeros(tvec.shape)
# xf = np.zeros(tvec.shape)
# filt = SkeletonFilter(joints)
# filt.filts['rhx'].reset_filter(5.)

# for i,t in enumerate(tvec):
#     val = 5 + np.sin(t) + 0.05*np.random.randn(1)
#     xo[i] = val
#     skel.right_hand.transform.translation.x = val
#     skelf = filt.filter_skeletons(skel)
#     xf[i] = skelf.right_hand.transform.translation.x

# plt.plot(tvec, xo, tvec, xf)
# plt.show()
