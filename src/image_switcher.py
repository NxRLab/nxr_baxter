#!/usr/bin/env python

# Adam Barber
# March 2014

import rospy

class ImageSwitcher(object):
    """
    Image Switcher

    This class acts as a way to handle switching images for Baxter and in
    particular the nxr_baxter_demo package. In particular, the object will
    check which mode it should be on based on a timer and have a separate timer
    based on the mode that decides which image to display. The images, and
    potentially a list of modes should be loaded from a file.

    """
    def timerCallback(self, event):
        print "Timer!"
    
    # written first without file stuff, lets just get it working hardcoded
    def __init__(self, mode='idle'):
        self.mode = mode
        rospy.Timer(rospy.Duration(1), self.timerCallback)
        print "Got here"



if __name__=="__main__":
    rospy.init_node('image_switcher_test_node')
    print "image switcher timer test"
    img_sw = ImageSwitcher()
    print "made object"
    rospy.spin()
    
