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
        print "Mode: ", self.mode
    
    # written first without file stuff, lets just get it working hardcoded
    def __init__(self, mode='idle', image_period=3.0):
        self.mode = mode
        self.modeChanged = True
        self.image_timer = None
        self.image_period = image_period
        self.startTimer()
        print "Got here"

    def change_mode(self, newMode):
        self.mode = newMode
        self.modeChanged = True
        #re-run the timer callback function now
        self.startTimer()

    def startTimer(self):
        if self.image_timer:
            self.image_timer.shutdown()
        self.image_timer = rospy.Timer(rospy.Duration(self.image_period),self.timerCallback)



if __name__=="__main__":
    rospy.init_node('image_switcher_test_node')
    print "image switcher timer test"
    img_sw = ImageSwitcher()
    print "made object"
    rospy.sleep(10)
    img_sw.change_mode('mode 2')
    rospy.spin()
    
