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
        """
        Callback function for the image switching period timer. Calls the image
        update function with the default 'False' value for the reset. This means
        that anytime the function gets called out of this method, it will not
        assume  the mode has been changed.
        """
        self._imageUpdate()

    def _imageUpdate(self, reset=False):
        """
        Does the majority of the work for this class. Will display the next
        image in the sequence, or restart the sequence if it hasn't been
        started. Should  only be called by the timer callback or whenever the
        mode/period has been changed and the timer needs to be reset.
        """
        if reset:
            print "Resetting, mode: %s" % self._mode
        else:
            print "Continuing mode: %s" % self._mode
    
    # written first without file stuff, lets just get it working hardcoded
    def __init__(self, mode='idle', image_period=3.0):
        """
        Constructor for the class, optionally takes a mode and an image
        switching period. Then starts the timer.
        """
        self._mode = mode
        self.image_timer = None
        self._image_period = image_period
        self._startTimer()

    def change_mode(self, newMode=None, newPeriod=None):
        """
        Takes in a new mode type and/or a new image switching period. Note that
        even if neither of these change, the timer will be reset.
        """
        if newMode:
            self._mode = newMode
        if newPeriod:
            self._image_period = newPeriod
        self._startTimer()

    def _startTimer(self):
        """
        This function shuts down the timer if it has been running, prints the
        first image in the sequence (by resetting _imageUpdate()) and then
        starts the timer with the stored period.
        """
        if self.image_timer:
            self.image_timer.shutdown()
        # Note that the timer won't show an image until the timer goes off, so 
        # let's call the function that the callback calls once
        self._imageUpdate(reset=True)
        self.image_timer = rospy.Timer(rospy.Duration(self._image_period),self.timerCallback)



if __name__=="__main__":
    rospy.init_node('image_switcher_test_node')
    print "image switcher timer test"
    img_sw = ImageSwitcher()
    print "made object"
    rospy.sleep(10)
    img_sw.change_mode('mode 2',1)
    rospy.spin()
    
