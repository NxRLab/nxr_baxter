#!/usr/bin/env python

# Adam Barber
# March 2014

# Ros imports
import rospy
import sensor_msgs.msg

# Other imports
import cv
import cv_bridge


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
        rospy.logdebug("Calling timerCallback")
        self._imageUpdate()

    def _imageUpdate(self, reset=False):
        """
        Does the majority of the work for this class. Will display the next
        image in the sequence, or restart the sequence if it hasn't been
        started. Should  only be called by the timer callback or whenever the
        mode/period has been changed and the timer needs to be reset.
        """
        rospy.logdebug("Calling imageUpdate")
        image_list = self.mode_to_images[self._mode]
        
        if reset:
            self.image_no = 0

        image_filename = image_list[self.image_no]
        msg = cv_bridge.CvBridge().cv_to_imgmsg(cv.LoadImage(image_filename))
        rospy.logdebug("Changing image")
        self._pub.publish(msg)
        
        self.image_no += 1
        if self.image_no >= len(image_list):
            self.image_no = 0


    # written first without file stuff, lets just get it working hardcoded
    def __init__(self, img_files_filename, mode='idle', image_period=0):
        """
        Constructor for the class, takes a filename that contains a list 
        of modes and their image files. Optionally takes a mode and an image
        switching period. Then starts the timer. An image period of 0 or
        negative means a one-shot timer.

        img_files_filename should be a file with a list of modes and
        filenames as strings with the following syntax:
        mode1 filename1_1 filename1_2
        mode2 filename2_1 filename2_2 filename2_3
        ...
        modeN filenameN
        """
        rospy.logdebug("Calling image_switcher.__init__")
        f = open(img_files_filename, 'r')
        self.mode_to_images = {}
        for line in f:
            split_line = line.split()
            #Allow pure whitespace lines
            if len(split_line) > 0:
                self.mode_to_images[split_line[0]] = split_line[1:]
        if mode not in self.mode_to_images.keys():
            #Maybe want to raise an exception here instead of this
            rospy.logerr("Image mode requested not in list of image modes. Requested: %s, going to top mode.", mode)
            mode = "top"
        self._mode = mode
        self.image_timer = None
        self._image_period = image_period
        self._image_no = 0
        self._pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
        self._startTimer()

    def change_mode(self, newMode=None, newPeriod=None):
        """
        Takes in a new mode type and/or a new image switching period. Note that
        even if neither of these change, the timer will be reset.
        """
        rospy.logdebug("Calling change_mode")
        if newMode and newMode != self._mode:
            self._mode = newMode
            self._image_period = newPeriod
            self._startTimer()
        else:
            rospy.logdebug("New image mode requested was already the image mode.")

    def _startTimer(self):
        """
        This function shuts down the timer if it has been running, prints the
        first image in the sequence (by resetting _imageUpdate()) and then
        starts the timer with the stored period.
        """
        rospy.logdebug("Calling _startTimer")
        if self.image_timer:
            self.image_timer.shutdown()
        # Note that the timer won't show an image until the timer goes off, so 
        # let's call the function that the callback calls once
        # Note that we only set up a timer if the period is > 0
        self._imageUpdate(reset=True)
        if self._image_period > 0:
            self.image_timer = rospy.Timer(rospy.Duration(self._image_period),self.timerCallback)



if __name__=="__main__":
    #Simple test script for ImageSwitcher class
    rospy.init_node('image_switcher_test_node', log_level=rospy.INFO)
    rospy.info("image switcher timer test")
    img_sw = ImageSwitcher('top')
    rospy.info("made object")
    rospy.sleep(6)
    img_sw.change_mode('crane_prep')
    rospy.spin()
    
