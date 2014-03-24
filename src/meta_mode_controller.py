#!/usr/bin/env python

# Adam Barber
# March 2014

# This script will create a node that provides a topic which publishes the
# overall node of the system. It will provide a way for nodes to ask to change
# the overall node.


# ROS IMPORTS
import rospy
from std_msgs.msg import String

# PYTHON IMPORTS


class MetaMode_Controller:
    def __init__(self, start_mode='idle_enabled'):
        rospy.logdebug("Calling MetaMode_Controller.__init__")

        # Define the list of possible modes, not definite Mime and crane are
        # fairly explanatory, idle_enabled means the motors and kinect are
        # enabled just waiting for user command, idle_disabled will be used once
        # we have a way to see if there's a user before turning things on and
        # during startup, and restart_kinect means that tracker_heartbeat has
        # determined it needs to restart all the kinect stuff.
        self.modes = ['mime', 'crane', 'idle_enabled', 'idle_disabled',
                      'restart_kinect']

        # Start topic
        self._pub = rospy.Publisher('meta_mode', String, latch=True)

        self.set_meta_mode(start_mode)

    def set_meta_mode(mode='idle_enabled'):
        rospy.logdebug("Calling MetaMode_Controller.set_meta_mode")
        if mode not in self.modes:
            rospy.logerr("Meta Mode %s does not match one of the possible meta\
 modes. Defaulting to 'idle_enabled.'", mode)
            start_mode = 'idle_enabled'
        self._current_mode = mode
        self._pub.publish(String(self._current_mode))

    def get_meta_mode():
        return self._current_mode
