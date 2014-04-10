#!/usr/bin/env python

# Adam Barber
# March 2014

# This script will create a node that provides a topic which publishes the
# overall node of the system. It will provide a way for nodes to ask to change
# the overall node.


# ROS IMPORTS
import rospy
from nxr_baxter_msgs.msg import MetaMode
from nxr_baxter_msgs.srv import *

# PYTHON IMPORTS


class MetaMode_Controller:

    def change_mode_callback(self, req):
        if req.mode not in [req.IDLE_DISABLED, req.IDLE_ENABLED, req.RESTART_KINECT, req.MIME, req.CRANE]:
            rospy.logerr("Meta Mode %d does not match one of the possible meta\ modes. Keeping same mode.", req.mode)
            return ChangeMetaModeResponse(False)
        elif req.mode != self.current_mode:
            self.current_mode = req.mode
            msg = MetaMode()
            msg.mode = req.mode
            self._pub.publish(msg)
            return ChangeMetaModeResponse(True)
        else:
            return ChangeMetaModeResponse(True)


    def __init__(self):
        rospy.logdebug("Calling MetaMode_Controller.__init__")
        rospy.loginfo("Starting meta mode controller")

        # Define the list of possible modes, not definite Mime and crane are
        # fairly explanatory, idle_enabled means the motors and kinect are
        # enabled just waiting for user command, idle_disabled will be used once
        # we have a way to see if there's a user before turning things on and
        # during startup, and restart_kinect means that tracker_heartbeat has
        # determined it needs to restart all the kinect stuff.

        # Start topic
        msg = MetaMode()
        msg.mode = msg.IDLE_ENABLED
        self.current_mode = msg.IDLE_ENABLED
        self._pub = rospy.Publisher('meta_mode', MetaMode, latch=True)
        self._pub.publish(msg)
        self.change_mode = rospy.Service('change_meta_mode', ChangeMetaMode, self.change_mode_callback)




if __name__=='__main__':
    rospy.init_node('meta_mode_controller', log_level = rospy.DEBUG)
    rospy.loginfo("Initializing meta_mode controller...")
    controller = MetaMode_Controller()
    rospy.spin()

