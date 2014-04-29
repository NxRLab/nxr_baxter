#!/usr/bin/env python

import roslib
import rospy

import sys

import cv
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image

import baxter_interface



def start_camera():
	# Instantiate all three cameras
	self._left_camera = baxter_interface.CameraController('left_hand_camera')
	self._right_camera = baxter_interface.CameraController('right_hand_camera')
	self._head_camera = baxter_interface.CameraController('head_camera')

	# Close all three cameras
	self._left_camera.close()
	self._right_camera.close()
	self._head_camera.close()

	# Open left camera
	self._left_camera.open()

	# Subscribe to left camera's feed
	camera_topic = '/cameras/left_hand_camera/image'
    _camera_sub = rospy.Subscriber(
        camera_topic,
        Image,
        self._on_camera) # Runs as callback

def _on_camera(self, data):
	try:
		self.cv_image = self._bridge.imgmsg_to_cv(data, "bgr8")
		local_image = np.asarray(self.cv_image)

		img = self.cv_image
	    msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
	    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
	    pub.publish(msg)
	except Exception:
		print 'OH NO - IMAGE WENT WRONG!!'


def main():
	start_camera()





if __name__ == '__main__':
    main()