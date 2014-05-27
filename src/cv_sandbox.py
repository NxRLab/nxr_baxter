#!/usr/bin/env python

import roslib
import rospy

import sys

import cv
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image

import baxter_interface


class vision(object):

	def __init__(self):
		# Instantiate all three cameras
		self._left_camera = baxter_interface.CameraController('left_hand_camera')
		self._right_camera = baxter_interface.CameraController('right_hand_camera')
		self._head_camera = baxter_interface.CameraController('head_camera')

		# Head screen publisher
		self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
		self._bridge = CvBridge()
		cv.NamedWindow("Image window", 1)

		# Start left hand camera
		self.start_camera()

	def put_image_on_screen(self):
		img = cv.LoadImage('/home/nxr-baxter/groovyws/src/nxr_baxter/images/Display-Booleans-Reset.jpg')
		msg = CvBridge().cv_to_imgmsg(img, encoding="bgr8")
		pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
		pub.publish(msg)
		rospy.sleep(1)

	def start_camera(self):
		# Close all three cameras
		self._left_camera.close()
		self._right_camera.close()
		self._head_camera.close()

		# Open left camera
		self._left_camera.open()
		self._left_camera.resolution = [1280, 800]

		# Subscribe to left camera's feed
		camera_topic = '/cameras/left_hand_camera/image'
		_camera_sub = rospy.Subscriber(
	        camera_topic,
	        Image,
	        self._on_camera) # Runs as callback

	def _on_camera(self, data):
		try:
			cv_image = self._bridge.imgmsg_to_cv(data, "bgr8")

		except CvBridgeError, e:
			print e

		cv.ShowImage("Image window", cv_image)
		msg = self._bridge.cv_to_imgmsg(cv_image, encoding="bgr8")
		self.image_pub.publish(msg)


def main():
	rospy.init_node('cv_sandbox')
	v = vision()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv.DestroyAllWindows()





if __name__ == '__main__':
    main()