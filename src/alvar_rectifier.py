#!/usr/bin/env python

import rospy
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker

class AlvarRectifier():

	def __init__(self):
		self.X_OFFSET, self.Y_OFFSET, self.Z_OFFSET = 0, 0, 0
		self.rectified_response = AlvarMarkers()
		
		rospy.init_node('nxr_ar_rectifier')
		rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.rectify_markers)
		self.ar_pub = rospy.Publisher('nxr_ar_rectifier', AlvarMarkers)								
		
		self.ar_rectifier()	

	def rectify_markers(self, response):
		for marker in response.markers:
			marker_pose = marker.pose.pose
			marker_pose.point.x += self.X_OFFSET
			marker_pose.point.y += self.Y_OFFSET
			marker_pose.point.z += self.Z_OFFSET
		
		self.rectified_response = response
	
	def ar_rectifier(self):
		self.ar_rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.ar_pub.publish(self.rectified_response)
			rospy.sleep(self.ar_rate)

if __name__ == '__main__':
	try:
		rectifier = AlvarRectifier()
	except Exception, e:
		print e

