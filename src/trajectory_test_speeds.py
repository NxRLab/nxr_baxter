#!/usr/bin/env python

# Kyle Lueptow
# Summer 2013

################
# ROS IMPORTS: #
################
import roslib
roslib.load_manifest('nxr_baxter')
import rospy
from std_msgs.msg import UInt16

####################
# RETHINK IMPORTS: #
####################
import baxter_interface
import dataflow

###############
# NU IMPORTS: #
###############
from trajectory import Trajectory
from vector_operations import (make_vector_from_POINTS,
	                           angle_between_vectors,
	                           vector_projection_onto_plane,
	                           shortest_vector_from_point_to_vector)

##################
# OTHER IMPORTS: #
##################
import os
import traceback
import threading
import Queue
import math
import operator

class trajectorytest():

	def __init__(self):
		"""
		Mime constructor
		"""
		self.left_arm = baxter_interface.Limb('left')
		self.right_arm = baxter_interface.Limb('right')

		self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
		self.pub_rate.publish(500)
		self.neutral_position_l = dict(zip(self.left_arm.joint_names(),
			                               7 * [0.0]))
		self.neutral_position_r = dict(zip(self.right_arm.joint_names(),
			                               7 * [0.0]))

	def move_thread(self, limb, position, queue, timeout=15.0):
		"""
		Adds the points sent by threads to the trajectory path
		"""
		try:
			trajectory = Trajectory(limb)
			trajectory.add_point(position, 0.4)
			trajectory.start()
			queue.put(None)
		except Exception, exception:
			queue.put(traceback.format_exc())
			queue.put(exception)

	def move(self, left_shoulder, left_elbow, left_hand,
		     right_shoulder, right_elbow, right_hand):
		"""
		Creates and sends threads for Baxter's arm movement
		"""
		# Collects angles from each controller
		angles = {'left': [], 'right': []}
		self.human_to_baxter(left_shoulder, left_elbow, left_hand,
		                     right_shoulder, right_elbow, right_hand, angles)
		
		# Creates threads for each of Baxter's arms
		left_queue = Queue.Queue()
		right_queue = Queue.Queue()
		left_thread = threading.Thread(target=self.move_thread,
			                           args=('left', angles['left'], left_queue))
		right_thread = threading.Thread(target=self.move_thread,
			                            args=('right', angles['right'], right_queue))
		left_thread.daemon = True
		right_thread.daemon = True
		left_thread.start()
		right_thread.start()
		dataflow.wait_for(lambda: \
			              not (left_thread.is_alive() or right_thread.is_alive()),
			              timeout=20.0,
			              rate=10)
		left_thread.join()
		right_thread.join()

	if __name__ == "__main__":
		print("\nInitializng node... ")
		rs = baxter_interface.RobotEnable()
    	print("Attempting to enable Baxter")
    	rs.enable()
		rospy.init_node("trajectorytest")
		
		i = 0
		while i < 6:
			if i = 0:
				self.move(0,0,0,0,0,0)
			elif i = 1:
				self.move(0.3,0.3,0.3,0.3,0.3,0.3)
			elif i = 2:
				self.move(0.6,0.6,0.6,0.6,0.6,0.6)
			elif i = 3:
				self.move(0.9,0.9,0.9,0.9,0.9,0.9)
			elif i = 4:
				self.move(1.2,1.2,1.2,1.2,1.2,1.2)
			elif i = 5:
				self.move(1.5,1.5,1.5,1.5,1.5,1.5)

		

		print("Completing tasks. Press Esc to quit.")
	    done = False
    	while not done and not rospy.is_shutdown():
        	c = iodevices.getch()
        	if c:
            	#catch Esc or ctrl-c
            	if c in ['\x1b', '\x03']:
                	done = True
            	else:
                	print("Press Esc to quit.")

    	print("All tasks completed.")
    	print("Disabling Baxter... ")
    	rs.disable()
	    print("done.") 